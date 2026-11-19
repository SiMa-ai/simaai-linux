// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// MLA_OP_BUF_ALLOC / IMPORT / INFO handlers + per-context buffer
// lifecycle. Per the ABI:
//   - phys_addr is exposed only to host-kind contexts (HOST_PROCESS,
//     CONTAINER); zeroed for VM_GUEST.
//   - Imported dma-bufs must be a SINGLE physically contiguous range
//     (adjacent mapped SG entries are allowed); else -EINVAL.
//   - The kernel holds its own reference on allocated (carveout)
//     buffers: MLA_OP_BUF_RELEASE or context teardown is the free.
//     Closing the dma-buf fd only drops userspace's view of the
//     memory.

#include <linux/capability.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/iosys-map.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/xarray.h>

#include "mla_drv.h"
#include "mla_memory.h"

/* ---------------------------------------------------------------------- *
 *  xarray helpers
 *
 *  The per-context xarray was initialized with XA_FLAGS_ALLOC1, so
 *  xa_alloc starts numbering at 1 and skips 0 (reserved as "invalid").
 *  xa_alloc / xa_load / xa_erase take the xarray's internal spinlock —
 *  no external mutex required.
 * ----------------------------------------------------------------------
 */

/*
 * Insert @buf into @ctx's per-context buffer xarray and assign it
 * an internal_buf_id. The xarray was initialized with XA_FLAGS_ALLOC1
 * so the assigned id is in [1, 2^32) — id 0 is reserved as "invalid".
 */
int mla_buffer_register(struct mla_context *ctx, struct mla_buffer *buf)
{
	u32 id;
	int ret;

	ret = xa_alloc(&ctx->buffers, &id, buf, xa_limit_32b, GFP_KERNEL);
	if (ret < 0)
		return ret;

	/* Publish the back-pointers only after the insert can no longer
	 * fail, and take the context reference that keeps ->ctx readable
	 * for this buffer's whole life (it may outlive the fd — see
	 * docs/CTX_LIFETIME_DESIGN.md). Dropped in mla_buffer_drop's
	 * final-free paths and in mla_dmabuf_release.
	 */
	buf->ctx  = mla_context_get(ctx);
	buf->mdev = ctx->mdev;   /* hot-path convenience; devm, never freed */
	buf->id   = id;
	return 0;
}

/* Look up a buffer by id under @ctx. NULL if id is unknown. */
struct mla_buffer *mla_buffer_find(struct mla_context *ctx, u32 id)
{
	return xa_load(&ctx->buffers, id);
}

/* ---------------------------------------------------------------------- *
 *  MLA_OP_BUF_ALLOC
 * ----------------------------------------------------------------------
 */

/**
 * mla_op_buf_alloc() — handle MLA_OP_BUF_ALLOC
 * @ctx: context to register the buffer with
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Allocates a fresh dma-buf from the carveout heap and registers it in
 * the context. The returned response carries both the dma-buf fd
 * (for userspace mmap / pass to other drivers) and the
 * internal_buf_id (for use in MLA_OP_JOB_SUBMIT iotensor arrays).
 *
 * The kernel keeps its own dmabuf reference inside the mla_buffer, so
 * the registration (and the memory) survives userspace closing the fd;
 * MLA_OP_BUF_RELEASE or context teardown is what frees it. The mmap is
 * WB-cacheable — userspace must bracket CPU access with
 * DMA_BUF_IOCTL_SYNC like for any other dma-heap buffer.
 *
 * Return: 0 on success; -ENOMEM / -EFAULT / -EINVAL otherwise.
 */
int mla_op_buf_alloc(struct mla_context *ctx,
		     const struct mla_uring_cmd_sqe *sqe,
		     const struct mla_cmd_hdr *hdr)
{
	struct mla_buf_alloc_req  req;
	struct mla_buf_alloc_resp resp;
	struct mla_buffer *buf;
	int fd, ret;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;
	if (!req.out_ptr || req.out_len < sizeof(resp))
		return -EINVAL;
	if (!req.size)
		return -EINVAL;
	if (req.flags & MLA_HEAP_FLAG_OCM)
		return -EOPNOTSUPP;

	buf = mla_heap_alloc(ctx->mdev, req.size, req.flags);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	ret = mla_buffer_register(ctx, buf);
	if (ret < 0) {
		mla_buffer_drop(buf);
		return ret;
	}

	/* Take the kernel's own counted dma-buf reference BEFORE handing the
	 * original to userspace. dma_buf_fd is get_unused_fd_flags +
	 * fd_install with no get_file, so it TRANSFERS a reference to the fd.
	 * mla_heap_alloc handed us the buffer's alloc reference; this get
	 * balances the fd transfer so the kernel retains exactly one counted
	 * dma-buf reference for the registered buffer (see
	 * docs/BUFFER_LIFETIME_DESIGN.md §2.1). It is dropped when
	 * buf->refcount reaches 0 via mla_buffer_drop -> mla_heap_free.
	 */
	get_dma_buf(buf->dmabuf);
	fd = dma_buf_fd(buf->dmabuf, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		ret = fd;
		dma_buf_put(buf->dmabuf);
		goto err_unreg;
	}

	memset(&resp, 0, sizeof(resp));
	resp.dmabuf_fd       = fd;
	resp.internal_buf_id = buf->id;
	resp.size_actual     = buf->size;
	resp.flags           = buf->flags;
	if (ctx->kind != MLA_CTX_VM_GUEST)
		resp.phys_addr = (u64)buf->dma_addr;
	dev_dbg(ctx->mdev->dev,
		"buf_alloc: OK buf_id=%u phys=0x%llx size=0x%zx (req.size=0x%llx) fd=%d\n",
		buf->id, (unsigned long long)buf->dma_addr,
		(size_t)buf->size, (unsigned long long)req.size, fd);

	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp))) {
		/* Unwind completely. The caller cannot clean up after itself
		 * here: the fd number and the buf_id existed only in @resp,
		 * whose copyout is what just failed, so -EFAULT names neither.
		 * Leaving them installed meant repeating the call with a bad
		 * out_ptr exhausted both the file table and the reserved pool.
		 *
		 * close_fd() acts on current->files, which is the same table
		 * dma_buf_fd's fd_install just wrote to in this same handler
		 * invocation, so the target is unambiguous. Userspace cannot yet
		 * have learned the number.
		 *
		 * That drops the fd's transferred dma-buf reference, which is
		 * the role the dma_buf_put on the fd<0 path plays — so from
		 * here err_unreg's unwind is exactly right.
		 */
		ret = -EFAULT;
		close_fd(fd);
		goto err_unreg;
	}

	return 0;

err_unreg:
	/* Releases the xarray reference, which drops the kernel dma-buf
	 * reference taken above. buf itself stays valid: the hand-off leaves
	 * the original (still-untransferred) reference below holding it.
	 */
	xa_erase(&ctx->buffers, buf->id);
	buf->id = 0;
	/* heap_backed: mla_buffer_drop routes to mla_heap_free, which
	 * unmaps/detaches/vunmaps and drops the buffer's alloc dma-buf
	 * reference, then frees the struct. The fd-transfer get taken above is
	 * already balanced before every jump here — by dma_buf_put on the fd<0
	 * path, and by close_fd on the copyout-failure path — so there is no
	 * separate original reference left to put.
	 */
	mla_buffer_drop(buf);
	return ret;
}

/* ---------------------------------------------------------------------- *
 *  MLA_OP_BUF_IMPORT
 * ----------------------------------------------------------------------
 */

/**
 * mla_op_buf_import() — handle MLA_OP_BUF_IMPORT
 * @ctx: context to register the imported buffer with
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Imports a dma-buf fd allocated by another subsystem (camera, ISP,
 * encoder, etc.) into this context. Attaches it for DMA, maps the
 * sg_table, and registers an internal_buf_id that submit-time tensor
 * descriptors can reference.
 *
 * Return: 0 on success; -EBADF for a bad fd; -ENOMEM otherwise.
 */
int mla_op_buf_import(struct mla_context *ctx,
		      const struct mla_uring_cmd_sqe *sqe,
		      const struct mla_cmd_hdr *hdr)
{
	struct mla_buf_import_req  req;
	struct mla_buf_import_resp resp;
	struct dma_buf            *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table           *sgt;
	struct mla_buffer         *buf;
	dma_addr_t dma_addr;
	int ret;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;
	if (!req.out_ptr || req.out_len < sizeof(resp))
		return -EINVAL;
	if (req.dmabuf_fd < 0)
		return -EINVAL;

	dmabuf = dma_buf_get(req.dmabuf_fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	attach = dma_buf_attach(dmabuf, ctx->mdev->dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		goto err_put;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_detach;
	}

	ret = mla_sg_dma_address(sgt, dmabuf->size, PAGE_SIZE, &dma_addr);
	if (ret) {
		dev_dbg(ctx->mdev->dev,
			"imported dma-buf is not one valid contiguous DMA interval\n");
		goto err_unmap;
	}

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	refcount_set(&buf->refcount, 1);
	atomic64_inc(&ctx->mdev->buf_inc_ops);
	buf->dmabuf   = dmabuf;
	buf->imported = true;
	buf->attach   = attach;
	buf->sgt      = sgt;
	buf->size     = dmabuf->size;
	buf->dma_addr = dma_addr;
	buf->flags    = MLA_BUF_FLAG_IMPORTED;

	/*
	 * Importing a carveout-heap export (one of our own BUF_ALLOCs
	 * coming back around, or a buffer userspace allocated straight
	 * from /dev/dma_heap/simaai,dms): the backing is no-map reserved
	 * memory with no linear-map alias the cache-maintenance code can
	 * touch, so the dma_sync_* family must be kept away from it (see
	 * no_cpu_alias) — maintenance is the exporter's job via its
	 * cpu-access ops. Detected by the exporter name, which the heap
	 * core stamps on every buffer it exports. mla-exported CMA
	 * section buffers ("mla") are page-backed and sync normally, so
	 * they are deliberately excluded.
	 *
	 * Foreign exporters are left alone. NOTE this driver does NOT sync
	 * their sgt either, despite having created the mapping: per-job
	 * IFM/OFM cache maintenance was removed and is the caller's
	 * obligation (see the block comment in mla_hw.c). The warning below
	 * covers the case where the caller cannot discharge it.
	 */
	buf->no_cpu_alias = mla_dmabuf_no_cpu_alias(ctx->mdev, dmabuf);

	/*
	 * Foreign exporter that implements NO cpu-access ops. Warn, count, and
	 * import anyway.
	 *
	 * Why this matters: cache maintenance for IFM/OFM is the caller's job,
	 * and the caller's only tool is DMA_BUF_IOCTL_SYNC — which dispatches
	 * into the EXPORTER's begin/end_cpu_access. dma_buf_end_cpu_access()
	 * returns 0 when that op is NULL, so for such an exporter the ioctl
	 * silently does nothing, and the MLA is non-coherent (no dma-coherent
	 * in its DT node). A CPU write to the buffer is then never written back
	 * and the device reads stale DRAM — cache-line-granular wrong data,
	 * no error, no fault.
	 *
	 * Why NOT -EINVAL. The dangerous condition is "exporter does no
	 * maintenance AND the caller CPU-touches the buffer". This code can
	 * only see the first half:
	 *   - Refusing would break the main legitimate use of importing a
	 *     camera/codec frame, namely pure device-to-device zero-copy where
	 *     the CPU never touches the buffer and no maintenance is needed.
	 *   - It would not even be a reliable gate. videobuf2-dma-contig
	 *     REGISTERS both ops as literal "return 0;" stubs
	 *     (videobuf2-dma-contig.c:427,434), so the V4L2 capture path — the
	 *     most likely importer of all — passes this test and is still
	 *     unsynced. A stub cannot be told from a real implementation
	 *     through the op pointer. al5r (Allegro codec, al_dmabuf.c:278)
	 *     declares neither op and so is caught; that asymmetry is exactly
	 *     why a refusal would give false assurance rather than safety.
	 * So the honest thing is to say so and let the caller proceed.
	 *
	 * warn_once, not per-import: a pooled pipeline re-imports per frame on
	 * this path (mla-rt's resolve_dmabuf_fd imports on first use of each
	 * fd), and a printk per frame is its own bug. The counter carries the
	 * long-term signal.
	 */
	if (!buf->no_cpu_alias && dmabuf->ops &&
	    !dmabuf->ops->begin_cpu_access && !dmabuf->ops->end_cpu_access) {
		atomic64_inc(&ctx->mdev->foreign_imports_no_cpu_ops);
		dev_warn_once(ctx->mdev->dev,
			      "BUF_IMPORT: exporter \"%s\" implements no dma-buf cpu-access ops. This driver does no IFM/OFM cache maintenance, and DMA_BUF_IOCTL_SYNC on this fd is a no-op, so if you write or read this buffer with the CPU nothing will maintain the caches and the MLA (non-coherent) will see stale data. Safe only if the buffer is never CPU-touched. See counters/foreign_imports_no_cpu_ops.\n",
			      dmabuf->exp_name ? dmabuf->exp_name : "<unnamed>");
	}

	ret = mla_buffer_register(ctx, buf);
	if (ret < 0) {
		/* Balance the buf_inc_ops bump above by hand. mla_buffer_drop()
		 * cannot do it for us: it only bumps buf_dec_ops when buf->ctx
		 * is set, and register failed before it took the context
		 * reference. Skipping the balance left the debugfs
		 * buf_inc_ops/buf_dec_ops delta permanently non-zero, and that
		 * delta is one of the regression's oracles.
		 *
		 * kfree rather than the drop helpers is correct HERE, and is the
		 * documented exception to the rule stated on mla_buffer_free():
		 * no context reference exists to leak, and the unmap/detach/put
		 * below own the dma-buf teardown on this path — routing through
		 * mla_buffer_drop() would repeat all three.
		 */
		atomic64_inc(&ctx->mdev->buf_dec_ops);
		kfree(buf);
		goto err_unmap;
	}

	/* Attribute pool occupancy to MLA in the OOM report. After register, so
	 * buf->ctx is set for the balancing subtraction. Unlike no_cpu_alias
	 * above this is not gated on backing: occupancy matters for both.
	 */
	if (mla_dmabuf_is_dms(ctx->mdev, dmabuf) ||
	    (ctx->mdev->heap && dmabuf->exp_name &&
	     !strcmp(dmabuf->exp_name, dma_heap_get_name(ctx->mdev->heap)))) {
		buf->counted_heap_bytes = true;
		atomic64_add(buf->size, &ctx->mdev->imported_heap_bytes);
	}

	memset(&resp, 0, sizeof(resp));
	resp.internal_buf_id = buf->id;
	resp.flags           = buf->flags;
	resp.size            = buf->size;
	if (ctx->kind != MLA_CTX_VM_GUEST)
		resp.phys_addr = (u64)buf->dma_addr;

	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp))) {
		/* Registered, so unwind through the drop helpers — NOT through
		 * err_unmap below, which would unmap, detach and put a second
		 * time on top of what mla_buffer_teardown()'s imported path
		 * already does. The caller cannot clean up itself: buf_id lived
		 * only in @resp.
		 */
		xa_erase(&ctx->buffers, buf->id);
		buf->id = 0;
		mla_buffer_drop(buf);
		return -EFAULT;
	}

	return 0;

err_unmap:
	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(dmabuf, attach);
err_put:
	dma_buf_put(dmabuf);
	return ret;
}

/* ---------------------------------------------------------------------- *
 *  MLA_OP_BUF_INFO
 * ----------------------------------------------------------------------
 */

/**
 * mla_op_buf_info() — handle MLA_OP_BUF_INFO
 * @ctx: context that owns the buffer
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Reports per-buffer metadata: size, alignment, flags, the dma_addr
 * (for buffers with a stable mapping), and which dma-heap it came
 * from. Useful for debugging and for callers that need to know the
 * physical address of a phys-attached buffer without relying on the
 * mmap fd path.
 *
 * Return: 0 on success; -ENOENT if internal_buf_id is unknown.
 */
int mla_op_buf_info(struct mla_context *ctx,
		    const struct mla_uring_cmd_sqe *sqe,
		    const struct mla_cmd_hdr *hdr)
{
	struct mla_buf_info_req  req;
	struct mla_buf_info_resp resp;
	struct mla_buffer       *buf;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;
	if (!req.out_ptr || req.out_len < sizeof(resp))
		return -EINVAL;

	buf = mla_buffer_find(ctx, req.internal_buf_id);
	if (!buf)
		return -ENOENT;

	memset(&resp, 0, sizeof(resp));
	resp.internal_buf_id = buf->id;
	resp.flags           = buf->flags;
	resp.size            = buf->size;
	resp.alignment       = buf->alignment ? buf->alignment : PAGE_SIZE;
	if (ctx->kind != MLA_CTX_VM_GUEST)
		resp.phys_addr = (u64)buf->dma_addr;

	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp)))
		return -EFAULT;
	return 0;
}

/* ---------------------------------------------------------------------- *
 *  MLA_OP_BUF_ATTACH_PHYS
 *
 *  Record a caller-supplied (phys_addr, size) range as a kernel-side
 *  mla_buffer. No dma-buf, no fd, no memremap — the device path uses
 *  b->dma_addr directly at JOB_SUBMIT. This keeps ATTACH-heavy
 *  workloads (e.g. llima KV-cache window registration) from
 *  exhausting RLIMIT_NOFILE. These buffers need no cache maintenance
 *  from anyone: the backing is write-combine no-map reserved memory,
 *  so writes reach DRAM without a clean.
 *
 *  CAP_SYS_RAWIO required.
 * ----------------------------------------------------------------------
 */

/**
 * mla_op_buf_attach_phys() — handle MLA_OP_BUF_ATTACH_PHYS
 * @ctx: context to register the buffer with
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Records a caller-supplied (phys, size) range directly as a kernel-side
 * mla_buffer, with no dma-buf, no fd, and no memremap. The range
 * typically comes from a reserved no_map region (e.g. simaai-mlamem)
 * that the kernel doesn't track via struct page; the device path
 * consumes buf->dma_addr directly at JOB_SUBMIT.
 *
 * Capability gate: requires CAP_SYS_RAWIO since attaching arbitrary
 * physical addresses is a privileged operation.
 *
 * Return: 0 on success; -EPERM without CAP_SYS_RAWIO; -EINVAL on
 * bad address range; -ENOMEM otherwise.
 */
int mla_op_buf_attach_phys(struct mla_context *ctx,
			   const struct mla_uring_cmd_sqe *sqe,
			   const struct mla_cmd_hdr *hdr)
{
	struct mla_buf_attach_phys_req  req;
	struct mla_buf_attach_phys_resp resp;
	struct mla_buffer *buf;
	int ret;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr),
			   sizeof(req)))
		return -EFAULT;
	ret = mla_dma_range_check(req.phys_addr, req.size, PAGE_SIZE);
	if (ret)
		return ret;
	if (req.size > SIZE_MAX || !PAGE_ALIGNED(req.size))
		return -EINVAL;
	/* A raw address does not pin allocator-owned pages. This legacy API
	 * remains available only outside managed RAM; reusable CMA must use
	 * BUF_ALLOC/BUF_IMPORT so accepted jobs retain the owning dma-buf.
	 */
	if (region_intersects(req.phys_addr, req.size, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_DISJOINT)
		return -EPERM;

	if (!req.out_ptr || req.out_len < sizeof(resp))
		return -EINVAL;

	/* fd-less attach: the kernel records (phys, size, buf_id) and
	 * the device path consumes b->dma_addr at JOB_SUBMIT. We do NOT
	 * memremap, do NOT export a dma-buf, and do NOT allocate an fd —
	 * userspace KV-cache-style workloads can register thousands of
	 * windows without hitting RLIMIT_NOFILE. Cache maintenance is
	 * unnecessary for phys_attached either way: WC backing in no-map
	 * reserved memory, so writes reach DRAM without a clean.
	 */
	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	refcount_set(&buf->refcount, 1);
	atomic64_inc(&ctx->mdev->buf_inc_ops);
	buf->dma_addr      = (dma_addr_t)req.phys_addr;
	buf->size          = (size_t)req.size;
	buf->alignment     = PAGE_SIZE;
	buf->flags         = req.flags;
	buf->phys_attached = true;

	ret = mla_buffer_register(ctx, buf);
	if (ret < 0)
		goto err_free;

	memset(&resp, 0, sizeof(resp));
	resp.dmabuf_fd       = -1;
	resp.internal_buf_id = buf->id;
	resp.phys_addr       = req.phys_addr;
	resp.size            = req.size;
	dev_dbg(ctx->mdev->dev,
		"buf_attach_phys: OK buf_id=%u phys=0x%llx size=0x%llx (fd-less)\n",
		buf->id, (unsigned long long)req.phys_addr,
		(unsigned long long)req.size);
	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp))) {
		/* Use mla_buffer_drop(), not a bare kfree: a bare kfree bypasses
		 * mla_buffer_free() and leaks the context reference taken by
		 * mla_buffer_register(), and skips the buf_dec_ops balance.
		 * phys_attached buffers have no dma-buf, no attachment and no
		 * vmap, so mla_buffer_drop()'s teardown here is just
		 * mla_buffer_free() (kfree + mla_context_put).
		 */
		xa_erase(&ctx->buffers, buf->id);
		buf->id = 0;
		mla_buffer_drop(buf);
		return -EFAULT;
	}
	return 0;

err_free:
	/* Register never ran (or failed), so there is no context reference and
	 * no xarray entry; kfree is right. But buf_inc_ops was already bumped
	 * above, and mla_buffer_drop() would not balance it without buf->ctx —
	 * so do it here. Same documented exception as BUF_IMPORT's register
	 * failure path.
	 */
	atomic64_inc(&ctx->mdev->buf_dec_ops);
	kfree(buf);
	return ret;
}

/* ---------------------------------------------------------------------- *
 *  Context teardown — release all registered buffers
 *
 *  Every dma-buf-backed buffer carries exactly one kernel-owned dma-buf
 *  reference, taken at registration; imported and carveout buffers
 *  additionally hold an attachment (and carveout also a vmap). Releasing
 *  the registration reference here drops ours and lets the dma-buf
 *  refcount decide when the memory actually goes.
 * ----------------------------------------------------------------------
 */

/*
 * kfree an mla_buffer and drop the context reference it was holding.
 *
 * Does exactly two things, in this order:
 *   1. kfree(buf)
 *   2. mla_context_put(buf->ctx)   — balances the mla_context_get() in
 *                                    mla_buffer_register
 *
 * THE ONLY place a REGISTERED mla_buffer is kfree'd, deliberately.
 * mla_buffer_drop reaches it directly for imported, alias and phys_attached
 * buffers, and indirectly via mla_pool_free (pool-owned) and
 * mla_dmabuf_release (dma-buf-backed); routing all of them through here is
 * what makes the get/put balance auditable. A missed put leaks one context
 * per buffer; a doubled put frees a context still in use. Do not open-code
 * kfree(buf) for a buffer that reached mla_buffer_register().
 *
 * ONE documented exception, on the paths where mla_buffer_register() itself
 * FAILED (mla_op_buf_import, mla_op_buf_attach_phys): buf->ctx was never
 * set, so there is no context reference to put, and routing through
 * mla_buffer_drop() would both fail to bump buf_dec_ops (it is guarded on
 * buf->ctx) and repeat a dma-buf teardown the caller's own error ladder
 * already performs. Those sites kfree directly AND bump buf_dec_ops by
 * hand to balance the buf_inc_ops taken with the initial refcount.
 *
 * Order matters: the put can be the LAST reference and free the context,
 * so it must come after the buffer is gone and nothing may touch buf or
 * buf->ctx afterwards. buf->ctx is cleared first so a
 * use-after-free-through-a-stale-buffer reads NULL rather than a freed
 * context.
 *
 * Safe in atomic context: mla_context_put's free path does not sleep
 * (see mla_context_destroy).
 */
void mla_buffer_free(struct mla_buffer *buf)
{
	struct mla_context *ctx = buf->ctx;

	/* Sole kfree site, so the only place the import count can be balanced
	 * for every teardown path. Runs before ctx is cleared.
	 */
	if (buf->counted_heap_bytes && ctx) {
		atomic64_sub(buf->size, &ctx->mdev->imported_heap_bytes);
		buf->counted_heap_bytes = false;
	}

	buf->ctx = NULL;
	kfree(buf);
	mla_context_put(ctx);
}

static void mla_buffer_teardown(struct mla_buffer *buf);
static void mla_buffer_release_work_fn(struct work_struct *work);

/*
 * Release one counted reference to a buffer. When the last reference
 * goes, tear the buffer down according to its shape.
 *
 * refcount_t is the authority for an mla_buffer's lifetime: the ctx xarray
 * registration and each mla_model->section_bufs[] entry hold one counted
 * reference each. The last reference does not always free the struct
 * directly — it hands off to whoever owns the memory:
 *
 *   dma-buf-backed  → drop the kernel's counted dma-buf reference; the
 *                     last dma_buf_put fires mla_dmabuf_release, which
 *                     frees the backing memory and the struct.
 *   dedup primary   → drop this holder's reference on the dedup entry;
 *                     the entry's own last put retires the primary.
 *
 * Both hand-offs keep the memory alive for as long as refcount is
 * non-zero, which is what makes refcount_inc sufficient to keep a buffer
 * AND its DMA mapping alive. See docs/BUFFER_LIFETIME_DESIGN.md.
 *
 * Caller must have already xa_erase'd the entry (when releasing the
 * registration reference). May sleep — dma_buf_put can call the
 * exporter's release op, and dma_buf_detach takes a mutex.
 *
 * buf_dec_ops counts drop CALLS, matching buf_inc_ops which counts
 * references TAKEN; that is what keeps the debugfs `dump` invariant
 * delta == 0 meaningful. Do not move this bump below the dec_and_test.
 *
 * The device is resolved via buf->ctx OR buf->mdev, not buf->ctx alone.
 * buf_inc_ops is bumped where the initial reference is taken — inside
 * mla_heap_alloc / mla_pool_alloc_cached, BEFORE mla_buffer_register
 * sets buf->ctx — so a drop on a register-failure path found ctx == NULL,
 * skipped the bump, and left the delta permanently non-zero. buf->mdev is
 * set at allocation for exactly this reason (see mla_buf_dev()).
 */
void mla_buffer_drop(struct mla_buffer *buf)
{
	struct mla_device *mdev = buf->ctx ? buf->ctx->mdev : buf->mdev;

	if (mdev)
		atomic64_inc(&mdev->buf_dec_ops);
	if (!refcount_dec_and_test(&buf->refcount))
		return;
	mla_buffer_teardown(buf);
}

/**
 * mla_buffer_drop_async() — release a reference from a context that may
 *                           not sleep
 * @buf: buffer to release one reference on
 *
 * Same accounting and same last-reference semantics as mla_buffer_drop(),
 * but the teardown is handed to mdev->buf_release_wq instead of running
 * inline. Used by mla_job_put_bindings(), which runs from
 * mla_fence_release() and can therefore be in softirq context (via
 * mla_sched_complete from the IRQ handler, or mla_job_timeout_fn) while
 * mla_buffer_teardown() may sleep.
 *
 * Only the LAST reference costs anything: the common case is
 * refcount_dec_and_test returning false, which never touches the queue.
 *
 * Deliberately a separate entry point rather than a change to
 * mla_buffer_drop(): mla_context_buffers_release() needs teardown to have
 * completed when it returns, and deferring the fd-close path would make
 * the debugfs `delta` / `contexts (0 open)` invariants timing-dependent —
 * those are the regression's oracles. buf_dec_ops is still bumped
 * synchronously here, so the accounting is never deferred, only the free.
 */
void mla_buffer_drop_async(struct mla_buffer *buf)
{
	struct mla_device *mdev = buf->mdev;

	/* ctx-or-mdev, same reason as mla_buffer_drop(). */
	if (buf->ctx && buf->ctx->mdev)
		atomic64_inc(&buf->ctx->mdev->buf_dec_ops);
	else if (mdev)
		atomic64_inc(&mdev->buf_dec_ops);
	if (!refcount_dec_and_test(&buf->refcount))
		return;

	/* No queue (probe partially failed, or a buffer with no device):
	 * fall back to inline teardown. Callers reaching here from softirq
	 * would be a latent sleep-in-atomic, so say so rather than hide it.
	 */
	if (!mdev || !mdev->buf_release_wq) {
		WARN_ONCE(!mdev || !mdev->buf_release_wq,
			  "mla: no buf_release_wq; tearing down buffer inline\n");
		mla_buffer_teardown(buf);
		return;
	}

	/* Safe to init here: a buffer's last reference drops exactly once, so
	 * this work item is queued at most once in the buffer's lifetime and
	 * cannot already be pending.
	 */
	INIT_WORK(&buf->release_work, mla_buffer_release_work_fn);
	queue_work(mdev->buf_release_wq, &buf->release_work);
}

/* Workqueue handler for mla_buffer_drop_async(). Runs the teardown that
 * could not run inline; by the time it fires the refcount is already 0 and
 * nobody else can reach the buffer.
 */
static void mla_buffer_release_work_fn(struct work_struct *work)
{
	struct mla_buffer *buf = container_of(work, struct mla_buffer,
					      release_work);

	mla_buffer_teardown(buf);
}

/*
 * The last-reference teardown itself, shared by mla_buffer_drop() and the
 * deferred path. Callers MUST have taken refcount to 0 first. MAY SLEEP.
 */
static void mla_buffer_teardown(struct mla_buffer *buf)
{
	/* Imported: we hold an attachment plus a counted reference on a
	 * dma-buf owned by someone else, so its ->priv is not us and this
	 * struct is ours to free.
	 */
	if (buf->imported) {
		if (buf->attach && buf->sgt)
			dma_buf_unmap_attachment(buf->attach, buf->sgt,
						 DMA_BIDIRECTIONAL);
		if (buf->attach)
			dma_buf_detach(buf->dmabuf, buf->attach);
		if (buf->dmabuf)
			dma_buf_put(buf->dmabuf);
		mla_buffer_free(buf);
		return;
	}

	/* Alias shell: hand the dedup entry's reference back and free the
	 * shell. The primary is retired by the entry's own last put.
	 */
	if (buf->dedup_alias) {
		struct mla_device *mdev = buf->ctx ? buf->ctx->mdev : NULL;

		if (mdev && buf->dedup)
			mla_section_dedup_put(mdev, buf->dedup);
		mla_buffer_free(buf);
		return;
	}

	/* Published dedup primary. The entry is what owns the memory once
	 * published, and it is deliberately NOT a counted holder of this
	 * buffer: the primary's refcount reaching 0 is exactly the signal
	 * that this holder is done, and the entry's refcount (1 for the
	 * primary + 1 per alias) decides when the memory goes. Making the
	 * entry take a reference here would close a cycle — primary waits on
	 * entry, entry waits on primary — and pin the section for the life of
	 * the module. Do not "fix" it that way; see
	 * docs/BUFFER_LIFETIME_DESIGN.md §4.2.
	 *
	 * dedup_put frees the primary via mla_pool_free (legacy MODEL_DEFINE
	 * pool primaries) or dma_buf_put of the kernel's registration
	 * reference (SECTION_ALLOC primaries) when the entry drops to 0.
	 */
	if (buf->dedup) {
		struct mla_device *mdev = buf->ctx ? buf->ctx->mdev : NULL;

		if (mdev)
			mla_section_dedup_put(mdev, buf->dedup);
		return;
	}

	if (buf->pool_owned) {
		/* Pool buffers are carveout- or cma-backed and have no userspace
		 * fd. buf->mdev is set at allocation (mla_heap_alloc for
		 * mla_pool_alloc, and mla_pool_alloc_cached) and again by
		 * mla_buffer_register; mla_pool_free performs the full backing
		 * teardown and routes the struct through mla_buffer_free, which
		 * releases the context ref.
		 */
		mla_pool_free(buf->mdev, buf);
	} else if (buf->heap_backed) {
		/* Carveout BUF_ALLOC / non-CPU_RMW SECTION_ALLOC: mla_buffer owns
		 * vmap + attachment + one dmabuf ref; mla_heap_free tears it
		 * all down and frees the struct. A still-open userspace fd keeps
		 * the heap memory alive on its own.
		 */
		mla_heap_free(buf->ctx ? buf->ctx->mdev : NULL, buf);
	} else if (buf->dmabuf) {
		/* mla-exported CMA dma-buf (CPU_RMW section). Kernel took its one
		 * counted ref at SECTION_ALLOC (BUFFER_LIFETIME_DESIGN.md §4.1);
		 * drop it — last dma_buf_put fires mla_dmabuf_release, freeing the
		 * struct + memory. committed or not.
		 */
		dma_buf_put(buf->dmabuf);
	} else {
		/* phys_attached (BUF_ATTACH_PHYS) owns nothing but the struct
		 * itself — no dma-buf, no memremap to undo here.
		 */
		mla_buffer_free(buf);
	}
}

/**
 * mla_op_buf_release() — handle MLA_OP_BUF_RELEASE
 * @ctx: context that owns the buffer
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Drops the per-context registration of @internal_buf_id and the
 * kernel's reference to the underlying dma-buf. The dma-buf itself
 * stays alive as long as userspace holds the fd; this op only
 * affects the per-context xarray entry, freeing the id for reuse.
 *
 * Releasing a buffer that's still bound to an in-flight or queued
 * job is safe: the job has already resolved the buf_id to a kernel
 * pointer at submit time and no longer consults the xarray. Future
 * JOB_SUBMIT calls referencing the released id return -ENOENT.
 *
 * Return: 0 on success; -EINVAL on malformed request; -EFAULT on
 * copy_from_user failure; -ENOENT if @internal_buf_id is unknown
 * to this context.
 */
int mla_op_buf_release(struct mla_context *ctx,
		       const struct mla_uring_cmd_sqe *sqe,
		       const struct mla_cmd_hdr *hdr)
{
	struct mla_buf_release_req req;
	struct mla_buffer *buf;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr),
			   sizeof(req)))
		return -EFAULT;

	buf = xa_erase(&ctx->buffers, req.internal_buf_id);
	if (!buf)
		return -ENOENT;

	/* Clear buf->id BEFORE dropping the ref. buf may still be held via
	 * mla_model->section_bufs[] (v2 MODEL_DEFINE bump); its later
	 * mla_model_destroy pass must NOT re-attempt xa_erase(id) since the
	 * id may now be recycled to a different buf (xa_alloc gives back the
	 * lowest free id). Zero id tells mla_model_destroy "the xarray owner
	 * already released; only drop the section_bufs ref."
	 */
	buf->id = 0;
	mla_buffer_drop(buf);
	return 0;
}

/**
 * mla_context_buffers_release() — release all buffers on context teardown
 * @ctx: context whose buffer xarray is being destroyed
 *
 * Walks the xarray, erases each entry, and drops each buffer's
 * reference via mla_buffer_drop(). Called from mla_context_shutdown()
 * on the fd close path.
 *
 * A buffer may legitimately OUTLIVE this call: a BUF_ALLOC or
 * SECTION_ALLOC buffer whose fd userspace still holds keeps its dma-buf
 * alive after we drop the kernel's reference, and fput is deferred to the
 * delayed_fput workqueue besides. Such a buffer keeps a counted reference
 * on the context, taken in mla_buffer_register, so buf->ctx stays valid
 * until mla_dmabuf_release drops it. Nothing here needs to sever
 * back-pointers — see docs/CTX_LIFETIME_DESIGN.md.
 */
void mla_context_buffers_release(struct mla_context *ctx)
{
	struct mla_buffer *buf;
	unsigned long index;

	/*
	 * xa_for_each releases the internal lock between iterations, so
	 * the cleanup path inside mla_buffer_drop (which may sleep —
	 * dma_buf_put can call the exporter's release op) is safe.
	 */
	xa_for_each(&ctx->buffers, index, buf) {
		xa_erase(&ctx->buffers, index);
		mla_buffer_drop(buf);
	}
}

/* ---------------------------------------------------------------------- *
 *  MLA_OP_SECTION_ALLOC / MLA_OP_SECTION_COMMIT (SECTION_ALLOC_ABI Phase 1)
 *
 *  Direct per-section allocation with dma-buf export.
 *
 *  SECTION_ALLOC(name, zone, flags, size, sha256):
 *    - HIT sec_ddp: refcount++, register alias, return {section_id,
 *                   dmabuf_fd=-1, dedup_hit=1}.
 *    - MISS: allocate a dma-buf-wrapped kernel buffer (coherent, or
 *            WB-cached when CPU_RMW), register in ctx, return
 *            {section_id, dmabuf_fd, dedup_hit=0}.
 *
 *  SECTION_COMMIT publishes into sec_ddp — see the dedup_requested
 *  branch in mla_op_section_commit() below — so the HIT path above
 *  also matches entries from other processes, not just legacy
 *  MODEL_DEFINE entries.
 *
 *  Concurrent same-SHA callers: both may MISS in sec_ddp and both
 *  allocate. Losers of the mla_section_dedup_insert race drop their
 *  own primary and use it anyway (accept-the-waste) rather than
 *  re-resolving to the winner's alias — see the branch below.
 *
 *  SECTION_COMMIT(section_id): takes the kernel-owned dma_buf ref and,
 *  when the section carried DEDUP_OK + SHA, publishes into sec_ddp.
 *  ----------------------------------------------------------------------
 */

int mla_op_section_alloc(struct mla_context *ctx,
			 const struct mla_uring_cmd_sqe *sqe,
			 const struct mla_cmd_hdr *hdr)
{
	struct mla_section_alloc_req  req;
	struct mla_section_alloc_resp resp;
	struct mla_section_dedup_entry *ddp;
	struct mla_buffer *buf = NULL;
	struct mla_buffer *alias = NULL;
	struct dma_buf    *dmabuf = NULL;
	int fd = -1;
	int ret;

	(void)hdr;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;
	if (req.reserved)
		return -EINVAL;
	if (!req.out_ptr || req.out_len < sizeof(resp))
		return -EINVAL;
	if (!req.size || (req.size & ~PAGE_MASK))
		return -EINVAL;
	if (req.flags & ~(u32)(MLA_SECTION_FLAG_DEDUP_OK |
			       MLA_SECTION_FLAG_RDONLY |
			       MLA_SECTION_FLAG_CPU_RMW))
		return -EINVAL;

	memset(&resp, 0, sizeof(resp));

	/* sec_ddp HIT path: matches legacy MODEL_DEFINE entries and entries
	 * published by SECTION_COMMIT (see mla_op_section_commit() below).
	 */
	if (req.flags & MLA_SECTION_FLAG_DEDUP_OK) {
		ddp = mla_section_dedup_lookup(ctx->mdev, req.sha256,
					       (size_t)req.size);
		if (ddp) {
			ret = mla_section_alloc_alias(ctx, ddp, &alias);
			if (ret < 0)
				return ret;
			resp.section_id  = alias->id;
			resp.dmabuf_fd   = -1;
			resp.size_actual = alias->size;
			resp.dedup_hit   = 1;
			if (copy_to_user(u64_to_user_ptr(req.out_ptr),
					 &resp, sizeof(resp))) {
				xa_erase(&ctx->buffers, alias->id);
				mla_buffer_drop(alias);
				return -EFAULT;
			}
			return 0;
		}
	}

	/* CPU_RMW sections prefer the shared heap, otherwise the cached DMA
	 * exporter. Other sections use the selected heap. Registration holds
	 * one dma-buf reference; mla_buffer_drop handles the separate heap
	 * wrapper or the cached exporter's dmabuf->priv lifetime.
	 */
	if (req.flags & MLA_SECTION_FLAG_CPU_RMW)
		buf = mla_secbuf_alloc_cached(ctx->mdev, req.size);
	else
		buf = mla_heap_alloc(ctx->mdev, req.size,
					 MLA_HEAP_FLAG_CACHED);
	if (IS_ERR(buf))
		return PTR_ERR(buf);
	dmabuf = buf->dmabuf;

	/* Remember the DEDUP intent + SHA so SECTION_COMMIT can publish
	 * into sec_ddp without another userspace round trip. Only set
	 * when the caller both opted in and passed a non-zero hash.
	 */
	if (req.flags & MLA_SECTION_FLAG_DEDUP_OK) {
		u8 zero_sha[32] = {0};

		if (memcmp(req.sha256, zero_sha, 32) != 0) {
			buf->dedup_requested = true;
			memcpy(buf->pending_sha, req.sha256, 32);
		}
	}

	ret = mla_buffer_register(ctx, buf);
	if (ret < 0)
		goto err_free_buf;

	/* Kernel-owned dma-buf reference, taken UNCONDITIONALLY for both
	 * backing types (v1 lifetime model). dma_buf_fd below transfers the
	 * original to userspace; this get is the kernel's single retained
	 * reference. Same rationale as mla_op_buf_alloc — see
	 * docs/BUFFER_LIFETIME_DESIGN.md §4.1. For carveout, dmabuf is the
	 * alloc ref plus this get; the fd transfer leaves the kernel holding
	 * one (dropped by mla_heap_free). For CMA, the export ref plus
	 * this get; the fd transfer leaves one (dropped by the uniform
	 * dma_buf_put, whose mla_dmabuf_release frees the struct). This is
	 * also why SECTION_COMMIT needs no get_dma_buf of its own: the
	 * section is safe against an early fd close from registration on.
	 */
	get_dma_buf(dmabuf);

	fd = dma_buf_fd(dmabuf, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		ret = fd;
		goto err_unreg;
	}

	resp.section_id  = buf->id;
	resp.dmabuf_fd   = fd;
	resp.size_actual = buf->size;
	resp.dedup_hit   = 0;

	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp))) {
		/* Userspace cannot close the fd itself: the fd number existed
		 * only in @resp, whose copyout is what just failed, so -EFAULT
		 * names neither the fd nor the section_id. Repeating the call
		 * with a bad out_ptr would exhaust the file table and the heap.
		 *
		 * Unwind in the same shape as err_unreg below, except that
		 * close_fd() replaces its trailing dma_buf_put: there the get was
		 * never transferred, here the fd took it.
		 *
		 * Drop the registration first, then the fd, and do not touch
		 * @buf afterwards — for a carveout backing mla_buffer_drop frees
		 * the struct, and for a CMA export the struct lives in
		 * dmabuf->priv and dies with the last dma_buf_put.
		 */
		xa_erase(&ctx->buffers, buf->id);
		buf->id = 0;
		mla_buffer_drop(buf);
		close_fd(fd);
		return -EFAULT;
	}
	return 0;

err_unreg:
	/* fd<0 after the unconditional get. Drop the registration reference
	 * (mla_buffer_drop — carveout runs mla_heap_free and frees the
	 * struct; CMA drops one of the export's two refs), then balance the
	 * still-untransferred get. Correct for both backing types: the get
	 * and this trailing put pair exactly. Reached only when no fd was
	 * installed; the copyout-failure path above closes the fd instead.
	 */
	xa_erase(&ctx->buffers, buf->id);
	buf->id = 0;
	mla_buffer_drop(buf);
	dma_buf_put(dmabuf);
	return ret;

err_free_buf:
	/* mla_buffer_register failed before any get was taken; free the
	 * freshly allocated buffer. Carveout owns a struct distinct from its
	 * dma-buf (mla_buffer_drop -> mla_heap_free); a CMA export's
	 * struct lives in dmabuf->priv and dies with the last dma_buf_put.
	 */
	if (buf->heap_backed)
		mla_buffer_drop(buf);
	else
		dma_buf_put(dmabuf);
	return ret;
}

/*
 * mla_op_section_commit — SECTION_COMMIT handler.
 *
 * Called by userspace after it has finished writing (memcpy + optional
 * DMA_BUF_IOCTL_SYNC) to a section returned by SECTION_ALLOC. Two
 * things happen here:
 *
 *   1. The section is marked committed. The memory already survives
 *      userspace closing its fd (which happens right after
 *      SECTION_COMMIT per SECTION_ALLOC_ABI.md §3.1) because
 *      SECTION_ALLOC took a kernel-owned dma-buf reference at
 *      registration, so COMMIT does not need one of its own. See
 *      docs/BUFFER_LIFETIME_DESIGN.md §4.1.
 *
 *   2. If the caller opted in to dedup (MLA_SECTION_FLAG_DEDUP_OK +
 *      non-zero SHA at SECTION_ALLOC), publish this buffer into
 *      sec_ddp so subsequent SECTION_ALLOC callers with the same SHA
 *      can alias against it.
 *
 * Idempotent: if the buffer is already committed (COMMIT called
 * twice), the second call is a no-op success. This matches the
 * userspace contract — COMMIT is the "I'm done" signal, and a
 * duplicate signal is harmless.
 *
 * Dedup race semantics — accept-the-waste (design doc §3.3 wrinkle
 * resolution). Two concurrent SECTION_ALLOC calls with the same SHA
 * both allocate, both COMMIT; one wins the dedup insert, the other
 * ends up with an orphaned (but still-referenced-by-caller) primary.
 * The loser's memory stays live until its section_id is released;
 * MODEL_DEFINE_V3 will index into whichever primary the caller's
 * section_ids[] points at, so consistency is preserved at the cost
 * of one section's DRAM in the narrow race window.
 */
int mla_op_section_commit(struct mla_context *ctx,
			  const struct mla_uring_cmd_sqe *sqe,
			  const struct mla_cmd_hdr *hdr)
{
	struct mla_section_commit_req req;
	struct mla_buffer *buf;

	(void)hdr;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;
	if (req.reserved)
		return -EINVAL;

	buf = mla_buffer_find(ctx, req.section_id);
	if (!buf)
		return -ENOENT;
	/* Reject buf shapes that can't have been produced by SECTION_ALLOC
	 * (imported, phys_attached, pool_owned) to catch API misuse.
	 */
	if (buf->imported || buf->phys_attached || buf->pool_owned)
		return -EINVAL;
	/* HIT-aliases: nothing to commit; the primary was committed by
	 * the caller that first produced this section. Idempotent
	 * success matches the userspace contract (COMMIT is always safe
	 * to call after ALLOC returns).
	 */
	if (buf->dedup_alias)
		return 0;
	if (!buf->dmabuf)
		return -EINVAL;
	if (buf->committed)
		return 0;

	/* No get_dma_buf here: SECTION_ALLOC already took the kernel's single
	 * counted dma-buf reference at registration, so the memory already
	 * survives userspace closing its fd, and a second reference here
	 * would need a second put to balance it. ->committed carries only
	 * state/validation meaning (MODEL_DEFINE requires it, and
	 * v3_resolve_section_refs rejects uncommitted ids), not lifetime
	 * meaning. See docs/BUFFER_LIFETIME_DESIGN.md §4.1.
	 */
	buf->committed = true;

	dev_dbg(ctx->mdev->dev,
		"SECTION_COMMIT id=%u dedup_req=%d\n",
		buf->id, buf->dedup_requested);

	/* Publish into sec_ddp if the caller asked for it at ALLOC.
	 * Race with a concurrent SECTION_COMMIT of the same SHA is
	 * handled by mla_section_dedup_insert — winner keeps its buf,
	 * loser sees ->dedup=NULL and MODEL_DEFINE_V3 uses this buf
	 * anyway (accept-the-waste).
	 */
	if (buf->dedup_requested) {
		struct mla_section_dedup_entry *entry;

		entry = mla_section_dedup_insert(ctx->mdev, buf->pending_sha, buf);
		if (IS_ERR(entry)) {
			/* -ENOMEM only. The buf is fine; it just isn't
			 * dedup-published. MODEL_DEFINE_V3 still uses it
			 * via section_ids[]. Clear the request flag so a
			 * later retry isn't silently attempted.
			 */
			buf->dedup_requested = false;
		} else if (mla_section_dedup_entry_buf(entry) != buf) {
			/* Race loser: didn't become the primary. Drop the
			 * refcount we bumped and forget the dedup entry —
			 * our buf lives on independently for this caller.
			 */
			mla_section_dedup_put(ctx->mdev, entry);
			buf->dedup_requested = false;
		}
		/* else: winner. buf->dedup was set inside insert(). */
	}
	return 0;
}
