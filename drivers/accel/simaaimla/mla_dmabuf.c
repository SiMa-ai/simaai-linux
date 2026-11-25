// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// dma-buf exporter for WB-cached CMA section buffers (SECTION_ALLOC
// with MLA_SECTION_FLAG_CPU_RMW — descriptor blobs whose per-job reloc
// RMW needs the cacheable linear-map alias plus range-scoped flushes).
//
// This is the only memory the MLA driver exports itself; every
// other allocation is a carveout-heap dma-buf (see mla_pool.c). The
// backing here is dma_alloc_noncoherent: real pages in the linear map,
// so sg tables build from the virtual address, userspace mmaps get the
// same WB attributes via remap_pfn_range, and cache maintenance runs
// through dma_sync_single_for_{cpu,device}.

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/iosys-map.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "mla_drv.h"
#include "mla_memory.h"

/*
 * Resolve the owning struct device for a buffer.
 *
 * buf->mdev is set on EVERY mla_buffer before it can be reached by any
 * dma_buf op, at all allocation sites: mla_heap_alloc (which backs
 * BUF_ALLOC, non-CPU_RMW SECTION_ALLOC and mla_pool_alloc),
 * mla_pool_alloc_cached, mla_secbuf_alloc_cached (before mla_dmabuf_export
 * publishes the buffer), and via mla_buffer_register for BUF_IMPORT /
 * BUF_ATTACH_PHYS. Register-failure paths free without ever exporting a
 * dma-buf, so there is no window where a live buffer has mdev == NULL.
 *
 * mdev itself is devm_kzalloc'd at probe and outlives every buffer and
 * every context, so this needs no liveness check — unlike buf->ctx,
 * which is only valid because the buffer holds a counted reference on it
 * (see mla_buffer_register / docs/CTX_LIFETIME_DESIGN.md).
 */
static inline struct device *mla_buf_dev(const struct mla_buffer *buf)
{
	/* Not a fallback: a NULL here means an allocation site forgot to set
	 * mdev, which is a driver bug worth surfacing rather than papering
	 * over with buf->ctx->mdev.
	 */
	if (WARN_ON_ONCE(!buf->mdev))
		return NULL;
	return buf->mdev->dev;
}

struct mla_dmabuf_attachment {
	struct sg_table table;
	bool            mapped;
};

/*
 * dma-buf ops vtable for CMA section buffers. Per-attachment state is
 * held in struct mla_dmabuf_attachment: a segmented sg_table and a
 * "mapped" flag indicating whether dma_map_sgtable has run.
 */
static int mla_dmabuf_attach(struct dma_buf *dmabuf,
			     struct dma_buf_attachment *attachment)
{
	struct mla_buffer *buf = dmabuf->priv;
	struct mla_dmabuf_attachment *a;
	int ret;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	if (!virt_addr_valid(buf->cpu_vaddr)) {
		kfree(a);
		return -EINVAL;
	}
	ret = mla_sg_from_contiguous_pages(&a->table,
					   virt_to_page(buf->cpu_vaddr), buf->size);
	if (ret) {
		kfree(a);
		return ret;
	}

	/* dma_alloc_noncoherent returns a linear-map address with real
	 * pages behind it, so the sg entry can be built from cpu_vaddr
	 * and mapped for the attaching device the ordinary way.
	 * mla_dmabuf_mmap relies on the same property.
	 */
	attachment->priv = a;
	return 0;
}

static void mla_dmabuf_detach(struct dma_buf *dmabuf,
			      struct dma_buf_attachment *attachment)
{
	struct mla_dmabuf_attachment *a = attachment->priv;

	sg_free_table(&a->table);
	kfree(a);
}

static struct sg_table *mla_dmabuf_map(struct dma_buf_attachment *attachment,
				       enum dma_data_direction direction)
{
	struct mla_dmabuf_attachment *a = attachment->priv;
	struct sg_table *table = &a->table;
	int ret;

	ret = dma_map_sgtable(attachment->dev, table, direction, 0);
	if (ret)
		return ERR_PTR(ret);
	a->mapped = true;
	return table;
}

static void mla_dmabuf_unmap(struct dma_buf_attachment *attachment,
			     struct sg_table *table,
			     enum dma_data_direction direction)
{
	struct mla_dmabuf_attachment *a = attachment->priv;

	dma_unmap_sgtable(attachment->dev, table, direction, 0);
	a->mapped = false;
}

static int mla_dmabuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct mla_buffer *buf = dmabuf->priv;
	/* Userspace can still mmap an fd whose context has been torn down;
	 * mla_buf_dev handles the ctx==NULL case.
	 */
	struct device *dev = mla_buf_dev(buf);
	unsigned long user_size = vma->vm_end - vma->vm_start;
	unsigned long user_off_bytes;
	unsigned long pfn;

	if ((vma->vm_flags & (VM_SHARED | VM_MAYSHARE)) == 0)
		return -EINVAL;

	/* The buffer was allocated via dma_alloc_noncoherent — WB
	 * cacheable linear-map alias on ARM64. Map the user vma with
	 * matching WB cacheable attributes via remap_pfn_range on the
	 * underlying pages; the kernel API has no dma_mmap_noncoherent
	 * helper. dma_mmap_coherent would splice pgprot_dmacoherent (WC)
	 * into the vma — userspace writes would land in a different
	 * cache domain than kernel reads, and kernel reads
	 * (v2_apply_static_section_relocs, mla_model_uses_semaphores_v2)
	 * would see stale bytes.
	 *
	 * Constraints checked below:
	 *   - virt_addr_valid: dma_alloc_noncoherent must return a
	 *     linear-map address for virt_to_page to be valid. Non-linear-
	 *     map backings (future vmalloc-like allocator) would silently
	 *     corrupt page-table entries; refuse loudly instead.
	 *   - user_size + vm_pgoff must fit within buf->size. The
	 *     dma-buf-core wrapper (dma_buf_mmap_internal) already checks
	 *     the byte-count bound; re-check here so a direct exporter
	 *     call (mla_dmabuf_ops.mmap) can't bypass it.
	 *   - vma->vm_page_prot is WB cacheable on ARM64 for a normal
	 *     PROT_WRITE|MAP_SHARED userspace mmap. dma_buf_mmap_internal
	 *     does NOT override it (verified against
	 *     drivers/dma-buf/dma-buf.c:205-223), so we trust the syscall
	 *     path's default.
	 */
	if (!virt_addr_valid(buf->cpu_vaddr)) {
		dev_warn_ratelimited(dev,
			"%s: buf=%p cpu_vaddr=%p not in linear map — refusing\n",
			__func__, buf, buf->cpu_vaddr);
		return -EINVAL;
	}
	if (vma->vm_pgoff > (ULONG_MAX >> PAGE_SHIFT))
		return -EINVAL;
	user_off_bytes = vma->vm_pgoff << PAGE_SHIFT;
	if (user_off_bytes >= buf->size ||
	    user_size > buf->size - user_off_bytes)
		return -EINVAL;
	pfn = page_to_pfn(virt_to_page((u8 *)buf->cpu_vaddr + user_off_bytes));
	return remap_pfn_range(vma, vma->vm_start, pfn,
			       user_size, vma->vm_page_prot);
}

static int mla_dmabuf_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct mla_buffer *buf = dmabuf->priv;

	iosys_map_set_vaddr(map, buf->cpu_vaddr);
	return 0;
}

static void mla_dmabuf_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	iosys_map_clear(map);
}

static int mla_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
				       enum dma_data_direction direction)
{
	struct mla_buffer *buf = dmabuf->priv;

	/* dma_sync_single_for_cpu to let the CPU see any device writes.
	 * Callers can invoke this via DMA_BUF_IOCTL_SYNC(SYNC_START).
	 */
	if (buf && buf->cma_backed)
		dma_sync_single_for_cpu(mla_buf_dev(buf),
					buf->dma_addr, buf->size, buf->dma_dir);
	return 0;
}

static int mla_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
				     enum dma_data_direction direction)
{
	struct mla_buffer *buf = dmabuf->priv;

	/* Userspace calls DMA_BUF_IOCTL_SYNC(SYNC_END|WRITE) after memcpy
	 * to flush dirty cache lines before munmap, ensuring the device sees
	 * the section bytes at MODEL_DEFINE / JOB_SUBMIT time.
	 */
	if (buf && buf->cma_backed)
		dma_sync_single_for_device(
			mla_buf_dev(buf),
			buf->dma_addr, buf->size, buf->dma_dir);
	return 0;
}

static void mla_dmabuf_release(struct dma_buf *dmabuf)
{
	struct mla_buffer *buf = dmabuf->priv;
	struct mla_device *mdev;

	if (!buf)
		return;

	/* This function runs after the fd closes: mla_context_shutdown may have
	 * already run. buf->ctx is nevertheless valid because mla_buffer_register
	 * took a counted reference on the context, and mla_buffer_free (below)
	 * releases it. The dma-buf refcount keeps buf alive until then.
	 * buf->mdev is the same device, cached to save a hop.
	 */
	mdev = buf->mdev;   /* always set; see mla_buf_dev() */

	/* Invariant check: xarray registration holds a counted reference on buf,
	 * so release cannot run before the xa_erase in mla_op_buf_release /
	 * mla_context_buffers_release (see docs/BUFFER_LIFETIME_DESIGN.md §I2).
	 * A non-NULL erase means the invariant broke; warn loudly rather than
	 * silently leak the registered buffer.
	 */
	if (buf->ctx && buf->id) {
		void *erased = xa_erase(&buf->ctx->buffers, buf->id);

		WARN_ONCE(erased,
			  "mla: dma-buf released while buf %u still registered — kernel dma-buf ref missing\n",
			  buf->id);
	}

	/* This exporter backs only CMA (CPU_RMW) section buffers, all
	 * cma_backed via dma_alloc_noncoherent — carveout buffers never carry
	 * mla_dmabuf_ops, so there is no dma_free_coherent case here.
	 */
	if (mdev && buf->cpu_vaddr)
		dma_free_noncoherent(mdev->dev, buf->size,
				     buf->cpu_vaddr, buf->dma_addr,
				     buf->dma_dir);
	/* mla_buffer_free() does the kfree AND calls mla_context_put() on
	 * buf->ctx, balancing the mla_context_get() in mla_buffer_register.
	 * For a buffer that outlived the fd, that put is the LAST reference,
	 * so this is where the context is finally freed. Do not open-code a
	 * kfree(buf) here — the put would be silently lost and the context
	 * would leak once per such buffer.
	 */
	mla_buffer_free(buf);
}

static const struct dma_buf_ops mla_dmabuf_ops = {
	.attach           = mla_dmabuf_attach,
	.detach           = mla_dmabuf_detach,
	.map_dma_buf      = mla_dmabuf_map,
	.unmap_dma_buf    = mla_dmabuf_unmap,
	.begin_cpu_access = mla_dmabuf_begin_cpu_access,
	.end_cpu_access   = mla_dmabuf_end_cpu_access,
	.mmap             = mla_dmabuf_mmap,
	.vmap             = mla_dmabuf_vmap,
	.vunmap           = mla_dmabuf_vunmap,
	.release          = mla_dmabuf_release,
};

/**
 * mla_secbuf_alloc_cached() — allocate a WB-cached CMA section buffer
 * @mdev: owning device
 * @size: requested length (page-aligned by this function)
 *
 * Used by MLA_OP_SECTION_ALLOC for CPU_RMW-flagged sections
 * (dma.descs). The returned mla_buffer carries `cma_backed=true` and a
 * dma-buf exported with mla_dmabuf_ops: begin/end_cpu_access route to
 * dma_sync_single_for_{cpu,device}, and mla_dmabuf_release frees the
 * memory via dma_free_noncoherent when the last dma-buf reference
 * drops.
 *
 * Uses the shared DMA heap when available, or the original DMS heap when
 * mla_desc_cma_enable is 0. Page-backed heap allocations retain
 * range-scoped flushes; legacy no-map DMS uses exporter cache hooks.
 *
 * Direction: DMA_TO_DEVICE — descriptor blobs are CPU-writes-DMA-reads.
 *
 * Return: mla_buffer pointer on success; ERR_PTR(-ENOMEM) on
 * allocation failure, etc.
 */
struct mla_buffer *mla_secbuf_alloc_cached(struct mla_device *mdev,
					   size_t size)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct mla_buffer *buf;
	struct dma_buf    *dmabuf;
	int ret;
	size_t aligned;

	if (mla_memory_size(size, &aligned))
		return ERR_PTR(-EINVAL);

	if (!mla_desc_cma_enable || mdev->heap != mdev->dms_heap)
		return mla_heap_alloc(mdev, size, MLA_HEAP_FLAG_CACHED);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	refcount_set(&buf->refcount, 1);
	atomic64_inc(&mdev->buf_inc_ops);
	buf->size       = aligned;
	buf->alignment  = PAGE_SIZE;
	buf->flags      = MLA_HEAP_FLAG_CACHED;
	buf->cma_backed = true;
	buf->dma_dir    = DMA_TO_DEVICE;
	buf->mdev       = mdev;   /* always set; mla_dmabuf_release reads it,
				   * and survives ctx teardown (mla_buf_dev())
				   */

	buf->cpu_vaddr = dma_alloc_noncoherent(mdev->dev, buf->size,
					       &buf->dma_addr,
					       buf->dma_dir,
					       GFP_KERNEL | __GFP_NOWARN);
	if (!buf->cpu_vaddr) {
		ret = -ENOMEM;
		goto err_free_buf;
	}

	ret = mla_dma_range_check(buf->dma_addr, buf->size, PAGE_SIZE);
	if (ret)
		goto err_free_dma;

	exp_info.exp_name = "mla";
	exp_info.owner    = THIS_MODULE;
	exp_info.ops      = &mla_dmabuf_ops;
	exp_info.size     = buf->size;
	exp_info.flags    = O_RDWR | O_CLOEXEC;
	exp_info.priv     = buf;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err_free_dma;
	}

	buf->dmabuf = dmabuf;
	return buf;

err_free_dma:
	dma_free_noncoherent(mdev->dev, buf->size, buf->cpu_vaddr,
			     buf->dma_addr, buf->dma_dir);
err_free_buf:
	atomic64_inc(&mdev->buf_dec_ops); /* balance buf_inc above */
	kfree(buf);
	return ERR_PTR(ret);
}
