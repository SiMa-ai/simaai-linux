// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// Kernel-owned pool helpers and DMA-heap integration.
//
// The MLA driver shares the DT-declared DMS DMA heap with userspace
// (/dev/dma_heap/simaai,dms). The selected DT profile determines whether
// the backing is a legacy no-map carveout or reusable, page-backed CMA.
// mla_heap_init() keeps this DMS identity and prefers the shared simaai,mla
// fallback heap when available.
// mla_heap_alloc() wraps a heap dma-buf in the mla_buffer shape the
// rest of the driver consumes:
//
//   - mla_heap_alloc(mdev, size, flags)
//        heap dma-buf + attach/map to mdev->dev (dma_addr) + vmap
//        (cpu_vaddr), refcount=1, heap_backed=true. The mla_buffer
//        owns one dmabuf reference; userspace fds (BUF_ALLOC) take
//        their own.
//   - mla_heap_free(mdev, buf)
//        symmetric teardown: vunmap + unmap + detach + dma_buf_put +
//        kfree.
//   - mla_pool_alloc(mdev, size)
//        mla_heap_alloc without a userspace fd, marked
//        pool_owned=true and counted in outstanding_alloc_bytes.
//   - mla_pool_free(mdev, buf)
//        symmetric teardown: carveout teardown for heap_backed,
//        dma_free_noncoherent for cma_backed, then mla_buffer_free
//        (the sole kfree site, which also releases the ctx ref).
//   - mla_pool_copy_from_image_fd(buf, image_fd, src_off, size)
//        copies size bytes from the source fd into buf->cpu_vaddr.
//        Source fd may be a dma-buf (preferred — vmap + memcpy) or a
//        regular file fd (fallback — kernel_read into buffer).
//
// Carveout memory is WB-cacheable and lies outside the kernel's view
// of RAM (no-map, no struct pages, no linear-map alias), so cache
// maintenance must go through the exporter's {begin,end}_cpu_access
// ops — mla_pool_sync_for_device dispatches accordingly. The
// dma_sync_* family would phys_to_virt() into unmapped territory and
// fault.
//
// Pool buffers and fd-carrying dma-buf buffers coexist in the same
// per-ctx xarray; mla_buffer_drop differentiates via buf->pool_owned.

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/iosys-map.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/rculist.h>
#include <linux/rcupdate.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include "mla_drv.h"
#include "mla_memory.h"

/* Descriptor cache policy, fixed at module load (0444). When non-zero,
 * mla_pool_alloc_cached uses dma_alloc_noncoherent to obtain a WB
 * (cacheable) linear-map alias for descriptor-blob sections; callers
 * must pair every DMA handoff with mla_pool_sync_for_device. When zero
 * (or on !CONFIG_DMA_NONCOHERENT platforms where the API returns
 * uncached), mla_pool_alloc_cached falls straight through to
 * mla_pool_alloc. Defined in mla_drv.c alongside the other params.
 */
extern uint mla_desc_cma_enable;

/* Content-hash dedup entry. Opaque to callers; only this file and the
 * section-load handler walk fields.
 */
struct mla_section_dedup_entry {
	struct hlist_node       hnode;       /* into mdev->sec_ddp.bucket[] */
	struct mla_buffer      *buf;         /* shared primary; pool-owned */
	u8                      sha256[32];
	/* Part of the key, not just metadata: a hash match alone doesn't
	 * guarantee equal size, and aliasing a section to a primary of the
	 * wrong size would read as a truncated or padded hit.
	 *
	 * Page-granular because that's the primary's PAGE_ALIGN'd allocation
	 * size, and identical content implies identical alignment, so this
	 * never rejects a legitimate hit.
	 */
	size_t                  size;
	refcount_t              refcount;    /* contexts holding this entry */
	struct mla_device      *mdev;        /* back-pointer for RCU free path */
	struct rcu_head         rcu;
};

struct mla_buffer *mla_section_dedup_entry_buf(
	const struct mla_section_dedup_entry *e)
{
	return e ? e->buf : NULL;
}

const u8 *mla_section_dedup_entry_sha256(
	const struct mla_section_dedup_entry *e)
{
	return e ? e->sha256 : NULL;
}

unsigned int mla_section_dedup_entry_refcount(
	const struct mla_section_dedup_entry *e)
{
	return e ? refcount_read(&e->refcount) : 0;
}

void mla_section_dedup_walk(struct mla_device *mdev,
			    mla_dedup_walker_cb cb, void *arg)
{
	struct mla_section_dedup_entry *e;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&mdev->sec_ddp.lock, flags);
	for (i = 0; i < MLA_SECTION_DEDUP_BUCKETS; i++) {
		hlist_for_each_entry(e, &mdev->sec_ddp.bucket[i], hnode)
			cb(e, refcount_read(&e->refcount), arg);
	}
	spin_unlock_irqrestore(&mdev->sec_ddp.lock, flags);
}

/* ----------------------------------------------------------------- *
 *  Carveout dma-heap consumer glue.
 * -----------------------------------------------------------------
 */

/**
 * mla_heap_init() - resolve the DT-owned DMA heap and its backing
 * @mdev: device being probed
 *
 * Keep the original DMS backing and attach reusable CMA to the device.
 * Prefer the shared simaai,mla heap when available, otherwise DMS itself.
 * Missing bindings preserve import-only operation; malformed pools fail
 * rather than silently allocating from default CMA.
 */
int mla_heap_init(struct mla_device *mdev)
{
	struct reserved_mem *rmem = NULL;
	struct device_node *np;
	struct dma_heap *heap, *dms_heap;
	const char *name;
	bool reusable;
	int ret = 0;

	np = of_parse_phandle(mdev->dev->of_node, "memory-region", 0);
	if (!np)
		return 0;

	reusable = of_device_is_compatible(np, "simaai,mla-cma");
	if (reusable) {
		const char *invalid = NULL;

		if (!of_device_is_compatible(np, "shared-dma-pool"))
			invalid = "missing shared-dma-pool compatible";
		else if (!of_property_read_bool(np, "reusable"))
			invalid = "missing reusable property";
		else if (of_property_read_bool(np, "no-map"))
			invalid = "no-map is not allowed";
		else if (of_property_read_bool(np, "linux,cma-default"))
			invalid = "linux,cma-default is not allowed";
		else if (of_property_read_bool(np, "linux,dma-default"))
			invalid = "linux,dma-default is not allowed";
		else if (of_device_is_compatible(np, "carved-out"))
			invalid = "carved-out compatible is not allowed";
		if (invalid) {
			ret = dev_err_probe(mdev->dev, -EINVAL,
					    "invalid MLA CMA pool %pOF: %s\n",
					    np, invalid);
			goto out;
		}
		rmem = of_reserved_mem_lookup(np);
		if (!rmem || !rmem->ops || !rmem->priv) {
			ret = dev_err_probe(mdev->dev, -ENODEV,
					    "MLA CMA pool %pOF is not initialized\n", np);
			goto out;
		}
	} else if (!of_device_is_compatible(np, "carved-out")) {
		/* A present but unsupported pool must not silently fall back. */
		ret = dev_err_probe(mdev->dev, -EINVAL,
				    "unsupported MLA memory-region %pOF\n", np);
		goto out;
	}

	name = np->full_name;
	of_property_read_string(np, "label", &name);
	dms_heap = dma_heap_find(name);
	if (!dms_heap) {
		ret = dev_err_probe(mdev->dev, -EPROBE_DEFER,
				    "DMA heap \"%s\" not registered\n", name);
		goto out;
	}

	/* memory-region still identifies and attaches the original DMS pool. */
	heap = dma_heap_find("simaai,mla");
	if (!heap)
		heap = dms_heap;

	if (reusable) {
		ret = of_reserved_mem_device_init(mdev->dev);
		if (ret) {
			dev_err_probe(mdev->dev, ret, "cannot attach MLA CMA pool %pOF\n", np);
			goto out;
		}
		if (mdev->dev->cma_area != rmem->priv) {
			of_reserved_mem_device_release(mdev->dev);
			ret = dev_err_probe(mdev->dev, -EINVAL,
					    "attached CMA area does not match MLA pool %pOF\n", np);
			goto out;
		}
		mdev->reserved_mem_attached = true;
	}
	/* Region identity for the OOM notifier; the carveout path skips the
	 * lookup above.
	 */
	if (!rmem)
		rmem = of_reserved_mem_lookup(np);
	if (rmem) {
		mdev->pool_name = rmem->name;
		mdev->pool_size = rmem->size;
	}

	mdev->heap = heap;
	mdev->dms_heap = dms_heap;
	mdev->dms_is_carveout = !reusable;
	dev_info(mdev->dev, "allocating from DMA heap \"%s\" (DMS %s heap \"%s\")\n",
		 dma_heap_get_name(heap), reusable ? "reusable CMA" : "carveout",
		 dma_heap_get_name(dms_heap));
out:
	of_node_put(np);
	return ret;
}

/* The shared heap returns DMS allocations with their original exporter. */
bool mla_dmabuf_is_dms(struct mla_device *mdev, struct dma_buf *dmabuf)
{
	return mdev->dms_heap && dmabuf->exp_name &&
	       !strcmp(dmabuf->exp_name, dma_heap_get_name(mdev->dms_heap));
}

bool mla_dmabuf_no_cpu_alias(struct mla_device *mdev, struct dma_buf *dmabuf)
{
	return mdev->dms_is_carveout && mla_dmabuf_is_dms(mdev, dmabuf);
}

/**
 * mla_heap_alloc() - allocate an mla_buffer from the selected DMA heap
 * @mdev:  owning device
 * @size:  requested size; page-aligned by this function
 * @flags: MLA_HEAP_FLAG_* recorded on the buffer (informational; all
 *         carveout memory is WB-cacheable)
 *
 * Allocates a dma-buf from mdev->heap, attaches + maps it for
 * mdev->dev (buf->dma_addr from the contiguous mapped extent) and vmaps
 * it (buf->cpu_vaddr). The returned mla_buffer owns one dmabuf
 * reference and the attach/map/vmap; mla_heap_free() tears all of
 * it down. Only buffers exported by the original no-map DMS heap lack a
 * linear-map alias; shared-heap fallback buffers are page-backed.
 *
 * Return: pointer on success; ERR_PTR(-ENODEV) when no heap was found
 * at probe, ERR_PTR(-EINVAL)/(-ENOMEM)/... otherwise.
 */
struct mla_buffer *mla_heap_alloc(struct mla_device *mdev, size_t size,
				      u32 flags)
{
	struct dma_buf_attachment *attach;
	struct mla_buffer *buf;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;
	struct iosys_map map;
	int ret;

	size_t aligned;

	if (mla_memory_size(size, &aligned))
		return ERR_PTR(-EINVAL);
	if (!mdev->heap)
		return ERR_PTR(-ENODEV);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	refcount_set(&buf->refcount, 1);
	atomic64_inc(&mdev->buf_inc_ops);
	buf->size         = aligned;
	buf->alignment    = PAGE_SIZE;
	buf->flags        = flags;
	buf->heap_backed  = true;
	buf->mdev         = mdev;   /* survives ctx teardown; see mla_buf_dev() */

	dmabuf = dma_heap_buffer_alloc(mdev->heap, buf->size,
				       O_RDWR | O_CLOEXEC, 0);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err_free_buf;
	}
	buf->dmabuf = dmabuf;
	buf->no_cpu_alias = mla_dmabuf_no_cpu_alias(mdev, dmabuf);

	attach = dma_buf_attach(dmabuf, mdev->dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		goto err_put;
	}
	buf->attach = attach;

	sgt = dma_buf_map_attachment_unlocked(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_detach;
	}
	ret = mla_sg_dma_address(sgt, buf->size, PAGE_SIZE, &buf->dma_addr);
	if (ret)
		goto err_unmap;
	buf->sgt = sgt;

	ret = dma_buf_vmap_unlocked(dmabuf, &map);
	if (ret)
		goto err_unmap;
	if (map.is_iomem || !map.vaddr) {
		dma_buf_vunmap_unlocked(dmabuf, &map);
		ret = -EINVAL;
		goto err_unmap;
	}
	buf->cpu_vaddr = map.vaddr;

	return buf;

err_unmap:
	dma_buf_unmap_attachment_unlocked(attach, sgt, DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(dmabuf, attach);
err_put:
	dma_buf_put(dmabuf);
err_free_buf:
	atomic64_inc(&mdev->buf_dec_ops); /* balance the inc above */
	kfree(buf);
	return ERR_PTR(ret);
}

/**
 * mla_heap_free() - release a heap-backed mla_buffer
 * @mdev: owning device (may be NULL on late teardown; unused)
 * @buf:  buffer to free; must have heap_backed=true
 *
 * Drops the vmap, the device mapping/attachment and the mla_buffer's
 * dmabuf reference, then frees the struct. The heap memory itself is
 * returned to its allocator when the last dmabuf reference drops
 * (a userspace fd from BUF_ALLOC may outlive the mla_buffer).
 */
void mla_heap_free(struct mla_device *mdev, struct mla_buffer *buf)
{
	struct iosys_map map;

	if (!buf)
		return;

	if (buf->cpu_vaddr) {
		iosys_map_set_vaddr(&map, buf->cpu_vaddr);
		dma_buf_vunmap_unlocked(buf->dmabuf, &map);
	}
	if (buf->attach && buf->sgt)
		dma_buf_unmap_attachment_unlocked(buf->attach, buf->sgt,
						  DMA_BIDIRECTIONAL);
	if (buf->attach)
		dma_buf_detach(buf->dmabuf, buf->attach);
	if (buf->dmabuf)
		dma_buf_put(buf->dmabuf);
	/* mla_buffer_free is the sole kfree site and releases the context
	 * reference mla_buffer_register took (BUFFER_LIFETIME_DESIGN §4.4).
	 * A raw kfree(buf) here would leak one mla_context per registered
	 * carveout buffer. mla_buffer_free is NULL-ctx safe (mla_context_put
	 * ignores NULL), so it is also correct on the pre-registration error
	 * paths that reach here via mla_buffer_drop.
	 */
	mla_buffer_free(buf);
}

/**
 * mla_pool_alloc() — allocate a pool-owned mla_buffer
 * @mdev: device the allocation belongs to (memory comes from the
 *        DMA heap over its DT-bound memory-region)
 * @size: requested size; will be page-aligned
 *
 * mla_heap_alloc with pool_owned=true and no userspace fd — the
 * kernel-side identity used by the section-load handler. Refcount is
 * initialized to 1 — the caller owns that reference until
 * mla_buffer_register hands it to a context (after which the
 * per-context xarray entry / xa_erase governs the lifecycle).
 *
 * Return: pointer on success; ERR_PTR(-EINVAL) for size==0,
 * ERR_PTR(-ENOMEM)/(-ENODEV) on allocation failure.
 */
struct mla_buffer *mla_pool_alloc(struct mla_device *mdev, size_t size)
{
	struct mla_buffer *buf = mla_heap_alloc(mdev, size, 0);

	if (IS_ERR(buf))
		return buf;

	buf->pool_owned = true;
	atomic64_add(buf->size, &mdev->outstanding_alloc_bytes);
	return buf;
}

/**
 * mla_pool_alloc_cached() — allocate a pool-owned WB descriptor buffer
 * @mdev: device the allocation belongs to
 * @size: requested size; will be page-aligned
 *
 * The shared heap handles descriptors too when available. Otherwise the
 * default DMA-noncoherent allocation supplies a WB linear-map alias for fast
 * CPU RMW. mla_desc_cma_enable=0 always uses the selected heap and full-buffer
 * exporter flushes. Normal page-backed descriptors retain range-scoped sync;
 * no-map DMS buffers require the exporter's cache-maintenance hooks.
 * Callers must publish CPU writes before every DMA handoff.
 *
 * Return: pointer on success; ERR_PTR(-EINVAL) for invalid size,
 * ERR_PTR(-ENOMEM)/(-ENODEV) on allocation failure.
 */
struct mla_buffer *mla_pool_alloc_cached(struct mla_device *mdev, size_t size)
{
	struct mla_buffer *buf;
	size_t aligned;
	int ret;

	if (mla_memory_size(size, &aligned))
		return ERR_PTR(-EINVAL);

	if (!mla_desc_cma_enable || mdev->heap != mdev->dms_heap)
		return mla_pool_alloc(mdev, size);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	refcount_set(&buf->refcount, 1);
	atomic64_inc(&mdev->buf_inc_ops);
	buf->size       = aligned;
	buf->alignment  = PAGE_SIZE;
	buf->pool_owned = true;
	buf->mdev       = mdev;   /* survives ctx teardown; see mla_buf_dev() */
	buf->cma_backed = true;
	buf->dma_dir    = DMA_TO_DEVICE;

	buf->cpu_vaddr = dma_alloc_noncoherent(mdev->dev, buf->size,
					       &buf->dma_addr,
					       buf->dma_dir,
					       GFP_KERNEL | __GFP_NOWARN);
	if (!buf->cpu_vaddr) {
		atomic64_inc(&mdev->buf_dec_ops); /* balance the set(1) */
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}
	ret = mla_dma_range_check(buf->dma_addr, buf->size, PAGE_SIZE);
	if (ret) {
		dma_free_noncoherent(mdev->dev, buf->size, buf->cpu_vaddr,
				     buf->dma_addr, buf->dma_dir);
		atomic64_inc(&mdev->buf_dec_ops);
		kfree(buf);
		return ERR_PTR(ret);
	}
	atomic64_add(buf->size, &mdev->outstanding_alloc_bytes);
	return buf;
}

/**
 * mla_pool_sync_for_device() — publish CPU-side writes to a buffer to
 * memory before the device reads them
 * @mdev: owning device
 * @buf:  buffer to sync
 *
 * cma_backed buffers (WB linear-map alias) clean the D-cache via
 * dma_sync_single_for_device. Heap buffers use the exporter's
 * end_cpu_access, which handles both normal pages and legacy no-map
 * memory without passing the latter to dma_sync_*. It is safe to call this
 * unconditionally over a mixed section_bufs[] array.
 */
void mla_pool_sync_for_device(struct mla_device *mdev, struct mla_buffer *buf)
{
	if (!buf)
		return;
	if (buf->cma_backed)
		dma_sync_single_for_device(mdev->dev, buf->dma_addr, buf->size,
					   buf->dma_dir);
	else if (buf->heap_backed && buf->dmabuf)
		dma_buf_end_cpu_access(buf->dmabuf, DMA_TO_DEVICE);
}

/**
 * mla_pool_sync_range_for_device() — flush a sub-range of a pool
 * buffer to DMA
 * @mdev:   owning device
 * @buf:    buffer to sync
 * @offset: starting byte offset within @buf
 * @size:   bytes to flush
 *
 * Bounds are clamped to @buf->size so callers don't need to worry
 * about page-alignment on either end — dma_sync_single_range_for_device
 * rounds outward to cache lines internally. Zero @size is a no-op.
 *
 * Page-backed heap buffers use their checked contiguous DMA mapping too.
 * Legacy no-map DMS buffers require the exporter's full-buffer clean.
 */
void mla_pool_sync_range_for_device(struct mla_device *mdev,
				    struct mla_buffer *buf,
				    size_t offset, size_t size)
{
	struct scatterlist *sg;
	unsigned int i;

	if (!buf || !size || offset >= buf->size)
		return;
	if (size > buf->size - offset)
		size = buf->size - offset;
	if (buf->cma_backed) {
		dma_sync_single_range_for_device(mdev->dev, buf->dma_addr,
						 offset, size, buf->dma_dir);
		return;
	}
	if (!mla_desc_cma_enable || mdev->heap == mdev->dms_heap ||
	    !buf->heap_backed || buf->no_cpu_alias) {
		mla_pool_sync_for_device(mdev, buf);
		return;
	}
	/* Preserve each mapped SG entry's DMA range and mapping direction. */
	for_each_sg(buf->sgt->sgl, sg, buf->sgt->nents, i) {
		size_t length = sg_dma_len(sg);
		size_t part;

		if (offset >= length) {
			offset -= length;
			continue;
		}
		part = min(size, length - offset);
		dma_sync_single_range_for_device(mdev->dev, sg_dma_address(sg),
						 offset, part, DMA_BIDIRECTIONAL);
		size -= part;
		if (!size)
			break;
		offset = 0;
	}
}

/**
 * mla_pool_free() — release a pool-owned mla_buffer
 * @mdev: device that produced the allocation
 * @buf:  buffer to free; must have a non-NULL cpu_vaddr (allocator
 *        postcondition)
 *
 * Releases the backing (carveout dma-buf for heap_backed,
 * dma_free_noncoherent for cma_backed) and the mla_buffer struct.
 * Caller must have already removed the buffer from the per-context
 * xarray (xa_erase) so the registry doesn't dangle.
 */
void mla_pool_free(struct mla_device *mdev, struct mla_buffer *buf)
{
	if (!buf)
		return;
	if (buf->cpu_vaddr && buf->pool_owned)
		atomic64_sub(buf->size, &mdev->outstanding_alloc_bytes);
	if (buf->heap_backed) {
		mla_heap_free(mdev, buf);
		return;
	}
	if (buf->cpu_vaddr && buf->cma_backed)
		dma_free_noncoherent(mdev->dev, buf->size,
				     buf->cpu_vaddr, buf->dma_addr,
				     buf->dma_dir);
	/* Route the struct through mla_buffer_free so the context reference
	 * taken at registration is released — a pool buffer registered in a
	 * ctx (bank blobs via mla_ops.c, section primaries) holds one.
	 * mla_buffer_free is the sole kfree site (BUFFER_LIFETIME_DESIGN §4.4).
	 */
	mla_buffer_free(buf);
}




/* ----------------------------------------------------------------- *
 *  Content-hash dedup table.
 *
 *  Per-device hash table keyed by SHA-256 of section content. When two
 *  contexts (or two loads in the same context) call
 *  MLA_OP_MODEL_LOAD_SECTIONS with directives whose hash matches an
 *  already-registered entry — and they both opted in via
 *  MLA_SECTION_FLAG_DEDUP_OK — the second call gets a shell mla_buffer
 *  aliased to the existing primary's memory, saving the DMA pool
 *  allocation + copy.
 * -----------------------------------------------------------------
 */

static u32 mla_dedup_bucket_idx(const u8 sha256[32])
{
	/* SHA-256 first 4 bytes are uniformly distributed; mod-bucket is
	 * fine. The bucket count (512) is power-of-2 so the modulo is a
	 * single AND, but %= keeps the intent obvious and works for any
	 * future re-tune.
	 */
	u32 v = ((u32)sha256[0] << 24) | ((u32)sha256[1] << 16) |
		((u32)sha256[2] << 8)  |  (u32)sha256[3];
	return v % MLA_SECTION_DEDUP_BUCKETS;
}

void mla_section_dedup_init(struct mla_device *mdev)
{
	int i;

	spin_lock_init(&mdev->sec_ddp.lock);
	for (i = 0; i < MLA_SECTION_DEDUP_BUCKETS; i++)
		INIT_HLIST_HEAD(&mdev->sec_ddp.bucket[i]);
}

void mla_section_dedup_destroy(struct mla_device *mdev)
{
	struct hlist_node *tmp;
	struct mla_section_dedup_entry *e;
	HLIST_HEAD(leaked_list);
	unsigned long flags;
	int i;
	int leaks = 0;

	/* All contexts on this device must already be torn down by the
	 * time we get here (miscdev release walks contexts). Any entry
	 * still in the table is a refcount imbalance; splice everything
	 * onto a local list under the lock, then log + free outside.
	 */
	spin_lock_irqsave(&mdev->sec_ddp.lock, flags);
	for (i = 0; i < MLA_SECTION_DEDUP_BUCKETS; i++) {
		hlist_for_each_entry_safe(e, tmp,
					  &mdev->sec_ddp.bucket[i], hnode) {
			hlist_del_init(&e->hnode);
			hlist_add_head(&e->hnode, &leaked_list);
			leaks++;
		}
	}
	spin_unlock_irqrestore(&mdev->sec_ddp.lock, flags);

	if (leaks)
		dev_warn(mdev->dev,
			 "section-dedup: %d entries leaked at teardown\n",
			 leaks);

	i = 0;
	hlist_for_each_entry_safe(e, tmp, &leaked_list, hnode) {
		hlist_del_init(&e->hnode);
		dev_warn(mdev->dev,
			 "LEAKED #%d entry=%p refcount=%u sha=%02x%02x%02x%02x%02x%02x%02x%02x buf=%p size=%zu\n",
			 ++i, e, refcount_read(&e->refcount),
			 e->sha256[0], e->sha256[1], e->sha256[2],
			 e->sha256[3], e->sha256[4], e->sha256[5],
			 e->sha256[6], e->sha256[7],
			 e->buf, e->buf ? e->buf->size : 0);
		if (e->buf) {
			e->buf->dedup = NULL;
			mla_pool_free(mdev, e->buf);
		}
		kfree(e);
	}
}

struct mla_section_dedup_entry *
mla_section_dedup_lookup(struct mla_device *mdev, const u8 sha256[32],
			 size_t size)
{
	u32 idx = mla_dedup_bucket_idx(sha256);
	struct mla_section_dedup_entry *e, *found = NULL;
	unsigned long flags;

	size = PAGE_ALIGN(size);

	spin_lock_irqsave(&mdev->sec_ddp.lock, flags);
	hlist_for_each_entry(e, &mdev->sec_ddp.bucket[idx], hnode) {
		/* Hash AND size — see the size field's comment. */
		if (e->size == size &&
		    memcmp(e->sha256, sha256, 32) == 0) {
			/* An entry can be LINKED and already at zero: see
			 * mla_section_dedup_put(). Skip a corpse rather than
			 * resurrect it — refcount_inc() on 0 is a saturating
			 * REFCOUNT_WARN and we would hand back an entry whose
			 * primary is about to be freed.
			 *
			 * Deliberately no retry loop: the dying entry stays
			 * linked until the putter reacquires this very lock, so
			 * a retry would spin holding it. Report "not found"
			 * instead; the caller allocates a fresh entry.
			 */
			if (!refcount_inc_not_zero(&e->refcount))
				continue;
			found = e;
			break;
		}
	}
	spin_unlock_irqrestore(&mdev->sec_ddp.lock, flags);
	return found;
}

struct mla_section_dedup_entry *
mla_section_dedup_insert(struct mla_device *mdev,
			 const u8 sha256[32],
			 struct mla_buffer *primary)
{
	struct mla_section_dedup_entry *fresh, *winner = NULL;
	struct mla_section_dedup_entry *cur;
	u32 idx = mla_dedup_bucket_idx(sha256);
	unsigned long flags;

	fresh = kzalloc(sizeof(*fresh), GFP_KERNEL);
	if (!fresh)
		return ERR_PTR(-ENOMEM);
	memcpy(fresh->sha256, sha256, 32);
	fresh->buf  = primary;
	fresh->mdev = mdev;
	/* Already PAGE_ALIGN'd by the allocator; align again so the key is
	 * normalised at exactly one place no matter who calls us.
	 */
	fresh->size = PAGE_ALIGN(primary->size);
	refcount_set(&fresh->refcount, 1);

	spin_lock_irqsave(&mdev->sec_ddp.lock, flags);
	hlist_for_each_entry(cur, &mdev->sec_ddp.bucket[idx], hnode) {
		/* Hash AND size, matching mla_section_dedup_lookup(): the
		 * race-loser test must use the same key as the finder, or two
		 * same-SHA different-size sections would each think they won.
		 */
		if (cur->size == fresh->size &&
		    memcmp(cur->sha256, sha256, 32) == 0) {
			/* Same corpse-skip as mla_section_dedup_lookup().
			 *
			 * Consequence worth knowing: skipping a dying entry
			 * means @fresh is added to a bucket that still, briefly,
			 * holds an entry with the same SHA. hlist_add_head puts
			 * @fresh at the head, so every subsequent lookup finds
			 * the live one, and the dying one is unlinked as soon as
			 * the putter retakes this lock. A transient duplicate,
			 * never a wrong answer.
			 */
			if (!refcount_inc_not_zero(&cur->refcount))
				continue;
			winner = cur;
			break;
		}
	}
	if (!winner) {
		hlist_add_head(&fresh->hnode, &mdev->sec_ddp.bucket[idx]);
		primary->dedup = fresh;     /* publish back-pointer under lock */
		winner = fresh;
	}
	spin_unlock_irqrestore(&mdev->sec_ddp.lock, flags);

	if (winner != fresh)
		kfree(fresh);               /* lost the race; caller frees primary */
	return winner;
}

static void mla_section_dedup_free_rcu(struct rcu_head *head)
{
	struct mla_section_dedup_entry *e =
		container_of(head, struct mla_section_dedup_entry, rcu);

	kfree(e);
}

void mla_section_dedup_put(struct mla_device *mdev,
			   struct mla_section_dedup_entry *e)
{
	unsigned long flags;

	if (!e)
		return;

	/* Defense-in-depth: catch an underflow attempt (entry already at 0,
	 * memory possibly freed and reused) BEFORE calling refcount_dec_and_test.
	 * The refcount API's own WARN + test-true on 0 would proceed to
	 * free memory that other holders may still reference. Bail loudly
	 * instead.
	 */
	if (unlikely(refcount_read(&e->refcount) == 0)) {
		WARN(1,
		     "%s: underflow attempt entry=%p — skipping teardown\n",
		     __func__, e);
		return;
	}

	/* The zero transition and the unlink must be one critical section: an
	 * entry visible at refcount 0 could be revived by a concurrent
	 * mla_section_dedup_lookup() (a saturating REFCOUNT_WARN) and then
	 * alias a primary this path is about to free below. Finders use
	 * refcount_inc_not_zero and skip such an entry, which only works if
	 * zero is never observable outside this lock.
	 *
	 * dec_and_test under a spinlock is fine: no allocation, no sleep. The
	 * teardown AFTER the unlink must stay outside the lock — it reaches
	 * dma_buf_put and can sleep.
	 */
	spin_lock_irqsave(&mdev->sec_ddp.lock, flags);
	if (!refcount_dec_and_test(&e->refcount)) {
		spin_unlock_irqrestore(&mdev->sec_ddp.lock, flags);
		return;
	}
	hlist_del_init(&e->hnode);
	spin_unlock_irqrestore(&mdev->sec_ddp.lock, flags);

	/* Free the shared primary synchronously so leak metrics reflect
	 * freed memory the moment the last holder drops. The entry itself
	 * is RCU-deferred to protect lookups that raced past the bucket
	 * lock. The primary may be pool-owned (legacy MODEL_DEFINE path
	 * via mla_pool_alloc/_cached), a carveout-heap buffer
	 * (SECTION_ALLOC without CPU_RMW), or an mla-exported CMA dma-buf
	 * (SECTION_ALLOC with CPU_RMW). Dispatch in that order; CMA
	 * primaries free through dma_buf_put which fires
	 * mla_dmabuf_release and tears down the allocation.
	 */
	if (e->buf) {
		struct mla_buffer *primary = e->buf;

		primary->dedup = NULL;
		e->buf = NULL;
		if (primary->pool_owned) {
			mla_pool_free(mdev, primary);
		} else if (primary->heap_backed) {
			mla_heap_free(mdev, primary);
		} else if (primary->dmabuf) {
			dma_buf_put(primary->dmabuf);
		} else {
			dev_warn(mdev->dev,
				 "sec_ddp: primary buf=%p has neither pool_owned nor dmabuf; leaking\n",
				 primary);
		}
	}

	call_rcu(&e->rcu, mla_section_dedup_free_rcu);
}
