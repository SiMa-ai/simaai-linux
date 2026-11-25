// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF CMA heap exporter
 *
 * Copyright (C) 2012, 2019, 2020 Linaro Ltd.
 * Author: <benjamin.gaignard@linaro.org> for ST-Ericsson.
 *
 * Also utilizing parts of Andrew Davis' SRAM heap:
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#define pr_fmt(fmt) "cma_heap: " fmt

#include <linux/cma.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-map-ops.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <uapi/linux/dma-heap.h>

#define DEFAULT_CMA_NAME "default_cma_region"
#define MLA_HEAP_NAME "simaai,mla"

struct cma_heap {
	struct dma_heap *heap;
	struct cma *cma;
	/* Successfully exported buffers, counted until final release. */
	atomic64_t allocated_bytes;
};

struct mla_heap {
	struct dma_heap *primary;
	struct cma_heap sysram;
	struct cma_heap fallback;
};

struct cma_heap_buffer {
	struct cma_heap *heap;
	struct list_head attachments;
	struct mutex lock;
	unsigned long len;
	struct page *cma_pages;
	struct page **pages;
	pgoff_t pagecount;
	int vmap_cnt;
	void *vaddr;
};

struct dma_heap_attachment {
	struct device *dev;
	struct sg_table table;
	struct list_head list;
	bool mapped;
};

static int cma_heap_attach(struct dma_buf *dmabuf,
			   struct dma_buf_attachment *attachment)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;
	int ret;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = sg_alloc_table_from_pages(&a->table, buffer->pages,
					buffer->pagecount, 0,
					buffer->pagecount << PAGE_SHIFT,
					GFP_KERNEL);
	if (ret) {
		kfree(a);
		return ret;
	}

	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->list);
	a->mapped = false;

	attachment->priv = a;

	mutex_lock(&buffer->lock);
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return 0;
}

static void cma_heap_detach(struct dma_buf *dmabuf,
			    struct dma_buf_attachment *attachment)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a = attachment->priv;

	mutex_lock(&buffer->lock);
	list_del(&a->list);
	mutex_unlock(&buffer->lock);

	sg_free_table(&a->table);
	kfree(a);
}

static struct sg_table *cma_heap_map_dma_buf(struct dma_buf_attachment *attachment,
					     enum dma_data_direction direction)
{
	struct dma_heap_attachment *a = attachment->priv;
	struct sg_table *table = &a->table;
	int ret;

	ret = dma_map_sgtable(attachment->dev, table, direction, 0);
	if (ret)
		return ERR_PTR(-ENOMEM);
	a->mapped = true;
	return table;
}

static void cma_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
				   struct sg_table *table,
				   enum dma_data_direction direction)
{
	struct dma_heap_attachment *a = attachment->priv;

	a->mapped = false;
	dma_unmap_sgtable(attachment->dev, table, direction, 0);
}

static int cma_heap_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					     enum dma_data_direction direction)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt)
		invalidate_kernel_vmap_range(buffer->vaddr, buffer->len);

	/*
	 * Maintain the CPU caches over the buffer's physical pages directly.
	 * The buffer is mapped cacheable into userspace and may be consumed by
	 * a non-coherent accelerator (e.g. EVXX) that accesses DRAM by physical
	 * address with no dma-buf attachment. These allocations are physically
	 * contiguous, so a single range starting at cma_pages covers the whole
	 * buffer. For DMA_FROM_DEVICE this invalidates, so the CPU sees the
	 * accelerator's writes instead of stale cache lines.
	 *
	 * This one pass also covers every attachment: the data caches are
	 * physically indexed and each attachment maps these same pages, so a
	 * per-attachment sync would repeat the same whole-buffer maintenance
	 * once per attachment. This assumes no attachment is bounced through
	 * swiotlb, whose copy-back only a per-attachment sync performs.
	 */
	arch_sync_dma_for_cpu(page_to_phys(buffer->cma_pages), buffer->len,
			      direction);
	mutex_unlock(&buffer->lock);

	return 0;
}

static int cma_heap_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
					   enum dma_data_direction direction)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt)
		flush_kernel_vmap_range(buffer->vaddr, buffer->len);

	/*
	 * Clean the CPU caches over the buffer's physical pages so a
	 * non-coherent accelerator reading DRAM by physical address observes
	 * the CPU's writes (counterpart to the invalidate in
	 * cma_heap_dma_buf_begin_cpu_access). As there, this one pass also
	 * covers every attachment.
	 */
	arch_sync_dma_for_device(page_to_phys(buffer->cma_pages), buffer->len,
				 direction);
	mutex_unlock(&buffer->lock);

	return 0;
}

static vm_fault_t cma_heap_vm_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct cma_heap_buffer *buffer = vma->vm_private_data;

	if (vmf->pgoff >= buffer->pagecount)
		return VM_FAULT_SIGBUS;

	return vmf_insert_pfn(vma, vmf->address, page_to_pfn(buffer->pages[vmf->pgoff]));
}

static const struct vm_operations_struct dma_heap_vm_ops = {
	.fault = cma_heap_vm_fault,
};

static int cma_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;

	if ((vma->vm_flags & (VM_SHARED | VM_MAYSHARE)) == 0)
		return -EINVAL;

	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);

	vma->vm_ops = &dma_heap_vm_ops;
	vma->vm_private_data = buffer;

	return 0;
}

static void *cma_heap_do_vmap(struct cma_heap_buffer *buffer)
{
	void *vaddr;

	vaddr = vmap(buffer->pages, buffer->pagecount, VM_MAP, PAGE_KERNEL);
	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

static int cma_heap_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;
	void *vaddr;
	int ret = 0;

	mutex_lock(&buffer->lock);
	if (buffer->vmap_cnt) {
		buffer->vmap_cnt++;
		iosys_map_set_vaddr(map, buffer->vaddr);
		goto out;
	}

	vaddr = cma_heap_do_vmap(buffer);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		goto out;
	}
	buffer->vaddr = vaddr;
	buffer->vmap_cnt++;
	iosys_map_set_vaddr(map, buffer->vaddr);
out:
	mutex_unlock(&buffer->lock);

	return ret;
}

static void cma_heap_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	if (!--buffer->vmap_cnt) {
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->lock);
	iosys_map_clear(map);
}

static void cma_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct cma_heap_buffer *buffer = dmabuf->priv;
	struct cma_heap *cma_heap = buffer->heap;

	if (buffer->vmap_cnt > 0) {
		WARN(1, "%s: buffer still mapped in the kernel\n", __func__);
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}

	/* free page list */
	kvfree(buffer->pages);
	/* release memory */
	if (cma_heap->cma)
		cma_release(cma_heap->cma, buffer->cma_pages, buffer->pagecount);
	else
		free_contig_range(page_to_pfn(buffer->cma_pages), buffer->pagecount);
	atomic64_sub(buffer->len, &cma_heap->allocated_bytes);
	kfree(buffer);
}

static const struct dma_buf_ops cma_heap_buf_ops = {
	.attach = cma_heap_attach,
	.detach = cma_heap_detach,
	.map_dma_buf = cma_heap_map_dma_buf,
	.unmap_dma_buf = cma_heap_unmap_dma_buf,
	.begin_cpu_access = cma_heap_dma_buf_begin_cpu_access,
	.end_cpu_access = cma_heap_dma_buf_end_cpu_access,
	.mmap = cma_heap_mmap,
	.vmap = cma_heap_vmap,
	.vunmap = cma_heap_vunmap,
	.release = cma_heap_dma_buf_release,
};

static struct dma_buf *cma_heap_allocate_buffer(struct dma_heap *heap,
						struct cma_heap *cma_heap,
						unsigned long len, u32 fd_flags)
{
	struct cma_heap_buffer *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	size_t size = PAGE_ALIGN(len);
	pgoff_t pagecount = size >> PAGE_SHIFT;
	unsigned long align = get_order(size);
	struct page *cma_pages;
	struct dma_buf *dmabuf;
	int ret = -ENOMEM;
	pgoff_t pg;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	buffer->len = size;

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	if (fatal_signal_pending(current)) {
		ret = -EINTR;
		goto free_buffer;
	}

	/* Heap exhaustion is expected during fallback; return the failure quietly. */
	if (cma_heap->cma) {
		cma_pages = cma_alloc(cma_heap->cma, pagecount, align, true);
	} else if (size <= SZ_512K) {
		void *addr;

		/* Small buffers can use free pages beside pinned allocations. */
		addr = alloc_pages_exact(size, GFP_KERNEL | __GFP_NOWARN |
					      __GFP_NORETRY);
		cma_pages = addr ? virt_to_page(addr) : NULL;
	} else {
		/* ACR_FLAGS_NONE rejects CMA pageblocks in this allocator. */
		cma_pages = alloc_contig_pages(pagecount,
					       GFP_HIGHUSER_MOVABLE | __GFP_NOWARN,
					       numa_node_id(), NULL);
	}
	if (!cma_pages)
		goto free_buffer;

	/* Clear the contiguous pages. */
	if (PageHighMem(cma_pages)) {
		unsigned long nr_clear_pages = pagecount;
		struct page *page = cma_pages;

		while (nr_clear_pages > 0) {
			void *vaddr = kmap_local_page(page);

			memset(vaddr, 0, PAGE_SIZE);
			kunmap_local(vaddr);
			/*
			 * Avoid wasting time zeroing memory if the process
			 * has been killed by SIGKILL.
			 */
			if (fatal_signal_pending(current)) {
				ret = -EINTR;
				goto free_cma;
			}
			page++;
			nr_clear_pages--;
		}
	} else {
		memset(page_address(cma_pages), 0, size);
	}
	if (fatal_signal_pending(current)) {
		ret = -EINTR;
		goto free_cma;
	}

	buffer->pages = kvmalloc_array(pagecount, sizeof(*buffer->pages), GFP_KERNEL);
	if (!buffer->pages) {
		ret = -ENOMEM;
		goto free_cma;
	}

	for (pg = 0; pg < pagecount; pg++)
		buffer->pages[pg] = &cma_pages[pg];

	buffer->cma_pages = cma_pages;
	buffer->heap = cma_heap;
	buffer->pagecount = pagecount;

	/* create the dmabuf */
	exp_info.exp_name = dma_heap_get_name(heap);
	exp_info.ops = &cma_heap_buf_ops;
	exp_info.size = buffer->len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_pages;
	}
	atomic64_add(buffer->len, &cma_heap->allocated_bytes);
	return dmabuf;

free_pages:
	kvfree(buffer->pages);
free_cma:
	if (cma_heap->cma)
		cma_release(cma_heap->cma, cma_pages, pagecount);
	else
		free_contig_range(page_to_pfn(cma_pages), pagecount);
free_buffer:
	kfree(buffer);
	if (fatal_signal_pending(current))
		ret = -EINTR;

	return ERR_PTR(ret);
}

static struct dma_buf *cma_heap_allocate(struct dma_heap *heap,
					 unsigned long len,
					 u32 fd_flags,
					 u64 heap_flags)
{
	return cma_heap_allocate_buffer(heap, dma_heap_get_drvdata(heap),
					len, fd_flags);
}

static u64 cma_heap_allocated_bytes(struct dma_heap *heap)
{
	struct cma_heap *cma_heap = dma_heap_get_drvdata(heap);

	return atomic64_read(&cma_heap->allocated_bytes);
}

static const struct dma_heap_ops cma_heap_ops = {
	.allocate = cma_heap_allocate,
	.allocated_bytes = cma_heap_allocated_bytes,
};

static bool mla_heap_resource_error(struct dma_buf *dmabuf)
{
	return IS_ERR(dmabuf) &&
		(PTR_ERR(dmabuf) == -ENOMEM || PTR_ERR(dmabuf) == -ENOSPC ||
		 PTR_ERR(dmabuf) == -E2BIG);
}

static struct dma_buf *mla_heap_allocate(struct dma_heap *heap,
					 unsigned long len, u32 fd_flags,
					 u64 heap_flags)
{
	struct mla_heap *mla = dma_heap_get_drvdata(heap);
	struct dma_buf *dmabuf;

	if (heap_flags & ~DMA_HEAP_VALID_HEAP_FLAGS ||
	    fd_flags & ~DMA_HEAP_VALID_FD_FLAGS)
		return ERR_PTR(-EINVAL);
	if (fatal_signal_pending(current))
		return ERR_PTR(-EINTR);

	/* Keep the original exporter and lifetime for delegated DMS buffers. */
	dmabuf = dma_heap_buffer_alloc(mla->primary, len, fd_flags, heap_flags);
	if (!mla_heap_resource_error(dmabuf))
		return dmabuf;
	if (fatal_signal_pending(current))
		return ERR_PTR(-EINTR);

	dmabuf = cma_heap_allocate_buffer(heap, &mla->sysram, len, fd_flags);
	if (!mla_heap_resource_error(dmabuf) || !mla->fallback.cma)
		return dmabuf;

	return cma_heap_allocate_buffer(heap, &mla->fallback, len, fd_flags);
}

static u64 mla_heap_allocated_bytes(struct dma_heap *heap)
{
	struct mla_heap *mla = dma_heap_get_drvdata(heap);

	/* DMS exports retain their original heap's accounting. */
	return atomic64_read(&mla->sysram.allocated_bytes) +
		atomic64_read(&mla->fallback.allocated_bytes);
}

static const struct dma_heap_ops mla_heap_ops = {
	.allocate = mla_heap_allocate,
	.allocated_bytes = mla_heap_allocated_bytes,
};

static int mla_heap_stats_show(struct seq_file *s, void *unused)
{
	struct mla_heap *mla = s->private;

	seq_printf(s, "sysram_bytes %lld\n",
		   atomic64_read(&mla->sysram.allocated_bytes));
	seq_printf(s, "cma_bytes %lld\n",
		   atomic64_read(&mla->fallback.allocated_bytes));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mla_heap_stats);

static int __init add_mla_fallback_heap(void)
{
	struct dma_heap_export_info exp_info = {
		.name = MLA_HEAP_NAME,
		.ops = &mla_heap_ops,
	};
	struct device_node *node;
	struct mla_heap *mla;
	struct dma_heap *heap;
	int ret;

	/* Existing Modalix DTs enable the policy without a separate heap node. */
	node = of_find_compatible_node(NULL, NULL, "simaai,mla-modalix");
	if (!node)
		return 0;
	if (!of_device_is_available(node)) {
		of_node_put(node);
		return 0;
	}
	of_node_put(node);

	mla = kzalloc(sizeof(*mla), GFP_KERNEL);
	if (!mla)
		return -ENOMEM;
	mla->primary = dma_heap_find("simaai,dms");
	if (!mla->primary) {
		ret = -ENODEV;
		goto free_heap;
	}

	mla->fallback.cma = dev_get_cma_area(NULL);
	exp_info.priv = mla;
	heap = dma_heap_add(&exp_info);
	if (IS_ERR(heap)) {
		ret = PTR_ERR(heap);
		goto free_heap;
	}
	debugfs_create_file("stats", 0444,
			    debugfs_create_dir("mla_heap", NULL),
			    mla, &mla_heap_stats_fops);
	return 0;

free_heap:
	kfree(mla);
	pr_err("failed to add MLA fallback heap: %d\n", ret);
	return ret;
}

static int __init __add_cma_heap(struct cma *cma, const char *name)
{
	struct dma_heap_export_info exp_info;
	struct cma_heap *cma_heap;

	cma_heap = kzalloc(sizeof(*cma_heap), GFP_KERNEL);
	if (!cma_heap)
		return -ENOMEM;
	cma_heap->cma = cma;

	exp_info.name = name;
	exp_info.ops = &cma_heap_ops;
	exp_info.priv = cma_heap;

	cma_heap->heap = dma_heap_add(&exp_info);
	if (IS_ERR(cma_heap->heap)) {
		int ret = PTR_ERR(cma_heap->heap);

		kfree(cma_heap);
		return ret;
	}

	return 0;
}

static int __init add_default_cma_heap(void)
{
	struct cma *default_cma = dev_get_cma_area(NULL);
	const char *legacy_cma_name;
	int ret;

	if (!default_cma)
		return 0;

	ret = __add_cma_heap(default_cma, DEFAULT_CMA_NAME);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_DMABUF_HEAPS_CMA_LEGACY)) {
		legacy_cma_name = cma_get_name(default_cma);
		if (!strcmp(legacy_cma_name, DEFAULT_CMA_NAME)) {
			pr_warn("legacy name and default name are the same, skipping legacy heap\n");
			return 0;
		}

		ret = __add_cma_heap(default_cma, legacy_cma_name);
		if (ret)
			pr_warn("failed to add legacy heap: %pe\n",
				ERR_PTR(ret));
	}

	return 0;
}
/*
 * A named MLA pool is a second CMA area, not a second allocator over the
 * default area. Its existing userspace name is independent of DT placement.
 * The shared-dma-pool reserved-memory initializer owns rmem->priv; never
 * instantiate the carveout gen_pool over these Linux-managed pages.
 */
static int __init add_mla_cma_heap(struct device_node *node)
{
	struct reserved_mem *rmem;
	struct cma *cma;
	const char *name;

	if (!of_device_is_compatible(node, "shared-dma-pool") ||
	    of_device_is_compatible(node, "carved-out") ||
	    !of_property_read_bool(node, "reusable") ||
	    of_property_read_bool(node, "no-map") ||
	    of_property_read_bool(node, "linux,cma-default") ||
	    of_property_read_bool(node, "linux,dma-default") ||
	    of_property_read_string(node, "label", &name) || !*name)
		return -EINVAL;

	rmem = of_reserved_mem_lookup(node);
	if (!rmem || !rmem->ops || !rmem->priv)
		return -ENODEV;

	/* These properties select the CMA reserved-memory initializer. */
	cma = rmem->priv;
	if (cma == dev_get_cma_area(NULL) ||
	    cma_get_base(cma) != rmem->base ||
	    cma_get_size(cma) != rmem->size)
		return -EINVAL;

	return __add_cma_heap(cma, name);
}

static int __init add_cma_heaps(void)
{
	struct device_node *node;
	int ret;

	ret = add_default_cma_heap();
	if (ret)
		return ret;

	for_each_compatible_node(node, NULL, "simaai,mla-cma") {
		if (!of_device_is_available(node))
			continue;

		ret = add_mla_cma_heap(node);
		if (ret) {
			pr_err("failed to add MLA CMA heap %pOF: %d\n", node, ret);
			of_node_put(node);
			return ret;
		}
	}

	/* The Makefile registers carveout heaps before this CMA initcall. */
	return add_mla_fallback_heap();
}
module_init(add_cma_heaps);
MODULE_DESCRIPTION("DMA-BUF CMA Heap");
