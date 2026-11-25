// SPDX-License-Identifier: GPL-2.0

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/highmem.h>
#include <linux/of_reserved_mem.h>

#include <linux/cacheflush.h>

struct carveout_heap_priv {
	struct dma_heap *heap;
	struct gen_pool *pool;
};

struct carveout_heap_buffer_priv {
	struct mutex lock;
	struct list_head attachments;

	unsigned long num_pages;
	struct carveout_heap_priv *heap;
	phys_addr_t paddr;
	void *vaddr;
	unsigned int vmap_cnt;
};

struct carveout_heap_attachment {
	struct list_head head;
	struct sg_table table;

	struct device *dev;
	bool mapped;
};

static int carveout_heap_attach(struct dma_buf *buf,
				struct dma_buf_attachment *attachment)
{
	struct carveout_heap_buffer_priv *priv = buf->priv;
	struct carveout_heap_attachment *a;
	struct sg_table *sgt;
	int ret;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	INIT_LIST_HEAD(&a->head);
	a->dev = attachment->dev;
	attachment->priv = a;

	/*
	 * The carveout region lies outside the memory ranges the kernel
	 * manages, so it has no struct pages and the sg entry cannot be
	 * built from a page or virtual address here. The DMA address is
	 * filled in at map time from the buffer's physical address.
	 */
	sgt = &a->table;
	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (ret)
		goto err_cleanup_attach;

	mutex_lock(&priv->lock);
	list_add(&a->head, &priv->attachments);
	mutex_unlock(&priv->lock);

	return 0;

err_cleanup_attach:
	kfree(a);
	return ret;
}

static void carveout_heap_detach(struct dma_buf *dmabuf,
				 struct dma_buf_attachment *attachment)
{
	struct carveout_heap_buffer_priv *priv = dmabuf->priv;
	struct carveout_heap_attachment *a = attachment->priv;

	mutex_lock(&priv->lock);
	list_del(&a->head);
	mutex_unlock(&priv->lock);

	sg_free_table(&a->table);
	kfree(a);
}

static struct sg_table *
carveout_heap_map_dma_buf(struct dma_buf_attachment *attachment,
			  enum dma_data_direction direction)
{
	struct carveout_heap_attachment *a = attachment->priv;
	struct carveout_heap_buffer_priv *priv = attachment->dmabuf->priv;
	struct sg_table *table = &a->table;
	unsigned long len = priv->num_pages * PAGE_SIZE;
	dma_addr_t dma_addr;

	/*
	 * No struct pages back the carveout region, so the page-based
	 * dma_map_sgtable() path cannot be used; map the buffer by its
	 * physical address instead.
	 */
	dma_addr = dma_map_resource(a->dev, priv->paddr, len, direction, 0);
	if (dma_mapping_error(a->dev, dma_addr))
		return ERR_PTR(-ENOMEM);

	sg_dma_address(table->sgl) = dma_addr;
	sg_dma_len(table->sgl) = len;

	a->mapped = true;

	return table;
}

static void carveout_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
					struct sg_table *table,
					enum dma_data_direction direction)
{
	struct carveout_heap_attachment *a = attachment->priv;
	struct carveout_heap_buffer_priv *priv = attachment->dmabuf->priv;
	unsigned long len = priv->num_pages * PAGE_SIZE;

	a->mapped = false;
	dma_unmap_resource(a->dev, sg_dma_address(table->sgl), len,
			   direction, 0);
}

/*
 * The carveout region is outside the kernel's view of RAM and has no
 * linear-map alias, so the generic DMA cache maintenance helpers (which
 * operate on phys_to_virt() addresses) cannot be used here. Instead,
 * every buffer keeps a kernel mapping (created at allocation) and the
 * caches are maintained by VA through that mapping. The data caches are
 * PIPT, so maintenance through this alias also covers lines filled
 * through cacheable userspace mmaps of the same buffer. Mirrors the
 * arm64 arch_sync_dma_for_{cpu,device}() semantics.
 */
static int
carveout_heap_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
				       enum dma_data_direction direction)
{
	struct carveout_heap_buffer_priv *priv = dmabuf->priv;
	unsigned long start = (unsigned long)priv->vaddr;
	unsigned long len = priv->num_pages * PAGE_SIZE;

	mutex_lock(&priv->lock);

	if (direction != DMA_TO_DEVICE)
		dcache_inval_poc(start, start + len);

	mutex_unlock(&priv->lock);

	return 0;
}

static int
carveout_heap_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				     enum dma_data_direction direction)
{
	struct carveout_heap_buffer_priv *priv = dmabuf->priv;
	unsigned long start = (unsigned long)priv->vaddr;
	unsigned long len = priv->num_pages * PAGE_SIZE;

	mutex_lock(&priv->lock);

	dcache_clean_poc(start, start + len);

	mutex_unlock(&priv->lock);

	return 0;
}

static int carveout_heap_mmap(struct dma_buf *dmabuf,
			      struct vm_area_struct *vma)
{
	struct carveout_heap_buffer_priv *priv = dmabuf->priv;
	unsigned long len = priv->num_pages * PAGE_SIZE;

	return vm_iomap_memory(vma, priv->paddr, len);
}

static int carveout_heap_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct carveout_heap_buffer_priv *priv = dmabuf->priv;

	mutex_lock(&priv->lock);

	/* The buffer is mapped for its whole lifetime at allocation time. */
	iosys_map_set_vaddr(map, priv->vaddr);
	priv->vmap_cnt++;

	mutex_unlock(&priv->lock);

	return 0;
}

static void carveout_heap_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct carveout_heap_buffer_priv *priv = dmabuf->priv;

	mutex_lock(&priv->lock);

	priv->vmap_cnt--;

	mutex_unlock(&priv->lock);

	iosys_map_clear(map);
}

static void carveout_heap_dma_buf_release(struct dma_buf *buf)
{
	struct carveout_heap_buffer_priv *buffer_priv = buf->priv;
	struct carveout_heap_priv *heap_priv = buffer_priv->heap;
	unsigned long len = buffer_priv->num_pages * PAGE_SIZE;

	WARN_ON(buffer_priv->vmap_cnt);
	memunmap(buffer_priv->vaddr);
	gen_pool_free(heap_priv->pool, buffer_priv->paddr, len);
	kfree(buffer_priv);
}

static const struct dma_buf_ops carveout_heap_buf_ops = {
	.attach		= carveout_heap_attach,
	.detach		= carveout_heap_detach,
	.map_dma_buf	= carveout_heap_map_dma_buf,
	.unmap_dma_buf	= carveout_heap_unmap_dma_buf,
	.begin_cpu_access	= carveout_heap_dma_buf_begin_cpu_access,
	.end_cpu_access	= carveout_heap_dma_buf_end_cpu_access,
	.mmap		= carveout_heap_mmap,
	.vmap		= carveout_heap_vmap,
	.vunmap		= carveout_heap_vunmap,
	.release	= carveout_heap_dma_buf_release,
};

static struct dma_buf *carveout_heap_allocate(struct dma_heap *heap,
						unsigned long len,
						u32 fd_flags,
						u64 heap_flags)
{
	struct carveout_heap_priv *heap_priv = dma_heap_get_drvdata(heap);
	struct carveout_heap_buffer_priv *buffer_priv;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *buf;
	phys_addr_t paddr;
	/* len is guaranteed to be page-aligned by the framework, so we can use it as is. */
	size_t size = len;
	int ret;

	/* The buffer is described by a single sg entry, whose length is a u32. */
	if (size > UINT_MAX)
		return ERR_PTR(-E2BIG);

	buffer_priv = kzalloc(sizeof(*buffer_priv), GFP_KERNEL);
	if (!buffer_priv)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&buffer_priv->attachments);
	mutex_init(&buffer_priv->lock);

	paddr = gen_pool_alloc(heap_priv->pool, size);
	if (!paddr) {
		ret = -ENOMEM;
		goto err_free_buffer_priv;
	}

	buffer_priv->paddr = paddr;
	buffer_priv->heap = heap_priv;
	buffer_priv->num_pages = size >> PAGE_SHIFT;

	/*
	 * Map the buffer for its whole lifetime: the region has no
	 * linear-map alias, and this mapping is what vmap hands out and
	 * what the cache maintenance in begin/end_cpu_access runs on.
	 */
	buffer_priv->vaddr = memremap(paddr, size, MEMREMAP_WB);
	if (!buffer_priv->vaddr) {
		ret = -ENOMEM;
		goto err_free_buffer;
	}

	/* create the dmabuf */
	exp_info.exp_name = dma_heap_get_name(heap);
	exp_info.ops = &carveout_heap_buf_ops;
	exp_info.size = size;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer_priv;

	buf = dma_buf_export(&exp_info);
	if (IS_ERR(buf)) {
		ret = PTR_ERR(buf);
		goto err_unmap;
	}

	return buf;

err_unmap:
	memunmap(buffer_priv->vaddr);
err_free_buffer:
	gen_pool_free(heap_priv->pool, paddr, len);
err_free_buffer_priv:
	kfree(buffer_priv);

	return ERR_PTR(ret);
}

static const struct dma_heap_ops carveout_heap_ops = {
	.allocate = carveout_heap_allocate,
};

static int __init carveout_heap_setup(struct device_node *node)
{
	struct dma_heap_export_info exp_info = {};
	const struct reserved_mem *rmem;
	struct carveout_heap_priv *priv;
	const char *name = node->full_name;
	struct dma_heap *heap;
	struct gen_pool *pool;
	int ret;

	/* Reusable memory belongs to the page allocator/CMA, never a gen_pool. */
	if (of_property_read_bool(node, "reusable"))
		return -EINVAL;

	rmem = of_reserved_mem_lookup(node);
	if (!rmem)
		return -EINVAL;

	/* Prefer an explicit label over the node name (which includes the unit address). */
	of_property_read_string(node, "label", &name);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	pool = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
	if (!pool) {
		ret = -ENOMEM;
		goto err_cleanup_heap;
	}
	priv->pool = pool;

	ret = gen_pool_add(pool, rmem->base, rmem->size, NUMA_NO_NODE);
	if (ret)
		goto err_release_mem_region;

	exp_info.name = name;
	exp_info.ops = &carveout_heap_ops;
	exp_info.priv = priv;

	heap = dma_heap_add(&exp_info);
	if (IS_ERR(heap)) {
		ret = PTR_ERR(heap);
		goto err_release_mem_region;
	}
	priv->heap = heap;

	return 0;

err_release_mem_region:
	gen_pool_destroy(pool);
err_cleanup_heap:
	kfree(priv);
	return ret;
}

static int __init carveout_heap_init(void)
{
	struct device_node *rmem_node;
	struct device_node *node;
	int ret;

	rmem_node = of_find_node_by_path("/reserved-memory");
	if (!rmem_node)
		return 0;

	for_each_child_of_node(rmem_node, node) {
		if (!of_device_is_compatible(node, "carved-out"))
			continue;

		ret = carveout_heap_setup(node);
		if (ret)
			return ret;
	}

	return 0;
}

module_init(carveout_heap_init);
