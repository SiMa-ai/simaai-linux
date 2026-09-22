// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2021 Sima ai
 *
 * Author: Roman Bulhakov <roman.bulhakov@sima.ai>
 */

#include <linux/cdev.h>
#include <linux/dma-buf.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fcntl.h>
#include <linux/fdtable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/radix-tree.h>
#include <linux/sched.h>
#include <uapi/linux/simaai/simaai_memory_ioctl.h>
#include <linux/of_reserved_mem.h>
#include <linux/overflow.h>
#if IS_ENABLED(CONFIG_SIMAAI_MEMORY_KUNIT_TEST)
#include <linux/completion.h>
#include <linux/kthread.h>
#include <kunit/test.h>
#endif

#include <linux/simaai-stu.h>
#include <linux/simaai-memcpy.h>

#define SIMAAI_MEMOERY_DEV_NAME "simaai-mem"

struct simaai_memdev;

struct simaai_memory_buffer {
	struct device		*dev;
	struct simaai_memdev	*memdev;
	u32			flags;
	void			*cpu_addr;
	dma_addr_t		phys_addr;
	dma_addr_t		bus_addr;
	u32			target;
	/* Requested memory size */
	size_t			size;
	/* Allocated memory size aligned to page boundary */
	size_t			aligned_size;
	u32			owner;
	struct kref		refcount;
	struct simaai_memory_buffer *parent;
	/* indicates offset from parent if this is buffer for segment */
	u64 			offset;
};

struct simaai_memory_filp_buffer {
	dma_addr_t		phys_addr;
	bool			exportable;
	struct list_head	node;
};

struct simaai_memory_filp_buffers {
	struct mutex		buffer_lock;
	struct list_head	buffer_head;
};

struct simaai_memory_dmabuf_attachment {
	struct sg_table	sgt;
	/* Serializes mapping direction and nesting state. */
	struct mutex	lock;
	unsigned int	map_count;
	enum dma_data_direction direction;
};

struct simaai_memdev {
	struct kref		refcount;
	u32			target;
	struct device		*dev;
	struct list_head	node;

	/* simaai stu driver handle */
	struct simaai_stu *stu;
	/* Serializes this device's export gate. */
	struct mutex		export_lock;
	bool			accepting_exports;
	bool			reserved_initialized;

};

static void simaai_memory_release_memdev(struct kref *ref)
{
	struct simaai_memdev *memdev = container_of(ref, struct simaai_memdev,
						    refcount);

	/* User-owned DMA-BUF FDs may outlive unbind; release the pool last. */
	if (memdev->reserved_initialized)
		of_reserved_mem_device_release(memdev->dev);
	put_device(memdev->dev);
	kfree(memdev);
}

struct simaai_memory_device {
	struct list_head	dev_head;

	struct mutex		buffer_lock;
	struct radix_tree_root	buffer_root;

	/* Character device */
	struct cdev		cdev;
	dev_t			dev_no;
	struct class		dev_class;
	bool			exist;
};

static struct simaai_memory_device simaaimem = { 0 };
static DEFINE_MUTEX(dev_lock);
static void simaai_memory_export_state_init(struct simaai_memdev *memdev)
{
	mutex_init(&memdev->export_lock);
	memdev->accepting_exports = true;
}

/*
 * Admission control only.  The gate decides whether a new export may start and
 * holds no reference, so it has no paired release: an exported DMA-BUF pins its
 * buffer, and the buffer holds the memdev reference that lets both outlive
 * unbind.  Do not reintroduce a counter here without also giving it a kref.
 */
static bool simaai_memory_exports_allowed(struct simaai_memdev *memdev)
{
	bool accepted;

	mutex_lock(&memdev->export_lock);
	accepted = memdev->accepting_exports;
	mutex_unlock(&memdev->export_lock);
	return accepted;
}

static void simaai_memory_export_stop(struct simaai_memdev *memdev)
{
	mutex_lock(&memdev->export_lock);
	memdev->accepting_exports = false;
	mutex_unlock(&memdev->export_lock);
}

static bool simaai_memory_cursor_exportable(const struct simaai_memory_filp_buffer *cursor)
{
	return cursor->exportable;
}

static void simaai_memory_registry_add(struct list_head *head,
				       struct simaai_memdev *memdev)
{
	list_add(&memdev->node, head);
}

static void simaai_memory_registry_remove(struct simaai_memdev *memdev)
{
	list_del_init(&memdev->node);
}

/* Caller serializes attachment state with a->lock. */
static int simaai_memory_map_state_get(struct simaai_memory_dmabuf_attachment *a,
				       enum dma_data_direction direction)
{
	if (a->map_count && a->direction != direction)
		return -EBUSY;
	if (a->map_count == UINT_MAX)
		return -EOVERFLOW;
	return a->map_count++;
}

/* Returns true when the underlying DMA mapping must be released. */
static bool simaai_memory_map_state_put(struct simaai_memory_dmabuf_attachment *a)
{
	if (!a->map_count)
		return false;
	return --a->map_count == 0;
}

static void simaai_release_buffer(struct kref *ref)
{
	struct simaai_memory_buffer *buffer =
		container_of(ref, struct simaai_memory_buffer, refcount);

	radix_tree_delete(&simaaimem.buffer_root, buffer->phys_addr);

	if (buffer->parent == NULL ) {
		dma_free_coherent(buffer->dev, buffer->aligned_size,
			  buffer->cpu_addr, buffer->phys_addr);
	}

	struct simaai_memdev *memdev = buffer->memdev;

	kfree(buffer);
	kref_put(&memdev->refcount, simaai_memory_release_memdev);
}

static struct simaai_memory_buffer *
simaai_allocate_segment_buffer(struct simaai_memdev *memdev,
			       u32 target, size_t size,
			       unsigned int flags, u64 phys_addr)
{
	struct simaai_memory_buffer *buffer = NULL;
	struct device *dev = memdev->dev;
	int res;

	if (!size)
		goto err_alloc;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL)
		goto err_alloc;

	buffer->dev = memdev->dev;
	buffer->memdev = memdev;
	buffer->size = size;
	buffer->aligned_size = size;
	buffer->flags = flags;
	buffer->target = target;
	buffer->owner = (u32) task_pid_nr(current);
	buffer->phys_addr = phys_addr;
	buffer->bus_addr = phys_addr;
	buffer->parent = NULL;
	buffer->offset = 0;
	kref_init(&buffer->refcount);

	mutex_lock(&simaaimem.buffer_lock);
	res = radix_tree_insert(&simaaimem.buffer_root, buffer->phys_addr, buffer);
	if (res != 0) {
		dev_err(dev, "radix_tree_insert failed with error %d\n", res);
		mutex_unlock(&simaaimem.buffer_lock);
		goto err_insert;		
	}
	mutex_unlock(&simaaimem.buffer_lock);
	kref_get(&memdev->refcount);

	return buffer;

err_insert:
	kfree(buffer);
err_alloc:
	return NULL;
}

static struct simaai_memory_buffer *
simaai_allocate_buffer(struct simaai_memdev *memdev, u32 target, size_t size, unsigned int flags)
{
	struct device *dev = memdev->dev;
	struct simaai_memory_buffer *buffer = NULL;
	int res;

	if (!size)
		goto err_alloc;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL)
		goto err_alloc;

	buffer->dev = dev;
	buffer->memdev = memdev;
	buffer->size = size;
	buffer->aligned_size = PAGE_ALIGN(size);
	buffer->flags = flags;
	buffer->target = target;
	buffer->owner = (u32) task_pid_nr(current);
	buffer->parent = NULL;
	buffer->offset = 0;

	buffer->cpu_addr = dma_alloc_coherent(dev, buffer->aligned_size, &buffer->phys_addr, GFP_USER);
	if (!buffer->cpu_addr) {
		dev_err(dev, "dma_alloc_coherent alloc of %zu bytes failed\n", buffer->aligned_size);
		goto err_dma;
	}

	buffer->bus_addr = buffer->phys_addr;
	kref_init(&buffer->refcount);

	mutex_lock(&simaaimem.buffer_lock);
	res = radix_tree_insert(&simaaimem.buffer_root, buffer->phys_addr, buffer);
	if (res != 0) {
		dev_err(dev, "radix_tree_insert failed with error %d\n", res);
		mutex_unlock(&simaaimem.buffer_lock);
		goto err_insert;		
	}
	mutex_unlock(&simaaimem.buffer_lock);
	kref_get(&memdev->refcount);

	return buffer;

err_insert:
	dma_free_coherent(buffer->dev, buffer->aligned_size, buffer->cpu_addr, buffer->phys_addr);
err_dma:
	kfree(buffer);
err_alloc:
	return NULL;
}

static void simaai_free_buffer(dma_addr_t phys_addr)
{
	struct simaai_memory_buffer *buffer;

	mutex_lock(&simaaimem.buffer_lock);
	buffer = radix_tree_lookup(&simaaimem.buffer_root, phys_addr);
	if(buffer != NULL) {
		if(buffer->parent != NULL)
			kref_put(&buffer->parent->refcount, simaai_release_buffer);

		kref_put(&buffer->refcount, simaai_release_buffer);
	}
		
	mutex_unlock(&simaaimem.buffer_lock);
}

/*
 * Look up a buffer only when it belongs to this open file and take the
 * references that keep both a segment and its backing parent alive.  The lock
 * order matches simaai_memory_dev_release(): per-file list, then global tree.
 */
static struct simaai_memory_buffer *
simaai_memory_get_owned_buffer(struct file *filp, dma_addr_t phys_addr)
{
	struct simaai_memory_filp_buffers *filp_buffers = filp->private_data;
	struct simaai_memory_filp_buffer *cursor;
	struct simaai_memory_buffer *candidate;
	struct simaai_memory_buffer *buffer = NULL;

	mutex_lock(&filp_buffers->buffer_lock);
	list_for_each_entry(cursor, &filp_buffers->buffer_head, node) {
		if (cursor->phys_addr != phys_addr)
			continue;

		mutex_lock(&simaaimem.buffer_lock);
		candidate = radix_tree_lookup(&simaaimem.buffer_root, phys_addr);
		if (candidate && simaai_memory_cursor_exportable(cursor) &&
		    simaai_memory_exports_allowed(candidate->memdev)) {
			buffer = candidate;
			kref_get(&buffer->refcount);
			if (buffer->parent)
				kref_get(&buffer->parent->refcount);
		}
		mutex_unlock(&simaaimem.buffer_lock);
		break;
	}
	mutex_unlock(&filp_buffers->buffer_lock);

	return buffer;
}

/*
 * Return the backing allocation only when this file owns one of its segments.
 * Segment mappings start at the parent's physical address, so a child cursor
 * also authorizes that parent mapping.  Holding the tree lock while taking the
 * kref closes FREE/mmap races without extending the per-file lock order.
 */
static struct simaai_memory_buffer *
simaai_memory_get_owned_mapping(struct file *filp, dma_addr_t phys_addr)
{
	struct simaai_memory_filp_buffers *filp_buffers = filp->private_data;
	struct simaai_memory_filp_buffer *cursor;
	struct simaai_memory_buffer *candidate, *root;
	struct simaai_memory_buffer *buffer = NULL;

	mutex_lock(&filp_buffers->buffer_lock);
	list_for_each_entry(cursor, &filp_buffers->buffer_head, node) {
		mutex_lock(&simaaimem.buffer_lock);
		candidate = radix_tree_lookup(&simaaimem.buffer_root,
					      cursor->phys_addr);
		root = candidate && candidate->parent ? candidate->parent : candidate;
		if (root && root->phys_addr == phys_addr) {
			kref_get(&root->refcount);
			buffer = root;
		}
		mutex_unlock(&simaaimem.buffer_lock);
		if (buffer)
			break;
	}
	mutex_unlock(&filp_buffers->buffer_lock);

	return buffer;
}

static int simaai_memory_validate_mapping(const struct simaai_memory_buffer *buffer,
					  dma_addr_t phys_addr, unsigned long size)
{
	if (!size || phys_addr != buffer->phys_addr ||
	    size > buffer->aligned_size)
		return -EINVAL;

	return 0;
}

static int simaai_memory_segment_total(const struct simaai_alloc_args *args,
				       u32 *total)
{
	u32 sum = 0;
	unsigned int i;

	if (!args->num_of_segments || args->num_of_segments > MAX_SEGMENTS)
		return -EINVAL;

	for (i = 0; i < args->num_of_segments; i++) {
		if (!args->size[i] || check_add_overflow(sum, args->size[i], &sum))
			return -EINVAL;
	}

	*total = sum;
	return 0;
}

static void simaai_memory_put_buffer(struct simaai_memory_buffer *buffer)
{
	mutex_lock(&simaaimem.buffer_lock);
	if (buffer->parent)
		kref_put(&buffer->parent->refcount, simaai_release_buffer);
	kref_put(&buffer->refcount, simaai_release_buffer);
	mutex_unlock(&simaaimem.buffer_lock);
}

static int simaai_memory_dmabuf_attach(struct dma_buf *dmabuf,
				       struct dma_buf_attachment *attachment)
{
	struct simaai_memory_buffer *buffer = dmabuf->priv;
	struct simaai_memory_dmabuf_attachment *a;
	phys_addr_t phys = buffer->phys_addr;
	phys_addr_t last;
	unsigned long pfn = PHYS_PFN(phys);
	int ret;

	if (check_add_overflow(phys, buffer->size - 1, &last) ||
	    !pfn_valid(pfn) || !pfn_valid(PHYS_PFN(last)))
		return -EINVAL;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = sg_alloc_table(&a->sgt, 1, GFP_KERNEL);
	if (ret) {
		kfree(a);
		return ret;
	}

	/*
	 * B4460 exposes the dma_alloc_coherent() handle as the CPU physical
	 * address (the same value used by this driver's mmap path).  Packed camera
	 * planes are contiguous but may start within a page, so retain that offset
	 * when presenting the exact segment to a DMA-BUF importer.
	 */
	sg_set_page(a->sgt.sgl, pfn_to_page(pfn), buffer->size,
		    offset_in_page(phys));
	a->direction = DMA_NONE;
	mutex_init(&a->lock);
	attachment->priv = a;

	return 0;
}

static void simaai_memory_dmabuf_detach(struct dma_buf *dmabuf,
					struct dma_buf_attachment *attachment)
{
	struct simaai_memory_dmabuf_attachment *a = attachment->priv;

	if (!a)
		return;

	mutex_lock(&a->lock);
	if (a->map_count)
		dma_unmap_sgtable(attachment->dev, &a->sgt, a->direction, 0);
	mutex_unlock(&a->lock);
	sg_free_table(&a->sgt);
	kfree(a);
	attachment->priv = NULL;
}

static struct sg_table *
simaai_memory_dmabuf_map(struct dma_buf_attachment *attachment,
			 enum dma_data_direction direction)
{
	struct simaai_memory_dmabuf_attachment *a = attachment->priv;
	int ret;

	if (direction == DMA_NONE)
		return ERR_PTR(-EINVAL);

	mutex_lock(&a->lock);
	ret = simaai_memory_map_state_get(a, direction);
	if (ret < 0)
		goto err_state;
	if (ret) {
		mutex_unlock(&a->lock);
		return &a->sgt;
	}

	ret = dma_map_sgtable(attachment->dev, &a->sgt, direction, 0);
	if (ret) {
		/* The first-map reservation must not look cached after failure. */
		simaai_memory_map_state_put(a);
		mutex_unlock(&a->lock);
		return ERR_PTR(ret);
	}

	a->map_count = 1;
	a->direction = direction;
	mutex_unlock(&a->lock);
	return &a->sgt;

err_state:
	mutex_unlock(&a->lock);
	return ERR_PTR(ret);
}

/*
 * The backing store is allocated with dma_alloc_coherent().  Importers still
 * need these hooks so DMA_BUF_IOCTL_SYNC is a valid operation, but no explicit
 * cache maintenance is required for the kernel's coherent mapping.  Cached
 * userspace aliases owned by libsimaaimem remain governed by that library's
 * device-written/CPU-read ownership protocol.
 */
static int
simaai_memory_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
				      enum dma_data_direction direction)
{
	return 0;
}

static int
simaai_memory_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
				    enum dma_data_direction direction)
{
	return 0;
}

static void simaai_memory_dmabuf_unmap(struct dma_buf_attachment *attachment,
				       struct sg_table *sgt,
				       enum dma_data_direction direction)
{
	struct simaai_memory_dmabuf_attachment *a = attachment->priv;

	mutex_lock(&a->lock);
	if (!simaai_memory_map_state_put(a))
		goto unlock;

	/* Pair with the direction accepted by map, not an importer's stale value. */
	dma_unmap_sgtable(attachment->dev, &a->sgt, a->direction, 0);
	a->direction = DMA_NONE;
unlock:
	mutex_unlock(&a->lock);
}

static void simaai_memory_dmabuf_release(struct dma_buf *dmabuf)
{
	simaai_memory_put_buffer(dmabuf->priv);
}

static const struct dma_buf_ops simaai_memory_dmabuf_ops = {
	.attach = simaai_memory_dmabuf_attach,
	.detach = simaai_memory_dmabuf_detach,
	.map_dma_buf = simaai_memory_dmabuf_map,
	.unmap_dma_buf = simaai_memory_dmabuf_unmap,
	.begin_cpu_access = simaai_memory_dmabuf_begin_cpu_access,
	.end_cpu_access = simaai_memory_dmabuf_end_cpu_access,
	.release = simaai_memory_dmabuf_release,
};

static long simaai_memory_export_dmabuf(struct file *filp, void __user *argp)
{
	struct simaai_export_dmabuf_args args;
	struct simaai_memory_buffer *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;
	dma_addr_t segment_phys, segment_bus;
	int fd;

	if (copy_from_user(&args, argp, sizeof(args)))
		return -EFAULT;

	if (args.flags & ~O_CLOEXEC)
		return -EINVAL;

	buffer = simaai_memory_get_owned_buffer(filp, args.phys_addr);
	if (!buffer)
		return -ENOENT;

	if (!buffer->size || args.size != buffer->size ||
	    args.bus_addr != buffer->bus_addr) {
		simaai_memory_put_buffer(buffer);
		return -EINVAL;
	}

	if (buffer->parent) {
		if (buffer->offset > buffer->parent->size ||
		    buffer->size > buffer->parent->size - buffer->offset ||
		    check_add_overflow(buffer->parent->phys_addr, buffer->offset,
				       &segment_phys) ||
		    check_add_overflow(buffer->parent->bus_addr, buffer->offset,
				       &segment_bus) ||
		    buffer->phys_addr != segment_phys ||
		    buffer->bus_addr != segment_bus) {
			simaai_memory_put_buffer(buffer);
			return -EINVAL;
		}
	}

	exp_info.exp_name = SIMAAI_MEMOERY_DEV_NAME;
	exp_info.owner = THIS_MODULE;
	exp_info.ops = &simaai_memory_dmabuf_ops;
	exp_info.size = buffer->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buffer;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		simaai_memory_put_buffer(buffer);
		return PTR_ERR(dmabuf);
	}

	fd = dma_buf_fd(dmabuf, args.flags);
	if (fd < 0) {
		dma_buf_put(dmabuf);
		return fd;
	}

	args.fd = fd;
	if (copy_to_user(argp, &args, sizeof(args))) {
		/* dma_buf_fd() installs the descriptor before returning it. */
		close_fd(fd);
		return -EFAULT;
	}

	return 0;
}

static int simaai_memory_dev_open(struct inode *inode, struct file *filp)
{
	struct simaai_memory_filp_buffers *filp_buffers;

	filp_buffers = kzalloc(sizeof(*filp_buffers), GFP_KERNEL);
	if (!filp_buffers)
		return -ENOMEM;

	INIT_LIST_HEAD(&filp_buffers->buffer_head);
	mutex_init(&filp_buffers->buffer_lock);

	filp->private_data = filp_buffers;

	return 0;
}

static int simaai_memory_dev_release(struct inode *inode, struct file *filp)
{
	struct simaai_memory_filp_buffers *filp_buffers =
			(struct simaai_memory_filp_buffers *) filp->private_data;
	struct simaai_memory_filp_buffer *cursor, *temp;

	mutex_lock(&filp_buffers->buffer_lock);
	list_for_each_entry_safe(cursor, temp, &filp_buffers->buffer_head, node) {
		simaai_free_buffer(cursor->phys_addr);
		list_del(&cursor->node);
		kfree(cursor);
	}
	mutex_unlock(&filp_buffers->buffer_lock);

	kfree(filp_buffers);

	return 0;
}

static long simaai_memory_insert_cursor(struct device *dev, struct file *filp,
					dma_addr_t phys_addr, bool exportable)
{
	struct simaai_memory_filp_buffers *filp_buffers =
			(struct simaai_memory_filp_buffers *) filp->private_data;
	struct simaai_memory_filp_buffer *filp_cursor;

	filp_cursor = kzalloc(sizeof(*filp_cursor), GFP_KERNEL);
	if (filp_cursor == NULL) {
		dev_err(dev, "Could not allocate cursor\n");
		return -ENOMEM;
	}

	filp_cursor->phys_addr = phys_addr;
	filp_cursor->exportable = exportable;
	INIT_LIST_HEAD(&filp_cursor->node);
	mutex_lock(&filp_buffers->buffer_lock);
	list_add_tail(&filp_cursor->node, &filp_buffers->buffer_head);
	mutex_unlock(&filp_buffers->buffer_lock);

	return 0;
}

static long simaai_memory_remove_cursor(struct file *filp, dma_addr_t phys_addr)
{
	struct simaai_memory_filp_buffers *filp_buffers =
			(struct simaai_memory_filp_buffers *) filp->private_data;
	struct simaai_memory_filp_buffer *cursor, *temp;

	mutex_lock(&filp_buffers->buffer_lock);
	list_for_each_entry_safe(cursor, temp, &filp_buffers->buffer_head, node) {
		if(cursor->phys_addr == phys_addr) {
			list_del(&cursor->node);
			kfree(cursor);
			break;
		}
	}
	mutex_unlock(&filp_buffers->buffer_lock);

	return 0;
}

static void free_segment_buffers(struct file *filp, struct simaai_alloc_args *aargs, unsigned int last_index)
{
	unsigned int iter = 0;

	for(iter = 0; iter <= last_index; iter++) {
		simaai_memory_remove_cursor(filp, aargs->phys_addr[iter]);
		simaai_free_buffer(aargs->phys_addr[iter]);
	}
}

static long simaai_memory_dev_ioctl(struct file *filp, unsigned int cmd,
				    unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct simaai_memory_buffer *buffer, *child;
	struct device *dev = NULL;
	struct simaai_memdev *cur;
	struct simaai_alloc_args aargs;
	struct simaai_free_args fargs;
	struct simaai_memory_info info;
	struct simaai_memcpy_args cp_args;
	long ret;
	u32 buffer_size = 0;
	unsigned int iter = 0;
	switch (cmd) {
	case SIMAAI_IOC_MEM_ALLOC_COHERENT:
		if (copy_from_user(&aargs, argp, sizeof(aargs)))
			return -EFAULT;
		ret = simaai_memory_segment_total(&aargs, &buffer_size);
		if (ret)
			return ret;

		mutex_lock(&dev_lock);
		list_for_each_entry(cur, &simaaimem.dev_head, node) {
			if (cur->target == aargs.target) {
				dev = cur->dev;
				break;
			}
		}

		if (dev)
			kref_get(&cur->refcount);
		mutex_unlock(&dev_lock);
		if (dev == NULL)
			return -EINVAL;

		buffer = simaai_allocate_buffer(cur, aargs.target, buffer_size, aargs.flags);
		kref_put(&cur->refcount, simaai_memory_release_memdev);
		if (buffer == NULL) {
			dev_err(dev, "Could not allocate buffer\n");
			return -ENOMEM;
		}

		if (cur->stu) {
			int res = simaai_stu_get_bus_address(cur->stu, buffer->phys_addr,
						&buffer->bus_addr);
			if (res != 0) {
				dev_err(dev, "Error getting Bus address\n");
				simaai_free_buffer(buffer->phys_addr);
				return -EFAULT;
			}
		}

		aargs.aligned_size = buffer->aligned_size;
		aargs.phys_addr[0] = buffer->phys_addr;
		aargs.bus_addr[0] = buffer->bus_addr;
		aargs.offset[0] = buffer->offset;

		ret = simaai_memory_insert_cursor(dev, filp, buffer->phys_addr, true);
		if(ret != 0)
			return ret;

		for (iter = 1; iter < aargs.num_of_segments; iter++) {

			child = simaai_allocate_segment_buffer(cur, aargs.target,
							       aargs.size[iter], aargs.flags,
							       aargs.phys_addr[iter - 1] +
							       aargs.size[iter - 1]);
			if (child == NULL) {
				dev_err(dev, "Failed to allocate segment buffer\n");
				free_segment_buffers(filp, &aargs, iter);
				return -ENOMEM;
			}

			if (cur->stu) {
			  int res = simaai_stu_get_bus_address(cur->stu, child->phys_addr,
							       &child->bus_addr);
			  if (res != 0) {
			    dev_err(dev, "Error getting Child Bus address\n");
			    simaai_free_buffer(child->phys_addr);
			    return -EFAULT;
			  }
			}
			child->parent = buffer;
			kref_get(&child->parent->refcount);
			aargs.aligned_size = child->aligned_size;
			aargs.phys_addr[iter] = child->phys_addr;
			aargs.bus_addr[iter] = child->bus_addr;
			aargs.offset[iter] = aargs.offset[iter - 1] + aargs.size[iter - 1];
			child->offset = aargs.offset[iter];

			ret = simaai_memory_insert_cursor(dev, filp, child->phys_addr, true);
			if(ret != 0) {
				dev_err(dev, "Failed insert cursor for segment buffer\n");
				free_segment_buffers(filp, &aargs, iter);
				return ret;
			}
		}
		aargs.size[0] = buffer->size;

		if (copy_to_user(argp, &aargs, sizeof(aargs)))
			return -EFAULT;		
		break;
	case SIMAAI_IOC_MEM_FREE:
		if (copy_from_user(&fargs, argp, sizeof(fargs)))
			return -EFAULT;
		if (!fargs.num_of_segments || fargs.num_of_segments > MAX_SEGMENTS)
			return -EINVAL;

		for(iter = 0; iter < fargs.num_of_segments; iter++) {
			ret = simaai_memory_remove_cursor(filp, fargs.phys_addr[iter]);
			simaai_free_buffer(fargs.phys_addr[iter]);
		}

		break;
	case SIMAAI_IOC_MEM_INFO:
		if (copy_from_user(&info, argp, sizeof(info)))
			return -EFAULT;
		mutex_lock(&simaaimem.buffer_lock);
		buffer = radix_tree_lookup(&simaaimem.buffer_root, info.phys_addr);
		mutex_unlock(&simaaimem.buffer_lock);
		if (buffer == NULL)
			return -EINVAL;
		
		kref_get(&buffer->refcount);
		if (buffer->parent)
			kref_get(&buffer->parent->refcount);
		info.size = buffer->size;
		info.aligned_size = buffer->aligned_size;
		info.flags = buffer->flags;
		info.phys_addr = buffer->phys_addr;
		info.bus_addr = buffer->bus_addr;
		info.target = buffer->target;
		info.offset = buffer->offset;
		ret = simaai_memory_insert_cursor(dev, filp, buffer->phys_addr, false);
		if(ret != 0)
			return ret;

		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		break;

	case SIMAAI_IOC_MEMCPY:
		if (copy_from_user(&cp_args, argp, sizeof(cp_args)))
			return -EFAULT;

		ret = simaai_sdma_memcpy(&cp_args);
		if(ret != 0)
			return ret;

		break;

	case SIMAAI_IOC_MEM_EXPORT_DMABUF:
		return simaai_memory_export_dmabuf(filp, argp);

	default:
		pr_info("simaai-mem: Bad ioctl number\n");
		return -EINVAL;
	}

	return 0;
}

#define pgprot_dmacoherent_cached(prot) \
	__pgprot_modify(prot, PTE_ATTRINDX_MASK, \
			PTE_ATTRINDX(MT_NORMAL) | PTE_PXN | PTE_UXN)

static void simaai_memory_vma_open(struct vm_area_struct *vma)
{
	struct simaai_memory_buffer *buffer = vma->vm_private_data;

	/* A forked/split VMA independently owns the backing allocation. */
	kref_get(&buffer->refcount);
}

static void simaai_memory_vma_close(struct vm_area_struct *vma)
{
	simaai_memory_put_buffer(vma->vm_private_data);
}

static const struct vm_operations_struct simaai_memory_vm_ops = {
	.open = simaai_memory_vma_open,
	.close = simaai_memory_vma_close,
};

static int simaai_memory_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct simaai_memory_buffer *buffer;
	u64 paddr;
	unsigned long vsize;

	paddr = vma->vm_pgoff << PAGE_SHIFT;
	vsize = vma->vm_end - vma->vm_start;
	buffer = simaai_memory_get_owned_mapping(filp, paddr);
	if (!buffer) {
		pr_err("simaai-mem: Can't mmap physical address: 0x%llx\n", paddr);
		return -ENOENT;
	}

	if (simaai_memory_validate_mapping(buffer, paddr, vsize)) {
		simaai_memory_put_buffer(buffer);
		return -EINVAL;
	}

	if (!(buffer->flags & SIMAAI_BUFFER_FLAG_CACHED))
		vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_dmacoherent_cached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT, vsize,
			    vma->vm_page_prot)) {
		dev_info(buffer->dev, "remap_pfn_range failed\n");
		simaai_memory_put_buffer(buffer);
		return -EAGAIN;
	}

	/* The VMA owns this reference until its final close, including forks. */
	vma->vm_private_data = buffer;
	vma->vm_ops = &simaai_memory_vm_ops;
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);

	return 0;
}

ssize_t simaai_memory_dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	char header [] = "| Physical address |      Parent      | #Ref |       Size       | Target |   Owner   |\n";
	char line [sizeof(header)];
	int line_length = sizeof(header);
	int l, total_count = 0;
	ssize_t total = 0;
	unsigned long long total_size = 0;
	struct simaai_memory_buffer *buffer;
	struct radix_tree_iter iter;
	unsigned long index = 0;
	void __rcu **slot;
	loff_t offset = 0;

	mutex_lock(&simaaimem.buffer_lock);
	if (*f_pos < line_length) {
		radix_tree_for_each_slot(slot, &simaaimem.buffer_root, &iter, index) {
			buffer = *((struct simaai_memory_buffer **)slot);
			if(!buffer->parent) {
				total_size += buffer->size;
				total_count++;
			}
		}
		l = line_length - *f_pos;
		if (l > count)
			l = count;
		sprintf(line, "| Total buffers allocated: %*d | Total allocated size: 0x%010llx           |\n",
			10, total_count, total_size);
		if (copy_to_user(&buf[total], &line[*f_pos], l)) {
			mutex_unlock(&simaaimem.buffer_lock);
			return -EFAULT;
		}
		*f_pos += l;
		total += l;
		count -= l;
		offset += line_length;
	}

	if (*f_pos < line_length * 2) {
		l = 2 * line_length - *f_pos;
		if (l > count)
			l = count;
		if (copy_to_user(&buf[total], &header[*f_pos - line_length], l)) {
			mutex_unlock(&simaaimem.buffer_lock);
			return -EFAULT;
		}
		*f_pos += l;
		total += l;
		count -= l;
		offset += line_length;
	}

	radix_tree_for_each_slot(slot, &simaaimem.buffer_root, &iter, index) {
		l = line_length - (*f_pos % line_length);
		l = l < count ? l : count;
		offset += l;
		if(*f_pos > offset)
			continue;
		buffer = *((struct simaai_memory_buffer **)slot);
		sprintf(line, "|   0x%010llx   |   0x%010llx   |  %*d  |   0x%010lx   |   %*d   | %*d |\n",
			buffer->phys_addr, (buffer->parent) ? (buffer->parent->phys_addr) : 0, 2,
			kref_read(&buffer->refcount), buffer->size, 2, buffer->target, 9, buffer->owner);
		if (copy_to_user(&buf[total], &line[*f_pos % line_length], l)) {
			mutex_unlock(&simaaimem.buffer_lock);
			return -EFAULT;
		}
		*f_pos += l;
		total += l;
		count -= l;
		if(count == 0)
			break;
	}
	mutex_unlock(&simaaimem.buffer_lock);

	return total;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open	= simaai_memory_dev_open,
	.release = simaai_memory_dev_release,
	.unlocked_ioctl = simaai_memory_dev_ioctl,
	.mmap = simaai_memory_dev_mmap,
	.read = simaai_memory_dev_read,
	.llseek	= noop_llseek,
};

static int simaai_create_char_dev(struct device *memdev)
{
	int ret;
	struct device *dev;

	ret = alloc_chrdev_region(&simaaimem.dev_no, 0, 1, SIMAAI_MEMOERY_DEV_NAME);
	if (ret) {
		dev_err(memdev, "Failed: alloc_chrdev_region\n");
		return ret;
	}

	cdev_init(&simaaimem.cdev, &fops);
	simaaimem.cdev.owner = THIS_MODULE;
	simaaimem.cdev.ops = &fops;

	ret = cdev_add(&simaaimem.cdev, simaaimem.dev_no, 1);
	if (ret) {
		dev_err(memdev, "Failed: cdev_add\n");
		goto err_cdev;
	}

	simaaimem.dev_class.name = SIMAAI_MEMOERY_DEV_NAME;

	ret = class_register(&simaaimem.dev_class);
	if (ret) {
		dev_err(memdev, "Failed: class_create\n");
		goto err_class;
	}

	dev = device_create(&simaaimem.dev_class,
			    NULL,
			    simaaimem.dev_no,
			    NULL,
			    SIMAAI_MEMOERY_DEV_NAME);
	if (IS_ERR(dev)) {
		dev_err(memdev, "Could not create files\n");
		ret = PTR_ERR(dev);
		goto err_device;
	}

	return 0;

err_device:
	class_unregister(&simaaimem.dev_class);
err_class:
	cdev_del(&simaaimem.cdev);
err_cdev:
	unregister_chrdev_region(simaaimem.dev_no, 1);
	return ret;
}

static int simaai_memory_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simaai_memdev *memdev;
	int ret, target = SIMAAI_TARGET_ALLOCATOR_DRAM;

	memdev = kzalloc(sizeof(*memdev), GFP_KERNEL);
	if (!memdev)
		return -ENOMEM;

	memdev->dev = dev;
	kref_init(&memdev->refcount);
	get_device(dev);
	simaai_memory_export_state_init(memdev);

	ret = of_property_read_u32(dev->of_node, "simaai,target", &target);
	if (ret) {
		dev_warn(dev, "Could not obtain simaai,target property\n");
	}


	ret = of_reserved_mem_device_init(dev);
	if(ret) {
		dev_err(dev, "Could not get reserved memory\n");
		kref_put(&memdev->refcount, simaai_memory_release_memdev);
		return ret;
	}
	memdev->reserved_initialized = true;

	memdev->target = target;

	memdev->stu = simaai_stu_get_by_phandle(dev->of_node, "simaai,stu");
	if (IS_ERR(memdev->stu)) {
		ret = PTR_ERR(memdev->stu);
		if (ret == -EPROBE_DEFER) {
			kref_put(&memdev->refcount, simaai_memory_release_memdev);
			return ret;
		}
		memdev->stu = NULL;
	} else
		dev_info(dev, "Success getting STU handle\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "Could not set DMA mask: %d\n", ret);
		kref_put(&memdev->refcount, simaai_memory_release_memdev);
		return ret;
	}
	memdev->target = target;
	mutex_lock(&dev_lock);
	if (!simaaimem.exist) {
		INIT_LIST_HEAD(&simaaimem.dev_head);
		mutex_init(&simaaimem.buffer_lock);
		INIT_RADIX_TREE(&simaaimem.buffer_root, GFP_KERNEL);
		ret = simaai_create_char_dev(dev);
		if (!ret)
			simaaimem.exist = true;
	}

	if (!ret)
		simaai_memory_registry_add(&simaaimem.dev_head, memdev);
	mutex_unlock(&dev_lock);

	if (ret) {
		dev_err(dev, "Could not create character device %s\n",
				SIMAAI_MEMOERY_DEV_NAME);
		kref_put(&memdev->refcount, simaai_memory_release_memdev);
		return ret;
	}

	platform_set_drvdata(pdev, memdev);
	dev_info(dev, "Registered memory %s\n", of_node_full_name(dev->of_node));
	return 0;
}

static void simaai_memory_remove(struct platform_device *pdev)
{
	struct simaai_memdev *memdev = (struct simaai_memdev *) platform_get_drvdata(pdev);

	/* Detach discovery first; existing buffers retain memdev and its DMA pool. */
	mutex_lock(&dev_lock);
	simaai_memory_registry_remove(memdev);
	mutex_unlock(&dev_lock);
	simaai_memory_export_stop(memdev);
	kref_put(&memdev->refcount, simaai_memory_release_memdev);
}

static const struct of_device_id simaai_memory_match[] = {
	{ .compatible = "simaai,memory-manager" },
	{},
};

MODULE_DEVICE_TABLE(of, simaai_memory_match);

static struct platform_driver simaai_memory_driver = {
	.probe	= simaai_memory_probe,
	.remove	= simaai_memory_remove,
	.driver	= {
		.name	= "simaai-memory",
		.of_match_table	= simaai_memory_match,
		/*
		 * Removal is unsupported.  Reserved memory is owned per struct
		 * device, not per memdev: of_reserved_mem_device_release()
		 * drops every assignment for the device, so a re-probe while an
		 * earlier memdev is still alive (an exported DMA-BUF keeps one
		 * alive past unbind) would strip dev->cma_area from the live
		 * memdev, and every later allocation would come from outside
		 * the reserved region and fail STU translation until reboot.
		 *
		 * Nothing needs bind/unbind: SIMAAI_MEMORY is bool, so this
		 * driver is always built in, and the node lives in base DT
		 * rather than an overlay.  Do not drop this without first
		 * making reserved-memory ownership per-memdev.
		 */
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(simaai_memory_driver);

MODULE_AUTHOR("Roman Bulhakov <roman.bulhakov@sima.ai>");
MODULE_AUTHOR("Yurii Konoalenko <yurii.konovalenko@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai DaVinci family memory management support functions");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_IMPORT_NS("DMA_BUF");

#if IS_ENABLED(CONFIG_SIMAAI_MEMORY_KUNIT_TEST)
static void simaai_memory_provenance_test(struct kunit *test)
{
	struct simaai_memory_filp_buffer cursor = { };

	KUNIT_EXPECT_FALSE(test, simaai_memory_cursor_exportable(&cursor));
	cursor.exportable = true;
	KUNIT_EXPECT_TRUE(test, simaai_memory_cursor_exportable(&cursor));
}

static void simaai_memory_map_state_test(struct kunit *test)
{
	struct simaai_memory_dmabuf_attachment a = { .direction = DMA_NONE };

	KUNIT_EXPECT_EQ(test, simaai_memory_map_state_get(&a, DMA_FROM_DEVICE), 0);
	a.direction = DMA_FROM_DEVICE;
	KUNIT_EXPECT_EQ(test, simaai_memory_map_state_get(&a, DMA_FROM_DEVICE), 1);
	KUNIT_EXPECT_EQ(test, simaai_memory_map_state_get(&a, DMA_TO_DEVICE), -EBUSY);
	KUNIT_EXPECT_FALSE(test, simaai_memory_map_state_put(&a));
	KUNIT_EXPECT_TRUE(test, simaai_memory_map_state_put(&a));
	KUNIT_EXPECT_FALSE(test, simaai_memory_map_state_put(&a));
}

static void simaai_memory_first_map_rollback_test(struct kunit *test)
{
	struct simaai_memory_dmabuf_attachment a = { .direction = DMA_NONE };

	/* dma_map_sgtable() failure rolls back its first-map reservation. */
	KUNIT_ASSERT_EQ(test, simaai_memory_map_state_get(&a, DMA_FROM_DEVICE), 0);
	KUNIT_EXPECT_TRUE(test, simaai_memory_map_state_put(&a));
	KUNIT_EXPECT_EQ(test, a.map_count, 0);
	KUNIT_EXPECT_EQ(test, simaai_memory_map_state_get(&a, DMA_FROM_DEVICE), 0);
	KUNIT_EXPECT_TRUE(test, simaai_memory_map_state_put(&a));
}

static void simaai_memory_removal_gate_test(struct kunit *test)
{
	struct simaai_memdev a = { }, b = { };

	simaai_memory_export_state_init(&a);
	simaai_memory_export_state_init(&b);
	KUNIT_ASSERT_TRUE(test, simaai_memory_exports_allowed(&a));
	KUNIT_ASSERT_TRUE(test, simaai_memory_exports_allowed(&b));
	simaai_memory_export_stop(&a);
	/* Stopping one device must not close the gate on any other. */
	KUNIT_EXPECT_FALSE(test, simaai_memory_exports_allowed(&a));
	KUNIT_EXPECT_TRUE(test, simaai_memory_exports_allowed(&b));
	/* The gate is level-triggered: repeated queries keep their answer. */
	KUNIT_EXPECT_FALSE(test, simaai_memory_exports_allowed(&a));
	KUNIT_EXPECT_TRUE(test, simaai_memory_exports_allowed(&b));
}

static void simaai_memory_registry_test(struct kunit *test)
{
	LIST_HEAD(registry);
	struct simaai_memdev a = { }, b = { };

	INIT_LIST_HEAD(&a.node);
	INIT_LIST_HEAD(&b.node);
	simaai_memory_registry_add(&registry, &a);
	simaai_memory_registry_add(&registry, &b);
	KUNIT_EXPECT_FALSE(test, list_empty(&registry));
	simaai_memory_registry_remove(&a);
	KUNIT_EXPECT_TRUE(test, list_empty(&a.node));
	KUNIT_EXPECT_FALSE(test, list_empty(&registry));
	simaai_memory_registry_remove(&b);
	KUNIT_EXPECT_TRUE(test, list_empty(&registry));
}

static void simaai_memory_mapping_bounds_test(struct kunit *test)
{
	struct simaai_memory_buffer buffer = {
		.phys_addr = 0x100000,
		.size = PAGE_SIZE + 1,
		.aligned_size = 2 * PAGE_SIZE,
	};
	int ret;

	ret = simaai_memory_validate_mapping(&buffer, buffer.phys_addr,
					     buffer.aligned_size);
	KUNIT_EXPECT_EQ(test, ret, 0);
	ret = simaai_memory_validate_mapping(&buffer, buffer.phys_addr,
					     buffer.aligned_size + 1);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = simaai_memory_validate_mapping(&buffer,
					     buffer.phys_addr + PAGE_SIZE,
					     PAGE_SIZE);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = simaai_memory_validate_mapping(&buffer, buffer.phys_addr, 0);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static void simaai_memory_segment_total_test(struct kunit *test)
{
	struct simaai_alloc_args args = { .num_of_segments = 2 };
	u32 total;

	args.size[0] = PAGE_SIZE;
	args.size[1] = 2 * PAGE_SIZE;
	KUNIT_EXPECT_EQ(test, simaai_memory_segment_total(&args, &total), 0);
	KUNIT_EXPECT_EQ(test, total, 3 * PAGE_SIZE);

	args.num_of_segments = 0;
	KUNIT_EXPECT_EQ(test, simaai_memory_segment_total(&args, &total), -EINVAL);
	args.num_of_segments = MAX_SEGMENTS + 1;
	KUNIT_EXPECT_EQ(test, simaai_memory_segment_total(&args, &total), -EINVAL);
	args.num_of_segments = 2;
	args.size[0] = U32_MAX;
	args.size[1] = 1;
	KUNIT_EXPECT_EQ(test, simaai_memory_segment_total(&args, &total), -EINVAL);
	args.size[0] = PAGE_SIZE;
	args.size[1] = 0;
	KUNIT_EXPECT_EQ(test, simaai_memory_segment_total(&args, &total), -EINVAL);
}

struct simaai_memory_export_worker {
	struct simaai_memdev *memdev;
	struct completion start;
	struct completion done;
	bool accepted;
};

static int simaai_memory_export_worker_fn(void *data)
{
	struct simaai_memory_export_worker *worker = data;

	wait_for_completion(&worker->start);
	worker->accepted = simaai_memory_exports_allowed(worker->memdev);
	complete(&worker->done);
	return 0;
}

static void simaai_memory_export_stop_concurrency_test(struct kunit *test)
{
	struct simaai_memdev memdev = { };
	struct simaai_memory_export_worker worker = { .memdev = &memdev };
	struct task_struct *task;
	unsigned long completed;

	simaai_memory_export_state_init(&memdev);
	init_completion(&worker.start);
	init_completion(&worker.done);
	task = kthread_run(simaai_memory_export_worker_fn, &worker,
			   "simaai-export-test");
	KUNIT_ASSERT_FALSE(test, IS_ERR(task));
	simaai_memory_export_stop(&memdev);
	complete(&worker.start);
	completed = wait_for_completion_timeout(&worker.done, msecs_to_jiffies(1000));
	KUNIT_ASSERT_TRUE(test, completed);
	KUNIT_EXPECT_FALSE(test, worker.accepted);
	KUNIT_EXPECT_FALSE(test, simaai_memory_exports_allowed(&memdev));
}

static struct kunit_case simaai_memory_test_cases[] = {
	KUNIT_CASE(simaai_memory_provenance_test),
	KUNIT_CASE(simaai_memory_map_state_test),
	KUNIT_CASE(simaai_memory_first_map_rollback_test),
	KUNIT_CASE(simaai_memory_removal_gate_test),
	KUNIT_CASE(simaai_memory_registry_test),
	KUNIT_CASE(simaai_memory_mapping_bounds_test),
	KUNIT_CASE(simaai_memory_segment_total_test),
	KUNIT_CASE(simaai_memory_export_stop_concurrency_test),
	{}
};

static struct kunit_suite simaai_memory_test_suite = {
	.name = "simaai-memory-dmabuf",
	.test_cases = simaai_memory_test_cases,
};

kunit_test_suite(simaai_memory_test_suite);
#endif
