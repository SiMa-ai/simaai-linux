// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2021 Sima ai
 *
 * Author: Roman Bulhakov <roman.bulhakov@sima.ai>
 */

#include <linux/cdev.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/radix-tree.h>
#include <linux/sched.h>
#include <uapi/linux/simaai/simaai_memory_ioctl.h>
#include <linux/of_reserved_mem.h>

#define SIMAAI_MEMOERY_DEV_NAME "simaai-mem"

struct simaai_memory_buffer {
	struct device		*dev;
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
	struct list_head	node;
};

struct simaai_memory_filp_buffers {
	struct mutex		buffer_lock;
	struct list_head	buffer_head;
};

struct simaai_memdev {
	u32			target;
	struct device		*dev;
	struct list_head	node;
};

struct simaai_memory_device {
	struct list_head	dev_head;

	struct mutex		buffer_lock;
	struct radix_tree_root	buffer_root;

	/* Character device */
	struct cdev		cdev;
	dev_t			dev_no;
	struct class		*dev_class;
	bool			exist;
};

static struct simaai_memory_device simaaimem = { 0 };
static DEFINE_MUTEX(dev_lock);

static void simaai_release_buffer(struct kref *ref)
{
	struct simaai_memory_buffer *buffer =
		container_of(ref, struct simaai_memory_buffer, refcount);

	radix_tree_delete(&simaaimem.buffer_root, buffer->phys_addr);

	if (buffer->parent == NULL ) {
		dma_free_coherent(buffer->dev, buffer->aligned_size,
			  buffer->cpu_addr, buffer->phys_addr);
	}

	kfree(buffer);
}

static void simaai_destroy_buffers(struct simaai_memdev *memdev)
{
	struct radix_tree_iter iter;
	unsigned long indices[16];
	unsigned long index;
	void __rcu **slot;
	struct simaai_memory_buffer *buffer;
	int i, nr;

	mutex_lock(&simaaimem.buffer_lock);

	/* A radix tree is freed by deleting all of its entries */
	index = 0;
	do {
		nr = 0;
		radix_tree_for_each_slot(slot, &simaaimem.buffer_root, &iter, index) {
			buffer = *((struct simaai_memory_buffer **)slot);
			if(buffer->dev == memdev->dev) {
				indices[nr] = iter.index;
				dma_free_coherent(buffer->dev, buffer->aligned_size,
						  buffer->cpu_addr, buffer->phys_addr);
				kfree(buffer);
				if (++nr == 16)
					break;
			}
		}
		for (i = 0; i < nr; i++) {
			index = indices[i];
			radix_tree_delete(&simaaimem.buffer_root, index);
		}
	} while (nr > 0);
	mutex_unlock(&simaaimem.buffer_lock);
}

static struct simaai_memory_buffer *
simaai_allocate_segment_buffer(struct device *dev, u32 target, size_t size, unsigned int flags,
		u64 phys_addr)
{
	struct simaai_memory_buffer *buffer = NULL;
	int res;

	if (!size)
		goto err_alloc;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL)
		goto err_alloc;

	buffer->dev = dev;
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

	return buffer;

err_insert:
	kfree(buffer);
err_alloc:
	return NULL;
}

static struct simaai_memory_buffer *
simaai_allocate_buffer(struct device *dev, u32 target, size_t size, unsigned int flags)
{
	struct simaai_memory_buffer *buffer = NULL;
	int res;

	if (!size)
		goto err_alloc;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL)
		goto err_alloc;

	buffer->dev = dev;
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

static long simaai_memory_insert_cursor(struct device *dev, struct file *filp, dma_addr_t phys_addr)
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
	long ret;
	u32 buffer_size = 0;
	unsigned int iter = 0;

	switch (cmd) {
	case SIMAAI_IOC_MEM_ALLOC_COHERENT:
		if (copy_from_user(&aargs, argp, sizeof(aargs)))
			return -EFAULT;

		mutex_lock(&dev_lock);
		list_for_each_entry(cur, &simaaimem.dev_head, node) {
			if (cur->target == aargs.target) {
				dev = cur->dev;
				break;
			}
		}
		mutex_unlock(&dev_lock);
		if (dev == NULL)
			return -EINVAL;

		if (aargs.num_of_segments != 1) {
			for(iter = 0; iter < aargs.num_of_segments; iter++) {
				buffer_size += aargs.size[iter];
			}
		} else {
			buffer_size = aargs.size[0];
		}

		buffer = simaai_allocate_buffer(dev, aargs.target, buffer_size, aargs.flags);
		if (buffer == NULL) {
			dev_err(dev, "Could not allocate buffer\n");
			return -ENOMEM;
		}
		aargs.aligned_size = buffer->aligned_size;
		aargs.phys_addr[0] = buffer->phys_addr;
		aargs.bus_addr[0] = buffer->bus_addr;
		aargs.offset[0] = buffer->offset;

		ret = simaai_memory_insert_cursor(dev, filp, buffer->phys_addr);
		if(ret != 0)
			return ret;

		for (iter = 1; iter < aargs.num_of_segments; iter++) {

			child = simaai_allocate_segment_buffer(dev, aargs.target, aargs.size[iter], aargs.flags,
						aargs.phys_addr[iter - 1] + aargs.size[iter -1]);
			if (child == NULL) {
				dev_err(dev, "Failed to allocate segment buffer\n");
				free_segment_buffers(filp, &aargs, iter);
				return -ENOMEM;
			}
								
			child->parent = buffer;
			kref_get(&child->parent->refcount);
			aargs.aligned_size = child->aligned_size;
			aargs.phys_addr[iter] = child->phys_addr;
			aargs.bus_addr[iter] = child->bus_addr;
			aargs.offset[iter] = aargs.offset[iter - 1] + aargs.size[iter - 1];
			child->offset = aargs.offset[iter];

			ret = simaai_memory_insert_cursor(dev, filp, child->phys_addr);
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
		ret = simaai_memory_insert_cursor(dev, filp, buffer->phys_addr);
		if(ret != 0)
			return ret;

		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		break;
	default:
		pr_info("simaai-mem: Bad ioctl number\n");
		return -EINVAL;
	}

	return 0;
}

#define pgprot_dmacoherent_cached(prot) \
	__pgprot_modify(prot, PTE_ATTRINDX_MASK, \
			PTE_ATTRINDX(MT_NORMAL) | PTE_PXN | PTE_UXN)

static int simaai_memory_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct simaai_memory_buffer *buffer;
	u64 paddr;
	unsigned long vsize;

	paddr = vma->vm_pgoff << PAGE_SHIFT;
	vsize = vma->vm_end - vma->vm_start;
	mutex_lock(&simaaimem.buffer_lock);
	buffer = radix_tree_lookup(&simaaimem.buffer_root, paddr);
	mutex_unlock(&simaaimem.buffer_lock);

	if(buffer == NULL) {
		pr_err("simaai-mem: Can't mmap physical address: 0x%lx\n", paddr);
		return -EINVAL;
	}

	if (!(buffer->flags & SIMAAI_BUFFER_FLAG_CACHED))
		vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_dmacoherent_cached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT, vsize,
			    vma->vm_page_prot)) {
		dev_info(buffer->dev, "remap_pfn_range failed\n");
		return -EAGAIN;
	}

	return 0;
}

ssize_t simaai_memory_dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	char header [] = "| Physical address |      Parent      | #Ref |       Size       | Target |   Owner   |\n";
	int line_length = sizeof(header);
	int l;
	ssize_t total = 0;
	struct simaai_memory_buffer *buffer;
	struct radix_tree_iter iter;
	unsigned long index = 0;
	void __rcu **slot;
	loff_t offset;

	if (*f_pos < line_length) {
		l = line_length - *f_pos;
		if (l > count)
			l = count;
		if (copy_to_user(&buf[total], &header[*f_pos], l))
			return -EFAULT;
		*f_pos += l;
		total += l;
		count -= l;
		offset += line_length;
	}

	mutex_lock(&simaaimem.buffer_lock);
	radix_tree_for_each_slot(slot, &simaaimem.buffer_root, &iter, index) {
		l = line_length - (*f_pos % line_length);
		l = l < count ? l : count;
		offset += l;
		if(*f_pos > offset)
			continue;
		buffer = *((struct simaai_memory_buffer **)slot);
		sprintf(header, "|   0x%010llx   |   0x%010llx   |  %*d  |   0x%010lx   |   %*d   | %*d |\n",
			buffer->phys_addr, (buffer->parent) ? (buffer->parent->phys_addr) : 0, 2,
			kref_read(&buffer->refcount), buffer->size, 2, buffer->target, 9, buffer->owner);
		if (copy_to_user(&buf[total], &header[*f_pos % line_length], l))
			return -EFAULT;
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

	simaaimem.dev_class = class_create(THIS_MODULE, SIMAAI_MEMOERY_DEV_NAME);
	if (IS_ERR(simaaimem.dev_class)) {
		dev_err(memdev, "Failed: class_create\n");
		ret = PTR_ERR(simaaimem.dev_class);
		goto err_class;
	}

	dev = device_create(simaaimem.dev_class,
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
	class_destroy(simaaimem.dev_class);
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

	memdev = devm_kzalloc(dev, sizeof(*memdev), GFP_KERNEL);
	if (!memdev)
		return -ENOMEM;

	memdev->dev = dev;

	ret = of_property_read_u32(dev->of_node, "simaai,target", &target);
	if (ret) {
		dev_warn(dev, "Could not obtain simaai,target property\n");
	}

	ret = of_reserved_mem_device_init(dev);
	if(ret) {
		dev_err(dev, "Could not get reserved memory\n");
		return ret;
	}

	memdev->target = target;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "Could not set DMA mask: %d\n", ret);
		return ret;
	}

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
		list_add(&memdev->node, &simaaimem.dev_head);
	mutex_unlock(&dev_lock);
	
	if (ret) {
		dev_err(dev, "Could not create character device %s\n",
			SIMAAI_MEMOERY_DEV_NAME);
		return ret;
	}

	platform_set_drvdata(pdev, memdev);
	dev_info(dev, "Registered memory %s\n", of_node_full_name(dev->of_node));

	return 0;
}

static int simaai_memory_remove(struct platform_device *pdev)
{
	struct simaai_memdev *memdev = (struct simaai_memdev *) platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	simaai_destroy_buffers(memdev);
	of_reserved_mem_device_release(dev);

	return 0;
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
	},
};

module_platform_driver(simaai_memory_driver);

MODULE_AUTHOR("Roman Bulhakov <roman.bulhakov@sima.ai>");
MODULE_AUTHOR("Yurii Konoalenko <yurii.konovalenko@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai DaVinci family memory management support functions");
MODULE_LICENSE("Dual MIT/GPL");
