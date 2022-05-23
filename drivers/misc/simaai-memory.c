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
#include <uapi/linux/simaai/simaai_memory_ioctl.h>
#include <linux/of_reserved_mem.h>

struct simaai_memory_buffer {
	struct device *dev;
	/* NOTE: change in datatype of id requires change in target_mask of per device struct */
	unsigned int id;
	void *cpu_addr;
	dma_addr_t phys_addr;
	/* Requested memory size */
	size_t size;
	/* Allocated memory size aligned to page boundary */
	size_t aligned_size;
	bool cma_region;
	struct kref refcount;
	struct list_head node;
};

struct simaai_memory_device {
	const char *name;
	struct device *dev;

	struct mutex buffer_lock;
	struct list_head buffer_head;

	/* Character device */
	struct cdev cdev;
	dev_t dev_no;
	struct class *dev_class;

	/* target memory mask, depends on memory buffer id*/
	unsigned int target_mask;
};

static void simaai_release_buffer(struct kref *ref)
{
	struct simaai_memory_buffer *buffer =
		container_of(ref, struct simaai_memory_buffer, refcount);

	list_del(&buffer->node);

	if (buffer->cma_region) {
		dma_free_contiguous(buffer->dev,
				    (struct page *)buffer->cpu_addr,
				    buffer->aligned_size);
	} else {
		dma_free_coherent(buffer->dev,
				  buffer->aligned_size,
				  buffer->cpu_addr,
				  buffer->phys_addr);
	}

	dev_info(buffer->dev, "Delete memory block id: %u(%#x) size: %zu p_addr: 0x%0llx\n",
		 buffer->id, buffer->id, buffer->aligned_size, buffer->phys_addr);

	kfree(buffer);
}

static void simaai_destroy_buffers(struct simaai_memory_device *simaai_memory)
{
	struct list_head *cur, *next;
	struct simaai_memory_buffer *buffer;

	mutex_lock(&simaai_memory->buffer_lock);
	list_for_each_safe(cur, next, &simaai_memory->buffer_head) {
		buffer = list_entry(cur, struct simaai_memory_buffer, node);
		simaai_release_buffer(&buffer->refcount);
	}
	mutex_unlock(&simaai_memory->buffer_lock);
}

static struct simaai_memory_buffer *
simaai_allocate_buffer(struct simaai_memory_device *simaai_memory, size_t size,
		       bool cma_region)
{
	struct simaai_memory_buffer *buffer = NULL, *last = NULL;

	if (!size)
		goto error;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		goto error;

	buffer->id = 0;
	buffer->dev = simaai_memory->dev;
	buffer->size = size;
	buffer->aligned_size = PAGE_ALIGN(size);

	if (cma_region && (buffer->aligned_size > PAGE_SIZE)) {
		buffer->cpu_addr = dma_alloc_contiguous(simaai_memory->dev,
							buffer->aligned_size,
							GFP_USER);
		if (!buffer->cpu_addr) {
			dev_err(simaai_memory->dev,
				"dma_alloc_contiguous alloc of %zu bytes failed\n",
				buffer->aligned_size);
			goto error;
		}
		buffer->phys_addr =
			page_to_phys((struct page *)buffer->cpu_addr);
		buffer->cma_region = true;
	} else {
		buffer->cpu_addr = dma_alloc_coherent(simaai_memory->dev,
						      buffer->aligned_size,
						      &buffer->phys_addr,
						      GFP_USER);
		if (!buffer->cpu_addr) {
			dev_err(simaai_memory->dev,
				"dma_alloc_coherent alloc of %zu bytes failed\n",
				buffer->aligned_size);
			goto error;
		}
		buffer->cma_region = false;
	}

	kref_init(&buffer->refcount);

	mutex_lock(&simaai_memory->buffer_lock);
	if (!list_empty(&simaai_memory->buffer_head)) {
		last = list_last_entry(&simaai_memory->buffer_head,
				       struct simaai_memory_buffer, node);
		buffer->id = last->id + 1;
	}

	/* Add target mask to id */
	buffer->id |= simaai_memory->target_mask;

	list_add_tail(&buffer->node, &simaai_memory->buffer_head);
	dev_info(buffer->dev, "Allocate memory block id: %u(%#x) size: %zu p_addr: 0x%0llx\n",
		 buffer->id, buffer->id, buffer->aligned_size, buffer->phys_addr);
	mutex_unlock(&simaai_memory->buffer_lock);

	return buffer;
error:
	kfree(buffer);
	return NULL;
}

static void simaai_free_buffer(struct simaai_memory_device *simaai_memory,
			       struct simaai_memory_buffer *buffer)
{
	mutex_lock(&simaai_memory->buffer_lock);
	kref_put(&buffer->refcount, simaai_release_buffer);
	mutex_unlock(&simaai_memory->buffer_lock);
}

static struct simaai_memory_buffer *
simaai_get_buffer(struct simaai_memory_device *simaai_memory, unsigned int id)
{
	struct simaai_memory_buffer *buffer, *cur;

	mutex_lock(&simaai_memory->buffer_lock);
	buffer = NULL;
	list_for_each_entry(cur, &simaai_memory->buffer_head, node) {
		if (cur->id == id) {
			buffer = cur;
			kref_get(&buffer->refcount);
			break;
		}
	}
	mutex_unlock(&simaai_memory->buffer_lock);

	return buffer;
}

static int simaai_memory_dev_open(struct inode *inode, struct file *filp)
{
	/* Used for buffer binding */
	filp->private_data = NULL;

	return 0;
}

static int simaai_memory_dev_release(struct inode *inode, struct file *filp)
{
	struct simaai_memory_device *simaai_memory =
		container_of(inode->i_cdev, struct simaai_memory_device,
			     cdev);
	struct simaai_memory_buffer *buffer = filp->private_data;

	if (buffer) {
		simaai_free_buffer(simaai_memory, buffer);
		filp->private_data = NULL;
	}

	return 0;
}

static long simaai_memory_dev_ioctl(struct file *filp, unsigned int cmd,
				    unsigned long arg)
{
	struct simaai_memory_device *simaai_memory =
		container_of(filp->f_inode->i_cdev, struct simaai_memory_device,
			     cdev);
	void __user *argp = (void __user *)arg;
	struct simaai_memory_buffer *buffer = filp->private_data;
	unsigned int size;
	unsigned int id;
	struct simaai_memory_info info;
	bool use_cma = false;

	switch (cmd) {
	case SIMAAI_IOC_MEM_ALLOC_GENERIC:
		use_cma = true;
		/* fall through */
	case SIMAAI_IOC_MEM_ALLOC_COHERENT:
		if (buffer)
			return -EINVAL;

		if (copy_from_user(&size, argp, sizeof(size)))
			return -EFAULT;

		buffer = simaai_allocate_buffer(simaai_memory, size, use_cma);
		if (!buffer) {
			dev_err(simaai_memory->dev, "Could not allocate buffer\n");
			return -ENOMEM;
		}

		filp->private_data = buffer;
		break;
	case SIMAAI_IOC_MEM_FREE:
		if (!buffer)
			return -EINVAL;

		simaai_free_buffer(simaai_memory, buffer);

		filp->private_data = NULL;
		break;
	case SIMAAI_IOC_MEM_GET:
		/* Support only one buffer attached to fd */
		if (buffer)
			return -EINVAL;

		if (copy_from_user(&id, argp, sizeof(id)))
			return -EFAULT;

		buffer = simaai_get_buffer(simaai_memory, id);
		if (!buffer) {
			dev_err(simaai_memory->dev, "Could not get buffer id: %u\n",
				id);
			return -ENODEV;
		}

		filp->private_data = buffer;
		break;
	case SIMAAI_IOC_MEM_INFO:
		if (!buffer)
			return -EINVAL;

		info.id = buffer->id;
		info.size = buffer->size;
		info.aligned_size = buffer->aligned_size;
		info.phys_addr = buffer->phys_addr;

		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		break;
	default:
		dev_info(simaai_memory->dev, "Bad ioctl number\n");
		return -EINVAL;
	}

	return 0;
}

static int simaai_memory_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct simaai_memory_device *simaai_memory =
		container_of(filp->f_inode->i_cdev, struct simaai_memory_device,
			     cdev);
	struct simaai_memory_buffer *buffer = filp->private_data;
	unsigned long offset;
	unsigned long paddr;
	unsigned long vsize;
	unsigned long psize;

	if (!buffer)
		return -ENODEV;

	offset = vma->vm_pgoff << PAGE_SHIFT;
	paddr = buffer->phys_addr + offset;
	vsize = vma->vm_end - vma->vm_start;
	psize = buffer->aligned_size - offset;

	if (vsize > psize) {
		dev_warn(simaai_memory->dev, "mapping spans too high\n");
		return -EINVAL;
	}

	vma->vm_page_prot = phys_mem_access_prot(filp,
						 paddr >> PAGE_SHIFT,
						 vsize,
						 vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT, vsize,
			    vma->vm_page_prot)) {
		dev_info(simaai_memory->dev, "remap_pfn_range failed\n");
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open	= simaai_memory_dev_open,
	.release = simaai_memory_dev_release,
	.unlocked_ioctl = simaai_memory_dev_ioctl,
	.mmap = simaai_memory_dev_mmap,
	.llseek	= noop_llseek,
};

static int simaai_create_char_dev(struct simaai_memory_device *simaai_memory)
{
	int ret;
	struct device *dev;

	ret = alloc_chrdev_region(&simaai_memory->dev_no, 0, 1,
				  simaai_memory->name);
	if (ret) {
		dev_err(simaai_memory->dev, "Failed: alloc_chrdev_region\n");
		return ret;
	}

	cdev_init(&simaai_memory->cdev, &fops);
	simaai_memory->cdev.owner = THIS_MODULE;
	simaai_memory->cdev.ops = &fops;

	ret = cdev_add(&simaai_memory->cdev, simaai_memory->dev_no, 1);
	if (ret) {
		dev_err(simaai_memory->dev, "Failed: cdev_add\n");
		goto err_cdev;
	}

	simaai_memory->dev_class = class_create(THIS_MODULE,
						simaai_memory->name);
	if (IS_ERR(simaai_memory->dev_class)) {
		dev_err(simaai_memory->dev, "Failed: class_create\n");
		ret = PTR_ERR(simaai_memory->dev_class);
		goto err_class;
	}

	dev = device_create(simaai_memory->dev_class,
			    NULL,
			    simaai_memory->dev_no,
			    NULL,
			    simaai_memory->name);
	if (IS_ERR(dev)) {
		dev_err(simaai_memory->dev, "Could not create files\n");
		ret = PTR_ERR(dev);
		goto err_device;
	}

	return 0;

err_device:
	class_destroy(simaai_memory->dev_class);
err_class:
	cdev_del(&simaai_memory->cdev);
err_cdev:
	unregister_chrdev_region(simaai_memory->dev_no, 1);
	return ret;
}

static void simaai_remove_char_dev(struct simaai_memory_device *simaai_mm)
{
	device_destroy(simaai_mm->dev_class, simaai_mm->dev_no);
	class_destroy(simaai_mm->dev_class);
	cdev_del(&simaai_mm->cdev);
	unregister_chrdev_region(simaai_mm->dev_no, 1);
}

static int simaai_memory_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simaai_memory_device *simaai_memory;
	int ret, target = SIMAAI_TARGET_ALLOCATOR_DRAM;

	simaai_memory = devm_kzalloc(dev, sizeof(*simaai_memory), GFP_KERNEL);
	if (!simaai_memory)
		return -ENOMEM;

	simaai_memory->dev = dev;
	ret = of_property_read_string(dev->of_node, "simaai,dev-name",
				      &simaai_memory->name);
	if (ret) {
		dev_err(dev, "Could not obtain simaai,dev-name property\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "simaai,target",
				      &target);
	if (ret) {
		dev_warn(dev, "Could not obtain simaai,target property\n");
	}

    /* Initialize reserved memory resources */
    if(target != SIMAAI_TARGET_ALLOCATOR_DRAM) {
		ret = of_reserved_mem_device_init(dev);
		if(ret) {
			dev_err(dev, "Could not get reserved memory\n");
			return ret;
		}
	}
	simaai_memory->target_mask = SIMAAI_SET_TARGET_ALLOCATOR(target);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "Could not set DMA mask: %d\n", ret);
		return ret;
	}

	ret = simaai_create_char_dev(simaai_memory);
	if (ret) {
		dev_err(dev, "Could not create character device %s\n",
			simaai_memory->name);
		return ret;
	}

	mutex_init(&simaai_memory->buffer_lock);
	INIT_LIST_HEAD(&simaai_memory->buffer_head);

	platform_set_drvdata(pdev, simaai_memory);
	dev_info(dev, "Registered\n");

	return 0;
}

static int simaai_memory_remove(struct platform_device *pdev)
{
	struct simaai_memory_device *simaai_memory = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	simaai_remove_char_dev(simaai_memory);
	simaai_destroy_buffers(simaai_memory);
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
MODULE_DESCRIPTION("SiMa.ai DaVinci family memory management support functions");
MODULE_LICENSE("Dual MIT/GPL");
