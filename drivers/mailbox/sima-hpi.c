// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright SiMa.ai (C) 2021. All rights reserved
 */

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/platform_device.h>

#define SIMAAI_HPI_DEV_NAME "simaai-hpi"

struct simaai_hpi {
	int irq;
	void __iomem *base;
	struct device *dev;
	struct cdev cdev;
	struct class *dev_class;
	dev_t dev_no;
	int dev_open_count;
	wait_queue_head_t q;
	atomic_t data_avail_to_read;
};

static irqreturn_t simaai_hpi_interrupt(int irq, void *dev_id)
{
	struct simaai_hpi *hpi = (struct simaai_hpi *)dev_id;

	BUG_ON(hpi->irq != irq);
	disable_irq_nosync(irq);
	atomic_inc(&hpi->data_avail_to_read);
	wake_up(&hpi->q);

	return IRQ_HANDLED;
}

static ssize_t simaai_hpi_read(struct file *filp, char  __user *bufp, size_t len, loff_t *ppos)
{
	return 0;
}

static ssize_t simaai_hpi_write(struct file *filp, const char *buff, size_t len, loff_t * off)
{
	struct simaai_hpi *hpi = (struct simaai_hpi *)filp->private_data;
	enable_irq(hpi->irq);

	return 0;
}

static int simaai_hpi_open(struct inode *inode, struct file *filp)
{
	struct simaai_hpi *hpi = container_of(inode->i_cdev,
						     struct simaai_hpi, cdev);
	filp->private_data = hpi;

	return 0;
}

static int simaai_hpi_release(struct inode *inop, struct file *filp)
{
	return 0;
}

static unsigned int simaai_hpi_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct simaai_hpi *hpi = (struct simaai_hpi *)filp->private_data;
	unsigned int mask = 0;
	static unsigned int nofirst = 0;

	poll_wait(filp, &hpi->q, wait);

	if(atomic_read(&hpi->data_avail_to_read)) {
		mask |= POLLIN | POLLRDNORM;
		atomic_set(&hpi->data_avail_to_read, 0);
	}

	if(!nofirst)
		nofirst = 1;

	return mask;
}
static struct file_operations simaai_hpi_fops = {
	.owner = THIS_MODULE,
	.open = simaai_hpi_open,
	.release = simaai_hpi_release,
	.write = simaai_hpi_write,
	.read = simaai_hpi_read,
	.poll = simaai_hpi_poll,
};

static int simaai_hpi_create_dev(struct simaai_hpi *hpi)
{
	int ret;

	ret = alloc_chrdev_region(&hpi->dev_no, 0, 1, SIMAAI_HPI_DEV_NAME);
	if (ret != 0) {
		dev_err(hpi->dev, "Failed: alloc_chrdev_region\n");
		return ret;
	}

	cdev_init(&hpi->cdev, &simaai_hpi_fops);
	hpi->cdev.owner = THIS_MODULE;
	hpi->cdev.ops = &simaai_hpi_fops;

	ret = cdev_add(&hpi->cdev, hpi->dev_no, 1);
	if (ret != 0) {
		dev_err(hpi->dev, "Failed: cdev_add\n");
		goto err_cdev;
	}

	hpi->dev_class = class_create(THIS_MODULE, SIMAAI_HPI_DEV_NAME);
	if (IS_ERR(hpi->dev_class)) {
		dev_err(hpi->dev, "Failed: class_create\n");
		ret = PTR_ERR(hpi->dev_class);
		goto err_class;
	}

	device_create(hpi->dev_class, NULL, hpi->dev_no, NULL, SIMAAI_HPI_DEV_NAME);

	return 0;

err_class:
	cdev_del(&hpi->cdev);
err_cdev:
	unregister_chrdev_region(hpi->dev_no, 1);
	return ret;
}

static void remove_char_dev(struct simaai_hpi *hpi)
{
	device_destroy(hpi->dev_class, hpi->dev_no);
	class_destroy(hpi->dev_class);
	cdev_del(&hpi->cdev);
	unregister_chrdev_region(hpi->dev_no, 1);
}

static int simaai_hpi_probe(struct platform_device *pdev)
{
	struct simaai_hpi *hpi;
	struct resource	*regs;
	int ret;

	hpi = devm_kzalloc(&pdev->dev, sizeof(*hpi), GFP_KERNEL);
	if (!hpi)
		return -ENOMEM;

	hpi->dev = &pdev->dev;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	hpi->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(hpi->base)) {
		dev_err(hpi->dev, "Wrong memory resource\n");
		return PTR_ERR(hpi->base);
	}

	hpi->irq = platform_get_irq(pdev, 0);
	if (hpi->irq < 0) {
		dev_err(hpi->dev, "Error geting HPI IRQ\n");
		return hpi->irq;
	}

	ret = devm_request_irq(hpi->dev, hpi->irq, simaai_hpi_interrupt,
			       IRQF_SHARED, KBUILD_MODNAME, hpi);
	if (ret) {
		dev_err(hpi->dev, "Error requesting HPI IRQ\n");
		return ret;
	}

	ret = irq_set_affinity(hpi->irq, cpumask_of(0));
	if (ret) {
		dev_err(hpi->dev, "Error setting HPI IRQ affinity\n");
		return ret;
	}

	init_waitqueue_head(&hpi->q);
	atomic_set(&hpi->data_avail_to_read, 0);
	platform_set_drvdata(pdev, hpi);
	ret = simaai_hpi_create_dev(hpi);

	dev_info(&pdev->dev, "HPI registered\n");

	return ret;
}

static int simaai_hpi_remove(struct platform_device *pdev)
{
	struct simaai_hpi *hpi = platform_get_drvdata(pdev);

	remove_char_dev(hpi);

	return 0;
}

static const struct of_device_id simaai_hpi_match[] = {
	{ .compatible = "simaai,hpi-1.0" },
	{},
};

MODULE_DEVICE_TABLE(of, simaai_hpi_match);

static struct platform_driver simaai_hpi_driver = {
	.probe	= simaai_hpi_probe,
	.remove	= simaai_hpi_remove,
	.driver	= {
		.name	= "simaai-hpi",
		.of_match_table	= simaai_hpi_match,
	},
};

module_platform_driver(simaai_hpi_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SiMa.ai HPI specific functions");
MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_ALIAS("platform:sima-hpi");
