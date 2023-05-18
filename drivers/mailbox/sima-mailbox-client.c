// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright SiMa.ai (C) 2021. All rights reserved
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched/signal.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <uapi/linux/sima/sima-mailbox.h>

/* Tx timeout in milliseconds */
#define SIMA_TX_TOUT		500

/* IPC descriptors FIFO depth */
#define SIMA_DESC_FIFO_DEPTH	16

struct sima_mbox_client {
	const char *name;
	struct device *dev;
	/* Tx */
	struct mbox_client tx_client;
	struct mbox_chan *tx_channel;
	void __iomem *tx_ring;
	struct mutex write_lock;
	/* Rx */
	struct mbox_client rx_client;
	struct mbox_chan *rx_channel;
	void __iomem *rx_ring;
	u32 rx_ring_size;
	struct kfifo rx_fifo;
	spinlock_t rx_fifo_lock;
	wait_queue_head_t rx_fifo_waitq;
	/* Character device stuff */
	struct cdev cdev;
	dev_t dev_no;
	struct class *dev_class;
	struct mutex dev_lock;
	int dev_open_count;
};

static int init_device(struct sima_mbox_client *mbox)
{
	int ret;

	kfifo_reset(&mbox->rx_fifo);

	mbox->tx_channel = mbox_request_channel_byname(&mbox->tx_client, "tx");
	if (IS_ERR(mbox->tx_channel)) {
		dev_err(mbox->dev, "Failed to request Tx channel\n");
		mbox->tx_channel = NULL;
		ret = -EINVAL;
		goto err;
	}

	mbox->rx_channel = mbox_request_channel_byname(&mbox->rx_client, "rx");
	if (IS_ERR(mbox->rx_channel)) {
		dev_err(mbox->dev, "Failed to request Rx channel\n");
		mbox->rx_channel = NULL;
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	if (mbox->tx_channel) {
		mbox_free_channel(mbox->tx_channel);
		mbox->tx_channel = NULL;
	}
	if (mbox->rx_channel) {
		mbox_free_channel(mbox->rx_channel);
		mbox->rx_channel = NULL;
	}
	return ret;
}

static void release_dev(struct sima_mbox_client *mbox)
{
	if (mbox->tx_channel) {
		mbox_free_channel(mbox->tx_channel);
		mbox->tx_channel = NULL;
	}
	if (mbox->rx_channel) {
		mbox_free_channel(mbox->rx_channel);
		mbox->rx_channel = NULL;
	}
}

static int mbox_dev_open(struct inode *inode, struct file *filp)
{
	struct sima_mbox_client *mbox = container_of(inode->i_cdev,
						     struct sima_mbox_client,
						     cdev);
	int ret = 0;

	filp->private_data = mbox;

	mutex_lock(&mbox->dev_lock);
	if (!mbox->dev_open_count)
		ret = init_device(mbox);
	if (!ret)
		mbox->dev_open_count++;
	mutex_unlock(&mbox->dev_lock);

	return ret;
}

static int mbox_dev_release(struct inode *inode, struct file *filp)
{
	struct sima_mbox_client *mbox = filp->private_data;

	mutex_lock(&mbox->dev_lock);
	mbox->dev_open_count--;
	if (!mbox->dev_open_count)
		release_dev(mbox);
	mutex_unlock(&mbox->dev_lock);

	return 0;
}

static ssize_t mbox_dev_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *fpos)
{
	struct sima_mbox_client *mbox = filp->private_data;
	struct sima_ipc_message msg;
	const size_t len = sizeof(msg);
	u32 i = 0; /* The first descriptor is used for now */
	int ret;

	if (count != len) {
		dev_err(mbox->dev,
			"Incorrect message length %zd, shall be %zd\n",
			count, len);
		return -EINVAL;
	}

	ret = copy_from_user(&msg, buf, len);
	if (ret)
		return -EFAULT;

	if (mutex_lock_interruptible(&mbox->write_lock))
		return -ERESTARTSYS;
	memcpy_toio(mbox->tx_ring, &msg, len);
	ret = mbox_send_message(mbox->tx_channel, &i);
	mutex_unlock(&mbox->write_lock);

	if (ret < 0) {
		dev_err(mbox->dev, "Failed to send message via mailbox\n");
		return ret;
	}

	return len;
}

static ssize_t mbox_dev_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *fpos)
{
	struct inode *inode = file_inode(filp);
	struct sima_mbox_client *mbox = filp->private_data;
	struct sima_ipc_message msg;
	int ret;

	if (count < sizeof(msg))
		return -EINVAL;

	while (!kfifo_out_spinlocked(&mbox->rx_fifo, &msg, sizeof(msg),
				     &mbox->rx_fifo_lock)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(mbox->rx_fifo_waitq,
					       !kfifo_is_empty(&mbox->rx_fifo));
		if (ret)
			return ret;
	}

	ret = copy_to_user(buf, &msg, sizeof(msg));
	if (ret) {
		ret = -EFAULT;
	} else {
		inode->i_atime = current_time(inode);
		ret = sizeof(msg);
	}

	return ret;
}

static __poll_t mbox_dev_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct sima_mbox_client *mbox = filp->private_data;
	/* Write is always available */
	__poll_t ret = EPOLLOUT | EPOLLWRNORM;

	poll_wait(filp, &mbox->rx_fifo_waitq, wait);

	if (!kfifo_is_empty(&mbox->rx_fifo))
		ret = EPOLLIN | EPOLLRDNORM;

	return ret;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open	= mbox_dev_open,
	.release = mbox_dev_release,
	.write	= mbox_dev_write,
	.read	= mbox_dev_read,
	.poll	= mbox_dev_poll,
	.llseek	= noop_llseek,
};

static int create_char_dev(struct sima_mbox_client *mbox)
{
	int ret;

	ret = alloc_chrdev_region(&mbox->dev_no, 0, 1, mbox->name);
	if (ret != 0) {
		dev_err(mbox->dev, "Failed: alloc_chrdev_region\n");
		return ret;
	}

	cdev_init(&mbox->cdev, &fops);
	mbox->cdev.owner = THIS_MODULE;
	mbox->cdev.ops = &fops;

	ret = cdev_add(&mbox->cdev, mbox->dev_no, 1);
	if (ret != 0) {
		dev_err(mbox->dev, "Failed: cdev_add\n");
		goto err_cdev;
	}

	mbox->dev_class = class_create(THIS_MODULE, mbox->name);
	if (IS_ERR(mbox->dev_class)) {
		dev_err(mbox->dev, "Failed: class_create\n");
		ret = PTR_ERR(mbox->dev_class);
		goto err_class;
	}

	device_create(mbox->dev_class, NULL, mbox->dev_no, NULL, mbox->name);

	return 0;

err_class:
	cdev_del(&mbox->cdev);
err_cdev:
	unregister_chrdev_region(mbox->dev_no, 1);
	return ret;
}

static void remove_char_dev(struct sima_mbox_client *mbox)
{
	device_destroy(mbox->dev_class, mbox->dev_no);
	class_destroy(mbox->dev_class);
	cdev_del(&mbox->cdev);
	unregister_chrdev_region(mbox->dev_no, 1);
}

static void message_from_remote(struct mbox_client *client, void *message)
{
	struct sima_mbox_client *mbox = dev_get_drvdata(client->dev);
	unsigned long flags;
	struct sima_ipc_message msg;
	u32 i = *(u32 *)message;
	void __iomem *desc = ((struct sima_ipc_message *)mbox->rx_ring) + i;

	if (unlikely(i >= mbox->rx_ring_size)) {
		dev_err(mbox->dev, "Rx descriptor index is out of range\n");
		return;
	}

	spin_lock_irqsave(&mbox->rx_fifo_lock, flags);
	memcpy_fromio(&msg, desc, sizeof(msg));
	kfifo_in(&mbox->rx_fifo, &msg, sizeof(msg));
	spin_unlock_irqrestore(&mbox->rx_fifo_lock, flags);

	wake_up_interruptible(&mbox->rx_fifo_waitq);
}

static int sima_mbox_client_probe(struct platform_device *pdev)
{
	struct sima_mbox_client *mbox;
	struct resource *res;
	int ret;
	u32 tx_tout = SIMA_TX_TOUT;
	u32 fifo_depth = SIMA_DESC_FIFO_DEPTH;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->dev = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node, "sima,dev-name",
				      &mbox->name);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot obtain sima,dev-name property\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "sima,tx-tout-ms",
				   &tx_tout);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "cannot obtain sima,tx-tout-ms property\n");
	}

	ret = of_property_read_u32(pdev->dev.of_node, "sima,rx-fifo-depth",
				   &fifo_depth);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "cannot obtain sima,rx-fifo-depth property\n");
	}

	/* Tx descriptors region */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tx_desc");
	mbox->tx_ring = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mbox->tx_ring)) {
		dev_err(&pdev->dev, "cannot remap Tx IPC descriptors memory\n");
		return PTR_ERR(mbox->tx_ring);
	}

	/* Rx descriptors region */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rx_desc");
	mbox->rx_ring = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mbox->rx_ring)) {
		dev_err(&pdev->dev, "cannot remap Rx IPC descriptors memory\n");
		return PTR_ERR(mbox->rx_ring);
	}

	mbox->rx_ring_size =
		resource_size(res) / sizeof(struct sima_ipc_message);

	ret = kfifo_alloc(&mbox->rx_fifo,
			  fifo_depth * sizeof(struct sima_ipc_message),
			  GFP_KERNEL);
	if (ret) {
		dev_err(&pdev->dev, "cannot allocate Rx FIFO\n");
		return ret;
	}

	mbox->tx_client.dev = &pdev->dev;
	mbox->tx_client.rx_callback = NULL;
	mbox->tx_client.tx_done = NULL;
	mbox->tx_client.tx_block = true;
	mbox->tx_client.knows_txdone = false;
	mbox->tx_client.tx_tout = tx_tout;

	mbox->rx_client.dev = &pdev->dev;
	mbox->rx_client.rx_callback = message_from_remote;
	mbox->rx_client.tx_done = NULL;
	mbox->rx_client.tx_block = true;
	mbox->rx_client.knows_txdone = false;
	mbox->rx_client.tx_tout = tx_tout;

	mutex_init(&mbox->dev_lock);
	mutex_init(&mbox->write_lock);
	spin_lock_init(&mbox->rx_fifo_lock);
	init_waitqueue_head(&mbox->rx_fifo_waitq);

	ret = create_char_dev(mbox);
	if (ret != 0)
		return ret;

	platform_set_drvdata(pdev, mbox);
	dev_info(&pdev->dev, "Successfully registered\n");

	return 0;
}

static int sima_mbox_client_remove(struct platform_device *pdev)
{
	struct sima_mbox_client *mbox = platform_get_drvdata(pdev);

	remove_char_dev(mbox);
	kfifo_free(&mbox->rx_fifo);

	return 0;
}

static const struct of_device_id sima_mbox_client_match[] = {
	{ .compatible = "sima,mailbox-client" },
	{},
};

MODULE_DEVICE_TABLE(of, sima_mbox_client_match);

static struct platform_driver sima_mbox_client_driver = {
	.probe	= sima_mbox_client_probe,
	.remove	= sima_mbox_client_remove,
	.driver	= {
		.name	= "sima_mailbox_client",
		.of_match_table	= sima_mbox_client_match,
	},
};

module_platform_driver(sima_mbox_client_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SiMa.ai Mailbox Client");
MODULE_AUTHOR("SiMa.ai");
