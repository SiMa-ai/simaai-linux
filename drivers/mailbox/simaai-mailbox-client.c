// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright SiMa.ai (C) 2021,2025 All rights reserved
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched/signal.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/mailbox/simaai-mailbox.h>

#define SIMA_MBOX_MSG_INDEX            GENMASK(14,0)
#define SIMA_MBOX_MSG_SIZE             GENMASK(29,15)

/* Tx timeout in milliseconds */
#define SIMA_TX_TOUT		500

/* IPC descriptors FIFO depth */
#define SIMA_DESC_FIFO_DEPTH	16

struct sima_mbox_client {
	const char *name;
	struct device *dev;
	struct mbox_client client;
	struct mbox_chan *channel;
	const char *channel_name;
	/* Tx */
	struct mutex write_lock;
	struct simaai_mbmsg *msgs;
	DECLARE_BITMAP(msgs_bmap, MBOX_TX_QUEUE_LEN);
	wait_queue_head_t tx_wq;
	/* Rx */
	u32 rx_ring_size;
	struct kfifo rx_fifo;
	spinlock_t rx_fifo_lock;
	wait_queue_head_t rx_wq;
	/* Character device stuff */
	struct cdev cdev;
	dev_t dev_no;
	struct class *dev_class;
	struct mutex dev_lock;
	int dev_open_count;
	int max_users;
	void __iomem *txbuf;
	void __iomem *rxbuf;
};

static int init_device(struct sima_mbox_client *mbox)
{
	kfifo_reset(&mbox->rx_fifo);

	mbox->channel = mbox_request_channel_byname(&mbox->client, 
			mbox->channel_name);
	if (IS_ERR(mbox->channel)) {
		dev_err(mbox->dev, "Failed to request channel %s\n",
				mbox->channel_name);
		mbox->channel = NULL;
		return -EINVAL;
	}

	return 0;
}

static void release_dev(struct sima_mbox_client *mbox)
{
	if (mbox->channel) {
		mbox_free_channel(mbox->channel);
		mbox->channel = NULL;
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
	if (!ret) {
		if ((mbox->dev_open_count < mbox->max_users) || (mbox->max_users == 0))
			mbox->dev_open_count++;
		else
			ret = -EBUSY;
	}
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

static int tx_msg_available(struct sima_mbox_client *mbox, int *msgid)
{
	int ret;
	
	ret = mutex_lock_interruptible(&mbox->write_lock);
	if (ret) {
		dev_err(mbox->dev, "Failed to lock mutex\n");
		return ret;
	}
	
	*msgid = bitmap_find_free_region(mbox->msgs_bmap, MBOX_TX_QUEUE_LEN, 0);
	mutex_unlock(&mbox->write_lock);

	return *msgid >= 0;
}

static ssize_t mbox_dev_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *fpos)
{
	struct sima_mbox_client *mbox = filp->private_data;
	ssize_t len = count > SIMAAI_MAX_DATA_SIZE ? SIMAAI_MAX_DATA_SIZE : count;
	struct simaai_mbmsg *msg;
	int ret, msgid;
	u32 data;

	if (!tx_msg_available(mbox, &msgid) && (filp->f_flags & O_NONBLOCK)) {
		dev_err(mbox->dev, "%d: Returning %d\n", __LINE__, -EAGAIN);
		return -EAGAIN;
	}

	if (msgid < 0) {
		ret = wait_event_interruptible(mbox->tx_wq, !tx_msg_available(mbox, &msgid));
		if (ret < 0) {
			dev_err(mbox->dev, "Failed to find available message placeholder\n");
			return ret;
		}
	}

	msg = (struct simaai_mbmsg *)((void *)(mbox->msgs) + msgid * SIMAAI_MAX_MSG_SIZE);
	ret = copy_from_user(msg->data, buf, len);
	msg->len = len;

	ret = mutex_lock_interruptible(&mbox->write_lock);

	memcpy_toio(mbox->txbuf, msg->data, msg->len);
	/* populate index(always 0) and size for mailbox msg register */
	data = FIELD_PREP(SIMA_MBOX_MSG_INDEX, 0) | FIELD_PREP(SIMA_MBOX_MSG_SIZE, len);

	ret = mbox_send_message(mbox->channel, &data);
	if (ret < 0) {
		dev_err(mbox->dev, "Failed to send message via mailbox\n");
		len = (ssize_t)ret;
	}

	mutex_unlock(&mbox->write_lock);

	ret = mutex_lock_interruptible(&mbox->write_lock);
	bitmap_release_region(mbox->msgs_bmap, msgid, 0);
	mutex_unlock(&mbox->write_lock);
	wake_up_all(&mbox->tx_wq);

	if (ret < 0) {
		dev_err(mbox->dev, "Failed to lock mutex\n");
		len = (ssize_t)ret;
	}

	return len;
}

static ssize_t mbox_dev_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *fpos)
{
	struct inode *inode = file_inode(filp);
	struct sima_mbox_client *mbox = filp->private_data;
	char msg_raw[SIMAAI_MAX_MSG_SIZE];
	struct simaai_mbmsg *msg;
	int ret;

	while (!kfifo_out_spinlocked(&mbox->rx_fifo, msg_raw, sizeof(msg_raw),
				     &mbox->rx_fifo_lock)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(mbox->rx_wq,
					       !kfifo_is_empty(&mbox->rx_fifo));
		if (ret)
			return ret;
	}

	msg = (struct simaai_mbmsg *)msg_raw;
	if (count > msg->len)
		count = msg->len;
	ret = copy_to_user(buf, msg->data, count);
	if (ret) {
		ret = -EFAULT;
	} else {
		inode->i_atime = current_time(inode);
		ret = count;
	}

	return ret;
}

static __poll_t mbox_dev_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct sima_mbox_client *mbox = filp->private_data;
	__poll_t req_events = poll_requested_events(wait);
	__poll_t ret = 0;

	if (req_events & (EPOLLIN | EPOLLRDNORM))
		poll_wait(filp, &mbox->rx_wq, wait);

	if (req_events & (EPOLLOUT | EPOLLWRNORM))
		poll_wait(filp, &mbox->tx_wq, wait);

	if (!kfifo_is_empty(&mbox->rx_fifo))
		ret |= EPOLLIN | EPOLLRDNORM;

	if (!bitmap_full(mbox->msgs_bmap, MBOX_TX_QUEUE_LEN))
		ret |= EPOLLOUT | EPOLLWRNORM;

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
	u32 data = *(u32 *) message;
	char msg_raw[SIMAAI_MAX_MSG_SIZE];
	struct simaai_mbmsg *msg = (struct simaai_mbmsg*)msg_raw;
	int index, size;

	index = FIELD_GET(SIMA_MBOX_MSG_INDEX, data);
	size = FIELD_GET(SIMA_MBOX_MSG_SIZE, data);

	if ((index + size) > SIMAAI_MAX_DATA_SIZE)
		size = SIMAAI_MAX_DATA_SIZE - index;

	msg->len = size;

	spin_lock_irqsave(&mbox->rx_fifo_lock, flags);
	memcpy_fromio(msg->data, mbox->rxbuf + index, size);
	kfifo_in(&mbox->rx_fifo, msg_raw, SIMAAI_MAX_MSG_SIZE);
	spin_unlock_irqrestore(&mbox->rx_fifo_lock, flags);

	wake_up_interruptible(&mbox->rx_wq);
}

static int sima_mbox_client_probe(struct platform_device *pdev)
{
	struct sima_mbox_client *mbox;
	int ret;
	u32 tx_tout = SIMA_TX_TOUT;
	u32 fifo_depth = SIMA_DESC_FIFO_DEPTH;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->msgs = devm_kzalloc(&pdev->dev, SIMAAI_MAX_MSG_SIZE *
				  MBOX_TX_QUEUE_LEN, GFP_KERNEL);
	if (!mbox->msgs)
		return -ENOMEM;
	mbox->dev = &pdev->dev;

	mbox->txbuf = devm_platform_ioremap_resource_byname(pdev, "txbuf");
	if (IS_ERR(mbox->txbuf))
		return PTR_ERR(mbox->txbuf);

	mbox->rxbuf = devm_platform_ioremap_resource_byname(pdev, "rxbuf");
	if (IS_ERR(mbox->rxbuf))
		return PTR_ERR(mbox->rxbuf);

	ret = of_property_read_string(pdev->dev.of_node, "simaai,dev-name",
				      &mbox->name);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot obtain simaai,dev-name property\n");
		return -ENXIO;
	}

	ret = of_property_read_string(pdev->dev.of_node, "simaai,channel",
				      &mbox->channel_name);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot obtain simaai,channel property\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "simaai,tx-tout-ms",
				   &tx_tout);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "cannot obtain simaai,tx-tout-ms property\n");
	}

	ret = of_property_read_u32(pdev->dev.of_node, "simaai,rx-fifo-depth",
				   &fifo_depth);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "cannot obtain simaai,rx-fifo-depth property\n");
	}

	ret = of_property_read_u32(pdev->dev.of_node, "simaai,max-users",
				   &mbox->max_users);

	ret = kfifo_alloc(&mbox->rx_fifo, fifo_depth * SIMAAI_MAX_MSG_SIZE, GFP_KERNEL);
	if (ret) {
		dev_err(&pdev->dev, "cannot allocate Rx FIFO\n");
		return ret;
	}

	mbox->client.dev = &pdev->dev;
	mbox->client.rx_callback = message_from_remote;
	mbox->client.tx_done = NULL;
	mbox->client.tx_block = true;
	mbox->client.knows_txdone = false;
	mbox->client.tx_tout = tx_tout;

	mutex_init(&mbox->dev_lock);
	mutex_init(&mbox->write_lock);
	spin_lock_init(&mbox->rx_fifo_lock);
	init_waitqueue_head(&mbox->rx_wq);
	init_waitqueue_head(&mbox->tx_wq);

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
	{ .compatible = "simaai,mailbox-client" },
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
