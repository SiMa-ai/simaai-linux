// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiMa.ai HPI mailbox client - /dev/simaai-hpi.
 *
 * Copyright (c) 2026 SiMa ai
 */

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/mailbox_client.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/simaai-hpi.h>

#define HPI_RSP_FIFO_DEPTH	4
#define HPI_XFER_TIMEOUT_MS	5000

/* kernel<->tRoot shmem setup handshake - matches troot hpi.h, not a UAPI */
#define SIMA_HPI_MSG_SHMEM_MAP		0x10000012
#define SIMA_HPI_MSG_SHMEM_MAP_ACK	0x10000013
#define SIMA_HPI_MSG_SHMEM_UNMAP	0x10000014
#define SIMA_HPI_MSG_SHMEM_UNMAP_ACK	0x10000015

#define HPI_STU_ID			4		/* tRoot STU segment / 0x80000000 window */
#define HPI_SHMEM_SIZE			0x2200000	/* 34 MiB */

struct hpi_client {
	struct device		*dev;
	struct mbox_client	cl;
	struct mbox_chan	*chan;
	struct miscdevice	misc;

	/* shared DDR - guarded by mem_lock, alive only while mem_users > 0 */
	struct mutex		mem_lock;
	unsigned int		mem_users;
	void			*shmem_cpu;
	dma_addr_t		shmem_dma;
	phys_addr_t		shmem_base;
	size_t			shmem_size;
	bool			crosses_window;
	bool			dynamic;	/* true: dma+SHMEM_MAP  false: fixed region */

	phys_addr_t		fixed_base;
	size_t			fixed_size;

	/* one in-flight transaction at a time */
	struct mutex		op_lock;
	spinlock_t		rsp_lock;
	wait_queue_head_t	rsp_wq;
	DECLARE_KFIFO(rsp_fifo, struct simaai_hpi_mbox_msg, HPI_RSP_FIFO_DEPTH);
};

/* per-open context; filp->private_data points here after open() */
struct hpi_file {
	struct hpi_client	*client;
	bool			mem_held;	/* this fd acquired shmem via GET_MEM */
};

static struct hpi_client *g_hpi_client;

static void hpi_client_rx(struct mbox_client *cl, void *mssg)
{
	struct hpi_client *client = container_of(cl, struct hpi_client, cl);

	if (!kfifo_in_spinlocked(&client->rsp_fifo, mssg, 1, &client->rsp_lock))
		dev_warn_ratelimited(client->dev, "response FIFO full, dropping\n");
	wake_up_interruptible(&client->rsp_wq);
}

static void hpi_client_tx_done(struct mbox_client *cl, void *mssg, int status)
{
	kfree(mssg);
}

static int hpi_post(struct hpi_client *client, u32 p0, u32 p1)
{
	struct simaai_hpi_mbox_msg *msg;
	int ret;

	msg = kmalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->p0 = p0;
	msg->p1 = p1;

	ret = mbox_send_message(client->chan, msg);
	if (ret < 0)
		kfree(msg);
	return ret < 0 ? ret : 0;
}

static int hpi_wait(struct hpi_client *client, struct simaai_hpi_mbox_msg *reply)
{
	long left = wait_event_interruptible_timeout(client->rsp_wq,
			!kfifo_is_empty(&client->rsp_fifo),
			msecs_to_jiffies(HPI_XFER_TIMEOUT_MS));

	if (left == 0)
		return -ETIMEDOUT;
	if (left < 0)
		return left;
	return kfifo_out_spinlocked(&client->rsp_fifo, reply, 1, &client->rsp_lock) == 1 ?
	       0 : -EIO;
}

/* raw transaction: reset, post, wait for one reply (blocking, process ctx) */
static int hpi_xfer(struct hpi_client *client, u32 p0, u32 p1,
		    struct simaai_hpi_mbox_msg *reply)
{
	unsigned long flags;
	int ret;

	mutex_lock(&client->op_lock);
	spin_lock_irqsave(&client->rsp_lock, flags);
	kfifo_reset(&client->rsp_fifo);
	spin_unlock_irqrestore(&client->rsp_lock, flags);

	ret = hpi_post(client, p0, p1);
	if (!ret)
		ret = hpi_wait(client, reply);
	mutex_unlock(&client->op_lock);
	return ret;
}

static int hpi_stu_map(struct hpi_client *client, u32 id, phys_addr_t win)
{
	struct simaai_hpi_mbox_msg reply;
	u32 p1 = ((u32)(win >> 8) & ~0xfu) | id;
	int ret = hpi_xfer(client, SIMA_HPI_MSG_SHMEM_MAP, p1, &reply);

	if (ret)
		return ret;
	return reply.p0 == SIMA_HPI_MSG_SHMEM_MAP_ACK ? 0 : -EIO;
}

static void hpi_stu_unmap(struct hpi_client *client, u32 id)
{
	struct simaai_hpi_mbox_msg reply;

	hpi_xfer(client, SIMA_HPI_MSG_SHMEM_UNMAP, id, &reply);
}

/* dynamic first, fall back to the fixed region for an old tRoot, holds mem_lock */
static int hpi_acquire(struct hpi_client *client)
{
	int ret;

	client->shmem_cpu = dma_alloc_coherent(client->dev, HPI_SHMEM_SIZE,
					       &client->shmem_dma, GFP_KERNEL);
	if (client->shmem_cpu) {
		client->shmem_base = client->shmem_dma;
		client->shmem_size = HPI_SHMEM_SIZE;
		/* If dma memory is in between two 512MB windows set 4th and 5th STUs */
		client->crosses_window =
			(client->shmem_base & (SZ_512M - 1)) + HPI_SHMEM_SIZE > SZ_512M;

		ret = hpi_stu_map(client, HPI_STU_ID, client->shmem_base);
		if (!ret) {
			if (client->crosses_window) {
				ret = hpi_stu_map(client, HPI_STU_ID + 1,
						  client->shmem_base + SZ_512M);
				if (ret) {
					hpi_stu_unmap(client, HPI_STU_ID);
					dma_free_coherent(client->dev, HPI_SHMEM_SIZE,
							  client->shmem_cpu, client->shmem_dma);
					client->shmem_cpu = NULL;
					client->shmem_size = 0;
					return ret;
				}
			}
			client->dynamic = true;
			dev_dbg(client->dev, "shmem dma=%pad%s\n", &client->shmem_dma,
				client->crosses_window ? " (2 windows)" : "");
			return 0;
		}

		/* SHMEM_MAP rejected = old tRoot,so drop buffer and fall back */
		dev_info(client->dev, "SHMEM_MAP unsupported, using fixed region\n");
		dma_free_coherent(client->dev, HPI_SHMEM_SIZE,
				  client->shmem_cpu, client->shmem_dma);
		client->shmem_cpu = NULL;
	}

	if (!client->fixed_size) {
		dev_err(client->dev, "no SHMEM_MAP and no memory-region\n");
		return -ENODEV;
	}

	/* fallback: fixed reserved region the old tRoot expects */
	client->shmem_base = client->fixed_base;
	client->shmem_size = client->fixed_size;
	client->dynamic = false;

	dev_info(client->dev, "shmem fixed %pa (old tRoot)\n", &client->shmem_base);
	return 0;
}


static void hpi_release(struct hpi_client *client)
{
	if (client->dynamic) {
		hpi_stu_unmap(client, HPI_STU_ID);
		if (client->crosses_window)
			hpi_stu_unmap(client, HPI_STU_ID + 1);
		dma_free_coherent(client->dev, client->shmem_size,
				  client->shmem_cpu, client->shmem_dma);
	}

	client->shmem_cpu = NULL;
	client->shmem_size = 0;
}

/* refcounted acquire/release - shared by all openers */
static int hpi_mem_get(struct hpi_client *client)
{
	int ret = 0;

	if (mutex_lock_interruptible(&client->mem_lock))
		return -ERESTARTSYS;
	if (!client->mem_users)
		ret = hpi_acquire(client);
	if (!ret)
		client->mem_users++;
	mutex_unlock(&client->mem_lock);
	return ret;
}

static void hpi_mem_put(struct hpi_client *client)
{
	mutex_lock(&client->mem_lock);
	if (!WARN_ON(!client->mem_users) && !--client->mem_users)
		hpi_release(client);
	mutex_unlock(&client->mem_lock);
}

static int hpi_do_send(struct hpi_client *client, void __user *uarg)
{
	struct simaai_hpi_msg umsg;

	if (copy_from_user(&umsg, uarg, sizeof(umsg)))
		return -EFAULT;
	return hpi_post(client, umsg.p0, umsg.p1);
}

static int hpi_do_recv(struct hpi_client *client, void __user *uarg)
{
	struct simaai_hpi_mbox_msg reply;
	struct simaai_hpi_msg umsg;
	int ret = hpi_wait(client, &reply);

	if (ret)
		return ret;
	umsg.p0 = reply.p0;
	umsg.p1 = reply.p1;
	return copy_to_user(uarg, &umsg, sizeof(umsg)) ? -EFAULT : 0;
}

static int hpi_do_xfer(struct hpi_client *client, void __user *uarg)
{
	struct simaai_hpi_mbox_msg reply;
	struct simaai_hpi_msg umsg;
	int ret;

	if (copy_from_user(&umsg, uarg, sizeof(umsg)))
		return -EFAULT;

	ret = hpi_xfer(client, umsg.p0, umsg.p1, &reply);
	if (ret)
		return ret;

	umsg.p0 = reply.p0;
	umsg.p1 = reply.p1;
	return copy_to_user(uarg, &umsg, sizeof(umsg)) ? -EFAULT : 0;
}

/* first GET_MEM on an fd allocates + SHMEM_MAPs; freed on close */
static int hpi_do_get_mem(struct hpi_file *priv, void __user *uarg)
{
	struct hpi_client *client = priv->client;
	struct simaai_hpi_mem mem;
	int ret;

	if (!priv->mem_held) {
		ret = hpi_mem_get(client);
		if (ret)
			return ret;
		priv->mem_held = true;
	}
	mem.phys = client->shmem_base;
	mem.size = client->shmem_size;
	return copy_to_user(uarg, &mem, sizeof(mem)) ? -EFAULT : 0;
}

/**
 * simaai_hpi_xfer() - send an HPI message (optionally with a payload) and wait
 * @req:     message words to send (p0/p1)
 * @rsp:     filled with the reply on success
 * @payload: optional data to stage into shmem,use NULL to skip
 * @len:     payload length (ignored when @payload is NULL)
 *
 * Memory is acquired ONLY when a payload is supplied. A plain request/reply
 * caller passes payload=NULL and never touches the shmem refcount.
 * Blocking, process context only.
 */

int simaai_hpi_xfer(const struct simaai_hpi_msg *req, struct simaai_hpi_msg *rsp,
		    const void *payload, size_t len)
{
	struct hpi_client *client = g_hpi_client;
	struct simaai_hpi_mbox_msg reply;
	bool need_mem = payload && len;
	int ret;

	if (!client)
		return -EPROBE_DEFER;
	if (!req || !rsp)
		return -EINVAL;

	if (need_mem) {
		ret = hpi_mem_get(client);		/* acquire only for payload */
		if (ret)
			return ret;
		if (!client->shmem_cpu || len > client->shmem_size) {
			ret = -ENXIO;		/* no kernel VA (fixed fallback) or too big */
			goto put;
		}
		/* TODO: Need to fix if the buffer is huge */
		memcpy(client->shmem_cpu, payload, len);
	}

	ret = hpi_xfer(client, req->p0, req->p1, &reply);
	if (!ret) {
		rsp->p0 = reply.p0;
		rsp->p1 = reply.p1;
	}
put:
	if (need_mem)
		hpi_mem_put(client);
	return ret;
}
EXPORT_SYMBOL_GPL(simaai_hpi_xfer);

static long hpi_client_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct hpi_file *priv = filp->private_data;
	struct hpi_client *client = priv->client;
	void __user *uarg = (void __user *)arg;

	switch (cmd) {
	case SIMAAI_HPI_SEND:	return hpi_do_send(client, uarg);
	case SIMAAI_HPI_RECV:	return hpi_do_recv(client, uarg);
	case SIMAAI_HPI_XFER:	return hpi_do_xfer(client, uarg);
	case SIMAAI_HPI_GET_MEM:	return hpi_do_get_mem(priv, uarg);
	default:		return -ENOTTY;
	}
}

static int hpi_client_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct hpi_file *priv = filp->private_data;
	struct hpi_client *client = priv->client;
	size_t size = vma->vm_end - vma->vm_start;
	phys_addr_t phys = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;

	if (!priv->mem_held)		/* must GET_MEM before mmap */
		return -EINVAL;
	if (!client->shmem_size || phys < client->shmem_base ||
	    phys - client->shmem_base > client->shmem_size - size)
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);	/* Normal-NC */
	return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       size, vma->vm_page_prot);
}

static int hpi_client_open(struct inode *inode, struct file *filp)
{
	struct hpi_client *client = container_of(filp->private_data,
						 struct hpi_client, misc);
	struct hpi_file *priv = kzalloc(sizeof(*priv), GFP_KERNEL);

	if (!priv)
		return -ENOMEM;
	priv->client = client;
	filp->private_data = priv;	/* replace misc ptr with per-fd ctx */
	return 0;
}

static int hpi_client_release(struct inode *inode, struct file *filp)
{
	struct hpi_file *priv = filp->private_data;

	if (priv->mem_held)
		hpi_mem_put(priv->client);
	kfree(priv);
	return 0;
}

static const struct file_operations hpi_client_fops = {
	.owner		= THIS_MODULE,
	.open		= hpi_client_open,
	.release	= hpi_client_release,
	.unlocked_ioctl	= hpi_client_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.mmap		= hpi_client_mmap,
};

/*
 * HACK: Fixed-region fallback - used only when tRoot rejects SHMEM_MAP (old
 * firmware). Remove this function together with the fixed_base/
 * fixed_size fields, the ->dynamic branches in hpi_acquire()/hpi_release(),
 * and the DT "simaai,fixed-shmem", once both tRoot and this driver are on the
 * dynamic (SHMEM_MAP) scheme.
 */

static int hpi_init_fixed(struct hpi_client *client)
{
	u64 range[2];

	if (of_property_read_u64_array(client->dev->of_node, "simaai,fixed-shmem", range, 2))
		return 0;

	client->fixed_base = range[0];
	client->fixed_size = range[1];
	dev_info(client->dev, "fallback region %pa (%zu bytes)\n",
		 &client->fixed_base, client->fixed_size);
	return 0;
}

static int hpi_client_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hpi_client *client;
	int ret;

	client = devm_kzalloc(dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->dev = dev;
	mutex_init(&client->mem_lock);
	mutex_init(&client->op_lock);
	spin_lock_init(&client->rsp_lock);
	init_waitqueue_head(&client->rsp_wq);
	INIT_KFIFO(client->rsp_fifo);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return dev_err_probe(dev, ret, "no 40-bit DMA\n");

	ret = hpi_init_fixed(client);
	if (ret)
		return dev_err_probe(dev, ret, "bad memory-region\n");

	client->cl.dev = dev;
	client->cl.rx_callback = hpi_client_rx;
	client->cl.tx_done = hpi_client_tx_done;
	client->cl.tx_block = false;
	client->cl.knows_txdone = false;

	client->chan = mbox_request_channel(&client->cl, 0);
	if (IS_ERR(client->chan))
		return dev_err_probe(dev, PTR_ERR(client->chan), "no mailbox channel\n");

	client->misc.minor = MISC_DYNAMIC_MINOR;
	client->misc.name = "simaai-hpi";
	client->misc.fops = &hpi_client_fops;
	client->misc.parent = dev;

	ret = misc_register(&client->misc);
	if (ret) {
		dev_err_probe(dev, ret, "misc_register failed\n");
		goto err_chan;
	}

	platform_set_drvdata(pdev, client);
	g_hpi_client = client;
	return 0;

err_chan:
	mbox_free_channel(client->chan);
	return ret;
}

static void hpi_client_remove(struct platform_device *pdev)
{
	struct hpi_client *client = platform_get_drvdata(pdev);

	g_hpi_client = NULL;
	misc_deregister(&client->misc);

	/* defensive: last close should have released it already */
	mutex_lock(&client->mem_lock);
	if (client->mem_users) {
		dev_warn(client->dev, "removing with %u user(s)\n", client->mem_users);
		hpi_release(client);
		client->mem_users = 0;
	}
	mutex_unlock(&client->mem_lock);

	mbox_free_channel(client->chan);
}

static const struct of_device_id hpi_client_of_match[] = {
	{ .compatible = "simaai,modalix-hpi-client" },
	{ }
};
MODULE_DEVICE_TABLE(of, hpi_client_of_match);

static struct platform_driver hpi_client_driver = {
	.probe	= hpi_client_probe,
	.remove	= hpi_client_remove,
	.driver	= {
		.name		= "simaai-hpi-client",
		.of_match_table	= hpi_client_of_match,
	},
};
module_platform_driver(hpi_client_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SiMa.ai HPI mailbox transport");
MODULE_AUTHOR("SiMa ai");

