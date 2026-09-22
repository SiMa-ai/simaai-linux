// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright SiMa.ai (C) 2021. All rights reserved
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/simaai-hpi.h>

/* HPI register offsets (from DT reg[0]) */
#define HOST_IRQ_EN	0x010
#define HOST_IRQ_STAT	0x014
#define HOST_MB_P0	0x030
#define HOST_MB_P1	0x034
#define HOST_MB_CTRL	0x038
#define ARC_MB_P0	0x040
#define ARC_MB_P1	0x044
#define ARC_MB_OWN	0x04c
#define HPI_ERR_STAT	0x060

#define MB_MSG		BIT(16)		/* message from ARC waiting     */
#define MB_RTN		BIT(17)		/* host->ARC message taken (tx) */
#define GLOBAL_ERR	BIT(21)
#define GLOBAL_MSK	BIT(31)

#define HOST_MB_SEND	0x2		/* HOST_MB_CTRL: post + xfer ownership */
#define ARC_MB_RELEASE	0x1		/* ARC_MB_OWN: release back to host    */

#define HPI_TOKEN_P0	0xdeadbeef
#define HPI_TOKEN_P1	0xbeefdead
#define HPI_INIT_P0	0x11111111
#define HPI_INIT_P1	0x22222222

struct simaai_hpi {
	int irq;
	void __iomem *base;
	struct device *dev;

	struct mbox_controller controller;
	struct mbox_chan chan[1];
	spinlock_t lock;
	bool initialized;
	bool token;
	bool tx_staged;
	struct simaai_hpi_mbox_msg tx_msg;
};

/* Post the staged request. Caller holds hpi->lock. */
static void simaai_hpi_flush(struct simaai_hpi *hpi)
{
	writel(hpi->tx_msg.p0, hpi->base + HOST_MB_P0);
	writel(hpi->tx_msg.p1, hpi->base + HOST_MB_P1);
	writel(HOST_MB_SEND, hpi->base + HOST_MB_CTRL);
	hpi->token = false;
}

static irqreturn_t simaai_hpi_interrupt(int irq, void *dev_id)
{
	struct simaai_hpi *hpi = dev_id;
	struct mbox_chan *chan = &hpi->chan[0];
	struct simaai_hpi_mbox_msg rx;
	bool do_rx = false, do_txdone = false;
	unsigned long flags;
	u32 stat;

	stat = readl(hpi->base + HOST_IRQ_STAT);
	if (!(stat & (GLOBAL_ERR | MB_RTN | MB_MSG)))
		return IRQ_NONE;

	if (stat & GLOBAL_ERR) {
		u32 err = readl(hpi->base + HPI_ERR_STAT);

		writel(GLOBAL_ERR, hpi->base + HOST_IRQ_STAT);
		dev_err_ratelimited(hpi->dev, "HPI error, ERR_STAT=0x%08x\n", err);
		return IRQ_HANDLED;
	}

	if (stat & MB_RTN) {
		writel(MB_RTN, hpi->base + HOST_IRQ_STAT);
		do_txdone = true;
	}

	if (stat & MB_MSG) {
		u32 p0 = readl(hpi->base + ARC_MB_P0);
		u32 p1 = readl(hpi->base + ARC_MB_P1);

		writel(ARC_MB_RELEASE, hpi->base + ARC_MB_OWN);
		writel(MB_MSG, hpi->base + HOST_IRQ_STAT);

		spin_lock_irqsave(&hpi->lock, flags);
		if (p0 == HPI_TOKEN_P0 && p1 == HPI_TOKEN_P1) {
			/* token grants us the turn; carries no data */
			if (!hpi->initialized) {
				hpi->tx_msg.p0 = HPI_INIT_P0;
				hpi->tx_msg.p1 = HPI_INIT_P1;
				simaai_hpi_flush(hpi);
				hpi->initialized = true;
			} else {
				hpi->token = true;
				if (hpi->tx_staged) {
					simaai_hpi_flush(hpi);
					hpi->tx_staged = false;
				}
			}
		} else {
			rx.p0 = p0;
			rx.p1 = p1;
			do_rx = true;
		}
		spin_unlock_irqrestore(&hpi->lock, flags);
	}

	if (do_rx)
		mbox_chan_received_data(chan, &rx);
	if (do_txdone)
		mbox_chan_txdone(chan, 0);

	return IRQ_HANDLED;
}

static int simaai_hpi_send_data(struct mbox_chan *chan, void *data)
{
	struct simaai_hpi *hpi = chan->con_priv;
	struct simaai_hpi_mbox_msg *msg = data;
	unsigned long flags;

	spin_lock_irqsave(&hpi->lock, flags);
	hpi->tx_msg = *msg;
	if (hpi->token) {
		simaai_hpi_flush(hpi);
		hpi->tx_staged = false;
	} else {
		hpi->tx_staged = true;
	}
	spin_unlock_irqrestore(&hpi->lock, flags);

	return 0;
}

static int simaai_hpi_startup(struct mbox_chan *chan)
{
	struct simaai_hpi *hpi = chan->con_priv;
	unsigned long flags;
	u32 stat;

	spin_lock_irqsave(&hpi->lock, flags);
	hpi->token = false;
	hpi->tx_staged = false;

	writel(GLOBAL_MSK | MB_MSG | MB_RTN, hpi->base + HOST_IRQ_EN);

	stat = readl(hpi->base + HOST_IRQ_STAT);
	if (stat == 0) {
		hpi->token = true;
		hpi->initialized = true;
	} else {
		hpi->initialized = false;
	}
	spin_unlock_irqrestore(&hpi->lock, flags);

	return 0;
}

static void simaai_hpi_shutdown(struct mbox_chan *chan)
{
	struct simaai_hpi *hpi = chan->con_priv;

	writel(0, hpi->base + HOST_IRQ_EN);
}

static const struct mbox_chan_ops simaai_hpi_ops = {
	.send_data	= simaai_hpi_send_data,
	.startup	= simaai_hpi_startup,
	.shutdown	= simaai_hpi_shutdown,
};

static int simaai_hpi_probe(struct platform_device *pdev)
{
	struct simaai_hpi *hpi;
	struct resource	*regs;
	int ret;

	hpi = devm_kzalloc(&pdev->dev, sizeof(*hpi), GFP_KERNEL);
	if (!hpi)
		return -ENOMEM;

	hpi->dev = &pdev->dev;
	spin_lock_init(&hpi->lock);

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

	writel(0, hpi->base + HOST_IRQ_EN);	/* mask before hooking the line */

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

	hpi->chan[0].con_priv = hpi;
	hpi->controller.dev = hpi->dev;
	hpi->controller.ops = &simaai_hpi_ops;
	hpi->controller.chans = hpi->chan;
	hpi->controller.num_chans = ARRAY_SIZE(hpi->chan);
	hpi->controller.txdone_irq = true;

	platform_set_drvdata(pdev, hpi);

	ret = devm_mbox_controller_register(hpi->dev, &hpi->controller);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "HPI registered\n");

	return 0;
}

static const struct of_device_id simaai_hpi_match[] = {
	{ .compatible = "simaai,hpi-1.0" },
	{},
};

MODULE_DEVICE_TABLE(of, simaai_hpi_match);

static struct platform_driver simaai_hpi_driver = {
	.probe	= simaai_hpi_probe,
	.driver	= {
		.name	= "simaai-hpi",
		.of_match_table	= simaai_hpi_match,
	},
};

module_platform_driver(simaai_hpi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SiMa.ai HPI specific functions");
MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_ALIAS("platform:sima-hpi");

