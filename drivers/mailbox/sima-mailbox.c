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

/* Mailbox controller registers area */
#define SIMA_MBOX_CFG_OFFSET		0x00
#define SIMA_MBOX_MSG_OFFSET		0x04
#define SIMA_MBOX_STAT_OFFSET		0x08

/* Mailbox Configuration Register bit definitions */
#define SIMA_MBOX_CFG_EN		BIT(0)
#define SIMA_MBOX_CFG_RX_INT_EN		BIT(1)
#define SIMA_MBOX_CFG_TX_INT_EN		BIT(2)
#define SIMA_MBOX_CFG_TX_INT_PENDING	BIT(3)
#define SIMA_MBOX_CFG_RX_INT_HOLD	BIT(4)

/* Mailbox Message Register bit definitions */
#define SIMA_MBOX_MSG_RX_INT_SET	BIT(0)

/* Mailbox Status Register bit definitions */
#define SIMA_MBOX_STAT_RX_INT_PENDING	BIT(0)

#define SIMA_MBOX_POLLING_MS		5

struct sima_mbox {
	const char *name;
	bool is_sender;
	bool intr_mode;
	int irq;
	void __iomem *mbox_base;
	struct device *dev;
	struct mbox_controller controller;

	/* For RX polling mode */
	struct timer_list rxpoll_timer;
	struct mbox_chan *chan;
};

static inline struct sima_mbox *mbox_chan_to_sima_mbox(struct mbox_chan *chan)
{
	if (!chan || !chan->con_priv)
		return NULL;

	return (struct sima_mbox *)chan->con_priv;
}

static inline bool sima_mbox_full(struct sima_mbox *mbox)
{
	u32 status = readl_relaxed(mbox->mbox_base + SIMA_MBOX_STAT_OFFSET);

	return (status & SIMA_MBOX_STAT_RX_INT_PENDING) ? true : false;
}

static irqreturn_t sima_mbox_tx_interrupt(int irq, void *ctx)
{
	struct mbox_chan *chan = (struct mbox_chan *)ctx;
	struct sima_mbox *mbox;
	u32 cfg;

	if (unlikely(!chan)) {
		pr_err("%s: invalid channel pointer\n", __func__);
		return IRQ_NONE;
	}
	mbox = mbox_chan_to_sima_mbox(chan);

	/* Sender MUST clear pending interrupt in the Configuration Register */
	cfg = readl_relaxed(mbox->mbox_base + SIMA_MBOX_CFG_OFFSET);
	cfg &= ~SIMA_MBOX_CFG_TX_INT_PENDING;
	writel_relaxed(cfg, mbox->mbox_base + SIMA_MBOX_CFG_OFFSET);

	mbox_chan_txdone(chan, 0);

	return IRQ_HANDLED;
}

static bool sima_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);

	/* Return false if mailbox is full */
	return !sima_mbox_full(mbox);
}

static int sima_mbox_tx_data(struct mbox_chan *chan, void *data)
{
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);
	u32 msg;

	if (!mbox || !data)
		return -EINVAL;

	if (!mbox->is_sender) {
		dev_warn(mbox->dev,
			 "failed to send. This is receiver mailbox.\n");
		return -EINVAL;
	}

	if (sima_mbox_full(mbox))
		return -EBUSY;

	/* Message payload is placed in MSG[31:2] bits */
	msg = (*(u32 *)data << 2) | SIMA_MBOX_MSG_RX_INT_SET;
	writel_relaxed(msg, mbox->mbox_base + SIMA_MBOX_MSG_OFFSET);

	return 0;
}

static int sima_mbox_tx_startup(struct mbox_chan *chan)
{
	int ret;
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);
	u32 cfg = SIMA_MBOX_CFG_EN | SIMA_MBOX_CFG_RX_INT_EN |
			SIMA_MBOX_CFG_RX_INT_HOLD;

	writel_relaxed(0, mbox->mbox_base + SIMA_MBOX_MSG_OFFSET);

	if (mbox->intr_mode) {
		ret = request_irq(mbox->irq, sima_mbox_tx_interrupt, 0,
				  mbox->name, chan);
		if (unlikely(ret)) {
			dev_err(mbox->dev,
				"failed to register mailbox interrupt: %d\n",
				ret);
			return ret;
		}
		cfg |= SIMA_MBOX_CFG_TX_INT_EN;
	}

	writel_relaxed(cfg, mbox->mbox_base + SIMA_MBOX_CFG_OFFSET);

	return 0;
}

static inline void sima_mbox_rx_data(struct mbox_chan *chan)
{
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);
	u32 data;

	if (sima_mbox_full(mbox)) {
		/* Message payload is placed in MSG[31:2] bits */
		data = readl_relaxed(mbox->mbox_base + SIMA_MBOX_MSG_OFFSET);
		data >>= 2;
		mbox_chan_received_data(chan, (void *)&data);

		/* Deassert Rx interrupt and notify a sender */
		writel_relaxed(0, mbox->mbox_base + SIMA_MBOX_MSG_OFFSET);
	}
}

static irqreturn_t sima_mbox_rx_interrupt(int irq, void *ctx)
{
	struct mbox_chan *chan = (struct mbox_chan *)ctx;

	if (unlikely(!chan)) {
		pr_err("%s: invalid channel pointer\n", __func__);
		return IRQ_NONE;
	}

	sima_mbox_rx_data(chan);

	return IRQ_HANDLED;
}

static void sima_mbox_rx_poll(struct timer_list *t)
{
	struct sima_mbox *mbox = from_timer(mbox, t, rxpoll_timer);

	sima_mbox_rx_data(mbox->chan);

	mod_timer(&mbox->rxpoll_timer,
		  jiffies + msecs_to_jiffies(SIMA_MBOX_POLLING_MS));
}

static bool sima_mbox_rx_peek_data(struct mbox_chan *chan)
{
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);

	return sima_mbox_full(mbox);
}

static int sima_mbox_rx_startup(struct mbox_chan *chan)
{
	int ret;
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);

	if (mbox->intr_mode) {
		ret = request_irq(mbox->irq, sima_mbox_rx_interrupt, 0,
				  mbox->name, chan);
		if (unlikely(ret)) {
			dev_err(mbox->dev,
				"failed to register mailbox interrupt: %d\n",
				ret);
			mbox->intr_mode = false;
			goto polling; /* use polling if failed */
		}

		return 0;
	}

polling:
	/* Setup polling timer */
	mbox->chan = chan;
	timer_setup(&mbox->rxpoll_timer, sima_mbox_rx_poll, 0);
	mod_timer(&mbox->rxpoll_timer,
		  jiffies + msecs_to_jiffies(SIMA_MBOX_POLLING_MS));

	return 0;
}

static int sima_mbox_startup(struct mbox_chan *chan)
{
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);
	int ret = 0;

	if (!mbox)
		return -EINVAL;

	if (mbox->is_sender)
		ret = sima_mbox_tx_startup(chan);
	else
		ret = sima_mbox_rx_startup(chan);

	return ret;
}

static void sima_mbox_shutdown(struct mbox_chan *chan)
{
	struct sima_mbox *mbox = mbox_chan_to_sima_mbox(chan);

	/* Sender is responsible for disabling mailbox */
	if (mbox->is_sender)
		writel_relaxed(0, mbox->mbox_base + SIMA_MBOX_CFG_OFFSET);

	if (mbox->intr_mode)
		free_irq(mbox->irq, chan);
	else if (!mbox->is_sender)
		del_timer_sync(&mbox->rxpoll_timer);
}

static const struct mbox_chan_ops sima_mbox_ops = {
	.send_data = sima_mbox_tx_data,
	.startup = sima_mbox_startup,
	.shutdown = sima_mbox_shutdown,
	.last_tx_done = sima_mbox_last_tx_done,
	.peek_data = sima_mbox_rx_peek_data,
};

static int sima_mbox_probe(struct platform_device *pdev)
{
	struct sima_mbox *mbox;
	struct resource	*regs;
	struct mbox_chan *chans;
	int ret;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	/* Allocated one channel */
	chans = devm_kzalloc(&pdev->dev, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	mbox->mbox_base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(mbox->mbox_base))
		return PTR_ERR(mbox->mbox_base);

	/* Check is it a sender or receiver? */
	mbox->is_sender =
		of_property_read_bool(pdev->dev.of_node, "sima,mbox-sender");

	mbox->irq = platform_get_irq(pdev, 0);
	if (mbox->irq >= 0)
		mbox->intr_mode = true;

	mbox->dev = &pdev->dev;
	mbox->name = pdev->name;

	/* Hardware supports only one channel. */
	chans[0].con_priv = mbox;
	mbox->controller.dev = mbox->dev;
	mbox->controller.num_chans = 1;
	mbox->controller.chans = chans;
	mbox->controller.ops = &sima_mbox_ops;

	if (mbox->is_sender) {
		if (mbox->intr_mode) {
			mbox->controller.txdone_irq = true;
		} else {
			mbox->controller.txdone_poll = true;
			mbox->controller.txpoll_period = SIMA_MBOX_POLLING_MS;
		}
	}

	ret = devm_mbox_controller_register(&pdev->dev, &mbox->controller);
	if (ret) {
		dev_err(&pdev->dev, "Register mailbox failed\n");
		goto err;
	}

	platform_set_drvdata(pdev, mbox);

	dev_info(&pdev->dev, "Mailbox registered\n");
err:
	return ret;
}

static const struct of_device_id sima_mbox_match[] = {
	{ .compatible = "sima,mailbox-1.0" },
	{},
};

MODULE_DEVICE_TABLE(of, sima_mbox_match);

static struct platform_driver sima_mbox_driver = {
	.probe	= sima_mbox_probe,
	.driver	= {
		.name	= "sima-mailbox",
		.of_match_table	= sima_mbox_match,
	},
};

module_platform_driver(sima_mbox_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SiMa.ai mailbox specific functions");
MODULE_AUTHOR("SiMa.ai");
MODULE_ALIAS("platform:sima-mailbox");
