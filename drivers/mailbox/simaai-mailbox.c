// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright SiMa.ai (C) 2021,2025. All rights reserved
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
#define SIMA_MBOX_MSG_DATA		GENMASK(31,2)

/* Mailbox Status Register bit definitions */
#define SIMA_MBOX_STAT_RX_INT_PENDING	BIT(0)

#define SIMA_MBOX_LP_CHAN		(0)
#define SIMA_MBOX_HP_CHAN		(1)
#define SIMA_MBOX_TX			(0)
#define SIMA_MBOX_RX			(1)

struct sima_resource {
	char name[32];
	int irq;
	void __iomem *regs;
};

struct sima_channel {
	struct device *dev;
	struct sima_resource res[2];
};

struct sima_mbox {
	struct mbox_controller controller;

	struct sima_channel channels[2];
	struct mbox_chan chans[2];
};

static inline struct sima_channel *mbox_chan_to_sima_chan(struct mbox_chan *chan)
{
	if (!chan || !chan->con_priv)
		return NULL;

	return (struct sima_channel *)chan->con_priv;
}

static inline bool sima_mbox_full(struct sima_channel *mbox, int res)
{
	u32 status = readl_relaxed(mbox->res[res].regs + SIMA_MBOX_STAT_OFFSET);

	return (status & SIMA_MBOX_STAT_RX_INT_PENDING) ? true : false;
}

static irqreturn_t sima_mbox_tx_interrupt(int irq, void *ctx)
{
	struct mbox_chan *chan = (struct mbox_chan *)ctx;
	struct sima_channel *mbox;
	u32 cfg;

	if (unlikely(!chan)) {
		pr_err("%s: invalid channel pointer\n", __func__);
		return IRQ_NONE;
	}
	mbox = mbox_chan_to_sima_chan(chan);

	/* Sender MUST clear pending interrupt in the Configuration Register */
	cfg = readl_relaxed(mbox->res[SIMA_MBOX_TX].regs + SIMA_MBOX_CFG_OFFSET);
	cfg &= ~SIMA_MBOX_CFG_TX_INT_PENDING;
	writel_relaxed(cfg, mbox->res[SIMA_MBOX_TX].regs + SIMA_MBOX_CFG_OFFSET);

	mbox_chan_txdone(chan, 0);

	return IRQ_HANDLED;
}

static bool sima_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct sima_channel *mbox = mbox_chan_to_sima_chan(chan);

	/* Return false if mailbox is full */
	return !sima_mbox_full(mbox, SIMA_MBOX_TX);
}

static int sima_mbox_tx_data(struct mbox_chan *chan, void *data)
{
	struct sima_channel *mbox = mbox_chan_to_sima_chan(chan);
	u32 *msg = (u32 *)data;
	u32 message; 

	if (!mbox || !data)
		return -EINVAL;

	if (sima_mbox_full(mbox, SIMA_MBOX_TX))
		return -EBUSY;
	
	/* Message payload is placed in MSG[31:2] bits */
	message = FIELD_PREP(SIMA_MBOX_MSG_DATA, *msg) | SIMA_MBOX_MSG_RX_INT_SET;
	writel_relaxed(message, mbox->res[SIMA_MBOX_TX].regs + SIMA_MBOX_MSG_OFFSET);

	return 0;
}

static int sima_mbox_tx_startup(struct mbox_chan *chan)
{
	int ret;
	struct sima_channel *mbox = mbox_chan_to_sima_chan(chan);
	u32 cfg = SIMA_MBOX_CFG_EN | SIMA_MBOX_CFG_RX_INT_EN |
			SIMA_MBOX_CFG_RX_INT_HOLD;

	writel_relaxed(0, mbox->res[SIMA_MBOX_TX].regs + SIMA_MBOX_MSG_OFFSET);

	cfg |= SIMA_MBOX_CFG_TX_INT_EN;

	writel_relaxed(cfg, mbox->res[SIMA_MBOX_TX].regs + SIMA_MBOX_CFG_OFFSET);

	return 0;
}

static inline void sima_mbox_rx_data(struct mbox_chan *chan)
{
	struct sima_channel *mbox = mbox_chan_to_sima_chan(chan);
	u32 data;

	if (sima_mbox_full(mbox, SIMA_MBOX_RX)) {
		data = readl_relaxed(mbox->res[SIMA_MBOX_RX].regs + SIMA_MBOX_MSG_OFFSET);
		data = FIELD_GET(SIMA_MBOX_MSG_DATA, data);

		mbox_chan_received_data(chan, &data);
		/* Deassert Rx interrupt and notify a sender */
		writel_relaxed(0, mbox->res[SIMA_MBOX_RX].regs + SIMA_MBOX_MSG_OFFSET);
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

static bool sima_mbox_rx_peek_data(struct mbox_chan *chan)
{
	struct sima_channel *mbox = mbox_chan_to_sima_chan(chan);

	return sima_mbox_full(mbox, SIMA_MBOX_RX);
}

static int sima_mbox_startup(struct mbox_chan *chan)
{
	struct sima_channel *mbox = mbox_chan_to_sima_chan(chan);
	int ret = 0;

	if (!mbox)
		return -EINVAL;

	ret = sima_mbox_tx_startup(chan);

	return ret;
}

static void sima_mbox_shutdown(struct mbox_chan *chan)
{
	struct sima_channel *mbox = mbox_chan_to_sima_chan(chan);

	/* Sender is responsible for disabling mailbox */
	writel_relaxed(0, mbox->res[SIMA_MBOX_TX].regs + SIMA_MBOX_CFG_OFFSET);

}

static const struct mbox_chan_ops sima_mbox_ops = {
	.send_data = sima_mbox_tx_data,
	.startup = sima_mbox_startup,
	.shutdown = sima_mbox_shutdown,
	.last_tx_done = sima_mbox_last_tx_done,
	.peek_data = sima_mbox_rx_peek_data,
};

static int sima_init_rs(struct platform_device *pdev, struct sima_resource *res,
		int chan, int dir, struct sima_mbox *mbox)
{
	struct device *dev = &pdev->dev;
	void __iomem * addr;
	int offset, ret;
	const char names[][5] = { "lptx", "lprx", "hptx", "hprx",};

	offset = (chan == SIMA_MBOX_LP_CHAN) ? 0 : 2;
	offset += (dir == SIMA_MBOX_TX) ? 0 : 1;

	addr = devm_platform_ioremap_resource(pdev, offset);
	if (IS_ERR(addr))
		return PTR_ERR(addr);
	res->regs = addr;

	res->irq = platform_get_irq(pdev, offset);
	if (res->irq < 0)
		return -ENOTSUPP;

	snprintf(res->name, sizeof(res->name) - 1, "%s_%s", pdev->name, names[offset]);

	if (dir == SIMA_MBOX_RX)
		ret = devm_request_irq(dev, res->irq, sima_mbox_rx_interrupt, 0, res->name, &mbox->chans[chan]);
	else
		ret = devm_request_irq(dev, res->irq, sima_mbox_tx_interrupt, 0, res->name, &mbox->chans[chan]);

	if (unlikely(ret)) {
		dev_err(dev,
			"failed to register mailbox interrupt: %d\n",
			ret);
		return -ENODEV;
	}

	return 0;
}

static int sima_mbox_probe(struct platform_device *pdev)
{
	struct sima_mbox *mbox;
	int i, ret;
	int rs[][2] = {
		{SIMA_MBOX_LP_CHAN, SIMA_MBOX_TX },
		{SIMA_MBOX_LP_CHAN, SIMA_MBOX_RX },
		{SIMA_MBOX_HP_CHAN, SIMA_MBOX_TX },
		{SIMA_MBOX_HP_CHAN, SIMA_MBOX_RX },
	};

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(rs); i++) {
		ret = sima_init_rs(pdev, &(mbox->channels[rs[i][0]].res[rs[i][1]]),
				   rs[i][0], rs[i][1], mbox);
		if (ret != 0)
			return ret;
	}

	mbox->channels[SIMA_MBOX_LP_CHAN].dev = &pdev->dev;
	mbox->chans[SIMA_MBOX_LP_CHAN].con_priv = &mbox->channels[SIMA_MBOX_LP_CHAN];
	mbox->channels[SIMA_MBOX_HP_CHAN].dev = &pdev->dev;
	mbox->chans[SIMA_MBOX_HP_CHAN].con_priv = &mbox->channels[SIMA_MBOX_HP_CHAN];
	mbox->controller.dev = &pdev->dev;
	mbox->controller.num_chans = 2;
	mbox->controller.chans = mbox->chans;
	mbox->controller.ops = &sima_mbox_ops;
	mbox->controller.txdone_irq = true;

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
	{ .compatible = "simaai,mailbox-1.0" },
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
