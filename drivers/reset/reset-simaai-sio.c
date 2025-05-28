// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiMa.ai SIO Reset Controller Driver
 * Copyright (c) 2023 Sima ai
 * Author: Yurii Konovalenko <yurii.konovalenko@sima.ai>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

#define SIMAAI_SIO_RESET_TIME_US	5
#define SIMAAI_SIO_RESET_RSTCLR		0x0
#define SIMAAI_SIO_RESET_RSTSET		0x4
#define SIMAAI_SIO_RESET_NUM		10


struct simaai_sio_reset_priv {
	spinlock_t			lock;
	void __iomem			*membase;
	struct reset_controller_dev	rcdev;
};

static struct simaai_sio_reset_priv *to_simaai_sio_reset_priv(
	struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct simaai_sio_reset_priv, rcdev);
}

static int simaai_sio_reset_update(struct reset_controller_dev *rcdev,
				   unsigned long id, bool assert)
{
	struct simaai_sio_reset_priv *priv = to_simaai_sio_reset_priv(rcdev);
	int offset = assert ? SIMAAI_SIO_RESET_RSTSET : SIMAAI_SIO_RESET_RSTCLR;
	u32 flags;

	spin_lock_irqsave(&priv->lock, flags);

	writel(BIT(id), priv->membase + offset);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int simaai_sio_reset_status(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct simaai_sio_reset_priv *priv = to_simaai_sio_reset_priv(rcdev);
	u32 reg;

	reg = readl(priv->membase + SIMAAI_SIO_RESET_RSTSET);

	return !!(reg & BIT(id));
}

static int simaai_sio_reset_assert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return simaai_sio_reset_update(rcdev, id, true);
}

static int simaai_sio_reset_deassert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	return simaai_sio_reset_update(rcdev, id, false);
}

static int simaai_sio_reset_reset(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	int ret;

	ret = simaai_sio_reset_assert(rcdev, id);
	if (ret)
		return ret;

	usleep_range(SIMAAI_SIO_RESET_TIME_US, SIMAAI_SIO_RESET_TIME_US * 2);

	return simaai_sio_reset_deassert(rcdev, id);
}

static const struct reset_control_ops simaai_sio_reset_ops = {
	.assert		= simaai_sio_reset_assert,
	.deassert	= simaai_sio_reset_deassert,
	.status		= simaai_sio_reset_status,
	.reset		= simaai_sio_reset_reset,
};

static int simaai_sio_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simaai_sio_reset_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->membase = devm_platform_ioremap_resource(pdev, 0);

	if (IS_ERR(priv->membase))
		return PTR_ERR(priv->membase);

	spin_lock_init(&priv->lock);
	priv->rcdev.owner	= THIS_MODULE;
	priv->rcdev.nr_resets	= SIMAAI_SIO_RESET_NUM;
	priv->rcdev.ops		= &simaai_sio_reset_ops;
	priv->rcdev.of_node	= dev->of_node;

	return devm_reset_controller_register(dev, &priv->rcdev);
}

static const struct of_device_id simaai_sio_reset_dt_ids[] = {
	{ .compatible = "simaai,reset-sio", },
	{ },
};
MODULE_DEVICE_TABLE(of, lantiq_rcu_reset_dt_ids);

static struct platform_driver simaai_sio_reset_driver = {
	.probe	= simaai_sio_reset_probe,
	.driver = {
		.name		= "simaai-sio-reset",
		.of_match_table	= simaai_sio_reset_dt_ids,
	},
};
module_platform_driver(simaai_sio_reset_driver);

MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai SIO Reset Controller Driver");
MODULE_LICENSE("GPL");
