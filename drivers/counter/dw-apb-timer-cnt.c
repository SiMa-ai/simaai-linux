// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare Timer Counter driver
 *
 * Copyright (c) 2022 Sima ai
 *
 * Author: Yurii Konovalenko <yurii.konovalenko@sima.ai>
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/counter.h>

#define DWAPBTMR_MAX_CHANNELS		0x08
#define DWAPBTMR_N_LOAD_COUNT		0x00
#define DWAPBTMR_N_CURRENT_VALUE		0x04
#define DWAPBTMR_N_CONTROL		0x08
#define DWAPBTMR_N_EOI			0x0c
#define DWAPBTMR_N_INT_STATUS		0x10
#define DWAPBTMR_CHANNEL_SIZE		0x14

#define APBTMR_CONTROL_ENABLE		(1 << 0)
/* 1: periodic, 0:free running. */
#define APBTMR_CONTROL_MODE_PERIODIC	(1 << 1)
#define APBTMR_CONTROL_INT		(1 << 2)

struct dwapbtimer_channel {
	u32 irq;
	u64 sw_counter;
	u64 last_val;
	void __iomem *base;
	struct device *dev;
	int id;
};

struct dwapbtimer_cnt {
	struct counter_device counter;
	u32 num_channels;
	struct dwapbtimer_channel channels[DWAPBTMR_MAX_CHANNELS];
};

struct stm32_timer_cnt {
	struct counter_device counter;
	struct regmap *regmap;
	struct clk *clk;
	u32 ceiling;
};

#define	DWAPBTIMER_CNT_SIGNAL(_id) { \
	.id = (_id), \
	.name = ("Channel "#_id" Clock") \
}

#define DWAPBTIMER_CNT_SYNAPSES(_id) { \
		.actions_list = dwapbtimer_synapse_actions, \
		.num_actions = ARRAY_SIZE(dwapbtimer_synapse_actions), \
		.signal = dwapbtimer_signals + (_id) \
}

#define DWAPBTIMER_CNT_COUNT(_id)  { \
	.id = (_id), \
	.name = "Channel "#_id" Count", \
	.functions_list = dwapbtimer_count_functions, \
	.num_functions = ARRAY_SIZE(dwapbtimer_count_functions), \
	.synapses = dwapbtimer_synapses + (_id), \
	.num_synapses = 1, \
	.ext = dwapbtimer_count_ext, \
	.num_ext = ARRAY_SIZE(dwapbtimer_count_ext) \
}

static inline u32 apbt_readl(struct dwapbtimer_channel *channel, u32 offs)
{
	return readl(channel->base + offs);
}

static inline void apbt_writel(struct dwapbtimer_channel *channel, u32 val,
		u32 offs)
{
	writel(val, channel->base + offs);
}

static void init_channel(struct dwapbtimer_channel *channel)
{
	channel->sw_counter = 0;
	channel->last_val = 0;
	apbt_writel(channel, ~0x0, DWAPBTMR_N_LOAD_COUNT);
	apbt_writel(channel, 0x0, DWAPBTMR_N_CONTROL);
}

static int dwapbtimer_count_write(struct counter_device *counter,
				  struct counter_count *count, u64 val)
{
	struct dwapbtimer_cnt *priv = counter_priv(counter);
	struct dwapbtimer_channel *channel = &priv->channels[count->id];
	u32 reg;


	reg = apbt_readl(channel, DWAPBTMR_N_CONTROL);
	apbt_writel(channel, reg&(~APBTMR_CONTROL_ENABLE), DWAPBTMR_N_CONTROL);
	channel->last_val = val;
	channel->sw_counter = val & 0xffffffff00000000;
	apbt_writel(channel, ~(val & 0xffffffff), DWAPBTMR_N_LOAD_COUNT);
	apbt_writel(channel, reg | APBTMR_CONTROL_ENABLE, DWAPBTMR_N_CONTROL);

	return 0;
}

static int dwapbtimer_count_read(struct counter_device *counter,
				 struct counter_count *count, u64 *val)
{
	struct dwapbtimer_cnt *priv = counter_priv(counter);
	struct dwapbtimer_channel *channel = &priv->channels[count->id];
	u64 new_val = channel->sw_counter |
			((u64)(~apbt_readl(channel, DWAPBTMR_N_CURRENT_VALUE)));

	/* There is low probability that we are reading counter value after
	 * counter overlapped but before interrupt handler increments software
	 * (upper) part of the counter. In this case we can increment it locally
	 * here.
	 */
	if (new_val < channel->last_val)
		new_val += 0x100000000;

	channel->last_val = new_val;

	*val = new_val;

	return 0;
}

static int dwapbtimer_function_read(struct counter_device *counter,
		struct counter_count *count, enum counter_function *function)
{
	/* Synopsys DesignWare counter supports only forward increase */
	*function = COUNTER_FUNCTION_INCREASE;

	return 0;
}

static int dwapbtimer_action_read(struct counter_device *counter,
			    struct counter_count *count,
			    struct counter_synapse *synapse,
			    enum counter_synapse_action *action)
{
	/* Synopsys DesignWare counter supports only rising edge */
	*action = COUNTER_SYNAPSE_ACTION_RISING_EDGE;

	return 0;
}

static const struct counter_ops dwapbtimer_cnt_ops = {
	.count_read = dwapbtimer_count_read,
	.count_write = dwapbtimer_count_write,
	.function_read = dwapbtimer_function_read,
	.action_read = dwapbtimer_action_read,
};

static int dwapbtimer_count_enable_read(struct counter_device *counter,
				       struct counter_count *count, u8 *preset_enable)
{
	struct dwapbtimer_cnt *priv = counter_priv(counter);
	struct dwapbtimer_channel *channel = &priv->channels[count->id];
	u32 reg;

	reg = apbt_readl(channel, DWAPBTMR_N_CONTROL);

	*preset_enable = !!(reg & APBTMR_CONTROL_ENABLE);

	return 0;
}

static int dwapbtimer_count_enable_write(struct counter_device *counter,
					struct counter_count *count, u8 enable)
{
	struct dwapbtimer_cnt *priv = counter_priv(counter);
	struct dwapbtimer_channel *channel = &priv->channels[count->id];
	u32 reg;

	reg = apbt_readl(channel, DWAPBTMR_N_CONTROL);

	if (enable)
		reg |= APBTMR_CONTROL_ENABLE;
	else
		reg &= (~APBTMR_CONTROL_ENABLE);

	apbt_writel(channel, reg, DWAPBTMR_N_CONTROL);

	return 0;
}

static struct counter_comp dwapbtimer_count_ext[] = {
	COUNTER_COMP_ENABLE(dwapbtimer_count_enable_read, dwapbtimer_count_enable_write),
};

static irqreturn_t dwapbtimer_cnt_interrupt(int irq, void *ch_id)
{
	struct dwapbtimer_channel *channel = (struct dwapbtimer_channel *)ch_id;
	u32 val;

	/* Verify if interrupt is valid, increment upper 32 bits of counter */
	val = apbt_readl(channel, DWAPBTMR_N_INT_STATUS);
	if (val == 0)
		dev_warn(channel->dev, "False counter interrupt: %d\n", irq);
	else
		channel->sw_counter += 0x100000000;

	/* Clean the interrupt */
	val = apbt_readl(channel, DWAPBTMR_N_EOI);

	return IRQ_HANDLED;
}

static enum counter_function dwapbtimer_count_functions[] = {
		COUNTER_FUNCTION_INCREASE
};

static enum counter_synapse_action dwapbtimer_synapse_actions[] = {
		COUNTER_SYNAPSE_ACTION_RISING_EDGE
};

static struct counter_signal dwapbtimer_signals[] = {
		DWAPBTIMER_CNT_SIGNAL(0), DWAPBTIMER_CNT_SIGNAL(1),
		DWAPBTIMER_CNT_SIGNAL(2), DWAPBTIMER_CNT_SIGNAL(3),
		DWAPBTIMER_CNT_SIGNAL(4), DWAPBTIMER_CNT_SIGNAL(5),
		DWAPBTIMER_CNT_SIGNAL(6), DWAPBTIMER_CNT_SIGNAL(7),
};

static struct counter_synapse dwapbtimer_synapses[] = {
		DWAPBTIMER_CNT_SYNAPSES(0), DWAPBTIMER_CNT_SYNAPSES(1),
		DWAPBTIMER_CNT_SYNAPSES(2), DWAPBTIMER_CNT_SYNAPSES(3),
		DWAPBTIMER_CNT_SYNAPSES(4), DWAPBTIMER_CNT_SYNAPSES(5),
		DWAPBTIMER_CNT_SYNAPSES(6), DWAPBTIMER_CNT_SYNAPSES(7)
};

static struct counter_count dwapbtimer_counts[] = {
		DWAPBTIMER_CNT_COUNT(0), DWAPBTIMER_CNT_COUNT(1),
		DWAPBTIMER_CNT_COUNT(2), DWAPBTIMER_CNT_COUNT(3),
		DWAPBTIMER_CNT_COUNT(4), DWAPBTIMER_CNT_COUNT(5),
		DWAPBTIMER_CNT_COUNT(6), DWAPBTIMER_CNT_COUNT(7)
};

static int dwapbtimer_cnt_probe(struct platform_device *pdev)
{
	struct counter_device *counter;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct dwapbtimer_cnt *priv;
	struct resource *r;
	void __iomem *base;
	u32 num_channels, ch;
	int ret;

	counter = devm_counter_alloc(&pdev->dev, sizeof(*priv));
	if (!counter)
		return -ENOMEM;
	priv = counter_priv(counter);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, r);
	if (IS_ERR(base))
		return PTR_ERR(base);


	ret = of_property_read_u32(node, "snps,channels", &num_channels);
	if (ret) {
		dev_warn(dev,
			 "Can't get number of channels, using 1 channel\n");
		num_channels = 1;
	}
	if (num_channels > DWAPBTMR_MAX_CHANNELS) {
		dev_warn(dev, "Maximum supported number of channels: %d\n",
				DWAPBTMR_MAX_CHANNELS);
		num_channels = DWAPBTMR_MAX_CHANNELS;
	}
	priv->num_channels = num_channels;

	for (ch = 0; ch < num_channels; ch++) {
		priv->channels[ch].base = base + (ch * DWAPBTMR_CHANNEL_SIZE);
		priv->channels[ch].dev = dev;
		priv->channels[ch].irq = platform_get_irq(pdev, ch);

		if (priv->channels[ch].irq < 0)
			dev_warn(dev, "ch%u: can't get irq, no u64 support\n",
				 ch);
		else {
			ret = request_irq(priv->channels[ch].irq,
					  dwapbtimer_cnt_interrupt,
					  IRQF_SHARED | IRQF_NO_SUSPEND,
					  dev_name(dev), &priv->channels[ch]);
			if (ret)
				dev_err(dev, "ch%u: can't request irq %d\n",
					ch, priv->channels[ch].irq);
		}
		init_channel(&priv->channels[ch]);
	}

	counter->name = dev_name(dev);
	counter->parent = dev;
	counter->ops = &dwapbtimer_cnt_ops;
	counter->counts = dwapbtimer_counts;
	counter->num_counts = num_channels;
	counter->signals = dwapbtimer_signals;
	counter->num_signals = num_channels;

	dev_info(dev, "Initialized %d counter channels\n", num_channels);

	/* Register Counter device */
	return devm_counter_add(dev, counter);
}

static const struct of_device_id dwapbtimer_cnt_of_match[] = {
	{ .compatible = "snps,dw-apb-timer-counter-2.12a", },
	{},
};
MODULE_DEVICE_TABLE(of, dwapbtimer_cnt_of_match);

static struct platform_driver dwapbtimer_cnt_driver = {
	.probe = dwapbtimer_cnt_probe,
	.driver = {
		.name = "dw-apb-timer-counter",
		.of_match_table = dwapbtimer_cnt_of_match,
	},
};
module_platform_driver(dwapbtimer_cnt_driver);

MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_ALIAS("platform:dw-apb-timer-counter");
MODULE_DESCRIPTION("Synopsys DesignWare Timer counter driver");
MODULE_LICENSE("GPL v2");
