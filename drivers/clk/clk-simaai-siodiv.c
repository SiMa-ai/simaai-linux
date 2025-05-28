// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the SiMa.ai SIO's clock divider
 * Copyright (c) 2023 Sima ai
 * Author: Yurii Konovalenko <yurii.konovalenko@sima.ai>
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/err.h>

#define SIMAAI_SIODIV_NUM		(10)
#define SIMAAI_SIODIV_EN		BIT(31)
#define SIMAAI_SIODIV_INT_SHIFT		(0)
#define SIMAAI_SIODIV_INT_WIDTH		(20)
#define SIMAAI_SIODIV_INT_MASK		((1 << SIMAAI_SIODIV_INT_WIDTH) - 1)
#define SIMAAI_SIODIV_FRAC_SHIFT	(20)
#define SIMAAI_SIODIV_FRAC_WIDTH	(8)
#define SIMAAI_SIODIV_FRAC_MASK		((1 << SIMAAI_SIODIV_FRAC_WIDTH) - 1)

struct simaai_siodiv_clk {
	struct clk_hw			hw;
	void __iomem			*reg;
	u32				ignore_unused;
};

struct simaai_siodiv_priv {
	struct clk			*pclk;
	void __iomem			*base;
	struct clk_hw_onecell_data	*clk_data;
	struct clk_init_data		init[SIMAAI_SIODIV_NUM];
	u32				ignore_unused;
};

static inline struct simaai_siodiv_clk *to_simaai_siodiv_clk(struct clk_hw *hw)
{
	return container_of(hw, struct simaai_siodiv_clk, hw);
}

u64 simaai_siodiv_best_div(u64 rate, u64 prate)
{
	u64 div;

	div = DIV_ROUND_DOWN_ULL((prate << SIMAAI_SIODIV_FRAC_WIDTH), rate);
	div = clamp(div, (u64)(1 << SIMAAI_SIODIV_FRAC_WIDTH),
		(u64)((1 << (SIMAAI_SIODIV_FRAC_WIDTH + SIMAAI_SIODIV_INT_WIDTH)) - 1));

	return div;
}

static int simaai_siodiv_is_enabled(struct clk_hw *hw)
{
	struct simaai_siodiv_clk *clk = to_simaai_siodiv_clk(hw);

	return !!(readl(clk->reg) & SIMAAI_SIODIV_EN);
}

static int simaai_siodiv_enable(struct clk_hw *hw)
{
	struct simaai_siodiv_clk *clk = to_simaai_siodiv_clk(hw);
	u32 regval;

	if (simaai_siodiv_is_enabled(hw))
		return 0;

	regval = readl(clk->reg);
	regval |= SIMAAI_SIODIV_EN;
	writel(regval, clk->reg);

	return 0;
}

static void simaai_siodiv_disable(struct clk_hw *hw)
{
	struct simaai_siodiv_clk *clk = to_simaai_siodiv_clk(hw);
	u32 regval;

	if (clk->ignore_unused)
		return;

	if (!simaai_siodiv_is_enabled(hw))
		return;

	regval = readl(clk->reg);
	regval &= ~SIMAAI_SIODIV_EN;
	writel(regval, clk->reg);
}

static void simaai_siodiv_write_div(void __iomem *reg, u64 div)
{
	u32 regval;

	regval = readl(reg);
	regval &= ~(SIMAAI_SIODIV_INT_MASK << SIMAAI_SIODIV_INT_SHIFT);
	regval &= ~(SIMAAI_SIODIV_FRAC_MASK << SIMAAI_SIODIV_FRAC_SHIFT);
	regval |= (div & SIMAAI_SIODIV_FRAC_MASK) << SIMAAI_SIODIV_FRAC_SHIFT;
	div >>= SIMAAI_SIODIV_FRAC_WIDTH;
	regval |= (div & SIMAAI_SIODIV_INT_MASK) << SIMAAI_SIODIV_INT_SHIFT;
	writel(regval, reg);
}

static int simaai_siodiv_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long prate)
{
	struct simaai_siodiv_clk *clk = to_simaai_siodiv_clk(hw);
	u64 div;

	div = simaai_siodiv_best_div((u64)rate, (u64)prate);
	simaai_siodiv_write_div(clk->reg, div);

	return 0;
}

static long simaai_siodiv_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	u64 div;

	div = simaai_siodiv_best_div((u64)rate, (u64)(*prate));

	return DIV_ROUND_DOWN_ULL(*prate << SIMAAI_SIODIV_FRAC_WIDTH, div);
}

static unsigned long simaai_siodiv_recalc_rate(struct clk_hw *hw,
					  unsigned long prate)
{
	struct simaai_siodiv_clk *clk = to_simaai_siodiv_clk(hw);
	u64 div;
	u32 regval;

	regval = readl(clk->reg);
	div = ((regval >> SIMAAI_SIODIV_INT_SHIFT) & SIMAAI_SIODIV_INT_MASK)
			<< SIMAAI_SIODIV_FRAC_WIDTH;
	div |= ((regval >> SIMAAI_SIODIV_FRAC_SHIFT) & SIMAAI_SIODIV_FRAC_MASK);

	return DIV_ROUND_DOWN_ULL(((u64)prate << SIMAAI_SIODIV_FRAC_WIDTH), div);
}

static const struct clk_ops simaai_siodiv_ops = {
	.enable		= simaai_siodiv_enable,
	.disable	= simaai_siodiv_disable,
	.is_enabled	= simaai_siodiv_is_enabled,
	.set_rate	= simaai_siodiv_set_rate,
	.round_rate	= simaai_siodiv_round_rate,
	.recalc_rate	= simaai_siodiv_recalc_rate,
};

static int simaai_siodiv_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct simaai_siodiv_priv *priv;
	const char *pclk_name;
	struct device *dev = &pdev->dev;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;


	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (of_property_read_u32(dev->of_node, "simaai,ignore-unused", &priv->ignore_unused))
		priv->ignore_unused = 0;

	priv->pclk = devm_clk_get(dev, NULL);
	pclk_name = __clk_get_name(priv->pclk);

	priv->clk_data = devm_kzalloc(dev, struct_size(priv->clk_data, hws,
			SIMAAI_SIODIV_NUM), GFP_KERNEL);
	if (!priv->clk_data)
		return -ENOMEM;

	priv->clk_data->num = SIMAAI_SIODIV_NUM;

	for (i = 0; i < SIMAAI_SIODIV_NUM; i++) {
		struct simaai_siodiv_clk *clk;

		clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
		if (!clk)
			return -ENOMEM;

		priv->init[i].name		= devm_kasprintf(dev, GFP_KERNEL, "%s_div_%u",
								 dev_name(dev), i);
		if (!priv->init[i].name)
			return -ENOMEM;
		priv->init[i].ops		= &simaai_siodiv_ops;
		priv->init[i].num_parents	= 1;
		priv->init[i].parent_names	= &pclk_name;
		priv->init[i].flags		= CLK_SET_RATE_GATE;
		clk->hw.init			= &priv->init[i];
		clk->reg			= priv->base + (i * 4);
		if (priv->ignore_unused & (1 << i)) {
			priv->init[i].flags	|= CLK_IGNORE_UNUSED;
			clk->ignore_unused	= 1;
			simaai_siodiv_enable(&clk->hw);
		}

		simaai_siodiv_write_div(clk->reg, 1 << SIMAAI_SIODIV_FRAC_WIDTH);

		ret = devm_clk_hw_register(dev, &clk->hw);
		if (ret)
			return ret;

		priv->clk_data->hws[i]		= &clk->hw;
	}

	platform_set_drvdata(pdev, priv);

	return devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_onecell_get,
			priv->clk_data);
}

static const struct of_device_id simaai_siodiv_ids[] = {
	{ .compatible = "simaai,clkdiv-sio", },
	{ },
};
MODULE_DEVICE_TABLE(of, simaai_siodiv_ids);

static struct platform_driver simaai_siodiv_driver = {
	.driver	= {
		.name		= "simaai-sio-clkdiv",
		.of_match_table	= simaai_siodiv_ids,
	},
	.probe	= simaai_siodiv_probe,
};
module_platform_driver(simaai_siodiv_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_DESCRIPTION("Driver for the SiMa.ai SIO's clock divider");
