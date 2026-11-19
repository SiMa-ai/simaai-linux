// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the SiMa.ai clock controller
 * Copyright (c) 2026 SiMa.ai
 * Author: Bharanidharan Ramalingam <bharanidharan.ramalingam@sima.ai>
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <dt-bindings/clock/simaai,clkc-prc.h>

#define PLL_FBDIV_MIN		16
#define PLL_FBDIV_MAX		4095
#define PLL_POSTDIV1_MAX	7
#define SIMAAI_PLL_LOCK_TIMEOUT_US	50
#define SIMAAI_PLL_EN        BIT(0)
#define SIMAAI_PLL_RSTN      BIT(1)
#define SIMAAI_PLL_BYP       BIT(2)
#define SIMAAI_PLL_REFDIV    GENMASK(9, 4)
#define SIMAAI_PLL_FBDIV     GENMASK(22, 11)
#define SIMAAI_PLL_POSTDIV1  GENMASK(25, 23)
#define SIMAAI_PLL_POSTDIV2  GENMASK(29, 27)
#define SIMAAI_PLL_LOCKED    BIT(30)
#define SIMAAI_PLL_TIMEOUT   BIT(31)

/**
 * struct simaai_prc_clk - runtime state for one clock owned by the controller
 * @hw:          clock framework handle
 * @reg:         MMIO address of this clock's control register
 * @enable_mask: bit(s) to set/clear to gate the clock on/off
 */

struct simaai_prc_clk {
	struct clk_hw	hw;
	void __iomem	*reg;
	u32		enable_mask;
};

#define to_simaai_prc_clk(_hw) container_of(_hw, struct simaai_prc_clk, hw)

/**
 * simaai_prc_clk_enable() - ungate a clock
 * @hw: clock framework handle
 *
 * Return: 0 always (the gate cannot fail).
 */

static int simaai_prc_clk_enable(struct clk_hw *hw)
{
	struct simaai_prc_clk *c = to_simaai_prc_clk(hw);
	u32 val = readl(c->reg);

	writel(val |= c->enable_mask, c->reg);

	pr_debug("%s %s val:0x%x\n", __func__, clk_hw_get_name(hw), readl(c->reg));
	return 0;
}

/**
 * simaai_prc_clk_disable() - gate a clock off
 * @hw: clock framework handle
 *
 * Clears the enable bit in the clock's control register.
 */

static void simaai_prc_clk_disable(struct clk_hw *hw)
{
	struct simaai_prc_clk *c = to_simaai_prc_clk(hw);
	u32 val = readl(c->reg);

	writel(val &= ~c->enable_mask, c->reg);

	pr_debug("%s %s val:0x%x\n", __func__, clk_hw_get_name(hw), readl(c->reg));
}

/**
 * simaai_prc_clk_is_enabled() - query whether a clock is ungated
 * @hw: clock framework handle
 *
 * Return: 1 if the enable bit is set, 0 otherwise.
 */

static int simaai_prc_clk_is_enabled(struct clk_hw *hw)
{
	struct simaai_prc_clk *c = to_simaai_prc_clk(hw);
	u32 val = 0;

	return !!(readl(c->reg) & c->enable_mask);
}

static void print_pll_bits(struct clk_hw *hw, u32 val)
{
	pr_debug("simaai-pll: %s = 0x%08x (to=%x lk=%x pd2=%x pd1=%x fb=%x ref=%x byp=%x rst=%x en=%x)\n",
		clk_hw_get_name(hw), val,
		(u32)FIELD_GET(SIMAAI_PLL_TIMEOUT,  val),
		(u32)FIELD_GET(SIMAAI_PLL_LOCKED,   val),
		(u32)FIELD_GET(SIMAAI_PLL_POSTDIV2, val),
		(u32)FIELD_GET(SIMAAI_PLL_POSTDIV1, val),
		(u32)FIELD_GET(SIMAAI_PLL_FBDIV,    val),
		(u32)FIELD_GET(SIMAAI_PLL_REFDIV,   val),
		(u32)FIELD_GET(SIMAAI_PLL_BYP,      val),
		(u32)FIELD_GET(SIMAAI_PLL_RSTN,     val),
		(u32)FIELD_GET(SIMAAI_PLL_EN,       val));
}

/**
 * simaai_prc_pll_recalc_rate() - compute the current PLL output rate
 * @hw:          clock framework handle
 * @parent_rate: rate of the PLL reference input, in Hz
 *
 * Reconstructs the rate from the programmed divider fields:
 *	rate = (parent_rate * FBDIV) / (REFDIV * POSTDIV1 * POSTDIV2)
 * and rounds it to the nearest kHz to hide sub-kHz rounding noise.
 *
 * Return: the output rate in Hz, or 0 if the PLL is unconfigured.
 */

static unsigned long simaai_prc_pll_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct simaai_prc_clk *c = to_simaai_prc_clk(hw);
	u32 val    = readl(c->reg);
	u32 fbdiv  = FIELD_GET(SIMAAI_PLL_FBDIV,    val);
	u32 refdiv = FIELD_GET(SIMAAI_PLL_REFDIV,   val);
	u32 p1     = FIELD_GET(SIMAAI_PLL_POSTDIV1, val);
	u32 p2     = FIELD_GET(SIMAAI_PLL_POSTDIV2, val);
	u32 div    = refdiv * p1 * p2;
	unsigned long rate;

	if (!fbdiv || !div)
		return 0;

	rate = div_u64((u64)parent_rate * fbdiv, div);
	return DIV_ROUND_CLOSEST(rate, 1000) * 1000;
}

/**
 * simaai_prc_pll_wait_lock() - wait for the PLL to lock after a rate change
 * @c: clock to poll
 *
 * The hardware raises LOCKED on success or TIMEOUT if it gives up. The
 * readl_poll_timeout_atomic() guard is a software backstop in case neither bit
 * ever asserts.
 *
 * Return: 0 on lock, -ETIMEDOUT if the hardware reports a lock timeout, or
 * the error from readl_poll_timeout_atomic() on software timeout.
 */

static int simaai_prc_pll_wait_lock(struct simaai_prc_clk *c)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(c->reg, val,
				       (val & SIMAAI_PLL_LOCKED) ||
				       (val & SIMAAI_PLL_TIMEOUT),
				       1, SIMAAI_PLL_LOCK_TIMEOUT_US);
	if (ret) {
		pr_err("simaai-pll: software timeout, reg=0x%08x\n", val);
		return ret;
	}

	if (val & SIMAAI_PLL_TIMEOUT) {
		pr_err("simaai-pll: hardware reported lock timeout, reg=0x%08x\n", val);
		return -ETIMEDOUT;
	}

	/* val & SIMAAI_PLL_LOCKED is set — success */
	return 0;
}

/**
 * simaai_prc_pll_best_div() - find the divider set closest to a target rate
 * @rate:        requested output rate, in Hz
 * @parent_rate: rate of the PLL reference input, in Hz
 * @refdiv:      REFDIV in effect (fixed at boot, preserved across rate changes)
 * @fbdiv:       feedback divider
 * @postdiv1:    post-divider1
 * @postdiv2:    post-divider2
 *
 * Sweeps every legal (POSTDIV1, POSTDIV2) combination subject to the hardware
 * rule POSTDIV2 <= POSTDIV1, derives the matching FBDIV for each, and keeps the
 * combination whose synthesized rate,
 *	parent_rate * FBDIV / (refdiv * POSTDIV1 * POSTDIV2),
 * is closest to @rate.
 *
 * Return: 0 on success, or -EINVAL if no in-range combination can produce @rate.
 */

static int simaai_prc_pll_best_div(unsigned long rate, unsigned long parent_rate,
				   u32 refdiv, u32 *fbdiv, u32 *postdiv1, u32 *postdiv2)
{
	unsigned long best_diff = ULONG_MAX;
	u32 p1, p2;

	*fbdiv = 0;
	*postdiv1 = 0;
	*postdiv2 = 0;

	for (p1 = 1; p1 <= PLL_POSTDIV1_MAX; p1++) {
		for (p2 = 1; p2 <= p1; p2++) {
			u32 div = refdiv * p1 * p2;
			u64 m = DIV_ROUND_CLOSEST_ULL((u64)rate * div, parent_rate);
			u64 actual;
			unsigned long diff;

			if (m < PLL_FBDIV_MIN || m > PLL_FBDIV_MAX)
				continue;

			actual = div_u64((u64)parent_rate * m, div);
			diff = abs_diff((unsigned long)actual, rate);

			if (diff < best_diff ||
			    (diff == best_diff && p2 < *postdiv2)) {
				best_diff = diff;
				*fbdiv = m;
				*postdiv1 = p1;
				*postdiv2 = p2;
			}
		}
	}

	return *fbdiv ? 0 : -EINVAL;
}

/**
 * simaai_prc_pll_round_rate() - clamp a requested rate to one the PLL can produce
 * @hw:          clock framework handle
 * @rate:        requested rate, in Hz
 * @parent_rate: rate of the PLL reference input, in Hz
 *
 * Runs the divider search and returns the rate the resulting
 * FBDIV/POSTDIV1/POSTDIV2 set would actually synthesize, without touching the
 * hardware.
 *
 * Return: the closest achievable rate in Hz, or -EINVAL if no divider set
 * satisfies the hardware constraints.
 */

static long simaai_prc_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	struct simaai_prc_clk *c = to_simaai_prc_clk(hw);
	u32 refdiv = FIELD_GET(SIMAAI_PLL_REFDIV, readl(c->reg));
	unsigned long out;
	u32 fbdiv, p1, p2;

	if (!refdiv)
		return -EINVAL;

	if (simaai_prc_pll_best_div(rate, *parent_rate, refdiv, &fbdiv, &p1, &p2))
		return -EINVAL;

	out = div_u64((u64)*parent_rate * fbdiv, (u64)refdiv * p1 * p2);
	return DIV_ROUND_CLOSEST(out, 1000) * 1000;
}

/**
 * simaai_prc_pll_set_rate() - program a new PLL output rate
 * @hw:          clock framework handle
 * @rate:        target rate, in Hz
 * @parent_rate: rate of the PLL reference input, in Hz
 *
 * Picks the best FBDIV/POSTDIV1/POSTDIV2 set (REFDIV is left untouched), then:
 * assert BYP so consumers run from the reference clock, clear RSTN, write the
 * divider fields (restoring RSTN in the same store), wait for lock, let the PLL
 * settle, then deassert BYP so consumers switch to the new rate.
 *
 * Return: 0 on success, -EINVAL if no divider set can reach @rate, or the
 * error from simaai_prc_pll_wait_lock() if the PLL fails to lock.
 */

static int simaai_prc_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct simaai_prc_clk *c = to_simaai_prc_clk(hw);
	u32 fbdiv, p1, p2, refdiv;
	u32 cur, v_byp, v_rst, v_prog, v_unbyp;
	int ret;
	unsigned long flags;

	cur = readl(c->reg);

	refdiv = FIELD_GET(SIMAAI_PLL_REFDIV, cur);
	if (!refdiv)
		return -EINVAL;

	ret = simaai_prc_pll_best_div(rate, parent_rate, refdiv, &fbdiv, &p1, &p2);
	if (ret)
		return ret;

	v_byp   = cur | SIMAAI_PLL_BYP;
	v_rst   = v_byp & ~SIMAAI_PLL_RSTN;
	v_prog  = (v_rst & ~(SIMAAI_PLL_FBDIV | SIMAAI_PLL_POSTDIV1 | SIMAAI_PLL_POSTDIV2))
		  | FIELD_PREP(SIMAAI_PLL_FBDIV,    fbdiv)
		  | FIELD_PREP(SIMAAI_PLL_POSTDIV1, p1)
		  | FIELD_PREP(SIMAAI_PLL_POSTDIV2, p2)
		  | SIMAAI_PLL_RSTN;
	v_unbyp = v_prog & ~SIMAAI_PLL_BYP;

	/* Interrupts will extend the window spent on the bypass clock */
	local_irq_save(flags);
	writel(v_byp,  c->reg);
	readl(c->reg);
	writel(v_rst,  c->reg);
	readl(c->reg);
	writel(v_prog, c->reg);

	ret = simaai_prc_pll_wait_lock(c);
	udelay(1);
	writel(v_unbyp, c->reg);
	local_irq_restore(flags);

	print_pll_bits(hw, readl(c->reg));
	return ret;
}

static const struct clk_ops simaai_prc_gate_ops = {
	.enable     = simaai_prc_clk_enable,
	.disable    = simaai_prc_clk_disable,
	.is_enabled = simaai_prc_clk_is_enabled,
};

static const struct clk_ops simaai_prc_pll_ops = {
	.recalc_rate = simaai_prc_pll_recalc_rate,
	.round_rate  = simaai_prc_pll_round_rate,
	.set_rate    = simaai_prc_pll_set_rate,
};

enum simaai_prc_clk_type { SIMAAI_GATE, SIMAAI_PLL };

struct simaai_prc_clk_desc {
	const char	*name;
	const char	*parent;
	u32		offset;	/* offset within MMIO base */
	u32		enable_mask;
	enum simaai_prc_clk_type	type;
};

/* ============================================================
 * Clock table — add one entry per clock this controller owns.
 * ============================================================
 */

static const struct simaai_prc_clk_desc simaai_prc_clocks[] = {
	[SIMAAI_PRC_CLK_MLA_PLL]  = { "mlapll", NULL, 0x014, SIMAAI_PLL_EN, SIMAAI_PLL},
	[SIMAAI_PRC_CLK_ISP_PLL] = { "isppll", NULL, 0x008, SIMAAI_PLL_EN, SIMAAI_PLL},
	[SIMAAI_PRC_CLK_MLA_GATE]  = { "mlaclk", "mlapll", 0x594, (BIT(0) | BIT(1)), SIMAAI_GATE},
	[SIMAAI_PRC_CLK_ISP_GATE] = { "ispclk", "isppll", 0x518, BIT(0), SIMAAI_GATE},
	[SIMAAI_PRC_CLK_APU_PLL] = { "apupll", NULL, 0x004, SIMAAI_PLL_EN, SIMAAI_PLL},

	/* TODO: add more clocks here */
};

#define SIMAAI_NUM_CLOCKS	ARRAY_SIZE(simaai_prc_clocks)

/**
 * simaai_prc_clkc_probe() - probe and register the clock controller
 * @pdev: platform device for the controller
 *
 * Maps the controller MMIO region, then walks the static clock table
 * registering each entry as either a PLL or a gate. The optional
 * "simaai,ignore-unused" DT bitmask marks clocks that must keep running even
 * when no consumer claims them. Finally exposes the clocks via an
 * of_clk_hw_onecell provider (#clock-cells = <1>).
 *
 * Return: 0 on success or a negative errno on failure.
 */

static int simaai_prc_clkc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *clk_data;
	const char *parent_name;
	void __iomem *base;
	u32 ignore_unused, v;
	int i, ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	pr_debug("%s ioremap done base 0x%llx\n", __func__, (u64)base);

	parent_name = of_clk_get_parent_name(dev->of_node, 0);
	if (!parent_name)
		return dev_err_probe(dev, -EINVAL, "missing parent clock\n");

	/* Optional bitmask: bit i keeps clock i running even if unused. */
	if (of_property_read_u32(dev->of_node, "simaai,ignore-unused", &ignore_unused))
		ignore_unused = 0;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws,
				SIMAAI_NUM_CLOCKS), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;
	clk_data->num = SIMAAI_NUM_CLOCKS;

	for (i = 0; i < SIMAAI_NUM_CLOCKS; i++) {
		const struct simaai_prc_clk_desc *d = &simaai_prc_clocks[i];
		struct clk_init_data init = {};
		struct simaai_prc_clk *c;
		const char *parent = d->parent ? d->parent : parent_name;

		c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
		if (!c)
			return -ENOMEM;

		init.name = d->name;
		/* PLLs get rate ops; gates get enable/disable ops. */
		init.ops = (d->type == SIMAAI_PLL) ? &simaai_prc_pll_ops : &simaai_prc_gate_ops;
		init.parent_names = &parent;
		init.num_parents = 1;
		if (ignore_unused & (1 << i))
			init.flags |= CLK_IGNORE_UNUSED;

		/* Applies to plls which have GATE */
		if (d->type == SIMAAI_PLL)
			init.flags |= CLK_SET_RATE_GATE;

		c->reg = base + d->offset;
		c->enable_mask = d->enable_mask;
		c->hw.init = &init;

		ret = devm_clk_hw_register(dev, &c->hw);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to register %s\n",
					     d->name);
		clk_data->hws[i] = &c->hw;
	}
	pr_debug("%s registered clk\n", __func__);
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
}

static const struct of_device_id simaai_prc_clkc_ids[] = {
	{ .compatible = "simaai,clkc-prc", },
	{ }
};
MODULE_DEVICE_TABLE(of, simaai_prc_clkc_ids);

static struct platform_driver simaai_prc_clkc_driver = {
	.driver	= {
		.name		= "simaai-clkc-prc",
		.of_match_table	= simaai_prc_clkc_ids,
	},
	.probe	= simaai_prc_clkc_probe,
};
module_platform_driver(simaai_prc_clkc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bharanidharan Ramalingam <bharanidharan.ramalingam@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai PRC clock controller");

