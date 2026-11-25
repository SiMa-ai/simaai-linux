// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI D-PHY controller driver
 * Core functions
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#include "dw-dphy-rx.h"
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/io.h>

struct range_dphy_gen2 {
	u32 freq;
	u8 hsfregrange;
};

static const struct range_dphy_gen2 range_gen2[] = {
	{ 80, 0x00 },   { 90, 0x10 },   { 100, 0x20 },  { 110, 0x30 },
	{ 120, 0x01 },  { 130, 0x11 },  { 140, 0x21 },  { 150, 0x31 },
	{ 160, 0x02 },  { 170, 0x12 },  { 180, 0x22 },  { 190, 0x32 },
	{ 205, 0x03 },  { 220, 0x13 },  { 235, 0x23 },  { 250, 0x33 },
	{ 275, 0x04 },  { 300, 0x14 },  { 325, 0x05 },  { 350, 0x15 },
	{ 400, 0x25 },  { 450, 0x06 },  { 500, 0x16 },  { 550, 0x07 },
	{ 600, 0x17 },  { 650, 0x08 },  { 700, 0x18 },  { 750, 0x09 },
	{ 800, 0x19 },  { 850, 0x29 },  { 900, 0x39 },  { 950, 0x0A },
	{ 1000, 0x1A }, { 1050, 0x2A }, { 1100, 0x3A }, { 1150, 0x0B },
	{ 1200, 0x1B }, { 1250, 0x2B }, { 1300, 0x3B }, { 1350, 0x0C },
	{ 1400, 0x1C }, { 1450, 0x2C }, { 1500, 0x3C }, { 1550, 0x0D },
	{ 1600, 0x1D }, { 1650, 0x2D }, { 1700, 0x0E }, { 1750, 0x1E },
	{ 1800, 0x2E }, { 1850, 0x3E }, { 1900, 0x0F }, { 1950, 0x1F },
	{ 2000, 0x2F },
};

struct range_dphy_gen3 {
	u32 freq;
	u8 hsfregrange;
};

static const struct range_dphy_gen3 range_gen3[] = {
	{ 80, 0x00 },   { 90, 0x10 },   { 100, 0x20 },  { 110, 0x30 },
	{ 120, 0x01 },  { 130, 0x11 },  { 140, 0x21 },  { 150, 0x31 },
	{ 160, 0x02 },  { 170, 0x12 },  { 180, 0x22 },  { 190, 0x32 },
	{ 205, 0x03 },  { 220, 0x13 },  { 235, 0x23 },  { 250, 0x33 },
	{ 275, 0x04 },  { 300, 0x14 },  { 325, 0x25 },  { 350, 0x35 },
	{ 400, 0x05 },  { 450, 0x16 },  { 500, 0x26 },  { 550, 0x37 },
	{ 600, 0x07 },  { 650, 0x18 },  { 700, 0x28 },  { 750, 0x39 },
	{ 800, 0x09 },  { 850, 0x19 },  { 900, 0x29 },  { 950, 0x3A },
	{ 1000, 0x0A }, { 1050, 0x1A }, { 1100, 0x2A }, { 1150, 0x3B },
	{ 1200, 0x0B }, { 1250, 0x1B }, { 1300, 0x2B }, { 1350, 0x3C },
	{ 1400, 0x0C }, { 1450, 0x1C }, { 1500, 0x2C }, { 1550, 0x3D },
	{ 1600, 0x0D }, { 1650, 0x1D }, { 1700, 0x2E }, { 1750, 0x3E },
	{ 1800, 0x0E }, { 1850, 0x1E }, { 1900, 0x2F }, { 1950, 0x3F },
	{ 2000, 0x0F }, { 2050, 0x40 }, { 2100, 0x41 }, { 2150, 0x42 },
	{ 2200, 0x43 }, { 2250, 0x44 }, { 2300, 0x45 }, { 2350, 0x46 },
	{ 2400, 0x47 }, { 2450, 0x48 }, { 2500, 0x49 },
};

u8 dw_dphy_setup_config(struct dw_dphy_rx *dphy)
{
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	u8 ret;
	int setup_config;

	if (dphy->max_lanes == CTRL_4_LANES) {
		dev_vdbg(&dphy->phy->dev, "CONFIG 4L\n");
		return CTRL_4_LANES;
	}

	ret = gpio_request(dphy->config_gpio, "config");
	if (ret < 0) {
			dev_vdbg(&dphy->phy->dev,
				 "could not acquire config (err=%d)\n", ret);
			return ret;
	}
	setup_config = gpio_get_value(dphy->config_gpio);
	gpio_free(dphy->config_gpio);

	dev_vdbg(&dphy->phy->dev,
		 "Booting in [%s] mode\n",
		 setup_config == CTRL_8_LANES ? "8L" : "4+4L");
	return setup_config;
#endif /* CONFIG_DWC_MIPI_TC_DPHY_GEN3 */
	return CTRL_4_LANES;
}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
static void dw_dphy_if_write(struct dw_dphy_rx *dphy, u32 address, u32 data)
{
	iowrite32(data, dphy->dphy1_if_addr + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		return;

	iowrite32(data, dphy->dphy2_if_addr + address);
}

u32 dw_dphy_if_read(struct dw_dphy_rx *dphy, u32 address)
{
	u32 if1 = 0, if2 = 0;

	if1 = ioread32(dphy->dphy1_if_addr + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		goto end;

	if (dphy->lanes_config == DPHYID)
		goto end;

	if2 = ioread32(dphy->dphy2_if_addr + address);

	if (if1 != if2)
		dev_vdbg(&dphy->phy->dev,
			 "Values read different for each interface\n");
end:
	return if1;
}
#endif

static void dw_dphy_glue_write(struct dw_dphy_rx *dphy, u32 address, u32 data)
{
	iowrite32(data, dphy->glue_base_address + address);
}

static u32 dw_dphy_glue_read(struct dw_dphy_rx *dphy, u32 address)
{
	return ioread32(dphy->glue_base_address + address);
}

void dw_dphy_write(struct dw_dphy_rx *dphy, u32 address, u32 data)
{
	iowrite32(data, dphy->base_address + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		return;

	if (address == R_CSI2_DPHY_TST_CTRL0)
		iowrite32(data, dphy->base_address + R_CSI2_DPHY2_TST_CTRL0);
	else if (address == R_CSI2_DPHY_TST_CTRL1)
		iowrite32(data, dphy->base_address + R_CSI2_DPHY2_TST_CTRL1);
}

u32 dw_dphy_read(struct dw_dphy_rx *dphy, u32 address)
{
	u32 dphy1 = 0, dphy2 = 0;

	dphy1 = ioread32(dphy->base_address + address);

	if (dphy->lanes_config == CTRL_4_LANES)
		goto end;

	if (address == R_CSI2_DPHY_TST_CTRL0)
		dphy2 = ioread32(dphy->base_address + R_CSI2_DPHY2_TST_CTRL0);
	else if (address == R_CSI2_DPHY_TST_CTRL1)
		dphy2 = ioread32(dphy->base_address + R_CSI2_DPHY2_TST_CTRL1);
	else
		return -ENODEV;

	if (dphy1 != dphy2)
		dev_vdbg(&dphy->phy->dev,
			 "Values read different for each interface\n");
end:
	return dphy1;
}

static void dw_dphy_glue_write_msk(struct dw_dphy_rx *dev, u32 address,
				   u32 data, u8 shift, u8 width)
{
	u32 mask = (1 << width) - 1;
	u32 temp = dw_dphy_glue_read(dev, address);

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	dw_dphy_glue_write(dev, address, temp);
}

static void dw_dphy_write_msk(struct dw_dphy_rx *dev, u32 address, u32 data,
			      u8 shift, u8 width)
{
	u32 temp = dw_dphy_read(dev, address);
	u32 mask = (1 << width) - 1;

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	dw_dphy_write(dev, address, temp);
}

static void dw_dphy_te_12b_write(struct dw_dphy_rx *dphy, u16 addr, u8 data)
{
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00, PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)(addr >> 8),
			  PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)addr, PHY_TESTDIN,
			  8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)data, PHY_TESTDIN,
			  8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
}

static void dw_dphy_te_8b_write(struct dw_dphy_rx *dphy, u8 addr, u8 data)
{
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL1, addr);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_TST_CTRL1, data);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
}

static void dw_dphy_te_write(struct dw_dphy_rx *dphy, u16 addr, u8 data)
{
	if (dphy->dphy_te_len == BIT12)
		dw_dphy_te_12b_write(dphy, addr, data);
	else
		dw_dphy_te_8b_write(dphy, addr, data);
}

static int dw_dphy_te_12b_read(struct dw_dphy_rx *dphy, u32 addr)
{
	u8 ret;

	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00, PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)(addr >> 8),
			  PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (u8)addr, PHY_TESTDIN,
			  8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00, 0, PHY_TESTDIN);
	ret = dw_dphy_read_msk(dphy, R_CSI2_DPHY_TST_CTRL1, PHY_TESTDOUT, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);
	dw_dphy_read(dphy, R_CSI2_DPHY_SHUTDOWNZ);
	udelay(1);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);

	return ret;
}

static int dw_dphy_te_8b_read(struct dw_dphy_rx *dphy, u32 addr)
{
	u8 ret;

	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, addr, PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTDIN, 8);
	ret = dw_dphy_read_msk(dphy, R_CSI2_DPHY_TST_CTRL1, PHY_TESTDOUT, 8);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);
	dw_dphy_read(dphy, R_CSI2_DPHY_SHUTDOWNZ);
	udelay(1);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);

	return ret;
}

int dw_dphy_te_read(struct dw_dphy_rx *dphy, u32 addr)
{
	int ret;

	if (dphy->dphy_te_len == BIT12)
		ret = dw_dphy_te_12b_read(dphy, addr);
	else
		ret = dw_dphy_te_8b_read(dphy, addr);

	return ret;
}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
static void dw_dphy_if_init(struct dw_dphy_rx *dphy)
{
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, TX_PHY);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 0);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 1);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, GLUELOGIC);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 0);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 1);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RX_PHY);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 0);
	dw_dphy_if_write(dphy, DPHYZCALCTRL, 1);
}
#endif

/* dphy_freq is the link frequency in Hz; the tables are indexed in Mbps. */
static int dw_dphy_data_rate_mbps(struct dw_dphy_rx *dphy, u32 *data_rate)
{
	u32 freq = dphy->dphy_freq;
	u32 min_freq = DPHY_MIN_DATA_RATE_MBPS * DPHY_LINK_FREQ_HZ_PER_MBPS;
	u32 max_freq = DPHY_MAX_DATA_RATE_MBPS * DPHY_LINK_FREQ_HZ_PER_MBPS;

	if (freq < min_freq) {
		dev_dbg(&dphy->phy->dev,
			"link frequency %u Hz is below the minimum %u Hz\n",
			freq, min_freq);
		return -EINVAL;
	}

	if (freq > max_freq) {
		dev_dbg(&dphy->phy->dev,
			"link frequency %u Hz exceeds the maximum %u Hz\n",
			freq, max_freq);
		return -ERANGE;
	}

	*data_rate = DIV_ROUND_UP(freq, DPHY_LINK_FREQ_HZ_PER_MBPS);

	return 0;
}

static u8 get_range_dphy_gen3_index_for_data_rate(u32 data_rate)
{
	u8 range = 0;

	for (range = 0; range < (ARRAY_SIZE(range_gen3) - 1); range++) {
		if (data_rate <= range_gen3[range].freq)
			break;
	}

	return range;
}

/* DDL oscillator target: fixed up to the deskew rate, scaled with the rate above it */
static u16 dw_dphy_osc_freq_target(u32 data_rate)
{
	if (data_rate <= DPHY_DESKEW_RATE_MBPS)
		return DPHY_OSC_FREQ_TARGET_MAX;

	return DIV_ROUND_CLOSEST(data_rate * DPHY_OSC_FREQ_TARGET_MAX *
				 DPHY_OSC_FREQ_TARGET_PCT,
				 DPHY_MAX_DATA_RATE_MBPS * 100);
}

static void dw_dphy_gen3_12bit_tc_power_up(struct dw_dphy_rx *dphy)
{
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, GLUELOGIC);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RX_PHY);
#endif
}

static void dw_dphy_gen3_8bit_tc_power_up(struct dw_dphy_rx *dphy,
					  u32 data_rate)
{
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, GLUELOGIC);
	dw_dphy_te_write(dphy, CFGCLKFREQRANGE_RX, 0x1C);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RX_PHY);
#endif
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX0_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX0_LSB, 0x02);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX1_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX1_LSB, 0x02);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX2_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX2_LSB, 0x02);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX3_MSB, 0x03);
	dw_dphy_te_write(dphy, OSC_FREQ_TARGET_RX3_LSB, 0x02);
	dw_dphy_te_write(dphy, BANDGAP_CTRL, 0x80);

	if (data_rate < 2000)
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE0, 0xC0);

	if (data_rate < 1000) {
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE1, 0xC0);
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE2, 0xC0);
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE3, 0xC0);
	}
}

static int dw_dphy_g118_settle(struct dw_dphy_rx *dphy, u32 data_rate)
{
	u32 total_settle, settle_time, byte_clk, lp_time;

	lp_time = dphy->lp_time;

	settle_time = (8 * (1000000 / data_rate)) + 115000;
	byte_clk = 8000000 / data_rate;
	total_settle = (settle_time + lp_time * 1000) / byte_clk;

	if (total_settle > 0xFF)
		total_settle = 0xFF;

	return total_settle;
}

static void dw_dphy_pwr_down(struct dw_dphy_rx *dphy)
{
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);

	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	if (dphy->lanes_config == CTRL_8_LANES)
		dw_dphy_write_msk(dphy, R_CSI2_DPHY2_TST_CTRL0, 0, PHY_TESTCLK,
				  1);

	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
}

static void dw_dphy_pwr_up(struct dw_dphy_rx *dphy)
{
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	if (dphy->lanes_config == CTRL_8_LANES)
		dw_dphy_write_msk(dphy, R_CSI2_DPHY2_TST_CTRL0, 1, PHY_TESTCLK,
				  1);
	dev_vdbg(&dphy->phy->dev, "DPHY power up.\n");
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);
	dw_dphy_read(dphy, R_CSI2_DPHY_SHUTDOWNZ);
	udelay(1);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
}

static int dw_dphy_gen3_12bit_configure(struct dw_dphy_rx *dphy, u32 data_rate)
{
	u16 osc_freq_target = dw_dphy_osc_freq_target(data_rate);
	u32 lane;

	dev_dbg(&dphy->phy->dev,
		"12bit: PHY GEN 3: %u Mbps, osc_freq_target %#x, hs eq %u\n",
		data_rate, osc_freq_target, dphy->hs_eq_strength);

	dw_dphy_gen3_12bit_tc_power_up(dphy);
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_2, (u8)osc_freq_target);
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_3, (u8)(osc_freq_target >> 8));
	/* Arm the DDL target above; optionally force the HS gain mode too. */
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_4,
			 RX_STARTUP_OVR_4_DDL_TARGET_OVR_EN |
			 (dphy->hs_gain_mode >= 0 ?
			  RX_STARTUP_OVR_4_HS_GAIN_OVR_EN |
			  RX_STARTUP_OVR_4_HS_GAIN(dphy->hs_gain_mode) : 0));

	if (dphy->phy_type) {
		dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_1, 0x01);
		dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_0, 0x80);
	}

	if (dphy->phy_type || data_rate <= DPHY_DESKEW_RATE_MBPS)
		dw_dphy_te_write(dphy, RX_SYS_7, 0x20);

	for (lane = 0; lane < min(dphy->lanes, DPHY_RX_MAX_DATA_LANES); lane++)
		dw_dphy_te_write(dphy, RX_LANE_HS_EQ(lane), dphy->hs_eq_strength);

	return 0;
}

static int dw_dphy_gen3_8bit_configure(struct dw_dphy_rx *dphy, u32 data_rate)
{
	u8 data;
	u8 range = get_range_dphy_gen3_index_for_data_rate(data_rate);

	dev_vdbg(&dphy->phy->dev, "8bit: PHY GEN 3: %u Mbps\n", data_rate);
	if (dphy->hs_gain_mode >= 0)
		dev_dbg(&dphy->phy->dev, "rx-hs-gain-mode needs the 12-bit map\n");

	dw_dphy_te_write(dphy, RX_SKEW_CAL,
			 dw_dphy_g118_settle(dphy, data_rate));
	data = 1 << 7 | range_gen3[range].hsfregrange;
	dw_dphy_te_write(dphy, HSFREQRANGE_8BIT, data);
	dw_dphy_gen3_8bit_tc_power_up(dphy, data_rate);

	return 0;
}

static int dw_dphy_gen2_configure(struct dw_dphy_rx *dphy, u32 data_rate)
{
	u8 data;
	u8 range = 0;

	/* HSFREQRANGE_8BIT is an address in the 8-bit test code map only */
	if (dphy->dphy_te_len != BIT8)
		return 0;

	dev_vdbg(&dphy->phy->dev, "PHY GEN 2: %u Mbps\n", data_rate);
	for (range = 0; (range < ARRAY_SIZE(range_gen2) - 1) &&
	     (data_rate > range_gen2[range].freq); range++)
		;

	data = range_gen2[range].hsfregrange << 1;
	dw_dphy_te_write(dphy, HSFREQRANGE_8BIT, data);

	return 0;
}

static void dw_dphy_glue_configure(struct dw_dphy_rx *dphy, u32 data_rate)
{
	u8 index = get_range_dphy_gen3_index_for_data_rate(data_rate);

	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_GENERAL, (range_gen3[index].hsfregrange) & 0xFF, GLUE_CTRL_GEN_HSFR, 7);
	dev_dbg(&dphy->phy->dev,
		"data rate : %u, hsfreqrange : %#x, dphy freq : %u, hs gain mode : %d\n",
		data_rate, range_gen3[index].hsfregrange, dphy->dphy_freq,
		dphy->hs_gain_mode);

	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_GENERAL, 0X41, GLUE_CTRL_GEN_CFG_CLK, 8);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG0, 1, GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG1, 1, GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG2, 1, GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG3, 1, GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG0, 1, GLUE_CTRL_FORCE_MODE, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG1, 1, GLUE_CTRL_FORCE_MODE, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG2, 1, GLUE_CTRL_FORCE_MODE, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG3, 1, GLUE_CTRL_FORCE_MODE, 1);
	udelay(1);
	
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_GENERAL, 1, GLUE_CTRL_GEN_ENBCLK, 1);
	udelay(1);
}

/* active-high test clear pulse, held for at least 15 ns */
static void dw_dphy_test_clear(struct dw_dphy_rx *dphy)
{
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_read(dphy, R_CSI2_DPHY_TST_CTRL0);
	udelay(1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
}

static int dw_dphy_configure(struct dw_dphy_rx *dphy)
{
	u32 data_rate;
	int ret;

	ret = dw_dphy_data_rate_mbps(dphy, &data_rate);
	if (ret)
		return ret;
	/* range_gen2[] is only consulted on the 8-bit test-code map */
	if (dphy->dphy_gen != GEN3 && dphy->dphy_te_len == BIT8 &&
	    data_rate > range_gen2[ARRAY_SIZE(range_gen2) - 1].freq) {
		dev_dbg(&dphy->phy->dev,
			"8-bit data rate %u Mbps exceeds the maximum %u Mbps\n",
			data_rate,
			range_gen2[ARRAY_SIZE(range_gen2) - 1].freq);
		return -ERANGE;
	}

	dw_dphy_pwr_down(dphy);
	dw_dphy_test_clear(dphy);

	if (dphy->dphy_gen == GEN3) {
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
		dw_dphy_if_init(dphy);
#endif
		if (dphy->dphy_te_len == BIT12)
			dw_dphy_gen3_12bit_configure(dphy, data_rate);
		else
			dw_dphy_gen3_8bit_configure(dphy, data_rate);
	} else {
		dw_dphy_gen2_configure(dphy, data_rate);
	}
	dw_dphy_glue_configure(dphy, data_rate);
	dw_dphy_pwr_up(dphy);

	return 0;
}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
static int dw_dphy_if_set_idelay(struct dw_dphy_rx *dphy, u8 dly, u8 cells)
{
	u32 val = 0;

	dw_dphy_if_write(dphy, IDLYCFG, 0);
	dw_dphy_if_write(dphy, IDLYSEL, cells);
	dw_dphy_if_write(dphy, IDLYCNTINVAL, dly);

	/* Pulse Value Set */
	dw_dphy_if_write(dphy, IDLYCFG, 1);
	usleep_range(10, 20);
	dw_dphy_if_write(dphy, IDLYCFG, 0);

	/* Pulse IDELAY CTRL Reset */
	dw_dphy_if_write(dphy, DPHY1REGRSTN, 0);
	usleep_range(10, 20);
	dw_dphy_if_write(dphy, DPHY1REGRSTN, 1);

	/* Get Value*/
	val = dw_dphy_if_read(dphy, IDLYCNTOUTVAL);

	if (val != dly) {
		dev_vdbg(&dphy->phy->dev,
			 "odelay config failed, set %d get %d", dly, val);
		return -EINVAL;
	}

	return 0;
}

int dw_dphy_if_get_idelay(struct dw_dphy_rx *dphy)
{
	return dw_dphy_if_read(dphy, IDLYCNTOUTVAL);
}

int dw_dphy_if_set_idelay_lane(struct dw_dphy_rx *dphy, u8 dly, u8 lane)
{
	int cell;

	switch (lane) {
	case 0:
		for (cell = 3; cell <= 10; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 1:
		for (cell = 14; cell <= 21; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 2:
		for (cell = 24; cell <= 31; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 3:
		for (cell = 34; cell <= 41; cell++)
			dw_dphy_if_set_idelay(dphy, dly, cell);
		break;
	case 4: /* ALL */
		dw_dphy_if_set_idelay(dphy, dly, 0x7F);
		break;
	default:
		dev_dbg(&dphy->phy->dev, "Lane Value not recognized\n");
		return -1;
	}
	return 0;
}
#endif

int dw_dphy_init(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	dev_vdbg(&dphy->phy->dev, "Init DPHY.\n");

	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);

	return 0;
}

static int dw_dphy_set_phy_state(struct dw_dphy_rx *dphy, u32 on)
{
	int ret;

	dphy->lanes_config = dw_dphy_setup_config(dphy);

	if (on) {
		ret = dw_dphy_configure(dphy);
		if (ret)
			return ret;
	} else {
		dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
		dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	}

	return 0;
}

int dw_dphy_power_on(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	return dw_dphy_set_phy_state(dphy, 1);
}

int dw_dphy_power_off(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	return dw_dphy_set_phy_state(dphy, 0);
}

int __dw_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);
	unsigned long hs_clk_rate = opts->mipi_dphy.hs_clk_rate;

	if (hs_clk_rate < DPHY_MIN_DATA_RATE_MBPS * 1000000UL)
		return -EINVAL;
	if (hs_clk_rate > DPHY_MAX_DATA_RATE_MBPS * 1000000UL)
		return -ERANGE;

	dphy->dphy_freq = DIV_ROUND_UP(hs_clk_rate, 2);
	dphy->lanes = opts->mipi_dphy.lanes;

	return 0;
}

int dw_dphy_reset(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	usleep_range(100, 200);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);

	return 0;
}
