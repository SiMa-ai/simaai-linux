// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Synopsys, Inc.
 *
 * Synopsys MIPI D-PHY controller driver
 * Core functions
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#include "dw-dphy-rx.h"
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/io.h>

static uint8_t data_lane_mask = 0xf;

#define MAX_DPHY_FREQ (1250000000)

struct range_dphy_gen2 {
	u32 freq;
	u8 hsfregrange;
};

struct range_dphy_gen2 range_gen2[] = {
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
	u32 osc_freq_target;
};

struct range_dphy_gen3 range_gen3[] = {

	{ 80, 0x00, 0x1C0 },   { 90, 0x10, 0x1C0 },   { 100, 0x20, 0x1C0 },
	{ 110, 0x30, 0x1C0 },  { 120, 0x01, 0x1C0 },  { 130, 0x11, 0x1C0 },
	{ 140, 0x21, 0x1C0 },  { 150, 0x31, 0x1C0 },  { 160, 0x02, 0x1C0 },
	{ 170, 0x12, 0x1C0 },  { 180, 0x22, 0x1C0 },  { 190, 0x32, 0x1C0 },
	{ 205, 0x03, 0x1C0 },  { 220, 0x13, 0x1C0 },  { 235, 0x23, 0x1C0 },
	{ 250, 0x33, 0x1C0 },  { 275, 0x04, 0x1C0 },  { 300, 0x14, 0x1C0 },
	{ 325, 0x25, 0x1C0 },  { 350, 0x35, 0x1C0 },  { 400, 0x05, 0x1C0 },
	{ 450, 0x16, 0x1C0 },  { 500, 0x26, 0x1C0 },  { 550, 0x37, 0x1C0 },
	{ 600, 0x07, 0x1C0 },  { 650, 0x18, 0x1C0 },  { 700, 0x28, 0x1C0 },
	{ 750, 0x39, 0x1C0 },  { 800, 0x09, 0x1C0 },  { 850, 0x19, 0x1C0 },
	{ 900, 0x29, 0x1C0 },  { 950, 0x3A, 0x1C0 },  { 1000, 0x0A, 0x1C0 },
	{ 1050, 0x1A, 0x1C0 }, { 1100, 0x2A, 0x1C0 }, { 1150, 0x3B, 0x1C0 },
	{ 1200, 0x0B, 0x1C0 }, { 1250, 0x1B, 0x1C0 }, { 1300, 0x2B, 0x1C0 },
	{ 1350, 0x3C, 0x1C0 }, { 1400, 0x0C, 0x1C0 }, { 1450, 0x1C, 0x1C0 },
	{ 1500, 0x2C, 0x1C0 }, { 1550, 0x3D, 0x116 }, { 1600, 0x0D, 0x11F },
	{ 1650, 0x1D, 0x128 }, { 1700, 0x2E, 0x131 }, { 1750, 0x3E, 0x13A },
	{ 1800, 0x0E, 0x143 }, { 1850, 0x1E, 0x14B }, { 1900, 0x2F, 0x154 },
	{ 1950, 0x3F, 0x15D }, { 2000, 0x0F, 0x166 }, { 2050, 0x40, 0x16F },
	{ 2100, 0x41, 0x178 }, { 2150, 0x42, 0x181 }, { 2200, 0x43, 0x18A },
	{ 2250, 0x44, 0x193 }, { 2300, 0x45, 0x19C }, { 2350, 0x46, 0x1A5 },
	{ 2400, 0x47, 0x1AE }, { 2450, 0x48, 0x1B7 }, { 2500, 0x49, 0x1C0 }
};

u8 dw_dphy_setup_config(struct dw_dphy_rx *dphy)
{
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	u8 ret;
	int setup_config;

	if (dphy->max_lanes == CTRL_4_LANES)
#endif
		return CTRL_4_LANES;

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	ret = gpio_request(dphy->config_gpio, "config");
	if (ret < 0) {
		pr_err("could not acquire config gpio (err=%d)\n", ret);
		return ret;
	}

	setup_config = gpio_get_value(dphy->config_gpio);
	pr_debug("CONFIG %s\n", setup_config == CTRL_8_LANES ? "8L" : "4+4L");
	gpio_free(dphy->config_gpio);

	return setup_config;
#endif
}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
void dw_dphy_if_write(struct dw_dphy_rx *dphy, u32 address, u32 data)
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
		pr_debug("Values read different for each interface\n");

end:
	return if1;
}
#endif

void dw_dphy_glue_write(struct dw_dphy_rx *dphy, u32 address, u32 data)
{
	iowrite32(data, dphy->glue_base_address + address);
}
u32 dw_dphy_glue_read(struct dw_dphy_rx *dphy, u32 address)
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
		pr_debug("Values read different for each dphy\n");

end:
	return dphy1;
}
void dw_dphy_glue_write_msk(struct dw_dphy_rx *dev, u32 address,
		u32 data, u8 shift, u8 width)
{
	u32 mask = (1 << width) - 1;
	u32 temp = dw_dphy_glue_read(dev, address);

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	dw_dphy_glue_write(dev, address, temp);
}

void dw_dphy_write_msk(struct dw_dphy_rx *dev, u32 address, u32 data,
		u8 shift, u8 width)
{
	u32 mask = (1 << width) - 1;
	u32 temp = dw_dphy_read(dev, address);

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	dw_dphy_write(dev, address, temp);
}

static void dw_dphy_te_12b_write(struct dw_dphy_rx *dphy, u16 addr,
		u8 data)
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
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);

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
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);

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

static void dw_dphy_gen3_12bit_tc_power_up(struct dw_dphy_rx *dphy)
{
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, GLUELOGIC);
#endif
	dw_dphy_te_write(dphy, CFGCLKFREQRANGE_TX, 0x1C);

	/* CLKSEL | UPDATEPLL | SHADOW_CLEAR | SHADOW_CTRL | FORCEPLL */
	dw_dphy_te_write(dphy, BYPASS, 0x3F);

	/* IO_DS3 | IO_DS2 | IO_DS1 | IO_DS0 */
	if (dphy->dphy_freq > 1500)
		dw_dphy_te_write(dphy, IO_DS, 0x0F);
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RESET);
	dw_dphy_if_write(dphy, DPHYGLUEIFTESTER, RX_PHY);
#endif
}

static void dw_dphy_gen3_8bit_tc_power_up(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq / 1000;
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

	if (input_freq < 2000)
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE0, 0xC0);

	if (input_freq < 1000) {
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE1, 0xC0);
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE2, 0xC0);
		dw_dphy_te_write(dphy, HS_RX_CTRL_LANE3, 0xC0);
	}
}

int dw_dphy_g118_settle(struct dw_dphy_rx *dphy)
{
	u32 input_freq, total_settle, settle_time, byte_clk, lp_time;

	lp_time = dphy->lp_time;
	input_freq = dphy->dphy_freq / 1000;

	settle_time = (8 * (1000000 / (input_freq))) + 115000;
	byte_clk = (8000000 / (input_freq));
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
		dw_dphy_write_msk(dphy, R_CSI2_DPHY2_TST_CTRL0, 0, PHY_TESTCLK, 1);

	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
}

int poll_csi_phy_stopstate_thread(void *arg)
{
	struct dw_dphy_rx *dphy = (struct dw_dphy_rx *)arg;

	const uint32_t phy_stopstateclk    = (1 << 16);

	const uint32_t stopstate_poll_mask = data_lane_mask | phy_stopstateclk;

	dev_info(dphy->dev, "THREAD : Wait until stopstatedata_n and stopstateclk are asserted %#x\n",
			stopstate_poll_mask);

	while (!kthread_should_stop()) {
		if ((dw_dphy_read(dphy, R_CSI2_DPHY_STOPSTATE)
				& stopstate_poll_mask) != stopstate_poll_mask) {
			ndelay(100);//delay 100ns
		} else {

			dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG0, 0, GLUE_CTRL_FORCE_MODE, 1);
			dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG1, 0, GLUE_CTRL_FORCE_MODE, 1);
			dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG2, 0, GLUE_CTRL_FORCE_MODE, 1);
			dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG3, 0, GLUE_CTRL_FORCE_MODE, 1);
			dphy->dphy_on = false;

			return 0;
		}
	}

	dphy->dphy_on = false;
	dev_info(dphy->dev, "THREAD : stopped !!!!\n");
	return 0;
}

static u8 get_range_dphy_gen3_index_for_data_rate(u32 data_rate)
{
	u8 data;
	u8 range = 0;

	for (range = 0; range < (ARRAY_SIZE(range_gen3) - 1); range++) {
		if (data_rate <= range_gen3[range].freq) {
			break;
		}
	}

	if (range == ARRAY_SIZE(range_gen3)) {
		pr_err("Failed to do look up for data rate %u\n", data_rate);
		return 0;
	}

	return range;
}

static void __dw_dphy_configure(struct dw_dphy_rx *dphy)
{
	u8 index = 0;
	u32 data_rate = 0; // in Mbps

	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	ndelay(15);//delay 15ns
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);

	if (dphy->dphy_freq == 0) {
		dev_err(dphy->dev, "dphy frequency 0. Consult documentation for correct value\n");
		data_rate = 0;
	} else if (dphy->dphy_freq > MAX_DPHY_FREQ) {
		dev_err(dphy->dev, "dphy frequency %#x is out of range, truncating it to max %#x\n",
					dphy->dphy_freq, MAX_DPHY_FREQ);
		data_rate = ((MAX_DPHY_FREQ * 2) / 1000000);
	} else {
		data_rate = ((dphy->dphy_freq * 2) / 1000000);
	}

	index = get_range_dphy_gen3_index_for_data_rate(data_rate);
	dev_info(dphy->dev, "data rate : %u, hsfreqrange : %#x, osc_freq_target : %#x, dphy freq : %u\n",
		data_rate, range_gen3[index].hsfregrange, range_gen3[index].osc_freq_target, dphy->dphy_freq);

	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_GENERAL, (range_gen3[index].hsfregrange) & 0xFF, GLUE_CTRL_GEN_HSFR, 7);

	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0xe2,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, (range_gen3[index].osc_freq_target) & 0xFF,
			PHY_TESTDIN, 8);

	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	ndelay(10);//delay 10ns

	//write 0xe3 -> 'h1
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0xe3,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x01,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	ndelay(10);//delay 10ns

	//write 0xe4 -> 'h1
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0xe4,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x01,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
			PHY_TESTDIN, 8);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
	ndelay(10);//delay 10ns

	if (data_rate <= 1500) {
		//step 8
		//write 0x08 -> 'h20
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x08,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x20,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		ndelay(10);//delay 10ns
	}

	//step 9
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_GENERAL,
			0X41, GLUE_CTRL_GEN_CFG_CLK, 8);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG0, 1,
			GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG1, 1,
			GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG2, 1,
			GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG3, 1,
			GLUE_CTRL_BASEDIR, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG0, 1,
			GLUE_CTRL_FORCE_MODE, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG1, 1,
			GLUE_CTRL_FORCE_MODE, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG2, 1,
			GLUE_CTRL_FORCE_MODE, 1);
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_REG3, 1,
			GLUE_CTRL_FORCE_MODE, 1);
	ndelay(15);//delay 15ns
	dw_dphy_glue_write_msk(dphy, R_GLUE_DPHY_CTRL_GENERAL, 1,
			GLUE_CTRL_GEN_ENBCLK, 1);
	ndelay(5);//delay 5ns

	if (data_rate > 1000 && data_rate <= 1500) {
		//step 17
		//write 0xe0 -> 'h80
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0xe0,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x80,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		ndelay(5);//delay 5ns

		//write 0xe1 -> 'h1
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0xe1,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x01,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 1, PHY_TESTEN, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL1, 0x00,
				PHY_TESTDIN, 8);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLK, 1);
		dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);
		ndelay(5);//delay 5ns
	}

	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
	ndelay(5); //delay 5ns
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);

	dphy->poll_thread = kthread_run(poll_csi_phy_stopstate_thread, (void *)dphy, "poll_thread");
	if (IS_ERR(dphy->poll_thread)) {
		dev_err(dphy->dev, "Failed to create poll thread\n");
	} else {
		dphy->dphy_on = true;
		dev_info(dphy->dev, "Poll Thread created successfully\n");
	}
}
static void dw_dphy_pwr_up(struct dw_dphy_rx *dphy)
{
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLK, 1);
	if (dphy->lanes_config == CTRL_8_LANES)
		dw_dphy_write_msk(dphy, R_CSI2_DPHY2_TST_CTRL0, 1,
				PHY_TESTCLK, 1);

	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 1);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);
}

static int dw_dphy_gen3_12bit_configure(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq;
	u8 range = 0;

	pr_debug("12bit: PHY GEN 3: Freq: %u\n", input_freq);
	for (range = 0; (range < ARRAY_SIZE(range_gen3) - 1) &&
			((input_freq / 1000) > range_gen3[range].freq);
	     range++)
		;
	dw_dphy_gen3_12bit_tc_power_up(dphy);
	dw_dphy_te_write(dphy, RX_SYS_1, range_gen3[range].hsfregrange);
	dw_dphy_te_write(dphy, RX_SYS_0, 0x20);
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_2,
			 (u8)range_gen3[range].osc_freq_target);
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_3,
			 (u8)(range_gen3[range].osc_freq_target >> 8));
	dw_dphy_te_write(dphy, RX_RX_STARTUP_OVR_4, 0x01);

	return 0;
}

static int dw_dphy_gen3_8bit_configure(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq;
	u8 data;
	u8 range = 0;

	pr_debug("8bit: PHY GEN 3: Freq: %u\n", input_freq);
	for (range = 0; (range < ARRAY_SIZE(range_gen3) - 1) &&
			((input_freq / 1000) > range_gen3[range].freq);
	     range++)
		;

	dw_dphy_te_write(dphy, RX_SKEW_CAL, dw_dphy_g118_settle(dphy));
	data = 1 << 7 | range_gen3[range].hsfregrange;
	dw_dphy_te_write(dphy, HSFREQRANGE_8BIT, data);
	dw_dphy_gen3_8bit_tc_power_up(dphy);

	return 0;
}

static int dw_dphy_gen2_configure(struct dw_dphy_rx *dphy)
{
	u32 input_freq = dphy->dphy_freq;
	u8 data;
	u8 range = 0;

	/* provide an initial active-high test clear pulse in TESTCLR  */
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 1, PHY_TESTCLR, 1);
	dw_dphy_write_msk(dphy, R_CSI2_DPHY_TST_CTRL0, 0, PHY_TESTCLR, 1);

	pr_debug("PHY GEN 2: Freq: %u\n", input_freq);
	for (range = 0; (range < ARRAY_SIZE(range_gen2) - 1) &&
			((input_freq / 1000) > range_gen2[range].freq);
	     range++)
		;

	data = range_gen2[range].hsfregrange << 1;
	dw_dphy_te_write(dphy, HSFREQRANGE_8BIT, data);

	return 0;
}

static int dw_dphy_configure(struct dw_dphy_rx *dphy)
{
	if (!dphy->dphy_on) {
		__dw_dphy_configure(dphy);
	} else {
		dev_info(dphy->dev, "poll thread is already running skipping configuring\n");
	}

	return 0;
}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
int dw_dphy_if_set_idelay(struct dw_dphy_rx *dphy, u8 dly, u8 cells)
{
	uint32_t val = 0;

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
		pr_debug("odelay config failed, set %d get %d", dly, val);
		return -1;
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
		pr_err("Lane Value not recognized\n");
		return -1;
	}
	return 0;
}
#endif

int dw_dphy_init(struct phy *phy)
{
	struct dw_dphy_rx *dphy = phy_get_drvdata(phy);

	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);

	return 0;
}

static int dw_dphy_set_phy_state(struct dw_dphy_rx *dphy, u32 on)
{
	u8 hs_freq;

	dphy->lanes_config = dw_dphy_setup_config(dphy);

	if (dphy->dphy_te_len == BIT12)
		hs_freq = RX_SYS_1;
	else
		hs_freq = HSFREQRANGE_8BIT;

	if (on) {
		dw_dphy_configure(dphy);
		pr_debug("HS Code: 0X%x\n", dw_dphy_te_read(dphy, hs_freq));
	} else {
		dw_dphy_write(dphy, R_CSI2_DPHY_SHUTDOWNZ, 0);
		dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
		if (dphy->dphy_on) {
			kthread_stop(dphy->poll_thread);
		}
		dphy->dphy_on = false;
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

int dw_dphy_config(struct phy *phy, union phy_configure_opts *opts)
{
	switch(opts->mipi_dphy.lanes) {
		case 1:
			data_lane_mask = 0x1;
			break;
		case 2:
			data_lane_mask = 0x3;
			break;
		case 3:
			data_lane_mask = 0x7;
			break;
		case 4:
			data_lane_mask = 0xf;
			break;
		default:
			return 1;
	}
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
