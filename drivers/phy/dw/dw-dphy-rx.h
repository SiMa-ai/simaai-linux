/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 Synopsys, Inc.
 *
 * Synopsys MIPI D-PHY controller driver
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#ifndef __PHY_SNPS_DPHY_RX_H__
#define __PHY_SNPS_DPHY_RX_H__

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/string.h>

/* DPHY interface register bank*/
#define R_CSI2_DPHY_SHUTDOWNZ 0x0
#define R_CSI2_DPHY_RSTZ 0x4
#define R_CSI2_DPHY_TST_CTRL0 0x10
#define R_CSI2_DPHY_TST_CTRL1 0x14
#define R_CSI2_DPHY2_TST_CTRL0 0x18
#define R_CSI2_DPHY2_TST_CTRL1 0x1C
#define R_GLUE_DPHY_CTRL_REG0    0x0
#define R_GLUE_DPHY_CTRL_REG1    0x4
#define R_GLUE_DPHY_CTRL_REG2    0x8
#define R_GLUE_DPHY_CTRL_REG3    0xC
#define R_GLUE_DPHY_CTRL_GENERAL 0x10

enum dphy_id_mask {
	DPHY_ID_LANE_SUPPORT = 0,
	DPHY_ID_GEN = 8,
};

enum dphy_gen_values {
	GEN2 = 1,
	GEN3 = 2,
};

enum dphy_interface_length {
	BIT8 = 8,
	BIT12 = 12,
};

enum tst_ctrl0 {
	PHY_TESTCLR = 0,
	PHY_TESTCLK = 1,
};

enum tst_ctrl1 {
	PHY_TESTDIN = 0,
	PHY_TESTDOUT = 8,
	PHY_TESTEN = 16,
};

enum glue_ctrl {
	GLUE_CTRL_FORCE_MODE = 0,
	GLUE_CTRL_GEN_ENBCLK = 2,
	GLUE_CTRL_BASEDIR = 4,
	GLUE_CTRL_GEN_CFG_CLK = 8,
	GLUE_CTRL_GEN_HSFR = 16,
};

enum lanes_config_values {
	CTRL_4_LANES = 0,
	CTRL_8_LANES = 1,
};

enum dphy_8bit_interface_addr {
	HSFREQRANGE_8BIT = 0x44,
};

enum dphy_12bit_interface_addr {
	RX_SYS_1 = 0x02,
	RX_SYS_7 = 0x08,
	RX_RX_STARTUP_OVR_0 = 0xe0,
	RX_RX_STARTUP_OVR_1 = 0xe1,
	RX_RX_STARTUP_OVR_2 = 0xe2,
	RX_RX_STARTUP_OVR_3 = 0xe3,
	RX_RX_STARTUP_OVR_4 = 0xe4,
};
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
/* Gen3 interface register bank */
#define IDLYCFG	0x00
#define IDLYSEL	0x04
#define IDLYCNTINVAL 0x08
#define IDLYCNTOUTVAL 0x0c
#define DPHY1REGRSTN 0x10
#define DPHYID 0x100
#endif

struct dw_dphy_rx {
	struct phy *phy;
	u32 dphy_freq;
	u32 dphy_gen;
	u32 dphy_te_len;
	u32 lanes_config;
	u32 max_lanes;
	u32 lp_time;
	void __iomem *base_address; /* test interface */
	void __iomem *glue_base_address; /* test interface */
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	void __iomem *dphy1_if_addr; /* gluelogic dphy 1 */
	void __iomem *dphy2_if_addr; /* gluelogic dphy 2 */
	int config_gpio;
#endif
	u8 dphy_on;
	struct device *dev;
};

int dw_dphy_init(struct phy *phy);
int dw_dphy_reset(struct phy *phy);
int dw_dphy_power_off(struct phy *phy);
int dw_dphy_power_on(struct phy *phy);
int dw_dphy_configure(struct phy *phy, union phy_configure_opts *opts);
u8 dw_dphy_setup_config(struct dw_dphy_rx *dphy);
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
u32 dw_dphy_if_read(struct dw_dphy_rx *dphy, u32 address);
#endif
void dw_dphy_write(struct dw_dphy_rx *dphy, u32 address, u32 data);
u32 dw_dphy_read(struct dw_dphy_rx *dphy, u32 address);
int dw_dphy_te_read(struct dw_dphy_rx *dphy, u32 addr);
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
int dw_dphy_if_get_idelay(struct dw_dphy_rx *dphy);
int dw_dphy_if_set_idelay_lane(struct dw_dphy_rx *dphy, u8 dly, u8 lane);

static inline
u32 dw_dphy_if_read_msk(struct dw_dphy_rx *dphy,
		u32 address, u8 shift, u8 width)
{
	return (dw_dphy_if_read(dphy, address) >> shift) & ((1 << width) - 1);
}
#endif /*IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3) */

static inline
u32 dw_dphy_read_msk(struct dw_dphy_rx *dev, u32 address, u8 shift,  u8 width)
{
	return (dw_dphy_read(dev, address) >> shift) & ((1 << width) - 1);
}

#endif /*__PHY_SNPS_DPHY_RX_H__*/
