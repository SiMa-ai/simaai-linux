// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Synopsys, Inc. and/or its affiliates.
 * Synopsys DesignWare XPCS helpers
 *
 * Author: Jose Abreu <Jose.Abreu@synopsys.com>
 */

#include <linux/delay.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/mdio.h>
#include <linux/phylink.h>
#include <linux/workqueue.h>
#include "pcs-xpcs.h"

#define phylink_pcs_to_xpcs(pl_pcs) \
	container_of((pl_pcs), struct dw_xpcs, pcs)

static const int xpcs_usxgmii_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_1000baseKX_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
	ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
	ETHTOOL_LINK_MODE_10baseT_Half_BIT,
	ETHTOOL_LINK_MODE_10baseT_Full_BIT,
	ETHTOOL_LINK_MODE_100baseT_Half_BIT,
	ETHTOOL_LINK_MODE_100baseT_Full_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
	ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
	ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const int xpcs_10gkr_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const int xpcs_xlgmii_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_25000baseCR_Full_BIT,
	ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
	ETHTOOL_LINK_MODE_25000baseSR_Full_BIT,
	ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT,
	ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT,
	ETHTOOL_LINK_MODE_40000baseSR4_Full_BIT,
	ETHTOOL_LINK_MODE_40000baseLR4_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseCR2_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseKR2_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseSR2_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseKR_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseSR_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseCR_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseLR_ER_FR_Full_BIT,
	ETHTOOL_LINK_MODE_50000baseDR_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseSR4_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseLR4_ER4_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseKR2_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseSR2_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseCR2_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseLR2_ER2_FR2_Full_BIT,
	ETHTOOL_LINK_MODE_100000baseDR2_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const int xpcs_sgmii_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_10baseT_Half_BIT,
	ETHTOOL_LINK_MODE_10baseT_Full_BIT,
	ETHTOOL_LINK_MODE_100baseT_Half_BIT,
	ETHTOOL_LINK_MODE_100baseT_Full_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
	ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const int xpcs_1000basex_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_1000baseX_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const int xpcs_2500basex_features[] = {
	ETHTOOL_LINK_MODE_Pause_BIT,
	ETHTOOL_LINK_MODE_Asym_Pause_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
	ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
	__ETHTOOL_LINK_MODE_MASK_NBITS,
};

static const phy_interface_t xpcs_usxgmii_interfaces[] = {
	PHY_INTERFACE_MODE_USXGMII,
};

static const phy_interface_t xpcs_10gkr_interfaces[] = {
	PHY_INTERFACE_MODE_10GKR,
};

static const phy_interface_t xpcs_xlgmii_interfaces[] = {
	PHY_INTERFACE_MODE_XGMII,
};

static const phy_interface_t xpcs_sgmii_interfaces[] = {
	PHY_INTERFACE_MODE_SGMII,
};

static const phy_interface_t xpcs_1000basex_interfaces[] = {
	PHY_INTERFACE_MODE_1000BASEX,
};

static const phy_interface_t xpcs_2500basex_interfaces[] = {
	PHY_INTERFACE_MODE_2500BASEX,
	PHY_INTERFACE_MODE_MAX,
};

enum {
	DW_XPCS_USXGMII,
	DW_XPCS_10GKR,
	DW_XPCS_XLGMII,
	DW_XPCS_SGMII,
	DW_XPCS_1000BASEX,
	DW_XPCS_2500BASEX,
	DW_XPCS_INTERFACE_MAX,
};

struct xpcs_compat {
	const int *supported;
	const phy_interface_t *interface;
	int num_interfaces;
	int an_mode;
	int (*pma_config)(struct dw_xpcs *xpcs);
};

struct xpcs_id {
	u32 id;
	u32 mask;
	const struct xpcs_compat *compat;
};

static const struct xpcs_compat *xpcs_find_compat(const struct xpcs_id *id,
						  phy_interface_t interface)
{
	int i, j;

	for (i = 0; i < DW_XPCS_INTERFACE_MAX; i++) {
		const struct xpcs_compat *compat = &id->compat[i];

		for (j = 0; j < compat->num_interfaces; j++)
			if (compat->interface[j] == interface)
				return compat;
	}

	return NULL;
}

int xpcs_get_an_mode(struct dw_xpcs *xpcs, phy_interface_t interface)
{
	const struct xpcs_compat *compat;

	compat = xpcs_find_compat(xpcs->id, interface);
	if (!compat)
		return -ENODEV;

	return compat->an_mode;
}
EXPORT_SYMBOL_GPL(xpcs_get_an_mode);

static bool __xpcs_linkmode_supported(const struct xpcs_compat *compat,
				      enum ethtool_link_mode_bit_indices linkmode)
{
	int i;

	for (i = 0; compat->supported[i] != __ETHTOOL_LINK_MODE_MASK_NBITS; i++)
		if (compat->supported[i] == linkmode)
			return true;

	return false;
}

#define xpcs_linkmode_supported(compat, mode) \
	__xpcs_linkmode_supported(compat, ETHTOOL_LINK_MODE_ ## mode ## _BIT)

static u32 _mdiobus_c45_addr(int dev, u32 reg)
{
	u32 ret = reg << 2;

	switch(dev) {
	case MDIO_MMD_PCS:
		ret += DW_MMD_PCS_OFFSET;
		break;
	case MDIO_MMD_VEND2:
		ret += DW_MMD_VEND2_OFFSET;
		break;
	case MDIO_MMD_VEND1:
		ret += DW_MMD_VEND1_OFFSET;
		break;
	case MDIO_MMD_AN:
		ret += DW_MMD_AN_OFFSET;
		break;
	default:
		ret += 0;
	}

	return ret;
}

int xpcs_read(struct dw_xpcs *xpcs, int dev, u32 reg)
{
	u32 reg_addr = _mdiobus_c45_addr(dev, reg);
	int ret = (int)(readl(xpcs->addr + reg_addr));

	return ret;
}

int xpcs_write(struct dw_xpcs *xpcs, int dev, u32 reg, u16 val)
{
	u32 reg_addr = _mdiobus_c45_addr(dev, reg);

	writel((u32)val, xpcs->addr + reg_addr);

	return 0;
}

static int xpcs_modify_changed(struct dw_xpcs *xpcs, int dev, u32 reg,
			       u16 mask, u16 set)
{
	u32 reg_addr = _mdiobus_c45_addr(dev, reg);
	int ret = (int)(readl(xpcs->addr + reg_addr));

	ret |= mask & set;
	ret &= (~mask) | set;
	writel((u32)ret, xpcs->addr + reg_addr);

	return ret;
}

static int xpcs_read_vendor(struct dw_xpcs *xpcs, int dev, u32 reg)
{
	return xpcs_read(xpcs, dev, DW_VENDOR | reg);
}

static int xpcs_write_vendor(struct dw_xpcs *xpcs, int dev, int reg,
			     u16 val)
{
	return xpcs_write(xpcs, dev, DW_VENDOR | reg, val);
}

int xpcs_read_vpcs(struct dw_xpcs *xpcs, int reg)
{
	return xpcs_read_vendor(xpcs, MDIO_MMD_PCS, reg);
}

int xpcs_write_vpcs(struct dw_xpcs *xpcs, int reg, u16 val)
{
	return xpcs_write_vendor(xpcs, MDIO_MMD_PCS, reg, val);
}

static int xpcs_poll_reset(struct dw_xpcs *xpcs, int dev)
{
	/* Poll until the reset bit clears (50ms per retry == 0.6 sec) */
	unsigned int retries = 12;
	int ret;

	do {
		msleep(50);
		ret = xpcs_read(xpcs, dev, MDIO_CTRL1);
		if (ret < 0)
			return ret;
	} while (ret & MDIO_CTRL1_RESET && --retries);

	return (ret & MDIO_CTRL1_RESET) ? -ETIMEDOUT : 0;
}

static int xpcs_soft_reset(struct dw_xpcs *xpcs,
			   const struct xpcs_compat *compat)
{
	int ret, dev;

	switch (compat->an_mode) {
	case DW_AN_C73:
		dev = MDIO_MMD_PCS;
		break;
	case DW_AN_C37_SGMII:
	case DW_AN_C37_USXGMII:
	case DW_2500BASEX:
	case DW_AN_C37_1000BASEX:
		dev = MDIO_MMD_VEND1;
		break;
	default:
		return -1;
	}

	if(!xpcs->skip_reset) {
		ret = xpcs_write(xpcs, dev, MDIO_CTRL1, MDIO_CTRL1_RESET);
		if (ret < 0)
			return ret;
	}

	return xpcs_poll_reset(xpcs, dev);
}

#define xpcs_warn(__xpcs, __state, __args...) \
({ \
	if ((__state)->link) \
		pr_warn(__args); \
})

static int xpcs_read_fault_c73(struct dw_xpcs *xpcs,
			       struct phylink_link_state *state)
{
	int ret;

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MDIO_STAT1);
	if (ret < 0)
		return ret;

	if (ret & MDIO_STAT1_FAULT) {
		xpcs_warn(xpcs, state, "Link fault condition detected!\n");
		return -EFAULT;
	}

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MDIO_STAT2);
	if (ret < 0)
		return ret;

	if (ret & MDIO_STAT2_RXFAULT)
		xpcs_warn(xpcs, state, "Receiver fault detected!\n");
	if (ret & MDIO_STAT2_TXFAULT)
		xpcs_warn(xpcs, state, "Transmitter fault detected!\n");

	ret = xpcs_read_vendor(xpcs, MDIO_MMD_PCS, DW_VR_XS_PCS_DIG_STS);
	if (ret < 0)
		return ret;

	if (ret & DW_RXFIFO_ERR) {
		xpcs_warn(xpcs, state, "FIFO fault condition detected!\n");
		return -EFAULT;
	}

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MDIO_PCS_10GBRT_STAT1);
	if (ret < 0)
		return ret;

	if (!(ret & MDIO_PCS_10GBRT_STAT1_BLKLK))
		xpcs_warn(xpcs, state, "Link is not locked!\n");

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MDIO_PCS_10GBRT_STAT2);
	if (ret < 0)
		return ret;

	if (ret & MDIO_PCS_10GBRT_STAT2_ERR) {
		xpcs_warn(xpcs, state, "Link has errors!\n");
		return -EFAULT;
	}

	return 0;
}

static int xpcs_read_link_c73(struct dw_xpcs *xpcs, bool an)
{
	bool link = true;
	int ret;

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MDIO_STAT1);
	if (ret < 0)
		return ret;

	if (!(ret & MDIO_STAT1_LSTATUS))
		link = false;

	if (an) {
		ret = xpcs_read(xpcs, MDIO_MMD_AN, MDIO_STAT1);
		if (ret < 0)
			return ret;

		if (!(ret & MDIO_STAT1_LSTATUS))
			link = false;
	}

	return link;
}

static int xpcs_get_max_usxgmii_speed(const unsigned long *supported)
{
	int max = SPEED_UNKNOWN;

	if (phylink_test(supported, 1000baseKX_Full))
		max = SPEED_1000;
	if (phylink_test(supported, 2500baseX_Full))
		max = SPEED_2500;
	if (phylink_test(supported, 10000baseKX4_Full))
		max = SPEED_10000;
	if (phylink_test(supported, 10000baseKR_Full))
		max = SPEED_10000;

	return max;
}

static void xpcs_config_usxgmii(struct dw_xpcs *xpcs, int speed)
{
	int ret, speed_sel;

	switch (speed) {
	case SPEED_10:
		speed_sel = DW_USXGMII_10;
		break;
	case SPEED_100:
		speed_sel = DW_USXGMII_100;
		break;
	case SPEED_1000:
		speed_sel = DW_USXGMII_1000;
		break;
	case SPEED_2500:
		speed_sel = DW_USXGMII_2500;
		break;
	case SPEED_5000:
		speed_sel = DW_USXGMII_5000;
		break;
	case SPEED_10000:
		speed_sel = DW_USXGMII_10000;
		break;
	default:
		/* Nothing to do here */
		return;
	}

	ret = xpcs_read_vpcs(xpcs, MDIO_CTRL1);
	if (ret < 0)
		goto out;

	ret = xpcs_write_vpcs(xpcs, MDIO_CTRL1, ret | DW_USXGMII_EN);
	if (ret < 0)
		goto out;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, MDIO_CTRL1);
	if (ret < 0)
		goto out;

	ret &= ~DW_USXGMII_SS_MASK;
	ret |= speed_sel | DW_USXGMII_FULL;

	ret = xpcs_write(xpcs, MDIO_MMD_VEND2, MDIO_CTRL1, ret);
	if (ret < 0)
		goto out;

	ret = xpcs_read_vpcs(xpcs, MDIO_CTRL1);
	if (ret < 0)
		goto out;

	ret = xpcs_write_vpcs(xpcs, MDIO_CTRL1, ret | DW_USXGMII_RST);
	if (ret < 0)
		goto out;

	return;

out:
	pr_err("%s: XPCS access returned %pe\n", __func__, ERR_PTR(ret));
}

static int _xpcs_config_aneg_c73(struct dw_xpcs *xpcs,
				 const struct xpcs_compat *compat)
{
	int ret, adv;

	/* By default, in USXGMII mode XPCS operates at 10G baud and
	 * replicates data to achieve lower speeds. Hereby, in this
	 * default configuration we need to advertise all supported
	 * modes and not only the ones we want to use.
	 */

	/* SR_AN_ADV3 */
	adv = 0;
	if (xpcs_linkmode_supported(compat, 2500baseX_Full))
		adv |= DW_C73_2500KX;

	/* TODO: 5000baseKR */

	ret = xpcs_write(xpcs, MDIO_MMD_AN, DW_SR_AN_ADV3, adv);
	if (ret < 0)
		return ret;

	/* SR_AN_ADV2 */
	adv = 0;
	if (xpcs_linkmode_supported(compat, 1000baseKX_Full))
		adv |= DW_C73_1000KX;
	if (xpcs_linkmode_supported(compat, 10000baseKX4_Full))
		adv |= DW_C73_10000KX4;
	if (xpcs_linkmode_supported(compat, 10000baseKR_Full))
		adv |= DW_C73_10000KR;

	ret = xpcs_write(xpcs, MDIO_MMD_AN, DW_SR_AN_ADV2, adv);
	if (ret < 0)
		return ret;

	/* SR_AN_ADV1 */
	adv = DW_C73_AN_ADV_SF;
	if (xpcs_linkmode_supported(compat, Pause))
		adv |= DW_C73_PAUSE;
	if (xpcs_linkmode_supported(compat, Asym_Pause))
		adv |= DW_C73_ASYM_PAUSE;

	return xpcs_write(xpcs, MDIO_MMD_AN, DW_SR_AN_ADV1, adv);
}

static int xpcs_config_aneg_c73(struct dw_xpcs *xpcs,
				const struct xpcs_compat *compat)
{
	int ret;

	ret = _xpcs_config_aneg_c73(xpcs, compat);
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_AN, MDIO_CTRL1);
	if (ret < 0)
		return ret;

	ret |= MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART;

	return xpcs_write(xpcs, MDIO_MMD_AN, MDIO_CTRL1, ret);
}

static int xpcs_aneg_done_c73(struct dw_xpcs *xpcs,
			      struct phylink_link_state *state,
			      const struct xpcs_compat *compat)
{
	int ret;

	ret = xpcs_read(xpcs, MDIO_MMD_AN, MDIO_STAT1);
	if (ret < 0)
		return ret;

	if (ret & MDIO_AN_STAT1_COMPLETE) {
		ret = xpcs_read(xpcs, MDIO_MMD_AN, DW_SR_AN_LP_ABL1);
		if (ret < 0)
			return ret;

		/* Check if Aneg outcome is valid */
		if (!(ret & DW_C73_AN_ADV_SF)) {
			xpcs_config_aneg_c73(xpcs, compat);
			return 0;
		}

		return 1;
	}

	return 0;
}

static int xpcs_read_lpa_c73(struct dw_xpcs *xpcs,
			     struct phylink_link_state *state)
{
	int ret;

	ret = xpcs_read(xpcs, MDIO_MMD_AN, MDIO_STAT1);
	if (ret < 0)
		return ret;

	if (!(ret & MDIO_AN_STAT1_LPABLE)) {
		phylink_clear(state->lp_advertising, Autoneg);
		return 0;
	}

	phylink_set(state->lp_advertising, Autoneg);

	/* Clause 73 outcome */
	ret = xpcs_read(xpcs, MDIO_MMD_AN, DW_SR_AN_LP_ABL3);
	if (ret < 0)
		return ret;

	if (ret & DW_C73_2500KX)
		phylink_set(state->lp_advertising, 2500baseX_Full);

	ret = xpcs_read(xpcs, MDIO_MMD_AN, DW_SR_AN_LP_ABL2);
	if (ret < 0)
		return ret;

	if (ret & DW_C73_1000KX)
		phylink_set(state->lp_advertising, 1000baseKX_Full);
	if (ret & DW_C73_10000KX4)
		phylink_set(state->lp_advertising, 10000baseKX4_Full);
	if (ret & DW_C73_10000KR)
		phylink_set(state->lp_advertising, 10000baseKR_Full);

	ret = xpcs_read(xpcs, MDIO_MMD_AN, DW_SR_AN_LP_ABL1);
	if (ret < 0)
		return ret;

	if (ret & DW_C73_PAUSE)
		phylink_set(state->lp_advertising, Pause);
	if (ret & DW_C73_ASYM_PAUSE)
		phylink_set(state->lp_advertising, Asym_Pause);

	linkmode_and(state->lp_advertising, state->lp_advertising,
		     state->advertising);
	return 0;
}

static void xpcs_resolve_lpa_c73(struct dw_xpcs *xpcs,
				 struct phylink_link_state *state)
{
	int max_speed = xpcs_get_max_usxgmii_speed(state->lp_advertising);

	state->pause = MLO_PAUSE_TX | MLO_PAUSE_RX;
	state->speed = max_speed;
	state->duplex = DUPLEX_FULL;
}

static int xpcs_get_max_xlgmii_speed(struct dw_xpcs *xpcs,
				     struct phylink_link_state *state)
{
	unsigned long *adv = state->advertising;
	int speed = SPEED_UNKNOWN;
	int bit;

	for_each_set_bit(bit, adv, __ETHTOOL_LINK_MODE_MASK_NBITS) {
		int new_speed = SPEED_UNKNOWN;

		switch (bit) {
		case ETHTOOL_LINK_MODE_25000baseCR_Full_BIT:
		case ETHTOOL_LINK_MODE_25000baseKR_Full_BIT:
		case ETHTOOL_LINK_MODE_25000baseSR_Full_BIT:
			new_speed = SPEED_25000;
			break;
		case ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT:
		case ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT:
		case ETHTOOL_LINK_MODE_40000baseSR4_Full_BIT:
		case ETHTOOL_LINK_MODE_40000baseLR4_Full_BIT:
			new_speed = SPEED_40000;
			break;
		case ETHTOOL_LINK_MODE_50000baseCR2_Full_BIT:
		case ETHTOOL_LINK_MODE_50000baseKR2_Full_BIT:
		case ETHTOOL_LINK_MODE_50000baseSR2_Full_BIT:
		case ETHTOOL_LINK_MODE_50000baseKR_Full_BIT:
		case ETHTOOL_LINK_MODE_50000baseSR_Full_BIT:
		case ETHTOOL_LINK_MODE_50000baseCR_Full_BIT:
		case ETHTOOL_LINK_MODE_50000baseLR_ER_FR_Full_BIT:
		case ETHTOOL_LINK_MODE_50000baseDR_Full_BIT:
			new_speed = SPEED_50000;
			break;
		case ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseSR4_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseLR4_ER4_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseKR2_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseSR2_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseCR2_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseLR2_ER2_FR2_Full_BIT:
		case ETHTOOL_LINK_MODE_100000baseDR2_Full_BIT:
			new_speed = SPEED_100000;
			break;
		default:
			continue;
		}

		if (new_speed > speed)
			speed = new_speed;
	}

	return speed;
}

static void xpcs_resolve_pma(struct dw_xpcs *xpcs,
			     struct phylink_link_state *state)
{
	state->pause = MLO_PAUSE_TX | MLO_PAUSE_RX;
	state->duplex = DUPLEX_FULL;

	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GKR:
		state->speed = SPEED_10000;
		break;
	case PHY_INTERFACE_MODE_XGMII:
		state->speed = xpcs_get_max_xlgmii_speed(xpcs, state);
		break;
	default:
		state->speed = SPEED_UNKNOWN;
		break;
	}
}

static int xpcs_validate(struct phylink_pcs *pcs, unsigned long *supported,
			 const struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(xpcs_supported) = { 0, };
	const struct xpcs_compat *compat;
	struct dw_xpcs *xpcs;
	int i;

	xpcs = phylink_pcs_to_xpcs(pcs);
	compat = xpcs_find_compat(xpcs->id, state->interface);

	/* Populate the supported link modes for this PHY interface type.
	 * FIXME: what about the port modes and autoneg bit? This masks
	 * all those away.
	 */
	if (compat)
		for (i = 0; compat->supported[i] != __ETHTOOL_LINK_MODE_MASK_NBITS; i++)
			set_bit(compat->supported[i], xpcs_supported);

	linkmode_and(supported, supported, xpcs_supported);

	return 0;
}

void xpcs_get_interfaces(struct dw_xpcs *xpcs, unsigned long *interfaces)
{
	int i, j;

	for (i = 0; i < DW_XPCS_INTERFACE_MAX; i++) {
		const struct xpcs_compat *compat = &xpcs->id->compat[i];

		for (j = 0; j < compat->num_interfaces; j++)
			if (compat->interface[j] < PHY_INTERFACE_MODE_MAX)
				__set_bit(compat->interface[j], interfaces);
	}
}
EXPORT_SYMBOL_GPL(xpcs_get_interfaces);

int xpcs_config_eee(struct dw_xpcs *xpcs, int mult_fact_100ns, int enable)
{
	int ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, DW_VR_MII_EEE_MCTRL0);
	if (ret < 0)
		return ret;

	if (enable) {
	/* Enable EEE */
		ret = DW_VR_MII_EEE_LTX_EN | DW_VR_MII_EEE_LRX_EN |
		      DW_VR_MII_EEE_TX_QUIET_EN | DW_VR_MII_EEE_RX_QUIET_EN |
		      DW_VR_MII_EEE_TX_EN_CTRL | DW_VR_MII_EEE_RX_EN_CTRL |
		      mult_fact_100ns << DW_VR_MII_EEE_MULT_FACT_100NS_SHIFT;
	} else {
		ret &= ~(DW_VR_MII_EEE_LTX_EN | DW_VR_MII_EEE_LRX_EN |
		       DW_VR_MII_EEE_TX_QUIET_EN | DW_VR_MII_EEE_RX_QUIET_EN |
		       DW_VR_MII_EEE_TX_EN_CTRL | DW_VR_MII_EEE_RX_EN_CTRL |
		       DW_VR_MII_EEE_MULT_FACT_100NS);
	}

	ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_EEE_MCTRL0, ret);
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, DW_VR_MII_EEE_MCTRL1);
	if (ret < 0)
		return ret;

	if (enable)
		ret |= DW_VR_MII_EEE_TRN_LPI;
	else
		ret &= ~DW_VR_MII_EEE_TRN_LPI;

	return xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_EEE_MCTRL1, ret);
}
EXPORT_SYMBOL_GPL(xpcs_config_eee);

static int xpcs_config_aneg_c37_sgmii(struct dw_xpcs *xpcs, unsigned int mode)
{
	int ret, mdio_ctrl;

	/* For AN for C37 SGMII mode, the settings are :-
	 * 1) VR_MII_MMD_CTRL Bit(12) [AN_ENABLE] = 0b (Disable SGMII AN in case
	      it is already enabled)
	 * 2) VR_MII_AN_CTRL Bit(2:1)[PCS_MODE] = 10b (SGMII AN)
	 * 3) VR_MII_AN_CTRL Bit(3) [TX_CONFIG] = 0b (MAC side SGMII)
	 *    DW xPCS used with DW EQoS MAC is always MAC side SGMII.
	 * 4) VR_MII_DIG_CTRL1 Bit(9) [MAC_AUTO_SW] = 1b (Automatic
	 *    speed/duplex mode change by HW after SGMII AN complete)
	 * 5) VR_MII_MMD_CTRL Bit(12) [AN_ENABLE] = 1b (Enable SGMII AN)
	 *
	 * Note: Since it is MAC side SGMII, there is no need to set
	 *	 SR_MII_AN_ADV. MAC side SGMII receives AN Tx Config from
	 *	 PHY about the link state change after C28 AN is completed
	 *	 between PHY and Link Partner. There is also no need to
	 *	 trigger AN restart for MAC-side SGMII.
	 */
	mdio_ctrl = xpcs_read(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL);
	if (mdio_ctrl < 0)
		return mdio_ctrl;

	if (mdio_ctrl & AN_CL37_EN) {
		ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL,
				 mdio_ctrl & ~AN_CL37_EN);
		if (ret < 0)
			return ret;
	}

#ifdef CONFIG_PCS_SIMAAI_MODALIX
	ret = xpcs_read(xpcs, MDIO_MMD_PCS, DW_VR_PCS_DIG_CTRL1);
	if (ret < 0)
		return ret;

	ret |= DW_VR_PCS_DIG_CTRL1_CL37_BP;
	ret = xpcs_write(xpcs, MDIO_MMD_PCS, DW_VR_PCS_DIG_CTRL1, ret);
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_CTRL);
#else
	ret = xpcs_read(xpcs, MDIO_MMD_AN, DW_VR_MII_AN_CTRL);
#endif
	if (ret < 0)
		return ret;

	ret &= ~(DW_VR_MII_PCS_MODE_MASK | DW_VR_MII_TX_CONFIG_MASK |
	       DW_VR_MII_MII_CTRL_MASK | DW_VR_MII_LINK_STS_MASK | DW_VR_MII_INTR_EN_MASK);
	ret |= (DW_VR_MII_PCS_MODE_C37_SGMII <<
		DW_VR_MII_AN_CTRL_PCS_MODE_SHIFT &
		DW_VR_MII_PCS_MODE_MASK);
	ret |= (DW_VR_MII_TX_CONFIG_PHY_SIDE_SGMII <<
		DW_VR_MII_AN_CTRL_TX_CONFIG_SHIFT &
		DW_VR_MII_TX_CONFIG_MASK);
	ret |= (DW_VR_MII_MII_CTRL_8BIT <<
		DW_VR_MII_AN_CTRL_MII_CTRL_SHIFT &
		DW_VR_MII_MII_CTRL_MASK);
	ret |= (DW_VR_MII_LINK_STS_UP <<
		DW_VR_MII_AN_CTRL_LINK_STS_SHIFT &
		DW_VR_MII_LINK_STS_MASK);
	ret |= (DW_VR_MII_INTR_EN_ENABLED <<
		DW_VR_MII_AN_CTRL_INTR_EN_SHIFT &
		DW_VR_MII_INTR_EN_MASK);

#ifdef CONFIG_PCS_SIMAAI_MODALIX
	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_CTRL, ret);
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND1, DW_VR_MII_DIG_CTRL1);
#else
	ret = xpcs_write(xpcs, MDIO_MMD_AN, DW_VR_MII_AN_CTRL, ret);
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_AN, DW_VR_MII_DIG_CTRL1);
#endif
	if (ret < 0)
		return ret;

	if (phylink_autoneg_inband(mode))
		ret |= DW_VR_MII_DIG_CTRL1_MAC_AUTO_SW;
	else
		ret &= ~DW_VR_MII_DIG_CTRL1_MAC_AUTO_SW;

#ifdef CONFIG_PCS_SIMAAI_MODALIX
	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_DIG_CTRL1, ret);
	if (ret < 0)
		return ret;

	/* Set 1G in SR_MII_CTRL SS13, SS6 bits and SR_MII_AN_ADV FD bit */
	mdio_ctrl |= SGMII_SPEED_SS6;
	mdio_ctrl &= ~SGMII_SPEED_SS13;

	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_MMD_CTRL, mdio_ctrl);
#else
	ret = xpcs_write(xpcs, MDIO_MMD_AN, DW_VR_MII_DIG_CTRL1, ret);
#endif
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, DW_VR_MII_AN_ADV);
	if (ret < 0)
		return ret;

	ret |= DW_VR_MII_AN_ADV_FD;

	ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_AN_ADV, ret);
	if (ret < 0)
		return ret;

	if (phylink_autoneg_inband(mode))
		ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL,
				 mdio_ctrl | AN_CL37_EN);

	return ret;
}

static int xpcs_config_aneg_c37_usxgmii(struct dw_xpcs *xpcs, unsigned int mode)
{
	int ret, mdio_ctrl;

	mdio_ctrl = xpcs_read(xpcs, MDIO_MMD_VEND1, DW_VR_MII_MMD_CTRL);
	if (mdio_ctrl < 0)
		return mdio_ctrl;

	if (mdio_ctrl & AN_CL37_EN) {
		ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_MMD_CTRL,
				 mdio_ctrl & ~AN_CL37_EN);
		if (ret < 0)
			return ret;
	}

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, DW_VR_PCS_DIG_CTRL1);
	if (ret < 0)
		return ret;

	ret |= DW_VR_PCS_DIG_CTRL1_CL37_BP;
	ret = xpcs_write(xpcs, MDIO_MMD_PCS, DW_VR_PCS_DIG_CTRL1, ret);
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_CTRL);
	if (ret < 0)
		return ret;

	ret &= ~(DW_VR_MII_PCS_MODE_MASK | DW_VR_MII_TX_CONFIG_MASK |
	       DW_VR_MII_MII_CTRL_MASK | DW_VR_MII_LINK_STS_MASK | DW_VR_MII_INTR_EN_MASK);
	ret |= (DW_VR_MII_TX_CONFIG_PHY_SIDE_SGMII <<
		DW_VR_MII_AN_CTRL_TX_CONFIG_SHIFT &
		DW_VR_MII_TX_CONFIG_MASK);
	ret |= (DW_VR_MII_MII_CTRL_8BIT <<
		DW_VR_MII_AN_CTRL_MII_CTRL_SHIFT &
		DW_VR_MII_MII_CTRL_MASK);
	ret |= (DW_VR_MII_LINK_STS_UP <<
		DW_VR_MII_AN_CTRL_LINK_STS_SHIFT &
		DW_VR_MII_LINK_STS_MASK);
	ret |= (DW_VR_MII_INTR_EN_ENABLED <<
		DW_VR_MII_AN_CTRL_INTR_EN_SHIFT &
		DW_VR_MII_INTR_EN_MASK);
	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_CTRL, ret);
	if (ret < 0)
		return ret;

	/* Set 10G in SR_MII_CTRL SS13, SS6 bits and SR_MII_AN_ADV FD bit */
	mdio_ctrl |= SGMII_SPEED_SS6;
	mdio_ctrl |= SGMII_SPEED_SS13;
	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_MMD_CTRL, mdio_ctrl);
	if (ret < 0)
		return ret;

	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_MMD_CTRL,
				mdio_ctrl | AN_CL37_EN);

	return ret;
}

static int xpcs_config_aneg_c37_1000basex(struct dw_xpcs *xpcs, unsigned int mode,
					  const unsigned long *advertising)
{
	phy_interface_t interface = PHY_INTERFACE_MODE_1000BASEX;
	int ret, mdio_ctrl, adv;
	bool changed = 0;

	/* According to Chap 7.12, to set 1000BASE-X C37 AN, AN must
	 * be disabled first:-
	 * 1) VR_MII_MMD_CTRL Bit(12)[AN_ENABLE] = 0b
	 * 2) VR_MII_AN_CTRL Bit(2:1)[PCS_MODE] = 00b (1000BASE-X C37)
	 */
	mdio_ctrl = xpcs_read(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL);
	if (mdio_ctrl < 0)
		return mdio_ctrl;

	if (mdio_ctrl & AN_CL37_EN) {
		ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL,
				 mdio_ctrl & ~AN_CL37_EN);
		if (ret < 0)
			return ret;
	}

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, DW_VR_MII_AN_CTRL);
	if (ret < 0)
		return ret;

	ret &= ~DW_VR_MII_PCS_MODE_MASK;
	ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_AN_CTRL, ret);
	if (ret < 0)
		return ret;

	/* Check for advertising changes and update the C45 MII ADV
	 * register accordingly.
	 */
	adv = phylink_mii_c22_pcs_encode_advertisement(interface,
						       advertising);
	if (adv >= 0) {
		ret = xpcs_modify_changed(xpcs, MDIO_MMD_VEND2,
					  MII_ADVERTISE, 0xffff, adv);
		if (ret < 0)
			return ret;

		changed = ret;
	}

	/* Clear CL37 AN complete status */
	ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_AN_INTR_STS, 0);
	if (ret < 0)
		return ret;

	if (phylink_autoneg_inband(mode) &&
	    linkmode_test_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, advertising)) {
		ret = xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL,
				 mdio_ctrl | AN_CL37_EN);
		if (ret < 0)
			return ret;
	}

	return changed;
}

static int xpcs_config_2500basex(struct dw_xpcs *xpcs)
{
	int ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND1, DW_VR_MII_DIG_CTRL1);
	if (ret < 0)
		return ret;
	ret |= DW_VR_MII_DIG_CTRL1_2G5_EN;
	ret &= ~DW_VR_MII_DIG_CTRL1_MAC_AUTO_SW;
#ifdef CONFIG_PCS_SIMAAI_MODALIX
	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_DIG_CTRL1, ret);
#else
	ret = xpcs_write(xpcs, MDIO_MMD_AN, DW_VR_MII_DIG_CTRL1, ret);
#endif
	if (ret < 0)
		return ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL);
	if (ret < 0)
		return ret;
	ret &= ~AN_CL37_EN;
	ret |= SGMII_SPEED_SS6;
	ret &= ~SGMII_SPEED_SS13;
	return xpcs_write(xpcs, MDIO_MMD_VEND2, DW_VR_MII_MMD_CTRL, ret);
}

int xpcs_do_config(struct dw_xpcs *xpcs, phy_interface_t interface,
		   unsigned int mode, const unsigned long *advertising)
{
	const struct xpcs_compat *compat;
	int ret;

	compat = xpcs_find_compat(xpcs->id, interface);
	if (!compat)
		return -ENODEV;

	/* This needs to be called only on Modalix board.
	 * Currently, we don't have anyway to identify it.
	 * We should implement SIMAAI specific ethernet and
	 * xpcs drivers when we upgrade kernel.
	 */
	simaai_xpcs_switch_mode(xpcs, interface);

	switch (compat->an_mode) {
	case DW_AN_C73:
		if (phylink_autoneg_inband(mode)) {
			ret = xpcs_config_aneg_c73(xpcs, compat);
			if (ret)
				return ret;
		}
		break;
	case DW_AN_C37_SGMII:
		ret = xpcs_config_aneg_c37_sgmii(xpcs, mode);
		if (ret)
			return ret;
		break;
	case DW_AN_C37_USXGMII:
		ret = xpcs_config_aneg_c37_usxgmii(xpcs, mode);
		if (ret)
			return ret;
		break;
	case DW_AN_C37_1000BASEX:
		ret = xpcs_config_aneg_c37_1000basex(xpcs, mode,
						     advertising);
		if (ret)
			return ret;
		break;
	case DW_2500BASEX:
		ret = xpcs_config_2500basex(xpcs);
		if (ret)
			return ret;
		break;
	default:
		return -1;
	}

	if (compat->pma_config) {
		ret = compat->pma_config(xpcs);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xpcs_do_config);

static int xpcs_config(struct phylink_pcs *pcs, unsigned int mode,
		       phy_interface_t interface,
		       const unsigned long *advertising,
		       bool permit_pause_to_mac)
{
	struct dw_xpcs *xpcs = phylink_pcs_to_xpcs(pcs);

	return xpcs_do_config(xpcs, interface, mode, advertising);
}

static int xpcs_get_state_c73(struct dw_xpcs *xpcs,
			      struct phylink_link_state *state,
			      const struct xpcs_compat *compat)
{
	int ret;

	/* Link needs to be read first ... */
	state->link = xpcs_read_link_c73(xpcs, state->an_enabled) > 0 ? 1 : 0;

	/* ... and then we check the faults. */
	ret = xpcs_read_fault_c73(xpcs, state);
	if (ret) {
		ret = xpcs_soft_reset(xpcs, compat);
		if (ret)
			return ret;

		state->link = 0;

		return xpcs_do_config(xpcs, state->interface, MLO_AN_INBAND, NULL);
	}

	if (state->an_enabled && xpcs_aneg_done_c73(xpcs, state, compat)) {
		state->an_complete = true;
		xpcs_read_lpa_c73(xpcs, state);
		xpcs_resolve_lpa_c73(xpcs, state);
	} else if (state->an_enabled) {
		state->link = 0;
	} else if (state->link) {
		xpcs_resolve_pma(xpcs, state);
	}

	return 0;
}

static int xpcs_get_state_c37_sgmii(struct dw_xpcs *xpcs,
				    struct phylink_link_state *state)
{
	int ret;

	/* Reset link_state */
	state->link = false;
	state->speed = SPEED_UNKNOWN;
	state->duplex = DUPLEX_UNKNOWN;
	state->pause = 0;
	
	/* For C37 SGMII mode, we check DW_VR_MII_AN_INTR_STS for link
	 * status, speed and duplex.
	 */
#ifdef CONFIG_PCS_SIMAAI_MODALIX
	ret = xpcs_read(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_INTR_STS);
#else
	ret = xpcs_read(xpcs, MDIO_MMD_AN, DW_VR_MII_AN_INTR_STS);
#endif
	if (ret < 0)
		return ret;

	if (ret & DW_VR_MII_C37_ANSGM_SP_LNKSTS) {
		int speed_value;

		state->link = true;

		speed_value = (ret & DW_VR_MII_AN_STS_C37_ANSGM_SP) >>
			      DW_VR_MII_AN_STS_C37_ANSGM_SP_SHIFT;
		if (speed_value == DW_VR_MII_C37_ANSGM_SP_1000)
			state->speed = SPEED_1000;
		else if (speed_value == DW_VR_MII_C37_ANSGM_SP_100)
			state->speed = SPEED_100;
		else
			state->speed = SPEED_10;

		if (ret & DW_VR_MII_AN_STS_C37_ANSGM_FD)
			state->duplex = DUPLEX_FULL;
		else
			state->duplex = DUPLEX_HALF;
	}
	ret &= ~(DW_VR_MII_CL37_ANCMPLT_INTR);

#ifdef CONFIG_PCS_SIMAAI_MODALIX
	ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_INTR_STS, ret);
#else
	ret = xpcs_write(xpcs, MDIO_MMD_AN, DW_VR_MII_AN_INTR_STS, ret);
#endif

	return 0;
}

static int xpcs_get_state_c37_usxgmii(struct dw_xpcs *xpcs,
				    struct phylink_link_state *state)
{
	int ret, ss = 0;

	/* Reset link_state */
	state->link = false;
	state->speed = SPEED_UNKNOWN;
	state->duplex = DUPLEX_UNKNOWN;
	state->pause = 0;

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MDIO_STAT1);
	if (ret < 0)
		return ret;

	if (ret & MDIO_STAT1_FAULT) {
		ret = simaai_xpcs_power_cycle(xpcs);
		if (ret < 0)
			return ret;
		udelay(10000);
	}

	/* For C37 USXGMII mode, we check DW_VR_MII_AN_INTR_STS for link
	 * status, speed and duplex.
	 */
	ret = xpcs_read(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_INTR_STS);
	if (ret < 0)
		return ret;

	if (ret & DW_VR_MII_AN_USXGMII_LNKSTS) {
		int speed_value;

		state->link = true;

		speed_value = (ret & DW_VR_MII_AN_USXGMII_SP) >>
			      DW_VR_MII_AN_USXGMII_SP_SHIFT;
		switch(speed_value) {
		case DW_VR_MII_AN_USXGMII_SP_10000:
			state->speed = SPEED_10000;
			ss = SGMII_SPEED_SS6 | SGMII_SPEED_SS13;
			break;
		case DW_VR_MII_AN_USXGMII_SP_1000:
			state->speed = SPEED_1000;
			ss = SGMII_SPEED_SS6;
			break;
		case DW_VR_MII_AN_USXGMII_SP_100:
			state->speed = SPEED_100;
			ss = SGMII_SPEED_SS13;
			break;
		case DW_VR_MII_AN_USXGMII_SP_10:
			state->speed = SPEED_10;
			ss = 0;
			break;
		case DW_VR_MII_AN_USXGMII_SP_5000:
			state->speed = SPEED_5000;
			ss = SGMII_SPEED_SS5 | SGMII_SPEED_SS13;
			break;
		case DW_VR_MII_AN_USXGMII_SP_2500:
			state->speed = SPEED_2500;
			ss = SGMII_SPEED_SS5;
			break;
		default:
			state->speed = SPEED_UNKNOWN;
			break;
		}

		if (ret & DW_VR_MII_AN_USXGMII_FD)
			state->duplex = DUPLEX_FULL;
		else
			state->duplex = DUPLEX_HALF;
	}

	if (ret & DW_VR_MII_CL37_ANCMPLT_INTR) {
		u32 val;
		ret &= ~(DW_VR_MII_CL37_ANCMPLT_INTR);
		ret = xpcs_write(xpcs, MDIO_MMD_VEND1, DW_VR_MII_AN_INTR_STS, ret);
		if (ret < 0)
			return ret;

		udelay(1000);
		ret = xpcs_read(xpcs, MDIO_MMD_PCS, DW_VR_PCS_DIG_CTRL1);
		if (ret < 0)
			return ret;

		ret |= DW_VR_PCS_DIG_CTRL1_USRA_RST;
		ret = xpcs_write(xpcs, MDIO_MMD_PCS, DW_VR_PCS_DIG_CTRL1, ret);

		/* Wait for DW_VR_PCS_DIG_CTRL1_USRA_RST to get cleared */
		ret =  read_poll_timeout(xpcs_read, val,
					!(val & DW_VR_PCS_DIG_CTRL1_USRA_RST),
					0, 1000000, false,
					xpcs, MDIO_MMD_PCS, DW_VR_PCS_DIG_CTRL1);
		if (ret < 0)
			pr_warn("Timed out for USRA_RST to clear\n");
	} else
		ret = 0;

	return ret;
}

static int xpcs_get_state_c37_1000basex(struct dw_xpcs *xpcs,
					struct phylink_link_state *state)
{
	int lpa, bmsr;

	if (state->an_enabled) {
		/* Reset link state */
		state->link = false;

		lpa = xpcs_read(xpcs, MDIO_MMD_VEND2, MII_LPA);
		if (lpa < 0 || lpa & LPA_RFAULT)
			return lpa;

		bmsr = xpcs_read(xpcs, MDIO_MMD_VEND2, MII_BMSR);
		if (bmsr < 0)
			return bmsr;

		phylink_mii_c22_pcs_decode_state(state, bmsr, lpa);
	}

	return 0;
}

static void xpcs_get_state(struct phylink_pcs *pcs,
			   struct phylink_link_state *state)
{
	struct dw_xpcs *xpcs = phylink_pcs_to_xpcs(pcs);
	const struct xpcs_compat *compat;
	int ret;

	compat = xpcs_find_compat(xpcs->id, state->interface);
	if (!compat)
		return;

	switch (compat->an_mode) {
	case DW_AN_C73:
		ret = xpcs_get_state_c73(xpcs, state, compat);
		if (ret) {
			pr_err("xpcs_get_state_c73 returned %pe\n",
			       ERR_PTR(ret));
			return;
		}
		break;
	case DW_AN_C37_SGMII:
		ret = xpcs_get_state_c37_sgmii(xpcs, state);
		if (ret) {
			pr_err("xpcs_get_state_c37_sgmii returned %pe\n",
			       ERR_PTR(ret));
		}
		break;
	case DW_AN_C37_USXGMII:
		ret = xpcs_get_state_c37_usxgmii(xpcs, state);
		if (ret) {
			pr_err("xpcs_get_state_c37_usxgmii returned %pe\n",
			       ERR_PTR(ret));
		}
		break;
	case DW_AN_C37_1000BASEX:
		ret = xpcs_get_state_c37_1000basex(xpcs, state);
		if (ret) {
			pr_err("xpcs_get_state_c37_1000basex returned %pe\n",
			       ERR_PTR(ret));
		}
		break;
	default:
		return;
	}
}

static void xpcs_link_up_sgmii(struct dw_xpcs *xpcs, unsigned int mode,
			       int speed, int duplex)
{
	int val, ret;

	if (phylink_autoneg_inband(mode))
		return;

	val = mii_bmcr_encode_fixed(speed, duplex);
	ret = xpcs_write(xpcs, MDIO_MMD_VEND2, MDIO_CTRL1, val);
	if (ret)
		pr_err("%s: xpcs_write returned %pe\n", __func__, ERR_PTR(ret));
}

static void xpcs_link_up_1000basex(struct dw_xpcs *xpcs, unsigned int mode,
				   int speed, int duplex)
{
	int val, ret;

	if (phylink_autoneg_inband(mode))
		return;

	switch (speed) {
	case SPEED_1000:
		val = BMCR_SPEED1000;
		break;
	case SPEED_100:
	case SPEED_10:
	default:
		pr_err("%s: speed = %d\n", __func__, speed);
		return;
	}

	if (duplex == DUPLEX_FULL)
		val |= BMCR_FULLDPLX;
	else
		pr_err("%s: half duplex not supported\n", __func__);

	ret = xpcs_write(xpcs, MDIO_MMD_VEND2, MDIO_CTRL1, val);
	if (ret)
		pr_err("%s: xpcs_write returned %pe\n", __func__, ERR_PTR(ret));
}

void xpcs_link_up(struct phylink_pcs *pcs, unsigned int mode,
		  phy_interface_t interface, int speed, int duplex)
{
	struct dw_xpcs *xpcs = phylink_pcs_to_xpcs(pcs);

	if (interface == PHY_INTERFACE_MODE_USXGMII)
		return xpcs_config_usxgmii(xpcs, speed);
	if (interface == PHY_INTERFACE_MODE_SGMII)
		return xpcs_link_up_sgmii(xpcs, mode, speed, duplex);
	if (interface == PHY_INTERFACE_MODE_1000BASEX)
		return xpcs_link_up_1000basex(xpcs, mode, speed, duplex);
}
EXPORT_SYMBOL_GPL(xpcs_link_up);

static void xpcs_an_restart(struct phylink_pcs *pcs)
{
	struct dw_xpcs *xpcs = phylink_pcs_to_xpcs(pcs);
	int ret;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, MDIO_CTRL1);
	if (ret >= 0) {
		ret |= BMCR_ANRESTART;
		xpcs_write(xpcs, MDIO_MMD_VEND2, MDIO_CTRL1, ret);
	}
}

static u32 xpcs_get_id(struct dw_xpcs *xpcs)
{
	int ret;
	u32 id;

	/* First, search C73 PCS using PCS MMD */
	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MII_PHYSID1);
	if (ret < 0)
		return 0xffffffff;

	id = ret << 16;

	ret = xpcs_read(xpcs, MDIO_MMD_PCS, MII_PHYSID2);
	if (ret < 0)
		return 0xffffffff;

	/* If Device IDs are not all zeros or all ones,
	 * we found C73 AN-type device
	 */
	if ((id | ret) && (id | ret) != 0xffffffff)
		return id | ret;

	/* Next, search C37 PCS using Vendor-Specific MII MMD */
	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, MII_PHYSID1);
	if (ret < 0)
		return 0xffffffff;

	id = ret << 16;

	ret = xpcs_read(xpcs, MDIO_MMD_VEND2, MII_PHYSID2);
	if (ret < 0)
		return 0xffffffff;

	/* If Device IDs are not all zeros, we found C37 AN-type device */
	if (id | ret)
		return id | ret;

	return 0xffffffff;
}

static const struct xpcs_compat synopsys_xpcs_compat[DW_XPCS_INTERFACE_MAX] = {
	[DW_XPCS_USXGMII] = {
		.supported = xpcs_usxgmii_features,
		.interface = xpcs_usxgmii_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_usxgmii_interfaces),
#ifdef CONFIG_PCS_SIMAAI_MODALIX
		.an_mode = DW_AN_C37_USXGMII,
#else
		.an_mode = DW_AN_C73,
#endif
	},
	[DW_XPCS_10GKR] = {
		.supported = xpcs_10gkr_features,
		.interface = xpcs_10gkr_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_10gkr_interfaces),
		.an_mode = DW_AN_C73,
	},
	[DW_XPCS_XLGMII] = {
		.supported = xpcs_xlgmii_features,
		.interface = xpcs_xlgmii_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_xlgmii_interfaces),
		.an_mode = DW_AN_C73,
	},
	[DW_XPCS_SGMII] = {
		.supported = xpcs_sgmii_features,
		.interface = xpcs_sgmii_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_sgmii_interfaces),
		.an_mode = DW_AN_C37_SGMII,
	},
	[DW_XPCS_1000BASEX] = {
		.supported = xpcs_1000basex_features,
		.interface = xpcs_1000basex_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_1000basex_interfaces),
		.an_mode = DW_AN_C37_1000BASEX,
	},
	[DW_XPCS_2500BASEX] = {
		.supported = xpcs_2500basex_features,
		.interface = xpcs_2500basex_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_2500basex_features),
		.an_mode = DW_2500BASEX,
	},
};

static const struct xpcs_compat nxp_sja1105_xpcs_compat[DW_XPCS_INTERFACE_MAX] = {
	[DW_XPCS_SGMII] = {
		.supported = xpcs_sgmii_features,
		.interface = xpcs_sgmii_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_sgmii_interfaces),
		.an_mode = DW_AN_C37_SGMII,
		.pma_config = nxp_sja1105_sgmii_pma_config,
	},
};

static const struct xpcs_compat nxp_sja1110_xpcs_compat[DW_XPCS_INTERFACE_MAX] = {
	[DW_XPCS_SGMII] = {
		.supported = xpcs_sgmii_features,
		.interface = xpcs_sgmii_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_sgmii_interfaces),
		.an_mode = DW_AN_C37_SGMII,
		.pma_config = nxp_sja1110_sgmii_pma_config,
	},
	[DW_XPCS_2500BASEX] = {
		.supported = xpcs_2500basex_features,
		.interface = xpcs_2500basex_interfaces,
		.num_interfaces = ARRAY_SIZE(xpcs_2500basex_interfaces),
		.an_mode = DW_2500BASEX,
		.pma_config = nxp_sja1110_2500basex_pma_config,
	},
};

static const struct xpcs_id xpcs_id_list[] = {
	{
		.id = SYNOPSYS_XPCS_ID,
		.mask = SYNOPSYS_XPCS_MASK,
		.compat = synopsys_xpcs_compat,
	}, {
		.id = NXP_SJA1105_XPCS_ID,
		.mask = SYNOPSYS_XPCS_MASK,
		.compat = nxp_sja1105_xpcs_compat,
	}, {
		.id = NXP_SJA1110_XPCS_ID,
		.mask = SYNOPSYS_XPCS_MASK,
		.compat = nxp_sja1110_xpcs_compat,
	},
};

static const struct phylink_pcs_ops xpcs_phylink_ops = {
	.pcs_validate = xpcs_validate,
	.pcs_config = xpcs_config,
	.pcs_get_state = xpcs_get_state,
	.pcs_an_restart = xpcs_an_restart,
	.pcs_link_up = xpcs_link_up,
};

struct dw_xpcs *xpcs_create(void __iomem *base, phy_interface_t interface,
			     int skip_reset, int irq)
{
	struct dw_xpcs *xpcs;
	u32 xpcs_id;
	int i, ret;

	if (IS_ERR_OR_NULL(base))
		return ERR_PTR(-ENODEV);

	xpcs = kzalloc(sizeof(*xpcs), GFP_KERNEL);
	if (!xpcs)
		return ERR_PTR(-ENOMEM);

	xpcs->addr = base;

	xpcs_id = xpcs_get_id(xpcs);

	for (i = 0; i < ARRAY_SIZE(xpcs_id_list); i++) {
		const struct xpcs_id *entry = &xpcs_id_list[i];
		const struct xpcs_compat *compat;

		if ((xpcs_id & entry->mask) != entry->id)
			continue;

		xpcs->id = entry;

		compat = xpcs_find_compat(entry, interface);
		if (!compat) {
			ret = -ENODEV;
			goto out;
		}

		xpcs->pcs.ops = &xpcs_phylink_ops;
		if (irq <= 0)
			xpcs->pcs.poll = true;
		xpcs->skip_reset = skip_reset;

		ret = xpcs_soft_reset(xpcs, compat);
		if (ret)
			goto out;

		return xpcs;
	}

	ret = -ENODEV;

out:
	kfree(xpcs);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(xpcs_create);

void xpcs_destroy(struct dw_xpcs *xpcs)
{
	kfree(xpcs);
}
EXPORT_SYMBOL_GPL(xpcs_destroy);

MODULE_LICENSE("GPL v2");
