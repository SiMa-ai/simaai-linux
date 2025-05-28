//SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2024 Sima ai
 */

#include <linux/pcs/pcs-xpcs.h>
#include "pcs-xpcs.h"

#ifdef CONFIG_PCS_SIMAAI_MODALIX
/* currently using SIMAAI specific literals.
 * Need to change them to DW ones later
 */

#define SIMAAI_VR_XS_PMA_MP_32G_LN_LINK_CTRL	0x2025c
#define SIMAAI_SR_XS_PCS_CTRL1			0x80000
#define SIMAAI_SR_XS_PCS_CTRL2			0x8001c
#define LPM					BIT(11)
#define SIMAAI_VR_XS_PMA_MP_32G_RX_CNTX_CTRL0	0x20248
#define RX_CNTX_SEL_0_10GBASER			0x05
#define RX_CNTX_SEL_0_SGMII			0x06
#define RX_CNTX_SEL_0_USXGMII_10G		0x0a
#define SIMAAI_VR_XS_PMA_MP_32G_TX_CNTX_CTRL0	0x200f8
#define TX_CNTX_SEL_0_10GBASER			0x05
#define TX_CNTX_SEL_0_SGMII			0x06
#define TX_CNTX_SEL_0_USXGMII_10G		0x0a
#define SIMAAI_VR_XS_PMA_MP_32G_TX_CM_CNTX_SEL0	0x200f0
#define CMN_CNTX_0_10GBASER			0x05
#define CMN_CNTX_0_SGMII			0x06
#define CMN_CNTX_0_USXGMII_10G			0x0a
#define SIMAAI_VR_XS_PMA_MP_25G_RX_WIDTH_CTRL	0x202c0
#define WIDTH_10BIT				0x1
#define WIDTH_32BIT				0x4
#define SIMAAI_VR_XS_PMA_MP_25G_TX_WIDTH_CTRL	0x20118
#define WAIT_US					1000

static int simaai_xpcs_poll_power_up(struct dw_xpcs *xpcs)
{
	int val, ret;

		/* Wait xpcs power-up good */
	ret = read_poll_timeout(xpcs_read_vpcs, val,
				(val & DW_PSEQ_ST) == DW_PSEQ_ST_GOOD,
				10000, 1000000, false,
				xpcs, DW_VR_XS_PCS_DIG_STS);
	if (ret < 0)
		dev_err(&xpcs->mdiodev->dev, "xpcs power-up timeout\n");

	return ret;
}

static int simaai_xpcs_poll_power_down(struct dw_xpcs *xpcs)
{
	int val, ret;

	ret = read_poll_timeout(xpcs_read_vpcs, val,
				(val & DW_PSEQ_ST) == DW_PSEQ_ST_DOWN,
				10000, 1000000, false,
				xpcs, DW_VR_XS_PCS_DIG_STS);
	if (ret < 0)
		dev_err(&xpcs->mdiodev->dev, "xpcs power-down timeout\n");

	return ret;
}

static int simaai_xpcs_supress_sig_loss_detect(struct dw_xpcs *xpcs, bool on)
{
	int ret = xpcs_read_vpcs(xpcs, DW_VR_XS_PCS_DBG_CTRL);
	if (ret < 0)
		return ret;

	if (on)
		ret |= (DW_SUPRESS_LOS_DET | DW_RX_DT_EN_CTL);
	else
		ret &= ~(DW_SUPRESS_LOS_DET | DW_RX_DT_EN_CTL);

	return xpcs_write_vpcs(xpcs, DW_VR_XS_PCS_DBG_CTRL, ret);
}

static int simaai_select_pcs_type(struct dw_xpcs *xpcs, phy_interface_t type)
{
	u32 val;

	switch (type) {
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_USXGMII:
		val = 0;
		break;
	case PHY_INTERFACE_MODE_SGMII:
		val = 1;
		break;
	case PHY_INTERFACE_MODE_5GBASER:
		val = 15;
		break;
	default:
		dev_err(&xpcs->mdiodev->dev,
			"unsupported interface type(%d)\n", type);
		return -ENOTSUPP;
	}

	writel(val, xpcs->addr + SIMAAI_SR_XS_PCS_CTRL2);

	return 0;
}

static u32 simaai_alloc_index(void)
{
	static atomic_t device_indexes = ATOMIC_INIT(1);

	return ((atomic_inc_return(&device_indexes) - 1) & 0xf);
}

static int simaai_xpcs_config_32g_phy(struct dw_xpcs *xpcs,
				      phy_interface_t type)
{
	u32 val1, val2, val3, val4, val5;

	switch (type) {
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_10GKR:
		val1 = RX_CNTX_SEL_0_10GBASER;
		val2 = TX_CNTX_SEL_0_10GBASER;
		val3 = CMN_CNTX_0_10GBASER;
		val4 = WIDTH_32BIT;
		val5 = WIDTH_32BIT;
		break;

	case PHY_INTERFACE_MODE_SGMII:
		val1 = RX_CNTX_SEL_0_SGMII;
		val2 = TX_CNTX_SEL_0_SGMII;
		val3 = CMN_CNTX_0_SGMII;
		val4 = WIDTH_10BIT;
		val5 = WIDTH_10BIT;
		break;

	case PHY_INTERFACE_MODE_USXGMII:
		val1 = RX_CNTX_SEL_0_USXGMII_10G;
		val2 = TX_CNTX_SEL_0_USXGMII_10G;
		val3 = CMN_CNTX_0_USXGMII_10G;
		val4 = WIDTH_32BIT;
		val5 = WIDTH_32BIT;
		break;

	default:
		dev_err(&xpcs->mdiodev->dev, "unsupported interface type\n");
		return -ENOTSUPP;
	}

	writel(val1, xpcs->addr + SIMAAI_VR_XS_PMA_MP_32G_RX_CNTX_CTRL0);
	writel(val2, xpcs->addr + SIMAAI_VR_XS_PMA_MP_32G_TX_CNTX_CTRL0);
	writel(val3, xpcs->addr + SIMAAI_VR_XS_PMA_MP_32G_TX_CM_CNTX_SEL0);
	writel(val4, xpcs->addr + SIMAAI_VR_XS_PMA_MP_25G_RX_WIDTH_CTRL);
	writel(val5, xpcs->addr + SIMAAI_VR_XS_PMA_MP_25G_TX_WIDTH_CTRL);

	if (!xpcs->link_num)
		xpcs->link_num = simaai_alloc_index();

	writel(xpcs->link_num,
	       xpcs->addr + SIMAAI_VR_XS_PMA_MP_32G_LN_LINK_CTRL);

	return 0;
}

int simaai_xpcs_power_cycle(struct dw_xpcs *xpcs)
{
	int ret;
	u32 val;

	val = readl(xpcs->addr + SIMAAI_SR_XS_PCS_CTRL1);
	val |= LPM;
	writel(val, xpcs->addr + SIMAAI_SR_XS_PCS_CTRL1);

	udelay(WAIT_US);

	ret = simaai_xpcs_poll_power_down(xpcs);
	if (ret < 0)
		goto out;

	udelay(WAIT_US);

	writel(0x440, xpcs->addr + SIMAAI_SR_XS_PCS_CTRL1);

	udelay(WAIT_US);

	ret = simaai_xpcs_poll_power_up(xpcs);
	if (ret < 0)
		goto out;

	udelay(WAIT_US);

	return 0;

out:
	dev_err(&xpcs->mdiodev->dev, "%s failed\n", __func__);
	return ret;
}

static int simaai_xpcs_switch_sgmii(struct dw_xpcs *xpcs)
{
	int ret;

	ret = simaai_select_pcs_type(xpcs, PHY_INTERFACE_MODE_SGMII);
	if (ret < 0)
		goto out;

	ret = xpcs_read_vpcs(xpcs, DW_VR_XS_PCS_DIG_CTRL1);
	if (ret < 0)
		goto out;

	/* clear DW_EN_2_5G_MODE bit */
	ret &= ~DW_EN_2_5G_MODE;
	ret = xpcs_write_vpcs(xpcs, DW_VR_XS_PCS_DIG_CTRL1, ret);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_config_32g_phy(xpcs, PHY_INTERFACE_MODE_SGMII);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_power_cycle(xpcs);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_supress_sig_loss_detect(xpcs, false);
	if (ret < 0)
		goto out;

	dev_info(&xpcs->mdiodev->dev,
		 "Interface successfully switched to SGMII\n");

	return 0;

out:
	dev_err(&xpcs->mdiodev->dev, "%s failed\n", __func__);
	return ret;
}

static int simaai_xpcs_switch_usxgmii(struct dw_xpcs *xpcs)
{
	int ret;

	ret = simaai_select_pcs_type(xpcs, PHY_INTERFACE_MODE_USXGMII);
	if (ret < 0)
		goto out;

	ret = xpcs_read_vpcs(xpcs, DW_VR_XS_PCS_KR_CTRL);
	if (ret < 0)
		goto out;

	/* clear usxgmii mode */
	ret &= ~(DW_USXG_MODE);
	/* set usxgmii mode to 10G */
	ret |= DW_USXG_MODE_10G;

	ret = xpcs_write_vpcs(xpcs, DW_VR_XS_PCS_KR_CTRL, ret);
	if (ret < 0)
		goto out;

	ret = xpcs_read_vpcs(xpcs, DW_VR_XS_PCS_DIG_CTRL1);
	if (ret < 0)
		goto out;

	/* enable usxgmii mode */
	ret |= DW_USXGMII_EN;

	ret = xpcs_write_vpcs(xpcs, DW_VR_XS_PCS_DIG_CTRL1, ret);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_config_32g_phy(xpcs, PHY_INTERFACE_MODE_USXGMII);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_power_cycle(xpcs);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_supress_sig_loss_detect(xpcs, true);
	if (ret < 0)
		goto out;

	dev_info(&xpcs->mdiodev->dev,
		 "Interface successfully switched to USXGMII\n");

	return 0;

out:
	dev_err(&xpcs->mdiodev->dev, "%s failed\n", __func__);
	return ret;
}

static int simaai_xpcs_switch_10g_base_r(struct dw_xpcs *xpcs)
{
	int ret;

	ret = simaai_select_pcs_type(xpcs, PHY_INTERFACE_MODE_10GBASER);
	if (ret < 0)
		goto out;

	ret = xpcs_read_vpcs(xpcs, DW_VR_XS_PCS_DIG_CTRL1);
	if (ret < 0)
		goto out;

	/* clear usxgmii mode */
	ret &= ~DW_USXGMII_EN;
	/* clear DW_EN_2_5G_MODE bit */
	ret &= ~DW_EN_2_5G_MODE;

	ret = xpcs_write_vpcs(xpcs, DW_VR_XS_PCS_DIG_CTRL1, ret);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_config_32g_phy(xpcs, PHY_INTERFACE_MODE_10GBASER);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_power_cycle(xpcs);
	if (ret < 0)
		goto out;

	ret = simaai_xpcs_supress_sig_loss_detect(xpcs, true);
	if (ret < 0)
		goto out;

	dev_info(&xpcs->mdiodev->dev,
		 "Interface successfully switched to 10GBASE-R\n");

	return 0;

out:
	dev_err(&xpcs->mdiodev->dev, "%s failed\n", __func__);
	return ret;
}

int simaai_xpcs_switch_mode(struct dw_xpcs *xpcs, phy_interface_t iface)
{
	int ret;

	switch (iface) {
	case PHY_INTERFACE_MODE_SGMII:
		ret = simaai_xpcs_switch_sgmii(xpcs);
		break;
	case PHY_INTERFACE_MODE_USXGMII:
		ret = simaai_xpcs_switch_usxgmii(xpcs);
		break;
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_10GBASER:
		ret = simaai_xpcs_switch_10g_base_r(xpcs);
		break;
	default:
		dev_err(&xpcs->mdiodev->dev,
			"%s: Unsupported interface=%d\n", __func__, iface);
		ret = -ENOTSUPP;
	}

	return ret;
}

#else
int simaai_xpcs_switch_mode(struct dw_xpcs *xpcs, phy_interface_t iface)
{
	return 0;
}

int simaai_xpcs_power_cycle(struct dw_xpcs *xpcs)
{
	return 0;
}

#endif
