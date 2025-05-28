/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Synopsys, Inc. and/or its affiliates.
 * Synopsys DesignWare XPCS helpers
 *
 * Author: Jose Abreu <Jose.Abreu@synopsys.com>
 */

#define SYNOPSYS_XPCS_ID		0x7996ced0
#define SYNOPSYS_XPCS_MASK		0xffffffff

#ifdef CONFIG_PCS_SIMAAI_MODALIX
#define DW_MMD_VEND1_OFFSET		0x780000
/* we don't have VEND2 registers, let's point it to vend1 */
#define DW_MMD_VEND2_OFFSET		DW_MMD_VEND1_OFFSET
#define DW_MMD_AN_OFFSET		0x180000
#define DW_MMD_PCS_OFFSET		0x080000
#else
#define DW_MMD_VEND1_OFFSET		0x60000
#define DW_MMD_VEND2_OFFSET		0x40000
#define DW_MMD_AN_OFFSET		0x40000
#define DW_MMD_PCS_OFFSET		0x0
#endif

/* Vendor regs access */
#define DW_VENDOR			BIT(15)

/* VR_XS_PCS */
#define DW_USXGMII_RST			BIT(10)
#define DW_USXGMII_EN			BIT(9)
#define DW_VR_XS_PCS_DIG_CTRL1		0x0000
#define DW_EN_2_5G_MODE			BIT(2)
#define DW_VR_XS_PCS_DBG_CTRL		0x0005
#define DW_SUPRESS_LOS_DET		BIT(4)
#define DW_RX_DT_EN_CTL			BIT(6)
#define DW_VR_XS_PCS_KR_CTRL		0x0007
#define DW_USXG_MODE			GENMASK(12, 10)
#define DW_USXG_MODE_10G		FIELD_PREP(GENMASK(12, 10), 0x0)
#define DW_USXG_MODE_5G			FIELD_PREP(GENMASK(12, 10), 0x1)
#define DW_VR_XS_PCS_DIG_STS		0x0010
#define DW_RXFIFO_ERR			GENMASK(6, 5)
#define DW_PSEQ_ST			GENMASK(4, 2)
#define DW_PSEQ_ST_GOOD			FIELD_PREP(GENMASK(4, 2), 0x4)
#define DW_PSEQ_ST_DOWN			FIELD_PREP(GENMASK(4, 2), 0x6)

/* SR_MII */
#define DW_USXGMII_FULL			BIT(8)
#define DW_USXGMII_SS_MASK		(BIT(13) | BIT(6) | BIT(5))
#define DW_USXGMII_10000		(BIT(13) | BIT(6))
#define DW_USXGMII_5000			(BIT(13) | BIT(5))
#define DW_USXGMII_2500			(BIT(5))
#define DW_USXGMII_1000			(BIT(6))
#define DW_USXGMII_100			(BIT(13))
#define DW_USXGMII_10			(0)

/* SR_AN */
#define DW_SR_AN_ADV1			0x10
#define DW_SR_AN_ADV2			0x11
#define DW_SR_AN_ADV3			0x12
#define DW_SR_AN_LP_ABL1		0x13
#define DW_SR_AN_LP_ABL2		0x14
#define DW_SR_AN_LP_ABL3		0x15

/* Clause 73 Defines */
/* AN_LP_ABL1 */
#define DW_C73_PAUSE			BIT(10)
#define DW_C73_ASYM_PAUSE		BIT(11)
#define DW_C73_AN_ADV_SF		0x1
/* AN_LP_ABL2 */
#define DW_C73_1000KX			BIT(5)
#define DW_C73_10000KX4			BIT(6)
#define DW_C73_10000KR			BIT(7)
/* AN_LP_ABL3 */
#define DW_C73_2500KX			BIT(0)
#define DW_C73_5000KR			BIT(1)

/* Clause 37 Defines */
/* VR MII MMD registers offsets */
#define DW_VR_MII_MMD_CTRL		0x0000
#define DW_VR_MII_AN_ADV		0x0004
#define DW_VR_MII_DIG_CTRL1		0x8000
#define DW_VR_MII_AN_CTRL		0x8001
#define DW_VR_MII_AN_INTR_STS		0x8002
/* Enable 2.5G Mode */
#define DW_VR_MII_DIG_CTRL1_2G5_EN	BIT(2)
/* EEE Mode Control Register */
#define DW_VR_MII_EEE_MCTRL0		0x8006
#define DW_VR_MII_EEE_MCTRL1		0x800b
#define DW_VR_MII_DIG_CTRL2		0x80e1

/* VR_MII_AN_ADV */
#define DW_VR_MII_AN_ADV_FD		BIT(5)

/* VR_MII_DIG_CTRL1 */
#define DW_VR_MII_DIG_CTRL1_PHY_MODE_CTRL	BIT(0)
#define DW_VR_MII_DIG_CTRL1_MAC_AUTO_SW		BIT(9)

/* VR_MII_DIG_CTRL2 */
#define DW_VR_MII_DIG_CTRL2_TX_POL_INV		BIT(4)
#define DW_VR_MII_DIG_CTRL2_RX_POL_INV		BIT(0)

/* VR_MII_AN_CTRL */
#define DW_VR_MII_AN_CTRL_MII_CTRL_SHIFT	8
#define DW_VR_MII_MII_CTRL_MASK		BIT(8)
#define DW_VR_MII_MII_CTRL_8BIT		0x1
#define DW_VR_MII_MII_CTRL_4BIT		0x0
#define DW_VR_MII_AN_CTRL_LINK_STS_SHIFT	4
#define DW_VR_MII_LINK_STS_MASK		BIT(4)
#define DW_VR_MII_LINK_STS_UP			0x1
#define DW_VR_MII_LINK_STS_DOWN		0x0
#define DW_VR_MII_AN_CTRL_TX_CONFIG_SHIFT	3
#define DW_VR_MII_TX_CONFIG_MASK		BIT(3)
#define DW_VR_MII_TX_CONFIG_PHY_SIDE_SGMII	0x1
#define DW_VR_MII_TX_CONFIG_MAC_SIDE_SGMII	0x0
#define DW_VR_MII_AN_CTRL_PCS_MODE_SHIFT	1
#define DW_VR_MII_PCS_MODE_MASK			GENMASK(2, 1)
#define DW_VR_MII_PCS_MODE_C37_1000BASEX	0x0
#define DW_VR_MII_PCS_MODE_C37_SGMII		0x2
#define DW_VR_MII_PCS_MODE_C37_QSGMII		0x3
#define DW_VR_MII_AN_CTRL_INTR_EN_SHIFT	0
#define DW_VR_MII_INTR_EN_MASK		BIT(0)
#define DW_VR_MII_INTR_EN_ENABLED		0x1
#define DW_VR_MII_INTR_EN_DISABLED		0x0

/* VR_MII_AN_INTR_STS */
#define DW_VR_MII_CL37_ANCMPLT_INTR			BIT(0)
#define DW_VR_MII_AN_STS_C37_ANSGM_FD		BIT(1)
#define DW_VR_MII_AN_STS_C37_ANSGM_SP_SHIFT	2
#define DW_VR_MII_AN_STS_C37_ANSGM_SP		GENMASK(3, 2)
#define DW_VR_MII_C37_ANSGM_SP_10		0x0
#define DW_VR_MII_C37_ANSGM_SP_100		0x1
#define DW_VR_MII_C37_ANSGM_SP_1000		0x2
#define DW_VR_MII_C37_ANSGM_SP_LNKSTS		BIT(4)
#define DW_VR_MII_AN_USXGMII_SP_SHIFT		10
#define DW_VR_MII_AN_USXGMII_SP			GENMASK(12, 10)
#define DW_VR_MII_AN_USXGMII_SP_10		0x0
#define DW_VR_MII_AN_USXGMII_SP_100		0x1
#define DW_VR_MII_AN_USXGMII_SP_1000		0x2
#define DW_VR_MII_AN_USXGMII_SP_10000		0x3
#define DW_VR_MII_AN_USXGMII_SP_2500		0x4
#define DW_VR_MII_AN_USXGMII_SP_5000		0x5
#define DW_VR_MII_AN_USXGMII_FD			BIT(13)
#define DW_VR_MII_AN_USXGMII_LNKSTS		BIT(14)

/* SR MII MMD Control defines */
#define AN_CL37_EN			BIT(12)	/* Enable Clause 37 auto-nego */
#define SGMII_SPEED_SS13		BIT(13)	/* SGMII speed along with SS6 and SS5 */
#define SGMII_SPEED_SS6			BIT(6)	/* SGMII speed along with SS13 and SS5 */
#define SGMII_SPEED_SS5			BIT(5)	/* SGMII speed along with SS13 and SS6 */

/* VR MII EEE Control 0 defines */
#define DW_VR_MII_EEE_LTX_EN			BIT(0)  /* LPI Tx Enable */
#define DW_VR_MII_EEE_LRX_EN			BIT(1)  /* LPI Rx Enable */
#define DW_VR_MII_EEE_TX_QUIET_EN		BIT(2)  /* Tx Quiet Enable */
#define DW_VR_MII_EEE_RX_QUIET_EN		BIT(3)  /* Rx Quiet Enable */
#define DW_VR_MII_EEE_TX_EN_CTRL		BIT(4)  /* Tx Control Enable */
#define DW_VR_MII_EEE_RX_EN_CTRL		BIT(7)  /* Rx Control Enable */

#define DW_VR_MII_EEE_MULT_FACT_100NS_SHIFT	8
#define DW_VR_MII_EEE_MULT_FACT_100NS		GENMASK(11, 8)

/* VR MII EEE Control 1 defines */
#define DW_VR_MII_EEE_TRN_LPI		BIT(0)	/* Transparent Mode Enable */

/* VR XS PCS Control defines */
#define DW_VR_PCS_DIG_CTRL1		0x8000
#define DW_VR_PCS_DIG_CTRL1_USRA_RST	BIT(10)  /* USXGMII Rate Adapter Reset (Port 0) */
#define DW_VR_PCS_DIG_CTRL1_CL37_BP	BIT(12)  /* Enable Clause 37 AN in BackPlane (BP) Configurations */

int xpcs_read(struct dw_xpcs *xpcs, int dev, u32 reg);
int xpcs_write(struct dw_xpcs *xpcs, int dev, u32 reg, u16 val);
int xpcs_read_vpcs(struct dw_xpcs *xpcs, int reg);
int xpcs_write_vpcs(struct dw_xpcs *xpcs, int reg, u16 val);
int nxp_sja1105_sgmii_pma_config(struct dw_xpcs *xpcs);
int nxp_sja1110_sgmii_pma_config(struct dw_xpcs *xpcs);
int nxp_sja1110_2500basex_pma_config(struct dw_xpcs *xpcs);
int simaai_xpcs_switch_mode(struct dw_xpcs *xpcs, phy_interface_t iface);
int simaai_xpcs_power_cycle(struct dw_xpcs *xpcs);
