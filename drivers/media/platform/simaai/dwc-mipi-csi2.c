// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 * Copyright 2026 SiMa Technologies, Inc.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/delay.h>
#include <linux/ratelimit.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-dv-timings.h>

#define SECS_PER_MIN 60

/* Driver-private V4L2 control IDs */
#define V4L2_CID_DWC_CSI2_IPI_CTRL_MODE	(V4L2_CID_USER_BASE + 0x1000)
#define V4L2_CID_DWC_CSI2_IPI_EMB_DATA_EN	(V4L2_CID_USER_BASE + 0x1001)

/* MIPI CSI-2 Host Controller Registers Define */

/* Core Version */
#define CSI2RX_VERSION					0x0

/* Number of Lanes */
#define CSI2RX_N_LANES					0x4
#define   CSI2RX_N_LANES_N_LANES(x)			FIELD_PREP(GENMASK(2, 0), (x) - 1)

/* Logic Reset */
#define CSI2RX_HOST_RESETN				0x8
#define   CSI2RX_HOST_RESETN_ENABLE			BIT(0)

/* Main Interrupt Status */
#define CSI2RX_INT_ST_MAIN				0xc
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_PHY		BIT(0)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_PKT		BIT(1)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_BNDRY_FRAMEL	BIT(2)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_SEQ_FRAME	BIT(3)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_CRC_FRAME	BIT(4)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_PLD_CRC		BIT(5)
#define   CSI2RX_INT_ST_MAIN_ERR_DID			BIT(6)
#define   CSI2RX_INT_ST_MAIN_ERR_ECC			BIT(7)
#define   CSI2RX_INT_ST_MAIN_ERR_PHY			BIT(16)
#define   CSI2RX_INT_ST_MAIN_ERR_LINE			BIT(17)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI		BIT(18)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI2		BIT(19)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI3		BIT(20)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI4		BIT(21)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI5		BIT(22)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI6		BIT(23)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI7		BIT(24)
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI8		BIT(25)

/* Payload CRC Fatal - status/mask/force triplet*/
#define CSI2RX_INT_ST_PLD_CRC_FATAL 0x2b0
#define CSI2RX_INT_MSK_PLD_CRC_FATAL 0x2b4
#define CSI2RX_INT_FORCE_PLD_CRC_FATAL 0x2b8
#define CSI2RX_INT_ST_PLD_CRC_FATAL_ERR BIT(0)

/* Fatal Interruption caused by Frame CRC - status/mask/force triplet */
#define CSI2RX_INT_ST_CRC_FRAME_FATAL 0x2a0
#define CSI2RX_INT_MSK_CRC_FRAME_FATAL 0x2a4
#define CSI2RX_INT_FORCE_CRC_FRAME_FATAL 0x2a8
/* Data monitor */
#define CSI2RX_DATA_IDS_1_DT				0x10
#define   CSI2RX_DATA_IDS_1_DT_DATA_ID0(x)		FIELD_PREP(GENMASK(5, 0), (x))
#define   CSI2RX_DATA_IDS_1_DT_DATA_ID1(x)		FIELD_PREP(GENMASK(13, 8), (x))
#define   CSI2RX_DATA_IDS_1_DT_DATA_ID2(x)		FIELD_PREP(GENMASK(21, 16), (x))
#define   CSI2RX_DATA_IDS_1_DT_DATA_ID3(x)		FIELD_PREP(GENMASK(29, 24), (x))

#define CSI2RX_DATA_IDS_2_DT				0x14
#define   CSI2RX_DATA_IDS_2_DT_DATA_ID4(x)		FIELD_PREP(GENMASK(5, 0), (x))
#define   CSI2RX_DATA_IDS_2_DT_DATA_ID5(x)		FIELD_PREP(GENMASK(13, 8), (x))
#define   CSI2RX_DATA_IDS_2_DT_DATA_ID6(x)		FIELD_PREP(GENMASK(21, 16), (x))
#define   CSI2RX_DATA_IDS_2_DT_DATA_ID7(x)		FIELD_PREP(GENMASK(29, 24), (x))

#define CSI2RX_DATA_IDS_1_VC				0x30
#define   CSI2RX_DATA_IDS_1_VC_DATA_ID0(x)		FIELD_PREP(GENMASK(3, 0), (x))
#define   CSI2RX_DATA_IDS_1_VC_DATA_ID1(x)		FIELD_PREP(GENMASK(11, 8), (x))
#define   CSI2RX_DATA_IDS_1_VC_DATA_ID2(x)		FIELD_PREP(GENMASK(19, 16), (x))
#define   CSI2RX_DATA_IDS_1_VC_DATA_ID3(x)		FIELD_PREP(GENMASK(27, 24), (x))

#define CSI2RX_DATA_IDS_2_VC				0x34
#define   CSI2RX_DATA_IDS_2_VC_DATA_ID4(x)		FIELD_PREP(GENMASK(3, 0), (x))
#define   CSI2RX_DATA_IDS_2_VC_DATA_ID5(x)		FIELD_PREP(GENMASK(11, 8), (x))
#define   CSI2RX_DATA_IDS_2_VC_DATA_ID6(x)		FIELD_PREP(GENMASK(19, 16), (x))
#define   CSI2RX_DATA_IDS_2_VC_DATA_ID7(x)		FIELD_PREP(GENMASK(27, 24), (x))

/* PHY Shutdown */
#define CSI2RX_DPHY_SHUTDOWNZ				0x40
#define   CSI2RX_DPHY_SHUTDOWNZ_ENABLE			BIT(0)

/* DPHY Reset */
#define CSI2RX_DPHY_RSTZ				0x44
#define   CSI2RX_DPHY_RSTZ_ENABLE			BIT(0)

/* RX PHY Status */
#define CSI2RX_DPHY_RX_STATUS				0x48
#define   CSI2RX_DPHY_RX_STATUS_DATA_LANE0_ULP		BIT(0)
#define   CSI2RX_DPHY_RX_STATUS_DATA_LANE1_ULP		BIT(1)
#define   CSI2RX_DPHY_RX_STATUS_CLK_LANE_ULP		BIT(16)
#define   CSI2RX_DPHY_RX_STATUS_CLK_LANE_HS		BIT(17)

/* STOP STATE PHY Status */
#define CSI2RX_DPHY_STOPSTATE				0x4c
#define   CSI2RX_DPHY_STOPSTATE_DATA_LANE0		BIT(0)
#define   CSI2RX_DPHY_STOPSTATE_DATA_LANE1		BIT(1)
#define   CSI2RX_DPHY_STOPSTATE_DATA_LANE2		BIT(2)
#define   CSI2RX_DPHY_STOPSTATE_DATA_LANE3		BIT(3)
#define   CSI2RX_DPHY_STOPSTATE_CLK_LANE		BIT(16)

/* DPHY Test and Control Interface 1 */
#define CSI2RX_DPHY_TEST_CTRL0				0x50
#define   CSI2RX_DPHY_TEST_CTRL0_TEST_CLR		BIT(0)
#define   CSI2RX_DPHY_TEST_CTRL0_TEST_CLKEN		BIT(1)

/* DPHY Test and Control Interface 2 */
#define CSI2RX_DPHY_TEST_CTRL1				0x54
#define   CSI2RX_DPHY_TEST_CTRL1_TEST_DIN(x)		FIELD_PREP(GENMASK(7, 0), (x))
#define   CSI2RX_DPHY_TEST_CTRL1_TEST_DOUT(x)		FIELD_GET(GENMASK(15, 8), (x))
#define   CSI2RX_DPHY_TEST_CTRL1_TEST_EN			BIT(16)

/* Pattern Generator vertical Resolution */
#define CSI2RX_PPI_PG_PATTERN_VRES			0x60
#define   CSI2RX_PPI_PG_PATTERN_VRES_VRES(x)		FIELD_PREP(GENMASK(15, 0), (x))

/* Pattern Generator horizontal Resolution */
#define CSI2RX_PPI_PG_PATTERN_HRES			0x64
#define   CSI2RX_PPI_PG_PATTERN_HRES_HRES(x)		FIELD_PREP(GENMASK(15, 0), (x))

/* Pattern Generator */
#define CSI2RX_PPI_PG_CONFIG				0x68
#define   CSI2RX_PPI_PG_CONFIG_PG_MODE(x)		FIELD_PREP(1, (x))
#define   CSI2RX_PPI_PG_CONFIG_DATA_TYPE(x)		FIELD_PREP(GENMASK(13, 8), (x))
#define   CSI2RX_PPI_PG_CONFIG_VIR_CHAN(x)		FIELD_PREP(GENMASK(15, 14), (x))
#define   CSI2RX_PPI_PG_CONFIG_VIR_CHAN_EX(x)		FIELD_PREP(GENMASK(17, 16), (x))
#define   CSI2RX_PPI_PG_CONFIG_VIR_CHAN_EX_2_EN		BIT(18)

/* Pattern Generator Enable */
#define CSI2RX_PPI_PG_ENABLE				0x6c
#define   CSI2RX_PPI_PG_ENABLE_EN			BIT(0)

/* Pattern Generator Status */
#define CSI2RX_PPI_PG_STATUS				0x70
#define   CSI2RX_PPI_PG_STATUS_ACTIVE			BIT(0)

/* IPI Mode */
#define  CSI2RX_IPI1_MODE				0x80
#define  CSI2RX_IPI2_MODE				0x200
#define  CSI2RX_IPI3_MODE				0x220
#define  CSI2RX_IPI4_MODE				0x240
#define   CSI2RX_IPI_MODE_CONTROLLER			BIT(0)
#define   CSI2RX_IPI_MODE_COLOR_MODE16			BIT(8)
#define   CSI2RX_IPI_MODE_CUT_THROUGH			BIT(16)
#define   CSI2RX_IPI_MODE_ENABLE			BIT(24)

/* IPI Virtual Channel */
#define CSI2RX_IPI1_VCID				0x84
#define CSI2RX_IPI2_VCID				0x204
#define CSI2RX_IPI3_VCID				0x224
#define CSI2RX_IPI4_VCID				0x244
#define   CSI2RX_IPI_VCID_VC(x)				FIELD_PREP(GENMASK(1, 0), (x))
#define   CSI2RX_IPI_VCID_VC_0_1(x)			FIELD_PREP(GENMASK(3, 2), (x))
#define   CSI2RX_IPI_VCID_VC_2				BIT(4)

/* IPI Data Type */
#define CSI2RX_IPI1_DATA_TYPE				0x88
#define CSI2RX_IPI2_DATA_TYPE				0x208
#define CSI2RX_IPI3_DATA_TYPE				0x228
#define CSI2RX_IPI4_DATA_TYPE				0x248
#define   CSI2RX_IPI_DATA_TYPE_DT(x)			FIELD_PREP(GENMASK(5, 0), (x))
#define   CSI2RX_IPI_DATA_TYPE_EMB_DATA_EN		BIT(8)

/* IPI Flush Memory */
#define CSI2RX_IPI1_MEM_FLUSH				0x8c
#define CSI2RX_IPI2_MEM_FLUSH				0x20c
#define CSI2RX_IPI3_MEM_FLUSH				0x22c
#define CSI2RX_IPI4_MEM_FLUSH				0x24c
#define   CSI2RX_IPI_MEM_FLUSH_AUTO			BIT(8)

/* IPI HSA */
#define CSI2RX_IPI1_HSA_TIME				0x90
#define CSI2RX_IPI2_HSA_TIME				0x210
#define CSI2RX_IPI3_HSA_TIME				0x230
#define CSI2RX_IPI4_HSA_TIME				0x250
#define   CSI2RX_IPI_HSA_TIME_VAL(x)			FIELD_PREP(GENMASK(11, 0), (x))

/* IPI HBP */
#define CSI2RX_IPI1_HBP_TIME				0x94
#define CSI2RX_IPI2_HBP_TIME				0x214
#define CSI2RX_IPI3_HBP_TIME				0x234
#define CSI2RX_IPI4_HBP_TIME				0x254
#define   CSI2RX_IPI_HBP_TIME_VAL(x)			FIELD_PREP(GENMASK(11, 0), (x))

/* IPI HSD */
#define CSI2RX_IPI1_HSD_TIME				0x98
#define CSI2RX_IPI2_HSD_TIME				0x218
#define CSI2RX_IPI3_HSD_TIME				0x238
#define CSI2RX_IPI4_HSD_TIME				0x258
#define   CSI2RX_IPI_HSD_TIME_VAL(x)			FIELD_PREP(GENMASK(11, 0), (x))

/* IPI HLINE */
#define CSI2RX_IPI_HLINE_TIME				0x9C
#define   CSI2RX_IPI_HLINE_TIME_VAL(x)			FIELD_PREP(GENMASK(14, 0), (x))

/* IPI Soft Reset */
#define CSI2RX_IPI_SOFTRSTN				0xa0

/* IPI Advanced Features */
#define CSI2RX_IPI1_ADV_FEATURES			0xac
#define CSI2RX_IPI2_ADV_FEATURES			0x21c
#define CSI2RX_IPI3_ADV_FEATURES			0x23c
#define CSI2RX_IPI4_ADV_FEATURES			0x25c
#define   CSI2RX_IPI_ADV_FEATURES_DT_OVER_WRITE_EN	BIT(0)
#define   CSI2RX_IPI_ADV_FEATURES_DT_OVER_WRITE(x)	FIELD_PREP(GENMASK(13, 8), (x))
#define   CSI2RX_IPI_ADV_FEATURES_LINE_EVENT_SEL	BIT(16)
#define   CSI2RX_IPI_ADV_FEATURES_SYNC_VIDEO_PKT	BIT(17)
#define   CSI2RX_IPI_ADV_FEATURES_SYNC_LS_PKT		BIT(18)
#define   CSI2RX_IPI_ADV_FEATURES_SYNC_NULL_PKT		BIT(19)
#define   CSI2RX_IPI_ADV_FEATURES_SYNC_BLANKING_PKT	BIT(20)
#define   CSI2RX_IPI_ADV_FEATURES_SYNC_EMBEDDED_PKT	BIT(21)
#define   CSI2RX_IPI_ADV_FEATURES_SYNC_EVENT_MODE	BIT(24)

/* IPI VSA */
#define CSI2RX_IPI_VSA_LINES				0xb0
#define   CSI2RX_IPI_VSA_LINES_VAL(x)			FIELD_PREP(GENMASK(9, 0), (x))

/* IPI VBP */
#define CSI2RX_IPI_VBP_LINES				0xb4
#define   CSI2RX_IPI_VBP_LINES_VAL(x)			FIELD_PREP(GENMASK(9, 0), (x))

/* IPI VFP */
#define CSI2RX_IPI_VFP_LINES				0xb8
#define   CSI2RX_IPI_VFP_LINES_VAL(x)			FIELD_PREP(GENMASK(9, 0), (x))

/* IPI VACTIVE */
#define CSI2RX_IPI_VACTIVE_LINES			0xbc
#define   CSI2RX_IPI_VACTIVE_LINES_VAL(x)		FIELD_PREP(GENMASK(13, 0), (x))

/* Vchan extended configuration */
#define CSI2RX_VIRTUAL_CHANNEL_EXT			0xc8

/* Fatal Interruption Caused by PHY */
#define CSI2RX_INT_ST_DPHY_FATAL			0xe0
#define   CSI2RX_INT_ST_DPHY_FATAL_ERR_SOT_LANE0	BIT(0)
#define   CSI2RX_INT_ST_DPHY_FATAL_ERR_SOT_LANE1	BIT(1)

/* Mask for Fatal Interruption Caused by PHY */
#define CSI2RX_INT_MSK_DPHY_FATAL			0xe4
#define   CSI2RX_INT_MSK_DPHY_FATAL_ERR_SOT_LANE0	BIT(0)
#define   CSI2RX_INT_MSK_DPHY_FATAL_ERR_SOT_LANE1	BIT(1)

/* Force for Fatal Interruption Caused by PHY */
#define CSI2RX_INT_FORCE_DPHY_FATAL			0xe8

/* Fatal Interruption Caused During Packet Construction */
#define CSI2RX_INT_ST_PKT_FATAL				0xf0
#define   CSI2RX_INT_ST_PKT_FATAL_ERR_ECC		BIT(0)
#define   CSI2RX_INT_ST_PKT_FATAL_ERR_PAYLOAD		BIT(1)

/* Mask for Fatal Interruption Caused During Packet Construction */
#define CSI2RX_INT_MSK_PKT_FATAL			0xf4
#define   CSI2RX_INT_MSK_PKT_FATAL_ERR_ECC		BIT(0)
#define   CSI2RX_INT_MSK_PKT_FATAL_ERR_PAYLOAD		BIT(1)

/* Force for Fatal Interruption Caused During Packet Construction */
#define CSI2RX_INT_FORCE_PKT_FATAL			0xf8

/* Interruption Caused by PHY */
#define CSI2RX_INT_ST_DPHY				0x110
#define   CSI2RX_INT_ST_DPHY_ERR_SOT_LANE0		BIT(0)
#define   CSI2RX_INT_ST_DPHY_ERR_SOT_LANE1		BIT(1)
#define   CSI2RX_INT_ST_DPHY_ERR_ESC_LANE0		BIT(16)
#define   CSI2RX_INT_ST_DPHY_ERR_ESC_LANE1		BIT(17)

/* Mask for Interruption Caused by PHY */
#define CSI2RX_INT_MSK_DPHY				0x114
#define   CSI2RX_INT_MSK_DPHY_SOT_ERR_LANE0		BIT(0)
#define   CSI2RX_INT_MSK_DPHY_SOT_ERR_LANE1		BIT(1)
#define   CSI2RX_INT_MSK_DPHY_ESC_ERR_LANE0		BIT(16)
#define   CSI2RX_INT_MSK_DPHY_ESC_ERR_LANE1		BIT(17)

/* Force for Interruption Caused by PHY */
#define CSI2RX_INT_FORCE_DPHY				0x118

/* Fatal Interruption Caused by IPI Interface */
#define CSI2RX_INT_ST_IPI_FATAL				0x140
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_IFFIFO_UNDERFLOW	BIT(0)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_IFFIFO_OVERFLOW	BIT(1)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_FRAME_SYNC	BIT(2)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_FIFO_NOT_EMPTY	BIT(3)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_HLINE_TIME	BIT(4)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_FIFO_OVERFLOW	BIT(5)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_PD_FIFO_OVERFLOW	BIT(6)

/* Mask for Fatal Interruption Caused by IPI Interface */
#define CSI2RX_INT_MSK_IPI_FATAL			0x144
#define   CSI2RX_INT_MSK_IPI_FATAL_ERR_IFFIFO_UNDERFLOW	BIT(0)
#define   CSI2RX_INT_MSK_IPI_FATAL_ERR_IFFIFO_OVERFLOW	BIT(1)
#define   CSI2RX_INT_MSK_IPI_FATAL_ERR_FRAME_SYNC	BIT(2)
#define   CSI2RX_INT_MSK_IPI_FATAL_ERR_FIFO_NOT_EMPTY	BIT(3)
#define   CSI2RX_INT_MSK_IPI_FATAL_ERR_HLINE_TIME	BIT(4)
#define   CSI2RX_INT_MSK_IPI_FATAL_ERR_FIFO_OVERFLOW	BIT(5)
#define   CSI2RX_INT_MSK_IPI_FATAL_ERR_PD_FIFO_OVERFLOW	BIT(6)

/* Force for Fatal Interruption Caused by IPI Interface */
#define CSI2RX_INT_FORCE_IPI_FATAL			0x148

/* Data De-Scrambling */
#define CSI2RX_SCRAMBLING				0x300

/* De-scrambler Seed for Lane 1 */
#define CSI2RX_SCRAMBLING_SEED1				0x304

/* De-scrambler Seed for Lane 2 */
#define CSI2RX_SCRAMBLING_SEED2				0x308

/* Glue Vis Control Glue Pix */
#define DWC_CSI2GLUE_VIS_CTRL_PIX1			0x0
#define DWC_CSI2GLUE_VIS_CTRL_PIX2			0x4
#define DWC_CSI2GLUE_VIS_CTRL_PIX3			0x8
#define DWC_CSI2GLUE_VIS_CTRL_PIX4			0xc
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_BUF_FULL_TH(x)	FIELD_PREP(GENMASK(6, 0), (x))
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_END_PATTERN(x)	FIELD_PREP(GENMASK(23, 8), (x))
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_XFER_FRAME_BOUND	BIT(24)
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_FRAME_FMT(x)	FIELD_PREP(GENMASK(27, 25), (x))
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_RAW8_PACK_EN	BIT(28)
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_DATA_ALIGN_EN	BIT(29)
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_SIGN_EXT_EN		BIT(30)
#define   DWC_CSI2GLUE_VIS_CTRL_PIX_SRAM_LP_EN		BIT(31)

/* Glue Vis Control General */
#define DWC_CSI2GLUE_VIS_CTRL				0x10
#define   DWC_CSI2GLUE_FILTINFO_SRC_BURST_LEN(x)	FIELD_PREP(GENMASK(7, 0), (x))
#define   DWC_CSI2GLUE_VIS_CTRL_BUF_SRAM_PIPE_EN	BIT(31)

/* DPHY Glue Control */
#define DWC_CSI2GLUE_DPHY_CTRL1				0x20
#define DWC_CSI2GLUE_DPHY_CTRL2				0x24
#define DWC_CSI2GLUE_DPHY_CTRL3				0x28
#define DWC_CSI2GLUE_DPHY_CTRL4				0x2C
#define   DWC_CSI2GLUE_DPHY_CTRL_FORCE_MODE		BIT(0)
#define   DWC_CSI2GLUE_DPHY_CTRL_BASEDIR		BIT(4)
#define   DWC_CSI2GLUE_DPHY_CTRL_SRAM_LP_EN		BIT(31)

/* DPHY Glue Pixel Count Control */
#define DWC_CSI2GLUE_PXL_CNT_CTRL1			0x70
#define DWC_CSI2GLUE_PXL_CNT_CTRL2			0x74
#define DWC_CSI2GLUE_PXL_CNT_CTRL3			0x78
#define DWC_CSI2GLUE_PXL_CNT_CTRL4			0x7c
#define   DWC_CSI2GLUE_PXL_CNT_CTRL_CNT_EN		BIT(0)
#define   DWC_CSI2GLUE_PXL_CNT_CTRL_CNT_RST		BIT(1)

/* Glue Channel Control */
#define DWC_CSI2GLUE_CHAN_CTRL1				0xb8
#define DWC_CSI2GLUE_CHAN_CTRL2				0xdc
#define DWC_CSI2GLUE_CHAN_CTRL3				0x100
#define DWC_CSI2GLUE_CHAN_CTRL4				0x124
#define   DWC_CSI2GLUE_CHAN_CTRL_EN			BIT(0)
#define   DWC_CSI2GLUE_CHAN_CTRL_DBITS(x)		FIELD_PREP(GENMASK(12, 8), (x))
#define   DWC_CSI2GLUE_CHAN_CTRL_TYPE_SPEC		BIT(14)
#define   DWC_CSI2GLUE_CHAN_CTRL_PACK			BIT(15)

/* Glue Statistics Control */
#define DWC_CSI2GLUE_STAT_CTRL1				0xbc
#define DWC_CSI2GLUE_STAT_CTRL2				0xe0
#define DWC_CSI2GLUE_STAT_CTRL3				0x104
#define DWC_CSI2GLUE_STAT_CTRL4				0x128
#define   DWC_CSI2GLUE_STAT_CTRL_RST_CNTS(x)		FIELD_PREP(GENMASK(7, 0), (x))
#define   DWC_CSI2GLUE_STAT_CTRL_OFLOW(x)		FIELD_PREP(GENMASK(23, 16), (x))

/* Glue Line Bytes */
#define DWC_CSI2GLUE_LNBYTES1				0xc0
#define DWC_CSI2GLUE_LNBYTES2				0xe4
#define DWC_CSI2GLUE_LNBYTES3				0x108
#define DWC_CSI2GLUE_LNBYTES4				0x12c
#define   DWC_CSI2GLUE_LNBYTES_LINE_BYTES(x)		FIELD_PREP(GENMASK(16, 0), (x))

/* Glue Frame Dimentions */
#define DWC_CSI2GLUE_FRMDIM1				0xc4
#define DWC_CSI2GLUE_FRMDIM2				0xe8
#define DWC_CSI2GLUE_FRMDIM3				0x10c
#define DWC_CSI2GLUE_FRMDIM4				0x130
#define   DWC_CSI2GLUE_FRMDIM_WIDTH(x)			FIELD_PREP(GENMASK(12, 0), (x))
#define   DWC_CSI2GLUE_FRMDIM_HEIGHT(x)			FIELD_PREP(GENMASK(27, 16), (x))

/* Glue Filter Information */
#define DWC_CSI2GLUE_FILTINFO1				0xc8
#define DWC_CSI2GLUE_FILTINFO2				0xec
#define DWC_CSI2GLUE_FILTINFO3				0x110
#define DWC_CSI2GLUE_FILTINFO4				0x134
#define   DWC_CSI2GLUE_FILTINFO_CHAN(x)			FIELD_PREP(GENMASK(4, 0), (x))
#define   DWC_CSI2GLUE_FILTINFO_DT(x)			FIELD_PREP(GENMASK(15, 8), (x))

/* Glue Metadata Configuration */
#define DWC_CSI2GLUE_META1				0xcc
#define DWC_CSI2GLUE_META2				0xf0
#define DWC_CSI2GLUE_META3				0x114
#define DWC_CSI2GLUE_META4				0x138
#define   DWC_CSI2GLUE_META_EMB_LD(x)			FIELD_PREP(GENMASK(15, 0), (x))
#define   DWC_CSI2GLUE_META_EMB_TR(x)			FIELD_PREP(GENMASK(31, 16), (x))

#define DWC_CSI2RX_PAD_SINK				0
#define DWC_CSI2RX_PAD_SOURCE				1
#define DWC_CSI2RX_PADS_NUM				8

#define DWC_CSI2RX_DEF_MBUS_CODE			MEDIA_BUS_FMT_UYVY8_1X16
#define DWC_CSI2RX_DEF_PIX_WIDTH			1920U
#define DWC_CSI2RX_DEF_PIX_HEIGHT			1080U
#define DWC_CSI2RX_MAX_PIX_WIDTH			0xffff
#define DWC_CSI2RX_MAX_PIX_HEIGHT			0xffff
#define DWC_CSI2RX_DEF_PIX_WIDTH			1920U
#define DWC_CSI2RX_DEF_HSA_TIME				0x10
#define DWC_CSI2RX_DEF_HBP_TIME				0xc8
#define DWC_CSI2RX_DEF_HSD_TIME				0x10
/* Fixed HSD time programmed into the IPI HSD_TIME register */
#define DWC_CSI2RX_HSD_TIME				10
#define DWC_CSI2RX_DEF_VSA_LINES			0x2
#define DWC_CSI2RX_DEF_VBP_LINES			0x2
#define DWC_CSI2RX_DEF_VFP_LINES			0xf

/* Default IPI enable delay (ms) if not overridden by the deserializer DT node */
#define DWC_CSI2RX_IPI_ENABLE_DELAY_MS			0

struct dwc_csi_event {
	u32 mask;
	const char * const name;
	unsigned int counter;
};

static const struct dwc_csi_event dwc_events[] = {
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI, "IPI Interface Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_ERR_PHY, "PHY Error" },
	{ CSI2RX_INT_ST_MAIN_ERR_ECC, "Header Single Bit Error" },
	{ CSI2RX_INT_ST_MAIN_ERR_DID, "Data ID Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_PLD_CRC, "Payload CRC Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_CRC_FRAME, "Frame CRC Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_SEQ_FRAME, "Frame Sequence Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_BNDRY_FRAMEL, "Frame Boundaries Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_PKT, "Packet Construction Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_PHY, "PHY Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_ERR_LINE, "Line Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI2, "IPI2 Interface Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI3, "IPI3 Interface Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI4, "IPI4 Interface Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI5, "IPI5 Interface Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI6, "IPI6 Interface Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI7, "IPI7 Interface Fatal Error" },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI8, "IPI8 Interface Fatal Error" },
};

#define DWC_NUM_EVENTS ARRAY_SIZE(dwc_events)
#define DWC_EVENT_MASK                                                         \
	(CSI2RX_INT_ST_MAIN_FATAL_ERR_PHY | CSI2RX_INT_ST_MAIN_FATAL_ERR_PKT | \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_BNDRY_FRAMEL |                           \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_SEQ_FRAME |                              \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_CRC_FRAME |                              \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_PLD_CRC | CSI2RX_INT_ST_MAIN_ERR_DID |   \
	 CSI2RX_INT_ST_MAIN_ERR_ECC | CSI2RX_INT_ST_MAIN_ERR_PHY |             \
	 CSI2RX_INT_ST_MAIN_ERR_LINE | CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI |      \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI2 |                                   \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI3 |                                   \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI4 |                                   \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI5 |                                   \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI6 |                                   \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI7 |                                   \
	 CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI8)
#define DWC_STOPSTATE_TIMEOUT 1000000

struct dwc_csi_pix_format {
	u32 code;
	u32 output;
	u32 data_type;
	u8 width;
};

enum {
	DWC_POLL_INACTIVE,
	DWC_POLL_SCHEDULED,
	DWC_POLL_ACTIVE,
	DWC_POLL_SUCCESS,
	DWC_POLL_TIMEOUT,
};

struct dwc_csi_device {
	struct device *dev;
	void __iomem *regs;
	void __iomem *glue;
	struct phy *phy;
	int irq;
	struct clk_bulk_data *clks;
	int num_clks;
	struct v4l2_subdev sd;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *source_sd;
	struct fwnode_handle *ep_fwnode;
	struct v4l2_ctrl_handler ctrl_handler;
	struct media_pad pads[DWC_CSI2RX_PADS_NUM];
	u16 remote_pad;

	struct v4l2_mbus_config_mipi_csi2 bus;
	u32 cfgclkfreqrange;
	u32 hsfreqrange;
	/* MIPI D-PHY link frequency (Hz), used to program the PHY HS clock */
	s64 link_freq;
	u64 enabled_streams;

	/* Delay before enabling the IPI interface, from deserializer DT node */
	u32 ipi_enable_delay_ms;

	/* Use driver mutex lock for the ctrl lock */
	struct mutex lock;

	struct dwc_csi_event events[DWC_NUM_EVENTS];
	const struct dwc_csi_pix_format *csi_fmt;
	struct v4l2_dv_timings dv_tmg;

	/* Rate limiter for CRC error live logging */
	struct ratelimit_state crc_err_rs;

	/*
	 * Private high-priority WQ: stop-state (LP-11) poll, deferred IPI
	 * enable, and IPI-overflow recovery.
	 */
	struct workqueue_struct *wq;
	struct work_struct work;
	int poll_state;

	/* Deferred IPI enable, used when an enable delay is configured */
	struct delayed_work ipi_work;
	/* IPI timing mode: false = camera timing (default), true = controller */
	bool ipi_mode_controller;

	/* Enable embedded data on IPI data type register */
	bool ipi_emb_data_en;

	/*
	 * IPI-overflow recovery (SOCSW-5392). On the IPI Interface Fatal IRQ
	 * (CSI pixel overflow when the video DMA halts on an invalid LLI) the
	 * hardirq schedules ovf_recover_work, which resets the CSI + glue and
	 * then asks the downstream DMA to resume via ovf_resume() (registered by
	 * the vdma capture driver, which owns the dma_chan). Rate-limited so a
	 * persistent fault can't reset-storm.
	 */
	struct work_struct ovf_recover_work;
	void (*ovf_quiesce)(void *data);
	void (*ovf_resume)(void *data);
	void *ovf_resume_data;
	bool streaming;
	unsigned int ovf_recover_count;
	ktime_t ovf_last_recover_time;	/* monotonic time of the last recovery, for decay */

	/* Used for pattern generator */
	bool pg_enable;
	enum {
		PATTERN_DISABLED,
		PATTERN_VERTICAL,
		PATTERN_HORIZONTAL,
	} pg_pattern;
};

#define dwc_ipi_read(csidev, reg, n) dwc_csi_read(csidev, ipi_regs[n].reg)
#define dwc_ipi_write(csidev, reg, n, val) dwc_csi_write(csidev, ipi_regs[n].reg, val)
#define dwc_gluen_read(csidev, reg, n) dwc_glue_read(csidev, ipi_regs[n].reg)
#define dwc_gluen_write(csidev, reg, n, val) dwc_glue_write(csidev, ipi_regs[n].reg, val)
#define DWC_CSI_REG(reg, n) CSI2RX_IPI##n##_##reg
#define DWC_GLUE_REG(reg, n) DWC_CSI2GLUE_##reg##n
#define IPI_REG_OP(reg, n, p) .reg = DWC_##p##_REG(reg, n),
#define DWC_IPI_REGS(n) { \
		IPI_REG_OP(MODE, n, CSI) \
		IPI_REG_OP(VCID, n, CSI) \
		IPI_REG_OP(DATA_TYPE, n, CSI) \
		IPI_REG_OP(MEM_FLUSH, n, CSI) \
		IPI_REG_OP(HSA_TIME, n, CSI) \
		IPI_REG_OP(HBP_TIME, n, CSI) \
		IPI_REG_OP(HSD_TIME, n, CSI) \
		IPI_REG_OP(ADV_FEATURES, n, CSI) \
		IPI_REG_OP(VIS_CTRL_PIX, n, GLUE) \
		IPI_REG_OP(DPHY_CTRL, n, GLUE) \
		IPI_REG_OP(PXL_CNT_CTRL, n, GLUE) \
		IPI_REG_OP(CHAN_CTRL, n, GLUE) \
		IPI_REG_OP(STAT_CTRL, n, GLUE) \
		IPI_REG_OP(LNBYTES, n, GLUE) \
		IPI_REG_OP(FRMDIM, n, GLUE) \
		IPI_REG_OP(FILTINFO, n, GLUE) \
		IPI_REG_OP(META, n, GLUE) }

struct dwc_csi_ipi_regs {
	u32 MODE;
	u32 VCID;
	u32 DATA_TYPE;
	u32 MEM_FLUSH;
	u32 HSA_TIME;
	u32 HBP_TIME;
	u32 HSD_TIME;
	u32 ADV_FEATURES;
	u32 VIS_CTRL_PIX;
	u32 DPHY_CTRL;
	u32 PXL_CNT_CTRL;
	u32 CHAN_CTRL;
	u32 STAT_CTRL;
	u32 LNBYTES;
	u32 FRMDIM;
	u32 FILTINFO;
	u32 META;
};

static struct dwc_csi_ipi_regs ipi_regs[4] = {
	DWC_IPI_REGS(1),
	DWC_IPI_REGS(2),
	DWC_IPI_REGS(3),
	DWC_IPI_REGS(4),
};

/* List of supported pixel formats for the subdev */
static const struct dwc_csi_pix_format dwc_csi_formats[] = {
	/* YUV formats */
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.output = MEDIA_BUS_FMT_UYVY8_1X16,
		.data_type = MIPI_CSI2_DT_YUV422_8B,
		.width = 16,
	},
	/* RGB formats */
	{
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.output = MEDIA_BUS_FMT_RGB565_1X16,
		.data_type = MIPI_CSI2_DT_RGB565,
		.width = 16,
	}, {
		.code = MEDIA_BUS_FMT_BGR888_1X24,
		.output = MEDIA_BUS_FMT_RGB888_1X24,
		.data_type = MIPI_CSI2_DT_RGB888,
		.width = 24,
	},
	/* RAW (Bayer and greyscale) formats. */
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.output = MEDIA_BUS_FMT_SBGGR8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
		.width = 8,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.output = MEDIA_BUS_FMT_SGBRG8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
		.width = 8,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.output = MEDIA_BUS_FMT_SGRBG8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
		.width = 8,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.output = MEDIA_BUS_FMT_SRGGB8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
		.width = 8,
	}, {
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.output = MEDIA_BUS_FMT_Y8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
		.width = 8,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.output = MEDIA_BUS_FMT_SBGGR10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
		.width = 10,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.output = MEDIA_BUS_FMT_SGBRG10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
		.width = 10,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.output = MEDIA_BUS_FMT_SGRBG10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
		.width = 10,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.output = MEDIA_BUS_FMT_SRGGB10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
		.width = 10,
	}, {
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.output = MEDIA_BUS_FMT_Y10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
		.width = 10,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.output = MEDIA_BUS_FMT_SBGGR12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
		.width = 12,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.output = MEDIA_BUS_FMT_SGBRG12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
		.width = 12,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.output = MEDIA_BUS_FMT_SGRBG12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
		.width = 12,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.output = MEDIA_BUS_FMT_SRGGB12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
		.width = 12,
	}, {
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.output = MEDIA_BUS_FMT_Y12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
		.width = 12,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.output = MEDIA_BUS_FMT_SBGGR14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
		.width = 14,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.output = MEDIA_BUS_FMT_SGBRG14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
		.width = 14,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.output = MEDIA_BUS_FMT_SGRBG14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
		.width = 14,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.output = MEDIA_BUS_FMT_SRGGB14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
		.width = 14,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR16_1X16,
		.output = MEDIA_BUS_FMT_SBGGR16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
		.width = 16,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG16_1X16,
		.output = MEDIA_BUS_FMT_SGBRG16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
		.width = 16,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG16_1X16,
		.output = MEDIA_BUS_FMT_SGRBG16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
		.width = 16,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB16_1X16,
		.output = MEDIA_BUS_FMT_SRGGB16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
		.width = 16,
	}
};

static const struct v4l2_mbus_framefmt dwc_csi_default_fmt = {
	.code = DWC_CSI2RX_DEF_MBUS_CODE,
	.width = DWC_CSI2RX_DEF_PIX_WIDTH,
	.height = DWC_CSI2RX_DEF_PIX_HEIGHT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SMPTE170M,
	.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_SMPTE170M),
	.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SMPTE170M),
	.quantization = V4L2_QUANTIZATION_LIM_RANGE,
};

static const struct v4l2_dv_timings_cap dwc_csi_dv_tmg_cap = {
	.type = V4L2_DV_BT_656_1120,
	.bt = {
		.min_width = 320,
		.max_width = 4096,
		.min_height = 240,
		.max_height = 3280,
		.min_pixelclock = 40000000,
		.max_pixelclock = 1250000000,
		.standards = 0,
		.capabilities = V4L2_DV_BT_CAP_CUSTOM | V4L2_DV_BT_CAP_INTERLACED
			| V4L2_DV_BT_CAP_PROGRESSIVE,
	},
};

static const struct dwc_csi_pix_format *find_csi_format(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc_csi_formats); i++)
		if (code == dwc_csi_formats[i].code)
			return &dwc_csi_formats[i];

	return &dwc_csi_formats[0];
}

static inline void dwc_csi_write(struct dwc_csi_device *csidev, unsigned int offset, u32 val)
{
	writel(val, csidev->regs + offset);
}

static inline u32 dwc_csi_read(struct dwc_csi_device *csidev, unsigned int offset)
{
	return readl(csidev->regs + offset);
}

static inline void dwc_glue_write(struct dwc_csi_device *csidev, unsigned int offset, u32 val)
{
	writel(val, csidev->glue + offset);
}

static inline u32 dwc_glue_read(struct dwc_csi_device *csidev, unsigned int offset)
{
	return readl(csidev->glue + offset);
}
/*
 * DWC MIPI CSI-2 Host Controller Hardware operation
 */
static int dwc_csi_device_pg_enable(struct dwc_csi_device *csidev)
{
	const struct dwc_csi_pix_format *csi_fmt = csidev->csi_fmt;
	struct v4l2_subdev *sd = &csidev->sd;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 val;
	u32 mask = 0xf;
	int i;

	if (!csidev->pg_enable)
		return 0;

	if (!csi_fmt) {
		dev_err(csidev->dev, "CSI pixel format is NULL\n");
		return -EINVAL;
	}

	if (csi_fmt->data_type != MIPI_CSI2_DT_RGB888) {
		dev_err(csidev->dev, "Pattern generator only support RGB888\n");
		return -EINVAL;
	}

	state = v4l2_subdev_lock_and_get_active_state(sd);
	/* Pattern generator create data stream only according to stream 0 */
	fmt = v4l2_subdev_state_get_format(state, DWC_CSI2RX_PAD_SINK, 0);

	val = CSI2RX_PPI_PG_PATTERN_HRES_HRES(fmt->width);
	dwc_csi_write(csidev, CSI2RX_PPI_PG_PATTERN_HRES, val);

	val = CSI2RX_PPI_PG_PATTERN_VRES_VRES(fmt->height);
	dwc_csi_write(csidev, CSI2RX_PPI_PG_PATTERN_VRES, val);

	val = CSI2RX_PPI_PG_CONFIG_DATA_TYPE(csi_fmt->data_type);
	val |= CSI2RX_PPI_PG_CONFIG_VIR_CHAN(0);
	val |= CSI2RX_PPI_PG_CONFIG_PG_MODE(csidev->pg_pattern);
	dwc_csi_write(csidev, CSI2RX_PPI_PG_CONFIG, val);

	/*
	 * Select line start packets to construct vertical
	 * timing information for IPI interface
	 */
	val = CSI2RX_IPI_ADV_FEATURES_SYNC_EVENT_MODE;
	val |= CSI2RX_IPI_ADV_FEATURES_SYNC_LS_PKT;
	val |= CSI2RX_IPI_ADV_FEATURES_LINE_EVENT_SEL;
	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i))
			dwc_ipi_write(csidev, ADV_FEATURES, i, val);

	val = CSI2RX_PPI_PG_ENABLE_EN;
	dwc_csi_write(csidev, CSI2RX_PPI_PG_ENABLE, val);

	v4l2_subdev_unlock_state(state);

	return 0;
}

static void dwc_csi_device_pg_disable(struct dwc_csi_device *csidev)
{
	dwc_csi_write(csidev, CSI2RX_PPI_PG_ENABLE, 0);
}

static void dwc_csi_ipi_enable(struct dwc_csi_device *csidev)
{
	u32 val;
	u32 mask = 0xf;
	int i;

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++) {
		if ((mask) & (1 << i)) {
			/* Memory is automatically flushed at each Frame Start */
			val = CSI2RX_IPI_MEM_FLUSH_AUTO;
			dwc_ipi_write(csidev, MEM_FLUSH, i, val);

			/* Enable IPI */
			val = dwc_ipi_read(csidev, MODE, i);
			val |= CSI2RX_IPI_MODE_ENABLE;
			dwc_ipi_write(csidev, MODE, i, val);

			val = DWC_CSI2GLUE_PXL_CNT_CTRL_CNT_EN;
			dwc_gluen_write(csidev, PXL_CNT_CTRL, i, val);
			dwc_gluen_write(csidev, STAT_CTRL, i, 0);
		}
	}
}

static void dwc_csi_delayed_ipi_enable(struct work_struct *work)
{
	struct dwc_csi_device *csidev =
		container_of(work, struct dwc_csi_device, ipi_work.work);

	dwc_csi_ipi_enable(csidev);
}

static void dwc_csi_ipi_disable(struct dwc_csi_device *csidev)
{
	int i;
	u32 mask = 0xf;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			dwc_ipi_write(csidev, MODE, i, 0);
			dwc_gluen_write(csidev, CHAN_CTRL, i, 0);
			val = DWC_CSI2GLUE_PXL_CNT_CTRL_CNT_RST;
			dwc_gluen_write(csidev, PXL_CNT_CTRL, i, val);
			val = DWC_CSI2GLUE_STAT_CTRL_RST_CNTS(0xff);
			dwc_gluen_write(csidev, STAT_CTRL, i, val);
		}
}

/*
 * Convert a horizontal interval given in pixels into nanoseconds, referenced
 * to the MIPI clock. The dv-timings supply HSYNC, HFP and HBP in pixels, and
 * the IPI timing registers (HSA_TIME / HBP_TIME / HLINE_TIME) are programmed
 * as a time in ns:
 *
 *   time_ns = (Pixels * 1e9) / mipi_clock(Hz)
 *
 * The reference clock comes from bt.pixelclock, which is the MIPI clock (not
 * the sensor pixel clock) provided through the device tree
 * (simaai,bt-pixelclock, e.g. 500000000 for 500 MHz).
 */
static u32 dwc_csi_pixels_to_ns(struct dwc_csi_device *csidev, u32 pixels)
{
	u64 pixelclock = csidev->dv_tmg.bt.pixelclock;

	if (!pixelclock)
		return 0;

	return div_u64((u64)pixels * 1000000000ULL, pixelclock);
}

/*
 * Calculate the IPI horizontal active PHY time, i.e. the time to receive the
 * active pixels over the D-PHY:
 *
 *   Hactive_phy_time = (Frame Width * Data type * 1000) /
 *                      (Data Rate * No. of Lanes)
 *
 *   Frame Width  - active horizontal resolution in pixels
 *   Data type    - bits per pixel of the CSI data type
 *   Data Rate    - D-PHY data rate per lane in Mbps (2 * link freq, DDR)
 *   No. of Lanes - number of active MIPI data lanes
 */
static u32 dwc_csi_calc_hactive_phy_time(struct dwc_csi_device *csidev, u32 width)
{
	const struct dwc_csi_pix_format *csi_fmt = csidev->csi_fmt;
	u32 num_lanes = csidev->bus.num_data_lanes;
	u64 data_rate_mbps;

	/* MIPI D-PHY data rate per lane (Mbps) = 2 * link frequency (DDR) */
	data_rate_mbps = div_u64(csidev->link_freq * 2, 1000000);

	if (!data_rate_mbps || !num_lanes)
		return 0;

	return div_u64((u64)width * csi_fmt->width * 1000,
		       data_rate_mbps * num_lanes);
}

static void dwc_csi_device_ipi_config(struct dwc_csi_device *csidev,
				      struct v4l2_subdev_state *state)
{
	const struct dwc_csi_pix_format *csi_fmt = csidev->csi_fmt;
	const struct v4l2_bt_timings *bt_tmg = &csidev->dv_tmg.bt;
	const struct v4l2_mbus_framefmt *fmt;
	u32 hsa_time, hbp_time, hfp_time, hactive_time;
	u32 hline_time;
	u32 width, height;
	u32 val = 0;
	u32 mask = 0xf;
	int i;

	/* Active resolution comes from the format set through .set_fmt */
	fmt = v4l2_subdev_state_get_format(state, DWC_CSI2RX_PAD_SINK, 0);
	width = fmt->width;
	height = fmt->height;

	/* HSYNC/HBP/HFP come from dv-timings in pixels; convert to ns at pixel clock */
	hsa_time = dwc_csi_pixels_to_ns(csidev, bt_tmg->hsync);
	hbp_time = dwc_csi_pixels_to_ns(csidev, bt_tmg->hbackporch);
	hfp_time = dwc_csi_pixels_to_ns(csidev, bt_tmg->hfrontporch);
	/* Hactive uses the D-PHY active receive time */
	hactive_time = dwc_csi_calc_hactive_phy_time(csidev, width);
	dev_dbg(csidev->dev, "HACTIVE_TIME: %u\n", hactive_time);

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			/* Select virtual channel and data type to be processed by IPI */
			val = CSI2RX_IPI_DATA_TYPE_DT(csi_fmt->data_type);
			if (csidev->ipi_emb_data_en)
				val |= CSI2RX_IPI_DATA_TYPE_EMB_DATA_EN;
			dwc_ipi_write(csidev, DATA_TYPE, i, val);

			/* Set virtual channel 0 as default */
			val  = CSI2RX_IPI_VCID_VC(0);
			dwc_ipi_write(csidev, VCID, i, CSI2RX_IPI_VCID_VC(i));

			/*
			 * Select IPI camera timing mode and allow the pixel stream
			 * to be non-continuous when pixel interface FIFO is empty
			 */
			val = dwc_ipi_read(csidev, MODE, i);
			if (csidev->ipi_mode_controller)
				val |= CSI2RX_IPI_MODE_CONTROLLER;
			else
				val &= ~CSI2RX_IPI_MODE_CONTROLLER;
			val &= ~CSI2RX_IPI_MODE_COLOR_MODE16;
			val |= CSI2RX_IPI_MODE_CUT_THROUGH;
			dwc_ipi_write(csidev, MODE, i, val);
		}

	val = CSI2RX_IPI_ADV_FEATURES_SYNC_LS_PKT;
	val |= CSI2RX_IPI_ADV_FEATURES_LINE_EVENT_SEL;
	val |= CSI2RX_IPI_ADV_FEATURES_SYNC_VIDEO_PKT;
	val |= CSI2RX_IPI_ADV_FEATURES_SYNC_EMBEDDED_PKT;

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			dwc_ipi_write(csidev, HSA_TIME, i,
				CSI2RX_IPI_HSA_TIME_VAL(hsa_time));
			dwc_ipi_write(csidev, HBP_TIME, i,
				CSI2RX_IPI_HBP_TIME_VAL(hbp_time));
			/* HSD time is fixed to a constant */
			dwc_ipi_write(csidev, HSD_TIME, i,
				CSI2RX_IPI_HSD_TIME_VAL(DWC_CSI2RX_HSD_TIME));
			dwc_ipi_write(csidev, ADV_FEATURES, i, val);
		}

	/* HLINE_TIME = HSA + HBP + HFP + Hactive PHY time */
	hline_time = hsa_time + hbp_time + hfp_time + hactive_time;
	if (csidev->ipi_mode_controller)
		dev_info(csidev->dev,
			 "hsa_time: %u, hbp_time: %u, hfp_time: %u, hactive_time: %u, hline_time: %u, width: %u, height: %u\n",
			 hsa_time, hbp_time, hfp_time, hactive_time, hline_time, width, height);
	val = CSI2RX_IPI_HLINE_TIME_VAL((u32)(hline_time/2));
	dwc_csi_write(csidev, CSI2RX_IPI_HLINE_TIME, val);
	val = CSI2RX_IPI_VSA_LINES_VAL(bt_tmg->vsync);
	dwc_csi_write(csidev, CSI2RX_IPI_VSA_LINES, val);
	val = CSI2RX_IPI_VBP_LINES_VAL(bt_tmg->vbackporch);
	dwc_csi_write(csidev, CSI2RX_IPI_VBP_LINES, val);
	val = CSI2RX_IPI_VFP_LINES_VAL(bt_tmg->vfrontporch);
	dwc_csi_write(csidev, CSI2RX_IPI_VFP_LINES, val);
	val = CSI2RX_IPI_VACTIVE_LINES_VAL(height);
	dwc_csi_write(csidev, CSI2RX_IPI_VACTIVE_LINES, val);
	dwc_csi_write(csidev, CSI2RX_VIRTUAL_CHANNEL_EXT, 0x0);

	/* Configure glue logic */
	val = DWC_CSI2GLUE_FILTINFO_SRC_BURST_LEN(0x8);
	val |= DWC_CSI2GLUE_VIS_CTRL_BUF_SRAM_PIPE_EN;
	dwc_glue_write(csidev, DWC_CSI2GLUE_VIS_CTRL, val);

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			val = DWC_CSI2GLUE_CHAN_CTRL_EN;
			///TODO: Clarify why 8 Dbits
			val |= DWC_CSI2GLUE_CHAN_CTRL_DBITS(0x8);
			val |= DWC_CSI2GLUE_CHAN_CTRL_PACK;
			dwc_gluen_write(csidev, CHAN_CTRL, i, val);

			///TODO: Clarify these hardcoded values
			val = DWC_CSI2GLUE_VIS_CTRL_PIX_BUF_FULL_TH(0x38);
			val |= DWC_CSI2GLUE_VIS_CTRL_PIX_END_PATTERN(0xBEEF);
			val |= DWC_CSI2GLUE_VIS_CTRL_PIX_XFER_FRAME_BOUND;
			val |= DWC_CSI2GLUE_VIS_CTRL_PIX_FRAME_FMT(0x4);
			dwc_gluen_write(csidev, VIS_CTRL_PIX, i, val);

			val = DWC_CSI2GLUE_LNBYTES_LINE_BYTES(width);
			dwc_gluen_write(csidev, LNBYTES, i, val);

			val = DWC_CSI2GLUE_FRMDIM_WIDTH(width);
			val |= DWC_CSI2GLUE_FRMDIM_HEIGHT(height);
			dwc_gluen_write(csidev, FRMDIM, i, val);

			val = DWC_CSI2GLUE_FILTINFO_DT(csi_fmt->data_type);
			val |= DWC_CSI2GLUE_FILTINFO_CHAN(0);
			dwc_gluen_write(csidev, FILTINFO, i, val);

			val = DWC_CSI2GLUE_META_EMB_LD(bt_tmg->il_vfrontporch * width *
				csi_fmt->width / 8);
			val |= DWC_CSI2GLUE_META_EMB_TR(bt_tmg->il_vbackporch * width *
				csi_fmt->width / 8);
			dwc_gluen_write(csidev, META, i, val);
		}

	/* Do IPI soft reset */
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, 0x0);
	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i))
			val |= 1 << (i * 4);
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, val);
}

static void dwc_csi_device_reset(struct dwc_csi_device *csidev)
{
	/* Reset mipi csi host, active low */
	dwc_csi_write(csidev, CSI2RX_HOST_RESETN, 0);
	dwc_csi_write(csidev, CSI2RX_HOST_RESETN, 1);
}

static void dwc_csi_device_startup(struct dwc_csi_device *csidev)
{
	/* Release DWC_mipi_csi2_host from reset */
	dwc_csi_device_reset(csidev);

	phy_init(csidev->phy);

	phy_reset(csidev->phy);
}

static int dwc_csi_get_dphy_configuration(struct dwc_csi_device *csidev,
					  union phy_configure_opts *opts)
{
	struct phy_configure_opts_mipi_dphy *cfg = &opts->mipi_dphy;

	memset(cfg, 0x0, sizeof(*cfg));
	cfg->hs_clk_rate = csidev->link_freq * 2;
	cfg->lanes = csidev->bus.num_data_lanes;

	return 0;
}

static void dwc_csi_dphy_prep(struct dwc_csi_device *csidev)
{
	u32 val;

	/* Release synopsis DPHY test codes from reset */
	dwc_csi_write(csidev, CSI2RX_DPHY_RSTZ, 0x0);
	dwc_csi_write(csidev, CSI2RX_DPHY_SHUTDOWNZ, 0x0);

	val = dwc_csi_read(csidev, CSI2RX_DPHY_TEST_CTRL0);
	val &= ~CSI2RX_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_csi_write(csidev, CSI2RX_DPHY_TEST_CTRL0, val);

	/*
	 * ndelay is not necessary have MMIO operation, need dummy read to make
	 * sure above write reach target.
	 */
	dwc_csi_read(csidev, CSI2RX_DPHY_TEST_CTRL0);
	/* Wait for at least 15ns */
	ndelay(15);

	val = dwc_csi_read(csidev, CSI2RX_DPHY_TEST_CTRL0);
	val |= CSI2RX_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_csi_write(csidev, CSI2RX_DPHY_TEST_CTRL0, val);
}

static void dwc_csi_dphy_release_reset(struct dwc_csi_device *csidev)
{
	/* Release PHY from reset */
	dwc_csi_write(csidev, CSI2RX_DPHY_SHUTDOWNZ, 0x1);
	/*
	 * ndelay is not necessary have MMIO operation, need dummy read to make
	 * sure above write reach target.
	 */
	dwc_csi_read(csidev, CSI2RX_DPHY_SHUTDOWNZ);
	ndelay(5);
	dwc_csi_write(csidev, CSI2RX_DPHY_RSTZ, 0x1);
	dwc_csi_read(csidev, CSI2RX_DPHY_RSTZ);
	ndelay(5);
}

static void stopstate_poll(struct work_struct *work)
{
	struct dwc_csi_device *csidev = container_of(work, typeof(*csidev), work);
	u32 phy_stopstate;
	u32 val;
	u32 mask = 0xf;
	int ret;
	int i;

	csidev->poll_state = DWC_POLL_ACTIVE;
	/* Check if lanes are in stop state */
	phy_stopstate = CSI2RX_DPHY_STOPSTATE_CLK_LANE;
	phy_stopstate |= GENMASK(csidev->bus.num_data_lanes - 1, 0);
	ret = readl_poll_timeout(csidev->regs + CSI2RX_DPHY_STOPSTATE,
				 val, (val & phy_stopstate) == phy_stopstate,
				 10, DWC_STOPSTATE_TIMEOUT);
	if (ret) {
		dev_err(csidev->dev, "Lanes are not in stop state(%#x)\n", val);
		csidev->poll_state = DWC_POLL_TIMEOUT;
	} else {
		for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
			if ((mask) & (1 << i)) {
				val = dwc_gluen_read(csidev, DPHY_CTRL, i);
				val &= ~DWC_CSI2GLUE_DPHY_CTRL_FORCE_MODE;
				dwc_gluen_write(csidev, DPHY_CTRL, i, val);
			}
		csidev->poll_state = DWC_POLL_SUCCESS;
	}
}

static int dwc_csi_device_init(struct dwc_csi_device *csidev)
{
	union phy_configure_opts opts;
	int ret;
	ktime_t end;

	ret = dwc_csi_get_dphy_configuration(csidev, &opts);
	if (ret)
		return ret;

	ret = phy_set_mode(csidev->phy, PHY_MODE_MIPI_DPHY);
	if (ret)
		return ret;

	ret = phy_configure(csidev->phy, &opts);
	if (ret)
		return ret;

	dwc_csi_write(csidev, CSI2RX_HOST_RESETN, 0);
	dwc_csi_dphy_prep(csidev);
	dwc_csi_write(csidev, CSI2RX_N_LANES, CSI2RX_N_LANES_N_LANES(opts.mipi_dphy.lanes));
	ret = phy_power_on(csidev->phy);
	dwc_csi_dphy_release_reset(csidev);
	dwc_csi_write(csidev, CSI2RX_HOST_RESETN, 0x1);

	if (ret)
		return ret;

	/* Wait until other polls are done*/
	end = ktime_add_us(ktime_get(), DWC_STOPSTATE_TIMEOUT);
	while (csidev->poll_state != DWC_POLL_INACTIVE) {
		if (ktime_after(ktime_get(), end)) {
			/*
			 * The previous poll should have expired on its own by
			 * now. Make sure its work item is no longer pending or
			 * running before we override the state and take over,
			 * so it can't race our poll_state writes below.
			 */
			dev_warn(csidev->dev, "Overriding stale stop-state poll(%d)\n",
				 csidev->poll_state);
			cancel_work_sync(&csidev->work);
			csidev->poll_state = DWC_POLL_INACTIVE;
			break;
		}
		udelay(1);
	}

	csidev->poll_state = DWC_POLL_SCHEDULED;
	/* Queue the stopstate poll on the high-priority WQ */
	queue_work(csidev->wq, &csidev->work);
	/*
	 * We need to wait only that work is at least active here
	 * We will wait for work completion after stream is enabled in sensor
	 */
	while (csidev->poll_state == DWC_POLL_SCHEDULED)
		udelay(1);

	return 0;
}

static int dwc_csi_device_hs_rx_start(struct dwc_csi_device *csidev)
{
	/* No delay: enable inline to avoid scheduling latency. */
	if (!csidev->ipi_enable_delay_ms) {
		dwc_csi_ipi_enable(csidev);
		return 0;
	}

	queue_delayed_work(csidev->wq, &csidev->ipi_work,
			   msecs_to_jiffies(csidev->ipi_enable_delay_ms));

	return 0;
}

static int dwc_csi_device_hs_rx_stop(struct dwc_csi_device *csidev)
{
	struct device *dev = csidev->dev;
	u32 val;

	/* Cancel pending delayed enable so it can't fire after teardown. */
	cancel_delayed_work_sync(&csidev->ipi_work);

	/*
	 * Quiesce the IPI (MODE=0 + glue CHAN_CTRL/counter reset) BEFORE cutting
	 * the PHY, mirroring the start order in reverse (start enables the IPI
	 * last). Powering off the PHY while the IPI is still enabled removes its
	 * input mid-frame and faults the pixel FIFO -- the noisy stop-time "IPI
	 * Interface Fatal". Disabling the IPI first drains it so it never fatals.
	 */
	dwc_csi_ipi_disable(csidev);

	phy_power_off(csidev->phy);
	phy_exit(csidev->phy);

	/* Check clock lanes are not in High Speed Mode */
	val = dwc_csi_read(csidev, CSI2RX_DPHY_RX_STATUS);
	if (val & CSI2RX_DPHY_RX_STATUS_CLK_LANE_HS) {
		dev_err(dev, "Clock lanes are still in HS mode\n");
		return -EINVAL;
	}

	return 0;
}

static void dwc_csi_device_enable_interrupts(struct dwc_csi_device *csidev, bool on)
{
	/* Define errors to be enabled */
	dwc_csi_write(csidev, CSI2RX_INT_MSK_DPHY_FATAL, on ? 0x3 : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_PKT_FATAL, on ? 0x3 : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_DPHY, on ? 0x30003 : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_IPI_FATAL, on ? 0x7f : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_PLD_CRC_FATAL,
		      on ? 0xFFFFFFFF : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_CRC_FRAME_FATAL,
		      on ? 0xFFFFFFFF : 0);
}

static void dwc_csi_clear_counters(struct dwc_csi_device *csidev)
{
	unsigned int i;

	for (i = 0; i < DWC_NUM_EVENTS; ++i)
		csidev->events[i].counter = 0;
}

static void dwc_csi_log_counters(struct dwc_csi_device *csidev)
{
	unsigned int i;
	int counter;

	for (i = 0; i < DWC_NUM_EVENTS; ++i) {
		counter = csidev->events[i].counter;
		if (counter > 0)
			dev_info(csidev->dev, "%s events: %d\n",
				 csidev->events[i].name,
				 counter);
	}
}

static void dwc_csi_dump_regs(struct dwc_csi_device *csidev)
{
#define DWC_MIPI_CSIS_DEBUG_REG(name)		{name, #name}
	static const struct {
		u32 offset;
		const char * const name;
	} registers[] = {
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_VERSION),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_N_LANES),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_HOST_RESETN),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_MAIN),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DATA_IDS_1_DT),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DATA_IDS_2_DT),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DATA_IDS_1_VC),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DATA_IDS_2_VC),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DPHY_SHUTDOWNZ),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DPHY_RSTZ),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DPHY_RX_STATUS),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DPHY_STOPSTATE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DPHY_TEST_CTRL0),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_DPHY_TEST_CTRL1),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_PPI_PG_PATTERN_VRES),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_PPI_PG_PATTERN_HRES),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_PPI_PG_CONFIG),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_PPI_PG_ENABLE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_PPI_PG_STATUS),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI1_MODE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI1_VCID),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI1_DATA_TYPE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI1_MEM_FLUSH),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI_SOFTRSTN),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI1_ADV_FEATURES),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_DPHY_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_PKT_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_DPHY_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_IPI_FATAL),
	};

	unsigned int i;
	u32 cfg;

	dev_dbg(csidev->dev, "--- REGISTERS ---\n");

	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		cfg = dwc_csi_read(csidev, registers[i].offset);
		dev_dbg(csidev->dev, "%14s[0x%02x]: 0x%08x\n",
			registers[i].name, registers[i].offset, cfg);
	}
}

/*
 * V4L2 subdev operations
 */

static inline struct dwc_csi_device *
sd_to_dwc_csi_device(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct dwc_csi_device, sd);
}

static int __dwc_csi_subdev_set_routing(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_subdev_krouting *routing)
{
	int ret;

	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -EINVAL;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	return v4l2_subdev_set_routing_with_fmt(sd, state, routing,
						&dwc_csi_default_fmt);
}

static int dwc_csi_subdev_init_state(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = DWC_CSI2RX_PAD_SINK,
			.sink_stream = 0,
			.source_pad = DWC_CSI2RX_PAD_SOURCE,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = DWC_CSI2RX_PAD_SINK + 2,
			.sink_stream = 0,
			.source_pad = DWC_CSI2RX_PAD_SOURCE + 2,
			.source_stream = 1,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = DWC_CSI2RX_PAD_SINK + 4,
			.sink_stream = 0,
			.source_pad = DWC_CSI2RX_PAD_SOURCE + 4,
			.source_stream = 2,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = DWC_CSI2RX_PAD_SINK + 6,
			.sink_stream = 0,
			.source_pad = DWC_CSI2RX_PAD_SOURCE + 6,
			.source_stream = 3,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	return __dwc_csi_subdev_set_routing(sd, sd_state, &routing);
}

static int dwc_csi_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_mbus_code_enum *code)
{
	/*
	 * The CSIS can't transcode in any way, the source format is identical
	 * to the sink format.
	 */
	if (code->pad == DWC_CSI2RX_PAD_SOURCE) {
		struct v4l2_mbus_framefmt *fmt;

		if (code->index > 0)
			return -EINVAL;

		fmt = v4l2_subdev_state_get_format(sd_state, code->pad,
						   code->stream);
		code->code = fmt->code;
		return 0;
	}

	if (code->index >= ARRAY_SIZE(dwc_csi_formats))
		return -EINVAL;

	code->code = dwc_csi_formats[code->index].code;

	return 0;
}

static int dwc_csi_subdev_set_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *sdformat)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	struct dwc_csi_pix_format const *csi_fmt;
	struct v4l2_mbus_framefmt *sink_fmt, *src_fmt;
	unsigned int align;

	/*
	 * The CSIS can't transcode in any way, the source format can't be
	 * modified.
	 */
	if (sdformat->pad == DWC_CSI2RX_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, sd_state, sdformat);

	/*
	 * Validate the media bus code and clamp and align the size.
	 *
	 * The total number of bits per line must be a multiple of 8. We thus
	 * need to align the width for formats that are not multiples of 8
	 * bits.
	 */
	csi_fmt = find_csi_format(sdformat->format.code);

	switch (csi_fmt->width % 8) {
	case 0:
		align = 0;
		break;
	case 4:
		align = 1;
		break;
	case 2:
	case 6:
		align = 2;
		break;
	default:
		/* 1, 3, 5, 7 */
		align = 3;
		break;
	}

	v4l_bound_align_image(&sdformat->format.width, 1,
			      DWC_CSI2RX_MAX_PIX_WIDTH, align,
			      &sdformat->format.height, 1,
			      DWC_CSI2RX_MAX_PIX_HEIGHT, 0, 0);

	/* Set default code if user set an invalid value */
	sdformat->format.code = csi_fmt->code;
	sdformat->format.field = V4L2_FIELD_NONE;

	sink_fmt = v4l2_subdev_state_get_format(sd_state, sdformat->pad,
						sdformat->stream);
	if (!sink_fmt)
		return -EINVAL;
	*sink_fmt = sdformat->format;

	/* Propagate the format from sink stream to source stream */
	src_fmt = v4l2_subdev_state_get_opposite_stream_format(sd_state,
							       sdformat->pad,
							       sdformat->stream);
	if (!src_fmt)
		return -EINVAL;
	*src_fmt = sdformat->format;
	/* The format on the source pad might change due to unpacking. */
	src_fmt->code = csi_fmt->output;

	if (sdformat->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	/* Store the CSIS format descriptor for active formats. */
	csidev->csi_fmt = csi_fmt;

	if (csidev->source_sd) {
		struct v4l2_subdev_format up = *sdformat;
		struct v4l2_subdev_state *ust;
		int ret = -ENODEV;

		ust = v4l2_subdev_lock_and_get_active_state(csidev->source_sd);
		if (ust) {
			ret = v4l2_subdev_routing_find_opposite_end(&ust->routing,
					csidev->remote_pad, sdformat->stream,
					&up.pad, &up.stream);
			v4l2_subdev_unlock_state(ust);
		}
		if (!ret)
			v4l2_subdev_call_state_active(csidev->source_sd, pad, set_fmt, &up);
	}

	return 0;
}

static int dwc_csi_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	struct v4l2_mbus_frame_desc source_fd;
	struct v4l2_subdev_route *route;
	struct v4l2_subdev_state *state;
	int ret;

	if (pad != DWC_CSI2RX_PAD_SOURCE)
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));

	ret = v4l2_subdev_call(csidev->source_sd, pad, get_frame_desc,
			       csidev->remote_pad, &source_fd);
	if (ret < 0) {
		dev_info(csidev->dev,
			 "Remote sub-device on pad %d should implement .get_frame_desc! Forcing VC = 0 and DT = %x\n",
			 pad, csidev->csi_fmt->data_type);
		fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = 1;
		fd->entry[0].pixelcode = csidev->csi_fmt->code;
		fd->entry[0].bus.csi2.vc = 0;
		fd->entry[0].bus.csi2.dt = csidev->csi_fmt->data_type;

		return 0;
	}

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_frame_desc_entry *entry = NULL;
		unsigned int i;

		if (route->source_pad != pad)
			continue;

		for (i = 0; i < source_fd.num_entries; ++i) {
			if (source_fd.entry[i].stream == route->sink_stream) {
				entry = &source_fd.entry[i];
				break;
			}
		}

		if (!entry) {
			dev_err(csidev->dev,
				"Failed to find stream from source frames desc\n");
			ret = -EPIPE;
			goto out_unlock;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;
		fd->entry[fd->num_entries].flags = entry->flags;
		fd->entry[fd->num_entries].length = entry->length;
		fd->entry[fd->num_entries].pixelcode = entry->pixelcode;
		fd->entry[fd->num_entries].bus.csi2.vc = entry->bus.csi2.vc;
		fd->entry[fd->num_entries].bus.csi2.dt = entry->bus.csi2.dt;

		fd->num_entries++;
	}

out_unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int dwc_csi_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	if (which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    media_entity_is_streaming(&sd->entity))
		return -EBUSY;

	return __dwc_csi_subdev_set_routing(sd, state, routing);
}

static int dwc_csi_start_stream(struct dwc_csi_device *csidev,
				struct v4l2_subdev_state *state)
{
	int ret;

	dwc_csi_device_startup(csidev);

	dwc_csi_device_ipi_config(csidev, state);

	ret = dwc_csi_device_init(csidev);
	if (ret)
		return ret;

	ret = dwc_csi_device_pg_enable(csidev);
	if (ret)
		goto err_cancel_poll;

	ret = dwc_csi_device_hs_rx_start(csidev);
	if (ret)
		goto err_cancel_poll;

	csidev->ovf_recover_count = 0;
	csidev->streaming = true;
	dwc_csi_device_enable_interrupts(csidev, true);

	return 0;

err_cancel_poll:
	/* device_init queued the stop-state poll; don't leave it running. */
	cancel_work_sync(&csidev->work);
	csidev->poll_state = DWC_POLL_INACTIVE;
	return ret;
}

static void dwc_csi_stop_stream(struct dwc_csi_device *csidev)
{
	csidev->streaming = false;
	dwc_csi_device_enable_interrupts(csidev, false);
	/* No new overflow IRQs will queue now; drain any in-flight recovery. */
	cancel_work_sync(&csidev->ovf_recover_work);
	dwc_csi_device_hs_rx_stop(csidev);
	dwc_csi_device_pg_disable(csidev);
}

static int dwc_csi_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	u64 sink_streams;
	bool started = false;
	int ret;
	int poll_result;

	if (!csidev->source_sd) {
		dev_err(csidev->dev, "Sensor don't link with CSIS pad\n");
		return -EPIPE;
	}

	if (!csidev->enabled_streams) {
		ret = pm_runtime_resume_and_get(csidev->dev);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_handler_setup(&csidev->ctrl_handler);
		if (ret < 0)
			goto err_runtime_put;

		dwc_csi_clear_counters(csidev);

		ret = dwc_csi_start_stream(csidev, state);
		if (ret < 0)
			goto err_runtime_put;

		started = true;
		dwc_csi_dump_regs(csidev);
		dwc_csi_log_counters(csidev);
	}

	sink_streams = v4l2_subdev_state_xlate_streams(state, DWC_CSI2RX_PAD_SOURCE,
						       DWC_CSI2RX_PAD_SINK,
						       &streams_mask);

	dev_dbg(csidev->dev, "remote sd: %s pad: %u, sink_stream:0x%llx\n",
		csidev->source_sd->name, csidev->remote_pad, sink_streams);

	ret = v4l2_subdev_enable_streams(csidev->source_sd, csidev->remote_pad,
					 sink_streams);
	if (ret == -EALREADY && started) {
		/* Earlier failed start left the sensor stream enabled and
		 * driving the lanes: disable it, restart the receiver, retry. */
		dev_warn(csidev->dev,
			 "stale enabled sensor stream (earlier failed start); recovering\n");
		v4l2_subdev_disable_streams(csidev->source_sd,
					    csidev->remote_pad, sink_streams);
		cancel_work_sync(&csidev->work);
		csidev->poll_state = DWC_POLL_INACTIVE;
		dwc_csi_stop_stream(csidev);
		ret = dwc_csi_start_stream(csidev, state);
		if (!ret)
			ret = v4l2_subdev_enable_streams(csidev->source_sd,
							 csidev->remote_pad,
							 sink_streams);
	}

	/* Wait for poll work to complete and store it's result*/
	flush_work(&csidev->work);
	poll_result = csidev->poll_state;
	csidev->poll_state = DWC_POLL_INACTIVE;

	if (ret)
		goto err_stop_stream;

	if (started && poll_result != DWC_POLL_SUCCESS) {
		ret = poll_result == DWC_POLL_TIMEOUT ? -ETIMEDOUT : -EPROTO;
		/* Wind the sensor back or no later start can succeed. */
		v4l2_subdev_disable_streams(csidev->source_sd,
					    csidev->remote_pad, sink_streams);
		goto err_stop_stream;
	}

	csidev->enabled_streams |= streams_mask;

	return 0;

err_stop_stream:
	if (started) {
		dwc_csi_stop_stream(csidev);
		pm_runtime_put(csidev->dev);
	}
	return ret;

err_runtime_put:
	pm_runtime_put(csidev->dev);
	return ret;
}

static int dwc_csi_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	u64 sink_streams;
	int ret;

	sink_streams = v4l2_subdev_state_xlate_streams(state, DWC_CSI2RX_PAD_SOURCE,
						       DWC_CSI2RX_PAD_SINK,
						       &streams_mask);
	ret = v4l2_subdev_disable_streams(csidev->source_sd, csidev->remote_pad,
					  sink_streams);
	if (ret)
		return ret;
	csidev->enabled_streams &= ~streams_mask;
	if (!csidev->enabled_streams) {
		dwc_csi_stop_stream(csidev);
		dwc_csi_log_counters(csidev);
		pm_runtime_put(csidev->dev);
	}
	return 0;
}

/*
 * Lightweight range check used instead of v4l2_valid_dv_timings().
 *
 * The core helper rejects any blanking interval larger than 3 * width, which
 * prevents the large front/back porches we need to stretch the IPI line time
 * in controller mode. Here we only validate the bounds that actually matter
 * for the hardware (frame type, active resolution and pixel clock) and let the
 * porches be arbitrary.
 */
static bool dwc_csi_timings_in_range(const struct v4l2_dv_timings *t,
				     const struct v4l2_dv_timings_cap *cap)
{
	const struct v4l2_bt_timings *bt = &t->bt;
	const struct v4l2_bt_timings_cap *bcap = &cap->bt;

	if (t->type != cap->type)
		return false;

	if (bt->width < bcap->min_width || bt->width > bcap->max_width ||
	    bt->height < bcap->min_height || bt->height > bcap->max_height)
		return false;

	if (bt->pixelclock < bcap->min_pixelclock ||
	    bt->pixelclock > bcap->max_pixelclock)
		return false;

	return true;
}

static int dwc_csi_s_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_dv_timings *timings)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	if (v4l2_match_dv_timings(&csidev->dv_tmg, timings, 0, false))
		return 0; /* no changes */

	if (!dwc_csi_timings_in_range(timings, &dwc_csi_dv_tmg_cap))
		return -ERANGE;

	/*
	 * We do not support applying dv timing on the fly.
	 * Settings will be applied on the next stream on.
	 */
	csidev->dv_tmg = *timings;

	return 0;
}

static int dwc_csi_g_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_dv_timings *timings)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	*timings = csidev->dv_tmg;

	return 0;
}

static int dwc_csi_query_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				     struct v4l2_dv_timings *timings)
{
	return dwc_csi_g_dv_timings(sd, pad, timings);
}

static int dwc_csi_get_dv_timings_cap(struct v4l2_subdev *sd,
				       struct v4l2_dv_timings_cap *cap)
{
	*cap = dwc_csi_dv_tmg_cap;
	return 0;
}

static int dwc_csi_enum_dv_timings(struct v4l2_subdev *sd,
				    struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &dwc_csi_dv_tmg_cap,
					NULL, NULL);
}

static int dwc_csi_subdev_log_status(struct v4l2_subdev *sd)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	dwc_csi_log_counters(csidev);
	return 0;
}

static const struct v4l2_subdev_core_ops dwc_csi_subdev_core_ops = {
	.log_status = dwc_csi_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_pad_ops dwc_csi_subdev_pad_ops = {
	.enum_mbus_code	= dwc_csi_subdev_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = dwc_csi_subdev_set_fmt,
	.get_frame_desc = dwc_csi_get_frame_desc,
	.set_routing = dwc_csi_set_routing,
	.enable_streams = dwc_csi_enable_streams,
	.disable_streams = dwc_csi_disable_streams,
	.s_dv_timings = dwc_csi_s_dv_timings,
	.g_dv_timings = dwc_csi_g_dv_timings,
	.query_dv_timings = dwc_csi_query_dv_timings,
	.dv_timings_cap = dwc_csi_get_dv_timings_cap,
	.enum_dv_timings = dwc_csi_enum_dv_timings,
};

static const struct v4l2_subdev_ops dwc_csi_subdev_ops = {
	.core  = &dwc_csi_subdev_core_ops,
	.pad   = &dwc_csi_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops dwc_csi_internal_ops = {
	.init_state = dwc_csi_subdev_init_state,
};

/*
 * Media entity operations
 */

static int dwc_csi_link_setup(struct media_entity *entity,
			      const struct media_pad *local_pad,
			      const struct media_pad *remote_pad, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	struct v4l2_subdev *remote_sd;

	dev_dbg(csidev->dev, "link setup %s -> %s", remote_pad->entity->name,
		local_pad->entity->name);

	/* We only care about the link to the source. */
	if (!(local_pad->flags & MEDIA_PAD_FL_SINK))
		return 0;

	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (csidev->source_sd)
			return -EBUSY;

		csidev->source_sd = remote_sd;
		csidev->remote_pad = remote_pad->index;
	} else {
		csidev->source_sd = NULL;
	}

	return 0;
}

static int dwc_csi_link_validate(struct media_link *link)
{
	struct media_pad *sink_pad = link->sink;
	struct v4l2_subdev *sink_sd;
	struct dwc_csi_device *csidev;

	sink_sd = media_entity_to_v4l2_subdev(sink_pad->entity);
	csidev = sd_to_dwc_csi_device(sink_sd);

	dev_dbg(csidev->dev, "entity name:%s pad index=%d\n",
		sink_sd->name, sink_pad->index);

	/*
	 * Skip link validate when pattern enabled since the source
	 * data will be from CSI pattern generator, not sensor.
	 */
	if (csidev->pg_enable && sink_pad->index == DWC_CSI2RX_PAD_SINK)
		return 0;

	return v4l2_subdev_link_validate(link);
}

static bool dwc_csi_pad_interdep(struct media_entity *entity, unsigned int pad0,
				 unsigned int pad1)
{
	return (((pad0 & 1) && (pad0 == pad1 + 1)) || ((pad1 & 1) && (pad1 == pad0 + 1)));
}

static const struct media_entity_operations dwc_csi_entity_ops = {
	.link_setup	= dwc_csi_link_setup,
	.link_validate	= dwc_csi_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.has_pad_interdep = dwc_csi_pad_interdep,
};

/*
 * Async subdev notifier
 */

static inline struct dwc_csi_device *
notifier_to_dwc_csi_device(struct v4l2_async_notifier *n)
{
	return container_of(n, struct dwc_csi_device, notifier);
}

static int dwc_csi_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_connection *asd)
{
	struct dwc_csi_device *csidev = notifier_to_dwc_csi_device(notifier);
	struct media_pad *sink = &csidev->sd.entity.pads[DWC_CSI2RX_PAD_SINK];
	s64 link_freq;
	int ret;

	ret = media_entity_get_fwnode_pad(&sd->entity, csidev->ep_fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(csidev->dev, "Failed to find pad for %s\n", sd->name);
		return ret;
	}

	csidev->source_sd = sd;
	csidev->remote_pad = ret;

	/*
	 * Allow the deserializer to override the delay applied before the IPI
	 * interface is enabled. Left untouched (default) if the property is
	 * absent or the source has no associated device node.
	 */
	if (sd->dev)
		device_property_read_u32(sd->dev, "simaai,ipi-enable-delay-ms",
					 &csidev->ipi_enable_delay_ms);

	ret = v4l2_create_fwnode_links_to_pad(sd, sink, MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(csidev->dev, "Failed to link pad with %s\n", sd->name);
		return ret;
	}

	link_freq = v4l2_get_link_freq(&sd->entity.pads[csidev->remote_pad],
				       csidev->csi_fmt->width,
				       csidev->bus.num_data_lanes * 2);
	if (link_freq < 0) {
		dev_err(csidev->dev, "Unable to obtain link frequency: %d\n",
			(int)link_freq);
		return link_freq;
	}

	/*
	 * link_freq is the sensor's MIPI link frequency used to compute the
	 * D-PHY data rate. bt.pixelclock is the MIPI clock supplied separately
	 * through the device tree (simaai,bt-pixelclock) and is used as the
	 * reference for converting the IPI blanking intervals to ns.
	 */
	csidev->link_freq = link_freq;
	return 0;
}

static const struct v4l2_async_notifier_operations dwc_csi_notify_ops = {
	.bound = dwc_csi_notify_bound,
};

static int dwc_csi_async_register(struct dwc_csi_device *csidev)
{
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct v4l2_async_connection *asd;
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	v4l2_async_subdev_nf_init(&csidev->notifier, &csidev->sd);

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(csidev->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return -ENOTCONN;

	csidev->ep_fwnode = fwnode_graph_get_remote_endpoint(ep);
	if (!csidev->ep_fwnode) {
		dev_err(csidev->dev, "Failed to get remote endpoint on port %u\n", 0);
		return -ENOTCONN;
	}

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		goto err_parse;

	for (i = 0; i < vep.bus.mipi_csi2.num_data_lanes; ++i) {
		if (vep.bus.mipi_csi2.data_lanes[i] != i + 1) {
			dev_err(csidev->dev,
				"data lanes reordering is not supported");
			ret = -EINVAL;
			goto err_parse;
		}
	}

	csidev->bus = vep.bus.mipi_csi2;

	dev_dbg(csidev->dev, "data lanes: %d\n", csidev->bus.num_data_lanes);
	dev_dbg(csidev->dev, "flags: 0x%08x\n", csidev->bus.flags);

	asd = v4l2_async_nf_add_fwnode_remote(&csidev->notifier, ep,
					      struct v4l2_async_connection);
	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto err_parse;
	}

	fwnode_handle_put(ep);

	csidev->notifier.ops = &dwc_csi_notify_ops;

	ret = v4l2_async_nf_register(&csidev->notifier);
	if (ret)
		goto err_notifier_clean;

	ret = v4l2_async_register_subdev(&csidev->sd);
	if (ret)
		goto err_unreg_notifier;

	return ret;

err_unreg_notifier:
	v4l2_async_nf_unregister(&csidev->notifier);
err_notifier_clean:
	v4l2_async_nf_cleanup(&csidev->notifier);
err_parse:
	fwnode_handle_put(ep);
	return ret;
}

/*
 * Pattern Generator Controller operations
 */

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
	"Horizontal Color Bars",
};

static inline struct dwc_csi_device *ctrl_to_csidev(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dwc_csi_device, ctrl_handler);
}

static int dwc_csi_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dwc_csi_device *csidev  = ctrl_to_csidev(ctrl);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		/* Pattern index start from 0 */
		csidev->pg_pattern = ctrl->val - 1;
		csidev->pg_enable = (ctrl->val) ? true : false;
		break;
	case V4L2_CID_DWC_CSI2_IPI_CTRL_MODE:
		csidev->ipi_mode_controller = ctrl->val;
		break;
	case V4L2_CID_DWC_CSI2_IPI_EMB_DATA_EN:
		csidev->ipi_emb_data_en = ctrl->val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops dwc_csi_ctrl_ops = {
	.s_ctrl = dwc_csi_s_ctrl,
};

static const struct v4l2_ctrl_config dwc_csi_ctrl_ipi_mode = {
	.ops	= &dwc_csi_ctrl_ops,
	.id	= V4L2_CID_DWC_CSI2_IPI_CTRL_MODE,
	.name	= "IPI Controller Mode",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,  /* camera timing mode by default */
};

static const struct v4l2_ctrl_config dwc_csi_ctrl_ipi_emb_data = {
	.ops	= &dwc_csi_ctrl_ops,
	.id	= V4L2_CID_DWC_CSI2_IPI_EMB_DATA_EN,
	.name	= "IPI Embedded Data Enable",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,
};

static int dwc_csi_controls_init(struct dwc_csi_device *csidev)
{
	struct v4l2_ctrl_handler *handler = &csidev->ctrl_handler;
	struct device_node *node = csidev->dev->of_node;
	struct v4l2_ctrl_config ipi_mode_cfg = dwc_csi_ctrl_ipi_mode;
	struct v4l2_ctrl_config ipi_emb_data_cfg = dwc_csi_ctrl_ipi_emb_data;
	u32 val;
	int ret;

	v4l2_ctrl_handler_init(handler, 3);

	/* Use driver mutex lock for the ctrl lock */
	handler->lock = &csidev->lock;

	v4l2_ctrl_new_std_menu_items(handler, &dwc_csi_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(test_pattern_menu) - 1,
				     0, 0, test_pattern_menu);

	/*
	 * Use the device tree values as the defaults for the IPI controls
	 * when present. Otherwise keep controller mode disabled and embedded
	 * data off (the defaults from the static control configs).
	 */
	if (!of_property_read_u32(node, "simaai,ipi-controller-mode", &val))
		ipi_mode_cfg.def = !!val;
	if (!of_property_read_u32(node, "simaai,ipi-emb-data-enable", &val))
		ipi_emb_data_cfg.def = !!val;

	v4l2_ctrl_new_custom(handler, &ipi_mode_cfg, NULL);
	v4l2_ctrl_new_custom(handler, &ipi_emb_data_cfg, NULL);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	csidev->sd.ctrl_handler = handler;
	return 0;
}

static void dwc_csi_controls_cleanup(void *data)
{
	struct dwc_csi_device *csidev = data;

	v4l2_ctrl_handler_free(&csidev->ctrl_handler);
}

/*
 * Suspend/resume
 */

static int dwc_csi_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

static int dwc_csi_system_resume(struct device *dev)
{
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0) {
		dev_err(dev, "force resume %s failed!\n", dev_name(dev));
		return ret;
	}

	return 0;
}

static int dwc_csi_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	clk_bulk_disable_unprepare(csidev->num_clks, csidev->clks);

	return 0;
}

static int dwc_csi_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	return clk_bulk_prepare_enable(csidev->num_clks, csidev->clks);
}

static const struct dev_pm_ops dwc_csi_device_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc_csi_system_suspend, dwc_csi_system_resume)
	SET_RUNTIME_PM_OPS(dwc_csi_runtime_suspend, dwc_csi_runtime_resume, NULL)
};

/*
 * IPI-overflow recovery delays (SOCSW-5392). The HW team's manual sequence used
 * sleep 20s / 100ms, but those were typing-speed artifacts — the true minimal
 * delays are unknown. Exposed as module params so they can be tuned down on the
 * bench without recompiling. ipi_ovf_reset_us is the delay while CSI+glue are
 * held in reset; ipi_ovf_post_us is the settle after each re-enable step.
 */
static bool ipi_overflow_recovery_enable = true;

module_param(ipi_overflow_recovery_enable, bool, 0644);
MODULE_PARM_DESC(ipi_overflow_recovery_enable,
                 "Enable IPI overflow recovery");

static unsigned int ipi_ovf_reset_us = 1000;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(ipi_ovf_reset_us, uint, 0644);
MODULE_PARM_DESC(ipi_ovf_reset_us, "IPI-overflow recovery: reset-hold delay (us)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */
static unsigned int ipi_ovf_post_us = 100;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(ipi_ovf_post_us, uint, 0644);
MODULE_PARM_DESC(ipi_ovf_post_us, "IPI-overflow recovery: per-step settle delay (us)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

/* Test knob: when false, resets only the IPI/glue and SKIPS the DMA resume,
 * to check on-bench whether an IPI-only reset recovers the chain or the DMA
 * descriptor must be re-validated (SOCSW-5392). Default true (full recovery). */
static bool ipi_ovf_dma_resume = true;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(ipi_ovf_dma_resume, bool, 0644);
MODULE_PARM_DESC(ipi_ovf_dma_resume, "IPI-overflow recovery: also resume the DMA (0 = IPI-only, test)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

/* Minimum gap between recovery attempts: re-firing within a frame period
 * resets the IPI faster than it can re-sync and capture never converges. */
static unsigned int ipi_ovf_holdoff_ms = 100;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(ipi_ovf_holdoff_ms, uint, 0644);
MODULE_PARM_DESC(ipi_ovf_holdoff_ms, "IPI-overflow recovery: minimum gap between attempts (ms)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

/* Clean-streaming window after which the recovery counter decays to 0 (a fresh
 * overflow after this long is a new incident, not part of a reset-storm). */
static unsigned int ipi_ovf_decay_ms = 10000;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(ipi_ovf_decay_ms, uint, 0644);
MODULE_PARM_DESC(ipi_ovf_decay_ms, "IPI-overflow recovery: reset the attempt count after this many ms of clean streaming");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

/* Bound CONSECUTIVE recovery attempts (within ipi_ovf_decay_ms of each other) so
 * a persistent fault can't reset-storm; the count decays after clean streaming. */
#define DWC_OVF_RECOVER_MAX	32

/*
 * Recover a controller-mode CSI IPI pixel overflow (SOCSW-5392): reset the glue
 * + CSI IPI for instance 0, re-sync the VCID (Synopsys: program a wrong id then
 * the correct one), then ask the downstream DMA to resume (it owns the channel,
 * so it is invoked through the vdma-registered callback). Runs in process
 * context off the IRQ — must NOT be called from hardirq.
 */
static void dwc_csi_ovf_recover_work(struct work_struct *w)
{
	struct dwc_csi_device *csidev =
		container_of(w, struct dwc_csi_device, ovf_recover_work);
	void (*quiesce)(void *data);
	void (*resume)(void *data);
	void *resume_data;
	u32 chan_ctrl, rstn;
	bool do_dma;

	mutex_lock(&csidev->lock);
	if (!csidev->streaming) {
		mutex_unlock(&csidev->lock);
		return;
	}

	/* Holdoff: skipping is safe, a still-dead pipe re-raises the fatal
	 * IRQ (and the vdma watchdog re-triggers). */
	if (csidev->ovf_recover_count &&
	    ktime_ms_delta(ktime_get(), csidev->ovf_last_recover_time) <
		    ipi_ovf_holdoff_ms) {
		mutex_unlock(&csidev->lock);
		return;
	}

	quiesce = csidev->ovf_quiesce;
	resume = csidev->ovf_resume;
	resume_data = csidev->ovf_resume_data;
	do_dma = quiesce && resume && ipi_ovf_dma_resume;

	/* 0: park the DMA before yanking its pixel source (resetting under a
	 * running block raises an async src bus error; resubmission races). */
	if (do_dma)
		quiesce(resume_data);

	/* 1: glue reset — disable glue channel 0 */
	chan_ctrl = dwc_csi_read(csidev, DWC_CSI2GLUE_CHAN_CTRL1);
	dwc_csi_write(csidev, DWC_CSI2GLUE_CHAN_CTRL1,
		      chan_ctrl & ~DWC_CSI2GLUE_CHAN_CTRL_EN);
	/* 2: assert IPI soft reset (active-low) for instance 0 */
	rstn = dwc_csi_read(csidev, CSI2RX_IPI_SOFTRSTN);
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, rstn & ~BIT(0));
	/* 3: VCID -> wrong id (re-sync trick) */
	dwc_csi_write(csidev, CSI2RX_IPI1_VCID, CSI2RX_IPI_VCID_VC(1));
	/* 4: hold in reset */
	usleep_range(ipi_ovf_reset_us, ipi_ovf_reset_us + 100);
	/* 5: re-enable glue channel 0 */
	dwc_csi_write(csidev, DWC_CSI2GLUE_CHAN_CTRL1, chan_ctrl);
	usleep_range(ipi_ovf_post_us, ipi_ovf_post_us + 50);
	/* 7: deassert IPI soft reset */
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, rstn | BIT(0));
	usleep_range(ipi_ovf_post_us, ipi_ovf_post_us + 50);
	/* 9: VCID -> correct id */
	dwc_csi_write(csidev, CSI2RX_IPI1_VCID, CSI2RX_IPI_VCID_VC(0));

	csidev->ovf_recover_count++;
	csidev->ovf_last_recover_time = ktime_get();
	mutex_unlock(&csidev->lock);

	/* 10: restart the parked DMA — must follow every quiesce above. */
	if (do_dma)
		resume(resume_data);

	dev_info_ratelimited(csidev->dev,
			     "IPI overflow recovery #%u (reset_us=%u post_us=%u)\n",
			     csidev->ovf_recover_count, ipi_ovf_reset_us,
			     ipi_ovf_post_us);
}

/*
 * Register the downstream DMA resume callback. Called by the vdma capture driver
 * (which owns the dma_chan) at stream-on; pass NULL cb to unregister.
 */
int dwc_csi_register_overflow_recovery(struct v4l2_subdev *sd,
				       void (*quiesce)(void *data),
				       void (*resume)(void *data), void *data)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	mutex_lock(&csidev->lock);
	csidev->ovf_quiesce = quiesce;
	csidev->ovf_resume = resume;
	csidev->ovf_resume_data = data;
	mutex_unlock(&csidev->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(dwc_csi_register_overflow_recovery);

/*
 * Trigger the same IPI-overflow recovery as the FATAL_ERR_IPI IRQ, but from an
 * external caller (the vdma frame watchdog) for SILENT stalls that never raise a
 * fatal interrupt. Atomic-safe (queue_work only); coalesces with a pending IRQ
 * recovery and honours the per-stream cap.
 */
void dwc_csi_trigger_overflow_recovery(struct v4l2_subdev *sd)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	/*
	 * No DWC_OVF_RECOVER_MAX cap here (unlike the IRQ path): the caller (the
	 * vdma frame watchdog) is rate-limited by its own timer and bounded by
	 * frame_watchdog_max_retry, which supports an explicit infinite-retry mode.
	 */
	if (csidev->streaming)
		queue_work(csidev->wq, &csidev->ovf_recover_work);
}
EXPORT_SYMBOL_GPL(dwc_csi_trigger_overflow_recovery);

/* CRC error log rate limit - tunable at runtime without recompiling */
static unsigned int crc_ratelimit_interval_min = 1;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(crc_ratelimit_interval_min, uint, 0644);
MODULE_PARM_DESC(crc_ratelimit_interval_min, "CRC rate limit interval (minutes)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */
static unsigned int crc_ratelimit_burst = 10;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(crc_ratelimit_burst, uint, 0644);
MODULE_PARM_DESC(crc_ratelimit_burst, "CRC error log rate limit: max prints per window");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

static irqreturn_t dwc_csi_irq_handler(int irq, void *priv)
{
	struct dwc_csi_device *csidev = priv;
	u32 status;
	int i;

	/* Hardware auto clean after read */
	status = dwc_csi_read(csidev, CSI2RX_INT_ST_MAIN);

	if (status & DWC_EVENT_MASK) {
		for (i = 0; i < DWC_NUM_EVENTS; ++i) {
			struct dwc_csi_event *event = &csidev->events[i];

			if (status & event->mask) {
				event->counter++;

				if (event->mask == CSI2RX_INT_ST_MAIN_FATAL_ERR_PLD_CRC ||
				    event->mask == CSI2RX_INT_ST_MAIN_FATAL_ERR_CRC_FRAME) {

					/*
					 * Re-read the live module param values
					 * every time, so writes to
					 * /sys/module/.../parameters/crc_ratelimit_*
					 * take effect immediately.
					 */
					csidev->crc_err_rs.interval =
						secs_to_jiffies(crc_ratelimit_interval_min * SECS_PER_MIN);
					csidev->crc_err_rs.burst = crc_ratelimit_burst;

					if (__ratelimit(&csidev->crc_err_rs))
						dev_warn(
							csidev->dev,
							"%s (INT_ST_MAIN=0x%08x) [total: %u]\n",
							event->name, status, event->counter);
				}
			}
		}
	}

    if (ipi_overflow_recovery_enable) {
    /* recovery code */

	/*
	 * IPI pixel overflow (DMA out of descriptors, SOCSW-5392): kick the
	 * deferred recovery. queue_work() coalesces if one is already pending;
	 * the per-stream cap stops a persistent fault reset-storming. The
	 * recovery sequence is mode-agnostic, so gate on a registered resume
	 * callback rather than controller mode — the overflow also hits the
	 * standard v4l2-m2m path (12h soak, SOCSW-isp-sensor-v4l2-collapse).
	 */
	if ((status & CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI) &&
	    csidev->ovf_resume && csidev->streaming) {
		/* decay: clean streaming since the last recovery => fresh incident */
		if (csidev->ovf_recover_count &&
		    ktime_ms_delta(ktime_get(), csidev->ovf_last_recover_time) >
			    ipi_ovf_decay_ms)
			csidev->ovf_recover_count = 0;
		if (csidev->ovf_recover_count < DWC_OVF_RECOVER_MAX)
			queue_work(csidev->wq, &csidev->ovf_recover_work);
	}
    }    
	return IRQ_HANDLED;
}

static inline void dwc_csi_param_init(struct dwc_csi_device *csidev)
{
	csidev->csi_fmt = &dwc_csi_formats[0];
	csidev->ipi_enable_delay_ms = DWC_CSI2RX_IPI_ENABLE_DELAY_MS;
}

static int dwc_csi_subdev_init(struct dwc_csi_device *csidev)
{
	struct v4l2_subdev *sd = &csidev->sd;
	int ret;

	v4l2_subdev_init(sd, &dwc_csi_subdev_ops);
	sd->owner = THIS_MODULE;
	snprintf(sd->name, sizeof(sd->name), "csidev-%s", dev_name(csidev->dev));
	sd->internal_ops = &dwc_csi_internal_ops;

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_STREAMS;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &dwc_csi_entity_ops;

	sd->dev = csidev->dev;

	csidev->pads[DWC_CSI2RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csidev->pads[DWC_CSI2RX_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csidev->pads[DWC_CSI2RX_PAD_SINK + 2].flags = MEDIA_PAD_FL_SINK;
	csidev->pads[DWC_CSI2RX_PAD_SOURCE + 2].flags = MEDIA_PAD_FL_SOURCE;
	csidev->pads[DWC_CSI2RX_PAD_SINK + 4].flags = MEDIA_PAD_FL_SINK;
	csidev->pads[DWC_CSI2RX_PAD_SOURCE + 4].flags = MEDIA_PAD_FL_SOURCE;
	csidev->pads[DWC_CSI2RX_PAD_SINK + 6].flags = MEDIA_PAD_FL_SINK;
	csidev->pads[DWC_CSI2RX_PAD_SOURCE + 6].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&csidev->sd.entity, DWC_CSI2RX_PADS_NUM,
				     csidev->pads);
	if (ret) {
		dev_err(csidev->dev, "Failed to init pads\n");
		return ret;
	}

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		media_entity_cleanup(&sd->entity);

	return ret;
}

static void dwc_csi_dv_timing_init(struct dwc_csi_device *csidev)
{
	struct device_node *node = csidev->dev->of_node;
	struct v4l2_dv_timings *dv_tmg = &csidev->dv_tmg;
	struct v4l2_bt_timings *bt = &dv_tmg->bt;
	u64 pixelclock;

	memset(dv_tmg, 0x0, sizeof(*dv_tmg));

	dv_tmg->type = V4L2_DV_BT_656_1120;
	bt->width = DWC_CSI2RX_DEF_PIX_WIDTH;
	bt->height = DWC_CSI2RX_DEF_PIX_HEIGHT;
	bt->hfrontporch = DWC_CSI2RX_DEF_HSD_TIME;
	bt->hsync = DWC_CSI2RX_DEF_HSA_TIME;
	bt->hbackporch = DWC_CSI2RX_DEF_HBP_TIME;
	bt->vfrontporch = DWC_CSI2RX_DEF_VFP_LINES;
	bt->vsync = DWC_CSI2RX_DEF_VSA_LINES;
	bt->vbackporch = DWC_CSI2RX_DEF_VBP_LINES;

	/*
	 * Allow the board device tree to override the default IPI blanking
	 * timings. Each property is optional and falls back to the default
	 * above when it is absent. The active resolution (width/height) is not
	 * taken from here; it comes from the format set through .set_fmt.
	 */
	of_property_read_u32(node, "simaai,bt-hfrontporch", &bt->hfrontporch);
	of_property_read_u32(node, "simaai,bt-hsync", &bt->hsync);
	of_property_read_u32(node, "simaai,bt-hbackporch", &bt->hbackporch);
	of_property_read_u32(node, "simaai,bt-vfrontporch", &bt->vfrontporch);
	of_property_read_u32(node, "simaai,bt-vsync", &bt->vsync);
	of_property_read_u32(node, "simaai,bt-vbackporch", &bt->vbackporch);
	if (!of_property_read_u64(node, "simaai,bt-pixelclock", &pixelclock))
		bt->pixelclock = pixelclock;

	dev_info(csidev->dev,
		 "IPI blanking timings: pixelclock=%llu hsync=%u hbackporch=%u hfrontporch=%u vsync=%u vbackporch=%u vfrontporch=%u\n",
		 bt->pixelclock, bt->hsync, bt->hbackporch, bt->hfrontporch,
		 bt->vsync, bt->vbackporch, bt->vfrontporch);
};

static void dwc_csi_subdev_cleanup(void *data)
{
	struct dwc_csi_device *csidev = data;

	v4l2_subdev_cleanup(&csidev->sd);
	media_entity_cleanup(&csidev->sd.entity);
}

static int dwc_csi_device_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc_csi_device *csidev;
	int irq;
	int ret;

	csidev = devm_kzalloc(dev, sizeof(*csidev), GFP_KERNEL);
	if (!csidev)
		return -ENOMEM;

	mutex_init(&csidev->lock);

	csidev->dev = dev;
	memcpy(csidev->events, dwc_events, sizeof(dwc_events));

	/* Cap CRC error warnings to 10 per every 1min to avoid log flooding */
	ratelimit_state_init(&csidev->crc_err_rs,
			      secs_to_jiffies(crc_ratelimit_interval_min * SECS_PER_MIN),
			      crc_ratelimit_burst);

	csidev->regs = devm_platform_ioremap_resource_byname(pdev, "csi");
	if (IS_ERR(csidev->regs)) {
		dev_err(dev, "Failed to get DWC csi2 register map\n");
		return PTR_ERR(csidev->regs);
	}

	csidev->glue = devm_platform_ioremap_resource_byname(pdev, "glue");
	if (IS_ERR(csidev->glue)) {
		dev_err(dev, "Failed to get DWC glue register map\n");
		return PTR_ERR(csidev->glue);
	}

	csidev->phy = devm_phy_get(dev, "rx");
	if (IS_ERR(csidev->phy))
		return dev_err_probe(dev, PTR_ERR(csidev->phy),
				     "Failed to get DPHY Rx\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "Failed to get IRQ\n");

	ret = devm_request_irq(dev, irq, dwc_csi_irq_handler, 0,
			       dev_name(dev), csidev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");
	csidev->irq = irq;

	csidev->num_clks = devm_clk_bulk_get_all(dev, &csidev->clks);

	dwc_csi_param_init(csidev);

	ret = dwc_csi_subdev_init(csidev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to initialize subdev\n");

	ret = devm_add_action_or_reset(dev, dwc_csi_subdev_cleanup, csidev);
	if (ret)
		return ret;

	ret = dwc_csi_controls_init(csidev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize controls\n");

	ret = devm_add_action_or_reset(dev, dwc_csi_controls_cleanup, csidev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, &csidev->sd);

	ret = dwc_csi_async_register(csidev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Async register failed\n");

	pm_runtime_enable(dev);

	/* Private high-priority WQ; the stop-state poll must run promptly. */
	csidev->wq = alloc_ordered_workqueue("%s-csi", WQ_HIGHPRI, dev_name(dev));
	if (!csidev->wq) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate WQ\n");
		goto err_pm_disable;
	}

	dwc_csi_dv_timing_init(csidev);
	INIT_WORK(&csidev->work, stopstate_poll);
	INIT_DELAYED_WORK(&csidev->ipi_work, dwc_csi_delayed_ipi_enable);
	INIT_WORK(&csidev->ovf_recover_work, dwc_csi_ovf_recover_work);
	csidev->poll_state = DWC_POLL_INACTIVE;

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	v4l2_async_nf_unregister(&csidev->notifier);
	v4l2_async_nf_cleanup(&csidev->notifier);
	v4l2_async_unregister_subdev(&csidev->sd);
	return ret;
}

static void dwc_csi_device_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);

	v4l2_async_nf_unregister(&csidev->notifier);
	v4l2_async_nf_cleanup(&csidev->notifier);
	v4l2_async_unregister_subdev(&csidev->sd);

	/*
	 * Free the IRQ and stop the watchdog trigger path before tearing the WQ
	 * down: otherwise dwc_csi_irq_handler() or the exported
	 * dwc_csi_trigger_overflow_recovery() could queue_work() onto a
	 * destroyed WQ if the device is unbound while streaming. Done before
	 * mutex_destroy() so a flushed ovf_recover_work can still take the lock.
	 */
	devm_free_irq(csidev->dev, csidev->irq, csidev);
	csidev->streaming = false;
	cancel_delayed_work_sync(&csidev->ipi_work);
	cancel_work_sync(&csidev->ovf_recover_work);
	destroy_workqueue(csidev->wq);

	pm_runtime_disable(&pdev->dev);

	fwnode_handle_put(csidev->sd.fwnode);
	mutex_destroy(&csidev->lock);

	pm_runtime_set_suspended(&pdev->dev);
}

static const struct of_device_id dwc_csi_device_of_match[] = {
	{ .compatible = "snps,dw-mipi-csi2-v150" },
	{ },
};
MODULE_DEVICE_TABLE(of, dwc_csi_device_of_match);

static struct platform_driver dwc_csi_device_driver = {
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "dwc-mipi-csi2",
		.of_match_table = dwc_csi_device_of_match,
		.pm             = &dwc_csi_device_pm_ops,
	},
	.probe  = dwc_csi_device_probe,
	.remove = dwc_csi_device_remove,
};

module_platform_driver(dwc_csi_device_driver);

MODULE_DESCRIPTION("DesignWare Core MIPI CSI2 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: dwc-mipi-csi2");
MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_AUTHOR("SiMa Technologies, Inc.");
