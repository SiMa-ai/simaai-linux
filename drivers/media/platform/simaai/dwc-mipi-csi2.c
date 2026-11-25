// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 * Copyright 2026 SiMa Technologies, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/bitops.h>
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
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-dv-timings.h>

#include <linux/media/simaai/metoak_rgbd.h>

#include "simaai-v4l2-vdma.h"

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
/* IPI1..IPI4 fatal bits, i.e. one per virtual channel. Consecutive from 18. */
#define   CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI_ANY		GENMASK(21, 18)

/* Payload CRC Fatal - status/mask/force triplet*/
#define CSI2RX_INT_ST_PLD_CRC_FATAL 0x2b0
#define CSI2RX_INT_MSK_PLD_CRC_FATAL 0x2b4
#define CSI2RX_INT_FORCE_PLD_CRC_FATAL 0x2b8
#define CSI2RX_INT_ST_PLD_CRC_FATAL_ERR BIT(0)

/* Fatal Interruption caused by Frame CRC - status/mask/force triplet */
#define CSI2RX_INT_ST_CRC_FRAME_FATAL 0x2a0
#define CSI2RX_INT_MSK_CRC_FRAME_FATAL 0x2a4
#define CSI2RX_INT_FORCE_CRC_FRAME_FATAL 0x2a8

/*
 * The four groups below feed INT_ST_MAIN bits 2, 3, 6 and 7. They were never
 * defined, so the driver could neither unmask nor read them: the errors landed
 * in the event counters but nothing ever printed them, and the per-VC detail
 * (one bit per virtual channel, like the CRC pair above) was lost. A mistuned
 * D-PHY hsfreqrange shows up here first, as ECC hits on FS/FE short packets
 * followed by frame boundary and frame sequence errors.
 */

/* Fatal Interruption caused by Frame Boundaries - status/mask/force triplet */
#define CSI2RX_INT_ST_BNDRY_FRAME_FATAL 0x280
#define CSI2RX_INT_MSK_BNDRY_FRAME_FATAL 0x284
#define CSI2RX_INT_FORCE_BNDRY_FRAME_FATAL 0x288

/* Fatal Interruption caused by Frame Sequence - status/mask/force triplet */
#define CSI2RX_INT_ST_SEQ_FRAME_FATAL 0x290
#define CSI2RX_INT_MSK_SEQ_FRAME_FATAL 0x294
#define CSI2RX_INT_FORCE_SEQ_FRAME_FATAL 0x298

/* Interruption caused by an unrecognised Data ID - status/mask/force triplet */
#define CSI2RX_INT_ST_DATA_ID 0x2c0
#define CSI2RX_INT_MSK_DATA_ID 0x2c4
#define CSI2RX_INT_FORCE_DATA_ID 0x2c8

/* Interruption caused by a corrected header ECC - status/mask/force triplet */
#define CSI2RX_INT_ST_ECC_CORRECTED 0x2d0
#define CSI2RX_INT_MSK_ECC_CORRECTED 0x2d4
#define CSI2RX_INT_FORCE_ECC_CORRECTED 0x2d8

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
/* Reserved in CSI-2, so no source ever sends it: the re-sync park. */
#define   DWC_CSI2RX_DT_RESERVED			0x3f
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
#define   DWC_CSI2RX_HLINE_TIME_MAX			GENMASK(14, 0)

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
/* Hardware field is 10 bits; a larger porch wraps silently (verified on HW). */
#define   CSI2RX_IPI_VFP_LINES_MAX			GENMASK(9, 0)

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

/* Interruption Caused by Line Construction */
#define CSI2RX_INT_ST_LINE				0x130

/* Mask for Interruption Caused by Line Construction */
#define CSI2RX_INT_MSK_LINE				0x134

/* Force for Interruption Caused by Line Construction */
#define CSI2RX_INT_FORCE_LINE				0x138

/* Fatal Interruption Caused by IPI Interface */
#define CSI2RX_INT_ST_IPI_FATAL				0x140
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_IFFIFO_UNDERFLOW	BIT(0)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_IFFIFO_OVERFLOW	BIT(1)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_FRAME_SYNC	BIT(2)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_FIFO_NOT_EMPTY	BIT(3)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_HLINE_TIME	BIT(4)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_FIFO_OVERFLOW	BIT(5)
#define   CSI2RX_INT_ST_IPI_FATAL_ERR_PD_FIFO_OVERFLOW	BIT(6)

/*
 * Per-IPI fatal blocks: 0x10 stride from IPI1 at 0x140, all sharing IPI1's
 * cause-bit layout. n is 0-based (0 = IPI1).
 */
#define CSI2RX_INT_ST_IPI_FATAL_N(n)			(0x140 + (n) * 0x10)
#define CSI2RX_INT_MSK_IPI_FATAL_N(n)			(0x144 + (n) * 0x10)

#define CSI2RX_INT_ST_IPI2_FATAL			0x150
#define CSI2RX_INT_ST_IPI3_FATAL			0x160
#define CSI2RX_INT_ST_IPI4_FATAL			0x170

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
#define   DWC_CSI2GLUE_VIS_CTRL_SRC_BURST_LEN(x)	FIELD_PREP(GENMASK(6, 0), (x))
#define   DWC_CSI2GLUE_VIS_CTRL_BUF_SRAM_PIPE_EN	BIT(31)

/*
 * Glue datapath policy. The glue forwards the IPI output as a tightly packed
 * byte stream - the raw MIPI payload, unpacked downstream - so the 8-bit data
 * unit is format-independent. Burst length and pixel-buffer full threshold
 * deviate from the hardware reset values (16 and 32) and are bring-up
 * validated across every shipping sensor; END_PATTERN and FRAME_FMT restate
 * the hardware reset values.
 */
#define DWC_CSI2GLUE_SRC_BURST_LEN_DEF			0x8
#define DWC_CSI2GLUE_PIX_BUF_FULL_TH_DEF		0x38
#define DWC_CSI2GLUE_STREAM_DBITS			8
#define DWC_CSI2GLUE_END_PATTERN_DEF			0xBEEF
#define DWC_CSI2GLUE_FRAME_FMT_DEF			0x4

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
#define   DWC_CSI2GLUE_STAT_CTRL_OFLOW_GET(x)		FIELD_GET(GENMASK(23, 16), (x))

/* Glue Line Bytes */
#define DWC_CSI2GLUE_LNBYTES1				0xc0
#define DWC_CSI2GLUE_LNBYTES2				0xe4
#define DWC_CSI2GLUE_LNBYTES3				0x108
#define DWC_CSI2GLUE_LNBYTES4				0x12c
#define   DWC_CSI2GLUE_LNBYTES_LINE_BYTES(x)		FIELD_PREP(GENMASK(15, 0), (x))

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
#define   DWC_CSI2GLUE_FILTINFO_CHAN(x)			FIELD_PREP(GENMASK(3, 0), (x))
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
/* One sink/source pad pair per virtual channel. */
#define DWC_CSI2RX_NUM_VC				(DWC_CSI2RX_PADS_NUM / 2)

#define DWC_CSI2RX_DEF_MBUS_CODE			MEDIA_BUS_FMT_UYVY8_1X16
#define DWC_CSI2RX_DEF_PIX_WIDTH			1920U
#define DWC_CSI2RX_DEF_PIX_HEIGHT			1080U
#define DWC_CSI2RX_MAX_PIX_WIDTH			0xffff
#define DWC_CSI2RX_MAX_PIX_HEIGHT			0xffff
#define DWC_CSI2RX_DEF_PIX_WIDTH			1920U
#define DWC_CSI2RX_DEF_HSA_TIME				0x10
#define DWC_CSI2RX_DEF_HBP_TIME				0xc8
#define DWC_CSI2RX_DEF_HSD_TIME				0x10
/* IPI pixel clock, the unit of every IPI horizontal timing register */
#define DWC_CSI2RX_DEF_PIXELCLOCK			500000000ULL
/* Fixed HSD time programmed into the IPI HSD_TIME register */
#define DWC_CSI2RX_HSD_TIME				10
#define DWC_CSI2RX_IPI_LINE_OVERFLOW_MARGIN_CY		8

/*
 * Maps a CSI-2 host source pad number to its virtual channel. Fixed by
 * dwc_csi_subdev_init_state()'s own pad layout (VC N -> sink pad 2N,
 * source pad 2N+1)
 *
 * Returns DWC_CSI2RX_NUM_VC (an otherwise out-of-range VC value) if pad
 * isn't a genuine source pad, so callers can reuse their own existing
 * "vc >= DWC_CSI2RX_NUM_VC" bounds check to catch both cases in one
 * place, rather than needing a second, separate error path here.
 */
static inline unsigned int dwc_csi_pad_to_vc(unsigned int pad)
{
	if (pad < DWC_CSI2RX_PAD_SOURCE || ((pad - DWC_CSI2RX_PAD_SOURCE) % 2))
		return DWC_CSI2RX_NUM_VC;
	return (pad - DWC_CSI2RX_PAD_SOURCE) / 2;
}

/* The sink counterpart, same convention. */
static inline unsigned int dwc_csi_sink_pad_to_vc(unsigned int pad)
{
	if ((pad - DWC_CSI2RX_PAD_SINK) % 2)
		return DWC_CSI2RX_NUM_VC;
	return (pad - DWC_CSI2RX_PAD_SINK) / 2;
}
#define DWC_CSI2RX_DEF_VSA_LINES			0x2
#define DWC_CSI2RX_DEF_VBP_LINES			0x2
#define DWC_CSI2RX_DEF_VFP_LINES			0xf

/* Default IPI enable delay (ms) if not overridden by the deserializer DT node */
#define DWC_CSI2RX_IPI_ENABLE_DELAY_MS			0
#define DWC_CSI2RX_IPI_ENABLE_DELAY_MAX_MS		5000

struct dwc_csi_event {
	u32 mask;
	const char * const name;
	/*
	 * INT_ST_* sub-register behind this INT_ST_MAIN bit, or 0 when none is
	 * defined. Reading it yields the detail and clears the source.
	 *
	 * The layout depends on the group: BNDRY/SEQ/CRC/PLD_CRC/DATA_ID/ECC are
	 * per-virtual-channel bitmaps (bit n = VC n), so with two cameras on
	 * separate VCs they say which one is failing; DPHY/DPHY_FATAL/PKT_FATAL/
	 * IPI_FATAL are per-error-cause bitmaps instead (e.g. IPI_FATAL 0x22 is
	 * ERR_IFFIFO_OVERFLOW | ERR_FIFO_OVERFLOW). The log prints the register
	 * offset alongside the value so the two cannot be confused.
	 */
	u32 st_reg;
	unsigned int counter;
};

static const struct dwc_csi_event dwc_events[] = {
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI, "IPI Interface Fatal Error",
	  CSI2RX_INT_ST_IPI_FATAL },
	{ CSI2RX_INT_ST_MAIN_ERR_PHY, "PHY Error", CSI2RX_INT_ST_DPHY },
	{ CSI2RX_INT_ST_MAIN_ERR_ECC, "Header Single Bit Error",
	  CSI2RX_INT_ST_ECC_CORRECTED },
	{ CSI2RX_INT_ST_MAIN_ERR_DID, "Data ID Error", CSI2RX_INT_ST_DATA_ID },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_PLD_CRC, "Payload CRC Fatal Error",
	  CSI2RX_INT_ST_PLD_CRC_FATAL },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_CRC_FRAME, "Frame CRC Fatal Error",
	  CSI2RX_INT_ST_CRC_FRAME_FATAL },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_SEQ_FRAME, "Frame Sequence Fatal Error",
	  CSI2RX_INT_ST_SEQ_FRAME_FATAL },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_BNDRY_FRAMEL, "Frame Boundaries Fatal Error",
	  CSI2RX_INT_ST_BNDRY_FRAME_FATAL },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_PKT, "Packet Construction Fatal Error",
	  CSI2RX_INT_ST_PKT_FATAL },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_PHY, "PHY Fatal Error",
	  CSI2RX_INT_ST_DPHY_FATAL },
	{ CSI2RX_INT_ST_MAIN_ERR_LINE, "Line Error", CSI2RX_INT_ST_LINE },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI2, "IPI2 Interface Fatal Error",
	  CSI2RX_INT_ST_IPI2_FATAL },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI3, "IPI3 Interface Fatal Error",
	  CSI2RX_INT_ST_IPI3_FATAL },
	{ CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI4, "IPI4 Interface Fatal Error",
	  CSI2RX_INT_ST_IPI4_FATAL },
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
/* re-init under a running source: a few frame blankings, not a full start */
#define DWC_REINIT_STOPSTATE_US 200000
/* a stream start or stop waits this long for a re-init or recovery to end */
#define DWC_RECOVER_WAIT_MS 2000

struct dwc_csi_pix_format {
	u32 code;
	u32 output;
	u32 data_type;
	/* Bits per image pixel, summed over its samples. */
	u8 width;
	/* bus samples per image pixel; 0 means 1 */
	u8 samples_per_pixel;
	/*
	 * The source raises no CSI-2 frame-start short packet: with event sync
	 * on, the IPI waits forever and the frame watchdog resets it.
	 */
	bool no_ipi_sync_event;
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
	/* Per-VC upstream link: which subdev/pad feeds sink pad 2*vc. */
	struct v4l2_subdev *source_sd[DWC_CSI2RX_NUM_VC];
	u16 remote_pad[DWC_CSI2RX_NUM_VC];
	u8 source_vc[DWC_CSI2RX_NUM_VC];	/* VC the source says it sends on */
	struct v4l2_ctrl_handler ctrl_handler;
	struct media_pad pads[DWC_CSI2RX_PADS_NUM];

	struct v4l2_mbus_config_mipi_csi2 bus;
	u32 cfgclkfreqrange;
	u32 hsfreqrange;
	/* MIPI D-PHY link frequency (Hz), used to program the PHY HS clock */
	s64 link_freq;
	u64 enabled_streams;

	/* Source-node delay, honored in either IPI timing mode. */
	u32 ipi_enable_delay_ms;
	/*
	 * Tracks whether any VC has already declared ipi_enable_delay_ms, so
	 * "first bound source wins" works for
	 * whichever VC genuinely binds first at runtime - not just VC0.
	 * Async registration order isn't guaranteed once all 4 ports are
	 * being watched, so a deserializer's delay-declaring device could
	 * legitimately be wired to any VC, not necessarily VC0.
	 */
	bool ipi_enable_delay_set;
	/* Receiver-owned delay used only in IPI controller mode. */
	u32 ipi_controller_enable_delay_ms;
	bool ipi_controller_enable_delay_set;

	/* Use driver mutex lock for the ctrl lock */
	struct mutex lock;

	struct dwc_csi_event events[DWC_NUM_EVENTS];
	struct dentry *dbg_dir;
	/*
	 * Per-receiver bench tunables, so a sweep on one receiver does not
	 * change the other. Recovery policy stays module-wide (hardirq reader).
	 * The glue pair is read under @lock; the bool is read on its own.
	 */
	unsigned int glue_src_burst_len;
	unsigned int glue_buf_full_th;
	bool ipi_cut_through;
	const struct dwc_csi_pix_format *csi_fmt[DWC_CSI2RX_NUM_VC];
	struct v4l2_dv_timings dv_tmg;

	/* Rate limiter for CSI error live logging (all INT_ST_MAIN events) */
	struct ratelimit_state crc_err_rs;

	/*
	 * Private high-priority WQ: stop-state (LP-11) poll, deferred IPI
	 * enable, and IPI-overflow recovery.
	 */
	struct workqueue_struct *wq;
	struct work_struct work;
	int poll_state;

	/* Deferred IPI enable, used when either enable delay is configured. */
	struct delayed_work ipi_work;
	/* IPI timing mode: false = camera timing (default), true = controller */
	bool ipi_mode_controller;

	/* dv-timings vertical front porch (kept short - see dwc_csi_ipi_vfp_lines) */
	u32 ipi_vfp_static;

	/* Enable embedded data on IPI data type register */
	bool ipi_emb_data_en;

	/*
	 * IPI-overflow recovery. On a burst of IPI Interface Fatal IRQs (CSI
	 * pixel overflow when the video DMA halts on an invalid LLI) the
	 * hardirq schedules ovf_recover_work, which resets the CSI + glue and
	 * then asks the downstream DMA to resume via ovf_resume[vc]() (registered
	 * by the vdma capture driver, which owns the dma_chan). Rate-limited per
	 * VC so a persistent fault on one VC can't reset-storm, and can't be
	 * masked by / mask another VC's own recovery need.
	 */
	struct work_struct ovf_recover_work;
	unsigned long ovf_pending_mask;	/* bit per VC awaiting recovery */
	/* re-arms the IPI-fatal mask a confirmed overflow storm turned off */
	struct delayed_work ovf_rearm_work;
	unsigned long ovf_masked_ipis;	/* bit per IPI the storm gate masked */
	int (*ovf_quiesce[DWC_CSI2RX_NUM_VC])(void *data);
	int (*ovf_resume[DWC_CSI2RX_NUM_VC])(void *data);
	void *ovf_resume_data[DWC_CSI2RX_NUM_VC];
	bool streaming;
	bool phy_inited;	/* phy_init() owed a phy_exit() */
	bool phy_powered;	/* phy_power_on() owed a phy_power_off() */
	bool start_check_ready;	/* enable_streams consumed the stop-state poll */
	bool start_in_flight;	/* an enable_streams is running on this receiver */
	bool recovering;	/* a re-init or overflow recovery owns the pipe */
	wait_queue_head_t recover_wq;	/* woken when recovering clears */
	unsigned int ovf_recover_count[DWC_CSI2RX_NUM_VC];
	ktime_t ovf_last_recover_time[DWC_CSI2RX_NUM_VC];	/* monotonic time of the last recovery per VC, for decay */
	unsigned int ovf_burst_count;	/* IPI fatals seen in the current burst window */
	ktime_t ovf_burst_start;	/* start of that fixed (non-sliding) window */
	ktime_t ovf_last_frame[DWC_CSI2RX_NUM_VC];	/* last good frame per VC (or IPI enable / recovery) */
	u32 ovf_frame_gap_ms[DWC_CSI2RX_NUM_VC];	/* last measured inter-frame gap per VC, for the stall threshold */

	/* start-up quality window: Data ID errors counted from IPI enable */
	struct delayed_work start_check_work;
	unsigned int start_check_did;	/* events[] index of the Data ID group */
	unsigned int start_check_pkt;	/* events[] index of the packet-fatal group */
	unsigned int start_check_base;
	unsigned int start_check_pkt_base;
	unsigned int start_check_attempts;
	u32 start_check_reinits;	/* per stream */
	u32 start_check_last;		/* Data ID errors in the last window */
	u32 start_check_last_pkt;	/* packet fatals in the last window */
	u32 start_check_last_good;	/* worst channel, last window */
	u32 start_check_last_bad;
	u32 start_check_stalls;		/* windows with more bad completions than good */
	u32 start_check_failed;		/* re-inits that did not complete */
	const char *start_check_trip;	/* what tripped the last re-init */
	u32 start_check_trip_did;	/* that window's counts */
	u32 start_check_trip_pkt;
	u32 start_check_trip_good;
	u32 start_check_trip_bad;
	u32 start_check_mask;		/* VCs with a live vdma when the window opened */
	u32 start_check_good[DWC_CSI2RX_NUM_VC];	/* good_frames at that point */
	u32 start_check_bad[DWC_CSI2RX_NUM_VC];
	u32 good_frames[DWC_CSI2RX_NUM_VC];	/* vdma completions, good and bad */
	u32 bad_frames[DWC_CSI2RX_NUM_VC];

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
		/*
		 * SIMOR is RAW8 on the wire with three samples per image pixel:
		 * same registers as Y8, derived from pixels x bpp or from the
		 * sample count.
		 */
		.code = MEDIA_BUS_FMT_METOAK_RGBD_3X8,
		.output = MEDIA_BUS_FMT_METOAK_RGBD_3X8,
		.data_type = MIPI_CSI2_DT_RAW8,
		.width = 8 * METOAK_RGBD_BPP,
		.samples_per_pixel = METOAK_RGBD_BPP,
		.no_ipi_sync_event = true,
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
/*
 * Mask of IPIs whose virtual channel actually has a sensor bound.
 *
 * The IPI and glue bring-up used a hardcoded 0xf, configuring and
 * enabling all four channels regardless of what is connected. On a
 * two-camera board that leaves channels 2 and 3 enabled with no source,
 * each still holding a buf_full_threshold in the glue pixel buffer.
 * Confirmed on Modalix HHHLV2: with a single camera streaming, all four
 * mipi_chctl_chN read 0x8801 - enable_chn set on every channel.
 *
 * Configure only the channels that can actually receive video.
 */
static u32 dwc_csi_active_ipi_mask(struct dwc_csi_device *csidev)
{
	unsigned int vc;
	u32 mask = 0;

	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++)
		if (csidev->source_sd[vc])
			mask |= BIT(vc);

	/*
	 * Never return 0. If nothing has bound yet, fall back to the old
	 * behaviour rather than configuring no channels at all.
	 */
	return mask ? mask : 0xf;
}

static int dwc_csi_device_pg_enable(struct dwc_csi_device *csidev)
{
	/* The pattern generator drives VC0 only. */
	const struct dwc_csi_pix_format *csi_fmt = csidev->csi_fmt[0];
	struct v4l2_subdev *sd = &csidev->sd;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 val;
	u32 mask = 0xf;
	int i;

	if (!csidev->pg_enable)
		return 0;

	if (!csi_fmt) {
		dev_dbg(csidev->dev, "CSI pixel format is NULL\n");
		return -EINVAL;
	}

	if (csi_fmt->data_type != MIPI_CSI2_DT_RGB888) {
		dev_dbg(csidev->dev, "Pattern generator only support RGB888\n");
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

/*
 * Start check: for start_check_ms after IPI enable count the Data ID
 * interrupts (unmasked only for the window); at or above
 * start_check_data_id re-initialise the PHY and host, at most
 * start_check_retries times per stream. 0 ms = off. Metoak S315 depth,
 * 1.5 s window: good starts count 0-3, bad starts 5-17.
 */
static unsigned int start_check_ms = 1500;
static unsigned int start_check_data_id = 5;
/* packet-fatal (uncorrectable header ECC) interrupts: good 7-24, bad 10-40 */
static unsigned int start_check_pkt_fatal = 25;
/*
 * More vdma completions flagged bad than good, of at least
 * DWC_START_CHECK_MIN_FRAMES, is a bad start (a consumer that queues
 * nothing gets no stall coverage and no false stall).
 */
#define DWC_START_CHECK_MIN_FRAMES	10
static bool start_check_stall = true;
static unsigned int start_check_retries = 2;

/*
 * Held under csidev->lock for the span of a re-init or overflow
 * recovery, which drops the lock around the DMA callbacks; a stream
 * start or stop waits it out.
 */
static void dwc_csi_set_recovering(struct dwc_csi_device *csidev, bool on)
{
	WRITE_ONCE(csidev->recovering, on);
	if (!on)
		wake_up_all(&csidev->recover_wq);
}

static void dwc_csi_wait_recovering(struct dwc_csi_device *csidev)
{
	if (!wait_event_timeout(csidev->recover_wq,
				!READ_ONCE(csidev->recovering),
				msecs_to_jiffies(DWC_RECOVER_WAIT_MS)))
		dev_dbg(csidev->dev, "recovery still running after %u ms\n",
			DWC_RECOVER_WAIT_MS);
}

/*
 * Open a window: sample the bases, unmask Data ID, schedule. Lock-free
 * on purpose: only the work reads the bases and a stop cancels the
 * window before any restart; ovf_resume[] is stable here because the
 * vdma registers before its stream starts; a racing locked clear leaves
 * a VC at 0/0, skipped.
 */
static void dwc_csi_start_check_arm(struct dwc_csi_device *csidev)
{
	u32 mask = 0;
	unsigned int i;

	if (csidev->start_check_did == DWC_NUM_EVENTS ||
	    csidev->start_check_pkt == DWC_NUM_EVENTS)
		return;
	/* only a VC with a live vdma can stall; a stopped one drops out */
	for (i = 0; i < DWC_CSI2RX_NUM_VC; i++)
		if (csidev->ovf_resume[i])
			mask |= BIT(i);
	csidev->start_check_base =
		READ_ONCE(csidev->events[csidev->start_check_did].counter);
	csidev->start_check_pkt_base =
		READ_ONCE(csidev->events[csidev->start_check_pkt].counter);
	csidev->start_check_mask = mask;
	for (i = 0; i < DWC_CSI2RX_NUM_VC; i++) {
		csidev->start_check_good[i] = READ_ONCE(csidev->good_frames[i]);
		csidev->start_check_bad[i] = READ_ONCE(csidev->bad_frames[i]);
	}
	dwc_csi_write(csidev, CSI2RX_INT_MSK_DATA_ID, 0xffff);
	/* not csidev->wq: its ordered slot may be held by work waiting on csidev->lock */
	queue_delayed_work(system_freezable_wq, &csidev->start_check_work,
			   msecs_to_jiffies(start_check_ms));
}

static void dwc_csi_ipi_enable(struct dwc_csi_device *csidev)
{
	u32 val;
	u32 mask = dwc_csi_active_ipi_mask(csidev);
	ktime_t now = ktime_get();
	int i;

	/* Pixels can only flow from here on: start the stall grace now, not
	 * at stream start (IPI enable may be deferred by ipi_enable_delay_ms). */
	for (i = 0; i < DWC_CSI2RX_NUM_VC; i++)
		WRITE_ONCE(csidev->ovf_last_frame[i], now);

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

	if (start_check_ms && READ_ONCE(csidev->streaming) &&
	    csidev->start_check_attempts <= start_check_retries)
		dwc_csi_start_check_arm(csidev);
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
	/*
	 * Must match dwc_csi_ipi_enable(): a disable that resets more channels
	 * than the next enable restores leaves the extra ones stuck. Measured
	 * on HHHLV2 with two cameras - disabling all four but enabling only
	 * VC0/VC1 left glue ch2/ch3 with STAT_CTRL RST_CNTS=0xff and
	 * PXL_CNT_CTRL CNT_RST set from the first stream-off onwards.
	 */
	u32 mask = dwc_csi_active_ipi_mask(csidev);
	u32 val;

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			u32 oflow;

			dwc_ipi_write(csidev, MODE, i, 0);
			dwc_gluen_write(csidev, CHAN_CTRL, i, 0);
			/* 8-bit hardware counter; wiped by the reset below */
			oflow = DWC_CSI2GLUE_STAT_CTRL_OFLOW_GET(
					dwc_gluen_read(csidev, STAT_CTRL, i));
			if (oflow)
				dev_dbg(csidev->dev,
					"glue ch%d overflow count: %u\n",
					i, oflow);
			val = DWC_CSI2GLUE_PXL_CNT_CTRL_CNT_RST;
			dwc_gluen_write(csidev, PXL_CNT_CTRL, i, val);
			val = DWC_CSI2GLUE_STAT_CTRL_RST_CNTS(0xff);
			dwc_gluen_write(csidev, STAT_CTRL, i, val);
		}
}

/*
 * IPI HSA_TIME / HBP_TIME / HSD_TIME / HLINE_TIME all count in IPI pixel-clock
 * cycles, and the line budget must add up:
 *
 *   HLINE_TIME = HSA + HBP + HFP + Hactive
 *
 * The dv-timings horizontal values are already in that domain (bt.pixelclock IS
 * the IPI clock, simaai,bt-pixelclock = 500 MHz), so they are programmed as-is.
 * Only intervals derived from a time - the D-PHY active window and the sensor
 * line - need converting.
 */
static u32 dwc_csi_ns_to_cycles(struct dwc_csi_device *csidev, u32 ns)
{
	return div_u64((u64)ns * csidev->dv_tmg.bt.pixelclock, 1000000000ULL);
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
/*
 * Glue pixel-path tunables, exposed through debugfs so they can be swept on
 * the bench without rebuilding.
 *
 * glue_src_burst_len : VIS_CTRL_GLUE_GENERAL[6:0], bytes per DMA burst request
 * glue_buf_full_th   : VIS_CTRL_GLUE_PIX[6:0],     pixel-buffer full threshold
 *
 * Neither governs the dual-camera line loss that prompted the sweep, recorded
 * here so it is not repeated: src_burst_len at 8/16/32/64 and buf_full_th at
 * 16/32/56/127 all leave MIPI_VIS_STAT_FRAME short by an exact multiple of 16
 * lines, and none produces a full 1080-line frame. The cause was the
 * deserializer running two of its four wired data lanes, not the glue.
 *
 * Both default to the in-tree constants and are re-applied on every stream-on,
 * so change the value and restart the stream for it to take effect.
 */
static u32 dwc_csi_calc_hactive_phy_time(struct dwc_csi_device *csidev, u32 width,
					 unsigned int vc)
{
	const struct dwc_csi_pix_format *csi_fmt = csidev->csi_fmt[vc];
	u32 num_lanes = csidev->bus.num_data_lanes;
	u64 data_rate_mbps;

	/* MIPI D-PHY data rate per lane (Mbps) = 2 * link frequency (DDR) */
	data_rate_mbps = div_u64(csidev->link_freq * 2, 1000000);

	if (!data_rate_mbps || !num_lanes)
		return 0;

	return div_u64((u64)width * csi_fmt->width * 1000,
		       data_rate_mbps * num_lanes);
}

/* Bench override for the IPI delay in either timing mode (-1 = DT/default). */
static int ipi_enable_delay_override_ms = -1;

static u32 dwc_csi_clamp_ipi_enable_delay(struct dwc_csi_device *csidev,
					  const char *property, u32 delay_ms)
{
	if (delay_ms <= DWC_CSI2RX_IPI_ENABLE_DELAY_MAX_MS)
		return delay_ms;

	dev_warn(csidev->dev, "%s %u ms exceeds %u ms; clamping\n",
		 property, delay_ms, DWC_CSI2RX_IPI_ENABLE_DELAY_MAX_MS);
	return DWC_CSI2RX_IPI_ENABLE_DELAY_MAX_MS;
}

/* The IPI line must run slightly LONGER than the sensor line: equal or
 * shorter underruns (Hline error 0x10), +8 or more overflows (0x3E).
 * +2 cycles is the HW-team's soak-validated point in the clean band. */
static unsigned int ipi_line_margin_cy = 2;

/* Sensor-derived HFP overrides the dv-timings one, which cannot track HMAX
 * across modes. Clear to program the dv-timings HFP verbatim (bench sweeps). */
static bool ipi_hline_from_sensor = true;

/*
 * Sensor line time in ns derived from the source subdev's HBLANK and
 * PIXEL_RATE controls, 0 when unavailable. In controller mode the IPI line
 * time must track the sensor line time (per-mode HMAX), which the static
 * dv-timings HFP cannot express across modes.
 */
/*
 * IPI front porch, clamped to the 10-bit register field (a larger value wraps
 * silently - verified on HW). The porch stays the SHORT static dv-timings
 * value on purpose: the IPI waits at frame end for the next sensor FS, so an
 * IPI frame shorter than the sensor's is always safe, while one sized to
 * match it overruns (the +margin cy/line makes the IPI frame longer in time)
 * and storms IPI_FATAL - measured on the LI module, 4 frames vs 888. A larger
 * sensor VBLANK (longer exposure) only lengthens the wait, so no tracking is
 * needed; runtime reprogramming would also need the IPI flush + soft-reset
 * sequence, which a plain register write is not.
 */
static u32 dwc_csi_ipi_vfp_lines(struct dwc_csi_device *csidev)
{
	u32 vfp = csidev->ipi_vfp_static;

	if (vfp > CSI2RX_IPI_VFP_LINES_MAX) {
		dev_dbg_ratelimited(csidev->dev,
				    "IPI vfp %u exceeds the %lu-line field, clamping\n",
				    vfp, (unsigned long)CSI2RX_IPI_VFP_LINES_MAX);
		vfp = CSI2RX_IPI_VFP_LINES_MAX;
	}
	return vfp;
}

static u32 dwc_csi_sensor_line_time_ns(struct dwc_csi_device *csidev, u32 width,
				       unsigned int vc, u64 *pixel_rate_hz)
{
	struct v4l2_ctrl *hblank, *rate;
	s64 pixel_rate;
	s32 hblank_pixels;
	u32 line_pixels;

	*pixel_rate_hz = 0;

	if (vc >= DWC_CSI2RX_NUM_VC)
		return 0;

	if (!csidev->source_sd[vc] || !csidev->source_sd[vc]->ctrl_handler)
		return 0;

	hblank = v4l2_ctrl_find(csidev->source_sd[vc]->ctrl_handler,
				V4L2_CID_HBLANK);
	rate = v4l2_ctrl_find(csidev->source_sd[vc]->ctrl_handler,
			      V4L2_CID_PIXEL_RATE);
	if (!hblank || !rate)
		return 0;

	pixel_rate = v4l2_ctrl_g_ctrl_int64(rate);
	if (pixel_rate <= 0)
		return 0;

	/*
	 * hblank == 0 means the driver does not model line timing (e.g. the
	 * econ MCU-bridge exposes a placeholder); no real sensor has zero
	 * horizontal blanking, so treat it as unavailable.
	 */
	hblank_pixels = v4l2_ctrl_g_ctrl(hblank);
	if (hblank_pixels <= 0)
		return 0;
	*pixel_rate_hz = pixel_rate;

	line_pixels = hblank_pixels + width;
	return div64_u64((u64)line_pixels * 1000000000ULL, pixel_rate);
}

/*
 * The stream this VC's sink pad carries. An aggregating source feeding
 * several VCs off one pad needs a distinct stream per VC.
 */
static u32 dwc_csi_sink_stream(struct v4l2_subdev_state *state, unsigned int vc)
{
	struct v4l2_subdev_route *route;
	unsigned int sink_pad = DWC_CSI2RX_PAD_SINK + 2 * vc;

	for_each_active_route(&state->routing, route)
		if (route->sink_pad == sink_pad)
			return route->sink_stream;

	return 0;
}

/*
 * Ask the source which VC it transmits on, and cache it. Falls back to the
 * pad-derived VC when the source has no frame descriptor.
 */
static unsigned int dwc_csi_get_subdev_vc(struct dwc_csi_device *csidev,
					  struct v4l2_subdev_state *state,
					  unsigned int vc)
{
	struct v4l2_mbus_frame_desc fd = { 0 };
	const struct v4l2_mbus_frame_desc_entry *entry = NULL;
	unsigned int i;

	if (vc >= DWC_CSI2RX_NUM_VC || !csidev->source_sd[vc])
		return vc;

	if (v4l2_subdev_call(csidev->source_sd[vc], pad, get_frame_desc,
			     csidev->remote_pad[vc], &fd))
		return vc;

	if (fd.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2 || !fd.num_entries)
		return vc;

	for (i = 0; i < fd.num_entries; i++)
		dev_dbg(csidev->dev,
			"%s pad %u: stream %u, vc %u, dt 0x%02x, pixelcode 0x%04x\n",
			csidev->source_sd[vc]->name, csidev->remote_pad[vc],
			fd.entry[i].stream, fd.entry[i].bus.csi2.vc,
			fd.entry[i].bus.csi2.dt, fd.entry[i].pixelcode);

	/*
	 * One entry per stream, each with its own VC. Pick the entry for the
	 * stream this sink pad consumes.
	 */
	if (fd.num_entries == 1) {
		entry = &fd.entry[0];
	} else {
		u32 stream = dwc_csi_sink_stream(state, vc);

		for (i = 0; i < fd.num_entries; i++)
			if (fd.entry[i].stream == stream) {
				entry = &fd.entry[i];
				break;
			}
	}

	if (!entry) {
		dev_dbg(csidev->dev,
			"VC%u: no frame desc entry for the stream on %s pad %u; using VC%u\n",
			vc, csidev->source_sd[vc]->name,
			csidev->remote_pad[vc], vc);
		return vc;
	}

	/*
	 * VIRTUAL_CHANNEL_EXT is off, so VCID is 2 bits and FIELD_PREP would
	 * silently truncate anything above 3.
	 */
	if (entry->bus.csi2.vc >= DWC_CSI2RX_NUM_VC) {
		dev_dbg(csidev->dev,
			"%s reports VC%u, out of range for the IPI VCID field; using VC%u\n",
			csidev->source_sd[vc]->name, entry->bus.csi2.vc, vc);
		return vc;
	}

	if (entry->bus.csi2.vc != vc)
		dev_dbg(csidev->dev,
			"VC%u: source sends on VC%u, programming IPI%u for it\n",
			vc, entry->bus.csi2.vc, vc + 1);

	return entry->bus.csi2.vc;
}

/*
 * Geometry for one IPI. IPI n is fed by VC n's sink pad, so each glue
 * channel is sized from its own VC's format.
 */
static const struct v4l2_mbus_framefmt *
dwc_csi_ipi_sink_format(struct v4l2_subdev_state *state, unsigned int ipi)
{
	return v4l2_subdev_state_get_format(state,
					    DWC_CSI2RX_PAD_SINK + 2 * ipi,
					    dwc_csi_sink_stream(state, ipi));
}

static void dwc_csi_check_line_quantization(struct dwc_csi_device *csidev,
					    u64 source_pixel_rate)
{
	u64 ipi_pixel_rate = csidev->dv_tmg.bt.pixelclock;
	u32 remaining_cy;

	/* A zero margin underruns; the rounded-up line plus margin must stay below +8 cycles. */
	if (!ipi_line_margin_cy ||
	    ipi_line_margin_cy >= DWC_CSI2RX_IPI_LINE_OVERFLOW_MARGIN_CY)
		goto warn;

	remaining_cy = DWC_CSI2RX_IPI_LINE_OVERFLOW_MARGIN_CY -
		ipi_line_margin_cy;
	/* Keep the bound strict even if the ns conversion rounding changes. */
	if (ipi_pixel_rate < (u64)remaining_cy * source_pixel_rate)
		return;

warn:
	dev_warn_ratelimited(csidev->dev,
			     "IPI line quantization unsafe: limit +%u cy, IPI %llu Hz, source %llu Hz, margin %u cy\n",
			     DWC_CSI2RX_IPI_LINE_OVERFLOW_MARGIN_CY,
			     ipi_pixel_rate, source_pixel_rate,
			     ipi_line_margin_cy);
}

/* Program the glue datapath for every IPI instance selected by mask */
static void dwc_csi_device_glue_config(struct dwc_csi_device *csidev,
				       struct v4l2_subdev_state *state,
				       u32 mask, u32 width, u32 height)
{
	const struct v4l2_bt_timings *bt_tmg = &csidev->dv_tmg.bt;
	unsigned int src_burst_len, buf_full_th;
	u32 val;
	int i;

	/* one read of the pair, so a sweep cannot program a torn set */
	mutex_lock(&csidev->lock);
	src_burst_len = csidev->glue_src_burst_len;
	buf_full_th = csidev->glue_buf_full_th;
	mutex_unlock(&csidev->lock);

	val = DWC_CSI2GLUE_VIS_CTRL_SRC_BURST_LEN(src_burst_len);
	val |= DWC_CSI2GLUE_VIS_CTRL_BUF_SRAM_PIPE_EN;
	dwc_glue_write(csidev, DWC_CSI2GLUE_VIS_CTRL, val);

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			const struct v4l2_mbus_framefmt *ifmt =
				dwc_csi_ipi_sink_format(state, i);
			const struct dwc_csi_pix_format *csi_fmt = csidev->csi_fmt[i];
			u32 iwidth = ifmt ? ifmt->width : width;
			u32 iheight = ifmt ? ifmt->height : height;
			/*
			 * @iwidth is the image width, iwire_width the samples on
			 * the wire: sample-counting registers take the latter,
			 * pixels x bpp the former.
			 */
			u32 iwire_width = iwidth *
					  max_t(u32, csi_fmt->samples_per_pixel, 1);

			val = DWC_CSI2GLUE_CHAN_CTRL_EN;
			val |= DWC_CSI2GLUE_CHAN_CTRL_DBITS(DWC_CSI2GLUE_STREAM_DBITS);
			val |= DWC_CSI2GLUE_CHAN_CTRL_PACK;
			dwc_gluen_write(csidev, CHAN_CTRL, i, val);

			val = DWC_CSI2GLUE_VIS_CTRL_PIX_BUF_FULL_TH(buf_full_th);
			val |= DWC_CSI2GLUE_VIS_CTRL_PIX_END_PATTERN(DWC_CSI2GLUE_END_PATTERN_DEF);
			val |= DWC_CSI2GLUE_VIS_CTRL_PIX_XFER_FRAME_BOUND;
			val |= DWC_CSI2GLUE_VIS_CTRL_PIX_FRAME_FMT(DWC_CSI2GLUE_FRAME_FMT_DEF);
			dwc_gluen_write(csidev, VIS_CTRL_PIX, i, val);

			val = DWC_CSI2GLUE_LNBYTES_LINE_BYTES(iwire_width);
			dwc_gluen_write(csidev, LNBYTES, i, val);

			val = DWC_CSI2GLUE_FRMDIM_WIDTH(iwire_width);
			val |= DWC_CSI2GLUE_FRMDIM_HEIGHT(iheight);
			dwc_gluen_write(csidev, FRMDIM, i, val);

			val = DWC_CSI2GLUE_FILTINFO_DT(csi_fmt->data_type);
			val |= DWC_CSI2GLUE_FILTINFO_CHAN(0);
			dwc_gluen_write(csidev, FILTINFO, i, val);

			val = DWC_CSI2GLUE_META_EMB_LD(bt_tmg->il_vfrontporch * iwidth *
				csi_fmt->width / 8);
			val |= DWC_CSI2GLUE_META_EMB_TR(bt_tmg->il_vbackporch * iwidth *
				csi_fmt->width / 8);
			dwc_gluen_write(csidev, META, i, val);
		}
}

static void dwc_csi_device_ipi_config(struct dwc_csi_device *csidev,
				      struct v4l2_subdev_state *state,
				      unsigned int vc)
{
	u32 vfp_lines;
	const struct v4l2_bt_timings *bt_tmg = &csidev->dv_tmg.bt;
	const struct v4l2_mbus_framefmt *fmt;
	u32 hsa_cy, hbp_cy, hfp_cy, hactive_cy;
	u32 hline_cy;
	u32 width, height;
	u32 val = 0;
	u32 mask = dwc_csi_active_ipi_mask(csidev);
	int i;

	fmt = dwc_csi_ipi_sink_format(state, vc);
	width = fmt->width;
	height = fmt->height;

	/* dv-timings horizontal values are already IPI pixel-clock cycles */
	hsa_cy = bt_tmg->hsync;
	hbp_cy = bt_tmg->hbackporch;
	hfp_cy = bt_tmg->hfrontporch;
	/* Hactive is the D-PHY active receive time, converted to cycles */
	/* Feeds HLINE_TIME, an IPI1-only register, so it pairs with VC0. */
	hactive_cy = dwc_csi_ns_to_cycles(csidev,
			dwc_csi_calc_hactive_phy_time(csidev, width, 0));

	/*
	 * Controller mode: derive HFP from the sensor line time when the
	 * sensor exposes it; the static dv-timings HFP is the fallback.
	 */
	if (csidev->ipi_mode_controller && ipi_hline_from_sensor) {
		u64 source_pixel_rate = 0;
		u32 line_ns = dwc_csi_sensor_line_time_ns(csidev, width, vc,
							  &source_pixel_rate);
		u32 line_cy = dwc_csi_ns_to_cycles(csidev, line_ns);
		u32 min_cy = hsa_cy + hbp_cy + hactive_cy + DWC_CSI2RX_HSD_TIME;

		if (line_cy > min_cy) {
			dwc_csi_check_line_quantization(csidev,
							source_pixel_rate);
			hfp_cy = line_cy - hsa_cy - hbp_cy - hactive_cy +
				 ipi_line_margin_cy;
			dev_dbg(csidev->dev,
				"IPI line timing from sensor: line %u cy (%u ns), hfp %u -> %u cy, margin %u cy\n",
				 line_cy, line_ns, bt_tmg->hfrontporch, hfp_cy,
				 ipi_line_margin_cy);
		} else if (line_ns) {
			dev_dbg(csidev->dev,
				"sensor line time %u cy below minimum %u cy, keeping dv-timings hfp\n",
				 line_cy, min_cy);
		}
	}

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			/* Select virtual channel and data type to be processed by IPI */
			val = CSI2RX_IPI_DATA_TYPE_DT(csidev->csi_fmt[i]->data_type);
			if (csidev->ipi_emb_data_en)
				val |= CSI2RX_IPI_DATA_TYPE_EMB_DATA_EN;
			dwc_ipi_write(csidev, DATA_TYPE, i, val);

			/*
			 * Filter on the VC the source reports. The pattern generator
			 * injects on VC0 regardless.
			 */
			csidev->source_vc[i] = csidev->pg_enable ? 0 :
				dwc_csi_get_subdev_vc(csidev, state, i);
			dwc_ipi_write(csidev, VCID, i,
				      CSI2RX_IPI_VCID_VC(csidev->source_vc[i]));

			/*
			 * Select IPI camera timing mode and allow the pixel stream
			 * to be non-continuous when pixel interface FIFO is empty
			 */
			val = dwc_ipi_read(csidev, MODE, i);
			/*
			 * Controller mode is IPI1-only: IPI2/3/4 have no
			 * HLINE/VSA/VBP/VFP/VACTIVE registers to program.
			 */
			if (csidev->ipi_mode_controller && i == 0) {
				val |= CSI2RX_IPI_MODE_CONTROLLER;
			} else {
				if (csidev->ipi_mode_controller)
					dev_dbg_once(csidev->dev,
						     "controller mode requested but IPI%u (VC%u) has no timing registers; using camera mode\n",
						     i + 1, i);
				val &= ~CSI2RX_IPI_MODE_CONTROLLER;
			}
			val &= ~CSI2RX_IPI_MODE_COLOR_MODE16;
			/*
			 * Cut-through forwards pixels as they arrive; clear it
			 * to store a line first and let the IPI smooth a bursty
			 * source.
			 */
			if (READ_ONCE(csidev->ipi_cut_through))
				val |= CSI2RX_IPI_MODE_CUT_THROUGH;
			else
				val &= ~CSI2RX_IPI_MODE_CUT_THROUGH;
			dwc_ipi_write(csidev, MODE, i, val);
		}

	val = CSI2RX_IPI_ADV_FEATURES_SYNC_LS_PKT;
	val |= CSI2RX_IPI_ADV_FEATURES_LINE_EVENT_SEL;
	val |= CSI2RX_IPI_ADV_FEATURES_SYNC_VIDEO_PKT;
	val |= CSI2RX_IPI_ADV_FEATURES_SYNC_EMBEDDED_PKT;
	val |= CSI2RX_IPI_ADV_FEATURES_SYNC_EVENT_MODE;

	/* no event-mode sync for a source that raises no event (Metoak S315) */
	if (csidev->csi_fmt[vc] && csidev->csi_fmt[vc]->no_ipi_sync_event)
		val &= ~CSI2RX_IPI_ADV_FEATURES_SYNC_EVENT_MODE;

	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			dwc_ipi_write(csidev, HSA_TIME, i,
				CSI2RX_IPI_HSA_TIME_VAL(hsa_cy));
			dwc_ipi_write(csidev, HBP_TIME, i,
				CSI2RX_IPI_HBP_TIME_VAL(hbp_cy));
			/* HSD time is fixed to a constant */
			dwc_ipi_write(csidev, HSD_TIME, i,
				CSI2RX_IPI_HSD_TIME_VAL(DWC_CSI2RX_HSD_TIME));
			dwc_ipi_write(csidev, ADV_FEATURES, i, val);
		}

	/* HLINE_TIME = HSA + HBP + HFP + Hactive PHY time, all in cycles */
	hline_cy = hsa_cy + hbp_cy + hfp_cy + hactive_cy;
	if (csidev->ipi_mode_controller)
		dev_dbg(csidev->dev,
			"hsa: %u, hbp: %u, hsd: %u, hfp: %u, hactive: %u, hline: %u cy, width: %u, height: %u\n",
			hsa_cy, hbp_cy, DWC_CSI2RX_HSD_TIME, hfp_cy,
			hactive_cy, hline_cy, width, height);
	/* A line longer than the 15-bit field would wrap to a far too short
	 * one - clamp instead; the wrap would be a pixel storm. */
	if (hline_cy > DWC_CSI2RX_HLINE_TIME_MAX) {
		dev_dbg(csidev->dev, "IPI hline %u cy exceeds the %lu cy maximum, clamping\n",
			hline_cy, DWC_CSI2RX_HLINE_TIME_MAX);
		hline_cy = DWC_CSI2RX_HLINE_TIME_MAX;
	}
	val = CSI2RX_IPI_HLINE_TIME_VAL(hline_cy);
	dwc_csi_write(csidev, CSI2RX_IPI_HLINE_TIME, val);
	val = CSI2RX_IPI_VSA_LINES_VAL(bt_tmg->vsync);
	dwc_csi_write(csidev, CSI2RX_IPI_VSA_LINES, val);
	val = CSI2RX_IPI_VBP_LINES_VAL(bt_tmg->vbackporch);
	dwc_csi_write(csidev, CSI2RX_IPI_VBP_LINES, val);
	csidev->ipi_vfp_static = bt_tmg->vfrontporch;
	vfp_lines = dwc_csi_ipi_vfp_lines(csidev);
	val = CSI2RX_IPI_VFP_LINES_VAL(vfp_lines);
	dwc_csi_write(csidev, CSI2RX_IPI_VFP_LINES, val);
	val = CSI2RX_IPI_VACTIVE_LINES_VAL(height);
	dwc_csi_write(csidev, CSI2RX_IPI_VACTIVE_LINES, val);
	dwc_csi_write(csidev, CSI2RX_VIRTUAL_CHANNEL_EXT, 0x0);

	dwc_csi_device_glue_config(csidev, state, mask, width, height);

	/*
	 * Build the reset mask from scratch: val still holds the last glue META
	 * word, whose bits overlap IPI3's and IPI4's reset bits.
	 */
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, 0x0);
	val = 0;
	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i))
			val |= 1 << (i * 4);
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, val);
}

/*
 * Re-apply one VC's geometry and data type: ipi_config() sized every bound
 * VC at first stream on, so a VC formatted later holds the probe default.
 */
static void dwc_csi_ipi_resize_vc(struct dwc_csi_device *csidev,
				  struct v4l2_subdev_state *state,
				  unsigned int vc)
{
	const struct dwc_csi_pix_format *csi_fmt = csidev->csi_fmt[vc];
	const struct v4l2_bt_timings *bt_tmg = &csidev->dv_tmg.bt;
	const struct v4l2_mbus_framefmt *fmt;
	u32 width, height, wire_width, val, rstn, vcid;

	fmt = dwc_csi_ipi_sink_format(state, vc);
	if (!fmt)
		return;

	width = fmt->width;
	height = fmt->height;
	/* Samples on the wire, for the registers that count them. META below
	 * stays on @width: it is pixels x bits-per-pixel, which already
	 * carries the ratio. */
	wire_width = width * max_t(u32, csi_fmt->samples_per_pixel, 1);
	/* the source's frame descriptor first: not under our lock */
	vcid = dwc_csi_get_subdev_vc(csidev, state, vc);

	/* under the lock: the overflow recovery holds this VC in soft reset */
	mutex_lock(&csidev->lock);
	val = DWC_CSI2GLUE_LNBYTES_LINE_BYTES(wire_width);
	dwc_gluen_write(csidev, LNBYTES, vc, val);

	val = DWC_CSI2GLUE_FRMDIM_WIDTH(wire_width);
	val |= DWC_CSI2GLUE_FRMDIM_HEIGHT(height);
	dwc_gluen_write(csidev, FRMDIM, vc, val);

	val = DWC_CSI2GLUE_META_EMB_LD(bt_tmg->il_vfrontporch * width *
		csi_fmt->width / 8);
	val |= DWC_CSI2GLUE_META_EMB_TR(bt_tmg->il_vbackporch * width *
		csi_fmt->width / 8);
	dwc_gluen_write(csidev, META, vc, val);

	val = CSI2RX_IPI_DATA_TYPE_DT(csi_fmt->data_type);
	if (csidev->ipi_emb_data_en)
		val |= CSI2RX_IPI_DATA_TYPE_EMB_DATA_EN;
	dwc_ipi_write(csidev, DATA_TYPE, vc, val);

	csidev->source_vc[vc] = vcid;
	dwc_ipi_write(csidev, VCID, vc, CSI2RX_IPI_VCID_VC(vcid));

	val = DWC_CSI2GLUE_FILTINFO_DT(csi_fmt->data_type);
	val |= DWC_CSI2GLUE_FILTINFO_CHAN(0);
	dwc_gluen_write(csidev, FILTINFO, vc, val);

	dev_dbg(csidev->dev, "VC%u: IPI reprogrammed for %ux%u dt 0x%02x\n",
		vc, width, height, csi_fmt->data_type);

	/* Latch it: reset this VC's IPI only (nibble-spaced, as elsewhere). */
	rstn = dwc_csi_read(csidev, CSI2RX_IPI_SOFTRSTN);
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, rstn & ~BIT(vc * 4));
	dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, rstn | BIT(vc * 4));
	mutex_unlock(&csidev->lock);
}

/* Releases whichever of phy_init() and phy_power_on() is still held. */
static void dwc_csi_phy_release(struct dwc_csi_device *csidev)
{
	if (csidev->phy_powered)
		phy_power_off(csidev->phy);
	if (csidev->phy_inited)
		phy_exit(csidev->phy);
	csidev->phy_powered = false;
	csidev->phy_inited = false;
}

static void dwc_csi_device_reset(struct dwc_csi_device *csidev)
{
	/* Reset mipi csi host, active low */
	dwc_csi_write(csidev, CSI2RX_HOST_RESETN, 0);
	dwc_csi_write(csidev, CSI2RX_HOST_RESETN, 1);
}

static int dwc_csi_device_startup(struct dwc_csi_device *csidev)
{
	int ret;

	/* Release DWC_mipi_csi2_host from reset */
	dwc_csi_device_reset(csidev);

	ret = phy_init(csidev->phy);
	if (ret)
		return ret;
	csidev->phy_inited = true;

	ret = phy_reset(csidev->phy);
	if (ret) {
		dwc_csi_phy_release(csidev);
		return ret;
	}

	return 0;
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
	val |= CSI2RX_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_csi_write(csidev, CSI2RX_DPHY_TEST_CTRL0, val);

	/*
	 * ndelay is not necessary have MMIO operation, need dummy read to make
	 * sure above write reach target.
	 */
	dwc_csi_read(csidev, CSI2RX_DPHY_TEST_CTRL0);
	/* Wait for at least 15ns */
	udelay(1);

	val = dwc_csi_read(csidev, CSI2RX_DPHY_TEST_CTRL0);
	val &= ~CSI2RX_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_csi_write(csidev, CSI2RX_DPHY_TEST_CTRL0, val);
}

static void dwc_csi_dphy_release_reset(struct dwc_csi_device *csidev)
{
	/* Release PHY from reset */
	dwc_csi_write(csidev, CSI2RX_DPHY_SHUTDOWNZ, 0x1);
	/* Flush shutdownz before releasing rstz. */
	dwc_csi_read(csidev, CSI2RX_DPHY_SHUTDOWNZ);
	udelay(1);
	dwc_csi_write(csidev, CSI2RX_DPHY_RSTZ, 0x1);
	dwc_csi_read(csidev, CSI2RX_DPHY_RSTZ);
	udelay(1);
}

static int dwc_csi_wait_stopstate(struct dwc_csi_device *csidev, u32 timeout_us)
{
	u32 phy_stopstate;
	u32 val;
	u32 mask = 0xf;
	int ret;
	int i;

	/* Check if lanes are in stop state */
	phy_stopstate = CSI2RX_DPHY_STOPSTATE_CLK_LANE;
	phy_stopstate |= GENMASK(csidev->bus.num_data_lanes - 1, 0);
	ret = readl_poll_timeout(csidev->regs + CSI2RX_DPHY_STOPSTATE,
				 val, (val & phy_stopstate) == phy_stopstate,
				 10, timeout_us);
	if (ret)
		dev_dbg(csidev->dev, "Lanes are not in stop state(%#x)\n", val);
	/* forcerxmode is released whether or not stop state was seen */
	for (i = 0; i < ARRAY_SIZE(ipi_regs); i++)
		if ((mask) & (1 << i)) {
			val = dwc_gluen_read(csidev, DPHY_CTRL, i);
			val &= ~DWC_CSI2GLUE_DPHY_CTRL_FORCE_MODE;
			dwc_gluen_write(csidev, DPHY_CTRL, i, val);
		}
	return ret;
}

static void stopstate_poll(struct work_struct *work)
{
	struct dwc_csi_device *csidev = container_of(work, typeof(*csidev), work);

	csidev->poll_state = DWC_POLL_ACTIVE;
	csidev->poll_state = dwc_csi_wait_stopstate(csidev, DWC_STOPSTATE_TIMEOUT) ?
			     DWC_POLL_TIMEOUT : DWC_POLL_SUCCESS;
}

static int dwc_csi_device_init_phy(struct dwc_csi_device *csidev)
{
	union phy_configure_opts opts;
	int ret;

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
	if (!ret)
		csidev->phy_powered = true;
	dwc_csi_dphy_release_reset(csidev);
	dwc_csi_write(csidev, CSI2RX_HOST_RESETN, 0x1);

	return ret;
}

static int dwc_csi_device_init(struct dwc_csi_device *csidev)
{
	ktime_t end;
	int ret;

	ret = dwc_csi_device_init_phy(csidev);
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
			dev_dbg(csidev->dev, "Overriding stale stop-state poll(%d)\n",
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
	u32 delay_ms;

	/* Instrumentation-only override deliberately bypasses the DT 5 s clamp. */
	if (ipi_enable_delay_override_ms >= 0)
		delay_ms = ipi_enable_delay_override_ms;
	else if (csidev->ipi_mode_controller &&
		 csidev->ipi_controller_enable_delay_set)
		delay_ms = csidev->ipi_controller_enable_delay_ms;
	else
		delay_ms = csidev->ipi_enable_delay_ms;

	/* No delay: enable inline to avoid scheduling latency. */
	if (!delay_ms) {
		dwc_csi_ipi_enable(csidev);
		return 0;
	}

	dev_dbg(csidev->dev, "deferring IPI enable by %u ms\n", delay_ms);
	queue_delayed_work(csidev->wq, &csidev->ipi_work,
			   msecs_to_jiffies(delay_ms));

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

	dwc_csi_phy_release(csidev);

	/* Check clock lanes are not in High Speed Mode */
	val = dwc_csi_read(csidev, CSI2RX_DPHY_RX_STATUS);
	if (val & CSI2RX_DPHY_RX_STATUS_CLK_LANE_HS) {
		dev_dbg(dev, "Clock lanes are still in HS mode\n");
		return -EINVAL;
	}

	return 0;
}

static void dwc_csi_device_enable_interrupts(struct dwc_csi_device *csidev, bool on)
{
	u32 active = dwc_csi_active_ipi_mask(csidev);
	unsigned int i;

	/* Define errors to be enabled */
	dwc_csi_write(csidev, CSI2RX_INT_MSK_DPHY_FATAL, on ? 0x3 : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_PKT_FATAL, on ? 0x3 : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_DPHY, on ? 0x30003 : 0);
	/* the start check unmasks Data ID for its window only */
	if (!on)
		dwc_csi_write(csidev, CSI2RX_INT_MSK_DATA_ID, 0);
	/*
	 * Only the IPIs with a bound source are configured; unmasking another
	 * would latch a fatal whose cause register nobody reads.
	 */
	csidev->ovf_masked_ipis = 0;
	for (i = 0; i < DWC_CSI2RX_NUM_VC; i++) {
		if (!(active & BIT(i)))
			continue;
		dwc_csi_write(csidev, CSI2RX_INT_MSK_IPI_FATAL_N(i),
			      on ? 0x7f : 0);
	}
	dwc_csi_write(csidev, CSI2RX_INT_MSK_PLD_CRC_FATAL,
		      on ? 0xFFFFFFFF : 0);
	dwc_csi_write(csidev, CSI2RX_INT_MSK_CRC_FRAME_FATAL,
		      on ? 0xFFFFFFFF : 0);
	/*
	 * BNDRY/SEQ/DATA_ID/ECC_CORRECTED stay masked as IRQ sources on
	 * purpose: their INT_ST_MAIN bits are visible (and their sub-status
	 * readable) whenever the handler runs for an unmasked source, which is
	 * how their counters have always been sampled. Unmasking them turns a
	 * structural per-line condition into a storm — the Metoak S315 depth
	 * stream repeats CSI frame numbers, measured as exactly one frame-
	 * sequence fatal per line (~8.2k IRQ/s at 22.7 fps x 360 lines).
	 */
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
			dev_dbg(csidev->dev, "%s events: %d\n",
				csidev->events[i].name, counter);
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
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI2_MODE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI2_VCID),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI2_DATA_TYPE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI3_MODE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI3_VCID),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI3_DATA_TYPE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI4_MODE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI4_VCID),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_IPI4_DATA_TYPE),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_DPHY_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_PKT_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_DPHY),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_IPI_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_IPI2_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_IPI3_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_IPI4_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_BNDRY_FRAME_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_SEQ_FRAME_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_CRC_FRAME_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_PLD_CRC_FATAL),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_DATA_ID),
		DWC_MIPI_CSIS_DEBUG_REG(CSI2RX_INT_ST_ECC_CORRECTED),
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
	if (dwc_csi_pad_to_vc(code->pad) < DWC_CSI2RX_NUM_VC) {
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
	if (dwc_csi_pad_to_vc(sdformat->pad) < DWC_CSI2RX_NUM_VC)
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

	{
		unsigned int vc = dwc_csi_sink_pad_to_vc(sdformat->pad);

		/* Store the CSIS format descriptor for active formats. */
		if (vc < DWC_CSI2RX_NUM_VC)
			csidev->csi_fmt[vc] = csi_fmt;

		if (vc < DWC_CSI2RX_NUM_VC && csidev->source_sd[vc]) {
			struct v4l2_subdev_format up = *sdformat;
			struct v4l2_subdev_state *ust;
			int ret = -ENODEV;

			ust = v4l2_subdev_lock_and_get_active_state(csidev->source_sd[vc]);
			if (ust) {
				ret = v4l2_subdev_routing_find_opposite_end(&ust->routing,
						csidev->remote_pad[vc], sdformat->stream,
						&up.pad, &up.stream);
				v4l2_subdev_unlock_state(ust);
			}
			if (!ret)
				v4l2_subdev_call_state_active(csidev->source_sd[vc], pad,
							      set_fmt, &up);
		}
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
	unsigned int vc;
	int ret;

	vc = dwc_csi_pad_to_vc(pad);
	if (vc >= DWC_CSI2RX_NUM_VC || !csidev->source_sd[vc])
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));

	ret = v4l2_subdev_call(csidev->source_sd[vc], pad, get_frame_desc,
			       csidev->remote_pad[vc], &source_fd);
	if (ret < 0) {
		/*
		 * Fall back to the queried pad's own VC and format; the pad is the only
		 * thing identifying the stream here.
		 */
		dev_dbg(csidev->dev,
			"Remote sub-device on pad %d should implement .get_frame_desc! Forcing VC = %u and DT = %x\n",
			pad, vc, csidev->csi_fmt[vc]->data_type);
		fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = 1;
		fd->entry[0].pixelcode = csidev->csi_fmt[vc]->code;
		fd->entry[0].bus.csi2.vc = vc;
		fd->entry[0].bus.csi2.dt = csidev->csi_fmt[vc]->data_type;

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
			dev_dbg(csidev->dev,
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

/*
 * The IPI and the glue are programmed from the sink-pad format, so a format the
 * source does not actually send never completes a frame: the IPI waits for
 * lines that never arrive and the capture blocks until the frame watchdog gives
 * up. Nothing else validates this (nobody starts a media pipeline here, so the
 * .link_validate ops never run), so check it before arming the hardware.
 */
static int dwc_csi_check_source_format(struct dwc_csi_device *csidev,
				       struct v4l2_subdev_state *state,
				       unsigned int vc)
{
	struct v4l2_subdev_format src = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	const struct v4l2_mbus_framefmt *sink;
	u32 stream;

	if (vc >= DWC_CSI2RX_NUM_VC || !csidev->source_sd[vc])
		return 0;

	stream = dwc_csi_sink_stream(state, vc);
	src.pad = csidev->remote_pad[vc];
	src.stream = stream;

	/* A source that cannot report its format is left to the old behaviour. */
	if (v4l2_subdev_call_state_active(csidev->source_sd[vc], pad, get_fmt, &src))
		return 0;

	sink = v4l2_subdev_state_get_format(state,
					    DWC_CSI2RX_PAD_SINK + 2 * vc, stream);
	if (!sink)
		return 0;

	if (src.format.width == sink->width &&
	    src.format.height == sink->height &&
	    src.format.code == sink->code)
		return 0;

	dev_dbg(csidev->dev,
		"format mismatch: %s sends %ux%u code 0x%04x, sink pad is set to %ux%u code 0x%04x\n",
		csidev->source_sd[vc]->name, src.format.width, src.format.height,
		src.format.code, sink->width, sink->height, sink->code);
	return -EPIPE;
}

static int dwc_csi_update_link_freq(struct dwc_csi_device *csidev, unsigned int vc)
{
	s64 link_freq;

	if (vc >= DWC_CSI2RX_NUM_VC || !csidev->source_sd[vc])
		return -ENODEV;

	link_freq = v4l2_get_link_freq(&csidev->source_sd[vc]->entity.pads[csidev->remote_pad[vc]],
				       csidev->csi_fmt[vc]->width,
				       csidev->bus.num_data_lanes * 2);
	if (link_freq < 0) {
		dev_dbg(csidev->dev, "Unable to obtain link frequency: %d\n",
			(int)link_freq);
		return link_freq;
	}
	if (link_freq != csidev->link_freq)
		dev_dbg(csidev->dev, "link frequency %lld -> %lld Hz\n",
			csidev->link_freq, link_freq);
	csidev->link_freq = link_freq;

	return 0;
}

static int dwc_csi_start_stream(struct dwc_csi_device *csidev,
				struct v4l2_subdev_state *state,
				unsigned int vc)
{
	int ret;

	ret = dwc_csi_check_source_format(csidev, state, vc);
	if (ret)
		return ret;

	/*
	 * The notifier only seeds LINK_FREQ. Refresh the value negotiated by
	 * set_fmt before programming the D-PHY and controller for every stream.
	 */
	ret = dwc_csi_update_link_freq(csidev, vc);
	if (ret)
		return ret;

	ret = dwc_csi_device_startup(csidev);
	if (ret)
		return ret;

	dwc_csi_device_ipi_config(csidev, state, vc);

	ret = dwc_csi_device_init(csidev);
	if (ret)
		goto err_phy_exit;

	ret = dwc_csi_device_pg_enable(csidev);
	if (ret)
		goto err_cancel_poll;

	memset(csidev->ovf_recover_count, 0, sizeof(csidev->ovf_recover_count));
	csidev->ovf_burst_count = 0;
	memset(csidev->ovf_frame_gap_ms, 0, sizeof(csidev->ovf_frame_gap_ms));
	csidev->start_check_attempts = 0;
	csidev->start_check_reinits = 0;
	csidev->start_check_last = 0;
	csidev->start_check_last_pkt = 0;
	csidev->start_check_last_good = 0;
	csidev->start_check_last_bad = 0;
	csidev->start_check_stalls = 0;
	csidev->start_check_failed = 0;
	csidev->start_check_trip = NULL;
	csidev->start_check_trip_did = 0;
	csidev->start_check_trip_pkt = 0;
	csidev->start_check_trip_good = 0;
	csidev->start_check_trip_bad = 0;
	memset(csidev->good_frames, 0, sizeof(csidev->good_frames));
	memset(csidev->bad_frames, 0, sizeof(csidev->bad_frames));
	WRITE_ONCE(csidev->streaming, true);
	dwc_csi_device_enable_interrupts(csidev, true);
	/* IPI enable may open the start-check window: after the masks. */
	dwc_csi_device_hs_rx_start(csidev);

	return 0;

err_cancel_poll:
	/* device_init queued the stop-state poll; don't leave it running. */
	cancel_work_sync(&csidev->work);
	csidev->poll_state = DWC_POLL_INACTIVE;
	dwc_csi_device_hs_rx_stop(csidev);
	dwc_csi_device_pg_disable(csidev);
	return ret;

err_phy_exit:
	dwc_csi_ipi_disable(csidev);
	dwc_csi_phy_release(csidev);
	return ret;
}

static void dwc_csi_stop_stream(struct dwc_csi_device *csidev)
{
	WRITE_ONCE(csidev->streaming, false);
	WRITE_ONCE(csidev->start_check_ready, false);
	dwc_csi_device_enable_interrupts(csidev, false);
	/*
	 * Let an in-flight hardirq finish queueing before draining recovery
	 * and the re-arm it may have scheduled.
	 */
	synchronize_irq(csidev->irq);
	cancel_work_sync(&csidev->ovf_recover_work);
	cancel_delayed_work_sync(&csidev->ovf_rearm_work);
	/* streaming is false: a late IPI enable can no longer re-arm it */
	cancel_delayed_work_sync(&csidev->start_check_work);
	/* a re-init the cancel flushed may have unmasked Data ID again */
	dwc_csi_write(csidev, CSI2RX_INT_MSK_DATA_ID, 0);
	dwc_csi_device_hs_rx_stop(csidev);
	dwc_csi_device_pg_disable(csidev);
}

static int dwc_csi_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	unsigned int vc, sink_pad;
	u64 sink_streams;
	bool started = false;
	int ret;
	int poll_result;

	vc = dwc_csi_pad_to_vc(pad);
	if (vc >= DWC_CSI2RX_NUM_VC)
		return -EINVAL;

	sink_pad = DWC_CSI2RX_PAD_SINK + 2 * vc;

	if (!csidev->source_sd[vc]) {
		dev_dbg(csidev->dev, "VC%u: sensor not linked to CSIS pad\n", vc);
		return -EPIPE;
	}
	/* the start check must not re-init across a start of another VC */
	WRITE_ONCE(csidev->start_in_flight, true);
	/* pairs with the check work: it sees this flag or we see recovering */
	smp_mb();
	dwc_csi_wait_recovering(csidev);

	/*
	 * NOTE (tracked as a follow-up, not fixed in this change): the IPI
	 * hardware config called from dwc_csi_start_stream() below still
	 * unconditionally configures/enables all DWC_CSI2RX_NUM_VC IPIs
	 * (mask=0xf) and, since it's only invoked here on the very first
	 * stream this device enables, will NOT reconfigure this VC's own IPI
	 * if a different VC was already streaming when this call happens.
	 * This change only fixes which pad/sensor-link this function itself
	 * talks to; it does not yet make the IPI-level bring-up per-VC.
	 */
	if (!csidev->enabled_streams) {
		ret = pm_runtime_resume_and_get(csidev->dev);
		if (ret < 0)
			goto err_in_flight;

		ret = v4l2_ctrl_handler_setup(&csidev->ctrl_handler);
		if (ret < 0)
			goto err_runtime_put;

		dwc_csi_clear_counters(csidev);

		ret = dwc_csi_start_stream(csidev, state, vc);
		if (ret < 0)
			goto err_runtime_put;

		started = true;
		dwc_csi_dump_regs(csidev);
		dwc_csi_log_counters(csidev);
	} else {
		/*
		 * Bring-up sized this VC's channel from whatever its sink pad held then.
		 * Give it its real geometry before it starts.
		 */
		dwc_csi_ipi_resize_vc(csidev, state, vc);
	}

	sink_streams = v4l2_subdev_state_xlate_streams(state, pad, sink_pad,
						       &streams_mask);

	dev_dbg(csidev->dev, "VC%u remote sd: %s pad: %u, sink_stream:0x%llx\n",
		vc, csidev->source_sd[vc]->name, csidev->remote_pad[vc], sink_streams);

	ret = v4l2_subdev_enable_streams(csidev->source_sd[vc], csidev->remote_pad[vc],
					 sink_streams);
	if (ret == -EALREADY && started) {
		/* Earlier failed start left the sensor stream enabled and
		 * driving the lanes: disable it, restart the receiver, retry. */
		dev_dbg(csidev->dev,
			"VC%u: stale enabled sensor stream (earlier failed start); recovering\n",
			vc);
		v4l2_subdev_disable_streams(csidev->source_sd[vc],
					    csidev->remote_pad[vc], sink_streams);
		cancel_work_sync(&csidev->work);
		csidev->poll_state = DWC_POLL_INACTIVE;
		dwc_csi_stop_stream(csidev);
		started = false;
		ret = dwc_csi_start_stream(csidev, state, vc);
		if (ret)
			goto err_runtime_put;
		started = true;
		ret = v4l2_subdev_enable_streams(csidev->source_sd[vc],
						 csidev->remote_pad[vc], sink_streams);
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
		v4l2_subdev_disable_streams(csidev->source_sd[vc],
					    csidev->remote_pad[vc], sink_streams);
		goto err_stop_stream;
	}

	csidev->enabled_streams |= streams_mask;
	WRITE_ONCE(csidev->start_check_ready, true);
	WRITE_ONCE(csidev->start_in_flight, false);

	return 0;

err_stop_stream:
	if (started) {
		dwc_csi_stop_stream(csidev);
		pm_runtime_put(csidev->dev);
	}
	WRITE_ONCE(csidev->start_in_flight, false);
	return ret;

err_runtime_put:
	pm_runtime_put(csidev->dev);
err_in_flight:
	WRITE_ONCE(csidev->start_in_flight, false);
	return ret;
}

static int dwc_csi_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	unsigned int vc, sink_pad;
	u64 sink_streams;
	int ret;

	vc = dwc_csi_pad_to_vc(pad);
	if (vc >= DWC_CSI2RX_NUM_VC || !csidev->source_sd[vc])
		return -EINVAL;

	sink_pad = DWC_CSI2RX_PAD_SINK + 2 * vc;

	sink_streams = v4l2_subdev_state_xlate_streams(state, pad, sink_pad,
						       &streams_mask);
	/* the start check must not re-init across a stop of another VC */
	WRITE_ONCE(csidev->start_in_flight, true);
	smp_mb();
	dwc_csi_wait_recovering(csidev);
	/* this VC leaves the window before its sensor stops */
	mutex_lock(&csidev->lock);
	csidev->start_check_mask &= ~BIT(vc);
	mutex_unlock(&csidev->lock);
	ret = v4l2_subdev_disable_streams(csidev->source_sd[vc], csidev->remote_pad[vc],
					  sink_streams);
	if (ret)
		goto out;
	csidev->enabled_streams &= ~streams_mask;
	if (!csidev->enabled_streams) {
		dwc_csi_stop_stream(csidev);
		dwc_csi_log_counters(csidev);
		pm_runtime_put(csidev->dev);
	}
out:
	WRITE_ONCE(csidev->start_in_flight, false);
	return ret;
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
	unsigned int vc;

	dev_dbg(csidev->dev, "link setup %s -> %s", remote_pad->entity->name,
		local_pad->entity->name);

	/* We only care about links to a sink pad. */
	if (!(local_pad->flags & MEDIA_PAD_FL_SINK))
		return 0;

	vc = dwc_csi_sink_pad_to_vc(local_pad->index);
	if (vc >= DWC_CSI2RX_NUM_VC)
		return -EINVAL;

	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);

	if (flags & MEDIA_LNK_FL_ENABLED) {
		/* Only this VC's own slot matters - a link already existing
		 * on a different VC must not block this one. */
		if (csidev->source_sd[vc])
			return -EBUSY;

		csidev->source_sd[vc] = remote_sd;
		csidev->remote_pad[vc] = remote_pad->index;
	} else {
		csidev->source_sd[vc] = NULL;
	}

	return 0;
}

static int dwc_csi_link_validate(struct media_link *link)
{
	struct media_pad *sink_pad = link->sink;
	struct v4l2_subdev *sink_sd;
	struct dwc_csi_device *csidev;
	int ret;

	sink_sd = media_entity_to_v4l2_subdev(sink_pad->entity);
	csidev = sd_to_dwc_csi_device(sink_sd);

	dev_dbg(csidev->dev, "link_validate: sink=%s:%d source=%s:%d\n",
		sink_sd->name, sink_pad->index,
		link->source->entity->name, link->source->index);

	/*
	 * Skip link validate when pattern enabled since the source
	 * data will be from CSI pattern generator, not sensor.
	 */
	if (csidev->pg_enable && sink_pad->index == DWC_CSI2RX_PAD_SINK)
		return 0;

	ret = v4l2_subdev_link_validate(link);
	if (ret)
		dev_err(csidev->dev,
			"link validation failed: %s:%u -> %s:%u: %d\n",
			link->source->entity->name, link->source->index,
			link->sink->entity->name, link->sink->index, ret);

	return ret;
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

/*
 * One of these is allocated per populated sink port (per VC) by
 * v4l2_async_nf_add_fwnode_remote(). It lets dwc_csi_notify_bound() recover
 * which VC a given .bound callback is for, since up to DWC_CSI2RX_NUM_VC of
 * these can be outstanding at once instead of just one.
 */
struct dwc_csi_asd {
	struct v4l2_async_connection base;
	unsigned int vc;
	struct fwnode_handle *ep_fwnode;
};

static int dwc_csi_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_connection *asd)
{
	struct dwc_csi_device *csidev = notifier_to_dwc_csi_device(notifier);
	struct dwc_csi_asd *dasd = container_of(asd, struct dwc_csi_asd, base);
	unsigned int vc = dasd->vc;
	struct media_pad *sink = &csidev->sd.entity.pads[DWC_CSI2RX_PAD_SINK + 2 * vc];
	u32 delay_ms;
	int ret;

	ret = media_entity_get_fwnode_pad(&sd->entity, dasd->ep_fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(csidev->dev, "Failed to find pad for %s (VC%u)\n",
			sd->name, vc);
		return ret;
	}

	csidev->source_sd[vc] = sd;
	csidev->remote_pad[vc] = ret;

	/*
	 * Allow the deserializer to override the delay applied before the IPI
	 * interface is enabled. Left untouched (default) if the property is
	 * absent or the source has no associated device node. This is a
	 * shared, link-level setting (not per-VC), so the first bound source
	 * to declare it wins; later VCs don't override it silently.
	 */
	if (sd->dev && !csidev->ipi_enable_delay_set) {
		if (!device_property_read_u32(sd->dev, "simaai,ipi-enable-delay-ms",
					      &delay_ms)) {
			csidev->ipi_enable_delay_ms =
				dwc_csi_clamp_ipi_enable_delay(csidev,
							       "legacy IPI enable delay",
							       delay_ms);
			csidev->ipi_enable_delay_set = true;
		}
	}

	ret = v4l2_create_fwnode_links_to_pad(sd, sink, MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(csidev->dev, "Failed to link pad with %s (VC%u)\n",
			sd->name, vc);
		return ret;
	}

	/*
	 * link_freq/D-PHY data rate is a property of the shared physical link,
	 * not per-VC, so it only needs to be established once. Seed it from
	 * whichever VC binds first; later VCs are assumed to share the same
	 * D-PHY and are not re-validated against it here.
	 */
	if (csidev->link_freq)
		return 0;


	/*
	 * Seed the link frequency now. Stream-on refreshes it from the source's
	 * negotiated ACTIVE format before D-PHY and IPI timing are programmed.
	 */
	return dwc_csi_update_link_freq(csidev, vc);
}

static const struct v4l2_async_notifier_operations dwc_csi_notify_ops = {
	.bound = dwc_csi_notify_bound,
};

static int dwc_csi_async_register(struct dwc_csi_device *csidev)
{
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct dwc_csi_asd *dasd;
	struct fwnode_handle *ep;
	struct fwnode_handle *rep;
	bool bus_parsed = false;
	unsigned int vc, i, found = 0;
	int ret;

	v4l2_async_subdev_nf_init(&csidev->notifier, &csidev->sd);

	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
		unsigned int sink_pad = DWC_CSI2RX_PAD_SINK + 2 * vc;

		/*
		 * Not every VC's sink port needs to be populated in the DT -
		 * a board may only wire up VC0, or VC0+VC1, etc. Skip
		 * whichever ports have no remote-endpoint instead of failing
		 * outright.
		 */
		ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(csidev->dev),
						     sink_pad, 0,
						     FWNODE_GRAPH_ENDPOINT_NEXT);
		if (!ep)
			continue;

		rep = fwnode_graph_get_remote_endpoint(ep);
		if (!rep) {
			dev_err(csidev->dev,
				"Failed to get remote endpoint on port %u\n",
				sink_pad);
			fwnode_handle_put(ep);
			ret = -ENOTCONN;
			goto err_cleanup;
		}

		/*
		 * Data-lane config (bus) is a property of the shared D-PHY,
		 * not per-VC - parse it once, from whichever populated port
		 * is found first.
		 */
		if (!bus_parsed) {
			ret = v4l2_fwnode_endpoint_parse(ep, &vep);
			if (ret) {
				fwnode_handle_put(rep);
				fwnode_handle_put(ep);
				goto err_cleanup;
			}

			for (i = 0; i < vep.bus.mipi_csi2.num_data_lanes; ++i) {
				if (vep.bus.mipi_csi2.data_lanes[i] != i + 1) {
					dev_err(csidev->dev,
						"data lanes reordering is not supported");
					ret = -EINVAL;
					fwnode_handle_put(rep);
					fwnode_handle_put(ep);
					goto err_cleanup;
				}
			}

			csidev->bus = vep.bus.mipi_csi2;

			dev_dbg(csidev->dev, "data lanes: %d\n", csidev->bus.num_data_lanes);
			dev_dbg(csidev->dev, "flags: 0x%08x\n", csidev->bus.flags);
			bus_parsed = true;
		}

		dasd = v4l2_async_nf_add_fwnode_remote(&csidev->notifier, ep,
						       struct dwc_csi_asd);
		if (IS_ERR(dasd)) {
			ret = PTR_ERR(dasd);
			fwnode_handle_put(rep);
			fwnode_handle_put(ep);
			goto err_cleanup;
		}

		dasd->vc = vc;
		dasd->ep_fwnode = rep;

		fwnode_handle_put(ep);
		found++;
	}

	if (!found) {
		ret = -ENOTCONN;
		goto err_cleanup;
	}

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
	return ret;

err_cleanup:
	v4l2_async_nf_cleanup(&csidev->notifier);
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

/* Master switch: 0 leaves overflows unrecovered so the raw error signature
 * stays visible on the bench. */
static bool ipi_ovf_recover_en = true;

/*
 * IPI-overflow recovery delays. The HW team's manual sequence used
 * sleep 20s / 100ms, but those were typing-speed artifacts — the true minimal
 * delays are unknown. Exposed as module params so they can be tuned down on the
 * bench without recompiling. ipi_ovf_reset_us is the delay while CSI+glue are
 * held in reset; ipi_ovf_post_us is the settle after each re-enable step.
 */
static unsigned int ipi_ovf_reset_us = 1000;
static unsigned int ipi_ovf_post_us = 100;

/* Test knob: when false, resets only the IPI/glue and SKIPS the DMA resume,
 * to check on-bench whether an IPI-only reset recovers the chain or the DMA
 * descriptor must be re-validated. Default true (full recovery). */
static bool ipi_ovf_dma_resume = true;

/* Minimum gap between recovery attempts: re-firing within a frame period
 * resets the IPI faster than it can re-sync and capture never converges. */
static unsigned int ipi_ovf_holdoff_ms = 100;

/* Clean-streaming window after which the recovery counter decays to 0 (a fresh
 * overflow after this long is a new incident, not part of a reset-storm). */
static unsigned int ipi_ovf_decay_ms = 10000;

/* Burst gate: recover only after ipi_ovf_burst_count fatal IRQs inside one
 * fixed ipi_ovf_burst_ms window (a lone fatal is not worth a pipe reset; every
 * measured overflow storm reaches 100 events in under 2 ms). <= 1 = legacy
 * single-event trigger. */
static unsigned int ipi_ovf_burst_count = 100;
static unsigned int ipi_ovf_burst_ms = 10;

/* Stall qualifier: a burst only triggers recovery if no good frame has been
 * delivered for this long. Storms with frames still landing (link/SI timing,
 * controller line margin) cannot be fixed by a reset — each one just costs the
 * in-flight frame. 0 = burst gate alone. */
static unsigned int ipi_ovf_stall_ms = 100;

/* Frame-relative stall threshold: the pipe only counts as stalled after
 * ipi_ovf_stall_frames x the last measured inter-frame gap without a good
 * frame (never below the ipi_ovf_stall_ms floor). At 10 fps the nominal gap
 * EQUALS the 100 ms floor, so a fixed threshold resets a flowing storm -
 * measured on the Metoak S315 depth stream: 8 recoveries and 5.8 of 10 fps.
 * 0 = fixed floor only. */
static unsigned int ipi_ovf_stall_frames = 3;

/* A confirmed storm masks the IPI-fatal IRQ at its source (a level condition
 * re-enters the handler at line rate — measured 99.7% of all CSI interrupts,
 * ~14.7k/s); the mask is re-armed after this long so the burst gate and stall
 * qualifier re-evaluate at a bounded rate. 0 = never mask (line-rate legacy;
 * required for meaningful ipi_ovf_burst_count <= 1 single-event A/B runs). */
static unsigned int ipi_ovf_rearm_ms = 100;

/* Bound CONSECUTIVE recovery attempts (within ipi_ovf_decay_ms of each other) so
 * a persistent fault can't reset-storm; the count decays after clean streaming. */
#define DWC_OVF_RECOVER_MAX	32

/*
 * Recover a controller-mode CSI IPI pixel overflow: reset the glue
 * + CSI IPI for instance 0, re-sync the VCID (Synopsys: program a wrong id then
 * the correct one), then ask the downstream DMA to resume (it owns the channel,
 * so it is invoked through the vdma-registered callback). Runs in process
 * context off the IRQ — must NOT be called from hardirq.
 */
static void dwc_csi_ovf_recover_work(struct work_struct *w)
{
	struct dwc_csi_device *csidev =
		container_of(w, struct dwc_csi_device, ovf_recover_work);
	unsigned int vc;

	mutex_lock(&csidev->lock);
	/*
	 * A running re-init re-enables everything itself. A start or stop is
	 * not waited for: its IPI programming runs under the lock too.
	 */
	if (!READ_ONCE(csidev->streaming) || READ_ONCE(csidev->recovering)) {
		mutex_unlock(&csidev->lock);
		return;
	}
	dwc_csi_set_recovering(csidev, true);

	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
		int (*quiesce)(void *data);
		int (*resume)(void *data);
		void *resume_data;
		u32 chan_ctrl, rstn, stat;
		int dma_ret = 0;
		bool do_dma;
		u32 dt;

		if (!test_and_clear_bit(vc, &csidev->ovf_pending_mask))
			continue;

		/* Holdoff: skipping is safe, a still-dead pipe re-raises the
		 * fatal IRQ (and the vdma watchdog re-triggers). Per-VC so one
		 * VC's persistent fault can't reset-storm, and can't mask (or
		 * be masked by) another VC's own recovery need. */
		if (csidev->ovf_recover_count[vc] &&
		    ktime_ms_delta(ktime_get(), csidev->ovf_last_recover_time[vc]) <
			    ipi_ovf_holdoff_ms)
			continue;

		quiesce = csidev->ovf_quiesce[vc];
		resume = csidev->ovf_resume[vc];
		resume_data = csidev->ovf_resume_data[vc];
		do_dma = quiesce && resume && ipi_ovf_dma_resume;

		/* Glue overflow count separates a starved pixel buffer
		 * (concurrency loss) from a stopped source. */
		stat = dwc_gluen_read(csidev, STAT_CTRL, vc);
		dev_dbg(csidev->dev,
			"VC%u: glue STAT_CTRL 0x%08x (oflow %lu) at recovery\n",
			vc, stat, FIELD_GET(GENMASK(23, 16), stat));
		dwc_gluen_write(csidev, STAT_CTRL, vc,
				DWC_CSI2GLUE_STAT_CTRL_RST_CNTS(0xff));
		dwc_gluen_write(csidev, STAT_CTRL, vc, 0);
		/* a channel just reset and re-parked is no sample for the start check */
		csidev->start_check_mask &= ~BIT(vc);

		/* 0: park the DMA before yanking its pixel source (resetting
		 * under a running block raises an async src bus error;
		 * resubmission races). If it cannot be parked, do not reset
		 * this VC at all: the watchdog re-triggers. */
		if (do_dma) {
			dma_ret = quiesce(resume_data);
			if (dma_ret) {
				dev_dbg_ratelimited(csidev->dev,
						    "VC%u: IPI overflow recovery skipped: DMA quiesce failed (%d)\n",
						    vc, dma_ret);
				continue;
			}
		}

		/* 1: gate the pipe - disable this VC's own glue channel and
		 * park its IPI on a reserved data type. Both stay gated until
		 * the DMA runs again: opening ingress against a quiesced DMA
		 * refills the FIFOs mid-reset. */
		chan_ctrl = dwc_gluen_read(csidev, CHAN_CTRL, vc);
		dwc_gluen_write(csidev, CHAN_CTRL, vc,
				chan_ctrl & ~DWC_CSI2GLUE_CHAN_CTRL_EN);
		/*
		 * 2: assert this VC's own IPI soft reset (active-low). Bits
		 * are nibble-spaced per the databook (IPI_SOFTRSTN: bit 0 =
		 * IPI1, bit 4 = IPI2, bit 8 = IPI3, bit 12 = IPI4) - NOT
		 * consecutive bits 0-3.
		 */
		rstn = dwc_csi_read(csidev, CSI2RX_IPI_SOFTRSTN);
		dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, rstn & ~BIT(vc * 4));
		/*
		 * 3: park the data type, not the VC id - with four cameras bound every
		 * id is live and an id park would land on a live stream.
		 */
		dwc_ipi_write(csidev, DATA_TYPE, vc,
			      CSI2RX_IPI_DATA_TYPE_DT(DWC_CSI2RX_DT_RESERVED));
		/* 4: hold in reset */
		usleep_range(ipi_ovf_reset_us, ipi_ovf_reset_us + 100);
		/* 5: deassert this VC's own IPI soft reset - out of reset,
		 * still gated. */
		dwc_csi_write(csidev, CSI2RX_IPI_SOFTRSTN, rstn | BIT(vc * 4));
		usleep_range(ipi_ovf_post_us, ipi_ovf_post_us + 50);

		csidev->ovf_recover_count[vc]++;
		csidev->ovf_last_recover_time[vc] = ktime_get();
		/* give this restarted pipe the same stall grace before another
		 * attempt is considered */
		WRITE_ONCE(csidev->ovf_last_frame[vc],
			   csidev->ovf_last_recover_time[vc]);

		/* 6: restart the parked DMA against the still-gated pipe. The
		 * resume callback belongs to the vdma driver and may sleep, so
		 * drop the lock across it as the single-VC path did. */
		mutex_unlock(&csidev->lock);
		if (do_dma)
			dma_ret = resume(resume_data);
		mutex_lock(&csidev->lock);

		/* 7: open the pipe downstream-up - glue consumer first, the
		 * real data type last. A failed resume leaves it gated (no
		 * storm while parked; the frame watchdog retries the whole
		 * recovery); EN is set explicitly so a retry cannot re-save
		 * the gated value as its restore state. */
		if (READ_ONCE(csidev->streaming) && !dma_ret) {
			dwc_gluen_write(csidev, CHAN_CTRL, vc,
					chan_ctrl | DWC_CSI2GLUE_CHAN_CTRL_EN);
			usleep_range(ipi_ovf_post_us, ipi_ovf_post_us + 50);
			dt = CSI2RX_IPI_DATA_TYPE_DT(csidev->csi_fmt[vc]->data_type);
			if (csidev->ipi_emb_data_en)
				dt |= CSI2RX_IPI_DATA_TYPE_EMB_DATA_EN;
			dwc_ipi_write(csidev, DATA_TYPE, vc, dt);
		}

		if (dma_ret)
			dev_dbg_ratelimited(csidev->dev,
					    "VC%u: IPI overflow recovery #%u: DMA resume failed (%d), pipe may stay stalled\n",
					    vc, csidev->ovf_recover_count[vc], dma_ret);
		else
			dev_dbg_ratelimited(csidev->dev,
					    "VC%u: IPI overflow recovery #%u (reset_us=%u post_us=%u)\n",
					    vc, csidev->ovf_recover_count[vc],
					    ipi_ovf_reset_us, ipi_ovf_post_us);
	}

	/* The burst gate is device-wide: one fatal storm is one incident. */
	csidev->ovf_burst_count = 0;
	dwc_csi_set_recovering(csidev, false);
	mutex_unlock(&csidev->lock);

	/* Re-arm the IPI-fatal mask if a confirmed storm turned it off - the
	 * FIFOs were just reset, and a masked source would blind the burst
	 * gate to an immediately recurring overflow. Runs through the re-arm
	 * work for the streaming gate; idempotent when never masked. */
	if (ipi_ovf_rearm_ms)
		mod_delayed_work(csidev->wq, &csidev->ovf_rearm_work, 0);
}

/* Re-arm the IPI-fatal interrupt a confirmed overflow storm masked. */
static void dwc_csi_ovf_rearm_work(struct work_struct *w)
{
	struct dwc_csi_device *csidev =
		container_of(to_delayed_work(w), struct dwc_csi_device,
			     ovf_rearm_work);

	mutex_lock(&csidev->lock);
	if (READ_ONCE(csidev->streaming) && READ_ONCE(csidev->recovering)) {
		/* a re-init or a recovery owns the masks for now */
		queue_delayed_work(csidev->wq, &csidev->ovf_rearm_work,
				   msecs_to_jiffies(50));
	} else if (READ_ONCE(csidev->streaming)) {
		unsigned int i;

		/*
		 * Only re-arm what the storm gate silenced, or a VC still storming
		 * would have its holdoff cut short.
		 */
		for (i = 0; i < DWC_CSI2RX_NUM_VC; i++)
			if (test_and_clear_bit(i, &csidev->ovf_masked_ipis))
				dwc_csi_write(csidev,
					      CSI2RX_INT_MSK_IPI_FATAL_N(i),
					      0x7f);
	}
	mutex_unlock(&csidev->lock);
}

/*
 * Register the downstream DMA resume callback. Called by the vdma capture driver
 * (which owns the dma_chan) at stream-on; pass NULL cb to unregister.
 */
int dwc_csi_register_overflow_recovery(struct v4l2_subdev *sd, unsigned int pad,
				       int (*quiesce)(void *data),
				       int (*resume)(void *data), void *data)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	unsigned int vc;

	vc = dwc_csi_pad_to_vc(pad);
	if (vc >= DWC_CSI2RX_NUM_VC)
		return -EINVAL;

	mutex_lock(&csidev->lock);
	csidev->ovf_quiesce[vc] = quiesce;
	csidev->ovf_resume[vc] = resume;
	csidev->ovf_resume_data[vc] = data;
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
void dwc_csi_trigger_overflow_recovery(struct v4l2_subdev *sd, unsigned int pad)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	unsigned int vc;

	vc = dwc_csi_pad_to_vc(pad);
	if (vc >= DWC_CSI2RX_NUM_VC)
		return;

	/*
	 * No DWC_OVF_RECOVER_MAX cap here (unlike the IRQ path): the caller (the
	 * vdma frame watchdog) is rate-limited by its own timer and bounded by
	 * frame_watchdog_max_retry, which supports an explicit infinite-retry mode.
	 */
	if (ipi_ovf_recover_en && READ_ONCE(csidev->streaming)) {
		set_bit(vc, &csidev->ovf_pending_mask);
		queue_work(csidev->wq, &csidev->ovf_recover_work);
	}
}
EXPORT_SYMBOL_GPL(dwc_csi_trigger_overflow_recovery);

/* Called by the vdma for every buffer it completes; the fatal-IRQ trigger
 * uses the good ones to tell a stalled pipe from a flowing one. */
void dwc_csi_frame_delivered(struct v4l2_subdev *sd, unsigned int pad, bool good)
{
	struct dwc_csi_device *csidev = sd_to_dwc_csi_device(sd);
	unsigned int vc = dwc_csi_pad_to_vc(pad);
	ktime_t now = ktime_get();
	s64 gap;

	if (vc >= DWC_CSI2RX_NUM_VC)
		return;

	if (!good) {
		WRITE_ONCE(csidev->bad_frames[vc],
			   READ_ONCE(csidev->bad_frames[vc]) + 1);
		return;
	}
	WRITE_ONCE(csidev->good_frames[vc],
		   READ_ONCE(csidev->good_frames[vc]) + 1);
	gap = ktime_ms_delta(now, READ_ONCE(csidev->ovf_last_frame[vc]));

	/* A storm that eats frames widens the measured gap and with it the
	 * stall threshold - the conservative direction. The sample follows
	 * *delivered* buffers, so a consumer holding its buffers stretches it
	 * beyond the sensor cadence; the 1 s bound keeps the worst threshold
	 * (3 s) inside the 5 s vdma watchdog, which stays the backstop. */
	WRITE_ONCE(csidev->ovf_frame_gap_ms[vc], (u32)clamp_t(s64, gap, 0, 1000));
	WRITE_ONCE(csidev->ovf_last_frame[vc], now);
}
EXPORT_SYMBOL_GPL(dwc_csi_frame_delivered);

/*
 * Re-initialise the receiver under a running source: park the DMA as
 * the overflow recovery does, repeat the PHY and host start sequence
 * (CSI2_RESETN keeps the IPI programming; only what dwc_csi_ipi_disable()
 * clears is restored), re-enable. The stop-state wait runs inline
 * because csidev->wq may be held by work blocked on csidev->lock; a
 * continuous-clock source times out at DWC_REINIT_STOPSTATE_US. Called
 * with csidev->lock held and `recovering` set; drops the lock around the
 * DMA callbacks.
 */
static int dwc_csi_device_reinit(struct dwc_csi_device *csidev)
{
	int (*quiesce[DWC_CSI2RX_NUM_VC])(void *data);
	int (*resume[DWC_CSI2RX_NUM_VC])(void *data);
	void *resume_data[DWC_CSI2RX_NUM_VC];
	u32 mode[DWC_CSI2RX_NUM_VC] = {};
	u32 chan[DWC_CSI2RX_NUM_VC] = {};
	u32 active = dwc_csi_active_ipi_mask(csidev);
	unsigned long parked = 0;
	bool stopped = false;
	unsigned int vc;
	int ret = 0;

	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
		quiesce[vc] = csidev->ovf_quiesce[vc];
		resume[vc] = csidev->ovf_resume[vc];
		resume_data[vc] = csidev->ovf_resume_data[vc];
		if (!(active & BIT(vc)) || !quiesce[vc] || !resume[vc])
			continue;
		mutex_unlock(&csidev->lock);
		ret = quiesce[vc](resume_data[vc]);
		mutex_lock(&csidev->lock);
		if (ret)
			goto resume;
		parked |= BIT(vc);
	}

	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
		if (!(active & BIT(vc)))
			continue;
		mode[vc] = dwc_ipi_read(csidev, MODE, vc) & ~CSI2RX_IPI_MODE_ENABLE;
		chan[vc] = dwc_gluen_read(csidev, CHAN_CTRL, vc);
	}

	dwc_csi_device_hs_rx_stop(csidev);
	stopped = true;
	ret = dwc_csi_device_startup(csidev);
	if (ret)
		goto resume;
	ret = dwc_csi_device_init_phy(csidev);
	if (ret)
		goto resume;
	/* a missed LP-11 is an observation, not a dead PHY: still re-enable */
	dwc_csi_wait_stopstate(csidev, DWC_REINIT_STOPSTATE_US);

resume:
	if (stopped) {
		for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
			if (!(active & BIT(vc)))
				continue;
			dwc_ipi_write(csidev, MODE, vc, mode[vc]);
			dwc_gluen_write(csidev, CHAN_CTRL, vc, chan[vc]);
		}
		if (csidev->phy_powered) {
			dwc_csi_device_hs_rx_start(csidev);
			/* teardown fatals are not an overflow of the new pipe */
			for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++)
				clear_bit(vc, &csidev->ovf_pending_mask);
		}
	}
	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
		if (!(parked & BIT(vc)))
			continue;
		mutex_unlock(&csidev->lock);
		if (resume[vc](resume_data[vc]) && !ret)
			ret = -EIO;
		mutex_lock(&csidev->lock);
	}
	return ret;
}

static void dwc_csi_start_check_work(struct work_struct *w)
{
	struct dwc_csi_device *csidev =
		container_of(w, struct dwc_csi_device, start_check_work.work);
	bool stalled = false;
	bool owned;
	unsigned int did, pkt, vc;
	u32 good = 0, bad = 0;
	const char *reason;
	int ret;

	mutex_lock(&csidev->lock);
	/* stop masked Data ID with the other groups before cancelling this work */
	if (!READ_ONCE(csidev->streaming))
		goto out;
	/*
	 * Not before enable_streams consumed its poll, not across another VC's
	 * start or stop, not over an overflow recovery: open a fresh window.
	 */
	owned = !READ_ONCE(csidev->recovering);
	if (owned)
		dwc_csi_set_recovering(csidev, true);
	/* pairs with enable_streams (recovering vs start_in_flight) */
	smp_mb();
	if (!owned || !READ_ONCE(csidev->start_check_ready) ||
	    READ_ONCE(csidev->start_in_flight)) {
		if (owned)
			dwc_csi_set_recovering(csidev, false);
		if (!start_check_ms) {
			dwc_csi_write(csidev, CSI2RX_INT_MSK_DATA_ID, 0);
			goto out;
		}
		dwc_csi_start_check_arm(csidev);
		goto out;
	}
	dwc_csi_write(csidev, CSI2RX_INT_MSK_DATA_ID, 0);
	did = READ_ONCE(csidev->events[csidev->start_check_did].counter) -
	      csidev->start_check_base;
	pkt = READ_ONCE(csidev->events[csidev->start_check_pkt].counter) -
	      csidev->start_check_pkt_base;
	csidev->start_check_last = did;
	csidev->start_check_last_pkt = pkt;
	/*
	 * The start that accepts frames and delivers none flags every buffer
	 * bad; both counts are vdma completions, so an idle consumer is not a
	 * stall. The readout keeps the stalled channel, else the worst ratio.
	 */
	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
		u32 g, b;

		if (!(csidev->start_check_mask & BIT(vc)) || !csidev->ovf_resume[vc])
			continue;
		g = READ_ONCE(csidev->good_frames[vc]) - csidev->start_check_good[vc];
		b = READ_ONCE(csidev->bad_frames[vc]) - csidev->start_check_bad[vc];
		if (!(g + b) || stalled)
			continue;
		if (g + b >= DWC_START_CHECK_MIN_FRAMES && g * 2 < g + b) {
			stalled = true;
			good = g;
			bad = b;
		} else if (!(good + bad) ||
			   (u64)g * (good + bad) < (u64)good * (g + b)) {
			good = g;
			bad = b;
		}
	}
	csidev->start_check_last_good = good;
	csidev->start_check_last_bad = bad;
	if (stalled)
		csidev->start_check_stalls++;
	reason = did >= start_check_data_id ? "data-id" :
		 pkt >= start_check_pkt_fatal ? "packet-fatal" :
		 stalled && start_check_stall ? "stall" : NULL;
	if (!reason || csidev->start_check_attempts >= start_check_retries) {
		dev_dbg(csidev->dev, "start check: %u Data ID, %u packet fatal, good %u bad %u in %u ms%s%s, attempt %u, keeping\n",
			did, pkt, good, bad, start_check_ms,
			reason ? ", " : "", reason ?: "",
			csidev->start_check_attempts);
		goto out_recovering;
	}
	csidev->start_check_attempts++;
	csidev->start_check_reinits++;
	csidev->start_check_trip = reason;
	csidev->start_check_trip_did = did;
	csidev->start_check_trip_pkt = pkt;
	csidev->start_check_trip_good = good;
	csidev->start_check_trip_bad = bad;
	ret = dwc_csi_device_reinit(csidev);
	if (ret)
		csidev->start_check_failed++;
	dev_dbg(csidev->dev, "start check: %u Data ID, %u packet fatal, good %u bad %u in %u ms, %s, re-init #%u: %d\n",
		did, pkt, good, bad, start_check_ms, reason,
		csidev->start_check_attempts, ret);
out_recovering:
	dwc_csi_set_recovering(csidev, false);
out:
	mutex_unlock(&csidev->lock);
}

/*
 * Error log rate limit. Named for CRC because that was the only error class
 * logged originally; it now gates every INT_ST_MAIN event, which matters most
 * for ECC/boundary/sequence errors since those fire at line rate on a
 * mistuned link.
 */
static unsigned int crc_ratelimit_interval_min = 1;
static unsigned int crc_ratelimit_burst = 10;

static irqreturn_t dwc_csi_irq_handler(int irq, void *priv)
{
	struct dwc_csi_device *csidev = priv;
	u32 status;
	int i;

	/* Hardware auto clean after read */
	status = dwc_csi_read(csidev, CSI2RX_INT_ST_MAIN);

	if (status & DWC_EVENT_MASK) {
		bool log;

		/*
		 * Re-read the live knob values every time, so writes to
		 * crc_ratelimit_* take effect immediately.
		 */
		csidev->crc_err_rs.interval =
			secs_to_jiffies(crc_ratelimit_interval_min * SECS_PER_MIN);
		csidev->crc_err_rs.burst = crc_ratelimit_burst;

		/*
		 * One ratelimit decision per interrupt rather than one per set
		 * bit: __ratelimit() takes a spinlock, and during an IPI
		 * overflow storm this handler runs thousands of times a second
		 * with several bits set on each pass.
		 */
		log = __ratelimit(&csidev->crc_err_rs);

		for (i = 0; i < DWC_NUM_EVENTS; ++i) {
			struct dwc_csi_event *event = &csidev->events[i];
			u32 sub = 0;

			if (!(status & event->mask))
				continue;

			event->counter++;

			/*
			 * Read the sub-status register: it carries the per-VC
			 * or per-cause detail and is the clear-on-read source
			 * behind this INT_ST_MAIN bit, so it also acknowledges.
			 * Always read it, even when the log line is suppressed.
			 */
			if (event->st_reg)
				sub = dwc_csi_read(csidev, event->st_reg);

			if (log)
				dev_dbg(csidev->dev,
					"%s (INT_ST_MAIN=0x%08x sub[0x%03x]=0x%08x) [total: %u]\n",
					event->name, status, event->st_reg,
					sub, event->counter);
		}
	}

	/*
	 * IPI pixel overflow (DMA out of descriptors): kick the
	 * deferred recovery once the fatals form a burst, per VC.
	 * queue_work() coalesces if one is already pending; the per-VC cap
	 * stops a persistent fault on one VC from reset-storming (and from
	 * being masked by, or masking, another VC's own recovery need). The
	 * recovery sequence is mode-agnostic, so gate on a registered resume
	 * callback rather than controller mode - the overflow also hits the
	 * standard v4l2-m2m path (12h soak).
	 *
	 * Each IPI has its own fatal-error bit in INT_ST_MAIN, consecutive
	 * from bit 18 (IPI1) through bit 21 (IPI4) - previously only bit 18
	 * (IPI1) was ever checked here, so a real overflow on IPI2/3/4 (i.e.
	 * any VC other than VC0) never reached this path at all.
	 *
	 * The burst gate is device-wide - one fatal storm is one incident -
	 * but the stall qualifier and the recovery cap are per VC, so a
	 * healthy VC cannot suppress a stalled one. The burst is counted
	 * outside the recovery gates: a confirmed storm is masked below even
	 * when recovery is off or unavailable.
	 */
	if ((status & CSI2RX_INT_ST_MAIN_FATAL_ERR_IPI_ANY) &&
	    READ_ONCE(csidev->streaming)) {
		ktime_t now = ktime_get();

		/* Burst gate: fixed window from the first fatal; a late event
		 * opens a new window rather than extending the old one. */
		if (!csidev->ovf_burst_count ||
		    ktime_ms_delta(now, csidev->ovf_burst_start) > ipi_ovf_burst_ms) {
			csidev->ovf_burst_start = now;
			csidev->ovf_burst_count = 1;
		} else {
			csidev->ovf_burst_count++;
		}
		if (csidev->ovf_burst_count >= ipi_ovf_burst_count) {
			csidev->ovf_burst_count = 0;

			if (ipi_ovf_recover_en) {
				unsigned int vc;
				bool any = false;

				for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
					unsigned int stall_ms = ipi_ovf_stall_ms;

					if (!(status & BIT(18 + vc)) ||
					    !csidev->ovf_resume[vc])
						continue;

					/* decay: clean streaming since the last
					 * recovery on this VC => fresh incident */
					if (csidev->ovf_recover_count[vc] &&
					    ktime_ms_delta(now, csidev->ovf_last_recover_time[vc]) >
						    ipi_ovf_decay_ms)
						csidev->ovf_recover_count[vc] = 0;

					/* frame-relative threshold, floor at
					 * stall_ms, from this VC's own cadence */
					if (ipi_ovf_stall_frames)
						stall_ms = max(stall_ms,
							       ipi_ovf_stall_frames *
							       READ_ONCE(csidev->ovf_frame_gap_ms[vc]));

					if (csidev->ovf_recover_count[vc] < DWC_OVF_RECOVER_MAX &&
					    (!ipi_ovf_stall_ms ||
					     ktime_ms_delta(now, READ_ONCE(csidev->ovf_last_frame[vc])) >=
						     stall_ms)) {
						set_bit(vc, &csidev->ovf_pending_mask);
						any = true;
					}
				}
				if (any)
					queue_work(csidev->wq, &csidev->ovf_recover_work);
			}

			/*
			 * The storm is confirmed and the decision above is
			 * taken; further events add no information. The fatal
			 * is a level condition that re-enters this handler at
			 * line rate for as long as the FIFO overflows, so mask
			 * it and let the delayed re-arm (or a completed
			 * recovery) restore it - the gate and qualifier then
			 * re-evaluate once per re-arm period instead of
			 * burning a hardirq per event.
			 *
			 * Mask only the IPIs in the burst, so a storm on one VC
			 * does not blind another VC's fatal.
			 */
			if (ipi_ovf_rearm_ms) {
				unsigned int i;
				bool masked = false;

				for (i = 0; i < DWC_CSI2RX_NUM_VC; i++) {
					if (!(status & BIT(18 + i)))
						continue;
					dwc_csi_write(csidev,
						      CSI2RX_INT_MSK_IPI_FATAL_N(i),
						      0);
					set_bit(i, &csidev->ovf_masked_ipis);
					masked = true;
				}
				if (masked)
					mod_delayed_work(csidev->wq,
							 &csidev->ovf_rearm_work,
							 msecs_to_jiffies(ipi_ovf_rearm_ms));
			}
		}
	}

	return IRQ_HANDLED;
}

static void dwc_csi_init_params(struct dwc_csi_device *csidev)
{
	unsigned int vc;

	for (vc = 0; vc < DWC_CSI2RX_NUM_VC; vc++) {
		csidev->csi_fmt[vc] = &dwc_csi_formats[0];
		csidev->source_vc[vc] = vc;
	}
	csidev->ipi_enable_delay_ms = DWC_CSI2RX_IPI_ENABLE_DELAY_MS;
	csidev->ipi_controller_enable_delay_set =
		!device_property_read_u32(csidev->dev,
					  "simaai,ipi-controller-enable-delay-ms",
					  &csidev->ipi_controller_enable_delay_ms);
	if (csidev->ipi_controller_enable_delay_set)
		csidev->ipi_controller_enable_delay_ms =
			dwc_csi_clamp_ipi_enable_delay(csidev,
						       "controller IPI enable delay",
						       csidev->ipi_controller_enable_delay_ms);
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
	bt->pixelclock = DWC_CSI2RX_DEF_PIXELCLOCK;

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

static void dwc_csi_debugfs_add_dev(struct dwc_csi_device *csidev);

static int dwc_csi_device_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc_csi_device *csidev;
	int irq;
	unsigned int i;
	int ret;

	csidev = devm_kzalloc(dev, sizeof(*csidev), GFP_KERNEL);
	if (!csidev)
		return -ENOMEM;

	mutex_init(&csidev->lock);
	init_waitqueue_head(&csidev->recover_wq);

	csidev->dev = dev;

	/* Pixel-path tunables start at the in-tree defaults, per instance.
	 * Registered after ->dev is set: the directory is named for it. */
	csidev->glue_src_burst_len = DWC_CSI2GLUE_SRC_BURST_LEN_DEF;
	csidev->glue_buf_full_th = DWC_CSI2GLUE_PIX_BUF_FULL_TH_DEF;
	csidev->ipi_cut_through = true;
	dwc_csi_debugfs_add_dev(csidev);

	memcpy(csidev->events, dwc_events, sizeof(dwc_events));

	/* Cap CRC error warnings to 10 per every 1min to avoid log flooding */
	ratelimit_state_init(&csidev->crc_err_rs,
			      secs_to_jiffies(crc_ratelimit_interval_min * SECS_PER_MIN),
			      crc_ratelimit_burst);
	/* The gated print is dev_dbg; the limiter itself must not print either. */
	ratelimit_set_flags(&csidev->crc_err_rs, RATELIMIT_MSG_ON_RELEASE);

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

	dwc_csi_init_params(csidev);

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

	dwc_csi_dv_timing_init(csidev);
	INIT_WORK(&csidev->work, stopstate_poll);
	INIT_DELAYED_WORK(&csidev->ipi_work, dwc_csi_delayed_ipi_enable);
	INIT_WORK(&csidev->ovf_recover_work, dwc_csi_ovf_recover_work);
	INIT_DELAYED_WORK(&csidev->ovf_rearm_work, dwc_csi_ovf_rearm_work);
	INIT_DELAYED_WORK(&csidev->start_check_work, dwc_csi_start_check_work);
	csidev->start_check_did = DWC_NUM_EVENTS;
	csidev->start_check_pkt = DWC_NUM_EVENTS;
	for (i = 0; i < DWC_NUM_EVENTS; i++) {
		if (csidev->events[i].mask == CSI2RX_INT_ST_MAIN_ERR_DID)
			csidev->start_check_did = i;
		if (csidev->events[i].mask == CSI2RX_INT_ST_MAIN_FATAL_ERR_PKT)
			csidev->start_check_pkt = i;
	}
	if (csidev->start_check_did == DWC_NUM_EVENTS ||
	    csidev->start_check_pkt == DWC_NUM_EVENTS)
		dev_dbg(dev, "start check disabled: event rows missing\n");
	csidev->poll_state = DWC_POLL_INACTIVE;

	/* Private high-priority WQ; the stop-state poll must run promptly. */
	csidev->wq = alloc_ordered_workqueue("%s-csi", WQ_HIGHPRI, dev_name(dev));
	if (!csidev->wq)
		return dev_err_probe(dev, -ENOMEM, "Failed to allocate WQ\n");

	/* Publish the subdevice only once every asynchronous resource is ready. */
	pm_runtime_enable(dev);
	ret = dwc_csi_async_register(csidev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Async register failed\n");
		goto err_pm_disable;
	}

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	destroy_workqueue(csidev->wq);
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
	WRITE_ONCE(csidev->streaming, false);
	cancel_delayed_work_sync(&csidev->start_check_work);
	cancel_delayed_work_sync(&csidev->ipi_work);
	cancel_work_sync(&csidev->ovf_recover_work);
	cancel_delayed_work_sync(&csidev->ovf_rearm_work);
	destroy_workqueue(csidev->wq);

	pm_runtime_disable(&pdev->dev);

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

/* Signed knob (-1 = use DT); debugfs has no s32 helper. */
static int ipi_delay_override_get(void *data, u64 *val)
{
	*val = *(int *)data;
	return 0;
}

static int ipi_delay_override_set(void *data, u64 val)
{
	*(int *)data = (int)val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ipi_delay_override_fops, ipi_delay_override_get,
			 ipi_delay_override_set, "%lld\n");

static struct dentry *dwc_csi_debugfs_dir;

/* debug knobs; absent on production (!SIMAAI_CAMERA_INSTRUMENTATION) builds */
static void dwc_csi_debugfs_init(void)
{
	if (!IS_ENABLED(CONFIG_SIMAAI_CAMERA_INSTRUMENTATION))
		return;

	dwc_csi_debugfs_dir = debugfs_create_dir("dwc-mipi-csi2", NULL);
	debugfs_create_bool("ipi_ovf_recover_en", 0644, dwc_csi_debugfs_dir,
			    &ipi_ovf_recover_en);
	debugfs_create_u32("ipi_ovf_reset_us", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_reset_us);
	debugfs_create_u32("ipi_ovf_post_us", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_post_us);
	debugfs_create_bool("ipi_ovf_dma_resume", 0644, dwc_csi_debugfs_dir,
			    &ipi_ovf_dma_resume);
	debugfs_create_u32("ipi_ovf_holdoff_ms", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_holdoff_ms);
	debugfs_create_u32("ipi_ovf_decay_ms", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_decay_ms);
	debugfs_create_u32("ipi_ovf_burst_count", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_burst_count);
	debugfs_create_u32("ipi_ovf_burst_ms", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_burst_ms);
	debugfs_create_u32("ipi_ovf_stall_ms", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_stall_ms);
	debugfs_create_u32("ipi_ovf_rearm_ms", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_rearm_ms);
	debugfs_create_u32("ipi_ovf_stall_frames", 0644, dwc_csi_debugfs_dir,
			   &ipi_ovf_stall_frames);
	debugfs_create_u32("start_check_ms", 0644, dwc_csi_debugfs_dir,
			   &start_check_ms);
	debugfs_create_u32("start_check_data_id", 0644, dwc_csi_debugfs_dir,
			   &start_check_data_id);
	debugfs_create_u32("start_check_pkt_fatal", 0644, dwc_csi_debugfs_dir,
			   &start_check_pkt_fatal);
	debugfs_create_bool("start_check_stall", 0644, dwc_csi_debugfs_dir,
			    &start_check_stall);
	debugfs_create_u32("start_check_retries", 0644, dwc_csi_debugfs_dir,
			   &start_check_retries);
	debugfs_create_file_unsafe("ipi_enable_delay_override_ms", 0644,
				   dwc_csi_debugfs_dir,
				   &ipi_enable_delay_override_ms,
				   &ipi_delay_override_fops);
	debugfs_create_u32("ipi_line_margin_cy", 0644, dwc_csi_debugfs_dir,
			   &ipi_line_margin_cy);
	debugfs_create_bool("ipi_hline_from_sensor", 0644, dwc_csi_debugfs_dir,
			    &ipi_hline_from_sensor);
	debugfs_create_u32("crc_ratelimit_interval_min", 0644, dwc_csi_debugfs_dir,
			   &crc_ratelimit_interval_min);
	debugfs_create_u32("crc_ratelimit_burst", 0644, dwc_csi_debugfs_dir,
			   &crc_ratelimit_burst);
}

/* per-instance tunable accessors, value read back from csidev */
#define DWC_CSI_TUNABLE_FOPS(name_, field_)				\
static int name_##_get(void *data, u64 *val)				\
{									\
	struct dwc_csi_device *csidev = data;				\
									\
	mutex_lock(&csidev->lock);					\
	*val = csidev->field_;						\
	mutex_unlock(&csidev->lock);					\
	return 0;							\
}									\
static int name_##_set(void *data, u64 val)				\
{									\
	struct dwc_csi_device *csidev = data;				\
									\
	mutex_lock(&csidev->lock);					\
	csidev->field_ = val;						\
	mutex_unlock(&csidev->lock);					\
	return 0;							\
}									\
DEFINE_DEBUGFS_ATTRIBUTE(name_##_fops, name_##_get, name_##_set, "%llu\n")

/*
 * Rejects a value above @max_: both glue fields reach a 7-bit register
 * through FIELD_PREP(), which truncates, on a control that can collapse
 * the link at the wrong value.
 */
#define DWC_CSI_TUNABLE_FOPS_RANGED(name_, field_, max_)		\
static int name_##_get(void *data, u64 *val)				\
{									\
	struct dwc_csi_device *csidev = data;				\
									\
	mutex_lock(&csidev->lock);					\
	*val = csidev->field_;						\
	mutex_unlock(&csidev->lock);					\
	return 0;							\
}									\
static int name_##_set(void *data, u64 val)				\
{									\
	struct dwc_csi_device *csidev = data;				\
									\
	if (val > (max_))						\
		return -EINVAL;						\
									\
	mutex_lock(&csidev->lock);					\
	csidev->field_ = val;						\
	mutex_unlock(&csidev->lock);					\
	return 0;							\
}									\
DEFINE_DEBUGFS_ATTRIBUTE(name_##_fops, name_##_get, name_##_set, "%llu\n")

DWC_CSI_TUNABLE_FOPS_RANGED(glue_src_burst_len, glue_src_burst_len,
			    GENMASK(6, 0));
DWC_CSI_TUNABLE_FOPS_RANGED(glue_buf_full_th, glue_buf_full_th,
			    GENMASK(6, 0));

static void dwc_csi_debugfs_del_dev(void *data)
{
	struct dwc_csi_device *csidev = data;

	debugfs_remove_recursive(csidev->dbg_dir);
	csidev->dbg_dir = NULL;
}

static int dwc_csi_events_show(struct seq_file *seq, void *unused)
{
	struct dwc_csi_device *csidev = seq->private;
	unsigned int i;

	for (i = 0; i < DWC_NUM_EVENTS; i++)
		seq_printf(seq, "%s %u\n", csidev->events[i].name,
			   READ_ONCE(csidev->events[i].counter));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dwc_csi_events);

/* per-stream re-init counts, the last window, and the window that tripped */
static int dwc_csi_start_check_show(struct seq_file *seq, void *unused)
{
	struct dwc_csi_device *csidev = seq->private;

	mutex_lock(&csidev->lock);
	seq_printf(seq, "reinits %u failed %u stalls %u window data_id %u packet_fatal %u good %u bad %u trip %s data_id %u packet_fatal %u good %u bad %u\n",
		   csidev->start_check_reinits, csidev->start_check_failed,
		   csidev->start_check_stalls, csidev->start_check_last,
		   csidev->start_check_last_pkt, csidev->start_check_last_good,
		   csidev->start_check_last_bad,
		   csidev->start_check_trip ?: "none",
		   csidev->start_check_trip_did, csidev->start_check_trip_pkt,
		   csidev->start_check_trip_good, csidev->start_check_trip_bad);
	mutex_unlock(&csidev->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dwc_csi_start_check);

static void dwc_csi_debugfs_add_dev(struct dwc_csi_device *csidev)
{
	if (!IS_ENABLED(CONFIG_SIMAAI_CAMERA_INSTRUMENTATION) ||
	    !dwc_csi_debugfs_dir)
		return;

	csidev->dbg_dir = debugfs_create_dir(dev_name(csidev->dev),
					     dwc_csi_debugfs_dir);
	/* readers hold csidev (devm): the directory must not outlive it */
	if (devm_add_action_or_reset(csidev->dev, dwc_csi_debugfs_del_dev,
				     csidev))
		return;

	debugfs_create_file_unsafe("glue_src_burst_len", 0644, csidev->dbg_dir,
				   csidev, &glue_src_burst_len_fops);
	debugfs_create_file_unsafe("glue_buf_full_th", 0644, csidev->dbg_dir,
				   csidev, &glue_buf_full_th_fops);
	debugfs_create_file("events", 0444, csidev->dbg_dir, csidev,
			    &dwc_csi_events_fops);
	debugfs_create_bool("ipi_cut_through", 0644, csidev->dbg_dir,
			    &csidev->ipi_cut_through);
	debugfs_create_file("start_check", 0444, csidev->dbg_dir, csidev,
			    &dwc_csi_start_check_fops);
}

static int __init dwc_csi_device_driver_init(void)
{
	int ret;

	/* the parent first: registering the driver probes every device */
	dwc_csi_debugfs_init();

	ret = platform_driver_register(&dwc_csi_device_driver);
	if (ret)
		debugfs_remove_recursive(dwc_csi_debugfs_dir);
	return ret;
}
module_init(dwc_csi_device_driver_init);

static void __exit dwc_csi_device_driver_exit(void)
{
	platform_driver_unregister(&dwc_csi_device_driver);
	debugfs_remove_recursive(dwc_csi_debugfs_dir);
}
module_exit(dwc_csi_device_driver_exit);

MODULE_DESCRIPTION("DesignWare Core MIPI CSI2 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: dwc-mipi-csi2");
MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_AUTHOR("SiMa Technologies, Inc.");
