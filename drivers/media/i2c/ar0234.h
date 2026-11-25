/* SPDX-License-Identifier: GPL-2.0 */
/*
 * onsemi AR0234 register map (AND9812 names) and limits (AND9820).
 * Copyright 2026 SiMa Technologies, Inc.
 */

#ifndef AR0234_H
#define AR0234_H

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/limits.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-cci.h>

/* Identification */
#define AR0234_MODEL_ID				CCI_REG16(0x3000)
#define AR0234_MODEL_ID_COLOR			0x0a56
#define AR0234_MODEL_ID_MONO			0x1a56

/* Readout window and frame timing, context A */
#define AR0234_Y_ADDR_START			CCI_REG16(0x3002)
#define AR0234_X_ADDR_START			CCI_REG16(0x3004)
#define AR0234_Y_ADDR_END			CCI_REG16(0x3006)
#define AR0234_X_ADDR_END			CCI_REG16(0x3008)
#define AR0234_FRAME_LENGTH_LINES		CCI_REG16(0x300a)
#define AR0234_FRAME_LENGTH_LINES_MAX		U16_MAX
#define AR0234_LINE_LENGTH_PCK			CCI_REG16(0x300c)
/* AND9820: the shortest line at every width; even values only. */
#define AR0234_LINE_LENGTH_PCK_MIN		612
#define AR0234_COARSE_INTEGRATION_TIME		CCI_REG16(0x3012)

/* RESET_REGISTER; bits 13 and 4 read back set and are written back as such. */
#define AR0234_RESET_REGISTER			CCI_REG16(0x301a)
#define AR0234_RESET_REGISTER_RSVD_SET		(BIT(13) | BIT(4))
#define AR0234_RESET_REGISTER_DRIVE_PINS	BIT(6)
#define AR0234_RESET_REGISTER_LOCK_REG		BIT(3)
#define AR0234_RESET_REGISTER_STREAM		BIT(2)
#define AR0234_STANDBY \
	(AR0234_RESET_REGISTER_RSVD_SET | AR0234_RESET_REGISTER_DRIVE_PINS | \
	 AR0234_RESET_REGISTER_LOCK_REG)
#define AR0234_STREAMING	(AR0234_STANDBY | AR0234_RESET_REGISTER_STREAM)

/* PLL */
#define AR0234_VT_PIX_CLK_DIV			CCI_REG16(0x302a)
#define AR0234_VT_SYS_CLK_DIV			CCI_REG16(0x302c)
#define AR0234_PRE_PLL_CLK_DIV			CCI_REG16(0x302e)
#define AR0234_PLL_MULTIPLIER			CCI_REG16(0x3030)
#define AR0234_OP_PIX_CLK_DIV			CCI_REG16(0x3036)
#define AR0234_OP_SYS_CLK_DIV			CCI_REG16(0x3038)

/* READ_MODE */
#define AR0234_READ_MODE			CCI_REG16(0x3040)
#define AR0234_READ_MODE_VERT_FLIP		BIT(15)
#define AR0234_READ_MODE_HORIZ_MIRROR		BIT(14)

/* GLOBAL_GAIN is xxxx.yyyyyyy (0x80 = 1x); ANALOG_GAIN is 2^[6:4] plus a fine step in [3:0]. */
#define AR0234_GLOBAL_GAIN			CCI_REG16(0x305e)
#define AR0234_ANALOG_GAIN			CCI_REG16(0x3060)

/* SMIA_TEST */
#define AR0234_SMIA_TEST			CCI_REG16(0x3064)
#define AR0234_SMIA_TEST_EMBEDDED_DATA		BIT(8)

/* DATAPATH_SELECT: output slew rates; bit 4 reads back set. */
#define AR0234_DATAPATH_SELECT			CCI_REG16(0x306e)
#define AR0234_DATAPATH_SLEW_RATE_CTRL_PARALLEL	GENMASK(15, 13)
#define AR0234_DATAPATH_SLEW_RATE_CTRL_PIXCLK	GENMASK(12, 10)
#define AR0234_DATAPATH_SELECT_RSVD_SET		BIT(4)
/* The LI slew setting */
#define AR0234_DATAPATH_SELECT_VAL \
	(FIELD_PREP_CONST(AR0234_DATAPATH_SLEW_RATE_CTRL_PARALLEL, 4) | \
	 FIELD_PREP_CONST(AR0234_DATAPATH_SLEW_RATE_CTRL_PIXCLK, 4) | \
	 AR0234_DATAPATH_SELECT_RSVD_SET)

/* Test pattern */
#define AR0234_TEST_PATTERN_MODE		CCI_REG16(0x3070)
#define AR0234_TEST_PATTERN_NONE		0
#define AR0234_TEST_PATTERN_SOLID_COLOR		1
#define AR0234_TEST_PATTERN_COLOR_BAR		2
#define AR0234_TEST_PATTERN_FADE_TO_GRAY	3
#define AR0234_TEST_PATTERN_WALKING_1		256
#define AR0234_TEST_DATA_RED			CCI_REG16(0x3072)
#define AR0234_TEST_DATA_GREENR			CCI_REG16(0x3074)
#define AR0234_TEST_DATA_BLUE			CCI_REG16(0x3076)
#define AR0234_TEST_DATA_GREENB			CCI_REG16(0x3078)
#define AR0234_TEST_DATA_MAX			GENMASK(9, 0)	/* 10 bits at any depth */

/* OPERATION_MODE_CTRL */
#define AR0234_OPERATION_MODE_CTRL		CCI_REG16(0x3082)
#define AR0234_OPERATION_MODE_LINEAR		3

/* Skipping */
#define AR0234_X_ODD_INC			CCI_REG16(0x30a2)
#define AR0234_Y_ODD_INC			CCI_REG16(0x30a6)
#define AR0234_ODD_INC_NO_SKIP			1

/* Pixel depth in and out of the data path; out below in truncates. */
#define AR0234_DATA_FORMAT_BITS			CCI_REG16(0x31ac)
#define AR0234_DATA_FORMAT_IN			GENMASK(12, 8)
#define AR0234_DATA_FORMAT_OUT			GENMASK(4, 0)
#define AR0234_DATA_FORMAT(in, out) \
	(FIELD_PREP_CONST(AR0234_DATA_FORMAT_IN, in) | \
	 FIELD_PREP_CONST(AR0234_DATA_FORMAT_OUT, out))

/* SERIAL_FORMAT: the interface type byte is read-only, the low byte is the lane count. */
#define AR0234_SERIAL_FORMAT			CCI_REG16(0x31ae)
#define AR0234_SERIAL_FORMAT_TYPE_MIPI		0x0200
#define AR0234_SERIAL_FORMAT_LANES		GENMASK(7, 0)

/* D-PHY timing and preambles, in op_pix_clk periods */
#define AR0234_FRAME_PREAMBLE			CCI_REG16(0x31b0)
#define AR0234_LINE_PREAMBLE			CCI_REG16(0x31b2)
#define AR0234_PREAMBLE_MASK			GENMASK(7, 0)
#define AR0234_MIPI_TIMING_0			CCI_REG16(0x31b4)
#define AR0234_T0_HS_PREPARE_MASK		GENMASK(15, 12)
#define AR0234_T0_HS_TRAIL_MASK			GENMASK(10, 6)
#define AR0234_T0_CLK_TRAIL_MASK		GENMASK(4, 0)
#define AR0234_MIPI_TIMING_1			CCI_REG16(0x31b6)
#define AR0234_T1_CLK_PREPARE_MASK		GENMASK(15, 12)
#define AR0234_T1_HS_EXIT_MASK			GENMASK(11, 6)
#define AR0234_T1_HS_ZERO_MASK			GENMASK(5, 0)
#define AR0234_MIPI_TIMING_2			CCI_REG16(0x31b8)
#define AR0234_T2_BGAP_MASK			GENMASK(15, 12)
#define AR0234_T2_CLK_PRE_MASK			GENMASK(11, 6)
#define AR0234_T2_CLK_POST_MASK			GENMASK(5, 0)
#define AR0234_MIPI_TIMING_3			CCI_REG16(0x31ba)
#define AR0234_T3_LPX_MASK			GENMASK(12, 7)
#define AR0234_T3_ULPM_RECOVER_MASK		GENMASK(6, 0)
#define AR0234_MIPI_TIMING_4			CCI_REG16(0x31bc)
#define AR0234_T4_CONT_TX_CLK			BIT(15)
#define AR0234_T4_RSVD_MASK			GENMASK(13, 7)
#define AR0234_T4_RSVD_VAL			0x10
#define AR0234_T4_T_INIT_MASK			GENMASK(6, 0)

/* COMPANDING: bit 0 enables DPCM; cleared, a narrower DATA_FORMAT_OUT truncates. */
#define AR0234_COMPANDING			CCI_REG16(0x31d0)

/* MIPI_CNTRL: the emitted data type must follow the pixel depth. */
#define AR0234_MIPI_CNTRL			CCI_REG16(0x3354)
#define AR0234_MIPI_CNTRL_DATA_TYPE		GENMASK(5, 0)

/* AND9820 Figure 1: 1940 x 1220 active pixels; X_ADDR 8..1927, Y_ADDR 8..1207 readable. */
#define AR0234_NATIVE_WIDTH			1940U
#define AR0234_NATIVE_HEIGHT			1220U
#define AR0234_PIXEL_ARRAY_LEFT			8U
#define AR0234_PIXEL_ARRAY_TOP			8U
#define AR0234_PIXEL_ARRAY_WIDTH		1920U
#define AR0234_PIXEL_ARRAY_HEIGHT		1200U

/* Required external clock */
#define AR0234_XCLK_FREQ			27000000ULL

/* AND9820 Table 13; FVCO = XCLK / PRE_PLL_CLK_DIV * PLL_MULTIPLIER, the multiplier even. */
#define AR0234_FVCO_MIN_HZ			384000000ULL
#define AR0234_FVCO_MAX_HZ			768000000ULL
#define AR0234_FSERIAL_MIN_BPS			360000000ULL
#define AR0234_FSERIAL_MAX_BPS			900000000ULL
#define AR0234_PLL_MULTIPLIER_MAX		0xff
/* AND9820 Table 13: the output pixel clock may not exceed 90 MHz; PIXCLK equals it here. */
#define AR0234_OP_CLK_MAX_HZ			90000000ULL
/* Link frequencies an endpoint may list */
#define AR0234_LINK_FREQS_MAX			4
/* D-PHY reference counts are the LI values at this OP_CLK: a 270 MHz link at 12-bit. */
#define AR0234_DPHY_REF_OP_CLK_HZ		45000000ULL
/*
 * AND9812 reset preambles, set for the power-on PLL's 90 MHz OP_CLK; the LI preambles
 * overran the blanking of short lines.
 */
#define AR0234_FRAME_PREAMBLE_DEF		112
#define AR0234_LINE_PREAMBLE_DEF		81
#define AR0234_PREAMBLE_REF_OP_CLK_HZ		90000000ULL
/* The PLL relocks after a multiplier or divider change and NAKs writes meanwhile. */
#define AR0234_PLL_RELOCK_DELAY_US		10000

/* Power sequencing */
#define AR0234_XCLR_MIN_DELAY_US		6200
#define AR0234_XCLR_DELAY_RANGE_US		1000

/* V4L2 control limits */
#define AR0234_FRAME_LENGTH_LINES_DEF		1220
#define AR0234_VBLANK_MIN			16
#define AR0234_EXPOSURE_MIN			4
#define AR0234_EXPOSURE_STEP			1
#define AR0234_EXPOSURE_DEFAULT			367
/* Rows kept between the longest exposure and the frame length */
#define AR0234_COARSE_MARGIN			4
#define AR0234_EXPOSURE_MAX_FOR(vts)		((vts) - AR0234_COARSE_MARGIN)
/* AND9820: coarse integration may not be FLL - 9..FLL - 7. */
#define AR0234_COARSE_BAND_FIRST		9
#define AR0234_COARSE_BAND_LAST			7
#define AR0234_COARSE_BAND_ROWS			(AR0234_COARSE_BAND_FIRST - \
						 AR0234_COARSE_BAND_LAST + 1)
/* The 7-bit context-A field is monotonic 1x..16x over 0..0x40. */
#define AR0234_ANA_GAIN_MIN			0
#define AR0234_ANA_GAIN_MAX			0x40
#define AR0234_ANA_GAIN_STEP			1
#define AR0234_ANA_GAIN_DEFAULT			0x0
#define AR0234_DGTL_GAIN_MIN			0x0080
#define AR0234_DGTL_GAIN_MAX			0x07ff
#define AR0234_DGTL_GAIN_DEFAULT		0x0080
#define AR0234_DGTL_GAIN_STEP			1
#define AR0234_TESTP_COLOUR_MIN			0
#define AR0234_TESTP_COLOUR_MAX			AR0234_TEST_DATA_MAX
#define AR0234_TESTP_COLOUR_STEP		1

/* Output depth; AND9820 Table 14 dividers: op = depth, vt = depth / 2, so PIXCLK = OP_CLK. */
struct ar0234_depth {
	u32 code;
	u8 vt_pix_clk_div;
	u8 op_pix_clk_div;
	u16 data_format_bits;
	u8 data_type;
};

/* PLL multiplier and dividers as written to the sensor */
struct ar0234_pll {
	u8 pre_pll_clk_div;
	u8 pll_multiplier;
	u8 vt_sys_clk_div;
	u8 vt_pix_clk_div;
	u8 op_sys_clk_div;
	u8 op_pix_clk_div;
};

/* Readout window in X_ADDR/Y_ADDR coordinates and its default frame length */
struct ar0234_mode {
	unsigned int width;
	unsigned int height;
	struct v4l2_rect crop;
	unsigned int fll_def;
};

struct ar0234_dphy_timing {
	u8 hs_prepare, hs_trail, clk_trail;
	u8 clk_prepare, hs_exit, hs_zero;
	u8 bgap, clk_pre, clk_post;
	u8 lpx, ulpm_recover;
	u8 t_init;
};

#endif /* AR0234_H */
