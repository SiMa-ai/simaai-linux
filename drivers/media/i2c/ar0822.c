// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for the ON Semiconductor AR0822 image sensor.
 *
 * Copyright (C) 2020 Leopard Imaging.
 *
 * Based on the Sony IMX219/IMX415 camera drivers.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include <linux/media/simaai/modalix_sensor_v4l2.h>

/* Chip ID */
#define AR0822_REG_CHIP_ID		CCI_REG16(0x3000)
#define AR0822_CHIP_ID			0x0f56
#define AR0822_REG_REVISION		CCI_REG16(0x300e)
/* OTPM rev 3 parts turn every multi-exposure row into an 8-px packet. */
#define AR0822_REVISION_NO_EHDR		0x2303

#define AR0822_REG_MODE_SELECT		CCI_REG16(0x301a)
#define AR0822_MODE_STOP_STREAMING	0x0000
#define AR0822_MODE_START_STREAMING	0x001c
#define AR0822_MODE_SW_RESET		0x0001

#define AR0822_XCLK_FREQ		27000000

/*
 * VCO 1971 MHz / OP_SYS_CLK_DIV. Both rates move OP_SYS_CLK_DIV alone, so
 * PRE_PLL_CLK_DIV, PLL_MULTIPLIER, VT_SYS_CLK_DIV and VT_PIX_CLK_DIV stay put
 * and the 164.25 MHz pixel clock is the same at either; ar0822_line_length()
 * reads the rate in use, so the line grows to carry the same pixels over the
 * slower link.
 *
 * The endpoint chooses. 985.5 Mbps per lane is AND90149-D Table 6's 985 column;
 * a carrier whose CSI routing cannot reach start-of-transmission at it declares
 * the halved rate instead, which is the difference between streaming and no
 * frames at all. The halved rate is not itself a Table 6 example: it is that
 * PLL with the divider doubled, which Table 5 permits and which stays inside
 * the 384 to 1971 Mbps Fserial range.
 */
#define AR0822_LINK_FREQ_985MBPS	492750000	/* OP_SYS_CLK_DIV 4 */
#define AR0822_LINK_FREQ_492MBPS	246375000	/* OP_SYS_CLK_DIV 8 */

/*
 * Pixel rate is fixed for all the modes. EXTCLK 27 MHz (the module's onboard
 * oscillator) / PRE_PLL_CLK_DIV 2 * PLL_MULTIPLIER 146 = 1971 MHz VCO, then
 * / (VT_SYS_CLK_DIV 2 * VT_PIX_CLK_DIV 6). The same VCO / OP_SYS_CLK_DIV gives
 * the link frequency the endpoint declares: 492.75 MHz at a divider of 4,
 * 246.375 MHz at 8.
 */
/*
 * vt_pix_clk: LINE_LENGTH_PCK and FRAME_LENGTH_LINES are counted in it, so all
 * the frame timing is. It is not the rate pixels leave the sensor - the readout
 * emits more than one per tick - so it is not what V4L2_CID_PIXEL_RATE reports.
 */
#define AR0822_VT_PIX_CLK		164250000

/* V_TIMING internal */
#define AR0822_REG_FRAME_LENGTH_LINES	CCI_REG16(0x300A)
#define AR0822_FRAME_LENGTH_MAX		0xffdc

/* H_TIMING internal */
#define AR0822_REG_LINE_LENGTH_PCK	CCI_REG16(0x300C)

/* Exposure control */
#define AR0822_REG_EXPOSURE		CCI_REG16(0x3012)

/*
 * SERIAL_FORMAT: [9:8] interface type (2 = MIPI), [2:0] active lane count.
 * The lane count comes from the endpoint, so one image serves a 2-lane
 * carrier and a 4-lane one.
 */
#define AR0822_REG_SERIAL_FORMAT	CCI_REG16(0x31AE)
#define AR0822_SERIAL_FORMAT_MIPI(lanes)	((2 << 8) | (lanes))

/* OP_SYS_CLK_DIV: VCO divider for the serial output clock, 4 or 8 here. */
#define AR0822_REG_OP_SYS_CLK_DIV	CCI_REG16(0x3038)

/* D-PHY timing, all in op_pix_clk periods (AND90037-D). */
#define AR0822_REG_FRAME_PREAMBLE	CCI_REG16(0x31B0)
#define AR0822_REG_LINE_PREAMBLE	CCI_REG16(0x31B2)
#define AR0822_REG_MIPI_TIMING_0	CCI_REG16(0x31B4)
#define AR0822_T0_HS_PREPARE		GENMASK(15, 12)
#define AR0822_T0_HS_TRAIL		GENMASK(10, 6)
#define AR0822_T0_CLK_TRAIL		GENMASK(4, 0)
#define AR0822_REG_MIPI_TIMING_1	CCI_REG16(0x31B6)
#define AR0822_T1_CLK_PREPARE		GENMASK(15, 12)
#define AR0822_T1_HS_EXIT		GENMASK(11, 6)
#define AR0822_T1_HS_ZERO		GENMASK(5, 0)
#define AR0822_REG_MIPI_TIMING_2	CCI_REG16(0x31B8)
#define AR0822_T2_BGAP			GENMASK(15, 12)
#define AR0822_T2_CLK_PRE		GENMASK(11, 6)
#define AR0822_T2_CLK_POST		GENMASK(5, 0)
#define AR0822_REG_MIPI_TIMING_3	CCI_REG16(0x31BA)
#define AR0822_T3_LPX			GENMASK(12, 7)
#define AR0822_T3_WAKE_UP		GENMASK(6, 0)
#define AR0822_REG_MIPI_TIMING_4	CCI_REG16(0x31BC)
#define AR0822_T4_CLK_ZERO		GENMASK(13, 7)
#define AR0822_T4_INIT			GENMASK(6, 0)
/* Bandgap settling is not a D-PHY interval; kept at the authored count. */
#define AR0822_T_BGAP			4
/* T_WAKE_UP counts in 8192 op_pix_clk periods, T_INIT in 1024 (AND90037-D). */
#define AR0822_T_WAKE_UP_UNIT		8192
#define AR0822_T_INIT_UNIT		1024

/*
 * MIPI D-PHY v1.2 transmitter minimums, each a time in ns plus a number of UI.
 * HS_ZERO and CLK_ZERO are bounded only as the PREPARE + ZERO sum.
 */
#define AR0822_DPHY_HS_PREPARE_MIN_NS		40
#define AR0822_DPHY_HS_PREPARE_MIN_UI		4
#define AR0822_DPHY_HS_PREPARE_ZERO_MIN_NS	145
#define AR0822_DPHY_HS_PREPARE_ZERO_MIN_UI	10
#define AR0822_DPHY_HS_TRAIL_MIN_NS		60
#define AR0822_DPHY_HS_TRAIL_MIN_UI		4
#define AR0822_DPHY_HS_EXIT_MIN_NS		100
#define AR0822_DPHY_LPX_MIN_NS			50
#define AR0822_DPHY_CLK_PREPARE_MIN_NS		38
#define AR0822_DPHY_CLK_PREPARE_ZERO_MIN_NS	300
#define AR0822_DPHY_CLK_TRAIL_MIN_NS		60
#define AR0822_DPHY_CLK_PRE_MIN_UI		8
#define AR0822_DPHY_CLK_POST_MIN_NS		60
#define AR0822_DPHY_CLK_POST_MIN_UI		52
#define AR0822_DPHY_WAKEUP_MIN_NS		NSEC_PER_MSEC
#define AR0822_DPHY_INIT_MIN_NS			(100 * NSEC_PER_USEC)
/* One extra count on the LP-state intervals, whose slow edges eat into them. */
#define AR0822_DPHY_LP_MARGIN			1
/* LINE_PREAMBLE beyond both lanes' LP-to-HS start-up; matches onsemi's sums. */
#define AR0822_LINE_PREAMBLE_PAD		19

/* Analog gain control. In HDR each exposure has its own gain register. */
#define AR0822_REG_ANALOG_GAIN		CCI_REG16(0x5900)
#define AR0822_REG_ANALOG_GAIN_T2	CCI_REG16(0x5902)
#define AR0822_REG_ANALOG_GAIN_T3	CCI_REG16(0x5904)

/*
 * GLOBAL_GAIN is the shortcut that writes every colour gain for every exposure
 * at once, so one write covers T1..T3 where the analog gain needs three.
 * Q7: 0x080 is 1.0x and the field stops at 0x7ff, a shade under 16x.
 */
#define AR0822_REG_DIGITAL_GAIN		CCI_REG16(0x305E)
#define AR0822_DIGITAL_GAIN_MIN		0x080
#define AR0822_DIGITAL_GAIN_MAX		0x7ff
#define AR0822_DIGITAL_GAIN_DEFAULT	0x080

/* Solid-colour test data, one register per Bayer position. */
/* Both flips latch only while streaming is disabled, so the controls are
 * grabbed for the duration of a stream. */
#define AR0822_REG_READ_MODE		CCI_REG16(0x3040)
#define AR0822_READ_MODE_VERT_FLIP	BIT(15)
#define AR0822_READ_MODE_HORIZ_MIRROR	BIT(14)
/* Binning averages the pixels the matching odd-increment skips over, so the
 * two always move together. Column binning is digital, row binning analog. */
#define AR0822_READ_MODE_COL_BIN	BIT(13)
#define AR0822_READ_MODE_ROW_BIN	BIT(12)

#define AR0822_REG_TEST_PATTERN		CCI_REG16(0x3070)
#define AR0822_TEST_PATTERN_DISABLE	0
#define AR0822_TEST_PATTERN_SOLID_COLOR	1
#define AR0822_TEST_PATTERN_COLOR_BARS	2
#define AR0822_TEST_PATTERN_GREY_COLOR	3
#define AR0822_TEST_PATTERN_PN9		4
#define AR0822_REG_TEST_DATA_RED	CCI_REG16(0x3072)
#define AR0822_REG_TEST_DATA_GREENR	CCI_REG16(0x3074)
#define AR0822_REG_TEST_DATA_BLUE	CCI_REG16(0x3076)
#define AR0822_REG_TEST_DATA_GREENB	CCI_REG16(0x3078)
#define AR0822_TEST_DATA_MAX		0x0fff

/*
 * Multi-exposure combination. The sensor powers up with MEC_CTRL1[15:12] = 2,
 * which is not one of the modes the register map lists; eHDR wants Smooth
 * Combination Max. Its transition thresholds live at T1's scale and have to be
 * scaled onto T2's, so they move with the exposure ratio.
 */
#define AR0822_REG_MEC_CTRL1		CCI_REG16(0x3D00)
#define AR0822_MEC_MODE_MASK		GENMASK(15, 12)
#define AR0822_MEC_MODE_SC_MAX		3
#define AR0822_REG_MEC_T1_S3		CCI_REG16(0x3D28)
#define AR0822_REG_MEC_T1_S4		CCI_REG16(0x3D2A)
#define AR0822_REG_MEC_T2_S3		CCI_REG16(0x3D34)
#define AR0822_REG_MEC_T2_S4		CCI_REG16(0x3D36)
#define AR0822_REG_MEC_T2_S34_HI	CCI_REG16(0x3D5E)
#define AR0822_MEC_S_MAX		0xfffff	/* the thresholds are 20-bit */

/*
 * eHDR control. The sensor captures the exposures, combines them on chip and
 * emits one linear stream, so these only ever change what leaves the sensor,
 * never how many streams the receiver has to deal with.
 */
#define AR0822_REG_EXPOSURE_RATIO	CCI_REG16(0x3238)

/*
 * EXPOSURE_RATIO holds log2 of each ratio: T2 = T1 >> ratio_t1_t2. The T2/T3
 * field keeps its reset value of 4x, unused while only two exposures run.
 */
#define AR0822_HDR_RATIO_MENU_TO_REG(i)	((i) + 1)

/*
 * Registers that describe how many exposures the sensor combines, and what
 * comes out the other side. Two exposures linearise to 16 bits and are read
 * out as-is; three linearise to 20 and have to be companded down to the
 * 16-bit wire, because there is no 20-bit Bayer transport to carry them.
 */
#define AR0822_REG_OPERATION_MODE_CTRL	CCI_REG16(0x3082)
#define AR0822_REG_DIGITAL_CTRL		CCI_REG16(0x30BA)
#define AR0822_REG_DATA_FORMAT_BITS	CCI_REG16(0x31AC)
#define AR0822_REG_COMPAND_EN		CCI_REG16(0x31D0)

/* OPERATION_MODE_CTRL[3:2] holds one less than the exposure count. */
#define AR0822_OPERATION_MODE_NUM_EXP(n)	(((n) - 1) << 2)
/* DIGITAL_CTRL[1:0] mirrors it, and sets the minimum line length. */
#define AR0822_DIGITAL_CTRL_NUM_EXP_MAX(n)	((n) - 1)

/* DATA_FORMAT_BITS: [12:8] the linearised width, [4:0] what is transmitted. */
#define AR0822_DATA_FORMAT(raw, out)	(((raw) << 8) | (out))
#define AR0822_HDR_OUTPUT_BITS		16
#define AR0822_HDR_LINEARISED_BITS(n)	((n) > 2 ? 20 : 16)

/* The linear modes read out 12 bits and represent 12 bits. */
#define AR0822_LINEAR_OUTPUT_BITS	12

/*
 * All 144 line buffers back T1 when two exposures run with automatic
 * allocation, which caps the short exposure at 144 lines. T2 follows T1
 * through the ratio, so the ratio also caps how long T1 may be.
 */
#define AR0822_HDR_DELAY_BUFFER_LINES	144

/* Exposure counts the driver has modes and timings for. */
#define AR0822_HDR_EXPOSURES_MAX	3

/*
 * The sensor emits its optical-black, embedded-data and statistics rows in the
 * vertical blanking; too few and it drops every other frame, which reads as
 * exactly half the rate the timing model predicts. AND90149-D lists what makes
 * up the floor but leaves the count to the Register Wizard, so it is measured:
 * 36 halves, 40 does not, on both a full-resolution and a binned mode.
 */
#define AR0822_VBLANK_MIN		40
/* LINE_LENGTH_PCK_ as programmed by the mode register list. */
#define AR0822_HTS			0x1320

#define AR0822_GAIN_MIN			0
#define AR0822_GAIN_STEP		1
#define AR0822_GAIN_DEFAULT		0
/* 0.375 dB/code up to 44.625 dB; codes 120-135 are discontinuous (120 = 0 dB). */
#define AR0822_GAIN_MAX			119

#define AR0822_EXPOSURE_MIN		(0x10 + 4)
#define AR0822_EXPOSURE_STEP		1
#define AR0822_EXPOSURE_DEFAULT		0x70

/* AR0822 native and active pixel array size. */
#define AR0822_NATIVE_WIDTH		3840U
#define AR0822_NATIVE_HEIGHT		2160U

struct ar0822_reg_list {
	unsigned int num_of_regs;
	const struct cci_reg_sequence *regs;
};

/* Mode : resolution and related config&values */
struct ar0822_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* The row readout floor for this mode, in pixel clocks. This is the
	 * limit the datasheet does not publish - analog-to-digital conversion
	 * and pixel-array memory readout - so it is measured, not derived. The
	 * serial-interface term is computed from the lane count and rate in
	 * ar0822_line_length(), which takes whichever is larger. */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	struct v4l2_fract timeperframe_min;

	/* Default framerate. */
	struct v4l2_fract timeperframe_default;

	/* Default register values */
	struct ar0822_reg_list reg_list;

	/* What this mode emits: enum modalix_sensor_wdr_mode, and the number
	 * of exposures the sensor combines into each frame. */
	u8 wdr_mode;
	u8 exposures;

	/* CSI-2 data type of the emitted stream. */
	u8 csi_dt;
};

static const s64 ar0822_link_freq_menu[] = {
	AR0822_LINK_FREQ_985MBPS,
	AR0822_LINK_FREQ_492MBPS,
};

/*
 * OP_SYS_CLK_DIV for each menu entry, in the same order: one fact in one place,
 * so a rate cannot be added without its divider.
 */
static const u16 ar0822_op_sys_clk_div[] = { 4, 8 };

static_assert(ARRAY_SIZE(ar0822_op_sys_clk_div) ==
	      ARRAY_SIZE(ar0822_link_freq_menu));

/*
 * Registers shared by all modes: linear, 12-bit. OP_SYS_CLK_DIV and the D-PHY
 * timing are not among them; they follow the endpoint's rate.
 * The windowing and output size registers are mode specific and are written
 * afterwards from the per-mode register list.
 */
static const struct cci_reg_sequence ar0822_common_regs[] = {
	/* AND90149-D Table 6, 985 Mbps column: VCO = 27 / 2 * 146 = 1971 MHz,
	 * PIXCLK = 1971 / (2 * 6) = 164.25 MHz, serial = 2 * 1971 / 4.
	 *
	 * The halved rate keeps this PLL and only doubles OP_SYS_CLK_DIV, which
	 * Table 5 allows (the divider takes 1, 2, 4 or 8) and which lands at
	 * 492.75 Mbps per lane, inside the 384 to 1971 Mbps Fserial range.
	 */
	{ CCI_REG16(0x302A), 0x0006 }, // VT_PIX_CLK_DIV
	{ CCI_REG16(0x302C), 0x0002 }, // VT_SYS_CLK_DIV
	{ CCI_REG16(0x302E), 0x0002 }, // PRE_PLL_CLK_DIV
	{ CCI_REG16(0x3030), 0x0092 }, // PLL_MULTIPLIER
	{ CCI_REG16(0x3036), 0x0006 }, // OP_WORD_CLK_DIV: output bit depth / 2
	/* OP_SYS_CLK_DIV is written from the endpoint's link frequency in
	 * ar0822_start_streaming(), not here: the carrier picks the rate.
	 */
	{ CCI_REG16(0x30A2), 0x0001 }, // X_ODD_INC_
	{ CCI_REG16(0x30A6), 0x0001 }, // Y_ODD_INC_
	/* SERIAL_FORMAT is written from the endpoint's lane count in
	 * ar0822_start_streaming(), not pinned here. */
	{ CCI_REG16(0x2510), 0x0720 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2122 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x26FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xFFFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0F8C }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20E0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8055 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA0E1 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3088 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3282 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA681 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FE }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9070 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x891D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x867F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FC }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x893F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0F92 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20E0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0F8F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20E0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9770 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FC }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8054 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x896C }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x200A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9030 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x200A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8040 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8948 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x200A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1597 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8808 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x200A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1F96 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20E0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA0C0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x200A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3044 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3088 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3282 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2004 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1FAA }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20E0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x20FF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3108 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2400 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3244 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3108 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2400 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2702 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3242 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3108 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2420 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2703 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3242 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3108 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2420 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2704 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3244 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8801 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x010F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8855 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3101 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3102 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3181 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3188 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3282 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3104 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xB0E4 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xAD92 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xBC0C }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1028 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0022 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xC020 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x003E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0045 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x00B0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0028 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x30C1 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8015 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA038 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x100F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0507 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA220 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0010 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x10C2 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xB760 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0033 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1082 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x100B }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1029 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA85A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x998D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xC810 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2004 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0ACE }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x13BB }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xC000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1055 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1A9D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2018 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0095 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0B4B }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1022 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA84B }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1015 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x80B9 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA101 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0021 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x00AF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x104B }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x01D9 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x01B5 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0012 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0F0E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8091 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x12A4 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA010 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1012 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x138E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x11A8 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1A2D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x10AF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x13D9 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1635 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0015 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x094B }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x11A1 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1015 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1030 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x004F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x019C }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xB146 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xC090 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0020 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x103C }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA882 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8055 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0129 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8801 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xB100 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2003 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0001 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x00A2 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x14AE }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x000A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x98BB }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2042 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0036 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1001 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x99BE }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2003 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x110A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1022 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0039 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x01C0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9C2E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x052E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x002A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3081 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2001 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3044 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x112A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x101D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x202A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0438 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x10B8 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x11B6 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9B6B }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1039 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1040 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xAD80 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0444 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x10C4 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1023 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1345 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x009F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x002B }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x30D0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3141 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3142 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3110 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3120 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3144 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3148 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3182 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3184 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3190 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x31A0 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3088 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2201 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x807D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2206 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8815 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8877 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0092 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x220E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2211 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8055 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3001 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2005 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8C61 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x8801 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1012 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x171F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0B9F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x101F }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0036 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0040 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0023 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x996E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0257 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1035 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9926 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0039 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x012E }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x11A3 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0048 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xC878 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x200D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x17C8 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0FC9 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x11C9 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x002A }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1057 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3281 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2001 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x3044 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2000 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xA420 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x9C25 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xB824 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2003 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x10D4 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xB06D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0035 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x004D }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1020 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0xB264 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x99C5 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x2012 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x0047 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1041 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x000C }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x1047 }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	{ CCI_REG16(0x2510), 0x7FFF }, // RESERVED_MFR2_2510
	/* COL_COUNT_VALUES1 (0x58E2) stays at its OTPM value: the vendor's
	 * 0x0AFC inverts the ADC readout on rev-3 OTPM parts (0x300E=0x2303). */
	{ CCI_REG16(0x3C70), 0x6828 }, // RESERVED_PARAM_3C70
	{ CCI_REG16(0x351E), 0x5255 }, // RESERVED_MFR_351E
	{ CCI_REG16(0x3092), 0x0826 }, // RESERVED_MFR_3092
	{ CCI_REG16(0x3428), 0x0209 }, // SEQUENCER_CONTROL
	{ CCI_REG16(0x3516), 0xFF04 }, // RESERVED_MFR_3516
	{ CCI_REG16(0x3528), 0x8484 }, // RESERVED_MFR_3528
	{ CCI_REG16(0x352A), 0x6F6F }, // RESERVED_MFR_352A
	{ CCI_REG16(0x3526), 0x9480 }, // RESERVED_MFR_3526
	{ CCI_REG16(0x3530), 0x3308 }, // RESERVED_MFR_3530
	{ CCI_REG16(0x3534), 0x4EBC }, // RESERVED_MFR_3534
	{ CCI_REG16(0x3504), 0x8AAA }, // RESERVED_MFR_3504
	{ CCI_REG16(0x3520), 0xFB00 }, // RESERVED_MFR_3520
	{ CCI_REG16(0x3522), 0x7097 }, // RESERVED_MFR_3522
	{ CCI_REG16(0x353C), 0x220C }, // RESERVED_MFR_353C
	{ CCI_REG16(0x3536), 0x4C6E }, // RESERVED_MFR_3536
	{ CCI_REG16(0x353E), 0x48F2 }, // RESERVED_MFR_353E
	{ CCI_REG16(0x3524), 0xB2C1 }, // RESERVED_MFR_3524
	{ CCI_REG16(0x3532), 0x4C8E }, // RESERVED_MFR_3532
	{ CCI_REG16(0x3D2A), 0x0FFF }, // RESERVED_MFR_3D2A
	{ CCI_REG16(0x3364), 0x00EC }, // RESERVED_MFR_3364
	{ CCI_REG16(0x31AC), 0x0C0C }, // DATA_FORMAT_BITS
	/* sequence_code must be 0 in every mode (AND90149-D Table 3); the
	 * 1 here came from a 1080p register dump. num_exp is set per mode. */
	{ CCI_REG16(0x3082), 0x0000 }, // OPERATION_MODE_CTRL
	{ CCI_REG16(0x30BA), 0x0000 }, // DIGITAL_CTRL
	{ CCI_REG16(0x31AC), 0x0C0C }, // DATA_FORMAT_BITS
	{ CCI_REG16(0x3342), 0x122C }, // MIPI_F1_PDT_EDT: pixel data type RAW12
	{ CCI_REG16(0x3352), 0x0000 }, // MIPI_DT_VC_CONFIG: single DT and VC
	{ CCI_REG16(0x300A), 0x0460 }, // FRAME_LENGTH_LINES_: multiple of 8
	{ CCI_REG16(0x300C), 0x1320 }, // LINE_LENGTH_PCK_: multiple of 8
	{ CCI_REG16(0x3012), 0x045F }, // COARSE_INTEGRATION_TIME
	{ CCI_REG16(0x5900), 0x0005 }, // SENSOR_GAIN
};

/*
 * The analog array is 3840x2160, which is already 16:9, so a full-array mode
 * loses nothing. Anything smaller is reached by skipping with binning rather
 * than by windowing wherever the factor divides: 2x gives 1920x1080 and 4x
 * gives 960x540, both at the full field of view. 1280x720, 640x480 and
 * 640x360 do not divide the array, so they take the largest window whose
 * binned result is the requested size - two thirds of the width - which is the
 * same field of view the ISP would leave if it cropped a binned mode instead,
 * for a fraction of the bandwidth.
 *
 * addr_start must be even and addr_end odd whenever an odd increment is set,
 * and row binning additionally requires an even y_addr_start.
 */
static const struct cci_reg_sequence mode_3840x2160_regs[] = {
	{ CCI_REG16(0x3004), 0x0000 }, // X_ADDR_START_
	{ CCI_REG16(0x3008), 0x0EFF }, // X_ADDR_END_
	{ CCI_REG16(0x3002), 0x0000 }, // Y_ADDR_START_
	{ CCI_REG16(0x3006), 0x086F }, // Y_ADDR_END_
	{ CCI_REG16(0x30A2), 0x0001 }, // X_ODD_INC_: no skip
	{ CCI_REG16(0x30A6), 0x0001 }, // Y_ODD_INC_: no skip
	{ CCI_REG16(0x3040), 0x0000 }, // READ_MODE: no binning
	{ CCI_REG16(0x3402), 0x0F00 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x0870 }, // Y_OUTPUT_CONTROL
};

/* 4:3 out of a 16:9 array: the full height, windowed horizontally. */
static const struct cci_reg_sequence mode_2880x2160_regs[] = {
	{ CCI_REG16(0x3004), 0x01E0 }, // X_ADDR_START_
	{ CCI_REG16(0x3008), 0x0D1F }, // X_ADDR_END_
	{ CCI_REG16(0x3002), 0x0000 }, // Y_ADDR_START_
	{ CCI_REG16(0x3006), 0x086F }, // Y_ADDR_END_
	{ CCI_REG16(0x30A2), 0x0001 }, // X_ODD_INC_: no skip
	{ CCI_REG16(0x30A6), 0x0001 }, // Y_ODD_INC_: no skip
	{ CCI_REG16(0x3040), 0x0000 }, // READ_MODE: no binning
	{ CCI_REG16(0x3402), 0x0B40 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x0870 }, // Y_OUTPUT_CONTROL
};

/* 1080p at the full field of view: the whole array, binned 2x2. */
static const struct cci_reg_sequence mode_1920x1080_regs[] = {
	{ CCI_REG16(0x3004), 0x0000 }, // X_ADDR_START_
	{ CCI_REG16(0x3008), 0x0EFF }, // X_ADDR_END_
	{ CCI_REG16(0x3002), 0x0000 }, // Y_ADDR_START_
	{ CCI_REG16(0x3006), 0x086F }, // Y_ADDR_END_
	{ CCI_REG16(0x30A2), 0x0003 }, // X_ODD_INC_: skip 2
	{ CCI_REG16(0x30A6), 0x0003 }, // Y_ODD_INC_: skip 2
	{ CCI_REG16(0x3040), 0x3000 }, // READ_MODE: column + row binning
	{ CCI_REG16(0x3402), 0x0780 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x0438 }, // Y_OUTPUT_CONTROL
};

/* 540p at the full field of view: the whole array, binned 4x4. */
static const struct cci_reg_sequence mode_960x540_regs[] = {
	{ CCI_REG16(0x3004), 0x0000 }, // X_ADDR_START_
	{ CCI_REG16(0x3008), 0x0EFF }, // X_ADDR_END_
	{ CCI_REG16(0x3002), 0x0000 }, // Y_ADDR_START_
	{ CCI_REG16(0x3006), 0x086F }, // Y_ADDR_END_
	{ CCI_REG16(0x30A2), 0x0007 }, // X_ODD_INC_: skip 4
	{ CCI_REG16(0x30A6), 0x0007 }, // Y_ODD_INC_: skip 4
	{ CCI_REG16(0x3040), 0x3000 }, // READ_MODE: column + row binning
	{ CCI_REG16(0x3402), 0x03C0 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x021C }, // Y_OUTPUT_CONTROL
};

/* 720p: 2560x1440 windowed, binned 2x2. */
static const struct cci_reg_sequence mode_1280x720_regs[] = {
	{ CCI_REG16(0x3004), 0x0280 }, // X_ADDR_START_ 640
	{ CCI_REG16(0x3008), 0x0C7F }, // X_ADDR_END_ 3199
	{ CCI_REG16(0x3002), 0x0168 }, // Y_ADDR_START_ 360
	{ CCI_REG16(0x3006), 0x0707 }, // Y_ADDR_END_ 1799
	{ CCI_REG16(0x30A2), 0x0003 }, // X_ODD_INC_: skip 2
	{ CCI_REG16(0x30A6), 0x0003 }, // Y_ODD_INC_: skip 2
	{ CCI_REG16(0x3040), 0x3000 }, // READ_MODE: column + row binning
	{ CCI_REG16(0x3402), 0x0500 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x02D0 }, // Y_OUTPUT_CONTROL
};

/* 480p (4:3): 2560x1920 windowed, binned 4x4. */
static const struct cci_reg_sequence mode_640x480_regs[] = {
	{ CCI_REG16(0x3004), 0x0280 }, // X_ADDR_START_ 640
	{ CCI_REG16(0x3008), 0x0C7F }, // X_ADDR_END_ 3199
	{ CCI_REG16(0x3002), 0x0078 }, // Y_ADDR_START_ 120
	{ CCI_REG16(0x3006), 0x07F7 }, // Y_ADDR_END_ 2039
	{ CCI_REG16(0x30A2), 0x0007 }, // X_ODD_INC_: skip 4
	{ CCI_REG16(0x30A6), 0x0007 }, // Y_ODD_INC_: skip 4
	{ CCI_REG16(0x3040), 0x3000 }, // READ_MODE: column + row binning
	{ CCI_REG16(0x3402), 0x0280 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x01E0 }, // Y_OUTPUT_CONTROL
};

/* 360p: 2560x1440 windowed, binned 4x4. */
static const struct cci_reg_sequence mode_640x360_regs[] = {
	{ CCI_REG16(0x3004), 0x0280 }, // X_ADDR_START_ 640
	{ CCI_REG16(0x3008), 0x0C7F }, // X_ADDR_END_ 3199
	{ CCI_REG16(0x3002), 0x0168 }, // Y_ADDR_START_ 360
	{ CCI_REG16(0x3006), 0x0707 }, // Y_ADDR_END_ 1799
	{ CCI_REG16(0x30A2), 0x0007 }, // X_ODD_INC_: skip 4
	{ CCI_REG16(0x30A6), 0x0007 }, // Y_ODD_INC_: skip 4
	{ CCI_REG16(0x3040), 0x3000 }, // READ_MODE: column + row binning
	{ CCI_REG16(0x3402), 0x0280 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x0168 }, // Y_OUTPUT_CONTROL
};

static const struct cci_reg_sequence mode_2048x1080_regs[] = {
	{ CCI_REG16(0x3004), 0x0388 }, // X_ADDR_START_
	{ CCI_REG16(0x3008), 0x0B87 }, // X_ADDR_END_
	{ CCI_REG16(0x3002), 0x0224 }, // Y_ADDR_START_
	{ CCI_REG16(0x3006), 0x065B }, // Y_ADDR_END_
	{ CCI_REG16(0x30A2), 0x0001 }, // X_ODD_INC_: no skip
	{ CCI_REG16(0x30A6), 0x0001 }, // Y_ODD_INC_: no skip
	{ CCI_REG16(0x3040), 0x0000 }, // READ_MODE: no binning
	{ CCI_REG16(0x3402), 0x0800 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x0438 }, // Y_OUTPUT_CONTROL
};

/*
 * Two-exposure eHDR without companding. The sensor combines T1 and T2 on chip
 * and reads the result out as one linear 16-bit stream, so the receiver still
 * sees an ordinary single-VC Bayer stream and only the ISP needs to be told
 * that the data is native WDR rather than plain linear.
 *
 * Two lanes at the halved link, 492.75 Mbps each, need 5120 clocks to ship 1920
 * 16-bit pixels, and the two-exposure readout adds analog-to-digital and
 * pixel-array-memory limits that only the vendor Register Wizard computes.
 * Sweeping the line length on hardware puts the real floor between 5464 and
 * 5536 clocks - the cliff is sharp, with capture clean above it and dead below
 * - so this sits about 18% clear of it.

 *
 * MIPI_F1_PDT_EDT is its own register and does not follow DATA_FORMAT_BITS.
 * Left at its RAW12 default the sensor emits correctly formed packets
 * carrying 16-bit pixels under a RAW12 tag, which a receiver filtering for
 * RAW16 discards without reporting a single error - no frames, no faults.
 */
/*
 * What a 16-bit mode adds on top of a window: the sensor linearises its
 * exposures and puts them on the wire at 16 bits, which needs both the output
 * word clock and the MIPI data type moved - MIPI_F1_PDT_EDT is its own
 * register and does not follow DATA_FORMAT_BITS. Written after the mode's
 * window list, so every window can be run at either depth.
 */
static const struct cci_reg_sequence ar0822_regs_16bit[] = {
	{ CCI_REG16(0x33DA), 0x0000 }, // OC_LUT_CONTROL: LUT compander, user LUT
	{ CCI_REG16(0x3036), 0x0008 }, // OP_WORD_CLK_DIV: output bit depth / 2
	{ CCI_REG16(0x3342), 0x122E }, // MIPI_F1_PDT_EDT: pixel data type RAW16
};


/*
 * eHDR line length, in pixel-clock periods. The serial interface has to ship
 * width x 16 bits per row, so its floor halves when the carrier wires four
 * lanes instead of two; the two-exposure readout's own analog-to-digital and
 * pixel-array-memory floors do not move. Measured by sweeping the register on
 * hardware while streaming - capture is clean above the floor and stops dead
 * below it, with a sharp edge: 5536 clocks on two lanes, 2960 on four. These
 * sit about 18% clear of each.
 *
 * Three exposures measured the same floor as two (clean at 2960, dead at
 * 2900 on four lanes), so the exposure count does not enter into it.
 */
/*
 * AND90149-D gives the serial-interface term directly:
 *
 *   MIPI active row transmit time = no_of_cols * bit_depth
 *                                   / (MIPI_speed * no_of_mipi_lanes)
 *
 * so it scales with the width, the depth, the lane rate and the lane count -
 * nothing about it needs to be tabulated per lane count.
 *
 * It is not the only floor. The datasheet names three, "analog-to-digital
 * conversion limited, pixel array memory readout limited, and serial interface
 * bandwidth limited", and publishes only the third; the other two come from the
 * Register Wizard, which we do not have. So the readout floor stays a measured
 * number per mode class rather than a computed one.
 *
 * The cliff was bisected at 2960 clean, 2900 dead, on four lanes, and measured
 * the same for two and three exposures - the extra digitisation happens inside
 * the same row period through the delay buffers. This is the value that has
 * actually been run, a fifth above that cliff rather than sitting on it.
 */
/*
 * Below this the readout does not keep up whatever the link would carry: the
 * guide names an analogue-to-digital and a pixel-array-memory limit on the line
 * length and leaves both to the Register Wizard. Measured on four lanes, on the
 * narrow binned windows where the link floor is far below it: 800 delivers no
 * frames, 896 runs clean.
 */
#define AR0822_HTS_MIN			896

/*
 * Line-time floor for the three-exposure combination. ar0822_line_length()
 * models the serial-interface limit only (eq. 4 of the developer guide); the
 * guide names three - analog-to-digital conversion, pixel array memory
 * readout, and serial interface bandwidth - and states the minimum
 * line_length_pck depends on the number of exposures. The first two are
 * internal to the array and the ADCs, so they do not scale with the lane
 * count; only eq. 4 does, and the max() below already carries that. One
 * constant therefore covers both lane counts.
 *
 * A line shorter than this stalls the receiver outright: the IPI overflows,
 * recovery fails and the frame watchdog gives up after one frame. Measured at
 * 1920x1080 on four lanes at 985 Mbps/lane - dead at 2432, clean at 2480 -
 * with the value set one step above the measured edge for part-to-part
 * margin.
 */
#define AR0822_HDR_HTS_MIN		2528

/*
 * Multi-exposure needs a longer line still - the sensor reads two or three
 * exposures per row before it combines them, which the link floor knows
 * nothing about. Measured the same way: 1600 delivers no frames, 1728 runs
 * clean.
 */
#define AR0822_HDR_HTS_READOUT_MIN	1728

#define AR0822_DEFAULT_MODE 0

/* Mode configs. The default mode must come first. */
static const struct ar0822_mode supported_modes_12bit[] = {
	{
		.width = 1920,
		.height = 1080,
		.line_length_pix = 0,
		/* Binned 2x2 from the whole array, so the field of view is the
		 * array's, not the output's. */
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 12000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	},
	{
		.width = 2048,
		.height = 1080,
		.line_length_pix = 0,
		.crop = {
			.left = 904,
			.top = 548,
			.width = 2048,
			.height = 1080,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2048x1080_regs),
			.regs = mode_2048x1080_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	},
	{
		.width = 3840,
		.height = 2160,
		.line_length_pix = 0,
		/* The whole array, unbinned: the sensor's full 16:9 field of view. */
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 6000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
			.regs = mode_3840x2160_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	},
	{
		.width = 2880,
		.height = 2160,
		.line_length_pix = 0,
		/* 4:3 at the full height the array has. */
		.crop = {
			.left = 480,
			.top = 0,
			.width = 2880,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2880x2160_regs),
			.regs = mode_2880x2160_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	},
	{
		.width = 1280,
		.height = 720,
		.line_length_pix = 0,
		/* 720p does not divide the array, so it takes the largest window that binns to it. */
		.crop = {
			.left = 640,
			.top = 360,
			.width = 2560,
			.height = 1440,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_regs),
			.regs = mode_1280x720_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	},
	{
		.width = 960,
		.height = 540,
		.line_length_pix = 0,
		/* Binned 4x4 from the whole array: 540p at the full field of view. */
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_960x540_regs),
			.regs = mode_960x540_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	},
	{
		.width = 640,
		.height = 480,
		.line_length_pix = 0,
		/* 480p (4:3), binned 4x4 from the largest window that gives it. */
		.crop = {
			.left = 640,
			.top = 120,
			.width = 2560,
			.height = 1920,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_regs),
			.regs = mode_640x480_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	},
	{
		.width = 640,
		.height = 360,
		.line_length_pix = 0,
		/* 360p, binned 4x4 from the largest window that gives it. */
		.crop = {
			.left = 640,
			.top = 360,
			.width = 2560,
			.height = 1440,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x360_regs),
			.regs = mode_640x360_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR,
		.exposures = 1,
		.csi_dt = MIPI_CSI2_DT_RAW12,
	}
};

/*
 * The same eight windows as the linear table, run at 16 bits: the sensor
 * linearises its exposures on chip and puts the result on the wire
 * uncompanded at two exposures and companded at three. Nothing about a
 * window depends on the depth, so both tables point at the same register
 * lists and ar0822_regs_16bit supplies the difference.
 */
static const struct ar0822_mode supported_modes_16bit[] = {
	{
		.width = 1920,
		.height = 1080,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
	{
		.width = 2048,
		.height = 1080,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 904,
			.top = 548,
			.width = 2048,
			.height = 1080,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2048x1080_regs),
			.regs = mode_2048x1080_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
	{
		.width = 3840,
		.height = 2160,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
			.regs = mode_3840x2160_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
	{
		.width = 2880,
		.height = 2160,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 480,
			.top = 0,
			.width = 2880,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2880x2160_regs),
			.regs = mode_2880x2160_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
	{
		.width = 1280,
		.height = 720,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 640,
			.top = 360,
			.width = 2560,
			.height = 1440,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_regs),
			.regs = mode_1280x720_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
	{
		.width = 960,
		.height = 540,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_960x540_regs),
			.regs = mode_960x540_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
	{
		.width = 640,
		.height = 480,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 640,
			.top = 120,
			.width = 2560,
			.height = 1920,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_regs),
			.regs = mode_640x480_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
	{
		.width = 640,
		.height = 360,
		.line_length_pix = AR0822_HDR_HTS_READOUT_MIN,
		.crop = {
			.left = 640,
			.top = 360,
			.width = 2560,
			.height = 1440,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 100000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x360_regs),
			.regs = mode_640x360_regs,
		},
		.wdr_mode = MODALIX_SENSOR_WDR_MODE_NATIVE,
		.exposures = 2,
		.csi_dt = MIPI_CSI2_DT_RAW16,
	},
};

/*
 * The supported formats. The array reads out GRBG unflipped, measured with the
 * solid-colour test pattern, so a mirror gives RGGB and a flip BGGR.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 12-bit linear modes. */
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	/* 16-bit two-exposure eHDR modes. */
	MEDIA_BUS_FMT_SGRBG16_1X16,
	MEDIA_BUS_FMT_SRGGB16_1X16,
	MEDIA_BUS_FMT_SBGGR16_1X16,
	MEDIA_BUS_FMT_SGBRG16_1X16,
};

/* Bayer-order groups of four advertised by enum_mbus_code. */
#define AR0822_CODE_GROUPS		(ARRAY_SIZE(codes) / 4)

/* Menu index i selects a ratio of 1 << (i + 1). */
static const char * const ar0822_hdr_ratio_menu[] = {
	"2x",
	"4x",
	"8x",
	"16x",
};

static const char * const ar0822_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

/* The menu keeps the order the other sensors use; the sensor's MODE field
 * swaps solid colour and colour bars. */
static const int ar0822_test_pattern_val[] = {
	AR0822_TEST_PATTERN_DISABLE,
	AR0822_TEST_PATTERN_COLOR_BARS,
	AR0822_TEST_PATTERN_SOLID_COLOR,
	AR0822_TEST_PATTERN_GREY_COLOR,
	AR0822_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const ar0822_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define AR0822_NUM_SUPPLIES ARRAY_SIZE(ar0822_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define AR0822_XCLR_MIN_DELAY_US	8000
#define AR0822_XCLR_DELAY_RANGE_US	1000

struct ar0822 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct device *dev;
	struct regmap *regmap;

	struct clk *xclk;
	u32 xclk_freq;

	/* Active MIPI data lanes, from the endpoint (2 or 4). */
	u8 num_data_lanes;

	/* Index into ar0822_link_freq_menu[] chosen by the endpoint; the rate
	 * and its OP_SYS_CLK_DIV are read from the tables with it.
	 */
	u8 link_freq_idx;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[AR0822_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *wdr_mode;
	struct v4l2_ctrl *exposures;
	struct v4l2_ctrl *linearised_bits;
	struct v4l2_ctrl *hdr_ratio;
	struct v4l2_ctrl *digital_gain;

	/* Streaming on/off */
	bool streaming;
	/* Silicon revision that cannot do multi-exposure readout. */
	bool single_exposure_only;
};

static inline struct ar0822 *to_ar0822(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct ar0822, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct ar0822_mode **mode_list,
				  unsigned int *num_modes)
{
	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;
		*num_modes = ARRAY_SIZE(supported_modes_12bit);
		break;
	/* 16-bit */
	case MEDIA_BUS_FMT_SRGGB16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SBGGR16_1X16:
		*mode_list = supported_modes_16bit;
		*num_modes = ARRAY_SIZE(supported_modes_16bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Get bayer order based on flip setting. */
static u32 ar0822_get_format_code(struct ar0822 *ar0822, u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
		i = 0;

	i = (i & ~3) | (ar0822->vflip->val ? 2 : 0) |
	    (ar0822->hflip->val ? 1 : 0);

	return codes[i];
}

static const struct ar0822_mode *ar0822_find_mode(struct ar0822 *ar0822,
						  u32 code, u32 width,
						  u32 height)
{
	const struct ar0822_mode *mode_list;
	unsigned int num_modes;

	get_mode_table(code, &mode_list, &num_modes);

	return v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      width, height);
}

/*
 * EXPOSURE_RATIO holds log2 of each step: [2:0] is T1/T2 and [6:4] is T2/T3.
 * Both stages get the same ratio, which is what the datasheet recommends and
 * keeps the combined transfer function even.
 */
static u32 ar0822_exposure_ratio_reg(unsigned int ratio_idx)
{
	unsigned int r = AR0822_HDR_RATIO_MENU_TO_REG(ratio_idx);

	return (r << 4) | r;
}

/*
 * Longest T1 the delay buffers allow. The shorter exposures are derived from
 * T1 by the ratio and have to fit in the 144 shared line buffers, which the
 * datasheet states as T2 + 2*T3 <= 144. With T2 = T1 >> r and T3 = T1 >> 2r
 * that rearranges to a ceiling on T1 itself, so a small ratio buys dynamic
 * range at the cost of how long the long exposure may be.
 */
/*
 * What a pixel represents once the ISP has expanded it. Only three exposures
 * compand: they linearise to 20 bits and no 20-bit Bayer transport exists, so
 * they are squeezed onto the same 16-bit wire the two-exposure mode uses
 * uncompanded. Everything else already represents what it transmits.
 */
static u8 ar0822_linearised_bits(const struct ar0822_mode *mode,
				 unsigned int exposures)
{
	if (mode->exposures < 2)
		return AR0822_LINEAR_OUTPUT_BITS;

	return AR0822_HDR_LINEARISED_BITS(exposures);
}

static u32 ar0822_hdr_exposure_ceiling(unsigned int exposures, unsigned int ratio_idx)
{
	unsigned int r = AR0822_HDR_RATIO_MENU_TO_REG(ratio_idx);

	if (exposures < 2)
		return U32_MAX;

	if (exposures == 2)
		return AR0822_HDR_DELAY_BUFFER_LINES << r;

	/* 144 * 2^2r / (2^r + 2) */
	return (u32)((u64)AR0822_HDR_DELAY_BUFFER_LINES << (2 * r)) /
	       ((1u << r) + 2u);
}

static int ar0822_apply_mec_config(struct ar0822 *ar0822, unsigned int exposures,
				   unsigned int ratio_idx);
static void ar0822_update_line_timing(struct ar0822 *ar0822,
				      const struct ar0822_mode *mode);

static int ar0822_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0822 *ar0822 =
		container_of(ctrl->handler, struct ar0822, ctrl_handler);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	u32 exposure_max, exposure_def;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&ar0822->sd);
	format = v4l2_subdev_state_get_format(state, 0);

	/* A part without eHDR takes the request and runs one exposure. */
	if (ctrl->id == MODALIX_SENSOR_V4L2_CID_EXPOSURES && ctrl->val > 1 &&
	    ar0822->single_exposure_only) {
		dev_warn(ar0822->dev,
			 "%d exposures requested on rev 0x%04x, but this rev does not support eHDR, running one\n",
			 ctrl->val, AR0822_REVISION_NO_EHDR);
		ctrl->val = 1;
	}

	/*
	 * VBLANK moves the frame length, and the HDR ratio moves how long the
	 * delay buffers can hold the short exposure. Either can change the
	 * usable exposure range, so recompute it from both.
	 */
	if (ctrl->id == V4L2_CID_VBLANK ||
	    ctrl->id == MODALIX_SENSOR_V4L2_CID_HDR_EXPOSURE_RATIO ||
	    ctrl->id == MODALIX_SENSOR_V4L2_CID_EXPOSURES) {
		unsigned int vblank = ar0822->vblank->val;
		unsigned int ratio = ar0822->hdr_ratio->val;
		unsigned int exposures = ar0822->exposures->val;

		if (ctrl->id == V4L2_CID_VBLANK)
			vblank = ctrl->val;
		else if (ctrl->id == MODALIX_SENSOR_V4L2_CID_HDR_EXPOSURE_RATIO)
			ratio = ctrl->val;
		else
			exposures = ctrl->val;

		/* Only an eHDR mode lets the count move, so the eHDR law
		 * applies without asking the mode again. */
		if (ctrl->id == MODALIX_SENSOR_V4L2_CID_EXPOSURES) {
			const struct ar0822_mode *cur;

			__v4l2_ctrl_s_ctrl(ar0822->linearised_bits,
					   AR0822_HDR_LINEARISED_BITS(exposures));

			/* The count also moves the line the sensor will run,
			 * and stream-on programs that line from the control.
			 * Move the derived timing with it, or HBLANK keeps
			 * describing the count we just left. VBLANK is not
			 * touched here: it is the frame rate, owned by the
			 * caller, and .set_fmt is what re-seats it. */
			cur = ar0822_find_mode(ar0822, format->code,
					       format->width, format->height);
			if (cur)
				ar0822_update_line_timing(ar0822, cur);
		}

		exposure_max = format->height + vblank - 4;
		exposure_max = min_t(u32, exposure_max,
				     ar0822_hdr_exposure_ceiling(exposures, ratio));
		exposure_def = min_t(u32, exposure_max, AR0822_EXPOSURE_DEFAULT);
		__v4l2_ctrl_modify_range(ar0822->exposure,
					 ar0822->exposure->minimum, exposure_max,
					 ar0822->exposure->step, exposure_def);
	}

	/*
	 * Applying V4L2 control values only makes sense while the device is
	 * powered up for streaming.
	 */
	if (!pm_runtime_get_if_in_use(ar0822->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(ar0822->regmap, AR0822_REG_ANALOG_GAIN,
				ctrl->val & 0xffff, NULL);
		/* Every exposure gets the same gain so the combined result
		 * keeps one response curve. */
		if (!ret && ar0822->exposures->val > 1)
			ret = cci_write(ar0822->regmap,
					AR0822_REG_ANALOG_GAIN_T2,
					ctrl->val & 0xffff, NULL);
		if (!ret && ar0822->exposures->val > 2)
			ret = cci_write(ar0822->regmap,
					AR0822_REG_ANALOG_GAIN_T3,
					ctrl->val & 0xffff, NULL);
		break;
	case MODALIX_SENSOR_V4L2_CID_HDR_EXPOSURE_RATIO:
		ret = cci_write(ar0822->regmap, AR0822_REG_EXPOSURE_RATIO,
				ar0822_exposure_ratio_reg(ctrl->val), NULL);
		/* The combination thresholds and the delay buffers are both
		 * derived from this ratio, so they move with it. */
		if (!ret)
			ret = ar0822_apply_mec_config(ar0822,
						      ar0822->exposures->val,
						      ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		/* One write covers every colour of every exposure. */
		ret = cci_write(ar0822->regmap, AR0822_REG_DIGITAL_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = cci_write(ar0822->regmap, AR0822_REG_TEST_DATA_RED,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = cci_write(ar0822->regmap, AR0822_REG_TEST_DATA_GREENR,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = cci_write(ar0822->regmap, AR0822_REG_TEST_DATA_BLUE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = cci_write(ar0822->regmap, AR0822_REG_TEST_DATA_GREENB,
				ctrl->val, NULL);
		break;
	case MODALIX_SENSOR_V4L2_CID_EXPOSURES:
		/* Applied as a set by ar0822_apply_hdr_config() at stream
		 * start; nothing to poke mid-stream. */
		break;
	case V4L2_CID_EXPOSURE:
		/* both registers are 16-bit, and the control ranges are now
		 * capped to match, so nothing here can wrap */
		ret = cci_write(ar0822->regmap, AR0822_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(ar0822->regmap, AR0822_REG_FRAME_LENGTH_LINES,
				format->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(ar0822->regmap, AR0822_REG_TEST_PATTERN,
				ar0822_test_pattern_val[ctrl->val], NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP: {
		u64 read_mode;

		/* The register also carries the embedded-data row counts, so
		 * keep everything but the two flip bits. */
		cci_read(ar0822->regmap, AR0822_REG_READ_MODE, &read_mode, &ret);
		read_mode &= ~(u64)(AR0822_READ_MODE_VERT_FLIP |
				    AR0822_READ_MODE_HORIZ_MIRROR);
		if (ar0822->vflip->val)
			read_mode |= AR0822_READ_MODE_VERT_FLIP;
		if (ar0822->hflip->val)
			read_mode |= AR0822_READ_MODE_HORIZ_MIRROR;
		cci_write(ar0822->regmap, AR0822_REG_READ_MODE, read_mode, &ret);
		break;
	}
	case V4L2_CID_HBLANK:
	case V4L2_CID_LINK_FREQ:
	case V4L2_CID_PIXEL_RATE:
	case MODALIX_SENSOR_V4L2_CID_WDR_MODE:
	case MODALIX_SENSOR_V4L2_CID_LINEARISED_BITS:
		/* Read-only, or applied through the mode register list. */
		break;
	default:
		dev_dbg(ar0822->dev, "ctrl(id:0x%x, val:0x%x) not handled\n",
			ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(ar0822->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0822_ctrl_ops = {
	.s_ctrl = ar0822_set_ctrl,
};

/*
 * How the selected mode reads out.
 *
 * The WDR mode is read-only: it follows from the media-bus format, and only
 * reports the consequence of that choice so the ISP cannot be told something
 * the sensor is not doing.
 *
 * The exposure count is writable, because the format cannot express it. The
 * two- and three-exposure eHDR modes have identical geometry and the same
 * 16-bit code - they differ only in how many exposures are combined and
 * whether the result is companded - so this is the axis that picks between
 * them. Linear modes pin it at one.
 */
static const struct v4l2_ctrl_config ar0822_ctrl_wdr_mode = {
	.ops = &ar0822_ctrl_ops,
	.id = MODALIX_SENSOR_V4L2_CID_WDR_MODE,
	.name = "WDR Mode",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = MODALIX_SENSOR_WDR_MODE_LINEAR,
	.max = MODALIX_SENSOR_WDR_MODE_FS_LIN,
	.step = 1,
	.def = MODALIX_SENSOR_WDR_MODE_LINEAR,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

/*
 * Created as the linear mode's fixed single exposure, and read-only with it.
 * An eHDR format widens the range and clears the flag from the active
 * set_fmt path; init_state only ever sets a TRY format, so a subdev that has
 * not had an active format set since probe is still in the linear default and
 * must not accept a count the sensor is not going to emit.
 */
static const struct v4l2_ctrl_config ar0822_ctrl_exposures = {
	.ops = &ar0822_ctrl_ops,
	.id = MODALIX_SENSOR_V4L2_CID_EXPOSURES,
	.name = "Exposures Per Frame",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 1,
	.step = 1,
	.def = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config ar0822_ctrl_linearised_bits = {
	.ops = &ar0822_ctrl_ops,
	.id = MODALIX_SENSOR_V4L2_CID_LINEARISED_BITS,
	.name = "Linearised Bit Depth",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = AR0822_LINEAR_OUTPUT_BITS,
	.max = AR0822_HDR_LINEARISED_BITS(AR0822_HDR_EXPOSURES_MAX),
	.step = 1,
	.def = AR0822_LINEAR_OUTPUT_BITS,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config ar0822_ctrl_hdr_ratio = {
	.ops = &ar0822_ctrl_ops,
	.id = MODALIX_SENSOR_V4L2_CID_HDR_EXPOSURE_RATIO,
	.name = "HDR Exposure Ratio",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 0,
	.max = ARRAY_SIZE(ar0822_hdr_ratio_menu) - 1,
	.def = ARRAY_SIZE(ar0822_hdr_ratio_menu) - 1,
	.qmenu = ar0822_hdr_ratio_menu,
};

/* The rate the endpoint chose, from the menu the index refers to. */
static s64 ar0822_link_freq(const struct ar0822 *ar0822)
{
	return ar0822_link_freq_menu[ar0822->link_freq_idx];
}

static int ar0822_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0822 *ar0822 = to_ar0822(sd);

	if (code->pad != 0)
		return -EINVAL;

	if (code->index >= AR0822_CODE_GROUPS)
		return -EINVAL;

	code->code = ar0822_get_format_code(ar0822, codes[code->index * 4]);

	return 0;
}

static int ar0822_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0822 *ar0822 = to_ar0822(sd);
	const struct ar0822_mode *mode_list;
	unsigned int num_modes;

	if (fse->pad != 0)
		return -EINVAL;

	get_mode_table(fse->code, &mode_list, &num_modes);

	if (fse->index >= num_modes)
		return -EINVAL;

	if (fse->code != ar0822_get_format_code(ar0822, fse->code))
		return -EINVAL;

	fse->min_width = mode_list[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = mode_list[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

/*
 * Line length this carrier runs the mode at. Only the multi-exposure modes
 * care: their floor is set by how fast 16-bit pixels can be shipped, which
 * depends on how many lanes are wired.
 */
/* What the link carries per pixel: the eHDR modes linearise onto a 16-bit
 * wire, the linear ones stay at 12. */
/* Shallowest and deepest the sensor puts on the wire: RAW12 and RAW16. */
#define AR0822_BITS_PER_PIXEL_MIN	12

static unsigned int ar0822_output_bits(const struct ar0822_mode *mode)
{
	return mode->csi_dt == MIPI_CSI2_DT_RAW16 ? 16 : 12;
}

/* LINE_LENGTH_PCK must be a multiple of eight for correct operation. */
/*
 * LINE_LENGTH_PCK must be a multiple of eight for correct operation
 * (AND90149-D), and its bit 0 is not writable in any case.
 */
static unsigned int ar0822_hts_round(u64 hts)
{
	return (unsigned int)round_up(hts, 8);
}

/*
 * The line has to be long enough to serialise itself and long enough for the
 * sensor to read the row out. Derive the first from the link, take the second
 * from the mode, and round the larger up to the granularity the register
 * needs.
 */
/*
 * The rate pixels leave the source pad: what the link carries, divided by the
 * depth on it. V4L2 pairs this with HBLANK - a line takes (width + HBLANK) /
 * pixel_rate - so it has to be the rate that makes that the real line time,
 * not the readout clock the registers are counted in.
 */
static u64 ar0822_pixel_rate(const struct ar0822 *ar0822,
			     const struct ar0822_mode *mode)
{
	u64 link_bits_per_s = 2ULL * ar0822->num_data_lanes *
			      ar0822_link_freq(ar0822);

	return div_u64(link_bits_per_s, ar0822_output_bits(mode));
}

/* The widest the source pad ever runs: the shallowest depth it can emit. */
static u64 ar0822_pixel_rate_max(const struct ar0822 *ar0822)
{
	return div_u64(2ULL * ar0822->num_data_lanes * ar0822_link_freq(ar0822),
		       AR0822_BITS_PER_PIXEL_MIN);
}

static unsigned int ar0822_line_length(const struct ar0822 *ar0822,
				       const struct ar0822_mode *mode)
{
	unsigned int bpp = ar0822_output_bits(mode);
	unsigned int fll;
	u64 link_bits_per_s;
	u64 hts_link, hts_rate;

	/* DDR, so each lane carries two bits per link clock. */
	link_bits_per_s = 2ULL * ar0822->num_data_lanes *
			  ar0822_link_freq(ar0822);

	/* eq. 4 as a pixel-clock count: cols * depth / link, times the pixel
	 * rate. */
	hts_link = DIV_ROUND_UP_ULL((u64)mode->width * bpp * AR0822_VT_PIX_CLK,
				    link_bits_per_s);

	/* eq. 4 covers only the active pixels. The line also carries the
	 * packet header and footer and the LP-HS transitions either side of
	 * them, which the guide does not quantify; a quarter of the active
	 * time covers it with margin. Measured at 3840 on four lanes: the
	 * receiver reports payload CRC errors from about an eighth over eq. 4
	 * and stops delivering frames at a sixteenth. */
	hts_link += hts_link / 4;

	/* Nothing is gained by a line shorter than the mode's fastest frame
	 * interval needs once the frame length is at its floor, and a short
	 * line costs readout margin, so take the longer of the two. */
	fll = round_up(mode->height + AR0822_VBLANK_MIN, 8);
	hts_rate = DIV_ROUND_DOWN_ULL((u64)AR0822_VT_PIX_CLK *
				      mode->timeperframe_min.numerator,
				      (u64)mode->timeperframe_min.denominator * fll);

	/*
	 * Only the three-exposure combination is bounded by the readout rather
	 * than the link. Two exposures stream cleanly on the computed length -
	 * 1920x1080 runs at its full 77.9 fps on LINE_LENGTH_PCK 1728 - so the
	 * floor is taken from the count the sensor will actually emit, not
	 * from the mode's, which reads two for both flavours of the 16-bit
	 * modes. set_fmt settles the control before it asks for the bounds.
	 */
	if (ar0822->exposures && ar0822->exposures->val > 2)
		hts_link = max_t(u64, hts_link, AR0822_HDR_HTS_MIN);

	return ar0822_hts_round(max3((u64)mode->line_length_pix,
				     max_t(u64, hts_link, AR0822_HTS_MIN),
				     hts_rate));
}


static
unsigned int ar0822_get_frame_length(const struct ar0822_mode *mode,
				     const struct v4l2_fract *timeperframe,
				     unsigned int line_length_pix)
{
	u64 frame_length;

	frame_length = (u64)timeperframe->numerator * AR0822_VT_PIX_CLK;
	do_div(frame_length,
	       (u64)timeperframe->denominator * line_length_pix);

	if (WARN_ON(frame_length > AR0822_FRAME_LENGTH_MAX))
		frame_length = AR0822_FRAME_LENGTH_MAX;

	return max_t(unsigned int, frame_length, mode->height);
}

/*
 * The frame-length floor and default for a mode, and the line length they are
 * derived from. Control registration needs the same three values a mode set
 * does.
 */
static unsigned int ar0822_frame_length_bounds(const struct ar0822 *ar0822,
					       const struct ar0822_mode *mode,
					       unsigned int *frm_length_min,
					       unsigned int *frm_length_default)
{
	unsigned int line_length = ar0822_line_length(ar0822, mode);

	*frm_length_min = ar0822_get_frame_length(mode, &mode->timeperframe_min,
						  line_length);
	*frm_length_default = ar0822_get_frame_length(mode,
						      &mode->timeperframe_default,
						      line_length);

	/* The sensor needs its optical-black, embedded-data and test rows in
	 * the blanking, so a frame length equal to the active height starves
	 * it and it delivers every other frame. ar0822_get_frame_length()
	 * bounds by the active height alone, which is not enough. */
	*frm_length_min = max_t(unsigned int, *frm_length_min,
				mode->height + AR0822_VBLANK_MIN);
	*frm_length_default = max_t(unsigned int, *frm_length_default,
				    *frm_length_min);

	/* FRAME_LENGTH_LINES must be a multiple of eight, and it is the active
	 * height plus VBLANK. Round both ends of the frame length up to eight
	 * and step in eights, so every value the control can take keeps the
	 * register compliant whatever the mode's height is - 540 is not itself
	 * a multiple of eight, so the minimum carries the remainder. */
	*frm_length_min = round_up(*frm_length_min, 8);
	*frm_length_default = round_up(*frm_length_default, 8);

	return line_length;
}

/*
 * HBLANK and PIXEL_RATE are pure functions of the line the sensor will run,
 * and that line depends on the exposure count - the three-exposure
 * combination is floored well above the computed length. The count can move
 * after .set_fmt has run (the ISP brokers it on behalf of the IPA), so both
 * paths resolve the timing here: otherwise the sensor takes the long line at
 * stream-on while HBLANK still advertises the short one, and the receiver's
 * IPI timing and the ISP's total_width and lines_per_second are all derived
 * from a line the sensor is not running.
 */
static void ar0822_update_line_timing(struct ar0822 *ar0822,
				      const struct ar0822_mode *mode)
{
	unsigned int frm_length_min, frm_length_default, hblank_min, line_length;
	u64 pixel_rate;

	line_length = ar0822_frame_length_bounds(ar0822, mode, &frm_length_min,
						 &frm_length_default);
	pixel_rate = ar0822_pixel_rate(ar0822, mode);
	hblank_min = div64_u64((u64)line_length * pixel_rate,
			       AR0822_VT_PIX_CLK) - mode->width;

	__v4l2_ctrl_modify_range(ar0822->hblank, hblank_min, hblank_min, 1,
				 hblank_min);
	__v4l2_ctrl_modify_range(ar0822->pixel_rate, pixel_rate, pixel_rate, 1,
				 pixel_rate);
}

/*
 * Update the vblank, hblank and exposure control ranges to match the selected
 * mode.  Called from .set_fmt with the subdev state (== control handler) lock
 * held, so the __v4l2_ctrl_modify_range() variants are used.
 */
static void ar0822_update_controls(struct ar0822 *ar0822,
				   const struct ar0822_mode *mode)
{
	unsigned int frm_length_min, frm_length_default;
	u32 exposure_max, exposure_def;

	/* Describe the mode before deriving anything that depends on it.
	 *
	 * The 16-bit modes take one to three exposures on identical geometry
	 * and media-bus code, so the format cannot tell them apart and the
	 * exposure count becomes a second selection axis. One exposure at 16
	 * bits is the plain 12-bit readout in a 16-bit word. Linear modes pin
	 * it at one. */
	__v4l2_ctrl_s_ctrl(ar0822->wdr_mode, mode->wdr_mode);
	if (mode->exposures > 1) {
		__v4l2_ctrl_modify_range(ar0822->exposures, 1,
					 AR0822_HDR_EXPOSURES_MAX, 1,
					 mode->exposures);
		ar0822->exposures->flags &= ~V4L2_CTRL_FLAG_READ_ONLY;
	} else {
		/* Collapsing the range makes the control framework mark it
		 * read-only, which is right for a linear mode but does not
		 * clear itself when the range opens back up. Say it plainly
		 * in both directions. */
		__v4l2_ctrl_modify_range(ar0822->exposures, 1, 1, 1, 1);
		ar0822->exposures->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	/* The count survives a mode set, so take it from the control rather
	 * than the mode's default - otherwise a three-exposure stream tells
	 * the ISP it is only two and the decompander never runs. */
	__v4l2_ctrl_s_ctrl(ar0822->linearised_bits,
			   ar0822_linearised_bits(mode, ar0822->exposures->val));

	/* Only the frame bounds are wanted here; the line the length implies is
	 * applied by ar0822_update_line_timing() below. */
	ar0822_frame_length_bounds(ar0822, mode, &frm_length_min,
				   &frm_length_default);

	/*
	 * frame_length_lines (R0x300A) is a single 16-bit register and the
	 * AR0822 has no long-exposure multiplier to extend it, so the range
	 * ends where the register does. Advertising a multiple of it only
	 * meant a large VBLANK wrapped to a much shorter frame.
	 */
	/* modify_range() keeps the current value when it still fits the new
	 * range, so a mode change would carry the previous mode's blanking
	 * across and run at its rate. The exposure bound below is derived from
	 * frm_length_default, so the two have to describe the same frame. */
	__v4l2_ctrl_modify_range(ar0822->vblank, frm_length_min - mode->height,
				 round_down(AR0822_FRAME_LENGTH_MAX, 8) - mode->height,
				 8, frm_length_default - mode->height);
	__v4l2_ctrl_s_ctrl(ar0822->vblank, frm_length_default - mode->height);

	/* Fixed by the mode's LINE_LENGTH_PCK, so min == max. The line length
	 * is counted in vt_pix_clk; scale it into the pixel_rate domain so
	 * that (width + HBLANK) / pixel_rate is the line time the sensor
	 * really runs, which is also what the receiver derives its own line
	 * timing from. */
	ar0822_update_line_timing(ar0822, mode);

	exposure_max = frm_length_default - 4;
	exposure_max = min_t(u32, exposure_max,
			     ar0822_hdr_exposure_ceiling(ar0822->exposures->val,
							 ar0822->hdr_ratio->val));
	exposure_def = min_t(u32, exposure_max, AR0822_EXPOSURE_DEFAULT);
	__v4l2_ctrl_modify_range(ar0822->exposure, ar0822->exposure->minimum,
				 exposure_max, ar0822->exposure->step,
				 exposure_def);
}

static int ar0822_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0822 *ar0822 = to_ar0822(sd);
	const struct ar0822_mode *mode;
	struct v4l2_mbus_framefmt *format;

	if (fmt->pad != 0)
		return -EINVAL;

	/* Bayer order varies with flips */
	fmt->format.code = ar0822_get_format_code(ar0822, fmt->format.code);

	mode = ar0822_find_mode(ar0822, fmt->format.code, fmt->format.width,
				fmt->format.height);

	format = v4l2_subdev_state_get_format(sd_state, fmt->pad);

	format->width = mode->width;
	format->height = mode->height;
	format->code = fmt->format.code;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ar0822_update_controls(ar0822, mode);

	return 0;
}

static int ar0822_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct ar0822 *ar0822 = to_ar0822(sd);
	const struct v4l2_mbus_framefmt *format;
	const struct ar0822_mode *mode;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		format = v4l2_subdev_state_get_format(sd_state, 0);
		mode = ar0822_find_mode(ar0822, format->code, format->width,
					format->height);
		sel->r = mode->crop;

		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = AR0822_NATIVE_WIDTH;
		sel->r.height = AR0822_NATIVE_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int ar0822_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = MEDIA_BUS_FMT_SGRBG12_1X12,
			.width = supported_modes_12bit[AR0822_DEFAULT_MODE].width,
			.height = supported_modes_12bit[AR0822_DEFAULT_MODE].height,
		},
	};

	ar0822_set_pad_format(sd, state, &fmt);

	return 0;
}

/* Start streaming */
/*
 * Everything about the readout that the exposure count decides. Kept out of
 * the per-mode register lists because two- and three-exposure share a mode:
 * same geometry, same media-bus code, different readout.
 */
/*
 * Combination parameters that follow the exposure ratio. The T2 thresholds are
 * the T1 ones scaled by T1/T2, and the delay buffers have to hold T2 and T3
 * for as many lines as those exposures last. Both therefore have to be redone
 * whenever the ratio moves, not just at stream start.
 */
static int ar0822_apply_mec_config(struct ar0822 *ar0822, unsigned int exposures,
				   unsigned int ratio_idx)
{
	unsigned int r = AR0822_HDR_RATIO_MENU_TO_REG(ratio_idx);
	u64 t1_s3, t1_s4, mec;
	u32 t2_s3, t2_s4;
	int ret = 0;

	if (exposures < 2)
		return 0;

	/* Pick the documented algorithm, leaving the noise-region enables in
	 * the low bits as the preset tuned them. */
	cci_read(ar0822->regmap, AR0822_REG_MEC_CTRL1, &mec, &ret);
	if (ret)
		return ret;
	mec &= ~(u64)AR0822_MEC_MODE_MASK;
	mec |= FIELD_PREP(AR0822_MEC_MODE_MASK, AR0822_MEC_MODE_SC_MAX);
	cci_write(ar0822->regmap, AR0822_REG_MEC_CTRL1, mec, &ret);

	cci_read(ar0822->regmap, AR0822_REG_MEC_T1_S3, &t1_s3, &ret);
	cci_read(ar0822->regmap, AR0822_REG_MEC_T1_S4, &t1_s4, &ret);
	if (ret)
		return ret;

	t2_s3 = min_t(u64, t1_s3 << r, AR0822_MEC_S_MAX);
	t2_s4 = min_t(u64, t1_s4 << r, AR0822_MEC_S_MAX);

	cci_write(ar0822->regmap, AR0822_REG_MEC_T2_S3, t2_s3 & 0xffff, &ret);
	cci_write(ar0822->regmap, AR0822_REG_MEC_T2_S4, t2_s4 & 0xffff, &ret);
	cci_write(ar0822->regmap, AR0822_REG_MEC_T2_S34_HI,
		  ((t2_s4 >> 16) << 8) | (t2_s3 >> 16), &ret);

	return ret;
}

static int ar0822_apply_hdr_config(struct ar0822 *ar0822,
				   const struct ar0822_mode *mode)
{
	unsigned int exposures = ar0822->exposures->val;
	int ret = 0;

	if (mode->exposures < 2)
		return 0;

	cci_write(ar0822->regmap, AR0822_REG_OPERATION_MODE_CTRL,
		  AR0822_OPERATION_MODE_NUM_EXP(exposures), &ret);
	cci_write(ar0822->regmap, AR0822_REG_DIGITAL_CTRL,
		  AR0822_DIGITAL_CTRL_NUM_EXP_MAX(exposures), &ret);
	/* Two exposures linearise to 16 bits and go out untouched; three
	 * linearise to 20 and are companded down to the same 16-bit wire.
	 * One exposure is the 12-bit ADC value, which the sensor places in
	 * the upper bits of the 16-bit word when told the raw size is 12. */
	cci_write(ar0822->regmap, AR0822_REG_DATA_FORMAT_BITS,
		  AR0822_DATA_FORMAT(exposures < 2 ? AR0822_LINEAR_OUTPUT_BITS :
				     AR0822_HDR_LINEARISED_BITS(exposures),
				     AR0822_HDR_OUTPUT_BITS), &ret);
	cci_write(ar0822->regmap, AR0822_REG_COMPAND_EN,
		  exposures > 2 ? 1 : 0, &ret);
	cci_write(ar0822->regmap, AR0822_REG_EXPOSURE_RATIO,
		  ar0822_exposure_ratio_reg(ar0822->hdr_ratio->val), &ret);
	if (ret)
		return ret;

	return ar0822_apply_mec_config(ar0822, exposures,
				       ar0822->hdr_ratio->val);
}

/*
 * A D-PHY v1.2 minimum of ns + ui as op_pix_clk periods, rounded up. A period
 * is bits UI (AND90149-D eq. 2), so the count follows lane rate and depth.
 */
static u32 ar0822_dphy_count(u64 lane_bps, unsigned int bits, u64 ns, u32 ui)
{
	return DIV64_U64_ROUND_UP(ns * lane_bps + (u64)ui * NSEC_PER_SEC,
				  (u64)bits * NSEC_PER_SEC);
}

/*
 * Each interval's count is its D-PHY minimum at this lane rate and depth. LPX
 * appears twice in LINE_PREAMBLE: once for the clock lane, once for the data.
 */
static int ar0822_write_dphy_timing(struct ar0822 *ar0822,
				    const struct ar0822_mode *mode)
{
	/* DDR, so each lane carries two bits per link clock. */
	u64 lane_bps = 2ULL * ar0822_link_freq(ar0822);
	unsigned int bits = ar0822_output_bits(mode);
	u32 hs_prepare, hs_zero, hs_trail, hs_exit, lpx;
	u32 clk_prepare, clk_zero, clk_trail, clk_pre, clk_post;
	u32 wake_up, t_init, line_preamble, frame_preamble;
	int ret = 0;

	hs_prepare = ar0822_dphy_count(lane_bps, bits,
				       AR0822_DPHY_HS_PREPARE_MIN_NS,
				       AR0822_DPHY_HS_PREPARE_MIN_UI);
	hs_zero = ar0822_dphy_count(lane_bps, bits,
				    AR0822_DPHY_HS_PREPARE_ZERO_MIN_NS,
				    AR0822_DPHY_HS_PREPARE_ZERO_MIN_UI);
	hs_zero -= hs_prepare;
	hs_trail = ar0822_dphy_count(lane_bps, bits,
				     AR0822_DPHY_HS_TRAIL_MIN_NS,
				     AR0822_DPHY_HS_TRAIL_MIN_UI);
	hs_exit = ar0822_dphy_count(lane_bps, bits,
				    AR0822_DPHY_HS_EXIT_MIN_NS, 0);
	hs_exit += AR0822_DPHY_LP_MARGIN;
	lpx = ar0822_dphy_count(lane_bps, bits, AR0822_DPHY_LPX_MIN_NS, 0);
	lpx += AR0822_DPHY_LP_MARGIN;

	clk_prepare = ar0822_dphy_count(lane_bps, bits,
					AR0822_DPHY_CLK_PREPARE_MIN_NS, 0);
	clk_zero = ar0822_dphy_count(lane_bps, bits,
				     AR0822_DPHY_CLK_PREPARE_ZERO_MIN_NS, 0);
	clk_zero -= clk_prepare;
	clk_trail = ar0822_dphy_count(lane_bps, bits,
				      AR0822_DPHY_CLK_TRAIL_MIN_NS, 0);
	clk_pre = ar0822_dphy_count(lane_bps, bits, 0,
				    AR0822_DPHY_CLK_PRE_MIN_UI);
	clk_post = ar0822_dphy_count(lane_bps, bits,
				     AR0822_DPHY_CLK_POST_MIN_NS,
				     AR0822_DPHY_CLK_POST_MIN_UI);

	wake_up = ar0822_dphy_count(lane_bps, bits,
				    AR0822_DPHY_WAKEUP_MIN_NS, 0);
	wake_up = DIV_ROUND_UP(wake_up, AR0822_T_WAKE_UP_UNIT);
	t_init = ar0822_dphy_count(lane_bps, bits, AR0822_DPHY_INIT_MIN_NS, 0);
	t_init = DIV_ROUND_UP(t_init, AR0822_T_INIT_UNIT);

	line_preamble = 2 * lpx + clk_prepare + clk_zero + clk_pre +
			hs_prepare + hs_zero + AR0822_LINE_PREAMBLE_PAD;
	frame_preamble = line_preamble + hs_trail + hs_exit + lpx +
			 hs_prepare + hs_zero;

	cci_write(ar0822->regmap, AR0822_REG_FRAME_PREAMBLE,
		  frame_preamble, &ret);
	cci_write(ar0822->regmap, AR0822_REG_LINE_PREAMBLE,
		  line_preamble, &ret);
	cci_write(ar0822->regmap, AR0822_REG_MIPI_TIMING_0,
		  FIELD_PREP(AR0822_T0_HS_PREPARE, hs_prepare) |
		  FIELD_PREP(AR0822_T0_HS_TRAIL, hs_trail) |
		  FIELD_PREP(AR0822_T0_CLK_TRAIL, clk_trail), &ret);
	cci_write(ar0822->regmap, AR0822_REG_MIPI_TIMING_1,
		  FIELD_PREP(AR0822_T1_CLK_PREPARE, clk_prepare) |
		  FIELD_PREP(AR0822_T1_HS_EXIT, hs_exit) |
		  FIELD_PREP(AR0822_T1_HS_ZERO, hs_zero), &ret);
	cci_write(ar0822->regmap, AR0822_REG_MIPI_TIMING_2,
		  FIELD_PREP(AR0822_T2_BGAP, AR0822_T_BGAP) |
		  FIELD_PREP(AR0822_T2_CLK_PRE, clk_pre) |
		  FIELD_PREP(AR0822_T2_CLK_POST, clk_post), &ret);
	cci_write(ar0822->regmap, AR0822_REG_MIPI_TIMING_3,
		  FIELD_PREP(AR0822_T3_LPX, lpx) |
		  FIELD_PREP(AR0822_T3_WAKE_UP, wake_up), &ret);
	cci_write(ar0822->regmap, AR0822_REG_MIPI_TIMING_4,
		  FIELD_PREP(AR0822_T4_CLK_ZERO, clk_zero) |
		  FIELD_PREP(AR0822_T4_INIT, t_init), &ret);

	return ret;
}

static int ar0822_start_streaming(struct ar0822 *ar0822,
				  struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format =
		v4l2_subdev_state_get_format(state, 0);
	const struct ar0822_mode *mode =
		ar0822_find_mode(ar0822, format->code, format->width,
				 format->height);
	const struct ar0822_reg_list *reg_list = &mode->reg_list;
	int ret = 0;

	/* Software reset the sensor before loading the mode registers. */
	cci_write(ar0822->regmap, AR0822_REG_MODE_SELECT,
		  AR0822_MODE_SW_RESET, &ret);
	if (ret)
		return ret;
	msleep(100);
	cci_write(ar0822->regmap, AR0822_REG_MODE_SELECT, 0x0000, &ret);
	if (ret)
		return ret;
	msleep(100);

	/* Apply the registers shared by all modes */
	ret = cci_multi_reg_write(ar0822->regmap, ar0822_common_regs,
				  ARRAY_SIZE(ar0822_common_regs), NULL);
	if (ret) {
		dev_err(ar0822->dev, "failed to set common registers\n");
		return ret;
	}

	/* The carrier decides how many lanes are wired and how fast they can
	 * run, so neither can live in the shared register list.
	 */
	ret = cci_write(ar0822->regmap, AR0822_REG_SERIAL_FORMAT,
			AR0822_SERIAL_FORMAT_MIPI(ar0822->num_data_lanes), NULL);
	if (ret) {
		dev_err(ar0822->dev, "failed to set the serial format\n");
		return ret;
	}

	ret = cci_write(ar0822->regmap, AR0822_REG_OP_SYS_CLK_DIV,
			ar0822_op_sys_clk_div[ar0822->link_freq_idx], NULL);
	if (ret) {
		dev_err(ar0822->dev, "failed to set the output clock divider\n");
		return ret;
	}

	ret = ar0822_write_dphy_timing(ar0822, mode);
	if (ret) {
		dev_err(ar0822->dev, "failed to set the D-PHY timing\n");
		return ret;
	}

	/* Apply the default values of the current mode */
	ret = cci_multi_reg_write(ar0822->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(ar0822->dev, "failed to set mode\n");
		return ret;
	}

	if (mode->csi_dt == MIPI_CSI2_DT_RAW16) {
		ret = cci_multi_reg_write(ar0822->regmap, ar0822_regs_16bit,
					  ARRAY_SIZE(ar0822_regs_16bit), NULL);
		if (ret) {
			dev_err(ar0822->dev, "failed to set the output depth\n");
			return ret;
		}
	}

	ret = ar0822_apply_hdr_config(ar0822, mode);
	if (ret) {
		dev_err(ar0822->dev, "failed to set the HDR configuration\n");
		return ret;
	}

	/* Line length depends on the lane count and the exposure count, so it
	 * cannot be pinned in the per-mode list. FRAME_LENGTH_LINES_ follows
	 * from VBLANK below. */
	ret = cci_write(ar0822->regmap, AR0822_REG_LINE_LENGTH_PCK,
			ar0822_line_length(ar0822, mode), NULL);
	if (ret) {
		dev_err(ar0822->dev, "failed to set the line length\n");
		return ret;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(ar0822->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return cci_write(ar0822->regmap, AR0822_REG_MODE_SELECT,
			 AR0822_MODE_START_STREAMING, NULL);
}

/* Stop streaming */
static void ar0822_stop_streaming(struct ar0822 *ar0822)
{
	/* set stream off register */
	cci_write(ar0822->regmap, AR0822_REG_MODE_SELECT,
		  AR0822_MODE_STOP_STREAMING, NULL);
}

static int ar0822_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0822 *ar0822 = to_ar0822(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (ar0822->streaming == enable)
		goto unlock;

	if (enable) {
		ret = pm_runtime_resume_and_get(ar0822->dev);
		if (ret < 0)
			goto unlock;

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = ar0822_start_streaming(ar0822, state);
		if (ret) {
			pm_runtime_put_sync(ar0822->dev);
			goto unlock;
		}
	} else {
		ar0822_stop_streaming(ar0822);
		pm_runtime_put(ar0822->dev);
	}

	ar0822->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(ar0822->vflip, enable);
	__v4l2_ctrl_grab(ar0822->hflip, enable);
	/*
	 * The exposure count and its ratio are applied as a set by
	 * ar0822_apply_hdr_config() at stream start, so a write mid-stream
	 * cannot reach the sensor. Left writable it would still move the
	 * control, the linearised bit depth and the exposure limits - and
	 * the ISP would then decompand three-exposure data the sensor is
	 * not producing. Refuse it instead.
	 */
	__v4l2_ctrl_grab(ar0822->exposures, enable);
	__v4l2_ctrl_grab(ar0822->hdr_ratio, enable);

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;
}

/*
 * Report the frame descriptor so downstream CSI-2 aggregators (e.g. GMSL
 * serdes) can learn the stream's data type and virtual channel.
 */
static int ar0822_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct ar0822 *ar0822 = to_ar0822(sd);
	const struct v4l2_mbus_framefmt *format;
	const struct ar0822_mode *mode;
	struct v4l2_subdev_state *state;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	format = v4l2_subdev_state_get_format(state, 0);
	mode = ar0822_find_mode(ar0822, format->code, format->width,
				format->height);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry->pixelcode = format->code;
	fd->entry->stream = 0;
	fd->entry->bus.csi2.vc = 0;
	/* eHDR is combined on chip, so even a multi-exposure mode leaves the
	 * sensor as one stream - only its depth changes. */
	fd->entry->bus.csi2.dt = mode->csi_dt;

	v4l2_subdev_unlock_state(state);

	return 0;
}

static const struct v4l2_subdev_core_ops ar0822_subdev_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ar0822_subdev_video_ops = {
	.s_stream = ar0822_set_stream,
};

static const struct v4l2_subdev_pad_ops ar0822_subdev_pad_ops = {
	.enum_mbus_code = ar0822_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar0822_set_pad_format,
	.get_selection = ar0822_get_selection,
	.enum_frame_size = ar0822_enum_frame_size,
	.get_frame_desc = ar0822_get_frame_desc,
};

static const struct v4l2_subdev_ops ar0822_subdev_ops = {
	.core = &ar0822_subdev_core_ops,
	.video = &ar0822_subdev_video_ops,
	.pad = &ar0822_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0822_internal_ops = {
	.init_state = ar0822_init_state,
};

/* Initialize control handlers */
static int ar0822_init_controls(struct ar0822 *ar0822)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &ar0822->ctrl_handler;
	const struct ar0822_mode *mode = &supported_modes_12bit[AR0822_DEFAULT_MODE];
	struct v4l2_fwnode_device_properties props;
	unsigned int frm_length_min, frm_length_default;
	unsigned int line_length, hblank;
	u32 exposure_max;
	u64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 20);
	if (ret)
		return ret;

	/* PIXEL_RATE follows the mode's depth, so it is a range the mode set
	 * narrows, not a constant. Read only either way. */
	pixel_rate = ar0822_pixel_rate(ar0822, mode);
	ar0822->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, 1,
					       ar0822_pixel_rate_max(ar0822), 1,
					       pixel_rate);
	if (ar0822->pixel_rate)
		ar0822->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* LINK_FREQ is also read only */
	ar0822->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &ar0822_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(ar0822_link_freq_menu) - 1,
				       ar0822->link_freq_idx,
				       ar0822_link_freq_menu);
	if (ar0822->link_freq)
		ar0822->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here. Limits that vary with the mode are reset
	 * by the ar0822_update_controls() call from .set_fmt, but they have to
	 * describe the default mode until then: .init_state only ever sets a
	 * TRY format, so a client that streams without setting one runs on
	 * what is registered here.
	 */
	/* FRAME_LENGTH_LINES must be a multiple of eight for correct
	 * operation, and it is the active height plus this, so the step has to
	 * be eight too. Every mode height in the table is a multiple of eight,
	 * which keeps the sum one as well. */
	line_length = ar0822_frame_length_bounds(ar0822, mode, &frm_length_min,
						 &frm_length_default);
	ar0822->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
					   V4L2_CID_VBLANK,
					   frm_length_min - mode->height,
					   round_down(AR0822_FRAME_LENGTH_MAX, 8) -
					   mode->height,
					   8, frm_length_default - mode->height);
	/* LINE_LENGTH_PCK follows the lane count and link rate and the frame
	 * rate is set through VBLANK, so this only ever reports. */
	hblank = div64_u64((u64)line_length * pixel_rate, AR0822_VT_PIX_CLK) -
		 mode->width;
	ar0822->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (ar0822->hblank)
		ar0822->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Bounded by the frame it is integrated in, exactly as a mode set
	 * bounds it. This used to be a nominal VTS no mode runs, and a client
	 * that has not set a format cannot ask for more than is registered
	 * here. */
	exposure_max = min_t(u32, frm_length_default - 4,
			     ar0822_hdr_exposure_ceiling(mode->exposures,
							 ar0822_ctrl_hdr_ratio.def));
	ar0822->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     AR0822_EXPOSURE_MIN,
					     exposure_max,
					     AR0822_EXPOSURE_STEP,
					     min_t(u32, exposure_max,
						   AR0822_EXPOSURE_DEFAULT));

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  AR0822_GAIN_MIN, AR0822_GAIN_MAX,
			  AR0822_GAIN_STEP, AR0822_GAIN_DEFAULT);

	/* Where analog gain runs out, this carries on: the tuning describes it
	 * already and the IPA skips it entirely if the control is absent. */
	ar0822->digital_gain = v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
						 V4L2_CID_DIGITAL_GAIN,
						 AR0822_DIGITAL_GAIN_MIN,
						 AR0822_DIGITAL_GAIN_MAX, 1,
						 AR0822_DIGITAL_GAIN_DEFAULT);

	/* Created before .set_fmt runs, which is what fills them in. The
	 * ratio defaults to 16x, where the delay-buffer limit on the long
	 * exposure sits above the frame length and so never binds. */
	ar0822->wdr_mode = v4l2_ctrl_new_custom(ctrl_hdlr,
						&ar0822_ctrl_wdr_mode, NULL);
	ar0822->exposures = v4l2_ctrl_new_custom(ctrl_hdlr,
						 &ar0822_ctrl_exposures, NULL);
	ar0822->linearised_bits = v4l2_ctrl_new_custom(ctrl_hdlr,
					&ar0822_ctrl_linearised_bits, NULL);
	ar0822->hdr_ratio = v4l2_ctrl_new_custom(ctrl_hdlr,
						 &ar0822_ctrl_hdr_ratio, NULL);

	ar0822->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (ar0822->hflip)
		ar0822->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ar0822->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (ar0822->vflip)
		ar0822->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &ar0822_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0822_test_pattern_menu) - 1,
				     0, 0, ar0822_test_pattern_menu);

	/* A known constant out of the sensor separates a sensor fault from an
	 * ISP one in a single capture, so keep all four positions settable. */
	v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops, V4L2_CID_TEST_PATTERN_RED,
			  0, AR0822_TEST_DATA_MAX, 1, 0);
	v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
			  V4L2_CID_TEST_PATTERN_GREENR,
			  0, AR0822_TEST_DATA_MAX, 1, 0);
	v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops, V4L2_CID_TEST_PATTERN_BLUE,
			  0, AR0822_TEST_DATA_MAX, 1, 0);
	v4l2_ctrl_new_std(ctrl_hdlr, &ar0822_ctrl_ops,
			  V4L2_CID_TEST_PATTERN_GREENB,
			  0, AR0822_TEST_DATA_MAX, 1, 0);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(ar0822->dev, "control init failed (%d)\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(ar0822->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ar0822_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	ar0822->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void ar0822_free_controls(struct ar0822 *ar0822)
{
	v4l2_ctrl_handler_free(ar0822->sd.ctrl_handler);
}

static int ar0822_check_hwcfg(struct device *dev, u8 *num_data_lanes,
			      u8 *link_freq_idx)
{
	unsigned int i;
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* The sensor can drive 1, 2 or 4 lanes; the carrier decides which are
	 * wired. Only the counts we have modes and timings for are accepted. */
	switch (ep_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		*num_data_lanes = ep_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		dev_err(dev, "%u data lanes is not supported, expected 2 or 4\n",
			ep_cfg.bus.mipi_csi2.num_data_lanes);
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	/*
	 * Either rate is accepted and the divider follows the one the endpoint
	 * names, so a carrier that cannot sync at the full rate runs the same
	 * driver at the halved one.
	 */
	if (ep_cfg.nr_of_link_frequencies != 1) {
		dev_err(dev, "expected exactly one link frequency, got %u\n",
			ep_cfg.nr_of_link_frequencies);
		goto error_out;
	}

	for (i = 0; i < ARRAY_SIZE(ar0822_link_freq_menu); i++) {
		if (ep_cfg.link_frequencies[0] == ar0822_link_freq_menu[i]) {
			*link_freq_idx = i;
			break;
		}
	}
	if (i == ARRAY_SIZE(ar0822_link_freq_menu)) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int ar0822_get_regulators(struct ar0822 *ar0822)
{
	unsigned int i;

	for (i = 0; i < AR0822_NUM_SUPPLIES; i++)
		ar0822->supplies[i].supply = ar0822_supply_name[i];

	return devm_regulator_bulk_get(ar0822->dev, AR0822_NUM_SUPPLIES,
				       ar0822->supplies);
}

static int ar0822_parse_hw_config(struct ar0822 *ar0822)
{
	struct device *dev = ar0822->dev;
	int ret;

	/* Check the hardware configuration in device tree */
	ret = ar0822_check_hwcfg(dev, &ar0822->num_data_lanes,
				 &ar0822->link_freq_idx);
	if (ret)
		return ret;

	ret = ar0822_get_regulators(ar0822);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/* Get system clock (xclk) */
	ar0822->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(ar0822->xclk))
		return dev_err_probe(dev, PTR_ERR(ar0822->xclk),
				     "failed to get xclk\n");

	ar0822->xclk_freq = clk_get_rate(ar0822->xclk);
	if (ar0822->xclk_freq != AR0822_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "xclk frequency %u not supported\n",
				     ar0822->xclk_freq);

	/* Request optional reset pin */
	ar0822->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(ar0822->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ar0822->reset_gpio),
				     "failed to get reset GPIO\n");

	return 0;
}

static void ar0822_identify_model(struct ar0822 *ar0822)
{
	u64 chip_id, revision;
	int ret;

	ret = cci_read(ar0822->regmap, AR0822_REG_CHIP_ID, &chip_id, NULL);
	if (ret)
		dev_warn(ar0822->dev, "failed to read chip id: %d\n", ret);
	else if (chip_id != AR0822_CHIP_ID)
		dev_warn(ar0822->dev, "unexpected chip id 0x%04llx\n", chip_id);
	else
		dev_info(ar0822->dev, "Detected AR0822 image sensor\n");

	if (cci_read(ar0822->regmap, AR0822_REG_REVISION, &revision, NULL))
		return;
	ar0822->single_exposure_only = revision == AR0822_REVISION_NO_EHDR;
	if (ar0822->single_exposure_only)
		dev_warn(ar0822->dev,
			 "revision 0x%04llx: this module does not support eHDR\n",
			 revision);
}

/* Power/clock management functions */
static int ar0822_power_on(struct ar0822 *ar0822)
{
	int ret;

	ret = regulator_bulk_enable(AR0822_NUM_SUPPLIES, ar0822->supplies);
	if (ret) {
		dev_err(ar0822->dev, "failed to enable regulators\n");
		return ret;
	}

	/* xclk is a board-provided clock; nothing to gate in SW. */
	ret = clk_prepare_enable(ar0822->xclk);
	if (ret) {
		dev_err(ar0822->dev, "failed to enable clock\n");
		regulator_bulk_disable(AR0822_NUM_SUPPLIES, ar0822->supplies);
		return ret;
	}

	if (ar0822->reset_gpio) {
		gpiod_set_value_cansleep(ar0822->reset_gpio, 0);
		msleep(100);
		gpiod_set_value_cansleep(ar0822->reset_gpio, 1);
	}

	usleep_range(AR0822_XCLR_MIN_DELAY_US,
		     AR0822_XCLR_MIN_DELAY_US + AR0822_XCLR_DELAY_RANGE_US);

	return 0;
}

static void ar0822_power_off(struct ar0822 *ar0822)
{
	if (ar0822->reset_gpio)
		gpiod_set_value_cansleep(ar0822->reset_gpio, 0);

	clk_disable_unprepare(ar0822->xclk);

	regulator_bulk_disable(AR0822_NUM_SUPPLIES, ar0822->supplies);
}

static int __maybe_unused ar0822_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0822 *ar0822 = to_ar0822(sd);

	if (ar0822->streaming)
		ar0822_stop_streaming(ar0822);

	return 0;
}

static int __maybe_unused ar0822_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0822 *ar0822 = to_ar0822(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (!ar0822->streaming)
		return 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	ret = ar0822_start_streaming(ar0822, state);
	if (ret) {
		ar0822_stop_streaming(ar0822);
		ar0822->streaming = false;
	}
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ar0822_subdev_init(struct ar0822 *ar0822)
{
	struct i2c_client *client = to_i2c_client(ar0822->dev);
	int ret;

	v4l2_i2c_subdev_init(&ar0822->sd, client, &ar0822_subdev_ops);

	ret = ar0822_init_controls(ar0822);
	if (ret)
		return ret;

	/* Initialize subdev */
	ar0822->sd.internal_ops = &ar0822_internal_ops;
	ar0822->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	ar0822->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	ar0822->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ar0822->sd.entity, 1, &ar0822->pad);
	if (ret) {
		dev_err(ar0822->dev, "failed to init entity pads: %d\n", ret);
		goto err_free_ctrls;
	}

	ar0822->sd.state_lock = ar0822->sd.ctrl_handler->lock;
	ret = v4l2_subdev_init_finalize(&ar0822->sd);
	if (ret) {
		dev_err(ar0822->dev, "subdev init error: %d\n", ret);
		goto err_media;
	}

	return 0;

err_media:
	media_entity_cleanup(&ar0822->sd.entity);
err_free_ctrls:
	ar0822_free_controls(ar0822);
	return ret;
}

static void ar0822_subdev_cleanup(struct ar0822 *ar0822)
{
	v4l2_subdev_cleanup(&ar0822->sd);
	media_entity_cleanup(&ar0822->sd.entity);
	ar0822_free_controls(ar0822);
}

static int ar0822_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0822 *ar0822;
	int ret;

	ar0822 = devm_kzalloc(dev, sizeof(*ar0822), GFP_KERNEL);
	if (!ar0822)
		return -ENOMEM;

	ar0822->dev = dev;

	ret = ar0822_parse_hw_config(ar0822);
	if (ret)
		return ret;

	ar0822->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ar0822->regmap))
		return dev_err_probe(dev, PTR_ERR(ar0822->regmap),
				     "failed to initialise CCI\n");

	/*
	 * The sensor must be powered on to read the chip ID register and to be
	 * initialised.  Power it up manually here and hand control to runtime
	 * PM once the subdev is registered.
	 */
	ret = ar0822_power_on(ar0822);
	if (ret)
		return ret;

	ar0822_identify_model(ar0822);

	ret = ar0822_subdev_init(ar0822);
	if (ret)
		goto err_power_off;

	/*
	 * Enable runtime PM.  As the device has been powered manually, mark it
	 * as active and take a usage reference without resuming it.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&ar0822->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto err_pm;
	}

	/*
	 * Finally, enable autosuspend and drop the usage reference.  The device
	 * will get suspended after the autosuspend delay, turning the power off.
	 */
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	ar0822_subdev_cleanup(ar0822);
err_power_off:
	ar0822_power_off(ar0822);

	return ret;
}

static void ar0822_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0822 *ar0822 = to_ar0822(sd);

	v4l2_async_unregister_subdev(sd);
	ar0822_subdev_cleanup(ar0822);

	/*
	 * Disable runtime PM.  In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(ar0822->dev);
	if (!pm_runtime_status_suspended(ar0822->dev))
		ar0822_power_off(ar0822);
	pm_runtime_set_suspended(ar0822->dev);
}

static int ar0822_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	return ar0822_power_on(to_ar0822(sd));
}

static int ar0822_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	ar0822_power_off(to_ar0822(sd));

	return 0;
}

static const struct dev_pm_ops ar0822_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ar0822_suspend, ar0822_resume)
	SET_RUNTIME_PM_OPS(ar0822_runtime_suspend, ar0822_runtime_resume, NULL)
};

static const struct of_device_id ar0822_dt_ids[] = {
	{ .compatible = "onsemi,ar0822" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ar0822_dt_ids);

static struct i2c_driver ar0822_i2c_driver = {
	.driver = {
		.name = "ar0822",
		.of_match_table = ar0822_dt_ids,
		.pm = &ar0822_pm_ops,
	},
	.probe = ar0822_probe,
	.remove = ar0822_remove,
};

module_i2c_driver(ar0822_i2c_driver);

MODULE_AUTHOR("zihongl@leopardimaging.com");
MODULE_DESCRIPTION("ON Semiconductor AR0822 sensor driver");
MODULE_LICENSE("GPL v2");
