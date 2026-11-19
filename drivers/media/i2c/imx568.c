// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for SONY IMX568 cameras.
 *
 * Based on Samsung S5K6AAFX SXGA 1/6" 1.3M CMOS Image Sensor driver
 * Copyright (C) 2011 Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Copyright (C) 2016, Synopsys, Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/mipi-csi2.h>
#include "imx568-timing.h"

/* Default when the endpoint gives no data-lanes. */
#define IMX568_DEFAULT_LANES 2
#define IMX568_INCK_FREQ_HZ	37125000

/*
 * From the datasheet, "20ms after PWDN goes low or 20ms after RESETB goes
 * high if reset is inserted after PWDN goes high, host can access sensor's
 * SCCB to initialize sensor."
 */
#define PWDN_ACTIVE_DELAY_MS	20

/* Streaming / master control */
#define IMX568_SW_STANDBY		CCI_REG8(0x3000)	/* 0=operating, 1=standby */
#define IMX568_XMSTA			CCI_REG8(0x3010)	/* master: 0=start, 1=stop */

/* Exposure (SHS) and analog gain */
#define IMX568_SHS			CCI_REG24_LE(0x3240)	/* shutter, 20-bit */
#define IMX568_GAIN			CCI_REG16_LE(0x3514)	/* analog gain, 10-bit */

/* Frame timing */
#define IMX568_HMAX			CCI_REG16_LE(0x30d8)
#define IMX568_VMAX			CCI_REG24_LE(0x30d4)	/* 24-bit */

/* Per-mode geometry / bit-depth registers */
#define IMX568_HWIDTH			CCI_REG16_LE(0x30d0)	/* VOPB_VBLK output width */
#define IMX568_FINFO_HWIDTH		CCI_REG16_LE(0x30d2)	/* FINFO output width */
#define IMX568_GMRWT			CCI_REG8(0x30e2)	/* AD readout timing */
#define IMX568_GMTWT			CCI_REG8(0x30e3)
#define IMX568_ROI_PH1			CCI_REG16_LE(0x3120)	/* FID0 ROI H position */
#define IMX568_ROI_PV1			CCI_REG16_LE(0x3122)	/* FID0 ROI V position */
#define IMX568_ROI_WH1			CCI_REG16_LE(0x3124)	/* FID0 ROI H width */
#define IMX568_ROI_WV1			CCI_REG16_LE(0x3126)	/* FID0 ROI V width */
#define IMX568_ADBIT			CCI_REG8(0x3200)	/* AD bit depth: [5:4] 0=10b,1=12b */
#define IMX568_ODBIT			CCI_REG8(0x3430)	/* output bit depth: [1:0] 0=10b,1=12b */

/* Named registers programmed by the common base table */
#define IMX568_INCKSEL_ST0		CCI_REG8(0x3014)
#define IMX568_INCKSEL_ST1		CCI_REG8(0x3015)
#define IMX568_INCKSEL_ST2		CCI_REG8(0x3016)
#define IMX568_INCKSEL_ST3		CCI_REG8(0x3018)
#define IMX568_INCKSEL_ST4		CCI_REG8(0x3019)
#define IMX568_INCKSEL_ST5		CCI_REG8(0x301b)
#define IMX568_GSDLY			CCI_REG8(0x30e6)
#define IMX568_ROI_MODE			CCI_REG8(0x3100)
#define IMX568_FID0_ROION		CCI_REG8(0x3104)
#define IMX568_INCKSEL_N0		CCI_REG8(0x321c)
#define IMX568_INCKSEL_N1		CCI_REG8(0x321d)
#define IMX568_INCKSEL_N2		CCI_REG8(0x321e)
#define IMX568_INCKSEL_N3		CCI_REG8(0x321f)
#define IMX568_INCKSEL_D0		CCI_REG8(0x3224)
#define IMX568_INCKSEL_D1		CCI_REG8(0x3225)
#define IMX568_INCKSEL_D2		CCI_REG8(0x3226)
#define IMX568_INCKSEL_D3		CCI_REG8(0x3227)
#define IMX568_VINT_EN			CCI_REG8(0x323e)
#define IMX568_PULSE2_EN		CCI_REG8(0x3480)
#define IMX568_GAIN_RTS			CCI_REG8(0x3502)
#define IMX568_BLKLEVEL			CCI_REG8(0x35b4)
#define IMX568_LANESEL			CCI_REG8(0x3904)
#define IMX568_EAV_SEL_MIPI		CCI_REG8(0x3942)
#define IMX568_MIPI_CLK_CONT		CCI_REG8(0x3c30)	/* continuous clock */
#define IMX568_TXCLKESC_FREQ		CCI_REG16_LE(0x3ca4)
#define IMX568_THS_PREPARE		CCI_REG16_LE(0x3cb4)
#define IMX568_TCLK_POST		CCI_REG16_LE(0x3cb6)
#define IMX568_THS_TRAIL		CCI_REG16_LE(0x3cb8)
#define IMX568_THS_ZERO			CCI_REG16_LE(0x3cba)
#define IMX568_TCLK_PREPARE		CCI_REG16_LE(0x3cbc)
#define IMX568_TCLK_TRAIL		CCI_REG16_LE(0x3cbe)
#define IMX568_TLPX			CCI_REG16_LE(0x3cc0)
#define IMX568_TCLK_ZERO		CCI_REG16_LE(0x3cc2)
#define IMX568_TCLK_PRE			CCI_REG16_LE(0x3cc4)
#define IMX568_THS_EXIT			CCI_REG16_LE(0x3cc6)

/* IMX568 native and active pixel array size */
#define IMX568_NATIVE_WIDTH		2464U
#define IMX568_NATIVE_HEIGHT		2064U

#define IMX568_PIXEL_ARRAY_LEFT		8U
#define IMX568_PIXEL_ARRAY_TOP		8U
#define IMX568_PIXEL_ARRAY_WIDTH	1920U
#define IMX568_PIXEL_ARRAY_HEIGHT	1080U

/* 2432x2048 all-pixel mode: 32-byte-stride-aligned crop of the 2448 area. */
/* Centered 1920x1080 window: 16+(2432-1920)/2, 8+(2048-1080)/2. */
#define IMX568_FHD_LEFT			272U
#define IMX568_FHD_TOP			492U

#define IMX568_FULL_WIDTH		2432U
#define IMX568_FULL_HEIGHT		2048U
#define IMX568_FULL_LEFT		16U
#define IMX568_FULL_TOP			8U
#define IMX568_FULL_HTS			IMX568_HMAX_1188_2LANE_FULL
#define IMX568_FULL_HTS_4LANE		IMX568_HMAX_1188_4LANE_FULL
#define IMX568_FULL_VTS			2250	/* VMAX (legal again under the SHS clamp) */

/* Cropped sub-modes (ROI crop of the native 2464x2064 array). All 32-byte
 * stride-aligned (width % 64 == 0 for RAW12). Their default frame period is
 * about 30 fps; the selected profile's minimum VBLANK bounds the maximum.
 */
#define IMX568_HD_WIDTH			1280U
#define IMX568_HD_HEIGHT		720U
#define IMX568_HD_LEFT			592U
#define IMX568_HD_TOP			672U
#define IMX568_QVGA_WIDTH		640U
#define IMX568_QVGA_HEIGHT		360U
#define IMX568_QVGA_LEFT		912U
#define IMX568_QVGA_TOP			852U
#define IMX568_W169_WIDTH		2432U
#define IMX568_W169_HEIGHT		1368U
#define IMX568_W169_LEFT		16U
#define IMX568_W169_TOP			348U
#define IMX568_W169_594_FRAME_CLOCKS \
	(IMX568_HMAX_594_2LANE_RAW12 * 1553)
#define IMX568_XGA_WIDTH		1024U
#define IMX568_XGA_HEIGHT		768U
#define IMX568_XGA_LEFT			720U
#define IMX568_XGA_TOP			648U
#define IMX568_SVGA_WIDTH		800U
#define IMX568_SVGA_HEIGHT		600U
#define IMX568_SVGA_LEFT		832U
#define IMX568_SVGA_TOP			732U
#define IMX568_VGA_WIDTH		640U
#define IMX568_VGA_HEIGHT		480U
#define IMX568_VGA_LEFT			912U
#define IMX568_VGA_TOP			792U
#define IMX568_SXGA_WIDTH		1280U
#define IMX568_SXGA_HEIGHT		960U
#define IMX568_SXGA_LEFT		592U
#define IMX568_SXGA_TOP			552U
#define IMX568_UXGA_WIDTH		1600U
#define IMX568_UXGA_HEIGHT		1200U
#define IMX568_UXGA_LEFT		432U
#define IMX568_UXGA_TOP			432U
#define IMX568_QXGA_WIDTH		2048U
#define IMX568_QXGA_HEIGHT		1536U
#define IMX568_QXGA_LEFT		208U
#define IMX568_QXGA_TOP			264U
#define IMX568_CROP_HTS			IMX568_HMAX_1188_2LANE_RAW12
#define IMX568_CROP_HTS_4LANE		IMX568_HMAX_1188_4LANE_RAW12
#define IMX568_CROP_VTS			2565	/* VMAX: 74.25MHz/(965*2565)=30fps */

/* Sony ROI floor VMAX >= height + GMRWT + GMTWT + GSDLY + 86, plus a 16-line guard band. */
#define IMX568_SHS_READOUT_LINES	86
#define IMX568_VBLANK_GUARD_LINES	16
#define IMX568_ROI_VBLANK_LINES		(IMX568_SHS_READOUT_LINES + \
					 IMX568_VBLANK_GUARD_LINES)
#define IMX568_VTS_MAX			32767
#define IMX568_VTS			(0xA02)

#define IMX568_HTS			IMX568_HMAX_1188_2LANE_RAW12
#define IMX568_HTS_4LANE		IMX568_HMAX_1188_4LANE_RAW12
#define IMX568_HTS_MAX			0x7fff

#define IMX568_GAIN_MIN			0
#define IMX568_GAIN_STEP		1
#define IMX568_GAIN_DEFAULT		0
#define IMX568_GAIN_MAX			(480)

#define IMX568_EXPOSURE_MIN		(0x10+4)
#define IMX568_EXPOSURE_STEP		1
#define IMX568_EXPOSURE_DEFAULT		(0x70)
/* SHS >= height + reserve: exposure <= VBLANK - (reserve + 1 - EXPOSURE_MIN). */

/* regulator supplies */
static const char * const imx568_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX568_NUM_SUPPLIES ARRAY_SIZE(imx568_supply_names)

#define IMX568_RATE_594		0
#define IMX568_RATE_891		1
#define IMX568_RATE_1188		2

static const s64 imx568_link_freqs[] = {
	[IMX568_RATE_594]	= 297000000,
	[IMX568_RATE_891]	= 445500000,
	[IMX568_RATE_1188]	= 594000000,
};

struct imx568_link_profile {
	u32 lane_rate;
	const struct cci_reg_sequence *reg_list;
	unsigned int num_regs;
};

/* IMX568-AAQJ-C sections 18 and 19.8, with a 37.125 MHz INCK. */
static const struct cci_reg_sequence imx568_594mbps_regs[] = {
	{ IMX568_INCKSEL_D2, 0x80 },
	{ IMX568_INCKSEL_D3, 0x90 },
	{ IMX568_THS_PREPARE, 0x002f },
	{ IMX568_TCLK_POST, 0x0067 },
	{ IMX568_THS_TRAIL, 0x002f },
	{ IMX568_THS_ZERO, 0x004f },
	{ IMX568_TCLK_PREPARE, 0x0027 },
	{ IMX568_TCLK_TRAIL, 0x0027 },
	{ IMX568_TLPX, 0x0027 },
	{ IMX568_TCLK_ZERO, 0x00b7 },
	{ IMX568_TCLK_PRE, 0x000f },
	{ IMX568_THS_EXIT, 0x0047 },
};

static const struct cci_reg_sequence imx568_891mbps_regs[] = {
	{ IMX568_INCKSEL_D2, 0xc0 },
	{ IMX568_INCKSEL_D3, 0xd0 },
	{ IMX568_THS_PREPARE, 0x003f },
	{ IMX568_TCLK_POST, 0x007f },
	{ IMX568_THS_TRAIL, 0x003f },
	{ IMX568_THS_ZERO, 0x006f },
	{ IMX568_TCLK_PREPARE, 0x0037 },
	{ IMX568_TCLK_TRAIL, 0x0037 },
	{ IMX568_TLPX, 0x002f },
	{ IMX568_TCLK_ZERO, 0x00f7 },
	{ IMX568_TCLK_PRE, 0x000f },
	{ IMX568_THS_EXIT, 0x005f },
};

static const struct cci_reg_sequence imx568_1188mbps_regs[] = {
	{ IMX568_INCKSEL_D2, 0x80 },
	{ IMX568_INCKSEL_D3, 0x80 },
	{ IMX568_THS_PREPARE, 0x004f },
	{ IMX568_TCLK_POST, 0x008f },
	{ IMX568_THS_TRAIL, 0x004f },
	{ IMX568_THS_ZERO, 0x0087 },
	{ IMX568_TCLK_PREPARE, 0x004f },
	{ IMX568_TCLK_TRAIL, 0x0047 },
	{ IMX568_TLPX, 0x003f },
	{ IMX568_TCLK_ZERO, 0x0137 },
	{ IMX568_TCLK_PRE, 0x000f },
	{ IMX568_THS_EXIT, 0x007f },
};

static const struct imx568_link_profile imx568_link_profiles[] = {
	[IMX568_RATE_594] = {
		.lane_rate = IMX568_LANE_RATE_594,
		.reg_list = imx568_594mbps_regs,
		.num_regs = ARRAY_SIZE(imx568_594mbps_regs),
	},
	[IMX568_RATE_891] = {
		.lane_rate = IMX568_LANE_RATE_891,
		.reg_list = imx568_891mbps_regs,
		.num_regs = ARRAY_SIZE(imx568_891mbps_regs),
	},
	[IMX568_RATE_1188] = {
		.lane_rate = IMX568_LANE_RATE_1188,
		.reg_list = imx568_1188mbps_regs,
		.num_regs = ARRAY_SIZE(imx568_1188mbps_regs),
	},
};

struct imx568_mode {
	struct v4l2_mbus_framefmt	format;
	struct v4l2_rect		crop;
	u32				frame_clocks_594;
	u16				fixed_hmax_2lane;
	u16				fixed_hmax_4lane;
	u16				fixed_vmax;
	const struct cci_reg_sequence	*reg_list;
	unsigned int			num_regs;
};

struct imx568_selected_timing {
	const struct imx568_link_profile *profile;
	const struct imx568_timing	*rate_timing;
	u32				hts;
	u32				vts;
	u32				vblank_min;
	u32				link_freq_index;
};

struct imx568 {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct mutex			lock;
	struct clk			*xclk;
	struct regmap			*regmap;
	struct gpio_desc		*pwdn;
	struct regulator_bulk_data supplies[IMX568_NUM_SUPPLIES];
	struct v4l2_ctrl_handler	ctrls;
	const struct imx568_mode	*mode;
	struct v4l2_ctrl		*pixel_rate;
	struct v4l2_ctrl		*hblank;
	struct v4l2_ctrl		*vblank;
	struct v4l2_ctrl		*exposure;
	struct v4l2_ctrl		*hflip;
	struct v4l2_ctrl		*vflip;
	struct v4l2_ctrl		*link_freq;
	const struct imx568_link_profile *link_profile;
	const struct imx568_timing	*rate_timing;
	u32				hts;
	u32				vts;
	u32				vblank_min;
	u32				link_freq_index;
	unsigned int			link_rate_mask;
	u8				lanes;	/* CSI-2 data lanes: 2 (LI SoM) or 4 (DVT) */
	/* Closes the gap before the V4L2 core updates s_stream_enabled. */
	bool				streaming;
};

/* AD readout rows of the 1188 Mbit/s, 2-lane, RAW12 row the 86-line reserve belongs to. */
#define IMX568_REF_AD_LINES		(IMX568_GMRWT_1188_2LANE_RAW12 + \
					 IMX568_GMTWT_1188_2LANE_RAW12 + \
					 IMX568_GSDLY_1188_2LANE_RAW12)

/* Readout reserve: 86 lines at 2 lanes, plus the extra AD readout rows a 4-lane row carries. */
static u32 imx568_readout_lines(const struct imx568_timing *timing, u8 lanes)
{
	u32 ad_lines = timing->gmrwt + timing->gmtwt + timing->gsdly;

	if (lanes != 4 || ad_lines <= IMX568_REF_AD_LINES)
		return IMX568_SHS_READOUT_LINES;
	return IMX568_SHS_READOUT_LINES + ad_lines - IMX568_REF_AD_LINES;
}

/* Vertical blank reserved for the readout, in exposure control units. */
static int imx568_exp_readout_overhead(const struct imx568 *sensor)
{
	return imx568_readout_lines(sensor->rate_timing, sensor->lanes) + 1 -
	       IMX568_EXPOSURE_MIN;
}

static inline struct imx568 *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx568, sd);
}

static const char * const imx568_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Color Squares",
	"Random Data",
};

/* Common analog/PLL/MIPI base written before every mode. */
static const struct cci_reg_sequence imx568_common_regs[] = {
	{ IMX568_INCKSEL_ST0, 0x05 },
	{ IMX568_INCKSEL_ST1, 0x91 },
	{ IMX568_INCKSEL_ST2, 0x50 },
	{ IMX568_INCKSEL_ST3, 0x20 },
	{ IMX568_INCKSEL_ST4, 0x02 },
	{ IMX568_INCKSEL_ST5, 0x1d },
	{ CCI_REG8(0x30d6), 0x00 },	/* VMAX[23:16] */
	{ IMX568_GSDLY, 0x08 },
	{ IMX568_ROI_MODE, 0x00 },
	{ IMX568_FID0_ROION, 0x03 },
	/* 37.125 MHz INCK-only dividers; D2/D3 are selected per lane rate. */
	{ IMX568_INCKSEL_N0, 0x40 },
	{ IMX568_INCKSEL_N1, 0x05 },
	{ IMX568_INCKSEL_N2, 0xe0 },
	{ IMX568_INCKSEL_N3, 0x00 },
	{ IMX568_INCKSEL_D0, 0x40 },
	{ IMX568_INCKSEL_D1, 0x14 },
	{ IMX568_VINT_EN, 0x30 },
	{ CCI_REG8(0x3240), 0x14 },	/* SHS[7:0] preset */
	{ IMX568_PULSE2_EN, 0x20 },
	{ IMX568_GAIN_RTS, 0x08 },
	{ CCI_REG8(0x3542), 0x27 },
	{ CCI_REG8(0x354a), 0x20 },
	{ CCI_REG8(0x359c), 0x0f },
	{ CCI_REG8(0x35a4), 0x30 },
	{ CCI_REG8(0x35a5), 0x12 },
	{ CCI_REG8(0x35a8), 0x30 },
	{ CCI_REG8(0x35a9), 0x42 },
	{ CCI_REG8(0x35ac), 0x62 },
	{ IMX568_BLKLEVEL, 0xf0 },
	{ CCI_REG8(0x35b6), 0x02 },
	{ CCI_REG8(0x35ec), 0x30 },
	{ CCI_REG8(0x35ed), 0x12 },
	{ CCI_REG8(0x35f0), 0xfb },
	{ CCI_REG8(0x35f1), 0x0b },
	{ CCI_REG8(0x35f2), 0xfb },
	{ CCI_REG8(0x35f3), 0x0b },
	{ CCI_REG8(0x3797), 0x20 },
	{ IMX568_LANESEL, 0x03 },
	{ IMX568_EAV_SEL_MIPI, 0x03 },
	{ IMX568_MIPI_CLK_CONT, 0x00 },
	{ IMX568_TXCLKESC_FREQ, 0x0980 },
	{ CCI_REG8(0x3e30), 0x4e },
	{ CCI_REG8(0x3e96), 0x01 },
	{ CCI_REG8(0x3ea0), 0x4c },
	{ CCI_REG8(0x3f3a), 0x04 },
	{ CCI_REG8(0x4056), 0x23 },
	{ CCI_REG8(0x4096), 0x23 },
	{ CCI_REG8(0x4182), 0x00 },
	{ CCI_REG8(0x41a2), 0x03 },
	{ CCI_REG8(0x4232), 0x3c },
	{ CCI_REG8(0x4306), 0x00 },
	{ CCI_REG8(0x4307), 0x00 },
	{ CCI_REG8(0x4308), 0x00 },
	{ CCI_REG8(0x4309), 0x00 },
	{ CCI_REG8(0x4310), 0x04 },
	{ CCI_REG8(0x4311), 0x04 },
	{ CCI_REG8(0x4312), 0x04 },
	{ CCI_REG8(0x4313), 0x04 },
	{ CCI_REG8(0x4467), 0x83 },
	{ CCI_REG8(0x4749), 0x9f },
	{ CCI_REG8(0x474a), 0x99 },
	{ CCI_REG8(0x474b), 0x09 },
	{ CCI_REG8(0x4788), 0x04 },
	{ CCI_REG8(0x479c), 0x40 },
	{ CCI_REG8(0x4864), 0xdc },
	{ CCI_REG8(0x4868), 0xdc },
	{ CCI_REG8(0x486c), 0xdc },
	{ CCI_REG8(0x48a4), 0xf4 },
	{ CCI_REG8(0x48a8), 0xf4 },
	{ CCI_REG8(0x48ac), 0xf4 },
};

/*
 * Per-mode deltas over imx568_common_regs. The trailing 1 ms settle delay
 * (formerly a WAIT pseudo-register) is applied in imx568_set_mode().
 * VMAX[15:0] is written here as CCI_REG16_LE(0x30d4); VMAX[23:16] comes from
 * imx568_common_regs and the full 24-bit VMAX is driven at runtime via VBLANK.
 */

/* 1920x1080 12-bit ROI (SRGGB12). */
static const struct cci_reg_sequence imx568_1920x1080_12bpp[] = {
	{ IMX568_HWIDTH, 0x0780 },		/* VOPB_VBLK_HWIDTH (=1920) */
	{ IMX568_FINFO_HWIDTH, 0x0780 },	/* FINFO_HWIDTH (=1920) */
	{ CCI_REG16_LE(0x30d4), 0x0a02 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0110 },		/* ROIPH1 (=272, centered) */
	{ IMX568_ROI_PV1, 0x01ec },		/* ROIPV1 (=492, centered) */
	{ IMX568_ROI_WH1, 0x0780 },
	{ IMX568_ROI_WV1, 0x0438 },
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 2432x2048 all-pixel 12-bit ROI (30 fps, lower-rate readout may derate). */
static const struct cci_reg_sequence imx568_2432x2048_12bpp[] = {
	{ IMX568_HWIDTH, 0x0980 },		/* VOPB_VBLK_HWIDTH (=2432) */
	{ IMX568_FINFO_HWIDTH, 0x0980 },	/* FINFO_HWIDTH (=2432) */
	{ CCI_REG16_LE(0x30d4), 0x08ca },	/* VMAX[15:0] (=2250) */
	{ IMX568_HMAX, IMX568_FULL_HTS },
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0010 },		/* ROIPH1 (=16, centre-crop of 2448) */
	{ IMX568_ROI_PV1, 0x0008 },		/* ROIPV1 (=8) */
	{ IMX568_ROI_WH1, 0x0980 },		/* ROIWH1 (=2432) */
	{ IMX568_ROI_WV1, 0x0800 },		/* ROIWV1 (=2048) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 1280x720 12-bit ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_1280x720_12bpp[] = {
	{ IMX568_HWIDTH, 0x0500 },		/* VOPB_VBLK_HWIDTH (=1280) */
	{ IMX568_FINFO_HWIDTH, 0x0500 },	/* FINFO_HWIDTH (=1280) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0250 },		/* ROIPH1 (=592) */
	{ IMX568_ROI_PV1, 0x02a0 },		/* ROIPV1 (=672) */
	{ IMX568_ROI_WH1, 0x0500 },		/* ROIWH1 (=1280) */
	{ IMX568_ROI_WV1, 0x02d0 },		/* ROIWV1 (=720) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 640x360 12-bit ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_640x360_12bpp[] = {
	{ IMX568_HWIDTH, 0x0280 },		/* VOPB_VBLK_HWIDTH (=640) */
	{ IMX568_FINFO_HWIDTH, 0x0280 },	/* FINFO_HWIDTH (=640) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0390 },		/* ROIPH1 (=912) */
	{ IMX568_ROI_PV1, 0x0354 },		/* ROIPV1 (=852) */
	{ IMX568_ROI_WH1, 0x0280 },		/* ROIWH1 (=640) */
	{ IMX568_ROI_WV1, 0x0168 },		/* ROIWV1 (=360) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 2432x1368 12-bit ROI crop: full-width 16:9, top/bottom cropped (~30 fps). */
static const struct cci_reg_sequence imx568_2432x1368_12bpp[] = {
	{ IMX568_HWIDTH, 0x0980 },		/* VOPB_VBLK_HWIDTH (=2432) */
	{ IMX568_FINFO_HWIDTH, 0x0980 },	/* FINFO_HWIDTH (=2432) */
	{ CCI_REG16_LE(0x30d4), 0x08ca },	/* VMAX[15:0] (=2250) */
	{ IMX568_HMAX, 0x044c },		/* HMAX (=1100) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0010 },		/* ROIPH1 (=16, full width) */
	{ IMX568_ROI_PV1, 0x015c },		/* ROIPV1 (=348, centered) */
	{ IMX568_ROI_WH1, 0x0980 },		/* ROIWH1 (=2432) */
	{ IMX568_ROI_WV1, 0x0558 },		/* ROIWV1 (=1368) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 1024x768 12-bit centered ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_1024x768_12bpp[] = {
	{ IMX568_HWIDTH, 0x0400 },		/* VOPB_VBLK_HWIDTH (=1024) */
	{ IMX568_FINFO_HWIDTH, 0x0400 },	/* FINFO_HWIDTH (=1024) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x02d0 },		/* ROIPH1 (=720, centered) */
	{ IMX568_ROI_PV1, 0x0288 },		/* ROIPV1 (=648, centered) */
	{ IMX568_ROI_WH1, 0x0400 },		/* ROIWH1 (=1024) */
	{ IMX568_ROI_WV1, 0x0300 },		/* ROIWV1 (=768) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 800x600 12-bit centered ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_800x600_12bpp[] = {
	{ IMX568_HWIDTH, 0x0320 },		/* VOPB_VBLK_HWIDTH (=800) */
	{ IMX568_FINFO_HWIDTH, 0x0320 },	/* FINFO_HWIDTH (=800) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0340 },		/* ROIPH1 (=832, centered) */
	{ IMX568_ROI_PV1, 0x02dc },		/* ROIPV1 (=732, centered) */
	{ IMX568_ROI_WH1, 0x0320 },		/* ROIWH1 (=800) */
	{ IMX568_ROI_WV1, 0x0258 },		/* ROIWV1 (=600) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 640x480 12-bit centered ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_640x480_12bpp[] = {
	{ IMX568_HWIDTH, 0x0280 },		/* VOPB_VBLK_HWIDTH (=640) */
	{ IMX568_FINFO_HWIDTH, 0x0280 },	/* FINFO_HWIDTH (=640) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0390 },		/* ROIPH1 (=912, centered) */
	{ IMX568_ROI_PV1, 0x0318 },		/* ROIPV1 (=792, centered) */
	{ IMX568_ROI_WH1, 0x0280 },		/* ROIWH1 (=640) */
	{ IMX568_ROI_WV1, 0x01e0 },		/* ROIWV1 (=480) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 1280x960 12-bit centered ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_1280x960_12bpp[] = {
	{ IMX568_HWIDTH, 0x0500 },		/* VOPB_VBLK_HWIDTH (=1280) */
	{ IMX568_FINFO_HWIDTH, 0x0500 },	/* FINFO_HWIDTH (=1280) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x0250 },		/* ROIPH1 (=592, centered) */
	{ IMX568_ROI_PV1, 0x0228 },		/* ROIPV1 (=552, centered) */
	{ IMX568_ROI_WH1, 0x0500 },		/* ROIWH1 (=1280) */
	{ IMX568_ROI_WV1, 0x03c0 },		/* ROIWV1 (=960) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 1600x1200 12-bit centered ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_1600x1200_12bpp[] = {
	{ IMX568_HWIDTH, 0x0640 },		/* VOPB_VBLK_HWIDTH (=1600) */
	{ IMX568_FINFO_HWIDTH, 0x0640 },	/* FINFO_HWIDTH (=1600) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x01b0 },		/* ROIPH1 (=432, centered) */
	{ IMX568_ROI_PV1, 0x01b0 },		/* ROIPV1 (=432, centered) */
	{ IMX568_ROI_WH1, 0x0640 },		/* ROIWH1 (=1600) */
	{ IMX568_ROI_WV1, 0x04b0 },		/* ROIWV1 (=1200) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 2048x1536 12-bit centered ROI crop (~30 fps default). */
static const struct cci_reg_sequence imx568_2048x1536_12bpp[] = {
	{ IMX568_HWIDTH, 0x0800 },		/* VOPB_VBLK_HWIDTH (=2048) */
	{ IMX568_FINFO_HWIDTH, 0x0800 },	/* FINFO_HWIDTH (=2048) */
	{ CCI_REG16_LE(0x30d4), 0x0a05 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x02 },
	{ IMX568_GMTWT, 0x10 },
	{ IMX568_ROI_PH1, 0x00d0 },		/* ROIPH1 (=208, centered) */
	{ IMX568_ROI_PV1, 0x0108 },		/* ROIPV1 (=264, centered) */
	{ IMX568_ROI_WH1, 0x0800 },		/* ROIWH1 (=2048) */
	{ IMX568_ROI_WV1, 0x0600 },		/* ROIWV1 (=1536) */
	{ IMX568_ADBIT, 0x15 },
	{ IMX568_ODBIT, 0x01 },
};

/* 1920x1080 10-bit ROI (SRGGB10). */
static const struct cci_reg_sequence imx568_1920x1080_10bpp[] = {
	{ IMX568_HWIDTH, 0x0780 },		/* VOPB_VBLK_HWIDTH (=1920) */
	{ IMX568_FINFO_HWIDTH, 0x0780 },	/* FINFO_HWIDTH (=1920) */
	{ CCI_REG16_LE(0x30d4), 0x0a02 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x04 },
	{ IMX568_GMTWT, 0x12 },
	{ IMX568_ROI_PH1, 0x0110 },		/* ROIPH1 (=272, centered) */
	{ IMX568_ROI_PV1, 0x01ec },		/* ROIPV1 (=492, centered) */
	{ IMX568_ROI_WH1, 0x0780 },		/* ROIWH1 (=1920) */
	{ IMX568_ROI_WV1, 0x0438 },		/* ROIWV1 (=1080) */
	{ IMX568_ADBIT, 0x05 },			/* ADBIT[5:4]=0 -> 10-bit */
	{ IMX568_ODBIT, 0x00 },			/* ODBIT[1:0]=0 -> 10-bit */
};

/* 1920x1080 8-bit ROI (SRGGB8). */
static const struct cci_reg_sequence imx568_1920x1080_8bpp[] = {
	{ IMX568_HWIDTH, 0x0780 },		/* VOPB_VBLK_HWIDTH (=1920) */
	{ IMX568_FINFO_HWIDTH, 0x0780 },	/* FINFO_HWIDTH (=1920) */
	{ CCI_REG16_LE(0x30d4), 0x0a02 },	/* VMAX[15:0] */
	{ IMX568_HMAX, 0x03c5 },		/* HMAX (=965) */
	{ IMX568_GMRWT, 0x04 },			/* 8-bit/2-lane/1188Mbps */
	{ IMX568_GMTWT, 0x16 },			/* 8-bit/2-lane/1188Mbps */
	{ IMX568_GSDLY, 0x0a },			/* 8-bit/2-lane/1188Mbps */
	{ IMX568_ROI_PH1, 0x0110 },		/* ROIPH1 (=272, centered) */
	{ IMX568_ROI_PV1, 0x01ec },		/* ROIPV1 (=492, centered) */
	{ IMX568_ROI_WH1, 0x0780 },		/* ROIWH1 (=1920) */
	{ IMX568_ROI_WV1, 0x0438 },		/* ROIWV1 (=1080) */
	{ IMX568_ADBIT, 0x25 },			/* ADBIT[5:4]=2 -> 8-bit */
	{ IMX568_ODBIT, 0x02 },			/* ODBIT[1:0]=2 -> 8-bit */
};

static const struct imx568_mode imx568_modes[] = {
	/* 1920x1080 12-bit ROI mode. */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_PIXEL_ARRAY_WIDTH,
			.height		= IMX568_PIXEL_ARRAY_HEIGHT
		},
		.crop = {
			.left		= IMX568_PIXEL_ARRAY_LEFT,
			.top		= IMX568_PIXEL_ARRAY_TOP,
			.width		= IMX568_PIXEL_ARRAY_WIDTH,
			.height		= IMX568_PIXEL_ARRAY_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_HTS,
		.fixed_hmax_4lane = IMX568_HTS_4LANE,
		.fixed_vmax	= IMX568_VTS,
		.reg_list	= imx568_1920x1080_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_1920x1080_12bpp)
	},
	/* 2432x2048 RAW12: 30 fps at the 594 Mbit/s period. */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_FULL_WIDTH,
			.height		= IMX568_FULL_HEIGHT
		},
		.crop = {
			.left		= IMX568_FULL_LEFT,
			.top		= IMX568_FULL_TOP,
			.width		= IMX568_FULL_WIDTH,
			.height		= IMX568_FULL_HEIGHT
		},
		.frame_clocks_594 =
			IMX568_HMAX_594_2LANE_RAW12 * IMX568_FULL_VTS,
		.fixed_hmax_2lane = IMX568_FULL_HTS,
		.fixed_hmax_4lane = IMX568_FULL_HTS_4LANE,
		.fixed_vmax	= IMX568_FULL_VTS,
		.reg_list	= imx568_2432x2048_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_2432x2048_12bpp)
	},
	/* 1280x720 12-bit ROI crop (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_HD_WIDTH,
			.height		= IMX568_HD_HEIGHT
		},
		.crop = {
			.left		= IMX568_HD_LEFT,
			.top		= IMX568_HD_TOP,
			.width		= IMX568_HD_WIDTH,
			.height		= IMX568_HD_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_1280x720_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_1280x720_12bpp)
	},
	/* 640x360 12-bit ROI crop (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_QVGA_WIDTH,
			.height		= IMX568_QVGA_HEIGHT
		},
		.crop = {
			.left		= IMX568_QVGA_LEFT,
			.top		= IMX568_QVGA_TOP,
			.width		= IMX568_QVGA_WIDTH,
			.height		= IMX568_QVGA_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_640x360_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_640x360_12bpp)
	},
	/* 2432x1368 RAW12: 30 fps native, ~25 fps at 594 Mbit/s. */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_W169_WIDTH,
			.height		= IMX568_W169_HEIGHT
		},
		.crop = {
			.left		= IMX568_W169_LEFT,
			.top		= IMX568_W169_TOP,
			.width		= IMX568_W169_WIDTH,
			.height		= IMX568_W169_HEIGHT
		},
		.frame_clocks_594 =
			IMX568_W169_594_FRAME_CLOCKS,
		.fixed_hmax_2lane = IMX568_FULL_HTS,
		.fixed_hmax_4lane = IMX568_FULL_HTS_4LANE,
		.fixed_vmax	= IMX568_FULL_VTS,
		.reg_list	= imx568_2432x1368_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_2432x1368_12bpp)
	},
	/* 1024x768 12-bit centered ROI (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_XGA_WIDTH,
			.height		= IMX568_XGA_HEIGHT
		},
		.crop = {
			.left		= IMX568_XGA_LEFT,
			.top		= IMX568_XGA_TOP,
			.width		= IMX568_XGA_WIDTH,
			.height		= IMX568_XGA_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_1024x768_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_1024x768_12bpp)
	},
	/* 800x600 12-bit centered ROI (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_SVGA_WIDTH,
			.height		= IMX568_SVGA_HEIGHT
		},
		.crop = {
			.left		= IMX568_SVGA_LEFT,
			.top		= IMX568_SVGA_TOP,
			.width		= IMX568_SVGA_WIDTH,
			.height		= IMX568_SVGA_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_800x600_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_800x600_12bpp)
	},
	/* 640x480 12-bit centered ROI (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_VGA_WIDTH,
			.height		= IMX568_VGA_HEIGHT
		},
		.crop = {
			.left		= IMX568_VGA_LEFT,
			.top		= IMX568_VGA_TOP,
			.width		= IMX568_VGA_WIDTH,
			.height		= IMX568_VGA_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_640x480_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_640x480_12bpp)
	},
	/* 1280x960 12-bit centered ROI (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_SXGA_WIDTH,
			.height		= IMX568_SXGA_HEIGHT
		},
		.crop = {
			.left		= IMX568_SXGA_LEFT,
			.top		= IMX568_SXGA_TOP,
			.width		= IMX568_SXGA_WIDTH,
			.height		= IMX568_SXGA_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_1280x960_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_1280x960_12bpp)
	},
	/* 1600x1200 12-bit centered ROI (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_UXGA_WIDTH,
			.height		= IMX568_UXGA_HEIGHT
		},
		.crop = {
			.left		= IMX568_UXGA_LEFT,
			.top		= IMX568_UXGA_TOP,
			.width		= IMX568_UXGA_WIDTH,
			.height		= IMX568_UXGA_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_1600x1200_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_1600x1200_12bpp)
	},
	/* 2048x1536 12-bit centered ROI (~30 fps default). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB12_1X12,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_QXGA_WIDTH,
			.height		= IMX568_QXGA_HEIGHT
		},
		.crop = {
			.left		= IMX568_QXGA_LEFT,
			.top		= IMX568_QXGA_TOP,
			.width		= IMX568_QXGA_WIDTH,
			.height		= IMX568_QXGA_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_CROP_HTS,
		.fixed_hmax_4lane = IMX568_CROP_HTS_4LANE,
		.fixed_vmax	= IMX568_CROP_VTS,
		.reg_list	= imx568_2048x1536_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_2048x1536_12bpp)
	},
	/* 1920x1080 10-bit ROI mode (SRGGB10). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB10_1X10,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_PIXEL_ARRAY_WIDTH,
			.height		= IMX568_PIXEL_ARRAY_HEIGHT
		},
		.crop = {
			.left		= IMX568_PIXEL_ARRAY_LEFT,
			.top		= IMX568_PIXEL_ARRAY_TOP,
			.width		= IMX568_PIXEL_ARRAY_WIDTH,
			.height		= IMX568_PIXEL_ARRAY_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_HTS,
		.fixed_hmax_4lane = IMX568_HTS_4LANE,
		.fixed_vmax	= IMX568_VTS,
		.reg_list	= imx568_1920x1080_10bpp,
		.num_regs	= ARRAY_SIZE(imx568_1920x1080_10bpp)
	},
	/* 1920x1080 8-bit ROI mode (SRGGB8). */
	{
		.format = {
			.code		= MEDIA_BUS_FMT_SRGGB8_1X8,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.field		= V4L2_FIELD_NONE,
			.width		= IMX568_PIXEL_ARRAY_WIDTH,
			.height		= IMX568_PIXEL_ARRAY_HEIGHT
		},
		.crop = {
			.left		= IMX568_PIXEL_ARRAY_LEFT,
			.top		= IMX568_PIXEL_ARRAY_TOP,
			.width		= IMX568_PIXEL_ARRAY_WIDTH,
			.height		= IMX568_PIXEL_ARRAY_HEIGHT
		},
		.fixed_hmax_2lane = IMX568_HTS,
		.fixed_hmax_4lane = IMX568_HTS_4LANE,
		.fixed_vmax	= IMX568_VTS,
		.reg_list	= imx568_1920x1080_8bpp,
		.num_regs	= ARRAY_SIZE(imx568_1920x1080_8bpp)
	},
};

/* Default sensor mode: 1920x1080 10-bit centered ROI; [0] is its 12-bit twin. */
#define IMX568_DEFAULT_MODE_IDX	11
#define IMX568_DEFAULT_MODE	(&imx568_modes[IMX568_DEFAULT_MODE_IDX])
#define IMX568_DEFAULT_FORMAT	(imx568_modes[IMX568_DEFAULT_MODE_IDX].format)

static unsigned int imx568_mode_bpp(const struct imx568_mode *mode)
{
	switch (mode->format.code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return 8;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	default:
		return 12;
	}
}

static const struct imx568_timing *
imx568_profile_timing(const struct imx568_link_profile *profile,
		      const struct imx568_mode *mode, u8 lanes)
{
	return imx568_timing_find(profile->lane_rate, imx568_mode_bpp(mode),
				  lanes);
}

static u64 imx568_mode_pixel_rate(const struct imx568 *sensor,
				  const struct imx568_mode *mode)
{
	return imx568_pixel_rate(sensor->link_profile->lane_rate, sensor->lanes,
				 imx568_mode_bpp(mode));
}

static u32 imx568_mode_line_length(const struct imx568 *sensor,
				   const struct imx568_mode *mode)
{
	return imx568_hmax_to_line_length(sensor->hts,
					  sensor->link_profile->lane_rate,
					  sensor->lanes,
					  imx568_mode_bpp(mode));
}

/* Fastest endpoint-supported profile that carries the active line. */
static int imx568_find_timing(const struct imx568 *sensor,
			      const struct imx568_mode *mode,
			      struct imx568_selected_timing *selected)
{
	int i;

	for (i = ARRAY_SIZE(imx568_link_profiles) - 1; i >= 0; i--) {
		const struct imx568_link_profile *candidate =
			&imx568_link_profiles[i];
		const struct imx568_timing *candidate_timing;
		u64 minimum_frame_clocks;
		u64 requested_frame_clocks;
		u64 target_frame_clocks;
		u64 payload_capacity;
		u64 payload_clocks;
		u64 candidate_vts;
		u32 candidate_vblank;
		u32 candidate_hmax;

		if (!(sensor->link_rate_mask & BIT(i)))
			continue;
		candidate_timing = imx568_profile_timing(candidate, mode,
							 sensor->lanes);
		if (!candidate_timing)
			continue;
		candidate_vblank = IMX568_ROI_VBLANK_LINES +
			candidate_timing->gmrwt + candidate_timing->gmtwt +
			candidate_timing->gsdly;
		/* 1188 Mbit/s modes keep their per-lane HMAX; other rates use the row. */
		if (candidate->lane_rate == IMX568_LANE_RATE_1188)
			candidate_hmax = sensor->lanes == 4 ?
				mode->fixed_hmax_4lane : mode->fixed_hmax_2lane;
		else
			candidate_hmax = candidate_timing->hmax;

		/* Validate peak active-line payload, not frame-averaged payload. */
		payload_clocks = ((u64)mode->format.width *
			imx568_mode_bpp(mode) + 64) * IMX568_HMAX_CLK_HZ;
		payload_capacity = (u64)candidate_hmax * sensor->lanes *
			candidate->lane_rate;
		if (payload_clocks > payload_capacity ||
		    candidate_hmax < candidate_timing->hmax)
			continue;

		minimum_frame_clocks = (u64)candidate_hmax *
			(mode->format.height + candidate_vblank);
		/* One native period per mode; 2432-wide modes keep the 594 Mbit/s period. */
		requested_frame_clocks =
			(u64)mode->fixed_hmax_2lane * mode->fixed_vmax;
		if (candidate->lane_rate == IMX568_LANE_RATE_594)
			requested_frame_clocks =
				max(requested_frame_clocks,
				    (u64)mode->frame_clocks_594);
		target_frame_clocks = max(requested_frame_clocks,
					  minimum_frame_clocks);
		candidate_vts = DIV_ROUND_UP_ULL(target_frame_clocks,
						 candidate_hmax);
		if (candidate_vts > IMX568_VTS_MAX)
			continue;

		selected->profile = candidate;
		selected->rate_timing = candidate_timing;
		selected->hts = candidate_hmax;
		selected->vts = candidate_vts;
		selected->vblank_min = candidate_vblank;
		selected->link_freq_index = i;
		return 0;
	}

	return -EINVAL;
}

static void imx568_apply_timing(struct imx568 *sensor,
				const struct imx568_selected_timing *selected)
{
	sensor->link_profile = selected->profile;
	sensor->rate_timing = selected->rate_timing;
	sensor->hts = selected->hts;
	sensor->vts = selected->vts;
	sensor->vblank_min = selected->vblank_min;
	sensor->link_freq_index = selected->link_freq_index;
}

static const struct imx568_mode *
imx568_find_mode(const struct imx568 *sensor,
		 const struct v4l2_mbus_framefmt *format,
		 struct imx568_selected_timing *selected)
{
	struct imx568_selected_timing candidate;
	const struct imx568_mode *best_mode;
	unsigned int pass;
	unsigned int i;

	/* Prefer the requested media-bus code, then adjust across all modes. */
	for (pass = 0; pass < 2; pass++) {
		u64 best = 0;

		best_mode = NULL;
		for (i = 0; i < ARRAY_SIZE(imx568_modes); i++) {
			u64 distance;

			if (!pass && imx568_modes[i].format.code != format->code)
				continue;
			if (imx568_find_timing(sensor, &imx568_modes[i],
					       &candidate))
				continue;

			distance =
				(u64)abs_diff(imx568_modes[i].format.width,
					      format->width) +
				abs_diff(imx568_modes[i].format.height,
					 format->height);
			if (best_mode && distance >= best)
				continue;

			best = distance;
			best_mode = &imx568_modes[i];
			*selected = candidate;
		}
		if (best_mode)
			return best_mode;
	}

	return NULL;
}

static int imx568_set_mode(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx568 *sensor = to_sensor(sd);
	int ret = 0;

	cci_multi_reg_write(sensor->regmap, imx568_common_regs,
			    ARRAY_SIZE(imx568_common_regs), &ret);
	if (ret) {
		dev_err(&client->dev, "write sensor common regs error\n");
		return ret;
	}

	cci_multi_reg_write(sensor->regmap, sensor->mode->reg_list,
			    sensor->mode->num_regs, &ret);
	if (ret) {
		dev_err(&client->dev, "write sensor mode regs error\n");
		return ret;
	}

	cci_multi_reg_write(sensor->regmap, sensor->link_profile->reg_list,
			    sensor->link_profile->num_regs, &ret);
	cci_write(sensor->regmap, IMX568_LANESEL,
		  sensor->lanes == 4 ? 0x02 : 0x03, &ret);
	cci_write(sensor->regmap, IMX568_HMAX, sensor->hts, &ret);
	cci_write(sensor->regmap, IMX568_GMRWT, sensor->rate_timing->gmrwt, &ret);
	cci_write(sensor->regmap, IMX568_GMTWT, sensor->rate_timing->gmtwt, &ret);
	cci_write(sensor->regmap, IMX568_GSDLY, sensor->rate_timing->gsdly, &ret);
	if (ret) {
		dev_err(&client->dev, "write sensor link timing error\n");
		return ret;
	}

	/* Per-mode settle delay (formerly the trailing WAIT in each table). */
	msleep(1);

	return 0;
}

static int imx568_stream_on(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx568 *sensor = to_sensor(sd);
	int ret;

	ret = imx568_set_mode(sd);
	if (ret) {
		dev_err(&client->dev, "Failed to program sensor mode: %d\n", ret);
		return ret;
	}

	/* Apply customized values from user when stream starts. */
	ret = __v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (ret)
		return ret;

	/* Leave standby, then start master mode. */
	ret = cci_write(sensor->regmap, IMX568_SW_STANDBY, 0x00, NULL);
	if (ret)
		return ret;
	msleep(250);
	ret = cci_write(sensor->regmap, IMX568_XMSTA, 0x00, NULL);
	if (ret)
		return ret;
	msleep(50);

	return 0;
}

static int imx568_stream_off(struct v4l2_subdev *sd)
{
	struct imx568 *sensor = to_sensor(sd);
	int ret = 0;

	/* Enter standby, then stop master mode. */
	cci_write(sensor->regmap, IMX568_SW_STANDBY, 0x01, &ret);
	msleep(200);
	cci_write(sensor->regmap, IMX568_XMSTA, 0x01, &ret);
	msleep(50);

	return ret;
}

static int imx568_power_on(struct device *dev)
{
	struct imx568 *sensor = to_sensor(dev_get_drvdata(dev));
	int ret;

	dev_dbg(dev, "IMX568 power on\n");

	ret = regulator_bulk_enable(IMX568_NUM_SUPPLIES, sensor->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		return ret;
	}

	if (sensor->pwdn) {
		gpiod_set_value_cansleep(sensor->pwdn, 0);
		msleep(PWDN_ACTIVE_DELAY_MS);
	}

	ret = clk_prepare_enable(sensor->xclk);
	if (ret < 0) {
		dev_err(dev, "clk prepare enable failed\n");
		goto error_pwdn;
	}

	/* Stream off to coax lanes into LP-11 state. */
	ret = imx568_stream_off(&sensor->sd);
	if (ret < 0) {
		dev_err(dev, "camera not available, check power\n");
		goto error_clk_disable;
	}

	return 0;

error_clk_disable:
	clk_disable_unprepare(sensor->xclk);
error_pwdn:
	gpiod_set_value_cansleep(sensor->pwdn, 1);
	regulator_bulk_disable(IMX568_NUM_SUPPLIES, sensor->supplies);

	return ret;
}

static int imx568_power_off(struct device *dev)
{
	struct imx568 *sensor = to_sensor(dev_get_drvdata(dev));
	int ret;

	dev_dbg(dev, "IMX568 power off\n");

	ret = imx568_stream_off(&sensor->sd);
	if (ret < 0)
		dev_err(dev, "software standby failed\n");

	msleep(10);
	clk_disable_unprepare(sensor->xclk);
	gpiod_set_value_cansleep(sensor->pwdn, 1);
	regulator_bulk_disable(IMX568_NUM_SUPPLIES, sensor->supplies);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int imx568_sensor_get_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	struct imx568 *sensor = to_sensor(sd);
	u64 val;
	int ret;

	ret = cci_read(sensor->regmap, CCI_REG8(reg->reg), &val, NULL);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int imx568_sensor_set_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	struct imx568 *sensor = to_sensor(sd);

	return cci_write(sensor->regmap, CCI_REG8(reg->reg), reg->val, NULL);
}
#endif

/* Subdev core operations registration */
static const struct v4l2_subdev_core_ops imx568_subdev_core_ops = {
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= imx568_sensor_get_register,
	.s_register		= imx568_sensor_set_register,
#endif
};

static int imx568_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx568 *sensor = to_sensor(sd);
	int ret;

	mutex_lock(&sensor->lock);
	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto error_unlock;

		ret = imx568_stream_on(sd);
		if (ret < 0) {
			dev_err(&client->dev, "stream start failed: %d\n", ret);
			goto error_pm;
		}
		sensor->streaming = true;
	} else {
		ret = imx568_stream_off(sd);
		if (ret < 0)
			dev_err(&client->dev, "stream stop failed: %d\n", ret);
		sensor->streaming = false;
		pm_runtime_put(&client->dev);
	}

	mutex_unlock(&sensor->lock);

	return ret;

error_pm:
	pm_runtime_put(&client->dev);
error_unlock:
	mutex_unlock(&sensor->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops imx568_subdev_video_ops = {
	.s_stream =		imx568_s_stream,
};

/* Unique mbus codes advertised across the mode table (highest depth first). */
static const u32 imx568_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SRGGB8_1X8,
};

static bool imx568_mode_supported(const struct imx568 *sensor,
				  const struct imx568_mode *mode)
{
	struct imx568_selected_timing selected;

	return !imx568_find_timing(sensor, mode, &selected);
}

static int imx568_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx568 *sensor = to_sensor(sd);
	unsigned int i, j, count = 0;

	for (i = 0; i < ARRAY_SIZE(imx568_mbus_codes); i++) {
		for (j = 0; j < ARRAY_SIZE(imx568_modes); j++) {
			if (imx568_modes[j].format.code != imx568_mbus_codes[i] ||
			    !imx568_mode_supported(sensor, &imx568_modes[j]))
				continue;
			if (count++ == code->index) {
				code->code = imx568_mbus_codes[i];
				return 0;
			}
			break;
		}
	}

	return -EINVAL;
}

static int imx568_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx568 *sensor = to_sensor(sd);
	unsigned int i, count = 0;

	/* List the fse->index-th mode that matches the requested code. */
	for (i = 0; i < ARRAY_SIZE(imx568_modes); i++) {
		if (imx568_modes[i].format.code != fse->code ||
		    !imx568_mode_supported(sensor, &imx568_modes[i]))
			continue;
		if (count++ != fse->index)
			continue;
		fse->min_width = imx568_modes[i].format.width;
		fse->max_width = imx568_modes[i].format.width;
		fse->min_height = imx568_modes[i].format.height;
		fse->max_height = imx568_modes[i].format.height;
		return 0;
	}

	return -EINVAL;
}

static int imx568_set_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct v4l2_mbus_framefmt *state_fmt;
	struct v4l2_rect *state_crop;
	struct imx568 *sensor = to_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx568_selected_timing selected = {};
	const struct imx568_mode *mode;

	/* Adjust TRY and ACTIVE through the same endpoint-capability filter. */
	mutex_lock(&sensor->lock);
	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && sensor->streaming) {
		mutex_unlock(&sensor->lock);
		return -EBUSY;
	}

	mode = imx568_find_mode(sensor, fmt, &selected);
	if (!mode) {
		dev_err(&client->dev,
			"no link profile for requested %ux%u code 0x%04x\n",
			fmt->width, fmt->height, fmt->code);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	state_fmt = v4l2_subdev_state_get_format(sd_state, format->pad);
	state_crop = v4l2_subdev_state_get_crop(sd_state, format->pad);
	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		int exposure_max, exposure_def;
		int hblank, vblank;
		u32 fps_milli;
		u32 line_length;
		u64 rate;

		/* Publish LINK_FREQ before the receiver's stream-on D-PHY setup. */
		imx568_apply_timing(sensor, &selected);
		sensor->mode = mode;
		rate = imx568_mode_pixel_rate(sensor, mode);
		line_length = imx568_mode_line_length(sensor, mode);
		fps_milli = DIV_ROUND_CLOSEST_ULL(IMX568_HMAX_CLK_HZ * 1000,
						  sensor->hts * sensor->vts);

		dev_dbg(&client->dev,
			"mode %ux%u %u-bit %u.%03u fps: %u Mbps/lane, HMAX %u, VMAX %u\n",
			mode->format.width, mode->format.height,
			imx568_mode_bpp(mode), fps_milli / 1000, fps_milli % 1000,
			sensor->link_profile->lane_rate / 1000000,
			sensor->hts, sensor->vts);

		__v4l2_ctrl_modify_range(sensor->pixel_rate, rate, rate, 1, rate);

		/* HBLANK is reported in the PIXEL_RATE pixel domain. */
		hblank = line_length - mode->format.width;
		/* min=max pins the value; modify_range alone won't move it */
		__v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank, 1,
					 hblank);

		vblank = sensor->vts - mode->format.height;
		__v4l2_ctrl_modify_range(sensor->vblank, sensor->vblank_min,
					 IMX568_VTS_MAX - mode->format.height,
					 1, vblank);
		__v4l2_ctrl_s_ctrl(sensor->vblank, vblank);

		/* Same readout-reserve law the VBLANK handler enforces. */
		exposure_max = vblank -
			       imx568_exp_readout_overhead(sensor);
		exposure_def = clamp_t(int, IMX568_EXPOSURE_DEFAULT,
				       IMX568_EXPOSURE_MIN, exposure_max);
		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 exposure_max, sensor->exposure->step,
					 exposure_def);

		__v4l2_ctrl_s_ctrl(sensor->link_freq, sensor->link_freq_index);
	}
	*state_fmt = mode->format;
	*state_crop = mode->crop;
	*fmt = *state_fmt;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int imx568_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(sd_state, sel->pad);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX568_NATIVE_WIDTH;
		sel->r.height = IMX568_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX568_PIXEL_ARRAY_TOP;
		sel->r.left = IMX568_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX568_NATIVE_WIDTH;
		sel->r.height = IMX568_NATIVE_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx568_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	int data_type;
	u32 code;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, 0, 0)->code;
	v4l2_subdev_unlock_state(state);

	switch (code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		data_type = MIPI_CSI2_DT_RAW8;
		break;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		data_type = MIPI_CSI2_DT_RAW10;
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		data_type = MIPI_CSI2_DT_RAW12;
		break;
	default:
		return -EINVAL;
	}

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].pixelcode = code;
	fd->entry[0].stream = 0;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = data_type;

	return 0;
}

static const struct v4l2_subdev_pad_ops imx568_subdev_pad_ops = {
	.get_frame_desc 	= imx568_get_frame_desc,
	.enum_mbus_code		= imx568_enum_mbus_code,
	.enum_frame_size	= imx568_enum_frame_size,
	.set_fmt		= imx568_set_pad_fmt,
	.get_fmt		= v4l2_subdev_get_fmt,
	.get_selection		= imx568_get_selection,
};

static const struct v4l2_subdev_ops imx568_subdev_ops = {
	.core		= &imx568_subdev_core_ops,
	.video		= &imx568_subdev_video_ops,
	.pad		= &imx568_subdev_pad_ops,
};

static int imx568_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx568 *sensor = to_sensor(sd);
	u64 val;
	int ret;

	ret = cci_read(sensor->regmap, IMX568_GAIN, &val, NULL);
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "read register 0x%04lx : 0x%04llx\n",
		CCI_REG_ADDR(IMX568_GAIN), val);

	return 0;
}

static int imx568_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {};

	format.format = IMX568_DEFAULT_FORMAT;

	return imx568_set_pad_fmt(sd, state, &format);
}

static const struct v4l2_subdev_internal_ops imx568_subdev_internal_ops = {
	.init_state = imx568_init_state,
};

static int imx568_s_analogue_gain(struct v4l2_subdev *sd, u32 val)
{
	struct imx568 *sensor = to_sensor(sd);

	/* 10-bit analog gain. */
	return cci_write(sensor->regmap, IMX568_GAIN, val & 0x3ff, NULL);
}

static int imx568_s_exposure(struct v4l2_subdev *sd, u32 val)
{
	struct imx568 *sensor = to_sensor(sd);
	/* SHS bounds track the CURRENT frame period: VMAX = height + VBLANK.
	 * Min from GMRWT+GMTWT (gwtr) readout floor, max from VMAX. */
	u32 vmax = sensor->mode->format.height + sensor->vblank->val;
	u32 exp_min = IMX568_EXPOSURE_MIN;
	int overhead = imx568_exp_readout_overhead(sensor);
	u32 exp_max = sensor->vblank->val > overhead + IMX568_EXPOSURE_MIN ?
		      sensor->vblank->val - overhead : IMX568_EXPOSURE_MIN;

	if (val <= exp_min)
		val = exp_min;
	else if (val >= exp_max)
		val = exp_max;

	/* Global shutter: SHS = (VMAX-1) + gwtr_floor - exposure_lines. */
	val = (vmax - 1) + exp_min - val;

	/*
	 * Sensor has 20 bits, but the bottom 4 bits are fractions of a line
	 * which we leave as zero (and don't receive in "val").
	 */
	return cci_write(sensor->regmap, IMX568_SHS, val & 0xfffff, NULL);
}

static int imx568_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx568 *sensor = container_of(ctrl->handler,
					    struct imx568, ctrls);
	struct v4l2_subdev *sd = &sensor->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int pm_ret;
	int ret = 0;

	/* The control handler and sensor state share sensor->lock. */

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = ctrl->val -
			       imx568_exp_readout_overhead(sensor);
		exposure_def = clamp_t(int, IMX568_EXPOSURE_DEFAULT,
				       IMX568_EXPOSURE_MIN, exposure_max);
		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 exposure_max, sensor->exposure->step,
					 exposure_def);
	}

	/*
	 * If the device is not powered up do not apply any controls
	 * to H/W at this time. Instead the controls will be restored
	 * at s_stream(1) time.
	 */
	pm_ret = pm_runtime_get_if_in_use(&client->dev);
	if (!pm_ret)
		return 0;
	if (pm_ret < 0) {
		if (pm_ret != -EINVAL)
			return pm_ret;
		/* CONFIG_PM=n is active; a disabled suspended device is not. */
		if (pm_runtime_status_suspended(&client->dev))
			return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:

		break;
	case V4L2_CID_AUTOGAIN:

		break;
	case V4L2_CID_EXPOSURE_AUTO:

		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret =  imx568_s_analogue_gain(sd, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx568_s_exposure(sd, ctrl->val);
		break;
	case V4L2_CID_VBLANK: {
		/* Frame rate = INCK2/(HMAX*VMAX); VMAX = height + VBLANK. */
		u32 vmax = sensor->mode->format.height + ctrl->val;

		ret = cci_write(sensor->regmap, IMX568_VMAX, vmax, NULL);
		/*
		 * SHS encodes VMAX - exposure, so a VMAX change invalidates the
		 * programmed SHS; re-derive it or the sensor is left with
		 * SHS > VMAX (garbled frames) when only VBLANK moved.
		 */
		if (!ret)
			ret = imx568_s_exposure(sd, sensor->exposure->val);
		break;
	}
	case V4L2_CID_HBLANK:
		/* HMAX is fixed per mode; frame rate is set via VBLANK/VMAX. */
		break;
	case V4L2_CID_TEST_PATTERN:
		break;

	/* Read-only, but we adjust it based on mode. */
	case V4L2_CID_PIXEL_RATE:
		/* Read-only, but we adjust it based on mode. */
		break;
	case V4L2_CID_LINK_FREQ:
		/* Read-only, selected from the mode's bandwidth requirement. */
		break;

	case V4L2_CID_HFLIP:
		/* There's an in-built hflip in the sensor, so account for that here. */

		break;
	case V4L2_CID_VFLIP:

		break;

	default:
		dev_info(&client->dev,
			 "Control (id:0x%x, val:0x%x) not supported\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	if (pm_ret > 0)
		pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx568_ctrl_ops = {
	.s_ctrl = imx568_s_ctrl,
};

static int imx568_configure_regulators(struct device *dev,
				       struct imx568 *sensor)
{
	unsigned int i;

	for (i = 0; i < IMX568_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = imx568_supply_names[i];

	return devm_regulator_bulk_get(dev, IMX568_NUM_SUPPLIES,
				       sensor->supplies);
}

static int imx568_init_controls(struct imx568 *sensor, struct device *dev)
{
	int hblank, exposure_max, exposure_def;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_config link_freq_cfg = {
		.ops = &imx568_ctrl_ops,
		.id = V4L2_CID_LINK_FREQ,
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.max = ARRAY_SIZE(imx568_link_freqs) - 1,
		.def = sensor->link_freq_index,
		.menu_skip_mask = GENMASK(ARRAY_SIZE(imx568_link_freqs) - 1, 0) &
				  ~sensor->link_rate_mask,
		.qmenu_int = imx568_link_freqs,
	};
	s64 pixel_rate = imx568_mode_pixel_rate(sensor, sensor->mode);

	v4l2_ctrl_handler_init(&sensor->ctrls, 10);
	sensor->ctrls.lock = &sensor->lock;

	v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu(&sensor->ctrls, &imx568_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       0, V4L2_EXPOSURE_MANUAL);

	exposure_max = (sensor->vts - sensor->mode->format.height) -
		       imx568_exp_readout_overhead(sensor);
	exposure_def = clamp_t(int, IMX568_EXPOSURE_DEFAULT,
			       IMX568_EXPOSURE_MIN, exposure_max);
	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX568_EXPOSURE_MIN,
					     exposure_max, IMX568_EXPOSURE_STEP,
					     exposure_def);

	/* min: 16 = 1.0x; max (10 bits); default: 32 = 2.0x. */
	v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, IMX568_GAIN_MIN, IMX568_GAIN_MAX, IMX568_GAIN_STEP, IMX568_GAIN_DEFAULT);

	/* Pixel-array rate derived from the selected CSI transport profile. */
	sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       pixel_rate, pixel_rate, 1,
					       pixel_rate);

	/* HBLANK uses the same V4L2 pixel domain as PIXEL_RATE. */
	hblank = imx568_mode_line_length(sensor, sensor->mode) -
		 sensor->mode->format.width;
	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					   V4L2_CID_VBLANK, sensor->vblank_min,
					   IMX568_VTS_MAX -
					   sensor->mode->format.height, 1,
					   sensor->vts -
					   sensor->mode->format.height);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &imx568_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx568_test_pattern_menu) - 1,
				     0, 0, imx568_test_pattern_menu);

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->link_freq = v4l2_ctrl_new_custom(&sensor->ctrls,
						 &link_freq_cfg, NULL);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_fwnode_device_parse(dev, &props);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &imx568_ctrl_ops,
					&props);

	if (sensor->ctrls.error)
		goto handler_free;

	sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->sd.ctrl_handler = &sensor->ctrls;

	return 0;

handler_free:
	dev_err(dev, "%s Controls initialization failed (%d)\n", __func__,
		sensor->ctrls.error);
	v4l2_ctrl_handler_free(&sensor->ctrls);

	return sensor->ctrls.error;
}

static int imx568_parse_dt(struct imx568 *sensor, struct device *dev,
			   struct device_node *np)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device_node *ep __free(device_node) =
		of_graph_get_endpoint_by_regs(np, 0, -1);
	unsigned int i, j;
	int ret;

	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &bus_cfg);
	if (ret)
		return ret;

	sensor->lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
	if (!sensor->lanes) {
		dev_warn(dev,
			 "endpoint is missing data-lanes, assuming %u lanes\n",
			 IMX568_DEFAULT_LANES);
		sensor->lanes = IMX568_DEFAULT_LANES;
	}
	if (sensor->lanes != 2 && sensor->lanes != 4) {
		dev_err(dev, "unsupported number of CSI-2 data lanes %u\n",
			sensor->lanes);
		v4l2_fwnode_endpoint_free(&bus_cfg);
		return -EINVAL;
	}
	if (bus_cfg.bus.mipi_csi2.flags &
	    V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) {
		dev_err(dev, "non-continuous CSI-2 clock is unsupported\n");
		v4l2_fwnode_endpoint_free(&bus_cfg);
		return -EINVAL;
	}
	if (!bus_cfg.nr_of_link_frequencies) {
		dev_warn(dev,
			 "endpoint is missing link-frequencies, allowing all supported rates\n");
		v4l2_fwnode_endpoint_free(&bus_cfg);
		return 0;
	}

	/* Keep the advertised rates the driver supports. */
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		for (j = 0; j < ARRAY_SIZE(imx568_link_freqs); j++)
			if (bus_cfg.link_frequencies[i] == imx568_link_freqs[j])
				break;
		if (j == ARRAY_SIZE(imx568_link_freqs)) {
			dev_warn(dev,
				 "ignoring unsupported endpoint link-frequency %llu\n",
				 bus_cfg.link_frequencies[i]);
			continue;
		}
		sensor->link_rate_mask |= BIT(j);
	}
	if (!sensor->link_rate_mask) {
		dev_err(dev, "endpoint has no supported link-frequency\n");
		v4l2_fwnode_endpoint_free(&bus_cfg);
		return -EINVAL;
	}

	v4l2_fwnode_endpoint_free(&bus_cfg);

	return 0;
}

static int imx568_probe(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct device *dev = &client->dev;
	struct imx568_selected_timing selected;
	struct imx568 *sensor;
	struct v4l2_subdev *sd;
	unsigned long xclk_freq;
	int ret;

	dev_dbg(dev, "enter SONY IMX568 camera driver probe\n");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->lanes = IMX568_DEFAULT_LANES;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "failed to initialise CCI regmap\n");

	if (IS_ENABLED(CONFIG_OF) && np) {
		ret = imx568_parse_dt(sensor, dev, np);
		if (ret) {
			dev_err(dev, "DT parsing error: %d\n", ret);
			return ret;
		}
	}
	if (!sensor->link_rate_mask)
		sensor->link_rate_mask = GENMASK(ARRAY_SIZE(imx568_link_freqs) - 1, 0);

	sensor->xclk = devm_v4l2_sensor_clk_get(dev, "inck");
	if (IS_ERR(sensor->xclk))
		return dev_err_probe(dev, PTR_ERR(sensor->xclk),
				     "failed to get INCK\n");

	xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq != IMX568_INCK_FREQ_HZ) {
		dev_err(dev, "unsupported INCK frequency: %lu Hz\n", xclk_freq);
		return -EINVAL;
	}

	/* Request the power down GPIO asserted. */
	sensor->pwdn = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwdn)) {
		dev_err(dev, "Failed to get reset gpio\n");
		return -EINVAL;
	}

	ret = imx568_configure_regulators(dev, sensor);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sensor->lock);

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &imx568_subdev_ops);
	sd->internal_ops = &imx568_subdev_internal_ops;

	sensor->mode = IMX568_DEFAULT_MODE;
	ret = imx568_find_timing(sensor, sensor->mode, &selected);
	if (ret) {
		dev_err(dev, "default mode has no valid link profile\n");
		goto mutex_destroy;
	}
	imx568_apply_timing(sensor, &selected);

	ret = imx568_init_controls(sensor, dev);
	if (ret)
		goto mutex_destroy;

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		goto ctrl_handler_free;

	ret = imx568_power_on(dev);
	if (ret)
		goto entity_cleanup;

	ret = imx568_detect(sd);
	if (ret < 0)
		goto power_off;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0) {
		dev_err(dev, "subdev state initialization failed: %d\n", ret);
		goto power_off;
	}

	/* The registered subdevice may be used immediately, so enable PM first. */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret < 0)
		goto pm_disable;

	pm_runtime_idle(dev);

	dev_info(dev, "SONY IMX568 camera driver probed\n");

	return 0;

pm_disable:
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(sd);
	imx568_power_off(dev);
	pm_runtime_set_suspended(dev);
	goto entity_cleanup;
power_off:
	imx568_power_off(dev);
entity_cleanup:
	media_entity_cleanup(&sd->entity);
ctrl_handler_free:
	v4l2_ctrl_handler_free(&sensor->ctrls);
mutex_destroy:
	mutex_destroy(&sensor->lock);

	return ret;
}

static void imx568_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *sensor = to_sensor(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	v4l2_device_unregister_subdev(sd);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		imx568_power_off(&client->dev);
		pm_runtime_set_suspended(&client->dev);
	}
	mutex_destroy(&sensor->lock);
}

static const struct dev_pm_ops imx568_pm_ops = {
	SET_RUNTIME_PM_OPS(imx568_power_off, imx568_power_on, NULL)
};

static const struct i2c_device_id imx568_id[] = {
	{ "imx568" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, imx568_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx568_of_match[] = {
	{ .compatible = "sony,imx568" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx568_of_match);
#endif

static struct i2c_driver imx568_driver = {
	.driver = {
		.of_match_table = of_match_ptr(imx568_of_match),
		.name	= "imx568",
		.pm	= &imx568_pm_ops,
	},
	.probe		= imx568_probe,
	.remove		= imx568_remove,
	.id_table	= imx568_id,
};

module_i2c_driver(imx568_driver);

MODULE_AUTHOR("Fengyu Sheng <fengyus@leopardimaging.com>");
MODULE_DESCRIPTION("A low-level driver for SONY imx568 sensors");
MODULE_LICENSE("GPL v2");
