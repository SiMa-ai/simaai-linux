// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for onsemi AR0234 cameras.
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd
 * Copyright 2026 SiMa Technologies, Inc.
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include <linux/bitfield.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/mipi-csi2.h>

#include "ar0234.h"

enum pad_types {
	IMAGE_PAD,
	NUM_PADS
};

/* Identical for every mode, depth and link rate; written in standby. */
static const struct cci_reg_sequence ar0234_common_regs[] = {
	{ AR0234_DATAPATH_SELECT,	AR0234_DATAPATH_SELECT_VAL },
	{ AR0234_X_ODD_INC,		AR0234_ODD_INC_NO_SKIP },
	{ AR0234_Y_ODD_INC,		AR0234_ODD_INC_NO_SKIP },
	{ AR0234_OPERATION_MODE_CTRL,	AR0234_OPERATION_MODE_LINEAR },
	{ AR0234_READ_MODE,		0 },
	{ AR0234_COMPANDING,		0 },
};

static const struct ar0234_depth ar0234_depths[] = {
	/* 10-bit leads: the default format and the fallback. */
	{ .code = MEDIA_BUS_FMT_SGRBG10_1X10, .vt_pix_clk_div = 5,
	  .op_pix_clk_div = 10, .data_format_bits = AR0234_DATA_FORMAT(10, 10),
	  .data_type = MIPI_CSI2_DT_RAW10 },
	/* The LI 12-bit setting; AND9812 lists only 8 and 10 for the output. */
	{ .code = MEDIA_BUS_FMT_SGRBG12_1X12, .vt_pix_clk_div = 6,
	  .op_pix_clk_div = 12, .data_format_bits = AR0234_DATA_FORMAT(12, 12),
	  .data_type = MIPI_CSI2_DT_RAW12 },
	/* 8-bit truncates the 10-bit pixel. */
	{ .code = MEDIA_BUS_FMT_SGRBG8_1X8, .vt_pix_clk_div = 4,
	  .op_pix_clk_div = 8, .data_format_bits = AR0234_DATA_FORMAT(10, 8),
	  .data_type = MIPI_CSI2_DT_RAW8 },
};

/* The monochrome part, same timing under grey codes. */
static const struct ar0234_depth ar0234_depths_mono[] = {
	{ .code = MEDIA_BUS_FMT_Y10_1X10, .vt_pix_clk_div = 5,
	  .op_pix_clk_div = 10, .data_format_bits = AR0234_DATA_FORMAT(10, 10),
	  .data_type = MIPI_CSI2_DT_RAW10 },
	{ .code = MEDIA_BUS_FMT_Y12_1X12, .vt_pix_clk_div = 6,
	  .op_pix_clk_div = 12, .data_format_bits = AR0234_DATA_FORMAT(12, 12),
	  .data_type = MIPI_CSI2_DT_RAW12 },
	{ .code = MEDIA_BUS_FMT_Y8_1X8, .vt_pix_clk_div = 4,
	  .op_pix_clk_div = 8, .data_format_bits = AR0234_DATA_FORMAT(10, 8),
	  .data_type = MIPI_CSI2_DT_RAW8 },
};

#define AR0234_NUM_DEPTHS ARRAY_SIZE(ar0234_depths)
static_assert(ARRAY_SIZE(ar0234_depths_mono) == AR0234_NUM_DEPTHS,
	      "the mono table must mirror the colour table");

/* D-PHY timing counts at the reference OP_CLK (LI values 0x31c5/0x214c/0x5048/0x0186/0x8805). */
static const struct ar0234_dphy_timing ar0234_dphy_timing_ref = {
	.hs_prepare = 3, .hs_trail = 7, .clk_trail = 5,
	.clk_prepare = 2, .hs_exit = 5, .hs_zero = 12,
	.bgap = 5, .clk_pre = 1, .clk_post = 8,
	.lpx = 3, .ulpm_recover = 6,
	.t_init = 5,
};

static const char * const ar0234_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"Walking 1s"
};

static const int ar0234_test_pattern_val[] = {
	AR0234_TEST_PATTERN_NONE,
	AR0234_TEST_PATTERN_COLOR_BAR,
	AR0234_TEST_PATTERN_SOLID_COLOR,
	AR0234_TEST_PATTERN_FADE_TO_GRAY,
	AR0234_TEST_PATTERN_WALKING_1,
};

/* regulator supplies */
static const char * const ar0234_supply_name[] = {
	/* Supplies can be enabled in any order */
	"vana",  /* Analog (2.8V) supply */
	"vdig",  /* Digital Core (1.8V) supply */
	"vddl",  /* IF (1.2V) supply */
};

#define AR0234_NUM_SUPPLIES ARRAY_SIZE(ar0234_supply_name)

/* 1080p reads the top 1080 rows of the readable area. */
static const struct ar0234_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.crop = { .left = AR0234_PIXEL_ARRAY_LEFT, .top = AR0234_PIXEL_ARRAY_TOP,
			  .width = 1920, .height = 1080 },
		.fll_def = AR0234_FRAME_LENGTH_LINES_DEF,
	},
	{
		.width = 1920,
		.height = 1200,
		.crop = { .left = AR0234_PIXEL_ARRAY_LEFT, .top = AR0234_PIXEL_ARRAY_TOP,
			  .width = 1920, .height = 1200 },
		.fll_def = AR0234_FRAME_LENGTH_LINES_DEF,
	},
};

struct ar0234 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct regmap *regmap;

	struct clk *xclk; /* system clock to AR0234 */
	u32 xclk_freq;

	/* PLLs for the DT link frequencies; each depth adds its pixel dividers */
	struct ar0234_pll plls[AR0234_LINK_FREQS_MAX];
	s64 link_freq_menu[AR0234_LINK_FREQS_MAX];
	unsigned int num_link_freqs;
	/* CSI-2 data lanes from the endpoint (2 or 4) */
	u32 num_data_lanes;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[AR0234_NUM_SUPPLIES];

	bool monochrome;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Frame length on the sensor, the one before it, and the coarse value written */
	u32 fll;
	u32 fll_prev;
	u32 coarse;
};

static inline struct ar0234 *to_ar0234(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct ar0234, sd);
}

static const struct ar0234_depth *ar0234_depths_of(const struct ar0234 *ar0234)
{
	return ar0234->monochrome ? ar0234_depths_mono : ar0234_depths;
}

/* The depth carrying @code, the table's first when none does. */
static const struct ar0234_depth *ar0234_depth_of(const struct ar0234 *ar0234,
						  u32 code)
{
	const struct ar0234_depth *depths = ar0234_depths_of(ar0234);
	unsigned int i;

	for (i = 0; i < AR0234_NUM_DEPTHS; i++)
		if (depths[i].code == code)
			return &depths[i];

	return &depths[0];
}

/*
 * Row time is LINE_LENGTH_PCK / PIXCLK and a shallower pixel raises PIXCLK: the shortest line at
 * every depth turns that into frame rate. The array reads four pixels per LINE_LENGTH_PCK clock.
 */
#define AR0234_PIXELS_PER_LINE			(4 * AR0234_LINE_LENGTH_PCK_MIN)

/* Pixel-array readout clock; the array emits four pixels per clock. */
static u64 ar0234_vt_pix_clk(const struct ar0234_pll *pll)
{
	return div_u64((u64)AR0234_XCLK_FREQ * pll->pll_multiplier,
		       pll->pre_pll_clk_div * pll->vt_sys_clk_div *
		       pll->vt_pix_clk_div);
}

/* Per-lane output clock in pixels/s; the serial link is DDR, hence the 2. */
static u64 ar0234_op_clk(const struct ar0234_pll *pll)
{
	return div_u64(2 * (u64)AR0234_XCLK_FREQ * pll->pll_multiplier,
		       pll->pre_pll_clk_div * pll->op_sys_clk_div *
		       pll->op_pix_clk_div);
}

/*
 * A shallower pixel raises OP_CLK, so each depth runs on the fastest listed link that keeps it in
 * range. -1 when none does.
 */
static int ar0234_link_index(const struct ar0234 *ar0234,
			     const struct ar0234_depth *depth)
{
	unsigned int i;
	int best = -1;

	for (i = 0; i < ar0234->num_link_freqs; i++) {
		struct ar0234_pll pll = ar0234->plls[i];

		pll.op_pix_clk_div = depth->op_pix_clk_div;
		if (ar0234_op_clk(&pll) > AR0234_OP_CLK_MAX_HZ)
			continue;
		if (best < 0 ||
		    ar0234->link_freq_menu[i] > ar0234->link_freq_menu[best])
			best = i;
	}

	return best;
}

static void ar0234_pll_for(const struct ar0234 *ar0234,
			   const struct ar0234_depth *depth,
			   struct ar0234_pll *pll)
{
	*pll = ar0234->plls[max(ar0234_link_index(ar0234, depth), 0)];
	pll->vt_pix_clk_div = depth->vt_pix_clk_div;
	pll->op_pix_clk_div = depth->op_pix_clk_div;
}

/* The code is offered where some listed link carries it. */
static bool ar0234_code_ok(const struct ar0234 *ar0234, u32 code)
{
	const struct ar0234_depth *depth = ar0234_depth_of(ar0234, code);

	return depth->code == code && ar0234_link_index(ar0234, depth) >= 0;
}

/* AND9820 Table 14: the array reads 4 * PIXCLK, the link lanes * OP_CLK; the smaller wins. */
static u64 ar0234_pixel_rate(const struct ar0234 *ar0234,
			     const struct ar0234_depth *depth)
{
	struct ar0234_pll pll;

	ar0234_pll_for(ar0234, depth, &pll);

	return min(4 * ar0234_vt_pix_clk(&pll),
		   ar0234->num_data_lanes * ar0234_op_clk(&pll));
}

/*
 * Counts are op_pix_clk periods, scaled with OP_CLK and rounded up. The depth gate keeps OP_CLK
 * within twice the reference, so none saturates; inlined for a constant bitfield mask.
 */
static __always_inline u64 ar0234_dphy_field(const struct ar0234_pll *pll,
					     u64 mask, u8 ref, u64 ref_hz)
{
	u64 count = DIV64_U64_ROUND_UP(ref * ar0234_op_clk(pll), ref_hz);

	return u64_encode_bits(min_t(u64, count, field_max(mask)), mask);
}

static void ar0234_write_dphy_timing(struct ar0234 *ar0234,
				     const struct ar0234_depth *depth, int *err)
{
	const struct ar0234_dphy_timing *t = &ar0234_dphy_timing_ref;
	struct ar0234_pll pll;

	ar0234_pll_for(ar0234, depth, &pll);
#define F(_mask, _member) \
	ar0234_dphy_field(&pll, (_mask), t->_member, AR0234_DPHY_REF_OP_CLK_HZ)

	cci_write(ar0234->regmap, AR0234_FRAME_PREAMBLE,
		  ar0234_dphy_field(&pll, AR0234_PREAMBLE_MASK, AR0234_FRAME_PREAMBLE_DEF,
				    AR0234_PREAMBLE_REF_OP_CLK_HZ), err);
	cci_write(ar0234->regmap, AR0234_LINE_PREAMBLE,
		  ar0234_dphy_field(&pll, AR0234_PREAMBLE_MASK, AR0234_LINE_PREAMBLE_DEF,
				    AR0234_PREAMBLE_REF_OP_CLK_HZ), err);
	cci_write(ar0234->regmap, AR0234_MIPI_TIMING_0,
		  F(AR0234_T0_HS_PREPARE_MASK, hs_prepare) |
		  F(AR0234_T0_HS_TRAIL_MASK, hs_trail) |
		  F(AR0234_T0_CLK_TRAIL_MASK, clk_trail), err);
	cci_write(ar0234->regmap, AR0234_MIPI_TIMING_1,
		  F(AR0234_T1_CLK_PREPARE_MASK, clk_prepare) |
		  F(AR0234_T1_HS_EXIT_MASK, hs_exit) |
		  F(AR0234_T1_HS_ZERO_MASK, hs_zero), err);
	cci_write(ar0234->regmap, AR0234_MIPI_TIMING_2,
		  F(AR0234_T2_BGAP_MASK, bgap) |
		  F(AR0234_T2_CLK_PRE_MASK, clk_pre) |
		  F(AR0234_T2_CLK_POST_MASK, clk_post), err);
	cci_write(ar0234->regmap, AR0234_MIPI_TIMING_3,
		  F(AR0234_T3_LPX_MASK, lpx) |
		  F(AR0234_T3_ULPM_RECOVER_MASK, ulpm_recover), err);
	cci_write(ar0234->regmap, AR0234_MIPI_TIMING_4,
		  AR0234_T4_CONT_TX_CLK | FIELD_PREP(AR0234_T4_RSVD_MASK, AR0234_T4_RSVD_VAL) |
		  F(AR0234_T4_T_INIT_MASK, t_init), err);
#undef F
}

/* The first code a listed link allows; 12-bit always is. */
static u32 ar0234_first_code(struct ar0234 *ar0234)
{
	const struct ar0234_depth *depths = ar0234_depths_of(ar0234);
	unsigned int i;

	for (i = 0; i < AR0234_NUM_DEPTHS; i++)
		if (ar0234_code_ok(ar0234, depths[i].code))
			return depths[i].code;

	return depths[0].code;
}

/* AND9820: mirror and flip keep the Bayer phase, so the code does not follow them. */
static u32 ar0234_get_format_code(struct ar0234 *ar0234, u32 code)
{
	return ar0234_code_ok(ar0234, code) ? code : ar0234_first_code(ar0234);
}

static int ar0234_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt);

static int ar0234_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = {
			.width = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	return ar0234_set_pad_format(sd, state, &format);
}

static bool ar0234_coarse_banned(u32 coarse, u32 fll)
{
	return coarse + AR0234_COARSE_BAND_FIRST >= fll &&
	       coarse + AR0234_COARSE_BAND_LAST <= fll;
}

/* Clear of both bands; the previous length's stays excluded until VBLANK changes again. */
static bool ar0234_coarse_is_clear(const struct ar0234 *ar0234, u32 coarse)
{
	return !ar0234_coarse_banned(coarse, ar0234->fll) &&
	       !ar0234_coarse_banned(coarse, ar0234->fll_prev);
}

/* Nearest clear value, the shorter on a tie; the merged bands span at most 2 * BAND_ROWS. */
static u32 ar0234_coarse_for(const struct ar0234 *ar0234, u32 want)
{
	u32 d;

	lockdep_assert_held(ar0234->sd.ctrl_handler->lock);

	for (d = 0; d <= 2 * AR0234_COARSE_BAND_ROWS; d++) {
		if (want >= d + AR0234_EXPOSURE_MIN &&
		    ar0234_coarse_is_clear(ar0234, want - d))
			return want - d;
		if (want + d <= ar0234->exposure->maximum &&
		    ar0234_coarse_is_clear(ar0234, want + d))
			return want + d;
	}

	return want;
}

/* Writes the nearest clear value and returns it, so the control can report it. */
static int ar0234_write_coarse(struct ar0234 *ar0234, u32 want, u32 *applied)
{
	u32 coarse = ar0234_coarse_for(ar0234, want);
	int ret = 0;

	if (coarse != ar0234->coarse) {
		ret = cci_write(ar0234->regmap, AR0234_COARSE_INTEGRATION_TIME,
				coarse, NULL);
		if (!ret)
			ar0234->coarse = coarse;
	}
	*applied = coarse;

	return ret;
}

/* The nearest clear coarse value is committed with the control, on every path. */
static int ar0234_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0234 *ar0234 =
		container_of(ctrl->handler, struct ar0234, ctrl_handler);

	if (ctrl->id == V4L2_CID_EXPOSURE)
		ctrl->val = ar0234_coarse_for(ar0234, ctrl->val);

	return 0;
}

/* The exposure range a frame length allows, the default kept inside it. */
static void ar0234_set_exposure_range(struct ar0234 *ar0234, u32 fll)
{
	int exposure_max = AR0234_EXPOSURE_MAX_FOR(fll);

	__v4l2_ctrl_modify_range(ar0234->exposure, ar0234->exposure->minimum,
				 exposure_max, ar0234->exposure->step,
				 min(exposure_max, AR0234_EXPOSURE_DEFAULT));
}

static int ar0234_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0234 *ar0234 =
		container_of(ctrl->handler, struct ar0234, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	u32 fll_prev = ar0234->fll_prev;
	u32 fll = ar0234->fll;
	u32 coarse;
	int pm;
	int ret;

	/* The handler lock is the state lock. */
	state = v4l2_subdev_get_locked_active_state(&ar0234->sd);
	fmt = v4l2_subdev_state_get_format(state, IMAGE_PAD);

	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Control state the band follows; the registers catch up while powered. */
		ar0234->fll_prev = ar0234->fll;
		ar0234->fll = fmt->height + ctrl->val;
		ar0234_set_exposure_range(ar0234, ar0234->fll);
	}

	/* The PLL follows the link at stream start. */
	if (ctrl->id == V4L2_CID_LINK_FREQ)
		return 0;

	/* Needs power; a negative result means runtime PM is off and the part is powered. */
	pm = pm_runtime_get_if_in_use(&client->dev);
	if (!pm)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(ar0234->regmap, AR0234_ANALOG_GAIN, ctrl->val,
				NULL);
		break;
	case V4L2_CID_EXPOSURE:
		/* Clear already: try_ctrl, or the commits around stream start. */
		ret = ar0234_write_coarse(ar0234, ctrl->val, &coarse);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(ar0234->regmap, AR0234_GLOBAL_GAIN, ctrl->val,
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(ar0234->regmap, AR0234_TEST_PATTERN_MODE,
				ar0234_test_pattern_val[ctrl->val], NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	{
		u64 reg;

		ret = cci_read(ar0234->regmap, AR0234_READ_MODE, &reg, NULL);
		if (ret)
			break;

		reg &= ~(AR0234_READ_MODE_HORIZ_MIRROR |
			AR0234_READ_MODE_VERT_FLIP);
		if (ar0234->hflip->val)
			reg |= AR0234_READ_MODE_HORIZ_MIRROR;
		if (ar0234->vflip->val)
			reg |= AR0234_READ_MODE_VERT_FLIP;

		ret = cci_write(ar0234->regmap, AR0234_READ_MODE, reg, NULL);
		break;
	}
	case V4L2_CID_VBLANK:
		/* Clear the coarse value of both bands before the new length lands. */
		ret = ar0234_write_coarse(ar0234, ar0234->exposure->cur.val,
					  &coarse);
		if (!ret && coarse != ar0234->exposure->cur.val)
			ret = __v4l2_ctrl_s_ctrl(ar0234->exposure, coarse);
		if (!ret)
			ret = cci_write(ar0234->regmap, AR0234_FRAME_LENGTH_LINES,
					ar0234->fll, NULL);
		if (ret) {
			ar0234->fll_prev = fll_prev;
			ar0234->fll = fll;
			ar0234_set_exposure_range(ar0234, fll);
			/* Back under the old lengths: clear the coarse value for them. */
			if (!ar0234_write_coarse(ar0234, ar0234->exposure->cur.val,
						 &coarse) &&
			    coarse != ar0234->exposure->cur.val)
				__v4l2_ctrl_s_ctrl(ar0234->exposure, coarse);
		}
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = cci_write(ar0234->regmap, AR0234_TEST_DATA_RED, ctrl->val,
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = cci_write(ar0234->regmap, AR0234_TEST_DATA_GREENR,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = cci_write(ar0234->regmap, AR0234_TEST_DATA_BLUE, ctrl->val,
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = cci_write(ar0234->regmap, AR0234_TEST_DATA_GREENB,
				ctrl->val, NULL);
		break;
	default:
		dev_dbg(&client->dev, "ctrl(id:0x%x,val:0x%x) is not handled\n",
			ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	if (pm > 0)
		pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0234_ctrl_ops = {
	.try_ctrl = ar0234_try_ctrl,
	.s_ctrl = ar0234_set_ctrl,
};

static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	const struct ar0234_depth *depths = ar0234_depths_of(ar0234);
	unsigned int i, n;

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	for (i = 0, n = 0; i < AR0234_NUM_DEPTHS; i++) {
		if (!ar0234_code_ok(ar0234, depths[i].code))
			continue;
		if (n++ == code->index) {
			code->code = depths[i].code;
			return 0;
		}
	}

	return -EINVAL;
}

static int ar0234_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != ar0234_get_format_code(ar0234, fse->code))
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void ar0234_update_pad_format(const struct ar0234_mode *mode, u32 code,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static int ar0234_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	const struct ar0234_depth *depth;
	const struct ar0234_mode *mode;
	int hblank, ret;
	u64 pixel_rate;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	/* The window, depth and PLL only latch in standby. */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width, height,
				      fmt->format.width, fmt->format.height);
	ar0234_update_pad_format(mode,
				 ar0234_get_format_code(ar0234, fmt->format.code),
				 &fmt->format);
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	*v4l2_subdev_state_get_crop(state, fmt->pad) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	/* The depth picks the link and the dividers, and with them the pixel rate. */
	depth = ar0234_depth_of(ar0234, fmt->format.code);
	ret = __v4l2_ctrl_s_ctrl(ar0234->link_freq,
				 ar0234_link_index(ar0234, depth));
	if (ret)
		return ret;
	pixel_rate = ar0234_pixel_rate(ar0234, depth);
	__v4l2_ctrl_modify_range(ar0234->pixel_rate, pixel_rate, pixel_rate, 1,
				 pixel_rate);
	__v4l2_ctrl_modify_range(ar0234->vblank, AR0234_VBLANK_MIN,
				 AR0234_FRAME_LENGTH_LINES_MAX - mode->height, 1,
				 mode->fll_def - mode->height);
	__v4l2_ctrl_s_ctrl(ar0234->vblank, mode->fll_def - mode->height);
	/* The VBLANK handler runs only when its value moves; derive the range here too. */
	ar0234_set_exposure_range(ar0234, mode->fll_def);
	hblank = AR0234_PIXELS_PER_LINE - mode->width;
	__v4l2_ctrl_modify_range(ar0234->hblank, hblank, hblank, 1, hblank);

	return 0;
}

static void ar0234_set_framefmt(struct ar0234 *ar0234,
				const struct ar0234_depth *depth, int *err)
{
	cci_write(ar0234->regmap, AR0234_DATA_FORMAT_BITS,
		  depth->data_format_bits, err);
}

/* Applied in standby after SERIAL_FORMAT. */
static void ar0234_write_window(struct ar0234 *ar0234,
				const struct ar0234_mode *mode, int *err)
{
	cci_write(ar0234->regmap, AR0234_Y_ADDR_START, mode->crop.top, err);
	cci_write(ar0234->regmap, AR0234_X_ADDR_START, mode->crop.left, err);
	cci_write(ar0234->regmap, AR0234_Y_ADDR_END,
		  mode->crop.top + mode->crop.height - 1, err);
	cci_write(ar0234->regmap, AR0234_X_ADDR_END,
		  mode->crop.left + mode->crop.width - 1, err);
	cci_write(ar0234->regmap, AR0234_FRAME_LENGTH_LINES, mode->fll_def, err);
	cci_write(ar0234->regmap, AR0234_LINE_LENGTH_PCK,
		  AR0234_LINE_LENGTH_PCK_MIN, err);
	cci_write(ar0234->regmap, AR0234_COARSE_INTEGRATION_TIME,
		  AR0234_EXPOSURE_DEFAULT, err);
}

static void ar0234_write_pll(struct ar0234 *ar0234,
			     const struct ar0234_depth *depth, int *err)
{
	struct ar0234_pll pll;

	ar0234_pll_for(ar0234, depth, &pll);
	cci_write(ar0234->regmap, AR0234_PLL_MULTIPLIER, pll.pll_multiplier, err);
	cci_write(ar0234->regmap, AR0234_PRE_PLL_CLK_DIV, pll.pre_pll_clk_div, err);
	cci_write(ar0234->regmap, AR0234_VT_SYS_CLK_DIV, pll.vt_sys_clk_div, err);
	cci_write(ar0234->regmap, AR0234_VT_PIX_CLK_DIV, pll.vt_pix_clk_div, err);
	cci_write(ar0234->regmap, AR0234_OP_SYS_CLK_DIV, pll.op_sys_clk_div, err);
	cci_write(ar0234->regmap, AR0234_OP_PIX_CLK_DIV, pll.op_pix_clk_div, err);
}

static int ar0234_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = AR0234_NATIVE_WIDTH;
		sel->r.height = AR0234_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/* The driver offers no window beyond the default. */
		sel->r.top = AR0234_PIXEL_ARRAY_TOP;
		sel->r.left = AR0234_PIXEL_ARRAY_LEFT;
		sel->r.width = AR0234_PIXEL_ARRAY_WIDTH;
		sel->r.height = AR0234_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int ar0234_start_streaming(struct ar0234 *ar0234,
				  struct v4l2_subdev_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	const struct v4l2_mbus_framefmt *fmt;
	const struct ar0234_depth *depth;
	const struct ar0234_mode *mode;
	u64 val;
	int ret = 0;

	fmt = v4l2_subdev_state_get_format(state, IMAGE_PAD);
	depth = ar0234_depth_of(ar0234, fmt->code);
	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width, height,
				      fmt->width, fmt->height);

	/*
	 * Software standby first: the PLL is powered down there, so the
	 * link-rate multiplier, the PLL/MIPI base and the D-PHY timing latch
	 * cleanly; SERIAL_FORMAT (lane count) only latches in standby too.
	 */
	cci_write(ar0234->regmap, AR0234_RESET_REGISTER,
		  AR0234_STANDBY, &ret);
	ar0234_write_pll(ar0234, depth, &ret);
	cci_multi_reg_write(ar0234->regmap, ar0234_common_regs,
			    ARRAY_SIZE(ar0234_common_regs), &ret);
	cci_write(ar0234->regmap, AR0234_MIPI_CNTRL,
		  FIELD_PREP(AR0234_MIPI_CNTRL_DATA_TYPE, depth->data_type),
		  &ret);
	ar0234_write_dphy_timing(ar0234, depth, &ret);
	cci_write(ar0234->regmap, AR0234_SERIAL_FORMAT,
		  AR0234_SERIAL_FORMAT_TYPE_MIPI |
		  FIELD_PREP(AR0234_SERIAL_FORMAT_LANES, ar0234->num_data_lanes),
		  &ret);
	ar0234_write_window(ar0234, mode, &ret);

	/* Still in standby: pixel depth and the embedded-data lines. */
	cci_read(ar0234->regmap, AR0234_SMIA_TEST, &val, &ret);
	cci_write(ar0234->regmap, AR0234_SMIA_TEST,
		  val & ~AR0234_SMIA_TEST_EMBEDDED_DATA, &ret);
	ar0234_set_framefmt(ar0234, depth, &ret);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* The window set the defaults; coarse 0 is below the minimum, so the first write lands. */
	ar0234->fll = mode->fll_def;
	ar0234->fll_prev = ar0234->fll;
	ar0234->coarse = 0;

	/* The band moved with the mode: commit a clear exposure first. */
	ret = __v4l2_ctrl_s_ctrl(ar0234->exposure, ar0234->exposure->cur.val);
	if (ret)
		return ret;

	/* The controls, READ_MODE among them, are written in standby too. */
	ret = __v4l2_ctrl_handler_setup(ar0234->sd.ctrl_handler);
	if (ret) {
		dev_err(&client->dev, "%s failed to apply the controls\n",
			__func__);
		return ret;
	}

	/* Latch, then let the PLL relock: the sensor NAKs writes until it settles. */
	cci_write(ar0234->regmap, AR0234_RESET_REGISTER,
		  AR0234_STREAMING, &ret);
	if (ret) {
		dev_err(&client->dev, "%s failed to latch mode\n", __func__);
		return ret;
	}
	fsleep(AR0234_PLL_RELOCK_DELAY_US);

	return 0;
}

static void ar0234_stop_streaming(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	int ret;

	/* set stream off register */
	ret = cci_write(ar0234->regmap, AR0234_RESET_REGISTER, AR0234_STANDBY,
			NULL);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int ar0234_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	ret = ar0234_start_streaming(ar0234, state);
	if (ret) {
		ar0234_stop_streaming(ar0234);
		pm_runtime_put_autosuspend(&client->dev);
		return ret;
	}

	/* READ_MODE only latches in standby. */
	__v4l2_ctrl_grab(ar0234->vflip, true);
	__v4l2_ctrl_grab(ar0234->hflip, true);

	return 0;
}

static int ar0234_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ar0234_stop_streaming(ar0234);
	__v4l2_ctrl_grab(ar0234->vflip, false);
	__v4l2_ctrl_grab(ar0234->hflip, false);
	pm_runtime_put_autosuspend(&client->dev);

	return 0;
}

/* Power/clock management functions */
static int ar0234_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);
	int ret;

	ret = regulator_bulk_enable(AR0234_NUM_SUPPLIES,
					ar0234->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(ar0234->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(ar0234->reset_gpio, 1);
	usleep_range(AR0234_XCLR_MIN_DELAY_US,
			AR0234_XCLR_MIN_DELAY_US + AR0234_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(AR0234_NUM_SUPPLIES, ar0234->supplies);

	return ret;
}

static int ar0234_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	gpiod_set_value_cansleep(ar0234->reset_gpio, 0);
	regulator_bulk_disable(AR0234_NUM_SUPPLIES, ar0234->supplies);
	clk_disable_unprepare(ar0234->xclk);

	return 0;
}

static int ar0234_get_regulators(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	unsigned int i;

	for (i = 0; i < AR0234_NUM_SUPPLIES; i++)
		ar0234->supplies[i].supply = ar0234_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
					AR0234_NUM_SUPPLIES,
					ar0234->supplies);
}

/* Verify chip ID */
static int ar0234_identify_module(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	unsigned int retry;
	int ret;
	u64 val;

	for (retry = 0; retry < 5; retry++) {
		ret = cci_read(ar0234->regmap, AR0234_MODEL_ID, &val, NULL);
		if (ret) {
			dev_warn(&client->dev,
				 "failed to read chip id %d (attempt %u/5)\n",
				 ret, retry + 1);
			continue;
		}

		if (val == AR0234_MODEL_ID_COLOR || val == AR0234_MODEL_ID_MONO) {
			ar0234->monochrome = val == AR0234_MODEL_ID_MONO;
			break;
		}

		dev_warn(&client->dev,
			 "chip id mismatch: %x!=%llx (attempt %u/5)\n",
			 AR0234_MODEL_ID_COLOR, val, retry + 1);
		ret = -EIO;
	}
	if (ret) {
		dev_err(&client->dev, "failed to identify chip id %d\n",
			ret);
		return ret;
	}

	dev_dbg(&client->dev, "model id %llx\n", val);

	return 0;
}

static const struct v4l2_subdev_core_ops ar0234_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ar0234_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static int ar0234_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	u32 code;

	if (pad != IMAGE_PAD)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, IMAGE_PAD, 0)->code;
	v4l2_subdev_unlock_state(state);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry->pixelcode = code;
	fd->entry->stream = 0;
	fd->entry->bus.csi2.vc = 0; /* Read from device tree */
	/* The descriptor must follow the format or the receiver unpacks a wrong DT. */
	fd->entry->bus.csi2.dt = ar0234_depth_of(to_ar0234(sd), code)->data_type;

	return 0;
}

static const struct v4l2_subdev_pad_ops ar0234_pad_ops = {
	.enum_mbus_code = ar0234_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar0234_set_pad_format,
	.get_selection = ar0234_get_selection,
	.enum_frame_size = ar0234_enum_frame_size,
	.get_frame_desc = ar0234_get_frame_desc,
	.enable_streams = ar0234_enable_streams,
	.disable_streams = ar0234_disable_streams,
};

static const struct v4l2_subdev_ops ar0234_subdev_ops = {
	.core = &ar0234_core_ops,
	.video = &ar0234_video_ops,
	.pad = &ar0234_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0234_internal_ops = {
	.init_state = ar0234_init_state,
};

/* Initialize control handlers */
static int ar0234_init_controls(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	const struct ar0234_mode *mode = &supported_modes[0];
	const struct ar0234_depth *depth =
		ar0234_depth_of(ar0234, ar0234_first_code(ar0234));
	unsigned int height = mode->height;
	u64 pixel_rate = ar0234_pixel_rate(ar0234, depth);
	int exposure_max, exposure_def, hblank;
	int i, ret;

	ctrl_hdlr = &ar0234->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	/* By default, PIXEL_RATE is read only */
	ar0234->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						pixel_rate, pixel_rate, 1,
						pixel_rate);
	if (ar0234->pixel_rate)
		ar0234->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initial vblank/hblank/exposure parameters based on current mode */
	ar0234->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_VBLANK, AR0234_VBLANK_MIN,
					AR0234_FRAME_LENGTH_LINES_MAX - height, 1,
					mode->fll_def - height);
	hblank = AR0234_PIXELS_PER_LINE - mode->width;
	ar0234->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_HBLANK, hblank, hblank,
					1, hblank);
	if (ar0234->hblank)
		ar0234->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = AR0234_EXPOSURE_MAX_FOR(mode->fll_def);
	exposure_def = min(exposure_max, AR0234_EXPOSURE_DEFAULT);
	ar0234->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
						V4L2_CID_EXPOSURE,
						AR0234_EXPOSURE_MIN, exposure_max,
						AR0234_EXPOSURE_STEP,
						exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			AR0234_ANA_GAIN_MIN, AR0234_ANA_GAIN_MAX,
			AR0234_ANA_GAIN_STEP, AR0234_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			AR0234_DGTL_GAIN_MIN, AR0234_DGTL_GAIN_MAX,
			AR0234_DGTL_GAIN_STEP, AR0234_DGTL_GAIN_DEFAULT);

	ar0234->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_HFLIP, 0, 1, 1, 0);

	ar0234->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(ar0234_test_pattern_menu) - 1,
					0, 0, ar0234_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		* The assumption is that
		* V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		* V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		* V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		*/
		v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
				V4L2_CID_TEST_PATTERN_RED + i,
				AR0234_TESTP_COLOUR_MIN,
				AR0234_TESTP_COLOUR_MAX,
				AR0234_TESTP_COLOUR_STEP,
				AR0234_TESTP_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	ar0234->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &ar0234_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ar0234->num_link_freqs - 1,
						   max(ar0234_link_index(ar0234, depth), 0),
						   ar0234->link_freq_menu);
	if (ar0234->link_freq)
		ar0234->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ar0234_ctrl_ops,
						&props);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ar0234->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void ar0234_free_controls(struct ar0234 *ar0234)
{
	v4l2_ctrl_handler_free(ar0234->sd.ctrl_handler);
}

/*
 * Realise the link frequency exactly: FVCO = link * sys divider in range, PRE_PLL_CLK_DIV 2 then 3
 * (the power-on value), and an even multiplier; the 2 * 2 layout keeps the LI register values.
 */
static int ar0234_pll_solve(struct device *dev, u64 link_freq,
			    struct ar0234_pll *pll)
{
	static const u8 sys_divs[] = { 2, 1 };
	static const u8 pre_divs[] = { 2, 3 };
	unsigned int i, j;

	/* Bound the DT value before multiplying; the lane rate is twice the link frequency. */
	if (link_freq < AR0234_FSERIAL_MIN_BPS / 2 ||
	    link_freq > AR0234_FSERIAL_MAX_BPS / 2) {
		dev_err(dev, "link frequency %llu Hz: lane rate outside %llu-%llu bit/s\n",
			link_freq, AR0234_FSERIAL_MIN_BPS, AR0234_FSERIAL_MAX_BPS);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(sys_divs); i++) {
		u64 fvco = link_freq * sys_divs[i];

		if (fvco < AR0234_FVCO_MIN_HZ || fvco > AR0234_FVCO_MAX_HZ)
			continue;

		for (j = 0; j < ARRAY_SIZE(pre_divs); j++) {
			u32 rem;
			u64 mult = div_u64_rem(fvco * pre_divs[j],
					       AR0234_XCLK_FREQ, &rem);

			if (rem || (mult & 1) || mult > AR0234_PLL_MULTIPLIER_MAX)
				continue;

			pll->pre_pll_clk_div = pre_divs[j];
			pll->pll_multiplier = mult;
			pll->vt_sys_clk_div = sys_divs[i];
			pll->op_sys_clk_div = sys_divs[i];
			return 0;
		}
	}

	dev_err(dev, "link frequency %llu Hz has no even PLL multiplier in range\n",
		link_freq);
	return -EINVAL;
}

static int ar0234_check_hwcfg(struct ar0234 *ar0234, struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	unsigned int i;
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

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2 &&
	    ep_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "only 2 or 4 data lanes are supported\n");
		goto error_out;
	}
	ar0234->num_data_lanes = ep_cfg.bus.mipi_csi2.num_data_lanes;

	if (!ep_cfg.nr_of_link_frequencies ||
	    ep_cfg.nr_of_link_frequencies > AR0234_LINK_FREQS_MAX) {
		dev_err(dev, "1 to %u link-frequencies are required\n",
			AR0234_LINK_FREQS_MAX);
		goto error_out;
	}

	/* The solver realises each frequency exactly, so the menu holds the DT values. */
	for (i = 0; i < ep_cfg.nr_of_link_frequencies; i++) {
		if (ar0234_pll_solve(dev, ep_cfg.link_frequencies[i],
				     &ar0234->plls[i]))
			goto error_out;
		ar0234->link_freq_menu[i] = ep_cfg.link_frequencies[i];
	}
	ar0234->num_link_freqs = ep_cfg.nr_of_link_frequencies;

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int ar0234_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0234 *ar0234;
	int ret;

	ar0234 = devm_kzalloc(&client->dev, sizeof(*ar0234), GFP_KERNEL);
	if (!ar0234)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ar0234->sd, client, &ar0234_subdev_ops);

	ar0234->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ar0234->regmap)) {
		ret = PTR_ERR(ar0234->regmap);
		dev_err(dev, "failed to initialise CCI: %d\n", ret);
		return ret;
	}

	/* Check the hardware configuration in device tree */
	ret = ar0234_check_hwcfg(ar0234, dev);
	if (ret)
		return ret;

	/* Get system clock (xclk) */
	ar0234->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(ar0234->xclk)) {
		if (PTR_ERR(ar0234->xclk) != -EPROBE_DEFER)
			dev_err(dev, "failed to get xclk %ld\n", PTR_ERR(ar0234->xclk));
		return PTR_ERR(ar0234->xclk);
	}

	ar0234->xclk_freq = clk_get_rate(ar0234->xclk);
	if (ar0234->xclk_freq != AR0234_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			ar0234->xclk_freq);
		return -EINVAL;
	}

	ret = ar0234_get_regulators(ar0234);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	ar0234->reset_gpio = devm_gpiod_get_optional(dev, "reset",
							GPIOD_OUT_HIGH);

	/*
	* The sensor must be powered for ar0234_identify_module()
	* to be able to read the CHIP_ID register
	*/
	ret = ar0234_power_on(dev);
	if (ret)
		return ret;

	ret = ar0234_identify_module(ar0234);
	if (ret)
		goto error_power_off;

	/* sensor doesn't enter LP-11 state upon power up until and unless
	* streaming is started, so upon power up switch the modes to:
	* streaming -> standby
	*/
	ret = cci_write(ar0234->regmap, AR0234_RESET_REGISTER, AR0234_STREAMING,
			NULL);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	/* put sensor back to standby mode */
	ret = cci_write(ar0234->regmap, AR0234_RESET_REGISTER, AR0234_STANDBY,
			NULL);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	ret = ar0234_init_controls(ar0234);
	if (ret)
		goto error_power_off;

	/* One lock for the controls and the state: s_ctrl reads the format. */
	ar0234->sd.state_lock = ar0234->ctrl_handler.lock;

	/* Initialize subdev */
	ar0234->sd.internal_ops = &ar0234_internal_ops;
	ar0234->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	ar0234->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	ar0234->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ar0234->sd.entity, NUM_PADS, ar0234->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&ar0234->sd);
	if (ret) {
		dev_err(dev, "failed to init subdev state: %d\n", ret);
		goto error_media_entity;
	}

	/* Runtime PM comes up before the subdev; the sensor sleeps a second after its last user. */
	pm_runtime_set_active(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&ar0234->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_pm;
	}

	pm_runtime_idle(dev);

	return 0;

error_pm:
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&ar0234->sd);

error_media_entity:
	media_entity_cleanup(&ar0234->sd.entity);

error_handler_free:
	ar0234_free_controls(ar0234);

error_power_off:
	ar0234_power_off(dev);

	return ret;
}

static void ar0234_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	ar0234_free_controls(ar0234);

	pm_runtime_disable(&client->dev);
	pm_runtime_dont_use_autosuspend(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ar0234_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id ar0234_dt_ids[] = {
	{ .compatible = "onsemi,ar0234" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0234_dt_ids);

static const struct dev_pm_ops ar0234_pm_ops = {
	SET_RUNTIME_PM_OPS(ar0234_power_off, ar0234_power_on, NULL)
};

static struct i2c_driver ar0234_i2c_driver = {
	.driver = {
		.name = "ar0234",
		.of_match_table	= ar0234_dt_ids,
		.pm = &ar0234_pm_ops,
	},
	.probe = ar0234_probe,
	.remove = ar0234_remove,
};

module_i2c_driver(ar0234_i2c_driver);

MODULE_AUTHOR("zihongl@leopardimaging.com");
MODULE_AUTHOR("Punit Singh <punit.singh@sima.ai>");
MODULE_DESCRIPTION("onsemi AR0234 sensor driver");
MODULE_LICENSE("GPL");
