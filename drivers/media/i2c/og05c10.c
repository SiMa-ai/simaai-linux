// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the OmniVision OG05C10 image sensor.
 *
 * Copyright (C) 2026 SiMa Technologies, Inc.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#include "og05c10_regs.h"

enum og05c10_pad {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct og05c10_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

/* Mode data is grouped by the V4L2 API and control that consumes it. */
struct og05c10_pll_config {
	unsigned long xclk_freq;
	s64 link_freq;
	u64 pixel_rate;
	u32 sa1_freq;
};

struct og05c10_crop_config {
	struct v4l2_rect native;
	struct v4l2_rect active;
};

struct og05c10_resolution_config {
	u32 width;
	u32 height;
	u32 mbus_code;
	u8 csi_data_type;
	u32 embedded_width;
	u32 embedded_height;
};

struct og05c10_fps_config {
	u32 vts_reg;
	u32 line_length_pixels;
	u32 hts;
	u32 vts_default;
	u32 vts_max;
	u32 linear_overhead_ns;
	struct v4l2_fract def;
};

struct og05c10_control_config {
	u32 reg;
	u32 min;
	u32 max;
	u32 step;
	u32 def;
	u32 margin;
	u8 value_shift;
};

struct og05c10_mode {
	struct og05c10_pll_config pll;
	struct og05c10_crop_config crop;
	struct og05c10_resolution_config resolution;
	struct og05c10_fps_config fps;
	struct og05c10_control_config exposure;
	struct og05c10_control_config analogue_gain;
	struct og05c10_reg_list reg_list;
};

static const s64 og05c10_link_freq_menu[] = {
	OG05C10_LINK_FREQ,
};

static const struct og05c10_mode og05c10_mode = {
	.pll = {
		.xclk_freq = OG05C10_XCLK_FREQ,
		.link_freq = OG05C10_LINK_FREQ,
		.pixel_rate = OG05C10_PIXEL_RATE,
		.sa1_freq = OG05C10_PLL_SA1_HZ,
	},
	.crop = {
		.native = {
			.left = OG05C10_NATIVE_LEFT,
			.top = OG05C10_NATIVE_TOP,
			.width = OG05C10_NATIVE_WIDTH,
			.height = OG05C10_NATIVE_HEIGHT,
		},
		.active = {
			.left = OG05C10_CROP_LEFT,
			.top = OG05C10_CROP_TOP,
			.width = OG05C10_CROP_WIDTH,
			.height = OG05C10_CROP_HEIGHT,
		},
	},
	.resolution = {
		.width = OG05C10_WIDTH,
		.height = OG05C10_HEIGHT,
		.mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.csi_data_type = MIPI_CSI2_DT_RAW12,
		.embedded_width = OG05C10_EMBEDDED_LINE_WIDTH,
		.embedded_height = OG05C10_NUM_EMBEDDED_LINES,
	},
	/*
	 * The measured native rate is 235/8 = 29.375 fps. It follows the
	 * sensor timing equation at VTS 2152 and must not be rounded to 30.
	 */
	.fps = {
		.vts_reg = OG05C10_REG_VTS,
		.line_length_pixels = OG05C10_LINE_LENGTH_PIXELS,
		.hts = OG05C10_HTS,
		.vts_default = OG05C10_VTS_DEFAULT,
		.vts_max = OG05C10_VTS_MAX,
		.linear_overhead_ns = OG05C10_LINEAR_OVERHEAD_NS,
		.def = {
			.numerator = 8,
			.denominator = 235,
		},
	},
	.exposure = {
		.reg = OG05C10_REG_EXPOSURE,
		.min = OG05C10_EXPOSURE_MIN,
		.max = OG05C10_VTS_MAX / 2 - OG05C10_EXPOSURE_MARGIN,
		.step = OG05C10_EXPOSURE_STEP,
		.def = OG05C10_EXPOSURE_DEFAULT,
		.margin = OG05C10_EXPOSURE_MARGIN,
	},
	.analogue_gain = {
		.reg = OG05C10_REG_ANALOGUE_GAIN,
		.min = OG05C10_ANALOGUE_GAIN_MIN,
		.max = OG05C10_ANALOGUE_GAIN_MAX,
		.step = OG05C10_ANALOGUE_GAIN_STEP,
		.def = OG05C10_ANALOGUE_GAIN_DEFAULT,
		.value_shift = OG05C10_ANALOGUE_GAIN_SHIFT,
	},
	.reg_list = {
		.regs = og05c10_1920x1080_regs,
		.num_regs = ARRAY_SIZE(og05c10_1920x1080_regs),
	},
};

static const char *const og05c10_supply_names[] = {
	"VANA", /* Analog (2.8V) supply */
	"VDIG", /* Digital core (1.05V) supply */
	"VDDL", /* I/O (1.8V) supply */
};

#define OG05C10_NUM_SUPPLIES ARRAY_SIZE(og05c10_supply_names)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define OG05C10_XCLR_MIN_DELAY_US	8000
#define OG05C10_XCLR_DELAY_RANGE_US	1000

struct og05c10 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];
	struct regmap *regmap;

	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OG05C10_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct og05c10_mode *mode;

	/* Serialize format, control and streaming state. */
	struct mutex mutex;
	bool streaming;
};

static inline struct og05c10 *to_og05c10(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct og05c10, sd);
}

static void og05c10_set_default_format(struct og05c10 *og05c10)
{
	og05c10->mode = &og05c10_mode;
}

static int og05c10_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct og05c10 *og05c10 = to_og05c10(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_state_get_format(fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&og05c10->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = og05c10_mode.resolution.width;
	try_fmt_img->height = og05c10_mode.resolution.height;
	try_fmt_img->code = og05c10_mode.resolution.mbus_code;
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = og05c10_mode.resolution.embedded_width;
	try_fmt_meta->height = og05c10_mode.resolution.embedded_height;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_state_get_crop(fh->state, IMAGE_PAD);
	*try_crop = og05c10_mode.crop.active;

	mutex_unlock(&og05c10->mutex);

	return 0;
}

static int og05c10_set_exposure(struct og05c10 *og05c10, u32 exposure)
{
	const struct og05c10_control_config *config = &og05c10->mode->exposure;

	return cci_write(og05c10->regmap, config->reg,
			 exposure << config->value_shift, NULL);
}

static int og05c10_set_analogue_gain(struct og05c10 *og05c10, u32 gain)
{
	const struct og05c10_control_config *config =
		&og05c10->mode->analogue_gain;

	/* The sensor stores its Q4 gain code left-shifted by four bits. */
	return cci_write(og05c10->regmap, config->reg,
			 gain << config->value_shift, NULL);
}

static unsigned int og05c10_get_frame_length(const struct og05c10_mode *mode,
					     const struct v4l2_fract *timeperframe);

static u32 og05c10_frame_length_to_vts(const struct og05c10_mode *mode,
				       u32 frame_length)
{
	const struct og05c10_fps_config *fps = &mode->fps;
	u64 frame_ns = (u64)frame_length * fps->line_length_pixels *
		       NSEC_PER_SEC;
	u32 row_ns = div_u64((u64)fps->hts * NSEC_PER_SEC,
			     mode->pll.sa1_freq);
	u32 vts;

	frame_ns = div_u64(frame_ns, mode->pll.pixel_rate);
	if (frame_ns > fps->linear_overhead_ns)
		vts = 2 * (div_u64(frame_ns - fps->linear_overhead_ns,
				   row_ns) + 2);
	else
		vts = fps->vts_default;

	return clamp_t(u32, vts, fps->vts_default, fps->vts_max);
}

static int og05c10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og05c10 *og05c10 =
		container_of(ctrl->handler, struct og05c10, ctrl_handler);
	const struct og05c10_control_config *exposure =
		&og05c10->mode->exposure;
	u32 frame_length = og05c10->mode->resolution.height +
			   og05c10->vblank->val;
	u32 vts = og05c10_frame_length_to_vts(og05c10->mode, frame_length);
	u32 exposure_max;
	int pm_ret;
	int ret;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = clamp_t(u32, vts / 2 - exposure->margin,
				       exposure->min, exposure->max);
		__v4l2_ctrl_modify_range(og05c10->exposure,
					 og05c10->exposure->minimum,
					 exposure_max,
					 og05c10->exposure->step,
					 min_t(u32, exposure->def,
					       exposure_max));
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	pm_ret = pm_runtime_get_if_in_use(og05c10->dev);
	if (pm_ret <= 0)
		return pm_ret < 0 ? pm_ret : 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = og05c10_set_analogue_gain(og05c10, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = og05c10_set_exposure(og05c10, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(og05c10->regmap,
				og05c10->mode->fps.vts_reg, vts,
				NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(og05c10->dev);

	return ret;
}

static const struct v4l2_ctrl_ops og05c10_ctrl_ops = {
	.s_ctrl = og05c10_set_ctrl,
};

static int og05c10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index)
			return -EINVAL;

		code->code = og05c10_mode.resolution.mbus_code;
	} else {
		if (code->index)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int og05c10_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		if (fse->index ||
		    fse->code != og05c10_mode.resolution.mbus_code)
			return -EINVAL;

		fse->min_width = og05c10_mode.resolution.width;
		fse->max_width = fse->min_width;
		fse->min_height = og05c10_mode.resolution.height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = og05c10_mode.resolution.embedded_width;
		fse->max_width = fse->min_width;
		fse->min_height = og05c10_mode.resolution.embedded_height;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static int og05c10_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	if (pad != IMAGE_PAD)
		return -EINVAL;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].pixelcode = og05c10_mode.resolution.mbus_code;
	fd->entry[0].stream = 0;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = og05c10_mode.resolution.csi_data_type;

	return 0;
}

static void og05c10_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void og05c10_update_image_pad_format(const struct og05c10_mode *mode,
					    struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->resolution.width;
	fmt->format.height = mode->resolution.height;
	fmt->format.code = mode->resolution.mbus_code;
	fmt->format.field = V4L2_FIELD_NONE;
	og05c10_reset_colorspace(&fmt->format);
}

static void
og05c10_update_metadata_pad_format(const struct og05c10_mode *mode,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->resolution.embedded_width;
	fmt->format.height = mode->resolution.embedded_height;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int og05c10_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct og05c10 *og05c10 = to_og05c10(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&og05c10->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *try_fmt;
	} else if (fmt->pad == IMAGE_PAD) {
		og05c10_update_image_pad_format(og05c10->mode, fmt);
	} else {
		og05c10_update_metadata_pad_format(og05c10->mode, fmt);
	}

	mutex_unlock(&og05c10->mutex);
	return 0;
}

static unsigned int
og05c10_get_frame_length(const struct og05c10_mode *mode,
			 const struct v4l2_fract *timeperframe)
{
	u64 denominator;

	denominator = (u64)timeperframe->denominator *
		      mode->fps.line_length_pixels;

	return max_t(unsigned int,
			 div64_u64((u64)timeperframe->numerator *
				   mode->pll.pixel_rate + denominator / 2,
				   denominator),
			 mode->resolution.height);
}

static unsigned int og05c10_vts_to_frame_length(const struct og05c10_mode *mode,
						u32 vts)
{
	u64 row_ns = div_u64((u64)mode->fps.hts * NSEC_PER_SEC,
			     mode->pll.sa1_freq);
	u64 frame_ns = (vts / 2 - 2) * row_ns +
		       mode->fps.linear_overhead_ns;
	u64 denominator = (u64)mode->fps.line_length_pixels * NSEC_PER_SEC;

	return div64_u64(frame_ns * mode->pll.pixel_rate + denominator / 2,
			 denominator);
}

static void og05c10_set_framing_limits(struct og05c10 *og05c10)
{
	unsigned int frame_length_default, frame_length_max, hblank;
	const struct og05c10_mode *mode = og05c10->mode;

	frame_length_default = og05c10_vts_to_frame_length(mode,
							   mode->fps.vts_default);
	frame_length_max = og05c10_vts_to_frame_length(mode, mode->fps.vts_max);
	__v4l2_ctrl_modify_range(og05c10->vblank,
				 frame_length_default - mode->resolution.height,
				 frame_length_max - mode->resolution.height, 1,
				 frame_length_default - mode->resolution.height);

	__v4l2_ctrl_s_ctrl(og05c10->vblank,
			   frame_length_default - mode->resolution.height);

	hblank = mode->fps.line_length_pixels - mode->resolution.width;
	__v4l2_ctrl_modify_range(og05c10->hblank, hblank, hblank, 1, hblank);
}

static int og05c10_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	struct og05c10 *og05c10 = to_og05c10(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&og05c10->mutex);

	if (fmt->pad == IMAGE_PAD) {
		og05c10_update_image_pad_format(&og05c10_mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		} else {
			og05c10->mode = &og05c10_mode;
			og05c10_set_framing_limits(og05c10);
		}
	} else {
		og05c10_update_metadata_pad_format(&og05c10_mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		}
	}

	mutex_unlock(&og05c10->mutex);

	return 0;
}

static const struct v4l2_rect *
__og05c10_get_pad_crop(struct og05c10 *og05c10,
		       struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &og05c10->mode->crop.active;
	}

	return NULL;
}

static int og05c10_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->pad != IMAGE_PAD)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct og05c10 *og05c10 = to_og05c10(sd);
		const struct v4l2_rect *crop;

		mutex_lock(&og05c10->mutex);
		crop = __og05c10_get_pad_crop(og05c10, sd_state, sel->pad,
					      sel->which);
		if (crop)
			sel->r = *crop;
		mutex_unlock(&og05c10->mutex);

		return crop ? 0 : -EINVAL;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = og05c10_mode.crop.native;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = og05c10_mode.crop.active;

		return 0;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = og05c10_mode.crop.native;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int og05c10_start_streaming(struct og05c10 *og05c10)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	const struct og05c10_reg_list *reg_list;
	int ret;

	/* Apply default values of current mode */
	reg_list = &og05c10->mode->reg_list;
	ret = cci_multi_reg_write(og05c10->regmap, reg_list->regs,
				  reg_list->num_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(og05c10->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */

	return cci_write(og05c10->regmap, OG05C10_REG_MODE_SELECT,
			 OG05C10_MODE_STREAMING, NULL);
}

/* Stop streaming */
static void og05c10_stop_streaming(struct og05c10 *og05c10)
{
	/* set stream off register */

	/* Stop driving XVS out (there is still a weak pull-up) */
	cci_write(og05c10->regmap, OG05C10_REG_MODE_SELECT,
		  OG05C10_MODE_STANDBY, NULL);
}

static int og05c10_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct og05c10 *og05c10 = to_og05c10(sd);
	int ret = 0;

	mutex_lock(&og05c10->mutex);
	if (og05c10->streaming == enable) {
		mutex_unlock(&og05c10->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(og05c10->dev);
		if (ret < 0)
			goto err_unlock;

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = og05c10_start_streaming(og05c10);
		if (ret)
			goto err_rpm_put;
	} else {
		og05c10_stop_streaming(og05c10);
		pm_runtime_put_autosuspend(og05c10->dev);
	}

	og05c10->streaming = enable;

	mutex_unlock(&og05c10->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put_sync(og05c10->dev);
err_unlock:
	mutex_unlock(&og05c10->mutex);

	return ret;
}

/* Power/clock management functions */
static int og05c10_power_on(struct device *dev)
{
	struct og05c10 *og05c10 = to_og05c10(i2c_get_clientdata(to_i2c_client(dev)));
	int ret;

	ret = regulator_bulk_enable(OG05C10_NUM_SUPPLIES,
				    og05c10->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(og05c10->xclk);
	if (ret)
		goto err_disable_regulators;

	gpiod_set_value_cansleep(og05c10->reset_gpio, 0);

	usleep_range(OG05C10_XCLR_MIN_DELAY_US,
		     OG05C10_XCLR_MIN_DELAY_US + OG05C10_XCLR_DELAY_RANGE_US);

	return 0;

err_disable_regulators:
	regulator_bulk_disable(OG05C10_NUM_SUPPLIES, og05c10->supplies);
	return ret;
}

static int og05c10_power_off(struct device *dev)
{
	struct og05c10 *og05c10 = to_og05c10(i2c_get_clientdata(to_i2c_client(dev)));

	gpiod_set_value_cansleep(og05c10->reset_gpio, 1);
	clk_disable_unprepare(og05c10->xclk);
	regulator_bulk_disable(OG05C10_NUM_SUPPLIES, og05c10->supplies);

	return 0;
}

static int og05c10_get_regulators(struct og05c10 *og05c10)
{
	unsigned int i;

	for (i = 0; i < OG05C10_NUM_SUPPLIES; i++)
		og05c10->supplies[i].supply = og05c10_supply_names[i];

	return devm_regulator_bulk_get(og05c10->dev,
				       OG05C10_NUM_SUPPLIES,
				       og05c10->supplies);
}

static int og05c10_get_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_interval *fi)
{
	struct og05c10 *og05c10 = to_og05c10(sd);
	const struct og05c10_mode *mode;
	unsigned int frame_length;

	if (fi->pad != IMAGE_PAD)
		return -EINVAL;

	mutex_lock(&og05c10->mutex);
	mode = og05c10->mode;
	frame_length = mode->resolution.height + og05c10->vblank->val;
	fi->interval.numerator = frame_length * mode->fps.def.numerator;
	fi->interval.denominator =
		og05c10_get_frame_length(mode, &mode->fps.def) *
		mode->fps.def.denominator;
	mutex_unlock(&og05c10->mutex);

	return 0;
}

static int og05c10_set_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_interval *fi)
{
	struct og05c10 *og05c10 = to_og05c10(sd);
	const struct og05c10_mode *mode;
	unsigned int frame_length;
	int ret;

	if (fi->pad != IMAGE_PAD)
		return -EINVAL;

	mutex_lock(&og05c10->mutex);
	mode = og05c10->mode;
	frame_length = og05c10_get_frame_length(mode, &fi->interval);
	frame_length = clamp_t(unsigned int, frame_length,
			       mode->resolution.height + og05c10->vblank->minimum,
			       mode->resolution.height + og05c10->vblank->maximum);
	ret = __v4l2_ctrl_s_ctrl(og05c10->vblank,
				 frame_length - mode->resolution.height);
	if (ret) {
		mutex_unlock(&og05c10->mutex);
		return ret;
	}
	/* Report the actual (clamped) interval back. */
	frame_length = mode->resolution.height + og05c10->vblank->val;
	fi->interval.numerator = frame_length * mode->fps.def.numerator;
	fi->interval.denominator =
		og05c10_get_frame_length(mode, &mode->fps.def) *
		mode->fps.def.denominator;
	mutex_unlock(&og05c10->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops og05c10_video_ops = {
	.s_stream = og05c10_set_stream,
};

static const struct v4l2_subdev_pad_ops og05c10_pad_ops = {
	.get_frame_desc = og05c10_get_frame_desc,
	.enum_mbus_code = og05c10_enum_mbus_code,
	.get_fmt = og05c10_get_pad_format,
	.set_fmt = og05c10_set_pad_format,
	.get_selection = og05c10_get_selection,
	.enum_frame_size = og05c10_enum_frame_size,
	.get_frame_interval = og05c10_get_frame_interval,
	.set_frame_interval = og05c10_set_frame_interval,
};

static const struct v4l2_subdev_core_ops og05c10_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops og05c10_subdev_ops = {
	.core = &og05c10_core_ops,
	.video = &og05c10_video_ops,
	.pad = &og05c10_pad_ops,
};

static const struct v4l2_subdev_internal_ops og05c10_internal_ops = {
	.open = og05c10_open,
};

/* Initialize control handlers */
static int og05c10_init_controls(struct og05c10 *og05c10)
{
	const struct og05c10_control_config *analogue_gain =
		&og05c10->mode->analogue_gain;
	const struct og05c10_control_config *exposure =
		&og05c10->mode->exposure;
	const struct og05c10_mode *mode = og05c10->mode;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	u32 frame_length_default;
	u32 frame_length_max;
	u32 hblank;
	u32 exposure_max;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_fwnode_device_parse(og05c10->dev, &props);
	if (ret)
		return ret;

	ctrl_hdlr = &og05c10->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	mutex_init(&og05c10->mutex);
	ctrl_hdlr->lock = &og05c10->mutex;

	v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_PIXEL_RATE,
			  mode->pll.pixel_rate, mode->pll.pixel_rate, 1,
			  mode->pll.pixel_rate);

	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(og05c10_link_freq_menu) - 1,
				      0, og05c10_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	frame_length_default = og05c10_vts_to_frame_length(mode,
							   mode->fps.vts_default);
	frame_length_max = og05c10_vts_to_frame_length(mode,
						       mode->fps.vts_max);
	og05c10->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					    V4L2_CID_VBLANK,
					   frame_length_default -
					   mode->resolution.height,
					   frame_length_max -
					   mode->resolution.height,
					   1,
					   frame_length_default -
					   mode->resolution.height);

	hblank = mode->fps.line_length_pixels - mode->resolution.width;
	og05c10->hblank = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
					    V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (og05c10->hblank)
		og05c10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	exposure_max = clamp_t(u32,
			       mode->fps.vts_default / 2 - exposure->margin,
			       exposure->min, exposure->max);
	og05c10->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					     exposure->min,
					     exposure_max,
					     exposure->step,
					     exposure->def);

	v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  analogue_gain->min, analogue_gain->max,
			  analogue_gain->step, analogue_gain->def);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(og05c10->dev, "control init failed: %d\n", ret);
		goto error;
	}

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &og05c10_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	og05c10->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&og05c10->mutex);

	return ret;
}

static void og05c10_free_controls(struct og05c10 *og05c10)
{
	v4l2_ctrl_handler_free(og05c10->sd.ctrl_handler);
	mutex_destroy(&og05c10->mutex);
}

static int og05c10_check_hwcfg(struct device *dev)
{
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

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != og05c10_mode.pll.link_freq) {
		dev_err(dev, "link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct of_device_id og05c10_dt_ids[] = {
	{ .compatible = "OV,og05c10" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, og05c10_dt_ids);

static int og05c10_identify_module(struct og05c10 *og05c10)
{
	u64 val;
	int ret;

	ret = cci_read(og05c10->regmap, OG05C10_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(og05c10->dev, ret,
				     "failed to read chip ID\n");

	if (val != OG05C10_CHIP_ID)
		return dev_err_probe(og05c10->dev, -ENODEV,
				     "unexpected chip ID 0x%06llx\n", val);

	dev_info(og05c10->dev, "detected OG05C10 image sensor\n");

	return 0;
}

static int og05c10_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct og05c10 *og05c10;
	unsigned long xclk_freq;
	int ret;

	og05c10 = devm_kzalloc(dev, sizeof(*og05c10), GFP_KERNEL);
	if (!og05c10)
		return -ENOMEM;

	og05c10->dev = dev;
	v4l2_i2c_subdev_init(&og05c10->sd, client, &og05c10_subdev_ops);

	ret = og05c10_check_hwcfg(dev);
	if (ret)
		return ret;

	og05c10->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(og05c10->xclk))
		return dev_err_probe(dev, PTR_ERR(og05c10->xclk),
				     "failed to get xclk\n");

	xclk_freq = clk_get_rate(og05c10->xclk);
	if (xclk_freq != og05c10_mode.pll.xclk_freq)
		return dev_err_probe(dev, -EINVAL,
				     "xclk frequency %lu Hz is not supported\n",
				     xclk_freq);

	ret = og05c10_get_regulators(og05c10);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	og05c10->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(og05c10->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(og05c10->reset_gpio),
				     "failed to get reset GPIO\n");

	og05c10->regmap = devm_cci_regmap_init_i2c(client, OG05C10_REG_ADDR_BITS);
	if (IS_ERR(og05c10->regmap))
		return dev_err_probe(dev, PTR_ERR(og05c10->regmap),
				     "failed to initialise CCI regmap\n");

	ret = og05c10_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on sensor\n");

	og05c10_set_default_format(og05c10);

	ret = og05c10_identify_module(og05c10);
	if (ret)
		goto err_power_off;

	ret = og05c10_init_controls(og05c10);
	if (ret)
		goto err_power_off;

	og05c10->sd.internal_ops = &og05c10_internal_ops;
	og05c10->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;
	og05c10->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	og05c10->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	og05c10->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&og05c10->sd.entity, NUM_PADS, og05c10->pad);
	if (ret)
		goto err_free_controls;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&og05c10->sd);
	if (ret)
		goto err_disable_pm;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_disable_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	media_entity_cleanup(&og05c10->sd.entity);
err_free_controls:
	og05c10_free_controls(og05c10);
err_power_off:
	og05c10_power_off(dev);
	return ret;
}

static void og05c10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og05c10 *og05c10 = to_og05c10(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	og05c10_free_controls(og05c10);

	pm_runtime_disable(og05c10->dev);
	if (!pm_runtime_status_suspended(og05c10->dev))
		og05c10_power_off(og05c10->dev);
	pm_runtime_set_suspended(og05c10->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(og05c10_pm_ops, og05c10_power_off,
				 og05c10_power_on, NULL);

static struct i2c_driver og05c10_i2c_driver = {
	.driver = {
		.name = "og05c10",
		.of_match_table	= og05c10_dt_ids,
		.pm = pm_ptr(&og05c10_pm_ops),
	},
	.probe = og05c10_probe,
	.remove = og05c10_remove,
};

module_i2c_driver(og05c10_i2c_driver);

MODULE_DESCRIPTION("OmniVision OG05C10 image sensor driver");
MODULE_LICENSE("GPL");
