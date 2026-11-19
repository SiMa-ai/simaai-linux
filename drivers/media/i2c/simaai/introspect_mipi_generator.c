// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 placeholder subdev driver for an Introspect Technology SV3C-DPTX
 * MIPI CSI-2/D-PHY generator wired to the Modalix Waveshare carrier board
 * (cam1 connector, 2-lane).
 *
 * This is a derivative of the Sony IMX477 sensor driver, with all CCI/I2C
 * register I/O removed: the generator has no CCI control interface at all
 * (Pinetree controls it over USB/Ethernet from a separate host PC, not
 * through any bus the SoC can see). This driver exists purely to satisfy
 * the V4L2 async-subdev binding the dw-mipi-csi2 host driver requires on
 * its remote endpoint - it reports a fixed RAW12 (SRGGB12) format and does
 * not talk to any physical chip.
 *
 * Derived from the Sony IMX477 sensor driver:
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/mipi-csi2.h>

/*
 * NOTE: several module_params from the original sensor driver this was
 * derived from (dpc_enable, trigger_mode, fstrobe_enable, fstrobe_cont_trig,
 * fstrobe_width, fstrobe_delay) have been removed entirely - each one only
 * ever did anything by way of a CCI write in introspect_gen_start_streaming(),
 * all of which are gone in this driver. Keeping dead parameters around that
 * silently do nothing is exactly the kind of trap this driver is trying to
 * avoid.
 */

#define INTROSPECT_GEN_XCLK_FREQ		24000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define INTROSPECT_GEN_PIXEL_RATE		840000000

/* V_TIMING internal */
#define INTROSPECT_GEN_VBLANK_MIN		48
#define INTROSPECT_GEN_FRAME_LENGTH_MAX		0xffdc

/* H_TIMING internal */
#define INTROSPECT_GEN_LINE_LENGTH_MAX		0xfff0

/* Long exposure multiplier */
#define INTROSPECT_GEN_LONG_EXP_SHIFT_MAX	7

/* Exposure control */
#define INTROSPECT_GEN_EXPOSURE_OFFSET		22
#define INTROSPECT_GEN_EXPOSURE_MIN		4
#define INTROSPECT_GEN_EXPOSURE_STEP		1
#define INTROSPECT_GEN_EXPOSURE_DEFAULT		0x640
#define INTROSPECT_GEN_EXPOSURE_MAX		(INTROSPECT_GEN_FRAME_LENGTH_MAX - \
					 INTROSPECT_GEN_EXPOSURE_OFFSET)

/* Analog gain control */
#define INTROSPECT_GEN_ANA_GAIN_MIN		0
#define INTROSPECT_GEN_ANA_GAIN_MAX		978
#define INTROSPECT_GEN_ANA_GAIN_STEP		1
#define INTROSPECT_GEN_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */

  #define INTROSPECT_GEN_IOP_SYSCK_DIV		0x02
  #define INTROSPECT_GEN_IOP_PREDIV		0x02

/* Embedded metadata stream structure */
#define INTROSPECT_GEN_EMBEDDED_LINE_WIDTH 16384
#define INTROSPECT_GEN_NUM_EMBEDDED_LINES 1

/* Data lane mode */

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* INTROSPECT_GEN native and active pixel array size. */
#define INTROSPECT_GEN_NATIVE_WIDTH		4072U
#define INTROSPECT_GEN_NATIVE_HEIGHT		3176U
#define INTROSPECT_GEN_PIXEL_ARRAY_LEFT		8U
#define INTROSPECT_GEN_PIXEL_ARRAY_TOP		16U
#define INTROSPECT_GEN_PIXEL_ARRAY_WIDTH	4056U
#define INTROSPECT_GEN_PIXEL_ARRAY_HEIGHT	3040U

/* Mode : resolution and related config&values */
struct introspect_gen_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default frame_length value to use in this mode */
	unsigned int frm_length_default;
};

/*
 * All mode-specific and common CCI register-sequence arrays removed -
 * they were only ever consumed by the CCI writes stripped from
 * introspect_gen_start_streaming(); nothing reads them anymore.
 */


/* Mode configs */
static const struct introspect_gen_mode supported_modes[] = {
	{
		/* 1080p 34fps binned and cropped mode */
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = INTROSPECT_GEN_PIXEL_ARRAY_LEFT,
			.top = INTROSPECT_GEN_PIXEL_ARRAY_TOP + 440,
			.width = 4056,
			.height = 2160,
		},
		.frm_length_default = 2197,
	},
};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 *
 * Two separate 4-entry sets, not one combined array. Exposing both bit
 * depths to enum_mbus_code() at once was the original bug during bring-up: the pipeline handler renegotiates to
 * whichever entry comes first, independent of what the generator is actually transmitting,
 * silently mismatching the CSI-2 host's IPI_DATA_TYPE filter. Exactly one
 * of these two sets is selected once, at probe time, via the endpoint's
 * sima,pixel-bits DT property - see introspect_gen_check_hwcfg() - and
 * that choice is fixed for the driver instance's lifetime. Never expose
 * both sets to the negotiation layer simultaneously.
 */
#define CODES_PER_FORMAT 4
static const u32 codes_raw8[CODES_PER_FORMAT] = {
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
};
static const u32 codes_raw12[CODES_PER_FORMAT] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

/*
 * introspect_gen_test_pattern_menu / introspect_gen_test_pattern_val removed - TEST_PATTERN
 * is not registered as a control in this driver (see introspect_gen_init_controls
 * and introspect_gen_set_ctrl), so these would otherwise be unused-variable
 * warnings.
 */

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define INTROSPECT_GEN_XCLR_MIN_DELAY_US	8000
#define INTROSPECT_GEN_XCLR_DELAY_RANGE_US	1000

struct introspect_gen {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *link_freq;

	u64 link_freq_value;
	/*
	 * Backing store for the V4L2_CID_LINK_FREQ menu. v4l2_ctrl_new_int_menu()
	 * keeps the pointer, so this cannot be a stack local.
	 */
	s64 link_freq_menu[1];

	/* Virtual channel this generator's stream is tagged with, read from
	 * the endpoint's sima,virtual-channel DT property (defaults to 0/VC0
	 * if absent, matching the CSI-2 host driver's own default).
	 */
	u32 vc;

	/* Which of codes_raw8[]/codes_raw12[] is active for this instance,
	 * read from the endpoint's sima,pixel-bits DT property (defaults to
	 * 12 if absent, matching the current RAW12 default). Fixed for the
	 * driver instance's lifetime - see codes_raw8[]/codes_raw12[]'s own
	 * comment for why this is never runtime-switched or exposed as
	 * multiple simultaneous enum_mbus_code() options.
	 */
	const u32 *codes;

	/* Current mode */
	const struct introspect_gen_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct introspect_gen *to_introspect_gen(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct introspect_gen, sd);
}

/* Get bayer order based on flip setting. */
static u32 introspect_gen_get_format_code(struct introspect_gen *introspect_gen, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&introspect_gen->mutex);

	for (i = 0; i < CODES_PER_FORMAT; i++)
		if (introspect_gen->codes[i] == code)
			break;

	if (i >= CODES_PER_FORMAT)
		i = 0;

	i = (i & ~3) | (introspect_gen->vflip->val ? 2 : 0) |
	    (introspect_gen->hflip->val ? 1 : 0);

	return introspect_gen->codes[i];
}

static void introspect_gen_set_default_format(struct introspect_gen *introspect_gen)
{
	/* Set default mode to max resolution */
	introspect_gen->mode = &supported_modes[0];
	/*
	 * RGGB order matches the Introspect generator's confirmed
	 * rawFormatBayerCell='RGGB' setting. Bit depth comes from whichever
	 * of codes_raw8[]/codes_raw12[] this instance selected via
	 * sima,pixel-bits (see that DT read in check_hwcfg() and
	 * codes_raw8[]/codes_raw12[]'s own comment for why only one bit
	 * depth is exposed at a time) - not hardcoded, so the declared
	 * default always matches what's actually negotiable.
	 */
	introspect_gen->fmt_code = introspect_gen->codes[0];
}

static int introspect_gen_get_bpp(unsigned int code)
{
	switch (code) {
	default:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return 12;

	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		return 8;
	}
}

static u8 introspect_gen_get_dt(u32 code)
{
	switch (code) {
	default:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return MIPI_CSI2_DT_RAW12;

	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		return MIPI_CSI2_DT_RAW8;
	}
}

static int introspect_gen_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_frame_desc *fd)
{
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);
	struct v4l2_subdev_state *state;
	u32 code;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, IMAGE_PAD)->code;
	v4l2_subdev_unlock_state(state);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry->pixelcode = code;
	fd->entry->stream = 0;
	fd->entry->bus.csi2.vc = introspect_gen->vc;
	fd->entry->bus.csi2.dt = introspect_gen_get_dt(code);

	return 0;
}

static int introspect_gen_set_format(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;
	return 0;
}

static int introspect_gen_init_state(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = INTROSPECT_GEN_NATIVE_WIDTH,
			.height = INTROSPECT_GEN_NATIVE_HEIGHT,
		},
	};

	introspect_gen_set_format(sd, state, &format);

	return 0;
}

static int introspect_gen_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_state_get_format(fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&introspect_gen->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = introspect_gen_get_format_code(introspect_gen,
						   MEDIA_BUS_FMT_SRGGB12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = INTROSPECT_GEN_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = INTROSPECT_GEN_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_state_get_crop(fh->state, IMAGE_PAD);
	try_crop->left = INTROSPECT_GEN_PIXEL_ARRAY_LEFT;
	try_crop->top = INTROSPECT_GEN_PIXEL_ARRAY_TOP;
	try_crop->width = INTROSPECT_GEN_PIXEL_ARRAY_WIDTH;
	try_crop->height = INTROSPECT_GEN_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&introspect_gen->mutex);

	return 0;
}

static void introspect_gen_adjust_exposure_range(struct introspect_gen *introspect_gen)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = introspect_gen->mode->height + introspect_gen->vblank->val -
		       INTROSPECT_GEN_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, introspect_gen->exposure->val);
	__v4l2_ctrl_modify_range(introspect_gen->exposure, introspect_gen->exposure->minimum,
				 exposure_max, introspect_gen->exposure->step,
				 exposure_def);
}

static int introspect_gen_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct introspect_gen *introspect_gen =
		container_of(ctrl->handler, struct introspect_gen, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&introspect_gen->sd);
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		introspect_gen_adjust_exposure_range(introspect_gen);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	/*
	 * ANALOGUE_GAIN, DIGITAL_GAIN, and TEST_PATTERN(+4 colour channels)
	 * are fully removed - both their v4l2_ctrl_new_std() registration in
	 * introspect_gen_init_controls() and their cases here. Verified they have no
	 * other referrers anywhere in this file, unlike HFLIP/VFLIP/VBLANK/
	 * HBLANK/EXPOSURE below.
	 */
	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		/*
		 * No CCI write: registered solely because libcamera's
		 * mandatory-control check requires it to exist - see the
		 * note above introspect_gen_init_controls(). No physical effect.
		 */
		break;
	case V4L2_CID_EXPOSURE:
		/*
		 * No CCI write: the generator's output isn't affected by
		 * this value. Control is kept registered (not stripped)
		 * because introspect_gen_adjust_exposure_range() reads/writes
		 * introspect_gen->exposure directly from outside set_ctrl (called
		 * from the pre-switch VBLANK check above) - removing the
		 * control object would leave that a dangling pointer.
		 */
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/*
		 * No CCI write: nothing physically flips on the generator's
		 * output. Kept registered (not stripped) because
		 * introspect_gen_get_format_code() unconditionally reads
		 * introspect_gen->hflip->val / introspect_gen->vflip->val during format
		 * negotiation, and __v4l2_ctrl_grab() is called on both at
		 * stream start/stop - both are NULL-deref crashes if these
		 * controls don't exist.
		 */
		break;
	case V4L2_CID_VBLANK:
		/*
		 * No CCI write: kept registered for the same reason as
		 * HBLANK/EXPOSURE - the generator's output isn't affected.
		 * (An earlier version of this comment claimed a local
		 * long_exp_shift computation was still needed here; checked
		 * and found incorrect - nothing read it, so the computation
		 * and its function were removed entirely.)
		 */
		break;
	case V4L2_CID_HBLANK:
		/* No CCI write: kept registered for the same reason as VBLANK. */
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops introspect_gen_ctrl_ops = {
	.s_ctrl = introspect_gen_set_ctrl,
};

static int introspect_gen_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		/* Exactly one format group now (codes_raw8[] or codes_raw12[],
		 * whichever this instance selected) - see their own comment
		 * for why this is deliberately never more than one.
		 */
		if (code->index >= 1)
			return -EINVAL;

		mutex_lock(&introspect_gen->mutex);
		code->code = introspect_gen_get_format_code(introspect_gen,
						    introspect_gen->codes[code->index * 4]);
		mutex_unlock(&introspect_gen->mutex);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int introspect_gen_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);
	u32 code;

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		if (fse->index >= ARRAY_SIZE(supported_modes))
			return -EINVAL;

		mutex_lock(&introspect_gen->mutex);
		code = introspect_gen_get_format_code(introspect_gen, fse->code);
		mutex_unlock(&introspect_gen->mutex);

		if (fse->code != code)
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = INTROSPECT_GEN_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = INTROSPECT_GEN_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void introspect_gen_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void introspect_gen_update_image_pad_format(struct introspect_gen *introspect_gen,
					   const struct introspect_gen_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	introspect_gen_reset_colorspace(&fmt->format);
}

static void introspect_gen_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = INTROSPECT_GEN_EMBEDDED_LINE_WIDTH;
	fmt->format.height = INTROSPECT_GEN_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int introspect_gen_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&introspect_gen->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				introspect_gen_get_format_code(introspect_gen, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			introspect_gen_update_image_pad_format(introspect_gen, introspect_gen->mode,
						       fmt);
			fmt->format.code =
			       introspect_gen_get_format_code(introspect_gen, introspect_gen->fmt_code);
		} else {
			introspect_gen_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&introspect_gen->mutex);
	return 0;
}

static void introspect_gen_set_framing_limits(struct introspect_gen *introspect_gen)
{
	unsigned int hblank_min;
	u64 line_length_min;
	int bpp;
	const struct introspect_gen_mode *mode = introspect_gen->mode;

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(introspect_gen->vblank,
				 INTROSPECT_GEN_VBLANK_MIN,
				 ((1 << INTROSPECT_GEN_LONG_EXP_SHIFT_MAX) *
					INTROSPECT_GEN_FRAME_LENGTH_MAX) - mode->height,
				 1, mode->frm_length_default - mode->height);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(introspect_gen->vblank,
			   mode->frm_length_default - mode->height);

	bpp = introspect_gen_get_bpp(introspect_gen->fmt_code);

	line_length_min = mode->width * bpp * (u64)INTROSPECT_GEN_PIXEL_RATE;
	do_div(line_length_min, introspect_gen->link_freq_value * 2 * 2 /*LANES*/);
	/* Allow 500pixel clocks for HS<>LP transitions (approx 0.6usecs) */
	line_length_min += 500;

	hblank_min = line_length_min - mode->width;
	__v4l2_ctrl_modify_range(introspect_gen->hblank, hblank_min,
				 INTROSPECT_GEN_LINE_LENGTH_MAX, 1, hblank_min);
	__v4l2_ctrl_s_ctrl(introspect_gen->hblank, hblank_min);
}

static int introspect_gen_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct introspect_gen_mode *mode;
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&introspect_gen->mutex);

	if (fmt->pad == IMAGE_PAD) {
		/* Bayer order varies with flips */
		fmt->format.code = introspect_gen_get_format_code(introspect_gen,
							  fmt->format.code);

		mode = v4l2_find_nearest_size(supported_modes,
					      ARRAY_SIZE(supported_modes),
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		introspect_gen_update_image_pad_format(introspect_gen, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (introspect_gen->mode != mode ||
			   introspect_gen->fmt_code != fmt->format.code) {
			introspect_gen->mode = mode;
			introspect_gen->fmt_code = fmt->format.code;
			introspect_gen_set_framing_limits(introspect_gen);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			introspect_gen_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&introspect_gen->mutex);

	return 0;
}

static const struct v4l2_rect *
__introspect_gen_get_pad_crop(struct introspect_gen *introspect_gen,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &introspect_gen->mode->crop;
	}

	return NULL;
}

static int introspect_gen_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct introspect_gen *introspect_gen = to_introspect_gen(sd);

		mutex_lock(&introspect_gen->mutex);
		sel->r = *__introspect_gen_get_pad_crop(introspect_gen, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&introspect_gen->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = INTROSPECT_GEN_NATIVE_WIDTH;
		sel->r.height = INTROSPECT_GEN_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = INTROSPECT_GEN_PIXEL_ARRAY_LEFT;
		sel->r.top = INTROSPECT_GEN_PIXEL_ARRAY_TOP;
		sel->r.width = INTROSPECT_GEN_PIXEL_ARRAY_WIDTH;
		sel->r.height = INTROSPECT_GEN_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int introspect_gen_start_streaming(struct introspect_gen *introspect_gen)
{
	struct i2c_client *client = v4l2_get_subdevdata(&introspect_gen->sd);
	int ret, bpp;

	/*
	 * All CCI writes removed - there is no sensor to configure. In the
	 * original driver this block wrote mode_common_regs/extra_regs, the
	 * PLL multiplier + link bit-rate registers, D-PHY control, lane
	 * mode, the mode's reg_list, fstrobe timing regs, CSI_DT_FMT_H/L +
	 * PXCK_DIV + ADBIT_MODE (derived from bpp below), on-sensor DPC
	 * enables, and the trigger-mode/stream-on sequence.
	 *
	 * Consequence worth keeping in mind: nothing in this driver sets the
	 * physical HS line rate, data type, or lane configuration on the
	 * wire anymore - that is now entirely Pinetree's responsibility on
	 * the generator side. The DT link-frequencies value (450MHz) is
	 * still validated by introspect_gen_check_hwcfg()'s pure-arithmetic check at
	 * probe time, independent of this function, so it still constrains
	 * what's a valid DT value even though nothing here writes it to a
	 * sensor register anymore.
	 */
	bpp = introspect_gen_get_bpp(introspect_gen->fmt_code);
	dev_dbg(&client->dev,
		"generator stub: fmt_code implies %d bpp (informational only, not written anywhere)\n",
		bpp);

	/* Apply customized values from user (all now-registered controls
	 * are safe no-ops via introspect_gen_set_ctrl(), see there for details)
	 */
	ret = __v4l2_ctrl_handler_setup(introspect_gen->sd.ctrl_handler);
	if (ret)
		return ret;


	return 0;
}

/* Stop streaming */
static void introspect_gen_stop_streaming(struct introspect_gen *introspect_gen)
{
	/*
	 * No CCI writes: there is no sensor MODE_SELECT/EXTOUT_EN register
	 * to write. Nothing to do on the generator side either - streaming
	 * stop is controlled entirely from Pinetree, not from this driver.
	 */
}

static int introspect_gen_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&introspect_gen->mutex);
	if (introspect_gen->streaming == enable) {
		mutex_unlock(&introspect_gen->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto err_unlock;

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = introspect_gen_start_streaming(introspect_gen);
		if (ret) {
			pm_runtime_put_sync(&client->dev);
			goto err_unlock;
		}
	} else {
		introspect_gen_stop_streaming(introspect_gen);
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	introspect_gen->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(introspect_gen->vflip, enable);
	__v4l2_ctrl_grab(introspect_gen->hflip, enable);

	mutex_unlock(&introspect_gen->mutex);

	return ret;

err_unlock:
	mutex_unlock(&introspect_gen->mutex);

	return ret;
}

/* Power/clock management functions */
static int introspect_gen_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);
	int ret;

	ret = clk_prepare_enable(introspect_gen->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

	gpiod_set_value_cansleep(introspect_gen->reset_gpio, 1);
	usleep_range(INTROSPECT_GEN_XCLR_MIN_DELAY_US,
		     INTROSPECT_GEN_XCLR_MIN_DELAY_US + INTROSPECT_GEN_XCLR_DELAY_RANGE_US);

	return 0;
}

static int introspect_gen_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);

	gpiod_set_value_cansleep(introspect_gen->reset_gpio, 0);
	clk_disable_unprepare(introspect_gen->xclk);

	return 0;
}

static int __maybe_unused introspect_gen_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);

	if (introspect_gen->streaming)
		introspect_gen_stop_streaming(introspect_gen);

	return 0;
}

static int __maybe_unused introspect_gen_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);
	int ret;

	if (introspect_gen->streaming) {
		ret = introspect_gen_start_streaming(introspect_gen);
		if (ret)
			goto error;
	}

	return 0;

error:
	introspect_gen_stop_streaming(introspect_gen);
	introspect_gen->streaming = 0;
	return ret;
}

/* Verify chip ID */
static int introspect_gen_identify_module(struct introspect_gen *introspect_gen)
{
	struct i2c_client *client = v4l2_get_subdevdata(&introspect_gen->sd);

	/*
	 * No CCI/I2C interface exists on the Introspect SV3C-DPTX generator -
	 * there is nothing to read here. This is intentionally always
	 * successful.
	 */
	dev_info(&client->dev,
		 "Introspect MIPI generator stub: skipping CCI chip-ID check (no sensor present)\n");

	return 0;
}

static const struct v4l2_subdev_core_ops introspect_gen_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops introspect_gen_video_ops = {
	.s_stream = introspect_gen_set_stream,
};

static const struct v4l2_subdev_pad_ops introspect_gen_pad_ops = {
	.get_frame_desc = introspect_gen_get_frame_desc,
	.enum_mbus_code = introspect_gen_enum_mbus_code,
	.get_fmt = introspect_gen_get_pad_format,
	.set_fmt = introspect_gen_set_pad_format,
	.get_selection = introspect_gen_get_selection,
	.enum_frame_size = introspect_gen_enum_frame_size,
};

static const struct v4l2_subdev_ops introspect_gen_subdev_ops = {
	.core = &introspect_gen_core_ops,
	.video = &introspect_gen_video_ops,
	.pad = &introspect_gen_pad_ops,
};

static const struct v4l2_subdev_internal_ops introspect_gen_internal_ops = {
	.open = introspect_gen_open,
	.init_state = introspect_gen_init_state,
};

/* Initialize control handlers */
static int introspect_gen_init_controls(struct introspect_gen *introspect_gen)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&introspect_gen->sd);
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &introspect_gen->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&introspect_gen->mutex);
	ctrl_hdlr->lock = &introspect_gen->mutex;

	/* By default, PIXEL_RATE is read only */
	introspect_gen->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &introspect_gen_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       INTROSPECT_GEN_PIXEL_RATE,
					       INTROSPECT_GEN_PIXEL_RATE, 1,
					       INTROSPECT_GEN_PIXEL_RATE);
	if (introspect_gen->pixel_rate)
		introspect_gen->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Report the endpoint's link-frequencies value as V4L2_CID_LINK_FREQ.
	 * Without this control the CSI-2 host's v4l2_get_link_freq() falls back
	 * to estimating the rate from PIXEL_RATE, which here is a fixed
	 * compile-time constant that does not track the generator's configured
	 * data rate. The estimate then programs the D-PHY HS clock at the wrong
	 * frequency - typically far above the link's actual rate, where it is
	 * clamped to the PHY maximum and the link never locks.
	 *
	 * The value comes from introspect_gen_check_hwcfg(), which runs earlier
	 * in probe() and has already validated it against the generator's
	 * achievable rates. Read-only: the rate is a property of the
	 * generator's own configuration, described in DT, not something the
	 * host may negotiate.
	 */
	introspect_gen->link_freq_menu[0] = introspect_gen->link_freq_value;
	introspect_gen->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &introspect_gen_ctrl_ops,
				       V4L2_CID_LINK_FREQ, 0, 0,
				       introspect_gen->link_freq_menu);
	if (introspect_gen->link_freq)
		introspect_gen->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the introspect_gen_set_framing_limits() call below.
	 */
	introspect_gen->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &introspect_gen_ctrl_ops,
					   V4L2_CID_VBLANK, INTROSPECT_GEN_VBLANK_MIN,
					   0xffff, 1, INTROSPECT_GEN_VBLANK_MIN);
	introspect_gen->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &introspect_gen_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	introspect_gen->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &introspect_gen_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     INTROSPECT_GEN_EXPOSURE_MIN,
					     INTROSPECT_GEN_EXPOSURE_MAX,
					     INTROSPECT_GEN_EXPOSURE_STEP,
					     INTROSPECT_GEN_EXPOSURE_DEFAULT);

	/*
	 * ANALOGUE_GAIN is registered (unlike DIGITAL_GAIN/TEST_PATTERN,
	 * which stay fully removed) because libcamera's CameraSensor::
	 * validateSensorDriver() treats it as a mandatory control for RAW
	 * sensors - its absence hard-fails camera creation with "Mandatory
	 * V4L2 control 0x009e0903 not available", independent of whether the
	 * control does anything physically. set_ctrl's case for it is a
	 * no-op, same as EXPOSURE/VBLANK/HBLANK/HFLIP/VFLIP.
	 */
	v4l2_ctrl_new_std(ctrl_hdlr, &introspect_gen_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  INTROSPECT_GEN_ANA_GAIN_MIN, INTROSPECT_GEN_ANA_GAIN_MAX,
			  INTROSPECT_GEN_ANA_GAIN_STEP, INTROSPECT_GEN_ANA_GAIN_DEFAULT);

	/*
	 * DIGITAL_GAIN and TEST_PATTERN(+4 colour channel controls) are not
	 * registered at all in this driver - confirmed these are NOT in
	 * libcamera's mandatory-control list at any point in its history,
	 * unlike ANALOGUE_GAIN above, so their removal stands.
	 */

	introspect_gen->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &introspect_gen_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (introspect_gen->hflip)
		introspect_gen->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	introspect_gen->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &introspect_gen_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (introspect_gen->vflip)
		introspect_gen->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &introspect_gen_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	introspect_gen->sd.ctrl_handler = ctrl_hdlr;

	mutex_lock(&introspect_gen->mutex);

	/* Setup exposure and frame/line length limits. */
	introspect_gen_set_framing_limits(introspect_gen);

	mutex_unlock(&introspect_gen->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&introspect_gen->mutex);

	return ret;
}

static void introspect_gen_free_controls(struct introspect_gen *introspect_gen)
{
	v4l2_ctrl_handler_free(introspect_gen->sd.ctrl_handler);
	mutex_destroy(&introspect_gen->mutex);
}

static int introspect_gen_check_link_freq(u64 link_frequency, u16 *mpy_out)
{
	u64 mpy = link_frequency * 2 * INTROSPECT_GEN_IOP_SYSCK_DIV * INTROSPECT_GEN_IOP_PREDIV;
	u64 tmp;

	do_div(mpy, INTROSPECT_GEN_XCLK_FREQ);

	tmp = mpy * (INTROSPECT_GEN_XCLK_FREQ / INTROSPECT_GEN_IOP_PREDIV);
	do_div(tmp, INTROSPECT_GEN_IOP_SYSCK_DIV * 2);

	if (tmp != link_frequency)
		return -EINVAL;

	if (mpy_out)
		*mpy_out = mpy;

	return 0;
}

static int introspect_gen_check_hwcfg(struct device *dev, struct introspect_gen *introspect_gen)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;
	int i;

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
	if ((ep_cfg.bus.mipi_csi2.num_data_lanes) == 0 || (ep_cfg.bus.mipi_csi2.num_data_lanes) > 4) {
		dev_err(dev, "invalid lanes configured\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	for (i = 0; i < ep_cfg.nr_of_link_frequencies; i++) {
		if (!introspect_gen_check_link_freq(ep_cfg.link_frequencies[i],
					    NULL)) {
			introspect_gen->link_freq_value = ep_cfg.link_frequencies[i];
			break;
		}
	}

	if (i == ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "No supported link frequency configured\n");
			ret = -EINVAL;
			goto error_out;
	}

	/*
	 * Which VC this generator's packets are tagged with. Optional -
	 * absent means VC0, matching the CSI-2 host driver's own default so
	 * existing single-VC overlays that don't set this keep working
	 * unchanged.
	 */
	introspect_gen->vc = 0;
	fwnode_property_read_u32(endpoint, "sima,virtual-channel",
				 &introspect_gen->vc);
	if (introspect_gen->vc > 3) {
		dev_err(dev, "sima,virtual-channel must be 0-3, got %u\n",
			introspect_gen->vc);
		ret = -EINVAL;
		goto error_out;
	}

	/*
	 * Which single bit depth this instance exposes to enum_mbus_code().
	 * Optional - absent means 12, matching the driver's long-standing
	 * default so existing overlays that don't set this keep working
	 * unchanged. Only 8 or 12 accepted; deliberately no "both" option -
	 * see codes_raw8[]/codes_raw12[]'s own comment for why.
	 */
	{
		u32 pixel_bits = 12;

		fwnode_property_read_u32(endpoint, "sima,pixel-bits", &pixel_bits);
		switch (pixel_bits) {
		case 8:
			introspect_gen->codes = codes_raw8;
			break;
		case 12:
			introspect_gen->codes = codes_raw12;
			break;
		default:
			dev_err(dev, "sima,pixel-bits must be 8 or 12, got %u\n",
				pixel_bits);
			ret = -EINVAL;
			goto error_out;
		}
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct of_device_id introspect_gen_dt_ids[] = {
	{ .compatible = "sima,mipi-gen" },
	{ /* sentinel */ }
};


static int introspect_gen_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct introspect_gen *introspect_gen;
	int ret;

	introspect_gen = devm_kzalloc(&client->dev, sizeof(*introspect_gen), GFP_KERNEL);
	if (!introspect_gen)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&introspect_gen->sd, client, &introspect_gen_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (introspect_gen_check_hwcfg(dev, introspect_gen))
		return -EINVAL;

	/* Get system clock (xclk) */
	introspect_gen->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(introspect_gen->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(introspect_gen->xclk);
	}

	introspect_gen->xclk_freq = clk_get_rate(introspect_gen->xclk);
	if (introspect_gen->xclk_freq != INTROSPECT_GEN_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			introspect_gen->xclk_freq);
		return -EINVAL;
	}

	/* Request optional enable pin */
	introspect_gen->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for introspect_gen_identify_module()
	 * to run (no-op check, no real CHIP_ID register exists)
	 */
	ret = introspect_gen_power_on(dev);
	if (ret)
		return ret;

	ret = introspect_gen_identify_module(introspect_gen);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	introspect_gen_set_default_format(introspect_gen);

	/*
	 * Enable runtime PM with 5s autosuspend. As the device has been powered
	 * manually, mark it as active, and increase the usage count without
	 * resuming the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);
	pm_runtime_use_autosuspend(dev);

	/* This needs the pm runtime to be registered. */
	ret = introspect_gen_init_controls(introspect_gen);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	introspect_gen->sd.internal_ops = &introspect_gen_internal_ops;
	introspect_gen->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	introspect_gen->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	introspect_gen->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	introspect_gen->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&introspect_gen->sd.entity, NUM_PADS, introspect_gen->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&introspect_gen->sd);
	if (ret) {
		media_entity_cleanup(&introspect_gen->sd.entity);
		return ret;
	}

	ret = v4l2_async_register_subdev_sensor(&introspect_gen->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	dev_info(dev, "Probe is successful\n");
	return 0;

error_media_entity:
	media_entity_cleanup(&introspect_gen->sd.entity);

error_handler_free:
	introspect_gen_free_controls(introspect_gen);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	introspect_gen_power_off(&client->dev);

	return ret;
}

static void introspect_gen_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct introspect_gen *introspect_gen = to_introspect_gen(sd);

	v4l2_async_unregister_subdev(sd);
	sd->internal_ops = NULL;
	media_entity_cleanup(&sd->entity);
	introspect_gen_free_controls(introspect_gen);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		introspect_gen_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

MODULE_DEVICE_TABLE(of, introspect_gen_dt_ids);

static const struct dev_pm_ops introspect_gen_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(introspect_gen_suspend, introspect_gen_resume)
	SET_RUNTIME_PM_OPS(introspect_gen_power_off, introspect_gen_power_on, NULL)
};

static struct i2c_driver introspect_gen_i2c_driver = {
	.driver = {
		.name = "introspect-mipi-gen",
		.of_match_table	= introspect_gen_dt_ids,
		.pm = &introspect_gen_pm_ops,
	},
	.probe = introspect_gen_probe,
	.remove = introspect_gen_remove,
};

module_i2c_driver(introspect_gen_i2c_driver);

MODULE_AUTHOR("Vishnu Nair <vishnu.nair@sima.ai>");
MODULE_DESCRIPTION("V4L2 stub subdev for Introspect Technology SV3C-DPTX MIPI generator");
MODULE_LICENSE("GPL v2");
