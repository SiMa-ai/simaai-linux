// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022-2025 Intel Corporation.

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string_choices.h>
#include <linux/videodev2.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define ISX031_REG_STATE		CCI_REG8(0x6005)
#define ISX031_STATE_STARTUP		0x02
#define ISX031_STATE_STREAMING		0x05

#define ISX031_REG_DRIVE_MODE		CCI_REG8(0x8a00)
#define ISX031_DRIVE_4LANE_30FPS	0x17
#define ISX031_DRIVE_2LANE_30FPS	0x18

#define ISX031_REG_MODE_SET		CCI_REG8(0x8a01)
#define ISX031_MODE_STANDBY		0x00
#define ISX031_MODE_STREAMING		0x80

#define ISX031_REG_MODE_LOCK		CCI_REG8(0xbef0)
#define ISX031_MODE_UNLOCK		0x53

#define ISX031_REG_CROP_EN		CCI_REG8(0x8aa8)
#define ISX031_REG_CROP_WIDTH		CCI_REG16_LE(0x8aaa)
#define ISX031_REG_CROP_LEFT		CCI_REG16_LE(0x8aac)
#define ISX031_REG_CROP_HEIGHT		CCI_REG16_LE(0x8aae)
#define ISX031_REG_CROP_TOP		CCI_REG16_LE(0x8ab0)
#define ISX031_REG_DCROP_DATA_SEL	CCI_REG8(0x8ada)
/* Copy of the crop window written by the vendor sequence, undocumented */
#define ISX031_REG_CROP_SHD_EN		CCI_REG8(0xbf04)
#define ISX031_REG_CROP_SHD_WIDTH	CCI_REG16_LE(0xbf06)
#define ISX031_REG_CROP_SHD_LEFT	CCI_REG16_LE(0xbf08)
#define ISX031_REG_CROP_SHD_HEIGHT	CCI_REG16_LE(0xbf0a)
#define ISX031_REG_CROP_SHD_TOP		CCI_REG16_LE(0xbf0c)

#define ISX031_REG_OTP_TYPE_NAME_L	CCI_REG8(0x7e8a)
#define ISX031_REG_OTP_TYPE_NAME_H	CCI_REG8(0x7e8b)
#define ISX031_OTP_TYPE_NAME_H_MASK	GENMASK(3, 0)
#define ISX031_MODULE_ID		0x031

#define ISX031_NATIVE_WIDTH		1920
#define ISX031_NATIVE_HEIGHT		1536
#define ISX031_FPS			30

#define ISX031_BOOT_DELAY_MS		130	/* I2C unusable after reset release */
#define ISX031_STANDBY_SETTLE_MS	50	/* vendor: before reprogramming crop */
#define ISX031_STATE_POLL_US		10000
#define ISX031_STATE_TIMEOUT_US		1000000
#define ISX031_I2C_RETRY_US		20000
#define ISX031_I2C_TIMEOUT_US		2000000	/* vendor: 100 x 20 ms */

static const s64 isx031_link_freqs[] = { 288000000 };

static const struct cci_reg_sequence isx031_init_table[] = {
	{ CCI_REG8(0xffff), 0x00 },	/* select mode */
	{ CCI_REG8(0x0171), 0x00 },	/* F_EBD off */
	{ CCI_REG8(0x0172), 0x00 },	/* R_EBD off */
	{ CCI_REG8(0xbf14), 0x00 },	/* SG_MODE_APL */
	{ CCI_REG8(0x8af0), 0x00 },	/* internal sync */
};

static const struct cci_reg_sequence isx031_framesync_stream_table[] = {
	{ CCI_REG8(0x8af0), 0x02 },	/* external pulse-based sync */
	{ CCI_REG8(0x8af1), 0x00 },
	{ CCI_REG8(0x8afe), 0x00 },
	{ CCI_REG8(0x8aff), 0x0c },
	{ CCI_REG8(0xbf14), 0x02 },	/* SG_MODE_APL */
};

/* Output crop windows out of the native 1920x1536 */
static const struct cci_reg_sequence isx031_crop_1920x1080[] = {
	{ ISX031_REG_CROP_EN, 0x01 },
	{ ISX031_REG_CROP_WIDTH, 1920 },
	{ ISX031_REG_CROP_LEFT, 0 },
	{ ISX031_REG_CROP_HEIGHT, 1080 },
	{ ISX031_REG_CROP_TOP, 228 },		/* (1536 - 1080) / 2 */
	{ ISX031_REG_DCROP_DATA_SEL, 0x03 },
	{ ISX031_REG_CROP_SHD_EN, 0x01 },
	{ ISX031_REG_CROP_SHD_WIDTH, 1920 },
	{ ISX031_REG_CROP_SHD_LEFT, 0 },
	{ ISX031_REG_CROP_SHD_HEIGHT, 1080 },
	{ ISX031_REG_CROP_SHD_TOP, 228 },
};

static const struct cci_reg_sequence isx031_crop_1280x720[] = {
	{ ISX031_REG_CROP_EN, 0x01 },
	{ ISX031_REG_CROP_WIDTH, 1280 },
	{ ISX031_REG_CROP_LEFT, 320 },		/* (1920 - 1280) / 2 */
	{ ISX031_REG_CROP_HEIGHT, 720 },
	{ ISX031_REG_CROP_TOP, 408 },		/* (1536 - 720) / 2 */
	{ ISX031_REG_DCROP_DATA_SEL, 0x03 },
	{ ISX031_REG_CROP_SHD_EN, 0x01 },
	{ ISX031_REG_CROP_SHD_WIDTH, 1280 },
	{ ISX031_REG_CROP_SHD_LEFT, 320 },
	{ ISX031_REG_CROP_SHD_HEIGHT, 720 },
	{ ISX031_REG_CROP_SHD_TOP, 408 },
};

struct isx031_mode_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct isx031_mode {
	u32 width;
	u32 height;
	u32 crop_top;
	u32 crop_left;
	struct isx031_mode_reg_list reg_list;
};

#define ISX031_DEFAULT_MODE 0

/* [ISX031_DEFAULT_MODE] is used at init and as the set_fmt fallback */
static const struct isx031_mode isx031_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.crop_top = 228,
		.crop_left = 0,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(isx031_crop_1920x1080),
			.regs = isx031_crop_1920x1080,
		},
	},
	{
		.width = 1280,
		.height = 720,
		.crop_top = 408,
		.crop_left = 320,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(isx031_crop_1280x720),
			.regs = isx031_crop_1280x720,
		},
	},
};

struct isx031 {
	struct device *dev;
	struct gpio_desc *reset;
	struct regmap *regmap;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;

	unsigned int num_data_lanes;
	bool frame_sync_enable;
};

static inline struct isx031 *to_isx031(struct v4l2_subdev *sd)
{
	return container_of(sd, struct isx031, subdev);
}

static const struct isx031_mode *isx031_find_mode(u32 width, u32 height)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(isx031_modes); i++) {
		if (isx031_modes[i].width == width &&
		    isx031_modes[i].height == height)
			return &isx031_modes[i];
	}

	return NULL;
}

/*
 * cci_write() equivalent that rides out the NACKs the sensor ISP answers
 * while it boots or changes state, retrying each byte for up to 2 s like the
 * vendor driver. Multi-byte (little-endian) registers go out one byte at a
 * time, low byte first, as the module was validated with. regmap_write() is
 * used because cci_write() logs every failed attempt.
 */
static int isx031_write(struct isx031 *sensor, u32 reg, u64 val, int *err)
{
	unsigned int addr = CCI_REG_ADDR(reg);
	unsigned int i;
	int ret = 0;

	if (err && *err)
		return *err;

	for (i = 0; i < CCI_REG_WIDTH_BYTES(reg); i++) {
		if (read_poll_timeout(regmap_write, ret, !ret,
				      ISX031_I2C_RETRY_US,
				      ISX031_I2C_TIMEOUT_US, false,
				      sensor->regmap, addr + i,
				      (val >> (8 * i)) & 0xff)) {
			dev_err(sensor->dev, "error writing reg 0x%04x: %d\n",
				addr + i, ret);
			break;
		}
	}

	if (err)
		*err = ret;

	return ret;
}

/* cci_read() counterpart of isx031_write(), with the same retries */
static int isx031_read(struct isx031 *sensor, u32 reg, u64 *val, int *err)
{
	unsigned int addr = CCI_REG_ADDR(reg);
	unsigned int byte, i;
	int ret = 0;

	if (err && *err)
		return *err;

	*val = 0;
	for (i = 0; i < CCI_REG_WIDTH_BYTES(reg); i++) {
		if (read_poll_timeout(regmap_read, ret, !ret,
				      ISX031_I2C_RETRY_US,
				      ISX031_I2C_TIMEOUT_US, false,
				      sensor->regmap, addr + i, &byte)) {
			dev_err(sensor->dev, "error reading reg 0x%04x: %d\n",
				addr + i, ret);
			break;
		}
		*val |= (u64)byte << (8 * i);
	}

	if (err)
		*err = ret;

	return ret;
}

static int isx031_write_table(struct isx031 *sensor,
			      const struct cci_reg_sequence *regs,
			      unsigned int num_regs, int *err)
{
	for (unsigned int i = 0; i < num_regs; i++)
		isx031_write(sensor, regs[i].reg, regs[i].val, err);

	return *err;
}

/* Poll the sensor state register, through NACKs, until it reads @state. */
static int isx031_wait_state(struct isx031 *sensor, unsigned int state)
{
	unsigned int val = 0;
	int ret, err;

	ret = read_poll_timeout(regmap_read, err, !err && val == state,
				ISX031_STATE_POLL_US, ISX031_STATE_TIMEOUT_US,
				false, sensor->regmap,
				CCI_REG_ADDR(ISX031_REG_STATE), &val);
	if (ret)
		dev_err(sensor->dev,
			"state 0x%02x not reached (0x%02x, err %d)\n",
			state, val, err);

	return ret;
}

/*
 * MODE_SET is locked: every write needs the unlock key right before it, so a
 * NACKed attempt is retried as the pair.
 */
static int isx031_unlock_mode_set(struct isx031 *sensor, u8 mode_set)
{
	return regmap_write(sensor->regmap, CCI_REG_ADDR(ISX031_REG_MODE_LOCK),
			    ISX031_MODE_UNLOCK) ?:
	       regmap_write(sensor->regmap, CCI_REG_ADDR(ISX031_REG_MODE_SET),
			    mode_set);
}

static int isx031_mode_transit(struct isx031 *sensor, u8 mode_set,
			       unsigned int state)
{
	int err;

	if (read_poll_timeout(isx031_unlock_mode_set, err, !err,
			      ISX031_I2C_RETRY_US, ISX031_I2C_TIMEOUT_US,
			      false, sensor, mode_set)) {
		dev_err(sensor->dev, "error writing mode 0x%02x: %d\n",
			mode_set, err);
		return err;
	}

	return isx031_wait_state(sensor, state);
}

static int isx031_ctrls_init(struct isx031 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 3);

	/* Nothing on this platform reads it; DT link-frequencies is not validated. */
	ctrl = v4l2_ctrl_new_int_menu(&sensor->ctrls, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(isx031_link_freqs) - 1, 0,
				      isx031_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, NULL, &props);

	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return dev_err_probe(sensor->dev, ret,
				     "failed to add controls\n");
	}
	sensor->subdev.ctrl_handler = &sensor->ctrls;

	return 0;
}

static int isx031_stream_on(struct isx031 *sensor)
{
	return isx031_mode_transit(sensor, ISX031_MODE_STREAMING,
				   ISX031_STATE_STREAMING);
}

static int isx031_stream_off(struct isx031 *sensor)
{
	return isx031_mode_transit(sensor, ISX031_MODE_STANDBY,
				   ISX031_STATE_STARTUP);
}

static int isx031_setup(struct isx031 *sensor, struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format =
		v4l2_subdev_state_get_format(state, 0);
	const struct isx031_mode *mode =
		isx031_find_mode(format->width, format->height);
	int ret;

	/*
	 * Wait for the ISP to boot and start from standby every time, even
	 * after a failed stop, and reprogram everything: a runtime suspend
	 * resets the sensor.
	 */
	ret = isx031_stream_off(sensor);
	if (ret)
		return ret;

	msleep(ISX031_STANDBY_SETTLE_MS);

	isx031_write_table(sensor, isx031_init_table,
			   ARRAY_SIZE(isx031_init_table), &ret);
	isx031_write_table(sensor, mode->reg_list.regs,
			   mode->reg_list.num_of_regs, &ret);
	if (sensor->frame_sync_enable)
		isx031_write_table(sensor, isx031_framesync_stream_table,
				   ARRAY_SIZE(isx031_framesync_stream_table),
				   &ret);
	isx031_write(sensor, ISX031_REG_DRIVE_MODE,
		     sensor->num_data_lanes == 4 ? ISX031_DRIVE_4LANE_30FPS :
						   ISX031_DRIVE_2LANE_30FPS, &ret);

	return ret;
}

static int isx031_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx031 *sensor = to_isx031(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = isx031_stream_off(sensor);

		pm_runtime_put_autosuspend(sensor->dev);

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = isx031_setup(sensor, state);
	if (ret)
		goto err_pm;

	ret = isx031_stream_on(sensor);
	if (ret)
		goto err_pm;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;

err_pm:
	/*
	 * In case of error, turn the power off synchronously as the device
	 * likely has no other chance to recover.
	 */
	pm_runtime_put_sync_suspend(sensor->dev);

	goto unlock;
}

static int isx031_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}

static int isx031_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(isx031_modes) ||
	    fse->code != MEDIA_BUS_FMT_UYVY8_1X16)
		return -EINVAL;

	fse->min_width = isx031_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = isx031_modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static int isx031_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index != 0 || fie->code != MEDIA_BUS_FMT_UYVY8_1X16 ||
	    !isx031_find_mode(fie->width, fie->height))
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = ISX031_FPS;

	return 0;
}

static int isx031_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	if (pad != 0)
		return -EINVAL;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry->pixelcode = MEDIA_BUS_FMT_UYVY8_1X16;
	fd->entry->stream = 0;
	fd->entry->bus.csi2.vc = 0;
	fd->entry->bus.csi2.dt = MIPI_CSI2_DT_YUV422_8B;

	return 0;
}

static int isx031_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	const struct isx031_mode *mode;
	struct v4l2_mbus_framefmt *format;

	/* An exact size match or the default mode, never the nearest size */
	mode = isx031_find_mode(fmt->format.width, fmt->format.height) ?:
	       &isx031_modes[ISX031_DEFAULT_MODE];

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	format->width = mode->width;
	format->height = mode->height;
	format->code = MEDIA_BUS_FMT_UYVY8_1X16;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_DEFAULT;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	fmt->format = *format;

	return 0;
}

static int isx031_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	const struct v4l2_mbus_framefmt *format;
	const struct isx031_mode *mode;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		format = v4l2_subdev_state_get_format(sd_state, 0);
		mode = isx031_find_mode(format->width, format->height);
		sel->r.top = mode->crop_top;
		sel->r.left = mode->crop_left;
		sel->r.width = mode->width;
		sel->r.height = mode->height;

		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = ISX031_NATIVE_WIDTH;
		sel->r.height = ISX031_NATIVE_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int isx031_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = isx031_modes[ISX031_DEFAULT_MODE].width,
			.height = isx031_modes[ISX031_DEFAULT_MODE].height,
		},
	};

	*v4l2_subdev_state_get_interval(state, 0) =
		(struct v4l2_fract){ .numerator = 1, .denominator = ISX031_FPS };

	return isx031_set_format(sd, state, &format);
}

static const struct v4l2_subdev_video_ops isx031_subdev_video_ops = {
	.s_stream = isx031_s_stream,
};

static const struct v4l2_subdev_pad_ops isx031_subdev_pad_ops = {
	.get_frame_desc = isx031_get_frame_desc,
	.enum_mbus_code = isx031_enum_mbus_code,
	.enum_frame_size = isx031_enum_frame_size,
	.enum_frame_interval = isx031_enum_frame_interval,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = isx031_set_format,
	.get_frame_interval = v4l2_subdev_get_frame_interval,
	/* The frame rate is fixed: setting it reports it */
	.set_frame_interval = v4l2_subdev_get_frame_interval,
	.get_selection = isx031_get_selection,
};

static const struct v4l2_subdev_ops isx031_subdev_ops = {
	.video = &isx031_subdev_video_ops,
	.pad = &isx031_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops isx031_internal_ops = {
	.init_state = isx031_init_state,
};

static int isx031_subdev_init(struct isx031 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &isx031_subdev_ops);
	sensor->subdev.internal_ops = &isx031_internal_ops;

	ret = isx031_ctrls_init(sensor);
	if (ret)
		return ret;

	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (ret < 0)
		goto err_ctrls;

	sensor->subdev.state_lock = sensor->subdev.ctrl_handler->lock;
	ret = v4l2_subdev_init_finalize(&sensor->subdev);
	if (ret)
		goto err_entity;

	return 0;

err_entity:
	media_entity_cleanup(&sensor->subdev.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
	return ret;
}

static void isx031_subdev_cleanup(struct isx031 *sensor)
{
	v4l2_subdev_cleanup(&sensor->subdev);
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

/*
 * The ISP may still NACK after the boot delay and boots into either standby or
 * streaming. The callers wait for it with isx031_stream_off(), so that runtime
 * resume cannot fail: the PM core latches resume errors.
 */
static void isx031_power_on(struct isx031 *sensor)
{
	if (sensor->reset) {
		/* Hold reset for POR, then give the ISP time to enable I2C. */
		usleep_range(2000, 3000);
		gpiod_set_value_cansleep(sensor->reset, 0);
		msleep(ISX031_BOOT_DELAY_MS);
	}
}

static void isx031_power_off(struct isx031 *sensor)
{
	gpiod_set_value_cansleep(sensor->reset, 1);
}

static int isx031_identify_model(struct isx031 *sensor)
{
	u64 name_l, name_h;
	u32 id;
	int ret = 0;

	isx031_read(sensor, ISX031_REG_OTP_TYPE_NAME_L, &name_l, &ret);
	isx031_read(sensor, ISX031_REG_OTP_TYPE_NAME_H, &name_h, &ret);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to read module ID\n");

	id = ((name_h & ISX031_OTP_TYPE_NAME_H_MASK) << 8) | name_l;
	if (id != ISX031_MODULE_ID)
		return dev_err_probe(sensor->dev, -ENODEV,
				     "invalid module ID 0x%03x\n", id);

	return 0;
}

static int isx031_parse_hw_config(struct isx031 *sensor)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	struct gpio_desc *fsin;
	u32 frame_sync = 0;
	int ret;

	device_property_read_u32(sensor->dev, "gmsl-frame-sync-enable",
				 &frame_sync);
	sensor->frame_sync_enable = frame_sync;
	dev_dbg(sensor->dev, "External GMSL frame_sync %s\n",
		str_enabled_disabled(sensor->frame_sync_enable));

	sensor->reset = devm_gpiod_get_optional(sensor->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");
	dev_dbg(sensor->dev, "Reset gpio %s\n",
		sensor->reset ? "found" : "not found");

	/*
	 * With GMSL frame sync, FSIN arrives over the link: requesting it as a
	 * local GPIO sets GPIO_OUT_DIS and disables the serializer video output.
	 */
	if (!sensor->frame_sync_enable) {
		fsin = devm_gpiod_get_optional(sensor->dev, "fsin",
					       GPIOD_OUT_LOW);
		if (IS_ERR(fsin))
			return dev_err_probe(sensor->dev, PTR_ERR(fsin),
					     "failed to get fsin GPIO\n");
		dev_dbg(sensor->dev, "Fsin gpio %s\n",
			fsin ? "found" : "not found");
	}

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!ep)
		return dev_err_probe(sensor->dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to parse endpoint\n");

	switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		sensor->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		return dev_err_probe(sensor->dev, -EINVAL,
				     "invalid number of CSI2 data lanes %d\n",
				     bus_cfg.bus.mipi_csi2.num_data_lanes);
	}

	return 0;
}

static int isx031_probe(struct i2c_client *client)
{
	struct isx031 *sensor;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	ret = isx031_parse_hw_config(sensor);
	if (ret)
		return ret;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, wait for it in standby and identify it.
	 */
	isx031_power_on(sensor);

	ret = isx031_stream_off(sensor);
	if (ret)
		goto err_power;

	ret = isx031_identify_model(sensor);
	if (ret)
		goto err_power;

	ret = isx031_subdev_init(sensor);
	if (ret)
		goto err_power;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->subdev);
	if (ret < 0)
		goto err_pm;

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	dev_info(sensor->dev, "isx031 sensor successfully probed\n");

	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	isx031_subdev_cleanup(sensor);
err_power:
	isx031_power_off(sensor);
	return ret;
}

static void isx031_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx031 *sensor = to_isx031(subdev);

	v4l2_async_unregister_subdev(subdev);

	isx031_subdev_cleanup(sensor);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		isx031_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static int isx031_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx031 *sensor = to_isx031(subdev);

	isx031_power_on(sensor);

	return 0;
}

static int isx031_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx031 *sensor = to_isx031(subdev);

	isx031_power_off(sensor);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(isx031_pm_ops, isx031_runtime_suspend,
				 isx031_runtime_resume, NULL);

static const struct of_device_id isx031_of_match[] = {
	{ .compatible = "sony,isx031" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, isx031_of_match);

static struct i2c_driver isx031_driver = {
	.probe = isx031_probe,
	.remove = isx031_remove,
	.driver = {
		.name = "isx031",
		.of_match_table = isx031_of_match,
		.pm = pm_ptr(&isx031_pm_ops),
	},
};

module_i2c_driver(isx031_driver);

MODULE_DESCRIPTION("isx031 sensor driver");
MODULE_AUTHOR("Hao Yao <hao.yao@intel.com>");
MODULE_AUTHOR("Jonathan Lui <jonathan.ming.jun.lui@intel.com>");
MODULE_AUTHOR("Wei Khang, Goh <wei.khang1.goh@intel.com>");
MODULE_LICENSE("GPL");
