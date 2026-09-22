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
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/mipi-csi2.h>

// Unsigned integer types (no negative values)
typedef uint8_t  u8;   // 8-bit unsigned  (0 to 255)
typedef uint16_t u16;  // 16-bit unsigned (0 to 65,535)
typedef uint32_t u32;  // 32-bit unsigned (0 to 4,294,967,295)
typedef uint64_t u64;  // 64-bit unsigned (0 to 18,446,744,073,709,551,615)

// Signed integer types (can represent negative values)
typedef int8_t  s8;    // 8-bit signed  (-128 to 127)
typedef int16_t s16;   // 16-bit signed (-32,768 to 32,767)
typedef int32_t s32;   // 32-bit signed (-2,147,483,648 to 2,147,483,647)
typedef int64_t s64;   // 64-bit signed (-9,223,372,036,854,775,808 to 9,223,372,036,854,775,807)

/*
 * From the datasheet, "20ms after PWDN goes low or 20ms after RESETB goes
 * high if reset is inserted after PWDN goes high, host can access sensor's
 * SCCB to initialize sensor."
 */
#define PWDN_ACTIVE_DELAY_MS	20

#define IMX568_REG_WAIT			0x0000

#define IMX568_SW_STANDBY		0x3000

#define IMX568_REG_EXP_HI		0x3242
#define IMX568_REG_EXP_MID		0x3241
#define IMX568_REG_EXP_LO		0x3240

#define IMX568_REG_GAIN_HI		0x3515
#define IMX568_REG_GAIN_LO		0x3514
#define IMX568_REG_HTS_HI		0x30d9
#define IMX568_REG_HTS_LO		0x30d8
#define IMX568_REG_VTS_HI		0x30d5
#define IMX568_REG_VTS_LO		0x30d4

#define IMX568_REG_CHIPID_H		IMX568_REG_GAIN_HI
#define IMX568_REG_CHIPID_L		IMX568_REG_GAIN_LO

/* IMX568 native and active pixel array size */
#define IMX568_NATIVE_WIDTH		2464U
#define IMX568_NATIVE_HEIGHT		2064U

#define IMX568_PIXEL_ARRAY_LEFT		8U
#define IMX568_PIXEL_ARRAY_TOP		8U
#define IMX568_PIXEL_ARRAY_WIDTH	1920U
#define IMX568_PIXEL_ARRAY_HEIGHT	1080U

#define IMX568_VBLANK_MIN		24
#define IMX568_VTS_MAX			32767
#define IMX568_VTS			(0xA02)

#define IMX568_HTS			0x3C5
#define IMX568_HTS_MAX			0x7fff

#define IMX568_GAIN_MIN			0
#define IMX568_GAIN_STEP		1
#define IMX568_GAIN_DEFAULT		0
#define IMX568_GAIN_MAX			(480)

#define IMX568_EXPOSURE_MIN		(0x10+4)
#define IMX568_EXPOSURE_STEP		1
#define IMX568_EXPOSURE_DEFAULT		(0x70)
#define IMX568_EXPOSURE_MAX		(IMX568_VTS-1)

/* regulator supplies */
static const char * const imx568_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX568_NUM_SUPPLIES ARRAY_SIZE(imx568_supply_names)

#define FREQ_INDEX_FULL		0
#define FREQ_INDEX_VGA		1
static const s64 imx568_link_freqs[] = {//link data rate div with 2
	[FREQ_INDEX_FULL]	= 1188000000,
};

struct regval_list {
	u16 addr;
	u8 data;
};

struct imx568_mode {
	struct v4l2_mbus_framefmt	format;
	struct v4l2_rect		crop;
	u64				pixel_rate;
	unsigned int			link_freq_index;
	int				hts;
	int				vts;
	const struct regval_list	*reg_list;
	unsigned int			num_regs;
};

struct imx568 {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct mutex			lock;
	struct clk			*xclk;
	struct gpio_desc		*pwdn;
	struct regulator_bulk_data supplies[IMX568_NUM_SUPPLIES];
	bool				clock_ncont;
	struct v4l2_ctrl_handler	ctrls;
	const struct imx568_mode	*mode;
	struct v4l2_ctrl		*pixel_rate;
	struct v4l2_ctrl		*hblank;
	struct v4l2_ctrl		*vblank;
	struct v4l2_ctrl		*exposure;
	struct v4l2_ctrl		*hflip;
	struct v4l2_ctrl		*vflip;
	struct v4l2_ctrl		*link_freq;
};

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

static const u8 imx568_test_pattern_val[] = {
	0x00,	/* Disabled */
};

static struct regval_list imx568_start_stream[] = {
	{0x3000, 0x00 },
	{IMX568_REG_WAIT, 250},
	{0x3010, 0x00},
	{IMX568_REG_WAIT, 50},
};

static struct regval_list imx568_stop_stream[] = {
	{0x3000, 0x01 },
	{IMX568_REG_WAIT, 200},
	{0x3010, 0x01},
	{IMX568_REG_WAIT, 50},
};

static struct regval_list imx568_1920x1080_12bpp[] = {
/*
IMX568-AAQJ
Interface: CSI-2_2Lane 1188Mbps/Lane / AD: 12bit Output: 12bit / INCK: 37.125MHz ROI mode / Master Mode / Frame rate: 30.03fps / Integration Time: 33.308ms / HMAX: 965 / VMAX: 2562
Tool ver: Ver4.3
*/
	{0x3014, 0x05},  // INCKSEL_ST0
	{0x3015, 0x91},  // INCKSEL_ST1
	{0x3016, 0x50},  // INCKSEL_ST2
	{0x3018, 0x20},  // INCKSEL_ST3
	{0x3019, 0x02},  // INCKSEL_ST4
	{0x301B, 0x1D},  // INCKSEL_ST5
	{0x30D0, 0x80},  // VOPB_VBLK_HWIDTH LSB
	{0x30D1, 0x07},  // VOPB_VBLK_HWIDTH MSB
	{0x30D2, 0x80},  // FINFO_HWIDTH LSB
	{0x30D3, 0x07},  // FINFO_HWIDTH MSB
	{0x30D4, 0x02},  // VMAX
	{0x30D5, 0x0A},  // VMAX
	{0x30D6, 0x00},  // VMAX
	{0x30D8, 0xC5},  // HMAX LSB
	{0x30D9, 0x03},  // HMAX MSB
	{0x30E2, 0x02},  // GMRWT
	{0x30E3, 0x10},  // GMTWT
	{0x30E6, 0x08},  // GSDLY
	{0x3100, 0x00},  // ROI_MODE
	{0x3104, 0x03},  // FID0_ROIH1ON  FID0_ROIV1ON  FID0_ROIH2ON  FID0_ROIV2ON  FID0_ROIH3ON  FID0_ROIV3ON  FID0_ROIH4ON  FID0_ROIV4ON
	{0x3120, 0x08},  // FID0_ROIPH1 LSB
	{0x3121, 0x00},  // FID0_ROIPH1 MSB
	{0x3122, 0x08},  // FID0_ROIPV1 LSB
	{0x3123, 0x00},  // FID0_ROIPV1 MSB
	{0x3124, 0x80},  // FID0_ROIWH1 LSB
	{0x3125, 0x07},  // FID0_ROIWH1 MSB
	{0x3126, 0x38},  // FID0_ROIWV1 LSB
	{0x3127, 0x04},  // FID0_ROIWV1 MSB
	{0x3200, 0x15},  // ADBIT
	{0x321C, 0x40},  // INCKSEL_N0
	{0x3224, 0x40},  // INCKSEL_D0
	{0x3226, 0x80},  // INCKSEL_D2
	{0x3227, 0x80},  // INCKSEL_D3
	{0x323E, 0x30},  // VINT_EN  VINT_EN_NOR
	{0x3240, 0x14},  // SHS
	{0x3430, 0x01},  // ODBIT
	{0x3480, 0x20},  // PULSE2_EN_NOR  PULSE2_EN_TRIG  PULSE2_POL
	{0x3502, 0x08},  // GAIN_RTS
	{0x3542, 0x27},  //
	{0x354A, 0x20},  //
	{0x359C, 0x0F},  //
	{0x35A4, 0x30},  //
	{0x35A5, 0x12},  //
	{0x35A8, 0x30},  //
	{0x35A9, 0x42},  //
	{0x35AC, 0x62},  //
	{0x35B4, 0xF0},  // BLKLEVEL
	{0x35B6, 0x02},  //
	{0x35EC, 0x30},  //
	{0x35ED, 0x12},  //
	{0x35F0, 0xFB},  //
	{0x35F1, 0x0B},  //
	{0x35F2, 0xFB},  //
	{0x35F3, 0x0B},  //
	{0x3797, 0x20},  //
	{0x3904, 0x03},  // LANESEL
	{0x3942, 0x03},  // EAV_SEL_MIPI
	{0x3C30, 0x00},  // Continous CLK
	{0x3CA4, 0x80},  // TXCLKESC_FREQ
	{0x3CA5, 0x09},  //
	{0x3E30, 0x4E},  //
	{0x3E96, 0x01},  //
	{0x3EA0, 0x4C},  //
	{0x3F3A, 0x04},  //
	{0x4056, 0x23},  //
	{0x4096, 0x23},  //
	{0x4182, 0x00},  //
	{0x41A2, 0x03},  //
	{0x4232, 0x3C},  //
	{0x4306, 0x00},  //
	{0x4307, 0x00},  //
	{0x4308, 0x00},  //
	{0x4309, 0x00},  //
	{0x4310, 0x04},  //
	{0x4311, 0x04},  //
	{0x4312, 0x04},  //
	{0x4313, 0x04},  //
	{0x4467, 0x83},  //
	{0x4749, 0x9F},  //
	{0x474A, 0x99},  //
	{0x474B, 0x09},  //
	{0x4788, 0x04},  //
	{0x479C, 0x40},  //
	{0x4864, 0xDC},  //
	{0x4868, 0xDC},  //
	{0x486C, 0xDC},  //
	{0x48A4, 0xF4},  //
	{0x48A8, 0xF4},  //
	{0x48AC, 0xF4},  //
	{IMX568_REG_WAIT, 1},

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
		.pixel_rate	= 198000000,
		.link_freq_index = FREQ_INDEX_FULL,
		.hts		= IMX568_HTS,
		.vts		= IMX568_VTS,
		.reg_list	= imx568_1920x1080_12bpp,
		.num_regs	= ARRAY_SIZE(imx568_1920x1080_12bpp)
	},
};

/* Default sensor mode is 2x2 binned 640x480 SBGGR10_1X10. */
#define IMX568_DEFAULT_MODE	(&imx568_modes[0])
#define IMX568_DEFAULT_FORMAT	(imx568_modes[0].format)

static int imx568_write(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = i2c_master_send(client, data, 3);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 3) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return 0;
}

static int imx568_read(struct v4l2_subdev *sd, u16 reg, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x = %d\n",
			__func__, reg, ret);
		return ret >= 0 ? -EINVAL : ret;
	}

	*val = buf[0];

	return 0;
}

static int imx568_write_array(struct v4l2_subdev *sd,
			      const struct regval_list *regs, int array_size)
{
	int i, ret;

	for (i = 0; i < array_size; i++) {
		if(IMX568_REG_WAIT == regs[i].addr){
			msleep(regs[i].data);
			continue;
		}
		ret = imx568_write(sd, regs[i].addr, regs[i].data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int imx568_set_mode(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx568 *sensor = to_sensor(sd);
	int ret;

	ret = imx568_write_array(sd, sensor->mode->reg_list,
				 sensor->mode->num_regs);
	if (ret < 0) {
		dev_err(&client->dev, "write sensor default regs error\n");
		return ret;
	}

	return 0;
}

static int imx568_stream_on(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//struct imx568 *sensor = to_sensor(sd);
	int ret;

	ret = imx568_set_mode(sd);
	if (ret) {
		dev_err(&client->dev, "Failed to program sensor mode: %d\n", ret);
		return ret;
	}

	/* Apply customized values from user when stream starts. */
	ret =  __v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (ret)
		return ret;

	return imx568_write_array(sd, imx568_start_stream,ARRAY_SIZE(imx568_start_stream));
}

static int imx568_stream_off(struct v4l2_subdev *sd)
{
	return imx568_write_array(sd, imx568_stop_stream,ARRAY_SIZE(imx568_stop_stream));
}

static int imx568_power_on(struct device *dev)
{
	struct imx568 *sensor = dev_get_drvdata(dev);
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
	struct imx568 *sensor = dev_get_drvdata(dev);
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
	int ret;
	u8 val;

	ret = imx568_read(sd, reg->reg & 0xff, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int imx568_sensor_set_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	return imx568_write(sd, reg->reg & 0xff, reg->val & 0xff);
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
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_rect *
__imx568_get_pad_crop(struct imx568 *imx568,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx568->mode->crop;
	}

	return NULL;
}

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
	} else {
		ret = imx568_stream_off(sd);
		if (ret < 0) {
			dev_err(&client->dev, "stream stop failed: %d\n", ret);
			goto error_pm;
		}
		pm_runtime_put(&client->dev);
	}

	mutex_unlock(&sensor->lock);

	return 0;

error_pm:
	pm_runtime_put(&client->dev);
error_unlock:
	mutex_unlock(&sensor->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops imx568_subdev_video_ops = {
	.s_stream =		imx568_s_stream,
};

/* This function returns the mbus code for the current settings of the
   HFLIP and VFLIP controls. */

static u32 imx568_get_mbus_code(struct v4l2_subdev *sd)
{
	struct imx568 *sensor = to_sensor(sd);

	return sensor->mode->format.code;
}

static int imx568_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = imx568_get_mbus_code(sd);

	return 0;
}

static int imx568_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct v4l2_mbus_framefmt *fmt;

	if (fse->code != imx568_get_mbus_code(sd) ||
	    fse->index >= ARRAY_SIZE(imx568_modes))
		return -EINVAL;

	fmt = &imx568_modes[fse->index].format;
	fse->min_width = fmt->width;
	fse->max_width = fmt->width;
	fse->min_height = fmt->height;
	fse->max_height = fmt->height;

	return 0;
}

static int imx568_get_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	const struct v4l2_mbus_framefmt *sensor_format;
	struct imx568 *sensor = to_sensor(sd);

	mutex_lock(&sensor->lock);
	switch (format->which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		sensor_format = v4l2_subdev_state_get_format(sd_state,
							     format->pad);
		break;
	default:
		sensor_format = &sensor->mode->format;
		break;
	}

	*fmt = *sensor_format;
	/* The code we pass back must reflect the current h/vflips. */
	fmt->code = imx568_get_mbus_code(sd);
	mutex_unlock(&sensor->lock);

	return 0;
}

static int imx568_set_pad_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct imx568 *sensor = to_sensor(sd);
	const struct imx568_mode *mode;

	mode = v4l2_find_nearest_size(imx568_modes, ARRAY_SIZE(imx568_modes),
				      format.width, format.height,
				      fmt->width, fmt->height);

	/* Update the sensor mode and apply at it at streamon time. */
	mutex_lock(&sensor->lock);
	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, format->pad) = mode->format;
	} else {
		int exposure_max, exposure_def;
		int hblank, vblank;

		sensor->mode = mode;
		__v4l2_ctrl_modify_range(sensor->pixel_rate, mode->pixel_rate,
					 mode->pixel_rate, 1, mode->pixel_rate);

		hblank = mode->hts - mode->format.width;
		__v4l2_ctrl_modify_range(sensor->hblank, hblank,
					 IMX568_HTS_MAX - mode->format.width, 1,
					 hblank);

		vblank = mode->vts - mode->format.height;
		__v4l2_ctrl_modify_range(sensor->vblank, IMX568_VBLANK_MIN,
					 IMX568_VTS_MAX - mode->format.height,
					 1, vblank);
		__v4l2_ctrl_s_ctrl(sensor->vblank, vblank);

		exposure_max = mode->vts - 4;
		exposure_def = min(exposure_max, IMX568_EXPOSURE_DEFAULT);
		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 exposure_max, sensor->exposure->step,
					 exposure_def);

		__v4l2_ctrl_s_ctrl(sensor->link_freq, mode->link_freq_index);
	}
	*fmt = mode->format;
	/* The code we pass back must reflect the current h/vflips. */
	fmt->code = imx568_get_mbus_code(sd);
	mutex_unlock(&sensor->lock);

	return 0;
}

static int imx568_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx568 *sensor = to_sensor(sd);

		mutex_lock(&sensor->lock);
		sel->r = *__imx568_get_pad_crop(sensor, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&sensor->lock);

		return 0;
	}

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
	u32 code;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	//Get the format code of stream 0 of image_pad
	code = v4l2_subdev_state_get_format(state, 0, 0)->code;

	v4l2_subdev_unlock_state(state);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1; //One one stream supported per image_pad

	fd->entry->pixelcode = code;
	fd->entry->stream = 0; //Set stream id
	fd->entry->bus.csi2.vc = 0; //Read from device tree
	fd->entry->bus.csi2.dt = MIPI_CSI2_DT_RAW12;

	return 0;
}

static const struct v4l2_subdev_pad_ops imx568_subdev_pad_ops = {
	.get_frame_desc 	= imx568_get_frame_desc,
	.enum_mbus_code		= imx568_enum_mbus_code,
	.enum_frame_size	= imx568_enum_frame_size,
	.set_fmt		= imx568_set_pad_fmt,
	.get_fmt		= imx568_get_pad_fmt,
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
	u8 read;
	int ret;

	ret = imx568_read(sd, IMX568_REG_CHIPID_H, &read);
	if (ret < 0)
		return ret;
	dev_dbg(&client->dev,
			 "read register 0x%04x : 0x%02x\n",
			 IMX568_REG_CHIPID_H, read);

	ret = imx568_read(sd, IMX568_REG_CHIPID_L, &read);
	if (ret < 0)
		return ret;
	dev_dbg(&client->dev,
			 "read register 0x%04x : 0x%02x\n",
			 IMX568_REG_CHIPID_L, read);

	return 0;
}

static int imx568_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_state_get_format(fh->state, 0);
	struct v4l2_rect *crop = v4l2_subdev_state_get_crop(fh->state, 0);

	crop->left = IMX568_PIXEL_ARRAY_LEFT;
	crop->top = IMX568_PIXEL_ARRAY_TOP;
	crop->width = IMX568_PIXEL_ARRAY_WIDTH;
	crop->height = IMX568_PIXEL_ARRAY_HEIGHT;

	*format = IMX568_DEFAULT_FORMAT;

	return 0;
}

static int imx568_set_format(struct v4l2_subdev *sd,
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

static int imx568_init_state(struct v4l2_subdev *sd,
                             struct v4l2_subdev_state *state)
{
        struct v4l2_subdev_format format = {
                .format = {
                        .width = IMX568_PIXEL_ARRAY_WIDTH,
                        .height = IMX568_PIXEL_ARRAY_HEIGHT,
                },
        };

        imx568_set_format(sd, state, &format);

        return 0;
}

static const struct v4l2_subdev_internal_ops imx568_subdev_internal_ops = {
	.open = imx568_open,
	.init_state = imx568_init_state,
};

static int imx568_s_analogue_gain(struct v4l2_subdev *sd, u32 val)
{
	int ret;

	/* 10 bits of gain, 2 in the high register. */
	ret = imx568_write(sd, IMX568_REG_GAIN_HI, (val >> 8) & 3);
	if (ret)
		return ret;

	return imx568_write(sd, IMX568_REG_GAIN_LO, val & 0xff);
}

static int imx568_s_exposure(struct v4l2_subdev *sd, u32 val)
{
	int ret;

	if(val <= IMX568_EXPOSURE_MIN){
		val = IMX568_EXPOSURE_MIN;
	}else if(val >= IMX568_EXPOSURE_MAX){
		val = IMX568_EXPOSURE_MAX;
	}

	val = IMX568_EXPOSURE_MAX + IMX568_EXPOSURE_MIN - val;

	/*
	 * Sensor has 20 bits, but the bottom 4 bits are fractions of a line
	 * which we leave as zero (and don't receive in "val").
	 */
	ret = imx568_write(sd, IMX568_REG_EXP_HI, (val >> 16) & 0xf);
	if (ret)
		return ret;

	ret = imx568_write(sd, IMX568_REG_EXP_MID, (val >> 8) & 0xff);
	if (ret)
		return ret;

	return imx568_write(sd, IMX568_REG_EXP_LO, val & 0xff);
}

static int imx568_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx568 *sensor = container_of(ctrl->handler,
					    struct imx568, ctrls);
	struct v4l2_subdev *sd = &sensor->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;


	/* v4l2_ctrl_lock() locks our own mutex */

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = sensor->mode->format.height + ctrl->val - 4;
		exposure_def = min(exposure_max, IMX568_EXPOSURE_DEFAULT);
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
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

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
	case V4L2_CID_VBLANK:

		break;
	case V4L2_CID_HBLANK:

		break;
	case V4L2_CID_TEST_PATTERN:
		break;

	/* Read-only, but we adjust it based on mode. */
	case V4L2_CID_PIXEL_RATE:
		/* Read-only, but we adjust it based on mode. */
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
		return -EINVAL;
	}

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
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int hblank, exposure_max, exposure_def;
	struct v4l2_fwnode_device_properties props;

	v4l2_ctrl_handler_init(&sensor->ctrls, 10);

	v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu(&sensor->ctrls, &imx568_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       0, V4L2_EXPOSURE_MANUAL);

	exposure_max = IMX568_EXPOSURE_MAX;
	exposure_def = min(exposure_max, IMX568_EXPOSURE_DEFAULT);
	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX568_EXPOSURE_MIN,
					     exposure_max, IMX568_EXPOSURE_STEP,
					     exposure_def);

	/* min: 16 = 1.0x; max (10 bits); default: 32 = 2.0x. */
	v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, IMX568_GAIN_MIN, IMX568_GAIN_MAX, IMX568_GAIN_STEP, IMX568_GAIN_DEFAULT);

	/* By default, PIXEL_RATE is read only, but it does change per mode */
	sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       sensor->mode->pixel_rate,
					       sensor->mode->pixel_rate, 1,
					       sensor->mode->pixel_rate);

	hblank = sensor->mode->hts - sensor->mode->format.width;
	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   IMX568_HTS_MAX -
					   sensor->mode->format.width, 1,
					   hblank);

	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx568_ctrl_ops,
					   V4L2_CID_VBLANK, IMX568_VBLANK_MIN,
					   IMX568_VTS_MAX -
					   sensor->mode->format.height, 1,
					   sensor->mode->vts -
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

	sensor->link_freq =
		v4l2_ctrl_new_int_menu(&sensor->ctrls, &imx568_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(imx568_link_freqs) - 1, 0,
				       imx568_link_freqs);
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
	dev_err(&client->dev, "%s Controls initialization failed (%d)\n",
		__func__, sensor->ctrls.error);
	v4l2_ctrl_handler_free(&sensor->ctrls);

	return sensor->ctrls.error;
}

static int imx568_parse_dt(struct imx568 *sensor, struct device_node *np)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device_node *ep __free(device_node) =
		of_graph_get_endpoint_by_regs(np, 0, -1);
	int ret;

	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &bus_cfg);
	if (ret)
		return ret;

	sensor->clock_ncont = bus_cfg.bus.mipi_csi2.flags &
			      V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;

	return 0;
}

static int imx568_probe(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct device *dev = &client->dev;
	struct imx568 *sensor;
	struct v4l2_subdev *sd;
	u32 xclk_freq;
	int ret;

	dev_dbg(dev, "enter SONY IMX568 camera driver probe\n");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF) && np) {
		ret = imx568_parse_dt(sensor, np);
		if (ret) {
			dev_err(dev, "DT parsing error: %d\n", ret);
			return ret;
		}
	}

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(sensor->xclk);
	}

	xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq != 25000000) {
		dev_err(dev, "Unsupported clock frequency: %u\n", xclk_freq);
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

	sensor->mode = IMX568_DEFAULT_MODE;

	ret = imx568_init_controls(sensor, dev);
	if (ret)
		goto mutex_destroy;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &imx568_subdev_ops);
	sd->internal_ops = &imx568_subdev_internal_ops;
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

	v4l2_subdev_init_finalize(sd);

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret < 0)
		goto power_off;

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	dev_info(dev, "SONY IMX568 camera driver probed\n");

	return 0;

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
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	v4l2_device_unregister_subdev(sd);
	pm_runtime_disable(&client->dev);
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
