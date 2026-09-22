// SPDX-License-Identifier: GPL-2.0
/*
 * xc9080 I2C camera sensor driver for V4L2 / Media Controller bring-up.
 *
 * Derived from Metoak camera S315 sensor driver for Rockchip platform
 *
 * Copyright (C) Metoak
 * Copyright (C) 2026 Sima.ai 
 *
 * Initialization sequences and register
 * definitions derived from vendor-supplied
 * Developer Guide for S315 Stereo Vision Module
 * Document provided without confidentiality
 * restrictions for developer implementation.
 *
 * Licensed under GPL v2.
 *
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "s315_init_data.h"

/* ----  sensor width / height ------------------------------ */
#define XC9080_WIDTH 1280
#define XC9080_HEIGHT 480
/*
 * MIPI bus format. For YUV 4:2:2 sensors this is normally one of:
 *   MEDIA_BUS_FMT_UYVY8_1X16  (most common - U Y V Y order)
 *  
 */

#define XC9080_MBUS_CODE MEDIA_BUS_FMT_UYVY8_1X16

#define XC9080_NUM_LANES 2
#define XC9080_BPP 16 /* YUV422 = 16 bits/pixel on the bus */


/* PIXEL_RATE in pixels/sec, as seen by the receiver */
#define XC9080_PIXEL_RATE(freq, lanes) (((u64)(freq)*2 * (lanes)) / XC9080_BPP)

/* ------------------------------------------------------------------------- */

#define XC9080_SLAVE_ADDR  0x1B
#define SC132GS_SLAVE_ADDR 0x30
#define SIMOR_SLAVE_ADDR   0x66

/* SIMOR FPS register addresses (8-bit register address) */
#define SIMOR_REG_FPSX_HIGH  0x61
#define SIMOR_REG_FPSX_LOW   0x62
#define SIMOR_REG_FPSY_HIGH  0x63
#define SIMOR_REG_FPSY_LOW   0x64

/* XC9080 Bypass control registers */
#define XC9080_REG_BANK_SEL  0xFFFD /* Bank select, always write 0x80 */
#define XC9080_REG_PAGE_SEL  0xFFFE /* Page select */
#define XC9080_PAGE_SYSTEM   0x50 /* System control page */
#define XC9080_REG_CMOS_SEL  0x004D /* I2C Bypass / CMOS select */
#define XC9080_SEL_NONE      0x00 /* Disable Bypass, ISP AE resumes */
#define XC9080_SEL_CMOSA     0x01 /* Select CMOS-A */
#define XC9080_SEL_CMOSB     0x02 /* Select CMOS-B */
#define XC9080_SEL_BOTH      0x03 /* Select both CMOS */

/* SC132GS FPS lookup table (supported FPS: 10, 12, 15, 20, 25) */
typedef struct {
	int fps;
	/* SIMOR-side config */
	uint8_t simor_fpsy_high;
	uint8_t simor_fpsy_low;
	/* SC132GS CMOS-side config (registers 0x320C~0x320F, 0x3228~0x3229) */
	uint8_t cmos_hts[4]; // 0x320C ~ 0x320F: HTS + VTS
	uint8_t cmos_blank[2]; // 0x3228 ~ 0x3229: Blank Rows
} FpsConfig;

static const FpsConfig fps_table[] = {
	/* FPS  FPSY_H FPSY_L  HTS_H HTS_L VTS_H VTS_L  BLK_H BLK_L */
	{ 10, 0x10, 0x7A, { 0x05, 0x78, 0x11, 0x94 }, { 0x11, 0x8E } },
	{  5, 0x0D, 0xBA, { 0x05, 0x78, 0x0E, 0xA6 }, { 0x0E, 0xA0 } },
	{ 15, 0x0A, 0xFB, { 0x05, 0x78, 0x0B, 0xB8 }, { 0x0B, 0xB2 } },
	{ 20, 0x08, 0x3C, { 0x05, 0x78, 0x08, 0xCA }, { 0x08, 0xC4 } },
	{ 25, 0x06, 0x96, { 0x05, 0x78, 0x07, 0x08 }, { 0x07, 0x02 } },
};

struct xc9080_sensor {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;

	struct v4l2_fract frame_interval;
	const FpsConfig *fps_cfg;

	unsigned int num_data_lanes;
	u64 link_freq;
	unsigned int link_freq_index;
	u64 pixel_rate;
	u32 startup_delay_ms;
};

static const struct v4l2_fract s315_supported_intervals[] = {
	{ .numerator = 1, .denominator = 10 }, /* 10 fps */
	{ .numerator = 1, .denominator = 5  }, /* 12 fps */
	{ .numerator = 1, .denominator = 15 }, /* 15 fps */
	{ .numerator = 1, .denominator = 20 }, /*  20 fps */
	{ .numerator = 1, .denominator = 25 }, /*  25 fps */
};

static const s64 xc9080_link_freqs[] = {
	450000000,
	480000000,
	540000000,
};

static const struct xc9080_regval xc9080_start_stream_data[] = {
	{ 0xfffd, 0x80 }, { 0xfffe, 0x26 }, { 0x8010, 0x0d },
	{ 0xfffe, 0x30 }, { 0x0004, 0x10 }, { 0x2300, 0xfc },
};

static const struct xc9080_regval xc9080_stop_stream_data[] = {
	{ 0xfffd, 0x80 },
	{ 0xfffe, 0x26 },
	{ 0x8010, 0x09 },
};

void xc9080_disable_ae(struct xc9080_sensor *);
void xc9080_enable_ae(struct xc9080_sensor *);

static inline struct xc9080_sensor *to_xc9080(struct v4l2_subdev *sd)
{
	return container_of(sd, struct xc9080_sensor, sd);
}

/* ===== Pad ops: format negotiation ====================================== */

static int xc9080_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = XC9080_MBUS_CODE;
	return 0;
}

static int xc9080_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != XC9080_MBUS_CODE)
		return -EINVAL;
	fse->min_width = fse->max_width = XC9080_WIDTH;
	fse->min_height = fse->max_height = XC9080_HEIGHT;
	return 0;
}

static int xc9080_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	/* We support exactly one format - clamp to it */
	fmt->format.code = XC9080_MBUS_CODE;
	fmt->format.width = XC9080_WIDTH;
	fmt->format.height = XC9080_HEIGHT;
	fmt->format.field = V4L2_FIELD_NONE;
	/* Color metadata for YUV422 (BT.601 SD / BT.709 HD - pick one) */
	fmt->format.colorspace = V4L2_COLORSPACE_REC709;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_709;
	fmt->format.quantization = V4L2_QUANTIZATION_LIM_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_709;

	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	return 0;
}

/* ===== Video ops: stream on/off (NO-OP, by design) ====================== */

static int simor_read_reg(struct i2c_client *client, u8 reg, u8 *recv_val)
{
	u8 reg_addr = reg; /* Register to read */

	struct i2c_msg msgs[2];
	int ret;

	if (!recv_val)
		return -EINVAL;

	/* Message 1: Write register address */
	msgs[0].addr = SIMOR_SLAVE_ADDR;
	msgs[0].flags = 0; /* Write */
	msgs[0].len = 1;
	msgs[0].buf = &reg_addr;

	/* Message 2: Read register value */
	msgs[1].addr = SIMOR_SLAVE_ADDR;
	msgs[1].flags = I2C_M_RD; /* Read */
	msgs[1].len = 1;
	msgs[1].buf = recv_val;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2)
		return (ret < 0) ? ret : -EIO;

	return 0;
}

static int simor_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	u8 buf[2];
	struct i2c_msg msg;
	int ret;

	buf[0] = reg;
	buf[1] = val;

	msg.addr = SIMOR_SLAVE_ADDR;
	msg.flags = 0; /* Write */
	msg.len = sizeof(buf);
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		return (ret < 0) ? ret : -EIO;

	return 0;
}

static int sc132gs_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];
	struct i2c_msg msg;
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = SC132GS_SLAVE_ADDR;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return (ret == 1) ? 0 : -EIO;
}

static int xc9080_read_reg(struct i2c_client *client, u16 dev_addr, u16 reg,
			   u8 *val)
{
	struct i2c_msg msgs[2];
	u8 reg_buf[2];
	int ret;

	reg_buf[0] = reg >> 8;
	reg_buf[1] = reg & 0xff;

	msgs[0].addr = dev_addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = reg_buf;

	msgs[1].addr = dev_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;

	ret = i2c_transfer(client->adapter, msgs, 2);

	return (ret == 2) ? 0 : -EIO;
	return 0;
}

static int xc9080_write_reg(struct xc9080_sensor *xc, u16 reg, u8 val)
{
	struct i2c_client *client = xc->client;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev,
			"I2C write failed: reg=0x%04x val=0x%02x ret=%d\n", reg,
			val, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int xc9080_write_array(struct xc9080_sensor *xc9080,
			      const struct xc9080_regval *regs,
			      unsigned int num_regs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_regs; i++) {
		ret = xc9080_write_reg(xc9080, regs[i].reg, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int xc9080_start_streaming(struct xc9080_sensor *xc9080)
{
	int ret;

	dev_info(&xc9080->client->dev, "xc9080: start streaming\n");

	ret = xc9080_write_array(xc9080, xc9080_start_stream_data,
				 ARRAY_SIZE(xc9080_start_stream_data));
	if (ret) {
		dev_err(&xc9080->client->dev,
			"xc: stream-on sequence failed\n");
		return ret;
	}
	return 0;
}

static int xc9080_stop_streaming(struct xc9080_sensor *xc9080)
{
	int ret;

	dev_info(&xc9080->client->dev, "xc: stop streaming\n");

	ret = xc9080_write_array(xc9080, xc9080_stop_stream_data,
				 ARRAY_SIZE(xc9080_stop_stream_data));
	if (ret) {
		dev_err(&xc9080->client->dev,
			"xc9080: stream-off sequence failed\n");
		return ret;
	}
	return 0;
}

static int xc9080_sensor_apply_fps(struct xc9080_sensor *xc9080)
{
	/* Step 1: Write SIMOR FPS registers
     * FPSX is fixed at 0x059D (for SC132GS) */
	simor_write_reg(xc9080->client, SIMOR_REG_FPSX_HIGH, 0x05);
	simor_write_reg(xc9080->client, SIMOR_REG_FPSX_LOW, 0x9D);

	simor_write_reg(xc9080->client, SIMOR_REG_FPSY_HIGH,
			xc9080->fps_cfg->simor_fpsy_high);
	simor_write_reg(xc9080->client, SIMOR_REG_FPSY_LOW,
			xc9080->fps_cfg->simor_fpsy_low);

	/* Step 2: Write SC132GS CMOS FPS registers via XC9080 Bypass
     * Suspend AE first, then select both CMOS for consistent config */
	xc9080_disable_ae(xc9080);

	xc9080_write_reg(xc9080, XC9080_REG_BANK_SEL, 0x80);
	xc9080_write_reg(xc9080, XC9080_REG_PAGE_SEL, XC9080_PAGE_SYSTEM);
	xc9080_write_reg(xc9080, XC9080_REG_CMOS_SEL, XC9080_SEL_BOTH);

	/* Write HTS + VTS (0x320C ~ 0x320F) */
	for (int i = 0; i < 4; i++) {
		sc132gs_write_reg(xc9080->client, 0x320C + i,
				  xc9080->fps_cfg->cmos_hts[i]);
	}

	/* Write Blank Rows (0x3228 ~ 0x3229) */
	for (int i = 0; i < 2; i++) {
		sc132gs_write_reg(xc9080->client, 0x3228 + i,
				  xc9080->fps_cfg->cmos_blank[i]);
	}
	/* Resume AE */
	xc9080_enable_ae(xc9080);
	return 0;
}

static int xc9080_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct xc9080_sensor *xc9080 =
		container_of(sd, struct xc9080_sensor, sd);
	int ret = 0;

	mutex_lock(&xc9080->lock);

	/* Intentional no-op: userspace handles sensor streaming over I2C. */
	dev_dbg(sd->dev, "s_stream(%d) - no-op\n", enable);

	if (enable) {
		/* then apply selected FPS timing */
		if (xc9080->fps_cfg != NULL) {
			ret = xc9080_sensor_apply_fps(xc9080);
			if (ret)
				return ret;
		} else {
			dev_info(sd->dev,"fps setting NULL\n");
		}
		/*
                 * Apply default & customized values
                 * and then start streaming.
                 */
		ret = xc9080_start_streaming(xc9080);

	} else {
		xc9080_stop_streaming(xc9080);
	}

	mutex_unlock(&xc9080->lock);
	return 0;
}

static int xc9080_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	/* index out of range → stop enumeration */
	if (fie->index >= ARRAY_SIZE(s315_supported_intervals))
		return -EINVAL;

	fie->interval = s315_supported_intervals[fie->index];

	return 0;
}

static int xc9080_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_interval *fi)
{
	u32 requested_fps;
	u32 best_fps;
	int i;
	int ret = 0;

	// struct xc9080_sensor *xc9080 = v4l2_get_subdevdata(sd);
	struct xc9080_sensor *xc9080 =
		container_of(sd, struct xc9080_sensor, sd);

	if (fi->interval.numerator == 0 || fi->interval.denominator == 0) {
		dev_err(sd->dev, "invalid interval %u/%u\n",
			fi->interval.denominator, fi->interval.numerator);
		return -EINVAL;
	}

	dev_info(sd->dev, ">>> numerator    = %u\n", fi->interval.numerator);
	dev_info(sd->dev, ">>> denominator  = %u\n", fi->interval.denominator);

	/* convert to fp */
	requested_fps = (fi->interval.denominator / fi->interval.numerator);
	dev_info(sd->dev, "requested FPS: %u\n", requested_fps);

	
	for (i = 0; i < ARRAY_SIZE(s315_supported_intervals); i++) {
		u32 supported_fps = s315_supported_intervals[i].denominator /
				    s315_supported_intervals[i].numerator;

		if (supported_fps == requested_fps) {
			/* exact match found */
			xc9080->frame_interval = s315_supported_intervals[i];
			dev_info(sd->dev, "exact match: %u fps\n",
				 requested_fps);
			best_fps = requested_fps;
			goto apply;
		}
	}

	
	dev_warn(sd->dev, "FPS %u not supported, \n", requested_fps);

	return -EINVAL;

apply:
	xc9080->frame_interval.numerator = 1;
	xc9080->frame_interval.denominator = best_fps;

	/* Look up config */
	const FpsConfig *cfg = NULL;
	for (int i = 0; i < sizeof(fps_table) / sizeof(fps_table[0]); i++) {
		if (fps_table[i].fps == best_fps) {
			cfg = &fps_table[i];
			break;
		}
	}
	if (!cfg)
		return -EINVAL; // Unsupported FPS

	xc9080->fps_cfg = cfg;

	/* write to HW apply selected FPS timing */
	ret = xc9080_sensor_apply_fps(xc9080);
	if (ret)
		return ret;

	/* return actual interval back to v4l2-ctl */
	fi->interval = xc9080->frame_interval;

	return 0;
}

static int xc9080_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_interval *fi)
{
	// struct xc9080_sensor *xc9080 = v4l2_get_subdevdata(sd);
	struct xc9080_sensor *xc9080 =
		container_of(sd, struct xc9080_sensor, sd);

	/* return whatever was last set (or default) */
	fi->interval = xc9080->frame_interval;

	dev_dbg(sd->dev, "get interval: %u/%u fps\n", fi->interval.denominator,
		fi->interval.numerator);

	return 0;
}

/* ===== Op tables ======================================================== */

static const struct v4l2_subdev_video_ops xc9080_video_ops = {

	.s_stream = xc9080_s_stream,
};

static const struct v4l2_subdev_pad_ops xc9080_pad_ops = {
	.enum_mbus_code = xc9080_enum_mbus_code,
	.enum_frame_size = xc9080_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt, /* core helper, reads state */
	.set_fmt = xc9080_set_fmt,
	.enum_frame_interval = xc9080_enum_frame_interval, /*  listing  fps */
	.get_frame_interval = xc9080_get_frame_interval,
	.set_frame_interval = xc9080_set_frame_interval,

};

static const struct v4l2_subdev_ops xc9080_subdev_ops = {
	.video = &xc9080_video_ops,
	.pad = &xc9080_pad_ops,
};

static const struct media_entity_operations xc9080_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ===== init_state: default format applied to fresh subdev state ========= */

static int xc9080_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, 0);

	fmt->code = XC9080_MBUS_CODE;
	fmt->width = XC9080_WIDTH;
	fmt->height = XC9080_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_709;
	return 0;
}

static const struct v4l2_subdev_internal_ops xc9080_internal_ops = {
	.init_state = xc9080_init_state,
};


/*
 * Suspend XC9080 ISP AE (call before CMOS access)
 * Setting CMOS select to BOTH freezes ISP AE
 */
void xc9080_disable_ae(struct xc9080_sensor *xc)
{
	xc9080_write_reg(xc, XC9080_REG_PAGE_SEL, XC9080_PAGE_SYSTEM);
	xc9080_write_reg(xc, XC9080_REG_CMOS_SEL, XC9080_SEL_BOTH);
}

/*
 * Resume XC9080 ISP AE (call after CMOS access)
 * Must restore SC132GS AE registers before disabling Bypass
 */
void xc9080_enable_ae(struct xc9080_sensor *xc)
{
	uint8_t ae_restore_val = 0x2B; /* SC132GS AE mode register default */

	xc9080_write_reg(xc, XC9080_REG_PAGE_SEL, XC9080_PAGE_SYSTEM);

	/* Restore CMOS-A AE register */
	xc9080_write_reg(xc, XC9080_REG_CMOS_SEL, XC9080_SEL_CMOSA);
	sc132gs_write_reg(xc->client, 0x3E03, ae_restore_val);

	/* Restore CMOS-B AE register */
	xc9080_write_reg(xc, XC9080_REG_CMOS_SEL, XC9080_SEL_CMOSB);
	sc132gs_write_reg(xc->client, 0x3E03, ae_restore_val);

	/* Disable Bypass, ISP AE resumes automatically */
	xc9080_write_reg(xc, XC9080_REG_CMOS_SEL, XC9080_SEL_NONE);
}

static int xc9080_full_init(struct xc9080_sensor *xc9080)
{
	int ret;

	ret = xc9080_write_array(xc9080, xc9080_stream_off_data,
				 ARRAY_SIZE(xc9080_stream_off_data));
	if (ret) {
		dev_err(&xc9080->client->dev,
			"xc9080 : init sequence failed\n");
		return ret;
	}

	ret = xc9080_write_array(xc9080, xc9080_init_data,
				 ARRAY_SIZE(xc9080_init_data));
	if (ret) {
		dev_err(&xc9080->client->dev,
			"xc9080 : init sequence failed\n");
		return ret;
	}

	/* Step 1: Stream off first to ensure clean ISP state */
	int sc132gs_init_data_count = ARRAY_SIZE(sc132gs_init_data);

	/* Step 3: Disable AE to prevent conflict with CMOS init */
	xc9080_disable_ae(xc9080);

	/* Step 4: Enable Bypass (both CMOS), write SC132GS init sequence */
	xc9080_write_reg(xc9080, 0xFFFD, 0x80);
	xc9080_write_reg(xc9080, 0xFFFE, 0x50);
	xc9080_write_reg(xc9080, 0x004D, 0x03); /* Bypass BOTH */

	for (int i = 0; i < sc132gs_init_data_count; i++) {
		sc132gs_write_reg(xc9080->client, sc132gs_init_data[i].reg,
				  sc132gs_init_data[i].val);
	}

	/* Step 5: Restore AE */
	xc9080_enable_ae(xc9080);

	/* Step 6: Wait 500ms then verify firmware handshake */
	msleep(500);
	/* ... see handshake verification code below ... */

	// Write handshake request
	xc9080_write_reg(xc9080, 0x0137, 0x66);
	msleep(5); // Wait 5ms

	dev_info(&xc9080->client->dev,"Sending handshake \n");

	// Read handshake response
	u8 firmware_flag = 0;
	xc9080_read_reg(xc9080->client, 0x1B, 0x0137, &firmware_flag);

	if (firmware_flag == 0x88) {
		dev_info(&xc9080->client->dev,"Received firmware flag = %#x \n ", firmware_flag);
		dev_info(&xc9080->client->dev,"XC9080 ISP initialization successful\n");
	} else {
		dev_info(&xc9080->client->dev,"XC9080 ISP may not be working (read back: 0x%02X)\n",
		       firmware_flag);
	}

	u8 simor_init_flag = 0;
	simor_read_reg(xc9080->client, 0xF0, &simor_init_flag);
	if (simor_init_flag == 0xEA) {
		dev_info(&xc9080->client->dev,"Received simor init flag = %#x \n ", simor_init_flag);
		dev_info(&xc9080->client->dev,"simor initialization successful\n");
	} else {
		dev_info(&xc9080->client->dev,"simor may not be working (read back: 0x%02X)\n",
		       firmware_flag);
	}

	return 0;
}

static int xc9080_find_link_freq_index(u64 freq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(xc9080_link_freqs); i++) {
		if (freq == xc9080_link_freqs[i])
			return i;
	}

	return -EINVAL;
}

static int xc9080_parse_hw_config(struct xc9080_sensor *s)
{
	struct device *dev = s->dev;
	struct fwnode_handle *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct gpio_desc *reset_gpio;
	unsigned int i;
	u32 startup_delay_ms;
	int ret;

	/* Optional custom startup delay */
	if (!device_property_read_u32(dev, "startup-delay-ms",
				      &startup_delay_ms))
		s->startup_delay_ms = startup_delay_ms;
	else
		s->startup_delay_ms = 0;

	/* Get GPIO from DTS */
	reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio)) {
		dev_err(dev, "failed to get reset gpio: %ld\n",
			PTR_ERR(reset_gpio));
		return PTR_ERR(reset_gpio);
	}

	s->reset_gpio = reset_gpio;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -ENXIO, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		s->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
		break;

	default:
		ret = dev_err_probe(dev, -EINVAL,
				    "invalid CSI-2 data lanes: %u\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(dev, -EINVAL,
				    "no link-frequencies defined\n");
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		int idx;

		idx = xc9080_find_link_freq_index(bus_cfg.link_frequencies[i]);
		if (idx < 0)
			continue;

		s->link_freq = bus_cfg.link_frequencies[i];
		s->link_freq_index = idx;

		dev_info(dev, "CSI-2 lanes=%u link_freq=%llu Hz index=%u\n",
			 s->num_data_lanes, s->link_freq, s->link_freq_index);

		ret = 0;
		goto done_endpoint_free;
	}

	ret = dev_err_probe(dev, -EINVAL,
			    "DTS link-frequencies do not match driver table\n");

done_endpoint_free:
	s->pixel_rate = XC9080_PIXEL_RATE(s->link_freq, s->num_data_lanes);

	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int xc9080_init_controls(struct xc9080_sensor *s)
{
	struct v4l2_ctrl_handler *hdl = &s->ctrls;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 2);
	if (ret)
		return ret;

	/* LINK_FREQ - many CSI bridges need this to configure the PHY */
	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(xc9080_link_freqs) - 1,
				      s->link_freq_index, xc9080_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* PIXEL_RATE - some bridges/userspace tools require this */
	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, s->pixel_rate,
				 s->pixel_rate, 1, s->pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	s->sd.ctrl_handler = hdl;
	return 0;
}

/* ===== Probe / remove =================================================== */

static int xc9080_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct xc9080_sensor *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->client = client;
	s->dev = dev;

	mutex_init(&s->lock);

	/* Initialize subdev (this sets sd->dev, sd->name, owner, etc.) */
	v4l2_i2c_subdev_init(&s->sd, client, &xc9080_subdev_ops);
	s->sd.internal_ops = &xc9080_internal_ops;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*  set default FPS  */
	s->frame_interval.numerator = 1;
	s->frame_interval.denominator = 25; /* default 25fps */

	/* Look up config */
	const FpsConfig *cfg = NULL;
	for (int i = 0; i < sizeof(fps_table) / sizeof(fps_table[0]); i++) {
		if (fps_table[i].fps == s->frame_interval.denominator) {
			cfg = &fps_table[i];
			break;
		}
	}
	if (!cfg)
		return -EINVAL; // Unsupported FPS

	s->fps_cfg = cfg;

	ret = xc9080_parse_hw_config(s);
	if (ret)
		return ret;
	/* Controls */
	ret = xc9080_init_controls(s);
	if (ret)
		return dev_err_probe(dev, ret, "ctrl init failed\n");

	/* Media pad: one source pad (sensor output) */
	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s->sd.entity.ops = &xc9080_media_ops;
	ret = media_entity_pads_init(&s->sd.entity, 1, &s->pad);
	if (ret)
		goto err_ctrls;

	/* Allocate per-handle subdev state machinery */
	s->sd.state_lock = &s->lock;
	ret = v4l2_subdev_init_finalize(&s->sd);
	if (ret)
		goto err_entity;

	/* Register with async core - bridge driver binds when DT graph matches */
	ret = v4l2_async_register_subdev_sensor(&s->sd);
	if (ret)
		goto err_finalize;

	/* Runtime PM (no-op on this driver but keeps frameworks happy) */
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	if (s->startup_delay_ms)
		msleep(s->startup_delay_ms);

	/* Do reset sequence */
	gpiod_set_value(s->reset_gpio, 1); /* assert reset */
	msleep(100);
	gpiod_set_value(s->reset_gpio, 0); /* deassert reset */
	msleep_interruptible(3000); /* wait for device ready */

	devm_gpiod_put(dev, s->reset_gpio); // relese gpio

	/* Init */
	ret = xc9080_full_init(s);
	if (ret)
		return dev_err_probe(dev, ret, "ctrl init failed\n");

	dev_info(dev, "xc9080 sensor registered: %dx%d, mbus=0x%x\n",
		 XC9080_WIDTH, XC9080_HEIGHT, XC9080_MBUS_CODE);
	return 0;

err_finalize:
	v4l2_subdev_cleanup(&s->sd);
err_entity:
	media_entity_cleanup(&s->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s->ctrls);
	return ret;
}

static void xc9080_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc9080_sensor *s = to_xc9080(sd);

	pm_runtime_disable(&client->dev);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->ctrls);
}

/* ===== DT / I2C boilerplate ============================================= */

static const struct of_device_id xc9080_of_match[] = {
	{ .compatible = "Metoak,xc9080" },
	{}
};
MODULE_DEVICE_TABLE(of, xc9080_of_match);

static const struct i2c_device_id xc9080_id[] = { { "xc9080_metoak", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, xc9080_id);

static struct i2c_driver xc9080_i2c_driver = {
    .driver = {
        .name           = "xc9080_metoak",
        .of_match_table = xc9080_of_match,
    },
    .probe    = xc9080_probe,
    .remove   = xc9080_remove,
    .id_table = xc9080_id,
};
module_i2c_driver(xc9080_i2c_driver);

MODULE_AUTHOR("Sagar Kontam <sagar.kontam@sima.ai>");
MODULE_DESCRIPTION("Metoak xc9080 sensor driver");
MODULE_LICENSE("GPL v2");
