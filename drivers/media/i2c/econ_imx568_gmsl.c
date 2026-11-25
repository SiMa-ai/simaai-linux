// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for econ IMX568 camera.
 * Copyright (C) 2026 e-con systems
 */
#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include "econ_imx568_gmsl.h"
#include "econ_imx568_serdes.h"

// Define EN_DEBUG_PRINTS Macro to enable debug prints
#define EN_DEBUG_PRINTS

/* IMX568 native and active pixel array size */
#define IMX568_NATIVE_WIDTH 2432U
#define IMX568_NATIVE_HEIGHT 2048U

unsigned char errorcheck(char *data, unsigned int len)
{
	unsigned int i = 0;
	unsigned char crc = 0x00;

	for (i = 0; i < len; i++)
		crc ^= data[i];

	return crc;
}

static int cam_read(struct i2c_client *client, u8 *val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = val,
	};

	dev_dbg(&client->dev,
         "cam_read:: client->addr = 0x%02x\n",
         client->addr);
	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	return 0;

err:
	dev_err(&client->dev, "Failed reading register ret = %d!\n", ret);
	return ret;
}

static int cam_write(struct i2c_client *client, u8 *val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = count,
		.buf = val,
	};
	dev_dbg(&client->dev,
         "cam_write:: client->addr = 0x%02x\n",
         client->addr);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register ret = %d!\n",
			ret);
		return ret;
	}

	return 0;
}

static int cam_get_cmd_status(struct i2c_client *client, uint8_t *cmd_id,
			      uint16_t *cmd_status, uint8_t *ret_code)
{
	unsigned char mc_data[100];
	unsigned char mc_ret_data[100];
	uint32_t payload_len = 0, err = 0;
	uint8_t orig_crc = 0, calc_crc = 0;

	// Number of bytes will be transmitted in the 2nd i2c transaction
	payload_len = 1;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(client, mc_data, TX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev,
			" %s(%d) MCU Get CMD Status Write Error - %d\n",
			__func__, __LINE__, err);
		return -1;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = *cmd_id;

	err = cam_write(client, mc_data, 3);
	if (err != 0) {
		dev_err(&client->dev,
			" %s(%d) MCU Get CMD Status Write Error - %d\n",
			__func__, __LINE__, err);
		return -1;
	}

	payload_len = CMD_STATUS_MSG_LEN;
	memset(mc_ret_data, 0x00, payload_len);

	err = cam_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev,
			" %s(%d) MCU Get CMD Status Length Error - %d\n",
			__func__, __LINE__, err);
		return -1;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 3);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev,
			" %s(%d) MCU Get CMD Status Error CRC 0x%02x != 0x%02x\n",
			__func__, __LINE__, orig_crc, calc_crc);
		return -1;
	}

	*cmd_id = mc_ret_data[2];
	*cmd_status = mc_ret_data[3] << 8 | mc_ret_data[4];
	*ret_code = mc_ret_data[payload_len - 1];

	return 0;
}

/* NOTE : Caller need to take the lock */
static int cam_set_ctrl(struct i2c_client *client, struct imx568 *priv,
			uint32_t arg_ctrl_id, uint8_t ctrl_type,
			int64_t curr_val)
{
	unsigned char mc_data[100];
	uint32_t payload_len = 0, ctrl_val_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0;
	int retry = 10;

	/* call ISP Ctrl config command */
	for (loop = 0; loop < priv->num_ctrls; loop++) {
		if (priv->ctrldb[loop] == arg_ctrl_id) {
			index = loop;
			break;
		}
	}
	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}
	payload_len = (ctrl_type == CTRL_STANDARD) ? 11 : 20;
	/* First Txn Payload length = 0 */
	ctrl_val_len = (ctrl_type == CTRL_STANDARD) ? 4 : 8;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	/* Second Txn */
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;

	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Control ID */
	mc_data[4] = arg_ctrl_id >> 24;
	mc_data[5] = arg_ctrl_id >> 16;
	mc_data[6] = arg_ctrl_id >> 8;
	mc_data[7] = arg_ctrl_id & 0xFF;

	/* Ctrl Type */
	mc_data[8] = ctrl_type;

	/* Ctrl Value */
	if (ctrl_type == CTRL_STANDARD) {
		mc_data[9] = curr_val >> 24;
		mc_data[10] = curr_val >> 16;
		mc_data[11] = curr_val >> 8;
		mc_data[12] = curr_val & 0xFF;
		/* CRC */
		mc_data[13] = errorcheck(&mc_data[2], payload_len);

	} else {
		mc_data[9] = V4L2_CTRL_TYPE_INTEGER64;
		mc_data[10] = ctrl_val_len >> 24;
		mc_data[11] = ctrl_val_len >> 16;
		mc_data[12] = ctrl_val_len >> 8;
		mc_data[13] = ctrl_val_len & 0xFF;
		for (loop = 0; loop < ctrl_val_len; loop++)
			mc_data[21 - loop] = (curr_val >> (8 * loop));
		/* CRC */
		mc_data[22] = errorcheck(&mc_data[2], payload_len);
	}
	err = cam_write(client, mc_data, payload_len + 3);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) MCU Set Ctrl Error - %d\n",
			__func__, __LINE__, err);
		ret = -1;
		goto exit;
	}

	while (--retry > 0) {
		usleep_range(1000, 2000);
		cmd_id = CMD_ID_SET_CTRL;
		if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev,
				" %s(%d) MCU Get CMD Status Error\n", __func__,
				__LINE__);
			ret = -1;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d MCU Get CMD Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			ret = -1;
			goto exit;
		}
	}

exit:
	return ret;
}

static int cam_set_exposure(struct i2c_client *client, struct imx568 *priv,
			    s64 val)
{
	int err = 0, retry = 5;
	uint64_t data = val;

	while (--retry > 0) {
		err = cam_set_ctrl(client, priv, EXPOSURE_CTRL_ID,
				   CTRL_EXTENDED, data);
		if (err < 0) {
			dev_err(&client->dev, "%s[%d] Fail! retrying\n",
				__func__, __LINE__);
			continue;
		}
		return 0;
	}

	dev_err(&client->dev, "%s[%d] Failed after retries!\n", __func__,
		__LINE__);
	return -EINVAL;
}

/*
 * Upper bound on the MCU frame rate, in fps. Exposed so the dual-camera
 * deserializer line-memory overflow can be traded against
 * frame rate without rebuilding: insmod econ_imx568_gmsl.ko max_framerate=15
 */
static unsigned int max_framerate = 30;
module_param(max_framerate, uint, 0644);
MODULE_PARM_DESC(max_framerate, "Cap the sensor frame rate in fps (default 30)");

static int cam_set_framerate(struct i2c_client *client, struct imx568 *priv,
			     s64 val)
{
	int err = 0, retry = 5;
	uint64_t data = val;

	while (--retry > 0) {
		err = cam_set_ctrl(client, priv, FRAMERATE_CTRL_ID,
				   CTRL_EXTENDED, data);
		if (err < 0) {
			dev_err(&client->dev, "%s[%d] Fail! retrying\n",
				__func__, __LINE__);
			continue;
		}
		return 0;
	}

	dev_err(&client->dev, "%s[%d] Failed after retries!\n", __func__,
		__LINE__);
	return -EINVAL;
}

// MCU APIs
static int cam_init(struct i2c_client *client)
{
	unsigned char mc_data[100];
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 10, err = 0, ret = 0;

	/* check current status of cam */
	cmd_id = CMD_ID_INIT_CAM;
	if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
		dev_err(&client->dev, " %s(%d) MCU CAM Init ISP Error\n",
			__func__, __LINE__);
		return -1;
	}

	if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
	    (retcode == ERRCODE_SUCCESS)) {
		dev_info(&client->dev, " %s %d CAM Initialized !!\n", __func__,
			 __LINE__);
		return 0;
	}

	/* call cam init command */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev,
			" %s(%d) MCU Get CMD CAM Init Error - %d\n",
			__func__, __LINE__, err);
		return -1;
	}

	while (--retry > 0) {
		msleep(100); // wait till sensor to initialise
		/* Some Sleep for init to process */
		usleep_range(1000, 2000);
		cmd_id = CMD_ID_INIT_CAM;
		if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID CAM INIT Error\n", __func__,
				__LINE__);
			usleep_range(5000, 6000);
			ret = -1;
			continue;
		}

		if (cmd_status == MCU_CMD_STATUS_SUCCESS) {
			dev_err(&client->dev, "%s(%d) CAM INIT Success !!\n",
				__func__, __LINE__);
			ret = 0;
			goto exit_init;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d MCU CMD ID CAM INIT Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			ret = -1;
			continue;
		}
	}
exit_init:
	return ret;
}

static inline struct imx568 *to_imx568(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx568, sd);
}

static void imx568_set_default_format(struct imx568 *imx568)
{
	imx568->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}

static int imx568_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}
// Setting Gain
static int32_t cam_set_gain(struct imx568 *imx568, uint64_t gain)
{
	unsigned char mc_data[100];
	uint32_t payload_len = 0;
	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int ret = 0, err = 0, retry = 5;
	int loop = 0;
	uint16_t ctrl_val_len = 0, index = 0;

	payload_len = 20;
	ctrl_val_len = 8;

	index = 0x00; // GAIN Control Index

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ret = cam_write(imx568->i2c_client, mc_data, 5);
	if (ret != 0) {
		dev_err(&imx568->i2c_client->dev,
			" %s(%d) MCU Get CMD CAM Init Error - %d\n",
			__func__, __LINE__, ret);

		return -1;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	/* Control ID */
	mc_data[4] = GAIN_CTRL_ID >> 24;
	mc_data[5] = GAIN_CTRL_ID >> 16;
	mc_data[6] = GAIN_CTRL_ID >> 8;
	mc_data[7] = GAIN_CTRL_ID & 0xFF;
	/* Ctrl Type */
	mc_data[8] = CTRL_EXTENDED;
	mc_data[9] = V4L2_CTRL_TYPE_INTEGER64;
	mc_data[10] = ctrl_val_len >> 24;
	mc_data[11] = ctrl_val_len >> 16;
	mc_data[12] = ctrl_val_len >> 8;
	mc_data[13] = ctrl_val_len & 0xFF;
	for (loop = 0; loop < ctrl_val_len; loop++)
		mc_data[21 - loop] = (gain >> (8 * loop));
	/* CRC */
	mc_data[22] = errorcheck(&mc_data[2], payload_len);

	ret = cam_write(imx568->i2c_client, mc_data, payload_len + 3);
	if (ret != 0) {
		dev_err(&imx568->i2c_client->dev,
			" %s(%d) MCU Get CMD CAM Init Error - %d\n",
			__func__, __LINE__, ret);

		return -1;
	}

	return ret;
}

/* The MCU has one total-gain control: 0..24 dB is analog, anything above it is
 * digital stacked on the full analog ceiling. Write analog+digital as one value —
 * writing either channel on its own drops the other's contribution. */
static int32_t sensor_apply_total_gain(struct imx568 *imx568)
{
	uint32_t total = imx568->again + imx568->dgain;

	return cam_set_gain(imx568, (uint64_t)total);
}

static int32_t sensor_set_analogue_gain(struct imx568 *imx568, int32_t gain)
{
	int32_t prev = imx568->again;
	int32_t ret = 0;

	if (prev != gain) {
		imx568->again = gain;
		ret = sensor_apply_total_gain(imx568);
		if (ret != 0)
			imx568->again = prev;
	}

	return ret;
}

static int32_t sensor_set_digital_gain(struct imx568 *imx568, int32_t gain)
{
	int32_t prev = imx568->dgain;
	int32_t ret = 0;

	if (prev != gain) {
		imx568->dgain = gain;
		ret = sensor_apply_total_gain(imx568);
		if (ret != 0)
			imx568->dgain = prev;
	}

	return ret;
}

static int32_t sensor_set_exposure(struct imx568 *imx568, uint32_t exposure_us)
{
	int32_t ret = 0;

	if (imx568->integration_time != exposure_us) {
		ret = cam_set_exposure(imx568->i2c_client, imx568, exposure_us);
		if (ret == 0)
			imx568->integration_time = exposure_us;
	}

	return ret;
}
static int imx568_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx568 *imx568 =
		container_of(ctrl->handler, struct imx568, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx568->sd);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sensor_set_analogue_gain(imx568, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = sensor_set_exposure(imx568, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = sensor_set_digital_gain(imx568, ctrl->val);
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops imx568_ctrl_ops = {
	.s_ctrl = imx568_set_ctrl,
};

static int imx568_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx568 *imx568 = to_imx568(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index > 0)
			return -EINVAL;

		code->code = codes[code->index];
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx568_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx568 *imx568 = to_imx568(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		if (fse->index >= imx568->nr_supported_formats)
			return -EINVAL;

		fse->min_width = imx568->cam_frmfmt[fse->index].size.width;
		fse->max_width = fse->min_width;
		fse->min_height = imx568->cam_frmfmt[fse->index].size.height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		// Not Implemented
		fse->min_width = IMX568_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX568_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx568_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx568_update_image_pad_format(struct imx568 *imx568,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = imx568->cam_frmfmt[imx568->frmfmt_mode].size.width;
	fmt->format.height =
		imx568->cam_frmfmt[imx568->frmfmt_mode].size.height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx568_reset_colorspace(&fmt->format);
}

static void imx568_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX568_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX568_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx568_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx568 *imx568 = to_imx568(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx568->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx568_update_image_pad_format(imx568, fmt);
			fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
			fmt->format.width =
				imx568->cam_frmfmt[imx568->frmfmt_mode]
					.size.width;
			fmt->format.height =
				imx568->cam_frmfmt[imx568->frmfmt_mode]
					.size.height;
		} else {
			imx568_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx568->mutex);
	return 0;
}

static int cam_stream_config(struct i2c_client *client, struct imx568 *priv,
			     uint32_t format, int mode, int frate_index)
{
	unsigned char mc_data[100];
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 10;

	// Find Index of the streaming mode
	for (loop = 0; (&priv->streamdb[loop]) != NULL; loop++) {
		if (priv->streamdb[loop] == mode) {
			index = loop + frate_index;
			break;
		}
	}
	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	dev_info(
		&client->dev,
		"Mode: %d, Width: %d, Height: %d, Format: 0x%x Framerate: %d\n",
		index, priv->cam_frmfmt[mode].size.width,
		priv->cam_frmfmt[mode].size.height, format,
		priv->cam_frmfmt[mode].framerates[frate_index]);

	// Payload length
	payload_len = 14;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	mc_data[4] = format >> 24;
	mc_data[5] = format >> 16;
	mc_data[6] = format >> 8;
	mc_data[7] = format & 0xFF;

	/* width */
	mc_data[8] = priv->cam_frmfmt[mode].size.width >> 8;
	mc_data[9] = priv->cam_frmfmt[mode].size.width & 0xFF;

	/* height */
	mc_data[10] = priv->cam_frmfmt[mode].size.height >> 8;
	mc_data[11] = priv->cam_frmfmt[mode].size.height & 0xFF;

	/* frame rate num */
	mc_data[12] = priv->cam_frmfmt[mode].framerates[frate_index] >> 8;
	mc_data[13] = priv->cam_frmfmt[mode].framerates[frate_index] & 0xFF;

	/* frame rate denom */
	mc_data[14] = 0x00;
	mc_data[15] = 0x01;

	mc_data[16] = errorcheck(&mc_data[2], payload_len); // CRC
	err = cam_write(
		client, mc_data,
		payload_len + 3); // Payload_len + CMD_SIGNATURE + CMD_ID + CRC
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) MCU Stream Config Error - %d\n",
			__func__, __LINE__, err);
		ret = -1;
		goto exit;
	}

	while (--retry > 0) {
		/* test Some time for processing command */
		usleep_range(1000, 2000);

		cmd_id = CMD_ID_STREAM_CONFIG;
		if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev,
				" %s(%d) MCU GET CMD Status Error : loop : %d\n",
				__func__, __LINE__, loop);
			ret = -1;
			continue;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			dev_info(&client->dev, " %s(%d) Status Success !!\n",
				 __func__, __LINE__);
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d ISP Get CMD Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			ret = -1;
			continue;
		}
		mdelay(1);
	}

exit:
	return ret;
}

static int imx568_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	struct imx568 *imx568 = to_imx568(sd);
	struct i2c_client *client = imx568->i2c_client;
	int err = 0, ret = 0, i = 0;
	int retry = 5;
	uint64_t curr_framerate = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx568->mutex);

	if (fmt->pad == IMAGE_PAD) {
		switch (fmt->format.code) {
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			imx568->format_fourcc = V4L2_PIX_FMT_SRGGB12;
			break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			imx568->format_fourcc = V4L2_PIX_FMT_SRGGB10;
			break;
		}

		for (i = 0; i < imx568->frm_fmt_size; i++) {
			if ((imx568->cam_frmfmt[i].size.width ==
			     fmt->format.width) &&
			    (imx568->cam_frmfmt[i].size.height ==
			     fmt->format.height) &&
			    (imx568->cam_frmfmt[i].fourcc ==
			     imx568->format_fourcc)) {
				imx568->frmfmt_mode =
					imx568->cam_frmfmt[i].mode;
				imx568->frate_index = 0;
				break;
			}
		}

		imx568_update_image_pad_format(imx568, fmt);

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
								fmt->pad);
			*framefmt = fmt->format;
		}
	} else {
		// Not Implemented
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
								fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx568_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx568->mutex);
	while (retry-- > 0) {
		err = cam_set_ctrl(imx568->i2c_client, imx568,
				   SENSOR_MODE_CTRL_ID, CTRL_STANDARD,
				   (int64_t)imx568->frmfmt_mode);
		if (err < 0)
			dev_err(&client->dev, "%s[%d]\n", __func__, __LINE__);
		else
			break;
	}

	if (retry < 0)
		return err;

	/* Real mode rate, capped at max_framerate (default 30): the ISP capture
	 * path sustains ~30 fps and the 64/76 fps 1080p modes starve it into a
	 * DMA stall.
	 *
	 * Lower it to trade frame rate for deserializer headroom. With both
	 * cameras streaming, each pipe's line memory drains through one shared
	 * CSI-2 output first-come-first-served, and at 30 fps the two pipes'
	 * transmissions fill ~62% of the line period - close enough that phase
	 * drift between the free-running sensors overflows the line memory
	 * (BACKTOP11 0x312 reads 0x66: LMO_Y, LMO_Z and both command FIFOs).
	 * Halving the rate doubles the line period without changing the
	 * per-line transmit time, so each pipe waits proportionally less.
	 */
	curr_framerate = imx568->cam_frmfmt[imx568->frmfmt_mode]
				 .framerates[imx568->frate_index];
	if (curr_framerate > max_framerate)
		curr_framerate = max_framerate;
	dev_info(&client->dev, "MCU framerate: %llu fps\n", curr_framerate);

	// set frame rate based on resolution
	retry = 5;
	while (retry-- > 0) {
		err = cam_set_framerate(client, imx568,
					curr_framerate * MULTIPLY_FACTOR);
		if (err < 0)
			dev_err(&client->dev, "%s[%d]\n", __func__, __LINE__);
		else
			break;
	}
	if (retry < 0)
		return err;

	mutex_lock(&imx568->mutex);

	retry = 5;
	while (retry-- > 0) {
		err = cam_stream_config(imx568->i2c_client, imx568,
					imx568->format_fourcc,
					imx568->frmfmt_mode,
					imx568->frate_index);
		if (err < 0) {
			dev_err(&client->dev, "%s[%d]\n", __func__, __LINE__);
			ret = err;
		} else {
			break;
		}
	}

	if (retry < 0) {
		mutex_unlock(&imx568->mutex);
		return ret;
	}

	mutex_unlock(&imx568->mutex);

	return ret;
}

static int cam_stream_on(struct i2c_client *client, struct imx568 *priv)
{
	unsigned char mc_data[100];
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 5, err = 0;

	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_ON;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(client, mc_data, TX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev, "%s(%d) MCU Stream On Write Error - %d\n",
			__func__, __LINE__, err);
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_ON;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev,
			" %s(%d) MCU Stream On Write Error - %d\n", __func__,
			__LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		usleep_range(1000, 2000);

		cmd_id = CMD_ID_STREAM_ON;
		if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev,
				" %s(%d) MCU Get CMD Stream On Error\n",
				__func__, __LINE__);
			err = -1;
			continue;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			dev_info(&client->dev,
				 " %s %dMCU Stream On Success !!\n", __func__,
				 __LINE__);
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d MCU Get CMD Stream On Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			err = -1;
			continue;
		}
		mdelay(1);
	}
exit:
	return err;
}

static int cam_stream_off(struct i2c_client *client, struct imx568 *priv)
{
	unsigned char mc_data[100];
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 5, err = 0;

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	usleep_range(1000, 2000);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	err = cam_write(client, mc_data, 2);
	usleep_range(1000, 2000);
	if (err != 0) {
		dev_err(&client->dev,
			"%s(%d) MCU Stream OFF Write Error - %d\n", __func__,
			__LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		usleep_range(1000, 2000);
		cmd_id = CMD_ID_STREAM_OFF;
		if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev,
				"%s(%d) MCU Get CMD Stream Off Error\n",
				__func__, __LINE__);
			err = -1;
			continue;
		}
		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			dev_info(&client->dev,
				 " %s %d MCU Get CMD Stream off Success !!\n",
				 __func__, __LINE__);
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d MCU Get CMD Stream off Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			err = -1;
			continue;
		}
		mdelay(1);
	}
exit:
	return err;
}

/* Start streaming */
static int imx568_start_streaming(struct imx568 *imx568)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx568->sd);
	int ret = 0, retry = 5;

	dev_info(&client->dev, "%s: Starting stream\n", __func__);
	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(imx568->sd.ctrl_handler);
	if (ret) {
		dev_err(&client->dev, "%s (%d) Setup Controls - Failed\n",
			__func__, __LINE__);
		return ret;
	}

	while (retry-- > 0) {
		ret = cam_stream_on(client, imx568);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s[%d] Stream On Failed! retrying\n", __func__,
				__LINE__);
			continue;
		}
		break;
	}
	if (retry < 0) {
		dev_err(&client->dev, "%s (%d) Stream_On - Failed\n", __func__,
			__LINE__);
	} else {
		dev_info(&client->dev, "%s (%d) Stream_On - Success\n",
			 __func__, __LINE__);
	}

	return ret;
}

/* Stop streaming */
static void imx568_stop_streaming(struct imx568 *imx568)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx568->sd);
	int ret = 0, retry = 5;

	dev_info(&client->dev, "%s: Stream Off\n", __func__);
	while (retry-- > 0) {
		ret = cam_stream_off(client, imx568);
		if (ret < 0)
			continue;
		break;
	}
	if (retry < 0) {
		dev_err(&client->dev, "%s (%d) Stream_OFF - Failed\n", __func__,
			__LINE__);
	} else {
		dev_info(&client->dev, "%s (%d) Stream_OFF - Success\n",
			 __func__, __LINE__);
	}
}

static int imx568_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx568 *imx568 = to_imx568(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx568->mutex);
	if (imx568->streaming == enable) {
		mutex_unlock(&imx568->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx568_start_streaming(imx568);
		if (ret)
			goto err_rpm_put;
	} else {
		imx568_stop_streaming(imx568);
		pm_runtime_put(&client->dev);
	}

	imx568->streaming = enable;

	mutex_unlock(&imx568->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx568->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx568_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *imx568 = to_imx568(sd);
	int ret;

	ret = clk_prepare_enable(imx568->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n", __func__);
		goto reg_off;
	}
	return 0;

reg_off:
	return ret;
}

static int imx568_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *imx568 = to_imx568(sd);

	clk_disable_unprepare(imx568->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx568->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx568_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *imx568 = to_imx568(sd);

	if (imx568->streaming)
		imx568_stop_streaming(imx568);

	return 0;
}

static int __maybe_unused imx568_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *imx568 = to_imx568(sd);
	int ret;

	if (imx568->streaming) {
		ret = imx568_start_streaming(imx568);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx568_stop_streaming(imx568);
	imx568->streaming = 0;
	return ret;
}

static int imx568_get_regulators(struct imx568 *imx568)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx568->sd);
	unsigned int i;

	for (i = 0; i < IMX568_NUM_SUPPLIES; i++)
		imx568->supplies[i].supply = imx568_supply_name[i];

	return devm_regulator_bulk_get(&client->dev, IMX568_NUM_SUPPLIES,
				       imx568->supplies);
}

static int imx568_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct imx568 *imx568 = to_imx568(sd);

	/*
	 * Report the rate the overlay actually asked for. The CSI-2 host
	 * programs its D-PHY from this, so a hardcoded value here silently
	 * mistrains the receiver whenever the board runs a different rate.
	 */
	cfg->link_freq = imx568->link_freq_value;
	return 0;
}

static int imx568_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct imx568 *imx568 = to_imx568(sd);
	struct v4l2_subdev_state *state;
	u32 code;

	if (pad != IMAGE_PAD)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	//Get the format code of stream 0 of image_pad
	code = v4l2_subdev_state_get_format(state, IMAGE_PAD, 0)->code;

	v4l2_subdev_unlock_state(state);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1; //One one stream supported per image_pad

	fd->entry->pixelcode = code;
	fd->entry->stream = 0; //Set stream id
	fd->entry->bus.csi2.vc = imx568->vc;
	fd->entry->bus.csi2.dt = MIPI_CSI2_DT_RAW12;

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
			.width = IMX568_NATIVE_WIDTH,
			.height = IMX568_NATIVE_HEIGHT,
		},
	};

	imx568_set_format(sd, state, &format);

	return 0;
}

static const struct v4l2_subdev_core_ops imx568_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx568_video_ops = {
	.s_stream = imx568_set_stream,
};

static const struct v4l2_subdev_pad_ops imx568_pad_ops = {
	.get_frame_desc = imx568_get_frame_desc,
	.enum_mbus_code = imx568_enum_mbus_code,
	.get_fmt = imx568_get_pad_format,
	.set_fmt = imx568_set_pad_format,
	.enum_frame_size = imx568_enum_frame_size,
	.get_mbus_config = imx568_get_mbus_config,
};

static const struct v4l2_subdev_ops imx568_subdev_ops = {
	.core = &imx568_core_ops,
	.video = &imx568_video_ops,
	.pad = &imx568_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx568_internal_ops = {
	.open = imx568_open,
	.init_state = imx568_init_state,
};

/* Initialize control handlers */
static int imx568_init_controls(struct imx568 *imx568)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx568->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx568->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &imx568->mutex;
	/* By default, PIXEL_RATE is read only */
	imx568->pixel_rate = v4l2_ctrl_new_std(
		ctrl_hdlr, &imx568_ctrl_ops, V4L2_CID_PIXEL_RATE,
		SENSOR_PIXEL_CLOCK, SENSOR_PIXEL_CLOCK, 1, SENSOR_PIXEL_CLOCK);
	if (imx568->pixel_rate)
		imx568->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* LINK_FREQ is also read only */
	imx568->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx568_ctrl_ops,
						   V4L2_CID_LINK_FREQ, 0, 0,
						   &imx568->link_freq_value);
	if (imx568->link_freq)
		imx568->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &imx568_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX568_ANA_GAIN_MIN, IMX568_ANA_GAIN_MAX,
			  IMX568_ANA_GAIN_STEP, IMX568_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx568_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX568_DGTL_GAIN_MIN, IMX568_DGTL_GAIN_MAX,
			  IMX568_DGTL_GAIN_STEP, IMX568_DGTL_GAIN_DEFAULT);

	imx568->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx568_ctrl_ops,
					   V4L2_CID_VBLANK, IMX568_VBLANK_MIN,
					   0xffff, 1, IMX568_VBLANK_MIN);
	imx568->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx568_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx568->exposure = v4l2_ctrl_new_std(
		ctrl_hdlr, &imx568_ctrl_ops, V4L2_CID_EXPOSURE,
		IMX568_EXPOSURE_MIN, IMX568_EXPOSURE_MAX, IMX568_EXPOSURE_STEP,
		IMX568_EXPOSURE_DEFAULT);

	imx568->sd.ctrl_handler = ctrl_hdlr;
	// No controls to initialize
	mutex_unlock(&imx568->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx568->mutex);

	return ret;
}

static void imx568_free_controls(struct imx568 *imx568)
{
	v4l2_ctrl_handler_free(imx568->sd.ctrl_handler);
	mutex_destroy(&imx568->mutex);
}

static int imx568_check_hwcfg(struct device *dev)
{
	u64 mbps;
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = { .bus_type =
						       V4L2_MBUS_CSI2_DPHY };
	int ret = -EINVAL;
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *imx568 = to_imx568(sd);

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Number of MIPI CSI2 data lanes */
	switch (ep_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		imx568->mipi_lane_config = ep_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		dev_err(dev, "Invalid number of CSI2 data lanes %d\n",
			ep_cfg.bus.mipi_csi2.num_data_lanes);
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1) {
		dev_err(dev, "expected exactly one link-frequency, got %u\n",
			ep_cfg.nr_of_link_frequencies);
		goto error_out;
	}

	/*
	 * D-PHY v1.2 spans 80-2500 Mbps/lane, and the deserializer's rate
	 * field selects a predefined frequency in 100 Mbps steps, so any
	 * rate in that range is representable at both ends.
	 */
	mbps = div_u64(ep_cfg.link_frequencies[0] * 2, 1000000);
	if (mbps < IMX568_MIN_LANE_RATE_MBPS ||
	    mbps > IMX568_MAX_LANE_RATE_MBPS) {
		dev_err(dev, "Link frequency out of range: %lld Hz (%llu Mbps/lane)\n",
			ep_cfg.link_frequencies[0], mbps);
		goto error_out;
	}
	imx568->link_freq_value = ep_cfg.link_frequencies[0];
	dev_info(dev, "CSI-2 link frequency: %llu Hz (%llu Mbps/lane)\n",
		 imx568->link_freq_value, imx568->link_freq_value * 2 / 1000000);

	/*
	 * Which CSI-2 virtual channel this camera's packets arrive on.
	 * The deserializer decides which VC to stamp (DSER_PIPE_*_CONF's
	 * MAP_DST_* bytes); this property tells the receiver which VC to
	 * accept, via .get_frame_desc. Both must name the same channel.
	 *
	 * Absent means VC0.
	 */
	imx568->vc = 0;
	fwnode_property_read_u32(endpoint, "sima,virtual-channel", &imx568->vc);
	if (imx568->vc > 3) {
		dev_err(dev, "sima,virtual-channel must be 0-3, got %u\n",
			imx568->vc);
		goto error_out;
	}
	dev_info(dev, "CSI-2 virtual channel: %u\n", imx568->vc);

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct of_device_id imx568_dt_ids[] = {
	{ .compatible = "econ,imx568-gmsl" },
	{ /* sentinel */ }
};

// MCU Firmware file read from rootfs - /lib/firmware
static int ecam_firmware_load(struct i2c_client *client)
{
	unsigned char fw_version[32] = { 0 }, bin_fw_version[32] = { 0 };
	int ret = 0;
	unsigned long bin_fw_pos = 0;

	/* Request firmware from the rootfs */
	ret = request_firmware(&cam_fw, cam_fw_name, &client->dev);

	if (ret < 0)
		return -ENOENT;

	bin_fw_pos = cam_fw->size - VERSION_FILE_OFFSET;
	cam_fw_buf = kmalloc(cam_fw->size + 1, GFP_KERNEL);
	cam_fw_buf[cam_fw->size] = '\0';
	memcpy(cam_fw_buf, cam_fw->data, cam_fw->size);

	return ERRCODE_SUCCESS;
}

int cam_bload_ascii2hex(unsigned char ascii)
{
	if (ascii <= '9')
		return (ascii - '0');
	else if ((ascii >= 'a') && (ascii <= 'f'))
		return (0xA + (ascii - 'a'));
	else if ((ascii >= 'A') && (ascii <= 'F'))
		return (0xA + (ascii - 'A'));

	return -1;
}
static int is_fw_update_required(struct i2c_client *client, struct imx568 *priv,
				 unsigned char *fw_version,
				 unsigned char *bin_fw_version)
{
	unsigned char mc_data[512];
	unsigned char mc_ret_data[512];
	uint32_t payload_len = 0, err = 0;
	unsigned long bin_fw_pos = strlen(cam_fw_buf) - VERSION_FILE_OFFSET;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, loop, i = 0;

	/* lock semaphore */
	mutex_lock(&priv->mutex);

	for (loop = bin_fw_pos; loop < (bin_fw_pos + 64); loop = loop + 2) {
		*(bin_fw_version + i) =
			(cam_bload_ascii2hex(cam_fw_buf[loop]) << 4 |
			 cam_bload_ascii2hex(cam_fw_buf[loop + 1]));
		i++;
	}

	/* Check for forced/always update field in the text firmware version */
	if (bin_fw_version[17] == '1') {
#ifdef EN_DEBUG_PRINTS
		dev_info(
			&client->dev,
			"Forced MCU Update Flag Enabled - Firmware Version - (%.32s)\n",
			bin_fw_version);
#endif
		ret = 2;
		goto exit;

	} else {
		/* Query firmware version from MCU */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		cam_write(client, mc_data, TX_LEN_PKT);
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;
		err = cam_write(client, mc_data, 2);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID Write PKT fw Version Error - %d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID Read PKT fw Version Error - %d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify checksum' */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}

		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID fw Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
			ret = -1;
			goto exit;
		}

		/* Read the actual version from MCU */
		payload_len = ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
			      HEADER_FOOTER_SIZE;
		memset(mc_ret_data, 0x00, payload_len);
		ret = cam_read(client, mc_ret_data, payload_len);
		if (ret != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU fw CMD ID Read Version Error - %d\n",
				__func__, __LINE__, ret);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2], 32);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x\n",
				__func__, __LINE__, errcode);
			ret = -1;
			goto exit;
		}

		for (loop = 0; loop < VERSION_SIZE; loop++)
			*(fw_version + loop) = mc_ret_data[2 + loop];

		for (i = 0; i < VERSION_SIZE; i++) {
			if (bin_fw_version[i] != fw_version[i]) {
				dev_info(
					&client->dev,
					"Previous Firmware Version - (%.32s)\n",
					fw_version);
				dev_info(&client->dev,
					 "Current Firmware Version - (%.32s)\n",
					 bin_fw_version);
				ret = 1;
				goto exit;
			}
		}
		ret = ERRCODE_SUCCESS;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mutex);

	return ret;
}
int cam_bload_get_version(struct i2c_client *client)
{
	int ret = 0;
	/*----------------------------- GET VERSION -------------------- */

	/*   Write Get Version CMD */
	g_bload_buf[0] = BL_GET_VERSION;
	g_bload_buf[1] = ~(BL_GET_VERSION);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Write Failed\n", __func__,
			__LINE__);
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	if (g_bload_buf[0] != 'y') {
		/*   NACK Received */
		dev_err(&client->dev, "%s (%d) - NACK Received... exiting..\n",
			__func__, __LINE__);
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	/* ---------------- GET VERSION END ------------------- */

	return 0;
}
int cam_bload_erase_flash(struct i2c_client *client)
{
	unsigned short int pagenum = 0x0000;
	int ret = 0, i = 0, checksum = 0;

	/* --------------- ERASE FLASH --------------------- */

	dev_info(&client->dev, " Erasing camera firmware...\n");

	for (i = 0; i < NUM_ERASE_CYCLES; i++) {
		checksum = 0x00;
		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_ERASE_MEM_NS;
		g_bload_buf[1] = ~(BL_ERASE_MEM_NS);

		ret = cam_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Read Failed\n",
				__func__, __LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev, " NACK Received... exiting..\n");
			return -1;
		}

		g_bload_buf[0] = (MAX_PAGES - 1) >> 8;
		g_bload_buf[1] = (MAX_PAGES - 1) & 0xFF;
		g_bload_buf[2] = g_bload_buf[0] ^ g_bload_buf[1];

		ret = cam_write(client, g_bload_buf, 3);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Read Failed\n",
				__func__, __LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev,
				"%s (%d) - NACK Received... exiting..\n",
				__func__, __LINE__);
			return -1;
		}

		for (pagenum = 0; pagenum < MAX_PAGES; pagenum++) {
			g_bload_buf[(2 * pagenum)] =
				(pagenum + (i * MAX_PAGES)) >> 8;
			g_bload_buf[(2 * pagenum) + 1] =
				(pagenum + (i * MAX_PAGES)) & 0xFF;
			checksum = checksum ^ g_bload_buf[(2 * pagenum)] ^
				   g_bload_buf[(2 * pagenum) + 1];
		}
		g_bload_buf[2 * MAX_PAGES] = checksum;

		ret = cam_write(client, g_bload_buf, (2 * MAX_PAGES) + 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Write Failed\n",
				__func__, __LINE__);
			return -1;
		}

poll_busy:
		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Read Failed\n",
				__func__, __LINE__);
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev,
				"%s (%d) - NACK Received... exiting..\n",
				__func__, __LINE__);
			return -1;
		}
	}

	/* ------------ ERASE FLASH END ----------------------- */

	return 0;
}
unsigned short int cam_bload_calc_crc16(unsigned char *buf, int len)
{
	unsigned short int crc = 0;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++)
		crc ^= buf[i];

	return crc;
}
unsigned char cam_bload_inv_errorcheck(unsigned char *buf, int len)
{
	unsigned int checksum = 0x00;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++)
		checksum = (checksum + buf[i]);

	checksum &= (0xFF);
	return (~(checksum) + 1);
}

int cam_bload_parse_send_cmd(struct i2c_client *client,
			     unsigned char *bytearray, int rec_len,
			     unsigned short int *orig_crc16)
{
	struct ihex_rec *ihex_rec = NULL;
	unsigned char checksum = 0, calc_checksum = 0;
	int i = 0, ret = 0;

	if (!bytearray)
		return -1;

	ihex_rec = (struct ihex_rec *)bytearray;
	ihex_rec->addr = htons(ihex_rec->addr);

	checksum = bytearray[rec_len - 1];

	calc_checksum = cam_bload_inv_errorcheck(bytearray, rec_len - 1);
	if (checksum != calc_checksum) {
		dev_err(&client->dev, " Invalid Checksum 0x%02x != 0x%02x !!\n",
			checksum, calc_checksum);
		return -1;
	}

	/*   TODO: send I2C Commands to Write */
	if ((ihex_rec->rectype == REC_TYPE_ELA) && (ihex_rec->addr == 0x0000) &&
	    (ihex_rec->datasize == 0x02)) {
		/*   Upper 32-bit configuration */
		g_bload_flashaddr = (ihex_rec->recdata[0] << 24) |
				    (ihex_rec->recdata[1] << 16);
	} else if (ihex_rec->rectype == REC_TYPE_DATA) {
		/*   Flash Data into Flashaddr */

		g_bload_flashaddr = (g_bload_flashaddr & 0xFFFF0000) |
				    (ihex_rec->addr);
		*orig_crc16 ^= cam_bload_calc_crc16(ihex_rec->recdata,
						    ihex_rec->datasize);

		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_WRITE_MEM_NS;
		g_bload_buf[1] = ~(BL_WRITE_MEM_NS);

		ret = cam_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Write Failed\n",
				__func__, __LINE__);
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Read Failed\n",
				__func__, __LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev,
				"%s (%d) - NACK Received... exiting..\n",
				__func__, __LINE__);
			return -1;
		}

		g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
		g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
		g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
		g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
		g_bload_buf[4] = g_bload_buf[0] ^ g_bload_buf[1] ^
				 g_bload_buf[2] ^ g_bload_buf[3];

		ret = cam_write(client, g_bload_buf, 5);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Write Failed\n",
				__func__, __LINE__);
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Read Failed\n",
				__func__, __LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev,
				"%s (%d) - NACK Received... exiting..\n",
				__func__, __LINE__);
			return -1;
		}

		g_bload_buf[0] = ihex_rec->datasize - 1;
		checksum = g_bload_buf[0];
		for (i = 0; i < ihex_rec->datasize; i++) {
			g_bload_buf[i + 1] = ihex_rec->recdata[i];
			checksum ^= g_bload_buf[i + 1];
		}

		g_bload_buf[i + 1] = checksum;

		ret = cam_write(client, g_bload_buf, i + 2);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Write Failed\n",
				__func__, __LINE__);
			return -1;
		}

poll_busy:
		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s (%d) - Read Failed\n",
				__func__, __LINE__);
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev,
				"%s (%d) - NACK Received... exiting..\n",
				__func__, __LINE__);
			return -1;
		}

	} else if (ihex_rec->rectype == REC_TYPE_SLA) {
		/*   Update Instruction pointer to this address */

	} else if (ihex_rec->rectype == REC_TYPE_EOF) {
		/*   End of File - Issue I2C Go Command */
	} else {
		/*   Unhandled Type */
		dev_err(&client->dev, "Unhandled Command Type\n");
		return -1;
	}

	return 0;
}
int cam_bload_read_fw(struct i2c_client *client, unsigned short int *orig_crc16)
{
	/* exclude NULL character at end of string */
	unsigned long hex_file_size = strlen(cam_fw_buf) - 1;
	unsigned char wbuf[MAX_BUF_LEN];
	int i = 0, recindex = 0, ret = 0;

	dev_info(&client->dev, "Flashing camera firmware...\n");

	for (i = 0; i < hex_file_size; i++) {
		if ((recindex == 0) && (cam_fw_buf[i] == ':')) {
		} else if (cam_fw_buf[i] == CR) {
		} else if (cam_fw_buf[i] == '"' || cam_fw_buf[i] == '\\' ||
			   cam_fw_buf[i] == 'n') {
		} else if (cam_fw_buf[i] == LF) {
			if (recindex == 0)
				break;

			/*   Analyze Packet and Send Commands */
			ret = cam_bload_parse_send_cmd(client, wbuf, recindex,
						       orig_crc16);
			if (ret < 0) {
				dev_err(&client->dev,
					"Error in Processing Commands\n");
				break;
			}

			recindex = 0;

		} else {
			/*   Parse Rec Data */
			ret = cam_bload_ascii2hex(cam_fw_buf[i]);
			if (ret < 0) {
				dev_err(&client->dev,
					"Invalid Character - 0x%02x !!\n",
					cam_fw_buf[i]);
				break;
			}

			wbuf[recindex] = (0xF0 & (ret << 4));
			i++;

			ret = cam_bload_ascii2hex(cam_fw_buf[i]);
			if (ret < 0) {
				dev_err(&client->dev,
					"Invalid Character - 0x%02x !!!!\n",
					cam_fw_buf[i]);
				break;
			}

			wbuf[recindex] |= (0x0F & ret);
			recindex++;
		}
	}

	/* ------------ PROGRAM FLASH END ----------------------- */

	return ret;
}
int cam_bload_read(struct i2c_client *client, unsigned int g_bload_flashaddr,
		   char *bytearray, unsigned int len)
{
	int ret = 0;

	g_bload_buf[0] = BL_READ_MEM;
	g_bload_buf[1] = ~(BL_READ_MEM);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Write Failed\n", __func__,
			__LINE__);
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, "%s (%d) - NACK Received... exiting..\n",
			__func__, __LINE__);
		return -1;
	}

	g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
	g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
	g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
	g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
	g_bload_buf[4] = g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^
			 g_bload_buf[3];

	ret = cam_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Write Failed\n", __func__,
			__LINE__);
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, "%s (%d) - NACK Received... exiting..\n",
			__func__, __LINE__);
		return -1;
	}

	g_bload_buf[0] = len - 1;
	g_bload_buf[1] = ~(len - 1);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Write Failed\n", __func__,
			__LINE__);
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		dev_err(&client->dev, "%s (%d) - NACK Received... exiting..\n",
			__func__, __LINE__);
		return -1;
	}

	ret = cam_read(client, bytearray, len);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	return 0;
}
int cam_bload_verify_flash(struct i2c_client *client,
			   unsigned short int orig_crc)
{
	char bytearray[FLASH_READ_LEN];
	unsigned short int calc_crc = 0;
	unsigned int flash_addr = FLASH_START_ADDRESS, i = 0;

	while ((i + FLASH_READ_LEN) <= FLASH_SIZE) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (cam_bload_read(client, flash_addr + i, bytearray,
				   FLASH_READ_LEN) < 0) {
			dev_err(&client->dev, " i2c_bload_read FAIL !!\n");
			return -1;
		}

		calc_crc ^= cam_bload_calc_crc16(bytearray, FLASH_READ_LEN);
		i += FLASH_READ_LEN;
	}

	if ((FLASH_SIZE - i) > 0) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (cam_bload_read(client, flash_addr + i, bytearray,
				   (FLASH_SIZE - i)) < 0) {
			dev_err(&client->dev, " i2c_bload_read FAIL !!\n");
			return -1;
		}

		calc_crc ^= cam_bload_calc_crc16(bytearray, FLASH_READ_LEN);
	}

	if (orig_crc != calc_crc) {
		dev_err(&client->dev,
			"CRC verification fail !! 0x%04x != 0x%04x\n", orig_crc,
			calc_crc);
		return -1;
	}

	dev_info(&client->dev, "CRC Verification Success 0x%04x == 0x%04x\n",
		 orig_crc, calc_crc);

	return 0;
}
int cam_bload_go(struct i2c_client *client)
{
	int ret = 0;

	g_bload_buf[0] = BL_GO;
	g_bload_buf[1] = ~(BL_GO);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Write Failed\n", __func__,
			__LINE__);
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	/*   Start Address */
	g_bload_buf[0] = (FLASH_START_ADDRESS & 0xFF000000) >> 24;
	g_bload_buf[1] = (FLASH_START_ADDRESS & 0x00FF0000) >> 16;
	g_bload_buf[2] = (FLASH_START_ADDRESS & 0x0000FF00) >> 8;
	g_bload_buf[3] = (FLASH_START_ADDRESS & 0x000000FF);
	g_bload_buf[4] = g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^
			 g_bload_buf[3];

	ret = cam_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Write Failed\n", __func__,
			__LINE__);
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s (%d) - Read Failed\n", __func__,
			__LINE__);
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, "%s (%d) - NACK Received... exiting..\n",
			__func__, __LINE__);
		return -1;
	}

	return 0;
}

static int cam_fw_update(struct i2c_client *client,
			 unsigned char *cam_fw_version)
{
	int ret = 0;
	unsigned short int bload_crc16 = 0;

	ret = cam_bload_get_version(client);
	if (ret < 0) {
		dev_err(&client->dev, " Error in Get Version\n");
		goto exit;
	}

	/* Erase firmware present in the MCU and flash new firmware*/
	ret = cam_bload_erase_flash(client);
	if (ret < 0) {
		dev_err(&client->dev, " Error in Erase Flash\n");
		goto exit;
	}

	if (cam_bload_read_fw(client, &bload_crc16) < 0) {
		dev_err(&client->dev, " verify_flash FAIL !!\n");
		goto exit;
	}

	if (cam_bload_verify_flash(client, bload_crc16) < 0) {
		dev_err(&client->dev, " verify_flash FAIL !!\n");
		goto exit;
	}

	/* Reverting from bootloader mode */
	if (cam_bload_go(client) < 0) {
		dev_err(&client->dev, " i2c_bload_go FAIL !!\n");
		goto exit;
	}
	dev_info(&client->dev, "(%s) - Firmware Updated - (%.32s)\n", __func__,
		 cam_fw_version);
exit:
	return 0;
}

static int cam_jump_bload(struct i2c_client *client, struct imx568 *priv)
{
	uint32_t payload_len = 0;
	int err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/*lock semaphore */
	mutex_lock(&priv->mutex);

	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FW_UPDT;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(client, mc_data, TX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
			__LINE__, err);
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FW_UPDT;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
			__LINE__, err);
		goto exit;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mutex);
	return err;
}

static int cam_lane_configuration(struct i2c_client *client,
				  struct imx568 *priv)
{
	int ret = 0, err;
	uint16_t payload_data;
	unsigned char mc_data[10];
	uint32_t payload_len = 0;
	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 5;

	/* lock semaphore */
	mutex_lock(&priv->mutex);

	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_LANE_CONFIG;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	/* Second Txn */
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_LANE_CONFIG;

	/* Lane Configuration */
	payload_data = priv->mipi_lane_config == 4 ? NUM_LANES_4 : NUM_LANES_2;
	mc_data[2] = payload_data >> 8;
	mc_data[3] = payload_data & 0xff;

	/* CRC */
	mc_data[4] = errorcheck(&mc_data[2], payload_len);
	err = cam_write(client, mc_data, payload_len + 3);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) MCU Set Ctrl Error - %d\n",
			__func__, __LINE__, err);
		ret = -1;
		goto exit;
	}

	while (--retry > 0) {
		usleep_range(1000, 2000);
		cmd_id = CMD_ID_LANE_CONFIG;
		if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev,
				" %s(%d) MCU Get CMD Status Error\n", __func__,
				__LINE__);
			ret = -1;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_ISP_UNINIT) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_ISP_UNINIT))) {
			dev_err(&client->dev,
				"(%s) %d MCU Get CMD Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			ret = -1;
			goto exit;
		}
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mutex);

	return ret;
}

static int cam_list_ctrls(struct i2c_client *client, struct imx568 *priv,
			  struct isp_ctrl_info *cam_ctrl_info)
{
	/* MCU communication variables */
	unsigned char mc_data[100];
	unsigned char mc_ret_data[100];
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0;
	int ret = 0, err = 0, i;
	int retry = 100;

	/* lock semaphore */
	mutex_lock(&priv->mutex);

	/* Array of Ctrl Info */
	while (--retry > 0) {
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		cam_write(client, mc_data, TX_LEN_PKT);
		usleep_range(1000, 2000);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, 5);
		usleep_range(1000, 2000);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID CTRLS Write Error - %d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);
		usleep_range(1000, 2000);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID List Ctrls Error - %d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID List Ctrls Error CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			priv->num_ctrls = index;
			break;
		}

		payload_len = ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
			      HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID List Ctrls Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
			ret = -1;
			goto exit;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID List Ctrls Read Error - %d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2],
				      payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID List Ctrls Error CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) MCU CMD ID List Ctrls Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
			ret = -1;
			goto exit;
		}
		if (cam_ctrl_info != NULL) {
			/* append ctrl info in array */
			cam_ctrl_info[index].ctrl_id =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 |
				mc_ret_data[4] << 8 | mc_ret_data[5];
			cam_ctrl_info[index].ctrl_type = mc_ret_data[6];
			switch (cam_ctrl_info[index].ctrl_type) {
			case CTRL_STANDARD:
				cam_ctrl_info[index].ctrl_data.std.ctrl_min =
					mc_ret_data[7] << 24 |
					mc_ret_data[8] << 16 |
					mc_ret_data[9] << 8 | mc_ret_data[10];

				cam_ctrl_info[index].ctrl_data.std.ctrl_max =
					mc_ret_data[11] << 24 |
					mc_ret_data[12] << 16 |
					mc_ret_data[13] << 8 | mc_ret_data[14];

				cam_ctrl_info[index].ctrl_data.std.ctrl_def =
					mc_ret_data[15] << 24 |
					mc_ret_data[16] << 16 |
					mc_ret_data[17] << 8 | mc_ret_data[18];

				cam_ctrl_info[index].ctrl_data.std.ctrl_step =
					mc_ret_data[19] << 24 |
					mc_ret_data[20] << 16 |
					mc_ret_data[21] << 8 | mc_ret_data[22];
				break;

			case CTRL_EXTENDED:
				cam_ctrl_info[index].ctrl_data.ext.val_type =
					mc_ret_data[7];
				cam_ctrl_info[index].ctrl_data.ext.val_length =
					mc_ret_data[8] << 24 |
					mc_ret_data[9] << 16 |
					mc_ret_data[10] << 8 | mc_ret_data[11];
				for (i = 0;
				     i < cam_ctrl_info[index]
						 .ctrl_data.ext.val_length;
				     i++)
					cam_ctrl_info[index]
						.ctrl_data.ext.val_data[i] =
						mc_ret_data[12 + i];
				if (cam_ctrl_info[index].ctrl_data.ext.val_type ==
				    V4L2_CTRL_TYPE_INTEGER64) {
					for (i = 0; i < EXTENDED_CTRL_SIZE;
					     i++) {
						cam_ctrl_info[index]
							.ctrl_data.ext
							.ctrl_min |=
							cam_ctrl_info[index]
								.ctrl_data.ext
								.val_data[i]
							<< 8 * (7 - i);
						cam_ctrl_info[index]
							.ctrl_data.ext
							.ctrl_max |=
							cam_ctrl_info[index]
								.ctrl_data.ext
								.val_data[8 + i]
							<< 8 * (7 - i);
						cam_ctrl_info[index]
							.ctrl_data.ext
							.ctrl_def |=
							cam_ctrl_info[index]
								.ctrl_data.ext
								.val_data[16 + i]
							<< 8 * (7 - i);
						cam_ctrl_info[index]
							.ctrl_data.ext
							.ctrl_step |=
							cam_ctrl_info[index]
								.ctrl_data.ext
								.val_data[24 + i]
							<< 8 * (7 - i);
					}
				} else if (cam_ctrl_info[index]
						   .ctrl_data.ext.val_type ==
					   V4L2_CTRL_TYPE_STRING) {
					for (i = 0; i < EXTENDED_CTRL_SIZE;
					     i++) {
						cam_ctrl_info[index]
							.ctrl_data.ext
							.ctrl_min |=
							cam_ctrl_info[index]
								.ctrl_data.ext
								.val_data[i]
							<< 8 * (7 - i);
						cam_ctrl_info[index]
							.ctrl_data.ext
							.ctrl_max |=
							cam_ctrl_info[index]
								.ctrl_data.ext
								.val_data[8 + i]
							<< 8 * (7 - i);
						cam_ctrl_info[index]
							.ctrl_data.ext
							.ctrl_step |=
							cam_ctrl_info[index]
								.ctrl_data.ext
								.val_data[24 + i]
							<< 8 * (7 - i);
					}
				}
				break;
			}
			priv->ctrldb[index] = cam_ctrl_info[index].ctrl_id;
		}
		index++;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mutex);

	return ret;
}
static int cam_list_fmts(struct i2c_client *client, struct imx568 *priv,
			 struct isp_stream_info *stream_info, int *frm_fmt_size)
{
	/* MCU communication variables */
	unsigned char mc_data[100];
	unsigned char mc_ret_data[100];
	uint32_t payload_len = 0, err = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0, mode = 0;

	int num_frates = 0, ret = 0, default_fmt_fourcc = 0;

	/* Stream Info Variables */

	/* lock semaphore */
	mutex_lock(&priv->mutex);

	/* List all formats from MCU and append to cam_frmfmt array */

	for (index = 0;; index++) {
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		cam_write(client, mc_data, TX_LEN_PKT);
		usleep_range(1000, 2000);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, 5);
		usleep_range(1000, 2000);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) i2c error while writing command to MCU -%d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);
		usleep_range(1000, 2000);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) i2c error while reading stream info. length from MCU - %d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) Stream info length CRC mismatch: 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}
		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			if (stream_info == NULL) {
				*frm_fmt_size = index;
				priv->frm_fmt_size = index;
			} else {
				*frm_fmt_size = mode;
				priv->frm_fmt_size = mode;
			}
			break;
		}

		payload_len = ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
			      HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) MCU's return code has error set - 0x%02x\n",
				__func__, __LINE__, errcode);
			ret = -1;
			goto exit;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		usleep_range(1000, 2000);
		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) i2c error while reading actual stream info. - %d\n",
				__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2],
				      payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) Stream info CRC mismatch: 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) MCU's response has errcode set - 0x%02x\n",
				__func__, __LINE__, errcode);
			ret = -1;
			goto exit;
		}
		if (stream_info != NULL) {
			/*
			 * check if any other format than UYVY is queried -
			 * do not append in array
			 */
			stream_info->fmt_fourcc =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 |
				mc_ret_data[4] << 8 | mc_ret_data[5];
			if (index == 0)
				default_fmt_fourcc = stream_info->fmt_fourcc;
			stream_info->width = mc_ret_data[6] << 8 |
					     mc_ret_data[7];
			stream_info->height = mc_ret_data[8] << 8 |
					      mc_ret_data[9];
			stream_info->frame_rate_type = mc_ret_data[10];

			switch (stream_info->frame_rate_type) {
			case FRAME_RATE_DISCRETE:
				stream_info->frame_rate.disc.frame_rate_num =
					mc_ret_data[11] << 8 | mc_ret_data[12];

				stream_info->frame_rate.disc.frame_rate_denom =
					mc_ret_data[13] << 8 | mc_ret_data[14];

				break;

			case FRAME_RATE_CONTINOUS:
				dev_err(&client->dev,
					" Stream format at index 0x%04x has unsupported FRAME_RATE_CONTINOUS\n",
					index);

				continue;
			}
			switch (stream_info->fmt_fourcc) {
			case V4L2_PIX_FMT_SRGGB12:
			case V4L2_PIX_FMT_SGBRG12:
			case V4L2_PIX_FMT_SGBRG10:
			case V4L2_PIX_FMT_SRGGB10:
			case V4L2_PIX_FMT_SRGGB8:
				priv->cam_frmfmt[mode].size.width =
					stream_info->width;
				priv->cam_frmfmt[mode].size.height =
					stream_info->height;
				num_frates =
					priv->cam_frmfmt[mode].num_framerates;

				*((int *)(priv->cam_frmfmt[mode].framerates) +
				  num_frates) =
					(int)(stream_info->frame_rate.disc
						      .frame_rate_num /
					      stream_info->frame_rate.disc
						      .frame_rate_denom);
				priv->cam_frmfmt[mode].num_framerates++;
				priv->cam_frmfmt[mode].mode = mode;
				priv->streamdb[index] = mode;
				priv->cam_frmfmt[mode].fourcc =
					stream_info->fmt_fourcc;
#ifdef EN_DEBUG_PRINTS
				dev_info(
					&client->dev,
					"stream mode : %d width : %d height : %d framerate : %d fourcc %c%c%c%c (%u bpp)\n",
					priv->cam_frmfmt[mode].mode,
					stream_info->width, stream_info->height,
					priv->cam_frmfmt[mode].num_framerates,
					stream_info->fmt_fourcc & 0xff,
					(stream_info->fmt_fourcc >> 8) & 0xff,
					(stream_info->fmt_fourcc >> 16) & 0xff,
					(stream_info->fmt_fourcc >> 24) & 0xff,
					(stream_info->fmt_fourcc == V4L2_PIX_FMT_SRGGB8) ? 8 :
					((stream_info->fmt_fourcc == V4L2_PIX_FMT_SRGGB10 ||
					  stream_info->fmt_fourcc == V4L2_PIX_FMT_SGBRG10) ? 10 : 12));
				dev_info(
					&client->dev,
					"stream_info->frame_rate.disc.frame_rate_num = %d --------------\n",
					stream_info->frame_rate.disc
						.frame_rate_num);
#endif

				mode++;
				break;

			default:
				dev_err(&client->dev,
					" Stream format index 0x%04x has unsupported format 0x%08x (supported 0x%08x, 0x%08x)\n",
					index, stream_info->fmt_fourcc,
					V4L2_PIX_FMT_SRGGB12,
					V4L2_PIX_FMT_SRGGB10);
			}
		}
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mutex);

	return ret;
}

int serdes_write_i2c(struct i2c_client *client, u16 sladdr, u8 *val, u32 count)
{
	int ret;

	struct i2c_msg msg = {
		.addr = sladdr,
		.flags = 0,
		.len = count,
		.buf = val,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register ret = %d!\n",
			ret);
		return ret;
	}
	return 0;
}

int serdes_read_i2c(struct i2c_client *client, u16 sladdr, u8 *val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = sladdr,
		.flags = 0,
		.buf = val,
	};

	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	return 0;

err:
	dev_err(&client->dev, "Failed reading register ret = %d!\n", ret);
	return ret;
}

s32 serdes_write_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val)
{
	u8 bcount = 3;
	u8 au8Buf[3] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	dev_dbg(&client->dev,
    	"serdes_write_16b_reg: client=0x%02x sladdr=0x%02x reg=0x%04x val=0x%02x\n",
    	client->addr,
    	sladdr,
    	reg,
    	val);

	if (serdes_write_i2c(client, sladdr, au8Buf, bcount) < 0) {
		dev_err(&client->dev,
			"%s:write reg error: reg = 0x%x,val = 0x%x\n", __func__,
			reg, val);
		return -EIO;
	}

	return 0;
}

s32 serdes_read_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 *val)
{
	u8 bcount;
	u8 au8RegBuf[2] = { 0 };
	u8 au8RdVal[1] = { 0 };

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;
	bcount = 2;

	dev_dbg(&client->dev,
    	"serdes_read_16b_reg: client=0x%02x sladdr=0x%02x reg=0x%04x val=0x%02x\n",
    	client->addr,
    	sladdr,
    	reg,
    	val);

	if (serdes_write_i2c(client, sladdr, au8RegBuf, bcount) < 0) {
		dev_err(&client->dev, "%s:write reg error:reg=0x%x\n", __func__,
			reg);
		return -EIO;
	}

	bcount = 1;
	if (serdes_read_i2c(client, sladdr, au8RdVal, bcount) < 0) {
		dev_err(&client->dev, "%s:read reg error:reg=0x%x\n", __func__,
			reg);
		return -EIO;
	}

	*val = au8RdVal[0];

	return 0;
}
static s32 serdes_parse_regdata(struct i2c_client *client,
				struct serdes_parse *regdata, u32 reg_cnt,
				u8 serdes_id)
{
	int i = 0;

	for (i = 0; i < reg_cnt; i++) {
		if (regdata[i].reg == 0xFFFF) {
			mdelay(100);
			continue;
		}

		if ((serdes_write_16b_reg(client, serdes_id, regdata[i].reg,
					  regdata[i].val)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__,
				__LINE__);
			return -EIO;
		}
	}

	return 0;
}

/*
 * Read the CSI-2 output geometry the board asks for straight from this
 * camera's endpoint. These are the same two properties that configure the SoC
 * receiver, so deriving the deserializer's side from them is what keeps the
 * two ends of the link in step. D-PHY is DDR, hence the doubling.
 */
static int dser_dphy_params_from_dt(struct i2c_client *client,
				    unsigned int *lanes, unsigned int *mbps)
{
	struct v4l2_fwnode_endpoint ep_cfg = { .bus_type =
						       V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	u64 rate;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!endpoint)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	ret = -EINVAL;
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2 &&
	    ep_cfg.bus.mipi_csi2.num_data_lanes != 4)
		goto out;
	if (ep_cfg.nr_of_link_frequencies != 1)
		goto out;

	rate = div_u64(ep_cfg.link_frequencies[0] * 2, 1000000);
	if (rate < IMX568_MIN_LANE_RATE_MBPS ||
	    rate > IMX568_MAX_LANE_RATE_MBPS) {
		dev_err(&client->dev,
			"Link frequency out of range: %lld Hz (%llu Mbps/lane)\n",
			ep_cfg.link_frequencies[0], rate);
		goto out;
	}

	*lanes = ep_cfg.bus.mipi_csi2.num_data_lanes;
	*mbps = rate;
	ret = 0;
out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	return ret;
}

/*
 * Program the deserializer's CSI-2 output for the lane count and lane rate the
 * device tree asks for. Both are derived from the same endpoint properties that
 * configure the SoC receiver - data-lanes and link-frequencies - so the two ends
 * of the link cannot be configured to disagree.
 *
 * Above 1.5 Gbps/lane the MIPI D-PHY v1.2 spec changes what is required, and
 * both vendors say so: the deserializer datasheet makes interlane deskew
 * mandatory at that rate and above, and the Synopsys D-PHY databook has the
 * receiver arm its deskew calibration automatically there and wait for a
 * pattern only the transmitter can send. Parameters counted in unit intervals
 * or byte clocks also lose a third of their duration going 1188 -> 1800 Mbps,
 * so several that are comfortable below the threshold go marginal together
 * above it. Everything conditional on high_rate below is that set.
 *
 * At or below 1.5 Gbps the register values are left exactly as the long-proven
 * four-lane configuration had them, so enabling this path cannot perturb it.
 */
static int dser_configure_dphy(struct i2c_client *client, unsigned int lanes,
			       unsigned int mbps)
{
	/* CSI2_LANE_CNT: 0xC0 = 4 lanes, 0x50 = 2 lanes */
	const u8 lane_cnt = (lanes == 4) ? 0xC0 : 0x50;
	/*
	 * BACKTOP22/25/28/31: bit5 = use predefined frequency, bits[4:0] select
	 * it in 100 Mbps steps. Round UP to the next step: the DT's 594 MHz is
	 * 1188 Mbps, and the part must be told 1200 (0x2C), not 1100 - the
	 * receiver's hsfreqrange bucket for 1188 does not reach down to 1100.
	 */
	const u8 rate = 0x20 | DIV_ROUND_UP(mbps, 100);
	const bool high_rate = mbps > 1500;
	static const u16 lane_cnt_reg[] = { 0x040A, 0x044A, 0x048A, 0x04CA };
	static const u16 dpll[] = { 0x1C00, 0x1D00, 0x1E00, 0x1F00 };
	static const u16 rate_reg[] = { 0x031D, 0x0320, 0x0323, 0x0326 };
	int i;

	for (i = 0; i < ARRAY_SIZE(lane_cnt_reg); i++)
		if (serdes_write_16b_reg(client, DES_ADDR, lane_cnt_reg[i],
					 lane_cnt) < 0)
			return -EIO;

	/*
	 * D-PHY transmit timing. The POR values sit only 6-9 ns above the MIPI
	 * minimums once the UI halves, and t_clk_przero is 306 ns against a
	 * 300 ns floor. These fields have minimums but no maximums, so widen
	 * them for the high-rate case only.
	 *   0x0331 t_hs_przero / t_hs_prep / t_clk_przero
	 *   0x0332 phy_Stdby_n 0b1100 (PHY2+PHY3) | t_lpx | t_hs_trail
	 *   0x0335 t_clk_prep
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0331,
				 high_rate ? 0xD1 : 0x00) < 0 ||
	    serdes_write_16b_reg(client, DES_ADDR, 0x0332,
				 high_rate ? 0xC5 : 0xC4) < 0 ||
	    serdes_write_16b_reg(client, DES_ADDR, 0x0335,
				 high_rate ? 0x40 : 0x00) < 0)
		return -EIO;

	/*
	 * The datasheet requires a PLL be held in soft reset across a change to
	 * its own predef_freq register (DPLL_0 bit0 = config_soft_rst_n, active
	 * low). Reset all four: in 2x4 mode Port B is PHY2+PHY3, and PHY3's PLL
	 * being left out is what stopped lanes 2-3 coming up at 1800 Mbps.
	 */
	for (i = 0; i < ARRAY_SIZE(dpll); i++)
		if (serdes_write_16b_reg(client, DES_ADDR, dpll[i], 0xF4) < 0)
			return -EIO;

	for (i = 0; i < ARRAY_SIZE(rate_reg); i++)
		if (serdes_write_16b_reg(client, DES_ADDR, rate_reg[i], rate) < 0)
			return -EIO;

	for (i = ARRAY_SIZE(dpll) - 1; i >= 0; i--)
		if (serdes_write_16b_reg(client, DES_ADDR, dpll[i], 0xF5) < 0)
			return -EIO;

	/*
	 * BACKTOP30 (0x0325) must come after the PLL sequence above.
	 * Bits[3:0] are PHY3's PLL fractional feedback, so holding that
	 * PLL in reset afterwards disturbs it. Bit 7 is BACKTOP_W_FRAME:
	 * the line buffer waits for a frame start before accepting data.
	 * Without it the first frame after stream-on is corrupt.
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0325, 0x80) < 0)
		return -EIO;

	/*
	 * Lane deskew on Port B's controller (MIPI_TX2). DESKEW_INIT bit7 is
	 * the auto-initial enable and DESKEW_PER bit7 the periodic enable;
	 * 0x00 is a reserved width encoding, not "off". Mandatory above
	 * 1.5 Gbps, optional below - left off there, as the working four-lane
	 * configuration has it.
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0483,
				 high_rate ? 0x87 : 0x00) < 0 ||
	    serdes_write_16b_reg(client, DES_ADDR, 0x0484,
				 high_rate ? 0x8F : 0x00) < 0)
		return -EIO;

	/*
	 * CSI2_TX_GAP counts MIPI byte clocks, so the POR 0x1C is 187 ns at
	 * 1188 Mbps but only 124 ns at 1800. Restore the settling time.
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0487,
				 high_rate ? 0x38 : 0x1C) < 0)
		return -EIO;

	dev_info(&client->dev,
		 "DESER CSI-2 output: %u lanes @ %u Mbps/lane (deskew %s)\n",
		 lanes, mbps, high_rate ? "on" : "off");

	return 0;
}

// SerDes - Configuration
//
// Worker: does the actual bring-up. Call serdes_config_init() instead -
// this must run exactly once per boot, not once per camera probe.
static s32 __serdes_config_init(struct i2c_client *client)
{
	uint8_t slave_addr = 0;
	bool link_a_present = false;
	bool link_b_present = false;

	dev_info(&client->dev, " Issuing CHIP reset for Deserializer ...\n");
	if ((serdes_write_16b_reg(client, DES_ADDR, 0x0010, 0x80)) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);

	if ((serdes_write_16b_reg(client, DES_ADDR, 0x000C, 0x15)) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);

	/*
	 * CTRL0 (0x0010): was 0x01 (LINK_CFG=01 only), which left
	 * AUTO_LINK (bit4) and REG_ENABLE (bit2) both cleared despite the
	 * "REG ENABLE" comment's intent. Per the MAX96716A/F user guide's
	 * Table 8 ("Single vs. Dual Link Operation"), AUTO_LINK must be 1
	 * for every link mode (single A, single B, and reverse splitter),
	 * not just dual-link - so clearing it here likely breaks the
	 * manual LINK_EN_A/B (0x0F00) link selection entirely, regardless
	 * of Pipe Z/VC config. REG_ENABLE (LDO) should also be on per the
	 * datasheet ("LDO should always be used unless VDD is 1.05V or
	 * lower") - this board's deserializer VDD rails are
	 * +V1.2_MAX96716/+V1.8_MAX96716 (HHHL_Schematics.pdf), both above
	 * that threshold. Fixed value: REG_ENABLE(0x04) | AUTO_LINK(0x10)
	 * | LINK_CFG=01(0x01) = 0x15.
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0010, 0x15) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);
	/* REGULATOR MANUAL1 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0012, 0x14) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);

	/* TEST: enable I2C pass-through channels 1 & 2, per MAX96716A
	 * user guide REG1 (0x0001) IIC_1_EN/IIC_2_EN - needed to reach
	 * peripherals (camera MCUs) behind each serializer, distinct
	 * from the primary interface used for the deserializer's own
	 * register access. Testing whether this resolves MCU
	 * unreachability. */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0001, 0x03) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);
	
	{
		uint8_t reg1_readback = 0;

		if (serdes_read_16b_reg(client, DES_ADDR, 0x0001, &reg1_readback) < 0) {
			dev_err(&client->dev, "%s(%d): REG1 readback failed\n", __func__, __LINE__);
		} else {
			dev_info(&client->dev, "REG1 (0x0001) readback = 0x%02x (expect 0x03)\n",
				 reg1_readback);
		}
	}

	/*
	 * DUAL-CAMERA BRING-UP, phase 1 of 3: LINK A ALONE.
	 *
	 * Both serializers are the identical part and boot at
	 * SER_NATIVE_ADDR, so with both links up an I2C write reaches both
	 * and a read returns the wire-AND of two devices. The only window in
	 * which exactly one of them is addressable is while its own link is
	 * the only one enabled - so serializer A's own address is rewritten
	 * here, before link B is ever brought up.
	 *
	 * CTRL0 values follow upstream max9296a_select_links()
	 * (drivers/media/i2c/maxim-serdes/max9296a.c, which carries
	 * max96716a_info with .num_links = 2 - this exact part):
	 * LINK_CFG bits[1:0] are a BITMASK of links (0b01 = A, 0b10 = B,
	 * 0b11 = both), AUTO_LINK (bit4) must stay CLEARED or the part
	 * auto-selects a single link, and RESET_ONESHOT (bit5) must be set
	 * in the same write with ~200 ms to retrain.
	 *
	 * Everything in this phase is BEST EFFORT: with no camera on link A
	 * the remote I2C fails with -EREMOTEIO, which must not break the
	 * working link B path. Failures are logged and skipped.
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0F00, 0x01) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);
	/* REG_ENABLE(0x04) | LINK_CFG=0b01 (link A) | RESET_ONESHOT(0x20) */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0010, 0x25) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(200);

	/*
	 * NORMALISE SERIALIZER A'S ADDRESS BEFORE PROBING FOR IT.
	 *
	 * A MAX9295A keeps its register state across a warm reboot and
	 * across a module reload - only removing power clears it. So on a
	 * cold boot serializer A answers at SER_NATIVE_ADDR and the rewrite
	 * below moves it to SER_A_ADDR, but on EVERY later bring-up without
	 * a power cycle it is already at SER_A_ADDR and nothing answers at
	 * the factory address.
	 *
	 * Probing only SER_NATIVE_ADDR therefore made a plain reboot look
	 * exactly like an unplugged camera: "no serializer at 0x40", link A
	 * skipped, VIDEO_PIPE_EN = 0x02, and Pipe Y never enabled. Camera A
	 * still probed and enumerated normally - its MCU alias survives the
	 * reboot too - so the failure only showed up as no video, which is
	 * easy to mistake for a pipe or serdes misconfiguration.
	 *
	 * If it is found at SER_A_ADDR, put it back to the factory address
	 * and let the normal path below run unchanged. Restoring rather
	 * than short-circuiting keeps a single bring-up sequence: the
	 * serializer is configured identically whether or not this ran.
	 */
	if (serdes_read_16b_reg(client, SER_NATIVE_ADDR, 0x0000, &slave_addr) < 0) {
		if (serdes_read_16b_reg(client, SER_A_ADDR, 0x0000, &slave_addr) == 0 &&
		    (slave_addr >> 1) == SER_A_ADDR) {
			dev_info(&client->dev,
				 "link A: serializer still at 0x%02x from an earlier load - restoring 0x%02x\n",
				 SER_A_ADDR, SER_NATIVE_ADDR);
			if (serdes_write_16b_reg(client, SER_A_ADDR, 0x0000,
						 SER_NATIVE_ADDR << 1) < 0)
				dev_err(&client->dev,
					"link A: could not restore DEV_ADDR to 0x%02x\n",
					SER_NATIVE_ADDR);
			else
				msleep(20);
		}
	}

	if (serdes_read_16b_reg(client, SER_NATIVE_ADDR, 0x0000, &slave_addr) < 0) {
		dev_info(&client->dev,
			 "link A: no serializer at 0x%02x or 0x%02x - skipping camera A bring-up\n",
			 SER_NATIVE_ADDR, SER_A_ADDR);
	} else {
		dev_info(&client->dev, "link A: serializer found, DEV_ADDR = 0x%02x\n",
			 slave_addr >> 1);

		/*
		 * Move serializer A off the shared factory address. DEV_ADDR
		 * (MAX9295A reg 0x0000) holds the 7-bit address in bits[7:1],
		 * so the value written is the address << 1. Every later access
		 * to this serializer must use SER_A_ADDR.
		 */
		if (serdes_write_16b_reg(client, SER_NATIVE_ADDR, 0x0000,
					 SER_A_ADDR << 1) < 0) {
			dev_err(&client->dev,
				"link A: DEV_ADDR rewrite to 0x%02x failed\n", SER_A_ADDR);
		} else {
			msleep(20);
			if (serdes_read_16b_reg(client, SER_A_ADDR, 0x0000, &slave_addr) < 0 ||
			    (slave_addr >> 1) != SER_A_ADDR) {
				dev_err(&client->dev,
					"link A: serializer did not answer at 0x%02x after rewrite\n",
					SER_A_ADDR);
			} else {
				dev_info(&client->dev,
					 "link A: serializer moved to 0x%02x\n", SER_A_ADDR);

				/*
				 * Both camera MCUs are natively 0x42.
				 * SER1_I2C_CONF gives serializer A the slot
				 * 0x44 -> 0x42, SER2_I2C_CONF gives serializer
				 * B 0x43 -> 0x42, so the two do not collide.
				 */
				if (serdes_parse_regdata(client, SER1_I2C_CONF,
							 ARRAY_SIZE(SER1_I2C_CONF),
							 SER_A_ADDR) < 0)
					dev_err(&client->dev,
						"link A: I2C translation setup failed\n");
				else
					dev_info(&client->dev,
						 "link A: camera MCU aliased to 0x%02x\n",
						 CAM_A_ALIAS);

				/*
				 * ROOT CAUSE (camera A produced no video):
				 * this block was missing from link A
				 * entirely. It is present in BOTH the
				 * known-working single-camera flow (git
				 * a3a79742351) and the working link B path
				 * below - link A was the only path without
				 * it, and camera A was the only camera that
				 * never streamed.
				 *
				 * These are serializer GPIO/pin-mode writes
				 * that configure the camera module itself,
				 * not the video pipe:
				 *   0x02CD = 0x40  boot pin driven low
				 *   0x02D3 = 0xC4  high-priority GPIO
				 *                  reception for the input
				 *                  trigger
				 *   0x02D6 = 0x40  camera to MASTER_MODE
				 *
				 * MASTER_MODE is the one that matters most:
				 * without it the module never free-runs, so
				 * the MCU still answers I2C and the sensor
				 * ID still reads back (probe succeeds) while
				 * no frame is ever emitted. That is why
				 * every Pipe Y register experiment was
				 * inert - the pipe was correct, its INPUT
				 * was empty. Pipe Y demonstrably forwards
				 * video the moment it is pointed at a link
				 * that carries any.
				 *
				 * Best effort, like the rest of phase 1: a
				 * failure here must not take down link B.
				 */
				if (serdes_write_16b_reg(client, SER_A_ADDR, 0x02CD, 0x40) < 0)
					dev_err(&client->dev,
						"link A: boot pin config failed\n");
				if (serdes_write_16b_reg(client, SER_A_ADDR, 0x02D3, 0xC4) < 0)
					dev_err(&client->dev,
						"link A: GPIO trigger config failed\n");
				if (serdes_write_16b_reg(client, SER_A_ADDR, 0x02D6, 0x40) < 0)
					dev_err(&client->dev,
						"link A: MASTER_MODE config failed\n");

				/*
				 * MCU reset, for exact parity with link B.
				 * cam_core_initialize() repeats this later
				 * against priv->ser_addr, but link B is also
				 * reset twice and works, so parity with the
				 * proven sequence is preferred over pruning.
				 */
				if (serdes_write_16b_reg(client, SER_A_ADDR, 0x02CA, 0x40) < 0)
					dev_err(&client->dev,
						"link A: MCU reset step 1 failed\n");
				usleep_range(10000, 11000);
				if (serdes_write_16b_reg(client, SER_A_ADDR, 0x02BE, 0x40) < 0)
					dev_err(&client->dev,
						"link A: MCU reset step 2 failed\n");
				usleep_range(10000, 11000);
				if (serdes_write_16b_reg(client, SER_A_ADDR, 0x02CA, 0x98) < 0)
					dev_err(&client->dev,
						"link A: MCU reset step 3 failed\n");
				usleep_range(10000, 11000);

				if (serdes_parse_regdata(client, SER1_CONF,
							 ARRAY_SIZE(SER1_CONF),
							 SER_A_ADDR) < 0)
					dev_err(&client->dev,
						"link A: serializer config failed\n");
				else {
					dev_info(&client->dev,
						 "link A: serializer configured\n");
					link_a_present = true;
				}
			}
		}
	}

	/*
	 * Phase 2 of 3: LINK B ALONE. Serializer B is still at the factory
	 * address and is now the only device answering there, since
	 * serializer A has been moved to SER_A_ADDR. The existing link B
	 * sequence below therefore keeps working against SER_NATIVE_ADDR
	 * unchanged.
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0F00, 0x02) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);
	/* REG_ENABLE(0x04) | LINK_CFG=0b10 (link B) | RESET_ONESHOT(0x20) */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0010, 0x26) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(200);


	/*
	 * First contact must use the shared native address, as SER1's own
	 * sequence above does - both serializers are the identical chip and
	 * unconfigured at this point, so both answer only at SER_NATIVE_ADDR
	 * until individually reprogrammed below. Addressing a not-yet-assigned
	 * address here returns -EREMOTEIO.
	 */
	/*
	 * BEST EFFORT, exactly like link A above. Each link is independent:
	 * a board may have a camera on link A only, link B only, or both,
	 * and this function must configure whatever is actually present.
	 * Only the case where NEITHER link came up is fatal - see the check
	 * after the link B block.
	 */
	if (serdes_write_16b_reg(client, SER_NATIVE_ADDR, 0x0010, 0x21) < 0) {
		dev_info(&client->dev,
			 "link B: no serializer at 0x%02x - skipping camera B bring-up\n",
			 SER_NATIVE_ADDR);
		goto link_b_done;
	}

	/*
	 * MANDATORY. 0x0010 = 0x21 carries RESET_ONESHOT (bit5), which
	 * resets the serializer and drops its link; reading it back before
	 * the link has retrained returns -EREMOTEIO and the device looks
	 * absent. Measured: an 11 ms gap fails, 100 ms works.
	 */
	msleep(100);

	if (serdes_read_16b_reg(client, SER_NATIVE_ADDR, 0x0000, &slave_addr) < 0 ||
	    SER_NATIVE_ADDR != (slave_addr >> 1)) {
		dev_info(&client->dev,
			 "link B: no serializer at 0x%02x - skipping camera B bring-up\n",
			 SER_NATIVE_ADDR);
		goto link_b_done;
	}
	link_b_present = true;

	dev_info(&client->dev, "link B: serializer found, DEV_ADDR = 0x%02x\n",
		 SER1_ADDR);

	/*
	 * NOT a chip-address change - the serializer itself keeps
	 * responding at SER1_ADDR/0x40, and every write below targets it.
	 * This programs I2C_4/I2C_5, the downstream-device
	 * translator for the CAMERA MCU behind the serializer (see the
	 * comment above SER2_I2C_CONF in econ_imx568_serdes.h), so camera 2's
	 * MCU answers at 0x43 instead of its native 0x42 and does not collide
	 * with camera 1's MCU once Link A is also up. The camera enumerates
	 * as econ-imx568-gmsl 0-0043.
	 */
	if (serdes_parse_regdata(client, SER2_I2C_CONF,
				 ARRAY_SIZE(SER2_I2C_CONF), SER1_ADDR) < 0) {
		dev_err(&client->dev,
			"%s: Failed to configure SIOB Serializer I2C translation\n",
			__func__);
		return -EIO;
	}
	dev_info(&client->dev, "SIOB Port I2C translated successfully\n");

	if (serdes_write_16b_reg(client, SER1_ADDR, 0x02CD, 0x40) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	if (serdes_write_16b_reg(client, SER1_ADDR, 0x02D3, 0xC4) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	if (serdes_write_16b_reg(client, SER1_ADDR, 0x02D6, 0x40) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}

	if (serdes_write_16b_reg(client, SER1_ADDR, 0x02CA, 0x40) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	usleep_range(10000, 11000);
	if (serdes_write_16b_reg(client, SER1_ADDR, 0x02BE, 0x40) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	usleep_range(10000, 11000);
	if (serdes_write_16b_reg(client, SER1_ADDR, 0x02CA, 0x98) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	usleep_range(10000, 11000);

	if (serdes_parse_regdata(client, SER2_CONF, ARRAY_SIZE(SER2_CONF),
				 SER1_ADDR) < 0) {
		dev_err(&client->dev,
			"%s: Failed to configure SIOB Serializer\n", __func__);
		return -EIO;
	}
	dev_info(&client->dev, "configuring SIOB serializer successful\n");

link_b_done:
	if (!link_a_present && !link_b_present) {
		dev_err(&client->dev,
			"no serializer found on either link - no camera connected?\n");
		return -ENODEV;
	}

	/*
	 * Phase 3 of 3: BOTH LINKS. Serializer A is at SER_A_ADDR and
	 * serializer B at SER_B_ADDR, and the two camera MCUs answer at
	 * CAM_A_ALIAS / CAM_B_ALIAS, so nothing collides any more. All
	 * deserializer pipe programming below runs with both links up.
	 */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0F00, 0x03) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(100);
	/* REG_ENABLE(0x04) | LINK_CFG=0b11 (both) | RESET_ONESHOT(0x20);
	 * AUTO_LINK(0x10) deliberately NOT set - it forces single-link
	 * auto-select, which is what kept only one link alive before. */
	if (serdes_write_16b_reg(client, DES_ADDR, 0x0010, 0x27) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(200);

	if (serdes_parse_regdata(client, DSER_PIPE_Z_SHARED_CONF,
				 ARRAY_SIZE(DSER_PIPE_Z_SHARED_CONF), DES_ADDR) < 0) {
		dev_err(&client->dev,
			"%s: Failed to configure DESER shared D-PHY/CSI-output regs\n",
			__func__);
		return -EIO;
	}
	dev_info(&client->dev, "configuring DESER shared D-PHY/CSI-output regs successful\n");

	{
		unsigned int lanes, mbps;

		if (dser_dphy_params_from_dt(client, &lanes, &mbps) < 0) {
			dev_err(&client->dev,
				"%s: no usable data-lanes/link-frequencies in DT\n",
				__func__);
			return -EIO;
		}
		if (dser_configure_dphy(client, lanes, mbps) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure DESER CSI-2 output\n",
				__func__);
			return -EIO;
		}
	}


	/* Pipe Z carries link B's video (VC1). Only program it when link B
	 * actually has a serializer. */
	if (link_b_present) {
		if (serdes_parse_regdata(client, DSER_PIPE_Z_CONF,
					 ARRAY_SIZE(DSER_PIPE_Z_CONF), DES_ADDR) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure DESER Pipe Z\n", __func__);
			return -EIO;
		}
		dev_info(&client->dev, "configuring Pipe Z successful\n");
	}

	/*
	 * Pipe Y - link A's VC0 path. Only programmed when link A actually
	 * answered during phase 1, so a link-B-only setup is left exactly
	 * as it was: Pipe Z alone, VIDEO_PIPE_EN = 0x02.
	 *
	 * This runs after Pipe Z so the proven Pipe Z register sequence is
	 * untouched; DSER_PIPE_Y_CONF ends by setting VIDEO_PIPE_EN = 0x03,
	 * superseding Pipe Z's 0x02 and bringing both pipes up together.
	 */
	if (link_a_present) {
		if (serdes_parse_regdata(client, DSER_PIPE_Y_CONF,
					 ARRAY_SIZE(DSER_PIPE_Y_CONF),
					 DES_ADDR) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure DESER Pipe Y\n", __func__);
			return -EIO;
		}
		dev_info(&client->dev,
			 "configuring Pipe Y successful - both pipes enabled\n");
	}

	/*
	 * VIDEO_PIPE_EN, written last as the single source of truth for
	 * which pipes are live: bit0 = Pipe Y (link A), bit1 = Pipe Z
	 * (link B). The DSER_PIPE_*_CONF tables also touch 0x0160, so this
	 * final write settles it for whichever combination of links is
	 * actually populated - A only, B only, or both.
	 */
	{
		u8 pipe_en = (link_a_present ? 0x01 : 0x00) |
			     (link_b_present ? 0x02 : 0x00);

		if (serdes_write_16b_reg(client, DES_ADDR, 0x0160, pipe_en) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			return -EIO;
		}
		dev_info(&client->dev,
			 "VIDEO_PIPE_EN = 0x%02x (link A %s, link B %s)\n",
			 pipe_en,
			 link_a_present ? "present" : "absent",
			 link_b_present ? "present" : "absent");
	}

	return 0;
}

/*
 * The deserializer is SHARED between the two cameras, but this driver
 * probes once per camera DT node. __serdes_config_init() starts by
 * issuing a CHIP RESET of the deserializer (0x0010 = 0x80) and then
 * reprograms every link, serializer and pipe from scratch - so letting
 * it run on the second probe would tear down the first camera's
 * configuration while that camera is already up.
 *
 * Run it exactly once per boot. The mutex covers the probe-ordering
 * race: which camera probes first is not deterministic, and both can be
 * in flight at the same time on different CPUs.
 *
 * On failure the flag is left clear so a later probe retries rather than
 * inheriting a half-configured deserializer.
 */
static DEFINE_MUTEX(serdes_init_lock);
static bool serdes_initialized;

static s32 serdes_config_init(struct i2c_client *client)
{
	s32 ret;

	mutex_lock(&serdes_init_lock);

	if (serdes_initialized) {
		dev_info(&client->dev,
			 "serdes already configured by an earlier probe - skipping\n");
		mutex_unlock(&serdes_init_lock);
		return 0;
	}

	ret = __serdes_config_init(client);
	if (ret == 0)
		serdes_initialized = true;
	else
		dev_err(&client->dev,
			"serdes bring-up failed (%d); a later probe will retry\n", ret);

	mutex_unlock(&serdes_init_lock);
	return ret;
}

/* --------------------- GPIO Toggling --------------------- */

static void toggle_gpio_mcu(struct gpio_desc *gpio, int val)
{
	if (gpiod_cansleep(gpio)) {
		gpiod_direction_output(gpio, val);
		gpiod_set_value_cansleep(gpio, val);
	} else {
		gpiod_direction_output(gpio, val);
		gpiod_set_value_cansleep(gpio, val);
	}
}
/* --------- Camera Module Initialization Process ---------- */

int cam_core_initialize(struct imx568 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	unsigned char fw_version[32] = { 0 }, bin_fw_version[32] = { 0 };
	int ret, loop, err = 0, pwdn_gpio_toggle = 0, retry = 5;
	int frm_fmt_size = 0;
	uint32_t lanes = 0;

	ret = of_property_read_u32(node, "camera_mipi_lanes", &lanes);
	if (ret < 0) {
		dev_err(dev, "Error in getting Camera MIPI Lanes\n");
		return -EINVAL;
	}
	priv->mipi_lane_config = lanes;

	/*
	 * Pick THIS camera's serializer. Everything below - the MIPI lane
	 * count at 0x0331 and the 0x02CA/0x02BE MCU-reset GPIOs - is
	 * per-camera and was previously hardcoded to SER1_ADDR, so with two
	 * cameras up it all landed on serializer B: camera A never got its
	 * lane count set (staying 4-lane from SER1_CONF, so the MIPI TX PLL
	 * never locked and no frames came out) and camera B's MCU was reset
	 * twice.
	 */
	priv->ser_addr = (client->addr == CAM_A_ALIAS) ? SER_A_ADDR : SER_B_ADDR;
	dev_info(dev, "camera 0x%02x -> serializer 0x%02x\n",
		 client->addr, priv->ser_addr);

	if (priv->mipi_lane_config == 2) { // 2-lane configuration
		if (serdes_write_16b_reg(client, priv->ser_addr, 0x0331, 0x13) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__,
				__LINE__);
			return -EIO;
		}

	/*
	 * The deserializer -> SoC lane count and lane rate are NOT set here.
	 * They are a property of the board's CSI-2 output, not of this camera,
	 * and both cameras share one deserializer - so they are programmed once
	 * by dser_configure_dphy() from the device tree. Writing them per
	 * camera here would overwrite that with a hardcoded value.
	 *
	 * Note this is a DIFFERENT link from the sensor -> serializer one
	 * configured at 0x0331 above: the deserializer retransmits on its own
	 * MIPI TX, so its lane count is independent of what the camera feeds in.
	 */
		/*
		 * phy_2x4 (bit2), NOT phy_4x2 (bit0). Per the register doc,
		 * phy_2x4 gives "two ports with four data lanes each, Port A
		 * uses PHY0 and PHY1, Port B uses PHY2 and PHY3" - which is
		 * how this board is wired, and matches MAP_DPHY_DEST = 0x2A
		 * (PHY2) on both pipes. The port stays in 2x4 mode even when
		 * only two of its four lanes are enabled - the lane count at
		 * 0x048A is what selects that, not the port mode.
		 */
		if (serdes_write_16b_reg(client, DES_ADDR, 0x0330, 0x04) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__,
				__LINE__);
			return -EIO;
		}

		/*
		 * NOTE: two data-path reset variants were tried here while
		 * hunting the no-video fault - RESET_ONESHOT_B (CTRL2/0x0012
		 * bit5) and RESET_LINK_B (CTRL3/0x0013 bit0, written 1 then
		 * 0). Both executed cleanly and both had exactly zero effect
		 * on VIDEO_LOCK(Z)/CSIPLL lock, because the real fault was
		 * upstream at the pipe input (VIDEO_PIPE_SEL stream-ID
		 * mismatch - see DSER_PIPE_Z_CONF in econ_imx568_serdes.h).
		 * Both have been removed: the known-working Link A flow does
		 * no link reset at all here, and keeping an unproven reset
		 * would only break parity with it.
		 */
	}

	dev_info(&client->dev, "Number of mipi lane: %d\n",
		 priv->mipi_lane_config);

	// Read MCU firmware bin name from device tree
	ret = of_property_read_string(node, "cam_fw_name", &cam_fw_name);

	if (ret) {
		dev_err(dev,
			"Unable to get cam firmware name from the Device tree\n");
		return -EINVAL;
	}

	// Check if the CAM firmware is loaded or not
	// If not loaded already, load the CAM firmware
	if (!is_fw_loaded) {
		if (ecam_firmware_load(client) < ERRCODE_SUCCESS) {
			dev_err(dev, "Failed to load cam firmware\n");
			return -ENOENT;
		}
#ifdef EN_DEBUG_PRINTS
		dev_info(dev, "Firmware Load Success\n");
#endif
		is_fw_loaded = 1;
	}
	// MCU Reset Sequence
	if (serdes_write_16b_reg(client, priv->ser_addr, 0x02CA, 0x40) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	usleep_range(10000, 11000);
	if (serdes_write_16b_reg(client, priv->ser_addr, 0x02BE, 0x40) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	usleep_range(10000, 11000);
	if (serdes_write_16b_reg(client, priv->ser_addr, 0x02CA, 0x98) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
		return -EIO;
	}
	msleep(3000); // Delay required to boot the MCU

	ret = is_fw_update_required(client, priv, fw_version, bin_fw_version);
	if (ret != 0) {
		if (ret > 0) {
			err = cam_jump_bload(client, priv);
			if (err < 0) {
				dev_err(dev,
					" Cannot go into bootloader mode\n");
				return -EIO;
			}
			msleep(100);
		} else {
			/* ret value has to be -1 */
#ifdef EN_DEBUG_PRINTS
			dev_info(dev, " Switching MCU to Bootloader mode\n");
#endif
		}

		ret = cam_bload_get_version(client);
		if (ret < 0) {
			dev_err(dev, " Error in Get Version\n");

			/*
			 * Since error in reading the bootloader version:
			 * set MCU to bootloader mode
			 */
			if (serdes_write_16b_reg(client, priv->ser_addr, 0x02CA,
						 0x40) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
				return -EIO;
			}
			usleep_range(10000, 11000);
			if (serdes_write_16b_reg(client, priv->ser_addr, 0x02BE,
						 0x98) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
				return -EIO;
			}
			usleep_range(10000, 11000);
			if (serdes_write_16b_reg(client, priv->ser_addr, 0x02CA,
						 0x98) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
				return -EIO;
			}
			usleep_range(10000, 11000);

			/* Reading the MCU Firmware version from bootloader mode */
			for (loop = 0; loop < MAX_ATTEMPTS; loop++) {
				ret = cam_bload_get_version(client);
				if (ret < 0) {
					dev_err(dev,
						"Error getting Firmware version.. Retrying...\n");
					msleep(1000);
					continue;
				} else {
					break;
				}
			}

			/*
			 * Failed reading FW version in bootloader mode even
			 * after MAX_attempts. Return Failure
			 */
			if (loop == MAX_ATTEMPTS) {
				dev_err(dev,
					"%s (%d) Error reading MCU FW version in bootloader mode also. Exiting.\n",
					__func__, __LINE__);
				return -EINVAL;
			}
		}

		/*Attempt Firmware Update */
		if (cam_fw_update(client, bin_fw_version) < 0) {
			dev_err(dev,
				"%s (%d) Error Updating MCU FW. Exiting.\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		// After Firmware do MCU reset
		if (serdes_write_16b_reg(client, priv->ser_addr, 0x02CA, 0x40) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__,
				__LINE__);
			return -EIO;
		}
		usleep_range(10000, 11000);
		if (serdes_write_16b_reg(client, priv->ser_addr, 0x02BE, 0x40) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__,
				__LINE__);
			return -EIO;
		}
		usleep_range(10000, 11000);
		if (serdes_write_16b_reg(client, priv->ser_addr, 0x02CA, 0x98) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__,
				__LINE__);
			return -EIO;
		}
		msleep(100); // Wait time for mcu to boot

	} else {
		/* Same Firmware version in MCU and bin file */
		dev_info(dev, "Cam Firmware Version - (%.32s)\n", fw_version);
	}

	/* Configure MIPI Lanes of the Sensor */
	retry = 5;
	while (--retry > 0) {
		if (cam_lane_configuration(client, priv) < 0) {
			dev_err(dev,
				"%s, Failed to set lane CONFIG Data. retrying!\n",
				__func__);
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to set lane CONFIG Data!\n", __func__);
		return -EFAULT;
	}

	/* Query the number of controls from MCU */
	retry = 5;
	while (--retry > 0) {
		if (cam_list_ctrls(client, priv, NULL) < 0) {
			dev_err(dev, "%s, init controls failure. retrying\n",
				__func__);
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to init controls!\n", __func__);
		return -EFAULT;
	}

	priv->cam_ctrl_info = devm_kzalloc(
		dev, sizeof(struct isp_ctrl_info) * priv->num_ctrls, GFP_KERNEL);
	if (!priv->cam_ctrl_info)
		return -ENOMEM;

	priv->ctrldb = devm_kzalloc(dev, sizeof(uint32_t) * priv->num_ctrls,
				    GFP_KERNEL);
	if (!priv->ctrldb)
		return -ENOMEM;

	/* Fill the controls */
	retry = 5;
	while (--retry > 0) {
		if (cam_list_ctrls(client, priv, priv->cam_ctrl_info) < 0) {
			dev_err(dev, "%s, Failed to init controls\n", __func__);
		} else {
#ifdef EN_DEBUG_PRINTS
			dev_info(dev, "Num of Controls - %d\n",
				 priv->num_ctrls);
#endif
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to init formats!\n", __func__);
		return -EFAULT;
	}

	/* Query the number of formats available from MCU */
	retry = 5;
	while (--retry > 0) {
		if (cam_list_fmts(client, priv, NULL, &frm_fmt_size) < 0) {
			dev_err(dev, "%s, Failed to init formats\n", __func__);
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to init formats!\n", __func__);
		return -EFAULT;
	}

	priv->nr_supported_formats = frm_fmt_size;
	priv->stream_info = devm_kzalloc(
		dev, sizeof(struct isp_stream_info) * (frm_fmt_size + 1), GFP_KERNEL);
	priv->streamdb =
		devm_kzalloc(dev, sizeof(int) * (frm_fmt_size + 1), GFP_KERNEL);
	if (!priv->streamdb) {
		dev_err(dev, "unable to allocate memory\n");
		return -ENOMEM;
	}

	priv->cam_frmfmt = devm_kzalloc(
		dev, sizeof(struct camera_common_frmfmt) * (frm_fmt_size + 1),
		GFP_KERNEL);
	if (!priv->cam_frmfmt)
		return -ENOMEM;

	/* Initialise the ISP */
	if (cam_init(client) < 0) {
		dev_err(dev, "Unable to INIT ISP\n");
		return -EFAULT;
	}

	for (loop = 0; loop <= (frm_fmt_size); loop++) {
		/* create Frame Rate array */
		priv->cam_frmfmt[loop].framerates = devm_kzalloc(
			dev, sizeof(int) * MAX_NUM_FRATES, GFP_KERNEL);
		if (!priv->cam_frmfmt[loop].framerates)
			return -ENOMEM;
	}

	/* List the formats from MCU */
	retry = 5;
	while (--retry > 0) {
		if (cam_list_fmts(client, priv, priv->stream_info,
				  &frm_fmt_size) < 0) {
			dev_err(dev, "Unable to List Fmts. retrying!\n");
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to List formats!\n", __func__);
		return -EFAULT;
	}
	return 0;
}

/*
 * Sensor register read:
 * One or two bytes read is supported
 */
static int sensor_reg_read(struct i2c_client *client, struct imx568 *priv,
			   uint16_t reg_addr, uint8_t reg_len)
{
	uint8_t mc_data[512], mc_ret_data[512];
	uint16_t reg_val = 0;
	uint16_t size = 0, send_len = 0, payload_len = 0;
	int retcode = ERRCODE_SUCCESS;
	int err = ERRCODE_SUCCESS;

	/*lock semaphore */
	mutex_lock(&priv->mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_READ;
	mc_data[2] = 0x00;
	mc_data[3] = 0x03;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_READ;
	mc_data[2] = reg_addr >> 8;
	mc_data[3] = reg_addr & 0xFF;
	mc_data[4] = reg_len; // one or two byte read is supported
	mc_data[5] = errorcheck(&mc_data[2], 3);

	err = cam_write(client, mc_data, RX_LEN_PKT);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n",
			__func__, __LINE__, err);
		goto exit;
	}

	memset(mc_ret_data, 0, 512);
	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
			__LINE__, err);
		goto exit;
	}

	send_len = (mc_ret_data[2] << 8) | mc_ret_data[3];

	payload_len = send_len + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0, 512);
	err = cam_read(client, mc_ret_data, send_len + HEADER_FOOTER_SIZE);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
			__LINE__, err);
		goto exit;
	}

	reg_val = mc_ret_data[4] << 8;
	reg_val = reg_val | mc_ret_data[5];

	retcode = mc_ret_data[payload_len - 1];

	if (retcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, "Error read %d\n", __LINE__);
		err = retcode;
		goto exit;
	}
	/* Unlock semaphore */
	mutex_unlock(&priv->mutex);
	return reg_val;
exit:
	/* Unlock semaphore */
	mutex_unlock(&priv->mutex);
	return err;
}

/*
 * Sensor register write:
 * Only 2-byte write is supported
 */
static int sensor_reg_write(struct i2c_client *client, struct imx568 *priv,
			    uint16_t reg_addr, uint16_t reg_val)
{
	uint8_t mc_data[512], mc_ret_data[512];
	uint16_t size = 0, send_len = 0, payload_len = 0;
	int err = 0;

	/* lock semaphore */
	mutex_lock(&priv->mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_WRITE;
	mc_data[2] = 0x00;
	mc_data[3] = 0x05;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	usleep_range(1000, 2000);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_WRITE;
	mc_data[2] = reg_addr >> 8;
	mc_data[3] = reg_addr & 0xFF;
	mc_data[4] = 0x02; // write data length - Only 2byte is supported
	mc_data[5] = reg_val >> 8;
	mc_data[6] = reg_val & 0xFF;
	mc_data[7] = errorcheck(&mc_data[2], 5);

	err = cam_write(client, mc_data, 8);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n",
			__func__, __LINE__, err);
		goto exit;
	}
	/* Unlock semaphore */
	mutex_unlock(&priv->mutex);
	return 0;
exit:
	/* Unlock semaphore */
	mutex_unlock(&priv->mutex);
	return err;
}

static int imx568_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx568 *imx568;
	const struct of_device_id *match;
	int ret = 0, retry = 5;
	u32 tm_of;
	uint16_t sensor_id = 0;

	imx568 = devm_kzalloc(&client->dev, sizeof(*imx568), GFP_KERNEL);
	if (!imx568)
		return -ENOMEM;

	imx568->i2c_client = client;
	v4l2_i2c_subdev_init(&imx568->sd, client, &imx568_subdev_ops);

	match = of_match_device(imx568_dt_ids, dev);
	if (!match)
		return -ENODEV;

	// serdes configuration
	ret = serdes_config_init(client);
	if (ret != 0) {
		dev_err(dev, "Serdes configuration failed\n");
		return -EINVAL;
	}

	/* Check the hardware configuration in device tree */
	if (imx568_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx568->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx568->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx568->xclk);
	}

	imx568->xclk_freq = clk_get_rate(imx568->xclk);
	if (imx568->xclk_freq != IMX568_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx568->xclk_freq);
		return -EINVAL;
	}

	/* Request cam reset pin */
	imx568->reset_gpio =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx568->boot_gpio))
		return dev_err_probe(dev, PTR_ERR(imx568->reset_gpio),
				     "Failed to get reset GPIO\n");

	/* Request cam boot pin */
	imx568->boot_gpio = devm_gpiod_get_optional(dev, "boot", GPIOD_OUT_LOW);
	if (IS_ERR(imx568->boot_gpio))
		return dev_err_probe(dev, PTR_ERR(imx568->boot_gpio),
				     "Failed to get boot GPIO\n");

	ret = imx568_power_on(dev);
	if (ret)
		return ret;

	// Initialize Mutex
	mutex_init(&imx568->mutex);

	// Camera Initialization
	ret = cam_core_initialize(imx568);
	if (ret)
		goto error_power_off;

	usleep_range(10000, 11000);

	retry = 5;
	while (retry-- > 0) {
		// Sensor register Write
		ret = sensor_reg_write(client, imx568, 0x3000, 0x00);
		if (ret)
			continue;

		msleep(20);
		// Sensor Register Read
		sensor_id = sensor_reg_read(client, imx568, 0x3816, 2);

		// Sensor register Write
		ret = sensor_reg_write(client, imx568, 0x3000, 0x01);
		if (ret)
			continue;

		sensor_id = sensor_id >> 5;

		if (sensor_id != IMX568_SENSOR_ID)
			continue;
		break;
	}

	if (retry < 0) {
		ret = -EINVAL;
		goto error_power_off;
	}

	sensor_id = IMX568_SENSOR_ID;
	dev_dbg(dev, "IMX568 Sensor ID: %d\n", sensor_id);

	if (sensor_id != IMX568_SENSOR_ID) {
		dev_err(dev, "The Connected camera is not IMX568 - %d != 568\n",
			sensor_id);
		ret = -EINVAL;
		goto error_power_off;
	}

	/* Initialize default format */
	imx568_set_default_format(imx568);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx568_init_controls(imx568);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx568->sd.internal_ops = &imx568_internal_ops;
	imx568->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx568->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx568->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	//imx568->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx568->sd.entity, NUM_PADS, imx568->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&imx568->sd);
	if (ret < 0) {
		dev_err(dev, "Failed in finalize %d\n", ret);
		goto error_media_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&imx568->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_subdev_cleanup;
	}

	dev_info(dev, "Detected IMX%d camera\n", sensor_id);

	return 0;

error_subdev_cleanup:
	v4l2_subdev_cleanup(&imx568->sd);

error_media_entity:
	media_entity_cleanup(&imx568->sd.entity);

error_handler_free:
	imx568_free_controls(imx568);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx568_power_off(&client->dev);

	return ret;
}

#define FREE_SAFE(dev, ptr)                   \
	do {                                  \
		if (ptr)                      \
			devm_kfree(dev, ptr); \
	} while (0)
static void imx568_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *imx568 = to_imx568(sd);
	int loop = 0;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx568_free_controls(imx568);

	/* Releasing the MCU firmware by the driver when rmmod is issued */
	if (is_fw_loaded == 1) {
		release_firmware(cam_fw);
		cam_fw = NULL;

		if (cam_fw_buf != NULL) {
			kfree(cam_fw_buf);
			cam_fw_buf = NULL;
		}

		is_fw_loaded = 0;
#ifdef EN_DEBUG_PRINTS
		dev_info(&client->dev, "Firmware Release Success\n");
#endif
	}

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx568_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	/* Free up memory */

	FREE_SAFE(&client->dev, imx568->cam_ctrl_info);

	for (loop = 0; loop < imx568->frm_fmt_size; loop++) {
		FREE_SAFE(&client->dev,
			  (void *)imx568->cam_frmfmt[loop].framerates);
	}

	FREE_SAFE(&client->dev, imx568->cam_frmfmt);

	FREE_SAFE(&client->dev, imx568->ctrldb);
	FREE_SAFE(&client->dev, imx568->streamdb);

	FREE_SAFE(&client->dev, imx568->stream_info);
	FREE_SAFE(&client->dev, imx568);
}

MODULE_DEVICE_TABLE(of, imx568_dt_ids);

static const struct dev_pm_ops imx568_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx568_suspend, imx568_resume)
		SET_RUNTIME_PM_OPS(imx568_power_off, imx568_power_on, NULL)
};

static struct i2c_driver imx568_i2c_driver = {
	.driver = {
		.name = "econ-imx568-gmsl",
		.of_match_table	= imx568_dt_ids,
		.pm = &imx568_pm_ops,
	},
	.probe = imx568_probe,
	.remove = imx568_remove,
};

module_i2c_driver(imx568_i2c_driver);

MODULE_AUTHOR("Kishore Kumar <kishore.kumar@e-consystems.com>");
MODULE_DESCRIPTION("e-con camera driver");
MODULE_LICENSE("GPL");
