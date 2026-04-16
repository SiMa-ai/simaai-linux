// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX568 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
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
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include "econ_imx568_drv.h"

// Define EN_DEBUG_PRINTS Macro to enable debug prints
#define EN_DEBUG_PRINTS


unsigned char errorcheck(char *data, unsigned int len)
{
        unsigned int i = 0;
        unsigned char crc = 0x00;

        for (i = 0; i < len; i++) {
                crc ^= data[i];
        }

        return crc;
}

static int cam_read(struct i2c_client *client, u8 * val, u32 count)
{
        int ret;
        struct i2c_msg msg = {
                .addr = client->addr,
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

static int cam_write(struct i2c_client *client, u8 * val, u32 count)
{
        int ret;
        struct i2c_msg msg = {
                .addr = client->addr,
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

/* --------------------- GPIO Toggling --------------------- */

static void toggle_gpio_mcu(struct gpio_desc *gpio, int val)
{
	if (gpiod_cansleep(gpio)) {
		gpiod_direction_output(gpio,val);
		gpiod_set_value_cansleep(gpio, val);
	} else {
		gpiod_direction_output(gpio,val);
		gpiod_set_value_cansleep(gpio, val);
	}
}

static int cam_get_cmd_status(struct i2c_client *client, uint8_t * cmd_id,
                              uint16_t * cmd_status, uint8_t * ret_code)
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

        cam_write(client, mc_data, TX_LEN_PKT);
        if (err != 0) {
                dev_err(&client->dev,
				" %s(%d) MCU Get CMD Status Write Error - %d \n", __func__, __LINE__, err);
                return -1;
        }

        mc_data[0] = CMD_SIGNATURE;
        mc_data[1] = CMD_ID_GET_STATUS;
        mc_data[2] = *cmd_id;

        err = cam_write(client, mc_data, 3);
        if (err != 0) {
                dev_err(&client->dev,
				" %s(%d) MCU Get CMD Status Write Error - %d \n", __func__, __LINE__, err);
                return -1;
        }

        payload_len = CMD_STATUS_MSG_LEN;
        memset(mc_ret_data, 0x00, payload_len);

        err = cam_read(client, mc_ret_data, payload_len);
        if (err != 0) {
                dev_err(&client->dev,
				" %s(%d) MCU Get CMD Status Length Error - %d \n", __func__, __LINE__, err);
                return -1;
        }

        /* Verify CRC */
        orig_crc = mc_ret_data[payload_len - 2];
        calc_crc = errorcheck(&mc_ret_data[2], 3);
        if (orig_crc != calc_crc) {
                dev_err(&client->dev,
				" %s(%d) MCU Get CMD Status Error CRC 0x%02x != 0x%02x \n", __func__,
				__LINE__, orig_crc, calc_crc);
                return -1;
        }

        *cmd_id = mc_ret_data[2];
        *cmd_status = mc_ret_data[3] << 8 | mc_ret_data[4];
        *ret_code = mc_ret_data[payload_len - 1];

        return 0;
}

/* NOTE : Caller need to take the lock */
static int cam_set_ctrl(struct i2c_client *client, struct imx568 *priv, uint32_t arg_ctrl_id,
                        uint8_t ctrl_type, int64_t curr_val)
{
        unsigned char mc_data[100];
        uint32_t payload_len = 0, ctrl_val_len = 0;

        uint16_t cmd_status = 0, index = 0xFFFF;
        uint8_t retcode = 0, cmd_id = 0;
        int loop = 0, ret = 0, err =0;
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
	payload_len =
		(ctrl_type == CTRL_STANDARD) ? 11 : 20;
        /* First Txn Payload length = 0 */
	ctrl_val_len =
		(ctrl_type == CTRL_STANDARD) ? 4 : 8;


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
			mc_data[9]  = V4L2_CTRL_TYPE_INTEGER64;
			mc_data[10] = ctrl_val_len >> 24;
			mc_data[11] = ctrl_val_len >> 16;
			mc_data[12] = ctrl_val_len >> 8;
			mc_data[13] = ctrl_val_len & 0xFF;
			for (loop = 0;loop < ctrl_val_len; loop++)
				mc_data[21-loop] = (curr_val >> (8 * loop));
			/* CRC */
			mc_data[22] = errorcheck(&mc_data[2], payload_len);
		}
        err = cam_write(client, mc_data, payload_len+3);
        if (err != 0) {
                dev_err(&client->dev," %s(%d) MCU Set Ctrl Error - %d \n", __func__,
                       __LINE__, err);
                ret = -1;
                goto exit;
        }

        while (--retry > 0) {
		yield();
                cmd_id = CMD_ID_SET_CTRL;
                if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
                    0) {
                        dev_err(&client->dev," %s(%d) MCU Get CMD Status Error \n", __func__,
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

static int cam_set_exposure(struct i2c_client *client, struct imx568 *priv, s64 val)
{
	int err = 0, retry = 5;
	uint64_t data = val;

	while (--retry > 0) {
		if ((err = cam_set_ctrl(client, priv, EXPOSURE_CTRL_ID, CTRL_EXTENDED, data)) < 0) {
			dev_err(&client->dev, "%s[%d] Fail! retrying\n",__func__,__LINE__);
			continue;
		} else {
			return 0;
		}
	}

	dev_err(&client->dev, "%s[%d] Failed after retries!\n",__func__,__LINE__);
	return -EINVAL;

}

static int cam_set_framerate(struct i2c_client *client, struct imx568 *priv, s64 val)
{
	int err = 0, retry = 5;
	uint64_t data = val;

	while (--retry > 0) {
		if ((err = cam_set_ctrl(client, priv, FRAMERATE_CTRL_ID, CTRL_EXTENDED, data)) < 0) {
			dev_err(&client->dev, "%s[%d] Fail! retrying\n",__func__,__LINE__);
			continue;
		} else {
			return 0;
		}
	}

	dev_err(&client->dev, "%s[%d] Failed after retries!\n",__func__,__LINE__);
	return -EINVAL;

}

// MCU APIs
static int cam_init(struct i2c_client *client)
{
        unsigned char mc_data[100];
        uint32_t payload_len = 0;

        uint16_t cmd_status = 0;
        uint8_t retcode = 0, cmd_id = 0;
	int retry = 10, err = 0 ,ret = 0;

        /* check current status of cam */
        cmd_id = CMD_ID_INIT_CAM;
        if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
                dev_err(&client->dev," %s(%d) MCU CAM Init ISP Error \n",
				__func__, __LINE__);
                return -1;
        }

        if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
            (retcode == ERRCODE_SUCCESS)) {
                dev_info(&client->dev," %s %d CAM Initialized !! \n",
				__func__, __LINE__ );
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
                dev_err(&client->dev," %s(%d) MCU Get CMD CAM Init Error - "
				"%d \n", __func__, __LINE__, err);
                return -1;
        }

        while (--retry > 0) {
		msleep (100); // wait till sensor to initialise
                /* Some Sleep for init to process */
		yield();
                cmd_id = CMD_ID_INIT_CAM;
                if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
                    0) {
                       dev_err(&client->dev," %s(%d) MCU CMD ID CAM INIT Error \n", __func__,
                               __LINE__);
                        msleep(5);
			ret = -1;
			continue;
                }

                if (cmd_status == MCU_CMD_STATUS_SUCCESS) {
                        dev_err(&client->dev,"%s(%d) CAM INIT Success !! \n", __func__,__LINE__);
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
		dev_err(&imx568->i2c_client->dev," %s(%d) MCU Get CMD CAM Init Error - "
				"%d \n", __func__, __LINE__, ret);

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
	mc_data[9]  = V4L2_CTRL_TYPE_INTEGER64;
	mc_data[10] = ctrl_val_len >> 24;
	mc_data[11] = ctrl_val_len >> 16;
	mc_data[12] = ctrl_val_len >> 8;
	mc_data[13] = ctrl_val_len & 0xFF;
	for (loop = 0;loop < ctrl_val_len; loop++)
		mc_data[21-loop] = (gain >> (8 * loop));
	/* CRC */
	mc_data[22] = errorcheck(&mc_data[2], payload_len);

	ret = cam_write(imx568->i2c_client, mc_data, payload_len + 3);
    if (ret != 0) {
		dev_err(&imx568->i2c_client->dev," %s(%d) MCU Get CMD CAM Init Error - "
				"%d \n", __func__, __LINE__, ret);

		return -1;
    }

	return ret;
}

#define AGAIN_PRECISION 12
#define LOG10_2_AGAIN_PREC ( 1233 ) // log10(2) << AGAIN_PRECISION
#define LOG_TO_DB ( 20 )
#define NORMALISE_FACTOR (LOG2_GAIN_SHIFT - AGAIN_PRECISION)
#define CONVERSION_FACTOR ((LOG10_2_AGAIN_PREC * LOG_TO_DB * GAIN_FACTOR))

static int32_t sensor_set_analogue_gain( struct imx568 *imx568, int32_t gain )
{
    uint32_t a_gain;
	int32_t ret = 0;

	if (imx568->again != gain) {
		// Conversion of log2_gain value to corresponded sensor gain value in dB
	    a_gain = (((gain >> NORMALISE_FACTOR) * CONVERSION_FACTOR)) >> AGAIN_PRECISION;
		// Conversion of dB to Gain Values to parse to the MCU to configure sensor
		ret = cam_set_gain (imx568, (uint64_t)a_gain);
		if (ret == 0) {
			imx568->again = gain;
		}
	}

    return ret;
}

static int32_t sensor_set_digital_gain( struct imx568 *imx568, int32_t gain )
{
    uint32_t d_gain;
    int32_t ret = 0;
	
	if (imx568->dgain != gain) {
		// Conversion of log2_gain value to corresponded sensor gain value in dB
		d_gain = (((gain >> NORMALISE_FACTOR) * CONVERSION_FACTOR)) >> AGAIN_PRECISION;
		// Conversion of dB to Gain Values to parse to the MCU to configure sensor
		d_gain = (d_gain) + (24 * GAIN_FACTOR); // 24 - Adding Sensor Analog Gain maximum: 24dB
		ret = cam_set_gain (imx568, (uint64_t)d_gain);
		if (ret == 0){
			imx568	->dgain = gain;
		}
	}

	return ret;
}

static int32_t sensor_set_exposure( struct imx568 *imx568, uint32_t integration_time)
{
    uint64_t exp = 0;
    int32_t ret = 0;

    if (imx568->integration_time != integration_time) {
		// Conversion of lines to exposure time (us)
		exp = (uint64_t)(integration_time) * (imx568->cam_frmfmt[imx568->frmfmt_mode].hmax);
		exp = (exp * EXPOSURE_FACTOR)/ SENSOR_PIXEL_CLOCK;
		ret = cam_set_exposure (imx568->i2c_client, imx568, exp);
		if (ret == 0) {
			imx568->integration_time = integration_time;
		}
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
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx568_update_image_pad_format(struct imx568 *imx568,
					   struct v4l2_subdev_format *fmt)
{
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
			v4l2_subdev_state_get_format(sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx568_update_image_pad_format(imx568, fmt);
			fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
			fmt->format.width = imx568->cam_frmfmt[imx568->frmfmt_mode].size.width;
			fmt->format.height = imx568->cam_frmfmt[imx568->frmfmt_mode].size.height;
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
        for (loop = 0; (&priv->streamdb[loop])!= NULL; loop++) {
                if (priv->streamdb[loop] == mode) {
                        index = loop + frate_index;
                        break;
                }
        }
        if (index == 0xFFFF) {
                ret = -EINVAL;
                goto exit;
        }

	dev_info (&client->dev, "Mode: %d, Width: %d, Height: %d, Format: 0x%x Framerate: %d\n",index,
		priv->cam_frmfmt[mode].size.width, priv->cam_frmfmt[mode].size.height, format,
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
	err = cam_write(client, mc_data, payload_len + 3); // Payload_len + CMD_SIGNATURE + CMD_ID + CRC
	if (err != 0) {
                dev_err(&client->dev," %s(%d) MCU Stream Config Error - %d \n",
				__func__, __LINE__, err);
                ret = -1;
                goto exit;
        }

        while (--retry > 0) {
		/* test Some time for processing command */
                yield();

                cmd_id = CMD_ID_STREAM_CONFIG;
                if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
                    0) {
                       dev_err(&client->dev,
				       " %s(%d) MCU GET CMD Status Error : loop : %d \n",
				       __func__, __LINE__, loop);
			ret = -1;
                        continue;
                }

                if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
                    (retcode == ERRCODE_SUCCESS)) {
                        dev_info(&client->dev, " %s(%d) Status Success !! \n", __func__, __LINE__);
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
		fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
		imx568_update_image_pad_format(imx568, fmt);

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		}
	} else {
		// Not Implemented
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx568_update_metadata_pad_format(fmt);
		}
	}

	switch (fmt->format.code) {
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			imx568->format_fourcc = V4L2_PIX_FMT_SRGGB12;
			break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			imx568->format_fourcc = V4L2_PIX_FMT_SRGGB10;
			break;
	}

	for (i = 0; i < imx568->frm_fmt_size; i++) {
		if ((imx568->cam_frmfmt[i].size.width == fmt->format.width)
				&& (imx568->cam_frmfmt[i].size.height ==
					fmt->format.height) && (imx568->cam_frmfmt[i].fourcc == imx568->format_fourcc)) {
			imx568->frmfmt_mode = imx568->cam_frmfmt[i].mode;
			imx568->frate_index = 0;
			break;
		}
	}

	mutex_unlock(&imx568->mutex);
	while (retry-- > 0 ) {
		if((err = cam_set_ctrl(imx568->i2c_client, imx568, SENSOR_MODE_CTRL_ID,
					CTRL_STANDARD, (int64_t)imx568->frmfmt_mode)) < 0)
			dev_err(&client->dev,"%s[%d]\n",__func__,__LINE__);
		else
			break;
	}

	if (retry < 0)
		return err;

	curr_framerate = imx568->cam_frmfmt[imx568->frate_index].framerates;

	// set frame rate based on resolution
	retry = 5;
	while (retry-- > 0 ) {
		if ((err = cam_set_framerate (client, imx568, curr_framerate  * MULTIPLY_FACTOR)) < 0)
			dev_err(&client->dev,"%s[%d]\n",__func__,__LINE__);
		else
			break;
	}
	if (retry < 0)
		return err;

	mutex_lock(&imx568->mutex);

	retry = 5;
	while (retry-- > 0 ) {
		if((err = cam_stream_config(imx568->i2c_client, imx568, imx568->format_fourcc,
					imx568->frmfmt_mode, imx568->frate_index)) < 0) {
			dev_err(&client->dev,"%s[%d]\n",__func__,__LINE__);
			ret = err;
		} else
			break;
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

        cam_write(client, mc_data, TX_LEN_PKT);

        mc_data[0] = CMD_SIGNATURE;
        mc_data[1] = CMD_ID_STREAM_ON;
        err = cam_write(client, mc_data, 2);
        if (err != 0) {
                dev_err(&client->dev,
				" %s(%d) MCU Stream On Write Error - %d \n", __func__, __LINE__, err);
                goto exit;
        }

        while (--retry > 0) {
                /* Some Sleep for init to process */
                yield();

                cmd_id = CMD_ID_STREAM_ON;
                if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
                    0) {
                       dev_err(&client->dev,
				       " %s(%d) MCU Get CMD Stream On Error \n", __func__, __LINE__);
		       err = -1;
		       continue;
                }

                if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
                    (retcode == ERRCODE_SUCCESS)) {
                        dev_info(&client->dev,
					" %s %dMCU Stream On Success !! \n", __func__, __LINE__);
			err = 0;
			goto exit;
                }

                if ((retcode != ERRCODE_BUSY) &&
                    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
                       dev_err(&client->dev,
                            "(%s) %d MCU Get CMD Stream On Error STATUS = "
			    "0x%04x RET = 0x%02x\n", __func__, __LINE__, cmd_status, retcode);
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

	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	msleep(1);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	err = cam_write(client, mc_data, 2);
	msleep(1);
	if (err != 0) {
		dev_err(&client->dev,
				"%s(%d) MCU Stream OFF Write Error - %d \n", __func__, __LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		yield();
		cmd_id = CMD_ID_STREAM_OFF;
		if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev,
					"%s(%d) MCU Get CMD Stream Off Error \n", __func__, __LINE__);
			err = -1;
			continue;
		}
		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				(retcode == ERRCODE_SUCCESS)) {
			dev_info(&client->dev,
					" %s %d MCU Get CMD Stream off Success !! \n", __func__, __LINE__ );
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
					"(%s) %d MCU Get CMD Stream off Error STATUS = "
					"0x%04x RET = 0x%02x\n", __func__, __LINE__, cmd_status, retcode);
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

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx568->sd.ctrl_handler);
	if (ret)
		return ret;

	while (retry-- > 0) {
		if ((ret = cam_stream_on(client, imx568)) > 0)
			continue;
		else
			break;
	}
	if(retry < 0){
		dev_err(&client->dev,"%s (%d) Stream_On - Failed\n", __func__, __LINE__);
	}

	return ret;
}

/* Stop streaming */
static void imx568_stop_streaming(struct imx568 *imx568)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx568->sd);
	int ret = 0, retry = 5;

	while (retry-- > 0) {
		if ((ret = cam_stream_off(client, imx568)) > 0)
			continue;
		else
			break;
	}
	if(retry < 0){
		dev_err(&client->dev,"%s (%d) Stream_OFF - Failed\n", __func__, __LINE__);
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

	ret = regulator_bulk_enable(IMX568_NUM_SUPPLIES,
				    imx568->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx568->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}
	return 0;

reg_off:

	regulator_bulk_disable(IMX568_NUM_SUPPLIES, imx568->supplies);

	return ret;
}

static int imx568_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx568 *imx568 = to_imx568(sd);

	regulator_bulk_disable(IMX568_NUM_SUPPLIES, imx568->supplies);

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

	return devm_regulator_bulk_get(&client->dev,
				       IMX568_NUM_SUPPLIES,
				       imx568->supplies);
}

static int imx568_get_mbus_config(struct v4l2_subdev *sd,
				    unsigned int pad,
				    struct v4l2_mbus_config *cfg)
{
	cfg->link_freq =  IMX568_DEFAULT_LINK_FREQ;
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
	imx568->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx568_ctrl_ops     ,
                       V4L2_CID_PIXEL_RATE,
                       SENSOR_PIXEL_CLOCK,
                       SENSOR_PIXEL_CLOCK, 1,
                       SENSOR_PIXEL_CLOCK);
	if (imx568->pixel_rate)
		imx568->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* LINK_FREQ is also read only */
	imx568->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx568_ctrl_ops,
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

	imx568->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx568_ctrl_ops,
                         V4L2_CID_EXPOSURE,
                         IMX568_EXPOSURE_MIN,
                         IMX568_EXPOSURE_MAX,
                         IMX568_EXPOSURE_STEP,
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
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
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
			dev_err(dev,"Invalid number of CSI2 data lanes %d\n", ep_cfg.bus.mipi_csi2.num_data_lanes);
			goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != IMX568_DEFAULT_LINK_FREQ) {
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

static const struct of_device_id imx568_dt_ids[] = {
	{ .compatible = "sony,imx568"},
	{ /* sentinel */ }
};

// MCU Firmware file read from rootfs - /lib/firmware
static int ecam_firmware_load(struct i2c_client *client)
{
        unsigned char fw_version[32] = {0}, bin_fw_version[32] = {0};
        int ret = 0;
        unsigned long bin_fw_pos = 0;

        /* Request firmware from the rootfs */
        ret = request_firmware(&cam_fw, cam_fw_name, &client->dev);

	if (ret < 0)
                return -ENOENT;

        bin_fw_pos = cam_fw->size - VERSION_FILE_OFFSET;
        cam_fw_buf = kmalloc (cam_fw->size + 1, GFP_KERNEL);
        cam_fw_buf[cam_fw->size] = '\0';
        memcpy(cam_fw_buf, cam_fw->data, cam_fw->size);

        return ERRCODE_SUCCESS;
}

int cam_bload_ascii2hex(unsigned char ascii)
{
	if (ascii <= '9') {
		return (ascii - '0');
	} else if ((ascii >= 'a') && (ascii <= 'f')) {
		return (0xA + (ascii - 'a'));
	} else if ((ascii >= 'A') && (ascii <= 'F')) {
		return (0xA + (ascii - 'A'));
	}

	return -1;
}
static int is_fw_update_required(struct i2c_client *client, struct imx568 *priv,
		unsigned char *fw_version, unsigned char *bin_fw_version)
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
		* (bin_fw_version + i) = (cam_bload_ascii2hex(cam_fw_buf[loop]) << 4 |
				cam_bload_ascii2hex(cam_fw_buf[loop + 1]));
		i ++;
	}

	/* Check for forced/always update field in the text firmware version */
	if (bin_fw_version[17] == '1') {
#ifdef EN_DEBUG_PRINTS
		dev_info(&client->dev,"Forced MCU Update Flag Enabled - Firmware Version - (%.32s) \n"
				,bin_fw_version);
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
			dev_err(&client->dev," %s(%d) MCU CMD ID Write PKT fw Version Error - %d \n", __func__,
					__LINE__, err);
			ret = -1;
			goto exit;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) MCU CMD ID Read PKT fw Version Error - %d \n", __func__,
					__LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify checksum' */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev," %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}

		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev," %s(%d) MCU CMD ID fw Errcode - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -1;
			goto exit;
		}

		/* Read the actual version from MCU */
		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
		memset(mc_ret_data, 0x00, payload_len);
		ret = cam_read(client, mc_ret_data, payload_len);
		if (ret != 0) {
			dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Version Error - %d \n", __func__,
					__LINE__, ret);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2], 32);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev," %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -1;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -1;
			goto exit;
		}

		for (loop = 0 ; loop < VERSION_SIZE ; loop++)
			*(fw_version+loop) = mc_ret_data[2+loop];


		for(i = 0; i < VERSION_SIZE; i++)
		{
			if(bin_fw_version[i] != fw_version[i]) {
				dev_info(&client->dev,"Previous Firmware Version - (%.32s)\n",
						fw_version);
				dev_info(&client->dev,"Current Firmware Version - (%.32s)\n",
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
		dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
		return -1;
	}

	if (g_bload_buf[0] != 'y') {
		/*   NACK Received */
		dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
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

	dev_info(&client->dev," Erasing camera firmware...\n");

        for (i = 0; i < NUM_ERASE_CYCLES; i++) {

                checksum = 0x00;
                /*   Write Erase Pages CMD */
                g_bload_buf[0] = BL_ERASE_MEM_NS;
                g_bload_buf[1] = ~(BL_ERASE_MEM_NS);

                ret = cam_write(client, g_bload_buf, 2);
                if (ret < 0) {
                        dev_err(&client->dev,"Write Failed \n");
                        return -1;
                }

                /*   Wait for ACK or NACK */
                ret = cam_read(client, g_bload_buf, 1);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                        return -1;
                }

                if (g_bload_buf[0] != RESP_ACK) {
                        /*   NACK Received */
                        dev_err(&client->dev," NACK Received... exiting.. \n");
                        return -1;
                }

                g_bload_buf[0] = (MAX_PAGES - 1) >> 8;
                g_bload_buf[1] = (MAX_PAGES - 1) & 0xFF;
                g_bload_buf[2] = g_bload_buf[0] ^ g_bload_buf[1];

                ret = cam_write(client, g_bload_buf, 3);
                if (ret < 0) {
                        dev_err(&client->dev,"Write Failed \n");
                        return -1;
                }

                /*   Wait for ACK or NACK */
                ret = cam_read(client, g_bload_buf, 1);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                        return -1;
                }

                if (g_bload_buf[0] != RESP_ACK) {
                        /*   NACK Received */
                        dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
                        return -1;
                }

                for (pagenum = 0; pagenum < MAX_PAGES; pagenum++) {
                        g_bload_buf[(2 * pagenum)] =
                            (pagenum + (i * MAX_PAGES)) >> 8;
                        g_bload_buf[(2 * pagenum) + 1] =
                            (pagenum + (i * MAX_PAGES)) & 0xFF;
                        checksum =
                            checksum ^ g_bload_buf[(2 * pagenum)] ^
                            g_bload_buf[(2 * pagenum) + 1];
                }
                g_bload_buf[2 * MAX_PAGES] = checksum;

                ret = cam_write(client, g_bload_buf, (2 * MAX_PAGES) + 1);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                        return -1;
                }

 poll_busy:
                /*   Wait for ACK or NACK */
                ret = cam_read(client, g_bload_buf, 1);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                        return -1;
                }

                if (g_bload_buf[0] == RESP_BUSY)
                        goto poll_busy;

                if (g_bload_buf[0] != RESP_ACK) {
                        /*   NACK Received */
                        dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
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

        for (i = 0; i < len; i++) {
                crc ^= buf[i];
        }

        return crc;
}
unsigned char cam_bload_inv_errorcheck(unsigned char *buf, int len)
{
        unsigned int checksum = 0x00;
        int i = 0;

        if (!buf || !(buf + len))
                return 0;

        for (i = 0; i < len; i++) {
                checksum = (checksum + buf[i]);
        }

        checksum &= (0xFF);
        return (~(checksum) + 1);
}

int cam_bload_parse_send_cmd(struct i2c_client *client,
                   unsigned char *bytearray, int rec_len,
		   unsigned short int *orig_crc16)
{
        IHEX_RECORD *ihex_rec = NULL;
        unsigned char checksum = 0, calc_checksum = 0;
        int i = 0, ret = 0;

        if (!bytearray)
                return -1;

        ihex_rec = (IHEX_RECORD *) bytearray;
        ihex_rec->addr = htons(ihex_rec->addr);

        checksum = bytearray[rec_len - 1];

        calc_checksum = cam_bload_inv_errorcheck(bytearray, rec_len - 1);
        if (checksum != calc_checksum) {
                dev_err(&client->dev," Invalid Checksum 0x%02x != 0x%02x !! \n",
				checksum, calc_checksum);
                return -1;
        }

        /*   TODO: send I2C Commands to Write */
        if ((ihex_rec->rectype == REC_TYPE_ELA) && (ihex_rec->addr == 0x0000) &&
            (ihex_rec->datasize = 0x02)) {
                /*   Upper 32-bit configuration */
                g_bload_flashaddr = (ihex_rec->recdata[0] <<
                                                        24) | (ihex_rec->
                                                               recdata[1]
                                                               << 16);
        } else if (ihex_rec->rectype == REC_TYPE_DATA) {
                /*   Flash Data into Flashaddr */

                g_bload_flashaddr =
                    (g_bload_flashaddr & 0xFFFF0000) | (ihex_rec->addr);
                *orig_crc16 ^=
                    cam_bload_calc_crc16(ihex_rec->recdata, ihex_rec->datasize);

                /*   Write Erase Pages CMD */
                g_bload_buf[0] = BL_WRITE_MEM_NS;
                g_bload_buf[1] = ~(BL_WRITE_MEM_NS);

                ret = cam_write(client, g_bload_buf, 2);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                        return -1;
                }

                /*   Wait for ACK or NACK */
                ret = cam_read(client, g_bload_buf, 1);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                        return -1;
                }

                if (g_bload_buf[0] != RESP_ACK) {
                        /*   NACK Received */
                        dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
                        return -1;
                }

                g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
                g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
                g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
                g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
                g_bload_buf[4] =
                    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^
                    g_bload_buf[3];

                ret = cam_write(client, g_bload_buf, 5);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                        return -1;
                }

                /*   Wait for ACK or NACK */
                ret = cam_read(client, g_bload_buf, 1);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                        return -1;
                }

                if (g_bload_buf[0] != RESP_ACK) {
                        /*   NACK Received */
                        dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
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
                        dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                        return -1;
                }

 poll_busy:
                /*   Wait for ACK or NACK */
                ret = cam_read(client, g_bload_buf, 1);
                if (ret < 0) {
                        dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                        return -1;
                }

                if (g_bload_buf[0] == RESP_BUSY)
                        goto poll_busy;

                if (g_bload_buf[0] != RESP_ACK) {
                        /*   NACK Received */
                        dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
                        return -1;
                }

        } else if (ihex_rec->rectype == REC_TYPE_SLA) {
                /*   Update Instruction pointer to this address */

        } else if (ihex_rec->rectype == REC_TYPE_EOF) {
                /*   End of File - Issue I2C Go Command */
                return 0;
        } else {

                /*   Unhandled Type */
                dev_err(&client->dev,"Unhandled Command Type \n");
                return -1;
        }

        return 0;
}
int cam_bload_read_fw(struct i2c_client *client,
			unsigned short int *orig_crc16)
{
        /* exclude NULL character at end of string */
	unsigned long hex_file_size = strlen(cam_fw_buf) - 1;
        unsigned char wbuf[MAX_BUF_LEN];
        int i = 0, recindex = 0, ret = 0;

	dev_info(&client->dev,"Flashing camera firmware...\n");

        for (i = 0; i < hex_file_size; i++) {
                if ((recindex == 0) && (cam_fw_buf[i] == ':')) {
                } else if (cam_fw_buf[i] == CR) {
				} else if (cam_fw_buf[i] ==
						'"' || cam_fw_buf[i] =='\\' ||
						cam_fw_buf[i] == 'n') {
                } else if (cam_fw_buf[i] == LF) {
                        if (recindex == 0) {
                                break;
                        }

                        /*   Analyze Packet and Send Commands */
                        ret = cam_bload_parse_send_cmd(client, wbuf, recindex,
					orig_crc16);
                        if (ret < 0) {
                                dev_err(&client->dev,"Error in Processing Commands \n");
                                break;
                        }

                        recindex = 0;

                } else {
                        /*   Parse Rec Data */
                        if ((ret = cam_bload_ascii2hex(cam_fw_buf[i])) < 0) {
                                dev_err(&client->dev,
						"Invalid Character - 0x%02x !! \n", cam_fw_buf[i]);
                                break;
                        }

                        wbuf[recindex] = (0xF0 & (ret << 4));
                        i++;

                        if ((ret = cam_bload_ascii2hex(cam_fw_buf[i])) < 0) {
                                dev_err(&client->dev,"Invalid Character - 0x%02x !!!! \n",
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
                dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                return -1;
        }

        /*   Wait for ACK or NACK */
        ret = cam_read(client, g_bload_buf, 1);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                return -1;
        }

        if (g_bload_buf[0] != RESP_ACK) {
                /*   NACK Received */
                dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
                return -1;
        }

        g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
        g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
        g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
        g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
        g_bload_buf[4] =
            g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

        ret = cam_write(client, g_bload_buf, 5);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                return -1;
        }

        /*   Wait for ACK or NACK */
        ret = cam_read(client, g_bload_buf, 1);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                return -1;
        }

        if (g_bload_buf[0] != RESP_ACK) {
                /*   NACK Received */
                dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
                return -1;
        }

        g_bload_buf[0] = len - 1;
        g_bload_buf[1] = ~(len - 1);

        ret = cam_write(client, g_bload_buf, 2);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                return -1;
        }

        /*   Wait for ACK or NACK */
        ret = cam_read(client, g_bload_buf, 1);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                return -1;
        }

        if (g_bload_buf[0] != RESP_ACK) {
                dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
                return -1;
        }

        ret = cam_read(client, bytearray, len);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
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

                if (cam_bload_read
                    (client, flash_addr + i, bytearray, FLASH_READ_LEN) < 0) {
                        dev_err(&client->dev," i2c_bload_read FAIL !! \n");
                        return -1;
                }

                calc_crc ^= cam_bload_calc_crc16(bytearray, FLASH_READ_LEN);
                i += FLASH_READ_LEN;
        }

        if ((FLASH_SIZE - i) > 0) {
                memset(bytearray, 0x0, FLASH_READ_LEN);

                if (cam_bload_read
                    (client, flash_addr + i, bytearray, (FLASH_SIZE - i))
                    < 0) {
                        dev_err(&client->dev," i2c_bload_read FAIL !! \n");
                        return -1;
                }

                calc_crc ^= cam_bload_calc_crc16(bytearray, FLASH_READ_LEN);
        }

        if (orig_crc != calc_crc) {
                dev_err(&client->dev,
				"CRC verification fail !! 0x%04x != 0x%04x \n", orig_crc, calc_crc);
                return -1;
        }

        dev_info(&client->dev,
			"CRC Verification Success 0x%04x == 0x%04x \n", orig_crc, calc_crc);

        return 0;
}
int cam_bload_go(struct i2c_client *client)
{
        int ret = 0;

        g_bload_buf[0] = BL_GO;
        g_bload_buf[1] = ~(BL_GO);

        ret = cam_write(client, g_bload_buf, 2);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                return -1;
        }

        ret = cam_read(client, g_bload_buf, 1);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                return -1;
        }

        /*   Start Address */
        g_bload_buf[0] = (FLASH_START_ADDRESS & 0xFF000000) >> 24;
        g_bload_buf[1] = (FLASH_START_ADDRESS & 0x00FF0000) >> 16;
        g_bload_buf[2] = (FLASH_START_ADDRESS & 0x0000FF00) >> 8;
        g_bload_buf[3] = (FLASH_START_ADDRESS & 0x000000FF);
        g_bload_buf[4] =
            g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

        ret = cam_write(client, g_bload_buf, 5);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Write Failed \n", __func__, __LINE__);
                return -1;
        }

        ret = cam_read(client, g_bload_buf, 1);
        if (ret < 0) {
                dev_err(&client->dev,"%s (%d) - Read Failed \n", __func__, __LINE__);
                return -1;
        }

        if (g_bload_buf[0] != RESP_ACK) {
                /*   NACK Received */
                dev_err(&client->dev,"%s (%d) - NACK Received... exiting..\n", __func__, __LINE__);
                return -1;
        }

        return 0;
}

static int cam_fw_update(struct i2c_client *client, unsigned char *cam_fw_version)
{
	int ret = 0;
	unsigned short int bload_crc16 = 0;

	ret = cam_bload_get_version(client);
	if (ret < 0) {
		dev_err(&client->dev," Error in Get Version \n");
		goto exit;
	}

	/* Erase firmware present in the MCU and flash new firmware*/
        ret = cam_bload_erase_flash(client);
        if (ret < 0) {
                dev_err(&client->dev," Error in Erase Flash \n");
                goto exit;
        }

        if (cam_bload_read_fw(client,&bload_crc16) < 0) {
                dev_err(&client->dev," verify_flash FAIL !! \n");
                goto exit;
        }

        if (cam_bload_verify_flash(client, bload_crc16) < 0) {
                dev_err(&client->dev," verify_flash FAIL !! \n");
                goto exit;
        }

	/* Reverting from bootloader mode */
        if (cam_bload_go(client) < 0) {
                dev_err(&client->dev," i2c_bload_go FAIL !! \n");
                goto exit;
        }
	dev_info(&client->dev,"(%s) - Firware Updated - (%.32s)\n",
			__func__,cam_fw_version);
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
	if (err !=0 ) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
				__func__, __LINE__, err);
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FW_UPDT;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
			__func__, __LINE__, err);
		goto exit;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&priv->mutex);
	return err;

}

static int cam_lane_configuration(struct i2c_client *client, struct imx568 *priv)
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
	err = cam_write(client, mc_data, payload_len+3);

        if (err != 0) {
                dev_err(&client->dev," %s(%d) MCU Set Ctrl Error - %d \n",
				__func__, __LINE__, err);
                ret = -1;
                goto exit;
        }

	while (--retry > 0) {
		yield();
                cmd_id = CMD_ID_LANE_CONFIG;
                if (cam_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
                        dev_err(&client->dev,
					" %s(%d) MCU Get CMD Status Error \n",
					__func__, __LINE__);
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
                           "(%s) %d MCU Get CMD Error STATUS = 0x%04x "
			   "RET = 0x%02x\n", __func__, __LINE__, cmd_status, retcode);
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
                          ISP_CTRL_INFO * cam_ctrl_info)
{
        /* MCU communication variables */
        unsigned char mc_data[100];
        unsigned char mc_ret_data[100];
        uint32_t payload_len = 0;
        uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
        uint16_t index = 0;
        int ret = 0, err =0,i;
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
                msleep(1);

                mc_data[0] = CMD_SIGNATURE;
                mc_data[1] = CMD_ID_GET_CTRL_INFO;
                mc_data[2] = index >> 8;
                mc_data[3] = index & 0xFF;
                mc_data[4] = errorcheck(&mc_data[2], 2);
                err = cam_write(client, mc_data, 5);
                msleep(1);
                if (err != 0) {
                        dev_err(&client->dev," %s(%d) MCU CMD ID CTRLS Write "
					"Error - %d \n", __func__, __LINE__, err);
                        ret = -1;
                        goto exit;
                }

                err = cam_read(client, mc_ret_data, RX_LEN_PKT);
                msleep(1);
                if (err != 0) {
                        dev_err(&client->dev," %s(%d) MCU CMD ID List Ctrls "
					"Error - %d \n", __func__, __LINE__, err);
                        ret = -1;
                        goto exit;
                }

                /* Verify CRC */
                orig_crc = mc_ret_data[4];
                calc_crc = errorcheck(&mc_ret_data[2], 2);
                if (orig_crc != calc_crc) {
                        dev_err(&client->dev,
					" %s(%d) MCU CMD ID List Ctrls Error CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
                        ret = -1;
                        goto exit;
                }

                if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
                        priv->num_ctrls = index;
                        break;
                }

                payload_len =
                    ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
                    HEADER_FOOTER_SIZE;
                errcode = mc_ret_data[5];
                if (errcode != ERRCODE_SUCCESS) {
                        dev_err(&client->dev,
					" %s(%d) MCU CMD ID List Ctrls Errcode - 0x%02x \n",
					__func__, __LINE__, errcode);
                        ret = -1;
                        goto exit;
                }

                memset(mc_ret_data, 0x00, payload_len);
                err = cam_read(client, mc_ret_data, payload_len);
                if (err != 0) {
                       dev_err(&client->dev,
				       " %s(%d) MCU CMD ID List Ctrls Read Error - %d \n",
				       __func__, __LINE__, err);
                        ret = -1;
                        goto exit;
                }

                /* Verify CRC */
                orig_crc = mc_ret_data[payload_len - 2];
                calc_crc =
                    errorcheck(&mc_ret_data[2],
                                 payload_len - HEADER_FOOTER_SIZE);
                if (orig_crc != calc_crc) {
                        dev_err(&client->dev,
					" %s(%d) MCU CMD ID List Ctrls Error CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
                        ret = -1;
                        goto exit;
                }

                /* Verify Errcode */
                errcode = mc_ret_data[payload_len - 1];
                if (errcode != ERRCODE_SUCCESS) {
                        dev_err(&client->dev,
					" %s(%d) MCU CMD ID List Ctrls Errcode - 0x%02x \n",
					__func__, __LINE__, errcode);
                        ret = -1;
                        goto exit;
                }
		if(cam_ctrl_info != NULL) {
			/* append ctrl info in array */
			cam_ctrl_info[index].ctrl_id =
			    mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
			    << 8 | mc_ret_data[5];
			cam_ctrl_info[index].ctrl_type = mc_ret_data[6];
			switch (cam_ctrl_info[index].ctrl_type) {
				case CTRL_STANDARD:
				        cam_ctrl_info[index].ctrl_data.std.ctrl_min =
				            mc_ret_data[7] << 24 | mc_ret_data[8] << 16 |
				            mc_ret_data[9] << 8 | mc_ret_data[10];

				        cam_ctrl_info[index].ctrl_data.std.ctrl_max =
				            mc_ret_data[11] << 24 | mc_ret_data[12] << 16 |
				            mc_ret_data[13]
				            << 8 | mc_ret_data[14];

				        cam_ctrl_info[index].ctrl_data.std.ctrl_def =
				            mc_ret_data[15] << 24 | mc_ret_data[16] << 16 |
				            mc_ret_data[17]
				            << 8 | mc_ret_data[18];

				        cam_ctrl_info[index].ctrl_data.std.ctrl_step =
				            mc_ret_data[19] << 24 | mc_ret_data[20] << 16 |
				            mc_ret_data[21]
				            << 8 | mc_ret_data[22];
				        break;

				case CTRL_EXTENDED:
					cam_ctrl_info[index].ctrl_data.ext.val_type = mc_ret_data[7];
					cam_ctrl_info[index].ctrl_data.ext.val_length =
						mc_ret_data[8] << 24 | mc_ret_data[9] << 16 |
							mc_ret_data[10] << 8 | mc_ret_data[11];
					for(i = 0 ; i < cam_ctrl_info[index].ctrl_data.ext.val_length ; i++)
						cam_ctrl_info[index].ctrl_data.ext.val_data[i] = mc_ret_data[12+i];
					if (cam_ctrl_info[index].ctrl_data.ext.val_type ==
							V4L2_CTRL_TYPE_INTEGER64) {
						for (i = 0; i < EXTENDED_CTRL_SIZE; i++) {
							cam_ctrl_info[index].ctrl_data.ext.ctrl_min |=
								cam_ctrl_info[index].ctrl_data.ext.val_data[i] << 8 * (7-i);
							cam_ctrl_info[index].ctrl_data.ext.ctrl_max |=
								cam_ctrl_info[index].ctrl_data.ext.val_data[8+i] << 8 * (7-i);
							cam_ctrl_info[index].ctrl_data.ext.ctrl_def |=
								cam_ctrl_info[index].ctrl_data.ext.val_data[16+i] << 8 * (7-i);
							cam_ctrl_info[index].ctrl_data.ext.ctrl_step |=
								cam_ctrl_info[index].ctrl_data.ext.val_data[24+i] << 8 * (7-i);
						}
					} else if(cam_ctrl_info[index].ctrl_data.ext.val_type ==
							V4L2_CTRL_TYPE_STRING) {
						for (i = 0; i < EXTENDED_CTRL_SIZE; i++) {
							cam_ctrl_info[index].ctrl_data.ext.ctrl_min |=
								cam_ctrl_info[index].ctrl_data.ext.val_data[i] << 8 * (7-i);
							cam_ctrl_info[index].ctrl_data.ext.ctrl_max |=
								cam_ctrl_info[index].ctrl_data.ext.val_data[8+i] << 8 * (7-i);
							cam_ctrl_info[index].ctrl_data.ext.ctrl_step |=
								cam_ctrl_info[index].ctrl_data.ext.val_data[24+i] << 8 * (7-i);
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
			ISP_STREAM_INFO *stream_info, int *frm_fmt_size)
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
                msleep(1);

                mc_data[0] = CMD_SIGNATURE;
                mc_data[1] = CMD_ID_GET_STREAM_INFO;
                mc_data[2] = index >> 8;
                mc_data[3] = index & 0xFF;
                mc_data[4] = errorcheck(&mc_data[2], 2);
                err = cam_write(client, mc_data, 5);
                msleep(1);
                if (err != 0) {
                        dev_err(&client->dev,
					" %s(%d) i2c error while writing command to MCU -%d \n",
					__func__, __LINE__, err);
                        ret = -1;
                        goto exit;
                }

                err = cam_read(client, mc_ret_data, RX_LEN_PKT);
                msleep(1);
                if (err != 0) {
                        dev_err(&client->dev,
					" %s(%d) i2c error while reading stream info. length from MCU - %d \n",
					__func__, __LINE__, err);
                        ret = -1;
                        goto exit;
                }

                /* Verify CRC */
                orig_crc = mc_ret_data[4];
                calc_crc = errorcheck(&mc_ret_data[2], 2);
                if (orig_crc != calc_crc) {
                        dev_err(&client->dev,
					" %s(%d)Checksum' mismatch in  MCU provided stream info. length: "
					"0x%02x != 0x%02x \n", __func__, __LINE__, orig_crc, calc_crc);
                        ret = -1;
                        goto exit;
                }
                if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			if(stream_info == NULL) {
				*frm_fmt_size = index;
				priv->frm_fmt_size = index;
			} else {
				*frm_fmt_size = mode;
				priv->frm_fmt_size = mode;
			}
                        break;
                }

                payload_len =
                    ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
                    HEADER_FOOTER_SIZE;
                errcode = mc_ret_data[5];
                if (errcode != ERRCODE_SUCCESS) {
                        dev_err(&client->dev,
					" %s(%d) MCU's return code has error set - 0x%02x \n",
					__func__, __LINE__, errcode);
                        ret = -1;
                        goto exit;
                }

                memset(mc_ret_data, 0x00, payload_len);
                err = cam_read(client, mc_ret_data, payload_len);
                msleep(1);
                if (err != 0) {
                        dev_err(&client->dev,
					" %s(%d) i2c error while reading actual stream info. - %d \n",
					__func__, __LINE__, err);
                        ret = -1;
                        goto exit;
                }

                /* Verify CRC */
                orig_crc = mc_ret_data[payload_len - 2];
                calc_crc =
                    errorcheck(&mc_ret_data[2],
                                 payload_len - HEADER_FOOTER_SIZE);
                if (orig_crc != calc_crc) {
                        dev_err(&client->dev,
					" %s(%d) Checksum' mismatch error in MCU provided stream info. : "
					"0x%02x != 0x%02x \n", __func__, __LINE__, orig_crc, calc_crc);
                        ret = -1;
                        goto exit;
                }

                /* Verify Errcode */
                errcode = mc_ret_data[payload_len - 1];
                if (errcode != ERRCODE_SUCCESS) {
                        dev_err(&client->dev,
					" %s(%d) MCU's response has errcode set - 0x%02x \n",
					__func__, __LINE__, errcode);
                        ret = -1;
                        goto exit;
                }
		if(stream_info != NULL) {
			/* check if any other format than UYVY is queried - do not append in array */
			stream_info->fmt_fourcc =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
				<< 8 | mc_ret_data[5];
			if(index == 0)
				default_fmt_fourcc = stream_info->fmt_fourcc;
			stream_info->width = mc_ret_data[6] << 8 | mc_ret_data[7];
			stream_info->height = mc_ret_data[8] << 8 | mc_ret_data[9];
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
							" The Stream format at index 0x%04x has FRAME_RATE_CONTINOUS,"
							"which is unsupported !! \n", index);

					continue;
			}
			switch (stream_info->fmt_fourcc){
				case V4L2_PIX_FMT_SRGGB12:
				case V4L2_PIX_FMT_SGBRG12:
				case V4L2_PIX_FMT_SGBRG10:
				case V4L2_PIX_FMT_SRGGB10:
				case V4L2_PIX_FMT_SRGGB8:
					priv->cam_frmfmt[mode].size.width = stream_info->width;
					priv->cam_frmfmt[mode].size.height =
						stream_info->height;
					num_frates = priv->cam_frmfmt[mode].num_framerates;

					*((int *)(priv->cam_frmfmt[mode].framerates)+num_frates) =
						(int)(stream_info->frame_rate.disc.frame_rate_num /
								stream_info->frame_rate.disc.frame_rate_denom);
					priv->cam_frmfmt[mode].num_framerates++;
					priv->cam_frmfmt[mode].mode = mode;
					priv->streamdb[index] = mode;
					priv->cam_frmfmt[mode].fourcc = stream_info->fmt_fourcc;
#ifdef EN_DEBUG_PRINTS
					dev_info(&client->dev, "stream mode : %d width : %d height : %d framerate : %d\n",
							priv->cam_frmfmt[mode].mode, stream_info->width, stream_info->height,
							priv->cam_frmfmt[mode].num_framerates);
					dev_info(&client->dev, "stream_info->frame_rate.disc.frame_rate_num = %d --------------\n",
							stream_info->frame_rate.disc.frame_rate_num);
#endif

					if ((stream_info->fmt_fourcc == V4L2_PIX_FMT_SRGGB12) ||
							(stream_info->fmt_fourcc == V4L2_PIX_FMT_SGBRG12)) {
						priv->cam_frmfmt[mode].hmax = 965;
						priv->cam_frmfmt[mode].vmax = 1192;
					}
					mode++;
					break;

				default:
					dev_err(&client->dev,
							" The Stream format at index 0x%04x has format 0x%08x ,"
							"which is unsupported \nSupported Formats are 0x%08x and 0x%08x!! \n",
							index, stream_info->fmt_fourcc, V4L2_PIX_FMT_SRGGB12,
							V4L2_PIX_FMT_SRGGB10);
			}

		}

        }

 exit:
        /* unlock semaphore */
        mutex_unlock(&priv->mutex);

        return ret;

}

/* --------- Camera Module Initialization Process ---------- */

int cam_core_initialize(struct imx568 *priv)
{
        struct i2c_client *client = priv->i2c_client;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	unsigned char fw_version[32] = {0}, bin_fw_version[32] = {0};
	int ret, loop, err = 0, pwdn_gpio_toggle = 0, retry = 5;
	int frm_fmt_size = 0;
	uint32_t lanes = 0;

	ret = of_property_read_u32(node, "camera_mipi_lanes", &lanes);
	if (ret < 0) {
	    dev_err(dev, "Error in getting Camera MIPI Lanes\n");
	    return -EINVAL;
	}
	priv->mipi_lane_config = lanes;

	// Read MCU firmware bin name from device tree
	ret = of_property_read_string(node, "cam_fw_name",&cam_fw_name);

	if (ret) {
                dev_err(dev, "Unable to get cam firmware name from the Device tree\n");
		return -EINVAL;
	}

	// Check if the CAM firmware is loaded or not
	// If not loaded already, load the CAM firmware
	if (!is_fw_loaded) {
		if (ecam_firmware_load(client) < ERRCODE_SUCCESS) {
			dev_err(dev, "Failed to load cam firmware\n");
			return -ENOENT;
		} else {
#ifdef EN_DEBUG_PRINTS
			dev_info (dev, "Firmware Load Success\n");
#endif
			is_fw_loaded = 1;
		}
	}

	// MCU Reset Sequence
	toggle_gpio_mcu(priv->reset_gpio, 0);
	toggle_gpio_mcu(priv->boot_gpio, 0);
	msleep(1);
	toggle_gpio_mcu(priv->reset_gpio, 1);
	msleep(100); // Delay required to boot the MCU

	if ((ret = is_fw_update_required(client, priv, fw_version, bin_fw_version)) != 0) {
		if (ret > 0) {
			if((err = cam_jump_bload(client, priv)) < 0) {
				dev_err(dev," Cannot go into bootloader mode\n");
				return -EIO;
			}
			msleep(100);
		} else {
			/* ret value has to be -1 */
#ifdef EN_DEBUG_PRINTS
			dev_info(dev," Switching MCU to Bootloader mode \n");
#endif
		}

		ret = cam_bload_get_version(client);
		if (ret < 0) {
			dev_err(dev," Error in Get Version \n");
			/* Since error in reading the bootloader version: set MCU to bootloader mode */

			toggle_gpio_mcu(priv->reset_gpio, 0);
			toggle_gpio_mcu(priv->boot_gpio, 1);
			msleep(1);
			toggle_gpio_mcu(priv->reset_gpio, 1);
			msleep(10);

			/* Reading the MCU Firmware version from bootloader mode */
			for(loop = 0;loop < MAX_ATTEMPTS; loop++) {
				ret = cam_bload_get_version(client);
				if (ret < 0) {
					dev_err(dev, "Error getting Firmware version.. Retrying...\n");
					msleep(1000);
					continue;
				} else {
					break;
				}
			}

			/* Failed reading FW version in bootloader mode even after MAX_attempts. Return Failure */
			if (loop == MAX_ATTEMPTS) {
				dev_err(dev, "%s (%d) Error in reading MCU FW version"
						"in bootloader mode also. Exiting. \n", __func__, __LINE__);
				return -EINVAL;
			}
		}

		/*Attempt Firmware Update */
		if (cam_fw_update(client,bin_fw_version) < 0) {
			dev_err(dev, "%s (%d) Error Updating MCU FW. Exiting. \n", __func__, __LINE__);
			return -EFAULT;
		}

		// MCU Reset Sequence
		toggle_gpio_mcu(priv->reset_gpio, 0);
		toggle_gpio_mcu(priv->boot_gpio, 0);
		msleep(1);
		toggle_gpio_mcu(priv->reset_gpio, 1);
		msleep(200); // Delay required to boot the MCU

	} else {
		/* Same Firmware version in MCU and bin file */
		dev_info(dev,"Cam Firmware Version - (%.32s)\n",
				fw_version);
	}

	/* Configure MIPI Lanes of the Sensor */
	retry = 5;
	while (--retry > 0) {
		if (cam_lane_configuration(client, priv) < 0) {
			dev_err(dev, "%s, Failed to set lane CONFIG Data. retrying!\n",__func__);
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to set lane CONFIG Data!\n",__func__);
		return -EFAULT;
	}

	/* Query the number of controls from MCU */
	retry = 5;
	while (--retry > 0) {
		if(cam_list_ctrls(client, priv, NULL) < 0) {
			dev_err(dev,"%s, init controls failure. retrying\n",__func__);
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to init controls!\n",__func__);
		return -EFAULT;
	}

	priv->cam_ctrl_info = devm_kzalloc(dev,
			sizeof(ISP_CTRL_INFO) * priv->num_ctrls, GFP_KERNEL);
	if(!priv->cam_ctrl_info) {
		dev_err(dev,"Unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->ctrldb = devm_kzalloc(dev,
			sizeof(uint32_t) * priv->num_ctrls , GFP_KERNEL);
	if (!priv->ctrldb) {
		dev_err(dev,"Unable to allocate memory!\n");
		return -ENOMEM;
	}

	/* Fill the controls */
	retry = 5;
	while (--retry > 0) {
		if(cam_list_ctrls(client, priv, priv->cam_ctrl_info) < 0) {
			dev_err(dev,"%s, Failed to init controls\n",__func__);
		} else {
#ifdef EN_DEBUG_PRINTS
			dev_info(dev, "Num of Controls - %d\n", priv->num_ctrls);
#endif
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to init formats!\n",__func__);
		return -EFAULT;
	}

	/* Query the number of formats available from MCU */
	retry = 5;
	while (--retry > 0) {
		if(cam_list_fmts(client, priv, NULL, &frm_fmt_size) < 0) {
			dev_err(dev,"%s, Failed to init formats\n",__func__);
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to init formats!\n",__func__);
		return -EFAULT;
	}

	priv->nr_supported_formats = frm_fmt_size;
	priv->stream_info = devm_kzalloc (dev,
			sizeof(ISP_STREAM_INFO) * (frm_fmt_size + 1), GFP_KERNEL);
	priv->streamdb = devm_kzalloc(dev, sizeof(int) * (frm_fmt_size + 1), GFP_KERNEL);
	if(!priv->streamdb ) {
		dev_err(dev, "unable to allocate memory\n");
		return -ENOMEM;
	}

	priv->cam_frmfmt = devm_kzalloc(dev,
			sizeof(struct camera_common_frmfmt) * (frm_fmt_size + 1) ,GFP_KERNEL);
	if(!priv->cam_frmfmt ) {
		dev_err(dev,"Unable to allocate memory\n");
		return -ENOMEM;
	}

	/* Initialise the ISP */
	if (cam_init(client) < 0) {
                dev_err(dev, "Unable to INIT ISP \n");
                return -EFAULT;
        }

	for (loop = 0; loop <= (frm_fmt_size); loop++) {
		/* create Frame Rate array */
		priv->cam_frmfmt[loop].framerates = devm_kzalloc (dev,
				sizeof(int) * MAX_NUM_FRATES, GFP_KERNEL);
		if (!priv->cam_frmfmt[loop].framerates) {
			dev_err(dev,"Unable to create memory\n");
			return -ENOMEM;
		}
	}

	/* List the formats from MCU */
	retry = 5;
	while (--retry > 0) {
		if (cam_list_fmts(client, priv, priv->stream_info, &frm_fmt_size) < 0) {
	                dev_err(dev, "Unable to List Fmts. retrying! \n");
			continue;
	        } else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(dev, "%s, Failed to List formats!\n",__func__);
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
	uint16_t size = 0, send_len =0, payload_len = 0;
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
		dev_err(&client->dev," %s(%d) MCU Write Error - %d \n",
				__func__,__LINE__, err);
		goto exit;
	}

	memset(mc_ret_data, 0 ,512);
	err = cam_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Error - %d \n",
				__func__,__LINE__, err);
		goto exit;
	}

	send_len = (mc_ret_data[2] << 8) | mc_ret_data[3];

	payload_len = send_len + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0 ,512);
	err = cam_read(client, mc_ret_data,
			send_len + HEADER_FOOTER_SIZE);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Error - %d \n",
				__func__,__LINE__, err);
		goto exit;
	}

	reg_val = mc_ret_data[4] << 8;
	reg_val = reg_val | mc_ret_data[5];

	retcode = mc_ret_data [payload_len - 1];

	if (retcode != ERRCODE_SUCCESS) {
		dev_err (&client->dev, "Error read %d\n", __LINE__);
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
	uint16_t size = 0, send_len =0, payload_len = 0;
	int err = 0;

	/* lock semaphore */
	mutex_lock(&priv->mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_WRITE;
	mc_data[2] = 0x00;
	mc_data[3] = 0x05;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);
	msleep(1);

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
		dev_err(&client->dev," %s(%d) MCU Write Error - %d \n"
				, __func__,__LINE__, err);
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

	ret = imx568_get_regulators(imx568);
	if (ret) {
		dev_err(dev, "Regulators not found  %d\n", ret);
	}

	/* Request cam reset pin */
	imx568->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx568->boot_gpio))
        return dev_err_probe(dev, PTR_ERR(imx568->reset_gpio), "Failed to get reset GPIO\n");

	/* Request cam boot pin */
	imx568->boot_gpio = devm_gpiod_get(dev, "boot", GPIOD_OUT_LOW);
	if (IS_ERR(imx568->boot_gpio))
        return dev_err_probe(dev, PTR_ERR(imx568->boot_gpio), "Failed to get boot GPIO\n");

	ret = imx568_power_on(dev);
	if (ret)
		return ret;

	// Initialize Mutex
	mutex_init(&imx568->mutex);

	// Camera Initialization
	ret = cam_core_initialize(imx568);
	if (ret)
		goto error_power_off;

	msleep(10);

	retry = 5;
	while (retry-- > 0) {
		// Sensor register Write
		ret = sensor_reg_write(client, imx568, 0x3000, 0x00);
		if (ret)
			continue;
	
		msleep (20);
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

	if (retry < 0){
		ret = -EINVAL;
		goto error_power_off;
	}

	dev_dbg(dev, "IMX568 Sensor ID: %d\n", sensor_id);

	if (sensor_id != IMX568_SENSOR_ID) {
		dev_err (dev, "The Connected camera is not IMX568 - %d != 568\n", sensor_id);
		ret = -EINVAL;
		goto error_power_off;
	}

	/* Initialize default format */
	imx568_set_default_format(imx568);

	// Set exposure to 5ms
	retry = 5;
	while (retry -- > 0) {
		if ((ret = cam_set_exposure (client, imx568, DEFAULT_EXPOSURE)) < 0)
			continue;
		else
			break;
	}

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
	imx568->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

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

#define FREE_SAFE(dev, ptr) \
	if(ptr) { \
		devm_kfree(dev, ptr); \
	}
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
		release_firmware (cam_fw);
		cam_fw = NULL;

		if (cam_fw_buf != NULL) {
			kfree (cam_fw_buf);
			cam_fw_buf = NULL;
		}

		is_fw_loaded = 0;
#ifdef EN_DEBUG_PRINTS
		dev_info (&client->dev, "Firmware Release Success\n");
#endif
	}

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx568_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	/* Free up memory */

	FREE_SAFE(&client->dev, imx568->cam_ctrl_info);

	for(loop = 0; loop < imx568->frm_fmt_size; loop++ ) {
		FREE_SAFE(&client->dev, (void *)imx568->cam_frmfmt[loop].framerates);
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
		.name = "imx568",
		.of_match_table	= imx568_dt_ids,
		.pm = &imx568_pm_ops,
	},
	.probe = imx568_probe,
	.remove = imx568_remove,
};

module_i2c_driver(imx568_i2c_driver);

MODULE_AUTHOR("Kishore Kumar <kishore.kumar@e-consystems.com>");
MODULE_DESCRIPTION("e-con camera driver");
MODULE_LICENSE("GPL v2");
