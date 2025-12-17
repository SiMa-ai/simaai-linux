// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX678 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h>
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
#include "econ_imx678.h"

// Define EN_DEBUG_PRINTS Macro to enable debug prints
//#define EN_DEBUG_PRINTS

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

static inline struct imx678 *to_imx678(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx678, sd);
}

static void imx678_set_default_format(struct imx678 *imx678)
{
	imx678->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}

static int imx678_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int imx678_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx678 *imx678 =
		container_of(ctrl->handler, struct imx678, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	int ret = 0;

	// Not Implemented
	return ret;
}

static const struct v4l2_ctrl_ops imx678_ctrl_ops = {
	.s_ctrl = imx678_set_ctrl,
};

static int imx678_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx678 *imx678 = to_imx678(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		code->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx678_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx678 *imx678 = to_imx678(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		fse->min_width = imx678->cam_frmfmt[fse->index].size.width;
		fse->max_width = fse->min_width;
		fse->min_height = imx678->cam_frmfmt[fse->index].size.height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		// Not Implemented
		fse->min_width = IMX678_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX678_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx678_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx678_update_image_pad_format(struct imx678 *imx678,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.field = V4L2_FIELD_NONE;
	imx678_reset_colorspace(&fmt->format);
}

static void imx678_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX678_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX678_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx678_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx678 *imx678 = to_imx678(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx678->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx678->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx678_update_image_pad_format(imx678, fmt);
			fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
		} else {
			imx678_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx678->mutex);
	return 0;
}

static int cam_stream_config(struct i2c_client *client, struct imx678 *priv,
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

static int imx678_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	struct imx678 *imx678 = to_imx678(sd);
	int err = 0, ret = 0, i = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx678->mutex);

	if (fmt->pad == IMAGE_PAD) {
		fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
		imx678_update_image_pad_format(imx678, fmt);

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		}
	} else {
		// Not Implemented
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx678_update_metadata_pad_format(fmt);
		}
	}

	switch (fmt->format.code) {
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			imx678->format_fourcc = V4L2_PIX_FMT_SRGGB12;
			break;
	}

	for (i = 0; i < imx678->frm_fmt_size; i++) {
		if ((imx678->cam_frmfmt[i].size.width == fmt->format.width)
				&& (imx678->cam_frmfmt[i].size.height ==
					fmt->format.height)) {
			imx678->frmfmt_mode = imx678->cam_frmfmt[i].mode;
			imx678->frate_index = 0;
			break;
		}
	}

	err = cam_stream_config(imx678->i2c_client, imx678, imx678->format_fourcc,
				imx678->frmfmt_mode, imx678->frate_index);
	if(err != 0) {
		ret = err;
	}

	mutex_unlock(&imx678->mutex);

	return ret;
}

static int cam_stream_on(struct i2c_client *client, struct imx678 *priv)
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

static int cam_stream_off(struct i2c_client *client, struct imx678 *priv)
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
static int imx678_start_streaming(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	int ret;

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx678->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = cam_stream_on(client, imx678);
	if(ret != 0){
		dev_err(&client->dev,"%s (%d) Stream_On - Failed\n", __func__, __LINE__);
	}

	return ret;
}

/* Stop streaming */
static void imx678_stop_streaming(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	int ret;

	ret = cam_stream_off(client, imx678);
	if(ret != 0){
		dev_err(&client->dev,"%s (%d) Stream_Off - Failed\n", __func__, __LINE__);
	}
}

static int imx678_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx678 *imx678 = to_imx678(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx678->mutex);
	if (imx678->streaming == enable) {
		mutex_unlock(&imx678->mutex);
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
		ret = imx678_start_streaming(imx678);
		if (ret)
			goto err_rpm_put;
	} else {
		imx678_stop_streaming(imx678);
		pm_runtime_put(&client->dev);
	}

	imx678->streaming = enable;

	mutex_unlock(&imx678->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx678->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx678_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	ret = regulator_bulk_enable(IMX678_NUM_SUPPLIES,
				    imx678->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx678->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}
	return 0;

reg_off:
	regulator_bulk_disable(IMX678_NUM_SUPPLIES, imx678->supplies);
	return ret;
}

static int imx678_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	regulator_bulk_disable(IMX678_NUM_SUPPLIES, imx678->supplies);
	clk_disable_unprepare(imx678->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx678->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx678_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	if (imx678->streaming)
		imx678_stop_streaming(imx678);

	return 0;
}

static int __maybe_unused imx678_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	if (imx678->streaming) {
		ret = imx678_start_streaming(imx678);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx678_stop_streaming(imx678);
	imx678->streaming = 0;
	return ret;
}

static int imx678_get_regulators(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	unsigned int i;

	for (i = 0; i < IMX678_NUM_SUPPLIES; i++)
		imx678->supplies[i].supply = imx678_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX678_NUM_SUPPLIES,
				       imx678->supplies);
}

static const struct v4l2_subdev_core_ops imx678_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx678_video_ops = {
	.s_stream = imx678_set_stream,
};

static const struct v4l2_subdev_pad_ops imx678_pad_ops = {
	.enum_mbus_code = imx678_enum_mbus_code,
	.get_fmt = imx678_get_pad_format,
	.set_fmt = imx678_set_pad_format,
	.enum_frame_size = imx678_enum_frame_size,
};

static const struct v4l2_subdev_ops imx678_subdev_ops = {
	.core = &imx678_core_ops,
	.video = &imx678_video_ops,
	.pad = &imx678_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx678_internal_ops = {
	.open = imx678_open,
};

/* Initialize control handlers */
static int imx678_init_controls(struct imx678 *imx678)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx678->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, imx678->num_ctrls);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &imx678->mutex;

	// No controls to initialize
	mutex_unlock(&imx678->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx678->mutex);

	return ret;
}

static void imx678_free_controls(struct imx678 *imx678)
{
	v4l2_ctrl_handler_free(imx678->sd.ctrl_handler);
	mutex_destroy(&imx678->mutex);
}

static int imx678_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

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
			imx678->mipi_lane_config = ep_cfg.bus.mipi_csi2.num_data_lanes;
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
	    ep_cfg.link_frequencies[0] != IMX678_DEFAULT_LINK_FREQ) {
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

static const struct of_device_id imx678_dt_ids[] = {
	{ .compatible = "sony,imx678"},
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
static int is_fw_update_required(struct i2c_client *client, struct imx678 *priv,
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

static int cam_jump_bload(struct i2c_client *client, struct imx678 *priv)
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

static int cam_lane_configuration(struct i2c_client *client, struct imx678 *priv)
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

static int cam_list_ctrls(struct i2c_client *client, struct imx678 *priv,
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
static int cam_list_fmts(struct i2c_client *client, struct imx678 *priv,
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
#ifdef EN_DEBUG_PRINTS
					dev_info(&client->dev, "stream mode : %d width : %d height : %d framerate : %d\n",
							priv->cam_frmfmt[mode].mode, stream_info->width, stream_info->height,
							priv->cam_frmfmt[mode].num_framerates);
					dev_info(&client->dev, "stream_info->frame_rate.disc.frame_rate_num = %d --------------\n",
							stream_info->frame_rate.disc.frame_rate_num);
#endif
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

/* --------------------- GPIO Toggling --------------------- */

static void toggle_gpio_mcu(unsigned int gpio, int val)
{
	if (gpio_cansleep(gpio)) {
		gpio_direction_output(gpio,val);
		gpio_set_value_cansleep(gpio, val);
	} else {
		gpio_direction_output(gpio,val);
		gpio_set_value(gpio, val);
	}
}

/* --------- Camera Module Initialization Process ---------- */

int cam_core_initialize(struct imx678 *priv)
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

		// After Firmware do MCU reset
		toggle_gpio_mcu(priv->reset_gpio, 0);
		toggle_gpio_mcu(priv->boot_gpio, 0);
		msleep(1);
		toggle_gpio_mcu(priv->reset_gpio, 1);
		msleep(100); // Wait time for mcu to boot

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

static int imx678_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx678 *imx678;
	const struct of_device_id *match;
	int ret;
	u32 tm_of;

	imx678 = devm_kzalloc(&client->dev, sizeof(*imx678), GFP_KERNEL);
	if (!imx678)
		return -ENOMEM;

	imx678->i2c_client = client;
	v4l2_i2c_subdev_init(&imx678->sd, client, &imx678_subdev_ops);

	match = of_match_device(imx678_dt_ids, dev);
	if (!match)
		return -ENODEV;

	/* Check the hardware configuration in device tree */
	if (imx678_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx678->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx678->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx678->xclk);
	}

	imx678->xclk_freq = clk_get_rate(imx678->xclk);
	if (imx678->xclk_freq != IMX678_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx678->xclk_freq);
		return -EINVAL;
	}

	ret = imx678_get_regulators(imx678);
	if (ret) {
		dev_err(dev, "failed to get regulators  %d\n", ret);
		return ret;
	}

	/* Request cam reset pin */
	imx678->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);

	/* Request cam boot pin */
	imx678->boot_gpio = of_get_named_gpio(node, "boot-gpios", 0);

	ret = imx678_power_on(dev);
	if (ret)
		return ret;

	// Initialize Mutex
	mutex_init(&imx678->mutex);

	// Camera Initialization
	ret = cam_core_initialize(imx678);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx678_set_default_format(imx678);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx678_init_controls(imx678);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx678->sd.internal_ops = &imx678_internal_ops;
	imx678->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx678->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx678->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx678->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx678->sd.entity, NUM_PADS, imx678->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx678->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	dev_info (dev, "Detected IMX678 Camera\n");

	return 0;

error_media_entity:
	media_entity_cleanup(&imx678->sd.entity);

error_handler_free:
	imx678_free_controls(imx678);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx678_power_off(&client->dev);

	// Free GPIOs
	gpio_free(imx678->reset_gpio);
	gpio_free(imx678->boot_gpio);

	return ret;
}

static void imx678_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx678_free_controls(imx678);

	// Free GPIOs
	gpio_free(imx678->reset_gpio);
	gpio_free(imx678->boot_gpio);

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
		imx678_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

MODULE_DEVICE_TABLE(of, imx678_dt_ids);

static const struct dev_pm_ops imx678_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx678_suspend, imx678_resume)
	SET_RUNTIME_PM_OPS(imx678_power_off, imx678_power_on, NULL)
};

static struct i2c_driver imx678_i2c_driver = {
	.driver = {
		.name = "imx678",
		.of_match_table	= imx678_dt_ids,
		.pm = &imx678_pm_ops,
	},
	.probe_new = imx678_probe,
	.remove = imx678_remove,
};

module_i2c_driver(imx678_i2c_driver);

MODULE_AUTHOR("Kishore Kumar <kishore.kumar@e-consystems.com>");
MODULE_DESCRIPTION("e-con camera driver");
MODULE_LICENSE("GPL v2");
