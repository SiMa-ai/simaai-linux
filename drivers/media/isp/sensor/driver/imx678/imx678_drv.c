/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2021 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

//-------------------------------------------------------------------------------------
// STRUCTURE:
//  VARIABLE SECTION:
//      CONTROLS - Dependence from preprocessor
//      DATA     - Modulation
//      RESET    - Reset function
//      MIPI     - MIPI settings
//      FLASH    - Flash support
//  CONSTANT SECTION
//      DRIVER
//-------------------------------------------------------------------------------------
#include "system_stdlib.h"
#include "system_types.h"
#include "acamera_command_api.h"
#include "acamera_logger.h"
#include "acamera_math.h"
#include "sensor_api.h"
#include "imx678_config.h"

#include "acamera_aframe.h"
#include "isp-v4l2.h"
#include "acamera_isp_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_settings.h"

#include <linux/dmaengine.h> 
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <asm/io.h>

// Added to communicate with the MCU I2C
#include "isp-v4l2.h"
#include <linux/i2c.h>

#include "sensor_bus_config.h"
#include <linux/simaai-stu.h>

extern struct simaai_stu *stu;
static struct workqueue_struct *wq;

// Filled in system_cma.c
extern struct platform_device *g_pdev;

#define MAX_RAW_FRAMES			(5)
extern int32_t get_calibrations_imx678( uint32_t wdr_mode, void *param );
extern acamera_settings *get_settings_by_id(u8 ctx_id);
extern  int8_t get_dma_index(uint32_t ctx_id);

// Formatting tool makes below declaration less readable and structured
// clang-format off
static sensor_mode_t supported_modes[] = {
    {
        .fps = 30 * 256,
		.vmax = 2250,
		.hmax = 1100,
        .wdr_mode = WDR_MODE_LINEAR,
        .resolution.width = SENSOR_IMAGE_WIDTH,
        .resolution.height = SENSOR_IMAGE_HEIGHT,
        .channel_info = {
            .channel_desc = {
                {
                    .exposure_bit_width = 12,
                    .data_type = DATA_TYPE_LINEAR,
                    .cv = CAP_CHANNEL_PASS_THROUGH
                }
            },
            .exposure_idx_to_channel_map = {
                0
            },
            .exposure_max_bit_width = 12,
            .locked_exp_info = {
                .locked_exp_ratio_flag = false,
                .locked_exp_ratio_val = 0,
                .locked_exp_ratio_short_flag = false,
                .locked_exp_ratio_short_val = 0,
                .locked_exp_ratio_medium_flag = false,
                .locked_exp_ratio_medium_val = 0,
                .locked_exp_ratio_medium2_flag = false,
                .locked_exp_ratio_medium2_val = 0
            }
        },
        .exposures = 1,
        .num_channels = 1
    },
    {
        .fps = 30 * 256,
		.vmax = 2250,
		.hmax = 1100,
        .wdr_mode = WDR_MODE_LINEAR,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .channel_info = {
            .channel_desc = {
                {
                    .exposure_bit_width = 12,		    
                    .data_type = DATA_TYPE_LINEAR,
                    .cv = CAP_CHANNEL_PASS_THROUGH
                }
            },
            .exposure_idx_to_channel_map = {
                0
            },
            .exposure_max_bit_width = 12,
            .locked_exp_info = {
                .locked_exp_ratio_flag = false,
                .locked_exp_ratio_val = 0,
                .locked_exp_ratio_short_flag = false,
                .locked_exp_ratio_short_val = 0,
                .locked_exp_ratio_medium_flag = false,
                .locked_exp_ratio_medium_val = 0,
                .locked_exp_ratio_medium2_flag = false,
                .locked_exp_ratio_medium2_val = 0
            }
        },
        .exposures = 1,
        .num_channels = 1
    }
};
// clang-format on

struct mipi_dma_cb {
	void *owner;
	aframe_t *raw_frame;
};

typedef struct _sensor_private_t {
    sensor_param_t param;
    uint64_t integration_time;
    int32_t again;
    int32_t dgain;
    struct i2c_client *client; // To communicate with the MCU
	// TODO : instead of individual function add the callback structure
	int (*sensor_get_frame )( void *owner, void **frame );
	int (*sensor_put_frame )( void *owner, void *frame );
	u8 ctx_id;
	void *owner;
	struct workqueue_struct *wq;

	struct semaphore sem_isp_to_dma;
	int first_cb;

	unsigned int last_cb_index;
	unsigned int last_ff_index;
	unsigned int is_streaming;
	struct mipi_dma_cb dma_cb_struct[MAX_RAW_FRAMES];
	struct dma_chan *dma;
	struct dma_interleaved_template xt;
	struct data_chunk sgl[1];
} sensor_private_t;

struct sensor_work {
	struct work_struct work;
	sensor_private_t *priv;
};

static sensor_private_t priv_array[FIRMWARE_CONTEXT_NUMBER];

//--------------------DATA-------------------------------------------------------------
//--------------------RESET------------------------------------------------------------
static void sensor_hw_reset_enable( void )
{
    return;
}

static void sensor_hw_reset_disable( void )
{
    return;
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

unsigned char errorcheck(char *data, unsigned int len)
{
	unsigned int i = 0;
	unsigned char crc = 0x00;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
	}

	return crc;
}
// I2C Initialization
static bool i2c_client_init(sensor_private_t *priv) {

	isp_v4l2_dev_t *isp_dev = isp_v4l2_get_dev(priv->ctx_id);
	if (!isp_dev) {
		LOG( LOG_ERR, "Failed to get isp dev for context %d", priv->ctx_id);
		return false;
	}

	struct v4l2_subdev *sd_sensor = isp_dev->sd[SD_CAMERA];
	if (!sd_sensor) {
		LOG( LOG_ERR, "Failed to get sensor sd for locaion %d", priv->ctx_id);
		return false;
	}

	priv->client = v4l2_get_subdevdata(sd_sensor);
	if(!(priv->client)) {
		LOG (LOG_CRIT, "Failed to get i2c client handle for context %d", priv->ctx_id);
		return false;
	}

	LOG(LOG_INFO, "SUCCESS I2C context %d with address %x",
			priv->ctx_id, priv->client->addr);

	return true;
}

// Setting Gain
int32_t cam_set_gain(void *sensor_priv, uint64_t gain)
{
	unsigned char mc_data[100];
        uint32_t payload_len = 0;
        uint16_t cmd_status = 0;
        uint8_t retcode = 0, cmd_id = 0;
        int ret = 0, err = 0, retry = 5;
	int loop = 0;
	uint16_t ctrl_val_len = 0, index = 0;
	sensor_private_t *priv = (sensor_private_t *)sensor_priv;

	if(!(priv->client)) {
		if (i2c_client_init(priv) == false) {
			LOG (LOG_ERR, "Failed: i2c client is not initialised");
		}
	}
	/* lock semaphore */
        mutex_lock(&mcu_i2c_mutex);

	payload_len = 20;
	ctrl_val_len = 8;

	index = 0x00; // GAIN Control Index

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ret = cam_write(priv->client, mc_data, 5);
	if (ret != 0) {
		LOG (LOG_ERR, "%s (%d): Failed err= %d\n", __func__, __LINE__, ret);
		goto exit;
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

	ret = cam_write(priv->client, mc_data, payload_len + 3);
	if (ret != 0) {
		LOG (LOG_ERR, "%s (%d): Failed err= %d\n", __func__, __LINE__, ret);
		goto exit;
	}
exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);
	return ret;
}

// Setting exposure
int32_t cam_set_exposure(void *sensor_priv, uint64_t exp)
{
	unsigned char mc_data[100];
        uint32_t payload_len = 0;
        uint16_t cmd_status = 0;
        uint8_t retcode = 0, cmd_id = 0;
        int ret = 0, err = 0, retry = 5;
	int loop = 0;
	uint16_t ctrl_val_len = 0, index = 0;
	sensor_private_t *priv = (sensor_private_t *)sensor_priv;

	if(!(priv->client)) {
		if (i2c_client_init(priv) == false) {
			LOG (LOG_ERR, "Failed: i2c client is not initialised");
		}
	}

	/* lock semaphore */
        mutex_lock(&mcu_i2c_mutex);

	payload_len = 20;
	ctrl_val_len = 8;

	index = 0x01; // Exposure Control Index

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ret = cam_write(priv->client, mc_data, 5);
	if (ret != 0) {
		LOG (LOG_ERR, "%s (%d): Failed err= %d\n", __func__, __LINE__, ret);
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	/* Control ID */
	mc_data[4] = EXPOSURE_CTRL_ID >> 24;
	mc_data[5] = EXPOSURE_CTRL_ID >> 16;
	mc_data[6] = EXPOSURE_CTRL_ID >> 8;
	mc_data[7] = EXPOSURE_CTRL_ID & 0xFF;
	/* Ctrl Type */
	mc_data[8] = CTRL_EXTENDED;
	mc_data[9]  = V4L2_CTRL_TYPE_INTEGER64;
	mc_data[10] = ctrl_val_len >> 24;
	mc_data[11] = ctrl_val_len >> 16;
	mc_data[12] = ctrl_val_len >> 8;
	mc_data[13] = ctrl_val_len & 0xFF;
	for (loop = 0;loop < ctrl_val_len; loop++)
		mc_data[21-loop] = (exp >> (8 * loop));
	/* CRC */
	mc_data[22] = errorcheck(&mc_data[2], payload_len);

	ret = cam_write(priv->client, mc_data, payload_len + 3);
	if (ret != 0) {
		LOG (LOG_ERR, "%s (%d): Failed err= %d\n", __func__, __LINE__, ret);
		goto exit;
	}
exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);
	return ret;
}

// Setting frame rate
int32_t cam_set_framerate(void *sensor_priv, uint32_t fps)
{
	unsigned char mc_data[100];
        uint32_t payload_len = 0;
        uint16_t cmd_status = 0;
        uint8_t retcode = 0, cmd_id = 0;
        int ret = 0, err = 0, retry = 5;
	int loop = 0;
	uint16_t ctrl_val_len = 0, index = 0;
	sensor_private_t *priv = (sensor_private_t *)sensor_priv;
	uint64_t framerate = 0;

	if(!(priv->client)) {
		if (i2c_client_init(priv) == false) {
			LOG (LOG_ERR, "Failed: i2c client is not initialised");
		}
	}

	// Conversion of fps to write to the mcu.
	framerate = (fps / 256) * EXPOSURE_FACTOR;

	if (framerate == 0)
		framerate = 5000000;

	/* lock semaphore */
        mutex_lock(&mcu_i2c_mutex);

	payload_len = 20;
	ctrl_val_len = 8;

	index = 0x03; // Frame rate Control Index

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ret = cam_write(priv->client, mc_data, 5);
	if (ret != 0) {
		LOG (LOG_ERR, "%s (%d): Failed err= %d\n", __func__, __LINE__, ret);
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	/* Control ID */
	mc_data[4] = FRAMERATE_CTRL_ID >> 24;
	mc_data[5] = FRAMERATE_CTRL_ID >> 16;
	mc_data[6] = FRAMERATE_CTRL_ID >> 8;
	mc_data[7] = FRAMERATE_CTRL_ID & 0xFF;
	/* Ctrl Type */
	mc_data[8] = CTRL_EXTENDED;
	mc_data[9]  = V4L2_CTRL_TYPE_INTEGER64;
	mc_data[10] = ctrl_val_len >> 24;
	mc_data[11] = ctrl_val_len >> 16;
	mc_data[12] = ctrl_val_len >> 8;
	mc_data[13] = ctrl_val_len & 0xFF;
	for (loop = 0;loop < ctrl_val_len; loop++)
		mc_data[21-loop] = (framerate >> (8 * loop));
	/* CRC */
	mc_data[22] = errorcheck(&mc_data[2], payload_len);

	ret = cam_write(priv->client, mc_data, payload_len + 3);
	if (ret != 0) {
		LOG (LOG_ERR, "%s (%d): Failed err= %d\n", __func__, __LINE__, ret);
		goto exit;
	}
exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return 0;
}

#define AGAIN_PRECISION 12
#define LOG10_2_AGAIN_PREC ( 1233 ) // log10(2) << AGAIN_PRECISION
#define LOG_TO_DB ( 20 )
#define SENSOR_AGAIN_STEP_UP ( 10 )
#define SENSOR_AGAIN_STEP_DOWN ( 3 )
#define NORMALISE_FACTOR (LOG2_GAIN_SHIFT - AGAIN_PRECISION)
#define CONVERSION_FACTOR (((LOG10_2_AGAIN_PREC * LOG_TO_DB * SENSOR_AGAIN_STEP_UP) / SENSOR_AGAIN_STEP_DOWN) >> NORMALISE_FACTOR)
#define NORMALISE_REG_FACTOR ( 2 * AGAIN_PRECISION )

static int32_t sensor_alloc_analog_gain( void *sensor_priv, int32_t gain )
{
    sensor_private_t *priv = sensor_priv;
    uint32_t a_gain;
	int32_t ret = 0;

    if (priv->again != gain) {
	    // Conversion of log2_gain value to corresponded sensor gain value in dB
	    a_gain = (gain * CONVERSION_FACTOR) >> NORMALISE_REG_FACTOR;
	    // Conversion of dB to Gain Values to parse to the MCU to configure sensor
	    a_gain = (a_gain * 3)/10;
	    a_gain = a_gain * GAIN_FACTOR;
	    ret = cam_set_gain (priv, (uint64_t)a_gain);
	    if (ret == 0) {
		    priv->again = gain;
	    }

    }

    return gain;
}

static int32_t sensor_alloc_digital_gain( void *sensor_priv, int32_t gain )
{
    sensor_private_t *priv = sensor_priv;
    sensor_param_t *cfg = &priv->param;
    uint32_t d_gain;
    int32_t ret = 0;

    if (priv->again == cfg->again_log2_max) {
	    if (priv->dgain != gain) {
		    // Conversion of log2_gain value to corresponded sensor gain value in dB
		    d_gain = (gain * CONVERSION_FACTOR) >> NORMALISE_REG_FACTOR;
		    // Conversion of dB to Gain Values to parse to the MCU to configure sensor
		    d_gain = ((d_gain * 3)/10) + 30; // 30 - Adding Sensor Analog Gain maximum: 30dB
		    d_gain = (d_gain * GAIN_FACTOR);
		    ret = cam_set_gain (priv, (uint64_t)d_gain);
		    if (ret == 0){
			    priv->dgain = gain;
		    }
	    }
    }

    return gain;
}

static void sensor_alloc_integration_time( void *sensor_priv, integration_times_t *int_times )
{
    sensor_private_t *priv = sensor_priv;
    sensor_param_t *cfg = &priv->param;
    uint32_t *int_time = &int_times->int_time;
    uint64_t exp = 0;
    int32_t ret = 0;

    if (priv->integration_time != *int_time) {
	    // Conversion of lines to exposure time (us)
	    exp = (uint64_t)(*int_time) * supported_modes[cfg->preset_mode].hmax;
	    exp = (exp * EXPOSURE_FACTOR)/ SENSOR_PIXEL_CLOCK;
	    ret = cam_set_exposure (priv, exp);
	    if (ret == 0) {
		    priv->integration_time = *int_time;
	    }
    }
}

static void sensor_alloc_white_balance_gains( void *sensor_priv, int32_t gain[4] )
{
	// Not supported for IMX678
    (void)sensor_priv; //unusued
    (void)gain;        //unusued
}

static void sensor_update( void *sensor_priv )
{
	LOG (LOG_INFO, "Sensor update called");
}

static void work_queue_fn(struct work_struct *work) {

	int rc = -1;
	uint64_t addr;
	struct sensor_work *w = container_of(work, struct sensor_work, work);
	sensor_private_t *priv = w->priv;
	
	if (!priv) {
		LOG (LOG_ERR, "Invalid sensor info");
		kfree(w);
		return;
	}

	if (priv->first_cb != 0 ) {
		down(&priv->sem_isp_to_dma);
	} else {
		priv->first_cb = 1;
	}

	rc = priv->sensor_put_frame(priv->owner, priv->dma_cb_struct[priv->last_cb_index].raw_frame);
	if (rc != 0) {
		LOG( LOG_ERR, "sensor put frame failed");
		kfree(w);
		return;
	}

	// TODO : make sure last_cb_index dont overrun last_ff_index
	priv->last_cb_index = (priv->last_cb_index + 1) % MAX_RAW_FRAMES;

	kfree(w);
}

static void dma_transfer_done_cb(void *param) {

	sensor_private_t *priv = (sensor_private_t *)param;
	struct sensor_work *w = kmalloc(sizeof(struct sensor_work), GFP_ATOMIC);  // Must be atomic
	if (!w) {
		LOG( LOG_ERR, "Failed to allocate work structure");
		return;
	}

	INIT_WORK(&w->work, work_queue_fn);
	w->priv = priv;

	if (!queue_work(priv->wq, &w->work)) {
        LOG( LOG_ERR, "Work is already scheduled !!!! %u", priv->ctx_id);
	}

    return;
}

static int submit_dma_request(sensor_private_t *priv, aframe_t *input_frame) {

	struct dma_async_tx_descriptor *desc;
	u32 flags;
	int32_t cookie = -1;
	dma_addr_t phys_addr;
	int rc = 0;

	flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	priv->xt.dir = DMA_DEV_TO_MEM;
	uint64_t addr = input_frame->planes[0].address.high;
	addr = (addr << 32) | (input_frame->planes[0].address.low);

	if (stu) {
		rc = simaai_stu_get_dev_address(stu, addr, &phys_addr);
		if (rc != 0) {
			LOG(LOG_CRIT, "Failed to get device address for phys address %#llx\n", addr);
			return rc;
		}
	} else {
		LOG( LOG_CRIT, "STU is not initialized\n");
		return -EINVAL;
	}

	priv->xt.dst_start = phys_addr;

	if ( (input_frame->planes[0].width !=  priv->param.active.width) ||
		 (input_frame->planes[0].height !=  priv->param.active.height)) {
		LOG (LOG_ERR, "Mismatch in sensor mode and raw frame resolution");
		return -EINVAL;
	}

	priv->xt.src_sgl = false;
	priv->xt.dst_inc = false;
	priv->xt.dst_sgl = true;
	priv->xt.frame_size = 1;
	priv->xt.sgl[0].size = (((priv->param.active.width) * (priv->param.active.height) *
							(priv->param.data_width)) / 8);
	priv->xt.sgl[0].icg = 0;
	priv->xt.numf = priv->param.active.height;

	desc = dmaengine_prep_interleaved_dma(priv->dma, &priv->xt, flags);
	if (!desc) {
		LOG( LOG_ERR, "ERROR : creating descriptor\n");
		return -EINVAL;
	}

	desc->callback = dma_transfer_done_cb;
	desc->callback_param = priv;

	cookie = dmaengine_submit(desc);
	if (priv->is_streaming) {
		dma_async_issue_pending(priv->dma);
	}

	LOG( LOG_INFO, "SUCCESS : MIPI submitting buffer");

	return 0;
}

static void sensor_set_mode( void *sensor_priv, uint8_t mode )
{
	int rc = -1;
    sensor_private_t *priv = sensor_priv;
    sensor_param_t *cfg = &priv->param;
	aframe_t *input_frame = NULL;
	uint32_t iter = 0;

	// I2C Init
    if (i2c_client_init(priv) == false) {
		LOG (LOG_ERR, "Failed: i2c client is not initialised");
    }
	
    cfg->active.width = supported_modes[mode].resolution.width;
    cfg->active.height = supported_modes[mode].resolution.height;
    cfg->total.width = supported_modes[mode].hmax;
    cfg->total.height = supported_modes[mode].vmax;
    cfg->integration_time_min = SENSOR_MIN_INTEGRATION_TIME;
    cfg->integration_time_max = SENSOR_MAX_INTEGRATION_TIME;
    cfg->integration_time_limit = SENSOR_MAX_INTEGRATION_TIME_LIMIT;
    cfg->preset_mode = mode;
    //56250; // total number of lines per second.
    cfg->lines_per_second = SENSOR_PIXEL_CLOCK / supported_modes[mode].hmax;

	LOG(LOG_DEBUG,"sensor : mode :%d active width %d, height %d , %d , %d",
								mode, cfg->active.width, cfg->active.height,cfg->total.width,
								cfg->total.height);

    cfg->sensor_exp_number = supported_modes[mode].exposures;
    cfg->num_channel = supported_modes[mode].num_channels;

	if (priv->sensor_get_frame) {

		while (priv->last_ff_index < MAX_RAW_FRAMES) {
			rc = priv->sensor_get_frame( priv->owner, (void **)&input_frame);
			if (rc < 0) {
				LOG( LOG_ERR, "ERROR : gett frame from streamer");
				return;
			}

			LOG (LOG_DEBUG, "SUCCESS getting frame from stramer %#llx, %#x:%x, width : %d, height : %d",
					input_frame->planes[0].virt_addr, input_frame->planes[0].address.low,
					input_frame->planes[0].address.high, input_frame->planes[0].width, input_frame->planes[0].height);

			// Fill the dma cb struct
			priv->dma_cb_struct[priv->last_ff_index].raw_frame = input_frame;
			priv->dma_cb_struct[priv->last_ff_index].owner = priv->owner;

			rc = submit_dma_request(priv, input_frame);
			if (rc != 0) {
				LOG(LOG_CRIT, "failed to submit dma request");
				return;
			}

			LOG( LOG_DEBUG, "SUCCESS submitting DMA request, iter : %u, %#llx", priv->last_ff_index, priv);
			priv->last_ff_index++;
		}

		priv->last_ff_index = 0;
	} else {
		LOG ( LOG_ERR, "get_frame callback is not initialized");
		return;
	}
}

static uint16_t sensor_get_id( void *sensor_priv )
{
    return 0xFFFF;
}

static const sensor_param_t *sensor_get_parameters( void *sensor_priv )
{
    sensor_private_t *priv = sensor_priv;
    return (const sensor_param_t *)&priv->param;
}

static uint8_t sensor_fps_control( void *sensor_priv, uint8_t fps )
{
    // This sensor does not support FPS switching.
	LOG( LOG_DEBUG, "Sensor fps control");
    return 0;
}

static uint32_t read_register( void *sensor_priv, uint32_t address )
{
	LOG( LOG_ERR, " Un-supported read sensor register for address %#x", address); 
    return 0;
}

static void write_register( void *sensor_priv, uint32_t address, uint32_t data )
{
	LOG( LOG_ERR, "un-supported write sensor register for address %#x value %#x", address, data);
}

static void stop_streaming( void *sensor_priv ) {

	LOG( LOG_DEBUG, "STOP streaming is called");

    sensor_private_t *priv = sensor_priv;
	dmaengine_terminate_sync(priv->dma);
	priv->is_streaming = 0;
}

static void start_streaming( void *sensor_priv ) {

	LOG( LOG_INFO, "START streaming is called");

    sensor_private_t *priv = sensor_priv;
	priv->is_streaming = 1;
	dma_async_issue_pending(priv->dma);
}

static void request_next_frame( void *sensor_priv ) {

    sensor_private_t *priv = sensor_priv;
	aframe_t *input_frame = NULL;

	if (priv->sensor_get_frame) {
		int rc = priv->sensor_get_frame( priv->owner, (void **)&input_frame);
		if (rc < 0) {
			LOG( LOG_ERR, "ERROR : getting frame from streamer");
			return;
		}

		LOG (LOG_DEBUG, "SUCCESS getting frame from stramer %#llx, %#x, width : %d, height : %d",
				input_frame->planes[0].virt_addr, input_frame->planes[0].address.low,
				input_frame->planes[0].width, input_frame->planes[0].height);

			// Fill the dma cb struct
		priv->dma_cb_struct[priv->last_ff_index].raw_frame = input_frame;
		priv->dma_cb_struct[priv->last_ff_index].owner = priv->owner;

		rc = submit_dma_request(priv, input_frame);
		if (rc != 0) {
			LOG(LOG_CRIT, "failed to submit dma request");
			return;
		}

		LOG( LOG_DEBUG, "SUCCESS submitting DMA request, iter : %u", priv->last_ff_index);
		priv->last_ff_index = (priv->last_ff_index + 1) % MAX_RAW_FRAMES;	
	}

	up(&priv->sem_isp_to_dma);
}

static void register_frame_callbacks( void *sensor_priv, const sensor_remote_callbacks_t *callbacks ) {

    sensor_private_t *priv = sensor_priv;
	priv->sensor_get_frame = callbacks->get_frame;
	priv->sensor_put_frame = callbacks->put_frame;
	priv->owner = callbacks->callback_owner;

}

static void sensor_deinit_imx678( void *sensor_priv )
{
    sensor_private_t *priv = sensor_priv;
	LOG( LOG_DEBUG, "Sensor deinit for context : %d", priv->ctx_id);

	up(&priv->sem_isp_to_dma);
	flush_workqueue(wq);
	priv->last_cb_index = 0;
	priv->last_ff_index = 0;
	priv->is_streaming = 0;
	priv->first_cb = 0;
	sema_init(&priv->sem_isp_to_dma, 0);
}

//--------------------Initialization------------------------------------------------------------

static int isp_register_dma_channels(sensor_private_t *priv, int ctx_id) {

	char dma_names[64];
	int8_t dma_index;

	if (!g_pdev) {
		LOG( LOG_ERR, "platform device is not initialised");
		return -EINVAL;
	}

	dma_index =  get_dma_index(ctx_id);
	if (dma_index < 0) {
		LOG (LOG_ERR, "ERROR getting dma index");
		return -EINVAL;
	}

	snprintf(dma_names, sizeof(dma_names)-1, "vdma%d", dma_index);

	LOG( LOG_DEBUG, "DMA channel request name is %s", dma_names);

	priv->dma = dma_request_slave_channel(&(g_pdev->dev), dma_names);
	if (priv->dma == NULL) {
		LOG( LOG_ERR, "no VDMA channel found by name vdma %s", dma_names);
		return -ENODEV;
	}

	LOG( LOG_DEBUG, "dma chan id %d", priv->dma->chan_id);

	return 0;
}


void sensor_init_imx678( void **priv_ptr, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const options )
{
    LOG(LOG_DEBUG, "imx678 sensor init for ctx : %u", location);

	char wq_name[64];
    sensor_private_t *priv = *priv_ptr = priv_array + location;
    sensor_param_t *cfg = &priv->param;
	int rc = -1;
	acamera_settings *ctx_settings = get_settings_by_id(location);
	u32 cdma_addr = 0;

    system_memset( cfg, 0, sizeof( *cfg ) );

    cfg->sensor_exp_number = SENSOR_EXP_NUMBER;
    cfg->again_log2_max = LOG2_SENSOR_AGAIN_MAXIMUM << LOG2_GAIN_SHIFT;
    cfg->dgain_log2_max = LOG2_SENSOR_DGAIN_MAXIMUM << LOG2_GAIN_SHIFT;
    cfg->integration_time_apply_delay = SENSOR_INTEGRATION_TIME_APPLY_DELAY;
    cfg->isp_exposure_channel_delay = 0;
    cfg->modes_table = supported_modes;
    cfg->modes_num = ARRAY_SIZE( supported_modes );
    cfg->again_accuracy = ( 1 << ( LOG2_GAIN_SHIFT - 2 ) );
    cfg->h_start = 0;
    cfg->v_start = 0;
    cfg->video_port_id = location;
    cfg->num_channel = 1;
    cfg->is_remote = options->is_remote;
    cfg->data_width = SENSOR_DATA_WIDTH;
    cfg->rggb_start = SENSOR_DATA_FORMAT;
    cfg->cfa_pattern = 0;
    cfg->shared_vc_clk = 0;

    ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
    ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
    ctrl->alloc_integration_time = sensor_alloc_integration_time;
    ctrl->alloc_white_balance_gains = sensor_alloc_white_balance_gains;
    ctrl->sensor_update = sensor_update;
    ctrl->set_mode = sensor_set_mode;
    ctrl->fps_control = sensor_fps_control;
    ctrl->get_parameters = sensor_get_parameters;
    ctrl->read_sensor_register = read_register;
    ctrl->write_sensor_register = write_register;
    ctrl->start_streaming = start_streaming;
    ctrl->stop_streaming = stop_streaming;
    ctrl->register_frame_callbacks = register_frame_callbacks;
    ctrl->request_next_frame = request_next_frame;
    ctrl->deinit = sensor_deinit_imx678;

	/* init work queue */
	if (!wq) {
		snprintf(wq_name, sizeof(wq_name) - 1, "isp_wq");
		wq = alloc_workqueue(wq_name, WQ_UNBOUND, 1);
		if (!wq ) {
			LOG( LOG_CRIT, "failed to create work queue for context : %u", location);
			return;
		}
	}

	priv->wq = wq;
	priv->last_cb_index = 0;
	priv->last_ff_index = 0;
	priv->is_streaming = 0;
	priv->ctx_id = location;
	sema_init(&priv->sem_isp_to_dma, 0);
	priv->first_cb = 0;

	rc = isp_register_dma_channels(priv, location);
	if (rc != 0) {
		LOG (LOG_ERR, "Failed to register dma channel for context : %d", location);
		return;
	}

	if (ctx_settings) {

		ctx_settings->get_calibrations = get_calibrations_imx678;
		cdma_addr = ctx_settings->isp_base;

		// Enable Mesh Shading
		acamera_isp_pipeline_bypass_mesh_shading_write(cdma_addr,0);
		// Enable ISP Digital Gain
		acamera_isp_pipeline_bypass_digital_gain_write(cdma_addr,0);
		// Enable CAC
		acamera_isp_pipeline_bypass_ca_correction_write(cdma_addr,0);
		// Enable DPC
		acamera_isp_pipeline_bypass_defect_pixel_write(cdma_addr,0);
		acamera_isp_pipeline_bypass_white_balance_write(cdma_addr,0);
		acamera_isp_pipeline_bypass_out_format_write(cdma_addr, 0);

		acamera_isp_pipeline_bypass_sensor_offset_wdr_write( cdma_addr, 1 );
		acamera_isp_pipeline_bypass_gamma_be_sq_write( cdma_addr, 1);
		acamera_isp_pipeline_bypass_gamma_fe_sq_write( cdma_addr, 1);
		// Black  level updated for IMX678 - 200 (dec) - 0xC8 (Hex)
		// This offset will be effective only when white balance is enabled.
		acamera_isp_offset_black_00_write( cdma_addr, 0xC8000 );
		acamera_isp_offset_black_01_write( cdma_addr, 0xC8000 );
		acamera_isp_offset_black_10_write( cdma_addr, 0xC8000);
		acamera_isp_offset_black_11_write( cdma_addr, 0xC8000 );

	} else {
		LOG (LOG_ERR, "Failed to get the ctx pointer for ctx :%d", location);
	}

    // Reset sensor during initialization
    sensor_hw_reset_enable();
    sensor_hw_reset_disable();

    LOG( LOG_DEBUG, "Sensor DPattern (id 0x%04x) initialized at position %d.", sensor_get_id( priv ), location );
}

//*************************************************************************************
