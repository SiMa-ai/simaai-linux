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
#include "og05c10_config.h"

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
#include <linux/unaligned.h>

#include "sensor_bus_config.h"
#include <linux/simaai-stu.h>

extern struct simaai_stu *stu;
static struct workqueue_struct *wq;

// Filled in system_cma.c
extern struct platform_device *g_pdev;

#define MAX_RAW_FRAMES			(5)
extern int32_t get_calibrations_og05c10( uint32_t wdr_mode, void *param );
extern acamera_settings *get_settings_by_id(u8 ctx_id);
extern  int8_t get_dma_index(uint32_t ctx_id);

// Formatting tool makes below declaration less readable and structured
// clang-format off
static sensor_mode_t supported_modes[] = {
    {
        .fps = 60 * 256,
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

static bool i2c_client_init(sensor_private_t *priv) {

	struct v4l2_subdev *sd_sensor = NULL;
	isp_v4l2_dev_t *isp_dev = isp_v4l2_get_dev(priv->ctx_id);
	if (!isp_dev) {
		LOG( LOG_ERR, "Failed to get isp dev for context %d", priv->ctx_id);
		return false;
	}
#if 0
#if MIPI_ISP_INTEGRATION
	sd_sensor = isp_dev->remote_sd[SD_CAMERA];
	if (!sd_sensor) {
		LOG( LOG_ERR, "Failed to get sensor sd for locaion %d", priv->ctx_id);
		return false;
	}
#endif
#endif

	priv->client = v4l2_get_subdevdata(sd_sensor);
	if(!(priv->client)) {
		LOG (LOG_CRIT, "Failed to get i2c client handle for context %d", priv->ctx_id);
		return false;
	}

	LOG(LOG_DEBUG, "SUCCESS initalizing i2c client for context %d with address %#x",
			priv->ctx_id, priv->client->addr);

	return true;
}
//--------------------FLASH------------------------------------------------------------

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

static int og05c10_write_regs(sensor_private_t *priv, u16 reg, u8 *buf, u8 len)
{
    if (len > 255) return -EINVAL; // 检查长度溢出
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = priv->client;
    
    b[0] = reg >> 8; // 高字节
    b[1] = reg & 0xFF; // 低字节
    memcpy(&b[2], buf, len);
    
    msg.addr = client->addr;
    msg.flags = 0;
    msg.buf = b;
    msg.len = len + 2;
    
    int ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0) {
        LOG(LOG_ERR, "I2C write error: %d", ret);
        return ret;
    }
    return 0;
}

static int og05c10_write_reg(sensor_private_t *priv, u16 reg, u8 data)
{
    u8 buf = 0;
    buf = data;
    return og05c10_write_regs(priv, reg, &buf, 1);
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
int32_t cam_set_exposure(void *sensor_priv, uint64_t exp)
{
#if 1
	sensor_private_t *priv = (sensor_private_t *)sensor_priv;
	int ret = 0;

	if(!(priv->client)) {
		if (i2c_client_init(priv) == false) {
			LOG (LOG_ERR, "Failed: i2c client is not initialised");
		}
	}
	//LOG(LOG_ERR, "SUCCESS I2C context %d with address %x exp=%llu",
	//		priv->ctx_id, priv->client->addr, exp);
	if(exp < SENSOR_EXP_MIN)
		exp = SENSOR_EXP_MIN;
	og05c10_write_reg(priv, SENSOR_EXP_REG_H, (exp>>16)&0xff);
	og05c10_write_reg(priv, SENSOR_EXP_REG_M, (exp>>8)&0xff);
	ret = og05c10_write_reg(priv, SENSOR_EXP_REG_L, exp&0xff);
#endif
	return ret;
}

int32_t cam_set_gain(void *sensor_priv, uint64_t gain)
{
	sensor_private_t *priv = (sensor_private_t *)sensor_priv;
	int ret = 0;
	
	if(!(priv->client)) {
		if (i2c_client_init(priv) == false) {
			LOG (LOG_ERR, "Failed: i2c client is not initialised");
		}
	}	
	og05c10_write_reg(priv, SENSOR_GAIN_REG_H, (gain>>8)&0xff);
	ret = og05c10_write_reg(priv, SENSOR_GAIN_REG_L, gain&0xff);

	return ret;
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

    priv->dgain = gain;
	//LOG (LOG_ERR, "sensor_alloc_digital_gain dgain=%d\n",priv->dgain);
    return gain;
}

static void sensor_alloc_integration_time( void *sensor_priv, integration_times_t *int_times )
{
    sensor_private_t *priv = sensor_priv;
    sensor_param_t *cfg = &priv->param;
    uint32_t *int_time = &int_times->int_time;
    uint64_t exp = 0;
    int32_t ret = 0;
	uint32_t sensor_hmax;

	//LOG (LOG_ERR, "sensor_alloc_integration_time int_times=%d\n",*int_times);
    if (priv->integration_time != *int_time) {
	    // Conversion of lines to exposure time (us)
	    //exp = (uint64_t)(*int_time) * supported_modes[cfg->preset_mode].hmax;
		sensor_hmax = SENSOR_VTS/2 - 10;
		exp = (uint64_t)(*int_time) * sensor_hmax;
	    exp = (exp * EXPOSURE_FACTOR)/ SENSOR_PIXEL_CLOCK*2;
	    ret = cam_set_exposure (priv, exp);
	    if (ret == 0) {
		    priv->integration_time = *int_time;
	    }
    }
}

static void sensor_alloc_white_balance_gains( void *sensor_priv, int32_t gain[4] )
{
    (void)sensor_priv; //unusued
    (void)gain;        //unusued
	//LOG (LOG_ERR, "sensor_alloc_white_balance_gains gain=%d\n",gain[0]);
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

	return 0;
}

static void sensor_set_mode( void *sensor_priv, uint8_t mode )
{
	int rc = -1;
    sensor_private_t *priv = sensor_priv;
    sensor_param_t *cfg = &priv->param;
	aframe_t *input_frame = NULL;
	uint32_t iter = 0;

    cfg->active.width = supported_modes[mode].resolution.width;
    cfg->active.height = supported_modes[mode].resolution.height;
    cfg->total.width = SENSOR_TOTAL_WIDTH;
    cfg->total.height = SENSOR_TOTAL_HEIGHT;
    cfg->integration_time_min = SENSOR_MIN_INTEGRATION_TIME;
    cfg->integration_time_max = SENSOR_MAX_INTEGRATION_TIME;
    cfg->integration_time_limit = SENSOR_MAX_INTEGRATION_TIME_LIMIT;
    cfg->preset_mode = mode;
    cfg->lines_per_second = 0;

    LOG(LOG_INFO,"sensor : mode :%d active width %d, height %d , %d , %d",
				mode, cfg->active.width, cfg->active.height,cfg->total.width,
				cfg->total.height);

    cfg->sensor_exp_number = supported_modes[mode].exposures;
    cfg->num_channel = supported_modes[mode].num_channels;

	if (priv->sensor_get_frame) {
		LOG( LOG_INFO, "get frame is called");

		while (priv->last_ff_index < MAX_RAW_FRAMES) {
			rc = priv->sensor_get_frame( priv->owner, (void **)&input_frame);
			if (rc < 0) {
				LOG( LOG_ERR, "ERROR : gett frame from streamer");
				return;
			}

			LOG (LOG_INFO, "SUCCESS getting frame from stramer %#llx, %#x:%x, width : %d, height : %d",
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

			LOG( LOG_INFO, "SUCCESS submitting DMA request, iter : %u, %#llx", priv->last_ff_index, priv);
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
    sensor_private_t *priv = (sensor_private_t *)sensor_priv;
    return priv->ctx_id;
}

static const sensor_param_t *sensor_get_parameters( void *sensor_priv )
{
    sensor_private_t *priv = sensor_priv;
    return (const sensor_param_t *)&priv->param;
}

static uint8_t sensor_fps_control( void *sensor_priv, uint8_t fps )
{
    // This sensor does not support FPS switching.
	LOG( LOG_ERR, "NOT IMPLEMENTED : Sensor FPS control");
    return 0;
}

static uint32_t read_register( void *sensor_priv, uint32_t address )
{

	sensor_private_t *priv = (sensor_private_t *)sensor_priv;
	u8 read_data[4];
	u8 addr_buf[2];
	addr_buf[0] = (address >> 8) & 0xFF;
	addr_buf[1] = address & 0xFF;

	LOG( LOG_DEBUG, "Read sensor register for address %#x", address);

	if(!(priv->client)) {
		if (i2c_client_init(priv) == false) {
			LOG (LOG_ERR, "read register failed because i2c client is not initialised");
			return 0;
		}
	}

	struct i2c_msg msgs[2] = {
		{
			.addr = priv->client->addr,
			.flags = 0, // Write register address first
			.len = 2,
			.buf = addr_buf,
		},
		{
			.addr = priv->client->addr,
			.flags = I2C_M_RD, // Read
			.len = 2,
			.buf = &read_data[2],
		}
	};

	if (i2c_transfer(priv->client->adapter, msgs, 2) != 2)
		LOG( LOG_ERR, "Failed to read 2-byte add :  %#x", address);
	else
		LOG( LOG_DEBUG, "Read : %#x from add : %#x", get_unaligned_be32(read_data), address);

 
    return get_unaligned_be32(read_data);
}

static void write_register( void *sensor_priv, uint32_t address, uint32_t data )
{
	sensor_private_t *priv = sensor_priv;
	u8 write_buf[4];
	write_buf[0] = (address >> 8) & 0xFF;	// High byte of address
	write_buf[1] = address & 0xFF;			// Low byte of address
	write_buf[2] = (data >> 8)  & 0xFF;		// High byte of data
	write_buf[3] = data & 0xFF;				// Low byte of data

	LOG( LOG_DEBUG, "write sensor register for address %#x value %#x", address, data);

	if(!(priv->client)) {
		if (i2c_client_init(priv) == false) {
			LOG (LOG_ERR, "write register failed because i2c client is not initialised");
			return;
		}
	}
	struct i2c_msg write_msg = {
		.addr = priv->client->addr,
		.flags = 0, // Write
		.len = sizeof(write_buf),
		.buf = write_buf,
	};

	if (i2c_transfer(priv->client->adapter, &write_msg, 1) != 1)
		LOG( LOG_ERR, "Failed to write 2-byte add :  %#x\n", address);
	else
		LOG( LOG_DEBUG, "Wrote 0x%02x to add : 0x%04x\n", get_unaligned_be16(&write_buf[2]), address);
}

static void stop_streaming( void *sensor_priv ) {

	LOG( LOG_INFO, "STOP streaming is called");

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

		// Fill the dma cb struct
		priv->dma_cb_struct[priv->last_ff_index].raw_frame = input_frame;
		priv->dma_cb_struct[priv->last_ff_index].owner = priv->owner;

		rc = submit_dma_request(priv, input_frame);
		if (rc != 0) {
			LOG(LOG_CRIT, "failed to submit dma request");
			return;
		}

		priv->last_ff_index = (priv->last_ff_index + 1) % MAX_RAW_FRAMES;	
	}

	up(&priv->sem_isp_to_dma);
}

static void register_frame_callbacks( void *sensor_priv, const sensor_remote_callbacks_t *callbacks ) {

	LOG( LOG_INFO, "Adding register frame callbacks");
    sensor_private_t *priv = sensor_priv;
	priv->sensor_get_frame = callbacks->get_frame;
	priv->sensor_put_frame = callbacks->put_frame;
	priv->owner = callbacks->callback_owner;

}

static void sensor_deinit_og05c10( void *sensor_priv )
{
    sensor_private_t *priv = sensor_priv;
	LOG( LOG_INFO, "Sensor deinit for context : %d", priv->ctx_id);

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


void sensor_init_og05c10( void **priv_ptr, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const options )
{
    LOG(LOG_DEBUG, "og05c10 sensor init for ctx : %u", location);

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
    ctrl->deinit = sensor_deinit_og05c10;

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
		LOG (LOG_INFO, "Failed to register dma channel for context : %d", location);
		return;
	}

	if (ctx_settings) {

		ctx_settings->get_calibrations = get_calibrations_og05c10;
		cdma_addr = ctx_settings->isp_base;

		acamera_isp_pipeline_bypass_sensor_offset_wdr_write( cdma_addr, 0 );
		acamera_isp_pipeline_bypass_white_balance_write(cdma_addr,0);
		acamera_isp_pipeline_bypass_out_format_write(cdma_addr, 0);
		acamera_isp_pipeline_bypass_gamma_be_sq_write( cdma_addr, 0);
		acamera_isp_pipeline_bypass_gamma_fe_sq_write( cdma_addr, 0);
		acamera_isp_offset_black_00_write( cdma_addr, 0xF0000 );
		acamera_isp_offset_black_01_write( cdma_addr, 0xF0000 );
		acamera_isp_offset_black_10_write( cdma_addr, 0xF0000);
		acamera_isp_offset_black_11_write( cdma_addr, 0xF0000 );
		acamera_isp_white_balance_gain_00_write(cdma_addr, 1008);
		acamera_isp_white_balance_gain_01_write(cdma_addr, 450);
		acamera_isp_white_balance_gain_10_write(cdma_addr, 450);
		acamera_isp_white_balance_gain_11_write(cdma_addr, 720);

	} else {
		LOG (LOG_ERR, "Failed to get the ctx pointer for ctx :%d", location);
	}

    // Reset sensor during initialization
    sensor_hw_reset_enable();
    sensor_hw_reset_disable();

    LOG( LOG_DEBUG, "Sensor DPattern (id 0x%04x) initialized at position %d.", sensor_get_id( priv ), location );
}

//*************************************************************************************
