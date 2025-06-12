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
#include "imx477_config.h"

#include "acamera_aframe.h"
#include <linux/dmaengine.h> 
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <asm/io.h>

//filled in system_cma.c
extern struct platform_device *g_pdev;
//static struct semaphore sem_dma_to_isp;
//static struct semaphore sem_isp_to_dma;
//static int first_cb = 0;

//#include "isp-v4l2.h"
#define MAX_RAW_FRAMES			(9)

// Formatting tool makes below declaration less readable and structured
// clang-format off
static sensor_mode_t supported_modes[] = {
    {
        .fps = 60 * 256,
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
    {   // PWL
        .fps = 22 * 256,
        .wdr_mode = WDR_MODE_NATIVE,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .channel_info = {
            .channel_desc = {
                {
                    .exposure_bit_width = 20,
                    .data_type = DATA_TYPE_WDR_NATIVE,
                    .cv = CAP_KNEE_POINT_DECOMPAND
                }
            },
            .exposure_idx_to_channel_map = {
                0
            },
            .exposure_max_bit_width = 20,
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
    uint16_t integration_time;
    int32_t again;
    int32_t dgain;
	// TODO : instead of individual function add the callback structure
	int (*sensor_get_frame )( void *owner, void **frame );
	int (*sensor_put_frame )( void *owner, void *frame );
	u8 ctx_id;
	void *owner;
	struct workqueue_struct *wq;
	struct work_struct work;

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

//--------------------FLASH------------------------------------------------------------

static int32_t sensor_alloc_analog_gain( void *sensor_priv, int32_t gain )
{
    sensor_private_t *priv = sensor_priv;

    priv->again = gain;
    return gain;
}

static int32_t sensor_alloc_digital_gain( void *sensor_priv, int32_t gain )
{
    sensor_private_t *priv = sensor_priv;

    priv->dgain = gain;
    return gain;
}

static void sensor_alloc_integration_time( void *sensor_priv, integration_times_t *int_times )
{
    sensor_private_t *priv = sensor_priv;
    uint32_t *int_time = &int_times->int_time;

    priv->integration_time = *int_time;
}

static void sensor_alloc_white_balance_gains( void *sensor_priv, int32_t gain[4] )
{
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
	//LOG( LOG_INFO, "work queue called");
	sensor_private_t *priv = container_of(work, sensor_private_t, work);
	
	if (!priv) {
		LOG (LOG_ERR, "Invalid sensor info");
		return;
	}
	LOG (LOG_INFO, "WQ : width %d , height %d , pixel bits %d last_index : %u, "
				"priv : %#llx, work : %#llx",
	        priv->param.active.width,
    	    priv->param.active.height,
        	priv->param.data_width,
			priv->last_cb_index,
			priv,
			&priv->work);

	if (priv->first_cb != 0 ) {
		down(&priv->sem_isp_to_dma);
	} else {
		priv->first_cb = 1;
	}

	rc = priv->sensor_put_frame(priv->owner,
			priv->dma_cb_struct[priv->last_cb_index].raw_frame);
	if (rc != 0) {
		LOG( LOG_ERR, "sensor put frame failed");
		return;
	}

	// TODO : make sure last_cb_index dont overrun last_ff_index
	priv->last_cb_index = (priv->last_cb_index + 1) % MAX_RAW_FRAMES;

}

static void dma_transfer_done_cb(void *param) {

	sensor_private_t *priv = (sensor_private_t *)param;
	LOG ( LOG_INFO, "Buffer transfer done : private pointer is %#llx, work %#llx", priv, &priv->work);

	if (!queue_work(priv->wq, &priv->work)) {
        LOG( LOG_ERR, "Work is already scheduled !!!! %u", priv->ctx_id);
	}

    return;
}

static int submit_dma_request(sensor_private_t *priv, aframe_t *input_frame) {

	struct dma_async_tx_descriptor *desc;
	u32 flags;
	int32_t cookie = -1;

	flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	priv->xt.dir = DMA_DEV_TO_MEM;
	uint64_t addr = input_frame->planes[0].address.high;
	addr = (addr << 32) | (input_frame->planes[0].address.low);
	priv->xt.dst_start = addr;
	LOG (LOG_INFO, "RAW : DMA address is %#llx", addr);

	if ( (input_frame->planes[0].width !=  priv->param.active.width) ||
		 (input_frame->planes[0].height !=  priv->param.active.height)) {
		LOG (LOG_ERR, "Mismatch in sensor mode and raw frame resolution");
		return -EINVAL;
	}

	LOG (LOG_INFO, "Submitting DMA request width %d , height %d , pixel bits %d \n",
        priv->param.active.width,
        priv->param.active.height,
        priv->param.data_width);

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
	LOG( LOG_INFO, "cookie submitted is %d, %#llx, %#llx", cookie, priv, &priv->work);

#if 0
	if (vb2_is_streaming(vb->vb2_queue) && pstream->start_dma_async) {
		LOG( LOG_INFO, "DMA async issue pending cookie:  %d", cookie);
		dma_async_issue_pending(pstream->dma);
	}
#endif
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
	
    cfg->active.width = SENSOR_IMAGE_WIDTH;
    cfg->active.height = SENSOR_IMAGE_HEIGHT;
    cfg->total.width = SENSOR_TOTAL_WIDTH;
    cfg->total.height = SENSOR_TOTAL_HEIGHT;
    cfg->integration_time_min = SENSOR_MIN_INTEGRATION_TIME;
    cfg->integration_time_max = SENSOR_MAX_INTEGRATION_TIME;
    cfg->integration_time_limit = SENSOR_MAX_INTEGRATION_TIME_LIMIT;
    cfg->preset_mode = mode;
    cfg->lines_per_second = 0;

    LOG(LOG_INFO,"sensor active width %d, height %d , %d , %d", cfg->active.width,
					cfg->active.height,cfg->total.width, cfg->total.height);
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
	LOG( LOG_INFO, "Sensor fps control");
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
	LOG( LOG_INFO, "Request next frame");
#if 1
	if (priv->sensor_get_frame) {
		int rc = priv->sensor_get_frame( priv->owner, (void **)&input_frame);
		if (rc < 0) {
			LOG( LOG_ERR, "ERROR : getting frame from streamer");
			return;
		}

		LOG (LOG_INFO, "SUCCESS getting frame from stramer %#llx, %#x, width : %d, height : %d",
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

		LOG( LOG_INFO, "SUCCESS submitting DMA request, iter : %u", priv->last_ff_index);
		priv->last_ff_index = (priv->last_ff_index + 1) % MAX_RAW_FRAMES;	
	}
	//msleep(100);
#endif

	up(&priv->sem_isp_to_dma);

	LOG (LOG_INFO, "request_next : semaphore raised");
}

static void register_frame_callbacks( void *sensor_priv, const sensor_remote_callbacks_t *callbacks ) {

	LOG( LOG_INFO, "Adding register frame callbacks");
    sensor_private_t *priv = sensor_priv;
	priv->sensor_get_frame = callbacks->get_frame;
	priv->sensor_put_frame = callbacks->put_frame;
	priv->owner = callbacks->callback_owner;

}

static void sensor_deinit( void *sensor_priv )
{
    sensor_private_t *priv = sensor_priv;
	LOG( LOG_INFO, "Sensor deinit for context : %d", priv->ctx_id);

	cancel_work_sync(&priv->work);
	// TODO : Why is init not called if deinit called?
	//if (priv->wq)
	//	destroy_workqueue(priv->wq);

	priv->last_cb_index = 0;
	priv->last_ff_index = 0;
	priv->is_streaming = 0;
	priv->first_cb = 0;
	sema_init(&priv->sem_isp_to_dma, 0);
}

//--------------------Initialization------------------------------------------------------------

int isp_register_dma_channels(sensor_private_t *priv, int ctx_id) {

	char dma_names[64];

	if (!g_pdev) {
		LOG( LOG_ERR, "platform device is not initialised");
		return -EINVAL;
	}

	snprintf(dma_names, sizeof(dma_names)-1, "vdma%d", ctx_id);

	LOG( LOG_INFO, "DMA channel request name is %s", dma_names);

	priv->dma = dma_request_slave_channel(&(g_pdev->dev), dma_names);
	if (priv->dma == NULL) {
		LOG( LOG_INFO, "no VDMA channel found by name vdma %s", dma_names);
		return -ENODEV;
	}

	LOG( LOG_INFO, "dma chan id %d", priv->dma->chan_id);

	return 0;
}


void sensor_init_imx477( void **priv_ptr, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const options )
{
    LOG(LOG_INFO, "imx477 sensor init for ctx : %u", location);

	char wq_name[64];
    sensor_private_t *priv = *priv_ptr = priv_array + location;
    sensor_param_t *cfg = &priv->param;
	int rc = -1;

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
    ctrl->deinit = sensor_deinit;

	/* init work queue */
	snprintf(wq_name, sizeof(wq_name) - 1, "isp_wq_%d", location);
	priv->wq = alloc_workqueue(wq_name, WQ_UNBOUND, MAX_RAW_FRAMES);
	if (!priv->wq ) {
		LOG( LOG_CRIT, "failed to create work queue for context : %u", location);
		return;
	}

	INIT_WORK(&priv->work, work_queue_fn);
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
    // Reset sensor during initialization
    sensor_hw_reset_enable();
    sensor_hw_reset_disable();

    LOG( LOG_INFO, "Sensor DPattern (id 0x%04x) initialized at position %d.", sensor_get_id( priv ), location );
}

//*************************************************************************************
