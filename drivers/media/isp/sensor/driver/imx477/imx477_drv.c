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
#include <linux/i2c.h>
#include <linux/unaligned.h>

#include <linux/simaai-stu.h>

extern struct simaai_stu *stu;
struct workqueue_struct *wq;

// Filled in system_cma.c
extern struct platform_device *g_pdev;

#define MAX_RAW_FRAMES			(5)
extern int32_t get_calibrations_imx477( uint32_t wdr_mode, void *param );
extern acamera_settings *get_settings_by_id(u8 ctx_id);
extern  int8_t get_dma_index(uint32_t ctx_id);

// Formatting tool makes below declaration less readable and structured
// clang-format off
static sensor_mode_t supported_modes[] = {
    {
        .fps = 30 * 256,
        .wdr_mode = WDR_MODE_LINEAR,
        .resolution.width = 2048,
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
    },
    {   // PWL
        .fps = 30 * 256,
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
    },
    {
        .fps = 10 * 256,
        .wdr_mode = WDR_MODE_LINEAR,
        .resolution.width = 4032,
        .resolution.height =3040,
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
    uint16_t integration_time;
    int32_t again;
    int32_t dgain;
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

//--------------------FLASH------------------------------------------------------------

static void write_register( void *sensor_priv, uint32_t address, uint32_t data );

static int32_t sensor_alloc_analog_gain( void *sensor_priv, int32_t gain )
{
    sensor_private_t *priv = sensor_priv;
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

}

static void sensor_update( void *sensor_priv )
{
    (void)sensor_priv; //unusued
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

static void stop_streaming( void *sensor_priv ) {

	LOG( LOG_INFO, "STOP streaming is called");

    sensor_private_t *priv = sensor_priv;
	priv->is_streaming = 0;
}

static void start_streaming( void *sensor_priv ) {

	LOG( LOG_INFO, "START streaming is called");

    sensor_private_t *priv = sensor_priv;
	priv->is_streaming = 1;
}

static void register_frame_callbacks( void *sensor_priv, const sensor_remote_callbacks_t *callbacks ) {

    sensor_private_t *priv = sensor_priv;
	priv->sensor_get_frame = callbacks->get_frame;
	priv->sensor_put_frame = callbacks->put_frame;
	priv->owner = callbacks->callback_owner;

}

static void sensor_deinit_imx477( void *sensor_priv )
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

void sensor_init_imx477( void **priv_ptr, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const options )
{
    LOG(LOG_DEBUG, "imx477 sensor init for ctx : %u", location);

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
    ctrl->start_streaming = start_streaming;
    ctrl->stop_streaming = stop_streaming;
    ctrl->register_frame_callbacks = register_frame_callbacks;
    ctrl->deinit = sensor_deinit_imx477;

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

	if (ctx_settings) {

		ctx_settings->get_calibrations = get_calibrations_imx477;
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
