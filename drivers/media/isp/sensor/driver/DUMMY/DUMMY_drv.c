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

#include "DUMMY_config.h"

#include <linux/videodev2.h>

static int bayer_pixelformat_to_rggb_start(uint32_t pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SRGGB14:
	case V4L2_PIX_FMT_SRGGB16:
		return 0; /* RGGB */
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGRBG14:
	case V4L2_PIX_FMT_SGRBG16:
		return 1; /* GRBG */
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGBRG14:
	case V4L2_PIX_FMT_SGBRG16:
		return 2; /* GBRG */
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SBGGR14:
	case V4L2_PIX_FMT_SBGGR16:
		return 3; /* BGGR */
	default:
		return -1;
	}
}

// Formatting tool makes below declaration less readable and structured
// clang-format off

#define SENSOR_MODE(w, h, bw, p) \
{ \
	.fps = 30 * 256, \
	.wdr_mode = WDR_MODE_LINEAR, \
	.pixel_format = p, \
	.resolution.width = w, \
	.resolution.height = h, \
	.data_width = bw, \
	.channel_info = { \
		.channel_desc = { \
			{ \
				.exposure_bit_width = bw, \
				.data_type = DATA_TYPE_LINEAR, \
				.cv = CAP_CHANNEL_PASS_THROUGH \
			} \
		}, \
		.exposure_idx_to_channel_map = { \
			0 \
		}, \
		.exposure_max_bit_width = bw, \
		.locked_exp_info = { \
			.locked_exp_ratio_flag = false, \
			.locked_exp_ratio_val = 0, \
			.locked_exp_ratio_short_flag = false, \
			.locked_exp_ratio_short_val = 0, \
			.locked_exp_ratio_medium_flag = false, \
			.locked_exp_ratio_medium_val = 0, \
			.locked_exp_ratio_medium2_flag = false, \
			.locked_exp_ratio_medium2_val = 0 \
		} \
	}, \
	.exposures = 1, \
	.num_channels = 1 \
}

static sensor_mode_t supported_modes[] = {
	SENSOR_MODE(1920, 1080, 12, V4L2_PIX_FMT_SRGGB12), //1920x1080 SRGGB12
	SENSOR_MODE(1920, 1080, 12, V4L2_PIX_FMT_SBGGR12), //1920x1080 SBGGR12
	SENSOR_MODE(1920, 1080, 12, V4L2_PIX_FMT_SGRBG12), //1920x1080 SGRBG12
	SENSOR_MODE(1920, 1080, 12, V4L2_PIX_FMT_SGBRG12), //1920x1080 SGBRG12

	SENSOR_MODE(1920, 1080, 10, V4L2_PIX_FMT_SRGGB10), //1920x1080 SRGGB10
	SENSOR_MODE(1920, 1080, 10, V4L2_PIX_FMT_SBGGR10), //1920x1080 SBGGR10
	SENSOR_MODE(1920, 1080, 10, V4L2_PIX_FMT_SGRBG10), //1920x1080 SGRBG10
	SENSOR_MODE(1920, 1080, 10, V4L2_PIX_FMT_SGBRG10), //1920x1080 SGBRG10

	SENSOR_MODE(2048, 1080, 12, V4L2_PIX_FMT_SRGGB12), //2048x1080 SRGGB12
	SENSOR_MODE(2048, 1080, 12, V4L2_PIX_FMT_SBGGR12), //2048x1080 SBGGR12
	SENSOR_MODE(2048, 1080, 12, V4L2_PIX_FMT_SGRBG12), //2048x1080 SGRBG12
	SENSOR_MODE(2048, 1080, 12, V4L2_PIX_FMT_SGBRG12), //2048x1080 SGBRG12

	SENSOR_MODE(2048, 1080, 10, V4L2_PIX_FMT_SRGGB10), //2048x1080 SRGGB10
	SENSOR_MODE(2048, 1080, 10, V4L2_PIX_FMT_SBGGR10), //2048x1080 SBGGR10
	SENSOR_MODE(2048, 1080, 10, V4L2_PIX_FMT_SGRBG10), //2048x1080 SGRBG10
	SENSOR_MODE(2048, 1080, 10, V4L2_PIX_FMT_SGBRG10), //2048x1080 SGBRG10

	SENSOR_MODE(2432, 2048, 12, V4L2_PIX_FMT_SRGGB12), //2432x2048 SRGGB12
	SENSOR_MODE(2432, 2048, 12, V4L2_PIX_FMT_SBGGR12), //2432x2048 SBGGR12
	SENSOR_MODE(2432, 2048, 12, V4L2_PIX_FMT_SGRBG12), //2432x2048 SGRBG12
	SENSOR_MODE(2432, 2048, 12, V4L2_PIX_FMT_SGBRG12), //2432x2048 SGBRG12

	SENSOR_MODE(2432, 2048, 10, V4L2_PIX_FMT_SRGGB10), //2432x2048 SRGGB10
	SENSOR_MODE(2432, 2048, 10, V4L2_PIX_FMT_SGBRG10), //2432x2048 SGBRG10
	SENSOR_MODE(2432, 2048, 10, V4L2_PIX_FMT_SBGGR10), //2432x2048 SBGGR10
	SENSOR_MODE(2432, 2048, 10, V4L2_PIX_FMT_SGRBG10), //2432x2048 SGRBG10
};
// clang-format on

typedef struct _sensor_private_t {
    sensor_param_t param;
    uint16_t integration_time;
    int32_t again;
    int32_t dgain;
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
}

static void sensor_set_mode( void *sensor_priv, uint8_t mode )
{
    sensor_private_t *priv = sensor_priv;
    sensor_param_t *cfg = &priv->param;

    cfg->active.width = supported_modes[mode].resolution.width;
    cfg->active.height = supported_modes[mode].resolution.height;
    cfg->data_width = supported_modes[mode].channel_info.channel_desc[0].exposure_bit_width;

    cfg->total.width = supported_modes[mode].resolution.width;
    cfg->total.height = supported_modes[mode].resolution.height;
    cfg->integration_time_min = SENSOR_MIN_INTEGRATION_TIME;
    cfg->integration_time_max = SENSOR_MAX_INTEGRATION_TIME;
    cfg->integration_time_limit = SENSOR_MAX_INTEGRATION_TIME_LIMIT;
    cfg->preset_mode = mode;
    cfg->lines_per_second = SENSOR_PIXEL_CLOCK / 1100;

    cfg->sensor_exp_number = supported_modes[mode].exposures;
    cfg->num_channel = supported_modes[mode].num_channels;
    cfg->rggb_start = bayer_pixelformat_to_rggb_start(supported_modes[mode].pixel_format);
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
    return 0;
}

static uint32_t read_register( void *sensor_priv, uint32_t address )
{
    return 0;
}

static void write_register( void *sensor_priv, uint32_t address, uint32_t data )
{
}
static void stop_streaming( void *sensor_priv ) {}

static void start_streaming( void *sensor_priv ) {}

static void request_next_frame( void *sensor_priv ) {}

static void register_frame_callbacks( void *sensor_priv, const sensor_remote_callbacks_t *callbacks ) {}

static void sensor_deinit( void *sensor_priv )
{
}

//--------------------Initialization------------------------------------------------------------
void sensor_init_dummy( void **priv_ptr, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const options )
{

    sensor_private_t *priv = *priv_ptr = priv_array + location;
    sensor_param_t *cfg = &priv->param;
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
    cfg->data_width = 12;
    cfg->rggb_start = 0;
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

    // Reset sensor during initialization
    sensor_hw_reset_enable();
    sensor_hw_reset_disable();

    LOG( LOG_INFO, "Sensor DPattern (id 0x%04x) initialized at position %d.", sensor_get_id( priv ), location );
}

//*************************************************************************************
