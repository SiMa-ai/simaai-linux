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

#include "system_timer.h"
#include "system_types.h"
#include "acamera_command_api.h"
#include "acamera_frame_stream_api.h"
#include "acamera_frontend_config.h"
#include "acamera_isp_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "acamera_math.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_SENSOR

#define OFFSET_BLACK_DEFAULT 0xF0000

// FSM event handler

void sensor_init( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t ctx_id = p_ictx->context_id;

    LOG( LOG_INFO, "%s ctx %u E", __FUNCTION__, ctx_id );

    // Init sensor hardware
    p_ictx->settings.sensor_init( &p_fsm->drv_priv, ctx_id, &p_fsm->ctrl, &p_ictx->settings.sensor_options );

    // Get sensor parameters
    p_fsm->s_param = p_fsm->ctrl.get_parameters( p_fsm->drv_priv );

    // Sensor preset mode
    const uint32_t preset_mode = p_ictx->settings.sensor_options.preset_mode;
	LOG (LOG_INFO, "preset mode set to %u, supported : %u", preset_mode, p_fsm->s_param->modes_num);

    // Set default paramemters values: preset mode, supported presets, width and height
    // This information should be available for V4L2 at init state
    override_context_param( p_ictx, SENSOR_PRESET_PARAM, preset_mode );
    override_context_param( p_ictx, SENSOR_HEIGHT_PARAM, p_fsm->s_param->modes_table[preset_mode].resolution.height );
    override_context_param( p_ictx, SENSOR_WIDTH_PARAM, p_fsm->s_param->modes_table[preset_mode].resolution.width );
    override_context_param( p_ictx, SENSOR_SUPPORTED_PRESETS_PARAM, p_fsm->s_param->modes_num );


    /* @todo check if this value can be hardcoded or should we add it to
    calibration. Testing locally I haven't really observed a big difference. */
    acamera_isp_offset_black_00_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );
    acamera_isp_offset_black_01_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );
    acamera_isp_offset_black_10_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );
    acamera_isp_offset_black_11_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );

    acamera_isp_pipeline_bypass_sensor_offset_linear_write( p_ictx->settings.isp_base, 0 );

    // Initialise remote sensor callbacks
    if ( p_fsm->s_param->is_remote ) {
        if ( p_fsm->ctrl.register_frame_callbacks ) {

            const sensor_remote_callbacks_t callbacks = {
                .callback_owner = p_fsm,
                .get_frame = remote_sensor_get_frame_callback_handler,
                .put_frame = remote_sensor_put_frame_callback_handler,
                .release_frame = remote_sensor_release_frame_callback_handler};

            p_fsm->ctrl.register_frame_callbacks( p_fsm->drv_priv, &callbacks );
        } else {
            LOG( LOG_ERR, "Error. Sensor API 'register_frame_callbacks' is not initialized (NULL).\n" );
        }
    }
}

void copy_sensor_mode( general_sensor_mode_t *dest, sensor_mode_t *source )
{

    uint8_t i = 0U;

    dest->resolution.height = source->resolution.height;
    dest->resolution.width = source->resolution.width;

    for ( i = 0U; i < GENERAL_ROUTER_MAX_ISP_CHANNELS; ++i ) {
        dest->channel_info.channel_desc[i].exposure_bit_width = source->channel_info.channel_desc[i].exposure_bit_width;
        dest->channel_info.channel_desc[i].data_type = (uint8_t)source->channel_info.channel_desc[i].data_type;
        dest->channel_info.channel_desc[i].cv = (uint8_t)source->channel_info.channel_desc[i].cv;

        dest->channel_info.exposure_idx_to_channel_map[i] = source->channel_info.exposure_idx_to_channel_map[i];
    }

    dest->channel_info.locked_exp_info.locked_exp_ratio_flag = source->channel_info.locked_exp_info.locked_exp_ratio_flag;
    dest->channel_info.locked_exp_info.locked_exp_ratio_val = source->channel_info.locked_exp_info.locked_exp_ratio_val;
    dest->channel_info.locked_exp_info.locked_exp_ratio_short_flag = source->channel_info.locked_exp_info.locked_exp_ratio_short_flag;
    dest->channel_info.locked_exp_info.locked_exp_ratio_short_val = source->channel_info.locked_exp_info.locked_exp_ratio_short_val;
    dest->channel_info.locked_exp_info.locked_exp_ratio_medium_flag = source->channel_info.locked_exp_info.locked_exp_ratio_medium_flag;
    dest->channel_info.locked_exp_info.locked_exp_ratio_medium_val = source->channel_info.locked_exp_info.locked_exp_ratio_medium_val;
    dest->channel_info.locked_exp_info.locked_exp_ratio_medium2_flag = source->channel_info.locked_exp_info.locked_exp_ratio_medium2_flag;
    dest->channel_info.locked_exp_info.locked_exp_ratio_medium2_val = source->channel_info.locked_exp_info.locked_exp_ratio_medium2_val;

    dest->fps = source->fps;
    dest->wdr_mode = source->wdr_mode;
    dest->exposures = source->exposures;
    dest->num_channels = source->num_channels;
}

void sensor_config( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    LOG( LOG_INFO, "%s ctx %d E", __FUNCTION__, p_ictx->context_id );

    // Get sensor parameters.
    p_fsm->s_param = p_fsm->ctrl.get_parameters( p_fsm->drv_priv );

    // Get preset mode and do range check.
    uint32_t preset_mode = get_context_param( p_ictx, SENSOR_PRESET_PARAM );

    // Check preset mode
    if ( preset_mode >= p_fsm->s_param->modes_num ) {
        LOG( LOG_WARNING, "Sensor preset (%u) is out of range [0;%u) and will be truncated.", preset_mode, p_fsm->s_param->modes_num );

        if ( p_fsm->s_param->modes_num > 0 ) {
            preset_mode = p_fsm->s_param->modes_num - 1;
        } else {
            preset_mode = 0;
        }

        override_context_param( p_ictx, SENSOR_PRESET_PARAM, preset_mode );
    }

    // Check if required number of input ports available (not needed for remote sensors)
    if ( !p_fsm->s_param->is_remote ) {

        int available_inputs = 0;
        WRAP_GENERAL_CMD( p_ictx, CMD_ID_MCFE_GET_SLOT_AVAIL_INPUTS, CMD_DIRECTION_GET, NULL, &available_inputs );

        if ( p_fsm->s_param->modes_table[preset_mode].num_channels > available_inputs ) {
            LOG( LOG_ERR, "Failed to set new sensor preset %d. Not enough free input ports available (required: %d, available: %d).",
                 preset_mode,
                 p_fsm->s_param->modes_table[preset_mode].num_channels,
                 available_inputs );
            LOG( LOG_ERR, "Sensor preset is set back to: %d.", p_fsm->s_param->preset_mode );

            // Reset sensor preset param to current valid preset
            override_context_param( p_ictx, SENSOR_PRESET_PARAM, p_fsm->s_param->preset_mode );

            return;
        }
    }

    // Update sensor hardware with given preset mode.
    p_fsm->ctrl.set_mode( p_fsm->drv_priv, preset_mode );

    // Set WDR mode context parameter.
    override_context_param( p_ictx, SENSOR_WDR_MODE_PARAM, p_fsm->s_param->modes_table[preset_mode].wdr_mode );

    // Update general_sensor_info.
    p_fsm->general_sensor_info.total_width = p_fsm->s_param->total.width;
    p_fsm->general_sensor_info.total_height = p_fsm->s_param->total.height;
    p_fsm->general_sensor_info.active_width = p_fsm->s_param->active.width;
    p_fsm->general_sensor_info.active_height = p_fsm->s_param->active.height;
    p_fsm->general_sensor_info.black_level = p_fsm->black_level;
    p_fsm->general_sensor_info.lines_per_second = p_fsm->s_param->lines_per_second;
    p_fsm->general_sensor_info.cfa_pattern = p_fsm->s_param->cfa_pattern;
    p_fsm->general_sensor_info.rggb_start = p_fsm->s_param->rggb_start;

    p_fsm->general_sensor_info.again_log2_max = p_fsm->s_param->again_log2_max;
    p_fsm->general_sensor_info.dgain_log2_max = p_fsm->s_param->dgain_log2_max;
    p_fsm->general_sensor_info.again_accuracy = p_fsm->s_param->again_accuracy;
    p_fsm->general_sensor_info.wb_gain_log2_max = p_fsm->s_param->wb_gain_log2_max;

    p_fsm->general_sensor_info.integration_time_min = p_fsm->s_param->integration_time_min;
    p_fsm->general_sensor_info.integration_time_max = p_fsm->s_param->integration_time_max;
    p_fsm->general_sensor_info.integration_time_long_max = p_fsm->s_param->integration_time_long_max;
    p_fsm->general_sensor_info.integration_time_medium_max = p_fsm->s_param->integration_time_medium_max;
    p_fsm->general_sensor_info.integration_time_limit = p_fsm->s_param->integration_time_limit;
    p_fsm->general_sensor_info.integration_time_precision = p_fsm->s_param->integration_time_precision;
    p_fsm->general_sensor_info.integration_time_apply_delay = p_fsm->s_param->integration_time_apply_delay;

    p_fsm->general_sensor_info.sensor_exp_number = p_fsm->s_param->sensor_exp_number;
    p_fsm->general_sensor_info.isp_exposure_channel_delay = p_fsm->s_param->isp_exposure_channel_delay;
    p_fsm->general_sensor_info.sensor_output_bits = p_fsm->s_param->data_width;

    p_fsm->general_sensor_info.is_remote = p_fsm->s_param->is_remote;

    /*Copy over current sensor mode to general info*/
    copy_sensor_mode( &p_fsm->general_sensor_info.current_sensor_mode, &p_fsm->s_param->modes_table[p_fsm->s_param->preset_mode] );

    // Update context parameters.
    override_context_param( p_ictx, SENSOR_HEIGHT_PARAM, p_fsm->s_param->modes_table[preset_mode].resolution.height );
    override_context_param( p_ictx, SENSOR_WIDTH_PARAM, p_fsm->s_param->modes_table[preset_mode].resolution.width );
    override_context_param( p_ictx, SENSOR_FPS_PARAM, ( p_fsm->s_param->lines_per_second << 8 ) / p_fsm->s_param->total.height );
    override_context_param( p_ictx, SENSOR_EXPOSURES_PARAM, p_fsm->s_param->sensor_exp_number );
    override_context_param( p_ictx, SENSOR_SUPPORTED_PRESETS_PARAM, p_fsm->s_param->modes_num );
    override_context_param( p_ictx, SENSOR_INTEGRATION_TIME_MIN_PARAM, p_fsm->s_param->integration_time_min );
    override_context_param( p_ictx, SENSOR_INTEGRATION_TIME_LIMIT_PARAM, p_fsm->s_param->integration_time_limit );
}

void sensor_ready( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    uint32_t ctx_id = p_ictx->context_id;

    if ( get_context_param( p_ictx, SENSOR_STREAMING_PARAM ) == 0 && p_fsm->ctrl.start_streaming != NULL ) {
#if ( ISP_RTL_VERSION_R == 0 )
        // We need a delay between sensors starts,
        // otherwise histograms comes at the same time and on R0 only one is served, resulting in AE not working.
        // Problem only present on split_architecture because of mcfe_fsm starting sensors at the same time.
        system_timer_usleep( 7000 );
#endif
        LOG( LOG_INFO, "Start streaming (pos = %u)", ctx_id );

        // Call sensor API.
        p_fsm->ctrl.start_streaming( p_fsm->drv_priv );

        // Update context param (override used to update read-only param).
        override_context_param( p_ictx, SENSOR_STREAMING_PARAM, 1 );
    }
}

void sensor_stopped( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    uint32_t ctx_id = p_ictx->context_id;

    if ( get_context_param( p_ictx, SENSOR_STREAMING_PARAM ) == 1 && p_fsm->ctrl.stop_streaming != NULL ) {
        LOG( LOG_INFO, "Stop streaming (pos = %u)", ctx_id );

        // Call sensor API
        p_fsm->ctrl.stop_streaming( p_fsm->drv_priv );

        // Update context param (override used to update read-only param).
        override_context_param( p_ictx, SENSOR_STREAMING_PARAM, 0 );

#if !MODEL_MODULE
        // Release sensor resources
        if ( p_fsm->ctrl.deinit ) {
            p_fsm->ctrl.deinit( p_fsm->drv_priv );
        } else {
            LOG( LOG_ERR, "Error. Sensor API 'deinit' is not initialized (NULL).\n" );
        }
#endif
    }
}

static uint32_t get_exposure_bit_width_by_id( sensor_fsm_ptr_t p_fsm, uint8_t id )
{
    const sensor_param_t *s_param = p_fsm->ctrl.get_parameters( p_fsm->drv_priv );
    const sensor_mode_t *s_mode = &s_param->modes_table[s_param->preset_mode];

    if ( id < s_mode->num_channels ) {
        const int channel_id = s_mode->channel_info.exposure_idx_to_channel_map[id];
        return s_mode->channel_info.channel_desc[channel_id].exposure_bit_width;
    }

    return 0;
}

void sensor_update_hw( sensor_fsm_ptr_t p_fsm )
{
    int stub = 0;
    exposure_set_t exp;

    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    WRAP_GENERAL_CMD( p_ictx, CMD_ID_FRAME_EXPOSURE_SET, CMD_DIRECTION_GET, &stub, &exp );
    const uint16_t again_log2 = ( uint16_t )( exp.info.again_log2 >> ( LOG2_GAIN_SHIFT - 5 ) ); // Makes again in format log2( gain ) * 32.

    const uint32_t ldr_gain_log2 = get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM );

    const uint32_t idx_r = CALIBRATION_BLACK_LEVEL_R;
    const uint32_t idx_b = CALIBRATION_BLACK_LEVEL_B;
    const uint32_t idx_gr = CALIBRATION_BLACK_LEVEL_GR;
    const uint32_t idx_gb = CALIBRATION_BLACK_LEVEL_GB;

    const uint32_t bl_r = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ) );
    const uint32_t bl_b = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ) );
    const uint32_t bl_gr = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ) );
    const uint32_t bl_gb = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ) );


    const uint32_t bl_r_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ) );
    const uint32_t bl_b_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ) );
    const uint32_t bl_gr_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ) );
    const uint32_t bl_gb_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ) );

    const uint32_t wdr_mode = get_context_param( p_ictx, SENSOR_WDR_MODE_PARAM );

    // Update black-level FSM variable with average black level.
    p_fsm->black_level = ( bl_r + bl_gr + bl_gb + bl_b ) >> 2;

    const sensor_param_t *s_param = p_fsm->ctrl.get_parameters( p_fsm->drv_priv );
    const sensor_mode_t *s_mode = &s_param->modes_table[s_param->preset_mode];

    const uint32_t black_level_shift_dg = ISP_DIGITAL_GAIN_INPUT_BIT_WIDTH - s_mode->channel_info.exposure_max_bit_width;
    const uint32_t black_level_dg = p_fsm->black_level << black_level_shift_dg;

    // Coefficient calculation depends on wdr_mode.
    if ( ( wdr_mode == WDR_MODE_FS_LIN || wdr_mode == WDR_MODE_FS_HDR ) && ( get_context_param( p_ictx, ISP_MODULES_MANUAL_FRAME_STITCH_PARAM ) == 0 ) ) {

        uint8_t i;
        for ( i = 0; i < s_mode->exposures; ++i ) {
            const uint32_t exposure_bit_width = get_exposure_bit_width_by_id( p_fsm, i );
            if ( !exposure_bit_width ) {
                LOG( LOG_ERR, "Exposure (id: %u) bit-width is set to zero, check sensor configuration.", i );
                break;
            }

            const int32_t black_level_stitch_shift_fs = ISP_FRAME_STITCH_INPUT_BIT_WIDTH - exposure_bit_width;
            if ( black_level_stitch_shift_fs < 0 ) {
                LOG( LOG_ERR, "Frame stitch black level shift is a negative value, check sensor configuration." );
                break;
            }

            switch ( i ) {
            case 0:
                acamera_isp_frame_stitch_black_level_long_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            case 1:
                acamera_isp_frame_stitch_black_level_medium_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            case 2:
                acamera_isp_frame_stitch_black_level_short_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            case 3:
                acamera_isp_frame_stitch_black_level_very_short_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            default:
                LOG( LOG_ERR, "Invalid exposure id (%u).", i );
                break;
            }
        }
    }

    // Update black-levels in pipeline when it's not manual.
    if ( get_context_param( p_ictx, ISP_MODULES_MANUAL_BLACK_LEVEL_PARAM ) == 0 ) {
        acamera_isp_offset_black_00_write( p_ictx->settings.isp_base, bl_r_low << black_level_shift_dg );
        acamera_isp_offset_black_01_write( p_ictx->settings.isp_base, bl_gr_low << black_level_shift_dg );
        acamera_isp_offset_black_10_write( p_ictx->settings.isp_base, bl_gb_low << black_level_shift_dg );
        acamera_isp_offset_black_11_write( p_ictx->settings.isp_base, bl_b_low << black_level_shift_dg );
#ifdef ACAMERA_ISP_DIGITAL_GAIN_OFFSET_DEFAULT
        acamera_isp_digital_gain_offset_write( p_ictx->settings.isp_base, black_level_dg );
#endif

// FE/BE and FS blackLevel set to the average of offset blackLevel.
#ifdef ACAMERA_ISP_GAMMA_FE_BLACK_LEVEL_IN_DL_DEFAULT // r0
        acamera_isp_gamma_fe_black_level_in_dl_write( p_ictx->settings.isp_base, black_level_dg );
        acamera_isp_gamma_be_black_level_out_dl_write( p_ictx->settings.isp_base, black_level_dg );
#endif
#ifdef ACAMERA_ISP_GAMMA_FE_BLACK_LEVEL_IN_SQ_DEFAULT // r1
        acamera_isp_gamma_fe_black_level_in_sq_write( p_ictx->settings.isp_base, black_level_dg );
        acamera_isp_gamma_be_black_level_out_sq_write( p_ictx->settings.isp_base, black_level_dg );
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_BLACK_LEVEL_OUT_DEFAULT
        acamera_isp_frame_stitch_black_level_out_write( p_ictx->settings.isp_base, black_level_dg );
#endif
    }
}

void sensor_request_next_frame( sensor_fsm_ptr_t p_fsm )
{

    // Check if sensor is remote type and request next frame.
    if ( p_fsm->general_sensor_info.is_remote ) {

        if ( p_fsm->ctrl.request_next_frame ) {

            p_fsm->ctrl.request_next_frame( p_fsm->drv_priv );
        } else {
            LOG( LOG_ERR, "Sensor API 'request_next_frame' is not defined (NULL)." );
        }
    }
}

////////////////////////////////////////////////////
// General router command handler

void sensor_get_general_sensor_info( sensor_fsm_ptr_t p_fsm, acamera_cmd_sensor_info *p_sensor_info )
{
    // Update black-level and copy p_fsm->general_sensor_info.
    p_fsm->general_sensor_info.black_level = p_fsm->black_level;
    *p_sensor_info = p_fsm->general_sensor_info;
}

int sensor_get_general_sensor_preset_info( sensor_fsm_ptr_t p_fsm, uint32_t sensor_preset_id, acamera_cmd_sensor_preset_info_t *p_sensor_preset_info )
{
    if ( sensor_preset_id >= p_fsm->s_param->modes_num ) {
        return -1;
    }

    p_sensor_preset_info->width = p_fsm->s_param->modes_table[sensor_preset_id].resolution.width;
    p_sensor_preset_info->height = p_fsm->s_param->modes_table[sensor_preset_id].resolution.height;
    p_sensor_preset_info->fps = p_fsm->s_param->modes_table[sensor_preset_id].fps;
    p_sensor_preset_info->wdr_mode = p_fsm->s_param->modes_table[sensor_preset_id].wdr_mode;
    p_sensor_preset_info->exposures = p_fsm->s_param->modes_table[sensor_preset_id].exposures;
    p_sensor_preset_info->num_channels = p_fsm->s_param->modes_table[sensor_preset_id].num_channels;
    p_sensor_preset_info->data_width = p_fsm->s_param->data_width;

    return 0;
}

int remote_sensor_put_frame_callback_handler( void *owner, void *frame )
{
#if !MODEL_MODULE
    if ( !owner ) {
        LOG( LOG_ERR, "Error. Invalid owner pointer (NULL)." );
        return -1;
    } else if ( !frame ) {
        LOG( LOG_ERR, "Error. Invalid frame pointer (NULL)." );
        return -1;
    }

    const sensor_fsm_ptr_t p_fsm = owner;
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const int ctx_id = (int)( p_ictx->context_id );

    // Set frame state to filled and return it to the frame streamer
    ( (aframe_t *)frame )->state = AFRAME_STATE_FULL;
	// TODO : hack to avoid putting the frame back to application m2m already does it
#if 0 
    if ( frame_stream_put_frame( (aframe_t *)frame ) ) {
        LOG( LOG_ERR, "Failed to put filled raw frame from the frame streamer." );
        return -1;
    }
#endif

    fsm_raise_event( p_fsm, event_id_isphw_frame_end_fe );

    //LOG( LOG_INFO, "Filled raw frame is ready to be fetched by the MCFE (context id: %d, ptr: %p).", ctx_id, frame );

#endif
    return 0;
}

int remote_sensor_get_frame_callback_handler( void *owner, void **frame )
{
#if !MODEL_MODULE
    if ( !owner ) {
        LOG( LOG_ERR, "Error. Invalid owner pointer (NULL)." );
        return -1;
    } else if ( !frame ) {
        LOG( LOG_ERR, "Error. Invalid frame pointer (NULL)." );
        return -1;
    }

    const sensor_fsm_ptr_t p_fsm = owner;
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const int ctx_id = (int)( p_ictx->context_id );

    if ( frame_stream_get_frame( ctx_id, AFRAME_TYPE_RAW, AFRAME_STATE_EMPTY, (aframe_t **)frame ) ) {
        LOG( LOG_ERR, "Failed to get empty raw frame from the frame streamer." );
        *frame = NULL;
        return -1;
    }

    //LOG( LOG_INFO, "Empty raw frame acquired (context id: %d, ptr: %p).", ctx_id, *frame );

#endif
    return 0;
}

int remote_sensor_release_frame_callback_handler( void *owner, void *frame )
{
#if !MODEL_MODULE
    if ( !owner ) {
        LOG( LOG_ERR, "Error. Invalid owner pointer (NULL)." );
        return -1;
    } else if ( !frame ) {
        LOG( LOG_ERR, "Error. Invalid frame pointer (NULL)." );
        return -1;
    }

    const sensor_fsm_ptr_t p_fsm = owner;
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const int ctx_id = (int)( p_ictx->context_id );

    ( (aframe_t *)frame )->state = AFRAME_STATE_EMPTY;

    if ( frame_stream_put_frame( frame ) ) {
        LOG( LOG_ERR, "Failed to put raw frame to the frame streamer." );
        return -1;
    }

    //LOG( LOG_INFO, "Raw frame released (context id: %d, ptr: %p).", ctx_id, frame );

#endif
    return 0;
}

void sensor_deinit( sensor_fsm_ptr_t p_fsm )
{
#if !MODEL_MODULE
    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    acamera_isp_pipeline_bypass_sensor_offset_wdr_write( isp_base, 1 );
    acamera_isp_pipeline_bypass_sensor_offset_linear_write( isp_base, 1 );

    if ( p_fsm->ctrl.deinit ) {
        p_fsm->ctrl.deinit( p_fsm->drv_priv );
    } else {
        LOG( LOG_ERR, "Error. Sensor API 'deinit' is not initialized (NULL).\n" );
    }
#endif
}
