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

#include "acamera_frontend_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"

#define MANUAL_TRIGGER_DEFAULT 0
#define FIELD_MODE_DEFAULT 0

#define SENSOR_HBLANK_MIN_DEFAULT 8
#define SENSOR_HBLANK_MAX_DEFAULT 65535
#define SENSOR_VBLANK_MIN_DEFAULT 1
#define SENSOR_VBLANK_MAX_DEFAULT 0xffffffff

#define ISP_HBLANK_MIN_DEFAULT ACAMERA_ISP_FRAME_CHECK_ISP_HBLANK_MIN_DEFAULT
#define ISP_HBLANK_MAX_DEFAULT ACAMERA_ISP_FRAME_CHECK_ISP_HBLANK_MAX_DEFAULT
#define ISP_VBLANK_MIN_DEFAULT ACAMERA_ISP_FRAME_CHECK_ISP_VBLANK_MIN_DEFAULT
#define ISP_VBLANK_MAX_DEFAULT ACAMERA_ISP_FRAME_CHECK_ISP_VBLANK_MAX_DEFAULT

/**
 * @brief      Fetches a sensor configuration and populates the FSM fields.
 *
 * @param[in]  p_fsm  The FSM pointer
 *
 */
static void get_sensor_config( frame_check_fsm_ptr_t p_fsm )
{
    acamera_cmd_sensor_info sensor_info;
    WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ),
                      CMD_ID_SENSOR_INFO,
                      CMD_DIRECTION_GET,
                      NULL, (uint32_t *)&sensor_info );

    p_fsm->sensor_config.height = sensor_info.active_height;
    p_fsm->sensor_config.width = sensor_info.active_width;
    p_fsm->sensor_config.hblank_max = sensor_info.total_width - sensor_info.active_width;
}

/**
 * @brief      Fetches a mcfe configuration and populates the FSM fields.
 *
 * @param[in]  p_fsm  The FSM pointer
 *
 */
static void get_mcfe_config( frame_check_fsm_ptr_t p_fsm )
{
    p_fsm->mcfe_config.hblank_max = acamera_frontend_mcfe_output_hblank_read( PHY_ADDR_ISP );
}


static void get_output_config( frame_check_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    int i;

    output_formatter_cfg_t output_formatter_config;
    WRAP_GENERAL_CMD( p_ictx, CMD_ID_OUTPUT_FORMATTER_DATA, CMD_DIRECTION_GET, NULL, (uint32_t *)&output_formatter_config );

    p_fsm->output_config.num_planes = output_formatter_config.num_planes;
    p_fsm->output_config.crop.width = get_context_param( p_ictx, IMAGE_CROP_WIDTH_ID_PARAM );
    p_fsm->output_config.crop.height = get_context_param( p_ictx, IMAGE_CROP_HEIGHT_ID_PARAM );
    p_fsm->output_config.crop.enabled = get_context_param( p_ictx, IMAGE_CROP_ENABLE_ID_PARAM );

#if defined( ISP_HAS_RGB_SCALER_FSM )
    p_fsm->output_config.rgb_scaler.width = get_context_param( p_ictx, IMAGE_RGB_SCALER_WIDTH_ID_PARAM );
    p_fsm->output_config.rgb_scaler.height = get_context_param( p_ictx, IMAGE_RGB_SCALER_HEIGHT_ID_PARAM );
    p_fsm->output_config.rgb_scaler.enabled = get_context_param( p_ictx, IMAGE_RGB_SCALER_ENABLE_ID_PARAM );
#endif

#if defined( ISP_HAS_RAW_SCALER_FSM )
    p_fsm->output_config.raw_scaler.width = get_context_param( p_ictx, IMAGE_RAW_SCALER_WIDTH_ID_PARAM );
    p_fsm->output_config.raw_scaler.height = get_context_param( p_ictx, IMAGE_RAW_SCALER_HEIGHT_ID_PARAM );
    p_fsm->output_config.raw_scaler.enabled = get_context_param( p_ictx, IMAGE_RAW_SCALER_ENABLE_ID_PARAM );
#endif

    for ( i = 0; i < p_fsm->output_config.num_planes; i++ ) {
        p_fsm->output_config.plane[i].axi = output_formatter_config.plane[i].axi;
        p_fsm->output_config.plane[i].h_subsampling = output_formatter_config.plane[i].axi_cfg.h_subsampling;
        p_fsm->output_config.plane[i].v_subsampling = output_formatter_config.plane[i].axi_cfg.v_subsampling;
    }
}

/**
 * @brief      Initializes the FSM and hardware associated.
 *
 * @param[in]  p_fsm  The FSM pointer
 *
 */
void frame_check_init( frame_check_fsm_ptr_t p_fsm )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    static int frontend_initalized = 0;

    if ( 0 == frontend_initalized ) {

        acamera_frontend_frame_check_sensor_1_manual_trigger_write( PHY_ADDR_ISP, MANUAL_TRIGGER_DEFAULT );
        acamera_frontend_frame_check_sensor_1_field_mode_write( PHY_ADDR_ISP, FIELD_MODE_DEFAULT );
        acamera_frontend_frame_check_sensor_1_hblank_min_write( PHY_ADDR_ISP, SENSOR_HBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_1_hblank_max_write( PHY_ADDR_ISP, SENSOR_HBLANK_MAX_DEFAULT );
        acamera_frontend_frame_check_sensor_1_vblank_min_write( PHY_ADDR_ISP, SENSOR_VBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_1_vblank_max_write( PHY_ADDR_ISP, SENSOR_VBLANK_MAX_DEFAULT );

        acamera_frontend_frame_check_sensor_2_manual_trigger_write( PHY_ADDR_ISP, MANUAL_TRIGGER_DEFAULT );
        acamera_frontend_frame_check_sensor_2_field_mode_write( PHY_ADDR_ISP, FIELD_MODE_DEFAULT );
        acamera_frontend_frame_check_sensor_2_hblank_min_write( PHY_ADDR_ISP, SENSOR_HBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_2_hblank_max_write( PHY_ADDR_ISP, SENSOR_HBLANK_MAX_DEFAULT );
        acamera_frontend_frame_check_sensor_2_vblank_min_write( PHY_ADDR_ISP, SENSOR_VBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_2_vblank_max_write( PHY_ADDR_ISP, SENSOR_VBLANK_MAX_DEFAULT );

        acamera_frontend_frame_check_sensor_3_manual_trigger_write( PHY_ADDR_ISP, MANUAL_TRIGGER_DEFAULT );
        acamera_frontend_frame_check_sensor_3_field_mode_write( PHY_ADDR_ISP, FIELD_MODE_DEFAULT );
        acamera_frontend_frame_check_sensor_3_hblank_min_write( PHY_ADDR_ISP, SENSOR_HBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_3_hblank_max_write( PHY_ADDR_ISP, SENSOR_HBLANK_MAX_DEFAULT );
        acamera_frontend_frame_check_sensor_3_vblank_min_write( PHY_ADDR_ISP, SENSOR_VBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_3_vblank_max_write( PHY_ADDR_ISP, SENSOR_VBLANK_MAX_DEFAULT );

        acamera_frontend_frame_check_sensor_4_manual_trigger_write( PHY_ADDR_ISP, MANUAL_TRIGGER_DEFAULT );
        acamera_frontend_frame_check_sensor_4_field_mode_write( PHY_ADDR_ISP, FIELD_MODE_DEFAULT );
        acamera_frontend_frame_check_sensor_4_hblank_min_write( PHY_ADDR_ISP, SENSOR_HBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_4_hblank_max_write( PHY_ADDR_ISP, SENSOR_HBLANK_MAX_DEFAULT );
        acamera_frontend_frame_check_sensor_4_vblank_min_write( PHY_ADDR_ISP, SENSOR_VBLANK_MIN_DEFAULT );
        acamera_frontend_frame_check_sensor_4_vblank_max_write( PHY_ADDR_ISP, SENSOR_VBLANK_MAX_DEFAULT );

        frontend_initalized = 1;
    }

    acamera_isp_frame_check_isp_hblank_min_write( p_ictx->settings.isp_base, ISP_HBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_hblank_max_write( p_ictx->settings.isp_base, ISP_HBLANK_MAX_DEFAULT );
    acamera_isp_frame_check_isp_vblank_min_write( p_ictx->settings.isp_base, ISP_VBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_vblank_max_write( p_ictx->settings.isp_base, ISP_VBLANK_MAX_DEFAULT );

    acamera_isp_frame_check_isp_out_1_hblank_min_write( p_ictx->settings.isp_base, ISP_HBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_out_1_hblank_max_write( p_ictx->settings.isp_base, ISP_HBLANK_MAX_DEFAULT );
    acamera_isp_frame_check_isp_out_1_vblank_min_write( p_ictx->settings.isp_base, ISP_VBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_out_1_vblank_max_write( p_ictx->settings.isp_base, ISP_VBLANK_MAX_DEFAULT );

    acamera_isp_frame_check_isp_out_2_hblank_min_write( p_ictx->settings.isp_base, ISP_HBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_out_2_hblank_max_write( p_ictx->settings.isp_base, ISP_HBLANK_MAX_DEFAULT );
    acamera_isp_frame_check_isp_out_2_vblank_min_write( p_ictx->settings.isp_base, ISP_VBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_out_2_vblank_max_write( p_ictx->settings.isp_base, ISP_VBLANK_MAX_DEFAULT );

    acamera_isp_frame_check_isp_out_3_hblank_min_write( p_ictx->settings.isp_base, ISP_HBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_out_3_hblank_max_write( p_ictx->settings.isp_base, ISP_HBLANK_MAX_DEFAULT );
    acamera_isp_frame_check_isp_out_3_vblank_min_write( p_ictx->settings.isp_base, ISP_VBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_out_3_vblank_max_write( p_ictx->settings.isp_base, ISP_VBLANK_MAX_DEFAULT );

    acamera_isp_frame_check_isp_crop_hblank_min_write( p_ictx->settings.isp_base, ISP_HBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_crop_hblank_max_write( p_ictx->settings.isp_base, ISP_HBLANK_MAX_DEFAULT );
    acamera_isp_frame_check_isp_crop_vblank_min_write( p_ictx->settings.isp_base, ISP_VBLANK_MIN_DEFAULT );
    acamera_isp_frame_check_isp_crop_vblank_max_write( p_ictx->settings.isp_base, ISP_VBLANK_MAX_DEFAULT );
}


/**
 * @brief      Configures the FSM and hardware associated.
 *
 * @param[in]  p_fsm  The FSM pointer
 *
 */
void frame_check_config( frame_check_fsm_ptr_t p_fsm )
{
    int input, slot, plane;
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    // Get basic configuration for the current context
    get_sensor_config( p_fsm );
    get_mcfe_config( p_fsm );
    get_output_config( p_fsm );

    // Check if slot uses more than one input and update sensor frame checkers configuration accordingly
    for ( input = 0; input < MODULE_MCFE_INPUT_PORT_MAX; ++input ) {

        slot = module_mcfe_get_slot_id_for_input( input, 0 );

        if ( ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id == slot ) {

            switch ( input ) {
            case 0:
                acamera_frontend_frame_check_sensor_1_active_width_write( PHY_ADDR_ISP, p_fsm->sensor_config.width );
                acamera_frontend_frame_check_sensor_1_active_height_write( PHY_ADDR_ISP, p_fsm->sensor_config.height );
                acamera_frontend_frame_check_sensor_1_hblank_max_write( PHY_ADDR_ISP, p_fsm->sensor_config.hblank_max );
                break;
            case 1:
                acamera_frontend_frame_check_sensor_2_active_width_write( PHY_ADDR_ISP, p_fsm->sensor_config.width );
                acamera_frontend_frame_check_sensor_2_active_height_write( PHY_ADDR_ISP, p_fsm->sensor_config.height );
                acamera_frontend_frame_check_sensor_2_hblank_max_write( PHY_ADDR_ISP, p_fsm->sensor_config.hblank_max );
                break;
            case 2:
                acamera_frontend_frame_check_sensor_3_active_width_write( PHY_ADDR_ISP, p_fsm->sensor_config.width );
                acamera_frontend_frame_check_sensor_3_active_height_write( PHY_ADDR_ISP, p_fsm->sensor_config.height );
                acamera_frontend_frame_check_sensor_3_hblank_max_write( PHY_ADDR_ISP, p_fsm->sensor_config.hblank_max );
                break;
            case 3:
                acamera_frontend_frame_check_sensor_4_active_width_write( PHY_ADDR_ISP, p_fsm->sensor_config.width );
                acamera_frontend_frame_check_sensor_4_active_height_write( PHY_ADDR_ISP, p_fsm->sensor_config.height );
                acamera_frontend_frame_check_sensor_4_hblank_max_write( PHY_ADDR_ISP, p_fsm->sensor_config.hblank_max );
                break;
            }
        }
    }

    // Calculate final width, height and hblank base on the current pipeline configuration
    // Expected hblank, width, height at each module's (raw scaler, rgb scaler, crop) input, default is MCFE output values
    uint32_t output_hblank = acamera_frontend_mcfe_output_hblank_read( PHY_ADDR_ISP );
    uint32_t output_height = p_fsm->sensor_config.height;
    uint32_t output_width = p_fsm->sensor_config.width;

// Process RAW scaler width, height and hblank contribution
// hblank_out = hblank_in * ( src_width * src_height ) * sqrt(3) / ( scaled_width * scaled_height )
// Same aspect ratio is assumed
#if defined( ISP_HAS_RAW_SCALER_FSM )
    if ( p_fsm->output_config.raw_scaler.enabled ) {
        output_hblank = ( output_hblank * ( output_width * output_height ) * 7 ) / ( p_fsm->output_config.raw_scaler.width * p_fsm->output_config.raw_scaler.height * 4 );
        output_height = p_fsm->output_config.raw_scaler.height;
        output_width = p_fsm->output_config.raw_scaler.width;
    }
#endif

// Process RGB scaler width, height and hblank contribution
// hblank_out = hblank_in + ( frame_width + hblank_in ) * floor( 1 / scale_ratio ) + ceil( ( 1 / scale_ratio - 1 ) * hblank_in )
// Same aspect ratio is assumed
#if defined( ISP_HAS_RGB_SCALER_FSM )
    if ( p_fsm->output_config.rgb_scaler.enabled ) {
        if ( output_width != p_fsm->output_config.rgb_scaler.width ) {
            output_hblank = ( output_hblank + ( output_width + output_hblank ) * ( output_width / p_fsm->output_config.rgb_scaler.width ) ) +
                            ( ( ( output_width * output_hblank + p_fsm->output_config.rgb_scaler.width / 2 ) / p_fsm->output_config.rgb_scaler.width ) - output_hblank );

            output_height = p_fsm->output_config.rgb_scaler.height;
            output_width = p_fsm->output_config.rgb_scaler.width;

// Combination of RAW scaler + RGB scaler may give up to 4x hblank increase, correct to tolerate it
#if defined( ISP_HAS_RAW_SCALER_FSM )
            if ( p_fsm->output_config.raw_scaler.enabled ) {
                output_hblank *= 4;
            }
#endif
        }
    }
#endif

    // Process Crop width, height and hblank contribution
    // hblank = ( frame_height_in - frame_height_out ) * ( frame_width_in + hblank_in ) + 8 + ( frame_width_in - frame_width_out )
    if ( p_fsm->output_config.crop.enabled ) {

        if ( ( p_fsm->output_config.crop.width != output_width ) || ( p_fsm->output_config.crop.height != output_height ) ) {
            output_hblank = ( output_height - p_fsm->output_config.crop.height ) * ( output_width + output_hblank ) + 8 +
                            ( output_width - p_fsm->output_config.crop.width );

            output_width = p_fsm->output_config.crop.width;
            output_height = p_fsm->output_config.crop.height;

// Combination of RAW scaler + Crop may give up to 2x hblank increase, correct to tolerate it
#if defined( ISP_HAS_RAW_SCALER_FSM )
            if ( p_fsm->output_config.raw_scaler.enabled ) {
                output_hblank *= 2;
            }
#endif
        }
    }

    // Update calculated hblank to allow 10% tolerance
    output_hblank = ( output_hblank * 11 ) / 10;

// ISP frame checker is located right after the MCFE so it uses sensor width/height and mcfe hblank max
#if ( ISP_RTL_VERSION_R >= 2 )
    acamera_isp_frame_check_isp_active_width_write( p_ictx->settings.isp_base, p_fsm->sensor_config.width );
    acamera_isp_frame_check_isp_active_height_write( p_ictx->settings.isp_base, p_fsm->sensor_config.height );
    acamera_isp_frame_check_isp_hblank_max_write( p_ictx->settings.isp_base, p_fsm->mcfe_config.hblank_max );
#endif

    // Write ISP_CROP frame checker width, height and hblank max registers
    acamera_isp_frame_check_isp_crop_active_width_write( p_ictx->settings.isp_base, output_width );
    acamera_isp_frame_check_isp_crop_active_height_write( p_ictx->settings.isp_base, output_height );
    acamera_isp_frame_check_isp_crop_hblank_max_write( p_ictx->settings.isp_base, output_hblank );

    // Configure ISP_OUT_X frame checkers
    for ( plane = 0; plane < p_fsm->output_config.num_planes; plane++ ) {

        frame_check_output_plane_config_t *plane_cfg = &( p_fsm->output_config.plane[plane] );
        uint32_t plane_hblank = output_hblank;

        if ( p_fsm->output_config.crop.enabled ) {
            plane_cfg->width = p_fsm->output_config.crop.width / p_fsm->output_config.plane[plane].h_subsampling;
            plane_cfg->height = p_fsm->output_config.crop.height / p_fsm->output_config.plane[plane].v_subsampling;
#if defined( ISP_HAS_RGB_SCALER_FSM )
        } else if ( p_fsm->output_config.rgb_scaler.enabled ) {
            plane_cfg->width = p_fsm->output_config.rgb_scaler.width / p_fsm->output_config.plane[plane].h_subsampling;
            plane_cfg->height = p_fsm->output_config.rgb_scaler.height / p_fsm->output_config.plane[plane].v_subsampling;
#endif
#if defined( ISP_HAS_RAW_SCALER_FSM )
        } else if ( p_fsm->output_config.raw_scaler.enabled ) {
            plane_cfg->width = p_fsm->output_config.raw_scaler.width / p_fsm->output_config.plane[plane].h_subsampling;
            plane_cfg->height = p_fsm->output_config.raw_scaler.height / p_fsm->output_config.plane[plane].v_subsampling;
#endif
        } else {
            plane_cfg->width = p_fsm->sensor_config.width / p_fsm->output_config.plane[plane].h_subsampling;
            plane_cfg->height = p_fsm->sensor_config.height / p_fsm->output_config.plane[plane].v_subsampling;
        }

        // Update hblank for the current plane (streaming output) based on subsampling
        if ( ( p_fsm->output_config.plane[plane].h_subsampling > 1 ) || ( p_fsm->output_config.plane[plane].v_subsampling > 1 ) ) {
            plane_hblank = plane_cfg->width * p_fsm->output_config.plane[plane].h_subsampling + 2 * output_hblank;
        }

        // Write ISP_OUT_X frame checker width and height registers
        switch ( plane_cfg->axi ) {
        case 1:
            acamera_isp_frame_check_isp_out_1_active_width_write( p_ictx->settings.isp_base, plane_cfg->width );
            acamera_isp_frame_check_isp_out_1_active_height_write( p_ictx->settings.isp_base, plane_cfg->height );
            acamera_isp_frame_check_isp_out_1_hblank_max_write( p_ictx->settings.isp_base, plane_hblank );
            break;

        case 2:
            acamera_isp_frame_check_isp_out_2_active_width_write( p_ictx->settings.isp_base, plane_cfg->width );
            acamera_isp_frame_check_isp_out_2_active_height_write( p_ictx->settings.isp_base, plane_cfg->height );
            acamera_isp_frame_check_isp_out_2_hblank_max_write( p_ictx->settings.isp_base, plane_hblank );
            break;

        case 3:
            acamera_isp_frame_check_isp_out_3_active_width_write( p_ictx->settings.isp_base, plane_cfg->width );
            acamera_isp_frame_check_isp_out_3_active_height_write( p_ictx->settings.isp_base, plane_cfg->height );
            acamera_isp_frame_check_isp_out_3_hblank_max_write( p_ictx->settings.isp_base, plane_hblank );
            break;

        default:
            break;
        }
    }
}
