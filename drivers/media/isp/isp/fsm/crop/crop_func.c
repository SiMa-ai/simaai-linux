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

#include "acamera_isp_ctx.h"
#include "acamera_logger.h"

void crop_init( crop_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    // Initialise context param.
    override_context_param( p_ictx, IMAGE_CROP_ENABLE_ID_PARAM, 0 );
    set_context_param( p_ictx, IMAGE_CROP_XOFFSET_ID_PARAM, 0 );
    set_context_param( p_ictx, IMAGE_CROP_YOFFSET_ID_PARAM, 0 );
    set_context_param( p_ictx, IMAGE_CROP_WIDTH_ID_PARAM, 0 );
    set_context_param( p_ictx, IMAGE_CROP_HEIGHT_ID_PARAM, 0 );
}

static void crop_get_input_dimensions( acamera_isp_ctx_ptr_t p_ictx, uint32_t *max_width_ptr, uint32_t *max_height_ptr )
{
    acamera_cmd_sensor_info sensor_info;
    uint8_t flag_max_set = 0;

    /* HW pipeline order : RAW scaler -- > RGB scaler --> CROP */

    /* Internal function, no need to check null pointers */

#if defined( ISP_HAS_RGB_SCALER_FSM )
    /* RGB scaler get highest prioroty since he set frame H/W that enter to CROP block in HW */
    const uint32_t rgb_scaler_en = get_context_param( p_ictx, IMAGE_RGB_SCALER_ENABLE_ID_PARAM );
    if ( !flag_max_set && rgb_scaler_en ) {
        *max_width_ptr = get_context_param( p_ictx, IMAGE_RGB_SCALER_WIDTH_ID_PARAM );
        *max_height_ptr = get_context_param( p_ictx, IMAGE_RGB_SCALER_HEIGHT_ID_PARAM );
        flag_max_set = 1;
    }
#endif

#if defined( ISP_HAS_RAW_SCALER_FSM )
    /* RAW scaler set frame H/W that enter to CROP block in HW only if RGB scale not in use */
    const uint32_t raw_scaler_en = get_context_param( p_ictx, IMAGE_RAW_SCALER_ENABLE_ID_PARAM );
    if ( !flag_max_set && raw_scaler_en ) {
        *max_width_ptr = get_context_param( p_ictx, IMAGE_RAW_SCALER_WIDTH_ID_PARAM );
        *max_height_ptr = get_context_param( p_ictx, IMAGE_RAW_SCALER_HEIGHT_ID_PARAM );
        flag_max_set = 1;
    }
#endif

    /* The sensor set frame H/W that enter to CROP block in HW only if RGB scale and RAW scaler are not in use. */
    if ( !flag_max_set ) {
        WRAP_GENERAL_CMD( p_ictx, CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&sensor_info );
        *max_width_ptr = sensor_info.active_width;
        *max_height_ptr = sensor_info.active_height;
    }
}

void crop_config( crop_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    // Check input values
    uint32_t max_width, max_height;
    uint32_t enable = get_context_param( p_ictx, IMAGE_CROP_ENABLE_ID_PARAM );
    const uint32_t xoffset = get_context_param( p_ictx, IMAGE_CROP_XOFFSET_ID_PARAM );
    const uint32_t yoffset = get_context_param( p_ictx, IMAGE_CROP_YOFFSET_ID_PARAM );
    const uint32_t width = get_context_param( p_ictx, IMAGE_CROP_WIDTH_ID_PARAM );
    const uint32_t height = get_context_param( p_ictx, IMAGE_CROP_HEIGHT_ID_PARAM );
    const uint32_t xsize = xoffset + width;
    const uint32_t ysize = yoffset + height;

    crop_get_input_dimensions( p_ictx, &max_width, &max_height );

    if ( enable ) {
        if ( ( xsize == 0 ) ||
             ( ysize == 0 ) ||
             ( xsize > max_width ) ||
             ( ysize > max_height ) ) {
            LOG( LOG_ERR, "Invalid crop configuration, size: (%ux%u), offset:%u,%u, max: (%ux%u)",
                 width,
                 height,
                 xoffset,
                 yoffset,
                 max_width,
                 max_height );
            enable = 0;
            set_context_param( p_ictx, IMAGE_CROP_ENABLE_ID_PARAM, 0 );
        }
    }

    // Configure crop FSM hw module.
    if ( enable ) {
        acamera_isp_fr_crop_start_x_write( p_ictx->settings.isp_base, xoffset );
        acamera_isp_fr_crop_start_y_write( p_ictx->settings.isp_base, yoffset );
        acamera_isp_fr_crop_size_x_write( p_ictx->settings.isp_base, width );
        acamera_isp_fr_crop_size_y_write( p_ictx->settings.isp_base, height );
        acamera_isp_fr_crop_enable_crop_write( p_ictx->settings.isp_base, 1 );
        acamera_isp_pipeline_bypass_crop_write( p_ictx->settings.isp_base, 0 );
    } else {
        acamera_isp_fr_crop_enable_crop_write( p_ictx->settings.isp_base, 0 );
        acamera_isp_pipeline_bypass_crop_write( p_ictx->settings.isp_base, 1 );
    }
}

void crop_deinit( crop_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    acamera_isp_fr_crop_enable_crop_write( p_ictx->settings.isp_base, 0 );
    acamera_isp_pipeline_bypass_crop_write( p_ictx->settings.isp_base, 1 );
}