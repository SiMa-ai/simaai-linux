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

#include "acamera.h"
#include "acamera_command_api.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "module_mcfe.h"
#include "util_addr_calc.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_MCFE

static void module_mcfe_usecase_create_sensor_config( mcfe_fsm_ptr_t p_fsm, module_mcfe_sensor_cfg_t *sensor_config )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t ctx_id = p_ictx->context_id;

    // Get sensor config.
    const sensor_param_t *s_param = ACAMERA_FSM2FSMGR_PTR( p_fsm )->sensor_fsm.ctrl.get_parameters( ACAMERA_FSM2FSMGR_PTR( p_fsm )->sensor_fsm.drv_priv );

    sensor_config->h_start = s_param->h_start;
    sensor_config->v_start = s_param->v_start;
    sensor_config->is_remote = s_param->is_remote;
    sensor_config->num_channel = s_param->num_channel;
    sensor_config->video_port_id = s_param->video_port_id;
    sensor_config->data_width = s_param->data_width;
    sensor_config->width = s_param->active.width;
    sensor_config->height = s_param->active.height;
    sensor_config->rggb_start = s_param->rggb_start;
    sensor_config->cfa_pattern = s_param->cfa_pattern;
    sensor_config->cdma_addr = p_ictx->settings.isp_base;

    LOG(LOG_INFO,"CTX %d [Sensor configuration] - remote=%d, vid=%d, ch=%d, dw=%d, hs=%d, vs=%d, w=%d, h=%d, rgb=%d, cfa=%d, cdmaOff=0x%x.",
         ctx_id,
         sensor_config->is_remote,
         sensor_config->video_port_id,
         sensor_config->num_channel,
         sensor_config->data_width,
         sensor_config->h_start,
         sensor_config->v_start,
         sensor_config->width,
         sensor_config->height,
         sensor_config->rggb_start,
         sensor_config->cfa_pattern,
         sensor_config->cdma_addr );
}

static void module_mcfe_usecase_create_output_config( mcfe_fsm_ptr_t p_fsm, module_mcfe_output_cfg_t *output_config )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t ctx_id = p_ictx->context_id;
    int i;

    output_formatter_cfg_t output_formatter_config;
    WRAP_GENERAL_CMD( p_ictx, CMD_ID_OUTPUT_FORMATTER_DATA, CMD_DIRECTION_GET, NULL, (uint32_t *)&output_formatter_config );

    // Prepare output config.
    output_config->format = output_formatter_config.format;
    output_config->num_planes = output_formatter_config.num_planes;
    output_config->crop.startx = get_context_param( p_ictx, IMAGE_CROP_XOFFSET_ID_PARAM );
    output_config->crop.starty = get_context_param( p_ictx, IMAGE_CROP_YOFFSET_ID_PARAM );
    output_config->crop.width = get_context_param( p_ictx, IMAGE_CROP_WIDTH_ID_PARAM );
    output_config->crop.height = get_context_param( p_ictx, IMAGE_CROP_HEIGHT_ID_PARAM );
    output_config->crop.enabled = get_context_param( p_ictx, IMAGE_CROP_ENABLE_ID_PARAM );
#if defined( ISP_HAS_RGB_SCALER_FSM )
    output_config->rgb_scaler.width = get_context_param( p_ictx, IMAGE_RGB_SCALER_WIDTH_ID_PARAM );
    output_config->rgb_scaler.height = get_context_param( p_ictx, IMAGE_RGB_SCALER_HEIGHT_ID_PARAM );
    output_config->rgb_scaler.enabled = get_context_param( p_ictx, IMAGE_RGB_SCALER_ENABLE_ID_PARAM );
#endif
#if defined( ISP_HAS_RAW_SCALER_FSM )
    output_config->raw_scaler.width = get_context_param( p_ictx, IMAGE_RAW_SCALER_WIDTH_ID_PARAM );
    output_config->raw_scaler.height = get_context_param( p_ictx, IMAGE_RAW_SCALER_HEIGHT_ID_PARAM );
    output_config->raw_scaler.enabled = get_context_param( p_ictx, IMAGE_RAW_SCALER_ENABLE_ID_PARAM );
#endif
    for ( i = 0; i < output_config->num_planes; i++ ) {
        output_config->plane[i].axi = output_formatter_config.plane[i].axi;
        output_config->plane[i].msb_align = output_formatter_config.plane[i].axi_cfg.msb_align;
        output_config->plane[i].data_width = output_formatter_config.plane[i].axi_cfg.data_width;
        output_config->plane[i].h_subsampling = output_formatter_config.plane[i].axi_cfg.h_subsampling;
        output_config->plane[i].v_subsampling = output_formatter_config.plane[i].axi_cfg.v_subsampling;
    }

    LOG( LOG_INFO, "CTX %d [Output configuration] - format=%d, planes=%d, axi1=%d, axi2=%d, axi3=%d.",
         ctx_id,
         output_config->format,
         output_config->num_planes,
         output_config->plane[0].axi,
         output_config->plane[1].axi,
         output_config->plane[2].axi );
}

static void module_mcfe_update_buffer_address_translation( const mcfe_fsm_ptr_t p_fsm, const module_mcfe_usecase_status_info_t *status_info )
{
    acamera_cmd_buffer_address_translation_info addr_translation_info = {0};

    // Check if output frames were swapped successfully and configure address translation if needed
    if ( status_info->out.set_swap_status == MODULE_MCFE_USECASE_FRAME_SET_SWAP_SUCCESS ) {
        // Check if frame memory provided by user so we need to configure address translation
        // All planes expected to be in the same memory region, so only first plane is used for configuration
        addr_translation_info.out.type = ( status_info->out.memory == AFRAME_MEMORY_USER ) ? CMD_BUFFER_ADDRESS_TRANSLATION_TYPE_TRANSLATE : CMD_BUFFER_ADDRESS_TRANSLATION_TYPE_DIRECT;
        addr_translation_info.out.address.high = status_info->out.plane_address[0].high;
        addr_translation_info.out.address.low = status_info->out.plane_address[0].low;

        unsigned long long address = ( (unsigned long long)addr_translation_info.out.address.high ) << 32;

        // If frame memory type is AFRAME_MEMORY_USER then low 32-bit of actual physical
        // address is reduced by ISPAS_MINUS_SYSPHY, compensating it for the log message
        address |= ( addr_translation_info.out.type ) ? ADDR_SYSPHY2ISPAS( addr_translation_info.out.address.low ) : addr_translation_info.out.address.low;
        LOG( LOG_INFO, "OUT buffer address translation, type: %u, address: 0x%llx", addr_translation_info.out.type, address );
    }

    // Check if raw frames were swapped successfully and configure address translation if needed
    if ( status_info->raw.set_swap_status == MODULE_MCFE_USECASE_FRAME_SET_SWAP_SUCCESS ) {
        // Check if frame memory provided by user so we need to configure address translation
        // All planes expected to be in the same memory region, so only first plane is used for configuration
        addr_translation_info.raw.type = ( status_info->raw.memory == AFRAME_MEMORY_USER ) ? CMD_BUFFER_ADDRESS_TRANSLATION_TYPE_TRANSLATE : CMD_BUFFER_ADDRESS_TRANSLATION_TYPE_DIRECT;
        addr_translation_info.raw.address.high = status_info->raw.plane_address[0].high;
        addr_translation_info.raw.address.low = status_info->raw.plane_address[0].low;

        unsigned long long address = ( (unsigned long long)addr_translation_info.raw.address.high ) << 32;

        // If frame memory type is AFRAME_MEMORY_USER then low 32-bit of actual physical
        // address is reduced by ISPAS_MINUS_SYSPHY, compensating it for the log message
        address |= ( addr_translation_info.raw.type ) ? ADDR_SYSPHY2ISPAS( addr_translation_info.raw.address.low ) : addr_translation_info.raw.address.low;
        LOG( LOG_INFO, "RAW buffer address translation, type: %u, address: 0x%llx", addr_translation_info.raw.type, address );
    }

    // Update address translation if needed
    if ( addr_translation_info.raw.type || addr_translation_info.out.type ) {
        WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ), CMD_ID_BUFFER_ADDRESS_TRANSLATION, CMD_DIRECTION_SET, (uint32_t *)&addr_translation_info, NULL );
    }
}

// FSM event handlers

/**
 *   @brief     Initialise FSM.
 *
 *   @param     p_fsm   Pointer to FSM private data
 *
 *   @details   Initialise corresponding MCFE and cdma_offsets.
 */
void mcfe_init( mcfe_fsm_ptr_t p_fsm )
{
}

/**
 *   @brief     Start MCFE FSM.
 *
 *   @param     p_fsm   Pointer to FSM private data
 *
 *   @details   Configure MCFE context with its sensor input and output format.
 */
void mcfe_config( mcfe_fsm_ptr_t p_fsm )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t ctx_id = p_ictx->context_id;

    module_mcfe_sensor_cfg_t sensor_config;
    module_mcfe_output_cfg_t output_config;
    module_mcfe_usecase_type_t usecase_type = MODULE_MCFE_USECASE_NONE;

    module_mcfe_usecase_create_sensor_config( p_fsm, &sensor_config );
    module_mcfe_usecase_create_output_config( p_fsm, &output_config );

    // Check sensor type and V4L2 interface mode to initialise usecase structure
    const uint32_t v4l2_interface_mode = get_context_param( p_ictx, V4L2_INTERFACE_MODE_PARAM );

	LOG (LOG_INFO, "inteface mode is %d", v4l2_interface_mode);
    switch ( v4l2_interface_mode ) {
    case V4L2_INTERFACE_MODE_NONE:
        usecase_type = ( ( sensor_config.is_remote ) ? ( MODULE_MCFE_USECASE_M2M ) : ( MODULE_MCFE_USECASE_TDMF ) );
        break;

    case V4L2_INTERFACE_MODE_CAPTURE:
        if ( sensor_config.is_remote ) {
            usecase_type = MODULE_MCFE_USECASE_M2M;	
            //LOG( LOG_ERR, "Failed to configure use-case (context id: %d). Remote sensor is not compatible with V4L2 inteface.", ctx_id );
        } else {
            usecase_type = MODULE_MCFE_USECASE_TDMF;
        }
        break;

    case V4L2_INTERFACE_MODE_M2M:
        if ( sensor_config.is_remote ) {
            LOG( LOG_ERR, "Failed to configure use-case (context id: %d). Remote sensor is not compatible with V4L2 inteface.", ctx_id );
        } else {
            usecase_type = MODULE_MCFE_USECASE_M2M;
        }
        break;

    default:
        usecase_type = MODULE_MCFE_USECASE_NONE;
        break;
    }

    int rc = module_mcfe_usecase_init( &( p_fsm->usecase_cfg ), usecase_type, ctx_id, ISP_HISTOGRAM_POSITION_IS_BE );
    if ( rc != MCFE_ERR_NONE ) {
        LOG( LOG_ERR, "Failed to configure use-case (context id: %d, type: %d), error code: (%d).",
             ctx_id,
             p_fsm->usecase_cfg.type,
             rc );
        return;
    }

    override_context_param( p_ictx, MCFE_USECASE_PARAM, usecase_type );

    module_mcfe_usecase_status_info_t status_info = {0};
    rc = p_fsm->usecase_cfg.functions.config( &( p_fsm->usecase_cfg ), &sensor_config, &output_config, &status_info );

    // Error handler
    if ( rc != MCFE_ERR_NONE ) {
        p_fsm->usecase_cfg.functions.release_resources( &( p_fsm->usecase_cfg ) );
        LOG( LOG_ERR, "Failed to configure use-case (context id: %d, type: %d), error code: (%d).",
             ctx_id,
             p_fsm->usecase_cfg.type,
             rc );
    } else {
        module_mcfe_update_buffer_address_translation( p_fsm, &status_info );

        rc = module_mcfe_update_inputs();
        if ( rc != MCFE_ERR_NONE ) {
            LOG( LOG_ERR, "Failed to update MCFE inputs in use" );
        }
    }
}

/**
 *   @brief     Start MCFE FSM.
 *
 *   @param     p_fsm   Pointer to FSM private data
 *
 *   @details   Start MCFE slot.
 */
void mcfe_start( mcfe_fsm_ptr_t p_fsm )
{
    if ( p_fsm->usecase_cfg.functions.start ) {
        p_fsm->usecase_cfg.functions.start( &( p_fsm->usecase_cfg ) );
    }
}

/**
 *   @brief     Stop MCFE FSM.
 *
 *   @param     p_fsm   Pointer to FSM private data
 *
 *   @details   This is not per-context approach. It stops and re-inits all slots again.
 */
void mcfe_stop( mcfe_fsm_ptr_t p_fsm )
{
    if ( p_fsm->usecase_cfg.functions.stop ) {
        p_fsm->usecase_cfg.functions.stop( &( p_fsm->usecase_cfg ) );
    }
}

/**
 *   @brief     Raw buffer ready event handler.
 *
 *   @param     p_fsm   Pointer to FSM private data
 */
//#define MCFE_DEBUG_FRAME_COUNTER
void mcfe_raw_buffer_ready( mcfe_fsm_ptr_t p_fsm )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

#if defined( MCFE_DEBUG_FRAME_COUNTER )
    static int raw_buffer_counter = 0;
    LOG( LOG_INFO, "isphw_frame_end_fe event occurred! (cnt = %d, ctx_id = [%u])",
         raw_buffer_counter++, ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id );
#endif

    if ( p_fsm->usecase_cfg.functions.process_event ) {
        module_mcfe_usecase_status_info_t status_info = {0};

        p_fsm->usecase_cfg.frame_sequence = p_ictx->frame_sequence;
        p_fsm->usecase_cfg.functions.process_event( &( p_fsm->usecase_cfg ), MODULE_MCFE_EVENT_RAW_BUFFER_READY, &status_info );

        module_mcfe_update_buffer_address_translation( p_fsm, &status_info );
    }
}

/**
 *   @brief     Out buffer ready event handler.
 *
 *   @param     p_fsm   Pointer to FSM private data
 */
void mcfe_out_buffer_ready( mcfe_fsm_ptr_t p_fsm )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

#if defined( MCFE_DEBUG_FRAME_COUNTER )
    static int out_buffer_counter = 0;
    LOG( LOG_INFO, "isphw_frame_end event occurred! (cnt = %d, ctx_id = [%u])",
         out_buffer_counter++, ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id );
#endif

    if ( p_fsm->usecase_cfg.functions.process_event ) {
        module_mcfe_usecase_status_info_t status_info = {0};

        p_fsm->usecase_cfg.frame_sequence = p_ictx->frame_sequence;
        p_fsm->usecase_cfg.functions.process_event( &( p_fsm->usecase_cfg ), MODULE_MCFE_EVENT_OUT_BUFFER_READY, &status_info );

        module_mcfe_update_buffer_address_translation( p_fsm, &status_info );

        if ( p_fsm->usecase_cfg.type == MODULE_MCFE_USECASE_M2M ) {
            set_context_param( ACAMERA_FSM2ICTX_PTR( p_fsm ), M2M_PROCESS_REQUEST_PARAM, 0 );
        }
    }
}

/**
 *   @brief     Deinitialise MCFE FSM.
 *
 *   @param     p_fsm   Pointer to FSM private data
 *
 *   @details   This is not per-context approach. It deinits all slots.
 */
void mcfe_deinit( mcfe_fsm_ptr_t p_fsm )
{
    module_mcfe_usecase_deinit( &( p_fsm->usecase_cfg ) );
}
