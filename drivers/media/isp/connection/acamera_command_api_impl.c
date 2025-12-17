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

#include "acamera_command_api_impl.h"
#include "system_types.h"
#include "acamera.h"
#include "acamera_calib_mgr.h"
#include "acamera_command_api.h"
#include "acamera_fsmgr_general_router.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"

#if FW_HAS_CONTROL_CHANNEL
#include "acamera_ctrl_channel.h"
#endif

extern void *get_ctx_ptr_by_id( uint32_t ctx_id );

// ------------------------------------------------------------------------------ //
//		TSYSTEM
// ------------------------------------------------------------------------------ //
#ifdef SYSTEM_LOGGER_LEVEL
uint8_t system_logger_level( void *instance, uint32_t value, uint8_t direction, uint32_t *ret_value )
{

    /** @bug       calling LOG a lot from here will cause kernel panic. Unsure
     *             why. */

    switch ( direction ) {
    case COMMAND_SET:
        *ret_value = value;
        alog_set_level( ( alog_level_t )( value ) );
        return SUCCESS;
    case COMMAND_GET:
        *ret_value = (uint32_t)alog_get_level();
        return SUCCESS;
    default:
        return NOT_IMPLEMENTED;
    }
}
#endif

#ifdef SYSTEM_LOGGER_MASK
uint8_t system_logger_mask( void *instance, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    switch ( direction ) {
    case COMMAND_SET:
        *ret_value = value;
        alog_set_mask( (alog_mask_t)value );
        return SUCCESS;
    case COMMAND_GET:
        *ret_value = (uint32_t)alog_get_mask();
        return SUCCESS;
    default:
        return NOT_IMPLEMENTED;
    }
}
#endif

#ifdef CONTEXT_STATE
uint8_t system_context_state( void *instance, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    acamera_isp_ctx_ptr_t p_ictx = instance;

    if ( direction == COMMAND_SET ) {

        uint32_t current_state = get_context_param( p_ictx, CONTEXT_STATE_PARAM );

        uint8_t is_transition_allowed = 0;

        // If current and target state are different then
        // check if state transition is allowed. This should be aligned with actual FSM transition scheme
        if ( current_state != value ) {
            switch ( current_state ) {
            case CTX_STATE_INIT:
                if ( value == CTX_STATE_CONFIG ) {
                    acamera_fsmgr_raise_event( &p_ictx->fsmgr, event_id_fsm_config );
                    is_transition_allowed = 1;
                }
                break;

            case CTX_STATE_CONFIG:
                if ( value == CTX_STATE_START ) {
                    acamera_fsmgr_raise_event( &p_ictx->fsmgr, event_id_fsm_start );
                    is_transition_allowed = 1;
                }
                break;

            case CTX_STATE_START:
                if ( value == CTX_STATE_STOP ) {
                    acamera_fsmgr_raise_event( &p_ictx->fsmgr, event_id_fsm_stop );
                    is_transition_allowed = 1;
                }
                break;

            case CTX_STATE_STOP:
                if ( value == CTX_STATE_CONFIG ) {
                    acamera_fsmgr_raise_event( &p_ictx->fsmgr, event_id_fsm_config );
                    is_transition_allowed = 1;
                } else if ( value == CTX_STATE_DEINIT ) {
                    acamera_fsmgr_raise_event( &p_ictx->fsmgr, event_id_fsm_deinit );
                    is_transition_allowed = 1;
                }
                break;

            default:
                break;
            }
        } else {
            is_transition_allowed = 1;
        }

        if ( !is_transition_allowed ) {
            return FAIL;
        }

        // No ret_value update is required here
        // as actual context state will be updated
        // upon handling an event
    }

    return SUCCESS;
}
#endif

#ifdef V4L2_INTERFACE_MODE
uint8_t v4l2_interface_mode( void *instance, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    switch ( direction ) {

    case COMMAND_SET:
#if V4L2_INTERFACE_BUILD
        // V4L2 interface enabled, allow parameter value change
        *ret_value = value;
        return SUCCESS;
#else
        // V4L2 interface is not enabled, keep original parameter value and report error
        return NOT_PERMITTED;
#endif
    case COMMAND_GET:
        // No special actions are needed on parameter value
        return SUCCESS;
    default:
        return NOT_IMPLEMENTED;
    }
}
#endif

#ifdef M2M_PROCESS_REQUEST
uint8_t m2m_process_request( void *instance, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    const acamera_isp_ctx_ptr_t p_ictx = instance;

    switch ( direction ) {

    case COMMAND_SET:
        if ( ( value != 0 ) && ( get_context_param( p_ictx, MCFE_USECASE_PARAM ) == M2M ) ) {
#if defined( ISP_HAS_MCFE_FSM )
            acamera_fsmgr_raise_event( &p_ictx->fsmgr, event_id_isphw_frame_end_fe );
#endif
            *ret_value = value;
        } else {
            *ret_value = 0;
        }

        return SUCCESS;

    case COMMAND_GET:
        // No special actions are needed on parameter value
        return SUCCESS;
    default:
        return NOT_IMPLEMENTED;
    }
}
#endif

// ------------------------------------------------------------------------------ //
//		TSENSOR
// ------------------------------------------------------------------------------ //
#ifdef SENSOR_INFO_PRESET
uint8_t sensor_info_preset( void *instance, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    acamera_isp_ctx_ptr_t p_ictx = instance;

    if ( direction == COMMAND_SET ) {
        acamera_cmd_sensor_preset_info_t p_sensor_preset_info;

        if ( general_handle_cmd( p_ictx, CMD_ID_SENSOR_PRESET_INFO, CMD_DIRECTION_GET, &value, &p_sensor_preset_info ) ) {
            return FAIL;
        }

		LOG (LOG_INFO, "setting sensor info preset W: %d, h:%d, dw:%d",
				p_sensor_preset_info.width, p_sensor_preset_info.height,
				p_sensor_preset_info.data_width);

        override_context_param( p_ictx, SENSOR_INFO_WDR_MODE_PARAM, p_sensor_preset_info.wdr_mode );
        override_context_param( p_ictx, SENSOR_INFO_EXPOSURES_PARAM, p_sensor_preset_info.exposures );
        override_context_param( p_ictx, SENSOR_INFO_WIDTH_PARAM, p_sensor_preset_info.width );
        override_context_param( p_ictx, SENSOR_INFO_HEIGHT_PARAM, p_sensor_preset_info.height );
        override_context_param( p_ictx, SENSOR_INFO_FPS_PARAM, p_sensor_preset_info.fps );
        override_context_param( p_ictx, SENSOR_INFO_CHANNELS_PARAM, p_sensor_preset_info.num_channels );
        override_context_param( p_ictx, SENSOR_INFO_DATA_WIDTH_PARAM, p_sensor_preset_info.data_width );

        *ret_value = value;
    }

    return SUCCESS;
}
#endif

// ------------------------------------------------------------------------------ //
//		CALIBRATION API
// ------------------------------------------------------------------------------ //
#ifdef BUFFER_DATA_TYPE
static uint8_t acamera_calibration_update( acamera_isp_ctx_ptr_t p_ictx, uint32_t id, uint8_t direction, void *data, uint32_t data_size, uint32_t *ret_value )
{
    *ret_value = 0;

    if ( direction == COMMAND_GET ) {
        if ( data_size == calib_mgr_lut_read( p_ictx->calib_mgr_data, data, data_size, id ) ) {
            return SUCCESS;
        } else {
            *ret_value = ERR_BAD_ARGUMENT;
            return FAIL;
        }
    } else if ( direction == COMMAND_SET ) {
        if ( data_size == calib_mgr_lut_write( p_ictx->calib_mgr_data, data, data_size, id ) ) {
            return SUCCESS;
        } else {
            *ret_value = ERR_BAD_ARGUMENT;
            return FAIL;
        }
    } else {
        return NOT_SUPPORTED;
    }
}

static uint32_t get_calibration_description( acamera_isp_ctx_ptr_t p_ictx, uint32_t id )
{
    uint32_t result = 0;

    if ( id < CALIBRATION_TOTAL_SIZE ) {
        // support only 1, 2 and 4 bytes format now.
        // it's enough 2 bits only for this data
        uint32_t width = ( calib_mgr_lut_width( p_ictx->calib_mgr_data, id ) > 0 ) ? ( calib_mgr_lut_width( p_ictx->calib_mgr_data, id ) - 1 ) : 0;
        uint32_t rows = calib_mgr_lut_rows( p_ictx->calib_mgr_data, id );
        uint32_t cols = calib_mgr_lut_cols( p_ictx->calib_mgr_data, id );
        // the description format is 2 bits for width
        // 15 bits for rows number and 15 bits for cols number
        result = ( width << 30 | ( rows << 15 ) | cols );
    } else {
        LOG( LOG_ERR, "Table pointer has invalid index %d. Maximum possible value is %d", (int)id, CALIBRATION_TOTAL_SIZE );
    }
    return result;
}

uint8_t buffer_data_type( void *instance, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    *ret_value = 0;

    acamera_isp_ctx_ptr_t p_ictx = (acamera_isp_ctx_ptr_t)instance;

    switch ( direction ) {
    case COMMAND_GET:
        *ret_value = get_calibration_description( p_ictx, value );
        return SUCCESS;
    default:
        return NOT_IMPLEMENTED;
    }
}
#endif // BUFFER_DATA_TYPE

uint8_t acamera_api_calibration( uint32_t ctx_id, uint8_t type, uint8_t id, uint8_t direction, void *data, uint32_t data_size, uint32_t *ret_value )
{
#ifdef BUFFER_DATA_TYPE
    uint8_t result = SUCCESS;
    acamera_isp_ctx_ptr_t p_ictx = get_ctx_ptr_by_id( ctx_id );
    *ret_value = 0;

    if ( !p_ictx || ( ( direction != COMMAND_SET ) && ( direction != COMMAND_GET ) ) ) {
        *ret_value = ERR_BAD_ARGUMENT;
        return FAIL;
    }

    if ( id >= CALIBRATION_TOTAL_SIZE ) {
        LOG( LOG_ERR, "Trying to get an access with an invalid LUT index %d", id );
        return FAIL;
    }

    int16_t internal_lut_idx = id;
    if ( internal_lut_idx == -1 ) {
        LOG( LOG_ERR, "Trying to get an access with an invalid LUT index %d", internal_lut_idx );
        return FAIL;
    }

#if FW_HAS_CONTROL_CHANNEL
    ctrl_channel_handle_api_calibration( ctx_id, type, id, direction, data, data_size );
#endif

    // update an internal look-up table
    result = acamera_calibration_update( p_ictx, internal_lut_idx, direction, data, data_size, ret_value );

    if ( direction == COMMAND_SET ) {
        acamera_fsmgr_raise_event( &p_ictx->fsmgr, event_id_fsm_reload_calibration );
        result = SUCCESS;
    }

    return result;
#else
    return FAIL;
#endif // BUFFER_DATA_TYPE
}

// ------------------------------------------------------------------------------ //
//		EVENTS
// ------------------------------------------------------------------------------ //
uint8_t acamera_api_event_ext( uint32_t ctx_id, uint8_t cmd_if_mode, uint32_t event_id )
{
    uint32_t fsmgr_event_id;

    acamera_isp_ctx_ptr_t p_ictx = get_ctx_ptr_by_id( ctx_id );

    if ( p_ictx == NULL ) {
        return FAIL;
    }

    // Event triggering is only allowed when Command APi is in passive mode
    if ( cmd_if_mode != CMD_IF_MODE_PASSIVE ) {
        return NOT_IMPLEMENTED;
    }

    switch ( event_id ) {
    case API_EVENT_ID_STOP:
        fsmgr_event_id = event_id_fsm_stop;
        break;
    case API_EVENT_ID_CONFIG:
        fsmgr_event_id = event_id_fsm_config;
        break;
    case API_EVENT_ID_START:
        fsmgr_event_id = event_id_fsm_start;
        break;
    default:
        return FAIL;
    }

#if FW_HAS_CONTROL_CHANNEL
    ctrl_channel_handle_api_event( ctx_id, cmd_if_mode, event_id );
#endif

    acamera_fsmgr_raise_event( &p_ictx->fsmgr, fsmgr_event_id );

    return SUCCESS;
}

uint8_t acamera_api_event( uint32_t ctx_id, uint32_t event_id )
{
    acamera_isp_ctx_ptr_t p_ictx = get_ctx_ptr_by_id( ctx_id );

    if ( p_ictx == NULL ) {
        return FAIL;
    }

    return acamera_api_event_ext( ctx_id, get_context_param( p_ictx, CMD_INTERFACE_MODE_PARAM ), event_id );
}

uint8_t acamera_command_ext( uint32_t ctx_id, uint8_t cmd_if_mode, uint8_t command_type, uint8_t command, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
// User module inherits Command Interface mode from kernel module via control channel
#if USER_MODULE
    acamera_isp_ctx_ptr_t p_ictx = get_ctx_ptr_by_id( ctx_id );

    if ( p_ictx == NULL ) {
        return FAIL;
    }

    if ( get_context_param( p_ictx, CMD_INTERFACE_MODE_PARAM ) != cmd_if_mode ) {
        override_context_param( p_ictx, CMD_INTERFACE_MODE_PARAM, cmd_if_mode );
    }
#endif

    return acamera_command( ctx_id, command_type, command, value, direction, ret_value );
}

uint8_t application_command_impl( uint8_t command_type, uint8_t command, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    const uint32_t ctx_id = get_isp_param( ACTIVE_CONTEXT_PARAM );
    const uint32_t cmd_if_mode = get_context_param( get_ctx_ptr_by_id( ctx_id ), CMD_INTERFACE_MODE_PARAM );
#if 0
    // If command interface is in passive mode, ignore all set commands (read-only mode)
    if ( ( cmd_if_mode != CMD_IF_MODE_ACTIVE ) && ( direction == COMMAND_SET ) ) {
        return NOT_PERMITTED;
    }
#endif

    return acamera_command( ctx_id, command_type, command, value, direction, ret_value );
}

uint8_t application_api_calibration_impl( uint8_t type, uint8_t id, uint8_t direction, void *data, uint32_t data_size, uint32_t *ret_value )
{
    const uint32_t ctx_id = get_isp_param( ACTIVE_CONTEXT_PARAM );

    return acamera_api_calibration( ctx_id, type, id, direction, data, data_size, ret_value );
}
