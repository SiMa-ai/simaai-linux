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

#include "acamera_command_api.h"
#include "acamera.h"
#include "acamera_command_api_impl.h"
#include "acamera_isp_ctx.h"
#include "command_id_to_params.h"

#if FW_HAS_CONTROL_CHANNEL
#include "acamera_ctrl_channel.h"
#endif

extern void *get_ctx_ptr_by_id( uint32_t ctx_id );

uint8_t acamera_command( uint32_t ctx_id, uint8_t command_type, uint8_t command, uint32_t value, uint8_t direction, uint32_t *ret_value )
{
    uint8_t ret = NOT_EXISTS;
    const acamera_isp_ctx_ptr_t p_ictx = get_ctx_ptr_by_id( ctx_id );
    const uint32_t cmd_if_mode = get_context_param( p_ictx, CMD_INTERFACE_MODE_PARAM );

    switch ( command_type ) {

    case TSELFTEST:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;

    case TGENERAL:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;

    case TSENSOR:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;

    case TSYSTEM:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;

    case TIMAGE:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;

    case TREGISTERS:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;

    case TSTATUS:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;

    case TISP_MODULES:
        ret = acamera_handle_param( ctx_id, command_id_to_param_id( command ), direction, value, ret_value );
        break;
    } // switch ( command_type )

#if FW_HAS_CONTROL_CHANNEL
    ctrl_channel_handle_api_command( ctx_id, cmd_if_mode, command_type, command, value, direction );
#endif

    if ( ret != SUCCESS ) {
        LOG( LOG_WARNING, "API COMMAND FAILED: ctx: %d, if_mode: %d, type: %d, cmd: %d, value: %lu, direction: %d, ret_value: %lu, result: %d",
             ctx_id,
             cmd_if_mode,
             command_type,
             command,
             (unsigned long)value,
             direction,
             ( ( ret_value ) ? (unsigned long)*ret_value : 0UL ),
             ret );
    } else {
        LOG( LOG_DEBUG, "API: ctx %d, if_mode: %d, type: %d, cmd: %d, value: %lu, direction: %d, ret_value: %lu, result: %d, module: %s",
             ctx_id,
             cmd_if_mode,
             command_type,
             command,
             (unsigned long)value,
             direction,
             ( ( ret_value ) ? (unsigned long)*ret_value : 0UL ),
             ret,
             ( ( KERNEL_MODULE ) ? "KERNEL" : "USER" ) );
    }

    return ret;
}
