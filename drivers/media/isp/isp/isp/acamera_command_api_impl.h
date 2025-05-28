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

#ifndef _ACAMERA_COMMAND_API_IMPL_H_
#define _ACAMERA_COMMAND_API_IMPL_H_
#include "system_types.h"
#include "acamera_isp_ctx.h"

uint8_t application_api_calibration_impl( uint8_t type, uint8_t id, uint8_t direction, void *data, uint32_t data_size, uint32_t *ret_value );
uint8_t application_command_impl( uint8_t command_type, uint8_t command, uint32_t value, uint8_t direction, uint32_t *ret_value );
uint8_t acamera_api_event_ext( uint32_t ctx_id, uint8_t cmd_if_mode, uint32_t event_id );
uint8_t acamera_command_ext( uint32_t ctx_id, uint8_t cmd_if_mode, uint8_t command_type, uint8_t command, uint32_t value, uint8_t direction, uint32_t *ret_value );
#endif // _ACAMERA_COMMAND_API_IMPL_H_
