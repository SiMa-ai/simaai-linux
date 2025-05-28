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

#if !defined( __ACAMERA_CONNECTION_H__ )
#define __ACAMERA_CONNECTION_H__

#include "acamera_control_config.h"

#define BUS_ERROR_RESET -1
#define BUS_ERROR_FATAL -2

void acamera_connection_init( void );
void acamera_connection_process( void );
void acamera_connection_destroy( void );
uint32_t acamera_read( uint8_t cmd, uint8_t cmd_type, uint32_t *data );
uint32_t acamera_write( uint8_t cmd, uint8_t cmd_type, uint32_t data );

typedef int32_t ( *slot_done_callback )( uint32_t slot_id, void *user_data );

#endif /* __ACAMERA_CONNECTION_H__ */
