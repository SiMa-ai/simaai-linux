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

#ifndef __ACAMERA_CMD_QUEUES_CONFIG_H__
#define __ACAMERA_CMD_QUEUES_CONFIG_H__

#include "system_isp_io.h"

// File generated by isp_config.py

// ------------------------------------------------------------------------------ //
// Instance 'cmd_queues' of module 'cmd_queues'
// ------------------------------------------------------------------------------ //

#define ACAMERA_CMD_QUEUES_BASE_ADDR ( 0x7c00 )
#define ACAMERA_CMD_QUEUES_SIZE ( 0x400 )

#define ACAMERA_CMD_QUEUES_ARRAY_DATA_DEFAULT ( 0x0 )
#define ACAMERA_CMD_QUEUES_ARRAY_DATA_DATASIZE ( 32 )
#define ACAMERA_CMD_QUEUES_ARRAY_DATA_OFFSET ( 0x7c00 )

// args: index (0-255), data (32-bit)
static __inline void acamera_cmd_queues_array_data_write( uint32_t base, uint32_t index, uint32_t data )
{
    system_isp_write_32( base + 0x7c00 + ( index << 2 ), data );
}

static __inline uint32_t acamera_cmd_queues_array_data_read( uint32_t base, uint32_t index )
{
    return system_isp_read_32( base + 0x7c00 + ( index << 2 ) );
}

// ------------------------------------------------------------------------------ //
#endif // __ACAMERA_CMD_QUEUES_CONFIG_H__
