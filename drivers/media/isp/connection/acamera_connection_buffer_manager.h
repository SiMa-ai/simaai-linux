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

#if !defined( __ACAMERA_BUFFER_MANAGER_H__ )
#define __ACAMERA_BUFFER_MANAGER_H__
#include "system_types.h"

typedef struct {
    uint32_t tx_base;
    uint32_t rx_base;
    uint32_t data_size;
    int rx_ack;
    int tx_ack;
} acamera_buffer_manager_t;

void acamera_buffer_manager_init( acamera_buffer_manager_t *p_ctrl, uint32_t base, uint32_t size );
int acamera_buffer_manager_read( acamera_buffer_manager_t *p_ctrl, uint8_t *data, int size );
int acamera_buffer_manager_write( acamera_buffer_manager_t *p_ctrl, const uint8_t *data, int size );

#endif /* __ACAMERA_BUFFER_MANAGER_H__ */
