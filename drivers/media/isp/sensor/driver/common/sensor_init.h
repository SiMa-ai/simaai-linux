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

#ifndef __SENSOR_INIT_H__
#define __SENSOR_INIT_H__

#include "acamera_configuration.h" //For SENSOR_SEQ_BINARY_AND_EXTERNAL
#include "acamera_sbus_api.h"


#if SENSOR_SEQ_BINARY_AND_EXTERNAL == 1
#define sensor_load_sequence sensor_load_binary_sequence
#else
#define sensor_load_sequence sensor_load_array_sequence
#endif

typedef struct sensor_reg_t {
    uint16_t address;
    uint16_t value;
} sensor_reg_t;

void sensor_load_binary_sequence( acamera_sbus_ptr_t p_sbus, char size, const char *sequence, int group );
void sensor_load_array_sequence( acamera_sbus_ptr_t p_sbus, char size, const sensor_reg_t *sequence, int group_start_idx );

#endif /* __SENSOR_INIT_H__ */
