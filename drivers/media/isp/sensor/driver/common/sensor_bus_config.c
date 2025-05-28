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

#include "sensor_bus_config.h"
#include "acamera_configuration.h"
#include "acamera_logger.h"


uint32_t bus_addr[] = {
    0x6000A000, 0x6000B000};


uint32_t get_sensor_bus_address( uint8_t sensor_pos )
{
    if ( sensor_pos < FIRMWARE_CONTEXT_NUMBER ) {
        return bus_addr[sensor_pos];
    } else {
        LOG( LOG_ERR, "Attempt to initialize more sensor instances than was configured." );
        return -1;
    }
}
