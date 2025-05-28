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

#include "system_spi.h"

int32_t system_spi_init( volatile void *regs )
{
    return 0;
}

uint32_t system_spi_rw48( uint32_t sel_mask, uint32_t control_inp, uint32_t address, uint8_t addr_size, uint32_t data, uint8_t data_size )
{
    return 0;
}

uint32_t system_spi_rw32( uint32_t sel_mask, uint32_t control, uint32_t data, uint8_t data_size )
{
    return 0;
}
