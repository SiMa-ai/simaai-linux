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

#include "system_i2c.h"

typedef struct _system_i2c_controller_ctx_t {

} system_i2c_controller_ctx_t;

int system_i2c_write( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, const uint8_t *data, uint32_t size )
{
    return I2C_OK;
}

int system_i2c_write_register( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, const uint8_t *register_address, uint32_t register_address_size, const uint8_t *data, uint32_t size )
{
    return I2C_OK;
}

int system_i2c_read( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, uint8_t *data, uint32_t size )
{
    return I2C_OK;
}

int system_i2c_read_register( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, const uint8_t *register_address, uint32_t register_address_size, uint8_t *data, uint32_t size )
{
    return I2C_OK;
}

void system_i2c_init( system_i2c_controller_ctx_t **p_ctx, volatile void *regs, int flags )
{
}

void system_i2c_deinit( system_i2c_controller_ctx_t *ctx )
{
}
