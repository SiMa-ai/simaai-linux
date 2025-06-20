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

#include "acamera_sbus_i2c.h"
#include "acamera_logger.h"

#include "system_i2c.h"
#include "system_interrupts.h"
#include "system_types.h"

static uint32_t fill_address( uint8_t *buf, uint32_t mask, uint32_t addr )
{
    uint32_t i, size;

    if ( mask & SBUS_MASK_ADDR_SKIP ) return 0;
    if ( mask & SBUS_MASK_ADDR_16BITS )
        size = 2;
    else if ( mask & SBUS_MASK_ADDR_32BITS )
        size = 4;
    else
        size = 1;

    for ( i = 0; i < size; i++ ) {
        // buffer is in little-endian format by default
        *buf++ = ( uint8_t )( addr & 0xFF );
        addr >>= 8;
    }
    return size;
}

static void i2c_io_write_sample( acamera_sbus_t *p_bus, uint32_t addr, uint32_t sample, uint8_t sample_size )
{
    // Make sure sample size is <= 4
    uint8_t addrbuf[4], databuf[4];
    uint32_t addr_size = fill_address( addrbuf, p_bus->mask, addr );
    int i;

    for ( i = 0; i < sample_size; i++ ) {
        // buffer is in little-endian format by default
        databuf[i] = ( uint8_t )( sample & 0xFF );
        sample >>= 8;
    }

    system_interrupts_disable();
    i = system_i2c_write_register( (system_i2c_controller_ctx_t *)p_bus->ctx, p_bus->device, addrbuf, addr_size, databuf, sample_size );
    if ( i != I2C_OK ) {
        LOG( LOG_ERR, "I2C not ok" );
        p_bus->last_error_code = i;
    }
    system_interrupts_enable();
}

static uint32_t i2c_io_read_sample( acamera_sbus_t *p_bus, uint32_t addr, uint8_t sample_size )
{
    // Make sure sample size is <= 4
    uint32_t res = 0;
    uint8_t addrbuf[4], databuf[4];
    uint32_t addr_size = fill_address( addrbuf, p_bus->mask, addr );
    int i;

    if ( p_bus->bus == 0 ) {
        LOG( LOG_ERR, "I2C bus address is zero. Please check the configuration of sensor connection " );
    }

    system_interrupts_disable();
    i = system_i2c_read_register( (system_i2c_controller_ctx_t *)p_bus->ctx, p_bus->device, addrbuf, addr_size, databuf, sample_size );
    if ( i != I2C_OK ) {
        LOG( LOG_ERR, "I2C not ok" );
        p_bus->last_error_code = i;
    }
    system_interrupts_enable();

    if ( i == I2C_OK ) {
        for ( i = sample_size; i--; ) {
            // buffer is in little-endian format by default
            res <<= 8;
            res += databuf[i];
        }
    }
    return res;
}

void acamera_sbus_i2c_init( acamera_sbus_t *p_bus )
{
    p_bus->read_sample = i2c_io_read_sample;
    p_bus->write_sample = i2c_io_write_sample;
    system_i2c_init( (system_i2c_controller_ctx_t **)&p_bus->ctx, p_bus->bus, I2C_FLAG_USE_RSTART );
}
void acamera_sbus_i2c_deinit( acamera_sbus_t *p_bus )
{
    system_i2c_deinit( (system_i2c_controller_ctx_t *)p_bus->ctx );
}

void i2c_init_access( void )
{
    LOG( LOG_ERR, "I2C INIT ACCESS IS DISABLED" );
    //system_i2c_init(32);
}
