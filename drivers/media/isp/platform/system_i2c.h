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

#ifndef __SYSTEM_I2C_H__
#define __SYSTEM_I2C_H__

#include "system_types.h"
#include "bitop.h"

// Return codes
#define I2C_TRUNCATED ( -3 )
#define I2C_TIMEDOUT ( -2 )
#define I2C_NOCONNECT ( -1 )
#define I2C_OK ( 0 )
#define I2C_NOACK ( 1 )
#define I2C_ARBITRATION_LOST ( 2 )

typedef enum {
    I2C_FLAG_USE_RSTART = BIT( 0 ),
} i2c_flag_t;

/// Opaque i2c context
struct _system_i2c_controller_ctx_t;
typedef struct _system_i2c_controller_ctx_t system_i2c_controller_ctx_t;


/**
 * @brief      Initialize i2c channel.
 *
 * @param      p_ctx         Pointer to opaque context structure
 * @param[in]  regs          The bus virtual address
 * @param[in]  flags         The flags
 *
 * @details    This function is called by firmware to establish i2c
 *             communication. It allocates and initializes a context structure.
 */
void system_i2c_init( system_i2c_controller_ctx_t **p_ctx, volatile void *regs, int flags );


/**
 * @brief      Close i2c connections.
 *
 * @param      ctx   Pointer to opaque context structure
 *
 * @details    This function is used by firmware to close an i2c connection.
 *             It deinitializes a context structure initialized by @ref
 *             system_i2c_init; exclusive access of the caller is assumed.
 */
void system_i2c_deinit( system_i2c_controller_ctx_t *ctx );


/**
 * @brief      Write data through i2c channel.
 *
 * @param      ctx            Pointer to context structure slave_address
 * @param[in]  slave_address  The slave address
 * @param[in]  data           The data
 * @param[in]  size           The size in bytes
 *
 * @return     Error codes:
 *              - I2C_TRUNCATED - Slave didn't fully complete read transaction.
 *              - I2C_TIMEDOUT  - Bus/data not ready in time, timed out.
 *              - I2C_NOCONNECT - No connection.
 *              - I2C_OK        - Success.
 *              - I2C_NOACK     - No acknowledge from a device.
 *
 * @details    This function is used by firmware to write a specified amount of
 *             data to a specific device.
 */
int system_i2c_write( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, const uint8_t *data, uint32_t size );


/**
 * @brief      Write register address and data through i2c channel.
 *
 * @param      ctx                    Pointer to context structure
 * @param[in]  slave_address          The slave address
 * @param[in]  register_address       The register address
 * @param[in]  register_address_size  The register address size
 * @param[in]  data                   The data
 * @param[in]  size                   The size in bytes
 *
 * @return     Error code:
 *              - I2C_TRUNCATED - Slave didn't fully complete read transaction.
 *              - I2C_TIMEDOUT  - Bus/data not ready in time, timed out.
 *              - I2C_NOCONNECT - No connection.
 *              - I2C_OK        - Success.
 *              - I2C_NOACK     - No acknowledge from a device.
 *
 * @details    This function is used by firmware to write an specified amount
 *             of data to a specific device.
 */
int system_i2c_write_register( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, const uint8_t *register_address, uint32_t register_address_size, const uint8_t *data, uint32_t size );


/**
 * @brief      Read data from i2c channel.
 *
 * @param      ctx            Pointer to context structure
 * @param      slave_address  Slave address
 * @param      data           Pointer to the start of the memory block
 * @param      size           Amount of bytes to write
 *
 * @return     Error codes:
 *              - I2C_TRUNCATED - Slave didn't fully complete read transaction.
 *              - I2C_TIMEDOUT  - Bus/data not ready in time, timed out.
 *              - I2C_NOCONNECT - No connection.
 *              - I2C_OK        - Success.
 *              - I2C_NOACK     - No acknowledge from a device.
 *
 * @details    This function is used by firmware to read an specified amount
 *             of data from a specific device.
 */
int system_i2c_read( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, uint8_t *data, uint32_t size );


/**
 * @brief      Write register address and read data from i2c channel.
 *
 * @param      ctx                    Pointer to context structure
 * @param      slave_address          Slave address
 * @param      register_address       Register address data
 * @param      register_address_size  Register address data size
 * @param      data                   Pointer to the start of the memory block
 * @param      size                   Amount of bytes to write
 *
 * @return     Error codes:
 *              - I2C_TRUNCATED - Slave didn't fully complete read transaction.
 *              - I2C_TIMEDOUT  - Bus/data not ready in time, timed out.
 *              - I2C_NOCONNECT - No connection.
 *              - I2C_OK        - Success.
 *              - I2C_NOACK     - No acknowledge from a device.
 *
 * @details    This function is used by firmware to read an specified amount
 *             of data from a specific device.
 */
int system_i2c_read_register( system_i2c_controller_ctx_t *ctx, uint32_t slave_address, const uint8_t *register_address, uint32_t register_address_size, uint8_t *data, uint32_t size );


#endif /* __SYSTEM_I2C_H__ */
