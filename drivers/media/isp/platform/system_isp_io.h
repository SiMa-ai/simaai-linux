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

#ifndef __SYSTEM_ISP_IO_H__
#define __SYSTEM_ISP_IO_H__

#include "system_types.h"

/**
 * @brief      Init register access
 *
 * @return     0 if successful, error code otherwise
 *
 */
int32_t system_isp_init( void );


/**
 * @brief      De-init register access
 *
 */
void system_isp_deinit( void );

uint8_t system_isp_mem_read( void *dst, uint32_t src, uint32_t elem, uint8_t elemSz );

/**
 *   Read 32 bit word from isp memory
 *
 *   This function returns a 32 bits word from ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to read 32 bits word.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_ADDR
 *
 *   @return 32 bits memory value
 */
uint32_t system_isp_read_32( uint32_t addr );


/**
 *   Read 16 bit word from isp memory
 *
 *   This function returns a 16 bits word from ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to read 16 bits word.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_ADDR
 *
 *   @return 16 bits memory value
 */
uint16_t system_isp_read_16( uint32_t addr );


/**
 *   Read 8 bit word from isp memory
 *
 *   This function returns a 8 bits word from ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to read 8 bits word.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_ADDR
 *
 *   @return 8 bits memory value
 */
uint8_t system_isp_read_8( uint32_t addr );


/**
 *   Write 32 bits word to isp memory
 *
 *   This function writes a 32 bits word to ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to write data.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_ADDR.
 *   @param data - data to be written
 */
void system_isp_write_32( uint32_t addr, uint32_t data );


/**
 *   Write 16 bits word to isp memory
 *
 *   This function writes a 16 bits word to ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to write data.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_ADDR.
 *   @param data - data to be written
 */
void system_isp_write_16( uint32_t addr, uint16_t data );


/**
 *   Write 8 bits word to isp memory
 *
 *   This function writes a 8 bits word to ISP memory with a given offset.
 *
 *   @param addr - the offset in ISP memory to write data.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_ADDR.
 *   @param data - data to be written
 */
void system_isp_write_8( uint32_t addr, uint8_t data );


#endif /* __SYSTEM_ISP_IO_H__ */
