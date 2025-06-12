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

#ifndef __SYSTEM_STDLIB_H__
#define __SYSTEM_STDLIB_H__

#include "system_types.h"


/**
 *   Copy block of memory
 *
 *   Copies the values of num bytes from
 *   the location pointed to by source directly to the memory block
 *   pointed to by destination.
 *
 *   @param   dst - pointer to the destination array where the content is to be copied
 *   @param   src - pointer to source of data to be copied
 *   @param   size - number of bytes to copy
 *
 *   @return  0 - success
 *           -1 - on error
 */

int32_t system_memcpy( void *dst, const void *src, uint32_t size );


/**
 *   Fill block of memory
 *
 *   Sets the first size bytes of the block of memory
 *   pointed by ptr to the specified value
 *
 *   @param   ptr - pointer to the block of memory to fill
 *   @param   value - valute to be set
 *   @param   size - number of bytes to be set to the value
 *
 *   @return  0 - success
 *           -1 - on error
 */

int32_t system_memset( void *ptr, uint8_t value, uint32_t size );


/**
 *   Copy block of physical memory to physical memory
 *
 *   Copies the values of num bytes from
 *   the location pointed to by source directly to the memory block
 *   pointed to by destination.
 *
 *   @param   dst - address to the destination array where the content is to be copied
 *   @param   src - address to source of data to be copied
 *   @param   size - number of bytes to copy
 *
 *   @return  0 - success
 *           -1 - on error
 */

int32_t system_memcpy_phy2phy( uint32_t dst_addr, uint32_t src_addr, uint32_t size );


/**
 *   Copy block of physical memory to physical memory
 *
 *   Copies the values of num bytes from
 *   the location pointed to by source directly to the memory block
 *   pointed to by destination.
 *
 *   @param   dst - address to the destination array where the content is to be copied
 *   @param   src - address to source of data to be copied
 *   @param   size - number of bytes to copy
 *
 *   @return  0 - success
 *           -1 - on error
 */

int32_t system64_memcpy_phy2phy( uint64_t dst_addr, uint64_t src_addr, uint32_t size );

/**
 *   Copy block of physical memory
 *
 *   Copies the values of num bytes from
 *   the location pointed to by source directly to the memory block
 *   pointed to by destination.
 *
 *   @param   dst - pointer to the destination array where the content is to be copied
 *   @param   src - address to source of data to be copied
 *   @param   size - number of bytes to copy
 *
 *   @return  0 - success
 *           -1 - on error
 */

int32_t system_memcpy_vir2phy( uint32_t dst_addr, void *src, uint32_t size );

int32_t system64_memcpy_vir2phy( uint64_t dst_addr, void *src, uint32_t size );


#endif // __SYSTEM_STDLIB_H__
