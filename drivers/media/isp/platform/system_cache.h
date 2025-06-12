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

#ifndef __SYSTEM_CACHE_H__
#define __SYSTEM_CACHE_H__

/**
 *   Enable instruction cache
 *
 *   @return  None
 */
void system_inst_cache_enable( void );

/**
 *   Disable instruction cache
 *
 *   @return  None
 */
void system_inst_cache_disable( void );

/**
 *   Enable data cache
 *
 *   @return  None
 */
void system_data_cache_enable( void );

/**
 *   Enable data cache
 *
 *   @return  None
 */
void system_data_cache_disable( void );

/**
 *   Invalidate data cache
 *
 *   @return  None
 */
void system_data_cache_inv( void );

/**
 *   Flush data cache
 *
 *   @return  None
 */
void system_data_cache_flush( void );

/**
 *   Invalidate data cache range
 *
 *   @param
 *        phys_addr - Physical address
 *        size - Size in bytes
 *
 *   @return  None
 */
void system_data_cache_inv_range( unsigned long phys_addr, unsigned long size );

/**
 *   Flush data cache range
 *
 *   @param
 *        phys_addr - Physical address
 *        size - Size in bytes
 *
 *   @return  None
 */
void system_data_cache_flush_range( unsigned long phys_addr, unsigned long size );

#endif /* __SYSTEM_CACHE_H__ */
