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

#ifndef __SYSTEM_PLATFORM_H__
#define __SYSTEM_PLATFORM_H__

#include "system_platform_device_numbers.h"

/**
 * @brief      Initialize system layer.
 */
int system_platform_init( void );


/**
 * @brief      Deinitialize system layer.
 */
void system_platform_deinit( void );


/**
 * @brief      Get the virtual address of the MMIO for a given device, suitable for usage within driver code.
 *
 * @param[in]  device           The device type, see @ref system_platform_device_t
 * @param[in]  id               The device ID, if applicable (used for example for sensors)
 *
 * @return     The virtual address, or NULL on error
 */
volatile void *system_platform_get_regbase_vaddr( system_platform_device_t device, unsigned int id );

/**
 * @brief      Get the physical address of the MMIO for a given device.
 *
 * @param[in]  device           The device type, see @ref system_platform_device_t
 * @param[in]  id               The device ID, if applicable (used for example for sensors)
 *
 * @return     The virtual address, or 0 on error
 */
uintptr_t system_platform_get_regbase_paddr( system_platform_device_t device, unsigned int id );

/**
 * @brief      Get the mapped size of the MMIO for a given device.
 *
 * @param[in]  device           The device type, see @ref system_platform_device_t
 * @param[in]  id               The device ID, if applicable (used for example for sensors)
 *
 * @return     The mapped size, or 0 on error
 */
size_t system_platform_get_regsize( system_platform_device_t device, unsigned int id );

#endif /* __SYSTEM_PLATFORM_H__ */
