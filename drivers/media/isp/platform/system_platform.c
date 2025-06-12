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

#include "system_platform.h"

uintptr_t system_platform_get_regbase_paddr( system_platform_device_t device, unsigned int id )
{
    return 0;
}

size_t system_platform_get_regsize( system_platform_device_t device, unsigned int id )
{
    return 0;
}

volatile void *system_platform_get_regbase_vaddr( system_platform_device_t device, unsigned int id )
{
    return NULL;
}

int system_platform_init( void )
{
    return 0;
}

void system_platform_deinit( void )
{
}
