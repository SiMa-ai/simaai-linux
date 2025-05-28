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

#include "system_control.h"
#include "system_interrupts.h"
#include "system_isp_io.h" //system_isp_init, system_isp_deinit
#include "system_platform.h"
#include "acamera_interrupts.h" // For acamera_interrupt_*
#include "system_chardev.h"

void bsp_init( void )
{
    system_platform_init();
    system_isp_init();

    /* Register access is now ready so we can setup interrupt registers. */
    acamera_interrupt_disable();
    system_interrupts_set_handler( acamera_interrupt_handler, NULL );
    acamera_interrupt_init();

    system_interrupts_init();

    /* Only enable interrupts once the system handler is ready. */
    acamera_interrupt_enable();
	system_chardev_init();
}

void bsp_destroy( void )
{
	system_chardev_destroy();
    system_interrupts_disable();
    system_interrupts_deinit();
    acamera_interrupt_deinit();

    system_isp_deinit();
    system_platform_deinit();
}
