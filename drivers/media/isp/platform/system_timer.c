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

#include "system_timer.h"

#include <linux/delay.h>
#include <linux/jiffies.h>


//================================================================================
// timer functions (for FPS calculation)
uint32_t system_timer_timestamp( void )
{
    uint32_t result = jiffies;
    return result;
}


void system_timer_init( void )
{
}


uint32_t system_timer_frequency( void )
{
    return HZ;
}


int32_t system_timer_usleep( uint32_t usec )
{
    usleep_range( usec, usec + 100 );
    return 0;
}

//================================================================================
