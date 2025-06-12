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

#include "system_assert.h"

#include <linux/bug.h>
#include <linux/kernel.h>

/** @file system_assert.c
    @addtogroup   platform Platform
    @{
*/

void _assert_handler( char const *const file, const int line, char const *const expression )
{
    panic( "### ASSERTION FAILED %s:%d: %s\n",
           file, line, expression ); //This will halt the kernel.

    // This function does not return.
    for ( ;; )
        ;
}


/** @} */
