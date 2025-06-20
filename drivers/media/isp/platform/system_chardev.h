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

#ifndef __SYSTEM_CHARDEV_H__
#define __SYSTEM_CHARDEV_H__

#include "system_types.h"

/**
 *   Spawn (misc) character device as the access channel for the Control Software.
 *   Make sense only for the Firmware running in Linux kernel space.
 *   One client at a time supported.
 *
 *   @return  0 - on success
 *           -1 - on error
 */
int system_chardev_init( void );

/**
 *   Nonblocking read from the character device.
 *
 *   @return  number of bytes read on success
 *           -1 is returned after userspace application connected or disconnected
 */
int system_chardev_read( char *data, int size );

/**
 *   Nonblocking write to the character device.
 *
 *   @return  number of bytes written on success
 *           -1 is returned after userspace application connected or disconnected
 */
int system_chardev_write( const char *data, int size );

/**
 *   Deregister character device.
 *
 *   @return  0 - on success
 *           -1 - on error
 */
int system_chardev_destroy( void );

#endif /* __SYSTEM_CHARDEV_H__ */
