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

#ifndef __SYSTEM_MUTEX_H__
#define __SYSTEM_MUTEX_H__

#include "system_types.h"

typedef void *sys_mutex;

/**
 * @brief   Initialize system mutex
 *
 * @param   lock - double pointer to system mutex to be initialised
 *
 * @return  0 - on success, error code on failure
 * 
 * @details The function initializes a system dependent mutex
 *
 */
int system_mutex_init( sys_mutex *lock );


/**
 * @brief   Locks the mutex pointed by lock
 *
 * @param   lock - pointer to mutex returned by system_mutex_init
 *
 * @return  0 - on success, error code on failure
 * 
 */
int system_mutex_lock( sys_mutex lock );


/**
 * @brief   Unlocks the mutex pointed by lock
 *
 * @param   lock - pointer to mutex returned by system_mutex_init
 *
 * @return  0 - on success, error code on failure
 * 
 */
int system_mutex_unlock( sys_mutex lock );


/**
 * @brief   Destroys the mutex pointed by lock
 *
 * @param   lock - pointer to mutex returned by system_mutex_init
 * 
 * @detail  The function destroys mutex which was created by system_mutex_init
 *
 */
void system_mutex_destroy( sys_mutex lock );

#endif //__SYSTEM_MUTEX_H__
