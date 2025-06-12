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

#include "system_mutex.h"

#include <linux/mutex.h>
#include <linux/slab.h>

int system_mutex_init( sys_mutex *lock )
{
    struct mutex *mlock = kmalloc( sizeof( struct mutex ), GFP_KERNEL | __GFP_NOFAIL );

    if ( mlock ) {
        *lock = mlock;
        mutex_init( mlock );
    }

    return mlock ? 0 : -1;
}

int system_mutex_lock( sys_mutex lock )
{
    mutex_lock( (struct mutex *)lock );

    return 0;
}

int system_mutex_unlock( sys_mutex lock )
{
    mutex_unlock( (struct mutex *)lock );

    return 0;
}

void system_mutex_destroy( sys_mutex lock )
{
    if ( lock ) {
        kfree( lock );
    }
}
