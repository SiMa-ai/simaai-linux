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

#include "util_queue.h"
#include "system_stdlib.h"
#include "system_types.h"

void aqueue_create( aqueue *hndl, void **holders, uint32_t max_size )
{
    system_memset( hndl, 0, sizeof( *hndl ) );

    hndl->max_size = max_size;
    hndl->array = holders;
}

void aqueue_destroy( aqueue *hndl )
{
    system_memset( hndl, 0, sizeof( *hndl ) );
}

int aqueue_enqueue( aqueue *hndl, void *element )
{
    if ( hndl->filled < hndl->max_size ) {
        hndl->array[hndl->head] = element;
        hndl->head = ( hndl->head + 1 ) % hndl->max_size;
        hndl->filled++;
    } else {
        return -1;
    }

    return 0;
}

void *aqueue_dequeue( aqueue *hndl )
{
    void *element = NULL;

    if ( hndl->filled == 0 ) {
        return NULL;
    }

    element = hndl->array[hndl->tail];
    hndl->array[hndl->tail] = NULL;
    hndl->tail = ( hndl->tail + 1 ) % hndl->max_size;
    hndl->filled--;

    return element;
}

uint32_t aqueue_size( aqueue *hndl )
{
    return hndl->filled;
}
