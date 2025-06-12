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

#include "acamera_loop_buf.h"
#include "bitop.h"

void acamera_loop_buffer_init( acamera_loop_buf_ptr_t p_buf, uint16_t *p_data_buf, int data_buf_size )
{
    p_buf->head = p_buf->tail = 0;
    p_buf->filter = 0;
    p_buf->filter_max = BITFIELD( ACAMERA_LOOP_BUFFER_FILTER_MAX_BITS );
    p_buf->p_data_buf = p_data_buf;
    p_buf->data_buf_size = data_buf_size;

    int i;
    for ( i = 0; i < ACAMERA_LOOP_BUFFER_FILTER_MAX_BITS; i++ ) {
        p_buf->filter_counters[i] = 0;
    }
}

uint16_t acamera_loop_buffer_read( acamera_loop_buf_ptr_t p_buf, int pos )
{
    pos += p_buf->tail;
    if ( pos >= p_buf->data_buf_size ) {
        pos -= p_buf->data_buf_size;
    }

    return p_buf->p_data_buf[pos];
}

int acamera_loop_buffer_write( acamera_loop_buf_ptr_t p_buf, int pos, uint16_t sample )
{
    pos += p_buf->head;
    if ( pos >= p_buf->data_buf_size ) {
        pos -= p_buf->data_buf_size;
    }

    p_buf->p_data_buf[pos++] = sample;

    if ( pos >= p_buf->data_buf_size ) {
        pos -= p_buf->data_buf_size;
    }

    return pos;
}
