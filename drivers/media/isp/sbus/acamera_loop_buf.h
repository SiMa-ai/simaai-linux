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

#ifndef __ACAMERA_LOOP_BUF_H__
#define __ACAMERA_LOOP_BUF_H__

#include "system_spinlock.h"

// Number of bits reserved for filter bits
#define ACAMERA_LOOP_BUFFER_FILTER_MAX_BITS ( 8 )

typedef struct _acamera_loop_buf_t acamera_loop_buf_t;
typedef struct _acamera_loop_buf_t *acamera_loop_buf_ptr_t;
typedef const struct _acamera_loop_buf_t *acamera_loop_buf_const_ptr_t;

struct _acamera_loop_buf_t {
    volatile int head;
    volatile int tail;
    volatile unsigned int filter;
    volatile unsigned int filter_max;
    volatile unsigned int filter_counters[ACAMERA_LOOP_BUFFER_FILTER_MAX_BITS];
    uint16_t *p_data_buf;
    int data_buf_size;
    sys_spinlock lock;
};

void acamera_loop_buffer_init( acamera_loop_buf_ptr_t p_buf, uint16_t *p_data_buf, int data_buf_size );
uint16_t acamera_loop_buffer_read( acamera_loop_buf_ptr_t p_buf, int pos );
int acamera_loop_buffer_write( acamera_loop_buf_ptr_t p_buf, int pos, uint16_t sample );

#endif /* __ACAMERA_LOOP_BUF_H__ */
