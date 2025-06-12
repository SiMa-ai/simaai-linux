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

#ifndef _ACAMERA_FSMGR_EVENT_QUEUE_H_
#define _ACAMERA_FSMGR_EVENT_QUEUE_H_

#include "acamera_loop_buf.h"

// If EVENT_QUEUE_EVENT_FLAG_FILTER is set only events of flagged event type will be pushed into the queue
// Filter is disabled on queue_pop()
#define EVENT_QUEUE_EVENT_FLAG_FILTER ( 1 )
#define EVENT_QUEUE_EVENT_FLAG_FSM_TYPE ( 2 )

typedef acamera_loop_buf_t acamera_event_queue_t;
typedef acamera_loop_buf_ptr_t acamera_event_queue_ptr_t;
typedef acamera_loop_buf_const_ptr_t acamera_event_queue_const_ptr_t;


static __inline void acamera_event_queue_init( acamera_event_queue_ptr_t p_queue, uint16_t *p_data_buf, int data_buf_size )
{
    system_spinlock_init( &p_queue->lock );
    acamera_loop_buffer_init( p_queue, p_data_buf, data_buf_size );
}

static __inline void acamera_event_queue_free( acamera_event_queue_ptr_t p_queue )
{
    system_spinlock_destroy( p_queue->lock );
}

void acamera_event_queue_push( acamera_event_queue_ptr_t p_queue, int event, unsigned int event_flags );
int acamera_event_queue_pop( acamera_event_queue_ptr_t p_queue );
int acamera_event_queue_is_empty( acamera_event_queue_ptr_t p_queue );
#endif /*_ACAMERA_FSMGR_EVENT_QUEUE_H_*/
