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

#include "acamera_frame_consumer.h"

#include "acamera_frame_stream_api.h"
#include "acamera_logger.h"

#define TEST_DOUBLE_BUFFER 0

#if TEST_DOUBLE_BUFFER
#include "system_timer.h"
#endif

static void frame_consumer_do_frame( aframe_t *frame )
{
#if TEST_DOUBLE_BUFFER
    {
        const size_t freq = 50; //Delay every 50 frames
        static size_t running_counter = 0;
        if ( ( ++running_counter % freq ) == 0 ) {
            system_timer_usleep( 20 * 1000 );
            LOG( LOG_NOTICE, "Frame consumer double buffering test, injecting 20ms delay." );
        }
    }
#endif

    if ( frame->state == AFRAME_STATE_EMPTY ) {
        frame->state = AFRAME_STATE_FULL;
    } else if ( frame->state == AFRAME_STATE_FULL ) {
        frame->state = AFRAME_STATE_EMPTY;
    }
}

static void frame_consumer_notify_callback( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, void *private )
{
    (void)private;
    aframe_t *frame = NULL;
    int rc = frame_stream_get_frame( ctx_id, type, state, &frame );
    if ( rc != 0 ) {
        LOG( LOG_CRIT, "frame_stream_isp_get_frame failed rc %d", rc );
        return;
    }

    frame_consumer_do_frame( frame );

    // We're done with the frame, put it back
    rc = frame_stream_put_frame( frame );
    if ( rc != 0 ) {
        LOG( LOG_CRIT, "frame_stream_isp_put_frame failed rc %d", rc );
        return;
    }
}

int frame_consumer_initialize( void )
{
    // Enable all frame streams on all context by default
    int i;
    for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        // Raw stream is disabled here so TDMF use-case doesn't deliver raw frames by default
        frame_stream_set_param( i, AFRAME_TYPE_RAW, FRAME_STREAM_PARAM_IS_ENABLED, 0 );
        frame_stream_set_param( i, AFRAME_TYPE_OUT, FRAME_STREAM_PARAM_IS_ENABLED, 1 );
        frame_stream_set_param( i, AFRAME_TYPE_META, FRAME_STREAM_PARAM_IS_ENABLED, 1 );
    }

    int rc = frame_stream_register_notify_callback(
        frame_consumer_notify_callback,
        FRAME_STREAM_EVENT_OUT_FULL | FRAME_STREAM_EVENT_RAW_FULL | FRAME_STREAM_EVENT_META_FULL,
        -1, -1, NULL );
    if ( rc < 0 ) {
        LOG( LOG_CRIT, "frame_stream_register_notify_callback failed rc %d", rc );
        return rc;
    }
    return 0;
}
