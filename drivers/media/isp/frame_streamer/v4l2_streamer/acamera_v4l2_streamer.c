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

#include "acamera_v4l2_streamer.h"
#include "acamera_logger.h"
#include "isp-v4l2.h"

// Static stream configuration. We don't pass hw specific frame information to V4L2 layer
// So when new frame empty frame arrives it would updated from below local copy
static frame_stream_cfg_t s_stream_cfg[AFRAME_TYPE_MAX];

int v4l2_streamer_create_stream( const frame_stream_cfg_t *stream_cfg )
{
    // Store stream configuration
    s_stream_cfg[stream_cfg->type] = *stream_cfg;

    return 0;
}

int v4l2_streamer_destroy_stream( const unsigned int ctx_id, const aframe_type_t type, const unsigned int force )
{
    (void)ctx_id;
    (void)type;
    (void)force;
    return 0;
}

int v4l2_streamer_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame )
{
    uint32_t i;

    int rc = isp_v4l2_stream_get_frame( ctx_id, type, state, frame );

    aframe_t *lframe = *frame;

    //LOG( LOG_INFO, "get frame: 0x%p, requested: context id: %u, type: %u, state: %u", lframe, ctx_id, type, state );

    // Update frame hw specific description fields
    if ( lframe != NULL ) {
        for ( i = 0; i < lframe->num_planes; i++ ) {
            lframe->planes[i].data_width = s_stream_cfg[type].planes[i].data_width;
            lframe->planes[i].hw_cfg = s_stream_cfg[type].planes[i].hw_cfg;
        }
#if 0
        LOG( LOG_INFO, "context id: %u, type: %u, state: %u, source: %u, num_planes: %u",
             lframe->context_id, lframe->type, lframe->state, lframe->source, lframe->num_planes );

        LOG( LOG_INFO, "Plane information:" );
        for ( i = 0; i < lframe->num_planes; i++ ) {
            LOG( LOG_INFO, "  |--Plane %u. width: %u, height: %u, line offset: %u, address: 0x%llx, data width: %u",
                 i,
                 lframe->planes[i].width,
                 lframe->planes[i].height,
                 lframe->planes[i].line_offset,
                 (uint64_t)lframe->planes[i].address.low | ( ( (uint64_t)lframe->planes[i].address.high ) << 32 ),
                 lframe->planes[i].data_width );
        }
#endif
    }

    return rc;
}

int v4l2_streamer_put_frame( aframe_t *frame )
{
    return isp_v4l2_stream_put_frame( frame );
}
