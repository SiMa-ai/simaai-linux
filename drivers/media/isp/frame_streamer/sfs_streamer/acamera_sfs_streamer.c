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

#include "acamera_sfs_streamer.h"
#include "system_cma.h"
#include "system_mutex.h"
#include "system_stdlib.h"
#include "acamera_logger.h"
#include "util_pool.h"

#define SFS_STREAM_MAX_FRAMES ( 10 )

/**
 * @brief Structure describing SimpleFrameStorage pool of frames
 * 
 */
typedef struct sfs_stream_pool_t {
    apool hndl;                                // Pool data structure
    apool_element data[SFS_STREAM_MAX_FRAMES]; // Pool elements (pointers to frames)
    sys_mutex lock;                            // Pool lock
} sfs_stream_pool_t;

/**
 * @brief Structure describing single SimpleFrameStorage stream i.e. frame stream of a certain type
 * 
 */
typedef struct sfs_stream_t {
    sfs_stream_pool_t frame_pool;           // Frame pool
    aframe_t frames[SFS_STREAM_MAX_FRAMES]; // Actual frame descriptor storage
    uint32_t num_frames;                    // Current number of frames in the stream
    uint8_t initialised;                    // Initialised flag
} sfs_stream_t;

/**
 * @brief   Structure describing stream context consisting of several frame streams
 *          Currently only RAW, OUT and META stream types are allocated
 * 
 */
typedef struct sfs_context_t {
    sfs_stream_t raw;
    sfs_stream_t out;
    sfs_stream_t meta;
} sfs_context_t;

static sfs_context_t sfs_contexts[FIRMWARE_CONTEXT_NUMBER];

/**
 * @brief   Helper function to get stream pointer
 * 
 * @param   context_id context id
 * @param   type frame stream type
 * @return  on success returns pointer to sfs_stream struct, otherwise NULL
 */
static sfs_stream_t *get_stream( const uint32_t context_id, const aframe_type_t type )
{

    if ( context_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Context id (%u) is out of range [0;%d].",
             context_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? FIRMWARE_CONTEXT_NUMBER - 1 : 0 ) );
        return NULL;
    }

    switch ( type ) {
    case AFRAME_TYPE_RAW:
        return &( sfs_contexts[context_id].raw );

    case AFRAME_TYPE_OUT:
        return &( sfs_contexts[context_id].out );

    case AFRAME_TYPE_META:
        return &( sfs_contexts[context_id].meta );

    default:
        LOG( LOG_ERR, "Invalid parameter. Incorrect frame type value (%d).", type );
        return NULL;
    }
}

/**
 * @brief   Helper function to deallocate frame memory
 * 
 * @param   frame pointer to a frame
 */
static void dealloc_frame( aframe_t *frame )
{
    if ( frame == NULL ) {
        return;
    }

    uint32_t i;
    const uint32_t num_planes = ( ( frame->num_planes > AFRAME_MAX_PLANES ) ? AFRAME_MAX_PLANES : frame->num_planes );
    for ( i = 0; i < num_planes; i++ ) {
        if ( frame->planes[i].address.low ) {
            system_cma_free( frame->planes[i].address.low );
            frame->planes[i].address.low = 0;
            frame->planes[i].address.high = 0;
            frame->planes[i].virt_addr = NULL;
        }
    }
}

/**
 * @brief   Helper function to allocate frame memory
 * 
 * @param   stream_cfg pointer to a stream configuration struct
 * @param   frame pointer to a frame to allocate memory for
 * @return  0 on success, otherwise -1 
 */
static int alloc_frame( const frame_stream_cfg_t *stream_cfg, aframe_t *frame )
{
    if ( ( stream_cfg == NULL ) || ( frame == NULL ) ) {
        return -1;
    }

    uint32_t i;
    for ( i = 0; i < stream_cfg->num_planes; i++ ) {
        aplane_t *plane = &frame->planes[i];
        const frame_stream_plane_cfg_t *plane_cfg = &stream_cfg->planes[i];

        const uint32_t data_size = plane_cfg->line_offset * plane_cfg->height;
        if ( data_size == 0 ) {
            LOG( LOG_ERR, "Failed to allocate frame. Incorrect plane parameters (line offset: %u, height: %u)",
                 plane_cfg->line_offset, plane_cfg->height );
            goto frame_alloc_failed;
        }

        // We do not use high 32-bit address part for system_cma allocated frames. Allocated physical address is 32-bit
        plane->address.high = 0;

        if ( system_cma_alloc( &( plane->virt_addr ), &( plane->address.low ), data_size ) ) {
            LOG( LOG_ERR, "Failed to allocate frame (plane: %d, plane data size: %u)", i, data_size );
            plane->virt_addr = NULL;
            plane->address.low = 0;
            goto frame_alloc_failed;
        }

        plane->height = plane_cfg->height;
        plane->width = plane_cfg->width;
        plane->data_width = plane_cfg->data_width;
        plane->line_offset = plane_cfg->line_offset;
        plane->bytes_used = data_size;
        plane->length = data_size;
        plane->hw_cfg = plane_cfg->hw_cfg;

        frame->num_planes = i + 1;
    }

    frame->context_id = stream_cfg->context_id;
    frame->type = stream_cfg->type;

    return 0;

frame_alloc_failed:

    dealloc_frame( frame );

    return -1;
}

int sfs_streamer_create_stream( const frame_stream_cfg_t *stream_cfg )
{
    if ( stream_cfg == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Frame stream configuration pointer is NULL." );
        return -1;
    } else if ( ( stream_cfg->num_frames == 0 ) || ( stream_cfg->num_frames > SFS_STREAM_MAX_FRAMES ) ) {
        LOG( LOG_ERR, "Invalid parameter. Requested number of frames (%u) is out of range [1;%d].",
             stream_cfg->num_frames, SFS_STREAM_MAX_FRAMES );
        return -1;
    } else if ( ( stream_cfg->num_planes == 0 ) || ( stream_cfg->num_planes > AFRAME_MAX_PLANES ) ) {
        LOG( LOG_ERR, "Invalid parameter. Requested number of planes (%u) is out of range [1;%d].",
             stream_cfg->num_planes, AFRAME_MAX_PLANES );
        return -1;
    }

    sfs_stream_t *stream = get_stream( stream_cfg->context_id, stream_cfg->type );

    if ( stream == NULL ) {
        return -1;
    }

    if ( stream->initialised ) {
        LOG( LOG_ERR, "Failed to create a stream. Stream has been already created." );
        return -1;
    }

    if ( system_mutex_init( &( stream->frame_pool.lock ) ) ) {
        LOG( LOG_ERR, "Failed to create a stream. Failed to initialise stream mutex." );
        return -1;
    }

    system_mutex_lock( stream->frame_pool.lock );

    apool_create( &( stream->frame_pool.hndl ), stream->frame_pool.data, stream_cfg->num_frames );

    uint32_t i;
    for ( i = 0; i < stream_cfg->num_frames; i++ ) {

        aframe_t *frame = &( stream->frames[i] );

        system_memset( frame, 0, sizeof( *frame ) );

        if ( alloc_frame( stream_cfg, frame ) ) {
            LOG( LOG_ERR, "Failed to create a stream. Frame allocation failed." );

            // Update stream num frames so deallocate function
            // could clean up partially allocated frames ( with plane allocation failed )
            stream->num_frames = i + 1;

            system_mutex_unlock( stream->frame_pool.lock );
            goto stream_create_failed;
        }

        stream->num_frames = i + 1;
        frame->frame_id = i + 1;
        frame->source = AFRAME_SOURCE_SIMPLE_FRAME_STORAGE;
        frame->memory = AFRAME_MEMORY_AUTO;

        apool_add( &( stream->frame_pool.hndl ), frame );
    }

    stream->initialised = 1;

    system_mutex_unlock( stream->frame_pool.lock );

    return 0;

stream_create_failed:

    sfs_streamer_destroy_stream( stream_cfg->context_id, stream_cfg->type, 0 );
    return -1;
}

int sfs_streamer_destroy_stream( const unsigned int ctx_id, const aframe_type_t type, const unsigned int force )
{
    sfs_stream_t *stream = get_stream( ctx_id, type );

    // Not supported by SimpleFrameStorage streamer
    (void)force;

    if ( stream == NULL ) {
        return -1;
    }

    if ( stream->frame_pool.lock ) {
        system_mutex_lock( stream->frame_pool.lock );
    }

    uint32_t i;
    const uint32_t num_frames = ( ( stream->num_frames > SFS_STREAM_MAX_FRAMES ) ? SFS_STREAM_MAX_FRAMES : stream->num_frames );
    for ( i = 0; i < num_frames; i++ ) {

        aframe_t *frame = &( stream->frames[i] );

        apool_remove( &( stream->frame_pool.hndl ), frame );

        dealloc_frame( frame );
    }

    apool_destroy( &( stream->frame_pool.hndl ) );

    if ( stream->frame_pool.lock ) {
        system_mutex_unlock( stream->frame_pool.lock );
        system_mutex_destroy( stream->frame_pool.lock );
    }

    system_memset( stream, 0, sizeof( *stream ) );

    return 0;
}

int sfs_streamer_put_frame( aframe_t *frame )
{
    if ( frame == NULL ) {
        LOG( LOG_ERR, "Incorrect parameter. Frame pointer is NULL." );
        return -1;
    }

    if ( frame->source != AFRAME_SOURCE_SIMPLE_FRAME_STORAGE ) {
        LOG( LOG_ERR, "Incorrect parameter. Frame (context id: %u, type: %d, source: %d) doesn't belong to SFS.",
             frame->context_id, frame->type, frame->source );
        return -1;
    }

    sfs_stream_t *stream = get_stream( frame->context_id, frame->type );

    if ( stream == NULL ) {
        return -1;
    }

    if ( !stream->initialised ) {
        LOG( LOG_ERR, "Failed to put frame. Stream is not initialised." );
        return -1;
    }

    system_mutex_lock( stream->frame_pool.lock );
    apool_add( &( stream->frame_pool.hndl ), frame );
    system_mutex_unlock( stream->frame_pool.lock );

    return 0;
}

int sfs_streamer_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame )
{
    if ( frame == NULL ) {
        LOG( LOG_ERR, "Incorrect parameter. Frame pointer is NULL." );
        return -1;
    }

    sfs_stream_t *stream = get_stream( ctx_id, type );

    if ( stream == NULL ) {
        return -1;
    }

    if ( !stream->initialised ) {
        LOG( LOG_ERR, "Failed to get frame. Stream is not initialised." );
        return -1;
    }

    system_mutex_lock( stream->frame_pool.lock );
    *frame = apool_remove_used( &( stream->frame_pool.hndl ) );
    system_mutex_unlock( stream->frame_pool.lock );

    if ( *frame == NULL ) {
        LOG( LOG_ERR, "Failed to get frame from SSF (context id: %u, type: %d). No frames available.", ctx_id, type );
        return -1;
    }

    // Update frame state to requested one
    ( *frame )->state = state;

    return 0;
}
