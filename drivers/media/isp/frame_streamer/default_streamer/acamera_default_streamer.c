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

#include "system_cma.h"
#include "system_mutex.h"
#include "system_stdlib.h"
#include "system_types.h"
#include "acamera_aframe.h"
#include "acamera_logger.h"

#include "util_pool.h"
#include "util_queue.h"

#include "acamera_default_streamer.h"

#define DS_STREAM_MAX_FRAMES ( 10 )

// Space reserved for both active and reclaim pools of FRAME_STREAM_MAX_FRAMES size each
#define DS_FRAME_STORAGE_SIZE ( DS_STREAM_MAX_FRAMES * 2 )

/**
 * @brief   Structure to keep stream queue information
 * 
 */
typedef struct ds_stream_queue_t {
    aqueue hndl;                          // Queue data structure
    aframe_t *data[DS_STREAM_MAX_FRAMES]; // Queue elements (pointers to frames)
    sys_mutex lock;                       // Queue lock
} ds_stream_queue_t;

/**
 * @brief   Structure to keep stream pool information
 * 
 */
typedef struct ds_stream_pool_t {
    apool hndl;                               // Pool data structure
    apool_element data[DS_STREAM_MAX_FRAMES]; // Pool elements (pointers to frames)
    sys_mutex lock;                           // Pool lock
} ds_stream_pool_t;

/**
 * @brief   Structure to keep stream information
 * 
 */
typedef struct ds_stream_t {
    ds_stream_queue_t empty_queue;                 // Empty frames queue
    ds_stream_queue_t filled_queue;                // Filled frames queue
    ds_stream_pool_t active_pool;                  // Active frames pool
    ds_stream_pool_t reclaim_pool;                 // Reclaimed frames pool
    aframe_t frame_storage[DS_FRAME_STORAGE_SIZE]; // Actual frame descriptor storage
    uint32_t num_frames;                           // Current number of allocated frames
    uint8_t initialised;                           // Initialised flag
} ds_stream_t;

/**
 * @brief Structure to keep stream context information
 * 
 */
typedef struct ds_context_t {
    ds_stream_t raw;  // Raw stream information
    ds_stream_t out;  // Out stream information
    ds_stream_t meta; // Metadata stream information
} ds_context_t;

static ds_context_t ds_contexts[FIRMWARE_CONTEXT_NUMBER];

/**
 * @brief   Helper function to get the stream object without checking initialised flag
 * 
 * @param   context_id context id
 * @param   type stream/frame type
 * @return  ds_stream_t pointer on success, otherwise NULL
 */
static ds_stream_t *get_stream_unsafe( const unsigned int context_id, const aframe_type_t type )
{
    if ( context_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Context id (%u) is out of range [0;%d].",
             context_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? ( FIRMWARE_CONTEXT_NUMBER - 1 ) : 0 ) );
        return NULL;
    }

    switch ( type ) {
    case AFRAME_TYPE_RAW:
        return &( ds_contexts[context_id].raw );

    case AFRAME_TYPE_OUT:
        return &( ds_contexts[context_id].out );

    case AFRAME_TYPE_META:
        return &( ds_contexts[context_id].meta );

    default:
        LOG( LOG_ERR, "Invalid parameter. Incorrect frame type value (%d).", type );
        return NULL;
    }
}

/**
 * @brief   Helper function to get the stream object. Checks whether stream is initialised
 * 
 * @param   context_id context id
 * @param   type frame/stream type to get
 * @return  ds_stream_t pointer on success, otherwise NULL
 */
static ds_stream_t *get_stream( const unsigned int context_id, const aframe_type_t type )
{
    ds_stream_t *stream = get_stream_unsafe( context_id, type );

    if ( stream == NULL ) {
        LOG( LOG_ERR, "Failed to get stream (context id: %d, type: %d).", context_id, type );
        return NULL;
    } else if ( !stream->initialised ) {
        LOG( LOG_ERR, "Failed to get stream (context id: %d, type: %d). Stream is not initialised.", context_id, type );
        return NULL;
    }

    return stream;
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

/**
 * @brief   Helper function to check whether submitted frame has to be reclaimed, reclaims if yes
 * 
 * @param   stream frame stream pointer passed frame belongs to
 * @param   frame frame pointer to be checked and reclaimed
 * @return  0 on success, otherwise -1
 */
static int reclaim_frame( ds_stream_t *stream, aframe_t *frame )
{
    if ( ( stream == NULL ) || ( frame == NULL ) ) {
        return -1;
    }

    system_mutex_lock( stream->reclaim_pool.lock );
    aframe_t *temp = apool_remove( &( stream->reclaim_pool.hndl ), frame );
    system_mutex_unlock( stream->reclaim_pool.lock );

    // Not found in reclaim pool. Already reclaimed or in the active pool
    if ( temp == NULL ) {
        return -1;
    }

    LOG( LOG_DEBUG, "%s, frame (ptr: %p) removed from reclaim pool and going to be deallocated", __func__, temp );

    // Deallocating returned frame
    dealloc_frame( temp );

    return 0;
}

/**
 * @brief   Helper function releasing all frames from passed queue and pool (if it contains such frames)
 * 
 * @param   pool pool pointer to check and remove frames from
 * @param   queue queue pointer to release frames from
 */
static void release_queue_from_pool( ds_stream_pool_t *pool, ds_stream_queue_t *queue )
{
    if ( ( pool == NULL ) || ( queue == NULL ) ) {
        return;
    }

    system_mutex_lock( queue->lock );
    system_mutex_lock( pool->lock );

    const uint32_t queue_size = aqueue_size( &( queue->hndl ) );

    uint32_t i;
    for ( i = 0; i < queue_size; ++i ) {
        aframe_t *frame = (aframe_t *)aqueue_dequeue( &( queue->hndl ) );
        if ( frame == NULL ) {
            break;
        }

        if ( apool_remove( &( pool->hndl ), frame ) == NULL ) {
            // This should never happen
            LOG( LOG_CRIT, "Frame is not present in the active pool, context id: %u, type: %u, id: %u, state: %u, ptr: %p.",
                 frame->context_id,
                 frame->type,
                 frame->frame_id,
                 frame->state,
                 frame );
            continue;
        }

        dealloc_frame( frame );
    }

    system_mutex_unlock( queue->lock );
    system_mutex_unlock( pool->lock );
}

/**
 * @brief   Helper function to move frames from source pool to destination pool
 * 
 * @param   src source pool pointer
 * @param   dst destination pool pointer
 */
void static move_frames( ds_stream_pool_t *src, ds_stream_pool_t *dst )
{
    if ( ( src == NULL ) || ( dst == NULL ) ) {
        return;
    }

    system_mutex_lock( src->lock );
    system_mutex_lock( dst->lock );

    const uint32_t pool_size = apool_size( &( src->hndl ) );

    uint32_t i;
    LOG( LOG_DEBUG, "%s, source pool size before move: %d", __func__, pool_size );

    for ( i = 0; i < pool_size; ++i ) {
        void *frame = apool_remove_used( &( src->hndl ) );
        LOG( LOG_DEBUG, "%s, frame (ptr: %p) is going to destination pool", __func__, frame );
        apool_add( &( dst->hndl ), frame );
    }

    system_mutex_unlock( src->lock );
    system_mutex_unlock( dst->lock );
}

/**
 * @brief   Helper function releasing frames of passed stream
 *          Frames that are currently not in queues (held by external module) will be moved to reclaim pool
 * 
 * @param   stream stream pointer to be cleaned up
 */
static void release_stream_frames( ds_stream_t *stream )
{
    if ( stream == NULL ) {
        return;
    }

    // Release empty frames queue from active pool
    release_queue_from_pool( &( stream->active_pool ), &( stream->empty_queue ) );

    // Release filled frames queue from active pool:
    release_queue_from_pool( &( stream->active_pool ), &( stream->filled_queue ) );

    // All queues are empty now
    // Moving all frames left in active pool (still may be in use) to reclaim pool
    move_frames( &( stream->active_pool ), &( stream->reclaim_pool ) );
}

/**
 * @brief   Get the empty frame from stream
 * 
 * @param   stream stream pointer to get empty frame from
 * @return  aframe_t* on success, otherwise NULL
 */
static aframe_t *get_empty_frame( ds_stream_t *stream )
{
    if ( stream == NULL ) {
        return NULL;
    }

    system_mutex_lock( stream->empty_queue.lock );
    aframe_t *frame = (aframe_t *)aqueue_dequeue( &( stream->empty_queue.hndl ) );
    system_mutex_unlock( stream->empty_queue.lock );

    if ( frame ) {
        frame->state = AFRAME_STATE_EMPTY;
    }

    return frame;
}

/**
 * @brief   Put empty frame into stream
 * 
 * @param   stream stream pointer to receive frame
 * @param   frame frame pointer
 * @return  0 on success, otherwise -1 
 */
static int put_empty_frame( ds_stream_t *stream, aframe_t *frame )
{
    if ( ( stream == NULL ) || ( frame == NULL ) ) {
        return -1;
    }

    system_mutex_lock( stream->empty_queue.lock );
    const int ret = aqueue_enqueue( &( stream->empty_queue.hndl ), frame );
    system_mutex_unlock( stream->empty_queue.lock );

    if ( ret ) {
        LOG( LOG_ERR, "Failed to enqueue empty frame (context id: %u, type: %d).", frame->context_id, frame->type );
        return -1;
    }

    return 0;
}

/**
 * @brief   Get the filled/full frame from stream
 * 
 * @param   stream stream pointer to take filled frame from
 * @return  aframe_t* on success, otherwise NULL
 */
static aframe_t *get_filled_frame( ds_stream_t *stream )
{
    if ( stream == NULL ) {
        return NULL;
    }

    system_mutex_lock( stream->filled_queue.lock );
    aframe_t *frame = (aframe_t *)aqueue_dequeue( &( stream->filled_queue.hndl ) );
    system_mutex_unlock( stream->filled_queue.lock );

    if ( frame ) {
        frame->state = AFRAME_STATE_FULL;
    }

    return frame;
}

/**
 * @brief   Put filled/full frame into stream
 * 
 * @param   stream stream pointer to receive filled/full frame
 * @param   frame frame pointer
 * @return  0 on success, otherwise -1 
 */
static int put_filled_frame( ds_stream_t *stream, aframe_t *frame )
{
    if ( ( stream == NULL ) || ( frame == NULL ) ) {
        return -1;
    }

    system_mutex_lock( stream->filled_queue.lock );
    const int ret = aqueue_enqueue( &( stream->filled_queue.hndl ), frame );
    system_mutex_unlock( stream->filled_queue.lock );

    if ( ret ) {
        LOG( LOG_ERR, "Failed to enqueue filled frame (context id: %u, type: %d).", frame->context_id, frame->type );
        return -1;
    }

    return 0;
}

int default_streamer_create_stream( const frame_stream_cfg_t *stream_cfg )
{
    if ( stream_cfg == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Frame stream configuration pointer is NULL." );
        return -1;
    } else if ( ( stream_cfg->num_frames == 0 ) || ( stream_cfg->num_frames > DS_STREAM_MAX_FRAMES ) ) {
        LOG( LOG_ERR, "Invalid parameter. Requested number of frames (%u) is out of range [1;%d].",
             stream_cfg->num_frames, DS_STREAM_MAX_FRAMES );
        return -1;
    } else if ( ( stream_cfg->num_planes == 0 ) || ( stream_cfg->num_planes > AFRAME_MAX_PLANES ) ) {
        LOG( LOG_ERR, "Invalid parameter. Requested number of planes (%u) is out of range [1;%d].",
             stream_cfg->num_planes, AFRAME_MAX_PLANES );
        return -1;
    }

    // Get stream poiter without checking initialised flag
    ds_stream_t *stream = get_stream_unsafe( stream_cfg->context_id, stream_cfg->type );
    if ( stream == NULL ) {
        LOG( LOG_ERR, "Failed to get stream (context id: %d, type: %d).", stream_cfg->context_id, stream_cfg->type );
        return -1;
    }

    if ( stream->initialised ) {
        LOG( LOG_ERR, "Failed to create a stream. Stream has been already created." );
        return -1;
    }

    if ( !stream->filled_queue.lock ) {
        if ( system_mutex_init( &( stream->filled_queue.lock ) ) ) {
            LOG( LOG_ERR, "Failed to create stream. Failed to initialise filled queue mutex." );
            goto stream_create_failed;
        }
    }

    if ( !stream->empty_queue.lock ) {
        if ( system_mutex_init( &( stream->empty_queue.lock ) ) ) {
            LOG( LOG_ERR, "Failed to create stream. Failed to initialise empty queue mutex." );
            goto stream_create_failed;
        }
    }

    if ( !stream->active_pool.lock ) {
        if ( system_mutex_init( &( stream->active_pool.lock ) ) ) {
            LOG( LOG_ERR, "Failed to create stream. Failed to initialise active pool mutex." );
            goto stream_create_failed;
        }
    }

    if ( !stream->reclaim_pool.lock ) {
        if ( system_mutex_init( &( stream->reclaim_pool.lock ) ) ) {
            LOG( LOG_ERR, "Failed to create stream. Failed to initialise reclaim pool mutex." );
            goto stream_create_failed;
        }
    }

    system_mutex_lock( stream->active_pool.lock );
    system_mutex_lock( stream->reclaim_pool.lock );
    system_mutex_lock( stream->empty_queue.lock );

    aqueue_create( &( stream->empty_queue.hndl ), (void **)stream->empty_queue.data, stream_cfg->num_frames );
    aqueue_create( &( stream->filled_queue.hndl ), (void **)stream->filled_queue.data, stream_cfg->num_frames );

    apool_create( &( stream->active_pool.hndl ), stream->active_pool.data, stream_cfg->num_frames );

    // Reclaim pool should be reset only when empty
    if ( !apool_size( &( stream->reclaim_pool.hndl ) ) ) {
        apool_create( &( stream->reclaim_pool.hndl ), stream->reclaim_pool.data, stream_cfg->num_frames );
    }

    uint32_t i, j;
    for ( i = 0, j = 0; ( i < DS_FRAME_STORAGE_SIZE ) && ( j < stream_cfg->num_frames ); i++ ) {

        // Get a pointer from a shared (between active and reclaim pools) frame descriptor storage
        aframe_t *frame = &( stream->frame_storage[i] );

        // Check if frame is in the reclaim pool, skip if yes
        if ( !apool_contains( &( stream->reclaim_pool.hndl ), frame ) ) {
            LOG( LOG_DEBUG, "Frame (ptr: %p) is currently in use (reclaim pool). Skipping...", frame );
            continue;
        }

        system_memset( frame, 0, sizeof( *frame ) );

        if ( alloc_frame( stream_cfg, frame ) ) {
            LOG( LOG_ERR, "Failed to create a stream. Frame allocation failed." );

            // Update stream num frames so deallocate function
            // could clean up partially allocated frames ( with plane allocation failed )
            stream->num_frames = j + 1;
            goto stream_create_failed_unlock;
        }

        stream->num_frames = j + 1;
        frame->frame_id = j + 1;
        frame->source = AFRAME_SOURCE_DEFAULT_STREAMER;
        frame->memory = AFRAME_MEMORY_AUTO;

        apool_add( &( stream->active_pool.hndl ), frame );
        aqueue_enqueue( &( stream->empty_queue.hndl ), frame );

        j++;
    }

    // Check if requested number of frames has been allocated
    if ( stream->num_frames != stream_cfg->num_frames ) {
        LOG( LOG_ERR, "Only %d out of %d frames has been allocated, not enough free entries in the frame storage.",
             stream->num_frames, stream_cfg->num_frames );
        goto stream_create_failed_unlock;
    }

    stream->initialised = 1;

    system_mutex_unlock( stream->empty_queue.lock );
    system_mutex_unlock( stream->reclaim_pool.lock );
    system_mutex_unlock( stream->active_pool.lock );

    return 0;

stream_create_failed_unlock:

    system_mutex_unlock( stream->empty_queue.lock );
    system_mutex_unlock( stream->reclaim_pool.lock );
    system_mutex_unlock( stream->active_pool.lock );

stream_create_failed:

    default_streamer_destroy_stream( stream_cfg->context_id, stream_cfg->type, 0 );

    return -1;
}

int default_streamer_destroy_stream( const unsigned int ctx_id, const aframe_type_t type, const unsigned int force )
{
    // Get stream poiter without checking initialised flag
    ds_stream_t *stream = get_stream_unsafe( ctx_id, type );

    if ( stream == NULL ) {
        LOG( LOG_ERR, "Failed to get stream (context id: %d, type: %d).", ctx_id, type );
        return -1;
    }

    // Releasing queued frames, releasing active pool, moving the rest to reclaim pool
    release_stream_frames( stream );

    // If force flag is set we are going to release all resources forcibly
    if ( force ) {
        system_mutex_lock( stream->reclaim_pool.lock );
        uint32_t i, reclaim_size = apool_size( &( stream->reclaim_pool.hndl ) );
        for ( i = 0; i < reclaim_size; ++i ) {
            aframe_t *frame = (aframe_t *)apool_remove_used( &( stream->reclaim_pool.hndl ) );
            if ( frame != NULL ) {
                dealloc_frame( frame );
            }
        }
        system_mutex_unlock( stream->reclaim_pool.lock );

        if ( stream->empty_queue.lock ) {
            system_mutex_destroy( stream->empty_queue.lock );
        }

        if ( stream->filled_queue.lock ) {
            system_mutex_destroy( stream->filled_queue.lock );
        }

        if ( stream->active_pool.lock ) {
            system_mutex_destroy( stream->active_pool.lock );
        }

        if ( stream->reclaim_pool.lock ) {
            system_mutex_destroy( stream->reclaim_pool.lock );
        }
    }

    stream->initialised = 0;

    return 0;
}

int default_streamer_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame )
{
    if ( frame == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Frame pointer is NULL." );
        return -1;
    }

    ds_stream_t *stream = get_stream( ctx_id, type );

    if ( stream == NULL ) {
        return -1;
    }

    switch ( state ) {
    case AFRAME_STATE_EMPTY:
        *frame = get_empty_frame( stream );
        break;

    case AFRAME_STATE_FULL:
        *frame = get_filled_frame( stream );
        break;

    default:
        LOG( LOG_ERR, "Invalid parameter. Frame state (%d) doesn't belong to any valid state.", state );
        *frame = NULL;
        break;
    }

    return ( *frame != NULL ) ? 0 : -1;
}

int default_streamer_put_frame( aframe_t *frame )
{
    if ( frame == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Frame pointer is NULL." );
        return -1;
    }

    if ( frame->source != AFRAME_SOURCE_DEFAULT_STREAMER ) {
        LOG( LOG_ERR, "Incorrect parameter. Frame (context id: %u, type: %d, source: %d) doesn't belong to Default Streamer.",
             frame->context_id, frame->type, frame->source );
        return -1;
    }

    // Check if frame needs to be reclaimed
    // Stream may not have initialised flag in that case
    // and be already deinitialised with all locked frames moved to reclaim pool
    ds_stream_t *stream = get_stream_unsafe( frame->context_id, frame->type );
    if ( stream == NULL ) {
        return -1;
    }

    // Check if frame is to be reclaimed
    if ( !reclaim_frame( stream, frame ) ) {
        LOG( LOG_DEBUG, "%s, frame (ptr: %p) successfully reclaimed and will not be enqueued.", __func__, frame );
        return 0;
    }

    // Normal put frame path
    stream = get_stream( frame->context_id, frame->type );

    if ( stream == NULL ) {
        return -1;
    }

    switch ( frame->state ) {
    case AFRAME_STATE_FULL:

        if ( put_filled_frame( stream, frame ) ) {
            return -1;
        }
        break;

    case AFRAME_STATE_EMPTY:

        if ( put_empty_frame( stream, frame ) ) {
            return -1;
        }
        break;

    default:
        LOG( LOG_ERR, "Invalid parameter. Frame state (%d) doesn't belong to any valid type.", frame->state );
        return -1;
    }

    return 0;
}