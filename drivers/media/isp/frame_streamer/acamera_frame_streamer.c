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

#include "acamera_frame_stream_api.h"
#include "acamera_logger.h"

#if V4L2_INTERFACE_BUILD
#include "acamera_sfs_streamer.h"
#include "acamera_v4l2_streamer.h"
#else
#include "acamera_default_streamer.h"
#endif

// Maximum number of possible callback users (modules)
// Two callbacks per context + one for application
#define FRAME_STREAMER_CALLBACKS_MAX ( ( FIRMWARE_CONTEXT_NUMBER * 2 ) + 1 )

/**
 * @brief Structure to store frame streamer API pointers
 * 
 */
typedef struct frame_streamer_api_config_record_t {
    int ( *create )( const frame_stream_cfg_t *stream_cfg );
    int ( *destroy )( const unsigned int ctx_id, const aframe_type_t type, const unsigned int force );
    int ( *get_frame )( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame );
    int ( *put_frame )( aframe_t *frame );
    aframe_source_t frame_source_id;
} frame_streamer_api_config_record_t;

typedef struct frame_streamer_api_config_t {
    int primary_streamer_index;                    // Primary streamer index in the API records array
    int secondary_streamer_index;                  // Secondary streamer index in the API records array
    frame_streamer_api_config_record_t records[2]; // Frame streamer API records
} frame_streamer_api_config_t;

/**
 * @brief   Structure to store callback information
 * 
 */
typedef struct callback_config_record_t {
    frame_stream_notify_callback func; // Callback function pointer
    unsigned int events;               // Events mask callback owner is interested in
    unsigned int contexts;             // Contexts mask callback owner is interested in
    void *private;                     // Callback owner private data pointer
} callback_config_record_t;

/**
 * @brief   Structure to store callback records
 * 
 */
typedef struct stream_callback_config_t {
    uint32_t last_record;                                           // Last record in records array
    callback_config_record_t records[FRAME_STREAMER_CALLBACKS_MAX]; // Callback records
} stream_callback_config_t;

/**
 * @brief   Structure to store stream parameter
 * 
 */
typedef struct frame_stream_param_t {
    int32_t value; // Parameter value
    struct {
        uint8_t initialised; // Initialised flag
        uint8_t reserved[3]; // Reserved / Padding
    } flags;                 // Flags
} frame_stream_param_t;

/**
 * @brief   Structure to store stream configuration
 * 
 */
typedef struct frame_stream_config_t {
    frame_stream_param_t params[FRAME_STREAM_PARAM_TYPE_MAX]; // Stream parameters
} acamera_frame_stream_config_t;

/**
 * @brief   Structure to store stream context
 *          Includes configuration of all stream types of current context
 * 
 */
typedef struct stream_context_config_t {
    acamera_frame_stream_config_t raw;  // Raw stream configuration
    acamera_frame_stream_config_t out;  // Out stream configuration
    acamera_frame_stream_config_t meta; // Metadata stream configuration
} stream_context_config_t;

/**
 * @brief   Structure to store all stream contexts configuration and callbacks
 * 
 */
typedef struct acamera_frame_streamer_config_t {
    stream_callback_config_t callback_config;                  // Callback configuration (global for all contexts)
    frame_streamer_api_config_t api_config;                    // Frame streamer API config
    stream_context_config_t contexts[FIRMWARE_CONTEXT_NUMBER]; // Stream context configuration
} acamera_frame_streamer_config_t;

// Define Frame streamer configuration
// Formatting tool makes below declaration less readable and structured
// clang-format off
static acamera_frame_streamer_config_t frame_streamer_config = {
    // Multiple braces to suppress false warning caused by gcc bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=80454
    .contexts = {{{{{0}}}}},
    .callback_config = { 0 },
    .api_config = {
#if V4L2_INTERFACE_BUILD
        .primary_streamer_index = 0,
        .secondary_streamer_index = 0,
        .records = {
            {
                .create = v4l2_streamer_create_stream,
                .destroy = v4l2_streamer_destroy_stream,
                .put_frame = v4l2_streamer_put_frame,
                .get_frame = v4l2_streamer_get_frame,
                .frame_source_id = AFRAME_SOURCE_V4L2_STREAMER
            },
            {
                .create = sfs_streamer_create_stream,
                .destroy = sfs_streamer_destroy_stream,
                .put_frame = sfs_streamer_put_frame,
                .get_frame = sfs_streamer_get_frame,
                .frame_source_id = AFRAME_SOURCE_SIMPLE_FRAME_STORAGE
            },
        }
#else
        .primary_streamer_index = 0,
        .secondary_streamer_index = 0,
        .records = {
            {
                .create = default_streamer_create_stream,
                .destroy = default_streamer_destroy_stream,
                .put_frame = default_streamer_put_frame,
                .get_frame = default_streamer_get_frame,
                .frame_source_id = AFRAME_SOURCE_DEFAULT_STREAMER
            },
        }
#endif
    }
};
// clang-format on


/**
 * @brief Helper function to notify callback owners if event of interest occurred
 * 
 * @param   ctx_id context id of occurred event
 * @param   type frame type of occurred event
 * @param   state frame state of occurred event
 */
static void callback_notifier( const unsigned int ctx_id, aframe_type_t type, aframe_state_t state )
{
    unsigned int event = 0;

    // Convert frame type and state into event bit
    if ( type == AFRAME_TYPE_RAW ) {
        if ( state == AFRAME_STATE_FULL ) {
            event = FRAME_STREAM_EVENT_RAW_FULL;
        } else if ( state == AFRAME_STATE_EMPTY ) {
            event = FRAME_STREAM_EVENT_RAW_EMPTY;
        }
    } else if ( type == AFRAME_TYPE_OUT ) {
        if ( state == AFRAME_STATE_FULL ) {
            event = FRAME_STREAM_EVENT_OUT_FULL;
        } else if ( state == AFRAME_STATE_EMPTY ) {
            event = FRAME_STREAM_EVENT_OUT_EMPTY;
        }
    } else if ( type == AFRAME_TYPE_META ) {
        if ( state == AFRAME_STATE_FULL ) {
            event = FRAME_STREAM_EVENT_META_FULL;
        } else if ( state == AFRAME_STATE_EMPTY ) {
            event = FRAME_STREAM_EVENT_META_EMPTY;
        }
    }

    const unsigned int context = ( 1U << ctx_id );

    uint32_t i;
    for ( i = 0; i <= frame_streamer_config.callback_config.last_record; i++ ) {
        callback_config_record_t *cr = &( frame_streamer_config.callback_config.records[i] );
        if ( ( cr->func != NULL ) && ( cr->events & event ) && ( cr->contexts & context ) ) {
            cr->func( ctx_id, type, state, cr->private );
        }
    }
}

int frame_stream_register_notify_callback( frame_stream_notify_callback callback, const unsigned int events, const unsigned int contexts, const int owner_id, void *private )
{
    // Check current mode (Register new | Update/Deregister existing callback)
    if ( owner_id < 0 ) {
        // New callback registration mode
        if ( callback == NULL ) {
            LOG( LOG_ERR, "Invalid parameter. Callback function pointer is NULL" );
            return -1;
        } else if ( !events ) {
            LOG( LOG_ERR, "Invalid parameter. Events mask is zero" );
            return -1;
        } else if ( !contexts ) {
            LOG( LOG_ERR, "Invalid parameter. Contexts mask is zero" );
            return -1;
        }

        int i;
        for ( i = 0; i < FRAME_STREAMER_CALLBACKS_MAX; i++ ) {
            if ( frame_streamer_config.callback_config.records[i].func == NULL ) {
                frame_streamer_config.callback_config.records[i].func = callback;
                frame_streamer_config.callback_config.records[i].events = events;
                frame_streamer_config.callback_config.records[i].contexts = contexts;
                frame_streamer_config.callback_config.records[i].private = private;

                if ( frame_streamer_config.callback_config.last_record < i ) {
                    frame_streamer_config.callback_config.last_record = i;
                }

                return i;
            }
        }

        LOG( LOG_ERR, "Failed to register callback. Callback list is full" );
        return -1;

    } else {
        // Update/Deregister existing callback mode
        if ( owner_id >= FRAME_STREAMER_CALLBACKS_MAX ) {
            LOG( LOG_ERR, "Invalid parameter. Owner id (%d) is out of range [0;%d]",
                 owner_id, ( ( FRAME_STREAMER_CALLBACKS_MAX > 0 ) ? FRAME_STREAMER_CALLBACKS_MAX - 1 : 0 ) );
            return -1;
        }

        // Check if it is an update
        if ( ( callback != NULL ) && ( ( events == 0 ) || ( contexts == 0 ) ) ) {
            LOG( LOG_ERR, "Invalid parameter. Events mask or contexts mask is zero" );
            return -1;
        }

        frame_streamer_config.callback_config.records[owner_id].func = callback;
        frame_streamer_config.callback_config.records[owner_id].events = events;
        frame_streamer_config.callback_config.records[owner_id].contexts = contexts;
        frame_streamer_config.callback_config.records[owner_id].private = private;

        // If callback is deregistered update last_record field
        if ( ( callback == NULL ) ) {
            uint32_t i, last_record = 0;
            for ( i = 0; i < FRAME_STREAMER_CALLBACKS_MAX; i++ ) {
                if ( frame_streamer_config.callback_config.records[i].func != NULL ) {
                    last_record = i;
                }
            }

            frame_streamer_config.callback_config.last_record = last_record;
        }
    }

    return owner_id;
}

int frame_stream_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame )
{
    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Context id (%u) is out of range (0-%d).",
             ctx_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? ( FIRMWARE_CONTEXT_NUMBER - 1 ) : 0 ) );
        return -1;
    } else if ( ( type <= AFRAME_TYPE_UNKNOWN ) || ( type >= AFRAME_TYPE_MAX ) ) {
        LOG( LOG_ERR, "Invalid parameter. Type (%d) doesn't belong to any valid type.", type );
        return -1;
    } else if ( ( state != AFRAME_STATE_EMPTY ) && ( state != AFRAME_STATE_FULL ) ) {
        LOG( LOG_ERR, "Invalid parameter. State (%d) doesn't belong to any valid state.", state );
        return -1;
    } else if ( frame == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Frame pointer is NULL." );
        return -1;
    }

    int32_t is_stream_enabled;
    if ( frame_stream_get_param( ctx_id, type, FRAME_STREAM_PARAM_IS_ENABLED, &is_stream_enabled ) ) {
        is_stream_enabled = 1;
        LOG( LOG_ERR, "Failed to get FRAME_STREAM_PARAM_IS_ENABLED parameter. Using default value (%d).", is_stream_enabled );
    }

	LOG (LOG_INFO, "is stream enabled %d", is_stream_enabled);

    frame_streamer_api_config_record_t *primary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.primary_streamer_index] );
    frame_streamer_api_config_record_t *secondary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.secondary_streamer_index] );

    if ( is_stream_enabled ) {
        if ( primary->get_frame ) {
            primary->get_frame( ctx_id, type, state, frame );
        } else {
            LOG( LOG_ERR, "Failed to get frame (context id: %u, type: %d, state: %d). Primary streamer API <get_frame> is not initialised.",
                 ctx_id, type, state );
            return -1;
        }
    } else {
        if ( secondary->get_frame ) {
            secondary->get_frame( ctx_id, type, state, frame );
        } else {
            LOG( LOG_ERR, "Failed to get frame (context id: %u, type: %d, state: %d). Secondary streamer API <get_frame> is not initialised.",
                 ctx_id, type, state );
            return -1;
        }
    }

    return ( *frame != NULL ) ? 0 : -1;
}

int frame_stream_put_frame( aframe_t *frame )
{
    if ( frame == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Frame pointer is NULL." );
        return -1;
    } else if ( frame->context_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Frame context id (%u) is out of range [0-%d].",
             frame->context_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? ( FIRMWARE_CONTEXT_NUMBER - 1 ) : 0 ) );
        return -1;
    } else if ( ( frame->type <= AFRAME_TYPE_UNKNOWN ) || ( frame->type >= AFRAME_TYPE_MAX ) ) {
        LOG( LOG_ERR, "Invalid parameter. Frame (context is: %u) type (%d) doesn't belong to any valid type.",
             frame->context_id, frame->type );
        return -1;
    } else if ( ( frame->state != AFRAME_STATE_EMPTY ) && ( frame->state != AFRAME_STATE_FULL ) ) {
        LOG( LOG_ERR, "Invalid parameter. Frame (context id: %u, type: %d) state (%d) doesn't belong to any valid state.",
             frame->context_id, frame->type, frame->state );
        return -1;
    }

    frame_streamer_api_config_record_t *primary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.primary_streamer_index] );
    frame_streamer_api_config_record_t *secondary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.secondary_streamer_index] );

    int rc;
    if ( frame->source == primary->frame_source_id ) {
        if ( primary->put_frame ) {
            rc = primary->put_frame( frame );
        } else {
            LOG( LOG_ERR, "Failed to put frame (context id: %u, type: %d, state: %d). Primary streamer API <put_frame> is not initialised.",
                 frame->context_id, frame->type, frame->state );
            return -1;
        }
    } else if ( frame->source == secondary->frame_source_id ) {
        if ( secondary->put_frame ) {
            rc = secondary->put_frame( frame );
        } else {
            LOG( LOG_ERR, "Failed to put frame (context id: %u, type: %d, state: %d). Secondary streamer API <put_frame> is not initialised.",
                 frame->context_id, frame->type, frame->state );
            return -1;
        }
    } else {
        LOG( LOG_ERR, "Invalid parameter. Frame (context id: %u, type: %d, state: %d) source (%d) doesn't belong to any valid source.",
             frame->context_id, frame->type, frame->state, frame->source );
        return -1;
    }

    callback_notifier( frame->context_id, frame->type, frame->state );

    return rc;
}

int frame_stream_create( const frame_stream_cfg_t *stream_cfg )
{
    if ( stream_cfg == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Frame stream configuration pointer is NULL." );
        return -1;
    } else if ( !stream_cfg->num_frames ) {
        LOG( LOG_ERR, "Invalid parameter. Requested number of frames is zero." );
        return -1;
    } else if ( stream_cfg->context_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Context id (%u) is out of range [0-%d].",
             stream_cfg->context_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? ( FIRMWARE_CONTEXT_NUMBER - 1 ) : 0 ) );
        return -1;
    } else if ( ( stream_cfg->num_planes == 0 ) || ( stream_cfg->num_planes > AFRAME_MAX_PLANES ) ) {
        LOG( LOG_ERR, "Invalid parameter. Requested number of planes (%u) is out of range [1;%d].",
             stream_cfg->num_planes, AFRAME_MAX_PLANES );
        return -1;
    } else if ( ( stream_cfg->type <= AFRAME_TYPE_UNKNOWN ) || ( stream_cfg->type >= AFRAME_TYPE_MAX ) ) {
        LOG( LOG_ERR, "Invalid parameter. Type (%d) doesn't belong to any valid type.", stream_cfg->type );
        return -1;
    }

    frame_streamer_api_config_record_t *primary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.primary_streamer_index] );
    frame_streamer_api_config_record_t *secondary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.secondary_streamer_index] );

    if ( primary->create ) {
        if ( primary->create( stream_cfg ) ) {
            LOG( LOG_ERR, "Failed to create stream (context id: %u, type: %d, streamer: primary).", stream_cfg->context_id, stream_cfg->type );
            return -1;
        }
    } else {
        LOG( LOG_ERR, "Failed to create stream (context id: %u, type: %d). Primary streamer API <create> is not initialised.",
             stream_cfg->context_id, stream_cfg->type );
        return -1;
    }

    // Check that primary and secondary are not the same streamer to not to call create twice
    if ( primary->create != secondary->create ) {
        if ( secondary->create ) {
            if ( secondary->create( stream_cfg ) ) {
                LOG( LOG_ERR, "Failed to create stream (context id: %u, type: %d, streamer: secondary).", stream_cfg->context_id, stream_cfg->type );
                return -1;
            }
        } else {
            LOG( LOG_ERR, "Failed to create stream (context id: %u, type: %d). Secondary streamer API <create> is not initialised.",
                 stream_cfg->context_id, stream_cfg->type );
            return -1;
        }
    }

    return 0;
}

int frame_stream_destroy( const unsigned int ctx_id, const aframe_type_t type, const unsigned int force )
{
    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Context id (%u) is out of range (0-%d).",
             ctx_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? ( FIRMWARE_CONTEXT_NUMBER - 1 ) : 0 ) );
        return -1;
    } else if ( ( type <= AFRAME_TYPE_UNKNOWN ) || ( type >= AFRAME_TYPE_MAX ) ) {
        LOG( LOG_ERR, "Invalid parameter. Type (%d) doesn't belong to any valid type.", type );
        return -1;
    }

    frame_streamer_api_config_record_t *primary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.primary_streamer_index] );
    frame_streamer_api_config_record_t *secondary = &( frame_streamer_config.api_config.records[frame_streamer_config.api_config.secondary_streamer_index] );

    int rc = 0;
    if ( primary->destroy ) {
        if ( primary->destroy( ctx_id, type, force ) ) {
            LOG( LOG_ERR, "Failed to destroy stream (context id: %u, type: %d, streamer: primary).", ctx_id, type );
            rc = -1;
        }
    } else {
        LOG( LOG_ERR, "Failed to destroy stream (context id: %u, type: %d). Primary streamer API <destroy> is not initialised.",
             ctx_id, type );
        return -1;
    }

    // Check that primary and secondary are not the same streamer to not to call destroy twice
    if ( primary->destroy != secondary->destroy ) {
        if ( secondary->destroy ) {
            if ( secondary->destroy( ctx_id, type, force ) ) {
                LOG( LOG_ERR, "Failed to destroy stream (context id: %u, type: %d, streamer: secondary).", ctx_id, type );
                rc = -1;
            }
        } else {
            LOG( LOG_ERR, "Failed to destroy stream (context id: %u, type: %d). Secondary streamer API <destroy> is not initialised.",
                 ctx_id, type );
            return -1;
        }
    }

    return rc;
}

int frame_stream_set_param( const unsigned int ctx_id, const aframe_type_t type, const frame_stream_param_type_t param_id, const int32_t value )
{
    if ( ( param_id < 0 ) || ( param_id >= FRAME_STREAM_PARAM_TYPE_MAX ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream parameter id (%d) is out of range [0;%d].",
             param_id, ( ( FRAME_STREAM_PARAM_TYPE_MAX > 0 ) ? ( FRAME_STREAM_PARAM_TYPE_MAX - 1 ) : ( 0 ) ) );
        return -1;
    }

    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Context id (%u) is out of range [0;%d]",
             ctx_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? ( FIRMWARE_CONTEXT_NUMBER - 1 ) : ( 0 ) ) );
        return -1;
    }

    frame_stream_param_t *params = NULL;

    switch ( type ) {
    case AFRAME_TYPE_RAW:
        params = frame_streamer_config.contexts[ctx_id].raw.params;
        break;

    case AFRAME_TYPE_OUT:
        params = frame_streamer_config.contexts[ctx_id].out.params;
        break;

    case AFRAME_TYPE_META:
        params = frame_streamer_config.contexts[ctx_id].meta.params;
        break;

    default:
        LOG( LOG_ERR, "Invalid parameter. Type (%d) doesn't belong to any valid type.", type );
        return -1;
    }

    params[param_id].value = value;

    if ( !params[param_id].flags.initialised ) {
        params[param_id].flags.initialised = 1;
    }

    return 0;
}

int frame_stream_get_param( const unsigned int ctx_id, const aframe_type_t type, const frame_stream_param_type_t param_id, int32_t *value )
{
    if ( value == NULL ) {
        LOG( LOG_ERR, "Invalid parameter. Stream parameter value pointer is NULL." );
        return -1;
    }

    if ( ( param_id < 0 ) || ( param_id >= FRAME_STREAM_PARAM_TYPE_MAX ) ) {
        LOG( LOG_ERR, "Invalid parameter. Stream parameter id (%d) is out of range [0;%d].",
             param_id, ( ( FRAME_STREAM_PARAM_TYPE_MAX > 0 ) ? ( FRAME_STREAM_PARAM_TYPE_MAX - 1 ) : ( 0 ) ) );
        return -1;
    }

    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid parameter. Context id (%u) is out of range [0;%d]",
             ctx_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? ( FIRMWARE_CONTEXT_NUMBER - 1 ) : ( 0 ) ) );
        return -1;
    }

    frame_stream_param_t *params = NULL;

    switch ( type ) {
    case AFRAME_TYPE_RAW:
        params = frame_streamer_config.contexts[ctx_id].raw.params;
        break;

    case AFRAME_TYPE_OUT:
        params = frame_streamer_config.contexts[ctx_id].out.params;
        break;

    case AFRAME_TYPE_META:
        params = frame_streamer_config.contexts[ctx_id].meta.params;
        break;

    default:
        LOG( LOG_ERR, "Invalid parameter. Type (%d) doesn't belong to any valid type.", type );
        return -1;
    }

    if ( !params[param_id].flags.initialised ) {
        LOG( LOG_ERR, "Uninitialised. Stream parameter (param_id: %u) has not been set.", param_id );
        return -1;
    }

    *value = params[param_id].value;

    return 0;
}
