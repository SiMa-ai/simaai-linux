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

#ifndef __ACAMERA_FRAME_STREAM_API_H__
#define __ACAMERA_FRAME_STREAM_API_H__

#include "acamera_aframe.h"

typedef struct frame_stream_plane_cfg_t {
    uint32_t width;         // Plane width
    uint32_t height;        // Plane height
    uint32_t line_offset;   // Plane line offset
    uint32_t data_width;    // Data bit width
    aplane_hw_cfg_t hw_cfg; // Hardware specific configuration
} frame_stream_plane_cfg_t;

typedef struct frame_stream_cfg_t {
    uint32_t context_id;                                // Context id
    aframe_type_t type;                                 // Frame type
    uint32_t num_planes;                                // Number of planes
    uint32_t num_frames;                                // Number of frames
    frame_stream_plane_cfg_t planes[AFRAME_MAX_PLANES]; // Planes information
} frame_stream_cfg_t;

typedef enum frame_stream_param_type_t {
    FRAME_STREAM_PARAM_IS_ENABLED = 0,
    FRAME_STREAM_PARAM_TYPE_MAX
} frame_stream_param_type_t;

typedef enum frame_stream_event_t {
    FRAME_STREAM_EVENT_RAW_EMPTY = ( 1 << 0 ),
    FRAME_STREAM_EVENT_RAW_FULL = ( 1 << 1 ),
    FRAME_STREAM_EVENT_OUT_EMPTY = ( 1 << 2 ),
    FRAME_STREAM_EVENT_OUT_FULL = ( 1 << 3 ),
    FRAME_STREAM_EVENT_META_EMPTY = ( 1 << 4 ),
    FRAME_STREAM_EVENT_META_FULL = ( 1 << 5 )
} frame_stream_event_t;

/**
 * @brief   Type definition for frame stream API notify callback
 *
 * @param   ctx_id context id that has generated notification event
 * @param   type frame stream type that has generated notification event
 * @param   state frame stream frame state that has generated notification event
 * @param   private callback owner private data pointer (if configured)
 *
 * @details Function of this type should be implemented by frame consumer application and
 *          registered via register_notify_callback()
 */
typedef void ( *frame_stream_notify_callback )( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, void *private );

/**
 * @brief   Puts (releases) processed (empty) output/raw frame
 *
 * @param   frame pointer to aframe_t structure to be put
 *
 * @return  0 on success
 *
 * @details This function should be called by a frame user (application/ISP driver)
 *          to release previously acquired output/raw frame.
 */
int frame_stream_put_frame( aframe_t *frame );

/**
 * @brief   Gets empty frame
 *
 * @param   ctx_id context id to query
 * @param   type frame type to get
 * @param   state frame state to get
 * @param   pointer to pointer to the aframe_t structure to be returned
 *
 * @return  0 on success
 *
 * @details This function should be called by a frame user (application/ISP driver)
 *          to get frame of required type/state
 */
int frame_stream_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame );

/**
 * @brief   Registers frame notification callback
 *
 * @param   callback pointer to frame_stream_notify_callback function
 * @param   events event mask created from frame_stream_event_t
 * @param   contexts context bit mask saying events of which contexts should trigger the callback.
 *          To set callback for current context only ( 1 << context_id ), for all contexts -1
 * @param   owner_id specifies owner id which is required to update or deregister existing callback.
 *          To make a new callback registration it must be set to -1
 * @param   private callback owner private data pointer (will be passed back on callback if required)
 *          May be NULL if not needed
 *
 * @return  owner id ( >= 0 ) on success, otherwise -1
 *
 * @details This function should be called by a frame user interested in receiving callbacks
 *          to register frame notification callback on specific frame event. Callback is called
 *          every time when there is a new specified frame event occurs. 
 */
int frame_stream_register_notify_callback( frame_stream_notify_callback callback, const unsigned int events, const unsigned int contexts, const int owner_id, void *private );

/**
 * @brief   Creates frame stream based on configuration
 *
 * @param   stream_cfg stream configuration for the new steam to be created

 * @return  0 on success
 *
 * @details This function should be called by ISP driver to create frame stream before
 *          calling functions *_get|put_frame()
 */
int frame_stream_create( const frame_stream_cfg_t *stream_cfg );

/**
 * @brief   Destroys specified frame stream
 *
 * @param   ctx_id context id to destroy stream of
 * @param   type frame stream type to destroy
 * @param   force will destroy frames without keeping used frames to delayed reclaim if set to non-zero
 *
 * @return  0 on success
 *
 * @details This function should be called by ISP driver to destroy frame stream
 *          when frame stream parameters changes or as a part of the deinitialization
 */
int frame_stream_destroy( const unsigned int ctx_id, const aframe_type_t type, const unsigned int force );

/**
 * @brief   Sets stream parameter to a specified value
 *
 * @param   ctx_id context id to set stream parameter
 * @param   type frame stream type set stream parameter
 * @param   param_id parameter id
 * @param   value parameter value to set
 *
 * @return  0 on success
 *
 * @details This function should be called in order to set stream parameter value of specified context/stream
 */
int frame_stream_set_param( const unsigned int ctx_id, const aframe_type_t type, const frame_stream_param_type_t param_id, const int32_t value );

/**
 * @brief   Gets stream parameter value
 *
 * @param   ctx_id context id to get stream parameter
 * @param   type frame stream type get stream parameter
 * @param   param_id parameter id
 * @param   value returned parameter value
 *
 * @return  0 on success
 *
 * @details This function should be called in order to get stream parameter value of specified context/stream
 */
int frame_stream_get_param( const unsigned int ctx_id, const aframe_type_t type, const frame_stream_param_type_t param_id, int32_t *value );

#endif // __ACAMERA_FRAME_STREAM_API_H__