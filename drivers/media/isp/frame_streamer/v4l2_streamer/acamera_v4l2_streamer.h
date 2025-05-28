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

#ifndef __ACAMERA_V4L2_STREAMER_H__
#define __ACAMERA_V4L2_STREAMER_H__

#include "acamera_frame_stream_api.h"

/**
 * @brief   Gets buffer
 *
 * @param   ctx_id context id to query
 * @param   type frame type to get
 * @param   state frame state to get
 * @param   frame pointer to pointer to the aframe_t structure to be returned
 *
 * @return  0 on success
 *
 * @details This function is the V4L2 frame streamer implementation
 *          of the Acamera Frame Stream API frame_stream_*_get_frame() API
 */
int v4l2_streamer_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame );

/**
 * @brief   Puts buffer
 *
 * @param   frame pointer to configured aframe_t structure
 *
 * @return  0 on success
 *
 * @details This function is the V4L2 frame streamer implementation
 *          of the Acamera Frame Stream API frame_stream_*_put_frame() API
 */
int v4l2_streamer_put_frame( aframe_t *frame );

/**
 * @brief   Creates frame stream based on configuration
 *
 * @param   stream_cfg stream configuration for the new steam to be created

 * @return  0 on success
 *
 * @details This function is the V4L2 frame streamer implementation
 *          of the Acamera Frame Stream API frame_stream_create() API
 */
int v4l2_streamer_create_stream( const frame_stream_cfg_t *stream_cfg );

/**
 * @brief   Destroys specified frame stream
 *
 * @param   ctx_id context id to destroy stream of
 * @param   type frame stream type to destroy
 * @param   force will forcibly destroy frames if set to non-zero
 *
 * @return  0 on success
 *
 * @details This function is the V4L2 frame streamer implementation
 *          of the Acamera Frame Stream API frame_stream_destroy() API
 */
int v4l2_streamer_destroy_stream( const unsigned int ctx_id, const aframe_type_t type, const unsigned int force );

#endif // __ACAMERA_V4L2_STREAMER_H__