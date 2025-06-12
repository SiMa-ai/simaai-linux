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

#ifndef __ACAMERA_AFRAME_H__
#define __ACAMERA_AFRAME_H__

#include "system_types.h"

#define AFRAME_MAX_PLANES ( 4 )

// Frame plane alignment requirements due to DMA engine access
#define AFRAME_PLANE_ALIGNMENT ( 32 )

#define AFRAME_ALIGN_PLANE( val )                \
    ( ( ( val ) + AFRAME_PLANE_ALIGNMENT - 1 ) - \
      ( ( ( val ) + AFRAME_PLANE_ALIGNMENT - 1 ) % AFRAME_PLANE_ALIGNMENT ) )

typedef enum aframe_hw_flags_t {
    AFRAME_HW_FLAG_MSB_ALIGN = ( 1 << 0 ), // HW specific flag MSB_ALIGN
    AFRAME_HW_FLAG_POOL = ( 1 << 1 ),      // HW specific flag POOL
    AFRAME_HW_FLAG_CLONE = ( 1 << 2 ),     // HW specific flag CLONE
    AFRAME_HW_FLAG_INFINITE = ( 1 << 3 )   // HW specific flag INFINITE
} aframe_hw_flags_t;

typedef enum aframe_type_t {
    AFRAME_TYPE_UNKNOWN, // Default value, unknown type
    AFRAME_TYPE_RAW,     // Raw frame type
    AFRAME_TYPE_OUT,     // Output frame type
    AFRAME_TYPE_META,    // Metadata frame type
    AFRAME_TYPE_MAX      // Max type value
} aframe_type_t;

typedef enum aframe_state_t {
    AFRAME_STATE_UNKNOWN, // Default value, unknown state
    AFRAME_STATE_EMPTY,   // Frame is empty
    AFRAME_STATE_FULL,    // Frame is full
    AFRAME_STATE_MAX      // Max state value
} aframe_state_t;

typedef enum aframe_source_t {
    AFRAME_SOURCE_UNKNOWN,              // Default value, unknown source
    AFRAME_SOURCE_DEFAULT_STREAMER,     // Frame originated from Default Frame Streamer
    AFRAME_SOURCE_SIMPLE_FRAME_STORAGE, // Frame originated from Simple Frame Storage
    AFRAME_SOURCE_V4L2_STREAMER,        // Frame originated from V4L2 Frame Streamer
    AFRAME_SOURCE_MAX                   // Max source value
} aframe_source_t;

typedef enum aframe_memory_t {
    AFRAME_MEMORY_UNKNOWN, // Default value, unknown memory
    AFRAME_MEMORY_AUTO,    // Frame is automatically allocated by the corresponding streamer
    AFRAME_MEMORY_USER,    // Frame is allocated by user
    AFRAME_MEMORY_MAX      // Max memory type value
} aframe_memory_t;

typedef struct aplane_hw_cfg_t {
    uint32_t axi;   // AXI output used
    uint32_t flags; // HW buffer flags
} aplane_hw_cfg_t;

typedef struct aplane_t {
    void *private;        // Embedded plane private data
    uint32_t width;       // Plane width
    uint32_t height;      // Plane height
    uint32_t line_offset; // Plane line offset
    uint32_t data_width;  // Plane data width
    uint32_t length;      // Total plane size in bytes (not payload)
    uint32_t bytes_used;  // Number of bytes used
    uint32_t data_offset; // Offset in the plane to the start of data
    struct {
        uint32_t low;  // Start of memory block (low 32-bit of address)
        uint32_t high; // Start of memory block (high 32-bit of address)
    } address;
    void *virt_addr;        // Virtual memory address if present
    uint32_t crc;           // Plane CRC checksum
    aplane_hw_cfg_t hw_cfg; // Hardware specific configuration
} aplane_t;

typedef struct aframe_t {
    void *private;                      // embedded frame private data
    uint32_t context_id;                // context id
    aframe_source_t source;             // frame source origin
    aframe_memory_t memory;             // frame memory type
    aframe_type_t type;                 // frame type
    uint32_t format;                    // frame format
    aframe_state_t state;               // frame state
    uint32_t frame_id;                  // frame id (buffer index)
    uint32_t sequence;                  // frame sequence number
    uint32_t num_planes;                // number of planes
    aplane_t planes[AFRAME_MAX_PLANES]; // planes information
} aframe_t;

#endif // __ACAMERA_AFRAME_H__