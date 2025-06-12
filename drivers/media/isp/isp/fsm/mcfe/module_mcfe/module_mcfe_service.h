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

#ifndef _MODULE_MCFE_SERVICE_H_
#define _MODULE_MCFE_SERVICE_H_

#include "acamera_aframe.h"
#include "acamera_isp_core_settings.h" // ISP_MCFE_HWIF_MAX_OUTPUTS
#include "module_mcfe_common.h"        // MODULE_MCFE_INPUT_PORT_MAX etc
#include "module_mcfe_hwif.h"

/* Enums & types definitions */
typedef enum {
    MODULE_MCFE_SLOT_MODE_INACTIVE,
    MODULE_MCFE_SLOT_MODE_PASS_THRU,
    MODULE_MCFE_SLOT_MODE_TDMF_FIX,
    MODULE_MCFE_SLOT_MODE_TDMF_FLEX,
    MODULE_MCFE_SLOT_MODE_MANUAL,
    MODULE_MCFE_SLOT_MODE_MANUAL_CFG_ONLY,
    MODULE_MCFE_SLOT_MODE_MAX
} module_mcfe_slot_mode_t;

typedef enum {
    MODULE_MCFE_SCHED_MODE_STOPPED,
    MODULE_MCFE_SCHED_MODE_MANUAL,
    MODULE_MCFE_SCHED_MODE_MAX
} module_mcfe_sched_mode_t;

typedef unsigned short mcfe_slot_id_t;

typedef struct _input_port_ids {
    int ids[MODULE_MCFE_INPUT_PORT_MAX];
    int count;
} module_mcfe_input_port_ids_t;

/* Data structures */
typedef struct _module_mcfe_input_cfg_t {
    // Port information
    unsigned char num_channel;

    unsigned char rggb_start;
    unsigned char cfa_pattern;
    // Resolution information
    unsigned short msb_align;
    unsigned short data_width;
    unsigned short active_width;
    unsigned short active_height;

    // Crop information
    unsigned short h_start;
    unsigned short h_size;
    unsigned short v_start;
    unsigned short v_size;
} module_mcfe_input_cfg_t;

typedef struct module_mcfe_buf_t {
    unsigned char enabled;
    unsigned char index;

    mcfe_hwif_buf_type_t type;

    unsigned long long address;
    unsigned short data_width;
    unsigned short flags;
    unsigned int line_offset;
    unsigned int active_width;
    unsigned int active_height;
} module_mcfe_buf_t;

typedef struct module_mcfe_buf_cfg_t {
    mcfe_hwif_buf_type_t type;

    unsigned long long address;
    unsigned short data_width;
    unsigned short flags;
    unsigned int line_offset;
    unsigned int active_width;
    unsigned int active_height;
} module_mcfe_buf_cfg_t;

typedef enum module_mcfe_bufset_type_t {
    MCFE_BUFSET_TYPE_NONE,
    MCFE_BUFSET_TYPE_RAW,
    MCFE_BUFSET_TYPE_OUT,
    MCFE_BUFSET_TYPE_MAX
} module_mcfe_bufset_type_t;

typedef enum module_mcfe_bufset_status_t {
    MCFE_BUFSET_STATUS_NONE,
    MCFE_BUFSET_STATUS_EMPTY,  // Buffer status empty
    MCFE_BUFSET_STATUS_FILLED, // Buffer status filled
    MCFE_BUFSET_STATUS_ANY,    // Buffer status is irrelevant, either empty or filled
    MCFE_BUFSET_STATUS_MAX
} module_mcfe_bufset_status_t;

typedef enum module_mcfe_bufset_swap_mode_t {
    MCFE_BUFSET_SWAP_NONE,
    MCFE_BUFSET_SWAP_EMPTY_IN_FILLED_OUT, // Replace filled buffer with empty one (i.e. output buffer)
    MCFE_BUFSET_SWAP_FILLED_IN_EMPTY_OUT, // Replace empty buffer with filled one (i.e. manual raw buffer)
    MCFE_BUFSET_SWAP_EMPTY_IN_EMPTY_OUT,  // Replace empty buffer with empty one (i.e. tdmf raw buffer)
    MCFE_BUFSET_SWAP_FILLED_IN_ANY_OUT,   // Replace buffer with filled one ( existing buffer state is irrelevant, m2m raw )
    MCFE_BUFSET_SWAP_EMPTY_IN_ANY_OUT,    // Replace buffer with empty one ( existing buffer state is irrelevant, m2m out )
    MCFE_BUFSET_SWAP_NO_IN_EMPTY_OUT,     // Return empty buffer ( no replacement buffer supplied, m2m processed raw )
    MCFE_BUFSET_SWAP_NO_IN_FILLED_OUT,    // Return filled buffer ( no replacement buffer supplied, m2m processed out )
    MCFE_BUFSET_SWAP_NO_IN_ANY_OUT,       // Return buffer with any status ( no replacement buffer supplied, m2m processed, force swap )
    MCFE_BUFSET_SWAP_MAX
} module_mcfe_bufset_swap_mode_t;

typedef enum module_mcfe_bufset_flags_t {
    MCFE_BUFSET_FLAG_CREATE_EMPTY = ( 1 << 0 ) // Create buffset with no actual frame assignment ( address is 0, frame pointer is NULL )
} module_mcfe_bufset_flags_t;

typedef struct module_mcfe_bufset_plane_cfg_t {
    uint32_t width;         // Plane width
    uint32_t height;        // Plane height
    uint32_t line_offset;   // Plane line offset
    uint32_t data_width;    // Data bit width
    aplane_hw_cfg_t hw_cfg; // Hardware specific configuration
} module_mcfe_bufset_plane_cfg_t;

typedef struct module_mcfe_bufset_frames_t {
    unsigned int num_frames;
    unsigned int num_planes;
    module_mcfe_bufset_plane_cfg_t cached_planes[AFRAME_MAX_PLANES]; // Cached plane configuration ( accessible when no frames are available )
    aframe_t *frame[MODULE_MCFE_BUFSET_MAX_FRAMES];                  // Pointers to associated aframes (Use 2 if double-buffered)
} module_mcfe_bufset_frames_t;

typedef struct module_mcfe_bufset_buffs_t {
    unsigned int num_planes;
    unsigned int num_buffers;
    mcfe_hwif_buf_type_t buffer_type;
    module_mcfe_buf_t *buff[MODULE_MCFE_BUFSET_MAX_FRAMES][AFRAME_MAX_PLANES]; // Pointers to hardware buffers assigned to aframe planes (Use 2 if double-buffered )
} module_mcfe_bufset_buffs_t;

typedef struct module_mcfe_bufset_t {
    unsigned char enabled;
    unsigned char index;
    unsigned int flags;

    module_mcfe_bufset_type_t type;

    module_mcfe_bufset_frames_t frames;
    module_mcfe_bufset_buffs_t buffs;

} module_mcfe_bufset_t;

typedef struct _module_mcfe_slot_cfg_t {
    module_mcfe_slot_mode_t slot_mode;
    unsigned char rggb_start;
    unsigned char cfa_pattern;

    unsigned char is_remote;
    unsigned char num_input;
    unsigned char num_output;
    unsigned char slot_input_port_id[MODULE_MCFE_INPUT_PORT_MAX];
    const module_mcfe_bufset_t *slot_bufset_raw;
    const module_mcfe_bufset_t *slot_bufset_out;

    unsigned long cdma_addr;
    uint8_t hist_position_is_be;
} module_mcfe_slot_cfg_t;

/* General functions */
int module_mcfe_service_init( void );

/* Scheduler control interface */
int module_mcfe_scheduler_set_mode( module_mcfe_sched_mode_t mode );
int module_mcfe_scheduler_start( void );
int module_mcfe_scheduler_stop( void );

/* Input control interface */
int module_mcfe_input_add_video( int video_input_idx, module_mcfe_input_port_ids_t *input_port_ids,    // configures input ports & xbar,
                                 module_mcfe_input_cfg_t *input_cfg );                                 // then return video_idx & input_port_id
int module_mcfe_input_remove_video( int video_input_idx, module_mcfe_input_port_ids_t *input_port_ids, // deallocates input ports & xbar
                                    module_mcfe_input_cfg_t *input_cfg );                              //

int module_mcfe_input_mode_manual( module_mcfe_input_cfg_t *input_cfg );                        // configures MCFE input channels for manual mode slot operation
int module_mcfe_input_get_input_cfg( int video_input_idx, module_mcfe_input_cfg_t *input_cfg ); // returns configuration for video_input
int module_mcfe_input_config_buffer( const module_mcfe_input_port_ids_t *input_port_ids,
                                     const module_mcfe_bufset_t *const input_bufset );                  // configures input buffers to input port
int module_mcfe_input_get_input_bufset( int input_port_id, const module_mcfe_bufset_t **input_bufset ); // returns input_bufq_id for input_port_id
int module_mcfe_input_start_video( int video_input_idx );                                               // starts input port in video_input_idx
int module_mcfe_input_stop_video( int video_input_idx );                                                // stops input port in video_input_idx

/* Slot control interface */
#if 0 // DYNAMIC_SLOT_ALLOCATION
int module_mcfe_slot_create( mcfe_slot_id_t *slot_id, module_mcfe_slot_cfg_t *slot_cfg );
#else
int module_mcfe_slot_create( mcfe_slot_id_t slot_id, module_mcfe_slot_cfg_t *slot_cfg );
#endif
int module_mcfe_slot_set_config( mcfe_slot_id_t slot_id, module_mcfe_slot_cfg_t *slot_cfg );
int module_mcfe_slot_get_config( mcfe_slot_id_t slot_id, module_mcfe_slot_cfg_t *slot_cfg );
int module_mcfe_slot_get_slot_id_with_input_id( mcfe_slot_id_t *slot_id, unsigned char input_id );
int module_mcfe_slot_start( mcfe_slot_id_t slot_id );
int module_mcfe_slot_trigger_manual( mcfe_slot_id_t slot_id );
int module_mcfe_slot_cdma_address_update( unsigned short slot_id, unsigned int addr_cfg_cdma, unsigned int addr_stats_cdma );
int module_mcfe_slot_stop( mcfe_slot_id_t slot_id );
int module_mcfe_slot_destroy( mcfe_slot_id_t slot_id );
int module_mcfe_slot_get_free_inputs( mcfe_slot_id_t slot_id );

/* Buffer set control interface */

/**
 * @brief Creates buffer set based on passed frames configuration (planes, type, etc...)
 *
 * @param frames        Pointer to array of pointers of one or two frames
 * @param bufset        Pointer to pointer to module_mcfe_bufset_id_t to be initialised
 * @param num_frames    Number of frames passed. 1 for single-buffered and 2 for double-buffered buffer set
 * @param flags         Flags from module_mcfe_bufset_flags_t enum
 * @return              MCFE_ERR_NONE if no error
 */
int module_mcfe_bufset_create( aframe_t *frames[], module_mcfe_bufset_t **bufset, const unsigned int num_frames, const unsigned int flags );

/**
 * @brief Destroys buffer set. All assigned HW buffers allocated marked as available
 *
 * @param bufset    Buffer set pointer
 * @return          MCFE_ERR_NONE if no error
 */
int module_mcfe_bufset_destroy( module_mcfe_bufset_t *bufset );

/**
 * @brief Swaps filled frame with empty frame if HW buffers of specified buffer set are filled.
 *
 * @param in        Input frame to replace one of the buffer set
 * @param out       Returned pointer to replaced frame
 * @param bufset    Buffer set pointer
 * @param mode      Swap mode specifying how frames should be swapped
 * @return          MCFE_ERR_NONE if no error
 * @details         This function usually called for output buffer sets when processing is done
 *                  to retrieve processed frame and replaced it with the new empty one.
 *                  In addition to that manual use-case calls this function to swap raw buffers
 *                  when new frame from sensor received
 */
int module_mcfe_bufset_swap( aframe_t *in,
                             aframe_t **out,
                             module_mcfe_bufset_t *bufset,
                             const module_mcfe_bufset_swap_mode_t mode );

/**
 * @brief Gets empty HW buffer id for a given buffer set and plane number (and a frame number if specified)
 *
 * @param bufset    Pointer to buffer set to check
 * @param frame_no  Frame number. -1 For any
 * @param plane_no  Plane number
 * @param status    Buffer set status to specify what kind of buffers we are interested in
 * @param buf_id    Returned id of an empty HW buffer
 * @return          MCFE_ERR_NONE if no error
 */
int module_mcfe_bufset_get_buf_id( const module_mcfe_bufset_t *bufset,
                                   const int frame_no,
                                   const int plane_no,
                                   const module_mcfe_bufset_status_t status,
                                   uint8_t *buf_id );

/**
 * @brief Sets status of all HW buffers in the buffer set to specified value.
 *
 * @param bufset    Buffer set pointer
 * @param status    Buffer set status
 * @return          MCFE_ERR_NONE if no error
 * @details         This function usually called when we need to reset status of the buffers of entire bufset
 *                  For example when failed to get empty output buffer for swap operation, reuse existing buffers
 *                  by reseting their status to empty
 */
int module_mcfe_bufset_set_buf_status( const module_mcfe_bufset_t *bufset, const module_mcfe_bufset_status_t status );

/**
 * @brief Gets frames description structure from buffer set.
 *
 * @param bufset    Buffer set pointer
 * @param frames    Pointer to frames structure to be populated
 * @return          MCFE_ERR_NONE if no error
 * @details         This function usually called when we need to get all frame pointers of a buffer set.
 *                  Normally it would happen before destroying buffer set to recycle frames held by buffer set
 */
int module_mcfe_bufset_get_frames( const module_mcfe_bufset_t *bufset, module_mcfe_bufset_frames_t *frames );
#endif // _MODULE_MCFE_SERVICE_H_
