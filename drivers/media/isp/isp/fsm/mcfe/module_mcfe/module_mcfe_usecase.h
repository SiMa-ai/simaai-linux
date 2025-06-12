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

#ifndef _MODULE_MCFE_USECASE_H_
#define _MODULE_MCFE_USECASE_H_

#include "acamera_frame_stream_api.h"
#include "module_mcfe.h"
#include "module_mcfe_service.h"


/* Types and definitions */
typedef enum {
    MODULE_MCFE_USECASE_NONE,
    MODULE_MCFE_USECASE_TDMF,
    MODULE_MCFE_USECASE_M2M,
    MODULE_MCFE_USECASE_MAX = MODULE_MCFE_USECASE_M2M
} module_mcfe_usecase_type_t;

typedef enum {
    MODULE_MCFE_USECASE_RAW_STREAM_CONFIGURED = ( 1U << 0 ),
    MODULE_MCFE_USECASE_OUT_STREAM_CONFIGURED = ( 1U << 1 ),
    MODULE_MCFE_USECASE_RAW_BUFSET_CONFIGURED = ( 1U << 2 ),
    MODULE_MCFE_USECASE_OUT_BUFSET_CONFIGURED = ( 1U << 3 ),
    MODULE_MCFE_USECASE_SLOT_CONFIGURED = ( 1U << 4 ),
    MODULE_MCFE_USECASE_INPUT_CONFIGURED = ( 1U << 5 ),
} module_mcfe_usecase_config_state_t;

struct _module_mcfe_usecase_config_t;
typedef struct _module_mcfe_usecase_config_t module_mcfe_usecase_config_t;

typedef struct module_mcfe_usecase_config_states_t {
    unsigned char reqd; // Required config state to run use-case
    unsigned char curr; // Current config state
    unsigned char last; // Config state on last check
} module_mcfe_usecase_config_states_t;

/**
 * @brief Possible frame swap status returned after frame event is processed
 * 
 */
typedef enum {
    MODULE_MCFE_USECASE_FRAME_SET_SWAP_ERROR,
    MODULE_MCFE_USECASE_FRAME_SET_SWAP_SUCCESS
} module_mcfe_usecase_event_frame_swap_status_t;

/**
 * @brief Frame status information structure
 * 
 */
typedef struct module_mcfe_usecase_frame_status_t {
    int set_swap_status;    // frame set/swap operation status
    aframe_memory_t memory; // frame memory type
    aframe_type_t type;     // frame type
    uint32_t num_planes;    // number of planes
    struct {
        uint32_t low;                   // start of memory block (low 32-bit of address)
        uint32_t high;                  // start of memory block (high 32-bit of address)
    } plane_address[AFRAME_MAX_PLANES]; // plane address information
} module_mcfe_usecase_frame_status_t;
/**
 * @brief Status information structure filled by some use-case API functions
 * 
 */
typedef struct module_mcfe_usecase_status_info_t {
    module_mcfe_usecase_frame_status_t raw; // raw frame status information
    module_mcfe_usecase_frame_status_t out; // output frame status information
} module_mcfe_usecase_status_info_t;

typedef struct _module_mcfe_usecase_functbl_t {
    int ( *config )( module_mcfe_usecase_config_t *, module_mcfe_sensor_cfg_t *, module_mcfe_output_cfg_t *, module_mcfe_usecase_status_info_t * );
    int ( *start )( module_mcfe_usecase_config_t * );
    int ( *stop )( module_mcfe_usecase_config_t * );
    int ( *process_event )( module_mcfe_usecase_config_t *, module_mcfe_event_t, module_mcfe_usecase_status_info_t * );
    int ( *release_resources )( module_mcfe_usecase_config_t * );
} module_mcfe_usecase_functbl_t;

typedef struct _module_mcfe_usecase_config_t {
    int initialized;
    int started;
    module_mcfe_usecase_config_states_t config_states;
    module_mcfe_usecase_type_t type;
    mcfe_slot_id_t slot_id;
    module_mcfe_slot_mode_t slot_mode;
    module_mcfe_usecase_functbl_t functions;
    module_mcfe_input_cfg_t input_config;
    module_mcfe_sensor_cfg_t sensor_config;
    module_mcfe_output_cfg_t output_config;
    module_mcfe_bufset_t *bufset_raw;
    module_mcfe_bufset_t *bufset_out;
    module_mcfe_input_port_ids_t input_port_ids;
    int video_input_idx;
    uint8_t hist_position_is_be;
    uint32_t frame_sequence;

} module_mcfe_usecase_config_t;

typedef int ( *module_mcfe_usecase_init_func_t )( module_mcfe_usecase_config_t *, mcfe_slot_id_t, int hist_position_is_be );
typedef int ( *module_mcfe_usecase_deinit_func_t )( module_mcfe_usecase_config_t * );

/* Init, deinit */
int module_mcfe_usecase_init( module_mcfe_usecase_config_t *, module_mcfe_usecase_type_t type, mcfe_slot_id_t slot_id, int hist_position_is_be );
int module_mcfe_usecase_deinit( module_mcfe_usecase_config_t * );

#endif
