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

#ifndef _MODULE_MCFE_H_
#define _MODULE_MCFE_H_

#include "acamera_isp_core_settings.h"
#include "module_mcfe_common.h"

typedef enum {
    MODULE_MCFE_EVENT_RAW_BUFFER_READY = 1,
    MODULE_MCFE_EVENT_OUT_BUFFER_READY,
    MODULE_MCFE_EVENT_MAX = MODULE_MCFE_EVENT_OUT_BUFFER_READY
} module_mcfe_event_t;

// MCFE config data type
typedef struct _module_mcfe_sensor_cfg_t {
    int h_start;
    int v_start;
    int is_remote;
    int num_channel;
    int video_port_id;
    int data_width;
    int width;
    int height;
    int rggb_start;
    int cfa_pattern;
    int cdma_addr;
} module_mcfe_sensor_cfg_t;

typedef struct {
    uint8_t data_width;
    uint8_t msb_align;
    uint8_t h_subsampling;
    uint8_t v_subsampling;
    uint8_t axi;
    uint8_t axi_mode;
    uint8_t axi_format;
} module_mcfe_plane_info_t;

typedef struct {
    uint8_t enabled;
    uint32_t startx;
    uint32_t starty;
    uint32_t width;
    uint32_t height;
} output_crop_t;

#if defined( ISP_HAS_RGB_SCALER_FSM ) || defined( ISP_HAS_RAW_SCALER_FSM )
typedef struct {
    uint8_t enabled;
    uint32_t width;
    uint32_t height;
} output_scaler_t;
#endif

typedef struct {
    uint8_t format;
    uint8_t num_planes;
    output_crop_t crop;
#if defined( ISP_HAS_RGB_SCALER_FSM )
    output_scaler_t rgb_scaler;
#endif
#if defined( ISP_HAS_RAW_SCALER_FSM )
    output_scaler_t raw_scaler;
#endif
    module_mcfe_plane_info_t plane[ISP_MCFE_HWIF_MAX_OUTPUTS];
} module_mcfe_output_cfg_t;

typedef struct _module_mcfe_config_t {
    unsigned char initialized;
    unsigned char running;
    int video_input_table[MODULE_MCFE_INPUT_PORT_MAX];

} module_mcfe_config_t;

// Init, deinit
int module_mcfe_init( void );
int module_mcfe_deinit( void );

// Control, config interface
int module_mcfe_start( void );
int module_mcfe_stop( void );

// Resource related functions
/**
 *   Get slot id for specified MCFE input channel
 *   If first_only is set only returns positive result if input is the first slot input
 */
int module_mcfe_get_slot_id_for_input( int input, int first_only );

/**
 *   Get input channel id for specified slot id
 */
int module_mcfe_get_input_for_slot_id( int slot_id );

/**
 *   Updates usage of MCFE input channels
 *
 *   Finds all slots corresponding to MCFE input channels and updates internal input table
 *
 */
int module_mcfe_update_inputs( void );

/**
 *   Get number of free inputs available for specified slot
 *
 *   If slot_id a valid slot id function will return sum of slot inputs and free inputs
 *   If slot_id is not a valid slot id function will return number of free inputs
 *
 */
int module_mcfe_get_free_inputs_for_slot_id( int slot_id );

#endif
