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

#if !defined( __SENSOR_FSM_H__ )
#define __SENSOR_FSM_H__

typedef struct _sensor_fsm_t sensor_fsm_t;
typedef struct _sensor_fsm_t *sensor_fsm_ptr_t;
typedef const struct _sensor_fsm_t *sensor_fsm_const_ptr_t;

enum _sensor_state_t {
    sensor_state_initialized,
    sensor_state_configured,
    sensor_state_reload_calibration,
    sensor_state_ready,
    sensor_state_stopped,
    sensor_state_deinit,
    sensor_state_update_hw,
    sensor_state_request_next_frame,
    sensor_state_invalid
};

typedef enum _sensor_state_t sensor_state_t;

#include "acamera_fsmgr_general_router.h"


void sensor_init( sensor_fsm_ptr_t p_fsm );
void sensor_deinit( sensor_fsm_ptr_t p_fsm );
void sensor_config( sensor_fsm_ptr_t p_fsm );
void sensor_ready( sensor_fsm_ptr_t p_fsm );
void sensor_stopped( sensor_fsm_ptr_t p_fsm );
void sensor_update_hw( sensor_fsm_ptr_t p_fsm );
void sensor_get_general_sensor_info( sensor_fsm_ptr_t p_fsm, acamera_cmd_sensor_info *p_sensor_info );
int sensor_get_general_sensor_preset_info( sensor_fsm_ptr_t p_fsm, uint32_t sensor_preset_id, acamera_cmd_sensor_preset_info_t *p_sensor_preset_info );
uint32_t sensor_get_lines_second( sensor_fsm_ptr_t p_fsm );
int remote_sensor_put_frame_callback_handler( void *owner, void *frame );
int remote_sensor_get_frame_callback_handler( void *owner, void **frame );
int remote_sensor_release_frame_callback_handler( void *owner, void *frame );
void sensor_request_next_frame( sensor_fsm_ptr_t p_fsm );

struct _sensor_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    sensor_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    uint8_t sensor_pos;
    acamera_cmd_sensor_info general_sensor_info;
    uint32_t black_level;
};


void sensor_fsm_clear( sensor_fsm_ptr_t p_fsm );
void sensor_fsm_switch_state( sensor_fsm_ptr_t p_fsm, sensor_state_t state );
void sensor_fsm_process_state( sensor_fsm_ptr_t p_fsm );
uint8_t sensor_fsm_process_event( sensor_fsm_ptr_t p_fsm, event_id_t event_id );

void sensor_fsm_process_interrupt( sensor_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void sensor_request_interrupt( sensor_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );


/* The depth a pedestal is expressed in, from a mode description. Exposed so a
 * caller that refreshes the mode after the sensor info was assembled can
 * re-derive the domain instead of leaving it paired with the old mode. */
uint32_t sensor_black_level_bits_of( acamera_calib_mgr_entry_t *p_cm, uint32_t wdr_mode,
                                     uint32_t operating_bits );

#endif /* __SENSOR_FSM_H__ */