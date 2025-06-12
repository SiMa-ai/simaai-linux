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

#if !defined( __DEFECT_PIXEL_FSM_H__ )
#define __DEFECT_PIXEL_FSM_H__

typedef struct _defect_pixel_fsm_t defect_pixel_fsm_t;
typedef struct _defect_pixel_fsm_t *defect_pixel_fsm_ptr_t;
typedef const struct _defect_pixel_fsm_t *defect_pixel_fsm_const_ptr_t;

enum _defect_pixel_state_t {
    defect_pixel_state_initialized,
    defect_pixel_state_configured,
    defect_pixel_state_reload_calibration,
    defect_pixel_state_ready,
    defect_pixel_state_stopped,
    defect_pixel_state_deinit,
    defect_pixel_state_update_hw,
    defect_pixel_state_invalid
};

typedef enum _defect_pixel_state_t defect_pixel_state_t;


void defect_pixel_init( defect_pixel_fsm_ptr_t p_fsm );
void defect_pixel_reload_calibration( defect_pixel_fsm_ptr_t p_fsm );
void defect_pixel_update_hw( defect_pixel_fsm_ptr_t p_fsm );
void defect_pixel_deinit( defect_pixel_fsm_ptr_t p_fsm );

struct _defect_pixel_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    defect_pixel_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    uint8_t frame_counter;
    uint16_t hp_strength;
    uint8_t hp_started;
};


void defect_pixel_fsm_clear( defect_pixel_fsm_ptr_t p_fsm );
void defect_pixel_fsm_switch_state( defect_pixel_fsm_ptr_t p_fsm, defect_pixel_state_t state );
void defect_pixel_fsm_process_state( defect_pixel_fsm_ptr_t p_fsm );
uint8_t defect_pixel_fsm_process_event( defect_pixel_fsm_ptr_t p_fsm, event_id_t event_id );

void defect_pixel_fsm_process_interrupt( defect_pixel_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void defect_pixel_request_interrupt( defect_pixel_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __DEFECT_PIXEL_FSM_H__ */
