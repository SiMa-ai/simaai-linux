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

#if !defined( __RADIAL_SHADING_FSM_H__ )
#define __RADIAL_SHADING_FSM_H__

typedef struct _radial_shading_fsm_t radial_shading_fsm_t;
typedef struct _radial_shading_fsm_t *radial_shading_fsm_ptr_t;
typedef const struct _radial_shading_fsm_t *radial_shading_fsm_const_ptr_t;

enum _radial_shading_state_t {
    radial_shading_state_initialized,
    radial_shading_state_configured,
    radial_shading_state_reload_calibration,
    radial_shading_state_ready,
    radial_shading_state_stopped,
    radial_shading_state_deinit,
    radial_shading_state_invalid
};

typedef enum _radial_shading_state_t radial_shading_state_t;


void radial_shading_init( radial_shading_fsm_ptr_t p_fsm );
void radial_shading_reload_calibration( radial_shading_fsm_ptr_t p_fsm );
void radial_shading_deinit( radial_shading_fsm_ptr_t p_fsm );


struct _radial_shading_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    radial_shading_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void radial_shading_fsm_clear( radial_shading_fsm_ptr_t p_fsm );
void radial_shading_fsm_switch_state( radial_shading_fsm_ptr_t p_fsm, radial_shading_state_t state );
void radial_shading_fsm_process_state( radial_shading_fsm_ptr_t p_fsm );
uint8_t radial_shading_fsm_process_event( radial_shading_fsm_ptr_t p_fsm, event_id_t event_id );

void radial_shading_fsm_process_interrupt( radial_shading_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void radial_shading_request_interrupt( radial_shading_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __RADIAL_SHADING_FSM_H__ */
