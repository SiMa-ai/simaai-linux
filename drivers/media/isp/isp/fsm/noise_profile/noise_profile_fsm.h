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

#if !defined( __NOISE_PROFILE_FSM_H__ )
#define __NOISE_PROFILE_FSM_H__

typedef struct _noise_profile_fsm_t noise_profile_fsm_t;
typedef struct _noise_profile_fsm_t *noise_profile_fsm_ptr_t;
typedef const struct _noise_profile_fsm_t *noise_profile_fsm_const_ptr_t;

enum _noise_profile_state_t {
    noise_profile_state_initialized,
    noise_profile_state_configured,
    noise_profile_state_reload_calibration,
    noise_profile_state_ready,
    noise_profile_state_stopped,
    noise_profile_state_deinit,
    noise_profile_state_invalid
};

typedef enum _noise_profile_state_t noise_profile_state_t;


void noise_profile_init( noise_profile_fsm_ptr_t p_fsm );
void noise_profile_reload_calibration( noise_profile_fsm_ptr_t p_fsm );
void noise_profile_deinit( noise_profile_fsm_ptr_t p_fsm );


struct _noise_profile_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    noise_profile_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void noise_profile_fsm_clear( noise_profile_fsm_ptr_t p_fsm );
void noise_profile_fsm_switch_state( noise_profile_fsm_ptr_t p_fsm, noise_profile_state_t state );
void noise_profile_fsm_process_state( noise_profile_fsm_ptr_t p_fsm );
uint8_t noise_profile_fsm_process_event( noise_profile_fsm_ptr_t p_fsm, event_id_t event_id );

void noise_profile_fsm_process_interrupt( noise_profile_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void noise_profile_request_interrupt( noise_profile_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __NOISE_PROFILE_FSM_H__ */
