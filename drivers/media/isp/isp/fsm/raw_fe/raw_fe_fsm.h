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

#if !defined( __RAW_FE_FSM_H__ )
#define __RAW_FE_FSM_H__

typedef struct _raw_fe_fsm_t raw_fe_fsm_t;
typedef struct _raw_fe_fsm_t *raw_fe_fsm_ptr_t;
typedef const struct _raw_fe_fsm_t *raw_fe_fsm_const_ptr_t;

enum _raw_fe_state_t {
    raw_fe_state_initialized,
    raw_fe_state_configured,
    raw_fe_state_reload_calibration,
    raw_fe_state_ready,
    raw_fe_state_stopped,
    raw_fe_state_deinit,
    raw_fe_state_update_hw,
    raw_fe_state_invalid
};

typedef enum _raw_fe_state_t raw_fe_state_t;


void raw_fe_init( raw_fe_fsm_ptr_t p_fsm );
void raw_fe_config( raw_fe_fsm_ptr_t p_fsm );
void raw_fe_update_hw( raw_fe_fsm_ptr_t p_fsm );
void raw_fe_reload_calibration( raw_fe_fsm_ptr_t p_fsm );
void raw_fe_deinit( raw_fe_fsm_ptr_t p_fsm );


struct _raw_fe_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    raw_fe_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void raw_fe_fsm_clear( raw_fe_fsm_ptr_t p_fsm );
void raw_fe_fsm_switch_state( raw_fe_fsm_ptr_t p_fsm, raw_fe_state_t state );
void raw_fe_fsm_process_state( raw_fe_fsm_ptr_t p_fsm );
uint8_t raw_fe_fsm_process_event( raw_fe_fsm_ptr_t p_fsm, event_id_t event_id );

void raw_fe_fsm_process_interrupt( raw_fe_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void raw_fe_request_interrupt( raw_fe_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __RAW_FE_FSM_H__ */
