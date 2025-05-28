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

#if !defined( __ML_BIST_FSM_H__ )
#define __ML_BIST_FSM_H__

typedef struct _ml_bist_fsm_t ml_bist_fsm_t;
typedef struct _ml_bist_fsm_t *ml_bist_fsm_ptr_t;
typedef const struct _ml_bist_fsm_t *ml_bist_fsm_const_ptr_t;

enum _ml_bist_state_t {
    ml_bist_state_initialized,
    ml_bist_state_configured,
    ml_bist_state_reload_calibration,
    ml_bist_state_ready,
    ml_bist_state_stopped,
    ml_bist_state_deinit,
    ml_bist_state_invalid
};

typedef enum _ml_bist_state_t ml_bist_state_t;

void ml_bist_init( ml_bist_fsm_ptr_t p_fsm );
void ml_bist_prepare_frame( ml_bist_fsm_ptr_t p_fsm );
void ml_bist_frame_end( ml_bist_fsm_ptr_t p_fsm );

struct _ml_bist_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    ml_bist_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    uint8_t current_test_number;
};


void ml_bist_fsm_clear( ml_bist_fsm_ptr_t p_fsm );
void ml_bist_fsm_switch_state( ml_bist_fsm_ptr_t p_fsm, ml_bist_state_t state );
void ml_bist_fsm_process_state( ml_bist_fsm_ptr_t p_fsm );
uint8_t ml_bist_fsm_process_event( ml_bist_fsm_ptr_t p_fsm, event_id_t event_id );

void ml_bist_fsm_process_interrupt( ml_bist_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void ml_bist_request_interrupt( ml_bist_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __ML_BIST_FSM_H__ */
