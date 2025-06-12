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

#if !defined( __CAC_FSM_H__ )
#define __CAC_FSM_H__

typedef struct _cac_fsm_t cac_fsm_t;
typedef struct _cac_fsm_t *cac_fsm_ptr_t;
typedef const struct _cac_fsm_t *cac_fsm_const_ptr_t;

enum _cac_state_t {
    cac_state_initialized,
    cac_state_configured,
    cac_state_reload_calibration,
    cac_state_ready,
    cac_state_stopped,
    cac_state_deinit,
    cac_state_invalid
};

typedef enum _cac_state_t cac_state_t;


void cac_init( cac_fsm_ptr_t p_fsm );
void cac_config( cac_fsm_ptr_t p_fsm );
void cac_reload_calibration( cac_fsm_ptr_t p_fsm );
void cac_deinit( cac_fsm_ptr_t p_fsm );


struct _cac_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    cac_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void cac_fsm_clear( cac_fsm_ptr_t p_fsm );
void cac_fsm_switch_state( cac_fsm_ptr_t p_fsm, cac_state_t state );
void cac_fsm_process_state( cac_fsm_ptr_t p_fsm );
uint8_t cac_fsm_process_event( cac_fsm_ptr_t p_fsm, event_id_t event_id );

void cac_fsm_process_interrupt( cac_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void cac_request_interrupt( cac_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __CAC_FSM_H__ */
