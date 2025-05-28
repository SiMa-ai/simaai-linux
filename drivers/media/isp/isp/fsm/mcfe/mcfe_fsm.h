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

#if !defined( __MCFE_FSM_H__ )
#define __MCFE_FSM_H__

typedef struct _mcfe_fsm_t mcfe_fsm_t;
typedef struct _mcfe_fsm_t *mcfe_fsm_ptr_t;
typedef const struct _mcfe_fsm_t *mcfe_fsm_const_ptr_t;

enum _mcfe_state_t {
    mcfe_state_initialized,
    mcfe_state_configured,
    mcfe_state_reload_calibration,
    mcfe_state_ready,
    mcfe_state_stopped,
    mcfe_state_deinit,
    mcfe_state_out_buffer_ready,
    mcfe_state_raw_buffer_ready,
    mcfe_state_invalid
};

typedef enum _mcfe_state_t mcfe_state_t;

#include "module_mcfe_usecase.h"
void mcfe_init( mcfe_fsm_ptr_t p_fsm );
void mcfe_config( mcfe_fsm_ptr_t p_fsm );
void mcfe_start( mcfe_fsm_ptr_t p_fsm );
void mcfe_stop( mcfe_fsm_ptr_t p_fsm );
void mcfe_deinit( mcfe_fsm_ptr_t p_fsm );
void mcfe_raw_buffer_ready( mcfe_fsm_ptr_t p_fsm );
void mcfe_out_buffer_ready( mcfe_fsm_ptr_t p_fsm );

struct _mcfe_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    mcfe_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    module_mcfe_usecase_config_t usecase_cfg;
};


void mcfe_fsm_clear( mcfe_fsm_ptr_t p_fsm );
void mcfe_fsm_switch_state( mcfe_fsm_ptr_t p_fsm, mcfe_state_t state );
void mcfe_fsm_process_state( mcfe_fsm_ptr_t p_fsm );
uint8_t mcfe_fsm_process_event( mcfe_fsm_ptr_t p_fsm, event_id_t event_id );

void mcfe_fsm_process_interrupt( mcfe_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void mcfe_request_interrupt( mcfe_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __MCFE_FSM_H__ */
