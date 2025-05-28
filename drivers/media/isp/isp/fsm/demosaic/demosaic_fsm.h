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

#if !defined( __DEMOSAIC_FSM_H__ )
#define __DEMOSAIC_FSM_H__

typedef struct _demosaic_fsm_t demosaic_fsm_t;
typedef struct _demosaic_fsm_t *demosaic_fsm_ptr_t;
typedef const struct _demosaic_fsm_t *demosaic_fsm_const_ptr_t;

enum _demosaic_state_t {
    demosaic_state_initialized,
    demosaic_state_configured,
    demosaic_state_reload_calibration,
    demosaic_state_ready,
    demosaic_state_stopped,
    demosaic_state_deinit,
    demosaic_state_invalid
};

typedef enum _demosaic_state_t demosaic_state_t;


void demosaic_init( demosaic_fsm_ptr_t p_fsm );
void demosaic_reload_calibration( demosaic_fsm_ptr_t p_fsm );
void demosaic_deinit( demosaic_fsm_ptr_t p_fsm );


struct _demosaic_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    demosaic_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void demosaic_fsm_clear( demosaic_fsm_ptr_t p_fsm );
void demosaic_fsm_switch_state( demosaic_fsm_ptr_t p_fsm, demosaic_state_t state );
void demosaic_fsm_process_state( demosaic_fsm_ptr_t p_fsm );
uint8_t demosaic_fsm_process_event( demosaic_fsm_ptr_t p_fsm, event_id_t event_id );

void demosaic_fsm_process_interrupt( demosaic_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void demosaic_request_interrupt( demosaic_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __DEMOSAIC_FSM_H__ */
