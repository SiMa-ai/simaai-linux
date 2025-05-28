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

#if !defined( __CROP_FSM_H__ )
#define __CROP_FSM_H__

typedef struct _crop_fsm_t crop_fsm_t;
typedef struct _crop_fsm_t *crop_fsm_ptr_t;
typedef const struct _crop_fsm_t *crop_fsm_const_ptr_t;

enum _crop_state_t {
    crop_state_initialized,
    crop_state_configured,
    crop_state_reload_calibration,
    crop_state_ready,
    crop_state_stopped,
    crop_state_deinit,
    crop_state_invalid
};

typedef enum _crop_state_t crop_state_t;

void crop_init( crop_fsm_ptr_t p_fsm );
void crop_config( crop_fsm_ptr_t p_fsm );
void crop_deinit( crop_fsm_ptr_t p_fsm );

struct _crop_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    crop_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void crop_fsm_clear( crop_fsm_ptr_t p_fsm );
void crop_fsm_switch_state( crop_fsm_ptr_t p_fsm, crop_state_t state );
void crop_fsm_process_state( crop_fsm_ptr_t p_fsm );
uint8_t crop_fsm_process_event( crop_fsm_ptr_t p_fsm, event_id_t event_id );

void crop_fsm_process_interrupt( crop_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void crop_request_interrupt( crop_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __CROP_FSM_H__ */
