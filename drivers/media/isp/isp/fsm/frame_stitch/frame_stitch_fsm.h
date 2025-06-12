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

#if !defined( __FRAME_STITCH_FSM_H__ )
#define __FRAME_STITCH_FSM_H__

typedef struct _frame_stitch_fsm_t frame_stitch_fsm_t;
typedef struct _frame_stitch_fsm_t *frame_stitch_fsm_ptr_t;
typedef const struct _frame_stitch_fsm_t *frame_stitch_fsm_const_ptr_t;

enum _frame_stitch_state_t {
    frame_stitch_state_initialized,
    frame_stitch_state_configured,
    frame_stitch_state_reload_calibration,
    frame_stitch_state_ready,
    frame_stitch_state_stopped,
    frame_stitch_state_deinit,
    frame_stitch_state_update_hw,
    frame_stitch_state_invalid
};

typedef enum _frame_stitch_state_t frame_stitch_state_t;


void frame_stitch_init( frame_stitch_fsm_ptr_t p_fsm );
void frame_stitch_config( frame_stitch_fsm_ptr_t p_fsm );
void frame_stitch_update_hw( frame_stitch_fsm_ptr_t p_fsm );
void frame_stitch_reload_calibration( frame_stitch_fsm_ptr_t p_fsm );
void frame_stitch_deinit( frame_stitch_fsm_ptr_t p_fsm );


struct _frame_stitch_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    frame_stitch_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void frame_stitch_fsm_clear( frame_stitch_fsm_ptr_t p_fsm );
void frame_stitch_fsm_switch_state( frame_stitch_fsm_ptr_t p_fsm, frame_stitch_state_t state );
void frame_stitch_fsm_process_state( frame_stitch_fsm_ptr_t p_fsm );
uint8_t frame_stitch_fsm_process_event( frame_stitch_fsm_ptr_t p_fsm, event_id_t event_id );

void frame_stitch_fsm_process_interrupt( frame_stitch_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void frame_stitch_request_interrupt( frame_stitch_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __FRAME_STITCH_FSM_H__ */
