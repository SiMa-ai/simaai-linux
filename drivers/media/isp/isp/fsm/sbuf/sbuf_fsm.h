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

#if !defined( __SBUF_FSM_H__ )
#define __SBUF_FSM_H__

typedef struct _sbuf_fsm_t sbuf_fsm_t;
typedef struct _sbuf_fsm_t *sbuf_fsm_ptr_t;
typedef const struct _sbuf_fsm_t *sbuf_fsm_const_ptr_t;

enum _sbuf_state_t {
    sbuf_state_initialized,
    sbuf_state_configured,
    sbuf_state_reload_calibration,
    sbuf_state_ready,
    sbuf_state_stopped,
    sbuf_state_deinit,
    sbuf_state_update_ae_idx,
    sbuf_state_update_awb_idx,
    sbuf_state_update_af_idx,
    sbuf_state_update_gamma_idx,
    sbuf_state_invalid
};

typedef enum _sbuf_state_t sbuf_state_t;

void sbuf_fsm_init( sbuf_fsm_ptr_t p_fsm );
void sbuf_config( sbuf_fsm_ptr_t p_fsm );
void sbuf_stop( sbuf_fsm_ptr_t p_fsm );
void sbuf_deinit( sbuf_fsm_ptr_t p_fsm );
void sbuf_update_ae_idx( sbuf_fsm_ptr_t p_fsm );
void sbuf_update_awb_idx( sbuf_fsm_ptr_t p_fsm );
void sbuf_update_af_idx( sbuf_fsm_ptr_t p_fsm );
void sbuf_update_gamma_idx( sbuf_fsm_ptr_t p_fsm );


struct _sbuf_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    sbuf_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    uint32_t opened;
    uint32_t mode;
    uint8_t is_paused;
};


void sbuf_fsm_clear( sbuf_fsm_ptr_t p_fsm );
void sbuf_fsm_switch_state( sbuf_fsm_ptr_t p_fsm, sbuf_state_t state );
void sbuf_fsm_process_state( sbuf_fsm_ptr_t p_fsm );
uint8_t sbuf_fsm_process_event( sbuf_fsm_ptr_t p_fsm, event_id_t event_id );

void sbuf_fsm_process_interrupt( sbuf_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void sbuf_request_interrupt( sbuf_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __SBUF_FSM_H__ */
