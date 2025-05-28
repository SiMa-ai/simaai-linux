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

#if !defined( __IRIDIX_FSM_H__ )
#define __IRIDIX_FSM_H__

typedef struct _iridix_fsm_t iridix_fsm_t;
typedef struct _iridix_fsm_t *iridix_fsm_ptr_t;
typedef const struct _iridix_fsm_t *iridix_fsm_const_ptr_t;

enum _iridix_state_t {
    iridix_state_initialized,
    iridix_state_configured,
    iridix_state_reload_calibration,
    iridix_state_ready,
    iridix_state_stopped,
    iridix_state_deinit,
    iridix_state_update_hw,
    iridix_state_update_algo,
    iridix_state_invalid
};

typedef enum _iridix_state_t iridix_state_t;


void iridix_config( iridix_fsm_ptr_t p_fsm );
void iridix_reload_calibration( iridix_fsm_ptr_t p_fsm );
void iridix_update_algo( iridix_fsm_ptr_t p_fsm );
void iridix_update_hw( iridix_fsm_ptr_t p_fsm );


struct _iridix_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    iridix_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    uint16_t strength_target;
    uint32_t strength_avg;
    uint16_t dark_enh;
    uint32_t iridix_contrast;
    uint32_t luma_slope;
    uint32_t luma_th;
};


void iridix_fsm_clear( iridix_fsm_ptr_t p_fsm );
void iridix_fsm_switch_state( iridix_fsm_ptr_t p_fsm, iridix_state_t state );
void iridix_fsm_process_state( iridix_fsm_ptr_t p_fsm );
uint8_t iridix_fsm_process_event( iridix_fsm_ptr_t p_fsm, event_id_t event_id );

void iridix_fsm_process_interrupt( iridix_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void iridix_request_interrupt( iridix_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __IRIDIX_FSM_H__ */
