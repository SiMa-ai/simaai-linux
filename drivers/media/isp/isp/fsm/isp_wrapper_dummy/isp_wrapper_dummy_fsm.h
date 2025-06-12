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

#if !defined( __ISP_WRAPPER_FSM_H__ )
#define __ISP_WRAPPER_FSM_H__

typedef struct _isp_wrapper_fsm_t isp_wrapper_fsm_t;
typedef struct _isp_wrapper_fsm_t *isp_wrapper_fsm_ptr_t;
typedef const struct _isp_wrapper_fsm_t *isp_wrapper_fsm_const_ptr_t;

enum _isp_wrapper_state_t {
    isp_wrapper_state_initialized,
    isp_wrapper_state_configured,
    isp_wrapper_state_reload_calibration,
    isp_wrapper_state_ready,
    isp_wrapper_state_stopped,
    isp_wrapper_state_deinit,
    isp_wrapper_state_invalid
};

typedef enum _isp_wrapper_state_t isp_wrapper_state_t;


void isp_wrapper_configure_raw_buffer_addr_translation( uint32_t ctx_id, uint8_t type, uint32_t high, uint32_t low );
void isp_wrapper_configure_out_buffer_addr_translation( uint32_t ctx_id, uint8_t type, uint32_t high, uint32_t low );
void isp_wrapper_init( isp_wrapper_fsm_ptr_t p_fsm );
void isp_wrapper_config( isp_wrapper_fsm_ptr_t p_fsm );
void isp_wrapper_start( isp_wrapper_fsm_ptr_t p_fsm );
void isp_wrapper_stop( isp_wrapper_fsm_ptr_t p_fsm );


struct _isp_wrapper_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    isp_wrapper_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void isp_wrapper_fsm_clear( isp_wrapper_fsm_ptr_t p_fsm );
void isp_wrapper_fsm_switch_state( isp_wrapper_fsm_ptr_t p_fsm, isp_wrapper_state_t state );
void isp_wrapper_fsm_process_state( isp_wrapper_fsm_ptr_t p_fsm );
uint8_t isp_wrapper_fsm_process_event( isp_wrapper_fsm_ptr_t p_fsm, event_id_t event_id );

void isp_wrapper_fsm_process_interrupt( isp_wrapper_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void isp_wrapper_request_interrupt( isp_wrapper_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __ISP_WRAPPER_FSM_H__ */
