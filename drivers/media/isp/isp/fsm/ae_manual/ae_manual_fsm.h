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

#if !defined( __AE_FSM_H__ )
#define __AE_FSM_H__

typedef struct _AE_fsm_t AE_fsm_t;
typedef struct _AE_fsm_t *AE_fsm_ptr_t;
typedef const struct _AE_fsm_t *AE_fsm_const_ptr_t;

enum _AE_state_t {
    AE_state_initialized,
    AE_state_configured,
    AE_state_reload_calibration,
    AE_state_ready,
    AE_state_stopped,
    AE_state_deinit,
    AE_state_update_algo,
    AE_state_invalid
};

typedef enum _AE_state_t AE_state_t;

#include "acamera_isp_core_settings.h"

#define ISP_HAS_AE_FSM 1

void ae_init( AE_fsm_ptr_t p_fsm );
void ae_config( AE_fsm_ptr_t p_fsm );
void ae_reload_calibration( AE_fsm_ptr_t p_fsm );
void ae_update_algo( AE_fsm_ptr_t p_fsm );

struct _AE_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    AE_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    int32_t exposure_log2;
    uint32_t exposure_ratio;
    int32_t new_exposure_log2;
    uint32_t new_exposure_ratio;
};


void AE_fsm_clear( AE_fsm_ptr_t p_fsm );
void AE_fsm_switch_state( AE_fsm_ptr_t p_fsm, AE_state_t state );
void AE_fsm_process_state( AE_fsm_ptr_t p_fsm );
uint8_t AE_fsm_process_event( AE_fsm_ptr_t p_fsm, event_id_t event_id );

void AE_fsm_process_interrupt( AE_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void AE_request_interrupt( AE_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __AE_FSM_H__ */
