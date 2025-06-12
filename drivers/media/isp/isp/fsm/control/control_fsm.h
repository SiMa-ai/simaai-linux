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

#if !defined( __CONTROL_FSM_H__ )
#define __CONTROL_FSM_H__

typedef struct _control_fsm_t control_fsm_t;
typedef struct _control_fsm_t *control_fsm_ptr_t;
typedef const struct _control_fsm_t *control_fsm_const_ptr_t;

enum _control_state_t {
    control_state_initialized,
    control_state_configured,
    control_state_reload_calibration,
    control_state_ready,
    control_state_stopped,
    control_state_deinit,
    control_state_update_hw,
    control_state_invalid
};

typedef enum _control_state_t control_state_t;

#include "acamera_configuration.h"
#include "acamera_sbus_api.h"

void control_init( control_fsm_ptr_t p_fsm );
void control_config( control_fsm_ptr_t p_fsm );
void control_update_hw( control_fsm_ptr_t p_fsm );

struct _control_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    control_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    acamera_sbus_t isp_sbus;
    uint8_t antifog_mode;
    uint8_t antifog_enabled;
    uint32_t tpg_enable;
    uint32_t tpg_mode;
};


void control_fsm_clear( control_fsm_ptr_t p_fsm );
void control_fsm_switch_state( control_fsm_ptr_t p_fsm, control_state_t state );
void control_fsm_process_state( control_fsm_ptr_t p_fsm );
uint8_t control_fsm_process_event( control_fsm_ptr_t p_fsm, event_id_t event_id );

void control_fsm_process_interrupt( control_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void control_request_interrupt( control_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __CONTROL_FSM_H__ */
