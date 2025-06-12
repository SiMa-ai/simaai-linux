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

#if !defined( __METADATA_FSM_H__ )
#define __METADATA_FSM_H__

typedef struct _metadata_fsm_t metadata_fsm_t;
typedef struct _metadata_fsm_t *metadata_fsm_ptr_t;
typedef const struct _metadata_fsm_t *metadata_fsm_const_ptr_t;

enum _metadata_state_t {
    metadata_state_initialized,
    metadata_state_configured,
    metadata_state_reload_calibration,
    metadata_state_ready,
    metadata_state_stopped,
    metadata_state_deinit,
    metadata_state_post_metadata,
    metadata_state_update_metadata,
    metadata_state_invalid
};

typedef enum _metadata_state_t metadata_state_t;

#include "metadata_api.h"
void metadata_init( metadata_fsm_ptr_t p_fsm );
void metadata_config( metadata_fsm_ptr_t p_fsm );
void metadata_update_meta( metadata_fsm_ptr_t p_fsm );
void metadata_post_meta( metadata_fsm_const_ptr_t p_fsm );
void metadata_stop( metadata_fsm_ptr_t p_fsm );
void metadata_deinit( metadata_fsm_ptr_t p_fsm );

struct _metadata_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    metadata_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    isp_metadata_t cur_metadata;
};


void metadata_fsm_clear( metadata_fsm_ptr_t p_fsm );
void metadata_fsm_switch_state( metadata_fsm_ptr_t p_fsm, metadata_state_t state );
void metadata_fsm_process_state( metadata_fsm_ptr_t p_fsm );
uint8_t metadata_fsm_process_event( metadata_fsm_ptr_t p_fsm, event_id_t event_id );

void metadata_fsm_process_interrupt( metadata_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void metadata_request_interrupt( metadata_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __METADATA_FSM_H__ */
