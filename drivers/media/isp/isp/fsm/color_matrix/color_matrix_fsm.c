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

#include "system_interrupts.h"
#include "acamera_isp_ctx.h"

void color_matrix_fsm_clear( color_matrix_fsm_t *p_fsm )
{
    p_fsm->light_source = AWB_LIGHT_SOURCE_D50;
    p_fsm->light_source_previous = AWB_LIGHT_SOURCE_D50;
    p_fsm->light_source_ccm = AWB_LIGHT_SOURCE_D50;
    p_fsm->light_source_ccm_previous = AWB_LIGHT_SOURCE_D50;
    p_fsm->light_source_change_frames = 20;
    p_fsm->light_source_change_frames_left = 0;
    p_fsm->manual_CCM = 0;
}

void color_matrix_request_interrupt( color_matrix_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    system_interrupts_disable();
    p_fsm->irq_mask |= mask;
    system_interrupts_enable();
}

void color_matrix_fsm_switch_state( color_matrix_fsm_t *p_fsm, color_matrix_state_t new_state )
{
    color_matrix_state_t prev_state = p_fsm->state;
    if ( new_state == prev_state ) return;
    p_fsm->state = new_state;
    if ( new_state == color_matrix_state_initialized ) {
        /* Switching into color_matrix_state_initialized */
        color_matrix_init( p_fsm );
    } else if ( new_state == color_matrix_state_configured ) {
        /* Switching into color_matrix_state_configured */
        color_matrix_config( p_fsm );
    } else if ( new_state == color_matrix_state_update_hw ) {
        /* Switching into color_matrix_state_update_hw */
        color_matrix_update_hw( p_fsm );
    }
}

void color_matrix_fsm_process_state( color_matrix_fsm_t *p_fsm )
{
    color_matrix_state_t state = p_fsm->state;
    for ( ;; ) {
        color_matrix_state_t prev_state = state;
        switch ( state ) {
        default:
            break;
        case color_matrix_state_reload_calibration:
            state = color_matrix_state_ready;
            break;
        case color_matrix_state_update_hw:
            state = color_matrix_state_ready;
            break;
        }
        if ( state == prev_state ) {
            break;
        }
        color_matrix_fsm_switch_state( p_fsm, state );
    }
}

uint8_t color_matrix_fsm_process_event( color_matrix_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_fsm_config:
        if ( p_fsm->state == color_matrix_state_initialized ) {
            color_matrix_fsm_switch_state( p_fsm, color_matrix_state_configured );
            b_event_processed = 1;
            break;
        }
        if ( p_fsm->state == color_matrix_state_stopped ) {
            color_matrix_fsm_switch_state( p_fsm, color_matrix_state_configured );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_start:
        if ( p_fsm->state == color_matrix_state_configured ) {
            color_matrix_fsm_switch_state( p_fsm, color_matrix_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_update_hw:
        if ( p_fsm->state == color_matrix_state_ready ) {
            color_matrix_fsm_switch_state( p_fsm, color_matrix_state_update_hw );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_stop:
        if ( p_fsm->state == color_matrix_state_ready ) {
            color_matrix_fsm_switch_state( p_fsm, color_matrix_state_stopped );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_reload_calibration:
        if ( p_fsm->state == color_matrix_state_ready ) {
            color_matrix_fsm_switch_state( p_fsm, color_matrix_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_deinit:
        if ( p_fsm->state == color_matrix_state_stopped ) {
            color_matrix_fsm_switch_state( p_fsm, color_matrix_state_deinit );
            b_event_processed = 1;
            break;
        }
        break;
    }
    if ( b_event_processed ) {
        color_matrix_fsm_process_state( p_fsm );
    }
    return b_event_processed;
}
