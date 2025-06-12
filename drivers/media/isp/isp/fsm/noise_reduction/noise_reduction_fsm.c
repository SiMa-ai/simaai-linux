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

void noise_reduction_fsm_clear( noise_reduction_fsm_t *p_fsm )
{
    p_fsm->temper_ev_previous_frame = 0;
    p_fsm->temper_diff_avg = 0;
    p_fsm->temper_diff_coeff = 10;
}

void noise_reduction_request_interrupt( noise_reduction_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    system_interrupts_disable();
    p_fsm->irq_mask |= mask;
    system_interrupts_enable();
}

void noise_reduction_fsm_switch_state( noise_reduction_fsm_t *p_fsm, noise_reduction_state_t new_state )
{
    noise_reduction_state_t prev_state = p_fsm->state;
    if ( new_state == prev_state ) return;
    p_fsm->state = new_state;
    if ( new_state == noise_reduction_state_configured ) {
        /* Switching into noise_reduction_state_configured */
        noise_reduction_config( p_fsm );
    } else if ( new_state == noise_reduction_state_reload_calibration ) {
        /* Switching into noise_reduction_state_reload_calibration */
        noise_reduction_reload_calibration( p_fsm );
    } else if ( new_state == noise_reduction_state_deinit ) {
        /* Switching into noise_reduction_state_deinit */
        noise_reduction_deinit( p_fsm );
    } else if ( new_state == noise_reduction_state_update_hw ) {
        /* Switching into noise_reduction_state_update_hw */
        noise_reduction_update_hw( p_fsm );
    }
}

void noise_reduction_fsm_process_state( noise_reduction_fsm_t *p_fsm )
{
    noise_reduction_state_t state = p_fsm->state;
    for ( ;; ) {
        noise_reduction_state_t prev_state = state;
        switch ( state ) {
        default:
            break;
        case noise_reduction_state_reload_calibration:
            state = noise_reduction_state_ready;
            break;
        case noise_reduction_state_update_hw:
            state = noise_reduction_state_ready;
            break;
        }
        if ( state == prev_state ) {
            break;
        }
        noise_reduction_fsm_switch_state( p_fsm, state );
    }
}

uint8_t noise_reduction_fsm_process_event( noise_reduction_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_fsm_config:
        if ( p_fsm->state == noise_reduction_state_initialized ) {
            noise_reduction_fsm_switch_state( p_fsm, noise_reduction_state_configured );
            b_event_processed = 1;
            break;
        }
        if ( p_fsm->state == noise_reduction_state_stopped ) {
            noise_reduction_fsm_switch_state( p_fsm, noise_reduction_state_configured );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_start:
        if ( p_fsm->state == noise_reduction_state_configured ) {
            noise_reduction_fsm_switch_state( p_fsm, noise_reduction_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_update_hw:
        if ( p_fsm->state == noise_reduction_state_ready ) {
            noise_reduction_fsm_switch_state( p_fsm, noise_reduction_state_update_hw );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_stop:
        if ( p_fsm->state == noise_reduction_state_ready ) {
            noise_reduction_fsm_switch_state( p_fsm, noise_reduction_state_stopped );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_reload_calibration:
        if ( p_fsm->state == noise_reduction_state_ready ) {
            noise_reduction_fsm_switch_state( p_fsm, noise_reduction_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_deinit:
        if ( p_fsm->state == noise_reduction_state_stopped ) {
            noise_reduction_fsm_switch_state( p_fsm, noise_reduction_state_deinit );
            b_event_processed = 1;
            break;
        }
        break;
    }
    if ( b_event_processed ) {
        noise_reduction_fsm_process_state( p_fsm );
    }
    return b_event_processed;
}
