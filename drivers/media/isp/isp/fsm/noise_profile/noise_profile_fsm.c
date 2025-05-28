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

void noise_profile_fsm_clear( noise_profile_fsm_t *p_fsm )
{
}

void noise_profile_request_interrupt( noise_profile_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    system_interrupts_disable();
    p_fsm->irq_mask |= mask;
    system_interrupts_enable();
}

void noise_profile_fsm_switch_state( noise_profile_fsm_t *p_fsm, noise_profile_state_t new_state )
{
    noise_profile_state_t prev_state = p_fsm->state;
    if ( new_state == prev_state ) return;
    p_fsm->state = new_state;
    if ( new_state == noise_profile_state_initialized ) {
        /* Switching into noise_profile_state_initialized */
        noise_profile_init( p_fsm );
    } else if ( new_state == noise_profile_state_reload_calibration ) {
        /* Switching into noise_profile_state_reload_calibration */
        noise_profile_reload_calibration( p_fsm );
    } else if ( new_state == noise_profile_state_deinit ) {
        /* Switching into noise_profile_state_deinit */
        noise_profile_deinit( p_fsm );
    }
}

void noise_profile_fsm_process_state( noise_profile_fsm_t *p_fsm )
{
    noise_profile_state_t state = p_fsm->state;
    for ( ;; ) {
        noise_profile_state_t prev_state = state;
        switch ( state ) {
        default:
            break;
        case noise_profile_state_reload_calibration:
            state = noise_profile_state_ready;
            break;
        }
        if ( state == prev_state ) {
            break;
        }
        noise_profile_fsm_switch_state( p_fsm, state );
    }
}

uint8_t noise_profile_fsm_process_event( noise_profile_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_fsm_config:
        if ( p_fsm->state == noise_profile_state_initialized ) {
            noise_profile_fsm_switch_state( p_fsm, noise_profile_state_configured );
            b_event_processed = 1;
            break;
        }
        if ( p_fsm->state == noise_profile_state_stopped ) {
            noise_profile_fsm_switch_state( p_fsm, noise_profile_state_configured );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_start:
        if ( p_fsm->state == noise_profile_state_configured ) {
            noise_profile_fsm_switch_state( p_fsm, noise_profile_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_stop:
        if ( p_fsm->state == noise_profile_state_ready ) {
            noise_profile_fsm_switch_state( p_fsm, noise_profile_state_stopped );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_reload_calibration:
        if ( p_fsm->state == noise_profile_state_ready ) {
            noise_profile_fsm_switch_state( p_fsm, noise_profile_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_deinit:
        if ( p_fsm->state == noise_profile_state_stopped ) {
            noise_profile_fsm_switch_state( p_fsm, noise_profile_state_deinit );
            b_event_processed = 1;
            break;
        }
        break;
    }
    if ( b_event_processed ) {
        noise_profile_fsm_process_state( p_fsm );
    }
    return b_event_processed;
}
