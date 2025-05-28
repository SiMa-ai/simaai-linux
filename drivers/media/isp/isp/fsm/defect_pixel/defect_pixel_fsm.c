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

void defect_pixel_fsm_clear( defect_pixel_fsm_t *p_fsm )
{
}

void defect_pixel_request_interrupt( defect_pixel_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    system_interrupts_disable();
    p_fsm->irq_mask |= mask;
    system_interrupts_enable();
}

void defect_pixel_fsm_switch_state( defect_pixel_fsm_t *p_fsm, defect_pixel_state_t new_state )
{
    defect_pixel_state_t prev_state = p_fsm->state;
    if ( new_state == prev_state ) return;
    p_fsm->state = new_state;
    if ( new_state == defect_pixel_state_initialized ) {
        /* Switching into defect_pixel_state_initialized */
        defect_pixel_init( p_fsm );
    } else if ( new_state == defect_pixel_state_reload_calibration ) {
        /* Switching into defect_pixel_state_reload_calibration */
        defect_pixel_reload_calibration( p_fsm );
    } else if ( new_state == defect_pixel_state_deinit ) {
        /* Switching into defect_pixel_state_deinit */
        defect_pixel_deinit( p_fsm );
    } else if ( new_state == defect_pixel_state_update_hw ) {
        /* Switching into defect_pixel_state_update_hw */
        defect_pixel_update_hw( p_fsm );
    }
}

void defect_pixel_fsm_process_state( defect_pixel_fsm_t *p_fsm )
{
    defect_pixel_state_t state = p_fsm->state;
    for ( ;; ) {
        defect_pixel_state_t prev_state = state;
        switch ( state ) {
        default:
            break;
        case defect_pixel_state_reload_calibration:
            state = defect_pixel_state_ready;
            break;
        case defect_pixel_state_update_hw:
            state = defect_pixel_state_ready;
            break;
        }
        if ( state == prev_state ) {
            break;
        }
        defect_pixel_fsm_switch_state( p_fsm, state );
    }
}

uint8_t defect_pixel_fsm_process_event( defect_pixel_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_fsm_config:
        if ( p_fsm->state == defect_pixel_state_initialized ) {
            defect_pixel_fsm_switch_state( p_fsm, defect_pixel_state_configured );
            b_event_processed = 1;
            break;
        }
        if ( p_fsm->state == defect_pixel_state_stopped ) {
            defect_pixel_fsm_switch_state( p_fsm, defect_pixel_state_configured );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_start:
        if ( p_fsm->state == defect_pixel_state_configured ) {
            defect_pixel_fsm_switch_state( p_fsm, defect_pixel_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_stop:
        if ( p_fsm->state == defect_pixel_state_ready ) {
            defect_pixel_fsm_switch_state( p_fsm, defect_pixel_state_stopped );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_update_hw:
        if ( p_fsm->state == defect_pixel_state_ready ) {
            defect_pixel_fsm_switch_state( p_fsm, defect_pixel_state_update_hw );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_reload_calibration:
        if ( p_fsm->state == defect_pixel_state_ready ) {
            defect_pixel_fsm_switch_state( p_fsm, defect_pixel_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_deinit:
        if ( p_fsm->state == defect_pixel_state_stopped ) {
            defect_pixel_fsm_switch_state( p_fsm, defect_pixel_state_deinit );
            b_event_processed = 1;
            break;
        }
        break;
    }
    if ( b_event_processed ) {
        defect_pixel_fsm_process_state( p_fsm );
    }
    return b_event_processed;
}
