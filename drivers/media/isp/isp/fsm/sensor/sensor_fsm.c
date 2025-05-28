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

void sensor_fsm_clear( sensor_fsm_t *p_fsm )
{
    p_fsm->sensor_pos = ( 0xff );
    p_fsm->drv_priv = NULL;
    p_fsm->s_param = NULL;
}

void sensor_request_interrupt( sensor_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    system_interrupts_disable();
    p_fsm->irq_mask |= mask;
    system_interrupts_enable();
}

void sensor_fsm_switch_state( sensor_fsm_t *p_fsm, sensor_state_t new_state )
{
    sensor_state_t prev_state = p_fsm->state;
    if ( new_state == prev_state ) return;
    p_fsm->state = new_state;
    if ( new_state == sensor_state_initialized ) {
        /* Switching into sensor_state_initialized */
        sensor_init( p_fsm );
    } else if ( new_state == sensor_state_configured ) {
        /* Switching into sensor_state_configured */
        sensor_config( p_fsm );
    } else if ( new_state == sensor_state_ready ) {
        /* Switching into sensor_state_ready */
        sensor_ready( p_fsm );
    } else if ( new_state == sensor_state_stopped ) {
        /* Switching into sensor_state_stopped */
        sensor_stopped( p_fsm );
    } else if ( new_state == sensor_state_deinit ) {
        /* Switching into sensor_state_deinit */
        sensor_deinit( p_fsm );
    } else if ( new_state == sensor_state_update_hw ) {
        /* Switching into sensor_state_update_hw */
        sensor_update_hw( p_fsm );
    } else if ( new_state == sensor_state_request_next_frame ) {
        /* Switching into sensor_state_request_next_frame */
        sensor_request_next_frame( p_fsm );
    }
}

void sensor_fsm_process_state( sensor_fsm_t *p_fsm )
{
    sensor_state_t state = p_fsm->state;
    for ( ;; ) {
        sensor_state_t prev_state = state;
        switch ( state ) {
        default:
            break;
        case sensor_state_reload_calibration:
            state = sensor_state_ready;
            break;
        case sensor_state_request_next_frame:
            state = sensor_state_ready;
            break;
        case sensor_state_update_hw:
            state = sensor_state_ready;
            break;
        }
        if ( state == prev_state ) {
            break;
        }
        sensor_fsm_switch_state( p_fsm, state );
    }
}

uint8_t sensor_fsm_process_event( sensor_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_fsm_config:
        if ( p_fsm->state == sensor_state_initialized ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_configured );
            b_event_processed = 1;
            break;
        }
        if ( p_fsm->state == sensor_state_stopped ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_configured );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_start:
        if ( p_fsm->state == sensor_state_configured ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_update_hw:
        if ( p_fsm->state == sensor_state_ready ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_update_hw );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_stop:
        if ( p_fsm->state == sensor_state_ready ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_stopped );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_reload_calibration:
        if ( p_fsm->state == sensor_state_ready ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_isphw_frame_end:
        if ( p_fsm->state == sensor_state_ready ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_request_next_frame );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_deinit:
        if ( p_fsm->state == sensor_state_stopped ) {
            sensor_fsm_switch_state( p_fsm, sensor_state_deinit );
            b_event_processed = 1;
            break;
        }
        break;
    }
    if ( b_event_processed ) {
        sensor_fsm_process_state( p_fsm );
    }
    return b_event_processed;
}
