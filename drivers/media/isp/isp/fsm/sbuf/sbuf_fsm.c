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

void sbuf_fsm_clear( sbuf_fsm_t *p_fsm )
{
    p_fsm->opened = 0;
    p_fsm->mode = 0;
    p_fsm->is_paused = 1;
}

void sbuf_request_interrupt( sbuf_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    system_interrupts_disable();
    p_fsm->irq_mask |= mask;
    system_interrupts_enable();
}

void sbuf_fsm_switch_state( sbuf_fsm_t *p_fsm, sbuf_state_t new_state )
{
    sbuf_state_t prev_state = p_fsm->state;
    if ( new_state == prev_state ) return;
    p_fsm->state = new_state;
    if ( new_state == sbuf_state_initialized ) {
        /* Switching into sbuf_state_initialized */
        sbuf_fsm_init( p_fsm );
    } else if ( new_state == sbuf_state_configured ) {
        /* Switching into sbuf_state_configured */
        sbuf_config( p_fsm );
    } else if ( new_state == sbuf_state_stopped ) {
        /* Switching into sbuf_state_stopped */
        sbuf_stop( p_fsm );
    } else if ( new_state == sbuf_state_deinit ) {
        /* Switching into sbuf_state_deinit */
        sbuf_deinit( p_fsm );
    } else if ( new_state == sbuf_state_update_ae_idx ) {
        /* Switching into sbuf_state_update_ae_idx */
        sbuf_update_ae_idx( p_fsm );
    } else if ( new_state == sbuf_state_update_awb_idx ) {
        /* Switching into sbuf_state_update_awb_idx */
        sbuf_update_awb_idx( p_fsm );
    } else if ( new_state == sbuf_state_update_af_idx ) {
        /* Switching into sbuf_state_update_af_idx */
        sbuf_update_af_idx( p_fsm );
    } else if ( new_state == sbuf_state_update_gamma_idx ) {
        /* Switching into sbuf_state_update_gamma_idx */
        sbuf_update_gamma_idx( p_fsm );
    }
}

void sbuf_fsm_process_state( sbuf_fsm_t *p_fsm )
{
    sbuf_state_t state = p_fsm->state;
    for ( ;; ) {
        sbuf_state_t prev_state = state;
        switch ( state ) {
        default:
            break;
        case sbuf_state_reload_calibration:
            state = sbuf_state_ready;
            break;
        case sbuf_state_update_ae_idx:
            state = sbuf_state_ready;
            break;
        case sbuf_state_update_af_idx:
            state = sbuf_state_ready;
            break;
        case sbuf_state_update_awb_idx:
            state = sbuf_state_ready;
            break;
        case sbuf_state_update_gamma_idx:
            state = sbuf_state_ready;
            break;
        }
        if ( state == prev_state ) {
            break;
        }
        sbuf_fsm_switch_state( p_fsm, state );
    }
}

uint8_t sbuf_fsm_process_event( sbuf_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_fsm_config:
        if ( p_fsm->state == sbuf_state_initialized ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_configured );
            b_event_processed = 1;
            break;
        }
        if ( p_fsm->state == sbuf_state_stopped ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_configured );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_start:
        if ( p_fsm->state == sbuf_state_configured ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_isphw_stats_ready_ae:
        if ( p_fsm->state == sbuf_state_ready ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_update_ae_idx );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_isphw_stats_ready_awb:
        if ( p_fsm->state == sbuf_state_ready ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_update_awb_idx );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_isphw_stats_ready_af:
        if ( p_fsm->state == sbuf_state_ready ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_update_af_idx );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_isphw_stats_ready_af_gamma:
        if ( p_fsm->state == sbuf_state_ready ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_update_gamma_idx );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_stop:
        if ( p_fsm->state == sbuf_state_ready ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_stopped );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_reload_calibration:
        if ( p_fsm->state == sbuf_state_ready ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_deinit:
        if ( p_fsm->state == sbuf_state_stopped ) {
            sbuf_fsm_switch_state( p_fsm, sbuf_state_deinit );
            b_event_processed = 1;
            break;
        }
        break;
    }
    if ( b_event_processed ) {
        sbuf_fsm_process_state( p_fsm );
    }
    return b_event_processed;
}
