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

void cmos_fsm_clear( cmos_fsm_t *p_fsm )
{
    p_fsm->isp_gain = 0;
    p_fsm->manual_gain_mode = 0;
    p_fsm->manual_gain = 1;
    p_fsm->exposure = 256;
    p_fsm->exposure_log2 = 2 << LOG2_GAIN_SHIFT;
    p_fsm->max_exposure_log2 = 2 << LOG2_GAIN_SHIFT;
    p_fsm->exposure_hist_pos = -1;
    p_fsm->flicker_freq = 50 * 256;
    p_fsm->exposure_ratio_in = 64;
    p_fsm->integration_time_short = 0;
    p_fsm->integration_time_medium = 0;
    p_fsm->integration_time_medium2 = 0;
    p_fsm->integration_time_long = 0;
    p_fsm->exposure_ratio = 64;
    p_fsm->log2_gain_avg = 0;
    p_fsm->isp_dgain_log2 = 0;
    p_fsm->target_gain_log2 = 0;
    p_fsm->exp_lut_valid_entries = 0;
    p_fsm->exp_partition_tbl = 0;
    p_fsm->maximum_isp_digital_gain = ( ( ACAMERA_ISP_DIGITAL_GAIN_GAIN_DATASIZE - 8 ) << LOG2_GAIN_SHIFT ) + log2_fixed_to_fixed( ( 1 << ACAMERA_ISP_DIGITAL_GAIN_GAIN_DATASIZE ) - 1, ACAMERA_ISP_DIGITAL_GAIN_GAIN_DATASIZE, LOG2_GAIN_SHIFT );
    p_fsm->cmos_frame_start_irq = ACAMERA_IRQ_FE_FRAME_START;
}

void cmos_request_interrupt( cmos_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask )
{
    system_interrupts_disable();
    p_fsm->irq_mask |= mask;
    system_interrupts_enable();
}

void cmos_fsm_switch_state( cmos_fsm_t *p_fsm, cmos_state_t new_state )
{
    cmos_state_t prev_state = p_fsm->state;
    if ( new_state == prev_state ) return;
    p_fsm->state = new_state;
    if ( new_state == cmos_state_initialized ) {
        /* Switching into cmos_state_initialized */
        cmos_init( p_fsm );
    } else if ( new_state == cmos_state_configured ) {
        /* Switching into cmos_state_configured */
        cmos_config( p_fsm );
    } else if ( new_state == cmos_state_reload_calibration ) {
        /* Switching into cmos_state_reload_calibration */
        cmos_reload_calibration( p_fsm );
    } else if ( new_state == cmos_state_deinit ) {
        /* Switching into cmos_state_deinit */
        cmos_deinit( p_fsm );
    } else if ( new_state == cmos_state_update_exposure ) {
        /* Switching into cmos_state_update_exposure */
        cmos_update_exposure( p_fsm );
        fsm_raise_event( p_fsm, event_id_algo_ae_done );
    } else if ( new_state == cmos_state_update_hw ) {
        /* Switching into cmos_state_update_hw */
        cmos_update_hw( p_fsm );
    }
}

void cmos_fsm_process_state( cmos_fsm_t *p_fsm )
{
    cmos_state_t state = p_fsm->state;
    for ( ;; ) {
        cmos_state_t prev_state = state;
        switch ( state ) {
        default:
            break;
        case cmos_state_reload_calibration:
            state = cmos_state_ready;
            break;
        case cmos_state_update_exposure:
            state = cmos_state_ready;
            break;
        case cmos_state_update_hw:
            state = cmos_state_ready;
            break;
        }
        if ( state == prev_state ) {
            break;
        }
        cmos_fsm_switch_state( p_fsm, state );
    }
}

uint8_t cmos_fsm_process_event( cmos_fsm_t *p_fsm, event_id_t event_id )
{
    uint8_t b_event_processed = 0;
    switch ( event_id ) {
    default:
        break;
    case event_id_fsm_config:
        if ( p_fsm->state == cmos_state_initialized ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_configured );
            b_event_processed = 1;
            break;
        }
        if ( p_fsm->state == cmos_state_stopped ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_configured );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_start:
        if ( p_fsm->state == cmos_state_configured ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_algo_ae_calculation_done:
        if ( p_fsm->state == cmos_state_ready ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_update_exposure );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_update_hw:
        if ( p_fsm->state == cmos_state_ready ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_update_hw );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_stop:
        if ( p_fsm->state == cmos_state_ready ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_stopped );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_reload_calibration:
        if ( p_fsm->state == cmos_state_ready ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_reload_calibration );
            b_event_processed = 1;
            break;
        }
        break;
    case event_id_fsm_deinit:
        if ( p_fsm->state == cmos_state_stopped ) {
            cmos_fsm_switch_state( p_fsm, cmos_state_deinit );
            b_event_processed = 1;
            break;
        }
        break;
    }
    if ( b_event_processed ) {
        cmos_fsm_process_state( p_fsm );
    }
    return b_event_processed;
}
