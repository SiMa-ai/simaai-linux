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

#include "system_types.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "acamera_math.h"

#include "sbuf.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_AE_MANUAL


////////////////////////////////////////////////////
// update exposure

static inline uint32_t adjacent_ratio_to_full( uint8_t exp_num, uint32_t ratio )
{
    switch ( exp_num ) {
    case 4:
        return ( ratio * ratio * ratio ) << 6;
        break;
    case 3:
        return ( ratio * ratio ) << 6;
        break;
    default:
    case 2:
        return ratio << 6;
        break;
    }
}

static inline uint32_t full_ratio_to_adjaced( uint8_t exp_num, uint32_t ratio )
{
    switch ( exp_num ) {
    case 4:
        return math_exp2( log2_fixed_to_fixed( ratio, 6, 8 ) / 3, 8, 6 ) >> 6;
        break;
    case 3:
        return sqrt32( ratio >> 6 );
        break;
    default:
    case 2:
        return ratio >> 6;
        break;
    }
}

static void ae_update_exposure( AE_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    acamera_cmd_sensor_info sensor_info = {0};
    WRAP_GENERAL_CMD( p_ictx,
                      CMD_ID_SENSOR_INFO,
                      CMD_DIRECTION_GET,
                      NULL, (uint32_t *)&sensor_info );

    if ( get_context_param( p_ictx, SYSTEM_MANUAL_EXPOSURE_RATIO_PARAM ) ) {
        /* in manual mode read the exposure from context parameters. */
        p_fsm->exposure_ratio = adjacent_ratio_to_full( sensor_info.sensor_exp_number,
                                                        get_context_param( p_ictx,
                                                                           SYSTEM_EXPOSURE_RATIO_PARAM ) );
    } else {
        p_fsm->exposure_ratio = p_fsm->new_exposure_ratio;
        set_context_param( p_ictx,
                           SYSTEM_EXPOSURE_RATIO_PARAM,
                           full_ratio_to_adjaced( sensor_info.sensor_exp_number,
                                                  p_fsm->exposure_ratio ) );
    }

    if ( get_context_param( p_ictx, SYSTEM_MANUAL_EXPOSURE_PARAM ) ) {
        /* in manual mode read the exposure from context parameters. */
        p_fsm->exposure_log2 = log2_fixed_to_fixed(
            get_context_param( p_ictx, SYSTEM_EXPOSURE_PARAM ),
            6,
            LOG2_GAIN_SHIFT );
    } else {
        p_fsm->exposure_log2 = p_fsm->new_exposure_log2;
        set_context_param( p_ictx,
                           SYSTEM_EXPOSURE_PARAM,
                           math_exp2( p_fsm->exposure_log2,
                                      LOG2_GAIN_SHIFT, 6 ) );
    }
}


////////////////////////////////////////////////////
// FSM event handlers

void ae_init( AE_fsm_ptr_t p_fsm )
{
    LOG( LOG_INFO, "Initialize ae_manual FSM" );
}

void ae_config( AE_fsm_ptr_t p_fsm )
{
    LOG( LOG_INFO, "AE Configure" );
}

void ae_reload_calibration( AE_fsm_ptr_t p_fsm )
{
}

void ae_update_algo( AE_fsm_ptr_t p_fsm )
{
    // Update exposure
    ae_update_exposure( p_fsm );

    // Set exposure target to CMOS
    acamera_cmd_exposure_target exp_target;
    exp_target.exposure_log2 = p_fsm->exposure_log2;
    exp_target.exposure_ratio = p_fsm->exposure_ratio;
    WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ), CMD_ID_EXPOSURE_TARGET, CMD_DIRECTION_SET, (uint32_t *)&exp_target, NULL );

    LOG( LOG_DEBUG, "ae exposure_log2: %d, exposure_ratio: %d", exp_target.exposure_log2, exp_target.exposure_ratio );

    // Raise ae_ready event
    fsm_raise_event( p_fsm, event_id_algo_ae_calculation_done );
}
