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

#include "acamera_command_api.h"
#include "acamera_isp_ctx.h"
#include "acamera_math.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_SHARPENING

void sharpening_update_hw( sharpening_fsm_t *p_fsm )
{
    uint16_t alt_d, alt_ud;
    uint32_t sharp_alt_d_idx = CALIBRATION_SHARP_ALT_D;
    uint32_t sharp_alt_ud_idx = CALIBRATION_SHARP_ALT_UD;
    uint16_t alt_du = 0;
    uint32_t sharp_alt_du_idx;

    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
#if ( ISP_RTL_VERSION_R == 2 )
    uint32_t raw_scaler_enable = get_context_param( p_ictx, IMAGE_RAW_SCALER_ENABLE_ID_PARAM );
    sharp_alt_d_idx = raw_scaler_enable ? CALIBRATION_SHARP_ALT_D_WRS : CALIBRATION_SHARP_ALT_D;
    sharp_alt_ud_idx = raw_scaler_enable ? CALIBRATION_SHARP_ALT_UD_WRS : CALIBRATION_SHARP_ALT_UD;
    sharp_alt_du_idx = raw_scaler_enable ? CALIBRATION_SHARP_ALT_DU_WRS : CALIBRATION_SHARP_ALT_DU;
#else
    sharp_alt_d_idx = CALIBRATION_SHARP_ALT_D;
    sharp_alt_ud_idx = CALIBRATION_SHARP_ALT_UD;
    sharp_alt_du_idx = CALIBRATION_SHARP_ALT_DU;
#endif
    const modulation_entry_t *sharp_alt_du_table_ptr = calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sharp_alt_du_idx );
    const modulation_entry_t *sharp_alt_d_table_ptr;
    const modulation_entry_t *sharp_alt_ud_table_ptr;

    const uint32_t ldr_gain_log2 = get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM );

    sharp_alt_d_table_ptr = calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sharp_alt_d_idx );
    sharp_alt_ud_table_ptr = calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sharp_alt_ud_idx );

    const uint32_t system_manual_directional_sharpening = get_context_param( p_ictx, SYSTEM_MANUAL_DIRECTIONAL_SHARPENING_PARAM );
    const uint32_t system_manual_un_directional_sharpening = get_context_param( p_ictx, SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING_PARAM );

    if ( system_manual_directional_sharpening != 0 ) {
        // Do not update values if manual mode
        alt_d = get_context_param( p_ictx, SYSTEM_DIRECTIONAL_SHARPENING_TARGET_PARAM );
    } else {
        alt_d = calc_modulation_u16( ldr_gain_log2,
                                     sharp_alt_d_table_ptr,
                                     calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ),
                                                         sharp_alt_d_idx ) );

        alt_du = calc_modulation_u16( ldr_gain_log2,
                                      sharp_alt_du_table_ptr,
                                      calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ),
                                                          sharp_alt_du_idx ) );

        alt_d = ( alt_d * p_fsm->sharpening_mult ) / 128;
        if ( alt_d >= ( 1 << ACAMERA_ISP_DEMOSAIC_SHARP_ALT_D_DATASIZE ) ) {
            alt_d = ( 1 << ACAMERA_ISP_DEMOSAIC_SHARP_ALT_D_DATASIZE ) - 1;
        }

        set_context_param( p_ictx, SYSTEM_DIRECTIONAL_SHARPENING_TARGET_PARAM, alt_d );
    }

    if ( system_manual_un_directional_sharpening != 0 ) {
        //  Do not update values if manual mode
        alt_ud = get_context_param( p_ictx, SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET_PARAM );
    } else {
        alt_ud = calc_modulation_u16( ldr_gain_log2,
                                      sharp_alt_ud_table_ptr,
                                      calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ),
                                                          sharp_alt_ud_idx ) );

        alt_ud = ( alt_ud * p_fsm->sharpening_mult ) / 128;
        if ( alt_ud >= ( 1 << ACAMERA_ISP_DEMOSAIC_SHARP_ALT_UD_DATASIZE ) ) {
            alt_ud = ( 1 << ACAMERA_ISP_DEMOSAIC_SHARP_ALT_UD_DATASIZE ) - 1;
        }

        set_context_param( p_ictx, SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET_PARAM, alt_ud );
    }

    if ( get_context_param( p_ictx, ISP_MODULES_MANUAL_DEMOSAIC_PARAM ) == 0 ) {
        if ( system_manual_directional_sharpening == 0 ) {
            //  Do not update values if manual mode

            acamera_isp_demosaic_sharp_alt_ld_write( p_ictx->settings.isp_base, alt_d );
            acamera_isp_demosaic_sharp_alt_ldu_write( p_ictx->settings.isp_base, alt_du );
        }
        if ( system_manual_un_directional_sharpening == 0 ) {
            acamera_isp_demosaic_sharp_alt_lu_write( p_ictx->settings.isp_base, alt_ud );
        } //  Do not update values if manual mode
    }
}
