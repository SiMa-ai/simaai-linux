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

static void noise_profile_lut_reload( noise_profile_fsm_ptr_t p_fsm )
{
#if defined( ACAMERA_ISP_NOISE_PROFILE_LUT_WEIGHT_LUT_DEFAULT ) || defined( ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_LUT_WEIGHT_LUT_DEFAULT ) || defined( ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_1_LUT_WEIGHT_LUT_DEFAULT )
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t isp_base = p_ictx->settings.isp_base;
    acamera_calib_mgr_entry_t *cm_ptr = ACAMERA_FSM2CM_PTR( p_fsm );

    const uint8_t *np_lin = NULL;
    const uint8_t *np_lut = calib_mgr_u8_lut_get( cm_ptr, CALIBRATION_NOISE_PROFILE );

#if defined( CALIBRATION_WDR_NP_LUT ) && defined( ACAMERA_ISP_NOISE_PROFILE_WDR_VS_WEIGHT_LUT_DEFAULT ) && defined( ACAMERA_ISP_NOISE_PROFILE_WDR_S_WEIGHT_LUT_DEFAULT )
    const uint32_t wdr_mode = get_context_param( p_ictx, SENSOR_WDR_MODE_PARAM );

    if ( ( wdr_mode != WDR_MODE_LINEAR ) && ( wdr_mode != WDR_MODE_NATIVE ) ) {
        np_lin = calib_mgr_u8_lut_get( cm_ptr, CALIBRATION_WDR_NP_LUT );
    }
#endif

    int32_t i;

    for ( i = 0; i < calib_mgr_lut_len( cm_ptr, CALIBRATION_NOISE_PROFILE ); i++ ) {

#ifdef ACAMERA_ISP_NOISE_PROFILE_LUT_WEIGHT_LUT_DEFAULT
        acamera_isp_noise_profile_lut_weight_lut_write( isp_base, i, np_lut[i] );
#endif

#ifdef ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_LUT_WEIGHT_LUT_DEFAULT
        acamera_isp_noise_profile_raw_frontend_lut_weight_lut_write( isp_base, i, np_lut[i] );
#endif

#ifdef ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_1_LUT_WEIGHT_LUT_DEFAULT
        acamera_isp_noise_profile_raw_frontend_1_lut_weight_lut_write( isp_base, i, np_lut[i] );
#endif

#ifdef ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_2_LUT_WEIGHT_LUT_DEFAULT
        acamera_isp_noise_profile_raw_frontend_2_lut_weight_lut_write( isp_base, i, np_lut[i] );
#endif

#ifdef ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_3_LUT_WEIGHT_LUT_DEFAULT
        acamera_isp_noise_profile_raw_frontend_3_lut_weight_lut_write( isp_base, i, np_lut[i] );
#endif

#ifdef ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_4_LUT_WEIGHT_LUT_DEFAULT
        acamera_isp_noise_profile_raw_frontend_4_lut_weight_lut_write( isp_base, i, np_lut[i] );
#endif
        if ( np_lin != NULL ) {
            acamera_isp_noise_profile_wdr_vs_weight_lut_write( isp_base, i, np_lin[i] );
            acamera_isp_noise_profile_wdr_s_weight_lut_write( isp_base, i, np_lin[i] );
            acamera_isp_noise_profile_wdr_m_weight_lut_write( isp_base, i, np_lin[i] );
            acamera_isp_noise_profile_wdr_l_weight_lut_write( isp_base, i, np_lin[i] );
#ifdef ACAMERA_ISP_NOISE_PROFILE_WDR_WEIGHT_LUT_DEFAULT
            acamera_isp_noise_profile_wdr_weight_lut_write( isp_base, i, np_lin[i] );
#endif
        }
    }
#endif /* ACAMERA_ISP_NOISE_PROFILE_LUT_WEIGHT_LUT_DEFAULT || ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_LUT_WEIGHT_LUT_DEFAULT || ACAMERA_ISP_NOISE_PROFILE_RAW_FRONTEND_1_LUT_WEIGHT_LUT_DEFAULT */
}

void noise_profile_init( noise_profile_fsm_ptr_t p_fsm )
{
}

void noise_profile_reload_calibration( noise_profile_fsm_ptr_t p_fsm )
{
    noise_profile_lut_reload( p_fsm );
}

void noise_profile_deinit( noise_profile_fsm_ptr_t p_fsm )
{
}