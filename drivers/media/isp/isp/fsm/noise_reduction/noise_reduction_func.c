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
#include "acamera_configuration.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "acamera_math.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_NOISE_REDUCTION

#define ISP_TOTAL_SINTER_CONTROL_IN_MANUAL_MODE 0

static void stitching_error_calculate( noise_reduction_fsm_t *p_fsm )
{
    /*
-------------------------------------------------------
    Modulation for new frame stitching ISP5-S-2.0
-------------------------------------------------------
*/

    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    acamera_calib_mgr_entry_t *p_cmgr = ACAMERA_FSM2CM_PTR( p_fsm );

    if ( get_context_param( p_ictx, ISP_MODULES_MANUAL_FRAME_STITCH_PARAM ) ) {
        return;
    }

    const uint32_t ldr_gain_log2 = get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM );

#ifdef ACAMERA_ISP_FRAME_STITCH_MCOFF_MODE_ENABLE_DEFAULT
    // Change to MC off mode when gain is higher than gain_log2 value found in calibration.
    uint16_t MC_off_enable_gain = calib_mgr_u32_lut_get( p_cmgr, CALIBRATION_FS_MC_OFF )[0];

    if ( ldr_gain_log2 > MC_off_enable_gain ) {
        acamera_isp_frame_stitch_mcoff_mode_enable_write( p_ictx->settings.isp_base, 1 );
    } else if ( ldr_gain_log2 <= MC_off_enable_gain ) {
        acamera_isp_frame_stitch_mcoff_mode_enable_write( p_ictx->settings.isp_base, 0 );
    }
#endif

    switch ( get_context_param( p_ictx, SENSOR_WDR_MODE_PARAM ) ) {
    case WDR_MODE_FS_HDR: // fall through
    case WDR_MODE_FS_LIN: {

#ifdef ACAMERA_ISP_FRAME_STITCH_LM_NP_MULT_DEFAULT
        const uint16_t lm_np = calc_modulation_u16( ldr_gain_log2, (modulation_entry_t *)calib_mgr_mod16_lut_get( p_cmgr, CALIBRATION_STITCHING_LM_NP ), calib_mgr_lut_rows( p_cmgr, CALIBRATION_STITCHING_LM_NP ) );
        acamera_isp_frame_stitch_lm_np_mult_write( p_ictx->settings.isp_base, lm_np );
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_LM_ALPHA_MOV_SLOPE_DEFAULT
        const uint16_t lm_mov_mult = calc_modulation_u16( ldr_gain_log2, (modulation_entry_t *)calib_mgr_mod16_lut_get( p_cmgr, CALIBRATION_STITCHING_LM_MOV_MULT ), calib_mgr_lut_rows( p_cmgr, CALIBRATION_STITCHING_LM_MOV_MULT ) );
        acamera_isp_frame_stitch_lm_alpha_mov_slope_write( p_ictx->settings.isp_base, lm_mov_mult );
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_MS_NP_MULT_DEFAULT
        const uint16_t ms_np = calc_modulation_u16( ldr_gain_log2, (modulation_entry_t *)calib_mgr_mod16_lut_get( p_cmgr, CALIBRATION_STITCHING_MS_NP ), calib_mgr_lut_rows( p_cmgr, CALIBRATION_STITCHING_MS_NP ) );
        acamera_isp_frame_stitch_ms_np_mult_write( p_ictx->settings.isp_base, ms_np );
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_MS_ALPHA_MOV_SLOPE_DEFAULT
        const uint16_t ms_mov_mult = calc_modulation_u16( ldr_gain_log2, (modulation_entry_t *)calib_mgr_mod16_lut_get( p_cmgr, CALIBRATION_STITCHING_MS_MOV_MULT ), calib_mgr_lut_rows( p_cmgr, CALIBRATION_STITCHING_MS_MOV_MULT ) );
        acamera_isp_frame_stitch_ms_alpha_mov_slope_write( p_ictx->settings.isp_base, ms_mov_mult );
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_SVS_NP_MULT_DEFAULT
        if ( calib_mgr_lut_exists( p_cmgr, CALIBRATION_STITCHING_SVS_NP ) ) {
            const uint16_t svs_np = calc_modulation_u16( ldr_gain_log2, (modulation_entry_t *)calib_mgr_mod16_lut_get( p_cmgr, CALIBRATION_STITCHING_SVS_NP ), calib_mgr_lut_rows( p_cmgr, CALIBRATION_STITCHING_SVS_NP ) );
            acamera_isp_frame_stitch_svs_np_mult_write( p_ictx->settings.isp_base, svs_np );
        }
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_SVS_ALPHA_MOV_SLOPE_DEFAULT
        if ( calib_mgr_lut_exists( p_cmgr, CALIBRATION_STITCHING_SVS_MOV_MULT ) ) {
            const uint16_t svs_mov_mult = calc_modulation_u16( ldr_gain_log2, (modulation_entry_t *)calib_mgr_mod16_lut_get( p_cmgr, CALIBRATION_STITCHING_SVS_MOV_MULT ), calib_mgr_lut_rows( p_cmgr, CALIBRATION_STITCHING_SVS_MOV_MULT ) );
            acamera_isp_frame_stitch_svs_alpha_mov_slope_write( p_ictx->settings.isp_base, svs_mov_mult );
        }
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_LM_MED_NOISE_INTENSITY_THRESH_DEFAULT
        const uint16_t stitching_lm_med_noise_intensity_thresh = calc_modulation_u16( ldr_gain_log2, (modulation_entry_t *)calib_mgr_mod16_lut_get( p_cmgr, CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY ), calib_mgr_lut_rows( p_cmgr, CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY ) );
        acamera_isp_frame_stitch_lm_med_noise_intensity_thresh_write( p_ictx->settings.isp_base, stitching_lm_med_noise_intensity_thresh );
#endif
        break;
    }
    default:
        break;
    }
}

static void sinter_strength_calculate( noise_reduction_fsm_t *p_fsm )
{
    uint32_t cmos_exp_ratio = 0;
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    WRAP_GENERAL_CMD( p_ictx, CMD_ID_CMOS_EXPOSURE_RATIO, CMD_DIRECTION_GET, NULL, &cmos_exp_ratio );
    uint32_t snr_thresh_master;

    if ( get_context_param( p_ictx, ISP_MODULES_MANUAL_SINTER_PARAM ) == 0 ) {
        // Get lut indexes for the current HDR mode.
        uint32_t sinter_strength_mc_contrast_idx = CALIBRATION_SINTER_STRENGTH_MC_CONTRAST;
        uint32_t sinter_strength_idx;
        uint32_t sinter_strength1_idx;
        uint32_t sinter_strength4_idx;
        uint32_t sinter_thresh1_idx;
        uint32_t sinter_thresh4_idx;
        uint32_t sinter_int_config_idx;

#if ( ISP_RTL_VERSION_R == 2 )
        uint32_t raw_scaler_enable = get_context_param( p_ictx, IMAGE_RAW_SCALER_ENABLE_ID_PARAM );
        sinter_strength_mc_contrast_idx = raw_scaler_enable ? CALIBRATION_SINTER_STRENGTH_MC_CONTRAST_WRS : CALIBRATION_SINTER_STRENGTH_MC_CONTRAST;
        sinter_strength_idx = raw_scaler_enable ? CALIBRATION_SINTER_STRENGTH_WRS : CALIBRATION_SINTER_STRENGTH;
        sinter_strength1_idx = raw_scaler_enable ? CALIBRATION_SINTER_STRENGTH1_WRS : CALIBRATION_SINTER_STRENGTH1;
        sinter_strength4_idx = raw_scaler_enable ? CALIBRATION_SINTER_STRENGTH4_WRS : CALIBRATION_SINTER_STRENGTH4;
        sinter_thresh1_idx = raw_scaler_enable ? CALIBRATION_SINTER_THRESH1_WRS : CALIBRATION_SINTER_THRESH1;
        sinter_thresh4_idx = raw_scaler_enable ? CALIBRATION_SINTER_THRESH4_WRS : CALIBRATION_SINTER_THRESH4;
        sinter_int_config_idx = raw_scaler_enable ? CALIBRATION_SINTER_INTCONFIG_WRS : CALIBRATION_SINTER_INTCONFIG;
#else
        sinter_strength_mc_contrast_idx = CALIBRATION_SINTER_STRENGTH_MC_CONTRAST;
        sinter_strength_idx = CALIBRATION_SINTER_STRENGTH;
        sinter_strength1_idx = CALIBRATION_SINTER_STRENGTH1;
        sinter_strength4_idx = CALIBRATION_SINTER_STRENGTH4;
        sinter_thresh1_idx = CALIBRATION_SINTER_THRESH1;
        sinter_thresh4_idx = CALIBRATION_SINTER_THRESH4;
        sinter_int_config_idx = CALIBRATION_SINTER_INTCONFIG;
#endif

        const uint32_t ldr_gain_log2 = get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM );

        snr_thresh_master = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength_idx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength_idx ) );
        // Adjust strength according to contrast.
        uint32_t iridix_contrast = ( ACAMERA_FSM2FSMGR_PTR( p_fsm )->iridix_fsm.iridix_contrast ); // U8.8
        iridix_contrast = iridix_contrast >> 8;
        uint32_t snr_thresh_master_contrast = calc_modulation_u16( iridix_contrast, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength_mc_contrast_idx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength_mc_contrast_idx ) );
        if ( snr_thresh_master_contrast > 0xFF )
            snr_thresh_master_contrast = 0xFF;
        // It will only affect short exposure.
        p_fsm->snr_thresh_contrast = snr_thresh_master_contrast;

        if ( snr_thresh_master > 0xFF )
            snr_thresh_master = 0xFF;

        set_context_param( p_ictx, SYSTEM_SINTER_THRESHOLD_TARGET_PARAM, snr_thresh_master );
        uint16_t sinter_strenght1 = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength1_idx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength1_idx ) );
        acamera_isp_sinter_strength_1_write( p_ictx->settings.isp_base, sinter_strenght1 );
        uint16_t sinter_strenght4 = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength4_idx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_strength4_idx ) );
        acamera_isp_sinter_strength_4_write( p_ictx->settings.isp_base, sinter_strenght4 );
        uint16_t sinter_thresh1 = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_thresh1_idx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_thresh1_idx ) );
        acamera_isp_sinter_thresh_1h_write( p_ictx->settings.isp_base, sinter_thresh1 );
        acamera_isp_sinter_thresh_1v_write( p_ictx->settings.isp_base, sinter_thresh1 );
        uint16_t sinter_thresh4 = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_thresh4_idx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_thresh4_idx ) );
        acamera_isp_sinter_thresh_4h_write( p_ictx->settings.isp_base, sinter_thresh4 );
        acamera_isp_sinter_thresh_4v_write( p_ictx->settings.isp_base, sinter_thresh4 );
        uint16_t sinter_int_config = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_int_config_idx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), sinter_int_config_idx ) );
        acamera_isp_sinter_int_config_write( p_ictx->settings.isp_base, sinter_int_config );
    } else {
        snr_thresh_master = get_context_param( p_ictx, SYSTEM_SINTER_THRESHOLD_TARGET_PARAM );
    }

    p_fsm->snr_thresh_master = snr_thresh_master;
}

void noise_reduction_config( noise_reduction_fsm_t *p_fsm )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    p_fsm->tnr_strength_target = 15;
    p_fsm->snr_strength_target = 15;
    p_fsm->tnr_thresh_master = p_fsm->tnr_strength_target; // Same as target.
    p_fsm->snr_thresh_master = p_fsm->snr_strength_target; // Same as target.
    const uint16_t *p_sinter_params = (const uint16_t *)calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_SINTER_PARAMS );
    /* @todo review with iq if we need a calibration table for this. */
    acamera_isp_frame_stitch_lm_thresh_high_write( p_ictx->settings.isp_base, 60947 );
    acamera_isp_frame_stitch_lm_thresh_low_write( p_ictx->settings.isp_base, 49151 );
    acamera_isp_frame_stitch_ms_thresh_high_write( p_ictx->settings.isp_base, 60947 );
    acamera_isp_frame_stitch_ms_thresh_low_write( p_ictx->settings.isp_base, 49151 );
    acamera_isp_frame_stitch_svs_thresh_high_write( p_ictx->settings.isp_base, 60947 );
    acamera_isp_frame_stitch_svs_thresh_low_write( p_ictx->settings.isp_base, 49151 );

    acamera_isp_sinter_view_filter_write( p_ictx->settings.isp_base, 0 );
    acamera_isp_sinter_int_select_write( p_ictx->settings.isp_base, *p_sinter_params );
    acamera_isp_sinter_scale_mode_write( p_ictx->settings.isp_base,
                                         ACAMERA_ISP_SINTER_SCALE_MODE_USE_ALL_FILTERS );

    acamera_isp_pipeline_bypass_sinter_write( p_ictx->settings.isp_base, 0 );
    acamera_isp_sinter_enable_write( p_ictx->settings.isp_base, 1 );
}

void noise_reduction_reload_calibration( noise_reduction_fsm_t *p_fsm )
{
    const uint8_t *p_sinter_radial_lut = (uint8_t *)calib_mgr_u8_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_SINTER_RADIAL_LUT );
    // Params must be in the following format: rm_enable, rm_centre_x, rm_centre_y, rm_off_centre_mult
    const uint16_t *p_sinter_radial_params = (const uint16_t *)calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_SINTER_RADIAL_PARAMS );
    int i;
    uint8_t number_of_nodes = calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_SINTER_RADIAL_LUT );

    for ( i = 0; i < number_of_nodes; ++i ) {
        acamera_isp_sinter_shading_rm_shading_lut_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, i, p_sinter_radial_lut[i] );
    }
    acamera_isp_sinter_rm_enable_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_sinter_radial_params[0] );
    acamera_isp_sinter_rm_center_x_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_sinter_radial_params[1] );
    acamera_isp_sinter_rm_center_y_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_sinter_radial_params[2] );
    acamera_isp_sinter_rm_off_center_mult_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_sinter_radial_params[3] );

#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    if ( calib_mgr_lut_exists( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NOISE_PROFILE_CONFIG ) ) {
        const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
        const uint16_t *config = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NOISE_PROFILE_CONFIG );
        const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
        const uint32_t *gamma_black_levels = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_GAMMA_BLACK_LEVELS );
        uint16_t data_tmp = 0, gain16 = 0;
        uint32_t gain_tmp = ( get_context_param( p_ictx, SYSTEM_ISP_DIGITAL_GAIN_PARAM ) << ( LOG2_GAIN_SHIFT - 5 ) );

        // Convert ISP Digital Gain to linear and prepare for sqrt32.
        gain_tmp = math_exp2( gain_tmp, LOG2_GAIN_SHIFT, 8 ) << 8;
        gain16 = sqrt32( gain_tmp );

        // Thresh1
        data_tmp = *config++;
        data_tmp = ( uint16_t )( MIN( 65535, (int32_t)gamma_black_levels[1] + MAX( 0, ( ( (int32_t)data_tmp * (int32_t)gain16 ) >> 8 ) - ( ( (int32_t)gamma_black_levels[1] * (int32_t)gain16 ) >> 8 ) ) ) );
        acamera_isp_noise_profile_thresh_1_write( isp_base, data_tmp );
        // Thresh2
        data_tmp = *config++;
        data_tmp = ( uint16_t )( MIN( 65535, (int32_t)gamma_black_levels[1] + MAX( 0, ( ( (int32_t)data_tmp * (int32_t)gain16 ) >> 8 ) - ( ( (int32_t)gamma_black_levels[1] * (int32_t)gain16 ) >> 8 ) ) ) );
        acamera_isp_noise_profile_thresh_2_write( isp_base, data_tmp );
        // Thresh3
        data_tmp = *config++;
        data_tmp = ( uint16_t )( MIN( 65535, (int32_t)gamma_black_levels[1] + MAX( 0, ( ( (int32_t)data_tmp * (int32_t)gain16 ) >> 8 ) - ( ( (int32_t)gamma_black_levels[1] * (int32_t)gain16 ) >> 8 ) ) ) );
        acamera_isp_noise_profile_thresh_3_write( isp_base, data_tmp );

        acamera_isp_noise_profile_use_exp_mask_write( isp_base, *config++ );
        acamera_isp_noise_profile_hlog_exp_rat_max_write( isp_base, *config++ );
        acamera_isp_noise_profile_hlog_exp_rat_0_1_max_diff_write( isp_base, *config++ );
        acamera_isp_noise_profile_hlog_exp_rat_0_2_max_diff_write( isp_base, *config++ );
        acamera_isp_noise_profile_hlog_exp_rat_0_3_max_diff_write( isp_base, *config++ );
    } else {
        LOG( LOG_ERR, "CALIBRATION_NOISE_PROFILE_CONFIG is missing, module will not be configured correctly!" );
    }
#endif /* ISP_RTL_VERSION_R */
}

void noise_reduction_update_hw( noise_reduction_fsm_t *p_fsm )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t manual_sinter = get_context_param( p_ictx, ISP_MODULES_MANUAL_SINTER_PARAM );

    if ( get_context_param( p_ictx, ISP_MODULES_MANUAL_DEMOSAIC_PARAM ) == 0 ) {
        const uint32_t ldr_gain_gain = get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM );
        int tbl_inx;

#if ( ISP_RTL_VERSION_R == 2 )
        uint32_t raw_scaler_enable = get_context_param( p_ictx, IMAGE_RAW_SCALER_ENABLE_ID_PARAM );
        tbl_inx = raw_scaler_enable ? CALIBRATION_DEMOSAIC_UU_SLOPE_WRS : CALIBRATION_DEMOSAIC_UU_SLOPE;
#else
        tbl_inx = CALIBRATION_DEMOSAIC_UU_SLOPE;
#endif
        acamera_isp_demosaic_uu_slope_write( p_ictx->settings.isp_base, calc_modulation_u16( ldr_gain_gain, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), tbl_inx ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), tbl_inx ) ) );
    }

    const uint32_t sensor_wdr_mode = get_context_param( p_ictx, SENSOR_WDR_MODE_PARAM );
    if ( ( sensor_wdr_mode == WDR_MODE_FS_LIN ) || ( sensor_wdr_mode == WDR_MODE_FS_HDR ) ) {
        stitching_error_calculate( p_fsm );
    }
//////////////////////////////////////////////////
/*  noise level calculation (for noise profile and sinter). Valid only for Moss */
#if ( ISP_RTL_VERSION_R == 2 )
    /* Note:
     *  We re-read gain_XX registers because the context values in SYSTEM_AWB_RED_GAIN_PARAM and
     *  SYSTEM_AWB_BLUE_GAIN_PARAM are store before black level compensation.
     *
     *  Alternative to re-read the actual gains would be to multiply the above context values
     *  by the static white red and blue (from static calibrations "_calibration_static_wb").
     *  This multiplication would lead to the same gain_00 and gain_11 but the reason as to why
     *  is not straightforward. So, for simplicity we re-read the actual gain registers.
     */

    uint16_t gain_00 = acamera_isp_white_balance_gain_00_read( p_ictx->settings.isp_base );
    uint16_t gain_01 = acamera_isp_white_balance_gain_01_read( p_ictx->settings.isp_base );
    uint16_t gain_11 = acamera_isp_white_balance_gain_11_read( p_ictx->settings.isp_base );
    /* during bootup the gains might be read as zero, which leads to a division by zero below) */
    if ( gain_01 != 0 ) {
        // gain_01 will not always be = 256, so this division is needed
        uint32_t rgain = div_fixed( gain_00, gain_01, 8 );
        uint32_t noise_level_r = calc_modulation_u32( rgain, calib_mgr_mod32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NOISE_LEVEL_R_LUT ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NOISE_LEVEL_R_LUT ) );
        // register noise_level_r is 8 bits long
        noise_level_r = MIN( noise_level_r, 255 );

        acamera_isp_noise_profile_noise_level_r_write( p_ictx->settings.isp_base, noise_level_r );

        // gain_01 will not always be = 256, so this division is needed
        uint32_t bgain = div_fixed( gain_11, gain_01, 8 );
        uint32_t noise_level_b = calc_modulation_u32( bgain, calib_mgr_mod32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NOISE_LEVEL_B_LUT ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NOISE_LEVEL_B_LUT ) );
        // register noise_level_b is 8 bits long
        noise_level_b = MIN( noise_level_b, 255 );

        acamera_isp_noise_profile_noise_level_b_write( p_ictx->settings.isp_base, noise_level_b );
    }
#endif

    if ( manual_sinter == 0 ) {
        // For control software
        p_fsm->snr_strength_target = get_context_param( p_ictx, SYSTEM_SINTER_THRESHOLD_TARGET_PARAM );
        sinter_strength_calculate( p_fsm );

        uint32_t cmos_exp_ratio = 0;
        WRAP_GENERAL_CMD( p_ictx, CMD_ID_CMOS_EXPOSURE_RATIO, CMD_DIRECTION_GET, NULL, &cmos_exp_ratio );

////// !!!!!!!!!!!!!!! This will only work if data is in sqrt domain at the input of Sinter !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// REN project may need this :S but redundant in C71.
//////////////////////////////////////////// Pipeline with gamma dl and old noise profile block //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if ( ISP_RTL_VERSION_R == 0 )

        uint32_t exp_ratio = 16 * 64; // Assuming a 16:1 16:1 sensor.
        uint32_t mult;
        uint32_t FS_thresh;
        uint32_t FS_thresh_sqrt_ms;

        mult = ( 131072 ) / exp_ratio;         // u1.11
        FS_thresh = 3500;                      // Assuming stitching point is 3500 in 12 bits.
        FS_thresh = ( FS_thresh * mult ) >> 3; // 0.DW_Out
        FS_thresh_sqrt_ms = sqrt64( (uint64_t)FS_thresh * 1048576 ) / 16 + acamera_isp_gamma_fe_black_level_out_dl_read( p_ictx->settings.isp_base );

#ifdef ACAMERA_ISP_NOISE_PROFILE_EXP_THRESH_DEFAULT
        acamera_isp_noise_profile_exp_thresh_write( p_ictx->settings.isp_base, (uint16_t)FS_thresh_sqrt_ms );
#endif

        int8_t long_thresh_r0 = 64 - ( 8 * log2_int_to_fixed( cmos_exp_ratio >> 6, 8, 0 ) >> 8 );
        int8_t short_thresh_r0 = 64;

        if ( long_thresh_r0 < 0 ) {
            long_thresh_r0 = 0;
        }

        p_fsm->snr_thresh_exp2 = p_fsm->snr_thresh_master + short_thresh_r0 + ( p_fsm->snr_thresh_contrast );
        p_fsm->snr_thresh_exp1 = p_fsm->snr_thresh_master + long_thresh_r0;


#ifdef ACAMERA_ISP_NOISE_PROFILE_SHORT_RATIO_DEFAULT
        acamera_isp_noise_profile_short_ratio_write( p_ictx->settings.isp_base, 4 );
#endif
#ifdef ACAMERA_ISP_NOISE_PROFILE_LONG_RATIO_DEFAULT
        acamera_isp_noise_profile_long_ratio_write( p_ictx->settings.isp_base, 4 );
#endif

#endif
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if ISP_TOTAL_SINTER_CONTROL_IN_MANUAL_MODE
        if ( manual_sinter == 0 ) // Do not update values if manual mode.
        {
#endif

//////// REN project may need this :S but redundant in C71.
#if ( ISP_RTL_VERSION_R == 0 )

#ifdef ACAMERA_ISP_SINTER_THRESH_LONG_DEFAULT
            acamera_isp_sinter_thresh_long_write( p_ictx->settings.isp_base, ( uint8_t )( p_fsm->snr_thresh_exp1 > 255 ? 255 : p_fsm->snr_thresh_exp1 ) );
#endif
#ifdef ACAMERA_ISP_SINTER_THRESH_SHORT_DEFAULT
            acamera_isp_sinter_thresh_short_write( p_ictx->settings.isp_base, ( uint8_t )( p_fsm->snr_thresh_exp2 > 255 ? 255 : p_fsm->snr_thresh_exp2 ) );
#endif

#else // ISP_RTL_VERSION_R

#ifdef ACAMERA_ISP_NOISE_PROFILE_GLOBAL_OFFSET_DEFAULT
        acamera_isp_noise_profile_global_offset_write( p_ictx->settings.isp_base, ( uint8_t )( p_fsm->snr_thresh_master > 255 ? 255 : p_fsm->snr_thresh_master ) );
#endif

#endif

#if ISP_TOTAL_SINTER_CONTROL_IN_MANUAL_MODE
        }
#endif
    }
}

void noise_reduction_deinit( noise_reduction_fsm_t *p_fsm )
{
    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;

    acamera_isp_sinter_enable_write( isp_base, 0 );
    acamera_isp_pipeline_bypass_sinter_write( isp_base, 1 );
}
