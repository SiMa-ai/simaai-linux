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
#include "acamera_logger.h"
#include "acamera_math.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_COLOR_MATRIX

#ifndef AWB_LIGHT_SOURCE_A
#define AWB_LIGHT_SOURCE_A 0x01
#endif

#ifndef AWB_LIGHT_SOURCE_D40
#define AWB_LIGHT_SOURCE_D40 0x02
#endif

#ifndef AWB_LIGHT_SOURCE_D50
#define AWB_LIGHT_SOURCE_D50 0x03
#endif

// Threshold for the LSC table hysterisis.
#define AWB_DLS_LIGHT_SOURCE_D40_D50_BORDER_low ( ( ( AWB_LIGHT_SOURCE_D50_TEMPERATURE + AWB_LIGHT_SOURCE_D40_TEMPERATURE ) >> 1 ) - 200 )
#define AWB_DLS_LIGHT_SOURCE_D40_D50_BORDER_high ( ( ( AWB_LIGHT_SOURCE_D40_TEMPERATURE + AWB_LIGHT_SOURCE_D50_TEMPERATURE ) >> 1 ) + 200 )
#define AWB_DLS_LIGHT_SOURCE_A_D40_BORDER_low ( ( ( AWB_LIGHT_SOURCE_A_TEMPERATURE + AWB_LIGHT_SOURCE_D40_TEMPERATURE ) >> 1 ) - 200 )
#define AWB_DLS_LIGHT_SOURCE_A_D40_BORDER_high ( ( ( AWB_LIGHT_SOURCE_D40_TEMPERATURE + AWB_LIGHT_SOURCE_A_TEMPERATURE ) >> 1 ) + 200 )

//==============Math Functions========================================================
void matrix_matrix_multiply( int16_t *a1, int16_t *a2, int16_t *result, int dim1, int dim2, int dim3 )
{
    int i, j, k;

    for ( i = 0; i < dim1; ++i ) {
        for ( j = 0; j < dim3; ++j ) {

            int32_t temp = 0;
            for ( k = 0; k < dim2; ++k ) {
                temp += ( ( (int32_t)a1[i * dim2 + k] * a2[k * dim3 + j] ) >> 8 );
            }

            result[i * dim3 + j] = (int16_t)temp;
        }
    }
}

void matrix_vector_multiply( int16_t *m, uint16_t *v, uint16_t *result, int dim1, int dim2 )
{
    int i, j;

    for ( i = 0; i < dim1; ++i ) {

        int32_t temp = 0;
        for ( j = 0; j < dim2; ++j ) {
            temp += ( ( (int32_t)m[i * dim2 + j] * v[j] ) >> 8 );
        }

        result[i] = (uint16_t)temp;
    }
}

//==============Conversion Functions========================================================

#if ( ISP_RTL_VERSION_R != 2 )
/* Not relevant for MOSS */

/**
 * @brief Converts CCM A coefficient from register format to 16-bit 2's complement format
 * @param[in] value CCM A coefficient in the register format:
 * signed magnitude 7.8-bit fixed-point with effective range 4.8 fixed-point ~ +/- 15.996
 *
 * @return CCM A coefficient in 16-bit 2's complement
 */
int16_t a_reg_to_twos_complement( uint16_t value )
{
    // Absolute value
    int16_t result = value & ( 0x7FFFu );

    // Range check, value should within 0..4095 range (12-bit)
    if ( result > 4095u ) {
        LOG( LOG_WARNING, "CCM A coefficient value (%s%u) is out of range (-4095..+4095) and will be truncated.",
             ( ( value & ( 1 << 15 ) ) ? "-" : "+" ), result );
        result = 4095u;
    }

    // Add sign
    if ( value & ( 1 << 15 ) ) {
        result = -result;
    }

    return result;
}

/* Not relevant for MOSS */
/**
 * @brief Converts CCM B coefficient from register format to 16-bit 2's complement format
 * @param[in] value CCM B coefficient in the register format (15-bit 2's complement)
 * 
 * @return CCM B coefficient in 16-bit 2's complement
 */
int16_t b_reg_to_twos_complement( uint16_t value )
{
    // Range check
    if ( value > 0x7FFFu ) {
        LOG( LOG_WARNING, "CCM B coefficient value (%u) is out of range (0..32767) and will be truncated.", value );
        value &= 0x7FFFu;
    }

    int16_t result = value;

    // Extend sign
    if ( value & ( 1 << 14 ) ) {
        result |= ( 1 << 15 );
    }

    return result;
}
#endif

/**
 * @brief Converts CCM A coefficient from 16-bit 2's complement format to register format
 * @param[in] value CCM A coefficient in 16-bit 2's complement format
 * 
 * @return CCM A coefficient in register format:
 * signed magnitude 7.8-bit fixed-point with effective range 4.8 fixed-point ~ +/- 15.996
 */

uint16_t twos_complement_to_a_reg( int16_t value )
{
    // Absolute value
    uint16_t result = ACAMERA_ABS( value );

    // Range check, value should within 0..4095 range (12-bit)
    if ( result > ( ( 1 << 12 ) - 1 ) ) {
        LOG( LOG_WARNING, "CCM A coefficient value (%d) is out of range (-4095..+4095) and will be truncated.", value );
        result = ( ( 1 << 12 ) - 1 );
    }

    // Add sign
    if ( value < 0 ) {
        result |= ( 1 << 15 );
    }

    return result;
}

/**
 * @brief Converts CCM B coefficient from 16-bit 2's complement format to register format
 * @param[in] value CCM B coefficient in 16-bit 2's complement
 * 
 * @return CCM B coefficient in the register format (15-bit 2's complement)
 */
uint16_t twos_complement_to_b_reg( int16_t value )
{
    // Range check
    if ( ( value > 16383 ) || ( value < -16384 ) ) {
        LOG( LOG_WARNING, "CCM B coefficient value (%d) is out of range (-16384..16383) and will be truncated.", value );
        if ( value > 0 ) {
            value = 16383;
        } else {
            value = -16384;
        }
    }

    // Everything is 2's complement, truncating to 15-bits
    uint16_t result = value & 0x7FFFu;

    return result;
}

//==============Saturation Related Functions========================================================
void saturation_modulate_strength( color_matrix_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    const uint32_t ldr_gain_log2 = get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM );

    uint32_t ccm_saturation_table_idx = CALIBRATION_SATURATION_STRENGTH;
    const modulation_entry_t *ccm_saturation_table = calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), ccm_saturation_table_idx );
    uint32_t ccm_saturation_table_len = calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), ccm_saturation_table_idx );
    uint16_t strength = calc_modulation_u16( ldr_gain_log2, ccm_saturation_table, ccm_saturation_table_len );

    set_context_param( p_ictx, SYSTEM_SATURATION_TARGET_PARAM, strength );
}

static void color_mat_calculate_saturation_matrix( int16_t *saturation_matrix, uint8_t saturation )
{
    static const int16_t identity[9] = {0x0100, 0x0000, 0x0000, 0x0000, 0x0100, 0x0000, 0x0000, 0x0000, 0x0100};
    static const int16_t black_white[9] = {0x004c, 0x0096, 0x001d, 0x004c, 0x0096, 0x001d, 0x004c, 0x0096, 0x001d};
    int i;
    int16_t alpha;
    // (1 - saturation)
    alpha = (int16_t)0x100 - ( (uint16_t)saturation << 1 );

    for ( i = 0; i < 9; ++i ) {
        int16_t result;
        // (1. - saturation) * _black_white
        result = ( (int32_t)alpha * black_white[i] + 0x80 ) >> 8;
        // += (saturation * _identity)
        result = result + ( ( (int32_t)identity[i] * ( (uint16_t)saturation << 1 ) + 0x80 ) >> 8 );
        saturation_matrix[i] = result;
    }
}

//==============Color Matrix Related Functions========================================================
void color_matrix_recalculate( color_matrix_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    // Saturation matrix
    if ( get_context_param( p_ictx, SYSTEM_MANUAL_SATURATION_PARAM ) == 0 ) { //  Do not update values if manual mode

        saturation_modulate_strength( p_fsm );
    }

    color_mat_calculate_saturation_matrix( p_fsm->color_saturation_matrix, (uint8_t)get_context_param( p_ictx, SYSTEM_SATURATION_TARGET_PARAM ) );
    // New colour management:
    matrix_matrix_multiply( p_fsm->color_saturation_matrix, p_fsm->color_correction_matrix, p_fsm->color_matrix, 3, 3, 3 );
}


#if ( ISP_RTL_VERSION_R == 2 )
//In case of MOSS , values are already in two complement format at the LUT
void color_matrix_setup( int16_t *p_matrix, const uint16_t *source_matrix, int16_t size )
{
    int i;

    for ( i = 0; i < size; i++ ) {
        p_matrix[i] = source_matrix[i];
    }
}
#else
//In case of Morgan, values are in registers format at LUT
void color_matrix_setup( int16_t *p_matrix, const uint16_t *source_matrix, int16_t size )
{
    int i;

    if ( size == ISP_CCM_SIZE ) { //CCM_A
        for ( i = 0; i < size; i++ ) {
            p_matrix[i] = a_reg_to_twos_complement( source_matrix[i] );
        }
    } else if ( size == ISP_CCM_B_SIZE ) { //CCM_B
        for ( i = 0; i < size; i++ ) {
            p_matrix[i] = b_reg_to_twos_complement( source_matrix[i] );
        }
    }
}
#endif


static void rgb2rgb_matrix_update( uint32_t isp_base, int16_t *CCM_A, int16_t *CCM_B )
{
#if ( ISP_RTL_VERSION_R == 2 )
    acamera_isp_pf_correction_ccm_coeff_rr_write( isp_base, CCM_A[0] );
    acamera_isp_color_matrix_rgb_coefft_r_r_write( isp_base, CCM_A[0] );
    acamera_isp_pf_correction_ccm_coeff_rg_write( isp_base, CCM_A[1] );
    acamera_isp_color_matrix_rgb_coefft_r_g_write( isp_base, CCM_A[1] );
    acamera_isp_pf_correction_ccm_coeff_rb_write( isp_base, CCM_A[2] );
    acamera_isp_color_matrix_rgb_coefft_r_b_write( isp_base, CCM_A[2] );
    acamera_isp_pf_correction_ccm_coeff_gr_write( isp_base, CCM_A[3] );
    acamera_isp_color_matrix_rgb_coefft_g_r_write( isp_base, CCM_A[3] );
    acamera_isp_pf_correction_ccm_coeff_gg_write( isp_base, CCM_A[4] );
    acamera_isp_color_matrix_rgb_coefft_g_g_write( isp_base, CCM_A[4] );
    acamera_isp_pf_correction_ccm_coeff_gb_write( isp_base, CCM_A[5] );
    acamera_isp_color_matrix_rgb_coefft_g_b_write( isp_base, CCM_A[5] );
    acamera_isp_pf_correction_ccm_coeff_br_write( isp_base, CCM_A[6] );
    acamera_isp_color_matrix_rgb_coefft_b_r_write( isp_base, CCM_A[6] );
    acamera_isp_pf_correction_ccm_coeff_bg_write( isp_base, CCM_A[7] );
    acamera_isp_color_matrix_rgb_coefft_b_g_write( isp_base, CCM_A[7] );
    acamera_isp_pf_correction_ccm_coeff_bb_write( isp_base, CCM_A[8] );
    acamera_isp_color_matrix_rgb_coefft_b_b_write( isp_base, CCM_A[8] );
#else
    acamera_isp_out_format_rgb2rgb_coef_a_11_write( isp_base, twos_complement_to_a_reg( CCM_A[0] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_12_write( isp_base, twos_complement_to_a_reg( CCM_A[1] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_13_write( isp_base, twos_complement_to_a_reg( CCM_A[2] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_21_write( isp_base, twos_complement_to_a_reg( CCM_A[3] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_22_write( isp_base, twos_complement_to_a_reg( CCM_A[4] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_23_write( isp_base, twos_complement_to_a_reg( CCM_A[5] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_31_write( isp_base, twos_complement_to_a_reg( CCM_A[6] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_32_write( isp_base, twos_complement_to_a_reg( CCM_A[7] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_33_write( isp_base, twos_complement_to_a_reg( CCM_A[8] ) );
    acamera_isp_out_format_rgb2rgb_coef_a_14_write( isp_base, twos_complement_to_a_reg( 0 ) );
    acamera_isp_out_format_rgb2rgb_coef_a_24_write( isp_base, twos_complement_to_a_reg( 0 ) );
    acamera_isp_out_format_rgb2rgb_coef_a_34_write( isp_base, twos_complement_to_a_reg( 0 ) );
#endif

    if ( CCM_B != NULL ) {
        acamera_isp_out_format_rgb2rgb_coef_b_1_write( isp_base, twos_complement_to_b_reg( CCM_B[0] ) );
        acamera_isp_out_format_rgb2rgb_coef_b_2_write( isp_base, twos_complement_to_b_reg( CCM_B[1] ) );
        acamera_isp_out_format_rgb2rgb_coef_b_3_write( isp_base, twos_complement_to_b_reg( CCM_B[2] ) );
    } else {
        acamera_isp_out_format_rgb2rgb_coef_b_1_write( isp_base, twos_complement_to_b_reg( 0 ) );
        acamera_isp_out_format_rgb2rgb_coef_b_2_write( isp_base, twos_complement_to_b_reg( 0 ) );
        acamera_isp_out_format_rgb2rgb_coef_b_3_write( isp_base, twos_complement_to_b_reg( 0 ) );
    }
}

void color_matrix_write( color_matrix_fsm_ptr_t p_fsm )
{
    int16_t *p_cm_target = p_fsm->manual_CCM ? p_fsm->manual_color_matrix : p_fsm->color_matrix;

    // LUV
    if ( !( acamera_isp_out_format_rgb2xyz_bypass_read( PHY_ADDR_ISP ) & 0x1 ) ) {

        acamera_isp_out_format_rgb2xyz_coef_a_11_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[0] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_12_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[1] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_13_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[2] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_21_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[3] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_22_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[4] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_23_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[5] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_31_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[6] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_32_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[7] ) );
        acamera_isp_out_format_rgb2xyz_coef_a_33_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_luv[8] ) );

        acamera_isp_out_format_rgb2xyz_coef_a_14_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( 0 ) );
        acamera_isp_out_format_rgb2xyz_coef_a_24_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( 0 ) );
        acamera_isp_out_format_rgb2xyz_coef_a_34_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( 0 ) );

        acamera_isp_out_format_rgb2yuv_coef_b_1_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_b_reg( p_fsm->color_matrix_luv_b[0] ) );
        acamera_isp_out_format_rgb2yuv_coef_b_2_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_b_reg( p_fsm->color_matrix_luv_b[1] ) );
        acamera_isp_out_format_rgb2yuv_coef_b_3_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_b_reg( p_fsm->color_matrix_luv_b[2] ) );
        return;
    }

    if ( !( acamera_isp_out_format_rgb2s_bypass_read( PHY_ADDR_ISP ) & 0x1 ) ) {
        rgb2rgb_matrix_update( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_fsm->color_matrix_s2, p_fsm->color_matrix_s2_b );
    } else if ( !( acamera_isp_out_format_ab2hs_bypass_read( PHY_ADDR_ISP ) & 0x1 ) ) {
        rgb2rgb_matrix_update( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_fsm->color_matrix_hs, p_fsm->color_matrix_hs_b );
    } else {
        // Default (rgb2rgb)
        LOG( LOG_DEBUG, "p_fsm->manual_CCM %d %d %d %d", p_fsm->manual_CCM, *( p_cm_target + 0 ), *( p_cm_target + 1 ), *( p_cm_target + 2 ) );

        rgb2rgb_matrix_update( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_cm_target, NULL );
    }

    // YUV (requires rgb2rgb)
    if ( !( acamera_isp_out_format_rgb2yuv_bypass_read( PHY_ADDR_ISP ) & 0x1 ) ) {

        acamera_isp_out_format_rgb2yuv_coef_a_11_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[0] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_12_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[1] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_13_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[2] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_21_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[3] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_22_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[4] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_23_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[5] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_31_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[6] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_32_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[7] ) );
        acamera_isp_out_format_rgb2yuv_coef_a_33_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_a_reg( p_fsm->color_matrix_yuv[8] ) );
        acamera_isp_out_format_rgb2yuv_coef_b_1_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_b_reg( p_fsm->color_matrix_yuv_b[0] ) );
        acamera_isp_out_format_rgb2yuv_coef_b_2_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_b_reg( p_fsm->color_matrix_yuv_b[1] ) );
        acamera_isp_out_format_rgb2yuv_coef_b_3_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, twos_complement_to_b_reg( p_fsm->color_matrix_yuv_b[2] ) );
    }
}

void color_matrix_change_CCMs( color_matrix_fsm_ptr_t p_fsm )
{
    uint8_t i;
    const uint16_t *p_mtrx;
    //	For CCM switching

    // Light source: A
    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_A_CCM );
    color_matrix_setup( p_fsm->color_matrix_A, p_mtrx, ISP_CCM_SIZE );

    // Light source: D40
    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_D40_CCM );
    color_matrix_setup( p_fsm->color_matrix_D40, p_mtrx, ISP_CCM_SIZE );

    // Light source: D50
    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_MT_ABSOLUTE_LS_D50_CCM );
    color_matrix_setup( p_fsm->color_matrix_D50, p_mtrx, ISP_CCM_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_MATRIX_YUV_PRESETS );
    color_matrix_setup( p_fsm->color_matrix_yuv, p_mtrx, ISP_CCM_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_MATRIX_LUV_PRESETS );
    color_matrix_setup( p_fsm->color_matrix_luv, p_mtrx, ISP_CCM_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_RGB2RGB_HS_CONVERSION );
    color_matrix_setup( p_fsm->color_matrix_hs, p_mtrx, ISP_CCM_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_RGB2RGB_S2_CONVERSION );
    color_matrix_setup( p_fsm->color_matrix_s2, p_mtrx, ISP_CCM_SIZE );

// Default tables
#if ( ISP_RTL_VERSION_R == 2 )
    static uint16_t cm_one[ISP_CCM_SIZE] = {1024, 0, 0, 0, 1024, 0, 0, 0, 1024};
#else
    static uint16_t cm_one[ISP_CCM_SIZE] = {256, 0, 0, 0, 256, 0, 0, 0, 256};
#endif
    color_matrix_setup( p_fsm->color_matrix_one, cm_one, ISP_CCM_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_MATRIX_B_YUV_PRESETS );
    color_matrix_setup( p_fsm->color_matrix_yuv_b, p_mtrx, ISP_CCM_B_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_MATRIX_B_LUV_PRESETS );
    color_matrix_setup( p_fsm->color_matrix_luv_b, p_mtrx, ISP_CCM_B_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_RGB2RGB_HS_CONVERSION_B );
    color_matrix_setup( p_fsm->color_matrix_hs_b, p_mtrx, ISP_CCM_B_SIZE );

    p_mtrx = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_RGB2RGB_S2_CONVERSION_B );
    color_matrix_setup( p_fsm->color_matrix_s2_b, p_mtrx, ISP_CCM_B_SIZE );

    if ( p_fsm->light_source_change_frames_left != 0 ) {

        // In CCM switching
        // call CCM switcher to update CCM
        // (note: this does mean CCM switching takes one frame less than normal)
        color_matrix_update_hw( p_fsm );
    } else {
        // Not moving, update current CCMs

        int16_t *p_ccm_next;

        switch ( p_fsm->light_source_ccm ) {

        case AWB_LIGHT_SOURCE_A:
            p_ccm_next = p_fsm->color_matrix_A;
            break;
        case AWB_LIGHT_SOURCE_D40:
            p_ccm_next = p_fsm->color_matrix_D40;
            break;
        case AWB_LIGHT_SOURCE_D50:
            p_ccm_next = p_fsm->color_matrix_D50;
            break;
        default:
            p_ccm_next = p_fsm->color_matrix_one;
            break;
        }

        for ( i = 0; i < 9; i++ ) {
            p_fsm->color_correction_matrix[i] = p_ccm_next[i];
        }
    }
}

/**
 * color_matrix_initialize() - FSM handler function
 * @p_fsm: Pointer to color_matrix FSM.

 * Return: None
 */
void color_matrix_init( color_matrix_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    p_fsm->saturation_target = 0x80;

    set_context_param( p_ictx, SYSTEM_SATURATION_TARGET_PARAM, p_fsm->saturation_target );
    set_context_param( p_ictx, SYSTEM_MANUAL_SATURATION_PARAM, 0 );
}

/**
 * color_matrix_configure() - FSM handler function
 * @p_fsm: Pointer to color_matrix fsm.

 * Return: None
 */
void color_matrix_config( color_matrix_fsm_ptr_t p_fsm )
{
    color_matrix_change_CCMs( p_fsm );
}

/**
 * color_matrix_update_hw() - FSM handler function
 * @p_fsm: Pointer to color_matrix FSM.

 * Return: None
 */
void color_matrix_update_hw( color_matrix_fsm_ptr_t p_fsm )
{
    //	For CCM switching
    if ( p_fsm->light_source_change_frames_left != 0 ) {

        int16_t *p_ccm_prev;
        int16_t *p_ccm_cur = p_fsm->color_correction_matrix;
        int16_t *p_ccm_target;
        int16_t delta;

        switch ( p_fsm->light_source_ccm_previous ) {

        case AWB_LIGHT_SOURCE_A:
            p_ccm_prev = p_fsm->color_matrix_A;
            break;
        case AWB_LIGHT_SOURCE_D40:
            p_ccm_prev = p_fsm->color_matrix_D40;
            break;
        case AWB_LIGHT_SOURCE_D50:
            p_ccm_prev = p_fsm->color_matrix_D50;
            break;
        default:
            p_ccm_prev = p_fsm->color_matrix_one;

            break;
        }

        switch ( p_fsm->light_source_ccm ) {

        case AWB_LIGHT_SOURCE_A:
            p_ccm_target = p_fsm->color_matrix_A;
            break;
        case AWB_LIGHT_SOURCE_D40:
            p_ccm_target = p_fsm->color_matrix_D40;
            break;
        case AWB_LIGHT_SOURCE_D50:
            p_ccm_target = p_fsm->color_matrix_D50;
            break;
        default:
            p_ccm_target = p_fsm->color_matrix_one;
            break;
        }

        int i;
        for ( i = 0; i < 9; ++i ) {

            // Smooth transition
            // using curr += (target - curr)/frames_left causes no movement in first half
            // for small movements
            if ( p_fsm->light_source_change_frames > 1 ) {
                delta = ( ( p_ccm_target[i] - p_ccm_prev[i] ) * ( p_fsm->light_source_change_frames - p_fsm->light_source_change_frames_left ) ) / ( p_fsm->light_source_change_frames - 1 ); // division by zero is checked
                p_ccm_cur[i] = p_ccm_prev[i] + delta;
            }
        }
    }

    color_matrix_recalculate( p_fsm );
    color_matrix_write( p_fsm );

    if ( p_fsm->light_source_change_frames_left > 0 ) {
        p_fsm->light_source_change_frames_left--;
    }
}

void color_matrix_get_info( const color_matrix_fsm_ptr_t p_fsm, acamera_cmd_ccm_info *p_info )
{
    p_info->light_source = p_fsm->light_source;
    p_info->light_source_previous = p_fsm->light_source_previous;
    p_info->light_source_ccm = p_fsm->light_source_ccm;
    p_info->light_source_ccm_previous = p_fsm->light_source_ccm_previous;
    p_info->light_source_change_frames = p_fsm->light_source_change_frames;
    p_info->light_source_change_frames_left = p_fsm->light_source_change_frames_left;
}

void color_matrix_set_info( color_matrix_fsm_ptr_t p_fsm, const acamera_cmd_ccm_info *p_info )
{
    p_fsm->light_source = p_info->light_source;
    p_fsm->light_source_previous = p_info->light_source_previous;
    p_fsm->light_source_ccm = p_info->light_source_ccm;
    p_fsm->light_source_ccm_previous = p_info->light_source_ccm_previous;
    p_fsm->light_source_change_frames = p_info->light_source_change_frames;
    p_fsm->light_source_change_frames_left = p_info->light_source_change_frames_left;
}
