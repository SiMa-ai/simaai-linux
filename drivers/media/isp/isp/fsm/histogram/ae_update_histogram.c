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
#include "acamera_frontend_config.h"
#include "acamera_isp_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "acamera_math.h"

// Called from histogram_fsm_interrupt - ae_read_full_histogram_data func.
void ae_histogram_dgain_compensation( histogram_fsm_ptr_t p_fsm )
{
    if ( acamera_isp_pipeline_bypass_digital_gain_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base ) == 0 ) {
        int stub = 0;
        exposure_set_t exp;
        uint16_t ispDGain;

        if ( ISP_HISTOGRAM_POSITION_IS_BE && ( ISP_HISTOGRAM_BE_POSITION == AE_HISTOGRAM_TAP_AFTER_DECOMPANDER || ISP_HISTOGRAM_BE_POSITION == AE_HISTOGRAM_TAP_AFTER_SHADING ) )
            return; // Stream already includes dgain.

        WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ), CMD_ID_FRAME_EXPOSURE_SET, CMD_DIRECTION_GET, &stub, &exp );
        ispDGain = math_exp2( exp.info.isp_dgain_log2, LOG2_GAIN_SHIFT, 8 );

        if ( ispDGain > 0x100 ) {
            //uint16_t pivot = div_fixed( ISP_METERING_HISTOGRAM_SIZE_BINS << 8, ispDGain, 8 ) >> 8; // Need only int part.
            uint16_t pivot = ( ( ISP_METERING_HISTOGRAM_SIZE_BINS << 8 ) / ispDGain );

            int i;

            // Moving all elems after pivot to the clipped area.
            for ( i = pivot; i <= ISP_METERING_HISTOGRAM_SIZE_BINS - 2; i++ ) {
                p_fsm->fullhist[ISP_METERING_HISTOGRAM_SIZE_BINS - 1] += p_fsm->fullhist[i];
                p_fsm->fullhist[i] = 0;
            }

            // Spreading all elems before pivot all over hist.
            for ( i = pivot - 1; i >= 0; i-- ) {
                uint32_t j = multiplication_fixed_to_fixed( (uint32_t)i, ispDGain, 0, 8 );
                if ( j != i ) {
                    p_fsm->fullhist[j] = p_fsm->fullhist[i];
                    p_fsm->fullhist[i] = 0;
                }
            }
        } else {
            if ( ispDGain < 0x100 )
                LOG( LOG_ERR, "ispDGain %x < 0x100 is not handled; ispGDain not applied to histogram.", ispDGain );
        }
    }
}

void ae_update_histogram_shading_lut( histogram_fsm_const_ptr_t p_fsm, int input )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t cfg_offset = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? p_ictx->settings.isp_base : PHY_ADDR_ISP;
    uint8_t hist_radial_shading_enable = 1;

    if ( acamera_isp_pipeline_bypass_radial_shading_read( p_ictx->settings.isp_base ) != 0 ) {
        hist_radial_shading_enable = 0;
    }

    // After FE, BE_WDRGAIN,BE_FS and BE_COMPAND, histogram already includes shading_correction.
    if ( ISP_HISTOGRAM_POSITION_IS_BE && ( ISP_HISTOGRAM_BE_POSITION == AE_HISTOGRAM_TAP_AFTER_SHADING ) ) {
        hist_radial_shading_enable = 0;
    }

    switch ( input ) {
    case 0:
        acamera_isp_metering_hist_1_radial_shading_enable_write( cfg_offset, hist_radial_shading_enable );
        break;
    case 1:
        acamera_isp_metering_hist_2_radial_shading_enable_write( cfg_offset, hist_radial_shading_enable );
        break;
    case 2:
        acamera_isp_metering_hist_3_radial_shading_enable_write( cfg_offset, hist_radial_shading_enable );
        break;
    case 3:
        acamera_isp_metering_hist_4_radial_shading_enable_write( cfg_offset, hist_radial_shading_enable );
        break;
    default:
        LOG( LOG_ERR, "Invalid metering hist input" );
        break;
    }

    if ( hist_radial_shading_enable == 0 ) {
        return;
    }

    const uint32_t *radial_shading_lut = calib_mgr_u32_lut_get( p_ictx->calib_mgr_data, CALIBRATION_SHADING_RADIAL_G );
    const uint32_t radial_shading_lut_size = calib_mgr_lut_len( p_ictx->calib_mgr_data, CALIBRATION_SHADING_RADIAL_G );

    if ( ( radial_shading_lut_size >> 1 ) != ( ACAMERA_ISP_METERING_HIST_1_SHADING_LUT_ARRAY_DIM_0_SIZE - 1 ) ) {
        LOG( LOG_ERR, "Failed to configure metering histogram %d shading lut. Radial shading (G) size mismatch (%d != %d).",
             input, ( radial_shading_lut_size >> 1 ), ( ACAMERA_ISP_METERING_HIST_1_SHADING_LUT_ARRAY_DIM_0_SIZE - 1 ) );
        return;
    }

    int i = 0;
    switch ( input ) {
    case 0:
        for ( i = 0; i < ACAMERA_ISP_METERING_HIST_1_SHADING_LUT_ARRAY_DIM_0_SIZE; i++ )
            acamera_isp_metering_hist_1_shading_lut_write( cfg_offset, i, radial_shading_lut[i << 1] );
        break;
    case 1:
        for ( i = 0; i < ACAMERA_ISP_METERING_HIST_1_SHADING_LUT_ARRAY_DIM_0_SIZE; i++ )
            acamera_isp_metering_hist_2_shading_lut_write( cfg_offset, i, radial_shading_lut[i << 1] );
        break;
    case 2:
        for ( i = 0; i < ACAMERA_ISP_METERING_HIST_1_SHADING_LUT_ARRAY_DIM_0_SIZE; i++ )
            acamera_isp_metering_hist_3_shading_lut_write( cfg_offset, i, radial_shading_lut[i << 1] );
        break;
    case 3:
        for ( i = 0; i < ACAMERA_ISP_METERING_HIST_1_SHADING_LUT_ARRAY_DIM_0_SIZE; i++ )
            acamera_isp_metering_hist_4_shading_lut_write( cfg_offset, i, radial_shading_lut[i << 1] );
        break;
    default:
        LOG( LOG_ERR, "Invalid metering hist input" );
        break;
    }
}

void ae_update_histogram_wbgain( histogram_fsm_const_ptr_t p_fsm, int input )
{
    uint8_t awb_enabled = 1;
    uint16_t wbGain[4];
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t cfg_offset = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? p_ictx->settings.isp_base : PHY_ADDR_ISP;

    if ( acamera_isp_pipeline_bypass_white_balance_read( p_ictx->settings.isp_base ) != 0 ) {
        awb_enabled = 0;
    }

    // After FE, BE_WDRGAIN,BE_FS and BE_COMPAND, histogram already includes wbGain.
    if ( ISP_HISTOGRAM_POSITION_IS_BE && ( ISP_HISTOGRAM_BE_POSITION == AE_HISTOGRAM_TAP_AFTER_SHADING ) ) {
        awb_enabled = 0; // stream already includes wb_gains
    }

    if ( awb_enabled == 0 ) {
        wbGain[0] = 256;
        wbGain[1] = 256;
        wbGain[2] = 256;
        wbGain[3] = 256;
    } else {
#if defined( ISP_HAS_AWB_MESH_NBP_FSM ) || defined( ISP_HAS_AWB_MANUAL_FSM )
        wbGain[0] = ACAMERA_FSM2FSMGR_PTR( p_fsm )->cmos_fsm.wb[0];
        wbGain[1] = ACAMERA_FSM2FSMGR_PTR( p_fsm )->cmos_fsm.wb[1];
        wbGain[2] = ACAMERA_FSM2FSMGR_PTR( p_fsm )->cmos_fsm.wb[2];
        wbGain[3] = ACAMERA_FSM2FSMGR_PTR( p_fsm )->cmos_fsm.wb[3];
#else
        wbGain[0] = acamera_isp_white_balance_gain_00_read( p_ictx->settings.isp_base );
        wbGain[1] = acamera_isp_white_balance_gain_01_read( p_ictx->settings.isp_base );
        wbGain[2] = acamera_isp_white_balance_gain_10_read( p_ictx->settings.isp_base );
        wbGain[3] = acamera_isp_white_balance_gain_11_read( p_ictx->settings.isp_base );
#endif

        if ( ISP_HISTOGRAM_POSITION_IS_BE && ( ISP_HISTOGRAM_BE_POSITION == AE_HISTOGRAM_TAP_AFTER_DECOMPANDER ) ) {
            // x format is fixed 6.8; to get sqrt of x in 6.8 format, we:
            // make make it format 6.16 (tmp = x<<8), then sqrt32(tmp) (format is 3.8 now).
            // Actual gain format here is 7.8 (used 6.8 in the example for simplicity).
            uint32_t i, tmp;
            for ( i = 0; i < 4; i++ ) {
                tmp = wbGain[i] << 8;      //format 7.16
                wbGain[i] = sqrt32( tmp ); //format 4.8
            }
        }
    }
    switch ( input ) {
    case 0:
        acamera_isp_metering_hist_1_gain_00_write( cfg_offset, wbGain[0] );
        acamera_isp_metering_hist_1_gain_01_write( cfg_offset, wbGain[1] );
        acamera_isp_metering_hist_1_gain_10_write( cfg_offset, wbGain[2] );
        acamera_isp_metering_hist_1_gain_11_write( cfg_offset, wbGain[3] );
        break;
    case 1:
        acamera_isp_metering_hist_2_gain_00_write( cfg_offset, wbGain[0] );
        acamera_isp_metering_hist_2_gain_01_write( cfg_offset, wbGain[1] );
        acamera_isp_metering_hist_2_gain_10_write( cfg_offset, wbGain[2] );
        acamera_isp_metering_hist_2_gain_11_write( cfg_offset, wbGain[3] );
        break;
    case 2:
        acamera_isp_metering_hist_3_gain_00_write( cfg_offset, wbGain[0] );
        acamera_isp_metering_hist_3_gain_01_write( cfg_offset, wbGain[1] );
        acamera_isp_metering_hist_3_gain_10_write( cfg_offset, wbGain[2] );
        acamera_isp_metering_hist_3_gain_11_write( cfg_offset, wbGain[3] );
        break;
    case 3:
        acamera_isp_metering_hist_4_gain_00_write( cfg_offset, wbGain[0] );
        acamera_isp_metering_hist_4_gain_01_write( cfg_offset, wbGain[1] );
        acamera_isp_metering_hist_4_gain_10_write( cfg_offset, wbGain[2] );
        acamera_isp_metering_hist_4_gain_11_write( cfg_offset, wbGain[3] );
        break;
    default:
        LOG( LOG_ERR, "Invalid metering hist input" );
        break;
    }
}

void ae_update_histogram_black_level( histogram_fsm_const_ptr_t p_fsm, int input )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint32_t cfg_offset = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? p_ictx->settings.isp_base : PHY_ADDR_ISP;
    uint32_t hist_bk_level;

#if MODEL_MODULE
    input = ISP_MODEL_INPUT_PORT_FOR_AE_UPDATE_HIST;
#endif

    if ( ISP_HISTOGRAM_POSITION_IS_BE ) {
#if defined( ACAMERA_ISP_GAMMA_FE_BLACK_LEVEL_OUT_SQ_DEFAULT )
        hist_bk_level = acamera_isp_gamma_fe_black_level_out_sq_read( p_ictx->settings.isp_base );
#elif defined( ACAMERA_ISP_GAMMA_FE_BLACK_LEVEL_OUT_DL_DEFAULT )
        hist_bk_level = acamera_isp_gamma_fe_black_level_out_dl_read( p_ictx->settings.isp_base ); //comes from json files for R1
#endif
        // If histogram is on sqrt domain adjust black level accordingly.
        // Histogram pipeline is 20-bit, sqrt pipeline is 16 bit.
        // Black level is given in 16-bit range so we have to upscale it to 20-bit to match
        if ( ISP_HISTOGRAM_BE_POSITION == AE_HISTOGRAM_TAP_AFTER_DECOMPANDER ) {
            hist_bk_level <<= 4;
        }
    } else {
#if defined( ISP_HAS_SENSOR_FSM )
        acamera_cmd_sensor_info sensor_info;
        if ( WRAP_GENERAL_CMD( p_ictx, CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&sensor_info ) ) {
            return;
        }

        const int32_t black_level_shift = ISP_HISTOGRAM_INPUT_BIT_WIDTH - sensor_info.current_sensor_mode.channel_info.exposure_max_bit_width;
        hist_bk_level = p_ictx->fsmgr.sensor_fsm.black_level << black_level_shift;
#else
        return;
#endif
    }
    /* coverity[const]*/
    switch ( input ) {
    case 0:
        acamera_isp_metering_hist_1_black_00_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_1_black_01_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_1_black_10_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_1_black_11_write( cfg_offset, hist_bk_level );
        break;
    case 1:
        acamera_isp_metering_hist_2_black_00_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_2_black_01_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_2_black_10_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_2_black_11_write( cfg_offset, hist_bk_level );
        break;
    case 2:
        acamera_isp_metering_hist_3_black_00_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_3_black_01_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_3_black_10_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_3_black_11_write( cfg_offset, hist_bk_level );
        break;
    case 3:
        acamera_isp_metering_hist_4_black_00_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_4_black_01_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_4_black_10_write( cfg_offset, hist_bk_level );
        acamera_isp_metering_hist_4_black_11_write( cfg_offset, hist_bk_level );
        break;
    default:
        LOG( LOG_WARNING, "Could not find the input port so not setting black level for it." );
    }
    return;
}

void ae_update_histogram_geometry( histogram_fsm_const_ptr_t p_fsm, int input )
{
    acamera_cmd_sensor_info sensor_info;

#if defined( ISP_HAS_RAW_SCALER_FSM )
    uint8_t histogram_switch;
    const uint8_t raw_scaler_enable = get_context_param( ACAMERA_FSM2ICTX_PTR( p_fsm ), IMAGE_RAW_SCALER_ENABLE_ID_PARAM );

    const uint16_t raw_scaler_height = get_context_param( ACAMERA_FSM2ICTX_PTR( p_fsm ), IMAGE_RAW_SCALER_HEIGHT_ID_PARAM );
    const uint16_t raw_scaler_width = get_context_param( ACAMERA_FSM2ICTX_PTR( p_fsm ), IMAGE_RAW_SCALER_WIDTH_ID_PARAM );
#endif

    WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ), CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&sensor_info );

    const uint32_t cfg_offset = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base : PHY_ADDR_ISP;

    switch ( input ) {
    case 0:
#if defined( ISP_HAS_RAW_SCALER_FSM )
        histogram_switch = acamera_isp_pipeline_histogram1_isp_switch_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base );
        //only tap point 3 located after raw scaler
        if ( raw_scaler_enable && ( histogram_switch == ACAMERA_ISP_PIPELINE_HISTOGRAM1_ISP_SWITCH_AFTER_MESH_SHADING ) ) {
            acamera_isp_metering_hist_1_active_width_write( cfg_offset, raw_scaler_width );
            acamera_isp_metering_hist_1_active_height_write( cfg_offset, raw_scaler_height );
        } else {
            acamera_isp_metering_hist_1_active_width_write( cfg_offset, sensor_info.active_width );
            acamera_isp_metering_hist_1_active_height_write( cfg_offset, sensor_info.active_height );
        }
#else
        acamera_isp_metering_hist_1_active_width_write( cfg_offset, sensor_info.active_width );
        acamera_isp_metering_hist_1_active_height_write( cfg_offset, sensor_info.active_height );
#endif

        acamera_isp_metering_hist_1_rggb_start_write( cfg_offset, sensor_info.rggb_start );
#ifdef ACAMERA_ISP_METERING_HIST_1_CFA_PATTERN_DEFAULT
        acamera_isp_metering_hist_1_cfa_pattern_write( cfg_offset, sensor_info.cfa_pattern );
#endif
        break;
    case 1:
#if defined( ISP_HAS_RAW_SCALER_FSM )
        histogram_switch = acamera_isp_pipeline_histogram2_isp_switch_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base );
        //only tap point 3 located after raw scaler
        if ( raw_scaler_enable && ( histogram_switch == ACAMERA_ISP_PIPELINE_HISTOGRAM2_ISP_SWITCH_AFTER_MESH_SHADING ) ) {
            acamera_isp_metering_hist_2_active_width_write( cfg_offset, raw_scaler_width );
            acamera_isp_metering_hist_2_active_height_write( cfg_offset, raw_scaler_height );
        } else {
            acamera_isp_metering_hist_2_active_width_write( cfg_offset, sensor_info.active_width );
            acamera_isp_metering_hist_2_active_height_write( cfg_offset, sensor_info.active_height );
        }
#else
        acamera_isp_metering_hist_2_active_width_write( cfg_offset, sensor_info.active_width );
        acamera_isp_metering_hist_2_active_height_write( cfg_offset, sensor_info.active_height );
#endif
        acamera_isp_metering_hist_2_rggb_start_write( cfg_offset, sensor_info.rggb_start );
#ifdef ACAMERA_ISP_METERING_HIST_2_CFA_PATTERN_DEFAULT
        acamera_isp_metering_hist_2_cfa_pattern_write( cfg_offset, sensor_info.cfa_pattern );
#endif
        break;
    case 2:
#if defined( ISP_HAS_RAW_SCALER_FSM )
        histogram_switch = acamera_isp_pipeline_histogram3_isp_switch_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base );
        //only tap point 3 located after raw scaler
        if ( raw_scaler_enable && ( histogram_switch == ACAMERA_ISP_PIPELINE_HISTOGRAM3_ISP_SWITCH_AFTER_MESH_SHADING ) ) {
            acamera_isp_metering_hist_3_active_width_write( cfg_offset, raw_scaler_width );
            acamera_isp_metering_hist_3_active_height_write( cfg_offset, raw_scaler_height );
        } else {
            acamera_isp_metering_hist_3_active_width_write( cfg_offset, sensor_info.active_width );
            acamera_isp_metering_hist_3_active_height_write( cfg_offset, sensor_info.active_height );
        }
#else
        acamera_isp_metering_hist_3_active_width_write( cfg_offset, sensor_info.active_width );
        acamera_isp_metering_hist_3_active_height_write( cfg_offset, sensor_info.active_height );
#endif
        acamera_isp_metering_hist_3_rggb_start_write( cfg_offset, sensor_info.rggb_start );
#ifdef ACAMERA_ISP_METERING_HIST_3_CFA_PATTERN_DEFAULT
        acamera_isp_metering_hist_3_cfa_pattern_write( cfg_offset, sensor_info.cfa_pattern );
#endif
        break;
    case 3:
#if defined( ISP_HAS_RAW_SCALER_FSM )
        histogram_switch = acamera_isp_pipeline_histogram4_isp_switch_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base );
        //only tap point 3 located after raw scaler
        if ( raw_scaler_enable && ( histogram_switch == ACAMERA_ISP_PIPELINE_HISTOGRAM4_ISP_SWITCH_AFTER_MESH_SHADING ) ) {
            acamera_isp_metering_hist_4_active_width_write( cfg_offset, raw_scaler_width );
            acamera_isp_metering_hist_4_active_height_write( cfg_offset, raw_scaler_height );
        } else {
            acamera_isp_metering_hist_4_active_width_write( cfg_offset, sensor_info.active_width );
            acamera_isp_metering_hist_4_active_height_write( cfg_offset, sensor_info.active_height );
        }
#else
        acamera_isp_metering_hist_4_active_width_write( cfg_offset, sensor_info.active_width );
        acamera_isp_metering_hist_4_active_height_write( cfg_offset, sensor_info.active_height );
#endif
        acamera_isp_metering_hist_4_rggb_start_write( cfg_offset, sensor_info.rggb_start );
#ifdef ACAMERA_ISP_METERING_HIST_4_CFA_PATTERN_DEFAULT
        acamera_isp_metering_hist_4_cfa_pattern_write( cfg_offset, sensor_info.cfa_pattern );
#endif
        break;
    default:
        LOG( LOG_ERR, "Invalid metering hist input" );
        break;
    }
}

void ae_update_histogram_zone_weights( histogram_fsm_const_ptr_t p_fsm, int input )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    uint8_t ( *nodes_used_horiz_read )( uint32_t base );
    uint8_t ( *nodes_used_vert_read )( uint32_t base );

#if ACAMERA_ISP_ZONES_AWB_WEIGHT_ARRAY_DIM_1_SIZE == 1
    void ( *weight_table_write )( uint32_t base, uint32_t index, uint8_t data );
#else
    void ( *weight_table_write )( uint32_t base, uint32_t index1, uint32_t index2, uint8_t data );
#endif

    switch ( input ) {
    case 0:
        nodes_used_horiz_read = acamera_isp_metering_hist_1_nodes_used_horiz_read;
        nodes_used_vert_read = acamera_isp_metering_hist_1_nodes_used_vert_read;
        weight_table_write = acamera_isp_metering_hist_1_weight_table_write;
        break;
    case 1:
        nodes_used_horiz_read = acamera_isp_metering_hist_2_nodes_used_horiz_read;
        nodes_used_vert_read = acamera_isp_metering_hist_2_nodes_used_vert_read;
        weight_table_write = acamera_isp_metering_hist_2_weight_table_write;
        break;
    case 2:
        nodes_used_horiz_read = acamera_isp_metering_hist_3_nodes_used_horiz_read;
        nodes_used_vert_read = acamera_isp_metering_hist_3_nodes_used_vert_read;
        weight_table_write = acamera_isp_metering_hist_3_weight_table_write;
        break;
    case 3:
        nodes_used_horiz_read = acamera_isp_metering_hist_4_nodes_used_horiz_read;
        nodes_used_vert_read = acamera_isp_metering_hist_4_nodes_used_vert_read;
        weight_table_write = acamera_isp_metering_hist_4_weight_table_write;
        break;
    default:
        LOG( LOG_ERR, "Invalid metering hist input" );
        return;
    }

    const uint16_t horz_zones = nodes_used_horiz_read( p_ictx->settings.isp_base );
    const uint16_t vert_zones = nodes_used_vert_read( p_ictx->settings.isp_base );

    const uint16_t *ptr_ae_zone_whgh_h = calib_mgr_u16_lut_get( p_ictx->calib_mgr_data, CALIBRATION_AE_ZONE_WGHT_HOR );
    const uint16_t *ptr_ae_zone_whgh_v = calib_mgr_u16_lut_get( p_ictx->calib_mgr_data, CALIBRATION_AE_ZONE_WGHT_VER );

    const uint32_t ae_zone_wght_hor_len = calib_mgr_lut_len( p_ictx->calib_mgr_data, CALIBRATION_AE_ZONE_WGHT_HOR );
    const uint32_t ae_zone_wght_ver_len = calib_mgr_lut_len( p_ictx->calib_mgr_data, CALIBRATION_AE_ZONE_WGHT_VER );

    if ( ( horz_zones * vert_zones ) > ISP_METERING_HISTOGRAM_ZONES_MAX ) {
        LOG( LOG_CRIT, "AE zone weights update failed. Number of zones configured (%d * %d = %d) is out of range (%d)",
             horz_zones, vert_zones, ( horz_zones * vert_zones ), ISP_METERING_HISTOGRAM_ZONES_MAX );
        return;
    }

    if ( horz_zones > ae_zone_wght_hor_len ) {
        LOG( LOG_CRIT, "AE zone weight update failed. Nodes Used Horiz > ae_zone_wght_hor_len (%u > %u)", horz_zones, ae_zone_wght_hor_len );
        return;
    }

    if ( vert_zones > ae_zone_wght_ver_len ) {
        LOG( LOG_CRIT, "AE zone weight update failed. Nodes Used Vert > ae_zone_wght_ver_len (%u > %u)", vert_zones, ae_zone_wght_ver_len );
        return;
    }

    // NOTE: histogram can be on CDMA or not depends on where it is located
    const uint32_t base_addr = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base : PHY_ADDR_ISP;

    uint16_t x, y, hw_y = 0, hw_x = 0;
    for ( y = 0; y < vert_zones; y++ ) {

        const uint16_t coeff_y = ptr_ae_zone_whgh_v[y];

        for ( x = 0; x < horz_zones; x++ ) {

            const uint16_t coeff_x = ptr_ae_zone_whgh_h[x];

            uint8_t ae_coeff = ( coeff_x * coeff_y ) >> 4;

            if ( ae_coeff > 1 ) {
                ae_coeff--;
            }


#if ACAMERA_ISP_ZONES_AWB_WEIGHT_ARRAY_DIM_1_SIZE == 1
            weight_table_write( base_addr, ( hw_y * vert_zones ) + hw_x, ae_coeff );
#else
            weight_table_write( base_addr, hw_y, hw_x, ae_coeff );
#endif

            // Convert current zone configuration to HW coordinates
            // HW array write function expects zone coordinates in 15x15 format
            if ( ++hw_x >= ACAMERA_ISP_METERING_HIST_1_WEIGHT_TABLE_ARRAY_DIM_0_SIZE ) {
                hw_y++;
                hw_x = 0;
            }
        }
    }
}

/**
 * configure_histogram_neq_lut() - Configure histogram neq lut based on calibration.
 *
 * Histogram configuration is written directly to isp hw registers.
 * The position of histogram is set to be on the input port.
 *
 * This function has two implementation depending on the Morgan ISP revision.
 * Revision 0 has no NEQ LUT present in Histogram engine.
 *
 * Return: None
 */
// Called from hist_fsm via ae_hist_changed event.
void configure_histogram_neq_lut( histogram_fsm_const_ptr_t p_fsm, int input )
{
#if defined( ISP_HAS_DECOMPANDER_FSM )
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    const uint32_t cfg_offset = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base : PHY_ADDR_ISP;
    // TODO2: handle case when decompander is disabled on input_formatter
    if ( !calib_mgr_lut_exists( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_DECOMPANDER_CONTROL ) )
        return;

    const decompander_ctrl *p_decompander_ctrl = (const decompander_ctrl *)calib_mgr_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_DECOMPANDER_CONTROL );

    // If we enable here for BE_FS, BE_DECOMPANDER or BE_SHADING, we need to add input_port = 0;
    // TODO: should check on decompanding happening in inputFormatter or gammaFE.
    const uint8_t enable = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? 0 : p_decompander_ctrl->hist_neq_lut_enable;

#if MODEL_MODULE
    input = ISP_MODEL_INPUT_PORT_FOR_AE_UPDATE_HIST;
#endif

    if ( enable == 0 ) {
        LOG( LOG_INFO, "Hist Neq lut OFF" );
        /* coverity[const]*/
        switch ( input ) {
        case 0:
            acamera_isp_metering_hist_1_neq_lut_enable_write( cfg_offset, 0 );
            break;
        case 1:
            acamera_isp_metering_hist_2_neq_lut_enable_write( cfg_offset, 0 );
            break;
        case 2:
            acamera_isp_metering_hist_3_neq_lut_enable_write( cfg_offset, 0 );
            break;
        case 3:
            acamera_isp_metering_hist_4_neq_lut_enable_write( cfg_offset, 0 );
            break;
        }
    } else {
        LOG( LOG_INFO, "Hist Neq lut ON" );
#ifdef ACAMERA_ISP_METERING_HIST_1_NEQ_LUT_POSITION_DEFAULT
        const uint8_t hist_neq_lut_pos = p_decompander_ctrl->hist_neq_lut_pos;
#endif
        const histogram_neq_lut *ptr_look =
            (histogram_neq_lut *)calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NEQ_LUT );
        const uint32_t rows = calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_NEQ_LUT );
        int i;

        switch ( input ) {
        case 0:
#ifdef ACAMERA_ISP_METERING_HIST_1_NEQ_LUT_POSITION_DEFAULT
            acamera_isp_metering_hist_1_neq_lut_position_write( cfg_offset, hist_neq_lut_pos );
#endif
            acamera_isp_metering_hist_1_neq_lut_enable_write( cfg_offset, 1 );
            for ( i = 0; i < rows; i++ ) {
                acamera_isp_metering_hist_1_lut_x_write( cfg_offset, i, ptr_look[i].x );
                acamera_isp_metering_hist_1_lut_y_write( cfg_offset, i, ptr_look[i].y );
            }
            break;
        case 1:
#ifdef ACAMERA_ISP_METERING_HIST_1_NEQ_LUT_POSITION_DEFAULT
            acamera_isp_metering_hist_2_neq_lut_position_write( cfg_offset, hist_neq_lut_pos );
#endif
            acamera_isp_metering_hist_2_neq_lut_enable_write( cfg_offset, 1 );
            for ( i = 0; i < rows; i++ ) {
                acamera_isp_metering_hist_2_lut_x_write( cfg_offset, i, ptr_look[i].x );
                acamera_isp_metering_hist_2_lut_y_write( cfg_offset, i, ptr_look[i].y );
            }
            break;
        case 2:
#ifdef ACAMERA_ISP_METERING_HIST_1_NEQ_LUT_POSITION_DEFAULT
            acamera_isp_metering_hist_3_neq_lut_position_write( cfg_offset, hist_neq_lut_pos );
#endif
            acamera_isp_metering_hist_3_neq_lut_enable_write( cfg_offset, 1 );
            for ( i = 0; i < rows; i++ ) {
                acamera_isp_metering_hist_3_lut_x_write( cfg_offset, i, ptr_look[i].x );
                acamera_isp_metering_hist_3_lut_y_write( cfg_offset, i, ptr_look[i].y );
            }
            break;
        case 3:
#ifdef ACAMERA_ISP_METERING_HIST_1_NEQ_LUT_POSITION_DEFAULT
            acamera_isp_metering_hist_4_neq_lut_position_write( cfg_offset, hist_neq_lut_pos );
#endif
            acamera_isp_metering_hist_4_neq_lut_enable_write( cfg_offset, 1 );
            for ( i = 0; i < rows; i++ ) {
                acamera_isp_metering_hist_4_lut_x_write( cfg_offset, i, ptr_look[i].x );
                acamera_isp_metering_hist_4_lut_y_write( cfg_offset, i, ptr_look[i].y );
            }
            break;
        }
    }
#else
    LOG( LOG_WARNING, "Histogram NEQ LUT is not present in Morgan ISP Revision 0!" );
#endif
#endif // defined(ISP_HAS_DECOMPANDER_FSM)
}

// Called from hist_fsm via ae_reconfigure/sensor_ready events.
void histogram_configure_position( histogram_fsm_const_ptr_t p_fsm )
{
    uint8_t hist_tap_fe_be; // Backend
    uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;

    if ( ISP_HISTOGRAM_POSITION_IS_BE )
        hist_tap_fe_be = 2;
    else
        hist_tap_fe_be = 0;
    acamera_frontend_pipeline_frontend_histogram1_switch_write( PHY_ADDR_ISP, hist_tap_fe_be );
    acamera_frontend_pipeline_frontend_histogram2_switch_write( PHY_ADDR_ISP, hist_tap_fe_be );
    acamera_frontend_pipeline_frontend_histogram3_switch_write( PHY_ADDR_ISP, hist_tap_fe_be );
    acamera_frontend_pipeline_frontend_histogram4_switch_write( PHY_ADDR_ISP, hist_tap_fe_be );

    // Set to 1 for current context.
    acamera_isp_metering_hist_1_skip_x_write( isp_base, 1 );
    acamera_isp_metering_hist_2_skip_x_write( isp_base, 1 );
    acamera_isp_metering_hist_3_skip_x_write( isp_base, 1 );
    acamera_isp_metering_hist_4_skip_x_write( isp_base, 1 );

    if ( !ISP_HISTOGRAM_POSITION_IS_BE ) {
        // Don't do anything more if position == FE
        return;
    }

    switch ( ISP_HISTOGRAM_BE_POSITION ) {
    case AE_HISTOGRAM_TAP_AFTER_WDRGAIN:
        acamera_isp_pipeline_histogram1_isp_switch_write( isp_base, 0 );
        acamera_isp_pipeline_histogram2_isp_switch_write( isp_base, 0 );
        acamera_isp_pipeline_histogram3_isp_switch_write( isp_base, 0 );
        acamera_isp_pipeline_histogram4_isp_switch_write( isp_base, 0 );
        LOG( LOG_DEBUG, "AE_HISTOGRAM_TAP_AFTER_WDRGAIN" );
        break;
    case AE_HISTOGRAM_TAP_AFTER_FS:
        acamera_isp_pipeline_histogram1_isp_switch_write( isp_base, 1 );
        LOG( LOG_DEBUG, "AE_HISTOGRAM_TAP_AFTER_FS" );
        break;
    case AE_HISTOGRAM_TAP_AFTER_DECOMPANDER:
        acamera_isp_pipeline_histogram1_isp_switch_write( isp_base, 2 );
        LOG( LOG_DEBUG, "AE_HISTOGRAM_TAP_AFTER_DECOMPANDER" );
        break;
    case AE_HISTOGRAM_TAP_AFTER_SHADING:
        acamera_isp_pipeline_histogram1_isp_switch_write( isp_base, 3 );
        LOG( LOG_DEBUG, "AE_HISTOGRAM_TAP_AFTER_SHADING" );
        break;
    // Skip case AE_HISTOGRAM_TAP_AFTER_COMPAND: as hist needs to be squared (gamma_sqrt).
    default:
        LOG( LOG_ERR, "Unsupported hist be value %#x.", ISP_HISTOGRAM_BE_POSITION );
        break;
    }
}
