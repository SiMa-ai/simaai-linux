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

#include "system_stdlib.h"
#include "acamera_isp_core_settings.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "acamera_math.h"
#include "acamera_metering_mem_config.h"
#include "bitop.h"
#include "sbuf.h"
#include "util_crc16.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_AWB_MANUAL


////////////////////////////////////////////////////
// FSM function helpers

static void awb_coeffs_write( const AWB_fsm_t *p_fsm )
{
    acamera_isp_out_format_rgb2rgb_coef_b_1_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_fsm->awb_warming[0] );
    acamera_isp_out_format_rgb2rgb_coef_b_2_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_fsm->awb_warming[1] );
    acamera_isp_out_format_rgb2rgb_coef_b_3_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, p_fsm->awb_warming[2] );
}

static void awb_zone_weights_update( AWB_fsm_ptr_t p_fsm )
{
#ifdef ACAMERA_ISP_ZONES_AWB_WEIGHT_DEFAULT
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    const uint16_t horz_zones = acamera_isp_metering_awb_nodes_used_horiz_read( p_ictx->settings.isp_base );
    const uint16_t vert_zones = acamera_isp_metering_awb_nodes_used_vert_read( p_ictx->settings.isp_base );

    const uint16_t *ptr_awb_zone_wght_h = calib_mgr_u16_lut_get( p_ictx->calib_mgr_data, CALIBRATION_AWB_ZONE_WGHT_HOR );
    const uint16_t *ptr_awb_zone_wght_v = calib_mgr_u16_lut_get( p_ictx->calib_mgr_data, CALIBRATION_AWB_ZONE_WGHT_VER );

    const uint32_t awb_zone_wght_hor_len = calib_mgr_lut_len( p_ictx->calib_mgr_data, CALIBRATION_AWB_ZONE_WGHT_HOR );
    const uint32_t awb_zone_wght_ver_len = calib_mgr_lut_len( p_ictx->calib_mgr_data, CALIBRATION_AWB_ZONE_WGHT_VER );

    if ( ( horz_zones * vert_zones ) > ISP_METERING_HISTOGRAM_ZONES_MAX ) {
        LOG( LOG_CRIT, "AWB zone weights update failed. Number of zones configured (%d * %d = %d) is out of range (%d)",
             horz_zones, vert_zones, ( horz_zones * vert_zones ), ISP_METERING_HISTOGRAM_ZONES_MAX );
        return;
    }

    if ( horz_zones > awb_zone_wght_hor_len ) {
        LOG( LOG_CRIT, "AWB zone weight update failed. AWB Nodes Used Horiz > awb_zone_wght_hor_len (%u > %u)", horz_zones, awb_zone_wght_hor_len );
        return;
    }

    if ( vert_zones > awb_zone_wght_ver_len ) {
        LOG( LOG_CRIT, "AWB zone weight update failed. AWB Nodes Used Vert > awb_zone_wght_ver_len (%u > %u)", vert_zones, awb_zone_wght_ver_len );
        return;
    }

    // NOTE: histogram can be on CDMA or not depends on where it is located
    const uint32_t base_addr = ( ISP_HISTOGRAM_POSITION_IS_BE ) ? ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base : PHY_ADDR_ISP;

    uint16_t x, y, hw_y = 0, hw_x = 0;
    for ( y = 0; y < vert_zones; y++ ) {

        const uint16_t coeff_y = ptr_awb_zone_wght_v[y];

        for ( x = 0; x < horz_zones; x++ ) {

            const uint16_t coeff_x = ptr_awb_zone_wght_h[x];

            uint8_t ae_coeff = ( coeff_x * coeff_y ) >> 4;

            if ( ae_coeff > 1 ) {
                ae_coeff--;
            }

#if ACAMERA_ISP_ZONES_AWB_WEIGHT_ARRAY_DIM_1_SIZE == 1
            acamera_isp_zones_awb_weight_write( base_addr, ( hw_y * vert_zones ) + hw_x, ae_coeff );
#else
            acamera_isp_zones_awb_weight_write( base_addr, hw_y, hw_x, ae_coeff );
#endif
            // Convert current zone configuration to HW coordinates
            // HW array write function expects zone coordinates in 15x15 format
            if ( ++hw_x >= ACAMERA_ISP_ZONES_AWB_WEIGHT_ARRAY_DIM_0_SIZE ) {
                hw_y++;
                hw_x = 0;
            }
        }
    }
#else
    LOG( LOG_WARNING, "AWB does not have zone weights" );
#endif // ACAMERA_ISP_ZONES_AWB_WEIGHT_DEFAULT
}


////////////////////////////////////////////////////
// awb_update helper functions

// For CCM switching
static void awb_process_light_source( AWB_fsm_t *p_fsm )
{
    acamera_cmd_ccm_info ccm_info;
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
#ifdef AWB_PRINT_DEBUG
    LOG( LOG_DEBUG, "p_high: %d, light_source_candidate: %d, temperature_detected: %d.\n",
         p_fsm->p_high,
         p_fsm->light_source_candidate,
         p_fsm->temperature_detected );
#endif

    const int high_gain = ( get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM ) >= calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CCM_ONE_GAIN_THRESHOLD )[0] ) ? 1 : 0;

    // Get the CCM info before use and update it
    WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ), CMD_ID_CCM_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&ccm_info );

#if 0
    if ( p_fsm->p_high > 60) {
        ++p_fsm->detect_light_source_frames_count;
        if ( p_fsm->detect_light_source_frames_count >= p_fsm->switch_light_source_detect_frames_quantity ) {
#ifdef AWB_PRINT_DEBUG
            if ( ccm_info.light_source != AWB_LIGHT_SOURCE_D50) {
                LOG( LOG_DEBUG, "Light source is changed." );
                LOG( LOG_DEBUG, "p_high=%d, using AWB_LIGHT_SOURCE_D50.", p_fsm->p_high );
            }
#endif

            ccm_info.light_source_previous = ccm_info.light_source;
            ccm_info.light_source = p_fsm->light_source_candidate;
            ccm_info.light_source_ccm_previous = ccm_info.light_source_ccm;
            ccm_info.light_source_ccm = high_gain ? AWB_LIGHT_SOURCE_UNKNOWN : p_fsm->light_source_candidate; // For low light set ccm = I
            ccm_info.light_source_change_frames = p_fsm->switch_light_source_change_frames_quantity;
            ccm_info.light_source_change_frames_left = p_fsm->switch_light_source_change_frames_quantity;

            WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ),  CMD_ID_CCM_INFO, CMD_DIRECTION_SET, (uint32_t *)&ccm_info, NULL );
        }

    } else
#endif
    if ( p_fsm->light_source_detected == p_fsm->light_source_candidate ) {
        if ( ( p_fsm->light_source_candidate != ccm_info.light_source ) || ( high_gain && ccm_info.light_source_ccm != AWB_LIGHT_SOURCE_UNKNOWN ) || ( !high_gain && ccm_info.light_source_ccm == AWB_LIGHT_SOURCE_UNKNOWN ) ) {
            ++p_fsm->detect_light_source_frames_count;
            if ( p_fsm->detect_light_source_frames_count >= p_fsm->switch_light_source_detect_frames_quantity && !ccm_info.light_source_change_frames_left ) {
                ccm_info.light_source_previous = ccm_info.light_source;
                ccm_info.light_source = p_fsm->light_source_candidate;
                ccm_info.light_source_ccm_previous = ccm_info.light_source_ccm;
                ccm_info.light_source_ccm = high_gain ? AWB_LIGHT_SOURCE_UNKNOWN : p_fsm->light_source_candidate; // for low light set ccm = I
                ccm_info.light_source_change_frames = p_fsm->switch_light_source_change_frames_quantity;
                ccm_info.light_source_change_frames_left = p_fsm->switch_light_source_change_frames_quantity;
                WRAP_GENERAL_CMD( ACAMERA_FSM2ICTX_PTR( p_fsm ), CMD_ID_CCM_INFO, CMD_DIRECTION_SET, (uint32_t *)&ccm_info, NULL );
#ifdef AWB_PRINT_DEBUG
                // These are rarer so can print wherever they are fired (i.e. not dependent on ittcount)
                LOG( LOG_DEBUG, "Light source is changed" );
                if ( ccm_info.light_source == AWB_LIGHT_SOURCE_A )
                    LOG( LOG_DEBUG, "AWB_LIGHT_SOURCE_A" );
                if ( ccm_info.light_source == AWB_LIGHT_SOURCE_D40 )
                    LOG( LOG_DEBUG, "AWB_LIGHT_SOURCE_D40" );
                if ( ccm_info.light_source == AWB_LIGHT_SOURCE_D50 )
                    LOG( LOG_DEBUG, "AWB_LIGHT_SOURCE_D50" );
#endif
            }
        }
    } else {
        p_fsm->detect_light_source_frames_count = 0;
    }
    p_fsm->light_source_detected = p_fsm->light_source_candidate;
}

static void awb_write_values( AWB_fsm_t *p_fsm )
{
    /* This function drives the cooling warming effect according to colour temperature.
See kruithof curve for a reference and background theory about this functionality
Tuning luts default
CALIBRATION_AWB_WARMING_LS_A = {256,256,256}; // u4.8
CALIBRATION_AWB_WARMING_LS_D50= {256,256,256};// u4.8
CALIBRATION_AWB_WARMING_LS_D75= {256,256,256};// u4.8
awb_warming_cct={7500,6000,4700,2800};// in Kelvin  // AWB_colour_preference

| blue gain
|   .                   .   .
|       .           .
|           . .  . .
|       .             .
|Red. gain                      .
|.                          .
-----|------|------|--------|-------
    70000  6000    4700     2800    CCT in kelvin
*/

    int32_t temperature = p_fsm->temperature_detected;

    // Calibration table: AWB Colour preference.
    //int16_t awb_warming_cct[4] = {7500,6000,4700,2800};
    acamera_calib_mgr_entry_t *p_calimgr = ACAMERA_FSM2CM_PTR( p_fsm );
    const int16_t *awb_warming_cct = (const int16_t *)calib_mgr_u16_lut_get( p_calimgr, CALIBRATION_AWB_WARMING_CCT );

    const int16_t *awb_warming_A = (const int16_t *)calib_mgr_u16_lut_get( p_calimgr, CALIBRATION_AWB_WARMING_LS_A );
    const int16_t *awb_warming_D75 = (const int16_t *)calib_mgr_u16_lut_get( p_calimgr, CALIBRATION_AWB_WARMING_LS_D75 );
    const int16_t *awb_warming_D50 = (const int16_t *)calib_mgr_u16_lut_get( p_calimgr, CALIBRATION_AWB_WARMING_LS_D50 );

    int16_t m = 0;
    if ( temperature >= awb_warming_cct[1] ) {
        // High temp

        // RED
        m = ( awb_warming_D50[0] - awb_warming_D75[0] ) / ( ( awb_warming_cct[1] - awb_warming_cct[0] ) ?: 1 );
        if ( m == 0 ) {
            p_fsm->awb_warming[0] = awb_warming_D50[0] >> 8;
        } else {
            p_fsm->awb_warming[0] = ( m * ( temperature - awb_warming_cct[0] ) + awb_warming_D75[0] ) >> 8;
        }
        // GREEN
        m = ( awb_warming_D50[1] - awb_warming_D75[1] ) / ( ( awb_warming_cct[1] - awb_warming_cct[0] ) ?: 1 );
        if ( m == 0 ) {
            p_fsm->awb_warming[1] = awb_warming_D50[1] >> 8;
        } else {
            p_fsm->awb_warming[1] = ( m * ( temperature - awb_warming_cct[0] ) + awb_warming_D75[1] ) >> 8;
        }
        // BLUE
        m = ( awb_warming_D50[2] - awb_warming_D75[2] ) / ( ( awb_warming_cct[1] - awb_warming_cct[0] ) ?: 1 );
        if ( m == 0 ) {
            p_fsm->awb_warming[2] = awb_warming_D50[2] >> 8;
        } else {
            p_fsm->awb_warming[2] = ( m * ( temperature - awb_warming_cct[0] ) + awb_warming_D75[2] ) >> 8;
        }
        //        printf( "1 temp %d r %d g %d b %d\n",
        //                (int)temperature, (int)( p_fsm->awb_warming[0] ),
        //                (int)( p_fsm->awb_warming[1] ), (int)( p_fsm->awb_warming[2] ) );

    } else if ( temperature <= awb_warming_cct[2] ) {
        // Low temp

        // RED
        m = ( awb_warming_A[0] - awb_warming_D50[0] ) / ( ( awb_warming_cct[3] - awb_warming_cct[2] ) ?: 1 );
        if ( m == 0 ) {
            p_fsm->awb_warming[0] = awb_warming_D50[0] >> 8;
        } else {
            p_fsm->awb_warming[0] = ( m * ( temperature - awb_warming_cct[3] ) + awb_warming_A[0] ) >> 8;
        }
        // GREEN
        m = ( awb_warming_A[1] - awb_warming_D50[1] ) / ( ( awb_warming_cct[3] - awb_warming_cct[2] ) ?: 1 );
        if ( m == 0 ) {
            p_fsm->awb_warming[1] = awb_warming_D50[1] >> 8;
        } else {
            p_fsm->awb_warming[1] = ( m * ( temperature - awb_warming_cct[3] ) + awb_warming_A[1] ) >> 8;
        }
        // BLUE
        m = ( awb_warming_A[2] - awb_warming_D50[2] ) / ( ( awb_warming_cct[3] - awb_warming_cct[2] ) ?: 1 );
        if ( m == 0 ) {
            p_fsm->awb_warming[2] = awb_warming_D50[2] >> 8;
        } else {
            p_fsm->awb_warming[2] = ( m * ( temperature - awb_warming_cct[3] ) + awb_warming_A[2] ) >> 8;
        }
    } else {
        // Mid temp
        p_fsm->awb_warming[0] = awb_warming_D50[0] >> 8;
        p_fsm->awb_warming[1] = awb_warming_D50[1] >> 8;
        p_fsm->awb_warming[2] = awb_warming_D50[2] >> 8;
    }
}

// Perform normalisation.
static void awb_normalise( AWB_fsm_t *p_fsm )
{
    int32_t wb[4];
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    wb[0] = log2_fixed_to_fixed( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_STATIC_WB )[0], 8, LOG2_GAIN_SHIFT );
    wb[1] = log2_fixed_to_fixed( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_STATIC_WB )[1], 8, LOG2_GAIN_SHIFT );
    wb[2] = log2_fixed_to_fixed( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_STATIC_WB )[2], 8, LOG2_GAIN_SHIFT );
    wb[3] = log2_fixed_to_fixed( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_STATIC_WB )[3], 8, LOG2_GAIN_SHIFT );

    {
        wb[0] += log2_fixed_to_fixed( p_fsm->rg_coef, 8, LOG2_GAIN_SHIFT );
        wb[3] += log2_fixed_to_fixed( p_fsm->bg_coef, 8, LOG2_GAIN_SHIFT );
    }

    {
        int i;
        int32_t min_wb = wb[0];
        for ( i = 1; i < 4; ++i ) {
            int32_t _wb = wb[i];
            if ( min_wb > _wb ) {
                min_wb = _wb;
            }
        }

        acamera_cmd_sensor_info sensor_info;
        if ( WRAP_GENERAL_CMD( p_ictx, CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&sensor_info ) ) {
            return;
        }

        uint8_t isp_input_bits = sensor_info.sensor_output_bits;
        int32_t diff = ( isp_input_bits << LOG2_GAIN_SHIFT ) - log2_fixed_to_fixed( ( 1 << isp_input_bits ) - sensor_info.black_level, 0, LOG2_GAIN_SHIFT ) - min_wb;
        for ( i = 0; i < 4; ++i ) {
            int32_t _wb = wb[i] + diff;
            p_fsm->wb_log2[i] = _wb;
        }
    }
}

static void awb_postprocess( AWB_fsm_t *p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    /* For both auto mode and manual mode, we use the same variables to save the red/blue gains and CCT. */
    p_fsm->rg_coef = get_context_param( p_ictx, SYSTEM_AWB_RED_GAIN_PARAM );
    p_fsm->bg_coef = get_context_param( p_ictx, SYSTEM_AWB_BLUE_GAIN_PARAM );
    p_fsm->temperature_detected = get_context_param( p_ictx, SYSTEM_AWB_CCT_PARAM );

    awb_process_light_source( p_fsm );
    awb_write_values( p_fsm );
    awb_normalise( p_fsm );
}

////////////////////////////////////////////////////
// FSM event handlers

void awb_init( AWB_fsm_t *p_fsm )
{
    LOG( LOG_INFO, "awb_init E" );

    // Initial AWB (rg, bg) is the identity.
    p_fsm->rg_coef = 0x100;
    p_fsm->bg_coef = 0x100;

    p_fsm->wb_log2[0] = 0;
    p_fsm->wb_log2[1] = 0;
    p_fsm->wb_log2[2] = 0;
    p_fsm->wb_log2[3] = 0;

    p_fsm->repeat_irq_mask = BIT( ACAMERA_IRQ_AWB_STATS );
    AWB_request_interrupt( p_fsm, p_fsm->repeat_irq_mask );

    LOG( LOG_INFO, "awb_init X" );
}

void awb_config( AWB_fsm_t *p_fsm )
{
    // Set the default awb values
    if ( MAX_AWB_ZONES < acamera_isp_metering_awb_nodes_used_horiz_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base ) * acamera_isp_metering_awb_nodes_used_vert_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base ) ) {
        LOG( LOG_CRIT, "MAX_AWB_ZONES is less than hardware reported zones" );
    }

    // awb_warming will not be reloaded on reload_calibration because it takes calibration for D50 as a starting value.
    p_fsm->awb_warming[0] = (int32_t)calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_AWB_WARMING_LS_D50 )[0];
    p_fsm->awb_warming[1] = (int32_t)calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_AWB_WARMING_LS_D50 )[1];
    p_fsm->awb_warming[2] = (int32_t)calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_AWB_WARMING_LS_D50 )[2];

    awb_coeffs_write( p_fsm );
}

void awb_reload_calibration( AWB_fsm_t *p_fsm )
{
    // Set the min/max temperatures and their gains:
    if ( ( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_TEMP )[0] != 0 ) && ( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_TEMP )[calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_TEMP ) - 1] != 0 ) && ( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_RG_POS_CALC )[0] != 0 ) && ( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_BG_POS_CALC )[0] != 0 ) && ( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_RG_POS_CALC )[calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_RG_POS_CALC ) - 1] != 0 ) && ( calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_BG_POS_CALC )[calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_BG_POS_CALC ) - 1] != 0 ) ) {
        p_fsm->min_temp = 1000000 / calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_TEMP )[calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_TEMP ) - 1];            // division by zero is checked
        p_fsm->max_temp = 1000000 / calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_COLOR_TEMP )[0];                                                                                       // division by zero is checked
        p_fsm->max_temp_rg = U16_MAX / calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_RG_POS_CALC )[0];                                                                                // division by zero is checked
        p_fsm->max_temp_bg = U16_MAX / calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_BG_POS_CALC )[0];                                                                                // division by zero is checked
        p_fsm->min_temp_rg = U16_MAX / calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_RG_POS_CALC )[calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_RG_POS_CALC ) - 1]; // division by zero is checked
        p_fsm->min_temp_bg = U16_MAX / calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_BG_POS_CALC )[calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_CT_BG_POS_CALC ) - 1]; // division by zero is checked
    } else {
        LOG( LOG_WARNING, "AVOIDED DIVISION BY ZERO" );
    }

    awb_zone_weights_update( p_fsm );
}

void awb_update_algo( AWB_fsm_t *p_fsm )
{
    awb_postprocess( p_fsm );
    /* There is yet no FSM waiting this event.
     * FSMs such as CCM can have algo_update state separately from hw_update.
     */
    //    fsm_raise_event( p_fsm, event_id_algo_awb_done );
}

void awb_update_hw( AWB_fsm_t *p_fsm )
{
    awb_coeffs_write( p_fsm );
}

void awb_deinit( AWB_fsm_ptr_t p_fsm )
{
}


////////////////////////////////////////////////////
// Hardware Interrupt Handler
#define AWB_STATS_SIZE ( 225 * 2 ) /*225 records of 64 bits*/
#define AWB_STATS_META_SIZE ( 4 )  /*meta data for statistics */
#define AWB_STATS_CRC_INDEX ( AWB_STATS_SIZE + AWB_STATS_META_SIZE - 1 )

#if STATISTICS_BUFFER_DATA_LOCALLY
static uint32_t l_statistics_data[AWB_STATS_SIZE + AWB_STATS_META_SIZE];
#endif /* STATISTICS_BUFFER_DATA_LOCALLY */

static uint32_t awb_get_data( const uint32_t base_address, const size_t index )
{
#if STATISTICS_BUFFER_DATA_LOCALLY
    (void)base_address;
    return l_statistics_data[index];
#else
    return system_isp_read_32( base_address +
                               ACAMERA_METERING_MEM_BASE_ADDR +
                               ISP_METERING_OFFSET_AWB +
                               ( index << 2 ) );
#endif /*STATISTICS_BUFFER_DATA_LOCALLY*/
}

static uint16_t awb_get_crc( const uint32_t base_address )
{
    uint16_t crc16 = 0xffff;
    size_t i;
    for ( i = 0; i < ( AWB_STATS_SIZE + AWB_STATS_META_SIZE - 1 ); ++i ) {
        const uint32_t data = awb_get_data( base_address, i );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 24, 8 ) );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 16, 8 ) );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 8, 8 ) );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 0, 8 ) );
    }

    return crc16;
}

static int awb_is_crc_valid( const uint32_t base_address )
{
    const uint32_t crc16 = awb_get_crc( base_address );
    /* The last entry in in a histogram set is the crc. */
    const uint32_t crc_hw = awb_get_data( base_address, AWB_STATS_CRC_INDEX );

    if ( crc16 != crc_hw ) {
        LOG( LOG_ERR, "No CRC match for awb 0x%x read, 0x%x calced", crc_hw, crc16 );
        return 0;
    }

    return 1;
}

static int awb_read_statistics( AWB_fsm_t *p_fsm )
{
    const uint32_t is_manual = get_context_param( p_fsm->p_fsmgr->p_ictx, SYSTEM_MANUAL_AWB_PARAM );
    sbuf_awb_t *p_sbuf_awb_stats = NULL;
    struct sbuf_item sbuf;

    uint32_t fw_id = ACAMERA_FSM_GET_FW_ID( p_fsm );

    system_memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_type = SBUF_TYPE_AWB;
    sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;

    if ( sbuf_get_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to get sbuf, return." );
        return -2;
    }

    p_sbuf_awb_stats = (sbuf_awb_t *)sbuf.buf_base;
    LOG( LOG_DEBUG, "Get sbuf ok, idx: %u, status: %u, addr: %p.", sbuf.buf_idx, sbuf.buf_status, sbuf.buf_base );

    if ( !is_manual ) {
        // Auto white balance statistics starts at address 1856 and contains up to 225 records of 64 bits each followed
        // by five 32-bit fields:
        // - Global R/G average (fix point 4.8 format)
        // - Global B/G average (fix point 4.8 format)
        // - Total averaged pixel count
        // - Frame counter
        // - CRC16
        // Each 64 bit record contains 2 16 bit fields corresponding to average R/G, B/G (fix point 4.8 format) in the
        // zone followed by 32-bit field for averaged pixel count.
        const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;

#if STATISTICS_BUFFER_DATA_LOCALLY

        // This is a large IO operation which also blocks spinlocks accessing the
        // IO. We read all of the stats data to be able to checksum it but only some
        // of it will be used.
        system_isp_mem_read( l_statistics_data,
                             isp_base + ACAMERA_METERING_MEM_BASE_ADDR + ISP_METERING_OFFSET_AWB,
                             AWB_STATS_SIZE + AWB_STATS_META_SIZE,
                             sizeof( uint32_t ) );
#endif /* STATISTICS_BUFFER_DATA_LOCALLY */

        if ( !awb_is_crc_valid( isp_base ) ) {
            /* No further processing. */
            /* Read done, set the buffer back for future using. */
            sbuf.buf_status = SBUF_STATUS_DATA_DONE;
            if ( sbuf_set_item( fw_id, &sbuf ) ) {
                LOG( LOG_ERR, "Error: Failed to set sbuf, return." );
                return -2;
            }
            return -1;
        }


        // Process the awb hardware values.
        size_t awb_record;
        p_fsm->sum = 0;
        p_fsm->curr_AWB_ZONES = acamera_isp_metering_awb_nodes_used_horiz_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base ) * acamera_isp_metering_awb_nodes_used_vert_read( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base );
        p_sbuf_awb_stats->curr_AWB_ZONES = p_fsm->curr_AWB_ZONES;

        for ( awb_record = 0; awb_record < p_fsm->curr_AWB_ZONES; ++awb_record ) {
            const uint32_t data = awb_get_data( isp_base, awb_record << 1 );
            // What we get from HW is G/R. It is also programmable in the latest
            // HW.AWB_STATS_MODE=0-->G/R and AWB_STATS_MODE=1-->R/G. rg_coef is
            // actually R_gain applied to R Pixels. Since we get (G * G_gain) / (R *
            // R_gain) from HW, we multiply by the gain rg_coef to negate its
            // effect.
            uint16_t irg = BF_GET( data, 0, 16 );
            uint16_t ibg = BF_GET( data, 16, 16 );

            irg = ( irg * ( p_fsm->rg_coef ) ) >> 8;
            ibg = ( ibg * ( p_fsm->bg_coef ) ) >> 8;

            irg = ( irg == 0 ) ? 1 : irg;
            ibg = ( ibg == 0 ) ? 1 : ibg;

            p_sbuf_awb_stats->stats_data[awb_record].rg = U16_MAX / irg;
            p_sbuf_awb_stats->stats_data[awb_record].bg = U16_MAX / ibg;
            p_sbuf_awb_stats->stats_data[awb_record].sum = awb_get_data( isp_base, ( awb_record << 1 ) + 1 );
            p_fsm->sum += p_sbuf_awb_stats->stats_data[awb_record].sum;
        }
    }

    /* Read done, set the buffer back for future using. */
    sbuf.buf_status = SBUF_STATUS_DATA_DONE;

    if ( sbuf_set_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to set sbuf, return." );
        return -2;
    }

    LOG( LOG_DEBUG, "Set sbuf ok, idx: %u, status: %u, addr: %p.", sbuf.buf_idx, sbuf.buf_status, sbuf.buf_base );
    return 0;
}

void AWB_fsm_process_interrupt( const AWB_fsm_t *p_fsm, uint8_t irq_event )
{
    switch ( irq_event ) {
    case ACAMERA_IRQ_AWB_STATS: {
        // Run update_algorithm in manual mode. The algorithm is still being skipped,
        // and so is the actual stats read in awb_read_statistics as well.
        // We need to avoid sbuf issues.
        const int rc = awb_read_statistics( (AWB_fsm_t *)p_fsm );
        if ( rc == 0 ) {
            fsm_raise_event( p_fsm, event_id_isphw_stats_ready_awb );
        } else {
            LOG( LOG_ERR, "Failed to read statistics, error: %d", rc );
        }
    } break;
    }
}


////////////////////////////////////////////////////
// General router command handler

void awb_get_info( AWB_fsm_t *p_fsm, acamera_cmd_wb_info *p_info )
{
    system_memcpy( p_info->wb_log2, p_fsm->wb_log2, sizeof( p_info->wb_log2 ) );
    p_info->temperature_detected = p_fsm->temperature_detected;
    p_info->p_high = p_fsm->p_high;
    p_info->light_source_candidate = p_fsm->light_source_candidate;
}
