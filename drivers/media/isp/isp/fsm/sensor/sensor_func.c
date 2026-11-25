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

#include "system_timer.h"
#include "system_types.h"
#include "acamera_command_api.h"
#include "acamera_frame_stream_api.h"
#include "acamera_frontend_config.h"
#include "acamera_isp_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "acamera_math.h"
#include "acamera_configuration.h"
#if V4L2_INTERFACE_BUILD
#include <linux/videodev2.h>
#include "isp-sensor-subdev.h"
#endif

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_SENSOR

#define OFFSET_BLACK_DEFAULT 0xF0000

/* Generous sensor AE/gain limits; the IPA constrains to the real sensor. */
#define SENSOR_AGAIN_LOG2_MAX               ( 4096 << LOG2_GAIN_SHIFT )
#define SENSOR_DGAIN_LOG2_MAX               ( 4096 << LOG2_GAIN_SHIFT )
#define SENSOR_AGAIN_ACCURACY               ( 1 << ( LOG2_GAIN_SHIFT - 2 ) )
#define SENSOR_INTEGRATION_TIME_MIN_DEF     1
#define SENSOR_INTEGRATION_TIME_MAX_DEF     65535
#define SENSOR_INTEGRATION_TIME_APPLY_DELAY_DEF 1
#define SENSOR_LINES_PER_SECOND_DEF         ( 900000000u / 1100u )

#if V4L2_INTERFACE_BUILD
/* The bayer enum must match the ISP TOP rggb_start HW field encoding. */
static_assert( ISP_BAYER_RGGB == ACAMERA_ISP_TOP_RGGB_START_R_GR_GB_B );
static_assert( ISP_BAYER_GRBG == ACAMERA_ISP_TOP_RGGB_START_GR_R_B_GB );
static_assert( ISP_BAYER_GBRG == ACAMERA_ISP_TOP_RGGB_START_GB_B_R_GR );
static_assert( ISP_BAYER_BGGR == ACAMERA_ISP_TOP_RGGB_START_B_GB_GR_R );

/* fourcc -> {rggb_start (enum isp_bayer_order), bits}. */
static int fourcc_to_bayer( uint32_t fourcc, uint8_t *rggb_start, uint8_t *bits )
{
    switch ( fourcc ) {
    case V4L2_PIX_FMT_SRGGB8:  *rggb_start = ISP_BAYER_RGGB; *bits = 8;  return 0;
    case V4L2_PIX_FMT_SGRBG8:  *rggb_start = ISP_BAYER_GRBG; *bits = 8;  return 0;
    case V4L2_PIX_FMT_SGBRG8:  *rggb_start = ISP_BAYER_GBRG; *bits = 8;  return 0;
    case V4L2_PIX_FMT_SBGGR8:  *rggb_start = ISP_BAYER_BGGR; *bits = 8;  return 0;
    case V4L2_PIX_FMT_SRGGB10: *rggb_start = ISP_BAYER_RGGB; *bits = 10; return 0;
    case V4L2_PIX_FMT_SGRBG10: *rggb_start = ISP_BAYER_GRBG; *bits = 10; return 0;
    case V4L2_PIX_FMT_SGBRG10: *rggb_start = ISP_BAYER_GBRG; *bits = 10; return 0;
    case V4L2_PIX_FMT_SBGGR10: *rggb_start = ISP_BAYER_BGGR; *bits = 10; return 0;
    case V4L2_PIX_FMT_SRGGB12: *rggb_start = ISP_BAYER_RGGB; *bits = 12; return 0;
    case V4L2_PIX_FMT_SGRBG12: *rggb_start = ISP_BAYER_GRBG; *bits = 12; return 0;
    case V4L2_PIX_FMT_SGBRG12: *rggb_start = ISP_BAYER_GBRG; *bits = 12; return 0;
    case V4L2_PIX_FMT_SBGGR12: *rggb_start = ISP_BAYER_BGGR; *bits = 12; return 0;
    case V4L2_PIX_FMT_SRGGB14: *rggb_start = ISP_BAYER_RGGB; *bits = 14; return 0;
    case V4L2_PIX_FMT_SGRBG14: *rggb_start = ISP_BAYER_GRBG; *bits = 14; return 0;
    case V4L2_PIX_FMT_SGBRG14: *rggb_start = ISP_BAYER_GBRG; *bits = 14; return 0;
    case V4L2_PIX_FMT_SBGGR14: *rggb_start = ISP_BAYER_BGGR; *bits = 14; return 0;
    case V4L2_PIX_FMT_SRGGB16: *rggb_start = ISP_BAYER_RGGB; *bits = 16; return 0;
    case V4L2_PIX_FMT_SGRBG16: *rggb_start = ISP_BAYER_GRBG; *bits = 16; return 0;
    case V4L2_PIX_FMT_SGBRG16: *rggb_start = ISP_BAYER_GBRG; *bits = 16; return 0;
    case V4L2_PIX_FMT_SBGGR16: *rggb_start = ISP_BAYER_BGGR; *bits = 16; return 0;
    default: return -1;
    }
}
#endif

// FSM event handler

void sensor_init( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    LOG_CAT( ALOG_CAT_SENSOR, LOG_INFO, "%s ctx %u E", __FUNCTION__, p_ictx->context_id );

    // Single mode; resolution/format arrive at sensor_config from S_FMT.
    override_context_param( p_ictx, SENSOR_PRESET_PARAM, 0 );
    override_context_param( p_ictx, SENSOR_SUPPORTED_PRESETS_PARAM, 1 );

    acamera_isp_offset_black_00_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );
    acamera_isp_offset_black_01_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );
    acamera_isp_offset_black_10_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );
    acamera_isp_offset_black_11_write( p_ictx->settings.isp_base, OFFSET_BLACK_DEFAULT );

    acamera_isp_pipeline_bypass_sensor_offset_linear_write( p_ictx->settings.isp_base, 0 );
}

void sensor_config( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    /* Built here and published only once it describes a mode we actually
     * read. Writing straight into p_fsm->general_sensor_info would destroy
     * the previous description at the memset below, before anything has
     * been read that could replace it. */
    acamera_cmd_sensor_info candidate;
    acamera_cmd_sensor_info *gsi = &candidate;
    int available_inputs = 0;

    LOG_CAT( ALOG_CAT_SENSOR, LOG_INFO, "%s ctx %d E", __FUNCTION__, p_ictx->context_id );

    WRAP_GENERAL_CMD( p_ictx, CMD_ID_MCFE_GET_SLOT_AVAIL_INPUTS, CMD_DIRECTION_GET, NULL, &available_inputs );
    if ( available_inputs < 1 ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "No free MCFE input port (available: %d).", available_inputs );
        return;
    }

    memset( gsi, 0, sizeof( *gsi ) );

    /* AE/gain clamps for the cmos HW programmer; the IPA constrains the
     * actual gains/exposure to the real sensor. */
    gsi->again_log2_max = SENSOR_AGAIN_LOG2_MAX;
    gsi->dgain_log2_max = SENSOR_DGAIN_LOG2_MAX;
    gsi->again_accuracy = SENSOR_AGAIN_ACCURACY;
    gsi->integration_time_min = SENSOR_INTEGRATION_TIME_MIN_DEF;
    gsi->integration_time_max = SENSOR_INTEGRATION_TIME_MAX_DEF;
    gsi->integration_time_limit = SENSOR_INTEGRATION_TIME_MAX_DEF;
    gsi->integration_time_apply_delay = SENSOR_INTEGRATION_TIME_APPLY_DELAY_DEF;
    gsi->lines_per_second = SENSOR_LINES_PER_SECOND_DEF;
    gsi->sensor_exp_number = 1;
    gsi->sensor_output_bits = 12;
    gsi->current_sensor_mode.wdr_mode = WDR_MODE_LINEAR;
    gsi->current_sensor_mode.exposures = 1;
    gsi->current_sensor_mode.num_channels = 1;

#if V4L2_INTERFACE_BUILD
    /* Pipeline-critical info: real subdev (live), else the stashed S_FMT
     * (file replay / sidecar). Never the legacy mode table. */
    {
        struct isp_real_sensor_info rinfo;
        uint32_t sfmt_w = get_context_param( p_ictx, SENSOR_WIDTH_PARAM );
        uint32_t sfmt_h = get_context_param( p_ictx, SENSOR_HEIGHT_PARAM );
        uint32_t fourcc = get_context_param( p_ictx, SENSOR_INFO_PIXEL_FORMAT_PARAM );
        uint8_t rggb, bits;

        const int query = isp_sensor_subdev_query( p_ictx->context_id, sfmt_w,
                                                   sfmt_h, &rinfo );

        if ( query == 0 ) {
            isp_sensor_info_apply( gsi, &rinfo );
        } else if ( query == -ENODEV && sfmt_w && sfmt_h &&
                    fourcc_to_bayer( fourcc, &rggb, &bits ) == 0 ) {
            /* No sensor subdev at all, so this is file replay and the
             * stashed S_FMT is the only description there is. */
            gsi->active_width = sfmt_w & ~31u; /* 32-px align for ISP stride */
            gsi->active_height = sfmt_h;
            gsi->total_width = sfmt_w;
            gsi->total_height = sfmt_h;
            gsi->rggb_start = rggb;
            gsi->sensor_output_bits = bits;
        } else {
            /* A sensor that is there but would not answer. The S_FMT says
             * nothing about the WDR mode or the exposure count, so the
             * replay branch would report a three-exposure stream as linear
             * and the decompander would never run. Publish nothing: the
             * candidate holds only defaults and a zero-sized mode, and what
             * the FSM already holds is at worst stale, which is better. */
            LOG_CAT( ALOG_CAT_GENERAL, LOG_CRIT,
                     "ctx %d: sensor query failed (%d); keeping the previous "
                     "mode description",
                     p_ictx->context_id, query );
            return;
        }
    }
#endif

    gsi->video_port_id = p_ictx->context_id;
    /* What a pixel represents, which is the wire width unless the sensor
     * companded a wider linearisation down onto it. */
    if ( !gsi->current_sensor_mode.channel_info.exposure_max_bit_width )
        gsi->current_sensor_mode.channel_info.exposure_max_bit_width = gsi->sensor_output_bits;
    gsi->current_sensor_mode.resolution.width = gsi->active_width;
    gsi->current_sensor_mode.resolution.height = gsi->active_height;

    /* Settled: publish it, then derive the context params from the same
     * snapshot so the two cannot disagree. */
    p_fsm->general_sensor_info = candidate;

    override_context_param( p_ictx, SENSOR_HEIGHT_PARAM, gsi->active_height );
    override_context_param( p_ictx, SENSOR_WIDTH_PARAM, gsi->active_width );
    if ( gsi->total_height )
        override_context_param( p_ictx, SENSOR_FPS_PARAM, ( gsi->lines_per_second << 8 ) / gsi->total_height );
    override_context_param( p_ictx, SENSOR_WDR_MODE_PARAM, gsi->current_sensor_mode.wdr_mode );
    override_context_param( p_ictx, SENSOR_EXPOSURES_PARAM, gsi->current_sensor_mode.exposures );
    override_context_param( p_ictx, SENSOR_SUPPORTED_PRESETS_PARAM, 1 );
    override_context_param( p_ictx, SENSOR_INTEGRATION_TIME_MIN_PARAM, gsi->integration_time_min );
    override_context_param( p_ictx, SENSOR_INTEGRATION_TIME_LIMIT_PARAM, gsi->integration_time_limit );
}

void sensor_ready( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    uint32_t ctx_id = p_ictx->context_id;

    if ( get_context_param( p_ictx, SENSOR_STREAMING_PARAM ) == 0 ) {
#if ( ISP_RTL_VERSION_R == 0 )
        // We need a delay between sensors starts,
        // otherwise histograms comes at the same time and on R0 only one is served, resulting in AE not working.
        // Problem only present on split_architecture because of mcfe_fsm starting sensors at the same time.
        system_timer_usleep( 7000 );
#endif
        LOG_CAT( ALOG_CAT_SENSOR, LOG_INFO, "Start streaming (pos = %u)", ctx_id );

        // The real sensor is streamed by its V4L2 driver; just track state.
        override_context_param( p_ictx, SENSOR_STREAMING_PARAM, 1 );
    }
}

void sensor_stopped( sensor_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    uint32_t ctx_id = p_ictx->context_id;

    if ( get_context_param( p_ictx, SENSOR_STREAMING_PARAM ) == 1 ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_INFO, "Stop streaming (pos = %u)", ctx_id );

        // The real sensor is stopped by its V4L2 driver; just track state.
        override_context_param( p_ictx, SENSOR_STREAMING_PARAM, 0 );
    }
}

/* The depth p_fsm->black_level is expressed in. The tuning records the depth
 * its black-level LUT was authored at; Native WDR is the exception, where the
 * sensor already delivers samples at its own operating width. Every consumer of
 * the pedestal - the register writes below, and everything reached through
 * general_sensor_info - resolves the domain through this one place. */
uint32_t sensor_black_level_bits_of( acamera_calib_mgr_entry_t *p_cm, uint32_t wdr_mode,
                                    uint32_t operating_bits )
{
    /* Native WDR is not always the ISP's input maximum. The on-chip merge keeps
     * long-exposure units at the bottom of a wider container, so the pedestal is
     * not a fixed fraction of full scale and only the operating depth puts the
     * subtraction where the data actually sits. AR0822 two-exposure eHDR
     * operates at 16 bits; aligning it by 20 is a factor of sixteen out. */
    if ( wdr_mode == WDR_MODE_NATIVE && operating_bits >= 8u &&
         operating_bits <= ISP_INPUT_PORT_MAX_BIT_WIDTH )
        return operating_bits;

    return calib_mgr_calibration_bit_depth( p_cm );
}

static uint32_t sensor_black_level_bits( sensor_fsm_ptr_t p_fsm )
{
    return sensor_black_level_bits_of(
        ACAMERA_FSM2CM_PTR( p_fsm ),
        get_context_param( ACAMERA_FSM2ICTX_PTR( p_fsm ), SENSOR_WDR_MODE_PARAM ),
        p_fsm->general_sensor_info.current_sensor_mode.channel_info.exposure_max_bit_width );
}

static uint32_t get_exposure_bit_width_by_id( sensor_fsm_ptr_t p_fsm, uint8_t id )
{
    if ( id < p_fsm->general_sensor_info.current_sensor_mode.num_channels ) {
        return p_fsm->general_sensor_info.sensor_output_bits;
    }

    return 0;
}

void sensor_update_hw( sensor_fsm_ptr_t p_fsm )
{
    int stub = 0;
    exposure_set_t exp;

    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    WRAP_GENERAL_CMD( p_ictx, CMD_ID_FRAME_EXPOSURE_SET, CMD_DIRECTION_GET, &stub, &exp );
    const uint16_t again_log2 = ( uint16_t )( exp.info.again_log2 >> ( LOG2_GAIN_SHIFT - 5 ) ); // Makes again in format log2( gain ) * 32.

    const uint32_t ldr_gain_log2 = get_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM );

    const uint32_t idx_r = MODALIX_ISP_CALIB_BLACK_LEVEL_R;
    const uint32_t idx_b = MODALIX_ISP_CALIB_BLACK_LEVEL_B;
    const uint32_t idx_gr = MODALIX_ISP_CALIB_BLACK_LEVEL_GR;
    const uint32_t idx_gb = MODALIX_ISP_CALIB_BLACK_LEVEL_GB;

    const uint32_t bl_r = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ) );
    const uint32_t bl_b = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ) );
    const uint32_t bl_gr = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ) );
    const uint32_t bl_gb = calc_modulation_u16( again_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ) );


    uint32_t bl_r_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_r ) );
    uint32_t bl_b_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_b ) );
    uint32_t bl_gr_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gr ) );
    uint32_t bl_gb_low = calc_modulation_u16( ldr_gain_log2, calib_mgr_mod16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ), calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx_gb ) );

    const uint32_t wdr_mode = get_context_param( p_ictx, SENSOR_WDR_MODE_PARAM );

    // Update black-level FSM variable with average black level.
    p_fsm->black_level = ( bl_r + bl_gr + bl_gb + bl_b ) >> 2;

    /* Clamp here, where the pedestal is produced, not only where it is
     * published. The register writes below shift it left and are masked or
     * truncated by the register width, so a pedestal at or above full scale
     * silently reaches hardware as zero - no pedestal at all - while userspace
     * is handed the clamped value. Both must see the same number. */
    {
        const uint32_t bl_full_scale = 1u << sensor_black_level_bits( p_fsm );

        /* The per-channel pedestals are written to offset_black_00..11, which
         * mask to 24 bits, so they wrap to zero on the same misdeclared depth
         * as the average does. Clamp them against the same full scale. */
        bl_r_low = min( bl_r_low, bl_full_scale - 1u );
        bl_gr_low = min( bl_gr_low, bl_full_scale - 1u );
        bl_gb_low = min( bl_gb_low, bl_full_scale - 1u );
        bl_b_low = min( bl_b_low, bl_full_scale - 1u );

        if ( p_fsm->black_level >= bl_full_scale ) {
            LOG_CAT_SAMPLED( ALOG_CAT_SENSOR, LOG_ERR,
                             "Black level %u exceeds the %u-bit full scale; check MODALIX_ISP_CALIB_CALIBRATION_BIT_DEPTH in the tuning.",
                             p_fsm->black_level, sensor_black_level_bits( p_fsm ) );
            p_fsm->black_level = bl_full_scale - 1;
        }
    }

    /* Align by the depth the pedestal is expressed in, not the operating depth:
     * the pedestal is a fixed canonical value, so it stays bit-depth-independent
     * (12-bit unchanged; sub-12-bit no longer over-subtracted). */
    const uint32_t black_level_bits = sensor_black_level_bits( p_fsm );
    const uint32_t black_level_shift_dg = ISP_DIGITAL_GAIN_INPUT_BIT_WIDTH - black_level_bits;
    const uint32_t black_level_dg = p_fsm->black_level << black_level_shift_dg;

    // Coefficient calculation depends on wdr_mode.
    if ( ( wdr_mode == WDR_MODE_FS_LIN || wdr_mode == WDR_MODE_FS_HDR ) && ( get_context_param( p_ictx, ISP_MODULES_MANUAL_FRAME_STITCH_PARAM ) == 0 ) ) {

        uint8_t i;
        for ( i = 0; i < p_fsm->general_sensor_info.current_sensor_mode.exposures; ++i ) {
            const uint32_t exposure_bit_width = get_exposure_bit_width_by_id( p_fsm, i );
            if ( !exposure_bit_width ) {
                LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Exposure (id: %u) bit-width is set to zero, check sensor configuration.", i );
                break;
            }

            /* black_level is in the calibration reference domain, so align the
             * stitch stage by the same depth as the linear path above. */
            const int32_t black_level_stitch_shift_fs = ISP_FRAME_STITCH_INPUT_BIT_WIDTH - (int32_t)black_level_bits;
            if ( black_level_stitch_shift_fs < 0 ) {
                LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Frame stitch black level shift is a negative value, check sensor configuration." );
                break;
            }

            switch ( i ) {
            case 0:
                acamera_isp_frame_stitch_black_level_long_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            case 1:
                acamera_isp_frame_stitch_black_level_medium_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            case 2:
                acamera_isp_frame_stitch_black_level_short_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            case 3:
                acamera_isp_frame_stitch_black_level_very_short_write( p_ictx->settings.isp_base, p_fsm->black_level << black_level_stitch_shift_fs );
                break;
            default:
                LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Invalid exposure id (%u).", i );
                break;
            }
        }
    }

    // Update black-levels in pipeline when it's not manual.
    if ( get_context_param( p_ictx, ISP_MODULES_MANUAL_BLACK_LEVEL_PARAM ) == 0 ) {
        acamera_isp_offset_black_00_write( p_ictx->settings.isp_base, bl_r_low << black_level_shift_dg );
        acamera_isp_offset_black_01_write( p_ictx->settings.isp_base, bl_gr_low << black_level_shift_dg );
        acamera_isp_offset_black_10_write( p_ictx->settings.isp_base, bl_gb_low << black_level_shift_dg );
        acamera_isp_offset_black_11_write( p_ictx->settings.isp_base, bl_b_low << black_level_shift_dg );
#ifdef ACAMERA_ISP_DIGITAL_GAIN_OFFSET_DEFAULT
        acamera_isp_digital_gain_offset_write( p_ictx->settings.isp_base, black_level_dg );
#endif

// FE/BE and FS blackLevel set to the average of offset blackLevel.
#ifdef ACAMERA_ISP_GAMMA_FE_BLACK_LEVEL_IN_DL_DEFAULT // r0
        acamera_isp_gamma_fe_black_level_in_dl_write( p_ictx->settings.isp_base, black_level_dg );
        acamera_isp_gamma_be_black_level_out_dl_write( p_ictx->settings.isp_base, black_level_dg );
#endif
#ifdef ACAMERA_ISP_GAMMA_FE_BLACK_LEVEL_IN_SQ_DEFAULT // r1
        acamera_isp_gamma_fe_black_level_in_sq_write( p_ictx->settings.isp_base, black_level_dg );
        acamera_isp_gamma_be_black_level_out_sq_write( p_ictx->settings.isp_base, black_level_dg );
#endif

#ifdef ACAMERA_ISP_FRAME_STITCH_BLACK_LEVEL_OUT_DEFAULT
        acamera_isp_frame_stitch_black_level_out_write( p_ictx->settings.isp_base, black_level_dg );
#endif
    }
}

void sensor_request_next_frame( sensor_fsm_ptr_t p_fsm )
{
    // Remote sensors are not supported; frames are driven by the V4L2 path.
    (void)p_fsm;
}

////////////////////////////////////////////////////
// General router command handler

void sensor_get_general_sensor_info( sensor_fsm_ptr_t p_fsm, acamera_cmd_sensor_info *p_sensor_info )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    /* The pedestal and the depth it is expressed in are one value: publish them
     * together so no consumer has to re-derive the domain and get it wrong. */
    const uint32_t black_level_bits = sensor_black_level_bits( p_fsm );
    const uint32_t full_scale = 1u << black_level_bits;
    uint32_t black_level = p_fsm->black_level;

    /* A pedestal at or above full scale means the tuning misdeclares the depth
     * it was calibrated at. Clamp so the pair stays usable - consumers subtract
     * it from full scale - and name the slot to correct. */
    if ( black_level >= full_scale ) {
        /* Sampled: this runs per frame, once per sensor-info fetch. */
        LOG_CAT_SAMPLED( ALOG_CAT_SENSOR, LOG_ERR, "Black level %u exceeds the %u-bit full scale; check MODALIX_ISP_CALIB_CALIBRATION_BIT_DEPTH in the tuning.", black_level, black_level_bits );
        black_level = full_scale - 1;
    }

    // Update black-level and copy p_fsm->general_sensor_info.
    p_fsm->general_sensor_info.black_level = black_level;
    p_fsm->general_sensor_info.black_level_bits = black_level_bits;
    *p_sensor_info = p_fsm->general_sensor_info;
}

int sensor_get_general_sensor_preset_info( sensor_fsm_ptr_t p_fsm, uint32_t sensor_preset_id, acamera_cmd_sensor_preset_info_t *p_sensor_preset_info )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const acamera_cmd_sensor_info *gsi = &p_fsm->general_sensor_info;

    // Single mode sourced from the real sensor / S_FMT.
    if ( sensor_preset_id != 0 ) {
        return -1;
    }

    p_sensor_preset_info->width = gsi->active_width;
    p_sensor_preset_info->height = gsi->active_height;
    p_sensor_preset_info->fps = gsi->total_height ? ( gsi->lines_per_second << 8 ) / gsi->total_height : 0;
    p_sensor_preset_info->wdr_mode = gsi->current_sensor_mode.wdr_mode;
    p_sensor_preset_info->exposures = gsi->current_sensor_mode.exposures;
    p_sensor_preset_info->num_channels = gsi->current_sensor_mode.num_channels;
    p_sensor_preset_info->data_width = gsi->sensor_output_bits;
    p_sensor_preset_info->pixel_format = get_context_param( p_ictx, SENSOR_INFO_PIXEL_FORMAT_PARAM );

    return 0;
}

int remote_sensor_put_frame_callback_handler( void *owner, void *frame )
{
#if !MODEL_MODULE
    if ( !owner ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Error. Invalid owner pointer (NULL)." );
        return -1;
    } else if ( !frame ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Error. Invalid frame pointer (NULL)." );
        return -1;
    }

    const sensor_fsm_ptr_t p_fsm = owner;
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const int ctx_id = (int)( p_ictx->context_id );

    // Set frame state to filled and return it to the frame streamer
    ( (aframe_t *)frame )->state = AFRAME_STATE_FULL;
	// TODO : hack to avoid putting the frame back to application m2m already does it
#if 0 
    if ( frame_stream_put_frame( (aframe_t *)frame ) ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Failed to put filled raw frame from the frame streamer." );
        return -1;
    }
#endif

    fsm_raise_event( p_fsm, event_id_isphw_frame_end_fe );

    //LOG( LOG_INFO, "Filled raw frame is ready to be fetched by the MCFE (context id: %d, ptr: %p).", ctx_id, frame );

#endif
    return 0;
}

int remote_sensor_get_frame_callback_handler( void *owner, void **frame )
{
#if !MODEL_MODULE
    if ( !owner ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Error. Invalid owner pointer (NULL)." );
        return -1;
    } else if ( !frame ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Error. Invalid frame pointer (NULL)." );
        return -1;
    }

    const sensor_fsm_ptr_t p_fsm = owner;
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const int ctx_id = (int)( p_ictx->context_id );

    if ( frame_stream_get_frame( ctx_id, AFRAME_TYPE_RAW, AFRAME_STATE_EMPTY, (aframe_t **)frame ) ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Failed to get empty raw frame from the frame streamer." );
        *frame = NULL;
        return -1;
    }

    //LOG( LOG_INFO, "Empty raw frame acquired (context id: %d, ptr: %p).", ctx_id, *frame );

#endif
    return 0;
}

int remote_sensor_release_frame_callback_handler( void *owner, void *frame )
{
#if !MODEL_MODULE
    if ( !owner ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Error. Invalid owner pointer (NULL)." );
        return -1;
    } else if ( !frame ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Error. Invalid frame pointer (NULL)." );
        return -1;
    }

    const sensor_fsm_ptr_t p_fsm = owner;
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const int ctx_id = (int)( p_ictx->context_id );

    ( (aframe_t *)frame )->state = AFRAME_STATE_EMPTY;

    if ( frame_stream_put_frame( frame ) ) {
        LOG_CAT( ALOG_CAT_SENSOR, LOG_ERR, "Failed to put raw frame to the frame streamer." );
        return -1;
    }

    //LOG( LOG_INFO, "Raw frame released (context id: %d, ptr: %p).", ctx_id, frame );

#endif
    return 0;
}

void sensor_deinit( sensor_fsm_ptr_t p_fsm )
{
#if !MODEL_MODULE
    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    acamera_isp_pipeline_bypass_sensor_offset_wdr_write( isp_base, 1 );
    acamera_isp_pipeline_bypass_sensor_offset_linear_write( isp_base, 1 );
#endif
}
