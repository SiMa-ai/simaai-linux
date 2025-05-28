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
#include "acamera_command_api.h"
#include "acamera_isp_ctx.h"
#include "metadata_api.h"
#if KERNEL_MODULE
#include <linux/slab.h>
#endif

#include "acamera_frame_stream_api.h"
#include "acamera_logger.h"

#define DEBUG_METADATA_FSM ( 0 )
#define METADATA_BUFFER_SIZE ( 2048 )

void metadata_init( metadata_fsm_t *p_fsm )
{
    system_memset( &p_fsm->cur_metadata, 0x0, sizeof( isp_metadata_t ) );

    p_fsm->repeat_irq_mask = BIT( ACAMERA_IRQ_BE_FRAME_END );
    metadata_request_interrupt( p_fsm, p_fsm->repeat_irq_mask );
}

void metadata_config( metadata_fsm_ptr_t p_fsm )
{
#if V4L2_INTERFACE_BUILD
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    frame_stream_cfg_t frame_stream_cfg = {0};

    // Metadata frame stream header configuration
    frame_stream_cfg.num_planes = 1;
    frame_stream_cfg.context_id = p_ictx->context_id;
    frame_stream_cfg.type = AFRAME_TYPE_META;
    frame_stream_cfg.num_frames = FRAME_STREAM_META_FRAME_COUNT;

    // Plane configuration
    frame_stream_cfg.planes[0].data_width = 8;
    frame_stream_cfg.planes[0].height = 1;
    frame_stream_cfg.planes[0].width = sizeof( isp_metadata_t );
    frame_stream_cfg.planes[0].line_offset = sizeof( isp_metadata_t );

    if ( frame_stream_create( &frame_stream_cfg ) != 0 ) {
        LOG( LOG_ERR, "Error, failed to create metadata frame stream." );
    }
#endif
}

void metadata_update_meta( metadata_fsm_t *p_fsm )
{
    isp_metadata_t *md = &p_fsm->cur_metadata;
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    uint32_t isp_base_addr = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    const sensor_param_t *s_param = p_ictx->fsmgr.sensor_fsm.ctrl.get_parameters( p_ictx->fsmgr.sensor_fsm.drv_priv );

    LOG( LOG_DEBUG, "updating metadata ( isp_base_addr = 0x%x)", isp_base_addr );

    // Reset everything
    system_memset( md, 0x0, sizeof( isp_metadata_t ) );

    // Frame sequence
    md->sequence = p_ictx->frame_sequence;

    // Basic info
    md->sensor_total_width = s_param->total.width;
    md->sensor_total_height = s_param->total.height;
    md->sensor_active_width = s_param->active.width;
    md->sensor_active_height = s_param->active.height;
    md->sensor_data_width = s_param->data_width;
    md->rggb_start = acamera_isp_top_rggb_start_read( isp_base_addr );

    md->isp_mode = get_context_param( p_ictx, SENSOR_WDR_MODE_PARAM );

#if defined( ISP_HAS_CMOS_FSM )
    exposure_set_t exp_set;
    int32_t frame = s_param->integration_time_apply_delay;
    system_memset( &exp_set, 0x0, sizeof( exposure_set_t ) );

    WRAP_GENERAL_CMD( p_ictx, CMD_ID_FRAME_EXPOSURE_SET, CMD_DIRECTION_GET, (const uint32_t *)&frame, (uint32_t *)&exp_set );

    // Read FPS first to avoid integer division by zero. May not be trapped on some architectures, like ARM:
    // Overflow and divide-by-zero are not trapped. Any integer division by zero returns zero
    md->fps = cmos_get_fps( &p_ictx->fsmgr.cmos_fsm );

    md->int_time = exp_set.data.integration_time;
    md->int_time_ms = ( md->int_time * 100000 / md->sensor_total_height ) * 256 / md->fps;
    md->int_time_medium = exp_set.data.integration_time_medium;
    md->int_time_long = exp_set.data.integration_time_long;
    md->again = exp_set.info.again_log2;
    md->dgain = exp_set.info.dgain_log2;
    md->isp_dgain = exp_set.info.isp_dgain_log2;
    md->exposure = exp_set.info.exposure_log2;
    md->exposure_equiv = math_exp2( md->exposure, LOG2_GAIN_SHIFT, 0 );
    md->gain_log2 = ( md->again + md->dgain + md->isp_dgain ) >> LOG2_GAIN_SHIFT;
#endif

    // Tuning params
    md->anti_flicker = get_context_param( p_ictx, SYSTEM_ANTIFLICKER_ENABLE_PARAM );

    md->gain_00 = acamera_isp_white_balance_gain_00_read( isp_base_addr );
    md->gain_01 = acamera_isp_white_balance_gain_01_read( isp_base_addr );
    md->gain_10 = acamera_isp_white_balance_gain_10_read( isp_base_addr );
    md->gain_11 = acamera_isp_white_balance_gain_11_read( isp_base_addr );

    md->black_level_00 = (int)acamera_isp_offset_black_00_read( isp_base_addr );
    md->black_level_01 = (int)acamera_isp_offset_black_01_read( isp_base_addr );
    md->black_level_10 = (int)acamera_isp_offset_black_10_read( isp_base_addr );
    md->black_level_11 = (int)acamera_isp_offset_black_11_read( isp_base_addr );

    md->lsc_table = acamera_isp_mesh_shading_mesh_alpha_bank_r_read( isp_base_addr );
#if defined( ISP_HAS_MESH_SHADING_FSM )
    md->lsc_blend = p_ictx->fsmgr.mesh_shading_fsm.shading_alpha;
#else
    md->lsc_blend = -1;
#endif
    md->lsc_mesh_strength = acamera_isp_mesh_shading_mesh_strength_read( isp_base_addr );

#if defined( ISP_HAS_AWB_MESH_NBP_FSM )
    md->awb_rgain = p_ictx->fsmgr.AWB_fsm.rg_coef;
    md->awb_bgain = p_ictx->fsmgr.AWB_fsm.bg_coef;
    md->awb_cct = p_ictx->fsmgr.AWB_fsm.temperature_detected;
#elif defined( ISP_HAS_AWB_MANUAL_FSM )
    md->awb_rgain = -1;
    md->awb_bgain = -1;
    md->awb_cct = p_ictx->fsmgr.AWB_fsm.temperature_detected;
#else
    md->awb_rgain = -1;
    md->awb_bgain = -1;
    md->awb_cct = -1;
#endif

    md->sinter_strength = get_context_param( p_ictx, SYSTEM_SINTER_THRESHOLD_TARGET_PARAM );
    md->sinter_strength1 = acamera_isp_sinter_strength_1_read( isp_base_addr );
    md->sinter_strength4 = acamera_isp_sinter_strength_4_read( isp_base_addr );
    md->sinter_thresh_1h = acamera_isp_sinter_thresh_1h_read( isp_base_addr );
    md->sinter_thresh_4h = acamera_isp_sinter_thresh_4h_read( isp_base_addr );

#ifdef ACAMERA_ISP_IRIDIX_STRENGTH_DEFAULT
    md->iridix_strength = acamera_isp_iridix_strength_read( isp_base_addr );
#elif defined( ACAMERA_ISP_IRIDIX_STRENGTH_INROI_DEFAULT )
    md->iridix_strength = acamera_isp_iridix_strength_inroi_read( isp_base_addr );
#else
    md->iridix_strength = -1;
#endif

    md->raw_frontend_dp_thresh = acamera_isp_raw_frontend_dp_threshold_read( isp_base_addr );
    md->raw_frontend_dp_slope = acamera_isp_raw_frontend_dp_slope_read( isp_base_addr );

    md->sharpening_directional = get_context_param( p_ictx, SYSTEM_DIRECTIONAL_SHARPENING_TARGET_PARAM );
    md->sharpening_unidirectional = get_context_param( p_ictx, SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET_PARAM );

    md->demosaic_np_offset = acamera_isp_demosaic_np_offset_read( isp_base_addr );

    md->ccm[CCM_R][CCM_R] = acamera_isp_out_format_rgb2rgb_coef_a_11_read( isp_base_addr );
    md->ccm[CCM_R][CCM_G] = acamera_isp_out_format_rgb2rgb_coef_a_12_read( isp_base_addr );
    md->ccm[CCM_R][CCM_B] = acamera_isp_out_format_rgb2rgb_coef_a_13_read( isp_base_addr );
    md->ccm[CCM_G][CCM_R] = acamera_isp_out_format_rgb2rgb_coef_a_21_read( isp_base_addr );
    md->ccm[CCM_G][CCM_G] = acamera_isp_out_format_rgb2rgb_coef_a_22_read( isp_base_addr );
    md->ccm[CCM_G][CCM_B] = acamera_isp_out_format_rgb2rgb_coef_a_23_read( isp_base_addr );
    md->ccm[CCM_B][CCM_R] = acamera_isp_out_format_rgb2rgb_coef_a_31_read( isp_base_addr );
    md->ccm[CCM_B][CCM_G] = acamera_isp_out_format_rgb2rgb_coef_a_32_read( isp_base_addr );
    md->ccm[CCM_B][CCM_B] = acamera_isp_out_format_rgb2rgb_coef_a_33_read( isp_base_addr );
}

#if DEBUG_METADATA_FSM
static void dump_gain( const char *name, int32_t gain_log2 )
{
    int32_t gain_log10 = gain_log2 * 6; // 1% error
    uint16_t gain_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_log10 ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    int32_t gain_ones = math_exp2( gain_log2, LOG2_GAIN_SHIFT, LOG2_GAIN_SHIFT ); // 1% error
    uint16_t gain_ones_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_ones ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    LOG( LOG_INFO, "%s gain: %d.%03d (%d.%03d dB)", name, gain_ones >> LOG2_GAIN_SHIFT, gain_ones_dec, gain_log10 >> LOG2_GAIN_SHIFT, gain_dec );
}

static void metadata_dump( const isp_metadata_t *md )
{
    const char *modes[WDR_MODE_COUNT] = {"WDR_MODE_LINEAR", "WDR_MODE_NATIVE", "WDR_MODE_FS_HDR", "WDR_MODE_FS_LIN"};

    LOG( LOG_INFO, "Frame sequence: %d", md->sequence );
    LOG( LOG_INFO, "Sensor total width: %d", md->sensor_total_width );
    LOG( LOG_INFO, "Sensor total height: %d", md->sensor_total_height );
    LOG( LOG_INFO, "Sensor active width: %d", md->sensor_active_width );
    LOG( LOG_INFO, "Sensor active height: %d", md->sensor_active_height );
    LOG( LOG_INFO, "Sensor data width: %d", md->sensor_data_width );
    LOG( LOG_INFO, "RGGB start: %d", md->rggb_start );

    LOG( LOG_INFO, "ISP mode: %s", modes[md->isp_mode] );

    LOG( LOG_INFO, "FPS: %u.%02u", md->fps >> 8, ( md->fps & 0xFF ) * (uint32_t)1000 / 256 );

    LOG( LOG_INFO, "Integration time: %d lines %lld.%02lld ms", md->int_time, md->int_time_ms / 100, md->int_time_ms % 100 );
    LOG( LOG_INFO, "Integration time medium: %d", md->int_time_medium );
    LOG( LOG_INFO, "Integration time long: %d", md->int_time_long );
    dump_gain( "A", md->again );
    dump_gain( "D", md->dgain );
    dump_gain( "ISP", md->isp_dgain );
    LOG( LOG_INFO, "Exposure_log2: %d", md->exposure );
    LOG( LOG_INFO, "Equivalent Exposure: %u lines", math_exp2( md->exposure, LOG2_GAIN_SHIFT, 0 ) );
    LOG( LOG_INFO, "Gain_log2: %d", md->gain_log2 );

    LOG( LOG_INFO, "Antiflicker: %s", md->anti_flicker ? "on" : "off" );

    LOG( LOG_INFO, "Gain 00: %d", md->gain_00 );
    LOG( LOG_INFO, "Gain 01: %d", md->gain_01 );
    LOG( LOG_INFO, "Gain 10: %d", md->gain_10 );
    LOG( LOG_INFO, "Gain 11: %d", md->gain_11 );
    LOG( LOG_INFO, "Black Level 00: %d", md->black_level_00 );
    LOG( LOG_INFO, "Black Level 01: %d", md->black_level_01 );
    LOG( LOG_INFO, "Black Level 10: %d", md->black_level_10 );
    LOG( LOG_INFO, "Black Level 11: %d", md->black_level_11 );

    LOG( LOG_INFO, "LSC table: %d", md->lsc_table );
    LOG( LOG_INFO, "LSC blend: %d", md->lsc_blend );
    LOG( LOG_INFO, "LSC Mesh strength: %d", md->lsc_mesh_strength );

    LOG( LOG_INFO, "AWB rg: %lld", md->awb_rgain );
    LOG( LOG_INFO, "AWB bg: %lld", md->awb_bgain );
    LOG( LOG_INFO, "AWB temperature: %lld", md->awb_cct );

    LOG( LOG_INFO, "Sinter strength: %d", md->sinter_strength );
    LOG( LOG_INFO, "Sinter strength1: %d", md->sinter_strength1 );
    LOG( LOG_INFO, "Sinter strength4: %d", md->sinter_strength4 );
    LOG( LOG_INFO, "Sinter thresh1h: %d", md->sinter_thresh_1h );
    LOG( LOG_INFO, "Sinter thresh4h: %d", md->sinter_thresh_4h );

    LOG( LOG_INFO, "Iridix strength: %d", md->iridix_strength );

    LOG( LOG_INFO, "Raw frontend DP threshold: %d", md->raw_frontend_dp_thresh );
    LOG( LOG_INFO, "Raw frontend DP slope: %d", md->raw_frontend_dp_slope );

    LOG( LOG_INFO, "Sharpening directional: %d", md->sharpening_directional );
    LOG( LOG_INFO, "Sharpening unidirectional: %d", md->sharpening_unidirectional );
    LOG( LOG_INFO, "Demosaic NP offset: %d", md->demosaic_np_offset );

    LOG( LOG_INFO, "CCM R_R: 0x%X", md->ccm[CCM_R][CCM_R] );
    LOG( LOG_INFO, "CCM R_G: 0x%X", md->ccm[CCM_R][CCM_G] );
    LOG( LOG_INFO, "CCM R_B: 0x%X", md->ccm[CCM_R][CCM_B] );
    LOG( LOG_INFO, "CCM G_R: 0x%X", md->ccm[CCM_G][CCM_R] );
    LOG( LOG_INFO, "CCM G_G: 0x%X", md->ccm[CCM_G][CCM_G] );
    LOG( LOG_INFO, "CCM G_B: 0x%X", md->ccm[CCM_G][CCM_B] );
    LOG( LOG_INFO, "CCM B_R: 0x%X", md->ccm[CCM_B][CCM_R] );
    LOG( LOG_INFO, "CCM B_G: 0x%X", md->ccm[CCM_B][CCM_G] );
    LOG( LOG_INFO, "CCM B_B: 0x%X", md->ccm[CCM_B][CCM_B] );
}
#endif

#if !V4L2_INTERFACE_BUILD && defined( ISP_HAS_MCFE_FSM )
static int format_gain( char *buf, const char *name, int32_t gain_log2 )
{
    int32_t gain_log10 = gain_log2 * 6; // 1% error
    uint16_t gain_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_log10 ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    int32_t gain_ones = math_exp2( gain_log2, LOG2_GAIN_SHIFT, LOG2_GAIN_SHIFT ); // 1% error
    uint16_t gain_ones_dec = ( uint16_t )( ( ( ACAMERA_ABS( gain_ones ) & ( ( 1 << LOG2_GAIN_SHIFT ) - 1 ) ) * 1000 ) >> LOG2_GAIN_SHIFT );
    return sprintf( buf, "%s gain: %d.%03d (%d.%03d dB)\n", name, gain_ones >> LOG2_GAIN_SHIFT, gain_ones_dec, gain_log10 >> LOG2_GAIN_SHIFT, gain_dec );
}

static void metadata_fill_buf( int32_t isp_base, const isp_metadata_t *md )
{
#if ( METADATA_BUFFER_SIZE > LEN_ADDR_META )
#error "METADATA_BUFFER_SIZE exceeds reserved memory region defined by LEN_ADDR_META"
#endif

#if KERNEL_MODULE
    char *data = kmalloc( METADATA_BUFFER_SIZE, GFP_KERNEL | __GFP_NOFAIL );
#else
    char data[METADATA_BUFFER_SIZE];
#endif
    char *buf = data;
    const char *modes[WDR_MODE_COUNT] = {"WDR_MODE_LINEAR", "WDR_MODE_NATIVE", "WDR_MODE_FS_HDR", "WDR_MODE_FS_LIN"};

    // sprintf appends \0 and returns the number of characters written,
    // excl. \0. There's no way an error (< 0) can be returned by the calls below.

    buf += sprintf( buf, "Frame sequence: %d\n", md->sequence );
    buf += sprintf( buf, "Sensor total width: %d\n", md->sensor_total_width );
    buf += sprintf( buf, "Sensor total height: %d\n", md->sensor_total_height );
    buf += sprintf( buf, "Sensor active width: %d\n", md->sensor_active_width );
    buf += sprintf( buf, "Sensor active height: %d\n", md->sensor_active_height );
    buf += sprintf( buf, "Sensor data width: %d\n", md->sensor_data_width );
    buf += sprintf( buf, "RGGB start: %d\n", md->rggb_start );

    buf += sprintf( buf, "ISP mode: %s\n", modes[md->isp_mode] );

    buf += sprintf( buf, "FPS: %u.%02u\n", md->fps >> 8, ( ( md->fps & 0xFF ) * 100 ) >> 8 );

    buf += sprintf( buf, "Integration time: %d lines %lld.%02lld ms\n",
                    md->int_time, (long long int)md->int_time_ms / 100, (long long int)( md->int_time_ms % 100 ) );
    buf += sprintf( buf, "Integration time medium: %d\n", md->int_time_medium );
    buf += sprintf( buf, "Integration time long: %d\n", md->int_time_long );
    buf += format_gain( buf, "A", md->again );
    buf += format_gain( buf, "D", md->dgain );
    buf += format_gain( buf, "ISP", md->isp_dgain );
    buf += sprintf( buf, "Exposure_log2: %d\n", md->exposure );
    buf += sprintf( buf, "Equivalent Exposure: %u lines\n",
                    math_exp2( md->exposure, LOG2_GAIN_SHIFT, 0 ) );
    buf += sprintf( buf, "Gain_log2: %d\n", md->gain_log2 );

    buf += sprintf( buf, "Antiflicker: %s\n", md->anti_flicker ? "on" : "off" );

    buf += sprintf( buf, "Gain 00: %d\n", md->gain_00 );
    buf += sprintf( buf, "Gain 01: %d\n", md->gain_01 );
    buf += sprintf( buf, "Gain 10: %d\n", md->gain_10 );
    buf += sprintf( buf, "Gain 11: %d\n", md->gain_11 );
    buf += sprintf( buf, "Black Level 00: %d\n", md->black_level_00 );
    buf += sprintf( buf, "Black Level 01: %d\n", md->black_level_01 );
    buf += sprintf( buf, "Black Level 10: %d\n", md->black_level_10 );
    buf += sprintf( buf, "Black Level 11: %d\n", md->black_level_11 );

    buf += sprintf( buf, "LSC table: %d\n", md->lsc_table );
    buf += sprintf( buf, "LSC blend: %d\n", md->lsc_blend );
    buf += sprintf( buf, "LSC Mesh strength: %d\n", md->lsc_mesh_strength );

    buf += sprintf( buf, "AWB rg: %lld\n", (long long int)md->awb_rgain );
    buf += sprintf( buf, "AWB bg: %lld\n", (long long int)md->awb_bgain );
    buf += sprintf( buf, "AWB temperature: %lld\n", (long long int)md->awb_cct );

    buf += sprintf( buf, "Sinter strength: %d\n", md->sinter_strength );
    buf += sprintf( buf, "Sinter strength1: %d\n", md->sinter_strength1 );
    buf += sprintf( buf, "Sinter strength4: %d\n", md->sinter_strength4 );
    buf += sprintf( buf, "Sinter thresh1h: %d\n", md->sinter_thresh_1h );
    buf += sprintf( buf, "Sinter thresh4h: %d\n", md->sinter_thresh_4h );

    buf += sprintf( buf, "Iridix strength: %d\n", md->iridix_strength );

    buf += sprintf( buf, "Raw frontend DP threshold: %d\n", md->raw_frontend_dp_thresh );
    buf += sprintf( buf, "Raw frontend DP slope: %d\n", md->raw_frontend_dp_slope );

    buf += sprintf( buf, "Sharpening directional: %d\n", md->sharpening_directional );
    buf += sprintf( buf, "Sharpening unidirectional: %d\n", md->sharpening_unidirectional );
    buf += sprintf( buf, "Demosaic NP offset: %d\n", md->demosaic_np_offset );

    buf += sprintf( buf, "CCM R_R: 0x%X\n", md->ccm[CCM_R][CCM_R] );
    buf += sprintf( buf, "CCM R_G: 0x%X\n", md->ccm[CCM_R][CCM_G] );
    buf += sprintf( buf, "CCM R_B: 0x%X\n", md->ccm[CCM_R][CCM_B] );
    buf += sprintf( buf, "CCM G_R: 0x%X\n", md->ccm[CCM_G][CCM_R] );
    buf += sprintf( buf, "CCM G_G: 0x%X\n", md->ccm[CCM_G][CCM_G] );
    buf += sprintf( buf, "CCM G_B: 0x%X\n", md->ccm[CCM_G][CCM_B] );
    buf += sprintf( buf, "CCM B_R: 0x%X\n", md->ccm[CCM_B][CCM_R] );
    buf += sprintf( buf, "CCM B_G: 0x%X\n", md->ccm[CCM_B][CCM_G] );
    buf += sprintf( buf, "CCM B_B: 0x%X\n", md->ccm[CCM_B][CCM_B] );

    // Dump to metadata memory.
    system_memcpy_vir2phy( isp_base + LEN_ADDR_ISP, data, METADATA_BUFFER_SIZE );

#if KERNEL_MODULE
    kfree( data );
#endif
}
#endif // not V4L2_INTERFACE_BUILD && defined( ISP_HAS_MCFE_FSM )

void metadata_post_meta( metadata_fsm_const_ptr_t p_fsm )
{
    const isp_metadata_t *fw_meta = &p_fsm->cur_metadata;

#if DEBUG_METADATA_FSM
    metadata_dump( fw_meta );
#endif

#if V4L2_INTERFACE_BUILD
    aframe_t *frame = NULL;
    const uint32_t context_id = ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id;

    if ( frame_stream_get_frame( context_id, AFRAME_TYPE_META, AFRAME_STATE_EMPTY, &frame ) ) {
        LOG( LOG_ERR, "Failed to get empty metadata buffer from the frame streamer (context id: %u)", context_id );
    } else {
        if ( frame->planes[0].virt_addr != NULL ) {

            // Check frame buffer size
            if ( frame->planes[0].length < sizeof( isp_metadata_t ) ) {
                LOG( LOG_ERR, "Metadata frame buffer size (%u) is less than required (%lu), truncating data. (context id: %u)",
                     frame->planes[0].length, sizeof( isp_metadata_t ), context_id );
                system_memcpy( frame->planes[0].virt_addr, fw_meta, frame->planes[0].length );
            } else {
                system_memcpy( frame->planes[0].virt_addr, fw_meta, sizeof( isp_metadata_t ) );
            }
            frame->state = AFRAME_STATE_FULL;
        } else {
            frame->state = AFRAME_STATE_EMPTY;
            LOG( LOG_ERR, "Empty metadata frame plane 0 virtual address is NULL (context id: %u)", context_id );
        }

        frame->sequence = fw_meta->sequence;
        frame_stream_put_frame( frame );
    }
#elif defined( ISP_HAS_MCFE_FSM )
    if ( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base != 0 ) {
        metadata_fill_buf( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, fw_meta );
    }
#endif
}

void metadata_fsm_process_interrupt( metadata_fsm_const_ptr_t p_fsm, uint8_t irq_event )
{
    switch ( irq_event ) {
    case ACAMERA_IRQ_BE_FRAME_END:
        fsm_raise_event( p_fsm, event_id_metadata_update );
        fsm_raise_event( p_fsm, event_id_metadata_ready );
        break;
    }
}

void metadata_stop( metadata_fsm_ptr_t p_fsm )
{
#if V4L2_INTERFACE_BUILD
    if ( frame_stream_destroy( ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id, AFRAME_TYPE_META, 1 ) ) {
        LOG( LOG_ERR, "Failed to destroy metadata frame stream." );
    }
#endif
}

void metadata_deinit( metadata_fsm_ptr_t p_fsm )
{
}
