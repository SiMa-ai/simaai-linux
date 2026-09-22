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
#include "acamera_frontend_config.h"
#include "acamera_histogram_mem_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "ae_update_histogram.h"
#include "bitop.h"
#include "util_crc16.h" //fast ccitt
#if defined( ISP_HAS_SBUF_FSM )
#include "isp-v4l2-meta-stats.h"
#endif

/* HISTOGRAM_BUF_LEN moved to histogram_fsm.h so the per-context buffer
 * can be sized as a struct member. Local synonym kept for readability. */
#define HISTOGRAM_BUFFER_SIZE HISTOGRAM_BUF_LEN

static int get_mem_hist_id( histogram_fsm_const_ptr_t p_fsm )
{
    if ( ISP_HISTOGRAM_POSITION_IS_BE ) {
        return 0;
    } else {
#if defined( ISP_HAS_MCFE_FSM )
        return module_mcfe_get_input_for_slot_id( ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id );
#else
        return ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id;
#endif /* ISP_HAS_MCFE_FSM */
    }
}

static uint32_t histogram_read( histogram_fsm_const_ptr_t p_fsm, const size_t index, const uint16_t histmem_offset )
{
#if STATISTICS_BUFFER_DATA_LOCALLY
    (void)histmem_offset; // On a locally buffered version just ignore the offset.
    return p_fsm->hist_buf[index];
#else
    return system_isp_read_32(
        ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base +
        ACAMERA_HISTOGRAM_MEM_BASE_ADDR +
        histmem_offset +
        ( index << 2 ) );
#endif
}

static uint16_t histogram_get_crc( histogram_fsm_const_ptr_t p_fsm, const uint16_t histmem_offset )
{
    uint16_t crc16 = 0xffff;
    size_t i;
    /* We do not include the CRC in the crc calculation. */
    for ( i = 0; i < ( HISTOGRAM_BUFFER_SIZE - 1 ); ++i ) {
        const uint32_t data = histogram_read( p_fsm, i, histmem_offset );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 24, 8 ) );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 16, 8 ) );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 8, 8 ) );
        crc16 = acrc_ccitt_byte( crc16, BF_GET( data, 0, 8 ) );
    }

    return crc16;
}

static uint8_t histogram_is_crc_valid( histogram_fsm_const_ptr_t p_fsm, const uint16_t histmem_offset )
{
    const uint32_t crc16 = histogram_get_crc( p_fsm, histmem_offset );
    /* The last entry in in a histogram set is the crc. */
    const uint32_t crc_hw = histogram_read( p_fsm, HISTOGRAM_BUFFER_SIZE - 1, histmem_offset );

    if ( crc16 != crc_hw ) {
        LOG( LOG_DEBUG, "CTX[%d] histogram CRC mismatch 0x%x expected, 0x%x calculated",
             (int)ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id,
             crc_hw,
             crc16 );
        return 0;
    }

    return 1;
}

static uint32_t histogram_decode_bin( const uint16_t bin )
{
    const size_t shift = BF_GET( bin, 12, 4 );
    const uint32_t data = BF_GET( bin, 0, 12 );
    return shift ? ( data | 0x1000 ) << ( shift - 1 ) : data;
}

static void ae_read_full_histogram_data( histogram_fsm_ptr_t p_fsm, const uint16_t histmem_offset )
{
    size_t i;
    uint32_t sum = 0;
    LOG( LOG_DEBUG, "Read ae stat ctx %d from [ base 0x%x / histmem_offset 0x%x ]",
         (int)ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id, (int)ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, histmem_offset );

#if STATISTICS_BUFFER_DATA_LOCALLY
    /* Snapshot into the per-context p_fsm->hist_buf so concurrent stats
     * reads on other ISP contexts don't share a scratch buffer. */
    system_isp_mem_read( p_fsm->hist_buf,
                         ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base +
                             ACAMERA_HISTOGRAM_MEM_BASE_ADDR +
                             histmem_offset,
                         HISTOGRAM_BUFFER_SIZE,
                         sizeof( uint32_t ) );
#endif /* STATISTICS_BUFFER_DATA_LOCALLY */

    p_fsm->hist_ready_mask = 0;

    if ( histogram_is_crc_valid( p_fsm, histmem_offset ) ) {
        for ( i = 0; i < ISP_METERING_HISTOGRAM_SIZE_BINS; i += 2 ) {
            /*Layout in 32 bit registers: */
            /* |31      bin 1         16 | 15       bin 0        0|*/
            /* |31      bin 3         16 | 15       bin 2        0|*/
            const uint32_t data = histogram_read( p_fsm, i >> 1, histmem_offset );
            p_fsm->fullhist[i + 0] = histogram_decode_bin( BF_GET( data, 0, 16 ) );
            p_fsm->fullhist[i + 1] = histogram_decode_bin( BF_GET( data, 16, 16 ) );
            sum += p_fsm->fullhist[i];
            sum += p_fsm->fullhist[i + 1];
        }

        // Pipeline with proper histogram dgain compensation should not call function below.
        ae_histogram_dgain_compensation( p_fsm );

        p_fsm->fullhist_sum = sum;
        p_fsm->hist_ready_mask |= BIT( 0 );

        /* Publish AE histogram + sum to the V4L2 META_CAPTURE queue.
         * Replaces the legacy sbuf get_item / set_item dance: with
         * the user-space chardev gone, there's no consumer of the
         * sbuf ring slot, so the indirection drops out. */
        modalix_meta_stats_publish_ae( ACAMERA_FSM_GET_FW_ID( p_fsm ),
                                       p_fsm->fullhist, sum );
    }
}


void histogram_init( histogram_fsm_ptr_t p_fsm )
{
    LOG( LOG_INFO, "Ctx %d E", ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id );
    p_fsm->repeat_irq_mask = BIT( ACAMERA_IRQ_AE_STATS );
    p_fsm->hist_is_on_sqrt = ISP_HISTOGRAM_POSITION_IS_BE && AE_HISTOGRAM_TAP_AFTER_DECOMPANDER;
    histogram_request_interrupt( p_fsm, p_fsm->repeat_irq_mask );

    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;

    /* By default, start every 3rd pixel in horizontal. */
    acamera_isp_metering_hist_1_skip_x_write( isp_base, 1 );
    acamera_isp_metering_hist_2_skip_x_write( isp_base, 1 );
    acamera_isp_metering_hist_3_skip_x_write( isp_base, 1 );
    acamera_isp_metering_hist_4_skip_x_write( isp_base, 1 );

    /* By default, start on the first row in vertical. */
    acamera_isp_metering_hist_1_skip_y_write( isp_base, 0 );
    acamera_isp_metering_hist_2_skip_y_write( isp_base, 0 );
    acamera_isp_metering_hist_3_skip_y_write( isp_base, 0 );
    acamera_isp_metering_hist_4_skip_y_write( isp_base, 0 );
}

void histogram_config( histogram_fsm_ptr_t p_fsm )
{
    LOG( LOG_INFO, "Ctx %d E", ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id );
    histogram_configure_position( p_fsm );

    ae_update_histogram_geometry( p_fsm, get_mem_hist_id( p_fsm ) );
    ae_update_histogram_black_level( p_fsm, get_mem_hist_id( p_fsm ) );
}

void histogram_reload_calibration( histogram_fsm_ptr_t p_fsm )
{
    if ( calib_mgr_lut_exists( ACAMERA_FSM2CM_PTR( p_fsm ), MODALIX_ISP_CALIB_STATISTICS_CONFIG ) ) {
        const uint32_t *config = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), MODALIX_ISP_CALIB_STATISTICS_CONFIG );
        const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
        acamera_isp_metering_black_level_awb_write( isp_base, config[0] );
        acamera_isp_metering_white_level_awb_write( isp_base, config[1] );
        /* Optional AWB pixel-qualification ratio window (G/R, G/B in u4.8).
         * The HW default [64..511] caps ratios at 2.0, which excludes green-
         * strong sensors entirely at the raw stats tap. */
        if ( calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), MODALIX_ISP_CALIB_STATISTICS_CONFIG ) >= 6 ) {
            acamera_isp_metering_cr_ref_min_awb_write( isp_base, config[2] );
            acamera_isp_metering_cr_ref_max_awb_write( isp_base, config[3] );
            acamera_isp_metering_cb_ref_min_awb_write( isp_base, config[4] );
            acamera_isp_metering_cb_ref_max_awb_write( isp_base, config[5] );
        }
    } else {
        LOG( LOG_ERR, "MODALIX_ISP_CALIB_STATISTICS_CONFIG is missing, module will not be configured correctly!" );
    }

    configure_histogram_neq_lut( p_fsm, get_mem_hist_id( p_fsm ) );
    ae_update_histogram_shading_lut( p_fsm, get_mem_hist_id( p_fsm ) );
    ae_update_histogram_zone_weights( p_fsm, get_mem_hist_id( p_fsm ) );

    // Update black levels as decompander configuration might change
    ae_update_histogram_black_level( p_fsm, get_mem_hist_id( p_fsm ) );
}

void histogram_update_hw( histogram_fsm_ptr_t p_fsm )
{
    ae_update_histogram_wbgain( p_fsm, get_mem_hist_id( p_fsm ) );
}

void histogram_fsm_process_interrupt( histogram_fsm_const_ptr_t p_fsm, uint8_t irq_event )
{
    switch ( irq_event ) {
    case ACAMERA_IRQ_ANTIFOG_HIST:
    case ACAMERA_IRQ_AE_STATS: {
        int mem_hist_id = get_mem_hist_id( p_fsm );
        if ( mem_hist_id >= 0 )
            ae_read_full_histogram_data( (histogram_fsm_ptr_t)p_fsm, mem_hist_id * ISP_METERING_HISTOGRAM_STRIDE_BYTES );
        if ( p_fsm->hist_ready_mask ) {
            // TODO do we need to have separate events for ae stats and hist stats?
            fsm_raise_event( p_fsm, event_id_isphw_stats_ready_ae );
        }
    } break;
    }
}
