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

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include "system_stdlib.h" // system_memcpy
#include "system_timer.h"  // sleep
#include "acamera_logger.h"
#include "sbuf.h"
#include "sbuf_mgr.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_SBUF


struct sbuf_context {
    struct sbuf_mgr sbuf_mgr;

    struct mutex fops_lock;
    struct miscdevice sbuf_dev;
    int dev_minor_id;
    char dev_name[SBUF_DEV_NAME_LEN];
    int dev_opened;

    uint32_t ctx_id;
    sbuf_fsm_t *p_fsm;

    struct mutex idx_set_lock;
    struct sbuf_idx_set idx_set;
    wait_queue_head_t idx_set_wait_queue;
};

static struct sbuf_context sbuf_contexts[FIRMWARE_CONTEXT_NUMBER];

static uint32_t get_cur_calibration_total_size( sbuf_fsm_ptr_t p_fsm )
{
    uint32_t result = 0;
    uint32_t idx = 0;

    LOG( LOG_DEBUG, "ctx_id: %d, CALIBRATION_TOTAL_SIZE: %d.", ACAMERA_FSM_GET_FW_ID( p_fsm ), CALIBRATION_TOTAL_SIZE );

    for ( idx = 0; idx < CALIBRATION_TOTAL_SIZE; idx++ ) {
        result += calib_mgr_lut_size( ACAMERA_FSM2CM_PTR( p_fsm ), idx );
        if ( result == 0 )
            LOG( LOG_ERR, "Error: LUT %d has 0 size.", idx );
        result += sizeof( LookupTable );
    }

    LOG( LOG_DEBUG, "Total size for all IQ LUTs is %d bytes", result );
    return result;
}

static int sbuf_calibration_init( struct sbuf_context *p_ctx )
{
    int rc = 0;
    uint32_t cnt = 0;
    uint32_t cali_total_size = 0;

    if ( !p_ctx ) {
        LOG( LOG_ERR, "Error: sbuf contex pointer is NULL." );
        return -1;
    }

    LOG( LOG_INFO, "sbuf_calibration_init. %p %p ", p_ctx, p_ctx->p_fsm );

    // Get total calibration size, including LUTs.
    cali_total_size = get_cur_calibration_total_size( p_ctx->p_fsm );

    if ( cali_total_size > SBUF_MAX_CALIBRATION_DATA_SIZE ) {
        LOG( LOG_CRIT, "Error: Not enough memory for calibration data, total_size: %d, max_size: %d.",
             cali_total_size, SBUF_MAX_CALIBRATION_DATA_SIZE );
        return -1;
    }

    // We can update the sbuf-calibration if no sbuf item used by UF.
    while ( sbuf_mgr_item_count_in_using( &p_ctx->sbuf_mgr ) != 0 ) {
        LOG( LOG_DEBUG, "Wait for UF to finish using." );
        // Sleep 3 ms.
        system_timer_usleep( 10 * 1000 );

        cnt++;

        if ( cnt >= 10 ) {
            LOG( LOG_CRIT, "Timeout waiting for calibration to be ready to update." );
            break;
        }
    }

    rc = update_cur_calibration_to_sbuf( p_ctx->p_fsm, &p_ctx->sbuf_mgr );

    p_ctx->p_fsm->is_paused = 0;

    return rc;
}

void sbuf_stop( sbuf_fsm_ptr_t p_fsm )
{
    uint32_t ctx_id = ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id;
    struct sbuf_context *p_ctx = &( sbuf_contexts[ctx_id] );

    if ( p_ctx->ctx_id != ctx_id ) {
        LOG( LOG_CRIT, "Error: ctx_id does not match, fsm ctx_id: %d, ctx_id: %d.", ctx_id, p_ctx->ctx_id );
        return;
    }

    p_ctx->p_fsm->is_paused = 1;
    p_ctx->sbuf_mgr.sbuf_base->kf_info.cali_info.is_fetched = 1;
}

static int sbuf_ctx_init( struct sbuf_context *p_ctx )
{
    int rc;

    rc = sbuf_mgr_init( &p_ctx->sbuf_mgr, p_ctx->ctx_id );
    if ( rc ) {
        LOG( LOG_ERR, "Init failed, error: sbuf_mgr init failed, ret: %d.", rc );
    }

    return rc;
}

/**
 *   @brief     Send/release the ae buffer to user-app.
 *
 *   @param     p_fsm   Pointer to FSM private data
 *   @return
 *   @warning
 *   @details   Function will be called when this FSM received isphw_stats_ready_ae event from histogram_fsm.
 */
void sbuf_update_ae_idx( sbuf_fsm_t *p_fsm )
{
    int rc = 0;
    struct sbuf_item sbuf;
#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    struct sbuf_item sbuf_iridix;
#endif
    struct sbuf_context *p_ctx = NULL;
    uint32_t ctx_id = ACAMERA_FSM_GET_FW_ID( p_fsm );
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    if ( ctx_id >= context_number ) {
        LOG( LOG_CRIT, "Fatal error: Invalid FW context ID: %d, max is: %d", ctx_id, context_number - 1 );
        return;
    }

    p_ctx = &( sbuf_contexts[ctx_id] );

    if ( !p_ctx->dev_opened ) {
        LOG( LOG_DEBUG, "Device is not opened, skip, ctx_id: %d.", ctx_id );
        return;
    }

    system_memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_status = SBUF_STATUS_DATA_DONE;
    sbuf.buf_type = SBUF_TYPE_AE;

    if ( sbuf_get_item( p_ctx->ctx_id, &sbuf ) ) {
        LOG( LOG_INFO, "Failed to get sbuf for ae done buffer, ctx_id: %d.", p_ctx->ctx_id );
        return;
    }

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    /* NOTE: Iridix */
    system_memset( &sbuf_iridix, 0, sizeof( sbuf_iridix ) );
    sbuf_iridix.buf_status = SBUF_STATUS_DATA_DONE;
    sbuf_iridix.buf_type = SBUF_TYPE_IRIDIX;

    if ( sbuf_get_item( p_ctx->ctx_id, &sbuf_iridix ) ) {
        LOG( LOG_INFO, "Failed to get sbuf for Iridix done buffer, ctx_id: %d.", p_ctx->ctx_id );

        // We need to return AE sbuf to sbuf_manager when error is happened.
        sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;
        sbuf_set_item( p_ctx->ctx_id, &sbuf );

        return;
    }
#endif

    rc = mutex_lock_interruptible( &p_ctx->idx_set_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: access lock failed, rc: %d.", rc );
        // We need to return this sbuf to sbuf_manager when error is happened.
        sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;
        sbuf_set_item( p_ctx->ctx_id, &sbuf );

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
        sbuf_iridix.buf_status = SBUF_STATUS_DATA_EMPTY;
        sbuf_set_item( p_ctx->ctx_id, &sbuf_iridix );
#endif
        return;
    }

    p_ctx->idx_set.ae_idx = sbuf.buf_idx;
    p_ctx->idx_set.ae_idx_valid = 1;

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    p_ctx->idx_set.iridix_idx = sbuf_iridix.buf_idx;
    p_ctx->idx_set.iridix_idx_valid = 1;
#endif

    mutex_unlock( &p_ctx->idx_set_lock );

    wake_up_interruptible( &p_ctx->idx_set_wait_queue );
}

void sbuf_update_awb_idx( sbuf_fsm_t *p_fsm )
{
    int rc = 0;
    struct sbuf_item sbuf;
    struct sbuf_context *p_ctx = NULL;
    uint32_t ctx_id = ACAMERA_FSM_GET_FW_ID( p_fsm );
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    if ( ctx_id >= context_number ) {
        LOG( LOG_CRIT, "Fatal error: Invalid FW context ID: %d, max is: %d", ctx_id, context_number - 1 );
        return;
    }

    p_ctx = &( sbuf_contexts[ctx_id] );
    if ( !p_ctx->dev_opened ) {
        LOG( LOG_DEBUG, "Device is not opened, skip, ctx_id: %d.", ctx_id );
        return;
    }

    system_memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_status = SBUF_STATUS_DATA_DONE;
    sbuf.buf_type = SBUF_TYPE_AWB;

    if ( sbuf_get_item( p_ctx->ctx_id, &sbuf ) ) {
        LOG( LOG_INFO, "Failed to get sbuf for AWB done buffer, ctx_id: %d.", p_ctx->ctx_id );
        return;
    }

    rc = mutex_lock_interruptible( &p_ctx->idx_set_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: access lock failed, rc: %d.", rc );
        // We need to return this sbuf to sbuf_manager when error is happened.
        sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;
        sbuf_set_item( p_ctx->ctx_id, &sbuf );
        return;
    }

    p_ctx->idx_set.awb_idx = sbuf.buf_idx;
    p_ctx->idx_set.awb_idx_valid = 1;

    mutex_unlock( &p_ctx->idx_set_lock );

    wake_up_interruptible( &p_ctx->idx_set_wait_queue );
}

void sbuf_update_af_idx( sbuf_fsm_t *p_fsm )
{
    int rc = 0;
    struct sbuf_item sbuf;
    struct sbuf_context *p_ctx = NULL;
    uint32_t ctx_id = ACAMERA_FSM_GET_FW_ID( p_fsm );
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    if ( ctx_id >= context_number ) {
        LOG( LOG_CRIT, "Fatal error: Invalid FW context ID: %d, max is: %d", ctx_id, context_number - 1 );
        return;
    }

    p_ctx = &( sbuf_contexts[ctx_id] );

    if ( !p_ctx->dev_opened ) {
        LOG( LOG_DEBUG, "Device is not opened, skip, ctx_id: %d.", ctx_id );
        return;
    }

    system_memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_status = SBUF_STATUS_DATA_DONE;
    sbuf.buf_type = SBUF_TYPE_AF;

    if ( sbuf_get_item( p_ctx->ctx_id, &sbuf ) ) {
        LOG( LOG_INFO, "Failed to get sbuf for af done buffer, ctx_id: %d.", p_ctx->ctx_id );
        return;
    }

    rc = mutex_lock_interruptible( &p_ctx->idx_set_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: access lock failed, rc: %d.", rc );
        // We need to return this sbuf to sbuf_manager when error is happened.
        sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;
        sbuf_set_item( p_ctx->ctx_id, &sbuf );
        return;
    }

    p_ctx->idx_set.af_idx = sbuf.buf_idx;
    p_ctx->idx_set.af_idx_valid = 1;

    mutex_unlock( &p_ctx->idx_set_lock );

    wake_up_interruptible( &p_ctx->idx_set_wait_queue );
}

void sbuf_update_gamma_idx( sbuf_fsm_t *p_fsm )
{
    int rc = 0;
    struct sbuf_item sbuf;
    struct sbuf_context *p_ctx = NULL;
    uint32_t ctx_id = ACAMERA_FSM_GET_FW_ID( p_fsm );
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    if ( ctx_id >= context_number ) {
        LOG( LOG_CRIT, "Fatal error: Invalid FW context ID: %d, max is: %d", ctx_id, context_number - 1 );
        return;
    }

    p_ctx = &( sbuf_contexts[ctx_id] );

    if ( !p_ctx->dev_opened ) {
        LOG( LOG_DEBUG, "Device is not opened, skip, ctx_id: %d.", ctx_id );
        return;
    }

    system_memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_status = SBUF_STATUS_DATA_DONE;
    sbuf.buf_type = SBUF_TYPE_GAMMA;

    if ( sbuf_get_item( p_ctx->ctx_id, &sbuf ) ) {
        LOG( LOG_INFO, "Failed to get sbuf for gamma done buffer, ctx_id: %d.", p_ctx->ctx_id );
        return;
    }

    rc = mutex_lock_interruptible( &p_ctx->idx_set_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: access lock failed, rc: %d.", rc );
        // We need to return this sbuf to sbuf_manager when error is happened.
        sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;
        sbuf_set_item( p_ctx->ctx_id, &sbuf );
        return;
    }

    p_ctx->idx_set.gamma_idx = sbuf.buf_idx;
    p_ctx->idx_set.gamma_idx_valid = 1;

    mutex_unlock( &p_ctx->idx_set_lock );

    wake_up_interruptible( &p_ctx->idx_set_wait_queue );
}

static uint32_t sbuf_is_ready_to_send_data( struct sbuf_context *p_ctx )
{
    uint32_t rc = 1;

    // If UF didn't fetched calibration data yet, we need to wait before send stats data.
    if ( !p_ctx->sbuf_mgr.sbuf_base->kf_info.cali_info.is_fetched ) {
        rc = 0;
    }

    return rc;
}

static uint32_t sbuf_is_paused( struct sbuf_context *p_ctx )
{
    return p_ctx->p_fsm->is_paused;
}

static void sbuf_recycle_idx_set( struct sbuf_context *p_ctx, struct sbuf_idx_set *p_idx_set )
{
    struct sbuf_item item;

    if ( p_idx_set->ae_idx_valid ) {
        item.buf_idx = p_idx_set->ae_idx;
        item.buf_type = SBUF_TYPE_AE;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
    }

    if ( p_idx_set->awb_idx_valid ) {
        item.buf_idx = p_idx_set->awb_idx;
        item.buf_type = SBUF_TYPE_AWB;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
    }

    if ( p_idx_set->af_idx_valid ) {
        item.buf_idx = p_idx_set->af_idx;
        item.buf_type = SBUF_TYPE_AF;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
    }

    if ( p_idx_set->gamma_idx_valid ) {
        item.buf_idx = p_idx_set->gamma_idx;
        item.buf_type = SBUF_TYPE_GAMMA;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
    }

    if ( p_idx_set->iridix_idx_valid ) {
        item.buf_idx = p_idx_set->iridix_idx;
        item.buf_type = SBUF_TYPE_IRIDIX;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
    }
}

static void sbuf_mgr_get_latest_idx_set( struct sbuf_context *p_ctx, struct sbuf_idx_set *p_idx_set )
{
    int rc;
    uint32_t wait = 0;

    // Set all items to invalid by default.
    system_memset( p_idx_set, 0, sizeof( *p_idx_set ) );

    rc = mutex_lock_interruptible( &p_ctx->idx_set_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: access lock failed, rc: %d.", rc );
        return;
    }

    if ( is_idx_set_has_valid_item( &p_ctx->idx_set ) ) {
        *p_idx_set = p_ctx->idx_set;

        // Reset the valid flag to prepare next read.
        system_memset( &p_ctx->idx_set, 0, sizeof( p_ctx->idx_set ) );
    } else {
        wait = 1;
    }

    mutex_unlock( &p_ctx->idx_set_lock );

    if ( wait ) {
        long time_out_in_jiffies = 30; /* Jiffies is dependant on HW, in x86 Ubuntu, it's 4 ms, 30 is 120ms. */

        /* Wait for the event */
        LOG( LOG_DEBUG, "Wait for data, timeout_in_jiffies: %ld, HZ: %d.", time_out_in_jiffies, HZ );
        rc = wait_event_interruptible_timeout( p_ctx->idx_set_wait_queue, is_idx_set_has_valid_item( &p_ctx->idx_set ), time_out_in_jiffies );
        LOG( LOG_DEBUG, "After timeout, rc: %d, is_idx_set_has_valid_item: %d.", rc, is_idx_set_has_valid_item( &p_ctx->idx_set ) );

        rc = mutex_lock_interruptible( &p_ctx->idx_set_lock );
        if ( rc ) {
            LOG( LOG_ERR, "Error: 2nd access lock failed, rc: %d.", rc );
            return;
        }

        *p_idx_set = p_ctx->idx_set;

        // Reset to invalid flag to prepare for next read since we already send to user space.
        system_memset( &p_ctx->idx_set, 0, sizeof( p_ctx->idx_set ) );
        mutex_unlock( &p_ctx->idx_set_lock );
    }
}

/**
 *   @brief     Process all data from user-app.
 *
 *   @param     p_fsm   Pointer to FSM private data
 *   @return
 *   @warning
 *   @details   Called from sbuf_fops_write. Sets the corresponding/dependent fsms and raise events.
 */
static void sbuf_mgr_apply_new_param( struct sbuf_context *p_ctx, struct sbuf_idx_set *p_idx_set )
{
    struct sbuf_item item;
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_ctx->p_fsm );

#if defined( ISP_HAS_AE_MANUAL_FSM )
    /* AE */
    if ( p_idx_set->ae_idx_valid && ( p_idx_set->ae_idx < SBUF_STATS_ARRAY_SIZE ) ) {
        sbuf_ae_t *p_sbuf_ae;

        p_sbuf_ae = (sbuf_ae_t *)&( p_ctx->sbuf_mgr.sbuf_base->ae_sbuf[p_idx_set->ae_idx] );
        /* Apply only when user-FW has new result. */
        if ( p_sbuf_ae->changed_flag ) {

            if ( get_context_param( p_ictx, SYSTEM_MANUAL_EXPOSURE_PARAM ) == 0 )
                ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                    ->AE_fsm.new_exposure_log2 = p_sbuf_ae->ae_exposure;

            if ( get_context_param( p_ictx, SYSTEM_MANUAL_EXPOSURE_RATIO_PARAM ) == 0 )
                ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                    ->AE_fsm.new_exposure_ratio = p_sbuf_ae->ae_exposure_ratio;

            override_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM, p_sbuf_ae->ldr_gain_log2 );
            override_context_param( p_ictx, STATUS_INFO_AE_HIST_MEAN_PARAM, p_sbuf_ae->ae_hist_mean );

            p_sbuf_ae->changed_flag = 0;
        }

        /* Set this sbuf back to sbuf_mgr. */
        item.buf_idx = p_idx_set->ae_idx;
        item.buf_type = SBUF_TYPE_AE;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );

        fsm_raise_event( &( ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )->AE_fsm ), event_id_split_ae_data_ready );
    }
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
    /* AWB */
    if ( p_idx_set->awb_idx_valid && ( p_idx_set->awb_idx < SBUF_STATS_ARRAY_SIZE ) ) {
        sbuf_awb_t *p_sbuf_awb;

        p_sbuf_awb = (sbuf_awb_t *)&( p_ctx->sbuf_mgr.sbuf_base->awb_sbuf[p_idx_set->awb_idx] );
        /* Apply only when user-FW has new result. */
        if ( p_sbuf_awb->changed_flag ) {
            // Context parameters are different between user and kernel
            set_context_param( p_ictx, SYSTEM_AWB_RED_GAIN_PARAM, p_sbuf_awb->awb_red_gain );
            set_context_param( p_ictx, SYSTEM_AWB_BLUE_GAIN_PARAM, p_sbuf_awb->awb_blue_gain );
            set_context_param( p_ictx, SYSTEM_AWB_CCT_PARAM, p_sbuf_awb->temperature_detected );

            ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                ->AWB_fsm.light_source_candidate = p_sbuf_awb->light_source_candidate;
            ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                ->AWB_fsm.p_high = p_sbuf_awb->p_high;
            LOG( LOG_DEBUG, "ctx: %d, new AWB param: awb_red_gain: %u, awb_blue_gain: %u, temperature: %d, light_source: %u, p_high: %u",
                 p_ctx->ctx_id,
                 p_sbuf_awb->awb_red_gain,
                 p_sbuf_awb->awb_blue_gain,
                 p_sbuf_awb->temperature_detected,
                 p_sbuf_awb->light_source_candidate,
                 p_sbuf_awb->p_high );
            p_sbuf_awb->changed_flag = 0;
        }

        /* Set this sbuf back to sbuf_mgr. */
        item.buf_idx = p_idx_set->awb_idx;
        item.buf_type = SBUF_TYPE_AWB;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
        //fsm_raise_event( &(ACAMERA_FSM2FSMGR_PTR(p_ctx->p_fsm)->AE_fsm), event_id_split_awb_data_ready );
    }
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
    /* Gamma */
    if ( p_idx_set->gamma_idx_valid && ( p_idx_set->gamma_idx < SBUF_STATS_ARRAY_SIZE ) ) {
        sbuf_gamma_t *p_sbuf_gamma;

        p_sbuf_gamma = (sbuf_gamma_t *)&( p_ctx->sbuf_mgr.sbuf_base->gamma_sbuf[p_idx_set->gamma_idx] );
        /* Apply only when user-FW has new result. */
        if ( p_sbuf_gamma->changed_flag ) {
            ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                ->gamma_manual_fsm.lut_length = p_sbuf_gamma->lut_length;
            system_memcpy( ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )->gamma_manual_fsm.lut_contrast, p_sbuf_gamma->lut_contrast, sizeof( p_sbuf_gamma->lut_contrast ) );
            fsm_raise_event( &( ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )->gamma_manual_fsm ), event_id_gamma_lut_ready );

            p_sbuf_gamma->changed_flag = 0;
            LOG( LOG_DEBUG, "Ctx: %d, new gamma param: lut_length: %u.", p_ctx->ctx_id, p_sbuf_gamma->lut_length );
        }

        /* Set this sbuf back to sbuf_mgr. */
        item.buf_idx = p_idx_set->gamma_idx;
        item.buf_type = SBUF_TYPE_GAMMA;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
    }
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    /* Iridix */
    if ( p_idx_set->iridix_idx_valid && ( p_idx_set->iridix_idx < SBUF_STATS_ARRAY_SIZE ) ) {
        sbuf_iridix_t *p_sbuf_iridix;

        p_sbuf_iridix = (sbuf_iridix_t *)&( p_ctx->sbuf_mgr.sbuf_base->iridix_sbuf[p_idx_set->iridix_idx] );
        /* Apply only when user-FW has new result. */
        if ( p_sbuf_iridix->changed_flag ) {
            ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                ->iridix_fsm.strength_target = p_sbuf_iridix->strength_target;
            ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                ->iridix_fsm.dark_enh = p_sbuf_iridix->dark_enh;
            set_context_param( p_ictx, SYSTEM_IRIDIX_DIGITAL_GAIN_PARAM, p_sbuf_iridix->digital_gain );
            ACAMERA_FSM2FSMGR_PTR( p_ctx->p_fsm )
                ->iridix_fsm.iridix_contrast = p_sbuf_iridix->contrast;
            override_context_param( p_ictx, STATUS_INFO_IRIDIX_CONTRAST_PARAM, p_sbuf_iridix->contrast );
        }
        p_sbuf_iridix->changed_flag = 0;

        /* Set this sbuf back to sbuf_mgr. */
        item.buf_idx = p_idx_set->iridix_idx;
        item.buf_type = SBUF_TYPE_IRIDIX;
        item.buf_status = SBUF_STATUS_DATA_EMPTY;

        sbuf_set_item( p_ctx->ctx_id, &item );
    }
#endif
}

static void sbuf_update_kf_info( struct sbuf_context *p_ctx )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_ctx->p_fsm );
    struct sbuf_mgr *p_sbuf_mgr = &p_ctx->sbuf_mgr;

    // Update sensor_info
    acamera_cmd_sensor_info *sensor_info = &p_sbuf_mgr->sbuf_base->kf_info.sbuf_sensor_info.sensor_info;
    WRAP_GENERAL_CMD( p_ictx, CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)sensor_info );

    // Update sbuf_cmos_info
    int32_t *max_exposure_log2 = &p_sbuf_mgr->sbuf_base->kf_info.sbuf_cmos_info.max_exposure_log2;
    WRAP_GENERAL_CMD( p_ictx, CMD_ID_CMOS_MAX_EXPOSURE_LOG2, CMD_DIRECTION_GET, NULL, (uint32_t *)max_exposure_log2 );

    int32_t *again_log2 = &p_sbuf_mgr->sbuf_base->kf_info.sbuf_cmos_info.again_log2;
    WRAP_GENERAL_CMD( p_ictx, CMD_ID_CMOS_ANALOG_GAIN, CMD_DIRECTION_GET, NULL, (uint32_t *)again_log2 );

    int32_t *dgain_log2 = &p_sbuf_mgr->sbuf_base->kf_info.sbuf_cmos_info.dgain_log2;
    WRAP_GENERAL_CMD( p_ictx, CMD_ID_CMOS_DIGITAL_GAIN, CMD_DIRECTION_GET, NULL, (uint32_t *)dgain_log2 );

    LOG( LOG_DEBUG, "ctx_id: %d, set sbuf_cmos_info max_exposure_log2: %d, again_log2: %d, dgain_log2: %d.",
         p_ctx->ctx_id,
         p_sbuf_mgr->sbuf_base->kf_info.sbuf_cmos_info.max_exposure_log2,
         p_sbuf_mgr->sbuf_base->kf_info.sbuf_cmos_info.again_log2,
         p_sbuf_mgr->sbuf_base->kf_info.sbuf_cmos_info.dgain_log2 );
}

int sbuf_get_item( uint32_t ctx_id, struct sbuf_item *item )
{
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    if ( ctx_id >= context_number ) {
        LOG( LOG_ERR, "Error: Invaid param, ctx_id: %d, max is: %d.", ctx_id, context_number - 1 );
        return -EINVAL;
    }

    return sbuf_bfmgr_get_item( &( sbuf_contexts[ctx_id].sbuf_mgr ), item );
}

int sbuf_set_item( uint32_t ctx_id, struct sbuf_item *item )
{
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    if ( ctx_id >= context_number ) {
        LOG( LOG_ERR, "Error: Invalid param, ctx_id: %d, max is: %d.", ctx_id, context_number - 1 );
        return -EINVAL;
    }

    LOG( LOG_DEBUG, "ctx_id: %d.", ctx_id );
    return sbuf_bfmgr_set_item( &( sbuf_contexts[ctx_id].sbuf_mgr ), item );
}

static int sbuf_fops_open( struct inode *inode, struct file *f )
{
    int rc;
    uint32_t i;
    struct sbuf_context *p_ctx = NULL;
    int minor = iminor( inode );
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    LOG( LOG_INFO, "Client is opening..., minor: %d.", minor );

    for ( i = 0; i < context_number; i++ ) {
        if ( sbuf_contexts[i].dev_minor_id == minor ) {
            p_ctx = &sbuf_contexts[i];
            break;
        }
    }

    if ( !p_ctx ) {
        LOG( LOG_CRIT, "Fatal error, sbuf contexts is crashed, contents dump:" );
        for ( i = 0; i < context_number; i++ ) {
            p_ctx = &sbuf_contexts[i];
            LOG( LOG_CRIT, "%d): ctx_id: %d, minor_id: %d, name: %s, p_fsm: %p.",
                 i, p_ctx->ctx_id, p_ctx->dev_minor_id, p_ctx->dev_name, p_ctx->p_fsm );
        }
        return -ERESTARTSYS;
    }

    LOG( LOG_DEBUG, "ctx_id: %d, name: '%s' is found for opening.", p_ctx->ctx_id, p_ctx->dev_name );

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Access lock failed, rc: %d.", rc );
        return rc;
    }

    if ( p_ctx->dev_opened ) {
        LOG( LOG_ERR, "Open failed, already opened." );
        rc = -EBUSY;
    } else {
        sbuf_mgr_init_sbuf( &p_ctx->sbuf_mgr );
        system_memset( &p_ctx->idx_set, 0, sizeof( p_ctx->idx_set ) );

        p_ctx->dev_opened = 1;
        rc = 0;
        LOG( LOG_DEBUG, "Open succeed." );

        LOG( LOG_DEBUG, "Bf set, private_data: %p.", f->private_data );
        f->private_data = p_ctx;
        LOG( LOG_DEBUG, "Af set, private_data: %p.", f->private_data );
    }

    mutex_unlock( &p_ctx->fops_lock );

    return rc;
}

static int sbuf_fops_release( struct inode *inode, struct file *f )
{
    int rc = 0;
    struct sbuf_context *p_ctx = (struct sbuf_context *)f->private_data;

    LOG( LOG_INFO, "p_ctx: %p, name: %s, ctx_id: %d, minor_id: %d.", p_ctx, p_ctx->dev_name, p_ctx->ctx_id, p_ctx->dev_minor_id );

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed." );
        return rc;
    }

    if ( p_ctx->dev_opened ) {
        p_ctx->dev_opened = 0;
        f->private_data = NULL;
        p_ctx->sbuf_mgr.sbuf_base->kf_info.cali_info.is_fetched = 0; // user-app will need this to
                                                                     // read calibrations next time it runs
        sbuf_mgr_reset( &p_ctx->sbuf_mgr );
    } else {
        LOG( LOG_CRIT, "Fatal error: wrong state dev_opened: %d.", p_ctx->dev_opened );
        rc = -EINVAL;
    }

    mutex_unlock( &p_ctx->fops_lock );

    return 0;
}

static ssize_t sbuf_fops_write( struct file *file, const char __user *buf, size_t count, loff_t *ppos )
{
    int rc = 0;
    struct sbuf_context *p_ctx = (struct sbuf_context *)file->private_data;
    struct sbuf_idx_set idx_set = {0};
    uint32_t len_to_copy = sizeof( struct sbuf_idx_set );

    LOG( LOG_DEBUG, "p_ctx: %p, name: %s, ctx_id: %d, minor_id: %d.", p_ctx, p_ctx->dev_name, p_ctx->ctx_id, p_ctx->dev_minor_id );

    if ( count != len_to_copy ) {
        LOG( LOG_ERR, "Write size mismatch, size: %u, expected: %d.", (uint32_t)count, len_to_copy );
        return -EINVAL;
    }

    rc = copy_from_user( &idx_set, buf, len_to_copy );
    if ( rc ) {
        LOG( LOG_ERR, "copy_from_user failed, not copied: %d, expected: %u.", rc, len_to_copy );
    }

    LOG( LOG_DEBUG, "Ctx: %d, write idx_set: %u(%u)-%u(%u)-%u(%u)-%u(%u)-%u(%u).",
         p_ctx->ctx_id,
         idx_set.ae_idx_valid, idx_set.ae_idx,
         idx_set.awb_idx_valid, idx_set.awb_idx,
         idx_set.af_idx_valid, idx_set.af_idx,
         idx_set.gamma_idx_valid, idx_set.gamma_idx,
         idx_set.iridix_idx_valid, idx_set.iridix_idx );

    sbuf_mgr_apply_new_param( p_ctx, &idx_set );

    return rc ? rc : len_to_copy;
}

static ssize_t sbuf_fops_read( struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
    int rc;
    struct sbuf_context *p_ctx = (struct sbuf_context *)file->private_data;
    struct sbuf_idx_set idx_set;
    uint32_t len_to_copy = sizeof( struct sbuf_idx_set );

    LOG( LOG_DEBUG, "p_ctx: %p, name: %s, ctx_id: %d, minor_id: %d.", p_ctx,
         p_ctx->dev_name,
         p_ctx->ctx_id,
         p_ctx->dev_minor_id );

    if ( count != len_to_copy ) {
        LOG( LOG_ERR, "Read size mismatch, ctx: %d, size: %u, expected: %d.", p_ctx->ctx_id, (uint32_t)count, len_to_copy );
        return -EINVAL;
    }

    if ( sbuf_is_paused( p_ctx ) ) {
        return -ENODATA;
    }

    if ( !sbuf_is_ready_to_send_data( p_ctx ) ) {
        LOG( LOG_ERR, "Not ready to send data. (ctx: %d)", p_ctx->ctx_id );
        return -ENODATA;
    }

    /* Get latest sbuf index set, it will wait if no data available. */
    sbuf_mgr_get_latest_idx_set( p_ctx, &idx_set );

    // 2nd Check because sbuf_mgr_get_latest_idx_set() will wait for data available.
    if ( !sbuf_is_ready_to_send_data( p_ctx ) ) {
        // Recycle items to sbuf_mgr.
        sbuf_recycle_idx_set( p_ctx, &idx_set );

        LOG( LOG_ERR, "Not ready to send data. (ctx: %d)", p_ctx->ctx_id );
        return -ENODATA;
    }

    rc = copy_to_user( buf, &idx_set, len_to_copy );
    if ( rc ) {
        LOG( LOG_ERR, "copy_to_user failed, ctx: %d, rc: %d.", p_ctx->ctx_id, rc );
    }

    LOG( LOG_DEBUG, "ctx: %d, read idx_set: %u(%u)-%u(%u)-%u(%u)-%u(%u)-%u(%u).",
         p_ctx->ctx_id,
         idx_set.ae_idx_valid, idx_set.ae_idx,
         idx_set.awb_idx_valid, idx_set.awb_idx,
         idx_set.af_idx_valid, idx_set.af_idx,
         idx_set.gamma_idx_valid, idx_set.gamma_idx,
         idx_set.iridix_idx_valid, idx_set.iridix_idx );

    /* Each time when user-FW read the data, we should keep kf_info updated. */
    sbuf_update_kf_info( p_ctx );

    return rc ? rc : len_to_copy;
}


static int sbuf_fops_mmap( struct file *file, struct vm_area_struct *vma )
{
    unsigned long user_buf_len = vma->vm_end - vma->vm_start;
    int rc;
    struct sbuf_context *p_ctx = (struct sbuf_context *)file->private_data;
    struct sbuf_mgr *p_sbuf_mgr = &p_ctx->sbuf_mgr;

    LOG( LOG_DEBUG, "p_ctx: %p, name: %s, ctx_id: %d, minor_id: %d.", p_ctx, p_ctx->dev_name, p_ctx->ctx_id, p_ctx->dev_minor_id );

    LOG( LOG_INFO, "User app want to get %ld bytes.", user_buf_len );

    /*
     * The user_buf_len will be page aligned even if struct fw_sbuf is not
     * page aligned. In this case, the size may be unmatched, but the
     * delta should not exceed 1 page.
     */
    if ( ( user_buf_len != sizeof( struct fw_sbuf ) ) &&
         ( user_buf_len - sizeof( struct fw_sbuf ) >= PAGE_SIZE ) ) {

        LOG( LOG_ERR, "Not matched buf size, User app size: %ld, kernel sbuf size: %zu.",
             user_buf_len,
             sizeof( struct fw_sbuf ) );
        return -EINVAL;
    }

    if ( !is_sbuf_inited( p_sbuf_mgr ) ) {
        LOG( LOG_ERR, "Error: sbuf is not initialized, can't map." );
        return -ENOMEM;
    }

    /* Remap the kernel buffer into the user app address space. */
    rc = remap_pfn_range( vma, vma->vm_start, virt_to_phys( p_sbuf_mgr->buf_used ) >> PAGE_SHIFT, user_buf_len, vma->vm_page_prot );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "remap of sbuf failed, return: %d.", rc );
        return rc;
    }

    /* We initialize the kf_info values here so that it can be seen in user-FW. */
    sbuf_update_kf_info( p_ctx );

    LOG( LOG_INFO, "mmap of %ld bytes OK.", user_buf_len );
    return 0;
}


static struct file_operations sbuf_mgr_fops = {
    .owner = THIS_MODULE,
    .open = sbuf_fops_open,
    .release = sbuf_fops_release,
    .read = sbuf_fops_read,
    .write = sbuf_fops_write,
    .llseek = noop_llseek,
    .mmap = sbuf_fops_mmap,
};

static int sbuf_register_chardev( struct sbuf_context *p_ctx )
{
    struct miscdevice *p_dev = &p_ctx->sbuf_dev;
    int rc = 0;

    if ( p_dev->name == NULL ) {
        system_memset( p_dev, 0, sizeof( *p_dev ) );

        rc = snprintf( p_ctx->dev_name, SBUF_DEV_NAME_LEN, SBUF_DEV_FORMAT, p_ctx->ctx_id );
        if ( rc >= SBUF_DEV_NAME_LEN ) {
            // Output name truncated
            rc = -ENAMETOOLONG;
        }
        if ( rc < 0 ) {
            return rc;
        }

        p_dev->name = p_ctx->dev_name;
        p_dev->minor = MISC_DYNAMIC_MINOR,
        p_dev->fops = &sbuf_mgr_fops;

        rc = misc_register( p_dev );
        if ( rc ) {
            // Error
            p_dev->name = NULL;
            return rc;
        }

        p_ctx->dev_minor_id = p_dev->minor;
        p_ctx->dev_opened = 0;
    }

    return rc;
}


void sbuf_config( sbuf_fsm_ptr_t p_fsm )
{
    uint32_t ctx_id = ACAMERA_FSM2ICTX_PTR( p_fsm )->context_id;
    struct sbuf_context *p_ctx = NULL;
    int rc;

    LOG( LOG_INFO, "===========> +++ sbuf_update_calibration_data" );

    p_ctx = &( sbuf_contexts[ctx_id] );
    if ( p_ctx->ctx_id != ctx_id ) {
        LOG( LOG_CRIT, "Error: ctx_id does not match, fsm ctx_id: %d, ctx_id: %d.", ctx_id, p_ctx->ctx_id );
        return;
    }

    // Prepare calibration for user-driver
    sbuf_calibration_init( p_ctx );

    rc = sbuf_register_chardev( p_ctx );
    if ( rc ) {
        LOG( LOG_ERR, "Register failed, error: register sbuf device failed, ret: %d.", rc );
    }

    LOG( LOG_INFO, "--- sbuf_update_calibration_data" );
}

void sbuf_fsm_init( sbuf_fsm_t *p_fsm )
{ // TODO: need to make the function able to be called multiple times (misc_register...).
    int rc;
    uint32_t ctx_id = ACAMERA_FSM_GET_FW_ID( p_fsm );
    struct sbuf_context *p_ctx;
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    LOG( LOG_INFO, "init sbuf FSM for ctx_id: %u.", ctx_id );

    LOG( LOG_DEBUG, "struct kf_info %zu ",
         sizeof( struct kf_info ) );
#if defined( ISP_HAS_AE_MANUAL_FSM )
    LOG( LOG_DEBUG, "sbuf_ae_t %zu x %u ",
         sizeof( sbuf_ae_t ),
         SBUF_STATS_ARRAY_SIZE );
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
    LOG( LOG_DEBUG, "sbuf_awb_t %zu x %u ",
         sizeof( sbuf_awb_t ),
         SBUF_STATS_ARRAY_SIZE );
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
    LOG( LOG_DEBUG, "sbuf_gamma_t %zu x %u ",
         sizeof( sbuf_gamma_t ),
         SBUF_STATS_ARRAY_SIZE );
#endif

#if defined( ISP_HAS_IRIDIX_HIST_FSM ) || defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    LOG( LOG_DEBUG, "sbuf_iridix_t %zu x %u ",
         sizeof( sbuf_iridix_t ),
         SBUF_STATS_ARRAY_SIZE );
#endif

    if ( ctx_id >= context_number ) {
        LOG( LOG_CRIT, "Fatal error: Invalid FW context ID: %u, max is: %d.", ctx_id, context_number - 1 );
        return;
    }

    p_ctx = &( sbuf_contexts[ctx_id] );
    system_memset( p_ctx, 0, sizeof( *p_ctx ) );

    p_ctx->ctx_id = ctx_id;
    p_ctx->p_fsm = p_fsm;

    rc = sbuf_ctx_init( p_ctx );
    if ( rc ) {
        LOG( LOG_ERR, "Init failed (ctx %d), ret: %d.", ctx_id, rc );
        return;
    }

    mutex_init( &p_ctx->fops_lock );
    mutex_init( &p_ctx->idx_set_lock );
    init_waitqueue_head( &p_ctx->idx_set_wait_queue );

    return;
}

void sbuf_deinit( sbuf_fsm_ptr_t p_fsm )
{
    uint32_t ctx_id = ACAMERA_FSM_GET_FW_ID( p_fsm );
    struct miscdevice *p_dev;
    struct sbuf_context *p_ctx;
    uint32_t context_number = FIRMWARE_CONTEXT_NUMBER;

    if ( ctx_id >= context_number ) {
        LOG( LOG_CRIT, "Fatal error: Invalid FW context ID: %u, max is: %d.", ctx_id, context_number - 1 );
        return;
    }

    p_ctx = &( sbuf_contexts[ctx_id] );
    p_dev = &p_ctx->sbuf_dev;

    if ( !p_dev->name ) {
        LOG( LOG_INFO, "Skip sbuf[%d] deregister due to NULL name.", ctx_id );
        return;
    } else {
        misc_deregister( p_dev );
        LOG( LOG_INFO, "Deregister sbuf dev '%s' ok.", p_dev->name );
    }

    LOG( LOG_INFO, "Deinit sbuf_context: %d). ctx_id: %d, dev: %s, minor_id: %d.",
         ctx_id,
         p_ctx->ctx_id,
         p_ctx->dev_name,
         p_ctx->dev_minor_id );

    sbuf_mgr_free( &p_ctx->sbuf_mgr );
}
