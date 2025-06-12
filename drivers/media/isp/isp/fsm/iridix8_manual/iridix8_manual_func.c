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
#include "acamera_logger.h"
#include "acamera_math.h"
#include "sbuf.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_IRIDIX8_MANUAL


////////////////////////////////////////////////////
// FSM event handlers

void iridix_config( iridix_fsm_t *p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    // Initialize parameters
    set_context_param( p_ictx, SYSTEM_IRIDIX_STRENGTH_TARGET_PARAM, p_fsm->strength_target );
    set_context_param( p_ictx, SYSTEM_MAXIMUM_IRIDIX_STRENGTH_PARAM, calib_mgr_u8_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_STRENGTH_MAXIMUM )[0] );

    // TODO: to be deleted.
    p_fsm->repeat_irq_mask = 0;
    iridix_request_interrupt( p_fsm, p_fsm->repeat_irq_mask );
}

void iridix_reload_calibration( iridix_fsm_t *p_fsm )
{
    uint16_t i;

    /* Configure the IRIDIX*/
    if ( calib_mgr_lut_exists( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX8_EXTENDED_CONTROL ) ) {
        const uint32_t *config = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX8_EXTENDED_CONTROL );
        acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

        /* Do not change the order of this init sequence. */
        acamera_isp_iridix_filter_mux_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_svariance_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_bright_pr_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_contrast_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_strength_inroi_write( p_ictx->settings.isp_base, *config++ );
        acamera_isp_iridix_strength_outroi_write( p_ictx->settings.isp_base, *config++ );
        acamera_isp_iridix_variance_intensity_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_variance_space_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_fwd_percept_control_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_rev_percept_control_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_slope_min_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        acamera_isp_iridix_slope_max_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
#if ( ISP_RTL_VERSION_R < 2 )
        acamera_isp_iridix_max_alg_type_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
#else
        /* max_alg_type not present in Moss */
        ++config;
#endif
        acamera_isp_iridix_white_level_write( p_ictx->settings.isp_base, (uint32_t)*config++ );
// Dynamically calibrated ivariance, luma_th and luma_slope are available only for Moss
#if ( ISP_RTL_VERSION_R == 2 )
        acamera_isp_iridix_ivariance_write( p_ictx->settings.isp_base, (uint8_t)*config++ );
        p_fsm->luma_slope = *config++;
        p_fsm->luma_th = *config++;
#endif
    } else {
        LOG( LOG_ERR, "CALIBRATION_IRIDIX8_EXTENDED_CONTROL is missing, module will not be configured correctly!" );
    }


    const uint32_t lut_element_size = calib_mgr_lut_width( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_ASYMMETRY );

    // 32 bit tables
    if ( lut_element_size == 4 ) {

        const uint32_t *lut_data = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_ASYMMETRY );
        for ( i = 0; i < calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_ASYMMETRY ); i++ ) {
            acamera_isp_iridix_lut_asymmetry_lut_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, i, lut_data[i] );
        }
        // 16 bit tables
    } else if ( lut_element_size == 2 ) {

        const uint16_t *lut_data = calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_ASYMMETRY );
        for ( i = 0; i < calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_ASYMMETRY ); i++ ) {
            acamera_isp_iridix_lut_asymmetry_lut_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, i, lut_data[i] );
        }
    } else {
        LOG( LOG_ERR, "Unsupported CALIBRATION_IRIDIX_ASYMMETRY LUT element size: expected 4 or 2 bytes, got %d bytes", lut_element_size );
    }

    // Update gtm tables
    const uint32_t *lut_gtm_x = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_GTM_LUT_X );
    const uint32_t *lut_gtm_y = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_GTM_LUT_Y );

    for ( i = 0; i < calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_GTM_LUT_X ); i++ ) {
        acamera_isp_iridix_lut_globaltm_x_lut_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, i, lut_gtm_x[i] );
    }

    for ( i = 0; i < calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_GTM_LUT_Y ); i++ ) {
        acamera_isp_iridix_lut_globaltm_y_lut_write( ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base, i, lut_gtm_y[i] );
    }
}

/* This function just gets an empty sbuf and sets its status to done,
 * so that it keeps the sbuf status to loop correctly.
 */
void iridix_update_algo( iridix_fsm_t *p_fsm )
{
    struct sbuf_item sbuf;

    uint32_t fw_id = ACAMERA_FSM_GET_FW_ID( p_fsm );

    system_memset( &sbuf, 0, sizeof( sbuf ) );
    sbuf.buf_type = SBUF_TYPE_IRIDIX;
    sbuf.buf_status = SBUF_STATUS_DATA_EMPTY;

    if ( sbuf_get_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to get iridix sbuf, return." );
        return;
    }

    sbuf_iridix_t *p_sbuf_iridix = (sbuf_iridix_t *)sbuf.buf_base;
    (void)p_sbuf_iridix->digital_gain; //Ignore digital gain, currently unused.

    sbuf.buf_status = SBUF_STATUS_DATA_DONE;
    if ( sbuf_set_item( fw_id, &sbuf ) ) {
        LOG( LOG_ERR, "Error: Failed to set sbuf, return." );
    }
}

void iridix_update_hw( iridix_fsm_t *p_fsm )
{
    const acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );
    const uint16_t iridix_digital_gain = get_context_param( p_ictx, SYSTEM_IRIDIX_DIGITAL_GAIN_PARAM );
    const uint16_t str_min = (uint16_t)get_context_param( p_ictx, SYSTEM_MINIMUM_IRIDIX_STRENGTH_PARAM ) << 8;
    const uint16_t str_max = (uint16_t)get_context_param( p_ictx, SYSTEM_MAXIMUM_IRIDIX_STRENGTH_PARAM ) << 8;

    // Manual mode operation.
    iridix_update_algo( p_fsm );

    if ( get_context_param( p_ictx, ISP_MODULES_MANUAL_IRIDIX_PARAM ) ) {
        /* In manual mode we overwrite the values passed via sbuf with values from the API*/
        p_fsm->strength_target = get_context_param( p_ictx, SYSTEM_IRIDIX_STRENGTH_TARGET_PARAM ) << 8;
    }

    /* Limit the value */
    p_fsm->strength_target = MAX( p_fsm->strength_target, str_min );
    p_fsm->strength_target = MIN( p_fsm->strength_target, str_max );

    /* We write back the value to the API to reflect our changes. */
    set_context_param( p_ictx, SYSTEM_IRIDIX_STRENGTH_TARGET_PARAM, p_fsm->strength_target >> 8 );
    override_context_param( p_ictx, STATUS_INFO_AWB_MIX_LIGHT_CONTRAST_PARAM, p_fsm->iridix_contrast );

    {
        acamera_isp_digital_gain_iridix_gain_write( p_ictx->settings.isp_base, iridix_digital_gain );

        exposure_set_t exp_set_fr_next;
        exposure_set_t exp_set_fr_prev;
        int32_t frame_next = 1, frame_prev = 0;

        WRAP_GENERAL_CMD( p_ictx, CMD_ID_FRAME_EXPOSURE_SET, CMD_DIRECTION_GET, (const uint32_t *)&frame_next, (uint32_t *)&exp_set_fr_next );
        WRAP_GENERAL_CMD( p_ictx, CMD_ID_FRAME_EXPOSURE_SET, CMD_DIRECTION_GET, (const uint32_t *)&frame_prev, (uint32_t *)&exp_set_fr_prev );

        int32_t diff = math_exp2( exp_set_fr_prev.info.exposure_log2 - exp_set_fr_next.info.exposure_log2, LOG2_GAIN_SHIFT, 8 );

        if ( diff < 0 )
            diff = 256;
        if ( diff >= ( 1 << ACAMERA_ISP_IRIDIX_COLLECTION_CORRECTION_DATASIZE ) )
            diff = ( 1 << ACAMERA_ISP_IRIDIX_COLLECTION_CORRECTION_DATASIZE );

        acamera_isp_iridix_collection_correction_write( p_ictx->settings.isp_base, diff );

        // Time filter for iridix strength.
        const uint8_t iridix_avg_coeff = calib_mgr_u8_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_IRIDIX_AVG_COEF )[0];
        uint16_t iridix_strength = p_fsm->strength_target;
        if ( iridix_avg_coeff > 1 ) {
            ( (iridix_fsm_ptr_t)p_fsm )->strength_avg += p_fsm->strength_target - p_fsm->strength_avg / iridix_avg_coeff; // Division by zero is checked.
            iridix_strength = ( uint16_t )( p_fsm->strength_avg / iridix_avg_coeff );                                     // Division by zero is checked.
        }

        acamera_isp_iridix_strength_inroi_write( p_ictx->settings.isp_base, iridix_strength >> 6 );
        acamera_isp_iridix_dark_enh_write( p_ictx->settings.isp_base, p_fsm->dark_enh );
        acamera_isp_iridix_slope_max_write( p_ictx->settings.isp_base, MIN( p_fsm->dark_enh, 255 ) );
    }
}


////////////////////////////////////////////////////
// FSM interrupt handler

// TODO: To be deleted
void iridix_fsm_process_interrupt( iridix_fsm_const_ptr_t p_fsm, uint8_t irq_event )
{
}
