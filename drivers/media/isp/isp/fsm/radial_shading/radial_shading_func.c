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

#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "acamera_radial_shading_mem_config.h"


static void radial_shading_lut_reload( radial_shading_fsm_ptr_t p_fsm )
{
    uint32_t i;

    acamera_calib_mgr_entry_t *cm_ptr = ACAMERA_FSM2CM_PTR( p_fsm );
    const int32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;

    // Radial shading lut: 32bit, 225x1: 0:32 Red, 64:96 Green, 128:160 Blue, 192:225 IR.
    const uint32_t *radial_shading_lut_r = calib_mgr_u32_lut_get( cm_ptr, CALIBRATION_SHADING_RADIAL_R );
    const uint32_t *radial_shading_lut_g = calib_mgr_u32_lut_get( cm_ptr, CALIBRATION_SHADING_RADIAL_G );
    const uint32_t *radial_shading_lut_b = calib_mgr_u32_lut_get( cm_ptr, CALIBRATION_SHADING_RADIAL_B );
    const uint32_t *radial_shading_lut_cm = calib_mgr_u32_lut_get( cm_ptr, CALIBRATION_SHADING_RADIAL_CENTRE_AND_MULT );
    const uint32_t radial_shading_lut_r_sz = calib_mgr_lut_len( cm_ptr, CALIBRATION_SHADING_RADIAL_R );
    const uint32_t radial_shading_lut_g_sz = calib_mgr_lut_len( cm_ptr, CALIBRATION_SHADING_RADIAL_G );
    const uint32_t radial_shading_lut_b_sz = calib_mgr_lut_len( cm_ptr, CALIBRATION_SHADING_RADIAL_B );

// Moss has no IR radial shading
#if ( ISP_RTL_VERSION_R != 2 )
    const uint32_t radial_shading_lut_ir_sz = calib_mgr_lut_len( cm_ptr, CALIBRATION_SHADING_RADIAL_IR );
    if ( radial_shading_lut_ir_sz != 33 ) {
        LOG( LOG_ERR, "CALIBRATION_SHADING_RADIAL_IR has wrong size %d", radial_shading_lut_ir_sz );
    } else {
        const uint32_t *radial_shading_lut_ir = calib_mgr_u32_lut_get( cm_ptr, CALIBRATION_SHADING_RADIAL_IR );
        for ( i = 0; i < radial_shading_lut_ir_sz; i++ )
            acamera_radial_shading_mem_array_data_write( isp_base, 192 + i, radial_shading_lut_ir[i] );
    }
    uint32_t expected_radial_shading_lut_cm_sz = 16;
#else
    uint32_t expected_radial_shading_lut_cm_sz = 12;
#endif

    const uint32_t radial_shading_lut_cm_sz = calib_mgr_lut_len( cm_ptr, CALIBRATION_SHADING_RADIAL_CENTRE_AND_MULT );
    if ( radial_shading_lut_r_sz != 33 || radial_shading_lut_g_sz != 33 || radial_shading_lut_b_sz != 33 ) {
        LOG( LOG_ERR, "CALIBRATION_SHADING_RADIAL_R/G/B have wrong sizes R %d, G %d, B %d", radial_shading_lut_r_sz, radial_shading_lut_g_sz, radial_shading_lut_b_sz );
    } else {
        for ( i = 0; i < radial_shading_lut_r_sz; i++ )
            acamera_radial_shading_mem_array_data_write( isp_base, 0 + i, radial_shading_lut_r[i] );
        for ( i = 0; i < radial_shading_lut_g_sz; i++ )
            acamera_radial_shading_mem_array_data_write( isp_base, 64 + i, radial_shading_lut_g[i] );
        for ( i = 0; i < radial_shading_lut_b_sz; i++ )
            acamera_radial_shading_mem_array_data_write( isp_base, 128 + i, radial_shading_lut_b[i] );
    }

    // New 16 elements format used:
    // RX RY MULTRX MULTRY GX GY MULTGX MULTGY BX BY MULTBX MULTBY IRX IRY MULTIRX MULTIRY
    if ( radial_shading_lut_cm_sz != expected_radial_shading_lut_cm_sz ) {
        LOG( LOG_ERR, "CALIBRATION_SHADING_RADIAL_CENTRE_AND_MULT has wrong size: expected %d got %d", expected_radial_shading_lut_cm_sz, radial_shading_lut_cm_sz );
    } else {
        acamera_isp_radial_shading_centerr_x_write( isp_base, radial_shading_lut_cm[0] );
        acamera_isp_radial_shading_centerr_y_write( isp_base, radial_shading_lut_cm[1] );
        acamera_isp_radial_shading_off_center_multrx_write( isp_base, radial_shading_lut_cm[2] );
        acamera_isp_radial_shading_off_center_multry_write( isp_base, radial_shading_lut_cm[3] );

        acamera_isp_radial_shading_centerg_x_write( isp_base, radial_shading_lut_cm[4] );
        acamera_isp_radial_shading_centerg_y_write( isp_base, radial_shading_lut_cm[5] );
        acamera_isp_radial_shading_off_center_multgx_write( isp_base, radial_shading_lut_cm[6] );
        acamera_isp_radial_shading_off_center_multgy_write( isp_base, radial_shading_lut_cm[7] );

        acamera_isp_radial_shading_centerb_x_write( isp_base, radial_shading_lut_cm[8] );
        acamera_isp_radial_shading_centerb_y_write( isp_base, radial_shading_lut_cm[9] );
        acamera_isp_radial_shading_off_center_multbx_write( isp_base, radial_shading_lut_cm[10] );
        acamera_isp_radial_shading_off_center_multby_write( isp_base, radial_shading_lut_cm[11] );

// Moss has no IR shading correction
#if ( ISP_RTL_VERSION_R != 2 )
        acamera_isp_radial_shading_centerir_x_write( isp_base, radial_shading_lut_cm[12] );
        acamera_isp_radial_shading_centerir_y_write( isp_base, radial_shading_lut_cm[13] );
        acamera_isp_radial_shading_off_center_multirx_write( isp_base, radial_shading_lut_cm[14] );
        acamera_isp_radial_shading_off_center_multiry_write( isp_base, radial_shading_lut_cm[15] );
#endif
    }
}

void radial_shading_init( radial_shading_fsm_ptr_t p_fsm )
{
    const int32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    acamera_isp_pipeline_bypass_radial_shading_write( isp_base, 0 );
}

void radial_shading_reload_calibration( radial_shading_fsm_ptr_t p_fsm )
{
    radial_shading_lut_reload( p_fsm );
}

void radial_shading_deinit( radial_shading_fsm_ptr_t p_fsm )
{
    const int32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    acamera_isp_pipeline_bypass_radial_shading_write( isp_base, 1 );
}
