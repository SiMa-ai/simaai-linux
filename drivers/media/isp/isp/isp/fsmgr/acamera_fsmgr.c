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
#if ACAMERA_ISP_PROFILING
#include "acamera_profiler.h"
#endif

#include "acamera_logger.h"

void acamera_fsmgr_init( acamera_fsmgr_t *p_fsmgr )
{
    LOG( LOG_INFO, "%s E", __FUNCTION__ );
    p_fsmgr->sensor_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->sensor_fsm.state = sensor_state_invalid;
    sensor_fsm_clear( &( p_fsmgr->sensor_fsm ) );
    p_fsmgr->control_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->control_fsm.state = control_state_invalid;
    control_fsm_clear( &( p_fsmgr->control_fsm ) );
    p_fsmgr->decompander_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->decompander_fsm.state = decompander_state_invalid;
    decompander_fsm_clear( &( p_fsmgr->decompander_fsm ) );
    p_fsmgr->frame_stitch_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->frame_stitch_fsm.state = frame_stitch_state_invalid;
    frame_stitch_fsm_clear( &( p_fsmgr->frame_stitch_fsm ) );
    p_fsmgr->output_formatter_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->output_formatter_fsm.state = output_formatter_state_invalid;
    output_formatter_fsm_clear( &( p_fsmgr->output_formatter_fsm ) );
    p_fsmgr->crop_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->crop_fsm.state = crop_state_invalid;
    crop_fsm_clear( &( p_fsmgr->crop_fsm ) );
    p_fsmgr->cmos_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->cmos_fsm.state = cmos_state_invalid;
    cmos_fsm_clear( &( p_fsmgr->cmos_fsm ) );
    p_fsmgr->AE_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->AE_fsm.state = AE_state_invalid;
    AE_fsm_clear( &( p_fsmgr->AE_fsm ) );
    p_fsmgr->histogram_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->histogram_fsm.state = histogram_state_invalid;
    histogram_fsm_clear( &( p_fsmgr->histogram_fsm ) );
    p_fsmgr->AWB_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->AWB_fsm.state = AWB_state_invalid;
    AWB_fsm_clear( &( p_fsmgr->AWB_fsm ) );
    p_fsmgr->color_matrix_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->color_matrix_fsm.state = color_matrix_state_invalid;
    color_matrix_fsm_clear( &( p_fsmgr->color_matrix_fsm ) );
    p_fsmgr->mesh_shading_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->mesh_shading_fsm.state = mesh_shading_state_invalid;
    mesh_shading_fsm_clear( &( p_fsmgr->mesh_shading_fsm ) );
    p_fsmgr->iridix_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->iridix_fsm.state = iridix_state_invalid;
    iridix_fsm_clear( &( p_fsmgr->iridix_fsm ) );
    p_fsmgr->mcfe_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->mcfe_fsm.state = mcfe_state_invalid;
    mcfe_fsm_clear( &( p_fsmgr->mcfe_fsm ) );
    p_fsmgr->frame_check_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->frame_check_fsm.state = frame_check_state_invalid;
    frame_check_fsm_clear( &( p_fsmgr->frame_check_fsm ) );
    p_fsmgr->metadata_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->metadata_fsm.state = metadata_state_invalid;
    metadata_fsm_clear( &( p_fsmgr->metadata_fsm ) );
    p_fsmgr->sharpening_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->sharpening_fsm.state = sharpening_state_invalid;
    sharpening_fsm_clear( &( p_fsmgr->sharpening_fsm ) );
    p_fsmgr->noise_reduction_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->noise_reduction_fsm.state = noise_reduction_state_invalid;
    noise_reduction_fsm_clear( &( p_fsmgr->noise_reduction_fsm ) );
    p_fsmgr->defect_pixel_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->defect_pixel_fsm.state = defect_pixel_state_invalid;
    defect_pixel_fsm_clear( &( p_fsmgr->defect_pixel_fsm ) );
    p_fsmgr->sbuf_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->sbuf_fsm.state = sbuf_state_invalid;
    sbuf_fsm_clear( &( p_fsmgr->sbuf_fsm ) );
    p_fsmgr->isp_wrapper_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->isp_wrapper_fsm.state = isp_wrapper_state_invalid;
    isp_wrapper_fsm_clear( &( p_fsmgr->isp_wrapper_fsm ) );
    p_fsmgr->cac_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->cac_fsm.state = cac_state_invalid;
    cac_fsm_clear( &( p_fsmgr->cac_fsm ) );
    p_fsmgr->radial_shading_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->radial_shading_fsm.state = radial_shading_state_invalid;
    radial_shading_fsm_clear( &( p_fsmgr->radial_shading_fsm ) );
    p_fsmgr->noise_profile_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->noise_profile_fsm.state = noise_profile_state_invalid;
    noise_profile_fsm_clear( &( p_fsmgr->noise_profile_fsm ) );
    p_fsmgr->demosaic_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->demosaic_fsm.state = demosaic_state_invalid;
    demosaic_fsm_clear( &( p_fsmgr->demosaic_fsm ) );
    p_fsmgr->ml_bist_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->ml_bist_fsm.state = ml_bist_state_invalid;
    ml_bist_fsm_clear( &( p_fsmgr->ml_bist_fsm ) );
    p_fsmgr->raw_fe_fsm.p_fsmgr = p_fsmgr;
    p_fsmgr->raw_fe_fsm.state = raw_fe_state_invalid;
    raw_fe_fsm_clear( &( p_fsmgr->raw_fe_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SENSOR_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    sensor_fsm_switch_state( &( p_fsmgr->sensor_fsm ), sensor_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SENSOR_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CONTROL_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    control_fsm_switch_state( &( p_fsmgr->control_fsm ), control_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CONTROL_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DECOMPANDER_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    decompander_fsm_switch_state( &( p_fsmgr->decompander_fsm ), decompander_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DECOMPANDER_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( FRAME_STITCH_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    frame_stitch_fsm_switch_state( &( p_fsmgr->frame_stitch_fsm ), frame_stitch_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( FRAME_STITCH_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( OUTPUT_FORMATTER_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    output_formatter_fsm_switch_state( &( p_fsmgr->output_formatter_fsm ), output_formatter_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( OUTPUT_FORMATTER_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CROP_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    crop_fsm_switch_state( &( p_fsmgr->crop_fsm ), crop_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CROP_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CMOS_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    cmos_fsm_switch_state( &( p_fsmgr->cmos_fsm ), cmos_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CMOS_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( AE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    AE_fsm_switch_state( &( p_fsmgr->AE_fsm ), AE_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( AE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if 0
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( HISTOGRAM_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    histogram_fsm_switch_state( &( p_fsmgr->histogram_fsm ), histogram_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( HISTOGRAM_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#endif
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( AWB_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    AWB_fsm_switch_state( &( p_fsmgr->AWB_fsm ), AWB_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( AWB_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( COLOR_MATRIX_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    color_matrix_fsm_switch_state( &( p_fsmgr->color_matrix_fsm ), color_matrix_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( COLOR_MATRIX_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( MESH_SHADING_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    mesh_shading_fsm_switch_state( &( p_fsmgr->mesh_shading_fsm ), mesh_shading_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( MESH_SHADING_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( IRIDIX_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    iridix_fsm_switch_state( &( p_fsmgr->iridix_fsm ), iridix_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( IRIDIX_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( MCFE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    mcfe_fsm_switch_state( &( p_fsmgr->mcfe_fsm ), mcfe_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( MCFE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if 0
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( FRAME_CHECK_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    frame_check_fsm_switch_state( &( p_fsmgr->frame_check_fsm ), frame_check_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( FRAME_CHECK_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( METADATA_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    metadata_fsm_switch_state( &( p_fsmgr->metadata_fsm ), metadata_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( METADATA_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#endif

#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SHARPENING_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    sharpening_fsm_switch_state( &( p_fsmgr->sharpening_fsm ), sharpening_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SHARPENING_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( NOISE_REDUCTION_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    noise_reduction_fsm_switch_state( &( p_fsmgr->noise_reduction_fsm ), noise_reduction_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( NOISE_REDUCTION_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DEFECT_PIXEL_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    defect_pixel_fsm_switch_state( &( p_fsmgr->defect_pixel_fsm ), defect_pixel_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DEFECT_PIXEL_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SBUF_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    sbuf_fsm_switch_state( &( p_fsmgr->sbuf_fsm ), sbuf_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SBUF_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( ISP_WRAPPER_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    isp_wrapper_fsm_switch_state( &( p_fsmgr->isp_wrapper_fsm ), isp_wrapper_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( ISP_WRAPPER_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CAC_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    cac_fsm_switch_state( &( p_fsmgr->cac_fsm ), cac_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CAC_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( RADIAL_SHADING_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    radial_shading_fsm_switch_state( &( p_fsmgr->radial_shading_fsm ), radial_shading_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( RADIAL_SHADING_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( NOISE_PROFILE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    noise_profile_fsm_switch_state( &( p_fsmgr->noise_profile_fsm ), noise_profile_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( NOISE_PROFILE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DEMOSAIC_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    demosaic_fsm_switch_state( &( p_fsmgr->demosaic_fsm ), demosaic_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DEMOSAIC_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if 0
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( ML_BIST_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    ml_bist_fsm_switch_state( &( p_fsmgr->ml_bist_fsm ), ml_bist_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( ML_BIST_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#endif

#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( RAW_FE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    raw_fe_fsm_switch_state( &( p_fsmgr->raw_fe_fsm ), raw_fe_state_initialized );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( RAW_FE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SENSOR_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    sensor_fsm_process_state( &( p_fsmgr->sensor_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SENSOR_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CONTROL_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    control_fsm_process_state( &( p_fsmgr->control_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CONTROL_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DECOMPANDER_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    decompander_fsm_process_state( &( p_fsmgr->decompander_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DECOMPANDER_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( FRAME_STITCH_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    frame_stitch_fsm_process_state( &( p_fsmgr->frame_stitch_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( FRAME_STITCH_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( OUTPUT_FORMATTER_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    output_formatter_fsm_process_state( &( p_fsmgr->output_formatter_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( OUTPUT_FORMATTER_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CROP_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    crop_fsm_process_state( &( p_fsmgr->crop_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CROP_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CMOS_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    cmos_fsm_process_state( &( p_fsmgr->cmos_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CMOS_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( AE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    AE_fsm_process_state( &( p_fsmgr->AE_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( AE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if 0
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( HISTOGRAM_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    histogram_fsm_process_state( &( p_fsmgr->histogram_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( HISTOGRAM_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#endif

#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( AWB_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    AWB_fsm_process_state( &( p_fsmgr->AWB_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( AWB_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( COLOR_MATRIX_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    color_matrix_fsm_process_state( &( p_fsmgr->color_matrix_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( COLOR_MATRIX_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( MESH_SHADING_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    mesh_shading_fsm_process_state( &( p_fsmgr->mesh_shading_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( MESH_SHADING_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( IRIDIX_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    iridix_fsm_process_state( &( p_fsmgr->iridix_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( IRIDIX_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( MCFE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    mcfe_fsm_process_state( &( p_fsmgr->mcfe_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( MCFE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if 0
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( FRAME_CHECK_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    frame_check_fsm_process_state( &( p_fsmgr->frame_check_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( FRAME_CHECK_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( METADATA_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    metadata_fsm_process_state( &( p_fsmgr->metadata_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( METADATA_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#endif

#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SHARPENING_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    sharpening_fsm_process_state( &( p_fsmgr->sharpening_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SHARPENING_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( NOISE_REDUCTION_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    noise_reduction_fsm_process_state( &( p_fsmgr->noise_reduction_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( NOISE_REDUCTION_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DEFECT_PIXEL_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    defect_pixel_fsm_process_state( &( p_fsmgr->defect_pixel_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DEFECT_PIXEL_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SBUF_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    sbuf_fsm_process_state( &( p_fsmgr->sbuf_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SBUF_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( ISP_WRAPPER_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    isp_wrapper_fsm_process_state( &( p_fsmgr->isp_wrapper_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( ISP_WRAPPER_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CAC_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    cac_fsm_process_state( &( p_fsmgr->cac_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CAC_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( RADIAL_SHADING_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    radial_shading_fsm_process_state( &( p_fsmgr->radial_shading_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( RADIAL_SHADING_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( NOISE_PROFILE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    noise_profile_fsm_process_state( &( p_fsmgr->noise_profile_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( NOISE_PROFILE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DEMOSAIC_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    demosaic_fsm_process_state( &( p_fsmgr->demosaic_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DEMOSAIC_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#if 0
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( ML_BIST_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    ml_bist_fsm_process_state( &( p_fsmgr->ml_bist_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( ML_BIST_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
#endif
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( RAW_FE_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
    raw_fe_fsm_process_state( &( p_fsmgr->raw_fe_fsm ) );
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( RAW_FE_PERF_SECTION, 1 );
#endif /* ACAMERA_ISP_PROFILING */
    LOG( LOG_INFO, "%s X", __FUNCTION__ );
}
void acamera_fsmgr_process_interrupt( acamera_fsmgr_t *p_fsmgr, uint8_t irq_event )
{
    system_fw_interrupt_mask_t fsm_mask, mask = 1 << irq_event;
    fsm_mask = p_fsmgr->cmos_fsm.irq_mask & mask;
    if ( fsm_mask ) {
#if ACAMERA_ISP_PROFILING
        acamera_profiler_start( CMOS_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
        p_fsmgr->cmos_fsm.irq_mask &= ( ~fsm_mask ) | ( p_fsmgr->cmos_fsm.repeat_irq_mask );
        cmos_fsm_process_interrupt( &( p_fsmgr->cmos_fsm ), irq_event );
#if ACAMERA_ISP_PROFILING
        acamera_profiler_stop( CMOS_PERF_SECTION, 0 );
#endif /* ACAMERA_ISP_PROFILING */
    }
#if 0
    fsm_mask = p_fsmgr->histogram_fsm.irq_mask & mask;
    if ( fsm_mask ) {
#if ACAMERA_ISP_PROFILING
        acamera_profiler_start( HISTOGRAM_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
        p_fsmgr->histogram_fsm.irq_mask &= ( ~fsm_mask ) | ( p_fsmgr->histogram_fsm.repeat_irq_mask );
        histogram_fsm_process_interrupt( &( p_fsmgr->histogram_fsm ), irq_event );
#if ACAMERA_ISP_PROFILING
        acamera_profiler_stop( HISTOGRAM_PERF_SECTION, 0 );
#endif /* ACAMERA_ISP_PROFILING */
    }
#endif
    fsm_mask = p_fsmgr->AWB_fsm.irq_mask & mask;
    if ( fsm_mask ) {
#if ACAMERA_ISP_PROFILING
        acamera_profiler_start( AWB_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
        p_fsmgr->AWB_fsm.irq_mask &= ( ~fsm_mask ) | ( p_fsmgr->AWB_fsm.repeat_irq_mask );
        AWB_fsm_process_interrupt( &( p_fsmgr->AWB_fsm ), irq_event );
#if ACAMERA_ISP_PROFILING
        acamera_profiler_stop( AWB_PERF_SECTION, 0 );
#endif /* ACAMERA_ISP_PROFILING */
    }
    fsm_mask = p_fsmgr->iridix_fsm.irq_mask & mask;
    if ( fsm_mask ) {
#if ACAMERA_ISP_PROFILING
        acamera_profiler_start( IRIDIX_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
        p_fsmgr->iridix_fsm.irq_mask &= ( ~fsm_mask ) | ( p_fsmgr->iridix_fsm.repeat_irq_mask );
        iridix_fsm_process_interrupt( &( p_fsmgr->iridix_fsm ), irq_event );
#if ACAMERA_ISP_PROFILING
        acamera_profiler_stop( IRIDIX_PERF_SECTION, 0 );
#endif /* ACAMERA_ISP_PROFILING */
    }
#if 0
    fsm_mask = p_fsmgr->metadata_fsm.irq_mask & mask;
    if ( fsm_mask ) {
#if ACAMERA_ISP_PROFILING
        acamera_profiler_start( METADATA_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
        p_fsmgr->metadata_fsm.irq_mask &= ( ~fsm_mask ) | ( p_fsmgr->metadata_fsm.repeat_irq_mask );
        metadata_fsm_process_interrupt( &( p_fsmgr->metadata_fsm ), irq_event );
#if ACAMERA_ISP_PROFILING
        acamera_profiler_stop( METADATA_PERF_SECTION, 0 );
#endif /* ACAMERA_ISP_PROFILING */
    }
    fsm_mask = p_fsmgr->ml_bist_fsm.irq_mask & mask;
    if ( fsm_mask ) {
#if ACAMERA_ISP_PROFILING
        acamera_profiler_start( ML_BIST_PERF_SECTION );
#endif /* ACAMERA_ISP_PROFILING */
        p_fsmgr->ml_bist_fsm.irq_mask &= ( ~fsm_mask ) | ( p_fsmgr->ml_bist_fsm.repeat_irq_mask );
        ml_bist_fsm_process_interrupt( &( p_fsmgr->ml_bist_fsm ), irq_event );
#if ACAMERA_ISP_PROFILING
        acamera_profiler_stop( ML_BIST_PERF_SECTION, 0 );
#endif /* ACAMERA_ISP_PROFILING */
    }
#endif
}

static const char *const event_name[] = {
    "algo_ae_calculation_done",
    "algo_ae_done",
    "fsm_config",
    "fsm_deinit",
    "fsm_reload_calibration",
    "fsm_start",
    "fsm_stop",
    "fsm_update_hw",
    "isphw_frame_end",
    "isphw_frame_end_fe",
    "isphw_stats_ready_ae",
    "isphw_stats_ready_af",
    "isphw_stats_ready_af_gamma",
    "isphw_stats_ready_awb",
    "metadata_ready",
    "metadata_update",
    "split_ae_data_ready",
    "unknown"};

int acamera_fsmgr_process_event( acamera_fsmgr_t *p_fsmgr )
{
    int global_event_processed_flag = 0, fsm_event_processed_flag = 0;

    const int event_id = acamera_event_queue_pop( &( p_fsmgr->event_queue ) );
    if ( event_id < 0 ) {
        return -1;
    }
    LOG( LOG_INFO, "Processing event: %d (%s)", event_id, event_name[event_id] );

// SENSOR FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SENSOR_PERF_SECTION );
#endif
    fsm_event_processed_flag = sensor_fsm_process_event( &( p_fsmgr->sensor_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SENSOR_PERF_SECTION, fsm_event_processed_flag );
#endif

// CONTROL FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CONTROL_PERF_SECTION );
#endif
    fsm_event_processed_flag = control_fsm_process_event( &( p_fsmgr->control_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CONTROL_PERF_SECTION, fsm_event_processed_flag );
#endif

// DECOMPANDER FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DECOMPANDER_PERF_SECTION );
#endif
    fsm_event_processed_flag = decompander_fsm_process_event( &( p_fsmgr->decompander_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DECOMPANDER_PERF_SECTION, fsm_event_processed_flag );
#endif

// FRAME_STITCH FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( FRAME_STITCH_PERF_SECTION );
#endif
    fsm_event_processed_flag = frame_stitch_fsm_process_event( &( p_fsmgr->frame_stitch_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( FRAME_STITCH_PERF_SECTION, fsm_event_processed_flag );
#endif

// OUTPUT_FORMATTER FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( OUTPUT_FORMATTER_PERF_SECTION );
#endif
    fsm_event_processed_flag = output_formatter_fsm_process_event( &( p_fsmgr->output_formatter_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( OUTPUT_FORMATTER_PERF_SECTION, fsm_event_processed_flag );
#endif

// CROP FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CROP_PERF_SECTION );
#endif
    fsm_event_processed_flag = crop_fsm_process_event( &( p_fsmgr->crop_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CROP_PERF_SECTION, fsm_event_processed_flag );
#endif

// CMOS FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CMOS_PERF_SECTION );
#endif
    fsm_event_processed_flag = cmos_fsm_process_event( &( p_fsmgr->cmos_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CMOS_PERF_SECTION, fsm_event_processed_flag );
#endif

// AE FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( AE_PERF_SECTION );
#endif
    fsm_event_processed_flag = AE_fsm_process_event( &( p_fsmgr->AE_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( AE_PERF_SECTION, fsm_event_processed_flag );
#endif

// HISTOGRAM FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( HISTOGRAM_PERF_SECTION );
#endif
    fsm_event_processed_flag = histogram_fsm_process_event( &( p_fsmgr->histogram_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( HISTOGRAM_PERF_SECTION, fsm_event_processed_flag );
#endif

// AWB FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( AWB_PERF_SECTION );
#endif
    fsm_event_processed_flag = AWB_fsm_process_event( &( p_fsmgr->AWB_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( AWB_PERF_SECTION, fsm_event_processed_flag );
#endif

// COLOR_MATRIX FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( COLOR_MATRIX_PERF_SECTION );
#endif
    fsm_event_processed_flag = color_matrix_fsm_process_event( &( p_fsmgr->color_matrix_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( COLOR_MATRIX_PERF_SECTION, fsm_event_processed_flag );
#endif

// MESH_SHADING FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( MESH_SHADING_PERF_SECTION );
#endif
    fsm_event_processed_flag = mesh_shading_fsm_process_event( &( p_fsmgr->mesh_shading_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( MESH_SHADING_PERF_SECTION, fsm_event_processed_flag );
#endif

// IRIDIX FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( IRIDIX_PERF_SECTION );
#endif
    fsm_event_processed_flag = iridix_fsm_process_event( &( p_fsmgr->iridix_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( IRIDIX_PERF_SECTION, fsm_event_processed_flag );
#endif

// MCFE FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( MCFE_PERF_SECTION );
#endif
    fsm_event_processed_flag = mcfe_fsm_process_event( &( p_fsmgr->mcfe_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( MCFE_PERF_SECTION, fsm_event_processed_flag );
#endif

#if 0
// FRAME_CHECK FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( FRAME_CHECK_PERF_SECTION );
#endif
    fsm_event_processed_flag = frame_check_fsm_process_event( &( p_fsmgr->frame_check_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( FRAME_CHECK_PERF_SECTION, fsm_event_processed_flag );
#endif

// METADATA FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( METADATA_PERF_SECTION );
#endif
    fsm_event_processed_flag = metadata_fsm_process_event( &( p_fsmgr->metadata_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( METADATA_PERF_SECTION, fsm_event_processed_flag );
#endif
#endif

// SHARPENING FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SHARPENING_PERF_SECTION );
#endif
    fsm_event_processed_flag = sharpening_fsm_process_event( &( p_fsmgr->sharpening_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SHARPENING_PERF_SECTION, fsm_event_processed_flag );
#endif

// NOISE_REDUCTION FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( NOISE_REDUCTION_PERF_SECTION );
#endif
    fsm_event_processed_flag = noise_reduction_fsm_process_event( &( p_fsmgr->noise_reduction_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( NOISE_REDUCTION_PERF_SECTION, fsm_event_processed_flag );
#endif

// DEFECT_PIXEL FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DEFECT_PIXEL_PERF_SECTION );
#endif
    fsm_event_processed_flag = defect_pixel_fsm_process_event( &( p_fsmgr->defect_pixel_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DEFECT_PIXEL_PERF_SECTION, fsm_event_processed_flag );
#endif

// SBUF FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( SBUF_PERF_SECTION );
#endif
    fsm_event_processed_flag = sbuf_fsm_process_event( &( p_fsmgr->sbuf_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( SBUF_PERF_SECTION, fsm_event_processed_flag );
#endif

// ISP_WRAPPER FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( ISP_WRAPPER_PERF_SECTION );
#endif
    fsm_event_processed_flag = isp_wrapper_fsm_process_event( &( p_fsmgr->isp_wrapper_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( ISP_WRAPPER_PERF_SECTION, fsm_event_processed_flag );
#endif

// CAC FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( CAC_PERF_SECTION );
#endif
    fsm_event_processed_flag = cac_fsm_process_event( &( p_fsmgr->cac_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( CAC_PERF_SECTION, fsm_event_processed_flag );
#endif

// RADIAL_SHADING FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( RADIAL_SHADING_PERF_SECTION );
#endif
    fsm_event_processed_flag = radial_shading_fsm_process_event( &( p_fsmgr->radial_shading_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( RADIAL_SHADING_PERF_SECTION, fsm_event_processed_flag );
#endif

// NOISE_PROFILE FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( NOISE_PROFILE_PERF_SECTION );
#endif
    fsm_event_processed_flag = noise_profile_fsm_process_event( &( p_fsmgr->noise_profile_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( NOISE_PROFILE_PERF_SECTION, fsm_event_processed_flag );
#endif

// DEMOSAIC FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( DEMOSAIC_PERF_SECTION );
#endif
    fsm_event_processed_flag = demosaic_fsm_process_event( &( p_fsmgr->demosaic_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( DEMOSAIC_PERF_SECTION, fsm_event_processed_flag );
#endif

// ML_BIST FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( ML_BIST_PERF_SECTION );
#endif
    fsm_event_processed_flag = ml_bist_fsm_process_event( &( p_fsmgr->ml_bist_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( ML_BIST_PERF_SECTION, fsm_event_processed_flag );
#endif

// RAW_FE FSM
#if ACAMERA_ISP_PROFILING
    acamera_profiler_start( RAW_FE_PERF_SECTION );
#endif
    fsm_event_processed_flag = raw_fe_fsm_process_event( &( p_fsmgr->raw_fe_fsm ), event_id );
    global_event_processed_flag |= fsm_event_processed_flag;
#if ACAMERA_ISP_PROFILING
    acamera_profiler_stop( RAW_FE_PERF_SECTION, fsm_event_processed_flag );
#endif

    if ( !global_event_processed_flag ) {
        return -1;
    }

    return event_id;
}
