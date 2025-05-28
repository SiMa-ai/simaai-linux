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

#if !defined( __ACAMERA_FSMGR_H__ )
#define __ACAMERA_FSMGR_H__

#include "system_types.h"
#include "acamera_configuration.h"
#include "acamera_interrupt_numbers.h"

#if !defined( __ACAMERA_ISP_CTX_H__ )
#error "acamera_fsmgr.h file should not be included from source file, include acamera_isp_ctx.h instead!"
#endif

typedef enum _event_id_t {
    event_id_algo_ae_calculation_done,
    event_id_algo_ae_done,
    event_id_fsm_config,
    event_id_fsm_deinit,
    event_id_fsm_reload_calibration,
    event_id_fsm_start,
    event_id_fsm_stop,
    event_id_fsm_update_hw,
    event_id_isphw_frame_end,
    event_id_isphw_frame_end_fe,
    event_id_isphw_stats_ready_ae,
    event_id_isphw_stats_ready_af,
    event_id_isphw_stats_ready_af_gamma,
    event_id_isphw_stats_ready_awb,
    event_id_metadata_ready,
    event_id_metadata_update,
    event_id_split_ae_data_ready,
    number_of_event_ids
} event_id_t;

struct _acamera_fsmgr_t;
typedef struct _acamera_fsmgr_t acamera_fsmgr_t;

#define ACAMERA_EVENT_QUEUE_SIZE 512

#include "ae_manual_fsm.h"
#include "awb_manual_fsm.h"
#include "cac_fsm.h"
#include "cmos_fsm.h"
#include "color_matrix_fsm.h"
#include "control_fsm.h"
#include "crop_fsm.h"
#include "decompander_fsm.h"
#include "defect_pixel_fsm.h"
#include "demosaic_fsm.h"
#include "frame_check_fsm.h"
#include "frame_stitch_fsm.h"
#include "histogram_fsm.h"
#include "iridix8_manual_fsm.h"
#include "isp_wrapper_dummy_fsm.h"
#include "mcfe_fsm.h"
#include "mesh_shading_fsm.h"
#include "metadata_fsm.h"
#include "ml_bist_fsm.h"
#include "noise_profile_fsm.h"
#include "noise_reduction_fsm.h"
#include "output_formatter_fsm.h"
#include "radial_shading_fsm.h"
#include "raw_fe_fsm.h"
#include "sbuf_fsm.h"
#include "sensor_fsm.h"
#include "sharpening_fsm.h"

#include "acamera_fsmgr_event_queue.h"

struct _acamera_fsmgr_t {
    acamera_isp_ctx_ptr_t p_ictx;
    sensor_fsm_t sensor_fsm;
    control_fsm_t control_fsm;
    decompander_fsm_t decompander_fsm;
    frame_stitch_fsm_t frame_stitch_fsm;
    output_formatter_fsm_t output_formatter_fsm;
    crop_fsm_t crop_fsm;
    cmos_fsm_t cmos_fsm;
    AE_fsm_t AE_fsm;
    histogram_fsm_t histogram_fsm;
    AWB_fsm_t AWB_fsm;
    color_matrix_fsm_t color_matrix_fsm;
    mesh_shading_fsm_t mesh_shading_fsm;
    iridix_fsm_t iridix_fsm;
    mcfe_fsm_t mcfe_fsm;
    frame_check_fsm_t frame_check_fsm;
    metadata_fsm_t metadata_fsm;
    sharpening_fsm_t sharpening_fsm;
    noise_reduction_fsm_t noise_reduction_fsm;
    defect_pixel_fsm_t defect_pixel_fsm;
    sbuf_fsm_t sbuf_fsm;
    isp_wrapper_fsm_t isp_wrapper_fsm;
    cac_fsm_t cac_fsm;
    radial_shading_fsm_t radial_shading_fsm;
    noise_profile_fsm_t noise_profile_fsm;
    demosaic_fsm_t demosaic_fsm;
    ml_bist_fsm_t ml_bist_fsm;
    raw_fe_fsm_t raw_fe_fsm;
    acamera_event_queue_t event_queue;
    uint16_t event_queue_data[ACAMERA_EVENT_QUEUE_SIZE];
};

void acamera_fsmgr_raise_event( acamera_fsmgr_t *p_fsmgr, event_id_t event_id );

#define fsm_raise_event( p_fsm, event_id ) \
    acamera_fsmgr_raise_event( ( p_fsm )->p_fsmgr, event_id )

void acamera_fsmgr_init( acamera_fsmgr_t *p_fsmgr );
void acamera_fsmgr_process_interrupt( acamera_fsmgr_t *p_fsmgr, uint8_t event );
int acamera_fsmgr_process_event( acamera_fsmgr_t *p_fsmgr );

#endif
