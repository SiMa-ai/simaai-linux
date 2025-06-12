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

#if !defined( __FRAME_CHECK_FSM_H__ )
#define __FRAME_CHECK_FSM_H__

typedef struct _frame_check_fsm_t frame_check_fsm_t;
typedef struct _frame_check_fsm_t *frame_check_fsm_ptr_t;
typedef const struct _frame_check_fsm_t *frame_check_fsm_const_ptr_t;

enum _frame_check_state_t {
    frame_check_state_initialized,
    frame_check_state_configured,
    frame_check_state_reload_calibration,
    frame_check_state_ready,
    frame_check_state_stopped,
    frame_check_state_deinit,
    frame_check_state_invalid
};

typedef enum _frame_check_state_t frame_check_state_t;

typedef struct _frame_check_scaler_crop_config_t {
    uint8_t enabled;
    uint32_t width;
    uint32_t height;
} frame_check_scaler_crop_config_t;

typedef struct _frame_check_output_plane_config_t {
    uint32_t width;
    uint32_t height;
    uint8_t h_subsampling;
    uint8_t v_subsampling;
    uint8_t axi;
} frame_check_output_plane_config_t;

typedef struct _frame_check_output_config_t {
    uint32_t mcfe_hblank_max;
    uint8_t num_planes;
    frame_check_scaler_crop_config_t crop;
    frame_check_output_plane_config_t plane[ISP_MCFE_HWIF_MAX_OUTPUTS];
} frame_check_output_config_t;

typedef struct _frame_check_sensor_config_t {
    uint32_t width;
    uint32_t height;
    uint32_t hblank_max;
} frame_check_sensor_config_t;

typedef struct _frame_check_mcfe_config_t {
    uint32_t hblank_max;
} frame_check_mcfe_config_t;


void frame_check_init( frame_check_fsm_ptr_t p_fsm );
void frame_check_config( frame_check_fsm_ptr_t p_fsm );


struct _frame_check_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    frame_check_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    frame_check_sensor_config_t sensor_config;
    frame_check_output_config_t output_config;
    frame_check_mcfe_config_t mcfe_config;
};


void frame_check_fsm_clear( frame_check_fsm_ptr_t p_fsm );
void frame_check_fsm_switch_state( frame_check_fsm_ptr_t p_fsm, frame_check_state_t state );
void frame_check_fsm_process_state( frame_check_fsm_ptr_t p_fsm );
uint8_t frame_check_fsm_process_event( frame_check_fsm_ptr_t p_fsm, event_id_t event_id );

void frame_check_fsm_process_interrupt( frame_check_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void frame_check_request_interrupt( frame_check_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __FRAME_CHECK_FSM_H__ */
