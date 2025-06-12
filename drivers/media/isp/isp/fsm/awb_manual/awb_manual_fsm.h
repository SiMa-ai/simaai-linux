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

#if !defined( __AWB_FSM_H__ )
#define __AWB_FSM_H__

typedef struct _AWB_fsm_t AWB_fsm_t;
typedef struct _AWB_fsm_t *AWB_fsm_ptr_t;
typedef const struct _AWB_fsm_t *AWB_fsm_const_ptr_t;

enum _AWB_state_t {
    AWB_state_initialized,
    AWB_state_configured,
    AWB_state_reload_calibration,
    AWB_state_ready,
    AWB_state_stopped,
    AWB_state_deinit,
    AWB_state_update_algo,
    AWB_state_update_hw,
    AWB_state_invalid
};

typedef enum _AWB_state_t AWB_state_t;

#include "acamera_fsmgr_general_router.h"
#include "acamera_isp_core_settings.h"

#define ISP_HAS_AWB_FSM 1
#define MAX_AWB_ZONES ( ISP_METERING_HISTOGRAM_ZONES_MAX )
#define AWB_LIGHT_SOURCE_UNKNOWN 0
#define AWB_LIGHT_SOURCE_A 0x01
#define AWB_LIGHT_SOURCE_D40 0x02
#define AWB_LIGHT_SOURCE_D50 0x03
#define AWB_LIGHT_SOURCE_A_TEMPERATURE 2850
#define AWB_LIGHT_SOURCE_D40_TEMPERATURE 4000
#define AWB_LIGHT_SOURCE_D50_TEMPERATURE 5000
#define AWB_DLS_LIGHT_SOURCE_A_D40_BORDER ( AWB_LIGHT_SOURCE_A_TEMPERATURE + AWB_LIGHT_SOURCE_D40_TEMPERATURE ) / 2
#define AWB_DLS_LIGHT_SOURCE_D40_D50_BORDER ( AWB_LIGHT_SOURCE_D40_TEMPERATURE + AWB_LIGHT_SOURCE_D50_TEMPERATURE ) / 2
#define AWB_DLS_SWITCH_LIGHT_SOURCE_DETECT_FRAMES_QUANTITY 15
#define AWB_DLS_SWITCH_LIGHT_SOURCE_CHANGE_FRAMES_QUANTITY 35
#define D50_DEFAULT 256

typedef struct _awb_zone_t {
    uint16_t rg;
    uint16_t bg;
    uint32_t sum;
} awb_zone_t;

void awb_init( AWB_fsm_ptr_t p_fsm );
void awb_config( AWB_fsm_ptr_t p_fsm );
void awb_reload_calibration( AWB_fsm_ptr_t p_fsm );
void awb_update_algo( AWB_fsm_ptr_t p_fsm );
void awb_update_hw( AWB_fsm_ptr_t p_fsm );
void awb_deinit( AWB_fsm_ptr_t p_fsm );

void awb_get_info( AWB_fsm_ptr_t p_psm, acamera_cmd_wb_info *p_info );

struct _AWB_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    AWB_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    uint16_t curr_AWB_ZONES;
    uint32_t sum;
    uint16_t rg_coef;
    uint16_t bg_coef;
    uint8_t p_high;
    int32_t wb_log2[4];
    int32_t temperature_detected;
    uint8_t light_source_detected;
    uint8_t light_source_candidate;
    uint8_t detect_light_source_frames_count;
    int32_t max_temp;
    int32_t min_temp;
    uint16_t max_temp_rg;
    uint16_t max_temp_bg;
    uint16_t min_temp_rg;
    uint16_t min_temp_bg;
    int32_t awb_warming[3];
    uint32_t switch_light_source_detect_frames_quantity;
    uint32_t switch_light_source_change_frames_quantity;
};


void AWB_fsm_clear( AWB_fsm_ptr_t p_fsm );
void AWB_fsm_switch_state( AWB_fsm_ptr_t p_fsm, AWB_state_t state );
void AWB_fsm_process_state( AWB_fsm_ptr_t p_fsm );
uint8_t AWB_fsm_process_event( AWB_fsm_ptr_t p_fsm, event_id_t event_id );

void AWB_fsm_process_interrupt( AWB_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void AWB_request_interrupt( AWB_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __AWB_FSM_H__ */
