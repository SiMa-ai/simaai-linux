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

#if !defined( __DECOMPANDER_FSM_H__ )
#define __DECOMPANDER_FSM_H__

typedef struct _decompander_fsm_t decompander_fsm_t;
typedef struct _decompander_fsm_t *decompander_fsm_ptr_t;
typedef const struct _decompander_fsm_t *decompander_fsm_const_ptr_t;

enum _decompander_state_t {
    decompander_state_initialized,
    decompander_state_configured,
    decompander_state_reload_calibration,
    decompander_state_ready,
    decompander_state_stopped,
    decompander_state_deinit,
    decompander_state_invalid
};

typedef enum _decompander_state_t decompander_state_t;


typedef struct _decompander_ctrl {
    uint8_t input_formatter_enable;
    uint8_t hist_neq_lut_enable;
    uint8_t hist_neq_lut_pos;
    uint8_t gamma_fe_lut_enable;
    uint8_t gamma_fe_sqrt_enable;
    uint8_t gamma_be_lut_enable;
    uint8_t gamma_be_sq_enable;
    uint8_t linear_data_source;
} decompander_ctrl;

typedef struct _input_formatter_ctrl {
    uint16_t in_fmt_mode;
    uint16_t input1_bitwidth;
    uint16_t input2_bitwidth;
    uint16_t input3_bitwidth;
    uint16_t input4_bitwidth;
    uint16_t input1_aligment;
    uint16_t input2_aligment;
    uint16_t input3_aligment;
    uint16_t input4_aligment;
    uint16_t black_level;
    uint16_t knee_point0;
    uint16_t knee_point1;
    uint16_t knee_point2;
    uint16_t slope0;
    uint16_t slope1;
    uint16_t slope2;
    uint16_t slope3;
} input_formatter_ctrl;

typedef struct _histogram_neq_lut {
    uint32_t x;
    uint32_t y;
} histogram_neq_lut;

void decompander_init( decompander_fsm_ptr_t p_fsm );
void decompander_deinit( decompander_fsm_ptr_t p_fsm );
void decompander_reload_calibration( decompander_fsm_ptr_t p_fsm );


struct _decompander_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    decompander_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
};


void decompander_fsm_clear( decompander_fsm_ptr_t p_fsm );
void decompander_fsm_switch_state( decompander_fsm_ptr_t p_fsm, decompander_state_t state );
void decompander_fsm_process_state( decompander_fsm_ptr_t p_fsm );
uint8_t decompander_fsm_process_event( decompander_fsm_ptr_t p_fsm, event_id_t event_id );

void decompander_fsm_process_interrupt( decompander_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void decompander_request_interrupt( decompander_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

#endif /* __DECOMPANDER_FSM_H__ */
