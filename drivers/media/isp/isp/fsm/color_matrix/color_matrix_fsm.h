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

#if !defined( __COLOR_MATRIX_FSM_H__ )
#define __COLOR_MATRIX_FSM_H__

typedef struct _color_matrix_fsm_t color_matrix_fsm_t;
typedef struct _color_matrix_fsm_t *color_matrix_fsm_ptr_t;
typedef const struct _color_matrix_fsm_t *color_matrix_fsm_const_ptr_t;

enum _color_matrix_state_t {
    color_matrix_state_initialized,
    color_matrix_state_configured,
    color_matrix_state_reload_calibration,
    color_matrix_state_ready,
    color_matrix_state_stopped,
    color_matrix_state_deinit,
    color_matrix_state_update_hw,
    color_matrix_state_invalid
};

typedef enum _color_matrix_state_t color_matrix_state_t;

#define ISP_CCM_SIZE 9
#define ISP_CCM_B_SIZE 3

#include "acamera_fsmgr_general_router.h"
void color_matrix_init( color_matrix_fsm_ptr_t p_fsm );
void color_matrix_config( color_matrix_fsm_ptr_t p_fsm );
void color_matrix_update_hw( color_matrix_fsm_ptr_t p_fsm );
void color_matrix_get_info( const color_matrix_fsm_ptr_t p_psm, acamera_cmd_ccm_info *p_info );
void color_matrix_set_info( color_matrix_fsm_ptr_t p_psm, const acamera_cmd_ccm_info *p_info );

struct _color_matrix_fsm_t {
    acamera_fsmgr_t *p_fsmgr;
    color_matrix_state_t state;
    system_fw_interrupt_mask_t irq_mask;
    system_fw_interrupt_mask_t repeat_irq_mask;
    uint8_t saturation_target;
    int16_t color_matrix[ISP_CCM_SIZE];
    int16_t color_matrix_yuv[ISP_CCM_SIZE];
    int16_t color_matrix_yuv_b[ISP_CCM_B_SIZE];
    int16_t color_matrix_luv[ISP_CCM_SIZE];
    int16_t color_matrix_luv_b[ISP_CCM_B_SIZE];
    int16_t color_matrix_hs[ISP_CCM_SIZE];
    int16_t color_matrix_hs_b[ISP_CCM_B_SIZE];
    int16_t color_matrix_s2[ISP_CCM_SIZE];
    int16_t color_matrix_s2_b[ISP_CCM_B_SIZE];
    int16_t color_correction_matrix[ISP_CCM_SIZE];
    int16_t color_saturation_matrix[ISP_CCM_SIZE];
    uint8_t light_source;
    uint8_t light_source_previous;
    uint8_t light_source_ccm;
    uint8_t light_source_ccm_previous;
    uint8_t light_source_change_frames;
    uint8_t light_source_change_frames_left;
    int16_t color_matrix_A[ISP_CCM_SIZE];
    int16_t color_matrix_D40[ISP_CCM_SIZE];
    int16_t color_matrix_D50[ISP_CCM_SIZE];
    int16_t color_matrix_one[ISP_CCM_SIZE];
    uint8_t manual_CCM;
    int16_t manual_color_matrix[ISP_CCM_SIZE];

    /*
     * CMD_ID_CCM_MATRIX channel: when set, the per-frame update path
     * uses user_matrix directly (the IPA-computed CCM at the detected
     * colour temperature), bypassing the legacy zone selector
     * + 35-frame blend. user_matrix_valid is set by sbuf_func.c when
     * a new sbuf_awb arrives with ccm_matrix_valid != 0; reset after
     * consume. Phase 5 makes this the only CCM path; the A/D40/D50
     * fields and the blend logic are removed then.
     */
    int16_t user_matrix[ISP_CCM_SIZE];
    uint8_t user_matrix_valid;
};

/*
 * Install a CCM pushed by the IPA. Called from sbuf_func.c after a new
 * sbuf_awb is consumed. Subsequent color_matrix_update_hw() uses this
 * matrix and skips the zone classifier + 35-frame blend.
 */
void color_matrix_set_user_matrix( color_matrix_fsm_ptr_t p_fsm,
                                   const int16_t matrix[ISP_CCM_SIZE] );


void color_matrix_fsm_clear( color_matrix_fsm_ptr_t p_fsm );
void color_matrix_fsm_switch_state( color_matrix_fsm_ptr_t p_fsm, color_matrix_state_t state );
void color_matrix_fsm_process_state( color_matrix_fsm_ptr_t p_fsm );
uint8_t color_matrix_fsm_process_event( color_matrix_fsm_ptr_t p_fsm, event_id_t event_id );

void color_matrix_fsm_process_interrupt( color_matrix_fsm_const_ptr_t p_fsm, uint8_t irq_event );

void color_matrix_request_interrupt( color_matrix_fsm_ptr_t p_fsm, system_fw_interrupt_mask_t mask );

/* ------------------------------------------------------------------------
 * LOG_CCM(printk_fn, tag, mat)
 *
 *   printk_fn  one of pr_emerg / pr_alert / pr_crit / pr_err / pr_warn /
 *              pr_notice / pr_info / pr_debug — the kernel-side equivalent
 *              of the userspace `level` argument. Picking the function name
 *              is one-token-edit at the call site.
 *
 *              pr_notice and above are always visible in dmesg subject to
 *              the console loglevel. pr_debug is gated by either
 *              CONFIG_DYNAMIC_DEBUG (flip on at runtime via
 *                /sys/kernel/debug/dynamic_debug/control)
 *              or a per-file -DDEBUG build flag.
 *
 *   tag        short string identifying the call site, e.g. "<- sbuf" or "applied".
 *   mat        pointer/array of int16_t × ISP_CCM_SIZE (9 coefficients).
 *
 * Kernel-side counterpart of LOG_CCM in awb_illuminant.h. Emits the raw 9
 * integers as a single dmesg line. Kernel printk has no %f formatter, so
 * the float decode lives on the userspace side only — the integers here
 * pair byte-for-byte with the userspace "CCM -> sbuf" log for
 * cross-referencing.
 *
 * Sidesteps the ISP-side LOG() macro because SYSTEM_LOG_DEFAULT_VALUE is
 * pinned at LOG_LEVEL_ERROR in acamera_configuration.h on the kernel
 * build, so anything below ERROR through LOG() would be silently dropped.
 * Going through pr_*() reaches dmesg directly.
 * ------------------------------------------------------------------------ */
#define LOG_CCM(printk_fn, tag, mat)                                                     \
    printk_fn("ISP CCM %s: %d,%d,%d | %d,%d,%d | %d,%d,%d\n",                            \
              (tag),                                                                     \
              (int)(mat)[0], (int)(mat)[1], (int)(mat)[2],                               \
              (int)(mat)[3], (int)(mat)[4], (int)(mat)[5],                               \
              (int)(mat)[6], (int)(mat)[7], (int)(mat)[8])

#endif /* __COLOR_MATRIX_FSM_H__ */
