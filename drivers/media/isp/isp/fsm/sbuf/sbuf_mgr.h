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

#include <linux/spinlock.h>
#include "sbuf.h"

/**
 * sbuf_item_arr_info - Struct to describe sbuf array current status.
 *
 * @item_total_count:   Number of sbuf for each buffer type.
 * @item_status_count:  Number of sbuf of each buffer status, sum of this
 *                      array should equals to item_total_count after each operation.
 * @write_idx:          Array index of sbuf for next write, the status should be DATA_EMPTY.
 * @read_idx:           Array index of sbuf for next read, the status should be DATA_DONE.
 *
 */
struct sbuf_item_arr_info {
    uint32_t item_total_count;
    uint32_t item_status_count[SBUF_STATUS_MAX];
    uint32_t write_idx;
    uint32_t read_idx;
};

struct sbuf_mgr {
    spinlock_t sbuf_lock;
    int sbuf_inited;
    uint32_t ctx_id;

    uint32_t len_allocated;
    uint32_t len_used;
    void *buf_allocated;
    void *buf_used;

    struct fw_sbuf *sbuf_base;

#if defined( ISP_HAS_AE_MANUAL_FSM )
    /* AE: array to describe sbuf item status */
    struct sbuf_item ae_sbuf_arr[SBUF_STATS_ARRAY_SIZE];
    /* AE: structure to describe array */
    struct sbuf_item_arr_info ae_arr_info;
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
    /* AWB: array to describe sbuf item status */
    struct sbuf_item awb_sbuf_arr[SBUF_STATS_ARRAY_SIZE];
    /* AWB: structure to describe array */
    struct sbuf_item_arr_info awb_arr_info;
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
    /* Gamma: array to describe sbuf item status */
    struct sbuf_item gamma_sbuf_arr[SBUF_STATS_ARRAY_SIZE];
    struct sbuf_item_arr_info gamma_arr_info;
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    /* Gamma: array to describe sbuf item status */
    struct sbuf_item iridix_sbuf_arr[SBUF_STATS_ARRAY_SIZE];
    struct sbuf_item_arr_info iridix_arr_info;
#endif
};

int is_sbuf_inited( struct sbuf_mgr *p_sbuf_mgr );
void sbuf_mgr_init_sbuf( struct sbuf_mgr *p_sbuf_mgr );
int sbuf_mgr_init( struct sbuf_mgr *p_sbuf_mgr, uint32_t ctx_id );
int update_cur_calibration_to_sbuf( sbuf_fsm_ptr_t p_fsm, struct sbuf_mgr *p_sbuf_mgr );
uint32_t sbuf_mgr_item_count_in_using( struct sbuf_mgr *p_sbuf_mgr );
void sbuf_mgr_reset( struct sbuf_mgr *p_sbuf_mgr );
int sbuf_mgr_free( struct sbuf_mgr *p_sbuf_mgr );
int sbuf_bfmgr_set_item( struct sbuf_mgr *p_sbuf_mgr, struct sbuf_item *item );
int sbuf_bfmgr_get_item( struct sbuf_mgr *p_sbuf_mgr, struct sbuf_item *item );
