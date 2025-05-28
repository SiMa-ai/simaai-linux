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

#include "sbuf_mgr.h"
#include <linux/mm.h>
#include <linux/slab.h>
#include "system_stdlib.h" // system_memcpy
#include "acamera_logger.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_SBUF

static const char *sbuf_status_str[] = {
    "DATA_EMPTY",
    "DATA_PREPARE",
    "DATA_DONE",
    "DATA_USING",
    "ERROR"};

static const char *sbuf_type_str[] = {
    "AE",
    "AWB",
    "AF",
    "GAMMA",
    "IRIDIX",
    "ERROR"};

int is_sbuf_inited( struct sbuf_mgr *p_sbuf_mgr )
{
    int tmp_inited;
    unsigned long irq_flags;

    spin_lock_irqsave( &p_sbuf_mgr->sbuf_lock, irq_flags );
    tmp_inited = p_sbuf_mgr->sbuf_inited;
    spin_unlock_irqrestore( &p_sbuf_mgr->sbuf_lock, irq_flags );

    return tmp_inited;
}

static int sbuf_mgr_alloc_sbuf( struct sbuf_mgr *p_sbuf_mgr )
{
    int i;

    if ( is_sbuf_inited( p_sbuf_mgr ) ) {
        LOG( LOG_ERR, "Error: sbuf alloc should not be called twice." );
        return -1;
    }

    /* Round up to whole number of pages. */
    p_sbuf_mgr->len_used = ( sizeof( struct fw_sbuf ) + 1 + PAGE_SIZE ) & PAGE_MASK;

    /* Allocate one more page for user-space mapping. */
    p_sbuf_mgr->len_allocated = p_sbuf_mgr->len_used - 1 + PAGE_SIZE;

    p_sbuf_mgr->buf_allocated = kzalloc( p_sbuf_mgr->len_allocated, GFP_KERNEL );
    if ( !p_sbuf_mgr->buf_allocated ) {
        LOG( LOG_CRIT, "Alloc memory failed." );
        return -ENOMEM;
    }

    /* Make the used buffer page aligned. */
    p_sbuf_mgr->buf_used = (void *)( ( (unsigned long)p_sbuf_mgr->buf_allocated + PAGE_SIZE - 1 ) & PAGE_MASK );

    LOG( LOG_INFO, "sbuf: len_needed: %zu, len_alloc: %u, len_used: %u, page_size: %lu, buf_alloc: %p, buf_used: %p.",
         sizeof( struct fw_sbuf ), p_sbuf_mgr->len_allocated, p_sbuf_mgr->len_used,
         PAGE_SIZE, p_sbuf_mgr->buf_allocated, p_sbuf_mgr->buf_used );

    /* Set the page as reserved so that it won't be swapped out. */
    for ( i = 0; i < p_sbuf_mgr->len_used; i += PAGE_SIZE ) {
        SetPageReserved( virt_to_page( p_sbuf_mgr->buf_used + i ) );
    }

    return 0;
}

void sbuf_mgr_init_sbuf( struct sbuf_mgr *p_sbuf_mgr )
{
    int i;
    unsigned long irq_flags;

    spin_lock_irqsave( &p_sbuf_mgr->sbuf_lock, irq_flags );
    p_sbuf_mgr->sbuf_inited = 1;
    p_sbuf_mgr->sbuf_base = (struct fw_sbuf *)p_sbuf_mgr->buf_used;

#if defined( ISP_HAS_AE_MANUAL_FSM )
    /***  For AE  ***/
    for ( i = 0; i < SBUF_STATS_ARRAY_SIZE; i++ ) {
        p_sbuf_mgr->ae_sbuf_arr[i].buf_idx = i;
        p_sbuf_mgr->ae_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
        p_sbuf_mgr->ae_sbuf_arr[i].buf_type = SBUF_TYPE_AE;
        p_sbuf_mgr->ae_sbuf_arr[i].buf_base = (void *)&( p_sbuf_mgr->sbuf_base->ae_sbuf[i] );
    }

    system_memset( &p_sbuf_mgr->ae_arr_info, 0, sizeof( p_sbuf_mgr->ae_arr_info ) );
    p_sbuf_mgr->ae_arr_info.item_total_count = SBUF_STATS_ARRAY_SIZE;
    p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY] = SBUF_STATS_ARRAY_SIZE;

    /* Init read_idx is ARRAY_SIZE, which is invalid. */
    p_sbuf_mgr->ae_arr_info.write_idx = 0;
    p_sbuf_mgr->ae_arr_info.read_idx = SBUF_STATS_ARRAY_SIZE;
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
    /***  For AWB  ***/
    for ( i = 0; i < SBUF_STATS_ARRAY_SIZE; i++ ) {
        p_sbuf_mgr->awb_sbuf_arr[i].buf_idx = i;
        p_sbuf_mgr->awb_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
        p_sbuf_mgr->awb_sbuf_arr[i].buf_type = SBUF_TYPE_AWB;
        p_sbuf_mgr->awb_sbuf_arr[i].buf_base = (void *)&( p_sbuf_mgr->sbuf_base->awb_sbuf[i] );
    }

    system_memset( &p_sbuf_mgr->awb_arr_info, 0, sizeof( p_sbuf_mgr->awb_arr_info ) );
    p_sbuf_mgr->awb_arr_info.item_total_count = SBUF_STATS_ARRAY_SIZE;
    p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY] = SBUF_STATS_ARRAY_SIZE;

    p_sbuf_mgr->awb_arr_info.write_idx = 0;
    p_sbuf_mgr->awb_arr_info.read_idx = SBUF_STATS_ARRAY_SIZE;
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
    /***  For Gamma Stats ***/
    for ( i = 0; i < SBUF_STATS_ARRAY_SIZE; i++ ) {
        p_sbuf_mgr->gamma_sbuf_arr[i].buf_idx = i;
        p_sbuf_mgr->gamma_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
        p_sbuf_mgr->gamma_sbuf_arr[i].buf_type = SBUF_TYPE_GAMMA;
        p_sbuf_mgr->gamma_sbuf_arr[i].buf_base = (void *)&( p_sbuf_mgr->sbuf_base->gamma_sbuf[i] );
    }

    system_memset( &p_sbuf_mgr->gamma_arr_info, 0, sizeof( p_sbuf_mgr->gamma_arr_info ) );
    p_sbuf_mgr->gamma_arr_info.item_total_count = SBUF_STATS_ARRAY_SIZE;
    p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY] = SBUF_STATS_ARRAY_SIZE;

    p_sbuf_mgr->gamma_arr_info.write_idx = 0;
    p_sbuf_mgr->gamma_arr_info.read_idx = SBUF_STATS_ARRAY_SIZE;
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    /***  For Iridix  ***/
    for ( i = 0; i < SBUF_STATS_ARRAY_SIZE; i++ ) {
        p_sbuf_mgr->iridix_sbuf_arr[i].buf_idx = i;
        p_sbuf_mgr->iridix_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
        p_sbuf_mgr->iridix_sbuf_arr[i].buf_type = SBUF_TYPE_IRIDIX;
        p_sbuf_mgr->iridix_sbuf_arr[i].buf_base = (void *)&( p_sbuf_mgr->sbuf_base->iridix_sbuf[i] );
    }

    system_memset( &p_sbuf_mgr->iridix_arr_info, 0, sizeof( p_sbuf_mgr->iridix_arr_info ) );
    p_sbuf_mgr->iridix_arr_info.item_total_count = SBUF_STATS_ARRAY_SIZE;
    p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY] = SBUF_STATS_ARRAY_SIZE;

    p_sbuf_mgr->iridix_arr_info.write_idx = 0;
    p_sbuf_mgr->iridix_arr_info.read_idx = SBUF_STATS_ARRAY_SIZE;
#endif

    spin_unlock_irqrestore( &p_sbuf_mgr->sbuf_lock, irq_flags );
}

int sbuf_mgr_init( struct sbuf_mgr *p_sbuf_mgr, uint32_t ctx_id )
{
    int rc;

    p_sbuf_mgr->sbuf_inited = 0;
    spin_lock_init( &( p_sbuf_mgr->sbuf_lock ) );

    rc = sbuf_mgr_alloc_sbuf( p_sbuf_mgr );
    if ( rc ) {
        LOG( LOG_ERR, "sbuf_mgr alloc buffer failed, ret: %d.", rc );
        return rc;
    }

    sbuf_mgr_init_sbuf( p_sbuf_mgr );
    p_sbuf_mgr->ctx_id = ctx_id;
    return 0;
}

// #define _GET_LUT_SIZE( lut ) ( lut->rows * lut->cols * lut->width )

/*
    sbuf calibration memory layout: N is CALIBRATION_TOTAL_SIZE

    ----------
    |  lut1  |---
    |--------|  |
    |  lut2  |  |
    |--------|  |
    |  ....  |  |
    |--------|  |
    |  lutN  |--|----
    |--------|  |   |
    |  data1 |<--   |
    |--------|      |
    |  data2 |      |
    |--------|      |
    |  ....  |      |
    |--------|      |
    |  dataN |<------
    |--------|
*/
int update_cur_calibration_to_sbuf( sbuf_fsm_ptr_t p_fsm, struct sbuf_mgr *p_sbuf_mgr )
{
    int rc = 0;
    uint32_t idx = 0;
    uint32_t lut_size = 0;

    uint8_t *sbuf_cali_base = (uint8_t *)&p_sbuf_mgr->sbuf_base->kf_info.cali_info.cali_data;
    struct sbuf_lookup_table *p_sbuf_lut_arr = (struct sbuf_lookup_table *)sbuf_cali_base;
    uint8_t *p_sbuf_cali_data = sbuf_cali_base + sizeof( struct sbuf_lookup_table ) * CALIBRATION_TOTAL_SIZE;

    p_sbuf_mgr->sbuf_base->kf_info.cali_info.is_fetched = 0;

    LOG( LOG_DEBUG, "sbuf_cali_base: %p, p_sbuf_lut_arr: %p, p_sbuf_cali_data: %p", sbuf_cali_base, p_sbuf_lut_arr, p_sbuf_cali_data );

    for ( idx = 0; idx < CALIBRATION_TOTAL_SIZE; idx++ ) {
        if ( !calib_mgr_lut_exists( ACAMERA_FSM2CM_PTR( p_fsm ), idx ) ) { //checks .ptr!!

            // NOTE: Don't touch ptr values, UF will manage it.
            //p_sbuf_lut_arr[idx].ptr = 0;
            p_sbuf_lut_arr[idx].rows = 0;
            p_sbuf_lut_arr[idx].cols = 0;
            p_sbuf_lut_arr[idx].width = 0;
            continue;
        }

        lut_size = calib_mgr_lut_size( ACAMERA_FSM2CM_PTR( p_fsm ), idx ); // _GET_LUT_SIZE( p_lut );

        // NOTE: Don't touch ptr values, UF will manage it.
        //p_sbuf_lut_arr[idx].ptr = NULL;
        p_sbuf_lut_arr[idx].rows = calib_mgr_lut_rows( ACAMERA_FSM2CM_PTR( p_fsm ), idx );
        p_sbuf_lut_arr[idx].cols = calib_mgr_lut_cols( ACAMERA_FSM2CM_PTR( p_fsm ), idx );
        p_sbuf_lut_arr[idx].width = calib_mgr_lut_width( ACAMERA_FSM2CM_PTR( p_fsm ), idx );

        calib_mgr_lut_read( ACAMERA_FSM2CM_PTR( p_fsm ), p_sbuf_cali_data, lut_size, idx );

        if ( idx <= 3 || idx >= ( CALIBRATION_TOTAL_SIZE - 3 ) ) {
            uint32_t el00 = 0;
            if ( p_sbuf_lut_arr[idx].cols > 0 && p_sbuf_lut_arr[idx].rows > 0 )
                system_memcpy( &el00, p_sbuf_cali_data, p_sbuf_lut_arr[idx].width );
            LOG( LOG_DEBUG, "cal[%d]: Raws x Cols x W: %dx%dx%d; elem00 %d[%#x]",
                 idx,
                 p_sbuf_lut_arr[idx].rows,
                 p_sbuf_lut_arr[idx].cols,
                 p_sbuf_lut_arr[idx].width,
                 el00, el00 );
        }

        p_sbuf_cali_data += lut_size;
    }

    return rc;
}

uint32_t sbuf_mgr_item_count_in_using( struct sbuf_mgr *p_sbuf_mgr )
{
    uint32_t rc = 0;
    unsigned long irq_flags;

    spin_lock_irqsave( &p_sbuf_mgr->sbuf_lock, irq_flags );

#if defined( ISP_HAS_AE_MANUAL_FSM )
    rc += p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_USING];
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
    rc += p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_USING];
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
    rc += p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_USING];
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    rc += p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_USING];
#endif

    spin_unlock_irqrestore( &p_sbuf_mgr->sbuf_lock, irq_flags );

    LOG( LOG_DEBUG, "sbuf item using total count: %u.", rc );

    return rc;
}

void sbuf_mgr_reset( struct sbuf_mgr *p_sbuf_mgr )
{
    int i;
    unsigned long irq_flags;

    spin_lock_init( &( p_sbuf_mgr->sbuf_lock ) );

    spin_lock_irqsave( &p_sbuf_mgr->sbuf_lock, irq_flags );

    for ( i = 0; i < SBUF_STATS_ARRAY_SIZE; i++ ) {
#if defined( ISP_HAS_AE_MANUAL_FSM )
        /***  For AE  ***/
        if ( SBUF_STATUS_DATA_USING == p_sbuf_mgr->ae_sbuf_arr[i].buf_status ) {
            p_sbuf_mgr->ae_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
            p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY]++;
            p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_USING]--;
        }

        if ( i == SBUF_STATS_ARRAY_SIZE - 1 ) {
            LOG( LOG_DEBUG, "AE sbuf arr info: read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
                 p_sbuf_mgr->ae_arr_info.read_idx,
                 p_sbuf_mgr->ae_arr_info.write_idx,
                 p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY],
                 p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_PREPARE],
                 p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_DONE],
                 p_sbuf_mgr->ae_arr_info.item_status_count[SBUF_STATUS_DATA_USING] );
        }
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
        /***  For AWB  ***/
        if ( SBUF_STATUS_DATA_USING == p_sbuf_mgr->awb_sbuf_arr[i].buf_status ) {
            p_sbuf_mgr->awb_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
            p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY]++;
            p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_USING]--;
        }

        if ( i == SBUF_STATS_ARRAY_SIZE - 1 ) {
            LOG( LOG_DEBUG, "AWB sbuf arr info: read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
                 p_sbuf_mgr->awb_arr_info.read_idx,
                 p_sbuf_mgr->awb_arr_info.write_idx,
                 p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY],
                 p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_PREPARE],
                 p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_DONE],
                 p_sbuf_mgr->awb_arr_info.item_status_count[SBUF_STATUS_DATA_USING] );
        }
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
        /***  For Gamma Stats ***/
        if ( SBUF_STATUS_DATA_USING == p_sbuf_mgr->gamma_sbuf_arr[i].buf_status ) {
            p_sbuf_mgr->gamma_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
            p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY]++;
            p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_USING]--;
        }

        if ( i == SBUF_STATS_ARRAY_SIZE - 1 ) {
            LOG( LOG_DEBUG, "Gamma sbuf arr info: read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
                 p_sbuf_mgr->gamma_arr_info.read_idx,
                 p_sbuf_mgr->gamma_arr_info.write_idx,
                 p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY],
                 p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_PREPARE],
                 p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_DONE],
                 p_sbuf_mgr->gamma_arr_info.item_status_count[SBUF_STATUS_DATA_USING] );
        }
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
        /***  For Iridix  ***/
        if ( SBUF_STATUS_DATA_USING == p_sbuf_mgr->iridix_sbuf_arr[i].buf_status ) {
            p_sbuf_mgr->iridix_sbuf_arr[i].buf_status = SBUF_STATUS_DATA_EMPTY;
            p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY]++;
            p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_USING]--;
        }

        if ( i == SBUF_STATS_ARRAY_SIZE - 1 ) {
            LOG( LOG_DEBUG, "Iridix sbuf arr info: read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
                 p_sbuf_mgr->iridix_arr_info.read_idx,
                 p_sbuf_mgr->iridix_arr_info.write_idx,
                 p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_EMPTY],
                 p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_PREPARE],
                 p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_DONE],
                 p_sbuf_mgr->iridix_arr_info.item_status_count[SBUF_STATUS_DATA_USING] );
        }
#endif
    }

    spin_unlock_irqrestore( &p_sbuf_mgr->sbuf_lock, irq_flags );
}

int sbuf_mgr_free( struct sbuf_mgr *p_sbuf_mgr )
{
    unsigned long irq_flags;

    if ( !is_sbuf_inited( p_sbuf_mgr ) ) {
        LOG( LOG_ERR, "Error: sbuf alloc is not initialized, can't free." );
        return -ENOMEM;
    }

    /* Set the flag before we free in case sb use it after free but before flag is clear. */
    spin_lock_irqsave( &p_sbuf_mgr->sbuf_lock, irq_flags );
    p_sbuf_mgr->sbuf_inited = 0;
    spin_unlock_irqrestore( &p_sbuf_mgr->sbuf_lock, irq_flags );

    LOG( LOG_INFO, "Prepare to free buffer %p.", p_sbuf_mgr->buf_allocated );
    if ( p_sbuf_mgr->buf_allocated ) {
        int i;
        /* Clear the reserved flag before free, so that no bug shows when freed. */
        for ( i = 0; i < p_sbuf_mgr->len_used; i += PAGE_SIZE ) {
            ClearPageReserved( virt_to_page( p_sbuf_mgr->buf_used + i ) );
        }

        kfree( p_sbuf_mgr->buf_allocated );
        LOG( LOG_INFO, "sbuf alloc buffer is freed." );
        p_sbuf_mgr->buf_allocated = NULL;
        p_sbuf_mgr->buf_used = NULL;
    } else {
        LOG( LOG_ERR, "Error: sbuf allocated memory is NULL." );
    }

    return 0;
}

static int sbuf_get_item_from_arr( struct sbuf_mgr *p_sbuf_mgr, struct sbuf_item *item, struct sbuf_item *arr, struct sbuf_item_arr_info *info )
{
    uint32_t count;
    unsigned long irq_flags;
    int rc = -EINVAL;

    /* Status cycle: Empty -> Prepare -> Done -> Using -> Empty. */
    if ( ( SBUF_STATUS_DATA_EMPTY != item->buf_status ) &&
         ( SBUF_STATUS_DATA_DONE != item->buf_status ) ) {
        LOG( LOG_ERR, "Invalid sbuf status: %d.", item->buf_status );
        return -EINVAL;
    }

    spin_lock_irqsave( &p_sbuf_mgr->sbuf_lock, irq_flags );

    LOG( LOG_DEBUG, "+++ sbuf arr info: ctx: %d, buf_type: %s, buf_status: %s, read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
         p_sbuf_mgr->ctx_id,
         sbuf_type_str[item->buf_type],
         sbuf_status_str[item->buf_status],
         info->read_idx,
         info->write_idx,
         info->item_status_count[SBUF_STATUS_DATA_EMPTY],
         info->item_status_count[SBUF_STATUS_DATA_PREPARE],
         info->item_status_count[SBUF_STATUS_DATA_DONE],
         info->item_status_count[SBUF_STATUS_DATA_USING] );

    /* sb wants to get empty buffer? */
    if ( SBUF_STATUS_DATA_EMPTY == item->buf_status ) {
        /*
         * Only support one buffer to prepare data, if there already
         * has a buffer being prepared, we should not give buffer anymore.
         */
        LOG( LOG_DEBUG, "DATA_EMPTY sbuf is getting." );
        if ( info->item_status_count[SBUF_STATUS_DATA_PREPARE] == 0 ) {
            /* Get the buffer information. */
            item->buf_idx = info->write_idx;
            item->buf_base = arr[item->buf_idx].buf_base;

            /* Update array information. */
            info->item_status_count[arr[item->buf_idx].buf_status]--;
            arr[item->buf_idx].buf_status = SBUF_STATUS_DATA_PREPARE;
            info->item_status_count[SBUF_STATUS_DATA_PREPARE]++;

            /* Prepare for next write index. */
            count = info->item_total_count;
            while ( --count ) {
                info->write_idx++;

                if ( info->write_idx >= info->item_total_count )
                    info->write_idx = 0;

                /* Find the buffer for next write, we can't use the sbuf which is in using by sb. */
                if ( arr[info->write_idx].buf_status != SBUF_STATUS_DATA_USING ) {
                    /* If we're going to overwrite the next read buffer, we should update the read_idx. */
                    if ( info->read_idx == info->write_idx ) {
                        do {
                            info->read_idx++;
                            if ( info->read_idx >= info->item_total_count )
                                info->read_idx = 0;

                            /* Check for a whole loop to avoid infinite loop in case of some error conditions. */
                            if ( info->read_idx == info->write_idx ) {
                                LOG( LOG_DEBUG, "No DONE buffer after a loop, reset read_idx." );
                                info->read_idx = info->item_total_count;
                                break;
                            }
                        } while ( arr[info->read_idx].buf_status != SBUF_STATUS_DATA_DONE );
                    }

                    /* Update status count. */
                    info->item_status_count[arr[info->write_idx].buf_status]--;
                    arr[info->write_idx].buf_status = SBUF_STATUS_DATA_EMPTY;
                    info->item_status_count[SBUF_STATUS_DATA_EMPTY]++;
                    break;
                }
            }

            rc = 0;
        } else {
            LOG( LOG_ERR, "Failed to get empty sbuf, ctx: %d, prepare count: %u.",
                 p_sbuf_mgr->ctx_id, info->item_status_count[SBUF_STATUS_DATA_PREPARE] );
        }
    } else {
        /* sb wants to get data_done buffer */
        /*
         * Only support one buffer to using the data, if there already
         * has a buffer being used, we should fail this request.
         */
        if ( ( info->item_status_count[SBUF_STATUS_DATA_USING] == 0 ) &&
             ( info->item_status_count[SBUF_STATUS_DATA_DONE] > 0 ) &&
             ( SBUF_STATUS_DATA_DONE == arr[info->read_idx].buf_status ) ) {

            /* Get the buffer information. */
            item->buf_idx = info->read_idx;
            item->buf_base = arr[item->buf_idx].buf_base;

            /* Update array information. */
            info->item_status_count[arr[item->buf_idx].buf_status]--;
            arr[item->buf_idx].buf_status = SBUF_STATUS_DATA_USING;
            info->item_status_count[SBUF_STATUS_DATA_USING]++;

            /* Prepare for next read index. */
            count = info->item_total_count;
            while ( --count ) {
                info->read_idx++;

                if ( info->read_idx >= info->item_total_count )
                    info->read_idx = 0;

                /* If we find next DTA_DONE buffer, break. */
                if ( arr[info->read_idx].buf_status == SBUF_STATUS_DATA_DONE )
                    break;
            }

            if ( arr[info->read_idx].buf_status != SBUF_STATUS_DATA_DONE ) {
                info->read_idx = info->item_total_count;
                LOG( LOG_DEBUG, "NOTE: no DONE buffer after a loop, reset read_idx." );
            }

            rc = 0;
        } else {
            LOG( LOG_ERR, "Failed to get DONE buf [ctx#%02d:/%s], USING: %u, DONE: %u, r_idx: %u, status: %u",
                 p_sbuf_mgr->ctx_id,
                 sbuf_type_str[item->buf_type],
                 info->item_status_count[SBUF_STATUS_DATA_USING],
                 info->item_status_count[SBUF_STATUS_DATA_DONE],
                 info->read_idx,
                 arr[info->read_idx].buf_status );
        }
    }

    LOG( LOG_DEBUG, "--- sbuf arr info: ctx: %d, buf_type: %s, buf_status: %s, read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
         p_sbuf_mgr->ctx_id,
         sbuf_type_str[item->buf_type],
         sbuf_status_str[item->buf_status],
         info->read_idx,
         info->write_idx,
         info->item_status_count[SBUF_STATUS_DATA_EMPTY],
         info->item_status_count[SBUF_STATUS_DATA_PREPARE],
         info->item_status_count[SBUF_STATUS_DATA_DONE],
         info->item_status_count[SBUF_STATUS_DATA_USING] );

    spin_unlock_irqrestore( &p_sbuf_mgr->sbuf_lock, irq_flags );

    if ( rc ) {
        LOG( LOG_DEBUG, "Get sbuf item failed: buf_type: %s, buf_status: %s.",
             sbuf_type_str[item->buf_type], sbuf_status_str[item->buf_status] );
    } else {
        LOG( LOG_DEBUG, "Get sbuf item OK: buf_type: %s, buf_idx: %u, buf_status: %s, buf_base: %p.",
             sbuf_type_str[item->buf_type], item->buf_idx, sbuf_status_str[item->buf_status], item->buf_base );
    }

    return rc;
}

static int sbuf_set_item_to_arr( struct sbuf_mgr *p_sbuf_mgr, struct sbuf_item *item, struct sbuf_item *arr, struct sbuf_item_arr_info *info )
{
    unsigned long irq_flags;
    int rc = -EINVAL;

    /* Status cycle: Empty -> Prepare -> Done -> Using -> Empty. */
    if ( ( SBUF_STATUS_DATA_EMPTY != item->buf_status ) &&
         ( SBUF_STATUS_DATA_DONE != item->buf_status ) ) {
        LOG( LOG_ERR, "Invalid state: %d.", item->buf_status );
        return -EINVAL;
    }

    spin_lock_irqsave( &p_sbuf_mgr->sbuf_lock, irq_flags );

    LOG( LOG_DEBUG, "+++ sbuf arr info: buf_type: %s, buf_status: %s, read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
         sbuf_type_str[item->buf_type],
         sbuf_status_str[item->buf_status],
         info->read_idx,
         info->write_idx,
         info->item_status_count[SBUF_STATUS_DATA_EMPTY],
         info->item_status_count[SBUF_STATUS_DATA_PREPARE],
         info->item_status_count[SBUF_STATUS_DATA_DONE],
         info->item_status_count[SBUF_STATUS_DATA_USING] );

    /* sb wants to set empty buffer after using? */
    if ( SBUF_STATUS_DATA_EMPTY == item->buf_status ) {
        /* The previous status of this buffer must be USING. */
        if ( ( info->item_status_count[SBUF_STATUS_DATA_USING] == 1 ) &&
             ( arr[item->buf_idx].buf_status == SBUF_STATUS_DATA_USING ) ) {

            /* Update array information. */
            arr[item->buf_idx].buf_status = SBUF_STATUS_DATA_EMPTY;
            info->item_status_count[SBUF_STATUS_DATA_EMPTY]++;
            info->item_status_count[SBUF_STATUS_DATA_USING]--;

            rc = 0;
        } else {
            LOG( LOG_ERR, "Failed to set empty sbuf, using count: %u, item buf_status: %u, buf_idx: %u.",
                 info->item_status_count[SBUF_STATUS_DATA_USING],
                 arr[item->buf_idx].buf_status,
                 item->buf_idx );
        }
    } else {
        /* sb wants to set data_done buffer after prepare */
        /*
         * Only support one buffer to using the data, if there already
         * has a buffer being used, we should fail this request.
         */
        /* The previous status of this buffer must be PREPARE. */
        if ( ( info->item_status_count[SBUF_STATUS_DATA_PREPARE] == 1 ) &&
             ( arr[item->buf_idx].buf_status == SBUF_STATUS_DATA_PREPARE ) ) {

            /* Update array information. */
            arr[item->buf_idx].buf_status = SBUF_STATUS_DATA_DONE;
            info->item_status_count[SBUF_STATUS_DATA_DONE]++;
            info->item_status_count[SBUF_STATUS_DATA_PREPARE]--;

            /* Set read_idx if it hasn't set properly. */
            if ( info->read_idx == info->item_total_count ) {
                info->read_idx = item->buf_idx;
            }

            rc = 0;
        } else {
            LOG( LOG_ERR, "Failed to set done sbuf, prepare count: %u, item buf_status: %u, buf_idx: %u.",
                 info->item_status_count[SBUF_STATUS_DATA_PREPARE],
                 arr[item->buf_idx].buf_status,
                 item->buf_idx );
        }
    }

    LOG( LOG_DEBUG, "--- sbuf arr info: buf_type: %s, buf_status: %u, read_idx: %u, write_idx: %u, status_count: %u-%u-%u-%u.",
         sbuf_type_str[item->buf_type],
         item->buf_status,
         info->read_idx,
         info->write_idx,
         info->item_status_count[SBUF_STATUS_DATA_EMPTY],
         info->item_status_count[SBUF_STATUS_DATA_PREPARE],
         info->item_status_count[SBUF_STATUS_DATA_DONE],
         info->item_status_count[SBUF_STATUS_DATA_USING] );

    spin_unlock_irqrestore( &p_sbuf_mgr->sbuf_lock, irq_flags );

    if ( rc ) {
        LOG( LOG_ERR, "Set item failed: buf_idx: %u, buf_type: %s, buf_status: %s, buf_base: %p.",
             item->buf_idx,
             sbuf_type_str[item->buf_type],
             sbuf_status_str[item->buf_status],
             item->buf_base );
    } else {
        LOG( LOG_DEBUG, "Set item OK: buf_type: %s, buf_idx: %u, buf_status: %s, buf_base: %p.",
             sbuf_type_str[item->buf_type],
             item->buf_idx,
             sbuf_status_str[item->buf_status],
             item->buf_base );
    }

    return rc;
}

int sbuf_bfmgr_set_item( struct sbuf_mgr *p_sbuf_mgr, struct sbuf_item *item )
{
    int rc;
    if ( !item ) {
        LOG( LOG_ERR, "Error: Invaid param, item is NULL." );
        return -EINVAL;
    }

    if ( !is_sbuf_inited( p_sbuf_mgr ) ) {
        LOG( LOG_ERR, "Error: sbuf is not initialized, can't set_item." );
        return -ENOMEM;
    }

    switch ( item->buf_type ) {
#if defined( ISP_HAS_AE_MANUAL_FSM )
    case SBUF_TYPE_AE:
        rc = sbuf_set_item_to_arr( p_sbuf_mgr, item, p_sbuf_mgr->ae_sbuf_arr, &p_sbuf_mgr->ae_arr_info );
        break;
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
    case SBUF_TYPE_AWB:
        rc = sbuf_set_item_to_arr( p_sbuf_mgr, item, p_sbuf_mgr->awb_sbuf_arr, &p_sbuf_mgr->awb_arr_info );
        break;
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
    case SBUF_TYPE_GAMMA:
        rc = sbuf_set_item_to_arr( p_sbuf_mgr, item, p_sbuf_mgr->gamma_sbuf_arr, &p_sbuf_mgr->gamma_arr_info );
        break;
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    case SBUF_TYPE_IRIDIX:
        rc = sbuf_set_item_to_arr( p_sbuf_mgr, item, p_sbuf_mgr->iridix_sbuf_arr, &p_sbuf_mgr->iridix_arr_info );
        break;
#endif
    default:
        LOG( LOG_ERR, "Error: Unsupported buf_type: %d.", item->buf_type );
        rc = -EINVAL;
        break;
    }

    return rc;
}
int sbuf_bfmgr_get_item( struct sbuf_mgr *p_sbuf_mgr, struct sbuf_item *item )
{
    int rc;

    if ( !item ) {
        LOG( LOG_ERR, "Error: Invaid param, item is NULL." );
        return -EINVAL;
    }

    if ( !is_sbuf_inited( p_sbuf_mgr ) ) {
        LOG( LOG_ERR, "Error: sbuf is not initialized, can't get_item." );
        return -ENOMEM;
    }

    switch ( item->buf_type ) {
#if defined( ISP_HAS_AE_MANUAL_FSM )
    case SBUF_TYPE_AE:
        rc = sbuf_get_item_from_arr( p_sbuf_mgr, item, p_sbuf_mgr->ae_sbuf_arr, &p_sbuf_mgr->ae_arr_info );
        break;
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
    case SBUF_TYPE_AWB:
        rc = sbuf_get_item_from_arr( p_sbuf_mgr, item, p_sbuf_mgr->awb_sbuf_arr, &p_sbuf_mgr->awb_arr_info );
        break;
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
    case SBUF_TYPE_GAMMA:
        rc = sbuf_get_item_from_arr( p_sbuf_mgr, item, p_sbuf_mgr->gamma_sbuf_arr, &p_sbuf_mgr->gamma_arr_info );
        break;
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
    case SBUF_TYPE_IRIDIX:
        rc = sbuf_get_item_from_arr( p_sbuf_mgr, item, p_sbuf_mgr->iridix_sbuf_arr, &p_sbuf_mgr->iridix_arr_info );
        break;
#endif
    default:
        LOG( LOG_ERR, "Error: Unsupported buf_type: %d.", item->buf_type );
        rc = -EINVAL;
        break;
    }

    return rc;
}
