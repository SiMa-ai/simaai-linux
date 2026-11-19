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

#include "acamera_calib_mgr.h"
#include "system_assert.h"
#include "system_stdlib.h"
#include "acamera_configuration.h"
#include "acamera_logger.h"

static acamera_calib_mgr_entry_t calib_mgr_data[FIRMWARE_CONTEXT_NUMBER];

acamera_calib_mgr_entry_t *acamera_calib_mgr_init( void *param )
{
    if ( param == NULL ) {
        return NULL;
    }

    int i, index = -1;

    // Find a free element for a new data
    for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        if ( calib_mgr_data[i].initialized == 0 ) {
            index = i;
            break;
        }
    }

    // Check if we found anything
    if ( index < 0 ) {
        LOG_CAT( ALOG_CAT_GENERAL, LOG_CRIT, "No free space available for new calibration manager entry (%d of %d used).", FIRMWARE_CONTEXT_NUMBER, FIRMWARE_CONTEXT_NUMBER );
        return NULL;
    }

    calib_mgr_data[index].get_calibrations = param;

    calib_mgr_data[index].initialized = 1;

    return &calib_mgr_data[index];
}

void acamera_calib_mgr_deinit( acamera_calib_mgr_entry_t *entry )
{
    if ( entry == NULL ) {
        return;
    }

    system_memset( entry, 0, sizeof( *entry ) );
}

/* The slot read behind calib_mgr_calibration_bit_depth(), done once per
 * update so that no reader has to hold a pointer into the calibration set. */
static uint32_t calib_mgr_read_calibration_bit_depth( acamera_calib_mgr_entry_t *entry )
{
    const uint16_t *v;

    if ( !calib_mgr_lut_exists( entry, MODALIX_ISP_CALIB_CALIBRATION_BIT_DEPTH ) ) {
        return 12u;
    }

    v = calib_mgr_u16_lut_get( entry, MODALIX_ISP_CALIB_CALIBRATION_BIT_DEPTH );
    /* Range-check the tuning-supplied depth: anything outside the ISP input
     * range is a malformed blob, and feeding it to the downstream shift math
     * would be undefined - fall back to the 12-bit default instead. */
    return ( v && *v >= 8u && *v <= ISP_INPUT_PORT_MAX_BIT_WIDTH ) ? *v : 12u;
}

int32_t acamera_calib_mgr_update( acamera_calib_mgr_entry_t *entry, uint32_t wdr_mode )
{
    int32_t result;

    if ( entry == NULL ) {
        return -1;
    }

    if ( entry->initialized != 1 ) {
        return -1;
    }

    /*
     * Fill a scratch set and publish it only if the hand-out succeeded.
     * Clearing the live slots first would leave the manager empty on a
     * failure that nobody acts on - control_config() ignores this result,
     * and required consumers index their slot without checking - so a
     * transient failure would take a working calibration down with it.
     */
    {
        /* On the stack, not static: there are FIRMWARE_CONTEXT_NUMBER
         * contexts and they update independently. 137 u64s. */
        ACameraCalibrations fresh;

        system_memset( fresh.calibrations, 0, sizeof( fresh.calibrations ) );

        result = entry->get_calibrations( wdr_mode, fresh.calibrations );
        if ( result == 0 ) {
            system_memcpy( entry->c.calibrations, fresh.calibrations,
                           sizeof( entry->c.calibrations ) );
        } else {
            LOG_CAT( ALOG_CAT_GENERAL, LOG_ERR,
                     "calibration update failed (%d); keeping the installed set",
                     result );
        }
    }

    /* Describes whatever is installed now, including nothing. */
    entry->calibration_bit_depth = calib_mgr_read_calibration_bit_depth( entry );

    return result;
}

/* The UAPI calibrations[] / lookup_table.ptr fields are typed as
 * __aligned_u64 so the wire format is 32/64-bit-portable. After the
 * ioctl handler patches the offsets to real kernel pointers, the
 * same byte storage is dereferenced as pointers via uintptr_t casts.
 * These two helpers centralise the cast so the rest of the driver
 * sees plain LookupTable * / void *. */
static inline LookupTable *calib_mgr_table_ptr( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (LookupTable *)(uintptr_t)entry->c.calibrations[idx];
}

static inline const void *calib_mgr_lut_data_ptr( const LookupTable *lut )
{
    return (const void *)(uintptr_t)lut->ptr;
}

int32_t calib_mgr_lut_exists( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    if ( ( idx < MODALIX_ISP_CALIB_TOTAL_SIZE ) && ( entry != NULL )
         && ( entry->c.calibrations[idx] != 0 )
         && ( calib_mgr_table_ptr( entry, idx )->ptr != 0 ) ) {
        return 1;
    } else {
        return 0;
    }
}

static inline LookupTable *calib_mgr_lookup_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    if ( idx < MODALIX_ISP_CALIB_TOTAL_SIZE ) {
        return ( ( entry != NULL ) ? calib_mgr_table_ptr( entry, idx ) : NULL );
    } else {
        LOG_CAT( ALOG_CAT_GENERAL, LOG_CRIT, "Calibration LUT index %d is out of range [0..%d]", (int)idx, MODALIX_ISP_CALIB_TOTAL_SIZE - 1 );
        return NULL;
    }
}

const void *calib_mgr_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    const void *result = NULL;
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );
    if ( lut != NULL ) {
        result = calib_mgr_lut_data_ptr( lut );
    } else {
        /* A slot the tuning did not provide is missing input, not a broken
         * invariant, and input arrives from user space. panic() is not an
         * answer to that: the boundary refuses a blob that omits a slot the
         * kernel reads unguarded, and everything else is guarded by
         * calib_mgr_lut_exists(). Say so and hand back NULL. */
        LOG_CAT( ALOG_CAT_GENERAL, LOG_CRIT, "Calibration LUT %d(%#X) was not provided by the tuning", (int)idx, (int)idx );
    }
    return result;
}

/* The blob says how wide a LUT's elements are; the FSM says how wide it
 * expects them by choosing a getter. The two have to agree, because the
 * count a consumer iterates - calib_mgr_lut_rows(), or calib_mgr_lut_len()
 * for the flat getters - is in elements, not bytes. Reading a width-2 LUT
 * through the u32 getter walks twice its data.
 *
 * The kernel cannot check this where the blob arrives: the boundary can
 * only tell whether a descriptor is self-consistent and inside the blob.
 * Only the call site knows the type it is about to read, so that is where
 * the check belongs. @cols is 0 when any column count will do, or 2 for
 * the modulation getters, whose entries are an (x, y) pair per row.
 */
static const void *calib_mgr_lut_get_typed( acamera_calib_mgr_entry_t *entry,
                                            uint32_t idx, uint32_t width, uint32_t cols )
{
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );

    if ( ( lut != NULL ) && ( lut->ptr != 0 ) &&
         ( ( lut->width != width ) || ( ( cols != 0 ) && ( lut->cols != cols ) ) ) ) {
        LOG_CAT( ALOG_CAT_GENERAL, LOG_CRIT,
                 "Calibration LUT %d(%#X) is %u x %u of %u-byte elements, but is being read as %u x %u-byte. Refusing it.",
                 (int)idx, (int)idx, (unsigned)lut->rows, (unsigned)lut->cols,
                 (unsigned)lut->width, (unsigned)cols, (unsigned)width );
        return NULL;
    }

    /* absent or empty: take the path the untyped getter has always taken,
     * so a missing slot still fails exactly the way callers expect */
    return calib_mgr_lut_get( entry, idx );
}

const void *calib_mgr_lut_get_sized( acamera_calib_mgr_entry_t *entry,
                                     uint32_t idx, uint32_t bytes )
{
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );

    if ( ( lut != NULL ) && ( lut->ptr != 0 ) ) {
        /* the boundary bounded this product against the blob, so it cannot
         * overflow here */
        const uint32_t have = (uint32_t)lut->rows * lut->cols * lut->width;

        if ( have < bytes ) {
            LOG_CAT( ALOG_CAT_GENERAL, LOG_CRIT,
                     "Calibration LUT %d(%#X) holds %u bytes but the caller reads %u. Refusing it.",
                     (int)idx, (int)idx, have, bytes );
            return NULL;
        }
    }

    return calib_mgr_lut_get( entry, idx );
}

// Use fast version of LUT access routines.
inline const uint8_t *calib_mgr_u8_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const uint8_t *)calib_mgr_lut_get_typed( entry, idx, 1, 0 );
}

inline const uint16_t *calib_mgr_u16_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const uint16_t *)calib_mgr_lut_get_typed( entry, idx, 2, 0 );
}

inline const uint32_t *calib_mgr_u32_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const uint32_t *)calib_mgr_lut_get_typed( entry, idx, 4, 0 );
}

inline uint32_t *calib_mgr_u32_lut_get_rw( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (uint32_t *)calib_mgr_lut_get_typed( entry, idx, 4, 0 );
}

inline const modulation_entry_t *calib_mgr_mod16_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const modulation_entry_t *)calib_mgr_lut_get_typed( entry, idx, 2, 2 );
}

inline const modulation_entry_32_t *calib_mgr_mod32_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const modulation_entry_32_t *)calib_mgr_lut_get_typed( entry, idx, 4, 2 );
}

inline uint32_t calib_mgr_lut_rows( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    uint32_t result = 0;
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );
    if ( lut != NULL ) {
        result = lut->rows;
    }
    return result;
}

inline uint32_t calib_mgr_lut_cols( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    uint32_t result = 0;
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );
    if ( lut != NULL ) {
        result = lut->cols;
    }
    return result;
}

inline uint32_t calib_mgr_lut_len( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    uint32_t result = 0;
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );
    if ( lut != NULL ) {
        result = lut->cols * lut->rows;
    }
    return result;
}

inline uint32_t calib_mgr_lut_width( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    uint32_t result = 0;
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );
    if ( lut != NULL ) {
        result = lut->width;
    }
    return result;
}

inline uint32_t calib_mgr_lut_size( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    uint32_t result = 0;
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );
    if ( lut != NULL ) {
        result = lut->cols * lut->rows * lut->width;
    }
    return result;
}

uint32_t calib_mgr_lut_read( acamera_calib_mgr_entry_t *entry, void *data, uint32_t data_size, uint32_t idx )
{
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );

    if ( lut == NULL || lut->ptr == 0 || data == NULL ) {
        return 0;
    }

    uint32_t lut_size = lut->cols * lut->rows * lut->width;

    if ( data_size != lut_size ) {
        return 0;
    }

    system_memcpy( data, calib_mgr_lut_data_ptr( lut ), lut_size );

    return lut_size;
}

uint32_t calib_mgr_lut_write( acamera_calib_mgr_entry_t *entry, const void *data, uint32_t data_size, uint32_t idx )
{
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );

    if ( lut == NULL || lut->ptr == 0 || data == NULL ) {
        return 0;
    }

    uint32_t lut_size = lut->cols * lut->rows * lut->width;

    if ( data_size != lut_size ) {
        return 0;
    }

    system_memcpy( (void *)(uintptr_t)lut->ptr, data, lut_size );

    return lut_size;
}
