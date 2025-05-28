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
        LOG( LOG_CRIT, "No free space available for new calibration manager entry (%d of %d used).", FIRMWARE_CONTEXT_NUMBER, FIRMWARE_CONTEXT_NUMBER );
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

int32_t acamera_calib_mgr_update( acamera_calib_mgr_entry_t *entry, uint32_t wdr_mode )
{
    if ( entry == NULL ) {
        return -1;
    }

    if ( entry->initialized != 1 ) {
        return -1;
    }

    // Fill calibration manager entry data
    system_memset( entry->c.calibrations, 0, sizeof( entry->c.calibrations ) );

    return entry->get_calibrations( wdr_mode, entry->c.calibrations );
}

int32_t calib_mgr_lut_exists( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    if ( ( idx < CALIBRATION_TOTAL_SIZE ) && ( entry != NULL ) && ( entry->c.calibrations[idx] != NULL ) && ( entry->c.calibrations[idx]->ptr != NULL ) ) {
        return 1;
    } else {
        return 0;
    }
}

static inline LookupTable *calib_mgr_lookup_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    if ( idx < CALIBRATION_TOTAL_SIZE ) {
        return ( ( entry != NULL ) ? ( entry->c.calibrations[idx] ) : NULL );
    } else {
        LOG( LOG_CRIT, "Calibration LUT index %d is out of range [0..%d]", (int)idx, CALIBRATION_TOTAL_SIZE - 1 );
        return NULL;
    }
}

const void *calib_mgr_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    const void *result = NULL;
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );
    if ( lut != NULL ) {
        result = lut->ptr;
    } else {
        LOG( LOG_CRIT, "Calibration LUT %d(%#X) is not initialized (NULL). Going to the infinite loop", (int)idx, (int)idx );
        assert( 0 );
    }
    return result;
}

// Use fast version of LUT access routines.
inline const uint8_t *calib_mgr_u8_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const uint8_t *)calib_mgr_lut_get( entry, idx );
}

inline const uint16_t *calib_mgr_u16_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const uint16_t *)calib_mgr_lut_get( entry, idx );
}

inline const uint32_t *calib_mgr_u32_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const uint32_t *)calib_mgr_lut_get( entry, idx );
}

inline uint32_t *calib_mgr_u32_lut_get_rw( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (uint32_t *)calib_mgr_lut_get( entry, idx );
}

inline const modulation_entry_t *calib_mgr_mod16_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const modulation_entry_t *)calib_mgr_lut_get( entry, idx );
}

inline const modulation_entry_32_t *calib_mgr_mod32_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx )
{
    return (const modulation_entry_32_t *)calib_mgr_lut_get( entry, idx );
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

    if ( lut == NULL || lut->ptr == NULL || data == NULL ) {
        return 0;
    }

    uint32_t lut_size = lut->cols * lut->rows * lut->width;

    if ( data_size != lut_size ) {
        return 0;
    }

    system_memcpy( data, lut->ptr, lut_size );

    return lut_size;
}

uint32_t calib_mgr_lut_write( acamera_calib_mgr_entry_t *entry, const void *data, uint32_t data_size, uint32_t idx )
{
    LookupTable *lut = calib_mgr_lookup_get( entry, idx );

    if ( lut == NULL || lut->ptr == NULL || data == NULL ) {
        return 0;
    }

    uint32_t lut_size = lut->cols * lut->rows * lut->width;

    if ( data_size != lut_size ) {
        return 0;
    }

    system_memcpy( (void *)lut->ptr, data, lut_size );

    return lut_size;
}
