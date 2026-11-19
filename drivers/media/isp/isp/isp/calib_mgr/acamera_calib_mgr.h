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

#ifndef __ACAMERA_CALIB_MGR_H__
#define __ACAMERA_CALIB_MGR_H__

/* The wire-format types and slot IDs live in the shared UAPI header
 * so the kernel driver and libcamera IPA agree on the calibration
 * blob layout byte-for-byte. acamera_calib_mgr.h now just provides
 * the kernel-internal API around them. */
#include <linux/media/simaai/modalix_isp_calib.h>

#include "acamera_calib_mgr_settings.h"
#include "acamera_isp_core_settings.h"
#include "acamera_math.h"

/* Kernel-internal aliases for the UAPI types. struct
 * modalix_isp_lookup_table is the on-the-wire descriptor; after the
 * kernel ioctl handler patches `ptr` offsets to real pointers, the
 * same struct doubles as the in-memory form. Callers that need a
 * void* must cast through uintptr_t (see calib_mgr_lut_get()). */
typedef struct modalix_isp_lookup_table LookupTable;
typedef struct modalix_isp_calibrations ACameraCalibrations;

/* Calibration manager entry */
typedef struct _acamera_calib_mgr_entry_t {
    int32_t initialized;
    int32_t ( *get_calibrations )( uint32_t wdr_mode, void *param );
    ACameraCalibrations c;
    /* Refreshed by acamera_calib_mgr_update(); see the getter below. */
    uint32_t calibration_bit_depth;
} acamera_calib_mgr_entry_t;

/**
 *   Initialize a calibration manager entry.
 *
 *   This function MUST be called before any other call to calibration manager.
 *   It initializes a pointer to get_calibations_*() function and inits internal data.
 *
 *   @param param - pointer to a get_calibations_*() function.
 *
 *   @return acamera_calib_mgr_entry_t pointer on success otherwise NULL
 */
acamera_calib_mgr_entry_t *acamera_calib_mgr_init( void *param );

/**
 *   Deinitialize a calibration manager entry.
 *
 *   @param entry - pointer to calibration manager entry.
 *
 *   @return none
 */
void acamera_calib_mgr_deinit( acamera_calib_mgr_entry_t *entry );

/**
 *   Update LUT tables of a calibration manager entry according to sensor mode.
 *
 *   @param entry - pointer to calibration manager entry.
 *   @param wdr_mode - sensor mode.
 *
 *   @return 0 on success otherwise -1
 */
int32_t acamera_calib_mgr_update( acamera_calib_mgr_entry_t *entry, uint32_t wdr_mode );

/**
 *   Check whether specified LUT exists in calibration manager entry.
 *
 *   @param entry - pointer to calibration manager entry.
 *   @param idx - LUT index
 *
 *   @return 1 if specified LUT exists otherwise 0
 */
int32_t calib_mgr_lut_exists( acamera_calib_mgr_entry_t *entry, uint32_t idx );

/**
 *   Get pointer to the specified LUT for read only operations.
 *   Pointer is const so treat data as read-only (do not recast to non-const)
 *i
 *inter to calibration manager entry.
 *index
 *i
 *   @return pointer to a LUT if specified LUT exists otherwise NULL
 */
const void *calib_mgr_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx );

/**
 *   Get a LUT the caller will read as a fixed byte extent - a struct, or an
 *   array of them - rather than through one of the typed element getters.
 *
 *   The typed getters check the element width the tuning declared against
 *   the width the call site reads. A caller that casts the data to a struct
 *   reads sizeof(struct) bytes whatever the descriptor says, so it has to
 *   state the extent instead and be told no when the tuning is smaller.
 *
 *   @param entry - pointer to calibration manager entry.
 *   @param idx   - LUT index
 *   @param bytes - how many bytes the caller will read
 *
 *   @return pointer to the LUT data, or NULL if it is absent or too small
 */
const void *calib_mgr_lut_get_sized( acamera_calib_mgr_entry_t *entry,
                                     uint32_t idx, uint32_t bytes );

const uint8_t *calib_mgr_u8_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx );

const uint16_t *calib_mgr_u16_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx );

const uint32_t *calib_mgr_u32_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx );

const modulation_entry_t *calib_mgr_mod16_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx );

const modulation_entry_32_t *calib_mgr_mod32_lut_get( acamera_calib_mgr_entry_t *entry, uint32_t idx );

/**
 *   Get pointer to the specified LUT for read-write operations.
 *
 *   @param entry - pointer to calibration manager entry.
 *   @param idx - LUT index
 *
 *   @return pointer to a LUT if specified LUT exists otherwise NULL
 */
uint32_t *calib_mgr_u32_lut_get_rw( acamera_calib_mgr_entry_t *entry, uint32_t idx );

/**
 *   Get property of a specified LUT.
 *
 *   @param entry - pointer to calibration manager entry.
 *   @param idx - LUT index
 *
 *   @return property value on success otherwise 0
 */
uint32_t calib_mgr_lut_rows( acamera_calib_mgr_entry_t *entry, uint32_t idx );

uint32_t calib_mgr_lut_cols( acamera_calib_mgr_entry_t *entry, uint32_t idx );

uint32_t calib_mgr_lut_len( acamera_calib_mgr_entry_t *entry, uint32_t idx );

uint32_t calib_mgr_lut_width( acamera_calib_mgr_entry_t *entry, uint32_t idx );

/* Bit depth the black-level / stats calibration values were authored at
 * (MODALIX_ISP_CALIB_CALIBRATION_BIT_DEPTH); default 12 when the tuning omits
 * it. The slot is optional, so probe with lut_exists first — the getters
 * assert on absent slots. */
/*
 * A value, not a pointer into the calibration set. The sensor-info path reads
 * this from an ioctl thread while the manager may be installing a replacement
 * set, and the set the slots pointed at is freed once the replacement is in.
 * The entry outlives every update, so a scalar cached in it is always safe to
 * read; the worst case is one update stale. Zero means no update has landed.
 */
static inline uint32_t calib_mgr_calibration_bit_depth( acamera_calib_mgr_entry_t *entry )
{
	return ( entry && entry->calibration_bit_depth ) ? entry->calibration_bit_depth
							 : 12u;
}

uint32_t calib_mgr_lut_size( acamera_calib_mgr_entry_t *entry, uint32_t idx );

/**
 *   Read/Write LUT data from/to supplied buffer.
 *
 *   @param entry - pointer to calibration manager entry.
 *   @param data - pointer to a buffer.
 *   @param data_size - size of a buffer in bytes.
 *   @param idx - LUT index.
 *
 *   @return number of bytes read/written otherwise 0
 */
uint32_t calib_mgr_lut_write( acamera_calib_mgr_entry_t *entry, const void *data, uint32_t data_size, uint32_t idx );
uint32_t calib_mgr_lut_read( acamera_calib_mgr_entry_t *entry, void *data, uint32_t data_size, uint32_t idx );

#endif // __ACAMERA_CALIB_MGR_H__
