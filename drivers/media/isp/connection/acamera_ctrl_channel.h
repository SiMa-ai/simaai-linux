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

#ifndef ACAMERA_CTRL_CHANNEL_H
#define ACAMERA_CTRL_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "system_types.h"
#include "acamera_configuration.h"
#include "acamera_settings.h"

/* Calibration-storage bridge. Backs the per-ctx calibration the kernel
 * firmware's calib_mgr read path consumes; populated by the V4L2
 * MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB control on the meta-stats node
 * (s_ctrl handler delegates to modalix_isp_install_user_calibrations).
 * The legacy /dev/isp_control_<N> chardev and its IOCTL/kfifo command
 * surface were retired in the V4L2 META migration. */

int32_t ctrl_channel_init( acamera_settings *settings, uint8_t num_of_contexts );
void    ctrl_channel_deinit( void );
void    ctrl_channel_process( void );  /* no-op; retained for callsite stability */

/**
 * modalix_isp_install_user_calibrations() - install a calibration blob
 * into the per-ctx slot the cbs[]/calib_mgr_update() chain reads.
 * Called from the meta-stats V4L2 s_ctrl handler when the IPA writes
 * MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB.
 *
 * @ctx_id: which ISP context the blob targets.
 * @data:   kernel-owned pointer to the offset-form blob.
 * @size:   total blob byte count (≤ MODALIX_ISP_V4L2_CALIBRATION_BLOB_MAX).
 *
 * Takes a copy, patches the offset-form __aligned_u64 fields to kernel
 * pointers in place, and atomically swaps it into the per-ctx slot.
 * Returns 0 on success or a negative errno.
 */
int modalix_isp_install_user_calibrations( uint32_t ctx_id, const void *data, size_t size );

/**
 * modalix_isp_apply_pipeline_bypass() - apply the IPA's ISP pipeline
 * bypass word (Mali reg 0xE040) to context @ctx_id. Masked RMW that
 * preserves reserved bits. Called from the meta-stats V4L2 s_ctrl
 * handler on MODALIX_ISP_V4L2_CID_ISP_BYPASS_CONFIG. Returns 0 or
 * a negative errno.
 */
int modalix_isp_apply_pipeline_bypass( uint32_t ctx_id, uint32_t bypass_word );

/**
 * modalix_isp_read_pipeline_bypass() - read back the live bypass word
 * for context @ctx_id into @out. Backs the G_EXT_CTRLS readback path.
 */
int modalix_isp_read_pipeline_bypass( uint32_t ctx_id, uint32_t *out );

#ifdef __cplusplus
}
#endif

#endif /* ACAMERA_CTRL_CHANNEL_H */
