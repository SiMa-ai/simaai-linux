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

#include "acamera_isp_ctx.h"

uint8_t ae_check_metering_stats_crc( histogram_fsm_ptr_t p_fsm );

/*
 * ae_read_aexp_zone_bins - snapshot, CRC-check and decode the aexp
 * per-zone metering block into p_fsm->aexp_zone_bins (four 16-bit
 * bins per zone) + aexp_nodes_h/v. Returns the zone count, 0 when
 * the block is unconfigured or the CRC does not match.
 */
uint32_t ae_read_aexp_zone_bins( histogram_fsm_ptr_t p_fsm );
