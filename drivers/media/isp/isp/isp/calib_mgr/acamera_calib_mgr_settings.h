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

#ifndef _ACAMERA_CALIB_MGR_SETTINGS_H_
#define _ACAMERA_CALIB_MGR_SETTINGS_H_

/* Slot IDs + total-size constant live in the shared UAPI header so the
 * kernel driver and libcamera IPA agree on the calibration blob layout
 * byte-for-byte. All kernel callers use the MODALIX_ISP_CALIB_* names
 * directly. */
#include <linux/media/simaai/modalix_isp_calib.h>

#endif /* _ACAMERA_CALIB_MGR_SETTINGS_H_ */
