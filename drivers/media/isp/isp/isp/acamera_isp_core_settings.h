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

#if !defined( __ACAMERA_ISP_CORE_SETTINGS_H__ )
#define __ACAMERA_ISP_CORE_SETTINGS_H__

#define ISP_DIGITAL_GAIN_INPUT_BIT_WIDTH 24
#define ISP_FRAME_STITCH_INPUT_BIT_WIDTH 16
#define ISP_HISTOGRAM_INPUT_BIT_WIDTH 20
#define ISP_INPUT_PORT_MAX_BIT_WIDTH 20
#define ISP_MCFE_HWIF_MAX_OUTPUTS 3
#define ISP_MCFE_MAX_INPUT 4
#define ISP_MCFE_MAX_OUT_BUF 64
#define ISP_MCFE_MAX_RAW_BUF 32
#define ISP_MCFE_MAX_SLOT 16
#define ISP_MCFE_START_SLOT_IDLE 0xff
#define ISP_METERING_ANTIFOG_HISTOGRAM_OFFSET 8448
#define ISP_METERING_HISTOGRAM_SIZE_BINS 1024
#define ISP_METERING_HISTOGRAM_SIZE_BYTES 2048
#define ISP_METERING_HISTOGRAM_STRIDE_BYTES 2112
#define ISP_METERING_HISTOGRAM_TAIL_SIZE_REGS 12
#define ISP_METERING_HISTOGRAM_ZONES_MAX 225
#define ISP_METERING_OFFSET_AWB 1856
#define ISP_OUT_FORMAT_LUT_RGB_SIZE_REGS 257

#endif /* __ACAMERA_ISP_CORE_SETTINGS_H__*/
