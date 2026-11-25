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

#include "acamera_configuration.h"
#include "acamera_settings.h"


/* All contexts are identical at init time — the sensor is resolved later
 * from the real V4L2 subdev and calibrations from the IPA-pushed blob
 * (V4L2 MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB; ctrl_channel_init()
 * overwrites .get_calibrations per ctx). */
#define ACAMERA_CTX_SETTINGS_DEFAULT \
    {                                                       \
        .sensor_name = "COMMON",                            \
        .get_calibrations = NULL,                           \
        .context_options = {                                \
            .cmd_if_is_passive_mode = 1,                    \
        },                                                  \
        .isp_base = 0,                                      \
    }

// Context settings
// clang-format off
static acamera_settings settings[FIRMWARE_CONTEXT_NUMBER] = {
#if ( FIRMWARE_CONTEXT_NUMBER >= 1 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 2 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 3 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 4 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 5 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 6 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 7 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 8 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 9 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 10 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 11 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 12 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 13 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 14 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 15 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 16 )
    ACAMERA_CTX_SETTINGS_DEFAULT,
#endif
};
// clang-format on
