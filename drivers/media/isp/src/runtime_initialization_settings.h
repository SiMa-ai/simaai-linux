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


// Extern functions for dummy sensor
extern void sensor_init_dummy( void **, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const );
extern int32_t get_calibrations_dummy( uint32_t wdr_mode, void *param );

extern void sensor_init_imx477( void **, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const );


// Context settings
// clang-format off
static acamera_settings settings[FIRMWARE_CONTEXT_NUMBER] = {
#if ( FIRMWARE_CONTEXT_NUMBER >= 1 )
    {
        .sensor_init = sensor_init_imx477,
        .sensor_name = "DUMMY",

        .sensor_options = {
            .is_remote = 1,
            .preset_mode = 0,
        },

        .get_calibrations = get_calibrations_dummy,

        .context_options = {
            .cmd_if_is_passive_mode = 1,
        },

        .isp_base = 0,
    },
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 2 )
    {
        .sensor_init = sensor_init_imx477,
        .sensor_name = "DUMMY",

        .sensor_options = {
            .is_remote = 1,
            .preset_mode = 0,
        },

        .get_calibrations = get_calibrations_dummy,

        .context_options = {
            .cmd_if_is_passive_mode = 1,
        },

        .isp_base = 0,
    },
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 3 )
    {
        .sensor_init = sensor_init_imx477,
        .sensor_name = "DUMMY",

        .sensor_options = {
            .is_remote = 1,
            .preset_mode = 0,
        },

        .get_calibrations = get_calibrations_dummy,

        .context_options = {
            .cmd_if_is_passive_mode = 1,
        },

        .isp_base = 0,
    },
#endif
#if ( FIRMWARE_CONTEXT_NUMBER >= 4 )
    {
        .sensor_init = sensor_init_imx477,
        .sensor_name = "DUMMY",

        .sensor_options = {
            .is_remote = 1,
            .preset_mode = 0,
        },

        .get_calibrations = get_calibrations_dummy,

        .context_options = {
            .cmd_if_is_passive_mode = 1,
        },

        .isp_base = 0,
    }
#endif
};
// clang-format on
