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

#include "acamera_command_api.h"
#include "acamera_logger.h"

extern int32_t get_calibrations_static_linear_imx477( void *param );
extern int32_t get_calibrations_dynamic_linear_dummy( void *param );

int32_t get_calibrations_imx477( uint32_t wdr_mode, void *param )
{
    int32_t ret = 0;

    switch ( wdr_mode ) {
    case WDR_MODE_LINEAR:
        LOG( LOG_DEBUG, "calibration switching to WDR_MODE_LINEAR %d ", (int)wdr_mode );
        ret += ( get_calibrations_static_linear_imx477( param ) +
				get_calibrations_dynamic_linear_dummy( param) );
        break;
    case WDR_MODE_NATIVE:
        LOG( LOG_CRIT, "Calibration mode WDR_MODE_NATIVE %d.NOT SUPPORTED", (int)wdr_mode );
		break;
    }

    return ret;
}
