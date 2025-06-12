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

#ifndef __ISP_PARAMS_H__
#define __ISP_PARAMS_H__

#include "common_params.h"


// ------------------------------------------------------------------------------ //
// ISP CLASS PARAMETERS
// ------------------------------------------------------------------------------ //
#define ACTIVE_CONTEXT_PARAM 0x00000100
#define CONTEXT_NUMBER_PARAM 0x00000101
#define FW_REVISION_PARAM 0x00000102
#define SYSTEM_LOGGER_LEVEL_PARAM 0x00000103
#define SYSTEM_LOGGER_MASK_PARAM 0x00000104


// ------------------------------------------------------------------------------ //
// ISP PARAM TYPE DEFINITION
// ------------------------------------------------------------------------------ //
typedef struct _isp_params {
    common_param params[ISP_PARAM_CLASS_MAX + 1];
} isp_params;


// ------------------------------------------------------------------------------ //
// PARAM INIT FUNCTION
// ------------------------------------------------------------------------------ //
void init_isp_params( isp_params *params );


#endif // __ISP_PARAMS_H__
