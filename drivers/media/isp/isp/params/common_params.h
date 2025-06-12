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

#ifndef __COMMON_PARAMS_H__
#define __COMMON_PARAMS_H__

#include "system_types.h"


// ------------------------------------------------------------------------------ //
// CONTEXT PARAMETERS CLASS
// ------------------------------------------------------------------------------ //
#define CONTEXT_PARAM_CLASS_MASK 0x00000080
#define CONTEXT_PARAM_CLASS_MAX 0x0000005F


// ------------------------------------------------------------------------------ //
// ISP PARAMETERS CLASS
// ------------------------------------------------------------------------------ //
#define ISP_PARAM_CLASS_MASK 0x00000100
#define ISP_PARAM_CLASS_MAX 0x00000004


// ------------------------------------------------------------------------------ //
// PARAM FLAGS
// ------------------------------------------------------------------------------ //
#define PARAM_FLAG_READ 0x00000001
#define PARAM_FLAG_WRITE 0x00000002
#define PARAM_FLAG_RECONFIG 0x00000004
#define PARAM_FLAG_RECONFIG_ON_LIST_VALUE 0x00000008
#define PARAM_FLAG_RECONFIG_ON_TRUE_DEFAULT_RESET 0x00000010
#define PARAM_FLAG_REGISTER_REQUEST 0x00000020
#define PARAM_FLAG_STOP_ON_FALSE 0x00000040
#define PARAM_FLAG_START_ON_TRUE 0x00000080
#define PARAM_FLAG_HANDLER_FUNCTION 0x00000100


// ------------------------------------------------------------------------------ //
// PARAM TYPE AND HANDLER FUNCTION DEFINITION
// ------------------------------------------------------------------------------ //
typedef struct _param_list_value {
    uint32_t value;
    uint32_t flags;
} param_list_value;

typedef uint8_t ( *param_handler_t )( void *instance, uint32_t set_value, uint8_t direction, uint32_t *get_value );

typedef struct _common_param {
    uint32_t value;
    uint32_t default_value;
    uint32_t min;
    uint32_t max;
    uint32_t flags;
    param_list_value *values_list;
    param_handler_t param_handler;
} common_param;

#endif // __COMMON_PARAMS_H__
