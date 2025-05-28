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

#ifndef _MODULE_MCFE_COMMON_H_
#define _MODULE_MCFE_COMMON_H_

#include "system_stdlib.h"
#include "acamera_configuration.h"
#include "acamera_isp_config.h"
#include "acamera_logger.h"

/* MCFE errors */
#define MCFE_ERR_NONE ( 0 )
#define MCFE_ERR_GENERAL ( -1 )
#define MCFE_ERR_INV_PARM ( -2 )
#define MCFE_ERR_NOT_INIT ( -3 )
#define MCFE_ERR_NOT_RUN ( -4 )
#define MCFE_ERR_BUSY ( -5 )
#define MCFE_ERR_NO_RES ( -6 )

/* Default values */
#define MODULE_MCFE_DEFAULT_WIDTH 2048
#define MODULE_MCFE_DEFAULT_HEIGHT 1080
#define MODULE_MCFE_DEFAULT_HC_START 0
#define MODULE_MCFE_DEFAULT_VC_START 0

/* Max capability information */
#define MODULE_MCFE_SLOT_ARRAY_SIZE 16
#define MODULE_MCFE_INPUT_PORT_MAX 4

#define MODULE_MCFE_MSB_ALIGN_INPUT 0x1

#define MODULE_MCFE_SLOT_INPUT_FIRST_ONLY 1
#define MODULE_MCFE_SLOT_INPUT_ALL_INPUTS 0

#define MODULE_MCFE_BUFSET_MAX_FRAMES 2

#endif
