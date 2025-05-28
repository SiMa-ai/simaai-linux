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

#ifndef __MODULE_MCFE_SERVICE_BUF_H__
#define __MODULE_MCFE_SERVICE_BUF_H__

#include "module_mcfe_service.h"

/**
 * @brief      Initializes the pool.
 *
 * @param[in]  type The buffer type.
 *
 */
void module_mcfe_buf_init_pool( const mcfe_hwif_buf_type_t type );

/**
 * @brief      Returns next free buffer in the pool.
 *
 * @param[in]  type The buffer type.
 *
 * @return     Pointer to free buffer or NULL if no free buffer is found. 
 *
 */
module_mcfe_buf_t *module_mcfe_buf_get_free( const mcfe_hwif_buf_type_t type );

/**
 * @brief      Counts number of free buffers in the pool
 *
 * @param[in]  type The buffer type.
 *
 * @return     Number of free buffers.
 *
 */
size_t module_mcfe_buf_get_free_count( const mcfe_hwif_buf_type_t type );

/**
 * @brief      Destroys HW buffer by resetting it registers to defaults
 *
 * @param[in]  buffer Pointer to a buffer.
 *
 */
void module_mcfe_buf_destroy( module_mcfe_buf_t *const buffer );

/**
 * @brief      Creates and configures HW buffer
 *
 * @param[in]  config Pointer to a buffer configuration.
 *
 * @return     Pointer to a created buffer.
 *
 */
module_mcfe_buf_t *module_mcfe_buf_create( module_mcfe_buf_cfg_t *const config );

#endif // __MODULE_MCFE_SERVICE_BUF_H__