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

#ifndef __ACAMERA_H__
#define __ACAMERA_H__

#include "system_semaphore.h"
#include "system_types.h"
#include "acamera_logger.h"
#include "acamera_settings.h"
#include "isp_params.h"


////////////////////////////////////////////////////
// ISP API functions

/**
 * @brief      Initialize ISP instance.
 *
 * @details    The firmware can control several context at the same time. Each
 *             context must be initialized with its own set of setting and
 *             independently from all other contexts.
 * 
 * @return     0 on success
 */
int32_t acamera_init( acamera_settings *settings, uint8_t num_of_contexts, semaphore_t sem );


/**
 * @brief      Terminate the firmware.
 *
 * @details    The function will close the firmware and free all previously
 *             allocated resources.
 * 
 * @return     0 on success
 */
int32_t acamera_deinit( void );


/**
 * @brief      Raise config event for given context.
 */
void acamera_config_ctx( uint8_t ctx_id );


/**
 * @brief      Raise config event for given context.
 */
void acamera_start_ctx( uint8_t ctx_id );


/**
 * @brief      Raise config event for given context.
 */
void acamera_stop_ctx( uint8_t ctx_id );


/**
 * @brief      Raise config event for given context.
 */
void acamera_deinit_ctx( uint8_t ctx_id );


/**
 * @brief      acamera ISP interrupt handler
 *
 * @details    Ideally #acamera_isp_ctx_process_interrupt should take a mask rather than
 *             IRQ number and handle all events for given slot.
 */
void acamera_process_interrupt( uint8_t slot, const uint32_t mask );


/**
 * @brief      acamera ISP event handler.
 *
 *
 * @details    The firmware can control several context at the same time. Each
 *             context must be given a CPU time to fulfil all tasks it has at
 *             the moment. This function has to be called for all contexts as
 *             frequent as possible to avoid delays.
 */
int32_t acamera_process_event( void );

////////////////////////////////////////////////////
// ISP param handlers

uint32_t get_isp_param( uint32_t param_id );

void set_isp_param( uint32_t param_id, uint32_t value );

void override_isp_param( uint32_t param_id, uint32_t value );

uint8_t acamera_handle_param( uint32_t ctx_id, uint32_t param_id, uint8_t direction, uint32_t set_value, uint32_t *get_value );


#endif // __ACAMERA_H__
