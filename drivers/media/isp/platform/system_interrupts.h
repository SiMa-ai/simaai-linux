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

/** @file system_interrupts.h
    @addtogroup   platform_interrupts Platform Interrupts
    @{
*/

#ifndef __SYSTEM_INTERRUPTS_H__
#define __SYSTEM_INTERRUPTS_H__

#include "system_types.h"

/** interrupt status. */
typedef enum {
    ISP_IRQ_STATUS_DEINIT = 0, /**< de-initialized */
    ISP_IRQ_STATUS_ENABLED,    /**< initialized: enabled*/
    ISP_IRQ_STATUS_DISABLED,   /**< initialized: disabled */
} irq_status_t;

/** Interrupt mask type.
 */
typedef uint32_t system_fw_interrupt_mask_t;

/** Interrupt callback handler type */
typedef void ( *system_interrupts_handler_t )( void *ptr );


/**
 * @brief      Initialize system interrupts
 *
 * @return     0 if successful, otherwise platform depedant error code.
 *
 * @details    This function initializes system dependent interrupt
 *             functionality. Usually only check if successful and report
 *             otherwise, if the init doesn't work there is not much we can do
 *             to handle it.
 */
int system_interrupts_init( void );


/**
 * @brief      De-initialize system interrupts
 *
 * @details    This function de-initializes system dependent interrupt
 *             functionality
 */
void system_interrupts_deinit( void );


/**
 * @brief      Set an interrupt handler
 *
 * @param      handler  a callback to handle interrupts
 * @param      param    pointer to a context which must be send to interrupt
 *                      handler
 *
 * @details    This function is used by application to set an interrupt handler
 *             for all ISP related interrupt events.
 */
void system_interrupts_set_handler( system_interrupts_handler_t handler, void *param );


/**
 * @brief      Enable system interrupts
 *
 * @details    This function is used by firmware to enable system interrupts in
 *             a case if they were disabled before
 */
void system_interrupts_enable( void );


/**
 * @brief      Disable system interrupts
 *
 * @details    This function is used by firmware to disable system interrupts
 *             for a short period of time. Usually IRQ register is updated by
 *             new interrupts but main interrupt handler is not called by a
 *             system.
 */
void system_interrupts_disable( void );

#endif /* __SYSTEM_INTERRUPTS_H__ */
/** @} */
