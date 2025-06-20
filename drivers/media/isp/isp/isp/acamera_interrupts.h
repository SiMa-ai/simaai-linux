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

/** @file acamera_interrupts.h
    @addtogroup   acamera_interrupts ISP Interrupts
    @{
*/

#ifndef __ACAMERA_INTERRUPTS_H__
#define __ACAMERA_INTERRUPTS_H__

#include "system_types.h"

/**
 * @brief      stores the interrupt register state
 * @note       not all bits are used but the ISP registers are 32 bits.
 */
typedef struct _acamera_interrupt_regs_t {
    uint32_t stats; /**< statstical interrupts */
    uint32_t mcfe;  /**< mcfe interrupts @note MALI-C71 only*/
    uint16_t sof;   /**< start of frame interrupts */
    uint16_t eof;   /**< end of frame interrupts */
} acamera_interrupt_regs_t;

/** Contains data which time critical to be read during interrupt and passed to a
 * handler. */
typedef struct _isr_data_t {
    acamera_interrupt_regs_t regs; /**<  register status*/
    uint8_t slot;                  /**< active slot */
} isr_data_t;

/**
 * @brief      Reads & acknowledges interrupt status
 *
 * @return     #acamera_interrupt_regs_t
 *
 */
acamera_interrupt_regs_t acamera_interrupt_read_acknowledge( void );

/**
 * @brief      Reads currently active slot
 *
 * @return     Slot id
 */
uint8_t acamera_interrupt_read_current_slot( void );

/**
 * @brief      Inits the ISP interrupts
 *
 */
void acamera_interrupt_init( void );

/**
 * @brief      Enables the ISP interrupts
 *
 */
void acamera_interrupt_enable( void );

/**
 * @brief      Disables the ISP interrupts
 *
 * @details    Disables ALL interrupts.
 */
void acamera_interrupt_disable( void );

/**
 * @brief      De-inits the ISP interrupts
 *
 * @details    Masks and disables all interrupts.
 */
void acamera_interrupt_deinit( void );

/**
 * @brief      Single context handler
 *
 * @param      param  Pointer to #isr_data_t structure which contains registers
 *                    and slot information.
 *
 * @details    This handler reads the registers and translates hardware
 *             interrupts to logical IRQs defined by #acamera_irq_t
 * @warning    This function *may* run from ISR context. On Linux this will be
 *             called from bottom half handler. It is still recommended to apply
 *             ISR best practises.
 */
void acamera_interrupt_handler( void *param );

#endif /* __ACAMERA_INTERRUPTS_H__ */
/** @} */
