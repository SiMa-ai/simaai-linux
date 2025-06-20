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

/** @file acamera_interrupt_numbers.h
    @addtogroup   acamera_interrupts ISP Interrupts
    @{
*/
#ifndef __ACAMERA_IRQ_TYPES_H__
#define __ACAMERA_IRQ_TYPES_H__
#include "system_types.h" //stdint
#include "bitop.h"        //BIT


/** Interrupt mask type.
 */
typedef uint32_t system_fw_interrupt_mask_t;

/** Logical interrupts generated from the ISP registers. */
typedef enum {
    ACAMERA_IRQ_BE_FRAME_END,      /**< Triggered by either MCFE or backend output formatter. */
    ACAMERA_IRQ_FE_FRAME_END,      /**< Triggered by front end frame end*/
    ACAMERA_IRQ_FE_FRAME_START,    /**< Triggered by front end frame start. */
    ACAMERA_IRQ_BE_FRAME_START,    /**< Triggered by back end, usually output formatter. */
    ACAMERA_IRQ_AE_STATS,          /**< Triggered by Start of frame front end (for previous slot) */
    ACAMERA_IRQ_AWB_STATS,         /**< Triggered by Start of frame front end (for previous slot)*/
    ACAMERA_IRQ_ANTIFOG_HIST,      /**< Triggered by Start of frame front end (for previous slot)*/
    ACAMERA_IRQ_AE_METERING_STATS, /**< Triggered by Start of frame front end (for previous slot)*/
    ACAMERA_IRQ_ISP,               /**< Triggered by the model.*/
    ACAMERA_IRQ_COUNT              /**< Number of logical interrupts. */
} acamera_irq_t;


#endif /*__ACAMERA_IRQ_TYPES_H__*/
/**@}*/
