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

/** @file system_interrupts.c */
#include "system_interrupts.h"
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>         // kmalloc()
#include <linux/workqueue.h>    // For bottom half approach.
#include "acamera_interrupts.h" // For isr_data_t

volatile uint32_t *mcfe_slot_read = NULL;
volatile uint32_t *start_status_read = NULL;
volatile uint32_t *start_mask_read = NULL;

volatile uint32_t *end_status_read = NULL;
volatile uint32_t *end_mask_read = NULL;

volatile uint32_t *stats_status_read = NULL;
volatile uint32_t *stats_mask_read = NULL;

volatile uint32_t *mcfe_status_read = NULL;
volatile uint32_t *mcfe_mask_read = NULL;

volatile uint32_t *start_clear_write = NULL;
volatile uint32_t *end_clear_write = NULL;

volatile uint32_t *stats_clear_write = NULL;
volatile uint32_t *mcfe_clear_write = NULL;

/* Cross platform globals. */
static system_interrupts_handler_t m_app_handler = NULL;                /**< Pointer to user interrupt handler. */
static void *m_app_param = NULL;                                        /**< Param to be passed to #m_app_handler. */
static irq_status_t m_interrupt_request_status = ISP_IRQ_STATUS_DEINIT; /**< Local to indicate module status. */

/* platform specific */
static void *m_vdev = NULL;
static int m_irq_num = -1;                    /**< Kernel IRQ line. */
static int m_irq_flags = -1;                  /**< Kernel IRQ flags. */
static int m_global_flag = 0;                 /**< Unsure what this is used for! */
static struct workqueue_struct *m_work_queue; /**< linux bottom half work queue. */

/** Work item typedef. Required for bottom half. */
typedef struct {
    struct work_struct work; /**< Work struct, required by BH. */
    isr_data_t data;         /**< Time sensitive data read by ISR, read on IRQ. */
} isp_work_t;


void system_set_global_flag( void )
{
    m_global_flag = 1;
}

#define ISP_CPU_CORE_NUMBER		(8)

/**
 * @brief      Bottom half handler.
 *
 * @param      work  The work
 *
 * @details    Passes the pointer to #isr_data_t to user handler.
 */
static void bh_work_handler( struct work_struct *work )
{
    /* Sanity null check. */
    if ( work == NULL ) return;

    isp_work_t *w = (isp_work_t *)work;

    if ( m_app_handler ) {
        m_app_handler( (void *)&w->data );
    }

    kfree( (void *)work );
}

/**
 * @brief      System interrupt handler.
 *
 * @param[in]  irq     The irq
 * @param      dev_id  Device id
 *
 * @return     One of the following codes:
 * - IRQ_NONE        Interrupt was not from this device or was not handled.
 * - IRQ_HANDLED     Interrupt was handled by this device.
 * - IRQ_WAKE_THREAD Handler requests to wake the handler thread.
 *
 * @details    Reads and acks the interrupt, passes the data to be handled in
 *             bottom half.
 */
static irqreturn_t system_interrupt_handler( int irq, void *dev_id )
{
    (void)irq;    // Unused.
    (void)dev_id; // Unused.

    /* Allocate work item using ATOMIC since we are running in ISR context. */
    isp_work_t *w_item = (isp_work_t *)kmalloc( sizeof( isp_work_t ), GFP_ATOMIC );

	//printk("Interrupt recieved %d", irq);

    if ( w_item == NULL ) {
        /* Failed to allocate work item. Ensure we acknowledged the interrupts
         * anyway but ignore the results. */
        (void)acamera_interrupt_read_acknowledge();
        printk( KERN_CRIT "Failed to allocate work item for bottom half handler!" );
        return IRQ_NONE;
    }

    INIT_WORK( (struct work_struct *)w_item, bh_work_handler );

    // Read ISP registers.
    w_item->data.slot = acamera_interrupt_read_current_slot();
    w_item->data.regs = acamera_interrupt_read_acknowledge();

    if ( queue_work( m_work_queue, (struct work_struct *)w_item ) == 0 ) {
        // Failed to enqueue, free to avoid memory leak.
        kfree( (void *)w_item );
        printk( KERN_CRIT "Failed to enqueue work item for bottom half handler!" );
        return IRQ_NONE;
    }

    // Succeeded to queue up work.
    return IRQ_HANDLED;
}

void system_interrupts_set_irq( void *pdev, int irq_num, int flags )
{
    m_vdev = pdev;
    m_irq_num = irq_num;
    m_irq_flags = ( flags & IRQF_TRIGGER_MASK ) | IRQF_SHARED;
}


int system_interrupts_init( void )
{
	cpumask_t mask;

    if ( m_interrupt_request_status != ISP_IRQ_STATUS_DEINIT ) {
        /* Interrupts are already initialized. */
        printk( KERN_WARNING "Interrupts are already init'd." );
        return 0;
    }

    if ( m_irq_num < 0 ) {
        printk( KERN_CRIT "Invalid interrupt line requested." );
        return -EINVAL;
    }

    /// @note       Be careful when changing workqueue type, this will affect
    ///             performance of the system and may cause MCFE output
    ///             overflow!
    m_work_queue = create_workqueue( "isp_bh_queue" );

    if ( m_work_queue == NULL ) {
        printk( KERN_CRIT "Failed to allocate memory for bottom half work queue." );
        return -ENOMEM;
    }

    const int rc = request_irq( m_irq_num,
                                &system_interrupt_handler,
                                m_irq_flags,
                                "isp",
                                m_vdev );

    if ( rc != 0 ) {
        printk( KERN_CRIT "Failed to register IRQ." );
        /* Failed to request the irq, destroy the workqueue to avoid leaks. */
        destroy_workqueue( m_work_queue );
        return rc;
    }

	cpumask_clear(&mask);
	cpumask_set_cpu(ISP_CPU_CORE_NUMBER, &mask);
	irq_set_affinity(m_irq_num, &mask);

    m_interrupt_request_status = ISP_IRQ_STATUS_ENABLED;
    return 0;
}

void system_interrupts_deinit( void )
{

    if ( m_interrupt_request_status == ISP_IRQ_STATUS_DEINIT ) {
        printk( KERN_WARNING "Interrupts are already deinit'd." );
        return;
    }
    system_interrupts_disable();

    /* Runs all items on the work queue. */
    flush_workqueue( m_work_queue );

    /* Safely destroys the workqueue, all work will be ran (hence deallocated). */
    destroy_workqueue( m_work_queue );

    free_irq( m_irq_num, m_vdev );
    m_interrupt_request_status = ISP_IRQ_STATUS_DEINIT;
}

void system_interrupts_set_handler( system_interrupts_handler_t handler, void *param )
{

    system_interrupts_disable();

    m_app_handler = handler;
    m_app_param = param;

    system_interrupts_enable();
}

void system_interrupts_enable( void )
{
    enable_irq( m_irq_num );
}

void system_interrupts_disable( void )
{
    disable_irq( m_irq_num );
}

irq_status_t system_interrupts_status( void )
{
    return m_interrupt_request_status;
}
