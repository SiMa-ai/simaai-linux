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
#include <linux/kfifo.h>        // SPSC ring used for IRQ -> BH handoff.
#include <linux/printk.h>       // pr_warn_ratelimited().
#include <linux/workqueue.h>    // For bottom half approach.
#include "acamera_configuration.h" // FIRMWARE_CONTEXT_NUMBER
#include "acamera_interrupts.h" // For isr_data_t
#include "acamera_logger.h"     // LOG()

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_GENERIC

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

/* Per-context IRQ→BH staging. One kfifo + one work_struct per ISP
 * context. Slot id read from the MCFE register indexes directly into
 * m_irq_ctx[]: FIRMWARE_CONTEXT_NUMBER == ISP_MCFE_MAX_SLOT (16), so
 * the HW slot number IS the SW context index — no remap needed.
 *
 * Each fifo is SPSC by construction: producer is the IRQ handler
 * (pinned to ISP_CPU_CORE_NUMBER via irq_set_affinity), consumer is
 * the matching bh_work. The workqueue's max_active is set to
 * FIRMWARE_CONTEXT_NUMBER so different contexts' bottom halves run
 * concurrently on different CPUs without contention.
 */
#define ISP_IRQ_FIFO_DEPTH 64u
typedef struct {
    struct kfifo       isr_data_fifo;   /**< SPSC ring of isr_data_t records. */
    struct work_struct bh_work;         /**< One per context, lifetime = module. */
    uint32_t           ctx_id;          /**< Slot / context id, set at init. */
    atomic_t           drops;           /**< FIFO-full counter, ratelimited. */
} isp_irq_ctx_state_t;

static isp_irq_ctx_state_t m_irq_ctx[FIRMWARE_CONTEXT_NUMBER];


void system_set_global_flag( void )
{
    m_global_flag = 1;
}

#define ISP_CPU_CORE_NUMBER		(6)

/**
 * @brief      Bottom half handler.
 *
 * @details    Drains this context's kfifo and dispatches every queued
 *             isr_data_t record to the registered user handler. Runs
 *             in workqueue context; never sleeps under a lock.
 */
static void bh_work_handler( struct work_struct *work )
{
    isp_irq_ctx_state_t *s;
    isr_data_t           data;
    unsigned int         drained = 0u;

    if ( work == NULL ) return;

    s = container_of( work, isp_irq_ctx_state_t, bh_work );

    while ( kfifo_out( &s->isr_data_fifo, &data, sizeof( data ) ) == sizeof( data ) ) {
        drained++;
        LOG( LOG_DEBUG, "BH ctx=%u dispatch #%u slot=%u stats=0x%x mcfe=0x%x sof=0x%x eof=0x%x",
             (unsigned)s->ctx_id,
             drained,
             (unsigned)data.slot,
             (unsigned)data.regs.stats,
             (unsigned)data.regs.mcfe,
             (unsigned)data.regs.sof,
             (unsigned)data.regs.eof );
        if ( m_app_handler ) {
            m_app_handler( (void *)&data );
        }
    }
    if ( drained == 0u ) {
        LOG( LOG_DEBUG, "BH ctx=%u ran with empty fifo (spurious wakeup)",
             (unsigned)s->ctx_id );
    }
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
    isr_data_t data;
    isp_irq_ctx_state_t *s;

    (void)irq;    // Unused.
    (void)dev_id; // Unused.

    /* Read & ack the hardware first — we own this IRQ once we've acked it. */
    data.slot = acamera_interrupt_read_current_slot();
    data.regs = acamera_interrupt_read_acknowledge();

    LOG( LOG_DEBUG, "ISR slot=%u stats=0x%x mcfe=0x%x sof=0x%x eof=0x%x",
         (unsigned)data.slot,
         (unsigned)data.regs.stats,
         (unsigned)data.regs.mcfe,
         (unsigned)data.regs.sof,
         (unsigned)data.regs.eof );

    /* HW MCFE slot id maps 1:1 to SW context: FIRMWARE_CONTEXT_NUMBER ==
     * ISP_MCFE_MAX_SLOT (16). An out-of-range slot would mean the HW
     * register is corrupt — we still need to ack the IRQ (already done
     * above), so just drop the record. */
    if ( data.slot >= FIRMWARE_CONTEXT_NUMBER ) {
        pr_warn_ratelimited( "ISP IRQ slot %u out of range (max %u), dropping\n",
                             (unsigned)data.slot, FIRMWARE_CONTEXT_NUMBER - 1u );
        return IRQ_HANDLED;
    }

    s = &m_irq_ctx[data.slot];

    /* SPSC fast path: no alloc, no spinlock — one IRQ producer per fifo
     * (pinned to ISP_CPU_CORE_NUMBER), one consumer (this ctx's
     * bh_work). On overflow the record is dropped (already acked at
     * HW) and the per-context counter is bumped. */
    if ( kfifo_in( &s->isr_data_fifo, &data, sizeof( data ) ) != sizeof( data ) ) {
        atomic_inc( &s->drops );
        pr_warn_ratelimited( "ISP ctx %u IRQ FIFO full, drops=%u\n",
                             (unsigned)data.slot,
                             (unsigned)atomic_read( &s->drops ) );
        return IRQ_HANDLED;
    }

    queue_work( m_work_queue, &s->bh_work );
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
	uint32_t i;
	int rc_kfifo;

    if ( m_interrupt_request_status != ISP_IRQ_STATUS_DEINIT ) {
        /* Interrupts are already initialized. */
        printk( KERN_WARNING "Interrupts are already init'd." );
        return 0;
    }

    if ( m_irq_num < 0 ) {
        printk( KERN_CRIT "Invalid interrupt line requested." );
        return -EINVAL;
    }

    /* Per-context IRQ staging: one kfifo + one work_struct per context.
     * Allocated up front so the IRQ path never calls kmalloc. */
    for ( i = 0u; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        rc_kfifo = kfifo_alloc( &m_irq_ctx[i].isr_data_fifo,
                                ISP_IRQ_FIFO_DEPTH * sizeof( isr_data_t ),
                                GFP_KERNEL );
        if ( rc_kfifo != 0 ) {
            printk( KERN_CRIT "Failed to alloc IRQ fifo for ctx %u (rc=%d).", i, rc_kfifo );
            while ( i-- > 0u ) {
                kfifo_free( &m_irq_ctx[i].isr_data_fifo );
            }
            return -ENOMEM;
        }
        INIT_WORK( &m_irq_ctx[i].bh_work, bh_work_handler );
        m_irq_ctx[i].ctx_id = i;
        atomic_set( &m_irq_ctx[i].drops, 0 );
    }

    /// @note       Be careful when changing workqueue type, this will affect
    ///             performance of the system and may cause MCFE output
    ///             overflow!
    ///
    /// max_active = FIRMWARE_CONTEXT_NUMBER so different contexts' bottom
    /// halves can run on different CPUs concurrently. Each work_struct is
    /// distinct (one per ctx) and each consumes its own SPSC kfifo.
	m_work_queue = alloc_workqueue( "isp_bh_queue",
						WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE | WQ_SYSFS | WQ_MEM_RECLAIM,
						FIRMWARE_CONTEXT_NUMBER );
    if ( m_work_queue == NULL ) {
        printk( KERN_CRIT "Failed to allocate memory for bottom half work queue." );
        for ( i = 0u; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
            kfifo_free( &m_irq_ctx[i].isr_data_fifo );
        }
        return -ENOMEM;
    }

    const int rc = request_irq( m_irq_num,
                                &system_interrupt_handler,
                                m_irq_flags,
                                "isp",
                                m_vdev );

    if ( rc != 0 ) {
        printk( KERN_CRIT "Failed to register IRQ." );
        destroy_workqueue( m_work_queue );
        for ( i = 0u; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
            kfifo_free( &m_irq_ctx[i].isr_data_fifo );
        }
        return rc;
    }

	cpumask_clear(&mask);
	cpumask_set_cpu(ISP_CPU_CORE_NUMBER, &mask);
	irq_set_affinity(m_irq_num, &mask);

    LOG( LOG_NOTICE, "system_interrupts_init: %u ctx fifos x %u records each, irq=%d on CPU%d",
         (unsigned)FIRMWARE_CONTEXT_NUMBER, (unsigned)ISP_IRQ_FIFO_DEPTH,
         m_irq_num, ISP_CPU_CORE_NUMBER );

    m_interrupt_request_status = ISP_IRQ_STATUS_ENABLED;
    return 0;
}

void system_interrupts_deinit( void )
{
    uint32_t i;
    unsigned int total_drops = 0u;

    if ( m_interrupt_request_status == ISP_IRQ_STATUS_DEINIT ) {
        printk( KERN_WARNING "Interrupts are already deinit'd." );
        return;
    }
    system_interrupts_disable();

    /* Free the IRQ first so no new records can be pushed into the fifos
     * while we're tearing them down. */
    free_irq( m_irq_num, m_vdev );

    /* Drain any pending bottom halves, then destroy the workqueue. After
     * destroy_workqueue() returns no bh_work_handler can be running. */
    flush_workqueue( m_work_queue );
    destroy_workqueue( m_work_queue );

    /* Safe to release the per-context kfifos now: no producer (IRQ freed)
     * and no consumer (workqueue destroyed). */
    for ( i = 0u; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        total_drops += (unsigned)atomic_read( &m_irq_ctx[i].drops );
        kfifo_free( &m_irq_ctx[i].isr_data_fifo );
    }

    LOG( LOG_DEBUG, "system_interrupts_deinit: total drops across %u ctx fifos = %u",
         (unsigned)FIRMWARE_CONTEXT_NUMBER, total_drops );

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
