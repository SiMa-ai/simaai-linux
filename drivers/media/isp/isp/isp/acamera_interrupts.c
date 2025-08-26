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

/**
 @file acamera_interrupts.c
 @todo       Figure out how how to better enable/disable interrupts used for the
             given use case.
 @addtogroup acamera_interrupts ISP Interrupts
 @{
*/

#include "acamera_interrupts.h"
#include "system_assert.h"
#include "acamera.h" // acamera_process_interrupt()
#include "acamera_configuration.h"
#include "acamera_frontend_config.h"   // Interrupt register access.
#include "acamera_interrupt_numbers.h" // IRQ numbers
#include "acamera_isp_core_settings.h"
#include "acamera_isp_ctx.h" // Need it to get context pointer.
#include "acamera_logger.h"  // For logging.

#if DEBUG_FRAME_COUNT
#include "system_timer.h" // Used to get system ticks for counters.
#endif

#if defined( ISP_HAS_MCFE_FSM )
#include "module_mcfe_common.h" // Needed for module_mcfe_get_slot_id_for_input
#endif

#ifdef CMD_QUEUE_DEBUG
#include "acamera_cmd_queues_config.h" // Microblaze only. Used for logging.
#endif

#if ISP_HAS_HISTOGRAM_FSM
#include "histogram_fsm.h" //Required for interrupt sources based on histogram tap points.
#endif


/** For MALI-C71 only. Allows the choice of using CDMA as the BE EOF event.  */
#define CDMA_AS_BE_EOF ( 1 )


/*----------  IRQ MASK FRAME START  ----------*/
/** Bit mask of sof interrupts in use */
#define IRQ_MASK_FRAME_START ( ACAMERA_FRONTEND_INTERRUPTS_ISP_START1_FIELD_MASK | \
                               ACAMERA_FRONTEND_INTERRUPTS_ISP_START2_FIELD_MASK | \
                               ACAMERA_FRONTEND_INTERRUPTS_ISP_START3_FIELD_MASK | \
                               ACAMERA_FRONTEND_INTERRUPTS_ISP_START4_FIELD_MASK | \
                               ACAMERA_FRONTEND_INTERRUPTS_ISP_OUT_FIELD_MASK )

/*----------  IRQ MASK FRAME END  ----------*/
/** Bit mask of eof interrupts in use */
#define IRQ_MASK_FRAME_END ( ACAMERA_FRONTEND_INTERRUPTS_ISP_START1_FIELD_MASK | \
                             ACAMERA_FRONTEND_INTERRUPTS_ISP_START2_FIELD_MASK | \
                             ACAMERA_FRONTEND_INTERRUPTS_ISP_START3_FIELD_MASK | \
                             ACAMERA_FRONTEND_INTERRUPTS_ISP_START4_FIELD_MASK | \
                             ACAMERA_FRONTEND_INTERRUPTS_ISP_OUT_FIELD_MASK )

/*----------  IRQ MASK STATS  ----------*/
/** Bit mask of stats interrupts in use */
#define IRQ_MASK_STATS ( ACAMERA_FRONTEND_INTERRUPTS_METERING_AWB_FIELD_MASK |    \
                         ACAMERA_FRONTEND_INTERRUPTS_ANTIFOG_HIST_FIELD_MASK |    \
                         ACAMERA_FRONTEND_INTERRUPTS_METERING_HIST_1_FIELD_MASK | \
                         ACAMERA_FRONTEND_INTERRUPTS_METERING_HIST_2_FIELD_MASK | \
                         ACAMERA_FRONTEND_INTERRUPTS_METERING_HIST_3_FIELD_MASK | \
                         ACAMERA_FRONTEND_INTERRUPTS_METERING_HIST_4_FIELD_MASK )

/*----------  MCFE ----------*/
/** Bit mask of mcfe interrupts in use */

#if ( ISP_RTL_VERSION_R >= 1 )
#define IRQ_MASK_MCFE ( ACAMERA_FRONTEND_INTERRUPTS_SLOT_1_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_2_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_3_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_4_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_5_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_6_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_7_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_8_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_9_FIELD_MASK |  \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_10_FIELD_MASK | \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_11_FIELD_MASK | \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_12_FIELD_MASK | \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_13_FIELD_MASK | \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_14_FIELD_MASK | \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_15_FIELD_MASK | \
                        ACAMERA_FRONTEND_INTERRUPTS_SLOT_16_FIELD_MASK )
#else
#define IRQ_MASK_MCFE ( 0U )
#endif


/**
 * @brief      Reads the interrupt registers.
 *
 * @return     Returns the mask of interrupts which have been triggered.
 *
 */
static inline acamera_interrupt_regs_t acamera_read_masked_status( void )
{
    acamera_interrupt_regs_t mask = {0};

    mask.sof = acamera_frontend_interrupts_frame_start_status_read( PHY_ADDR_ISP ) & acamera_frontend_interrupts_frame_start_mask_read( PHY_ADDR_ISP );
    mask.eof = acamera_frontend_interrupts_frame_end_status_read( PHY_ADDR_ISP ) & acamera_frontend_interrupts_frame_end_mask_read( PHY_ADDR_ISP );
    mask.stats = acamera_frontend_interrupts_stats_status_read( PHY_ADDR_ISP ) & acamera_frontend_interrupts_stats_mask_read( PHY_ADDR_ISP );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    mask.mcfe = acamera_frontend_interrupts_mcfe_status_read( PHY_ADDR_ISP ) & acamera_frontend_interrupts_mcfe_mask_read( PHY_ADDR_ISP );
#else
    mask.mcfe = 0;
#endif
    return mask;
}



static inline acamera_interrupt_regs_t acamera_actual_masked_status( void )
{
    acamera_interrupt_regs_t mask = {0};

    mask.sof = acamera_interrupts_frame_start_status_read( PHY_ADDR_ISP ) & acamera_interrupts_frame_start_mask_read( PHY_ADDR_ISP );
    mask.eof = acamera_interrupts_frame_end_status_read( PHY_ADDR_ISP ) & acamera_interrupts_frame_end_mask_read( PHY_ADDR_ISP );
    mask.stats = acamera_interrupts_stats_status_read( PHY_ADDR_ISP ) & acamera_interrupts_stats_mask_read( PHY_ADDR_ISP );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    mask.mcfe = acamera_interrupts_mcfe_status_read( PHY_ADDR_ISP ) & acamera_interrupts_mcfe_mask_read( PHY_ADDR_ISP );
#else
    mask.mcfe = 0;
#endif
    return mask;
}

/**
 * @brief      Reads the interrupt mask.
 *
 * @return     Interrupt mask structure
 */
static inline acamera_interrupt_regs_t acamera_read_interrupt_mask( void )
{
    acamera_interrupt_regs_t mask;
    mask.sof = acamera_frontend_interrupts_frame_start_mask_read( PHY_ADDR_ISP );
    mask.eof = acamera_frontend_interrupts_frame_end_mask_read( PHY_ADDR_ISP );
    mask.stats = acamera_frontend_interrupts_stats_mask_read( PHY_ADDR_ISP );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    mask.mcfe = acamera_frontend_interrupts_mcfe_mask_read( PHY_ADDR_ISP );
#else
    mask.mcfe = 0;
#endif
    return mask;
}

/**
 * @brief      Reads level0 registers for all sources.
 *
 * @return     Interrupt mask structure
 */
static inline acamera_interrupt_regs_t acamera_read_level0( void )
{
    acamera_interrupt_regs_t mask;
    mask.sof = acamera_frontend_interrupts_frame_start_level0_read( PHY_ADDR_ISP );
    mask.eof = acamera_frontend_interrupts_frame_end_level0_read( PHY_ADDR_ISP );
    mask.stats = acamera_frontend_interrupts_stats_level0_read( PHY_ADDR_ISP );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    mask.mcfe = acamera_frontend_interrupts_mcfe_level0_read( PHY_ADDR_ISP );
#else
    mask.mcfe = 0;
#endif
    return mask;
}

/**
 * @brief      Reads level1 registers for all sources.
 *
 * @return     Interrupt mask structure
 *
 */
static inline acamera_interrupt_regs_t acamera_read_level1( void )
{
    acamera_interrupt_regs_t mask;
    mask.sof = acamera_frontend_interrupts_frame_start_level1_read( PHY_ADDR_ISP );
    mask.eof = acamera_frontend_interrupts_frame_end_level1_read( PHY_ADDR_ISP );
    mask.stats = acamera_frontend_interrupts_stats_level1_read( PHY_ADDR_ISP );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    mask.mcfe = acamera_frontend_interrupts_mcfe_level1_read( PHY_ADDR_ISP );
#else
    mask.mcfe = 0;
#endif
    return mask;
}

/**
 * @brief      Writes to level0 registers.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_write_level0( const acamera_interrupt_regs_t mask )
{
    acamera_frontend_interrupts_frame_start_level0_write( PHY_ADDR_ISP, mask.sof );
    acamera_frontend_interrupts_frame_end_level0_write( PHY_ADDR_ISP, mask.eof );
    acamera_frontend_interrupts_stats_level0_write( PHY_ADDR_ISP, mask.stats );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    acamera_frontend_interrupts_mcfe_level0_write( PHY_ADDR_ISP, mask.mcfe );
#endif
}

/**
 * @brief      Writes to level1 registers.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_write_level1( const acamera_interrupt_regs_t mask )
{
    acamera_frontend_interrupts_frame_start_level1_write( PHY_ADDR_ISP, mask.sof );
    acamera_frontend_interrupts_frame_end_level1_write( PHY_ADDR_ISP, mask.eof );
    acamera_frontend_interrupts_stats_level1_write( PHY_ADDR_ISP, mask.stats );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    acamera_frontend_interrupts_mcfe_level1_write( PHY_ADDR_ISP, mask.mcfe );
#endif
}


/**
 * @brief      Writes to the masks registers to enable/disable interrupts.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_write_interrupt_mask( const acamera_interrupt_regs_t mask )
{
    acamera_frontend_interrupts_frame_start_mask_write( PHY_ADDR_ISP, mask.sof );
    acamera_frontend_interrupts_frame_end_mask_write( PHY_ADDR_ISP, mask.eof );
    acamera_frontend_interrupts_stats_mask_write( PHY_ADDR_ISP, mask.stats );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    acamera_frontend_interrupts_mcfe_mask_write( PHY_ADDR_ISP, mask.mcfe );
#endif
}


/**
 * @brief      Writes to the status registers to clear the interrupt.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_clear_interrupt( const acamera_interrupt_regs_t mask )
{
    acamera_frontend_interrupts_frame_start_clear_write( PHY_ADDR_ISP, mask.sof );
    acamera_frontend_interrupts_frame_end_clear_write( PHY_ADDR_ISP, mask.eof );
    acamera_frontend_interrupts_stats_clear_write( PHY_ADDR_ISP, mask.stats );
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    acamera_frontend_interrupts_mcfe_clear_write( PHY_ADDR_ISP, mask.mcfe );
#endif
}


/**
 * @brief      Enables the interrupts specified by mask without clearing existing values.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_enable_interrupts( const acamera_interrupt_regs_t mask )
{
    acamera_interrupt_regs_t current_mask = acamera_read_interrupt_mask();
    current_mask.sof |= mask.sof;
    current_mask.eof |= mask.eof;
    current_mask.stats |= mask.stats;
    current_mask.mcfe |= mask.mcfe;
    acamera_write_interrupt_mask( current_mask );
}

/**
 * @brief      Clears given interrupts from the mask, disabling them.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_disable_interrupts( const acamera_interrupt_regs_t mask )
{
    acamera_interrupt_regs_t current_mask = acamera_read_interrupt_mask();
    current_mask.sof &= ~mask.sof;
    current_mask.eof &= ~mask.eof;
    current_mask.stats &= ~mask.stats;
    current_mask.mcfe &= ~mask.mcfe;
    acamera_write_interrupt_mask( current_mask );
}

/**
 * @brief      Sets given level0 bits.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_set_level0( const acamera_interrupt_regs_t mask )
{
    acamera_interrupt_regs_t current_mask = acamera_read_level0();
    current_mask.sof |= mask.sof;
    current_mask.eof |= mask.eof;
    current_mask.stats |= mask.stats;
    current_mask.mcfe |= mask.mcfe;
    acamera_write_level0( current_mask );
}

/**
 * @brief      Clears given level0 bits.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_clear_level0( const acamera_interrupt_regs_t mask )
{
    acamera_interrupt_regs_t current_mask = acamera_read_level0();
    current_mask.sof &= ~mask.sof;
    current_mask.eof &= ~mask.eof;
    current_mask.stats &= ~mask.stats;
    current_mask.mcfe &= ~mask.mcfe;
    acamera_write_level0( current_mask );
}

/**
 * @brief      Sets given level1 bits.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_set_level1( const acamera_interrupt_regs_t mask )
{
    acamera_interrupt_regs_t current_mask = acamera_read_level1();
    current_mask.sof |= mask.sof;
    current_mask.eof |= mask.eof;
    current_mask.stats |= mask.stats;
    current_mask.mcfe |= mask.mcfe;
    acamera_write_level1( current_mask );
}

/**
 * @brief      Clears given level1 bits.
 *
 * @param[in]  mask  Interrupt mask structure
 *
 */
static inline void acamera_clear_level1( const acamera_interrupt_regs_t mask )
{
    acamera_interrupt_regs_t current_mask = acamera_read_level1();
    current_mask.sof &= ~mask.sof;
    current_mask.eof &= ~mask.eof;
    current_mask.stats &= ~mask.stats;
    current_mask.mcfe &= ~mask.mcfe;
    acamera_write_level1( current_mask );
}

/**
 * @brief      Writes to the manual trigger register.
 *
 * @param[in]  trigger  The trigger
 *
 */
static inline void acamera_write_manual_trigger( const uint32_t trigger )
{
    acamera_frontend_interrupts_interrupt_trigger_write( PHY_ADDR_ISP, trigger );
}

acamera_interrupt_regs_t acamera_interrupt_read_acknowledge( void )
{
    const acamera_interrupt_regs_t mask = acamera_read_masked_status();

    /* Here we are assuming that all triggered interrupts will be serviced.
     * This is to clear the status so that the IRQ line is not kept asserted.
     * If there is a possibility that the handler fails to service an interrupt,
     * then perhaps we need to return a mask of interrupts that have been
     * serviced. */
    acamera_clear_interrupt( mask );

    return mask;
}

uint8_t acamera_interrupt_read_current_slot( void )
{
    return acamera_frontend_mcfe_current_slot_read( PHY_ADDR_ISP );
}


#ifdef CMD_QUEUE_DEBUG
static inline void acamera_cmd_queues_debug( const size_t index )
{
    /* Since the function is marked inline, this should be optimized out when
     * CMD_QUEUE_DEBUG is not defined */
    const uint32_t dbg_num = acamera_cmd_queues_array_data_read( PHY_ADDR_ISP, index );
    acamera_cmd_queues_array_data_write( PHY_ADDR_ISP, index, dbg_num + 1 );
}

static inline void acamera_cmd_queues_debug_with_ctx( const size_t index, const size_t ctx )
{
    /* Since the function is marked inline, this should be optimized out when
     * CMD_QUEUE_DEBUG is not defined */
    const uint32_t dbg_num = acamera_cmd_queues_array_data_read( PHY_ADDR_ISP, index );
    acamera_cmd_queues_array_data_write( PHY_ADDR_ISP, index, dbg_num + 1 );
    acamera_cmd_queues_array_data_write( PHY_ADDR_ISP, ( index + 1 ), ctx );
}


static void acamera_cmd_queues_debug_mask( const size_t slot, const size_t mask )
{
    if ( BIT( ACAMERA_IRQ_BE_FRAME_END ) & mask ) {
        acamera_cmd_queues_debug_with_ctx( 2, slot );
    }
    if ( BIT( ACAMERA_IRQ_FE_FRAME_END ) & mask ) {
        acamera_cmd_queues_debug( 6 + ( slot % 4 ) );
    }
    if ( BIT( ACAMERA_IRQ_BE_FRAME_START ) & mask ) {
        acamera_cmd_queues_debug_with_ctx( 10, slot );
    }
    if ( BIT( ACAMERA_IRQ_FE_FRAME_START ) & mask ) {
        acamera_cmd_queues_debug( 12 + ( slot % 4 ) );
    }
}

#endif


#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1

/**
 * @brief      Determines if power of two.
 *
 * @param[in]  mask  The mask
 *
 * @return     True if power of two, False otherwise.
 *
 * @details    Used to check if any MCFE interrupts have been missed. If so,
 *             something seriously wrong is going on with ISR handling.
 */
static int is_power_of_two( const uint32_t mask )
{
    return ( mask && !( mask & ( mask - 1 ) ) ) ? 1 : 0;
}
#endif /* ISP_RTL_VERSION_R */


#if DEBUG_FRAME_COUNT
static void count_frame_end( const uint32_t slot )
{
    static size_t last_stamps[ISP_MCFE_MAX_SLOT] = {0};
    static size_t counters[ISP_MCFE_MAX_SLOT] = {0};
    const size_t fps = 30;


    if ( slot >= ISP_MCFE_MAX_SLOT ) return;

    if ( ( ++counters[slot] % fps ) == 0 ) {
        const size_t this_stamp = system_timer_timestamp();
        const size_t elapsed_stamp = this_stamp - last_stamps[slot];
        const size_t elapsed_ms = ( elapsed_stamp * 1000 ) / system_timer_frequency();
        const size_t current_fps = ( fps * system_timer_frequency() ) / elapsed_stamp;
        const size_t average_frame_time = elapsed_ms / fps;

        last_stamps[slot] = this_stamp;

        LOG( LOG_NOTICE, "frame end irq, slot %d, count %zu, ftime %zums, fps: %zu",
             slot,
             counters[slot],
             average_frame_time,
             current_fps );
    }
}
#endif

/**
 *
 * @details    This function is responsible for mapping between hard interrupt
 *             bits as set in the registers and logical IRQs (#acamera_irq_t).
 *             The function determines what IRQs to set to which context. There
 *             are a few possibilities:
 *             - Hardcoded slot id in the register, example includes
 *               #ACAMERA_FRONTEND_INTERRUPTS_ISP_START1_FIELD_MASK. These interrupts may
 *               need to be remapped with #module_mcfe_get_slot_id_for_input
 *             - Current slot id. The current active slot can be read with
 *               #acamera_frontend_mcfe_current_slot_read. These interrupts
 *               correspond to the backend.
 *             - Last slot id, because of timing when we detect a new frame
 *               enters the backend we store that information in the
 *               #last_isp_slot variable so that end of frame interrupts for
 *               that context can be processed without worrying about current
 *               active slot.
 *
 *             Here is an example on how to map an interrupt should you wish to
 *             add more functionality:
 *
 * @code
 * void acamera_interrupt_handler( void *param )
 * {
 *      const acamera_interrupt_regs_t mask = ( *( (isr_data_t *)param ) ).regs;
 *      const uint8_t curr_slot = ( *( (isr_data_t *)param ) ).slot;
 *      static uint8_t last_isp_slot = ISP_MCFE_MAX_SLOT;
 *
 *      uint32_t irq_masks[ISP_MCFE_MAX_SLOT] = {0};
 *
 *      if ( mask.<register> & DESIRED_BIT ) {
 *          irq_masks[<desired_slot>] |= BIT( <IRQ_NUMBER> );
 *      }
 * }
 * @endcode
 * @details    The IRQ bitmask together with slot id will then be passed to
 *             #acamera_process_interrupt
 */
void acamera_interrupt_handler( void *param )
{
    assert( param ); // This function should *not* be called with NULL parameters.

    /* Extract IRQ information, these are the inputs to the handler. */
    const acamera_interrupt_regs_t mask = ( (isr_data_t *)param )->regs;
    const uint8_t curr_slot = ( (isr_data_t *)param )->slot;

    static uint8_t last_isp_slot = ISP_MCFE_MAX_SLOT;
    uint32_t irq_masks[ISP_MCFE_MAX_SLOT] = {0}; // Stores the IRQ mask for each of the slots. This should be
                                                 // relatively low, since max slot number is 16 so stack size
                                                 // shouldn't be a problem.


#ifdef CMD_QUEUE_DEBUG
    static int cnt = 0;
    acamera_cmd_queues_array_data_write( PHY_ADDR_ISP, 0, ++cnt );
    acamera_cmd_queues_debug( 1 );
#endif

#if 0
    LOG(LOG_INFO,"mask stats %x, mcfe %x, eof %x, sof %x , scheduler status %d, curr_slot %d",
            mask.stats, mask.mcfe, mask.eof, mask.sof, 
	    acamera_frontend_mcfe_scheduler_status_read(PHY_ADDR_ISP),
	    curr_slot);
#endif

/*
 * ########### BACK END : FRAME END ###########
 *
 *  - For MALI_C71 if CDMA_AS_BE_EOF is set we use the MCFE slot done interrupt
 *    to trigger the back end-end of frame. We check which slot is set in the
 *    MCFE register and set the #ACAMERA_IRQ_BE_FRAME_END IRQ for that slot.
 *  - For MALI_C71 if CDMA_AS_BE_EOF is not set and for non MALI_C71 builds we
 *    use the #ACAMERA_FRONTEND_INTERRUPTS_ISP_OUT_FIELD_MASK in the EoF register.
 *    We rely on the logical flow of frames and make the assumption that the
 *    last read active slot to trigger SoF will also be the slot id to trigger
 *    EoF.
 */
#if ( ISP_RTL_VERSION_R >= 1 ) //TODO: Figure out the behaviour for R2, for now assume same as R1
    if ( CDMA_AS_BE_EOF ) {
        // ----------- ISP MCFE CDMA out ------------
        if ( mask.mcfe & IRQ_MASK_MCFE ) {
            // Shift the mask so that the first bit corresponds to SLOT1 field.
            const uint32_t mcfe_slot_mask = mask.mcfe >> ACAMERA_FRONTEND_INTERRUPTS_SLOT_1_FIELD_OFFSET;

            if ( !is_power_of_two( mcfe_slot_mask ) ) {
                LOG( LOG_ERR, "More than one MCFE slot done, failed to proccess previous slot (MCFE:0x%x).", mcfe_slot_mask );
            }

            uint8_t slot;
            /* coverity[same_on_both_sides] */
            for ( slot = 0; slot < MIN( ISP_MCFE_MAX_SLOT, FIRMWARE_CONTEXT_NUMBER ); ++slot ) {
                if ( mcfe_slot_mask & ( 1u << slot ) ) {
                    if ( slot != last_isp_slot ) {
                        LOG( LOG_ERR, "FS slot %u doesn't match CDMA out slot %u!!!", last_isp_slot, slot );
                    }
                    // No need for range check, it has to be correct due to
                    // loop constraint.
                    irq_masks[slot] |= BIT( ACAMERA_IRQ_BE_FRAME_END );
#if DEBUG_FRAME_COUNT
                    count_frame_end( slot );
#endif
                }
            }
        }
    } else
#endif /* ISP_RTL_VERSION_R */
    {
        // ----------- ISP FRAME END --------------
        if ( mask.eof & ACAMERA_FRONTEND_INTERRUPTS_ISP_OUT_FIELD_MASK ) {
            // Skip processing when prev_slot is not valid.
            if ( last_isp_slot < ISP_MCFE_MAX_SLOT ) {
                irq_masks[last_isp_slot] |= BIT( ACAMERA_IRQ_BE_FRAME_END );
            } else {
                LOG( LOG_ERR, "Frame end received before frame start on first processing!!!" );
            }
        }
    }


    /*
     * ########### FRONT END: FRAME START/END ###########
     *
     * Loop over #ISP_MCFE_MAX_INPUT slots corresponding to the register bits and
     * figure out the interrupt mask for SoF and EoF (shared bit layout). Since
     * this is a front end hard slot id it may be remapped in the  firmware. We
     * can look up the correct slot id with #module_mcfe_get_slot_id_for_input
     *
     * 1. If EoF is set for the slot then trigger #ACAMERA_IRQ_FE_FRAME_END.
     * 2. If SoF is set for the slot then trigger #ACAMERA_IRQ_FE_FRAME_START
     */
    {
        uint8_t slot;
        for ( slot = 0; slot < ISP_MCFE_MAX_INPUT; ++slot ) {
#if defined( ISP_HAS_MCFE_FSM )
            const int remapped_slot_id = module_mcfe_get_slot_id_for_input( slot, MODULE_MCFE_SLOT_INPUT_FIRST_ONLY );
            // If this is -1 then it has not been mapped. If it is larger than
            // ISP_MCFE_MAX_SLOT then something is horribly wrong.
            if ( remapped_slot_id < 0 || remapped_slot_id >= ISP_MCFE_MAX_SLOT ) continue;
#endif

            /* SOF and EOF share the same bitmask so we can calculate it once. */
            const uint16_t sofeof_mask = ACAMERA_FRONTEND_INTERRUPTS_ISP_START1_FIELD_MASK << slot;
            uint32_t irq_mask = 0; // Stores IRQ events for this slot.

            if ( mask.eof & sofeof_mask )
                irq_mask |= BIT( ACAMERA_IRQ_FE_FRAME_END );

            if ( mask.sof & sofeof_mask ) {
                irq_mask |= BIT( ACAMERA_IRQ_FE_FRAME_START );

#if !ISP_HISTOGRAM_POSITION_IS_BE
                if ( last_isp_slot < ISP_MCFE_MAX_SLOT ) irq_masks[last_isp_slot] |= BIT( ACAMERA_IRQ_AE_STATS );
#endif
            }

            // By this point we know the index is correct so we do not need to check.

#if defined( ISP_HAS_MCFE_FSM )
            irq_masks[remapped_slot_id] |= irq_mask;
#else
            irq_masks[slot] |= irq_mask;
#endif
        }
    }

    /*
     * ########### BACK END : FRAME START ###########
     * 1. Frame enters back end, trigger #ACAMERA_IRQ_BE_FRAME_START for the new frame slot id.
     * 2. By *implication* the stats for previous slot id must be ready so trigger stat IRQs.
     */
    if ( mask.sof & ACAMERA_FRONTEND_INTERRUPTS_ISP_OUT_FIELD_MASK ) {
        if ( last_isp_slot < ISP_MCFE_MAX_SLOT ) {
            irq_masks[last_isp_slot] |= BIT( ACAMERA_IRQ_ANTIFOG_HIST ) |
                                        BIT( ACAMERA_IRQ_AWB_STATS ) |
                                        BIT( ACAMERA_IRQ_AE_METERING_STATS );

#if ISP_HISTOGRAM_POSITION_IS_BE
            irq_masks[last_isp_slot] |= BIT( ACAMERA_IRQ_AE_STATS );
#endif
        }

	//LOG(LOG_INFO,"Bit set ACAMERA_IRQ_BE_FRAME_START");
        irq_masks[curr_slot] |= BIT( ACAMERA_IRQ_BE_FRAME_START );
        last_isp_slot = curr_slot;

	//LOG(LOG_INFO,"last_isp_slot is %d", last_isp_slot);
    }

    /*
     * ########### Per context logical IRQ handler ###########
     */
    {
        uint8_t slot;
        for ( slot = 0; slot < ISP_MCFE_MAX_SLOT; ++slot ) {
            if ( irq_masks[slot] )
                acamera_process_interrupt( slot, irq_masks[slot] );
        }
    }
}


void acamera_interrupt_init( void )
{
    const acamera_interrupt_regs_t mask = {.sof = IRQ_MASK_FRAME_START,
                                           .eof = IRQ_MASK_FRAME_END,
                                           .stats = IRQ_MASK_STATS,
                                           .mcfe = IRQ_MASK_MCFE};
    /* Ensure all interrupts are off. */
    acamera_interrupt_disable();
    acamera_write_manual_trigger( 0 );

    acamera_clear_interrupt( mask );

    /* Set up register values. */
    acamera_clear_level0( mask );
    acamera_clear_level1( mask );

	// TODO: Temporary fix to avoid ioremapping in interrupt context
	// Find a better way to do ioremap before processing interrupts
#if 1
    acamera_interrupt_read_current_slot();
    acamera_interrupt_read_acknowledge();
#endif    
}


void acamera_interrupt_enable( void )
{
    const acamera_interrupt_regs_t mask = {.sof = IRQ_MASK_FRAME_START,
                                           .eof = IRQ_MASK_FRAME_END,
                                           .stats = IRQ_MASK_STATS,
                                           .mcfe = IRQ_MASK_MCFE};
    acamera_enable_interrupts( mask );
}


void acamera_interrupt_disable( void )
{
    const acamera_interrupt_regs_t mask = {.sof = -1,
                                           .eof = -1,
                                           .stats = -1,
                                           .mcfe = -1};
    acamera_disable_interrupts( mask );
}
void acamera_interrupt_deinit( void )
{
    const acamera_interrupt_regs_t mask = {.sof = -1,
                                           .eof = -1,
                                           .stats = -1,
                                           .mcfe = -1};
    acamera_interrupt_disable();
    acamera_clear_interrupt( mask );
}

/** @} */
