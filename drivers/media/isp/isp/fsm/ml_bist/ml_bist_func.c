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

#include "system_isp_io.h"
#include "acamera_faults_cfg_config.h"
#include "acamera_frontend_config.h"
#include "acamera_isp_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "bitop.h" //Bit Field macros.

typedef enum {
    BLOCK_OPERATION_CHECK_FAULTS = 0,
    BLOCK_OPERATION_CLEAR_FAULTS
} block_operation_t;

// ******************* Module Level BIST(Built-In Self-Test) ***********************
typedef enum {
    RAW_FRONTEND = 0,
    FRAME_STITCH,
    SINTER,
    RADIAL_SHADING,
    IRIDIX,
    DEMOSAIC,
    /* DEMOSAIC_RGBIR and DEMOSAIC_RCCC might be available or not, depending on hw version */
    DEMOSAIC_RGBIR,
    DEMOSAIC_RCCC,
    OUTFORMAT,
    /* SCALER_RAW,SCALER_RGB and CNR are valid only for moss */
    SCALER_RAW,
    SCALER_RGB,
    CNR,
    TEST_BLOCK_MAX
} test_block_t;


/** tests ran by this FSM in sequence. */
typedef enum {
    ML_BIST_TEST0 = 0,
    ML_BIST_TEST1,
    ML_BIST_TEST2,
    ML_BIST_TEST3,
    ML_BIST_TEST4,
    ML_BIST_TEST5,
    ML_BIST_TEST6,
    ML_BIST_TEST7,
    ML_BIST_TEST_MAX,
} ml_bist_test_t;

static void ml_bist_clear_faults( const ml_bist_test_t current_test_number );

/* for r0 and two r1 hw revisions that use r0 bist hardware */
#if ( ISP_RTL_VERSION_R < 1 ) || ( HW_REVISION == 83008 ) || ( HW_REVISION == 91883 )
static void ml_bist_reset_crc_faults( const test_block_t isp_block );
static void ml_bist_check_for_faults( const ml_bist_test_t current_test_number );
#endif

/* r1 and above, except these two r1 hw revisions that use r0 bist hardware */
#if ( ISP_RTL_VERSION_R >= 1 ) && ( HW_REVISION != 83008 ) && ( HW_REVISION != 91883 )
static void ml_bist_process_blocks( const ml_bist_test_t current_test_number, const block_operation_t operation );
#endif

void ml_bist_fsm_process_interrupt( ml_bist_fsm_const_ptr_t p_fsm, uint8_t irq_event )
{
    switch ( irq_event ) {
    case ACAMERA_IRQ_BE_FRAME_END:
        ml_bist_frame_end( (ml_bist_fsm_ptr_t)p_fsm );
        break;
    }
}

void ml_bist_init( ml_bist_fsm_ptr_t p_fsm )
{

    // Called once at boot up
    p_fsm->repeat_irq_mask = BIT( ACAMERA_IRQ_BE_FRAME_END );

    ml_bist_clear_faults( ML_BIST_TEST0 );

    // Finally request interrupts
    ml_bist_request_interrupt( p_fsm, p_fsm->repeat_irq_mask );
}

void ml_bist_frame_end( ml_bist_fsm_ptr_t p_fsm )
{
    /* ISP blocks to configure.
	 * 1. Raw Frontend
	 * 2. Frame Stitch
	 * 3. Sinter
	 * 4. Iridix
	 * 5. Gamma FeDI (Ignored)
	 * 6. Gamma BeDI (Ignored)
	 * 7. Radial Shading
	 * 8. Demosaic
	 * 9. Demosaic RGBIr
	 * 10. Demosaic RGCCC
	 * 11. Out Format
     * 12. Scaler Raw (for Moss)
     * 13. Scaler RGB (for Moss)
     * 14. CNR (for Moss)
     */
    /** Helper structure to manage test mode data. Each test is a bitfield where each bit
     * corresponds to a test. To enable test 0 for exampel set bit 0 etc. */
    struct
    {
        uint8_t sinter;
        uint16_t frame_stitch;
        uint8_t raw_frontend;
        uint8_t iridix;
        uint8_t radial_shading;
        uint8_t out_format;
        uint8_t demosaic_rccc;
        uint8_t demosaic_rgbir;
        uint8_t demosaic;
        uint8_t scaler_raw;
        uint8_t scaler_rgb;
        uint8_t cnr;
    } test_mode_data = {0};

    const uint32_t base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    /* we only force crc bist mismatch error if the user enables it via ACT api
     * NOTE 1: Remember to disable it again
     * NOTE 2: Bypassed modules will not generate a crc mismatch
     */
    if ( get_context_param( p_ictx, ISP_MODULES_FORCE_BIST_MISMATCH_PARAM ) != 0 ) {
        LOG( LOG_ERR, " ############### FORCING BIST CRC MISMATCH  ############## " );
        test_mode_data.sinter = BIT( 0 );
        test_mode_data.frame_stitch = BIT( 0 );
        test_mode_data.raw_frontend = BIT( 0 );
        test_mode_data.iridix = BIT( 0 );
        test_mode_data.radial_shading = BIT( 0 );
        test_mode_data.out_format = BIT( 0 );
        test_mode_data.demosaic_rccc = BIT( 0 );
        test_mode_data.demosaic_rgbir = BIT( 0 );
        test_mode_data.demosaic = BIT( 0 );
        test_mode_data.scaler_raw = BIT( 0 );
        test_mode_data.scaler_rgb = BIT( 0 );
        test_mode_data.cnr = BIT( 0 );
    } else {
        /* each batch of tests runs on vblank period, so the total time must not exceed it
        * keep in mind that between the modules the tests run in parallel.
        * for any given module, it will run in series only if we enable more than one test (bit)
        */
        switch ( p_fsm->current_test_number ) {
        case ML_BIST_TEST0:                           // Highest time: 20977 cycles
            test_mode_data.sinter = BIT( 3 );         // Test bit 3: 10392 cycles
            test_mode_data.frame_stitch = BIT( 3 );   // Test bit 3: 19343 cycles
            test_mode_data.raw_frontend = BIT( 2 );   // Test bit 2: 19466 cycles
            test_mode_data.radial_shading = BIT( 3 ); // Test bit 3: 9426 cycles
            test_mode_data.out_format = BIT( 3 );     // Test bit 3: 18162 cycles
            test_mode_data.demosaic_rccc = BIT( 3 );  // Test bit 3: 16289 cycles
            test_mode_data.demosaic_rgbir = BIT( 3 ); // Test bit 3: 15301 cycles
            test_mode_data.demosaic = BIT( 3 );       // Test bit 3: 10254 cycles
            test_mode_data.scaler_raw = BIT( 2 );     //Test bit 2: 20977 cycles
            test_mode_data.scaler_rgb = BIT( 2 );     //Test bit 2: 16909 cycles
            test_mode_data.cnr = BIT( 2 );            //Test bit 2: 5359 cycles
            break;
        case ML_BIST_TEST1:                       // Highest time: 29169 cycles
            test_mode_data.iridix = BIT( 2 );     // Test bit 2: 29169 cycles
            test_mode_data.scaler_raw = BIT( 3 ); //Test bit 3: 25878 cycles
            test_mode_data.scaler_rgb = BIT( 3 ); //Test bit 3: 25852 cycles
            test_mode_data.cnr = BIT( 3 );        //Test bit 3: 17791 cycles
            break;
        case ML_BIST_TEST2:                           // Highest time: 21055 cycles
            test_mode_data.sinter = BIT( 4 );         // Test bit 4: 12786 cycles
            test_mode_data.frame_stitch = BIT( 4 );   // Test bit 4: 11951 cycles
            test_mode_data.raw_frontend = BIT( 2 );   // Test bit 2: 19466 cycles
            test_mode_data.radial_shading = BIT( 4 ); // Test bit 4: 6418 cycles
            test_mode_data.out_format = BIT( 4 );     // Test bit 4: 11148 cycles
            test_mode_data.demosaic_rccc = BIT( 4 );  // Test bit 4: 8769 cycles
            test_mode_data.demosaic_rgbir = BIT( 4 ); // Test bit 4: 16493 cycles
            test_mode_data.demosaic = BIT( 4 );       // Test bit 4: 6766 cycles
            test_mode_data.scaler_raw = BIT( 4 );     //Test bit 4: 10050 cycles
            test_mode_data.scaler_rgb = BIT( 4 );     //Test bit 4: 9485 cycles
            test_mode_data.cnr = BIT( 4 );            //Test bit 4: 21055 cycles
            break;
        case ML_BIST_TEST3:                       // Highest time: 15832 cycles
            test_mode_data.iridix = BIT( 3 );     //Test bit 3: 15832 cycles
            test_mode_data.scaler_raw = BIT( 5 ); //Test bit 5: 5599 cycles
            test_mode_data.scaler_rgb = BIT( 5 ); //Test bit 5: 5310 cycles
            test_mode_data.cnr = BIT( 5 );        //Test bit 5: 5176 cycles
            break;
        case ML_BIST_TEST4:                                    // Highest time: 31039 cycles
            test_mode_data.sinter = BIT( 5 );                  // Test bit 5: 9235 cycles
            test_mode_data.frame_stitch = BIT( 5 ) | BIT( 2 ); // Test bits 5 and 2: 10640+20399 =  31039 cycles
            test_mode_data.raw_frontend = BIT( 5 );            // Test bit 5: 9731 cycles
            test_mode_data.radial_shading = BIT( 5 );          // Test bit 5: 2739 cycles
            test_mode_data.out_format = BIT( 5 );              // Test bit 5: 10878 cycles
            test_mode_data.demosaic_rccc = BIT( 5 );           // Test bit 5: 10594 cycles
            test_mode_data.demosaic_rgbir = BIT( 5 );          // Test bit 5: 11634 cycles
            test_mode_data.demosaic = BIT( 5 );                // Test bit 5: 3247 cycles
            test_mode_data.scaler_raw = BIT( 2 );              //Test bit 2: 20977 cycles
            test_mode_data.scaler_rgb = BIT( 2 );              //Test bit 2: 16909 cycles
            test_mode_data.cnr = BIT( 2 );                     //Test bit 2: 5359 cycles
            break;
        case ML_BIST_TEST5:                       // Highest time: 32495 cycles
            test_mode_data.iridix = BIT( 4 );     // Test bit 4: 32495 cycles
            test_mode_data.scaler_raw = BIT( 3 ); //Test bit 3: 25878 cycles
            test_mode_data.scaler_rgb = BIT( 3 ); //Test bit 3: 25852 cycles
            test_mode_data.cnr = BIT( 3 );        //Test bit 3: 17791 cycles
            break;
        case ML_BIST_TEST6:                   // Highest time: 33792 cycles
            test_mode_data.sinter = BIT( 2 ); // Test bit 2: 18760 cycles
            /* frame_stitch: bits 6,7 and 8 work for r0. There, BIST for frame_stitch is extended */
            test_mode_data.frame_stitch = BIT( 6 ) | BIT( 7 ) | BIT( 8 ); // Test bits 6 and 7 and 8: 9216 + 9216 + 15360 = 33792 cycles
            test_mode_data.raw_frontend = BIT( 3 ) | BIT( 4 );            // Test bits 3 and 4: 18146 + 9730 = 27876 cycles
            test_mode_data.radial_shading = BIT( 2 );                     // Test bit 2: 19058 cycles
            test_mode_data.out_format = BIT( 2 );                         // Test bit 2: 19093 cycles
            test_mode_data.demosaic_rccc = BIT( 2 );                      // Test bit 2: 16289 cycles
            test_mode_data.demosaic_rgbir = BIT( 2 );                     // Test bit 2: 14977 cycles
            test_mode_data.demosaic = BIT( 2 );                           // Test bit 2: 19914 cycles
            test_mode_data.scaler_raw = BIT( 4 );                         //Test bit 4: 10050 cycles
            test_mode_data.scaler_rgb = BIT( 4 );                         //Test bit 4: 9485 cycles
            test_mode_data.cnr = BIT( 4 );                                //Test bit 4: 21055 cycles
            break;
        case ML_BIST_TEST7:                       // Highest time: 16883 cycles
            test_mode_data.iridix = BIT( 5 );     // Test Bit 5 = 16883 cycles
            test_mode_data.scaler_raw = BIT( 5 ); //Test bit 5: 5599 cycles
            test_mode_data.scaler_rgb = BIT( 5 ); //Test bit 5: 5310 cycles
            test_mode_data.cnr = BIT( 5 );        //Test bit 5: 5176 cycles
            break;
        default:
            /*Invalid test number. Reset and return. */
            p_fsm->current_test_number = ML_BIST_TEST0;
            return;
        }
    }

/* for r0 and two r1 hw revisions that also use r0 bist hardware */
#if ( ISP_RTL_VERSION_R < 1 ) || ( HW_REVISION == 83008 ) || ( HW_REVISION == 91883 )
    ml_bist_check_for_faults( p_fsm->current_test_number );
#else
    ml_bist_process_blocks( p_fsm->current_test_number, BLOCK_OPERATION_CHECK_FAULTS );
#endif

    ml_bist_clear_faults( p_fsm->current_test_number );

    acamera_isp_test_mode_raw_frontend_write( base, test_mode_data.raw_frontend );
    acamera_isp_test_mode_frame_stitch_write( base, test_mode_data.frame_stitch );
    acamera_isp_test_mode_sinter_write( base, test_mode_data.sinter );
    acamera_isp_test_mode_radial_shading_write( base, test_mode_data.radial_shading );
    acamera_isp_test_mode_iridix_write( base, test_mode_data.iridix );
    acamera_isp_test_mode_demosaic_write( base, test_mode_data.demosaic );
#ifdef ACAMERA_ISP_TEST_MODE_DEMOSAIC_RGBIR_DEFAULT
    acamera_isp_test_mode_demosaic_rgbir_write( base, test_mode_data.demosaic_rgbir );
#endif
#ifdef ACAMERA_ISP_TEST_MODE_DEMOSAIC_RCCC_DEFAULT
    acamera_isp_test_mode_demosaic_rccc_write( base, test_mode_data.demosaic_rccc );
#endif
    acamera_isp_test_mode_out_format_write( base, test_mode_data.out_format );
/* those are valid only for moss */
#if ( ISP_RTL_VERSION_R == 2 )
    acamera_isp_test_mode_scaler_raw_write( base, test_mode_data.scaler_raw );
    acamera_isp_test_mode_scaler_rgb_write( base, test_mode_data.scaler_rgb );
    acamera_isp_test_mode_cnr_write( base, test_mode_data.cnr );
#endif

    if ( ( ++p_fsm->current_test_number ) >= ML_BIST_TEST_MAX )
        p_fsm->current_test_number = ML_BIST_TEST0;
}

static void ml_bist_clear_faults( const ml_bist_test_t current_test_number )
{
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_in_crc1_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_in_crc2_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_in_crc3_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_in_crc4_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_out_crc1_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_out_crc2_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_out_crc3_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_fifo_out_crc4_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcfe_error_buffer_config_config_crc_mismatch_write( PHY_ADDR_ISP, 1 );

    acamera_faults_cfg_faults_cfg_status_mcbe_error_fifo_in_crc1_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcbe_error_fifo_in_crc2_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcbe_error_fifo_in_crc3_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcbe_error_fifo_in_crc4_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_mcbe_error_buffer_config_config_crc_mismatch_write( PHY_ADDR_ISP, 1 );

#if ( ISP_RTL_VERSION_R >= 1 )
    acamera_faults_cfg_faults_cfg_status_exp_fifo_error_fifo_in_crc_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
#endif

/* for r0 and two r1 hw revisions that also use r0 bist hardware */
#if ( ISP_RTL_VERSION_R < 1 ) || ( HW_REVISION == 83008 ) || ( HW_REVISION == 91883 )

    ml_bist_reset_crc_faults( RAW_FRONTEND );
    ml_bist_reset_crc_faults( FRAME_STITCH );
    ml_bist_reset_crc_faults( SINTER );
    ml_bist_reset_crc_faults( RADIAL_SHADING );
    ml_bist_reset_crc_faults( IRIDIX );
    ml_bist_reset_crc_faults( DEMOSAIC );
    ml_bist_reset_crc_faults( DEMOSAIC_RGBIR );
    ml_bist_reset_crc_faults( DEMOSAIC_RCCC );
    ml_bist_reset_crc_faults( OUTFORMAT );
#else
    ml_bist_process_blocks( current_test_number, BLOCK_OPERATION_CLEAR_FAULTS );
#endif

    acamera_faults_cfg_faults_cfg_status_crc_check_1_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_crc_check_2_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_crc_check_3_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_crc_check_a1_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_crc_check_a2_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_crc_check_a3_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
    acamera_faults_cfg_faults_cfg_status_pc_fifo_error_fifo_in_crc2_fifo_crc_mismatch_write( PHY_ADDR_ISP, 1 );
}

/* these specific HW_REVISION 83008 and 91883 use r0 BIST structure, so must use the alternative
 * ml_bist_reset_crc_faults and ml_bist_check_for_faults
 */
#if ( ISP_RTL_VERSION_R >= 1 ) && ( HW_REVISION != 83008 ) && ( HW_REVISION != 91883 )

#define TEST_ERROR_MAP( storage, bit_position ) ( ( storage & ( 1 << bit_position ) ) != 0 )

#define CLEAR_BLOCK_FAULT( name, type )                                                  \
    {                                                                                    \
        acamera_faults_cfg_faults_cfg_status_##name##_##type##_write( PHY_ADDR_ISP, 1 ); \
    }

#define CLEAR_ALL_BLOCK_FAULTS( name )                                 \
    {                                                                  \
        CLEAR_BLOCK_FAULT( name, crc_data_bist_o_fault_crc_mismatch )  \
        CLEAR_BLOCK_FAULT( name, bist_sequencer_error_test_timeout )   \
        CLEAR_BLOCK_FAULT( name, bist_sequencer_error_test_collision ) \
    }

#define CHECK_BLOCK_FAULT( block_name, fault_type, error_storage, error_bit_position )                 \
    {                                                                                                  \
        uint32_t err;                                                                                  \
        err = acamera_faults_cfg_faults_cfg_status_##block_name##_##fault_type##_read( PHY_ADDR_ISP ); \
        if ( err )                                                                                     \
            error_storage |= 1 << error_bit_position;                                                  \
    }

#define CHECK_ALL_BLOCK_FAULTS( block_name, error_storage, error_bit_position )                                                         \
    {                                                                                                                                   \
        CHECK_BLOCK_FAULT( block_name, crc_data_bist_o_fault_crc_mismatch, error_storage.test_crc_mismatch_error, error_bit_position ); \
        CHECK_BLOCK_FAULT( block_name, bist_sequencer_error_test_timeout, error_storage.test_timeout_error, error_bit_position );       \
        CHECK_BLOCK_FAULT( block_name, bist_sequencer_error_test_collision, error_storage.test_collision_error, error_bit_position );   \
    }

#define PROCESS_BLOCK_OPERATION( operation, block_name, error_storage, error_bit_position ) \
    {                                                                                       \
        switch ( operation ) {                                                              \
        case BLOCK_OPERATION_CHECK_FAULTS:                                                  \
            CHECK_ALL_BLOCK_FAULTS( block_name, error_storage, error_bit_position );        \
            break;                                                                          \
        case BLOCK_OPERATION_CLEAR_FAULTS:                                                  \
            CLEAR_ALL_BLOCK_FAULTS( block_name );                                           \
            break;                                                                          \
        }                                                                                   \
    }

static void ml_bist_process_blocks( const ml_bist_test_t current_test_number, const block_operation_t operation )
{
    /*
    each block defined in test_block_t is mapped into a bit for each test (crc mismatch, timeout, collision)
    in case a test fails, the respective bit will be set by the macros above.
    */
    static struct {
        uint32_t test_crc_mismatch_error;
        uint32_t test_timeout_error;
        uint32_t test_collision_error;
    } error_map;
    error_map.test_crc_mismatch_error = 0;
    error_map.test_timeout_error = 0;
    error_map.test_collision_error = 0;
    static test_block_t block;
    for ( block = 0; block <= TEST_BLOCK_MAX; block++ ) {
        switch ( block ) {
        case RAW_FRONTEND:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_RAW_FRONTEND_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, raw_frontend, error_map, RAW_FRONTEND );
#endif
            break;
        case FRAME_STITCH:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_FRAME_STITCH_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, frame_stitch, error_map, FRAME_STITCH );
#endif
            break;
        case SINTER:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_SINTER_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, sinter, error_map, SINTER );
#endif
            break;
        case RADIAL_SHADING:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_RADIAL_SHADING_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, radial_shading, error_map, RADIAL_SHADING );
#endif
            break;
        case IRIDIX:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_IRIDIX_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, iridix, error_map, IRIDIX );
#endif
            break;
        case DEMOSAIC:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_DEMOSAIC_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, demosaic, error_map, DEMOSAIC );
#endif
            break;
        case DEMOSAIC_RGBIR:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_DEMOSAIC_RGBIR_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, demosaic_rgbir, error_map, DEMOSAIC_RGBIR );
#endif
            break;
        case DEMOSAIC_RCCC:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_DEMOSAIC_RCCC_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, demosaic_rccc, error_map, DEMOSAIC_RCCC );
#endif
            break;
        case OUTFORMAT:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_OUT_FORMAT_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, out_format, error_map, OUTFORMAT );
#endif
            break;
        case SCALER_RAW:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_SCALER_RAW_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, scaler_raw, error_map, SCALER_RAW );
#endif
            break;
        case SCALER_RGB:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_SCALER_RGB_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, scaler_rgb, error_map, SCALER_RGB );
#endif
            break;
        case CNR:
#ifdef ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_CNR_CRC_DATA_BIST_O_FAULT_CRC_MISMATCH_DEFAULT
            PROCESS_BLOCK_OPERATION( operation, cnr, error_map, CNR );
#endif
            break;
        default:
            break;
        }
    }

    for ( block = 0; block < TEST_BLOCK_MAX; block++ ) {
        uint8_t errors = 0;
        if ( TEST_ERROR_MAP( error_map.test_collision_error, block ) )
            errors = 0x01;
        if ( TEST_ERROR_MAP( error_map.test_timeout_error, block ) )
            errors |= 0x02;
        if ( TEST_ERROR_MAP( error_map.test_crc_mismatch_error, block ) )
            errors |= 0x04;
        if ( errors > 0 )
            LOG( LOG_ERR, "ML_BIST fail collision=%d timeout=%d crc_mismatch=%d block %d test %d ", errors & 0x01, ( errors & 0x02 ) >> 1, ( errors & 0x04 ) >> 2, block, current_test_number );
    }
}

#else

/**
 * @brief      Resets fauls for given block. Write 1 to reset it
 *
 * @param[in]  isp_block  The isp block
 *
 */
static void ml_bist_reset_crc_faults( const test_block_t isp_block )
{
    switch ( isp_block ) {
    case RAW_FRONTEND:
        acamera_faults_cfg_faults_cfg_status_raw_frontend_crc_data_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_raw_frontend_crc_hp_median_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_raw_frontend_crc_hotpixel_marker_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    case FRAME_STITCH:
        acamera_faults_cfg_faults_cfg_status_frame_stitch_crc_data_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
#if ( ISP_RTL_VERSION_R >= 1 ) //for HW_REVISION  83008 and 91883
        acamera_faults_cfg_faults_cfg_status_frame_stitch_crc_exp_mask_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
#endif
        break;
    case SINTER:
        acamera_faults_cfg_faults_cfg_status_sinter_crc_vdata_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    case RADIAL_SHADING:
        acamera_faults_cfg_faults_cfg_status_radial_shading_crc_data_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    case IRIDIX:
        acamera_faults_cfg_faults_cfg_status_iridix_crc_data_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    case DEMOSAIC:
        acamera_faults_cfg_faults_cfg_status_demosaic_crc_red_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_crc_grn_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_crc_blu_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_crc_mask_sad_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    case DEMOSAIC_RGBIR:
#if ( ISP_RTL_VERSION_R >= 1 ) //for HW_REVISION  83008 and 91883
        acamera_faults_cfg_faults_cfg_status_demosaic_rgbir_crc_irmask_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
#endif
        acamera_faults_cfg_faults_cfg_status_demosaic_rgbir_crc_rplane_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_rgbir_crc_gplane_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_rgbir_crc_bplane_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_rgbir_crc_iplane_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_rgbir_crc_declip_data_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    case DEMOSAIC_RCCC:
        acamera_faults_cfg_faults_cfg_status_demosaic_rccc_crc_vdata1_rc_r_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_demosaic_rccc_crc_vdata2_rc_c_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    case OUTFORMAT:
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata1_rgb_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata2_rgb_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata3_rgb_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata1_cm_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata2_cm_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata3_cm_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata1_ir_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata2_yuv_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata3_yuv_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
#if ( ISP_RTL_VERSION_R >= 1 ) //for HW_REVISION  83008 and 91883
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata1_yuv_y_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
#endif
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata2_luv_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata3_luv_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata1_luv_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata1_hs_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata2_hs_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        acamera_faults_cfg_faults_cfg_status_out_format_crc_vdata1_s_o_fault_crc_mismatch_write( PHY_ADDR_ISP, 1 );
        break;
    default:
        break;
    }
}

static const char *ml_bist_get_test_name( const ml_bist_test_t test )
{
    switch ( test ) {
    case ML_BIST_TEST0:
        return "ML_BIST_TEST0";
    case ML_BIST_TEST1:
        return "ML_BIST_TEST1";
    case ML_BIST_TEST2:
        return "ML_BIST_TEST2";
    case ML_BIST_TEST3:
        return "ML_BIST_TEST3";
    case ML_BIST_TEST4:
        return "ML_BIST_TEST4";
    case ML_BIST_TEST5:
        return "ML_BIST_TEST5";
    case ML_BIST_TEST6:
        return "ML_BIST_TEST6";
    case ML_BIST_TEST7:
        return "ML_BIST_TEST7";
    default:
        return "ML_BIST_ERROR";
    }
}


/**
 * @brief      Checks for Module level BIST faults in ISP registers
 *
 * @param[in]  current_test_number  The test run number
 *
 * @note       While the api exists to read invididual bits in the registers for
 *             all of the faults here we cannot use it since the overhead of
 *             calling so many register access functions is too much for this FSM.
 *             For this reason we hardcode the values here.
 *
 */
static void ml_bist_check_for_faults( const ml_bist_test_t test )
{
    const char *test_name = ml_bist_get_test_name( test );

    /* The number in the name is the address offset. For example: reg60 is PHY_ADDR_ISP + 0x01C00 + 0x60*/
    const uint32_t reg60 = system_isp_read_32( PHY_ADDR_ISP + ACAMERA_FAULTS_CFG_BASE_ADDR + ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_FRAME_STITCH_CRC_DATA_O_FAULT_CRC_MISMATCH_OFFSET );
    const uint32_t reg64 = system_isp_read_32( PHY_ADDR_ISP + ACAMERA_FAULTS_CFG_BASE_ADDR + ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_RAW_FRONTEND_CRC_DATA_O_FAULT_CRC_MISMATCH_OFFSET );
    const uint32_t reg68 = system_isp_read_32( PHY_ADDR_ISP + ACAMERA_FAULTS_CFG_BASE_ADDR + ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_RADIAL_SHADING_CRC_DATA_O_FAULT_CRC_MISMATCH_OFFSET );
    const uint32_t reg6c = system_isp_read_32( PHY_ADDR_ISP + ACAMERA_FAULTS_CFG_BASE_ADDR + ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_DEMOSAIC_CRC_GRN_O_FAULT_CRC_MISMATCH_OFFSET );
    const uint32_t reg70 = system_isp_read_32( PHY_ADDR_ISP + ACAMERA_FAULTS_CFG_BASE_ADDR + ACAMERA_FAULTS_CFG_FAULTS_CFG_STATUS_OUT_FORMAT_CRC_VDATA2_HS_O_FAULT_CRC_MISMATCH_OFFSET );

    const uint32_t err_frame_stich = BF_GET( reg60, 29, 2 );   //frame stich error register 0x60, 2 bits, starting at offset 29.
    const uint32_t err_fe = BF_GET( reg64, 18, 3 );            //front end error register 0x64, 3 bits, starting at offset 18
    const uint32_t err_sinter = BF_GET( reg64, 31, 1 );        //sinter error register 0x64, 1 bit, starting at offset 31
    const uint32_t err_radial = BF_GET( reg68, 9, 1 );         //radial shading erorr, register 0x68, 1 bit, starting at offset 9
    const uint32_t err_iridix = BF_GET( reg68, 18, 1 );        //iridix error, register 0x68, 1 bit, starting at offset 18
    const uint32_t err_demosaic_rccc = BF_GET( reg68, 26, 2 ); //demosaic rccc error, register 0x68, 2 bits, starting at offset 26
    const uint32_t err_demosaic1 = BF_GET( reg68, 31, 1 );     //demosaic error, register 0x68, 1 bit, starting at offset 31
    const uint32_t err_demosaic2 = BF_GET( reg6c, 0, 3 );      //demosaic error, register 0x6C, 3 bits, starting at offset 0
    const uint32_t err_demo_rgbir = BF_GET( reg6c, 8, 6 );     //demosaic rgbir error, register 0x6C, 6 bits, starting at offset 8
    const uint32_t err_out_fmt1 = BF_GET( reg6c, 18, 14 );     //out formatter error, register 0x6C, 14 bits, starting at offset 18
    const uint32_t err_out_fmt2 = BF_GET( reg70, 0, 2 );       //out formatter error, register 0x70, 2 bits, starting at offset 0

    if ( err_fe ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x) in Raw FrontEnd, test: %s", err_fe, test_name );
    }

    if ( err_frame_stich ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x) Frame Stich, test: %s", err_frame_stich, test_name );
    }

    if ( err_sinter ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x) in Sinter, test: %s", err_sinter, test_name );
    }

    if ( err_radial ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x) in Radial Shading, test: %s", err_radial, test_name );
    }

    if ( err_iridix ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x) in Iridix, test: %s", err_iridix, test_name );
    }

    if ( err_demosaic_rccc ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x) in Demosaic RCCC, test: %s", err_demosaic_rccc, test_name );
    }

    if ( err_demosaic1 || err_demosaic2 ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x, 0x%x) in Demosaic, test: %s", err_demosaic1, err_demosaic2, test_name );
    }

    if ( err_demo_rgbir ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x) in Demosaic RGBIr, test: %s", err_demo_rgbir, test_name );
    }

    if ( err_out_fmt1 || err_out_fmt2 ) {
        LOG( LOG_ERR, "ML_BIST Error(0x%x, 0x%x) in Out Format, test: %s", err_out_fmt1,
             err_out_fmt2, test_name );
    }
}

#endif
