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

#include "system_assert.h"
#include "acamera_command_api.h"
#include "acamera_frontend_config.h"
#include "acamera_gamma_be0_mem_config.h"
#include "acamera_gamma_be1_mem_config.h"
#include "acamera_gamma_fe_mem_config.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"
#include "util_pool.h"

#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_DECOMPANDER


#define DECOMPANDER_HAS_SQ defined( ACAMERA_ISP_GAMMA_BE_BLACK_LEVEL_OUT_SQ_DEFAULT )
#define DECOMPANDER_HAS_DL defined( ACAMERA_ISP_GAMMA_BE_BLACK_LEVEL_OUT_DL_DEFAULT )

#define DEFAULT_CHANNEL_SELECT_1_ROUTE ( 0 )
#define DEFAULT_CHANNEL_SELECT_2_ROUTE ( 1 )
#define DEFAULT_CHANNEL_SELECT_3_ROUTE ( 2 )
#define DEFAULT_CHANNEL_SELECT_4_ROUTE ( 3 )

#define MAX_SENSOR_INPUT_CHANNELS ( 4 )

#if !( DECOMPANDER_HAS_SQ ) && !( DECOMPANDER_HAS_DL )
#error "DECOMPANDER_HAS_SQ or DECOMPANDER_HAS_DL required"
#endif

/**
 * input_formatter_initial_capabilities() - assigns the default capabilities for input formatter to channel_processing_level structure
 * @cv: pointer to channel_processing_level structure
 *
 * Return: None
 */
static void input_formatter_initial_capabilities( channel_processing_level *cv )
{
    cv[0] = CAP_MAX;
    cv[1] = CAP_3_FROM_2_DECODE_L;
    cv[2] = CAP_CHANNEL_PASS_THROUGH;
    cv[3] = CAP_CHANNEL_PASS_THROUGH;
}

/**
 * input_formatter_bitwidht_alignment() - Set input formatter bit-width alignment.
 * @isp_base: ISP hw context memory base address.
 * @p_in_fmt: Pointer to input formatter control structure.
 *
 * This function has two implementations, depending on the Morgan ISP revision.
 *
 * Return: None
 */
#if ( ISP_RTL_VERSION_R >= 1 )
static void input_formatter_bitwidth_alignment( uint32_t isp_base, const input_formatter_ctrl *p_in_fmt )
{
    acamera_isp_input_formatter_input1_bitwidth_select_write( isp_base, p_in_fmt->input1_bitwidth );
    acamera_isp_input_formatter_input2_bitwidth_select_write( isp_base, p_in_fmt->input2_bitwidth );
    acamera_isp_input_formatter_input3_bitwidth_select_write( isp_base, p_in_fmt->input3_bitwidth );
    acamera_isp_input_formatter_input4_bitwidth_select_write( isp_base, p_in_fmt->input4_bitwidth );

    acamera_isp_input_formatter_input1_alignment_write( isp_base, p_in_fmt->input1_aligment );
    acamera_isp_input_formatter_input2_alignment_write( isp_base, p_in_fmt->input2_aligment );
    acamera_isp_input_formatter_input3_alignment_write( isp_base, p_in_fmt->input3_aligment );
    acamera_isp_input_formatter_input4_alignment_write( isp_base, p_in_fmt->input4_aligment );
}
#else
static void input_formatter_bitwidth_alignment( uint32_t isp_base, const input_formatter_ctrl *p_in_fmt )
{
    acamera_isp_input_formatter_input_bitwidth_select_write( isp_base, p_in_fmt->input1_bitwidth );
    acamera_isp_input_formatter_input_alignment_write( isp_base, p_in_fmt->input1_aligment );
}
#endif // ISP_RTL_VERSION_R

static int if_configure_input_channel_switch( decompander_fsm_ptr_t p_fsm, channel_processing_level *cv )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    acamera_cmd_sensor_info sensor_info;
    if ( WRAP_GENERAL_CMD( p_ictx, CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&sensor_info ) ) {
        return -1;
    }

    general_sensor_mode_t *s_mode = &sensor_info.current_sensor_mode;

    int i;
    uint16_t channel_counter;
    uintptr_t temp_iterator;
    uintptr_t temp_holder;
    uint16_t max_cap_index;
    uintptr_t max_cap_index_id;
    uint16_t channel_conf_flag;

    general_channel_desc_t *channel_desc = s_mode->channel_info.channel_desc;

    apool blocked_if_channel, blocked_input_channel;
    apool_element blocked_if_channel_el[MAX_SENSOR_INPUT_CHANNELS], blocked_input_channel_el[MAX_SENSOR_INPUT_CHANNELS];

    apool_create( &blocked_if_channel, blocked_if_channel_el, MAX_SENSOR_INPUT_CHANNELS );
    apool_create( &blocked_input_channel, blocked_input_channel_el, MAX_SENSOR_INPUT_CHANNELS );

    /* Restoring channel select default configuration */
    acamera_isp_pipeline_channel1_select_write( p_ictx->settings.isp_base, DEFAULT_CHANNEL_SELECT_1_ROUTE );
    acamera_isp_pipeline_channel2_select_write( p_ictx->settings.isp_base, DEFAULT_CHANNEL_SELECT_2_ROUTE );
    acamera_isp_pipeline_channel3_select_write( p_ictx->settings.isp_base, DEFAULT_CHANNEL_SELECT_3_ROUTE );
    acamera_isp_pipeline_channel4_select_write( p_ictx->settings.isp_base, DEFAULT_CHANNEL_SELECT_4_ROUTE );
    acamera_isp_pipeline_bypass_channel_switch_write( p_ictx->settings.isp_base, 1 );

    for ( channel_counter = 0; channel_counter < s_mode->num_channels; ++channel_counter ) {

        /* Find the channel with the most significant capability: */
        max_cap_index = CAP_MIN;
        max_cap_index_id = 0;
        for ( temp_iterator = 0; temp_iterator < s_mode->num_channels; ++temp_iterator ) {
            if ( !apool_contains( &blocked_input_channel, (void *)temp_iterator ) ) {
                continue;
            }

            if ( max_cap_index < channel_desc[temp_iterator].cv ) {
                max_cap_index = channel_desc[temp_iterator].cv;
                max_cap_index_id = temp_iterator;
            }
        }

        if ( max_cap_index == CAP_MIN ) {
            LOG( LOG_CRIT, "Failed to find channel with highest capability requirement!\n" );
            return -1;
        }
        apool_add( &blocked_input_channel, (void *)max_cap_index_id );

        channel_conf_flag = 0;
        for ( i = 0; i < s_mode->num_channels; ++i ) {
            /* Try to configure the chosen channel from the code above */
            temp_holder = i;
            if ( !apool_contains( &blocked_if_channel, (void *)temp_holder ) ) {
                continue;
            }

            if ( cv[i] >= channel_desc[max_cap_index_id].cv ) {
                switch ( i ) {
                case 0:
                    acamera_isp_pipeline_channel1_select_write( p_ictx->settings.isp_base, max_cap_index_id );
                    break;
                case 1:
                    acamera_isp_pipeline_channel2_select_write( p_ictx->settings.isp_base, max_cap_index_id );
                    break;
                case 2:
                    acamera_isp_pipeline_channel3_select_write( p_ictx->settings.isp_base, max_cap_index_id );
                    break;
                case 3:
                    acamera_isp_pipeline_channel4_select_write( p_ictx->settings.isp_base, max_cap_index_id );
                    break;
                default:
                    break;
                }

                channel_conf_flag = 1;
                apool_add( &blocked_if_channel, (void *)temp_holder );
                break;
            }
        }

        if ( !channel_conf_flag ) {
            LOG( LOG_CRIT, "Failed to configure input formatter channels!\n" );
            return -1;
        }
    }

    uint32_t channel_map[4] = {0};
    channel_map[0] = acamera_isp_pipeline_channel1_select_read( p_ictx->settings.isp_base );
    channel_map[1] = acamera_isp_pipeline_channel2_select_read( p_ictx->settings.isp_base );
    channel_map[2] = acamera_isp_pipeline_channel3_select_read( p_ictx->settings.isp_base );
    channel_map[3] = acamera_isp_pipeline_channel4_select_read( p_ictx->settings.isp_base );

    if ( channel_map[0] != DEFAULT_CHANNEL_SELECT_1_ROUTE ||
         channel_map[1] != DEFAULT_CHANNEL_SELECT_2_ROUTE ||
         channel_map[2] != DEFAULT_CHANNEL_SELECT_3_ROUTE ||
         channel_map[3] != DEFAULT_CHANNEL_SELECT_4_ROUTE ) {
        /* Configuration changed, disabling fs_channel_switch bypass */
        acamera_isp_pipeline_bypass_channel_switch_write( p_ictx->settings.isp_base, 0 );
    }

    return 0;
}

/**
 * configure_input_formatter() - Configure input formatter based on calibration.
 * @p_ictx: Pointer to isp calibration data.
 * @isp_base: ISP hw context memory base address.
 *
 * Return: None
 */
static void configure_input_formatter(
    decompander_fsm_ptr_t p_fsm,
    const decompander_ctrl *ctrl )
{
    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    const input_formatter_ctrl *p_in_fmt = (const input_formatter_ctrl *)
        calib_mgr_u16_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_INPUT_FORMATTER );

    channel_processing_level cv[4] = {0};
    input_formatter_initial_capabilities( cv );

    if_configure_input_channel_switch( p_fsm, cv );

    /*Input channel configuration:*/

    if ( ctrl->input_formatter_enable ) {
        acamera_isp_input_formatter_mode_in_write( isp_base, p_in_fmt->in_fmt_mode );

        input_formatter_bitwidth_alignment( isp_base, p_in_fmt );

        acamera_isp_input_formatter_black_level_write( isp_base, p_in_fmt->black_level );

        acamera_isp_input_formatter_knee_point0_write( isp_base, p_in_fmt->knee_point0 );
        acamera_isp_input_formatter_knee_point1_write( isp_base, p_in_fmt->knee_point1 );
        acamera_isp_input_formatter_knee_point2_write( isp_base, p_in_fmt->knee_point2 );

        acamera_isp_input_formatter_slope0_select_write( isp_base, p_in_fmt->slope0 );
        acamera_isp_input_formatter_slope1_select_write( isp_base, p_in_fmt->slope1 );
        acamera_isp_input_formatter_slope2_select_write( isp_base, p_in_fmt->slope2 );
        acamera_isp_input_formatter_slope3_select_write( isp_base, p_in_fmt->slope3 );
    }

    acamera_isp_pipeline_linear_data_source_write( isp_base, ctrl->linear_data_source );
    acamera_isp_pipeline_bypass_input_formatter_write( isp_base, ~ctrl->input_formatter_enable );
}

/**
 * configure_gamma_fe() - Configure gamma fe based on calibration.
 * @p_ictx: Pointer to ISP calibration data.
 * @isp_base: ISP hw context memory base address.
 *
 * Return: None
 */
static void configure_gamma_fe(
    decompander_fsm_ptr_t p_fsm,
    const decompander_ctrl *ctrl )
{
    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;

    if ( ctrl->gamma_fe_lut_enable ) {
        const uint32_t gamma_fe_size = calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_GAMMA_FE );
        const uint32_t *gamma_fe_lut = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_GAMMA_FE );
        size_t i;
        for ( i = 0; i < gamma_fe_size - 1; i++ )
            acamera_gamma_fe_mem_array_data_write( isp_base, i, gamma_fe_lut[i] );
        acamera_isp_gamma_fe_data_last_write( isp_base, gamma_fe_lut[gamma_fe_size - 1] );
    }

    if ( ctrl->gamma_fe_sqrt_enable ) {
        const uint32_t *gamma_black_levels = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_GAMMA_BLACK_LEVELS );
#if DECOMPANDER_HAS_SQ
        acamera_isp_gamma_fe_black_level_in_sq_write( isp_base, *gamma_black_levels++ );
        acamera_isp_gamma_fe_black_level_out_sq_write( isp_base, *gamma_black_levels++ );
#elif DECOMPANDER_HAS_DL
        acamera_isp_gamma_fe_black_level_in_dl_write( isp_base, *gamma_black_levels++ );
        acamera_isp_gamma_fe_black_level_out_dl_write( isp_base, *gamma_black_levels++ );
        acamera_isp_gamma_fe_alpha_write( isp_base, *gamma_black_levels++ );
#endif
    }


#if DECOMPANDER_HAS_SQ
    acamera_isp_pipeline_bypass_gamma_fe_sq_write( isp_base, ~ctrl->gamma_fe_sqrt_enable );
#elif DECOMPANDER_HAS_DL
    acamera_isp_pipeline_bypass_gamma_fe_dl_write( isp_base, ~ctrl->gamma_fe_sqrt_enable );
    acamera_isp_gamma_fe_enable_dl_write( isp_base, ctrl->gamma_fe_sqrt_enable );
#endif

    acamera_isp_pipeline_bypass_gamma_fe_write( isp_base, ~ctrl->gamma_fe_lut_enable );
    acamera_isp_gamma_fe_enable_write( isp_base, ctrl->gamma_fe_lut_enable );
}

/**
 * configure_gamma_be() - Configure gamma be based on calibration.
 * @p_ictx: Pointer to ISP calibration data.
 * @isp_base: ISP hw context memory base address.
 * @mode: Current WDR mode
 *
 * Return: None
 */
static void configure_gamma_be( decompander_fsm_ptr_t p_fsm,
                                const decompander_ctrl *ctrl )
{
    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;
    void *p_cm = ACAMERA_FSM2CM_PTR( p_fsm );

    if ( ctrl->gamma_be_lut_enable ) {
        const uint32_t *gamma_be_0_lut = calib_mgr_u32_lut_get( p_cm, CALIBRATION_GAMMA_BE0 );
        const uint32_t *gamma_be_1_lut = calib_mgr_u32_lut_get( p_cm, CALIBRATION_GAMMA_BE1 );
        const uint32_t gamma_be_0_len = calib_mgr_lut_len( p_cm, CALIBRATION_GAMMA_BE0 );
        const uint32_t gamma_be_1_len = calib_mgr_lut_len( p_cm, CALIBRATION_GAMMA_BE1 );
        const uint32_t gamma_be_0_hw_len = ( ACAMERA_GAMMA_BE0_MEM_SIZE / ( ACAMERA_GAMMA_BE0_MEM_ARRAY_DATA_DATASIZE >> 3 ) ) + 1;
        const uint32_t gamma_be_1_hw_len = ( ACAMERA_GAMMA_BE1_MEM_SIZE / ( ACAMERA_GAMMA_BE1_MEM_ARRAY_DATA_DATASIZE >> 3 ) ) + 1;

        if ( gamma_be_0_len != gamma_be_0_hw_len ) {
            LOG( LOG_ERR, "Wrong elements number in gamma_be_0_lut: expected %d got %d.", gamma_be_0_hw_len, gamma_be_0_len );
            return;
        }
        if ( gamma_be_1_len != gamma_be_1_hw_len ) {
            LOG( LOG_ERR, "Wrong elements number in gamma_be_1_lut: expected %d got %d.", gamma_be_1_hw_len, gamma_be_1_len );
            return;
        }

        size_t i;
        for ( i = 0; i < gamma_be_0_len - 1; i++ ) {
            acamera_gamma_be0_mem_array_data_write( isp_base, i, gamma_be_0_lut[i] );
        }
        acamera_isp_gamma_be_data_last_0_write( isp_base, gamma_be_0_lut[gamma_be_0_len - 1] );

        for ( i = 0; i < gamma_be_1_len - 1; i++ ) {
            acamera_gamma_be1_mem_array_data_write( isp_base, i, gamma_be_1_lut[i] );
        }
        acamera_isp_gamma_be_data_last_1_write( isp_base, gamma_be_1_lut[gamma_be_1_len - 1] );


        LOG( LOG_DEBUG, "gamma_be_0_lut length: %d.", gamma_be_0_len );
        LOG( LOG_DEBUG, "gamma_be_1_lut length: %d.", gamma_be_1_len );
    }


    if ( ctrl->gamma_be_sq_enable ) {
        const uint32_t *gamma_black_levels = calib_mgr_u32_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_GAMMA_BLACK_LEVELS );
#if DECOMPANDER_HAS_SQ
        acamera_isp_gamma_be_black_level_out_sq_write( isp_base, *gamma_black_levels++ );
        acamera_isp_gamma_be_black_level_in_sq_write( isp_base, *gamma_black_levels++ );
#elif DECOMPANDER_HAS_DL
        acamera_isp_gamma_be_black_level_out_dl_write( isp_base, *gamma_black_levels++ );
        acamera_isp_gamma_be_black_level_in_dl_write( isp_base, *gamma_black_levels++ );
        acamera_isp_gamma_be_alpha_write( isp_base, *gamma_black_levels++ );
#endif
    }


#if DECOMPANDER_HAS_SQ
    acamera_isp_pipeline_bypass_gamma_be_sq_write( isp_base, ~ctrl->gamma_be_sq_enable );
#elif DECOMPANDER_HAS_DL
    acamera_isp_pipeline_bypass_gamma_be_dl_write( isp_base, ~ctrl->gamma_be_sq_enable );
    acamera_isp_gamma_be_enable_dl_write( isp_base, ctrl->gamma_be_sq_enable );
#endif

    acamera_isp_pipeline_bypass_gamma_be_write( isp_base, ~ctrl->gamma_be_lut_enable );
    acamera_isp_gamma_be_enable_0_write( isp_base, ctrl->gamma_be_lut_enable );
    acamera_isp_gamma_be_enable_1_write( isp_base, ctrl->gamma_be_lut_enable );
}

/**
 * decompander_initialize() - FSM handler function.
 * @p_fsm: Pointer to decompander FSM.

 * Return: None
 */
void decompander_init( decompander_fsm_ptr_t p_fsm )
{
}

/**
 * @brief      Tear down the FSM
 *
 * @param[in]  p_fsm  Pointer to the FSM data
 *
 */
void decompander_deinit( decompander_fsm_ptr_t p_fsm )
{
    const uint32_t isp_base = ACAMERA_FSM2ICTX_PTR( p_fsm )->settings.isp_base;

#if DECOMPANDER_HAS_SQ
    acamera_isp_pipeline_bypass_gamma_fe_sq_write( isp_base, 1 );
    acamera_isp_pipeline_bypass_gamma_be_sq_write( isp_base, 1 );
#elif DECOMPANDER_HAS_DL
    acamera_isp_pipeline_bypass_gamma_fe_dl_write( isp_base, 1 );
    acamera_isp_pipeline_bypass_gamma_be_dl_write( isp_base, 1 );
    acamera_isp_gamma_fe_enable_dl_write( isp_base, 0 );
    acamera_isp_gamma_be_enable_dl_write( isp_base, 0 );
#endif

    acamera_isp_pipeline_bypass_gamma_fe_write( isp_base, 1 );
    acamera_isp_pipeline_bypass_gamma_be_write( isp_base, 1 );

    acamera_isp_gamma_be_enable_0_write( isp_base, 0 );
    acamera_isp_gamma_be_enable_1_write( isp_base, 0 );
    acamera_isp_gamma_fe_enable_write( isp_base, 0 );
}

/**
 * decompander_reload_calibration() - FSM handler function.
 * @p_fsm: Pointer to decompander FSM.

 * Return: None
 */
void decompander_reload_calibration( decompander_fsm_ptr_t p_fsm )
{
    /* If tuning is missing, do nothing with input formatter. */
    if ( calib_mgr_lut_exists( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_DECOMPANDER_CONTROL ) ) {
        assert( calib_mgr_lut_len( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_DECOMPANDER_CONTROL ) ==
                sizeof( decompander_ctrl ) );
        const decompander_ctrl *p_decompander_ctrl = (const decompander_ctrl *)calib_mgr_lut_get( ACAMERA_FSM2CM_PTR( p_fsm ), CALIBRATION_DECOMPANDER_CONTROL );
        configure_input_formatter( p_fsm, p_decompander_ctrl );
        configure_gamma_fe( p_fsm, p_decompander_ctrl );
        configure_gamma_be( p_fsm, p_decompander_ctrl );
    }
}