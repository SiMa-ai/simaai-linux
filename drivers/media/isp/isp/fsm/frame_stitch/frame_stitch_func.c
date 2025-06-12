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

#include "acamera_command_api.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"

/*Channel select channels are counted from 1, but are configured from 0 and up*/
#define DEFAULT_FS_CHANNEL_SELECT_1_ROUTE 0
#define DEFAULT_FS_CHANNEL_SELECT_2_ROUTE 1
#define DEFAULT_FS_CHANNEL_SELECT_3_ROUTE 2
#define DEFAULT_FS_CHANNEL_SELECT_4_ROUTE 3

/**
 *   @brief     initialise fsm
 *
 *   @param     p_fsm   pointer to fsm private data
 *   @return
 *   @warning
 *   @details   initialise corresponding HW/SW modules to initial status
 */
void frame_stitch_init( frame_stitch_fsm_ptr_t p_fsm )
{
    return;
}

void get_channel_info_by_id( frame_stitch_fsm_ptr_t p_fsm, uint8_t id, general_channel_desc_t *channel_desc )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    acamera_cmd_sensor_info sensor_info;
    if ( WRAP_GENERAL_CMD( p_ictx, CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&sensor_info ) ) {
        return;
    }

    const general_sensor_mode_t *s_mode = &sensor_info.current_sensor_mode;

    *channel_desc = s_mode->channel_info.channel_desc[id];
}

/**
 *   @brief     configure fsm
 *
 *   @param     p_fsm   pointer to fsm private data
 *   @return
 *   @warning
 *   @details   configure corresponding HW/SW modules to given mode of operation with given parameters.
 */
void frame_stitch_config( frame_stitch_fsm_ptr_t p_fsm )
{
    acamera_isp_ctx_ptr_t p_ictx = ACAMERA_FSM2ICTX_PTR( p_fsm );

    acamera_cmd_sensor_info sensor_info;
    if ( WRAP_GENERAL_CMD( p_ictx, CMD_ID_SENSOR_INFO, CMD_DIRECTION_GET, NULL, (uint32_t *)&sensor_info ) ) {
        return;
    }

    const general_sensor_mode_t *s_mode = &sensor_info.current_sensor_mode;

    uint32_t channel_map[4];

    /*Restoring fs channel select default configuration*/
    acamera_isp_pipeline_bypass_frame_stitch_write( p_ictx->settings.isp_base, 1 );
    acamera_isp_pipeline_bypass_fs_channel_switch_write( p_ictx->settings.isp_base, 1 );
    acamera_isp_pipeline_fs_channel1_select_write( p_ictx->settings.isp_base, DEFAULT_FS_CHANNEL_SELECT_1_ROUTE );
    acamera_isp_pipeline_fs_channel2_select_write( p_ictx->settings.isp_base, DEFAULT_FS_CHANNEL_SELECT_2_ROUTE );
    acamera_isp_pipeline_fs_channel3_select_write( p_ictx->settings.isp_base, DEFAULT_FS_CHANNEL_SELECT_3_ROUTE );
    acamera_isp_pipeline_fs_channel4_select_write( p_ictx->settings.isp_base, DEFAULT_FS_CHANNEL_SELECT_4_ROUTE );

    /*Disabling frame stitch bypass if we have more than one channel*/
    if ( s_mode->num_channels > 1 ) {
        acamera_isp_pipeline_bypass_frame_stitch_write( p_ictx->settings.isp_base, 0 );
    }

    channel_map[0] = acamera_isp_pipeline_channel1_select_read( p_ictx->settings.isp_base );
    channel_map[1] = acamera_isp_pipeline_channel2_select_read( p_ictx->settings.isp_base );
    channel_map[2] = acamera_isp_pipeline_channel3_select_read( p_ictx->settings.isp_base );
    channel_map[3] = acamera_isp_pipeline_channel4_select_read( p_ictx->settings.isp_base );

    channel_data_type latest_configured_expo = DATA_TYPE_WDR_ENUM_MIN;
    channel_data_type shortest_unconfigured_expo;
    uint8_t shortest_unconfigured_channel_id = 0;
    uint8_t latest_configured_channel_id = 0;
    uint32_t i, k;

    for ( i = 0; i < s_mode->num_channels; ++i ) {
        shortest_unconfigured_expo = DATA_TYPE_WDR_ENUM_MAX;
        for ( k = 0; k < s_mode->num_channels; ++k ) {
            general_channel_desc_t channel_info = {0};
            get_channel_info_by_id( p_fsm, channel_map[k], &channel_info );
            if ( channel_info.data_type > DATA_TYPE_WDR_ENUM_MIN &&
                 channel_info.data_type < DATA_TYPE_WDR_ENUM_MAX &&
                 channel_info.data_type < shortest_unconfigured_expo &&
                 channel_info.data_type > latest_configured_expo ) {
                shortest_unconfigured_expo = channel_info.data_type;
                shortest_unconfigured_channel_id = k;
            }
        }

        /*
            1.Find shortest channel:
                1.1.Check for the original id and check the channel;
        */

        switch ( latest_configured_channel_id ) {
        case 0:
            acamera_isp_pipeline_fs_channel1_select_write( p_ictx->settings.isp_base, shortest_unconfigured_channel_id );
            break;
        case 1:
            acamera_isp_pipeline_fs_channel2_select_write( p_ictx->settings.isp_base, shortest_unconfigured_channel_id );
            break;
        case 2:
            acamera_isp_pipeline_fs_channel3_select_write( p_ictx->settings.isp_base, shortest_unconfigured_channel_id );
            break;
        case 3:
            acamera_isp_pipeline_fs_channel4_select_write( p_ictx->settings.isp_base, shortest_unconfigured_channel_id );
            break;
        default:
            break;
        }

        latest_configured_expo = shortest_unconfigured_expo;
        latest_configured_channel_id++;
    }

    /*Re-using channel_map array to check for changes to the initial configuration*/
    channel_map[0] = acamera_isp_pipeline_fs_channel1_select_read( p_ictx->settings.isp_base );
    channel_map[1] = acamera_isp_pipeline_fs_channel2_select_read( p_ictx->settings.isp_base );
    channel_map[2] = acamera_isp_pipeline_fs_channel3_select_read( p_ictx->settings.isp_base );
    channel_map[3] = acamera_isp_pipeline_fs_channel4_select_read( p_ictx->settings.isp_base );

    if ( channel_map[0] != DEFAULT_FS_CHANNEL_SELECT_1_ROUTE ||
         channel_map[1] != DEFAULT_FS_CHANNEL_SELECT_2_ROUTE ||
         channel_map[2] != DEFAULT_FS_CHANNEL_SELECT_3_ROUTE ||
         channel_map[3] != DEFAULT_FS_CHANNEL_SELECT_4_ROUTE ) {
        /*Configuration changed, disabling fs_channel_switch bypasss*/
        acamera_isp_pipeline_bypass_fs_channel_switch_write( p_ictx->settings.isp_base, 0 );
    }
}

/**
 *   @brief     update fsm hw
 *
 *   @param     p_fsm   pointer to fsm private data
 *   @return
 *   @warning
 *   @details   update corresponding HW/SW modules on timing event generated by ISP HW or Algorithm.
 */
void frame_stitch_update_hw( frame_stitch_fsm_ptr_t p_fsm )
{
    return;
}

/**
 *   @brief     reload fsm calibration LUTs
 *
 *   @param     p_fsm   pointer to fsm private data
 *   @return
 *   @warning
 *   @details   reloads corresponding calibration LUTs and updates HW.
 */
void frame_stitch_reload_calibration( frame_stitch_fsm_ptr_t p_fsm )
{
    frame_stitch_config( p_fsm );
}

/**
 *   @brief     de-initialise fsm
 *
 *   @param     p_fsm   pointer to fsm private data
 *   @return
 *   @warning
 *   @details   de-initialise corresponding HW/SW modules to reset status and release all allocated resources.
 */
void frame_stitch_deinit( frame_stitch_fsm_ptr_t p_fsm )
{
    return;
}
