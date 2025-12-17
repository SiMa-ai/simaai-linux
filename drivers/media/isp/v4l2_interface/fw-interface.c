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

#include <asm/div64.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>

#include "acamera_command_api.h"
#include "acamera_logger.h"
#include "fw-interface.h"
#include "isp-v4l2-common.h"
#include "isp-v4l2-stream.h"
#include "isp-v4l2.h"
#include "isp-vb2.h"

static int isp_fw_do_set_cmd( uint32_t ctx_id, uint8_t command_type, uint8_t command, uint32_t value );
static int isp_fw_do_get_cmd( uint32_t ctx_id, uint8_t command_type, uint8_t command, uint32_t *ret_val );

/**
 * @brief Converts ISP V4L2 Stream type to AFrame type
 *
 * @param stream_type ISP V4L2 Stream type ID
 * @param stream_direction ISP V4L2 Stream direction
 * @return aframe_type_t Returns correct AFrame type on success, AFRAME_TYPE_UNKNOWN otherwise
 */
aframe_type_t isp_fw_stream_type_to_aframe_type( isp_v4l2_stream_type_t stream_type, isp_v4l2_stream_direction_t stream_direction )
{

    switch ( stream_type ) {
    case V4L2_STREAM_TYPE_RAW:
        if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
            return AFRAME_TYPE_RAW;
        } else {
            return AFRAME_TYPE_UNKNOWN;
        }

    case V4L2_STREAM_TYPE_OUT:
        if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
            return AFRAME_TYPE_OUT;
        } else {
            return AFRAME_TYPE_UNKNOWN;
        }

    case V4L2_STREAM_TYPE_META:
        if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
            return AFRAME_TYPE_META;
        } else {
            return AFRAME_TYPE_UNKNOWN;
        }

    case V4L2_STREAM_TYPE_M2M:
        if ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) {
            return AFRAME_TYPE_OUT;
        } else {
            return AFRAME_TYPE_RAW;
        }

    default:
        LOG( LOG_ERR, "Unknown stream type supplied: %d", stream_type );
        return AFRAME_TYPE_UNKNOWN;
    }
}

/**
 * @brief Converts AFrame type to ISP V4L2 Stream type
 *
 * @param type AFrame type ID
 * @return aframe_type_t Returns correct ISP V4L2 Stream Type on success, V4L2_STREAM_TYPE_MAX otherwise
 */
isp_v4l2_stream_type_t isp_fw_aframe_type_to_stream_type( aframe_type_t frame_type )
{
    switch ( frame_type ) {
    case AFRAME_TYPE_RAW:
        return V4L2_STREAM_TYPE_RAW;

    case AFRAME_TYPE_OUT:
        return V4L2_STREAM_TYPE_OUT;

    case AFRAME_TYPE_META:
        return V4L2_STREAM_TYPE_META;

    default:
        LOG( LOG_CRIT, "Unknown frame type supplied: %d", frame_type );
        return V4L2_STREAM_TYPE_MAX;
    }
}

/**
 * @brief Gets current state of the specified context
 *
 * @param ctx_id Context ID
 * @return int Returns current context state on success, CTX_STATE_UNKNOWN otherwise
 */
static int isp_fw_get_context_state( uint32_t ctx_id )
{
    uint32_t context_state;

    uint8_t rc;
    if ( ( rc = acamera_command( ctx_id, TSYSTEM, CONTEXT_STATE, 0, COMMAND_GET, &context_state ) ) != SUCCESS ) {
        LOG( LOG_ERR, "Failed to get context (id: %u) state. Command API call returned error: %u", ctx_id, rc );
        return CTX_STATE_UNKNOWN;
    }

    return context_state;
}

/**
 * @brief Sets V4L2 interface mode for the specified context
 *
 * @param ctx_id Context ID
 * @param mode Target V4L2 interface mode
 * @return int Returns 0 on success, -1 otherwise
 */
static int isp_fw_set_v4l2_interface_mode( uint32_t ctx_id, uint32_t mode )
{
    uint8_t rc;
    if ( ( rc = acamera_command( ctx_id, TSYSTEM, V4L2_INTERFACE_MODE, mode, COMMAND_SET, NULL ) ) != SUCCESS ) {
        LOG( LOG_ERR, "Failed to set context (id: %u) V4L2 interface mode. Command API call returned error: %u", ctx_id, rc );
        return -1;
    }

    return 0;
}

/**
 * fw_interface control interface
 */
int fw_intf_is_context_initialised( uint32_t ctx_id )
{
    const uint32_t context_state = isp_fw_get_context_state( ctx_id );

    switch ( context_state ) {
    case CTX_STATE_INIT:   // fallthrough
    case CTX_STATE_CONFIG: // fallthrough
    case CTX_STATE_START:  // fallthrough
    case CTX_STATE_STOP:
        return 1;
    default:
        return 0;
    }
}

int fw_intf_is_context_configured( uint32_t ctx_id )
{
    return ( isp_fw_get_context_state( ctx_id ) == CTX_STATE_CONFIG );
}

int fw_intf_is_context_started( uint32_t ctx_id )
{
    return ( isp_fw_get_context_state( ctx_id ) == CTX_STATE_START );
}

int fw_intf_is_context_stopped( uint32_t ctx_id )
{
    const uint32_t context_state = isp_fw_get_context_state( ctx_id );

    return ( ( context_state == CTX_STATE_STOP ) || ( context_state == CTX_STATE_INIT ) );
}

int fw_intf_context_config( uint32_t ctx_id )
{
    LOG( LOG_DEBUG, "Configuring context (id: %u)", ctx_id );

    uint8_t rc;
    if ( ( rc = acamera_command( ctx_id, TSYSTEM, CONTEXT_STATE, CTX_STATE_CONFIG, COMMAND_SET, NULL ) ) != SUCCESS ) {
        LOG( LOG_ERR, "Failed to configure context (id: %u). Command API call returned error: %u", ctx_id, rc );
        return -1;
    }

    return 0;
}

int fw_intf_context_start( uint32_t ctx_id )
{
    LOG( LOG_DEBUG, "Starting context (id: %u)", ctx_id );

    uint8_t rc;
    if ( ( rc = acamera_command( ctx_id, TSYSTEM, CONTEXT_STATE, CTX_STATE_START, COMMAND_SET, NULL ) ) != SUCCESS ) {
        LOG( LOG_ERR, "Failed to start context (id: %d). Command API call returned error: %u", ctx_id, rc );
        return -1;
    }

    return 0;
}

int fw_intf_context_stop( uint32_t ctx_id )
{
    LOG( LOG_DEBUG, "Stopping context (id: %u)", ctx_id );

    uint8_t rc;
    if ( ( rc = acamera_command( ctx_id, TSYSTEM, CONTEXT_STATE, CTX_STATE_STOP, COMMAND_SET, NULL ) ) != SUCCESS ) {
        LOG( LOG_ERR, "Failed to stop context (id: %d). Command API call returned error: %d", ctx_id, rc );
        return -1;
    }

    return 0;
}

int fw_intf_m2m_process_request( uint32_t ctx_id )
{
    LOG( LOG_DEBUG, "Requesting M2M context (id: %u) to process a frame", ctx_id );

    uint8_t rc;
    if ( ( rc = acamera_command( ctx_id, TSYSTEM, M2M_PROCESS_REQUEST, 1, COMMAND_SET, NULL ) ) != SUCCESS ) {
        LOG( LOG_ERR, "Failed to request M2M frame process on context (id: %d). Command API call returned error: %d", ctx_id, rc );
        return -1;
    }

    return 0;
}

int fw_intf_isp_get_current_ctx_id( uint32_t ctx_id )
{
    int active_ctx_id = -1;

    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "ISP context (id: %u) is not initialised yet", ctx_id );
        return -EBUSY;
    }

#if defined( TGENERAL ) && defined( ACTIVE_CONTEXT )
    acamera_command( ctx_id, TGENERAL, ACTIVE_CONTEXT, 0, COMMAND_GET, &active_ctx_id );
#endif

    return active_ctx_id;
}

int fw_intf_isp_update_sensor_info_mode( uint32_t ctx_id, isp_v4l2_sensor_info *sensor_info )
{
    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to update sensor mode. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }
#if defined( TSENSOR ) && defined( SENSOR_PRESET )
    // Get sensor current preset
    uint32_t current_preset = 0, found = 0, i, j;
    acamera_command( ctx_id, TSENSOR, SENSOR_PRESET, 0, COMMAND_GET, &current_preset );

	LOG (LOG_INFO, "current present mode is %d, %d", current_preset, sensor_info->num_modes);

    // Update sensor info current mode and submode based on sensor current preset
    for ( i = 0; ( ( i < sensor_info->num_modes ) && ( i < V4L2_SENSOR_INFO_MODES_MAX ) ); i++ ) {
		LOG (LOG_INFO, "num of sub modes : %d", sensor_info->mode[i].num_sub_modes);
        for ( j = 0; ( ( j < sensor_info->mode[i].num_sub_modes ) && ( j < V4L2_SENSOR_INFO_SUB_MODES_MAX ) ); j++ ) {
            if ( sensor_info->mode[i].sub_mode[j].sensor_preset == current_preset ) {
                sensor_info->cur_mode = i;
                sensor_info->mode[i].cur_sub_mode = j;
                found = 1;
                break;
            } else {
				LOG (LOG_INFO, "No match at %d,%d sensor preset : %d", i, j, sensor_info->mode[i].sub_mode[j].sensor_preset);
			} 
        }
    }

    if ( found == 0 ) {
        LOG( LOG_ERR, "Failed to update sensor info mode and submode for sensor preset: %u, no match found", current_preset );
        sensor_info->cur_mode = 0;
        sensor_info->mode[0].cur_sub_mode = 0;
    }

    LOG( LOG_INFO, "Sensor info mode: %u, sub mode: %u, sensor preset: %u",
         sensor_info->cur_mode, sensor_info->mode[sensor_info->cur_mode].cur_sub_mode, current_preset );
#else
    sensor_info->cur_mode = 0;
    sensor_info->mode[0].cur_sub_mode = 0;
#endif

    return 0;
}

int fw_intf_isp_get_sensor_info( uint32_t ctx_id, isp_v4l2_sensor_info *sensor_info )
{
    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to get sensor info. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }

#if defined( TSENSOR ) && defined( SENSOR_SUPPORTED_PRESETS ) && defined( SENSOR_INFO_PRESET ) && \
    defined( SENSOR_INFO_FPS ) && defined( SENSOR_INFO_WIDTH ) && defined( SENSOR_INFO_HEIGHT )
    LOG( LOG_INFO, "Sensor APIs found, initializing sensor info structure" );

    int i, j;
    uint32_t num_presets = 0;

    /* reset buffer */
    memset( sensor_info, 0x0, sizeof( isp_v4l2_sensor_info ) );

    /* get sensor preset number */
    acamera_command( ctx_id, TSENSOR, SENSOR_SUPPORTED_PRESETS, 0, COMMAND_GET, &num_presets );
    if ( num_presets > V4L2_SENSOR_INFO_MODES_MAX ) {
        LOG( LOG_ERR, "Sensor preset number (%u) is out of reserved range ( V4L2_SENSOR_INFO_MODES_MAX = %u )",
             num_presets, V4L2_SENSOR_INFO_MODES_MAX );
        num_presets = V4L2_SENSOR_INFO_MODES_MAX;
    }

	LOG( LOG_INFO, "num of presets %d", num_presets);
    /**
     * fill sensor info structure. Unique resolutions added as modes.
     * Sensor modes with the same resolution added as submodes under corresponding mode
     */
    for ( i = 0; i < num_presets; i++ ) {
        uint32_t width = 0, height = 0, fps = 0, exposures = 0, num_channels = 0, data_width = 0, ret_val = 0;

        /* get next preset */
        acamera_command( ctx_id, TSENSOR, SENSOR_INFO_PRESET, i, COMMAND_SET, &ret_val );
        acamera_command( ctx_id, TSENSOR, SENSOR_INFO_FPS, 0, COMMAND_GET, &fps );
        acamera_command( ctx_id, TSENSOR, SENSOR_INFO_WIDTH, 0, COMMAND_GET, &width );
        acamera_command( ctx_id, TSENSOR, SENSOR_INFO_HEIGHT, 0, COMMAND_GET, &height );
        acamera_command( ctx_id, TSENSOR, SENSOR_INFO_EXPOSURES, 0, COMMAND_GET, &exposures );
        acamera_command( ctx_id, TSENSOR, SENSOR_INFO_CHANNELS, 0, COMMAND_GET, &num_channels );
        acamera_command( ctx_id, TSENSOR, SENSOR_INFO_DATA_WIDTH, 0, COMMAND_GET, &data_width );
        LOG( LOG_INFO, "index : %d, width: %04u, height: %04u, data width: %d", i, width, height, data_width);

        /* find existing mode index from sensor_info with corresponding resolution */
        for ( j = 0; j < sensor_info->num_modes; j++ ) {
            if ( ( sensor_info->mode[j].width == width ) && ( sensor_info->mode[j].height == height ) ) {
                break;
            }
        }

        /* update mode */
        if ( sensor_info->mode[j].num_sub_modes < V4L2_SENSOR_INFO_SUB_MODES_MAX ) {

            const uint8_t cur_sub_mode = sensor_info->mode[j].num_sub_modes;

            sensor_info->mode[j].width = width;
            sensor_info->mode[j].height = height;
            sensor_info->mode[j].sub_mode[cur_sub_mode].exposures = exposures;
            sensor_info->mode[j].sub_mode[cur_sub_mode].fps = fps;
            sensor_info->mode[j].sub_mode[cur_sub_mode].data_width = data_width;
            sensor_info->mode[j].sub_mode[cur_sub_mode].num_channels = num_channels;
            sensor_info->mode[j].sub_mode[cur_sub_mode].sensor_preset = i;
            sensor_info->mode[j].num_sub_modes++;

            // Check if this mode is a new one
            if ( sensor_info->num_modes <= j ) {
                sensor_info->num_modes++;
            }
        } else {
            LOG( LOG_ERR, "Found number of sub modes (%u) for mode (%d) is out of reserved range ( V4L2_SENSOR_INFO_SUB_MODES_MAX = %u )",
                 sensor_info->mode[j].num_sub_modes, j, V4L2_SENSOR_INFO_SUB_MODES_MAX );
        }
    }

    // Update sensor info current mode and submode
    fw_intf_isp_update_sensor_info_mode( ctx_id, sensor_info );

    // Print out sensor info structure
    LOG( LOG_INFO, "/* dump sensor info structure ----------------------------------" );
    for ( i = 0; i < sensor_info->num_modes; i++ ) {
        LOG( LOG_INFO, "   |--mode: %02d, width: %04u, height: %04u", i, sensor_info->mode[i].width, sensor_info->mode[i].height );
        for ( j = 0; j < sensor_info->mode[i].num_sub_modes; j++ )
            LOG( LOG_INFO, "      |--sub mode %02d: fps: %u, exposures: %u, channels: %u, data width: %u, sensor preset: %u",
                 j,
                 sensor_info->mode[i].sub_mode[j].fps / 256,
                 sensor_info->mode[i].sub_mode[j].exposures,
                 sensor_info->mode[i].sub_mode[j].num_channels,
                 sensor_info->mode[i].sub_mode[j].data_width,
                 sensor_info->mode[i].sub_mode[j].sensor_preset );
    }
    LOG( LOG_INFO, "--------------------------------------------------------------*/" );

#else
    /* Return default settings (1080p) */
    LOG( LOG_INFO, "Sensor APIs not found, initializing sensor info structure with default values (1080p)" );
    sensor_info->num_modes = 1;
    sensor_info->cur_mode = 0;
    sensor_info->mode[0].num_sub_modes = 1;
    sensor_info->mode[0].cur_sub_mode = 0;
    sensor_info->mode[0].sub_mode[0].fps = 30 * 256;
    sensor_info->mode[0].sub_mode[0].exposures = 1;
    sensor_info->mode[0].sub_mode[0].num_channels = 1;
    sensor_info->mode[0].sub_mode[0].data_width = 12;
    sensor_info->mode[0].sub_mode[0].sensor_preset = 0;
    sensor_info->mode[0].width = 1920;
    sensor_info->mode[0].height = 1080;
#endif

    return 0;
}

/**
 * fw-interface per-stream control interface
 */
int fw_intf_stream_start( uint32_t ctx_id, isp_v4l2_stream_type_t stream_type, unsigned long stream_open_mask )
{
    LOG( LOG_INFO, "Starting stream, context id: %u, stream type: %d", ctx_id, stream_type );

    int rc = 0;

    // Get Frame Streamer stream types. For non-m2m streams out will be AFRAME_TYPE_UNKNOWN
    const aframe_type_t frame_type_cap = isp_fw_stream_type_to_aframe_type( stream_type, V4L2_STREAM_DIRECTION_CAP );
    const aframe_type_t frame_type_out = isp_fw_stream_type_to_aframe_type( stream_type, V4L2_STREAM_DIRECTION_OUT );

    // If stream type is correct and supported disable it
    if ( frame_type_cap != AFRAME_TYPE_UNKNOWN ) {
        // Disable frame streamer stream
        rc |= frame_stream_set_param( ctx_id, frame_type_cap, FRAME_STREAM_PARAM_IS_ENABLED, 1 );
    }

    if ( frame_type_out != AFRAME_TYPE_UNKNOWN ) {
        // Disable frame streamer stream
        rc |= frame_stream_set_param( ctx_id, frame_type_out, FRAME_STREAM_PARAM_IS_ENABLED, 1 );
    }

    // Check if ISP context has been already started and start if necessary
    if ( !fw_intf_is_context_started( ctx_id ) ) {

        // Check if device has m2m stream opened and configure ISP V4L2 interface type
        const uint32_t v4l2_interface_mode = ( ( stream_open_mask & ( 1UL << V4L2_STREAM_TYPE_M2M ) ) ? V4L2_INTERFACE_MODE_M2M : V4L2_INTERFACE_MODE_CAPTURE );

        if ( isp_fw_set_v4l2_interface_mode( ctx_id, v4l2_interface_mode ) ) {
            LOG( LOG_ERR, "Error. Failed to configure V4L2 interface mode, stream type: %d, context id: %d.", stream_type, ctx_id );
        }

        rc |= fw_intf_context_config( ctx_id );

        // Wait until ISP context is actually configured
        uint32_t iterations = 0;
        while ( ( !fw_intf_is_context_configured( ctx_id ) ) && ( iterations < 1000 ) ) {
            msleep( 1 );
            iterations++;
        }

        if ( iterations < 1000 ) {
            LOG( LOG_INFO, "Stream, context id: %d, type: %d configured after %u ms", ctx_id, stream_type, iterations );
        } else {
            LOG( LOG_ERR, "Error. Failed to configure stream, context id: %d, type: %d. Timeout reached after: %u ms", ctx_id, stream_type, iterations );
            rc = -1;
        }

        rc |= fw_intf_context_start( ctx_id );

        iterations = 0;
        while ( ( !fw_intf_is_context_started( ctx_id ) ) && ( iterations < 1000 ) ) {
            msleep( 1 );
            iterations++;
        }

        if ( iterations < 1000 ) {
            LOG( LOG_INFO, "Stream, context id: %d, type: %d started after %u ms", ctx_id, stream_type, iterations );
        } else {
            LOG( LOG_ERR, "Error. Failed to start stream, context id: %d, type: %d. Timeout reached after: %u ms", ctx_id, stream_type, iterations );
            rc = -1;
        }
    } else {
		LOG (LOG_NOTICE, "context already started");
	}

    return rc;
}

void fw_intf_stream_stop( uint32_t ctx_id, isp_v4l2_stream_type_t stream_type, unsigned long stream_on_mask )
{
    LOG( LOG_INFO, "Stopping stream, context id: %u, stream type: %d, is last stream: %s",
         ctx_id, stream_type, ( ( stream_on_mask == 0 ) ? "yes" : "no" ) );

    // Get Frame Streamer stream types. For non-m2m streams out will be AFRAME_TYPE_UNKNOWN
    const aframe_type_t frame_type_cap = isp_fw_stream_type_to_aframe_type( stream_type, V4L2_STREAM_DIRECTION_CAP );
    const aframe_type_t frame_type_out = isp_fw_stream_type_to_aframe_type( stream_type, V4L2_STREAM_DIRECTION_OUT );

    // If stream type is correct and supported disable it
    if ( frame_type_cap != AFRAME_TYPE_UNKNOWN ) {
        // Disable frame streamer stream
        frame_stream_set_param( ctx_id, frame_type_cap, FRAME_STREAM_PARAM_IS_ENABLED, 0 );
    }

    // If stream type is correct and supported disable it
    if ( frame_type_out != AFRAME_TYPE_UNKNOWN ) {
        // Disable frame streamer stream
        frame_stream_set_param( ctx_id, frame_type_out, FRAME_STREAM_PARAM_IS_ENABLED, 0 );
    }

    // If this is not last stream, wait until ISP returns V4L2 buffers for this stream
    if ( stream_on_mask != 0 ) {
        msleep( 50 );
    }

    // In case this is the last stream and ISP context is not stopped, then stop it
    if ( stream_on_mask == 0 ) {
        if ( !fw_intf_is_context_stopped( ctx_id ) ) {
            fw_intf_context_stop( ctx_id );

            // Wait until ISP context is actually stopped so we can be sure that all V4L2 buffers released
            uint32_t iterations = 0;
            while ( ( !fw_intf_is_context_stopped( ctx_id ) ) && ( iterations < 1000 ) ) {
                msleep( 1 );
                iterations++;
            }

            if ( iterations < 1000 ) {
                LOG( LOG_INFO, "Stream, context id: %d, type: %d stopped after %u ms", ctx_id, stream_type, iterations );
            } else {
                LOG( LOG_ERR, "Error. Failed to stop stream, context id: %d, type: %d. Timeout reached after: %u ms",
                     ctx_id, stream_type, iterations );
            }

            if ( isp_fw_set_v4l2_interface_mode( ctx_id, V4L2_INTERFACE_MODE_NONE ) ) {
                LOG( LOG_ERR, "Error. Failed to configure V4L2 interface mode, stream type: %d, context id: %d.", stream_type, ctx_id );
            }
        } else {
            LOG( LOG_INFO, "Stream, context id: %d, type: %d has been already stopped", ctx_id, stream_type );
        }
    }
}

/**
 * fw-interface per-stream config interface
 */
int fw_intf_stream_set_resolution( uint32_t ctx_id, isp_v4l2_sensor_info *sensor_info,
                                   isp_v4l2_stream_type_t stream_type, uint32_t *width, uint32_t *height )
{
    if ( ( sensor_info == NULL ) || ( width == NULL ) || ( height == NULL ) ) {
        LOG( LOG_ERR, "Invalid parameter. One or several parameters are NULL" );
        return -EINVAL;
    }

    /*
     * Stream type
     *   - V4L2_STREAM_TYPE_OUT: directly update sensor resolution while crop functionality is in development.
     *   - V4L2_STREAM_TYPE_RAW: directly update sensor resolution since FR doesn't have down-scaler.
     */
    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to set stream resolution. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }

    LOG( LOG_INFO, "stream_set_resolution, context id: %u, stream type: %d, width: %d, height: %d",
         ctx_id, stream_type, *width, *height );

    if ( stream_type == V4L2_STREAM_TYPE_RAW  || stream_type == V4L2_STREAM_TYPE_OUT) {
#if defined( TSENSOR ) && defined( SENSOR_PRESET )

        uint32_t sensor_preset = 0;
        uint32_t target_width = *width;
        uint32_t target_height = *height;

        uint32_t current_width = 0, current_height = 0;

        //check if we need to change sensor preset
        acamera_command( ctx_id, TSENSOR, SENSOR_WIDTH, 0, COMMAND_GET, &current_width );
        acamera_command( ctx_id, TSENSOR, SENSOR_HEIGHT, 0, COMMAND_GET, &current_height );

        LOG( LOG_INFO, "stream_set_resolution, target resolution: %dx%d, current resolution: %dx%d",
             target_width, target_height, current_width, current_height );

        if ( ( current_width != target_width ) || ( current_height != target_height ) ) {

            uint32_t mode, sub_mode = 0, fps = 0;
            // Search sensor info for the mode with matching resolution
            for ( mode = 0; mode < sensor_info->num_modes; mode++ ) {
                if ( sensor_info->mode[mode].width == target_width && sensor_info->mode[mode].height == target_height ) {

                    // Find sub mode with the highest FPS value.
                    // This should be changed in the future to pick required resolution from application
                    for ( sub_mode = 0; sub_mode < sensor_info->mode[mode].num_sub_modes; sub_mode++ ) {
                        if ( sensor_info->mode[mode].sub_mode[sub_mode].fps > fps ) {
                            fps = sensor_info->mode[mode].sub_mode[sub_mode].fps;
                            sensor_preset = sensor_info->mode[mode].sub_mode[sub_mode].sensor_preset;
                        }
                    }
                    break;
                }
            }

            // Check if we found requested resolution in the sensor info
            if ( mode >= sensor_info->num_modes ) {
                LOG( LOG_ERR, "Unsupported resolution: %dx%d requested, reverting to current resolution: %dx%d",
                     target_width, target_height, current_width, current_height );
                *width = current_width;
                *height = current_height;
                return 0;
            }

            /* set sensor resolution preset */
            LOG( LOG_INFO, "Setting new sensor resolution: %dx%d, sensor preset: %d, fps: %d",
                 target_width, target_height, sensor_preset, fps / 256 );

            uint32_t ret_val;
            uint8_t rc = acamera_command( ctx_id, TSENSOR, SENSOR_PRESET, sensor_preset, COMMAND_SET, &ret_val );
            if ( rc ) {
                LOG( LOG_ERR, "Failed to set sensor preset to: %u, rc: %u.", sensor_preset, rc );
                return rc;
            }

            // Update sensor info current mode and sub mode
            sensor_info->cur_mode = mode;
            sensor_info->mode[mode].cur_sub_mode = sub_mode;

        } else {
            acamera_command( ctx_id, TSENSOR, SENSOR_PRESET, 0, COMMAND_GET, &sensor_preset );
            LOG( LOG_INFO, "No resolution change required, using current sensor resolution: %dx%d, sensor preset: %d",
                 target_width, target_height, sensor_preset );
        }
#endif
    }

    return 0;
}

int fw_intf_stream_set_output_format( uint32_t ctx_id, isp_v4l2_stream_type_t stream_type, isp_v4l2_stream_direction_t stream_direction, uint32_t format )
{

#if defined( TIMAGE ) && defined( OUTPUT_FORMAT_ID )
    uint32_t value;
    const char *string_value;

    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to set output format. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }

    if ( ( stream_type == V4L2_STREAM_TYPE_OUT ) || ( ( stream_type == V4L2_STREAM_TYPE_M2M ) && ( stream_direction == V4L2_STREAM_DIRECTION_CAP ) ) ) {
        switch ( format ) {
        case V4L2_PIX_FMT_ABGR32:
            value = OF_MODE_BGRA32;
            string_value = "OF_MODE_BGRA32";
            break;

        case V4L2_PIX_FMT_BGR24:
            value = OF_MODE_BGR888;
            string_value = "OF_MODE_BGR888";
            break;

        case V4L2_PIX_FMT_RGB24:
            value = OF_MODE_RGB888;
            string_value = "OF_MODE_RGB888";
            break;

        case V4L2_PIX_FMT_NV12:
            value = OF_MODE_Y8UV88_2X2;
            string_value = "OF_MODE_Y8UV88_2X2";
            break;

        case ISP_V4L2_PIX_FMT_NULL:
            value = OF_MODE_DISABLE;
            string_value = "OF_MODE_DISABLE";
            break;

        default:
            LOG( LOG_ERR, "Requested output format: 0x%x is not supported by firmware", format );
            return -1;
            break;
        }

        uint32_t ret_val = 0;

        uint8_t rc = acamera_command( ctx_id, TIMAGE, OUTPUT_FORMAT_ID, value, COMMAND_SET, &ret_val );
        LOG( LOG_INFO, "Output format for stream type: %d was set to %s (0x%x)", stream_type, string_value, format );
        if ( rc ) {
            LOG( LOG_ERR, "TIMAGE->OUTPUT_FORMAT_ID API failed, value: 0x%x, rc: %d", value, rc );
        }
    }

#else
    LOG( LOG_ERR, "Failed to set output format. Firmware API OUTPUT_FORMAT_ID is missing" );
#endif

    return 0;
}


/**
 * Internal handler for control interface functions
 */
static bool isp_fw_do_validate_control( uint32_t id )
{
    return 1;
}

static int isp_fw_do_set_test_pattern( uint32_t ctx_id, int enable )
{
#if defined( TSYSTEM ) && defined( TEST_PATTERN_ENABLE_ID )
    int result;
    uint32_t ret_val = 0;

    LOG( LOG_INFO, "test_pattern: %d.", enable );

    if ( enable < 0 )
        return -EIO;

    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to set test pattern state. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }

    result = acamera_command( ctx_id, TSYSTEM, TEST_PATTERN_ENABLE_ID, enable ? 1 : 0, COMMAND_SET, &ret_val );
    if ( result ) {
        LOG( LOG_ERR, "Failed to set TEST_PATTERN_ENABLE_ID to %u, ret_value: %d.", enable, result );
        return result;
    }
#endif

    return 0;
}

static int isp_fw_do_set_test_pattern_type( uint32_t ctx_id, int pattern_type )
{
#if defined( TSYSTEM ) && defined( TEST_PATTERN_MODE_ID )
    int result;
    uint32_t ret_val = 0;

    LOG( LOG_INFO, "test_pattern_type: %d.", pattern_type );

    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to set test pattern type. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }

    result = acamera_command( ctx_id, TSYSTEM, TEST_PATTERN_MODE_ID, pattern_type, COMMAND_SET, &ret_val );
    if ( result ) {
        LOG( LOG_ERR, "Failed to set TEST_PATTERN_MODE_ID to %d, ret_value: %d.", pattern_type, result );
        return result;
    }
#endif

    return 0;
}

static int isp_fw_do_set_cmd( uint32_t ctx_id, uint8_t command_type, uint8_t command, uint32_t value )
{
    int rc = 0;
    uint32_t ret_val = 0;

    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to process set command. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }

    rc = acamera_command( ctx_id, command_type, command, value, COMMAND_SET, &ret_val );
    return rc;
}

static int isp_fw_do_get_cmd( uint32_t ctx_id, uint8_t command_type, uint8_t command, uint32_t *ret_val )
{
    int rc = 0;

    if ( !fw_intf_is_context_initialised( ctx_id ) ) {
        LOG( LOG_ERR, "Failed to process get command. Context (id: %u) is not initialised.", ctx_id );
        return -EBUSY;
    }

    rc = acamera_command( ctx_id, command_type, command, 0, COMMAND_GET, ret_val );
    return rc;
}

/**
 * fw_interface config interface
 */
bool fw_intf_validate_control( uint32_t id )
{
    return isp_fw_do_validate_control( id );
}

int fw_intf_set_test_pattern( uint32_t ctx_id, int val )
{
    return isp_fw_do_set_test_pattern( ctx_id, val );
}

int fw_intf_set_test_pattern_type( uint32_t ctx_id, int val )
{
    return isp_fw_do_set_test_pattern_type( ctx_id, val );
}

int fw_intf_set_system_freeze_firmware( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_FREEZE_FIRMWARE )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_FREEZE_FIRMWARE, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_FREEZE_FIRMWARE to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_sensor_info_preset( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_PRESET )
    rc = isp_fw_do_set_cmd( ctx_id, TSENSOR, SENSOR_INFO_PRESET, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SENSOR_INFO_PRESET to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}


int fw_intf_set_sensor_preset( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_PRESET )
    rc = isp_fw_do_set_cmd( ctx_id, TSENSOR, SENSOR_PRESET, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SENSOR_PRESET to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_exposure( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_EXPOSURE )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_EXPOSURE, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_EXPOSURE to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_integration_time( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_INTEGRATION_TIME )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_INTEGRATION_TIME, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_INTEGRATION_TIME to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_max_integration_time( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_MAX_INTEGRATION_TIME )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_MAX_INTEGRATION_TIME, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_MAX_INTEGRATION_TIME to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_sensor_analog_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_SENSOR_ANALOG_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_SENSOR_ANALOG_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_SENSOR_ANALOG_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_sensor_digital_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_isp_digital_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_ISP_DIGITAL_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_ISP_DIGITAL_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_ISP_DIGITAL_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_directional_sharpening( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_DIRECTIONAL_SHARPENING )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_DIRECTIONAL_SHARPENING, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_DIRECTIONAL_SHARPENING to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_un_directional_sharpening( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_exposure_ratio( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_EXPOSURE_RATIO )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_EXPOSURE_RATIO, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_EXPOSURE_RATIO to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_awb( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_AWB )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_AWB, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_AWB to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_antiflicker_enable( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_ANTIFLICKER_ENABLE )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_ANTIFLICKER_ENABLE, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_ANTIFLICKER_ENABLE to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_manual_saturation( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_SATURATION )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_SATURATION, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MANUAL_SATURATION to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_max_exposure_ratio( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_EXPOSURE_RATIO )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_EXPOSURE_RATIO, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MAX_EXPOSURE_RATIO to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_exposure( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_EXPOSURE )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_EXPOSURE, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_EXPOSURE to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_integration_time( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_INTEGRATION_TIME )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_INTEGRATION_TIME, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_INTEGRATION_TIME to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_exposure_ratio( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_EXPOSURE_RATIO )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_EXPOSURE_RATIO, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_EXPOSURE_RATIO to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_max_integration_time( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_INTEGRATION_TIME )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_INTEGRATION_TIME, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MAX_INTEGRATION_TIME to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_sensor_analog_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SENSOR_ANALOG_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_SENSOR_ANALOG_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_SENSOR_ANALOG_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_max_sensor_analog_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_SENSOR_ANALOG_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_SENSOR_ANALOG_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MAX_SENSOR_ANALOG_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_sensor_digital_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SENSOR_DIGITAL_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_SENSOR_DIGITAL_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_SENSOR_DIGITAL_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_max_sensor_digital_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_SENSOR_DIGITAL_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_SENSOR_DIGITAL_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MAX_SENSOR_DIGITAL_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_isp_digital_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_ISP_DIGITAL_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_ISP_DIGITAL_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_ISP_DIGITAL_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_max_isp_digital_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_ISP_DIGITAL_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_ISP_DIGITAL_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MAX_ISP_DIGITAL_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_directional_sharpening_target( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_DIRECTIONAL_SHARPENING_TARGET )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_DIRECTIONAL_SHARPENING_TARGET, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_DIRECTIONAL_SHARPENING_TARGET to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_un_directional_sharpening_target( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_awb_red_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_AWB_RED_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_AWB_RED_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_AWB_RED_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_awb_blue_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_AWB_BLUE_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_AWB_BLUE_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_AWB_BLUE_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_awb_cct( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_AWB_CCT )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_AWB_CCT, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_AWB_CCT to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_saturation_target( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SATURATION_TARGET )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_SATURATION_TARGET, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_SATURATION_TARGET to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_anti_flicker_frequency( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_ANTI_FLICKER_FREQUENCY )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_ANTI_FLICKER_FREQUENCY, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_ANTI_FLICKER_FREQUENCY to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_iridix_digital_gain( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_IRIDIX_DIGITAL_GAIN )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_IRIDIX_DIGITAL_GAIN, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_IRIDIX_DIGITAL_GAIN to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_sinter_threshold_target( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SINTER_THRESHOLD_TARGET )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_SINTER_THRESHOLD_TARGET, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_SINTER_THRESHOLD_TARGET to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_minimum_iridix_strength( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MINIMUM_IRIDIX_STRENGTH )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MINIMUM_IRIDIX_STRENGTH, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MINIMUM_IRIDIX_STRENGTH to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_maximum_iridix_strength( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAXIMUM_IRIDIX_STRENGTH )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_MAXIMUM_IRIDIX_STRENGTH, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_MAXIMUM_IRIDIX_STRENGTH to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_iridix_strength_target( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_IRIDIX_STRENGTH_TARGET )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_IRIDIX_STRENGTH_TARGET, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_IRIDIX_STRENGTH_TARGET to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_logger_level( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_LOGGER_LEVEL )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_LOGGER_LEVEL, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_LOGGER_LEVEL to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_logger_mask( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_LOGGER_MASK )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, SYSTEM_LOGGER_MASK, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set SYSTEM_LOGGER_MASK to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_iridix( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_IRIDIX )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_IRIDIX, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_IRIDIX to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_sinter( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_SINTER )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_SINTER, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_SINTER to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_frame_stitch( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_FRAME_STITCH )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_FRAME_STITCH, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_FRAME_STITCH to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_raw_frontend( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_RAW_FRONTEND )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_RAW_FRONTEND, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_RAW_FRONTEND to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_black_level( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_BLACK_LEVEL )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_BLACK_LEVEL, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_BLACK_LEVEL to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_shading( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_SHADING )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_SHADING, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_SHADING to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_demosaic( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_DEMOSAIC )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_DEMOSAIC, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_DEMOSAIC to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_force_bist_mismatch( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_FORCE_BIST_MISMATCH )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_FORCE_BIST_MISMATCH, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_FORCE_BIST_MISMATCH to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_m2m_process_request( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( M2M_PROCESS_REQUEST )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, M2M_PROCESS_REQUEST, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set M2M_PROCESS_REQUEST to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_system_v4l2_interface_mode( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( V4L2_INTERFACE_MODE )
    rc = isp_fw_do_set_cmd( ctx_id, TSYSTEM, V4L2_INTERFACE_MODE, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set V4L2_INTERFACE_MODE to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_isp_modules_manual_cnr( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_CNR )
    rc = isp_fw_do_set_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_CNR, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ISP_MODULES_MANUAL_CNR to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_output_format_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_FORMAT_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, OUTPUT_FORMAT_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set OUTPUT_FORMAT_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_output_format_manual_cfg_apply_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}
#if ( ISP_RTL_VERSION_R == 2 )
int fw_intf_set_raw_scaler_enable_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RAW_SCALER_ENABLE_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_RAW_SCALER_ENABLE_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_RAW_SCALER_ENABLE_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_raw_scaler_width_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RAW_SCALER_WIDTH_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_RAW_SCALER_WIDTH_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_RAW_SCALER_WIDTH_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_raw_scaler_height_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RAW_SCALER_HEIGHT_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_RAW_SCALER_HEIGHT_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_RAW_SCALER_HEIGHT_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_rgb_scaler_enable_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RGB_SCALER_ENABLE_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_RGB_SCALER_ENABLE_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_RGB_SCALER_ENABLE_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_rgb_scaler_width_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RGB_SCALER_WIDTH_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_RGB_SCALER_WIDTH_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_RGB_SCALER_WIDTH_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_rgb_scaler_height_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RGB_SCALER_HEIGHT_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_RGB_SCALER_HEIGHT_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_RGB_SCALER_HEIGHT_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

#endif //( ISP_RTL_VERSION_R == 2 )
int fw_intf_set_output_axi1_format_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_AXI1_FORMAT_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, OUTPUT_AXI1_FORMAT_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set OUTPUT_AXI1_FORMAT_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_output_axi2_format_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_AXI2_FORMAT_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, OUTPUT_AXI2_FORMAT_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set OUTPUT_AXI2_FORMAT_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_output_axi3_format_id( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_AXI3_FORMAT_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, OUTPUT_AXI3_FORMAT_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set OUTPUT_AXI3_FORMAT_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_image_crop_xoffset( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_XOFFSET_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_CROP_XOFFSET_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_CROP_XOFFSET_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_image_crop_yoffset( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_YOFFSET_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_CROP_YOFFSET_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_CROP_YOFFSET_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_image_crop_height( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_HEIGHT_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_CROP_HEIGHT_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_CROP_HEIGHT_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}


int fw_intf_set_image_crop_width( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_WIDTH_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_CROP_WIDTH_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_CROP_WIDTH_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_image_crop_enable( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_ENABLE_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TIMAGE, IMAGE_CROP_ENABLE_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set IMAGE_CROP_ENABLE_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_ae_compensation( uint32_t ctx_id, int val )
{
    int rc = -1;
    return rc;
}

int fw_intf_set_active_context( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TGENERAL ) && defined( ACTIVE_CONTEXT )
    rc = isp_fw_do_set_cmd( ctx_id, TGENERAL, ACTIVE_CONTEXT, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set ACTIVE_CONTEXT to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_register_value( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_VALUE_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TREGISTERS, REGISTERS_VALUE_ID, (uint32_t)val );

    if ( rc ) {
        LOG( LOG_ERR, "Failed to set REGISTERS_VALUE_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_register_source( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_SOURCE_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TREGISTERS, REGISTERS_SOURCE_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set REGISTERS_VALUE_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_register_size( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_SIZE_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TREGISTERS, REGISTERS_SIZE_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set REGISTERS_SIZE_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_set_register_address( uint32_t ctx_id, int val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_ADDRESS_ID )
    rc = isp_fw_do_set_cmd( ctx_id, TREGISTERS, REGISTERS_ADDRESS_ID, (uint32_t)val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to set REGISTERS_ADDRESS_ID to %d, rc: %d.", val, rc );
    }
#endif

    return rc;
}

int fw_intf_get_test_pattern( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( TEST_PATTERN_ENABLE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, TEST_PATTERN_ENABLE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get TEST_PATTERN_ENABLE_ID, rc: %d.", rc );
    } else {
        *ret_val = ( *ret_val == 1 ) ? 1 : 0;
    }
#endif

    return rc;
}

int fw_intf_get_system_m2m_process_request( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( M2M_PROCESS_REQUEST )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, M2M_PROCESS_REQUEST, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get M2M_PROCESS_REQUEST, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_v4l2_interface_mode( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( V4L2_INTERFACE_MODE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, V4L2_INTERFACE_MODE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get V4L2_INTERFACE_MODE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_test_pattern_type( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( TEST_PATTERN_MODE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, TEST_PATTERN_MODE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get TEST_PATTERN_MODE_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_supported_presets( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_SUPPORTED_PRESETS )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_SUPPORTED_PRESETS, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_SUPPORTED_PRESETS, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_preset( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_PRESET )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_PRESET, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_PRESET, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_wdr_mode( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_WDR_MODE )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_WDR_MODE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_WDR_MODE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_streaming( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_STREAMING )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_STREAMING, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_STREAMING, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_exposures( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_EXPOSURES )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_EXPOSURES, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_EXPOSURES, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_fps( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_FPS )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_FPS, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_FPS, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_width( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_WIDTH )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_WIDTH, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_WIDTH, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_height( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_HEIGHT )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_HEIGHT, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_HEIGHT, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_preset( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_PRESET )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_PRESET, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_PRESET, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_wdr_mode( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_WDR_MODE )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_WDR_MODE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_WDR_MODE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_fps( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_FPS )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_FPS, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_FPS, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_width( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_WIDTH )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_WIDTH, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_WIDTH, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_height( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_HEIGHT )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_HEIGHT, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_HEIGHT, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_exposures( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_EXPOSURES )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_EXPOSURES, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_EXPOSURES, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_channels( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_CHANNELS )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_CHANNELS, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_CHANNELS, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_sensor_info_data_width( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INFO_DATA_WIDTH )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INFO_DATA_WIDTH, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INFO_DATA_WIDTH, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_freeze_firmware( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_FREEZE_FIRMWARE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_FREEZE_FIRMWARE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_FREEZE_FIRMWARE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_exposure( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_EXPOSURE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_EXPOSURE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_EXPOSURE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_max_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_MAX_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_MAX_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_MAX_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_sensor_analog_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_SENSOR_ANALOG_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_SENSOR_ANALOG_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_SENSOR_ANALOG_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_sensor_digital_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_isp_digital_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_ISP_DIGITAL_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_ISP_DIGITAL_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_ISP_DIGITAL_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_directional_sharpening( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_DIRECTIONAL_SHARPENING )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_DIRECTIONAL_SHARPENING, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_DIRECTIONAL_SHARPENING, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_un_directional_sharpening( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_exposure_ratio( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_EXPOSURE_RATIO )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_EXPOSURE_RATIO, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_EXPOSURE_RATIO, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_awb( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_AWB )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_AWB, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_AWB, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_antiflicker_enable( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_ANTIFLICKER_ENABLE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_ANTIFLICKER_ENABLE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_ANTIFLICKER_ENABLE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_manual_saturation( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MANUAL_SATURATION )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MANUAL_SATURATION, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MANUAL_SATURATION, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_max_exposure_ratio( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_EXPOSURE_RATIO )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_EXPOSURE_RATIO, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MAX_EXPOSURE_RATIO, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_max_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MAX_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_short_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SHORT_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_SHORT_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_SHORT_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_middle_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MIDDLE_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MIDDLE_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MIDDLE_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_middle2_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MIDDLE2_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MIDDLE2_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MIDDLE2_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_long_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_LONG_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_LONG_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_LONG_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_integration_time_precision( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_INTEGRATION_TIME_PRECISION )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_INTEGRATION_TIME_PRECISION, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_INTEGRATION_TIME_PRECISION, rc: %d.", rc );
    }
#endif

    return rc;
}


int fw_intf_get_system_max_sensor_analog_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_SENSOR_ANALOG_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_SENSOR_ANALOG_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MAX_SENSOR_ANALOG_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_max_sensor_digital_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_SENSOR_DIGITAL_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_SENSOR_DIGITAL_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MAX_SENSOR_DIGITAL_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_max_isp_digital_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAX_ISP_DIGITAL_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MAX_ISP_DIGITAL_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MAX_ISP_DIGITAL_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_directional_sharpening_target( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_DIRECTIONAL_SHARPENING_TARGET )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_DIRECTIONAL_SHARPENING_TARGET, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_DIRECTIONAL_SHARPENING_TARGET, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_un_directional_sharpening_target( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_manual_iridix( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_IRIDIX )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_IRIDIX, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_IRIDIX, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_manual_sinter( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_SINTER )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_SINTER, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_SINTER, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_manual_frame_stitch( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_FRAME_STITCH )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_FRAME_STITCH, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_FRAME_STITCH, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_manual_raw_frontend( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_RAW_FRONTEND )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_RAW_FRONTEND, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_RAW_FRONTEND, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_manual_black_level( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_BLACK_LEVEL )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_BLACK_LEVEL, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_BLACK_LEVEL, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_manual_shading( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_SHADING )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_SHADING, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_SHADING, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_force_bist_mismatch( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_FORCE_BIST_MISMATCH )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_FORCE_BIST_MISMATCH, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_FORCE_BIST_MISMATCH, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_isp_modules_manual_demosaic( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_DEMOSAIC )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_DEMOSAIC, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_DEMOSAIC, rc: %d.", rc );
    }
#endif

    return rc;
}

#if ( ISP_RTL_VERSION_R == 2 )
int fw_intf_get_isp_modules_manual_cnr( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TISP_MODULES ) && defined( ISP_MODULES_MANUAL_CNR )
    rc = isp_fw_do_get_cmd( ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_CNR, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ISP_MODULES_MANUAL_CNR, rc: %d.", rc );
    }
#endif

    return rc;
}
#endif

int fw_intf_get_image_crop_xoffset( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_XOFFSET_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_CROP_XOFFSET_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_CROP_XOFFSET_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_image_crop_yoffset( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_YOFFSET_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_CROP_YOFFSET_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_CROP_YOFFSET_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_output_format_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_FORMAT_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, OUTPUT_FORMAT_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get OUTPUT_FORMAT_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_output_format_manual_cfg_apply_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID, rc: %d.", rc );
    }
#endif
    return rc;
}

int fw_intf_get_output_axi1_format_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_AXI1_FORMAT_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, OUTPUT_AXI1_FORMAT_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get OUTPUT_AXI1_FORMAT_ID, rc: %d.", rc );
    }
#endif
    return rc;
}
#if ( ISP_RTL_VERSION_R == 2 )
int fw_intf_get_raw_scaler_width_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RAW_SCALER_WIDTH_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_RAW_SCALER_WIDTH_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_RAW_SCALER_WIDTH_ID, rc: %d.", rc );
    }
#endif
    return rc;
}

int fw_intf_get_raw_scaler_height_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RAW_SCALER_HEIGHT_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_RAW_SCALER_HEIGHT_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_RAW_SCALER_HEIGHT_ID, rc: %d.", rc );
    }
#endif
    return rc;
}


int fw_intf_get_raw_scaler_enable_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RAW_SCALER_ENABLE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_RAW_SCALER_ENABLE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_RAW_SCALER_ENABLE_ID, rc: %d.", rc );
    }
#endif
    return rc;
}

int fw_intf_get_rgb_scaler_width_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RGB_SCALER_WIDTH_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_RGB_SCALER_WIDTH_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_RGB_SCALER_WIDTH_ID, rc: %d.", rc );
    }
#endif
    return rc;
}

int fw_intf_get_rgb_scaler_height_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RGB_SCALER_HEIGHT_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_RGB_SCALER_HEIGHT_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_RGB_SCALER_HEIGHT_ID, rc: %d.", rc );
    }
#endif
    return rc;
}

int fw_intf_get_rgb_scaler_enable_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_RGB_SCALER_ENABLE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_RGB_SCALER_ENABLE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_RGB_SCALER_ENABLE_ID, rc: %d.", rc );
    }
#endif
    return rc;
}
#endif

int fw_intf_get_output_axi2_format_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_AXI2_FORMAT_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, OUTPUT_AXI2_FORMAT_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get OUTPUT_AXI2_FORMAT_ID, rc: %d.", rc );
    }
#endif
    return rc;
}

int fw_intf_get_output_axi3_format_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( OUTPUT_AXI3_FORMAT_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, OUTPUT_AXI3_FORMAT_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get OUTPUT_AXI3_FORMAT_ID, rc: %d.", rc );
    }
#endif
    return rc;
}

int fw_intf_get_image_crop_height( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_HEIGHT_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_CROP_HEIGHT_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_CROP_HEIGHT_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_image_crop_width( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_WIDTH_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_CROP_WIDTH_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_CROP_WIDTH_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_image_crop_enable( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TIMAGE ) && defined( IMAGE_CROP_ENABLE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TIMAGE, IMAGE_CROP_ENABLE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get IMAGE_CROP_ENABLE_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_exposure( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_EXPOSURE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_EXPOSURE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_EXPOSURE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_integration_time( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_INTEGRATION_TIME )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_INTEGRATION_TIME, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_INTEGRATION_TIME, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_exposure_ratio( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_EXPOSURE_RATIO )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_EXPOSURE_RATIO, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_EXPOSURE_RATIO, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_sensor_analog_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SENSOR_ANALOG_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_SENSOR_ANALOG_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_SENSOR_ANALOG_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_sensor_digital_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SENSOR_DIGITAL_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_SENSOR_DIGITAL_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_SENSOR_DIGITAL_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_isp_digital_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_ISP_DIGITAL_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_ISP_DIGITAL_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_ISP_DIGITAL_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_awb_red_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_AWB_RED_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_AWB_RED_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_AWB_RED_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_awb_blue_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_AWB_BLUE_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_AWB_BLUE_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_AWB_BLUE_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_awb_cct( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_AWB_CCT )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_AWB_CCT, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_AWB_CCT, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_saturation_target( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SATURATION_TARGET )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_SATURATION_TARGET, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_SATURATION_TARGET, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_anti_flicker_frequency( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_ANTI_FLICKER_FREQUENCY )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_ANTI_FLICKER_FREQUENCY, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_ANTI_FLICKER_FREQUENCY, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_iridix_digital_gain( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_IRIDIX_DIGITAL_GAIN )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_IRIDIX_DIGITAL_GAIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_IRIDIX_DIGITAL_GAIN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_sinter_threshold_target( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_SINTER_THRESHOLD_TARGET )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_SINTER_THRESHOLD_TARGET, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_SINTER_THRESHOLD_TARGET, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_minimum_iridix_strength( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MINIMUM_IRIDIX_STRENGTH )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MINIMUM_IRIDIX_STRENGTH, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MINIMUM_IRIDIX_STRENGTH, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_maximum_iridix_strength( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_MAXIMUM_IRIDIX_STRENGTH )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_MAXIMUM_IRIDIX_STRENGTH, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_MAXIMUM_IRIDIX_STRENGTH, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_iridix_strength_target( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_IRIDIX_STRENGTH_TARGET )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_IRIDIX_STRENGTH_TARGET, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_IRIDIX_STRENGTH_TARGET, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_buffer_data_type( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( BUFFER_DATA_TYPE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, BUFFER_DATA_TYPE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get BUFFER_DATA_TYPE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_status_info_exposure_log2( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSTATUS ) && defined( STATUS_INFO_EXPOSURE_LOG2_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TSTATUS, STATUS_INFO_EXPOSURE_LOG2_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get STATUS_INFO_EXPOSURE_LOG2_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_cmd_interface_mode( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( CMD_INTERFACE_MODE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, CMD_INTERFACE_MODE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get CMD_INTERFACE_MODE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_logger_level( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_LOGGER_LEVEL )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_LOGGER_LEVEL, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_LOGGER_LEVEL, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_logger_mask( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( SYSTEM_LOGGER_MASK )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, SYSTEM_LOGGER_MASK, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SYSTEM_LOGGER_MASK, rc: %d.", rc );
    }
#endif

    return rc;
}


int fw_intf_get_system_context_state( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( CONTEXT_STATE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, CONTEXT_STATE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get CONTEXT_STATE, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_system_mcfe_usecase( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSYSTEM ) && defined( MCFE_USECASE )
    rc = isp_fw_do_get_cmd( ctx_id, TSYSTEM, MCFE_USECASE, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get MCFE_USECASE, rc: %d.", rc );
    }
#endif

    return rc;
}


int fw_intf_get_status_info_gain_log2( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSTATUS ) && defined( STATUS_INFO_GAIN_LOG2_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TSTATUS, STATUS_INFO_GAIN_LOG2_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get STATUS_INFO_GAIN_LOG2_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_status_info_gain_ones( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSTATUS ) && defined( STATUS_INFO_GAIN_ONES_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TSTATUS, STATUS_INFO_GAIN_ONES_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get STATUS_INFO_GAIN_ONES_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_status_info_exposure_residual_log2_id( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSTATUS ) && defined( STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TSTATUS, STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_status_info_iridix_contrast( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSTATUS ) && defined( STATUS_INFO_IRIDIX_CONTRAST )
    rc = isp_fw_do_get_cmd( ctx_id, TSTATUS, STATUS_INFO_IRIDIX_CONTRAST, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get STATUS_INFO_IRIDIX_CONTRAST, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_status_info_ae_hist_mean( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSTATUS ) && defined( STATUS_INFO_AE_HIST_MEAN )
    rc = isp_fw_do_get_cmd( ctx_id, TSTATUS, STATUS_INFO_AE_HIST_MEAN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get STATUS_INFO_AE_HIST_MEAN, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_status_info_awb_mix_light_contrast( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSTATUS ) && defined( STATUS_INFO_AWB_MIX_LIGHT_CONTRAST )
    rc = isp_fw_do_get_cmd( ctx_id, TSTATUS, STATUS_INFO_AWB_MIX_LIGHT_CONTRAST, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get STATUS_INFO_AWB_MIX_LIGHT_CONTRAST, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_info_fw_revision( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSELFTEST ) && defined( FW_REVISION )
    rc = isp_fw_do_get_cmd( ctx_id, TSELFTEST, FW_REVISION, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get FW_REVISION, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_ae_compensation( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;
    return rc;
}

int fw_intf_get_context_number( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TGENERAL ) && defined( CONTEXT_NUMBER )
    rc = isp_fw_do_get_cmd( ctx_id, TGENERAL, CONTEXT_NUMBER, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get CONTEXT_NUMBER, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_active_context( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TGENERAL ) && defined( ACTIVE_CONTEXT )
    rc = isp_fw_do_get_cmd( ctx_id, TGENERAL, ACTIVE_CONTEXT, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get ACTIVE_CONTEXT, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_integration_time_limits( uint32_t ctx_id, int *min_val, int *max_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INTEGRATION_TIME_MIN ) && defined( SENSOR_INTEGRATION_TIME_LIMIT )
    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INTEGRATION_TIME_MIN, min_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INTEGRATION_TIME_MIN rc: %d", rc );
        return rc;
    }

    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INTEGRATION_TIME_LIMIT, max_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INTEGRATION_TIME_LIMIT rc: %d", rc );
        return rc;
    }
#endif

    return rc;
}

int fw_intf_get_sensor_integration_time_limit( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INTEGRATION_TIME_LIMIT )

    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INTEGRATION_TIME_LIMIT, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INTEGRATION_TIME_LIMIT rc: %d", rc );
        return rc;
    }
#endif

    return rc;
}


int fw_intf_get_sensor_integration_time_min( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TSENSOR ) && defined( SENSOR_INTEGRATION_TIME_MIN )

    rc = isp_fw_do_get_cmd( ctx_id, TSENSOR, SENSOR_INTEGRATION_TIME_MIN, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get SENSOR_INTEGRATION_TIME_MIN rc: %d", rc );
        return rc;
    }
#endif

    return rc;
}

int fw_intf_get_register_value( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_VALUE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TREGISTERS, REGISTERS_VALUE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get REGISTERS_VALUE_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_register_source( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_SOURCE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TREGISTERS, REGISTERS_SOURCE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get REGISTERS_SOURCE_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_register_size( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_SIZE_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TREGISTERS, REGISTERS_SIZE_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get REGISTERS_SIZE_ID, rc: %d.", rc );
    }
#endif

    return rc;
}

int fw_intf_get_register_address( uint32_t ctx_id, int *ret_val )
{
    int rc = -1;

#if defined( TREGISTERS ) && defined( REGISTERS_ADDRESS_ID )
    rc = isp_fw_do_get_cmd( ctx_id, TREGISTERS, REGISTERS_ADDRESS_ID, ret_val );
    if ( rc ) {
        LOG( LOG_ERR, "Failed to get REGISTERS_ADDRESS_ID, rc: %d.", rc );
    }
#endif

    return rc;
}
