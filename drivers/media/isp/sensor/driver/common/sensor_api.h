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

#ifndef __ACAMERA_SENSOR_API_H__
#define __ACAMERA_SENSOR_API_H__

#include "system_types.h"

// Denotes the maximum number of sensor channels
#define MAX_CHANNELS ( 10 )

// This structure represents image resolution.
// It is used in the sensor driver to keep information about the frame width and frame height.
typedef struct _image_resolution_t {

    uint16_t width;
    uint16_t height;

} image_resolution_t;

/**
 * @brief Sensor exposure data type enumeration
 * Order is important!
 *
 */
typedef enum _channel_data_type {
    DATA_TYPE_WDR_ENUM_MIN,
    DATA_TYPE_VS,
    DATA_TYPE_VS_S,
    DATA_TYPE_S,
    DATA_TYPE_S_M,
    DATA_TYPE_M,
    DATA_TYPE_M_L,
    DATA_TYPE_L,
    DATA_TYPE_LINEAR,
    DATA_TYPE_WDR_NATIVE,
    DATA_TYPE_WDR_ENUM_MAX,
} channel_data_type;

/**
 * @brief Represents input channel capabilities/requirements
 *
 */
typedef enum _channel_processing_level {
    CAP_MIN,
    CAP_NATIVE_PASS_THROUGH,
    CAP_CHANNEL_PASS_THROUGH,
    CAP_3_FROM_2_DECODE_L,
    CAP_3_FROM_2_DECODE_S,
    CAP_LOG_DECOMPAND,
    CAP_KNEE_POINT_DECOMPAND,
    CAP_MAX,
} channel_processing_level;

/**
 * @brief               sensor channel description struct
 *
 * @exposure_bit_width  final exposure bit width after decompanding (if applicable)
 * @data_type           denotes exposure type for this channel
 * @cv                  channel capability
 *
 */
typedef struct channel_desc_t {
    const uint16_t exposure_bit_width;
    const channel_data_type data_type;
    const channel_processing_level cv;
} channel_desc_t;

/**
 * @brief               locked exposure info struct
 * Each locked exposure is represented by a set of flag and value variables.
 * If a flag variable from the set is not set to true, the locked exposure will NOT be applied!
 *
 * @locked_exp_ratio_flag - denotes locked exposure ratio
 * @locked_exp_ratio_val - denotes locked exposure ratio value (only taken into account if the corresponding flag is set to true)
 * @locked_exp_ratio_short_flag - denotes locked short exposure ratio
 * @locked_exp_ratio_short_flag - denotes locked short exposure ratio value (only taken into account if the corresponding flag is set to true)
 * @locked_exp_ratio_medium_flag - denotes locked medium exposure ratio
 * @locked_exp_ratio_medium_flag - denotes locked medium exposure ratio value (only taken into account if the corresponding flag is set to true)
 * @locked_exp_ratio_medium2_flag - denotes locked medium2 exposure ratio
 * @locked_exp_ratio_medium2_flag - denotes locked medium2 exposure ratio value (only taken into account if the corresponding flag is set to true)
 */
typedef struct locked_exp_info_t {
    const bool locked_exp_ratio_flag;
    const uint32_t locked_exp_ratio_val;
    const bool locked_exp_ratio_short_flag;
    const uint32_t locked_exp_ratio_short_val;
    const bool locked_exp_ratio_medium_flag;
    const uint32_t locked_exp_ratio_medium_val;
    const bool locked_exp_ratio_medium2_flag;
    const uint32_t locked_exp_ratio_medium2_val;
} locked_exp_info_t;

/**
 * @brief                       sensor channels information struct
 *
 * @channel_desc                channel description
 * @exposure_idx_to_channel_map exposure index to channel id map table.
 *                              Contains channel id in decsending order based on exposure length
 *                              0 - Longest, 1 - Next longest, e.t.c so we can always access channel description
 *                              by index without iterating all elements
 * @exposure_max_bit_width      maximum of all channel_desc[*].exposure_bit_width
 * @locked_exp_info             provides information about locked exposures and their respective values
 *
 */
typedef struct channel_info_t {
    const channel_desc_t channel_desc[MAX_CHANNELS];
    const uint8_t exposure_idx_to_channel_map[MAX_CHANNELS];
    const uint8_t exposure_max_bit_width;
    const locked_exp_info_t locked_exp_info;
} channel_info_t;

// A sensor can support several different predefined modes.
// This structure keeps all necessary information about a mode.
typedef struct _sensor_mode_t {
    const image_resolution_t resolution; // Resolution of the mode.
    const channel_info_t channel_info;   // Channel information.
    const uint32_t fps;                  // The FPS value multiplied by 256. Used for preset information queries
    const uint8_t wdr_mode;              // The wdr mode.
    const uint8_t exposures;             // How many exposures this mode supports.
    const uint8_t num_channels;          // How many virtual channels are in this mode.
} sensor_mode_t;

// Function prototypes to support remote sensors

/**
 * @brief   put_frame callback function type.
 *
 * @param   owner - pointer to module which owns the callback function. Will be passed back on a callback call
 * @param   frame - pointer to a filled frame that should be delivered to the external module
 *
 * @return  0 on success
 *
 * @detail  put_frame_callback function is called by sensor to deliver filled frame pointer to the external module
 *          Applicable only for remote sensors
 */
typedef int ( *sensor_put_frame_callback_func_t )( void *owner, void *frame );

/**
 * @brief   get_frame callback function type.
 *
 * @param   owner - pointer to module which owns the callback function. Will be passed back on a callback call
 * @param   frame - pointer to pointer to an empty frame that should be provided by external module
 *
 * @return  0 on success
 *
 * @detail  get_frame_callback function is called by sensor to request empty frame pointer from external module
 *          Applicable only for remote sensors
 */
typedef int ( *sensor_get_frame_callback_func_t )( void *owner, void **frame );

/**
 * @brief   release_frame callback function type.
 *
 * @param   owner - pointer to module which owns the callback function. Will be passed back on a callback call
 * @param   frame - pointer to a frame to be released by an external module
 *
 * @return  0 on success
 *
 * @detail  release_frame_callback function is called by sensor to deliver frame pointer to be released by external module
 *          Usually happens during deinit or mode change
 *          Applicable for remote sensors only
 */
typedef int ( *sensor_release_frame_callback_func_t )( void *owner, void *frame );

// Remote sensor callbacks struct type definition
typedef struct sensor_remote_callbacks_t {

    void *callback_owner;
    sensor_get_frame_callback_func_t get_frame;
    sensor_put_frame_callback_func_t put_frame;
    sensor_release_frame_callback_func_t release_frame;

} sensor_remote_callbacks_t;

// Data structure to support remote sensors
typedef struct _sensor_remote_config_t {

    sensor_remote_callbacks_t callbacks;

} sensor_remote_config_t;

// Declaration only, definition is below.
typedef struct _sensor_control_t sensor_control_t;

// Sensor parameters structure keeps information about the current sensor state.
typedef struct _sensor_param_t {

    image_resolution_t total;                // Total resolution of the image with blanking.
    image_resolution_t active;               // Active resolution without blanking.
    int32_t again_log2_max;                  // Maximum analog gain value in log2 format.
    int32_t dgain_log2_max;                  // Maximum digital gain value in log2 format.
    int32_t wb_gain_log2_max;                // Maximum white balance gain value in log2 format.
    int32_t again_accuracy;                  // Precision of the gain - If required gain step is less then this do not try to allocate it.
    uint32_t integration_time_min;           // Minimum integration time for the sensor in lines.
    uint32_t integration_time_max;           // Maximum integration time for the sensor in lines without dropping fps.
    uint32_t integration_time_medium_max;    // Maximum integration time for medium exposure.
    uint32_t integration_time_medium2_max;   // Maximum integration time for medium 2 exposure (4 exposures).
    uint32_t integration_time_long_max;      // Maximum integration time for long exposure.
    uint32_t integration_time_limit;         // Maximum possible integration time for the sensor.
    uint16_t day_light_integration_time_max; // Limit of integration time for non-flickering light source.
    uint8_t integration_time_precision;      // Integration time precision fractional part number of bits.
    uint8_t integration_time_apply_delay;    // Delay to apply integration time in frames.
    uint8_t isp_exposure_channel_delay;      // Select which WDR exposure channel gain is delayed 0-none, 1-long, 2-medium, 3-short (only 0 and 1 implemented).
    uint32_t lines_per_second;               // Number of lines per second used for antiflicker.
    int32_t sensor_exp_number;               // Number of different exposures supported by the sensor.
    sensor_mode_t *modes_table;              // Table of predefined preset modes which are supported by the sensor.
    uint32_t modes_num;                      // The number of predefined preset modes.
    uint8_t preset_mode;                     // Current preset mode. This value is from the range [ 0 : modes_num - 1 ].
    uint16_t h_start;                        // Sensor dependant offsets for input port.
    uint16_t v_start;                        // Sensor dependant offsets for input port.
    uint8_t is_remote;                       // Sensor type (local/remote).
    uint8_t video_port_id;                   // Sensor video port number / context id. (0-3 local) / (0-15 remote).
    uint8_t num_channel;                     // Number of virtual channels (HDR).
    uint8_t data_width;                      // Bits per pixel.
    uint8_t rggb_start;                      // RGGB pattern start (R/Gr/Gb/B).
    uint8_t cfa_pattern;                     // CFA pattern type (RGGB/RCCC/RIrGB/RGIrB).
    uint8_t shared_vc_clk;                   // Sensor shares video clock between virtual channel
    sensor_remote_config_t remote_cfg;       // Remote sensor specific configuration

} sensor_param_t;

// This structure represents image resolution.
// It is used in the sensor driver to keep information about the frame width and frame height.
typedef struct _integration_times_t {
    uint32_t int_time;
    uint32_t int_time_M;
    uint32_t int_time_M2;
    uint32_t int_time_L;

} integration_times_t;

// This structure is used as an option configuration set
// To be passed as a parameter to a sensor driver init function and
// have a way to alternate default parameter values from set-file (like is_remote)
// set-file -> options -> sensor driver init func -> configured sensor
typedef struct _sensor_options_t {
    uint8_t is_remote;
    uint8_t preset_mode;
} sensor_options_t;

struct _sensor_control_t {

    /**
     * @brief   Allocate analog gain
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   gain - Analog gain value in log2 format precision is defined by LOG2_GAIN_SHIFT.
     *
     * @return  The real analog gain which will be applied.
     *
     * @detail  This function sets the sensor analog gain.
     *          Gain should be just saved here for the future.
     *          The real sensor analog gain update must be implemented in sensor_update routine.
     */
    int32_t ( *alloc_analog_gain )( void *sensor_priv, int32_t gain );


    /**
     * @brief   Allocate digital gain
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   gain - Digital gain value in log2 format precision is defined by LOG2_GAIN_SHIFT.
     *
     * @return  The real digital gain which will be applied.
     *
     * @detail  This function sets the sensor digital gain.
     *          Gain should be just saved here for the future.
     *          The real sensor digital gain update must be implemented in sensor_update routine.
     */
    int32_t ( *alloc_digital_gain )( void *sensor_priv, int32_t gain );


    /**
     * @brief   Allocate integration time
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   int_time - pointer to a structure containing multiple/single exposure integration times.
     *
     * @detail  This function sets the sensor integration time.
     *          Integration time should be just saved here for the future.
     *          The real time update must be implemented in sensor_update routine.
     */
    void ( *alloc_integration_time )( void *sensor_priv, integration_times_t *int_time );

    /**
     * @brief   Allocate white balance gains
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   gain - Array of per channel digital gains values log2 format
     *
     * @detail  This function sets white balance gains to sensor.
     *          White balance gains should be just saved here for the future.
     *          The real update must be implemented in sensor_update routine.
     */
    void ( *alloc_white_balance_gains )( void *sensor_priv, int32_t gain[4] );


    /**
     * @brief   Update all sensor parameters
     *
     * @param   sensor_priv - pointer to a sensor private data.
     *
     * @detail  The function is called from IRQ thread in vertical blanking.
     *          All sensor parameters must be updated here.
     */
    void ( *sensor_update )( void *sensor_priv );


    /**
     * @brief   Set the sensor mode.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   mode - The new mode to set.
     *
     * @detail  Sensor can support several modes. This function is used to switch among them.
     */
    void ( *set_mode )( void *sensor_priv, uint8_t mode );


    /**
     * @brief   Set the new fps mode.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   fps - The new fps to set.
     *
     * @detail  This function is used to update sensor fps configuration.
     */
    uint8_t ( *fps_control )( void *sensor_priv, uint8_t fps );


    /**
     * @brief   Get sensor parameters.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     *
     * @return  pointer to sensor parameters structure.
     *
     * @detail  This function returns a pointer to the sensor parameters structure.
     */
    const sensor_param_t *( *get_parameters )( void *sensor_priv );


    /**
     * @brief   Read on-sensor register.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   address - Address of register.
     *
     * @return  Value read from register.
     *
     * @detail  This function reads the content of sensor registers.
     */
    uint32_t ( *read_sensor_register )( void *sensor_priv, uint32_t address );


    /**
     * @brief   Write on-sensor register.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   address - Address of register.
     * @param   data - Data to write to register location.
     *
     * @detail  This function write data to sensor registers.
     */
    void ( *write_sensor_register )( void *sensor_priv, uint32_t address, uint32_t data );


    /**
     * @brief   Start sensor streaming.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     *
     * @detail  This function starts sensor streaming of image frames.
     */
    void ( *start_streaming )( void *sensor_priv );


    /**
     * @brief   Stop sensor streaming
     *
     * @param   sensor_priv - pointer to a sensor private data.
     *
     * @detail  This function stops sensor generation of image frames.
     */
    void ( *stop_streaming )( void *sensor_priv );


    /**
     * @brief   Deinitialize sensor
     *
     * @param   sensor_priv - pointer to a sensor private data.
     *
     * @detail  This function deinitializes the sensor and frees up all resources.
     */
    void ( *deinit )( void *sensor_priv );


    /**
     * @brief   Request next frame.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     *
     * @detail  This function is called by external source to request next frame from sensor.
     *          Applicable only for remote sensors
     */
    void ( *request_next_frame )( void *sensor_priv );


    /**
     * @brief   Registers/Deregisters frame callback functions.
     *
     * @param   sensor_priv - pointer to a sensor private data.
     * @param   callbacks - pointer to sensor_remote_callbacks_t structure with configured frame callbacks function pointers. (NULL to deregister)
     *
     * @detail  This function is called by external module to register all frame callback functions at one time.
     *          Applicable only for remote sensors
     */
    void ( *register_frame_callbacks )( void *sensor_priv, const sensor_remote_callbacks_t *callbacks );
};


typedef sensor_control_t *sensor_control_ptr_t;


#endif /* __ACAMERA_SENSOR_API_H__ */
