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

#ifndef __GENERAL__ROUTER_H__
#define __GENERAL__ROUTER_H__

#include "system_types.h"

#define GENERAL_ROUTER_MAX_ISP_CHANNELS ( 10 )

typedef enum {
    CMD_DIRECTION_GET,
    CMD_DIRECTION_SET,
} acamera_cmd_direction;

typedef enum {
    CMD_ID_RAW_SCALER_RATIO,
    CMD_ID_AE_INFO,
    CMD_ID_CCM_INFO,
    CMD_ID_CMOS_CURRENT_EXPOSURE_LOG2,
    CMD_ID_CMOS_EXPOSURE_RATIO,
    CMD_ID_CMOS_HISTORY_SIZE,
    CMD_ID_CMOS_MAX_EXPOSURE_LOG2,
    CMD_ID_CMOS_TOTAL_GAIN,
    CMD_ID_CMOS_ANALOG_GAIN,
    CMD_ID_CMOS_DIGITAL_GAIN,
    CMD_ID_DEFECT_PIXEL_INFO,
    CMD_ID_EXPOSURE_TARGET,
    CMD_ID_FRAME_EXPOSURE_SET,
    CMD_ID_HIST_INFO,
    CMD_ID_OUTPUT_FORMATTER_DATA,
    CMD_ID_SENSOR_INFO,
    CMD_ID_SENSOR_PRESET_INFO,
    CMD_ID_SENSOR_ALLOC_ANALOG_GAIN,
    CMD_ID_SENSOR_ALLOC_DIGITAL_GAIN,
    CMD_ID_SENSOR_ALLOC_WB_GAINS,
    CMD_ID_SENSOR_ALLOC_INTEGRATION_TIME,
    CMD_ID_SENSOR_UPDATE,
    CMD_ID_SENSOR_SET_WDR_MODE,
    CMD_ID_WB_INFO,
    CMD_ID_WB_FRAME_QUANTITY,
    CMD_ID_MCFE_GET_SLOT_AVAIL_INPUTS,
    CMD_ID_HIST_IS_ON_SRQT,
    CMD_ID_BUFFER_ADDRESS_TRANSLATION,

    // MAX
    CMD_ID_MAX,
} acamera_cmd_id;

typedef enum {
    CMD_RESULT_OK,
    CMD_RESULT_INVALID_PARAM,
    CMD_RESULT_NOT_SUPPORTED,
    CMD_RESULT_ERR,
} acamera_cmd_result;

typedef struct {
    int32_t exposure_log2;
    uint32_t exposure_ratio;
    uint32_t contrast;
    uint32_t ae_hist_mean;
} acamera_cmd_ae_info;

typedef struct {
    int32_t exposure_log2;
    uint32_t exposure_ratio;
} acamera_cmd_exposure_target;

typedef struct {
    uint32_t again_reg;
    uint32_t again_log2;
} acamera_cmd_set_analog_gain;

typedef struct _exposure_data_set_t {
    uint32_t integration_time; /* fixed point format Q"sensor_info.integration_time_precision" */
    uint32_t again_reg;
    uint32_t dgain_reg;
    int32_t isp_dgain_log2;
    uint32_t exposure_ratio;
    uint32_t actual_integration_time; /* fixed point format Q"sensor_info.integration_time_precision" */

    uint32_t exposure_ratio_short;
    uint32_t integration_time_long; /* fixed point format Q"sensor_info.integration_time_precision" */

    uint32_t exposure_ratio_medium;
    uint32_t integration_time_medium; /* fixed point format Q"sensor_info.integration_time_precision" */

    uint32_t exposure_ratio_medium2;
    uint32_t integration_time_medium2; /* fixed point format Q"sensor_info.integration_time_precision" */
} exposure_data_set_t;

typedef struct _exposure_info_set_t {
    int32_t exposure_log2;
    int32_t again_log2;
    int32_t dgain_log2;
    int32_t isp_dgain_log2;
} exposure_info_set_t;

typedef struct _exposure_set_t {
    exposure_info_set_t info;
    exposure_data_set_t data;
} exposure_set_t;

typedef struct {
    uint8_t light_source;
    uint8_t light_source_previous;
    uint8_t light_source_ccm;
    uint8_t light_source_ccm_previous;
    uint8_t light_source_change_frames;
    uint8_t light_source_change_frames_left;
} acamera_cmd_ccm_info;

typedef struct {
    int32_t wb_log2[4];
    int32_t temperature_detected;
    uint8_t p_high;
    uint8_t light_source_candidate;
} acamera_cmd_wb_info;

typedef struct {
    int16_t accel_angle;
} acamera_cmd_accel_data;


typedef struct {
    uint32_t switch_light_source_detect_frames_quantity;
    uint32_t switch_light_source_change_frames_quantity;
} acamera_cmd_wb_frame_quantity;


// This structure represents image resolution.
// It is used in the sensor driver to keep information about the frame width and frame height.
typedef struct _general_image_resolution_t {

    uint16_t width;
    uint16_t height;

} general_image_resolution_t;


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
typedef struct general_locked_exp_info_t {
    bool locked_exp_ratio_flag;
    uint32_t locked_exp_ratio_val;
    bool locked_exp_ratio_short_flag;
    uint32_t locked_exp_ratio_short_val;
    bool locked_exp_ratio_medium_flag;
    uint32_t locked_exp_ratio_medium_val;
    bool locked_exp_ratio_medium2_flag;
    uint32_t locked_exp_ratio_medium2_val;
} general_locked_exp_info_t;

/**
 * @brief               sensor channel description struct
 *
 * @exposure_bit_width  final exposure bit width after decompanding (if applicable)
 * @data_type           denotes exposure type for this channel
 * @cv                  channel capability
 *
 */
typedef struct _general_channel_desc_t {
    uint16_t exposure_bit_width;
    uint8_t data_type;
    uint8_t cv;
} general_channel_desc_t;

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
typedef struct _general_channel_info_t {
    general_channel_desc_t channel_desc[GENERAL_ROUTER_MAX_ISP_CHANNELS];
    uint8_t exposure_idx_to_channel_map[GENERAL_ROUTER_MAX_ISP_CHANNELS];
    uint8_t exposure_max_bit_width;
    general_locked_exp_info_t locked_exp_info;
} general_channel_info_t;

// A sensor can support several different predefined modes.
// This structure keeps all necessary information about a mode.
typedef struct _general_sensor_mode_t {
    general_image_resolution_t resolution; // Resolution of the mode.
    general_channel_info_t channel_info;   // Channel information.
    uint32_t fps;                          // The FPS value multiplied by 256. Used for preset information queries
    uint8_t wdr_mode;                      // The wdr mode.
    uint8_t exposures;                     // How many exposures this mode supports.
    uint8_t num_channels;                  // How many virtual channels are in this mode.
} general_sensor_mode_t;

typedef struct {
    uint16_t total_width;
    uint16_t total_height;
    uint16_t active_width;
    uint16_t active_height;
    uint16_t black_level;
    uint32_t lines_per_second;
    uint8_t cfa_pattern;
    int32_t again_log2_max;
    int32_t dgain_log2_max;
    int32_t again_accuracy;
    int32_t wb_gain_log2_max;
    uint32_t integration_time_min;
    uint32_t integration_time_max;
    uint32_t integration_time_medium_max;
    uint32_t integration_time_long_max;
    uint32_t integration_time_limit;
    uint8_t integration_time_precision;
    uint8_t integration_time_apply_delay;
    uint8_t sensor_exp_number;
    uint8_t isp_exposure_channel_delay;
    uint8_t sensor_output_bits;
    uint8_t is_remote;
    general_sensor_mode_t current_sensor_mode;
    uint8_t rggb_start;
} acamera_cmd_sensor_info;

typedef struct acamera_cmd_sensor_preset_info_t {
    uint16_t width;       // Width
    uint16_t height;      // Height
    uint32_t fps;         // FPS
    uint8_t wdr_mode;     // The wdr mode
    uint8_t exposures;    // How many exposures this preset supports
    uint8_t num_channels; // How many channels this preset uses
    uint8_t data_width;   // Sensor data width
} acamera_cmd_sensor_preset_info_t;

typedef struct {
    uint32_t int_time;
    uint32_t int_time_M;
    uint32_t int_time_M2;
    uint32_t int_time_L;

    // NOTE:
    // We need this dummy variable because we want to convert this struct pointer
    // to (const uint32_t *), without this dummy, build has warnings and nios2
    // toolchain tream warning as errors.
    uint32_t dummy;
} acamera_cmd_sensor_integration_time;

typedef struct {
    uint32_t fullhist_sum;
    const uint32_t *fullhist;
} acamera_cmd_hist_info;

typedef struct {
    uint16_t autolevel_black_level;
    uint16_t autolevel_white_level;

    // NOTE:
    // We need this dummy variable because we want to convert this struct pointer
    // to (const uint32_t *), without this dummy, build has warnings and nios2
    // toolchain tream warning as errors.
    uint32_t dummy;
} acamera_cmd_sharpening_info;

typedef struct {
    uint8_t hp_started;
} acamera_cmd_defect_pixel_info;

typedef struct acamera_cmd_sensor_remote_frame_info {
    void *p_remote_sensor_frame;
} acamera_cmd_sensor_remote_frame_info;

typedef enum {
    CMD_BUFFER_ADDRESS_TRANSLATION_TYPE_UNKNOWN,  // Unknown translation type
    CMD_BUFFER_ADDRESS_TRANSLATION_TYPE_DIRECT,   // No buffer address translation required
    CMD_BUFFER_ADDRESS_TRANSLATION_TYPE_TRANSLATE // Buffer address translation required
} acamera_cmd_buffer_address_translation_type;

typedef struct acamera_cmd_buffer_address_translation_info {
    struct {
        acamera_cmd_buffer_address_translation_type type;
        struct {
            uint32_t high;
            uint32_t low;
        } address;
    } raw, out;
} acamera_cmd_buffer_address_translation_info;

acamera_cmd_result general_handle_cmd( void *p_ictx, acamera_cmd_id cmd_id, acamera_cmd_direction dir, const void *value, void *ret_value );

/**
 * @brief      Wrapper macro to call general_handle_cmd and check return code
 * @details    If return code is not #CMD_RESULT_OK then print error message. Returns error code
 */
#define WRAP_GENERAL_CMD( p_ictx, cmd_id, dir, value, ret_value ) ( {                     \
    acamera_cmd_result rc;                                                                \
    rc = general_handle_cmd( ( p_ictx ), ( cmd_id ), ( dir ), ( value ), ( ret_value ) ); \
    if ( rc != CMD_RESULT_OK ) {                                                          \
        LOG( LOG_ERR,                                                                     \
             "general_handle_cmd(..., " #cmd_id ", " #dir ", ...) failed with error: %d", \
             rc );                                                                        \
    }                                                                                     \
    rc;                                                                                   \
} )

/**
 * @brief       Value that allow to avoid error condition:
 *              "Not enough free input ports available (required: %d, available: %d)"
 *              in function: void sensor_config( sensor_fsm_ptr_t p_fsm ); file: sensor_func.c
 *              for "PLATFORM_MODEL" build configuration
 */
#if defined( PLATFORM_MODEL )
#define PLATFORM_MODEL_GENERAL_MCFE_SLOT_AVAIL_INPUTS ( 2 )
#endif


#endif /* __GENERAL__ROUTER_H__ */
