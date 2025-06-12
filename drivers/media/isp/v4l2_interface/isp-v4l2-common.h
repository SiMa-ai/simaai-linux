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

#ifndef _ISP_V4L2_COMMON_H_
#define _ISP_V4L2_COMMON_H_

#include "acamera_configuration.h"
#include "acamera_frame_stream_api.h"

/* Sensor data types */
#define V4L2_SENSOR_INFO_MODES_MAX ( 10U )     // Maximum number of sensor modes (unique resolutions)
#define V4L2_SENSOR_INFO_SUB_MODES_MAX ( 10U ) // Maximum number of sensor sub modes (unique sensor modes with the same resolution)

/**
 * @brief Struct describes sensor sub mode.
 * Sensor modes with the same resolution converted into sub modes and stored under common mode
 * Sub modes are differentiated by FPS and exposure number
 *
 */
typedef struct _isp_v4l2_sensor_sub_mode {
    uint32_t fps;           ///< Sensor FPS (multiplied by 256)
    uint32_t sensor_preset; ///< Sensor preset index which matches to this sub mode
    uint8_t exposures;      ///< Sensor exposure number
    uint8_t num_channels;   ///< Sensor channels number (used to configure raw buffer planes number)
    uint8_t data_width;     ///< Sensor data width (used to configure raw buffers)
} isp_v4l2_sensor_sub_mode;

/**
 * @brief Struct describes sensor mode (unique resolution)
 *
 */
typedef struct _isp_v4l2_sensor_mode {
    uint32_t width;                                                    ///< Sensor image width
    uint32_t height;                                                   ///< Sensor image height
    isp_v4l2_sensor_sub_mode sub_mode[V4L2_SENSOR_INFO_SUB_MODES_MAX]; ///< Sensor sub modes
    uint8_t num_sub_modes;                                             ///< Number of sub modes
    uint8_t cur_sub_mode;                                              ///< Current sub mode
} isp_v4l2_sensor_mode;

/**
 * @brief Struct holds sensor information
 *
 */
typedef struct _isp_v4l2_sensor_info {
    /* resolution preset */
    isp_v4l2_sensor_mode mode[V4L2_SENSOR_INFO_MODES_MAX]; ///< Sensor modes (unique resolutions)
    uint8_t num_modes;                                     ///< Number of sensor modes
    uint8_t cur_mode;                                      ///< Current sensor mode
} isp_v4l2_sensor_info;

#define V4L2_CAN_UPDATE_SENSOR 0
#define V4L2_RESTORE_FR_BASE0 1

/* custom v4l2 formats */
#define ISP_V4L2_PIX_FMT_META v4l2_fourcc( 'M', 'E', 'T', 'A' ) /* META */
#define ISP_V4L2_PIX_FMT_NULL v4l2_fourcc( 'N', 'U', 'L', 'L' ) /* format NULL to disable */
#ifndef V4L2_PIX_FMT_SBGGR14
#define V4L2_PIX_FMT_SBGGR14 v4l2_fourcc( 'B', 'G', '1', '4' ) /* 14  BGBG.. GRGR.. */
#endif

/* custom v4l2 events */
#define V4L2_EVENT_ACAMERA_CLASS ( V4L2_EVENT_PRIVATE_START + 0xA * 1000 )
#define V4L2_EVENT_ACAMERA_FRAME_READY ( V4L2_EVENT_ACAMERA_CLASS + 0x1 )
#define V4L2_EVENT_ACAMERA_STREAM_OFF ( V4L2_EVENT_ACAMERA_CLASS + 0x2 )

/* custom v4l2 controls */
#define ISP_V4L2_CID_ISP_V4L2_CLASS ( 0x00f00000 | 1 )
#define ISP_V4L2_CID_BASE ( 0x00f00000 | 0xf000 )

enum isp_v4l2_cid {
    ISP_V4L2_CID_TEST_PATTERN = ISP_V4L2_CID_BASE,
    ISP_V4L2_CID_TEST_PATTERN_TYPE,
    ISP_V4L2_CID_SENSOR_SUPPORTED_PRESETS,
    ISP_V4L2_CID_SENSOR_PRESET,
    ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_MIN,
    ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_LIMIT,
    ISP_V4L2_CID_SENSOR_WDR_MODE,
    ISP_V4L2_CID_SENSOR_STREAMING,
    ISP_V4L2_CID_SENSOR_EXPOSURES,
    ISP_V4L2_CID_SENSOR_FPS,
    ISP_V4L2_CID_SENSOR_WIDTH,
    ISP_V4L2_CID_SENSOR_HEIGHT,
    ISP_V4L2_CID_SENSOR_INFO_PRESET,
    ISP_V4L2_CID_SENSOR_INFO_WDR_MODE,
    ISP_V4L2_CID_SENSOR_INFO_FPS,
    ISP_V4L2_CID_SENSOR_INFO_WIDTH,
    ISP_V4L2_CID_SENSOR_INFO_HEIGHT,
    ISP_V4L2_CID_SENSOR_INFO_EXPOSURES,
    ISP_V4L2_CID_SYSTEM_FREEZE_FIRMWARE,
    ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE,
    ISP_V4L2_CID_SENSOR_INFO_CHANNELS,
    ISP_V4L2_CID_SENSOR_INFO_DATA_WIDTH,
    ISP_V4L2_CID_SYSTEM_MANUAL_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_MANUAL_MAX_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_ANALOG_GAIN,
    ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN,
    ISP_V4L2_CID_SYSTEM_MANUAL_ISP_DIGITAL_GAIN,
    ISP_V4L2_CID_SYSTEM_MANUAL_DIRECTIONAL_SHARPENING,
    ISP_V4L2_CID_SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING,
    ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE_RATIO,
    ISP_V4L2_CID_SYSTEM_MANUAL_AWB,
    ISP_V4L2_CID_SYSTEM_ANTIFLICKER_ENABLE,
    ISP_V4L2_CID_SYSTEM_MANUAL_SATURATION,
    ISP_V4L2_CID_SYSTEM_MAX_EXPOSURE_RATIO,
    ISP_V4L2_CID_SYSTEM_EXPOSURE,
    ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_SHORT_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_MIDDLE_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_MIDDLE2_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_LONG_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME_PRECISION,
    ISP_V4L2_CID_SYSTEM_MAX_INTEGRATION_TIME,
    ISP_V4L2_CID_SYSTEM_EXPOSURE_RATIO,
    ISP_V4L2_CID_SYSTEM_SENSOR_ANALOG_GAIN,
    ISP_V4L2_CID_SYSTEM_MAX_SENSOR_ANALOG_GAIN,
    ISP_V4L2_CID_SYSTEM_SENSOR_DIGITAL_GAIN,
    ISP_V4L2_CID_SYSTEM_MAX_SENSOR_DIGITAL_GAIN,
    ISP_V4L2_CID_SYSTEM_ISP_DIGITAL_GAIN,
    ISP_V4L2_CID_SYSTEM_MAX_ISP_DIGITAL_GAIN,
    ISP_V4L2_CID_SYSTEM_DIRECTIONAL_SHARPENING_TARGET,
    ISP_V4L2_CID_SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET,
    ISP_V4L2_CID_SYSTEM_AWB_RED_GAIN,
    ISP_V4L2_CID_SYSTEM_AWB_BLUE_GAIN,
    ISP_V4L2_CID_SYSTEM_AWB_CCT,
    ISP_V4L2_CID_SYSTEM_SATURATION_TARGET,
    ISP_V4L2_CID_SYSTEM_ANTI_FLICKER_FREQUENCY,
    ISP_V4L2_CID_SYSTEM_IRIDIX_DIGITAL_GAIN,
    ISP_V4L2_CID_SYSTEM_SINTER_THRESHOLD_TARGET,
    ISP_V4L2_CID_SYSTEM_MINIMUM_IRIDIX_STRENGTH,
    ISP_V4L2_CID_SYSTEM_MAXIMUM_IRIDIX_STRENGTH,
    ISP_V4L2_CID_SYSTEM_IRIDIX_STRENGTH_TARGET,
    ISP_V4L2_CID_SYSTEM_BUFFER_DATA_TYPE_ID,
    ISP_V4L2_CID_SYSTEM_LOGGER_LEVEL_ID,
    ISP_V4L2_CID_SYSTEM_LOGGER_MASK_ID,
    ISP_V4L2_CID_SYSTEM_CMD_INTERFACE_MODE_ID,
    ISP_V4L2_CID_SYSTEM_CONTEXT_STATE_ID,
    ISP_V4L2_CID_SYSTEM_MCFE_USECASE_ID,
    ISP_V4L2_CID_SYSTEM_M2M_PROCESS_REQUEST,
    ISP_V4L2_CID_SYSTEM_V4L2_INTERFACE_MODE,

    // ISP_MODULES
    ISP_V4L2_CID_ISP_MODULES_MANUAL_IRIDIX,
    ISP_V4L2_CID_ISP_MODULES_MANUAL_SINTER,
    ISP_V4L2_CID_ISP_MODULES_MANUAL_FRAME_STITCH,
    ISP_V4L2_CID_ISP_MODULES_MANUAL_RAW_FRONTEND,
    ISP_V4L2_CID_ISP_MODULES_MANUAL_BLACK_LEVEL,
    ISP_V4L2_CID_ISP_MODULES_MANUAL_SHADING,
    ISP_V4L2_CID_ISP_MODULES_MANUAL_DEMOSAIC,
    ISP_V4L2_CID_ISP_MODULES_FORCE_BIST_MISMATCH,
#if ( ISP_RTL_VERSION_R == 2 )
    ISP_V4L2_CID_ISP_MODULES_MANUAL_CNR,
#endif

    // TIMAGE
    ISP_V4L2_CID_IMAGE_CROP_XOFFSET_ID,
    ISP_V4L2_CID_IMAGE_CROP_YOFFSET_ID,
    ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_ID,
    ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID,
    ISP_V4L2_CID_IMAGE_OUTPUT_AXI1_FORMAT_ID,
    ISP_V4L2_CID_IMAGE_OUTPUT_AXI2_FORMAT_ID,
    ISP_V4L2_CID_IMAGE_OUTPUT_AXI3_FORMAT_ID,
    ISP_V4L2_CID_IMAGE_CROP_HEIGHT_ID,
    ISP_V4L2_CID_IMAGE_CROP_WIDTH_ID,
    ISP_V4L2_CID_IMAGE_CROP_ENABLE_ID,
#if ( ISP_RTL_VERSION_R == 2 )
    ISP_V4L2_CID_IMAGE_RAW_SCALER_ENABLE_ID,
    ISP_V4L2_CID_IMAGE_RAW_SCALER_WIDTH_ID,
    ISP_V4L2_CID_IMAGE_RAW_SCALER_HEIGHT_ID,
    ISP_V4L2_CID_IMAGE_RGB_SCALER_ENABLE_ID,
    ISP_V4L2_CID_IMAGE_RGB_SCALER_WIDTH_ID,
    ISP_V4L2_CID_IMAGE_RGB_SCALER_HEIGHT_ID,
#endif


    ISP_V4L2_CID_STATUS_INFO_EXPOSURE_LOG2,
    ISP_V4L2_CID_STATUS_INFO_GAIN_LOG2,
    ISP_V4L2_CID_STATUS_INFO_GAIN_ONES,
    ISP_V4L2_CID_STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID,
    ISP_V4L2_CID_STATUS_INFO_IRIDIX_CONTRAST,
    ISP_V4L2_CID_STATUS_INFO_AE_HIST_MEAN,

    ISP_V4L2_CID_STATUS_INFO_AWB_MIX_LIGHT_CONTRAST,
    ISP_V4L2_CID_INFO_FW_REVISION,

    // TGENERAL
    ISP_V4L2_CID_CONTEXT_NUMBER,
    ISP_V4L2_CID_ACTIVE_CONTEXT,

    // TREGISTERS
    ISP_V4L2_CID_REGISTERS_VALUE_ID,
    ISP_V4L2_CID_REGISTERS_SOURCE_ID,
    ISP_V4L2_CID_REGISTERS_SIZE_ID,
    ISP_V4L2_CID_REGISTERS_ADDRESS_ID,
};

/**
 * @brief Stream types
 *
 */
typedef enum {
    V4L2_STREAM_TYPE_RAW = 0,
    V4L2_STREAM_TYPE_OUT,
    V4L2_STREAM_TYPE_META,
    V4L2_STREAM_TYPE_M2M,
    V4L2_STREAM_TYPE_MAX
} isp_v4l2_stream_type_t;

/**
 * @brief Stream directios
 *
 */
typedef enum {
    V4L2_STREAM_DIRECTION_CAP = 0, // Capture stream direction. Processed frames, ISP->V4L2->User
    V4L2_STREAM_DIRECTION_OUT,     // Output stream direction. Frames to process, User->V4L2->ISP
    V4L2_STREAM_DIRECTION_MAX
} isp_v4l2_stream_direction_t;

#endif
