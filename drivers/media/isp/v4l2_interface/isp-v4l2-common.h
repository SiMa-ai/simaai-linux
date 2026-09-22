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
#define V4L2_SENSOR_INFO_MODES_MAX ( 32U )     // Maximum number of sensor modes (unique resolutions)
#define V4L2_SENSOR_INFO_SUB_MODES_MAX ( 32U ) // Maximum number of sensor sub modes (unique sensor modes with the same resolution)

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
} isp_v4l2_sensor_sub_mode;

/**
 * @brief Struct describes sensor mode (unique resolution)
 *
 */
typedef struct _isp_v4l2_sensor_mode {
    uint32_t width;                                                    ///< Sensor image width
    uint32_t height;                                                   ///< Sensor image height
    uint8_t data_width;     ///< Sensor data width (used to configure raw buffers)
    uint32_t pixel_format;  ///< Sensor pixel format (V4L2_PIX_FMT_XXX)
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

/* The custom V4L2 controls, events, and pixel formats — previously
 * defined inline here — moved to the shared UAPI header so user-space
 * tools (libcamera, v4l2-ctl, debugging scripts) get the same
 * identifiers the kernel uses. */
#include <linux/media/simaai/modalix_isp_v4l2.h>

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
