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

#ifndef __DUMMY_CONFIG_H__
#define __DUMMY_CONFIG_H__

#define FULL_EXTRA_HEIGHT 0
#define FULL_EXTRA_WIDTH 0
#define ISP_IMAGE_HEIGHT SENSOR_IMAGE_HEIGHT
#define ISP_IMAGE_WIDTH SENSOR_IMAGE_WIDTH
#define LOG2_SENSOR_AGAIN_MAXIMUM 5 // Sensor Analog Gain: 0dB - 30dB
#define LOG2_SENSOR_DGAIN_MAXIMUM 3 // Sensor Digital Gain: 30dB to 48dB
#define PREVIEW_EXTRA_HEIGHT 0
#define PREVIEW_EXTRA_WIDTH 0
#define RESOLUTION_CHANGE_ENABLED 0
#define SENSOR_AF_MOVE_DELAY 20
#define SENSOR_ANALOG_GAIN_APPLY_DELAY 1
#define SENSOR_BLACK_LEVEL_CORRECTION 0
#define SENSOR_BOARD_MASTER_CLOCK 24000
#define SENSOR_BUS i2c
#define SENSOR_DAY_LIGHT_INTEGRATION_TIME_LIMIT 300
#define SENSOR_DEV_ADDRESS 42
#define SENSOR_DIGITAL_GAIN_APPLY_DELAY 1
#define SENSOR_EXP_NUMBER 1
#define SENSOR_IMAGE_HEIGHT 2160
#define SENSOR_IMAGE_WIDTH 3840
#define SENSOR_INTEGRATION_TIME_APPLY_DELAY 1
#define SENSOR_MAX_INTEGRATION_TIME 11249 // Number of lines per frame - 1
#define SENSOR_MAX_INTEGRATION_TIME_LIMIT SENSOR_MAX_INTEGRATION_TIME
#define SENSOR_MAX_INTEGRATION_TIME_NATIVE SENSOR_MAX_INTEGRATION_TIME
#define SENSOR_MAX_INTEGRATION_TIME_PREVIEW 2586 - 14
#define SENSOR_MIN_INTEGRATION_TIME 3
#define SENSOR_MIN_INTEGRATION_TIME_NATIVE SENSOR_MIN_INTEGRATION_TIME
#define SENSOR_OUTPUT_BITS 10
#define SENSOR_SEQUENCE_FULL_RES_HALF_FPS 2
#define SENSOR_SEQUENCE_FULL_RES_MAX_FPS 0
#define SENSOR_SEQUENCE_NAME default
#define SENSOR_SEQUENCE_PREVIEW_RES_HALF_FPS 3
#define SENSOR_SEQUENCE_PREVIEW_RES_MAX_FPS 1
#define SENSOR_TOTAL_HEIGHT 11250 // Vmax for 5fps
#define SENSOR_TOTAL_HEIGHT_PREVIEW 2586
#define SENSOR_TOTAL_WIDTH 1320 // Hmax
#define SENSOR_TOTAL_WIDTH_PREVIEW 3264
#define SPI_CLOCK_DIV 40
#define SPI_CONTROL_MASK ( RX_NEG_MSK | ( CHAR_LEN_MSK & 32 ) | AUTO_SS_MSK | LSB_MSK )
// sensor_defaults.set

//Additional sensor input raw configuration
#define SENSOR_DATA_WIDTH (12)
#define SENSOR_DATA_FORMAT (0)
#define RAW_INPUT_ISP "/home/root/raw8.rggb"

#endif

int32_t cam_set_exposure(void *sensor_priv, uint64_t exp);
int32_t cam_set_gain(void *sensor_priv, uint64_t gain);
int32_t cam_set_framerate(void *sensor_priv, uint32_t fps);

/* ---------------- Mutex Macro Definition ------------------ */

DEFINE_MUTEX(mcu_i2c_mutex);

#define EXPOSURE_CTRL_ID 0x009A200A
#define GAIN_CTRL_ID 0x009A2009
#define FRAMERATE_CTRL_ID 0x009A200B
#define EXPOSURE_FACTOR 1000000
#define SENSOR_PIXEL_CLOCK 74250000
#define GAIN_FACTOR	10

#define CMD_SIGNATURE 0x43
typedef enum _cmd_id
{
	CMD_ID_SET_CTRL = 0x11,
} HOST_CMD_ID;

enum
{
        CTRL_STANDARD = 0x01,
        CTRL_EXTENDED = 0x02,
};
