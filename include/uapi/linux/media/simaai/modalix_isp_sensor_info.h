/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Modalix ISP sensor-info ABI.
 *
 * Single source of truth for the structures exchanged between the kernel
 * ISP driver and the userspace IPA over
 * MODALIX_ISP_V4L2_CID_SENSOR_INFO_BLOB. The kernel control's payload size
 * is sizeof(acamera_cmd_sensor_info); the IPA reads it into the same type.
 * Both sides MUST include this header so the layout cannot drift.
 */

#ifndef __UAPI_MODALIX_ISP_SENSOR_INFO_H__
#define __UAPI_MODALIX_ISP_SENSOR_INFO_H__

#include <linux/types.h>

#define GENERAL_ROUTER_MAX_ISP_CHANNELS 10

typedef struct _general_image_resolution_t {
	__u16 width;
	__u16 height;
} general_image_resolution_t;

typedef struct general_locked_exp_info_t {
	__u8  locked_exp_ratio_flag;
	__u32 locked_exp_ratio_val;
	__u8  locked_exp_ratio_short_flag;
	__u32 locked_exp_ratio_short_val;
	__u8  locked_exp_ratio_medium_flag;
	__u32 locked_exp_ratio_medium_val;
	__u8  locked_exp_ratio_medium2_flag;
	__u32 locked_exp_ratio_medium2_val;
} general_locked_exp_info_t;

typedef struct _general_channel_desc_t {
	__u16 exposure_bit_width;
	__u8  data_type;
	__u8  cv;
} general_channel_desc_t;

typedef struct _general_channel_info_t {
	general_channel_desc_t channel_desc[GENERAL_ROUTER_MAX_ISP_CHANNELS];
	__u8 exposure_idx_to_channel_map[GENERAL_ROUTER_MAX_ISP_CHANNELS];
	__u8 exposure_max_bit_width;
	general_locked_exp_info_t locked_exp_info;
} general_channel_info_t;

typedef struct _general_sensor_mode_t {
	general_image_resolution_t resolution;
	general_channel_info_t channel_info;
	__u32 fps;          /* FPS value multiplied by 256 */
	__u8  wdr_mode;
	__u8  exposures;
	__u8  num_channels;
} general_sensor_mode_t;

typedef struct {
	__u16 total_width;
	__u16 total_height;
	__u16 active_width;
	__u16 active_height;
	__u16 black_level;
	__u32 lines_per_second;
	__u8  cfa_pattern;
	__s32 again_log2_max;
	__s32 dgain_log2_max;
	__s32 again_accuracy;
	__s32 wb_gain_log2_max;
	__u32 integration_time_min;
	__u32 integration_time_max;
	__u32 integration_time_medium_max;
	__u32 integration_time_long_max;
	__u32 integration_time_limit;
	__u8  integration_time_precision;
	__u8  integration_time_apply_delay;
	__u8  sensor_exp_number;
	__u8  isp_exposure_channel_delay;
	__u8  sensor_output_bits;
	__u8  is_remote;
	general_sensor_mode_t current_sensor_mode;
	__u8  rggb_start;
	__u16 h_start;          /* sensor input-port offset */
	__u16 v_start;          /* sensor input-port offset */
	__u8  video_port_id;    /* sensor video port / context id (MCFE routing) */
} acamera_cmd_sensor_info;

#endif /* __UAPI_MODALIX_ISP_SENSOR_INFO_H__ */
