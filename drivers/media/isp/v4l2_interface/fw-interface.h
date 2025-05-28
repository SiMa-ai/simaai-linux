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

#ifndef _FW_INTERFACE_H_
#define _FW_INTERFACE_H_

#include <linux/videodev2.h>
#include "isp-v4l2-common.h"

/* fw-interface stream<->aframe type conversion */
isp_v4l2_stream_type_t isp_fw_aframe_type_to_stream_type( aframe_type_t type );
aframe_type_t isp_fw_stream_type_to_aframe_type( isp_v4l2_stream_type_t stream_type, isp_v4l2_stream_direction_t stream_direction );

/* fw-interface isp control interface */
int fw_intf_is_context_initialised( uint32_t ctx_id );
int fw_intf_is_context_configured( uint32_t ctx_id );
int fw_intf_is_context_started( uint32_t ctx_id );
int fw_intf_is_context_stopped( uint32_t ctx_id );
int fw_intf_context_config( uint32_t ctx_id );
int fw_intf_context_start( uint32_t ctx_id );
int fw_intf_context_stop( uint32_t ctx_id );
int fw_intf_isp_get_current_ctx_id( uint32_t ctx_id );
int fw_intf_isp_get_sensor_info( uint32_t ctx_id, isp_v4l2_sensor_info *sensor_info );
int fw_intf_isp_update_sensor_info_mode( uint32_t ctx_id, isp_v4l2_sensor_info *sensor_info );

/* fw-interface per-stream control interface */
int fw_intf_stream_start( uint32_t ctx_id, isp_v4l2_stream_type_t stream_type, unsigned long stream_open_mask );
void fw_intf_stream_stop( uint32_t ctx_id, isp_v4l2_stream_type_t stream_type, unsigned long stream_on_mask );
int fw_intf_m2m_process_request( uint32_t ctx_id );

/* fw-interface per-stream config interface */
int fw_intf_stream_set_resolution( uint32_t ctx_id, isp_v4l2_sensor_info *sensor_info,
                                   isp_v4l2_stream_type_t stream_type, uint32_t *width, uint32_t *height );
int fw_intf_stream_set_output_format( uint32_t ctx_id, isp_v4l2_stream_type_t stream_type, isp_v4l2_stream_direction_t stream_direction, uint32_t format );

/* fw-interface isp config interface */
bool fw_intf_validate_control( uint32_t id );
int fw_intf_set_test_pattern( uint32_t ctx_id, int val );
int fw_intf_set_test_pattern_type( uint32_t ctx_id, int val );
int fw_intf_set_sensor_info_preset( uint32_t ctx_id, int val );
int fw_intf_set_sensor_preset( uint32_t ctx_id, int val );
int fw_intf_set_system_freeze_firmware( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_exposure( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_integration_time( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_max_integration_time( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_sensor_analog_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_sensor_digital_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_isp_digital_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_directional_sharpening( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_un_directional_sharpening( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_exposure_ratio( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_awb( uint32_t ctx_id, int val );
int fw_intf_set_system_antiflicker_enable( uint32_t ctx_id, int val );
int fw_intf_set_system_manual_saturation( uint32_t ctx_id, int val );
int fw_intf_set_system_max_exposure_ratio( uint32_t ctx_id, int val );
int fw_intf_set_system_exposure( uint32_t ctx_id, int val );
int fw_intf_set_system_integration_time( uint32_t ctx_id, int val );
int fw_intf_set_system_exposure_ratio( uint32_t ctx_id, int val );
int fw_intf_set_system_max_integration_time( uint32_t ctx_id, int val );
int fw_intf_set_system_sensor_analog_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_max_sensor_analog_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_sensor_digital_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_max_sensor_digital_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_isp_digital_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_max_isp_digital_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_directional_sharpening_target( uint32_t ctx_id, int val );
int fw_intf_set_system_un_directional_sharpening_target( uint32_t ctx_id, int val );
int fw_intf_set_system_awb_red_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_awb_blue_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_awb_cct( uint32_t ctx_id, int val );
int fw_intf_set_system_saturation_target( uint32_t ctx_id, int val );
int fw_intf_set_system_anti_flicker_frequency( uint32_t ctx_id, int val );
int fw_intf_set_system_iridix_digital_gain( uint32_t ctx_id, int val );
int fw_intf_set_system_sinter_threshold_target( uint32_t ctx_id, int val );
int fw_intf_set_system_minimum_iridix_strength( uint32_t ctx_id, int val );
int fw_intf_set_system_maximum_iridix_strength( uint32_t ctx_id, int val );
int fw_intf_set_system_iridix_strength_target( uint32_t ctx_id, int val );
int fw_intf_set_system_logger_level( uint32_t ctx_id, int val );
int fw_intf_set_system_logger_mask( uint32_t ctx_id, int val );
int fw_intf_set_system_m2m_process_request( uint32_t ctx_id, int val );
int fw_intf_set_system_v4l2_interface_mode( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_iridix( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_sinter( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_frame_stitch( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_raw_frontend( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_black_level( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_shading( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_force_bist_mismatch( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_demosaic( uint32_t ctx_id, int val );
int fw_intf_set_isp_modules_manual_cnr( uint32_t ctx_id, int val );
int fw_intf_set_image_crop_xoffset( uint32_t ctx_id, int val );
int fw_intf_set_image_crop_yoffset( uint32_t ctx_id, int val );
int fw_intf_set_image_crop_height( uint32_t ctx_id, int val );
int fw_intf_set_image_crop_width( uint32_t ctx_id, int val );
int fw_intf_set_image_crop_enable( uint32_t ctx_id, int val );
int fw_intf_set_output_format_id( uint32_t ctx_id, int val );
int fw_intf_set_output_format_manual_cfg_apply_id( uint32_t ctx_id, int val );
#if ( ISP_RTL_VERSION_R == 2 )
int fw_intf_set_raw_scaler_enable_id( uint32_t ctx_id, int val );
int fw_intf_set_raw_scaler_width_id( uint32_t ctx_id, int val );
int fw_intf_set_raw_scaler_height_id( uint32_t ctx_id, int val );
int fw_intf_set_rgb_scaler_enable_id( uint32_t ctx_id, int val );
int fw_intf_set_rgb_scaler_width_id( uint32_t ctx_id, int val );
int fw_intf_set_rgb_scaler_height_id( uint32_t ctx_id, int val );
#endif
int fw_intf_set_output_axi1_format_id( uint32_t ctx_id, int val );
int fw_intf_set_output_axi2_format_id( uint32_t ctx_id, int val );
int fw_intf_set_output_axi3_format_id( uint32_t ctx_id, int val );

int fw_intf_set_ae_compensation( uint32_t ctx_id, int val );
int fw_intf_set_active_context( uint32_t ctx_id, int val );
int fw_intf_set_register_source( uint32_t ctx_id, int val );
int fw_intf_set_register_size( uint32_t ctx_id, int val );
int fw_intf_set_register_address( uint32_t ctx_id, int val );
int fw_intf_set_register_value( uint32_t ctx_id, int val );

int fw_intf_get_test_pattern( uint32_t ctx_id, int *ret_val );
int fw_intf_get_test_pattern_type( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_supported_presets( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_preset( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_wdr_mode( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_streaming( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_exposures( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_fps( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_width( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_height( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_preset( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_wdr_mode( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_fps( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_width( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_height( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_exposures( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_channels( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_info_data_width( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_integration_time_limit( uint32_t ctx_id, int *ret_val );
int fw_intf_get_sensor_integration_time_min( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_freeze_firmware( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_exposure( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_max_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_sensor_analog_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_sensor_digital_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_max_sensor_digital_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_directional_sharpening( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_un_directional_sharpening( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_isp_digital_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_exposure_ratio( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_awb( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_antiflicker_enable( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_manual_saturation( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_max_exposure_ratio( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_max_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_max_sensor_analog_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_max_isp_digital_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_directional_sharpening_target( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_un_directional_sharpening_target( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_manual_iridix( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_manual_sinter( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_manual_frame_stitch( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_manual_raw_frontend( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_manual_black_level( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_manual_shading( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_force_bist_mismatch( uint32_t ctx_id, int *ret_val );
int fw_intf_get_isp_modules_manual_demosaic( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_m2m_process_request( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_v4l2_interface_mode( uint32_t ctx_id, int *ret_val );
#if ( ISP_RTL_VERSION_R == 2 )
int fw_intf_get_isp_modules_manual_cnr( uint32_t ctx_id, int *ret_val );
#endif
int fw_intf_get_image_crop_xoffset( uint32_t ctx_id, int *ret_val );
int fw_intf_get_image_crop_yoffset( uint32_t ctx_id, int *ret_val );
int fw_intf_get_output_format_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_output_format_manual_cfg_apply_id( uint32_t ctx_id, int *ret_val );
#if ( ISP_RTL_VERSION_R == 2 )
int fw_intf_get_raw_scaler_enable_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_raw_scaler_width_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_raw_scaler_height_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_rgb_scaler_enable_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_rgb_scaler_width_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_rgb_scaler_height_id( uint32_t ctx_id, int *ret_val );
#endif
int fw_intf_get_output_axi1_format_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_output_axi2_format_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_output_axi3_format_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_image_crop_height( uint32_t ctx_id, int *ret_val );
int fw_intf_get_image_crop_width( uint32_t ctx_id, int *ret_val );
int fw_intf_get_image_crop_enable( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_exposure( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_short_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_middle_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_middle2_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_long_integration_time( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_integration_time_precision( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_exposure_ratio( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_sensor_analog_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_sensor_digital_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_isp_digital_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_awb_red_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_awb_blue_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_awb_cct( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_saturation_target( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_anti_flicker_frequency( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_iridix_digital_gain( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_sinter_threshold_target( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_minimum_iridix_strength( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_maximum_iridix_strength( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_iridix_strength_target( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_buffer_data_type( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_logger_level( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_logger_mask( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_cmd_interface_mode( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_context_state( uint32_t ctx_id, int *ret_val );
int fw_intf_get_system_mcfe_usecase( uint32_t ctx_id, int *ret_val );
int fw_intf_get_status_info_exposure_log2( uint32_t ctx_id, int *ret_val );
int fw_intf_get_status_info_gain_log2( uint32_t ctx_id, int *ret_val );
int fw_intf_get_status_info_gain_ones( uint32_t ctx_id, int *ret_val );
int fw_intf_get_status_info_exposure_residual_log2_id( uint32_t ctx_id, int *ret_val );
int fw_intf_get_status_info_iridix_contrast( uint32_t ctx_id, int *ret_val );
int fw_intf_get_status_info_ae_hist_mean( uint32_t ctx_id, int *ret_val );
int fw_intf_get_status_info_awb_mix_light_contrast( uint32_t ctx_id, int *ret_val );
int fw_intf_get_info_fw_revision( uint32_t ctx_id, int *ret_val );
int fw_intf_get_ae_compensation( uint32_t ctx_id, int *ret_val );
int fw_intf_get_context_number( uint32_t ctx_id, int *ret_val );
int fw_intf_get_active_context( uint32_t ctx_id, int *ret_val );
int fw_intf_get_integration_time_limits( uint32_t ctx_id, int *min_val, int *max_val );
int fw_intf_get_register_source( uint32_t ctx_id, int *ret_val );
int fw_intf_get_register_size( uint32_t ctx_id, int *ret_val );
int fw_intf_get_register_address( uint32_t ctx_id, int *ret_val );
int fw_intf_get_register_value( uint32_t ctx_id, int *ret_val );
#endif
