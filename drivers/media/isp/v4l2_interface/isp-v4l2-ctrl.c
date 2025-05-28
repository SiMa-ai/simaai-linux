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

#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#include "acamera_logger.h"

#include "fw-interface.h"
#include "isp-v4l2-common.h"
#include "isp-v4l2-ctrl.h"

static int isp_v4l2_ctrl_check_valid( struct v4l2_ctrl *ctrl )
{
    if ( ctrl->is_int == 1 ) {
        if ( ctrl->val < ctrl->minimum || ctrl->val > ctrl->maximum )
            return -EINVAL;
    }

    return 0;
}

static int isp_v4l2_ctrl_s_ctrl_standard( struct v4l2_ctrl *ctrl )
{
    int ret = 0;

    LOG( LOG_INFO, "Control - id: 0x%x, value: %d, is_int: %d, min: %lld, max: %lld",
         ctrl->id, ctrl->val, ctrl->is_int, ctrl->minimum, ctrl->maximum );

    if ( isp_v4l2_ctrl_check_valid( ctrl ) < 0 ) {
        return -EINVAL;
    }

    /**
     * Currently no user-class controls ( V4L2_CID_BASE ) supported
     */

    return ret;
}

static int isp_v4l2_ctrl_g_ctrl_custom( struct v4l2_ctrl *ctrl )
{
    int ret = -EINVAL;

    struct v4l2_ctrl_handler *hdl = ctrl->handler;

    isp_v4l2_ctrl_t *isp_ctrl = cst_hdl_to_isp_ctrl( hdl );
    int ctx_id = isp_ctrl->ctx_id;

    LOG( LOG_INFO, "Control - id:0x%x, val:%d, is_int:%d, min:%lld, max:%lld.\n",
         ctrl->id, ctrl->val, ctrl->is_int, ctrl->minimum, ctrl->maximum );

    if ( isp_v4l2_ctrl_check_valid( ctrl ) < 0 ) {
        LOG( LOG_ERR, "Invalid param: id:0x%x, val:0x%x, is_int:%d, min:0x%llx, max:0x%llx.\n",
             ctrl->id, ctrl->val, ctrl->is_int, ctrl->minimum, ctrl->maximum );

        return -EINVAL;
    }

    switch ( ctrl->id ) {
    case ISP_V4L2_CID_TEST_PATTERN:
        ret = fw_intf_get_test_pattern( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get test_pattern: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_TEST_PATTERN_TYPE:
        ret = fw_intf_get_test_pattern_type( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get test_pattern_type: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_SUPPORTED_PRESETS:
        ret = fw_intf_get_sensor_supported_presets( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor supported presets: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_PRESET:
        ret = fw_intf_get_sensor_preset( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor preset: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_WDR_MODE:
        ret = fw_intf_get_sensor_wdr_mode( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_wdr_mode: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_STREAMING:
        ret = fw_intf_get_sensor_streaming( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_streaming: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_EXPOSURES:
        ret = fw_intf_get_sensor_exposures( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_exposures: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_FPS:
        ret = fw_intf_get_sensor_fps( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_fps: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_WIDTH:
        ret = fw_intf_get_sensor_width( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_width: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_HEIGHT:
        ret = fw_intf_get_sensor_height( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_height: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_PRESET:
        ret = fw_intf_get_sensor_info_preset( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_preset: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_WDR_MODE:
        ret = fw_intf_get_sensor_info_wdr_mode( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_wdr_mode: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_FPS:
        ret = fw_intf_get_sensor_info_fps( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_fps: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_WIDTH:
        ret = fw_intf_get_sensor_info_width( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_width: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_HEIGHT:
        ret = fw_intf_get_sensor_info_height( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_height: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_EXPOSURES:
        ret = fw_intf_get_sensor_info_exposures( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_exposures: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_CHANNELS:
        ret = fw_intf_get_sensor_info_channels( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_channels: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_DATA_WIDTH:
        ret = fw_intf_get_sensor_info_data_width( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_info_data_width: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_LIMIT:
        ret = fw_intf_get_sensor_integration_time_limit( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_integration_time_limit: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_MIN:
        ret = fw_intf_get_sensor_integration_time_min( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get sensor_integration_time_min: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_FREEZE_FIRMWARE:
        ret = fw_intf_get_system_freeze_firmware( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_freeze_firmware: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE:
        ret = fw_intf_get_system_manual_exposure( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_exposure: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_INTEGRATION_TIME:
        ret = fw_intf_get_system_manual_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_integration_time: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_MAX_INTEGRATION_TIME:
        ret = fw_intf_get_system_manual_max_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_max_integration_time: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_ANALOG_GAIN:
        ret = fw_intf_get_system_manual_sensor_analog_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_sensor_analog_gain: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN:
        ret = fw_intf_get_system_manual_sensor_digital_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_sensor_digital_gain: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_ISP_DIGITAL_GAIN:
        ret = fw_intf_get_system_manual_isp_digital_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_isp_digital_gain: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_DIRECTIONAL_SHARPENING:
        ret = fw_intf_get_system_manual_directional_sharpening( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_directional_sharpening: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING:
        ret = fw_intf_get_system_manual_un_directional_sharpening( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_un_directional_sharpening: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE_RATIO:
        ret = fw_intf_get_system_manual_exposure_ratio( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_exposure_ratio: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_AWB:
        ret = fw_intf_get_system_manual_awb( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_awb: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_ANTIFLICKER_ENABLE:
        ret = fw_intf_get_system_antiflicker_enable( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_antiflicker_enable: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_SATURATION:
        ret = fw_intf_get_system_manual_saturation( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_manual_saturation: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_EXPOSURE_RATIO:
        ret = fw_intf_get_system_max_exposure_ratio( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_max_exposure_ratio: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_EXPOSURE:
        ret = fw_intf_get_system_exposure( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_exposure: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME:
        ret = fw_intf_get_system_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_integration_time: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_INTEGRATION_TIME:
        ret = fw_intf_get_system_max_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_max_integration_time: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_EXPOSURE_RATIO:
        ret = fw_intf_get_system_exposure_ratio( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_exposure_ratio: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_SENSOR_ANALOG_GAIN:
        ret = fw_intf_get_system_sensor_analog_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_sensor_analog_gain: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_SENSOR_ANALOG_GAIN:
        ret = fw_intf_get_system_max_sensor_analog_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_max_sensor_analog_gain: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_SENSOR_DIGITAL_GAIN:
        ret = fw_intf_get_system_sensor_digital_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_sensor_digital_gain: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_SENSOR_DIGITAL_GAIN:
        ret = fw_intf_get_system_max_sensor_digital_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_max_sensor_digital_gain: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_ISP_DIGITAL_GAIN:
        ret = fw_intf_get_system_isp_digital_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_isp_digital_gain: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_ISP_DIGITAL_GAIN:
        ret = fw_intf_get_system_max_isp_digital_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_max_isp_digital_gain: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_DIRECTIONAL_SHARPENING_TARGET:
        ret = fw_intf_get_system_directional_sharpening_target( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_directional_sharpening_target: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET:
        ret = fw_intf_get_system_un_directional_sharpening_target( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_un_directional_sharpening_target: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_AWB_RED_GAIN:
        ret = fw_intf_get_system_awb_red_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_awb_red_gain: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_AWB_BLUE_GAIN:
        ret = fw_intf_get_system_awb_blue_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_awb_blue_gain: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_AWB_CCT:
        ret = fw_intf_get_system_awb_cct( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_awb_cct: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_SATURATION_TARGET:
        ret = fw_intf_get_system_saturation_target( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_saturation_target: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_ANTI_FLICKER_FREQUENCY:
        ret = fw_intf_get_system_anti_flicker_frequency( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_anti_flicker_frequency: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_SHORT_INTEGRATION_TIME:
        ret = fw_intf_get_system_short_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_short_integration_time: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MIDDLE_INTEGRATION_TIME:
        ret = fw_intf_get_system_middle_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get ystem_middle_integration_time: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MIDDLE2_INTEGRATION_TIME:
        ret = fw_intf_get_system_middle2_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_middle_integration_time: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_LONG_INTEGRATION_TIME:
        ret = fw_intf_get_system_long_integration_time( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_long_integration_time: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME_PRECISION:
        ret = fw_intf_get_system_integration_time_precision( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_integration_time_precision: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_IRIDIX_DIGITAL_GAIN:
        ret = fw_intf_get_system_iridix_digital_gain( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_iridix_digital_gain: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_SINTER_THRESHOLD_TARGET:
        ret = fw_intf_get_system_sinter_threshold_target( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_sinter_threshold_target: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MINIMUM_IRIDIX_STRENGTH:
        ret = fw_intf_get_system_minimum_iridix_strength( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_minimum_iridix_strength: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MAXIMUM_IRIDIX_STRENGTH:
        ret = fw_intf_get_system_maximum_iridix_strength( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_maximum_iridix_strength: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_IRIDIX_STRENGTH_TARGET:
        ret = fw_intf_get_system_iridix_strength_target( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_iridix_strength_target: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_BUFFER_DATA_TYPE_ID:
        ret = fw_intf_get_system_buffer_data_type( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_system_buffer_data_type: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_CMD_INTERFACE_MODE_ID:
        ret = fw_intf_get_system_cmd_interface_mode( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_system_cmd_interface_mode: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_LOGGER_LEVEL_ID:
        ret = fw_intf_get_system_logger_level( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_system_logger_level: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_LOGGER_MASK_ID:
        ret = fw_intf_get_system_logger_mask( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_system_logger_mask: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_CONTEXT_STATE_ID:
        ret = fw_intf_get_system_context_state( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_system_context_state: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_MCFE_USECASE_ID:
        ret = fw_intf_get_system_mcfe_usecase( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_system_context_state: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_IRIDIX:
        ret = fw_intf_get_isp_modules_manual_iridix( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_iridix: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_SINTER:
        ret = fw_intf_get_isp_modules_manual_sinter( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_sinter: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_FRAME_STITCH:
        ret = fw_intf_get_isp_modules_manual_frame_stitch( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_frame_stitch: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_RAW_FRONTEND:
        ret = fw_intf_get_isp_modules_manual_raw_frontend( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_raw_frontend: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_BLACK_LEVEL:
        ret = fw_intf_get_isp_modules_manual_black_level( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_black_level: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_SHADING:
        ret = fw_intf_get_isp_modules_manual_shading( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_shading: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_FORCE_BIST_MISMATCH:
        ret = fw_intf_get_isp_modules_force_bist_mismatch( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_isp_modules_force_bist_mismatch: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_DEMOSAIC:
        ret = fw_intf_get_isp_modules_manual_demosaic( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_demosaic: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_M2M_PROCESS_REQUEST:
        ret = fw_intf_get_system_m2m_process_request( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_m2m_process_request: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_V4L2_INTERFACE_MODE:
        ret = fw_intf_get_system_v4l2_interface_mode( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get system_v4l2_interface_mode: %d.\n", ctrl->val );
        break;
#if ( ISP_RTL_VERSION_R == 2 )
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_CNR:
        ret = fw_intf_get_isp_modules_manual_cnr( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get isp_modules_manual_cnr: %d.\n", ctrl->val );
        break;
#endif
    case ISP_V4L2_CID_IMAGE_CROP_XOFFSET_ID:
        ret = fw_intf_get_image_crop_xoffset( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get image_crop_xoffset: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_YOFFSET_ID:
        ret = fw_intf_get_image_crop_yoffset( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get image_crop_yoffset: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_ID:
        ret = fw_intf_get_output_format_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get image_output_format_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID:
        ret = fw_intf_get_output_format_manual_cfg_apply_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_output_format_manual_cfg_apply_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_AXI1_FORMAT_ID:
        ret = fw_intf_get_output_axi1_format_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_output_axi1_format_id: %d.\n", ctrl->val );
        break;
#if ( ISP_RTL_VERSION_R == 2 )
    case ISP_V4L2_CID_IMAGE_RAW_SCALER_ENABLE_ID:
        ret = fw_intf_get_raw_scaler_enable_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_raw_scaler_enable_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RAW_SCALER_WIDTH_ID:
        ret = fw_intf_get_raw_scaler_width_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_raw_scaler_width_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RAW_SCALER_HEIGHT_ID:
        ret = fw_intf_get_raw_scaler_height_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_raw_scaler_height_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RGB_SCALER_ENABLE_ID:
        ret = fw_intf_get_rgb_scaler_enable_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_rgb_scaler_enable_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RGB_SCALER_WIDTH_ID:
        ret = fw_intf_get_rgb_scaler_width_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_rgb_scaler_width_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RGB_SCALER_HEIGHT_ID:
        ret = fw_intf_get_rgb_scaler_height_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_rgb_scaler_height_id: %d.\n", ctrl->val );
        break;
#endif //ISP_RTL_VERSION_R == 2
    case ISP_V4L2_CID_IMAGE_OUTPUT_AXI2_FORMAT_ID:
        ret = fw_intf_get_output_axi2_format_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_output_axi2_format_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_AXI3_FORMAT_ID:
        ret = fw_intf_get_output_axi3_format_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_output_axi3_format_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_HEIGHT_ID:
        ret = fw_intf_get_image_crop_height( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get image_crop_height: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_WIDTH_ID:
        ret = fw_intf_get_image_crop_width( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get image_crop_width: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_ENABLE_ID:
        ret = fw_intf_get_image_crop_enable( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get image_crop_enable: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_STATUS_INFO_EXPOSURE_LOG2:
        ret = fw_intf_get_status_info_exposure_log2( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get status_info_exposure_log2: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_STATUS_INFO_GAIN_LOG2:
        ret = fw_intf_get_status_info_gain_log2( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get status_info_gain_log2: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_STATUS_INFO_GAIN_ONES:
        ret = fw_intf_get_status_info_gain_ones( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get status_info_gain_ones: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID:
        ret = fw_intf_get_status_info_exposure_residual_log2_id( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_status_info_exposure_residual_log2_id: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_STATUS_INFO_IRIDIX_CONTRAST:
        ret = fw_intf_get_status_info_iridix_contrast( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_status_info_iridix_contrast: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_STATUS_INFO_AE_HIST_MEAN:
        ret = fw_intf_get_status_info_ae_hist_mean( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get fw_intf_get_status_info_ae_hist_mean: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_STATUS_INFO_AWB_MIX_LIGHT_CONTRAST:
        ret = fw_intf_get_status_info_awb_mix_light_contrast( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "get status_info_awb_mix_light_contrast: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_INFO_FW_REVISION:
        ret = fw_intf_get_info_fw_revision( ctx_id, ctrl->p_new.p_u32 );
        LOG( LOG_INFO, "fw_intf_get_info_fw_revision: 0x%x, rc: %d.\n", ctrl->p_new.p_u32[0], ret );
        break;
    case ISP_V4L2_CID_CONTEXT_NUMBER:
        ret = fw_intf_get_context_number( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_context_number: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_ACTIVE_CONTEXT:
        ret = fw_intf_get_active_context( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_active_context: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_REGISTERS_VALUE_ID:
        ret = fw_intf_get_register_value( ctx_id, ctrl->p_new.p_u32 );
        LOG( LOG_INFO, "fw_intf_get_register_value: %d, rc: %d.\n", ctrl->p_new.p_u32[0], ret );
        break;
    case ISP_V4L2_CID_REGISTERS_SOURCE_ID:
        ret = fw_intf_get_register_source( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_register_source: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_REGISTERS_SIZE_ID:
        ret = fw_intf_get_register_size( ctx_id, &ctrl->val );
        LOG( LOG_INFO, "fw_intf_get_register_size: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_REGISTERS_ADDRESS_ID:
        ret = fw_intf_get_register_address( ctx_id, ctrl->p_new.p_u32 );
        LOG( LOG_INFO, "fw_intf_get_register_address: 0x%x, rc: %d.\n", ctrl->p_new.p_u32[0], ret );
        break;

    default:
        LOG( LOG_INFO, "Unknown control ID: %d.\n", ctrl->id );
        break;
    }

    return ret;
}

static int isp_v4l2_ctrl_s_ctrl_custom( struct v4l2_ctrl *ctrl )
{
    int ret = -EINVAL;

    struct v4l2_ctrl_handler *hdl = ctrl->handler;

    isp_v4l2_ctrl_t *isp_ctrl = cst_hdl_to_isp_ctrl( hdl );
    int ctx_id = isp_ctrl->ctx_id;

    LOG( LOG_INFO, "Control - id:0x%x, val:%d, is_int:%d, min:%lld, max:%lld.\n",
         ctrl->id, ctrl->val, ctrl->is_int, ctrl->minimum, ctrl->maximum );

    if ( isp_v4l2_ctrl_check_valid( ctrl ) < 0 ) {
        LOG( LOG_ERR, "Invalid param: id:0x%x, val:0x%x, is_int:%d, min:0x%llx, max:0x%llx.\n",
             ctrl->id, ctrl->val, ctrl->is_int, ctrl->minimum, ctrl->maximum );

        return -EINVAL;
    }

    switch ( ctrl->id ) {
    case ISP_V4L2_CID_TEST_PATTERN:
        LOG( LOG_INFO, "new test_pattern: %d.\n", ctrl->val );
        ret = fw_intf_set_test_pattern( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_TEST_PATTERN_TYPE:
        LOG( LOG_INFO, "new test_pattern_type: %d.\n", ctrl->val );
        ret = fw_intf_set_test_pattern_type( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_PRESET:
        LOG( LOG_INFO, "new sensor preset: %d.\n", ctrl->val );
        ret = fw_intf_set_sensor_preset( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SENSOR_INFO_PRESET:
        LOG( LOG_INFO, "new sensor info preset: %d.\n", ctrl->val );
        ret = fw_intf_set_sensor_info_preset( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_FREEZE_FIRMWARE:
        LOG( LOG_INFO, "new system_freeze_firmware: %d.\n", ctrl->val );
        ret = fw_intf_set_system_freeze_firmware( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE:
        LOG( LOG_INFO, "new system_manual_exposure: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_exposure( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_INTEGRATION_TIME:
        LOG( LOG_INFO, "new system_manual_integration_time: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_integration_time( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_MAX_INTEGRATION_TIME:
        LOG( LOG_INFO, "new system_manual_max_integration_time: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_max_integration_time( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_ANALOG_GAIN:
        LOG( LOG_INFO, "new system_manual_sensor_analog_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_sensor_analog_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN:
        LOG( LOG_INFO, "new system_manual_sensor_digital_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_sensor_digital_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_ISP_DIGITAL_GAIN:
        LOG( LOG_INFO, "new system_manual_isp_digital_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_isp_digital_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_DIRECTIONAL_SHARPENING:
        LOG( LOG_INFO, "new system_manual_directional_sharpening: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_directional_sharpening( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING:
        LOG( LOG_INFO, "new system_manual_un_directional_sharpening: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_un_directional_sharpening( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE_RATIO:
        LOG( LOG_INFO, "new system_manual_exposure_ratio: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_exposure_ratio( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_AWB:
        LOG( LOG_INFO, "new system_manual_awb: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_awb( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_ANTIFLICKER_ENABLE:
        LOG( LOG_INFO, "new system_antiflicker_enable: %d.\n", ctrl->val );
        ret = fw_intf_set_system_antiflicker_enable( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MANUAL_SATURATION:
        LOG( LOG_INFO, "new system_manual_saturation: %d.\n", ctrl->val );
        ret = fw_intf_set_system_manual_saturation( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_EXPOSURE_RATIO:
        LOG( LOG_INFO, "new system_max_exposure_ratio: %d.\n", ctrl->val );
        ret = fw_intf_set_system_max_exposure_ratio( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_EXPOSURE:
        LOG( LOG_INFO, "new system_exposure: %d.\n", ctrl->val );
        ret = fw_intf_set_system_exposure( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME:
        LOG( LOG_INFO, "new system_integration_time: %d.\n", ctrl->val );
        ret = fw_intf_set_system_integration_time( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_EXPOSURE_RATIO:
        LOG( LOG_INFO, "new system_exposure_ratio: %d.\n", ctrl->val );
        ret = fw_intf_set_system_exposure_ratio( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_INTEGRATION_TIME:
        LOG( LOG_INFO, "new system_max_integration_time: %d.\n", ctrl->val );
        ret = fw_intf_set_system_max_integration_time( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_SENSOR_ANALOG_GAIN:
        LOG( LOG_INFO, "new system_sensor_analog_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_sensor_analog_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_SENSOR_ANALOG_GAIN:
        LOG( LOG_INFO, "new system_max_sensor_analog_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_max_sensor_analog_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_SENSOR_DIGITAL_GAIN:
        LOG( LOG_INFO, "new system_sensor_digital_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_sensor_digital_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_SENSOR_DIGITAL_GAIN:
        LOG( LOG_INFO, "new system_max_sensor_digital_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_max_sensor_digital_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_ISP_DIGITAL_GAIN:
        LOG( LOG_INFO, "new system_isp_digital_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_isp_digital_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MAX_ISP_DIGITAL_GAIN:
        LOG( LOG_INFO, "new system_max_isp_digital_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_max_isp_digital_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_DIRECTIONAL_SHARPENING_TARGET:
        LOG( LOG_INFO, "new system_directional_sharpening_target: %d.\n", ctrl->val );
        ret = fw_intf_set_system_directional_sharpening_target( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET:
        LOG( LOG_INFO, "new system_un_directional_sharpening_target: %d.\n", ctrl->val );
        ret = fw_intf_set_system_un_directional_sharpening_target( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_AWB_RED_GAIN:
        LOG( LOG_INFO, "new system_awb_red_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_awb_red_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_AWB_BLUE_GAIN:
        LOG( LOG_INFO, "new system_awb_blue_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_awb_blue_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_AWB_CCT:
        LOG( LOG_INFO, "new system_awb_cct: %d.\n", ctrl->val );
        ret = fw_intf_set_system_awb_cct( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_SATURATION_TARGET:
        LOG( LOG_INFO, "new system_saturation_target: %d.\n", ctrl->val );
        ret = fw_intf_set_system_saturation_target( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_ANTI_FLICKER_FREQUENCY:
        LOG( LOG_INFO, "new system_anti_flicker_frequency: %d.\n", ctrl->val );
        ret = fw_intf_set_system_anti_flicker_frequency( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_IRIDIX_DIGITAL_GAIN:
        LOG( LOG_INFO, "new system_iridix_digital_gain: %d.\n", ctrl->val );
        ret = fw_intf_set_system_iridix_digital_gain( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_SINTER_THRESHOLD_TARGET:
        LOG( LOG_INFO, "new system_sinter_threshold_target: %d.\n", ctrl->val );
        ret = fw_intf_set_system_sinter_threshold_target( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MINIMUM_IRIDIX_STRENGTH:
        LOG( LOG_INFO, "new system_minimum_iridix_strength: %d.\n", ctrl->val );
        ret = fw_intf_set_system_minimum_iridix_strength( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_MAXIMUM_IRIDIX_STRENGTH:
        LOG( LOG_INFO, "new system_maximum_iridix_strength: %d.\n", ctrl->val );
        ret = fw_intf_set_system_maximum_iridix_strength( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_IRIDIX_STRENGTH_TARGET:
        LOG( LOG_INFO, "new system_iridix_strength_target: %d.\n", ctrl->val );
        ret = fw_intf_set_system_iridix_strength_target( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_LOGGER_LEVEL_ID:
        ret = fw_intf_set_system_logger_level( ctx_id, ctrl->val );
        LOG( LOG_INFO, "fw_intf_set_system_logger_level: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_SYSTEM_LOGGER_MASK_ID:
        ret = fw_intf_set_system_logger_mask( ctx_id, ctrl->val );
        LOG( LOG_INFO, "fw_intf_set_system_logger_mask: %d, rc: %d.\n", ctrl->val, ret );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_IRIDIX:
        LOG( LOG_INFO, "new isp_modules_manual_iridix: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_iridix( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_SINTER:
        LOG( LOG_INFO, "new isp_modules_manual_sinter: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_sinter( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_FRAME_STITCH:
        LOG( LOG_INFO, "new isp_modules_manual_frame_stitch: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_frame_stitch( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_RAW_FRONTEND:
        LOG( LOG_INFO, "new isp_modules_manual_raw_frontend: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_raw_frontend( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_BLACK_LEVEL:
        LOG( LOG_INFO, "new isp_modules_manual_black_level: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_black_level( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_SHADING:
        LOG( LOG_INFO, "new isp_modules_manual_shading: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_shading( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_FORCE_BIST_MISMATCH:
        LOG( LOG_INFO, "new fw_intf_set_isp_modules_force_bist_mismatch: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_force_bist_mismatch( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_DEMOSAIC:
        LOG( LOG_INFO, "new isp_modules_manual_demosaic: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_demosaic( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_M2M_PROCESS_REQUEST:
        LOG( LOG_INFO, "new system_m2m_process_request: %d.\n", ctrl->val );
        ret = fw_intf_set_system_m2m_process_request( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_SYSTEM_V4L2_INTERFACE_MODE:
        LOG( LOG_INFO, "new system_v4l2_interface_mode: %d.\n", ctrl->val );
        ret = fw_intf_set_system_v4l2_interface_mode( ctx_id, ctrl->val );
        break;
#if ( ISP_RTL_VERSION_R == 2 )
    case ISP_V4L2_CID_ISP_MODULES_MANUAL_CNR:
        LOG( LOG_INFO, "new isp_modules_manual_cnr: %d.\n", ctrl->val );
        ret = fw_intf_set_isp_modules_manual_cnr( ctx_id, ctrl->val );
        break;
#endif
    case ISP_V4L2_CID_IMAGE_CROP_XOFFSET_ID:
        LOG( LOG_INFO, "new image_crop_xoffset: %d.\n", ctrl->val );
        ret = fw_intf_set_image_crop_xoffset( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_YOFFSET_ID:
        LOG( LOG_INFO, "new image_crop_yoffset: %d.\n", ctrl->val );
        ret = fw_intf_set_image_crop_yoffset( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_ID:
        ret = fw_intf_set_output_format_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set image_output_format_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID:
        ret = fw_intf_set_output_format_manual_cfg_apply_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_set_output_format_manual_cfg_apply_id: %d.\n", ctrl->val );
        break;
#if ( ISP_RTL_VERSION_R == 2 )
    case ISP_V4L2_CID_IMAGE_RAW_SCALER_ENABLE_ID:
        ret = fw_intf_set_raw_scaler_enable_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_set_raw_scaler_enable_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RAW_SCALER_WIDTH_ID:
        ret = fw_intf_set_raw_scaler_width_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_set_raw_scaler_width_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RAW_SCALER_HEIGHT_ID:
        ret = fw_intf_set_raw_scaler_height_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_set_raw_scaler_height_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RGB_SCALER_ENABLE_ID:
        ret = fw_intf_set_rgb_scaler_enable_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_set_rgb_scaler_enable_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RGB_SCALER_WIDTH_ID:
        ret = fw_intf_set_rgb_scaler_width_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_set_rgb_scaler_width_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_RGB_SCALER_HEIGHT_ID:
        ret = fw_intf_set_rgb_scaler_height_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_set_rgb_scaler_height_id: %d.\n", ctrl->val );
        break;
#endif
    case ISP_V4L2_CID_IMAGE_OUTPUT_AXI1_FORMAT_ID:
        ret = fw_intf_set_output_axi1_format_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_get_output_axi1_format_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_AXI2_FORMAT_ID:
        ret = fw_intf_set_output_axi2_format_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_get_output_axi2_format_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_OUTPUT_AXI3_FORMAT_ID:
        ret = fw_intf_set_output_axi3_format_id( ctx_id, ctrl->val );
        LOG( LOG_INFO, "set fw_intf_get_output_axi3_format_id: %d.\n", ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_HEIGHT_ID:
        LOG( LOG_INFO, "new image_crop_height: %d.\n", ctrl->val );
        ret = fw_intf_set_image_crop_height( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_WIDTH_ID:
        LOG( LOG_INFO, "new image_crop_width: %d.\n", ctrl->val );
        ret = fw_intf_set_image_crop_width( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_IMAGE_CROP_ENABLE_ID:
        LOG( LOG_INFO, "new image_crop_enable: %d.\n", ctrl->val );
        ret = fw_intf_set_image_crop_enable( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_REGISTERS_VALUE_ID:
        LOG( LOG_INFO, "new register value : %d.\n", ctrl->p_new.p_u32[0] );
        ret = fw_intf_set_register_value( ctx_id, ctrl->p_new.p_u32[0] );
        break;
    case ISP_V4L2_CID_REGISTERS_SOURCE_ID:
        LOG( LOG_INFO, "new register source : %d.\n", ctrl->val );
        ret = fw_intf_set_register_source( ctx_id, ctrl->val );
        break;
    case ISP_V4L2_CID_REGISTERS_SIZE_ID:
        LOG( LOG_INFO, "new register size : %d.\n", ctrl->val );
        ret = fw_intf_set_register_size( ctx_id, ( 1 << ( ctrl->val + 3 ) ) );
        break;
    case ISP_V4L2_CID_REGISTERS_ADDRESS_ID:
        LOG( LOG_INFO, "new register address : 0x%x.\n", ctrl->p_new.p_u32[0] );
        ret = fw_intf_set_register_address( ctx_id, ctrl->p_new.p_u32[0] );
        break;
    case ISP_V4L2_CID_ACTIVE_CONTEXT:
        LOG( LOG_INFO, "new active context : %d.\n", ctrl->val );
        ret = fw_intf_set_active_context( ctx_id, ctrl->val );
        break;
    default:
        LOG( LOG_INFO, "Unknown control ID: %d.\n", ctrl->id );
    }
    return ret;
}

static const struct v4l2_ctrl_ops isp_v4l2_ctrl_ops_custom = {
    .s_ctrl = isp_v4l2_ctrl_s_ctrl_custom,
    .g_volatile_ctrl = isp_v4l2_ctrl_g_ctrl_custom,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_test_pattern = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_TEST_PATTERN,
    .name = "ISP Test Pattern",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const char *isp_v4l2_ctrl_test_pattern_type_menu_values[] = {
    "TPG_FLAT_FIELD",
    "TPG_H_GRADIENT",
    "TPG_V_GRADIENT",
    "TPG_V_BARS",
    "TPG_ARB_RECT",
    "TPG_RANDOM",
    NULL};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_test_pattern_type = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_TEST_PATTERN_TYPE,
    .name = "ISP Test Pattern Type",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0, /*PG_FLAT_FIELD*/
    .max = 5, /*TPG_RANDOM*/
    .def = 3, /*TPG_V_BARS */
    .qmenu = isp_v4l2_ctrl_test_pattern_type_menu_values,
};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_supported_presets = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_SUPPORTED_PRESETS,
    .name = "Sensor supported presets",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 16,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_preset = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_PRESET,
    .name = "ISP Sensor Preset",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 16,
    .step = 1,
    .def = 0,
};

static const char *isp_v4l2_ctrl_ctrl_sensor_wdr_mode[] = {
    "WDR_MODE_LINEAR",
    "WDR_MODE_NATIVE",
    "WDR_MODE_FS_HDR",
    "WDR_MODE_FS_LIN",
    NULL};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_wdr_mode = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_WDR_MODE,
    .name = "Sensor wdr mode",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0,
    .max = 3,
    .def = 0,
    .qmenu = isp_v4l2_ctrl_ctrl_sensor_wdr_mode,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_streaming = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_STREAMING,
    .name = "Sensor streaming",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_exposures = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_EXPOSURES,
    .name = "Sensor exposures",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 4,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_fps = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_FPS,
    .name = "Sensor FPS",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 240,
    .step = 1,
    .def = 60,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_width = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_WIDTH,
    .name = "Sensor width",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 7680,
    .step = 1,
    .def = 1920,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_height = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_HEIGHT,
    .name = "Sensor height",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 4320,
    .step = 1,
    .def = 1080,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_info_preset = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_PRESET,
    .name = "Sensor info preset",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 16,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_info_wdr_mode = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_WDR_MODE,
    .name = "Sensor info wdr mode",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0,
    .max = 3,
    .qmenu = isp_v4l2_ctrl_ctrl_sensor_wdr_mode,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_info_fps = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_FPS,
    .name = "Sensor info fps",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 240,
    .step = 1,
    .def = 60,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_info_width = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_WIDTH,
    .name = "Sensor info width",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 7680,
    .step = 1,
    .def = 1920,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_info_height = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_HEIGHT,
    .name = "Sensor info height",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 4320,
    .step = 1,
    .def = 1080,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_info_exposures = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_EXPOSURES,
    .name = "Sensor info exposures",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 4,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_info_channels = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_CHANNELS,
    .name = "Sensor info channels",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 4,
    .step = 1,
    .def = 1,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_data_width = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INFO_DATA_WIDTH,
    .name = "Sensor data width",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 32,
    .step = 1,
    .def = 12,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_integration_time_limit = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_LIMIT,
    .name = "Sensor integration time max",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_sensor_integration_time_min = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_MIN,
    .name = "Sensor integration time min",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_freeze_firmware = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_FREEZE_FIRMWARE,
    .name = "Freeze ISP firmware",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_exposure = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE,
    .name = "En manual exposure",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_INTEGRATION_TIME,
    .name = "En manual integration time",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_max_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_MAX_INTEGRATION_TIME,
    .name = "En manual max integration time",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_sensor_analog_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_ANALOG_GAIN,
    .name = "En manual sensor analog gain",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_sensor_digital_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN,
    .name = "En manual sensor digital gain",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_isp_digital_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_ISP_DIGITAL_GAIN,
    .name = "En manual ISP digital gain",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_directional_sharpening = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_DIRECTIONAL_SHARPENING,
    .name = "Syst man direct sharpening",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_un_directional_sharpening = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING,
    .name = "Syst man un direct sharpening",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_exposure_ratio = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE_RATIO,
    .name = "En manual exposure ratio",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_awb = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_AWB,
    .name = "En manual AWB",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_antiflicker_enable = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_ANTIFLICKER_ENABLE,
    .name = "System antiflicker enable",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_manual_saturation = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MANUAL_SATURATION,
    .name = "En manual saturation",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_max_exposure_ratio = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MAX_EXPOSURE_RATIO,
    .name = "Maximum exposure ratio",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 256,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_exposure = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_EXPOSURE,
    .name = "Current exposure",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME,
    .name = "Current integration time",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 1000000,
    .step = 1,
    .def = 5000,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_short_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_SHORT_INTEGRATION_TIME,
    .name = "Get short integration time",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_middle_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MIDDLE_INTEGRATION_TIME,
    .name = "Get middle integration time",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_middle2_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MIDDLE2_INTEGRATION_TIME,
    .name = "Get middle2 integration time",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_long_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_LONG_INTEGRATION_TIME,
    .name = "Get long integration time",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_integration_time_precision = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME_PRECISION,
    .name = "Get integration time precision",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7FFFFFFF,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_exposure_ratio = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_EXPOSURE_RATIO,
    .name = "Current exposure ratio",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 256,
    .step = 1,
    .def = SYSTEM_EXPOSURE_RATIO_DEFAULT,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_max_integration_time = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MAX_INTEGRATION_TIME,
    .name = "Max integration time",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 1000000,
    .step = 1,
    .def = 5000,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_sensor_analog_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_SENSOR_ANALOG_GAIN,
    .name = "Sensor analog gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_max_sensor_analog_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MAX_SENSOR_ANALOG_GAIN,
    .name = "Max sensor analog gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_sensor_digital_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_SENSOR_DIGITAL_GAIN,
    .name = "Sensor digital gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_max_sensor_digital_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MAX_SENSOR_DIGITAL_GAIN,
    .name = "Max sensor digital gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_isp_digital_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_ISP_DIGITAL_GAIN,
    .name = "Isp digital gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_max_isp_digital_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MAX_ISP_DIGITAL_GAIN,
    .name = "Max isp digital gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_directional_sharpening_target = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_DIRECTIONAL_SHARPENING_TARGET,
    .name = "Syst direct sharpening target",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_un_directional_sharpening_target = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET,
    .name = "Syst un direct sharp target",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_awb_red_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_AWB_RED_GAIN,
    .name = "AWB red gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 256,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_awb_blue_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_AWB_BLUE_GAIN,
    .name = "AWB blue gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 256,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_awb_cct = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_AWB_CCT,
    .name = "System awb cct",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 5000,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_saturation_target = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_SATURATION_TARGET,
    .name = "System saturation target",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_anti_flicker_frequency = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_ANTI_FLICKER_FREQUENCY,
    .name = "System anti flicker frequency",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_iridix_digital_gain = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_IRIDIX_DIGITAL_GAIN,
    .name = "System iridix digital gain",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 256,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_sinter_threshold_target = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_SINTER_THRESHOLD_TARGET,
    .name = "system sinter threshold target",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_minimum_iridix_strength = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MINIMUM_IRIDIX_STRENGTH,
    .name = "System minimum iridix strength",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_maximum_iridix_strength = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MAXIMUM_IRIDIX_STRENGTH,
    .name = "System maximum iridix strength",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 255,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_system_iridix_strength_target = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_IRIDIX_STRENGTH_TARGET,
    .name = "System iridix strength target",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_iridix = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_IRIDIX,
    .name = "En manual iridix",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_sinter = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_SINTER,
    .name = "En manual sinter",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_frame_stitch = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_FRAME_STITCH,
    .name = "En manual frame stitch",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_raw_frontend = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_RAW_FRONTEND,
    .name = "En manual raw frontend",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_black_level = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_BLACK_LEVEL,
    .name = "En manual black level",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_shading = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_SHADING,
    .name = "En manual shading",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_demosaic = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_DEMOSAIC,
    .name = "En manual demosaic",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_force_bist_mismatch = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_FORCE_BIST_MISMATCH,
    .name = "Force CRC bist mismatch",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_cmd_system_m2m_process_request = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_M2M_PROCESS_REQUEST,
    .name = "M2M process request",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const char *isp_v4l2_ctrl_cmd_system_v4l2_interface_mode_menu_values[] = {
    "V4L2_INTERFACE_MODE_NONE",
    "V4L2_INTERFACE_MODE_CAPTURE",
    "V4L2_INTERFACE_MODE_M2M",
    NULL,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_cmd_system_v4l2_interface_mode = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_V4L2_INTERFACE_MODE,
    .name = "V4L2 interface mode",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0, /*V4L2_INTERFACE_MODE_NONE*/
    .max = 2, /*V4L2_INTERFACE_MODE_M2M*/
    .def = 0,
    .qmenu = isp_v4l2_ctrl_cmd_system_v4l2_interface_mode_menu_values,
};

#if ( ISP_RTL_VERSION_R == 2 )
static const struct v4l2_ctrl_config isp_v4l2_ctrl_isp_modules_manual_cnr = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ISP_MODULES_MANUAL_CNR,
    .name = "Enable manual CNR",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};
#endif

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_crop_xoffset = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_CROP_XOFFSET_ID,
    .name = "Image crop xoffset",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_crop_yoffset = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_CROP_YOFFSET_ID,
    .name = "Image crop yoffset",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 0,
};

static const char *isp_v4l2_ctrl_image_output_format_id_menu_values[] = {
    "OF_MODE_MANUAL",
    "OF_MODE_DISABLE",
    "OF_MODE_R14G14B14",
    "OF_MODE_RGB888",
    "OF_MODE_Y14UV88",
    "OF_MODE_Y14VU88",
    "OF_MODE_HS88",
    "OF_MODE_H_LSB",
    "OF_MODE_H_MSB",
    "OF_MODE_S_14_LSB",
    "OF_MODE_S_14_MSB",
    "OF_MODE_S2_8",
    "OF_MODE_S2_14_LSB",
    "OF_MODE_S2_14_MSB",
    "OF_MODE_L14U14V14",
    "OF_MODE_L14UV88",
    "OF_MODE_L14VU88",
    "OF_MODE_RAW8",
    "OF_MODE_RAW10",
    "OF_MODE_RAW12",
    "OF_MODE_RAW16",
    "OF_MODE_RAW24",
    "OF_MODE_Y8UV88",
    "OF_MODE_Y8UV88_2X1",
    "OF_MODE_Y8UV88_2X2",
    "OF_MODE_Y8VU88",
    "OF_MODE_Y8VU88_2X1",
    "OF_MODE_Y8VU88_2X2",
    "OF_MODE_Y8U8V8",
    "OF_MODE_Y14U14V14_MSB",
    "OF_MODE_Y14U14V14_LSB",
    "OF_MODE_Y14_MSB",
    "OF_MODE_Y14_LSB",
    "OF_MODE_Y8",
    "OF_MODE_Y10",
    "OF_MODE_L14MSB_U14LSB_V14LSB",
    "OF_MODE_BGR888",
    "OF_MODE_ARGB32",
    "OF_MODE_BGRA32",
#if ( ISP_RTL_VERSION_R == 2 )
    "OF_MODE_RGBA32",
    "OF_MODE_ABGR32",
#endif
    "OF_MODE_IR14LSB",
    "OF_MODE_IR14MSB",
    "OF_MODE_IR8",
    "OF_MODE_R14LSB",
    "OF_MODE_R14MSB",
    NULL};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_output_format_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_ID,
    .name = "Image output format ID",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0, /*OF_MODE_MANUAL*/
#if ( ISP_RTL_VERSION_R == 2 )
    .max = 0x2D, /*OF_MODE_R14MSB*/
#else
    .max = 0x2B, /*OF_MODE_R14MSB*/
#endif
    .def = 0x26, /*OF_MODE_BGRA32 */
    .qmenu = isp_v4l2_ctrl_image_output_format_id_menu_values,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_output_format_manual_cfg_apply_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID,
    .name = "Output format manual cfg ID",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};
#if ( ISP_RTL_VERSION_R == 2 )
static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_raw_scaler_enable_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_RAW_SCALER_ENABLE_ID,
    .name = "Raw scaler enable id",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_raw_scaler_width_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_RAW_SCALER_WIDTH_ID,
    .name = "Raw scaler width id",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 4095,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_raw_scaler_height_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_RAW_SCALER_HEIGHT_ID,
    .name = "Raw scaler height id",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 4095,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_rgb_scaler_enable_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_RGB_SCALER_ENABLE_ID,
    .name = "Rgb scaler enable id",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_rgb_scaler_width_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_RGB_SCALER_WIDTH_ID,
    .name = "Rgb scaler width id",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_rgb_scaler_height_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_RGB_SCALER_HEIGHT_ID,
    .name = "Rgb scaler height id",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 0,
};
#endif

static const char *isp_v4l2_ctrl_image_output_axi1_format_id_menu_values[] = {
    "OF_AXI_FORMAT_DISABLE",
    "OF_AXI_FORMAT_R14_LSB",
    "OF_AXI_FORMAT_R14_MSB",
    "OF_AXI_FORMAT_G14_LSB",
    "OF_AXI_FORMAT_G14_MSB",
    "OF_AXI_FORMAT_B14_LSB",
    "OF_AXI_FORMAT_B14_MSB",
    "OF_AXI_FORMAT_RGB_RGB24",
    "OF_AXI_FORMAT_RGB_BGR24",
    "OF_AXI_FORMAT_RGB_ARGB32",
    "OF_AXI_FORMAT_RGB_BGRA32",
#if ( ISP_RTL_VERSION_R == 2 )
    "OF_AXI_FORMAT_RGB_RGBA32",
    "OF_AXI_FORMAT_RGB_ABGR32",
#endif
    "OF_AXI_FORMAT_Y_Y10",
    "OF_AXI_FORMAT_YUV_Y8",
    "OF_AXI_FORMAT_YUV_Y14_LSB",
    "OF_AXI_FORMAT_YUV_Y14_MSB",
    "OF_AXI_FORMAT_YUV_U8",
    "OF_AXI_FORMAT_YUV_U14_LSB",
    "OF_AXI_FORMAT_YUV_U14_MSB",
    "OF_AXI_FORMAT_YUV_V8",
    "OF_AXI_FORMAT_YUV_V14_LSB",
    "OF_AXI_FORMAT_YUV_V14_MSB",
    "OF_AXI_FORMAT_YUV_UV88",
    "OF_AXI_FORMAT_YUV_UV88_DOWNSAMPLED",
    "OF_AXI_FORMAT_YUV_UV88_DOWNSAMPLED_H",
    "OF_AXI_FORMAT_YUV_VU88",
    "OF_AXI_FORMAT_YUV_VU88_DOWNSAMPLED",
    "OF_AXI_FORMAT_YUV_VU88_DOWNSAMPLED_H",
    "OF_AXI_FORMAT_S_S8",
    "OF_AXI_FORMAT_S_S14_LSB",
    "OF_AXI_FORMAT_S_S14_MSB",
    "OF_AXI_FORMAT_HS_H14_LSB",
    "OF_AXI_FORMAT_HS_H14_MSB",
    "OF_AXI_FORMAT_HS_S14_LSB",
    "OF_AXI_FORMAT_HS_S14_MSB",
    "OF_AXI_FORMAT_HS_HS88",
    "OF_AXI_FORMAT_LUV_L14",
    "OF_AXI_FORMAT_LUV_U14_LSB",
    "OF_AXI_FORMAT_LUV_U14_MSB",
    "OF_AXI_FORMAT_LUV_V14_LSB",
    "OF_AXI_FORMAT_LUV_V14_MSB",
    "OF_AXI_FORMAT_LUV_UV88",
    "OF_AXI_FORMAT_LUV_VU88",
    "OF_AXI_FORMAT_IR_IR8",
    "OF_AXI_FORMAT_IR_IR14_LSB",
    "OF_AXI_FORMAT_IR_IR14_MSB",
    "OF_AXI_FORMAT_RAW_RAW8",
    "OF_AXI_FORMAT_RAW_RAW10_LSB",
    "OF_AXI_FORMAT_RAW_RAW10_MSB",
    "OF_AXI_FORMAT_RAW_RAW10_DENSE",
    "OF_AXI_FORMAT_RAW_RAW12_LSB",
    "OF_AXI_FORMAT_RAW_RAW12_MSB",
    "OF_AXI_FORMAT_RAW_RAW12_DENSE",
    "OF_AXI_FORMAT_RAW_RAW16",
    "OF_AXI_FORMAT_RAW_RAW24",
    "OF_AXI_FORMAT_RAW_RAW24_DENSE",
    NULL};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_output_axi1_format_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_OUTPUT_AXI1_FORMAT_ID,
    .name = "Image output AXI1 format ID",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0, /*OF_AXI_FORMAT_DISABLE*/
#if ( ISP_RTL_VERSION_R == 2 )
    .max = 0x38, /*OF_AXI_FORMAT_RAW_RAW24_DENSE*/
#else
    .max = 0x36, /*OF_AXI_FORMAT_RAW_RAW24_DENSE*/
#endif
    .def = 0xA, /*OF_AXI_FORMAT_RGB_BGRA32*/
    .qmenu = isp_v4l2_ctrl_image_output_axi1_format_id_menu_values,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_output_axi2_format_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_OUTPUT_AXI2_FORMAT_ID,
    .name = "Image output AXI2 format ID",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0, /*OF_AXI_FORMAT_DISABLE*/
#if ( ISP_RTL_VERSION_R == 2 )
    .max = 0x38, /*OF_AXI_FORMAT_RAW_RAW24_DENSE*/
#else
    .max = 0x36, /*OF_AXI_FORMAT_RAW_RAW24_DENSE*/
#endif
    .def = 0xA, /*OF_AXI_FORMAT_RGB_BGRA32*/
    .qmenu = isp_v4l2_ctrl_image_output_axi1_format_id_menu_values,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_output_axi3_format_id = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_OUTPUT_AXI3_FORMAT_ID,
    .name = "Image output AXI3 format ID",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0, /*OF_AXI_FORMAT_DISABLE*/
#if ( ISP_RTL_VERSION_R == 2 )
    .max = 0x38, /*OF_AXI_FORMAT_RAW_RAW24_DENSE*/
#else
    .max = 0x36, /*OF_AXI_FORMAT_RAW_RAW24_DENSE*/
#endif
    .def = 0xA, /*OF_AXI_FORMAT_RGB_BGRA32*/
    .qmenu = isp_v4l2_ctrl_image_output_axi1_format_id_menu_values,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_crop_height = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_CROP_HEIGHT_ID,
    .name = "Image crop height",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_crop_width = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_CROP_WIDTH_ID,
    .name = "Image crop width",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 65535,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_image_crop_enable = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_IMAGE_CROP_ENABLE_ID,
    .name = "Image crop enable",
    .type = V4L2_CTRL_TYPE_BOOLEAN,
    .min = 0,
    .max = 1,
    .step = 1,
    .def = 0,
};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_status_info_exposure_log2 = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_STATUS_INFO_EXPOSURE_LOG2,
    .name = "Get expososure log2",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_status_info_gain_log2 = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_STATUS_INFO_GAIN_LOG2,
    .name = "Get gain log2",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_status_info_gain_ones = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_STATUS_INFO_GAIN_ONES,
    .name = "Get gain ones",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_status_info_exposure_residual_log2 = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID,
    .name = "Get exposure residual log2",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_status_info_iridix_contrast = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_STATUS_INFO_IRIDIX_CONTRAST,
    .name = "Get iridix contrast",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_status_info_ae_hist_mean = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_STATUS_INFO_AE_HIST_MEAN,
    .name = "Get AE hist mean",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_status_info_awb_mix_light_contrast = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_STATUS_INFO_AWB_MIX_LIGHT_CONTRAST,
    .name = "Get AWB mix light contrast",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_info_fw_revision = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_INFO_FW_REVISION,
    .name = "Get firmware revision",
    .type = V4L2_CTRL_TYPE_U32,
    .min = 0,
    .max = 0xffffffff,
    .step = 1,
    .def = 0,
    .dims = {1},
    .elem_size = sizeof( u32 ),
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_context_number = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_CONTEXT_NUMBER,
    .name = "Context number",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = FIRMWARE_CONTEXT_NUMBER,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_active_context = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_ACTIVE_CONTEXT,
    .name = "Active context",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = FIRMWARE_CONTEXT_NUMBER - 1,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_register_value = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_REGISTERS_VALUE_ID,
    .name = "Register Value",
    .type = V4L2_CTRL_TYPE_U32,
    .min = 0,
    .max = 0xffffffff,
    .step = 1,
    .def = 0,
    .dims = {1},
    .elem_size = sizeof( u32 ),
};

static const char *isp_v4l2_ctrl_reg_source_menu_values[] = {
    "SENSOR",
    "LENS",
    "ISP",
    "MIPI",
    NULL};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_register_source = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_REGISTERS_SOURCE_ID,
    .name = "Register source",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0,
    .max = 3,
    .def = 0,
    .qmenu = isp_v4l2_ctrl_reg_source_menu_values,
};

static const s64 isp_v4l2_ctrl_reg_size_menu_values[] = {
    8, 16, 32};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_register_size = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_REGISTERS_SIZE_ID,
    .name = "Register size",
    .type = V4L2_CTRL_TYPE_INTEGER_MENU,
    .min = 0,
    .max = 2,
    .def = 2,
    .qmenu_int = isp_v4l2_ctrl_reg_size_menu_values,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_register_address = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_REGISTERS_ADDRESS_ID,
    .name = "Register address",
    .type = V4L2_CTRL_TYPE_U32,
    .min = 0,
    .max = 0xffffffff,
    .step = 1,
    .def = 0,
    .dims = {1},
    .elem_size = sizeof( u32 ),
};


static const struct v4l2_ctrl_config isp_v4l2_ctrl_buffer_data_type = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_BUFFER_DATA_TYPE_ID,
    .name = "Buffer data type",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 0x7fffffff,
    .step = 1,
    .def = 0,
};

static const char *isp_v4l2_ctrl_logger_level_menu_values[] = {
    "SYSTEM_LOGGER_DEBUG",
    "SYSTEM_LOGGER_INFO",
    "SYSTEM_LOGGER_NOTICE",
    "SYSTEM_LOGGER_WARNING",
    "SYSTEM_LOGGER_ERROR",
    "SYSTEM_LOGGER_CRITICAL",
    "SYSTEM_LOGGER_ALERT",
    "SYSTEM_LOGGER_EMERG",
    "SYSTEM_LOGGER_NOTHING",
    NULL};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_logger_level = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_LOGGER_LEVEL_ID,
    .name = "Logger level",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0,
    .max = 8,
    .def = 2,
    .qmenu = isp_v4l2_ctrl_logger_level_menu_values,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_logger_mask = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_LOGGER_MASK_ID,
    .name = "Logger mask",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 255,
};

static const char *isp_v4l2_ctrl_cmd_interface_mode_menu_values[] = {
    "CMD_IF_MODE_ACTIVE",
    "CMD_IF_MODE_PASSIVE",
    NULL};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_cmd_interface_mode = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_CMD_INTERFACE_MODE_ID,
    .name = "Command interface mode",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0, /* CMD_IF_MODE_PASSIVE */
    .max = 1, /* CMD_IF_MODE_PASSIVE*/
    .def = 0,
    .qmenu = isp_v4l2_ctrl_cmd_interface_mode_menu_values,
};

static const char *isp_v4l2_ctrl_cmd_context_state_menu_values[] = {
    "CTX_STATE_UNKNOWN",
    "CTX_STATE_INIT",
    "CTX_STATE_CONFIG",
    "CTX_STATE_START",
    "CTX_STATE_STOP",
    "CTX_STATE_DEINIT",
    NULL};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_cmd_context_state = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_CONTEXT_STATE_ID,
    .name = "Context state",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0,
    .max = 5,
    .def = 0,
    .qmenu = isp_v4l2_ctrl_cmd_context_state_menu_values,
};

static const char *isp_v4l2_ctrl_cmd_mcfe_usecase_menu_values[] = {
    "NONE",
    "TDMF",
    "M2M",
    NULL};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_cmd_mcfe_usecase = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .id = ISP_V4L2_CID_SYSTEM_MCFE_USECASE_ID,
    .name = "mcfe usecase",
    .type = V4L2_CTRL_TYPE_MENU,
    .min = 0,
    .max = 2,
    .def = 0,
    .qmenu = isp_v4l2_ctrl_cmd_mcfe_usecase_menu_values,
};

static const struct v4l2_ctrl_ops isp_v4l2_ctrl_ops = {
    .s_ctrl = isp_v4l2_ctrl_s_ctrl_standard,
};

static const struct v4l2_ctrl_config isp_v4l2_ctrl_class = {
    .ops = &isp_v4l2_ctrl_ops_custom,
    .flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_WRITE_ONLY,
    .id = ISP_V4L2_CID_ISP_V4L2_CLASS,
    .name = "ARM ISP Controls",
    .type = V4L2_CTRL_TYPE_CTRL_CLASS,
};


#define ADD_CTRL_STD( id, min, max, step, def )                  \
    {                                                            \
        if ( fw_intf_validate_control( id ) ) {                  \
            v4l2_ctrl_new_std( hdl_std_ctrl, &isp_v4l2_ctrl_ops, \
                               id, min, max, step, def );        \
        }                                                        \
    }

#define ADD_CTRL_STD_MENU( id, max, skipmask, def )                   \
    {                                                                 \
        if ( fw_intf_validate_control( id ) ) {                       \
            v4l2_ctrl_new_std_menu( hdl_std_ctrl, &isp_v4l2_ctrl_ops, \
                                    id, max, skipmask, def );         \
        }                                                             \
    }


#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 1, 1 ) )
#define ADD_CTRL_CST_VOLATILE( id, cfg, priv )                          \
    {                                                                   \
        if ( fw_intf_validate_control( id ) ) {                         \
            tmp_ctrl = v4l2_ctrl_new_custom( hdl_cst_ctrl, cfg, priv ); \
            if ( tmp_ctrl )                                             \
                tmp_ctrl->flags |= ( V4L2_CTRL_FLAG_VOLATILE |          \
                                     V4L2_CTRL_FLAG_EXECUTE_ON_WRITE ); \
        }                                                               \
    }

#else
#define ADD_CTRL_CST_VOLATILE( id, cfg, priv )                          \
    {                                                                   \
        if ( fw_intf_validate_control( id ) ) {                         \
            tmp_ctrl = v4l2_ctrl_new_custom( hdl_cst_ctrl, cfg, priv ); \
            if ( tmp_ctrl )                                             \
                tmp_ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;             \
        }                                                               \
    }

#endif

#define ADD_CTRL_CST( id, cfg, priv )                        \
    {                                                        \
        if ( fw_intf_validate_control( id ) ) {              \
            v4l2_ctrl_new_custom( hdl_cst_ctrl, cfg, priv ); \
        }                                                    \
    }


static void update_ctrl_cfg_system_integration_time( uint32_t ctx_id, struct v4l2_ctrl_config *ctrl_cfg )
{
    int integration_time_min;
    int integration_time_max;
    int rc;

    *ctrl_cfg = isp_v4l2_ctrl_system_integration_time;

    rc = fw_intf_get_integration_time_limits( ctx_id, &integration_time_min, &integration_time_max );
    if ( rc == 0 ) {
        ctrl_cfg->min = integration_time_min;
        ctrl_cfg->max = integration_time_max;
        ctrl_cfg->def = integration_time_min;
    }
}

int isp_v4l2_ctrl_init( uint32_t ctx_id, isp_v4l2_ctrl_t *ctrl )
{
    struct v4l2_ctrl_handler *hdl_std_ctrl = &ctrl->ctrl_hdl_std_ctrl;
    struct v4l2_ctrl_handler *hdl_cst_ctrl = &ctrl->ctrl_hdl_cst_ctrl;
    struct v4l2_ctrl *tmp_ctrl;
    struct v4l2_ctrl_config tmp_ctrl_cfg;

    ctrl->ctx_id = ctx_id;

    LOG( LOG_INFO, "[ctrl] ctx_id#%d: ctrl: %p, hdl_std_ctrl: %p, ctrl_hdl_cst_ctrl: %p.",
         ctx_id, ctrl, hdl_std_ctrl, hdl_cst_ctrl );

    /* Init and add standard controls */
    v4l2_ctrl_handler_init( hdl_std_ctrl, 10 );
    v4l2_ctrl_new_custom( hdl_std_ctrl, &isp_v4l2_ctrl_class, NULL );

    /* Init and add custom controls */
    v4l2_ctrl_handler_init( hdl_cst_ctrl, 64 );
    v4l2_ctrl_new_custom( hdl_cst_ctrl, &isp_v4l2_ctrl_class, NULL );

    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_TEST_PATTERN, &isp_v4l2_ctrl_test_pattern, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_TEST_PATTERN_TYPE, &isp_v4l2_ctrl_test_pattern_type, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_SUPPORTED_PRESETS, &isp_v4l2_ctrl_sensor_supported_presets, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_PRESET, &isp_v4l2_ctrl_sensor_preset, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_WDR_MODE, &isp_v4l2_ctrl_sensor_wdr_mode, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_STREAMING, &isp_v4l2_ctrl_sensor_streaming, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_EXPOSURES, &isp_v4l2_ctrl_sensor_exposures, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_FPS, &isp_v4l2_ctrl_sensor_fps, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_WIDTH, &isp_v4l2_ctrl_sensor_width, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_HEIGHT, &isp_v4l2_ctrl_sensor_height, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_PRESET, &isp_v4l2_ctrl_sensor_info_preset, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_WDR_MODE, &isp_v4l2_ctrl_sensor_info_wdr_mode, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_FPS, &isp_v4l2_ctrl_sensor_info_fps, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_WIDTH, &isp_v4l2_ctrl_sensor_info_width, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_HEIGHT, &isp_v4l2_ctrl_sensor_info_height, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_EXPOSURES, &isp_v4l2_ctrl_sensor_info_exposures, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_CHANNELS, &isp_v4l2_ctrl_sensor_info_channels, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INFO_DATA_WIDTH, &isp_v4l2_ctrl_sensor_data_width, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_LIMIT, &isp_v4l2_ctrl_sensor_integration_time_limit, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_MIN, &isp_v4l2_ctrl_sensor_integration_time_min, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_FREEZE_FIRMWARE, &isp_v4l2_ctrl_system_freeze_firmware, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE, &isp_v4l2_ctrl_system_manual_exposure, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_INTEGRATION_TIME, &isp_v4l2_ctrl_system_manual_integration_time, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_MAX_INTEGRATION_TIME, &isp_v4l2_ctrl_system_manual_max_integration_time, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_ANALOG_GAIN, &isp_v4l2_ctrl_system_manual_sensor_analog_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN, &isp_v4l2_ctrl_system_manual_sensor_digital_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_ISP_DIGITAL_GAIN, &isp_v4l2_ctrl_system_manual_isp_digital_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_DIRECTIONAL_SHARPENING, &isp_v4l2_ctrl_system_manual_directional_sharpening, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING, &isp_v4l2_ctrl_system_manual_un_directional_sharpening, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE_RATIO, &isp_v4l2_ctrl_system_manual_exposure_ratio, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_AWB, &isp_v4l2_ctrl_system_manual_awb, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_ANTIFLICKER_ENABLE, &isp_v4l2_ctrl_system_antiflicker_enable, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MANUAL_SATURATION, &isp_v4l2_ctrl_system_manual_saturation, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MAX_EXPOSURE_RATIO, &isp_v4l2_ctrl_system_max_exposure_ratio, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_EXPOSURE, &isp_v4l2_ctrl_system_exposure, NULL );

    update_ctrl_cfg_system_integration_time( ctx_id, &tmp_ctrl_cfg );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME, &tmp_ctrl_cfg, NULL );

    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_EXPOSURE_RATIO, &isp_v4l2_ctrl_system_exposure_ratio, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MAX_INTEGRATION_TIME, &isp_v4l2_ctrl_system_max_integration_time, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_SHORT_INTEGRATION_TIME, &isp_v4l2_ctrl_system_short_integration_time, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_LONG_INTEGRATION_TIME, &isp_v4l2_ctrl_system_long_integration_time, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MIDDLE_INTEGRATION_TIME, &isp_v4l2_ctrl_system_middle_integration_time, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MIDDLE2_INTEGRATION_TIME, &isp_v4l2_ctrl_system_middle2_integration_time, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME_PRECISION, &isp_v4l2_ctrl_system_integration_time_precision, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_SENSOR_ANALOG_GAIN, &isp_v4l2_ctrl_system_sensor_analog_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MAX_SENSOR_ANALOG_GAIN, &isp_v4l2_ctrl_system_max_sensor_analog_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_SENSOR_DIGITAL_GAIN, &isp_v4l2_ctrl_system_sensor_digital_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MAX_SENSOR_DIGITAL_GAIN, &isp_v4l2_ctrl_system_max_sensor_digital_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_ISP_DIGITAL_GAIN, &isp_v4l2_ctrl_system_isp_digital_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MAX_ISP_DIGITAL_GAIN, &isp_v4l2_ctrl_system_max_isp_digital_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_DIRECTIONAL_SHARPENING_TARGET, &isp_v4l2_ctrl_system_directional_sharpening_target, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET, &isp_v4l2_ctrl_system_un_directional_sharpening_target, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_AWB_RED_GAIN, &isp_v4l2_ctrl_system_awb_red_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_AWB_BLUE_GAIN, &isp_v4l2_ctrl_system_awb_blue_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_AWB_CCT, &isp_v4l2_ctrl_system_awb_cct, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_SATURATION_TARGET, &isp_v4l2_ctrl_system_saturation_target, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_ANTI_FLICKER_FREQUENCY, &isp_v4l2_ctrl_system_anti_flicker_frequency, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_IRIDIX_DIGITAL_GAIN, &isp_v4l2_ctrl_system_iridix_digital_gain, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_SINTER_THRESHOLD_TARGET, &isp_v4l2_ctrl_system_sinter_threshold_target, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MINIMUM_IRIDIX_STRENGTH, &isp_v4l2_ctrl_system_minimum_iridix_strength, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MAXIMUM_IRIDIX_STRENGTH, &isp_v4l2_ctrl_system_maximum_iridix_strength, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_IRIDIX_STRENGTH_TARGET, &isp_v4l2_ctrl_system_iridix_strength_target, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_IRIDIX, &isp_v4l2_ctrl_isp_modules_manual_iridix, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_SINTER, &isp_v4l2_ctrl_isp_modules_manual_sinter, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_FRAME_STITCH, &isp_v4l2_ctrl_isp_modules_manual_frame_stitch, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_RAW_FRONTEND, &isp_v4l2_ctrl_isp_modules_manual_raw_frontend, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_BLACK_LEVEL, &isp_v4l2_ctrl_isp_modules_manual_black_level, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_SHADING, &isp_v4l2_ctrl_isp_modules_manual_shading, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_DEMOSAIC, &isp_v4l2_ctrl_isp_modules_manual_demosaic, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_FORCE_BIST_MISMATCH, &isp_v4l2_ctrl_isp_modules_force_bist_mismatch, NULL );

#if ( ISP_RTL_VERSION_R == 2 )
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ISP_MODULES_MANUAL_CNR, &isp_v4l2_ctrl_isp_modules_manual_cnr, NULL );
#endif
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_CROP_XOFFSET_ID, &isp_v4l2_ctrl_image_crop_xoffset, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_CROP_YOFFSET_ID, &isp_v4l2_ctrl_image_crop_yoffset, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_CROP_HEIGHT_ID, &isp_v4l2_ctrl_image_crop_height, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_CROP_WIDTH_ID, &isp_v4l2_ctrl_image_crop_width, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_CROP_ENABLE_ID, &isp_v4l2_ctrl_image_crop_enable, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_ID, &isp_v4l2_ctrl_image_output_format_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID, &isp_v4l2_ctrl_image_output_format_manual_cfg_apply_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_OUTPUT_AXI1_FORMAT_ID, &isp_v4l2_ctrl_image_output_axi1_format_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_OUTPUT_AXI2_FORMAT_ID, &isp_v4l2_ctrl_image_output_axi2_format_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_OUTPUT_AXI3_FORMAT_ID, &isp_v4l2_ctrl_image_output_axi3_format_id, NULL );
#if ( ISP_RTL_VERSION_R == 2 )
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_RAW_SCALER_ENABLE_ID, &isp_v4l2_ctrl_image_raw_scaler_enable_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_RAW_SCALER_WIDTH_ID, &isp_v4l2_ctrl_image_raw_scaler_width_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_RAW_SCALER_HEIGHT_ID, &isp_v4l2_ctrl_image_raw_scaler_height_id, NULL );

    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_RGB_SCALER_ENABLE_ID, &isp_v4l2_ctrl_image_rgb_scaler_enable_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_RGB_SCALER_WIDTH_ID, &isp_v4l2_ctrl_image_rgb_scaler_width_id, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_IMAGE_RGB_SCALER_HEIGHT_ID, &isp_v4l2_ctrl_image_rgb_scaler_height_id, NULL );
#endif

    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_STATUS_INFO_EXPOSURE_LOG2, &isp_v4l2_ctrl_status_info_exposure_log2, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_STATUS_INFO_GAIN_LOG2, &isp_v4l2_ctrl_status_info_gain_log2, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_STATUS_INFO_GAIN_ONES, &isp_v4l2_ctrl_status_info_gain_ones, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID, &isp_v4l2_ctrl_status_info_exposure_residual_log2, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_STATUS_INFO_IRIDIX_CONTRAST, &isp_v4l2_ctrl_status_info_iridix_contrast, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_STATUS_INFO_AE_HIST_MEAN, &isp_v4l2_ctrl_status_info_ae_hist_mean, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_STATUS_INFO_AWB_MIX_LIGHT_CONTRAST, &isp_v4l2_ctrl_status_info_awb_mix_light_contrast, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_INFO_FW_REVISION, &isp_v4l2_ctrl_info_fw_revision, NULL );

    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_CONTEXT_NUMBER, &isp_v4l2_ctrl_context_number, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_ACTIVE_CONTEXT, &isp_v4l2_ctrl_active_context, NULL );

    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_REGISTERS_VALUE_ID, &isp_v4l2_ctrl_register_value, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_REGISTERS_SOURCE_ID, &isp_v4l2_ctrl_register_source, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_REGISTERS_SIZE_ID, &isp_v4l2_ctrl_register_size, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_REGISTERS_ADDRESS_ID, &isp_v4l2_ctrl_register_address, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_BUFFER_DATA_TYPE_ID, &isp_v4l2_ctrl_buffer_data_type, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_LOGGER_LEVEL_ID, &isp_v4l2_ctrl_logger_level, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_LOGGER_MASK_ID, &isp_v4l2_ctrl_logger_mask, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_CMD_INTERFACE_MODE_ID, &isp_v4l2_ctrl_cmd_interface_mode, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_CONTEXT_STATE_ID, &isp_v4l2_ctrl_cmd_context_state, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_MCFE_USECASE_ID, &isp_v4l2_ctrl_cmd_mcfe_usecase, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_M2M_PROCESS_REQUEST, &isp_v4l2_ctrl_cmd_system_m2m_process_request, NULL );
    ADD_CTRL_CST_VOLATILE( ISP_V4L2_CID_SYSTEM_V4L2_INTERFACE_MODE, &isp_v4l2_ctrl_cmd_system_v4l2_interface_mode, NULL );

/* Add control handler to v4l2 device */
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 20, 0 ) )
    v4l2_ctrl_add_handler( hdl_std_ctrl, hdl_cst_ctrl, NULL, false );
#else
    v4l2_ctrl_add_handler( hdl_std_ctrl, hdl_cst_ctrl, NULL );
#endif

    v4l2_ctrl_handler_setup( hdl_std_ctrl );
    v4l2_ctrl_handler_setup( hdl_cst_ctrl );

    return 0;
}

void isp_v4l2_ctrl_deinit( isp_v4l2_ctrl_t *ctrl )
{
    v4l2_ctrl_handler_free( &ctrl->ctrl_hdl_std_ctrl );
    v4l2_ctrl_handler_free( &ctrl->ctrl_hdl_cst_ctrl );
}
