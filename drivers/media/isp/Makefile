obj-$(CONFIG_ARM_MALI_ISP_C71AE) += isp.o

ccflags-y := -I$(srctree)/$(src)/isp/isp
ccflags-y += -I$(srctree)/$(src)/isp/isp/calib_mgr
ccflags-y += -I$(srctree)/$(src)/isp/isp/fsmgr
ccflags-y += -I$(srctree)/$(src)/isp/params
ccflags-y += -I$(srctree)/$(src)/isp/fsm/iridix8_manual
ccflags-y += -I$(srctree)/$(src)/isp/fsm/isp_wrapper_dummy
ccflags-y += -I$(srctree)/$(src)/isp/fsm/sharpening
ccflags-y += -I$(srctree)/$(src)/isp/fsm/demosaic
ccflags-y += -I$(srctree)/$(src)/isp/fsm/noise_profile
ccflags-y += -I$(srctree)/$(src)/isp/fsm/decompander
ccflags-y += -I$(srctree)/$(src)/isp/fsm/ae_manual
ccflags-y += -I$(srctree)/$(src)/isp/fsm/mcfe
ccflags-y += -I$(srctree)/$(src)/isp/fsm/mcfe/module_mcfe
ccflags-y += -I$(srctree)/$(src)/isp/fsm/cac
ccflags-y += -I$(srctree)/$(src)/isp/fsm/color_matrix
ccflags-y += -I$(srctree)/$(src)/isp/fsm/cmos
ccflags-y += -I$(srctree)/$(src)/isp/fsm/control
ccflags-y += -I$(srctree)/$(src)/isp/fsm/sbuf
ccflags-y += -I$(srctree)/$(src)/isp/fsm/crop
ccflags-y += -I$(srctree)/$(src)/isp/fsm/frame_stitch
ccflags-y += -I$(srctree)/$(src)/isp/fsm/noise_reduction
ccflags-y += -I$(srctree)/$(src)/isp/fsm/frame_check
ccflags-y += -I$(srctree)/$(src)/isp/fsm/output_formatter
ccflags-y += -I$(srctree)/$(src)/isp/fsm/defect_pixel
ccflags-y += -I$(srctree)/$(src)/isp/fsm/awb_manual
ccflags-y += -I$(srctree)/$(src)/isp/fsm/raw_fe
ccflags-y += -I$(srctree)/$(src)/isp/fsm/sensor
ccflags-y += -I$(srctree)/$(src)/isp/fsm/metadata
ccflags-y += -I$(srctree)/$(src)/isp/fsm/radial_shading
ccflags-y += -I$(srctree)/$(src)/isp/fsm/histogram
ccflags-y += -I$(srctree)/$(src)/isp/fsm/ml_bist
ccflags-y += -I$(srctree)/$(src)/isp/fsm/mesh_shading
ccflags-y += -I$(srctree)/$(src)/isp/reg
ccflags-y += -I$(srctree)/$(src)/src
ccflags-y += -I$(srctree)/$(src)/util/queue
ccflags-y += -I$(srctree)/$(src)/util/math
ccflags-y += -I$(srctree)/$(src)/util/pool
ccflags-y += -I$(srctree)/$(src)/util/crc
ccflags-y += -I$(srctree)/$(src)/util
ccflags-y += -I$(srctree)/$(src)/util/logger
ccflags-y += -I$(srctree)/$(src)/sbus
ccflags-y += -I$(srctree)/$(src)/v4l2_interface
ccflags-y += -I$(srctree)/$(src)/connection
ccflags-y += -I$(srctree)/$(src)/frame_streamer/sfs_streamer
ccflags-y += -I$(srctree)/$(src)/frame_streamer
ccflags-y += -I$(srctree)/$(src)/frame_streamer/frame_consumer
ccflags-y += -I$(srctree)/$(src)/frame_streamer/default_streamer
ccflags-y += -I$(srctree)/$(src)/frame_streamer/v4l2_streamer
ccflags-y += -I$(srctree)/$(src)/sensor/driver/common
ccflags-y += -I$(srctree)/$(src)/sensor/driver/imx477
ccflags-y += -I$(srctree)/$(src)/platform

isp-objs +=  sensor/driver/common/sensor_bus_config.o isp/fsm/sensor/sensor_fsm.o isp/fsm/control/control_fsm.o isp/fsm/decompander/decompander_fsm.o isp/fsm/frame_stitch/frame_stitch_fsm.o isp/fsm/output_formatter/output_formatter_fsm.o isp/fsm/crop/crop_fsm.o isp/fsm/cmos/cmos_fsm.o isp/fsm/ae_manual/ae_manual_fsm.o isp/fsm/histogram/histogram_fsm.o isp/fsm/awb_manual/awb_manual_fsm.o isp/fsm/color_matrix/color_matrix_fsm.o isp/fsm/mesh_shading/mesh_shading_fsm.o isp/fsm/iridix8_manual/iridix8_manual_fsm.o isp/fsm/mcfe/mcfe_fsm.o isp/fsm/frame_check/frame_check_fsm.o isp/fsm/metadata/metadata_fsm.o isp/fsm/sharpening/sharpening_fsm.o isp/fsm/noise_reduction/noise_reduction_fsm.o isp/fsm/defect_pixel/defect_pixel_fsm.o isp/fsm/sbuf/sbuf_fsm.o isp/fsm/isp_wrapper_dummy/isp_wrapper_dummy_fsm.o isp/fsm/cac/cac_fsm.o isp/fsm/radial_shading/radial_shading_fsm.o isp/fsm/noise_profile/noise_profile_fsm.o isp/fsm/demosaic/demosaic_fsm.o isp/fsm/ml_bist/ml_bist_fsm.o isp/fsm/raw_fe/raw_fe_fsm.o isp/isp/fsmgr/acamera_fsmgr.o isp/isp/acamera_command_api.o isp/params/isp_params.o isp/params/context_params.o isp/params/command_id_to_params.o util/crc/util_crc16.o util/logger/acamera_logger.o util/math/acamera_math.o util/pool/util_pool.o util/queue/util_queue.o isp/isp/acamera.o isp/isp/acamera_interrupts.o isp/isp/acamera_isp_ctx.o isp/isp/calib_mgr/acamera_calib_mgr.o isp/isp/fsmgr/acamera_fsmgr_event_queue.o isp/isp/fsmgr/acamera_fsmgr_general_router.o frame_streamer/acamera_frame_streamer.o frame_streamer/default_streamer/acamera_default_streamer.o frame_streamer/frame_consumer/acamera_frame_consumer.o  frame_streamer/v4l2_streamer/acamera_v4l2_streamer.o frame_streamer/sfs_streamer/acamera_sfs_streamer.o sbus/acamera_loop_buf.o sbus/acamera_sbus.o sbus/acamera_sbus_i2c.o sbus/acamera_sbus_isp.o sbus/acamera_sbus_spi.o isp/fsm/sensor/sensor_func.o isp/fsm/control/control_func.o isp/fsm/decompander/decompander_func.o isp/fsm/frame_stitch/frame_stitch_func.o isp/fsm/output_formatter/output_formatter_func.o isp/fsm/crop/crop_func.o isp/fsm/cmos/cmos_func.o isp/fsm/ae_manual/ae_manual_func.o isp/fsm/histogram/ae_update_histogram.o isp/fsm/histogram/ae_util_crc.o isp/fsm/histogram/histogram_func.o isp/fsm/awb_manual/awb_manual_func.o isp/fsm/color_matrix/color_matrix_func.o isp/fsm/mesh_shading/mesh_shading_func.o isp/fsm/iridix8_manual/iridix8_manual_func.o isp/fsm/mcfe/module_mcfe/module_mcfe.o isp/fsm/mcfe/module_mcfe/module_mcfe_hwif_common.o isp/fsm/mcfe/module_mcfe/module_mcfe_service.o isp/fsm/mcfe/module_mcfe/module_mcfe_service_buf.o isp/fsm/mcfe/module_mcfe/module_mcfe_service_bufset.o isp/fsm/mcfe/module_mcfe/module_mcfe_service_input.o isp/fsm/mcfe/module_mcfe/module_mcfe_service_slot.o isp/fsm/mcfe/module_mcfe/module_mcfe_usecase.o isp/fsm/mcfe/module_mcfe/module_mcfe_usecase_m2m.o isp/fsm/mcfe/module_mcfe/module_mcfe_usecase_tdmf.o isp/fsm/mcfe/module_mcfe/mcfe_r1/module_mcfe_hwif_impl.o isp/fsm/mcfe/mcfe_func.o isp/fsm/frame_check/frame_check_func.o isp/fsm/metadata/metadata_func.o isp/fsm/sharpening/sharpening_func.o isp/fsm/noise_reduction/noise_reduction_func.o isp/fsm/defect_pixel/defect_pixel_func.o isp/fsm/sbuf/sbuf_func.o isp/fsm/sbuf/sbuf_mgr.o isp/fsm/isp_wrapper_dummy/isp_wrapper_dummy_func.o isp/fsm/cac/cac_func.o isp/fsm/radial_shading/radial_shading_func.o isp/fsm/noise_profile/noise_profile_func.o isp/fsm/demosaic/demosaic_func.o isp/fsm/ml_bist/ml_bist_func.o isp/fsm/raw_fe/raw_fe_func.o sensor/calibrations/acamera_calibrations_dynamic_linear_dummy.o sensor/calibrations/acamera_calibrations_static_linear_dummy.o sensor/calibrations/acamera_calibrations_dynamic_native_dummy.o sensor/calibrations/acamera_calibrations_static_native_dummy.o sensor/calibrations/acamera_get_calibrations_dummy.o sensor/driver/common/sensor_init.o sensor/driver/imx477/imx477_drv.o sensor/driver/common/sensor_interface_mipi.o connection/acamera_connection.o connection/application_command_api.o connection/acamera_command_api_impl.o connection/acamera_connection_buffer_manager.o connection/acamera_ctrl_channel_k2u.o platform/system_assert.o platform/system_chardev.o platform/system_cma.o platform/system_control.o platform/system_interrupts.o platform/system_mutex.o platform/system_profiler.o platform/system_semaphore.o platform/system_spinlock.o platform/system_stdlib.o platform/system_file_io.o platform/system_timer.o src/main.o platform/system_i2c.o platform/system_isp_io.o platform/system_platform.o platform/system_spi.o v4l2_interface/isp-v4l2.o v4l2_interface/isp-vb2.o v4l2_interface/isp-v4l2-stream.o v4l2_interface/fw-interface.o v4l2_interface/isp-v4l2-ctrl.o 
