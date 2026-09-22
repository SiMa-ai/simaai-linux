/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * SiMa Modalix ISP — V4L2 custom controls, events, and pixel formats.
 *
 * Defines the vendor-specific extensions user-space exercises through
 * the standard V4L2 interfaces:
 *
 *   - struct modalix_isp_v4l2_cid             — vendor control IDs
 *     (VIDIOC_S_CTRL / VIDIOC_G_CTRL targets), starting at
 *     MODALIX_ISP_V4L2_CID_BASE in the V4L2_CTRL_CLASS_USER vendor
 *     range.
 *
 *   - V4L2_EVENT_MODALIX_ISP_*                — vendor event types
 *     (VIDIOC_SUBSCRIBE_EVENT targets), starting at
 *     V4L2_EVENT_MODALIX_ISP_CLASS in the V4L2_EVENT_PRIVATE_START
 *     range.
 *
 *   - V4L2_PIX_FMT_MODALIX_*                  — vendor 4CC pixel
 *     formats not in upstream <linux/videodev2.h>.
 *
 * All identifiers exposed here are stable: never renumber existing
 * CIDs (they're indices userspace persists in test scripts and config
 * files); new CIDs append to the end. Same convention as
 * modalix_isp_calib.h.
 */

#ifndef _UAPI_MODALIX_ISP_V4L2_H
#define _UAPI_MODALIX_ISP_V4L2_H

#include <linux/types.h>
#include <linux/videodev2.h>

/* ============================================================================
 * Vendor pixel formats
 * ============================================================================*/

/* Opaque metadata payload — passes through the metadata video device
 * without per-byte interpretation by the V4L2 framework. */
#define V4L2_PIX_FMT_MODALIX_META  v4l2_fourcc('M', 'E', 'T', 'A')

/* Sentinel for "no format" — used to disable a stream's pixfmt slot. */
#define V4L2_PIX_FMT_MODALIX_NULL  v4l2_fourcc('N', 'U', 'L', 'L')

/* 14-bit Bayer BG/GR — defined here only if upstream videodev2.h
 * doesn't carry it (older kernels). */
#ifndef V4L2_PIX_FMT_SBGGR14
#define V4L2_PIX_FMT_SBGGR14       v4l2_fourcc('B', 'G', '1', '4')
#endif

/* ============================================================================
 * Vendor V4L2 events
 *
 * The driver fires frame-ready and stream-off notifications via
 * VIDIOC_DQEVENT. User-space subscribes with VIDIOC_SUBSCRIBE_EVENT.
 * ============================================================================*/

#define V4L2_EVENT_MODALIX_ISP_CLASS         (V4L2_EVENT_PRIVATE_START + 0xA * 1000)
#define V4L2_EVENT_MODALIX_ISP_FRAME_READY   (V4L2_EVENT_MODALIX_ISP_CLASS + 0x1)
#define V4L2_EVENT_MODALIX_ISP_STREAM_OFF    (V4L2_EVENT_MODALIX_ISP_CLASS + 0x2)

/* ============================================================================
 * Vendor V4L2 control IDs
 *
 * Class anchor + base address; all per-control IDs are dense values
 * starting from MODALIX_ISP_V4L2_CID_BASE.
 *
 * NOTE on conditionally-compiled entries: the kernel build may guard
 * a few RTL-R2-only CIDs (CNR, RAW/RGB scaler). Those values are
 * still allocated unconditionally here so user-space callers don't
 * have to know about the kernel build flag — the kernel returns
 * -EINVAL via the control framework when an unsupported CID is set
 * on the running hardware.
 * ============================================================================*/

/* Anchor vendor CIDs under V4L2_CTRL_CLASS_USER (0x00980000). The
 * V4L2 control framework auto-adds the class control (the per-handler
 * "Class" entry) only for registered classes; the original 0x00f00000
 * anchor was rejected with -ERANGE the moment a non-COMPOUND ctrl
 * (e.g. an INTEGER CMOS ctrl) was added to the handler. V4L2_CTRL_ID2CLASS
 * derives the class from the high 16 bits of the CID, so placing
 * MODALIX_ISP_V4L2_CID_BASE inside the USER class is enough to make
 * v4l2_ctrl_handler_init's auto-class-add succeed. */
#define MODALIX_ISP_V4L2_CID_CLASS  V4L2_CTRL_CLASS_USER
#define MODALIX_ISP_V4L2_CID_BASE   (V4L2_CTRL_CLASS_USER | 0xf000)

enum modalix_isp_v4l2_cid {
	MODALIX_ISP_V4L2_CID_TEST_PATTERN = MODALIX_ISP_V4L2_CID_BASE,
	MODALIX_ISP_V4L2_CID_TEST_PATTERN_TYPE,
	MODALIX_ISP_V4L2_CID_SENSOR_SUPPORTED_PRESETS,
	MODALIX_ISP_V4L2_CID_SENSOR_PRESET,
	MODALIX_ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_MIN,
	MODALIX_ISP_V4L2_CID_SENSOR_INTEGRATION_TIME_LIMIT,
	MODALIX_ISP_V4L2_CID_SENSOR_WDR_MODE,
	MODALIX_ISP_V4L2_CID_SENSOR_STREAMING,
	MODALIX_ISP_V4L2_CID_SENSOR_EXPOSURES,
	MODALIX_ISP_V4L2_CID_SENSOR_FPS,
	MODALIX_ISP_V4L2_CID_SENSOR_WIDTH,
	MODALIX_ISP_V4L2_CID_SENSOR_HEIGHT,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_PRESET,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_WDR_MODE,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_FPS,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_WIDTH,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_HEIGHT,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_EXPOSURES,
	MODALIX_ISP_V4L2_CID_SYSTEM_FREEZE_FIRMWARE,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_CHANNELS,
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_DATA_WIDTH,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_MAX_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_ANALOG_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_SENSOR_DIGITAL_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_ISP_DIGITAL_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_DIRECTIONAL_SHARPENING,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_UN_DIRECTIONAL_SHARPENING,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_EXPOSURE_RATIO,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_AWB,
	MODALIX_ISP_V4L2_CID_SYSTEM_ANTIFLICKER_ENABLE,
	MODALIX_ISP_V4L2_CID_SYSTEM_MANUAL_SATURATION,
	MODALIX_ISP_V4L2_CID_SYSTEM_MAX_EXPOSURE_RATIO,
	MODALIX_ISP_V4L2_CID_SYSTEM_EXPOSURE,
	MODALIX_ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_SHORT_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_MIDDLE_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_MIDDLE2_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_LONG_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_INTEGRATION_TIME_PRECISION,
	MODALIX_ISP_V4L2_CID_SYSTEM_MAX_INTEGRATION_TIME,
	MODALIX_ISP_V4L2_CID_SYSTEM_EXPOSURE_RATIO,
	MODALIX_ISP_V4L2_CID_SYSTEM_SENSOR_ANALOG_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_MAX_SENSOR_ANALOG_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_SENSOR_DIGITAL_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_MAX_SENSOR_DIGITAL_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_ISP_DIGITAL_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_MAX_ISP_DIGITAL_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_DIRECTIONAL_SHARPENING_TARGET,
	MODALIX_ISP_V4L2_CID_SYSTEM_UN_DIRECTIONAL_SHARPENING_TARGET,
	MODALIX_ISP_V4L2_CID_SYSTEM_AWB_RED_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_AWB_BLUE_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_AWB_CCT,
	MODALIX_ISP_V4L2_CID_SYSTEM_SATURATION_TARGET,
	MODALIX_ISP_V4L2_CID_SYSTEM_ANTI_FLICKER_FREQUENCY,
	MODALIX_ISP_V4L2_CID_SYSTEM_IRIDIX_DIGITAL_GAIN,
	MODALIX_ISP_V4L2_CID_SYSTEM_SINTER_THRESHOLD_TARGET,
	MODALIX_ISP_V4L2_CID_SYSTEM_MINIMUM_IRIDIX_STRENGTH,
	MODALIX_ISP_V4L2_CID_SYSTEM_MAXIMUM_IRIDIX_STRENGTH,
	MODALIX_ISP_V4L2_CID_SYSTEM_IRIDIX_STRENGTH_TARGET,
	MODALIX_ISP_V4L2_CID_SYSTEM_BUFFER_DATA_TYPE_ID,
	MODALIX_ISP_V4L2_CID_SYSTEM_LOGGER_LEVEL_ID,
	MODALIX_ISP_V4L2_CID_SYSTEM_LOGGER_MASK_ID,
	MODALIX_ISP_V4L2_CID_SYSTEM_CMD_INTERFACE_MODE_ID,
	MODALIX_ISP_V4L2_CID_SYSTEM_CONTEXT_STATE_ID,
	MODALIX_ISP_V4L2_CID_SYSTEM_MCFE_USECASE_ID,
	MODALIX_ISP_V4L2_CID_SYSTEM_M2M_PROCESS_REQUEST,
	MODALIX_ISP_V4L2_CID_SYSTEM_V4L2_INTERFACE_MODE,

	/* ISP_MODULES */
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_IRIDIX,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_SINTER,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_FRAME_STITCH,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_RAW_FRONTEND,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_BLACK_LEVEL,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_SHADING,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_DEMOSAIC,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_FORCE_BIST_MISMATCH,
	MODALIX_ISP_V4L2_CID_ISP_MODULES_MANUAL_CNR, /* RTL_VERSION_R == 2 only */

	/* TIMAGE */
	MODALIX_ISP_V4L2_CID_IMAGE_CROP_XOFFSET_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_CROP_YOFFSET_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_OUTPUT_FORMAT_MANUAL_CFG_APPLY_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_OUTPUT_AXI1_FORMAT_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_OUTPUT_AXI2_FORMAT_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_OUTPUT_AXI3_FORMAT_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_CROP_HEIGHT_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_CROP_WIDTH_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_CROP_ENABLE_ID,
	MODALIX_ISP_V4L2_CID_IMAGE_RAW_SCALER_ENABLE_ID,    /* RTL_VERSION_R == 2 only */
	MODALIX_ISP_V4L2_CID_IMAGE_RAW_SCALER_WIDTH_ID,     /* RTL_VERSION_R == 2 only */
	MODALIX_ISP_V4L2_CID_IMAGE_RAW_SCALER_HEIGHT_ID,    /* RTL_VERSION_R == 2 only */
	MODALIX_ISP_V4L2_CID_IMAGE_RGB_SCALER_ENABLE_ID,    /* RTL_VERSION_R == 2 only */
	MODALIX_ISP_V4L2_CID_IMAGE_RGB_SCALER_WIDTH_ID,     /* RTL_VERSION_R == 2 only */
	MODALIX_ISP_V4L2_CID_IMAGE_RGB_SCALER_HEIGHT_ID,    /* RTL_VERSION_R == 2 only */

	/* TSTATUS_INFO */
	MODALIX_ISP_V4L2_CID_STATUS_INFO_EXPOSURE_LOG2,
	MODALIX_ISP_V4L2_CID_STATUS_INFO_GAIN_LOG2,
	MODALIX_ISP_V4L2_CID_STATUS_INFO_GAIN_ONES,
	MODALIX_ISP_V4L2_CID_STATUS_INFO_EXPOSURE_RESIDUAL_LOG2_ID,
	MODALIX_ISP_V4L2_CID_STATUS_INFO_IRIDIX_CONTRAST,
	MODALIX_ISP_V4L2_CID_STATUS_INFO_AE_HIST_MEAN,
	MODALIX_ISP_V4L2_CID_STATUS_INFO_AWB_MIX_LIGHT_CONTRAST,
	MODALIX_ISP_V4L2_CID_INFO_FW_REVISION,

	/* TGENERAL */
	MODALIX_ISP_V4L2_CID_CONTEXT_NUMBER,
	MODALIX_ISP_V4L2_CID_ACTIVE_CONTEXT,

	/* TREGISTERS */
	MODALIX_ISP_V4L2_CID_REGISTERS_VALUE_ID,
	MODALIX_ISP_V4L2_CID_REGISTERS_SOURCE_ID,
	MODALIX_ISP_V4L2_CID_REGISTERS_SIZE_ID,
	MODALIX_ISP_V4L2_CID_REGISTERS_ADDRESS_ID,

	/* kf_info readers — replace the legacy fw_sbuf->kf_info mmap
	 * the IPA used at configure() time. Exposed on the per-context
	 * V4L2 META_CAPTURE node so the IPA reads via VIDIOC_G_EXT_CTRLS
	 * on the same fd it already opens for stats. */
	MODALIX_ISP_V4L2_CID_SENSOR_INFO_BLOB,
	MODALIX_ISP_V4L2_CID_CMOS_MAX_EXPOSURE_LOG2,
	MODALIX_ISP_V4L2_CID_CMOS_AGAIN_LOG2,
	MODALIX_ISP_V4L2_CID_CMOS_DGAIN_LOG2,

	/* IPA → kernel calibration blob push. Replaces the legacy
	 * MODALIX_ISP_IOC_SET_CALIBRATION ioctl that ran on
	 * /dev/isp_control_<N>. The IPA performs one VIDIOC_S_EXT_CTRLS
	 * on the per-context meta-stats node with this CID, carrying the
	 * full offset-form modalix_isp_calibration_data byte layout as
	 * the U8-array payload. The kernel s_ctrl handler runs
	 * synchronously inside the ioctl — by the time the ioctl
	 * returns, the calibration is live in the per-ctx calib bridge
	 * slot the FSM read path consumes. */
	MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB,

	/* IPA -> kernel ISP pipeline bypass word (Mali reg 0xE040). The IPA
	 * derives the 32-bit value from the tuning file's isp_config: block
	 * and pushes it with one VIDIOC_S_EXT_CTRLS on the per-context
	 * meta-stats node; the kernel applies it to that context's config
	 * space, preserving reserved bits. G_EXT_CTRLS reads back the live
	 * register. Replaces the per-sensor hardcoded bypass pokes the ISP
	 * sensor drivers did at init. */
	MODALIX_ISP_V4L2_CID_ISP_BYPASS_CONFIG,

	/* i2c bus of the sensor feeding this m2m context; -1 = unset. */
	MODALIX_ISP_V4L2_CID_SENSOR_SOURCE_BUS,
};

/* Hard upper bound on a single calibration blob the kernel will
 * accept on MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB. Real blobs are
 * ~30–60 KiB depending on sensor; 256 KiB leaves generous headroom
 * and matches what the V4L2 ctrl framework will let us declare as a
 * DYNAMIC_ARRAY upper bound. */
#define MODALIX_ISP_V4L2_CALIBRATION_BLOB_MAX  (256u * 1024u)

#endif /* _UAPI_MODALIX_ISP_V4L2_H */
