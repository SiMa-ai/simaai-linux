/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * SiMa Modalix ISP — V4L2 META_OUTPUT params buffer layout.
 *
 * Symmetric counterpart of modalix_isp_stats_buffer.h. Where the
 * stats buffer carries kernel→user 3A statistics, this buffer carries
 * user→kernel 3A decisions: AE exposure, AWB gains + CCM, the
 * gamma-contrast LUT, and iridix tone-mapping knobs.
 *
 * Producer: the libcamera IPA fills a struct modalix_isp_params_buffer
 * and queues it via VIDIOC_QBUF on the per-context META_OUTPUT video
 * device. Consumer: the kernel BH dequeues, walks the valid_mask, and
 * applies each populated section through the same code path the
 * legacy sbuf "changed_flag" channel used.
 *
 * Sparse population is the norm: the IPA only sets bits in valid_mask
 * for sections whose values actually changed this frame. Section
 * bodies are layout-stable: new sections append at the END so older
 * kernels reading a newer-IPA buffer see the sections they understand
 * at unchanged offsets.
 *
 * Version compatibility: MODALIX_ISP_PARAMS_BUFFER_VERSION_V1 is the
 * initial wire format; bump on any incompatible change.
 */

#ifndef _UAPI_MODALIX_ISP_PARAMS_BUFFER_H
#define _UAPI_MODALIX_ISP_PARAMS_BUFFER_H

#include <linux/types.h>

#define MODALIX_ISP_PARAMS_BUFFER_VERSION_V1  1u

/* valid_mask bit positions in struct modalix_isp_params_buffer. */
#define MODALIX_ISP_PARAMS_VALID_AE           (1u << 0)
#define MODALIX_ISP_PARAMS_VALID_AWB          (1u << 1)
#define MODALIX_ISP_PARAMS_VALID_GAMMA        (1u << 2)
#define MODALIX_ISP_PARAMS_VALID_IRIDIX       (1u << 3)

/* Same gamma-LUT length the ISP register block expects (257 entries
 * — endpoint-inclusive). Surfacing it in UAPI lets user-space size
 * the buffer without pulling in kernel-internal headers. */
#define MODALIX_ISP_GAMMA_LUT_LENGTH          257u

/* Same CCM-matrix dimensions the AWB block expects (3x3, row-major,
 * sign-magnitude s7.8 — bit 15 = sign, bits 14:0 = mag * 256). Same
 * encoding as the calib_mgr ILLUMINANT_CCMS slot. */
#define MODALIX_ISP_CCM_MATRIX_LEN            9u
#define MODALIX_ISP_AWB_WARMING_LEN          3u

/**
 * struct modalix_isp_ae_params - AE decisions for the upcoming frame.
 *
 * @exposure_log2:       target exposure in log2 form (signed Q-format
 *                       matching acamera_cmd_ae_info.exposure_log2).
 * @exposure_ratio:      sensor exposure ratio (multi-exposure stacks).
 * @hist_mean:           mean of the most recent AE histogram, used by
 *                       downstream stats consumers (debug / iridix).
 * @ldr_gain_log2:       LDR digital gain in log2 form.
 */
struct modalix_isp_ae_params {
	__s32  exposure_log2;
	__u32  exposure_ratio;
	__u32  hist_mean;
	__u32  ldr_gain_log2;
};

/**
 * struct modalix_isp_awb_params - AWB decisions for the upcoming
 * frame.
 *
 * @red_gain / @blue_gain:        per-channel WB gains (Q-format
 *                                matching SYSTEM_AWB_RED_GAIN_PARAM
 *                                / SYSTEM_AWB_BLUE_GAIN_PARAM).
 * @temperature_detected:         detected colour temperature in K.
 * @p_high:                       probability-of-high-CCT discriminator
 *                                from the IPA's light-source selector.
 * @light_source_candidate:       AWB candidate enum (illuminant id).
 * @ccm_matrix:                   3x3 colour-correction matrix, encoded
 *                                as MODALIX_ISP_CCM_MATRIX_LEN entries
 *                                in sign-magnitude s7.8. Only consulted
 *                                when @ccm_matrix_valid != 0.
 * @ccm_matrix_valid:             1 = use @ccm_matrix this frame, 0 =
 *                                fall back to the kernel's interpolated
 *                                CCM (legacy A/D40/D50 selector).
 * @awb_warming:                  per-channel R/G/B colour-temperature
 *                                warming OFFSET, signed, written to the
 *                                output formatter's rgb2rgb_coef_b_{1,2,3}
 *                                (signed 2's-complement 15-bit). Computed
 *                                by the IPA; the kernel only writes it.
 *                                Consulted only when @awb_warming_valid != 0.
 * @awb_warming_valid:            1 = use @awb_warming this frame.
 */
struct modalix_isp_awb_params {
	__u32  red_gain;
	__u32  blue_gain;
	__s32  temperature_detected;
	__u8   p_high;
	__u8   light_source_candidate;
	__u8   ccm_matrix_valid;
	__u8   _reserved0;
	__s16  ccm_matrix[MODALIX_ISP_CCM_MATRIX_LEN];
	__s16  awb_warming[MODALIX_ISP_AWB_WARMING_LEN];
	__u8   awb_warming_valid;
	__u8   _reserved1;
};

/**
 * struct modalix_isp_gamma_params - gamma-contrast LUT for the
 * upcoming frame.
 *
 * @lut:                          LUT entries; @lut_length is the count
 *                                of valid leading entries (the kernel
 *                                pads / truncates to the ISP register
 *                                block size).
 * @lut_length:                   number of valid entries in @lut. The
 *                                ISP block expects
 *                                MODALIX_ISP_GAMMA_LUT_LENGTH; values
 *                                shorter than that imply a shorter LUT
 *                                that the kernel may pad.
 */
struct modalix_isp_gamma_params {
	__u32  lut[MODALIX_ISP_GAMMA_LUT_LENGTH];
	__u16  lut_length;
	__u16  _reserved;
};

/**
 * struct modalix_isp_iridix_params - iridix tone-mapping knobs.
 *
 * @strength_target / @dark_enh / @digital_gain / @contrast: directly
 *                                map to the iridix HW block's
 *                                strength_target / dark_enh / digital
 *                                gain / contrast registers. Same
 *                                semantics as the legacy sbuf_iridix_t
 *                                payload.
 */
struct modalix_isp_iridix_params {
	__u16  strength_target;
	__u16  dark_enh;
	__u16  digital_gain;
	__u16  _reserved;
	__u32  contrast;
};

/**
 * struct modalix_isp_params_buffer - V4L2 META_OUTPUT buffer payload.
 *
 * @version:       MODALIX_ISP_PARAMS_BUFFER_VERSION_V1 for the layout
 *                 described here. Future bumps coordinate with new
 *                 ABI.
 * @frame_id:      target frame id the IPA wants these parameters
 *                 applied to. The kernel uses this for logging /
 *                 debug correlation; it does not gate application.
 * @valid_mask:    bitmask of MODALIX_ISP_PARAMS_VALID_* flags
 *                 indicating which sub-sections were filled this
 *                 frame. Sections without their bit set hold
 *                 undefined data.
 * @ae / @awb / @gamma / @iridix: per-block decisions; see each
 *                                struct's docs for field semantics.
 */
struct modalix_isp_params_buffer {
	__u32                            version;
	__u32                            frame_id;
	__u32                            valid_mask;
	__u32                            _reserved;

	struct modalix_isp_ae_params     ae;
	struct modalix_isp_awb_params    awb;
	struct modalix_isp_gamma_params  gamma;
	struct modalix_isp_iridix_params iridix;
};

#endif /* _UAPI_MODALIX_ISP_PARAMS_BUFFER_H */
