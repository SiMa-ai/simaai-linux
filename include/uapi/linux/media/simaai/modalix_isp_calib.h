/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * SiMa Modalix ISP — calibration data format UAPI.
 *
 * Defines the on-the-wire encoding of the calibration blob carried
 * through the MODALIX_ISP_IOC_SET_CALIBRATION ioctl
 * (<linux/media/simaai/modalix_isp_ctrl.h>). Layout is:
 *
 *   struct modalix_isp_calibrations {
 *       __aligned_u64 calibrations[MODALIX_ISP_CALIB_TOTAL_SIZE];
 *   };
 *   ... followed by a tightly-packed pool of
 *   struct modalix_isp_lookup_table entries and their LUT data.
 *
 * In the wire form, every "pointer-shaped" field is encoded as an
 * offset (in bytes) from the start of the blob:
 *   - calibrations[i] is the offset to a modalix_isp_lookup_table, or 0
 *     when the slot is unused.
 *   - lookup_table.ptr   is the offset to the LUT's data bytes,
 *     or 0 when the slot has no data array.
 *
 * The kernel patches these offsets to real pointers in place after
 * copy_from_user — see ctrl_channel_fops_ioctl() in the driver. The
 * patched-up form keeps the same byte layout (offsets and pointers are
 * both 8-byte values on the supported architectures), so the same
 * struct definition serves both wire and in-memory roles.
 *
 * Skipping empty slots in user-space is supported: the kernel
 * deserializer guards every offset with an "if (offset)" check, so
 * setting calibrations[i] = 0 simply means "this LUT is absent". The
 * dense 124-entry layout is preserved for ABI stability — slot IDs are
 * indices into this array and must never be renumbered. New LUTs add
 * to the end; existing slot IDs are versioned by convention.
 *
 * The slot ID #defines below are stable identifiers for each LUT;
 * they're consumed by both the IPA serializer (libcamera) and every
 * FSM in the kernel that calls calib_mgr_lut_get(entry, slot_id).
 */

#ifndef _UAPI_MODALIX_ISP_CALIB_H
#define _UAPI_MODALIX_ISP_CALIB_H

#include <linux/types.h>

/**
 * struct modalix_isp_lookup_table - per-LUT descriptor in the
 * calibration blob.
 *
 * @ptr:   on the wire, the offset (in bytes) from the blob base to
 *         the LUT's data array, or 0 if the LUT carries no payload.
 *         After kernel-side patching, holds the real kernel pointer.
 *         Always 8 bytes; cast to/from (const void *) via uintptr_t.
 * @rows:  number of rows for 2-D LUTs, 1 for 1-D.
 * @cols:  number of columns, in element units.
 * @width: element width in bytes (1, 2, or 4 for u8/u16/u32 LUTs).
 * @_reserved: pads the struct to 16 bytes for stable alignment.
 */
struct modalix_isp_lookup_table {
	__aligned_u64 ptr;
	__u16         rows;
	__u16         cols;
	__u16         width;
	__u16         _reserved;
};

/**
 * MODALIX_ISP_CALIB_TOTAL_SIZE - number of LUT slots in the dense
 * calibration array.
 *
 * Treat this as a versioned constant: never renumber existing slot
 * IDs, never shrink this. New LUTs append to the end (current next
 * free slot id is 0x87).
 */
#define MODALIX_ISP_CALIB_TOTAL_SIZE 136

/**
 * struct modalix_isp_calibrations - the blob's pointer-table root.
 *
 * @calibrations: dense array of offsets (wire form) / pointers
 *                (in-memory form) to per-LUT descriptors. Index by
 *                MODALIX_ISP_CALIB_* slot IDs. NULL/zero means "this
 *                slot is not provided by this YAML"; readers must
 *                tolerate sparse population.
 */
struct modalix_isp_calibrations {
	__aligned_u64 calibrations[MODALIX_ISP_CALIB_TOTAL_SIZE];
};

/* ============================================================================
 * Static calibration values.
 * ============================================================================*/
#define MODALIX_ISP_CALIB_LIGHT_SRC                           0x00000000
#define MODALIX_ISP_CALIB_RG_POS                              0x00000001
#define MODALIX_ISP_CALIB_BG_POS                              0x00000002
#define MODALIX_ISP_CALIB_MESH_RGBG_WEIGHT                    0x00000003
#define MODALIX_ISP_CALIB_MESH_LS_WEIGHT                      0x00000004
#define MODALIX_ISP_CALIB_MESH_COLOR_TEMPERATURE              0x00000005
#define MODALIX_ISP_CALIB_WB_STRENGTH                         0x00000006
#define MODALIX_ISP_CALIB_SKY_LUX_TH                          0x00000007
#define MODALIX_ISP_CALIB_CT_RG_POS_CALC                      0x00000008
#define MODALIX_ISP_CALIB_CT_BG_POS_CALC                      0x00000009
#define MODALIX_ISP_CALIB_COLOR_TEMP                          0x0000000A
#define MODALIX_ISP_CALIB_CT65POS                             0x0000000B
#define MODALIX_ISP_CALIB_CT40POS                             0x0000000C
#define MODALIX_ISP_CALIB_CT30POS                             0x0000000D
#define MODALIX_ISP_CALIB_EVTOLUX_EV_LUT                      0x0000000E
#define MODALIX_ISP_CALIB_EVTOLUX_LUX_LUT                     0x0000000F
#define MODALIX_ISP_CALIB_BLACK_LEVEL_R                       0x00000010
#define MODALIX_ISP_CALIB_BLACK_LEVEL_GR                      0x00000011
#define MODALIX_ISP_CALIB_BLACK_LEVEL_GB                      0x00000012
#define MODALIX_ISP_CALIB_BLACK_LEVEL_B                       0x00000013
#define MODALIX_ISP_CALIB_STATIC_WB                           0x00000014
#define MODALIX_ISP_CALIB_MT_ABSOLUTE_LS_A_CCM                0x00000015
#define MODALIX_ISP_CALIB_MT_ABSOLUTE_LS_D40_CCM              0x00000016
#define MODALIX_ISP_CALIB_MT_ABSOLUTE_LS_D50_CCM              0x00000017
#define MODALIX_ISP_CALIB_SHADING_LS_A_R                      0x00000018
#define MODALIX_ISP_CALIB_SHADING_LS_A_G                      0x00000019
#define MODALIX_ISP_CALIB_SHADING_LS_A_B                      0x0000001A
#define MODALIX_ISP_CALIB_SHADING_LS_TL84_R                   0x0000001B
#define MODALIX_ISP_CALIB_SHADING_LS_TL84_G                   0x0000001C
#define MODALIX_ISP_CALIB_SHADING_LS_TL84_B                   0x0000001D
#define MODALIX_ISP_CALIB_SHADING_LS_D65_R                    0x0000001E
#define MODALIX_ISP_CALIB_SHADING_LS_D65_G                    0x0000001F
#define MODALIX_ISP_CALIB_SHADING_LS_D65_B                    0x00000020
#define MODALIX_ISP_CALIB_AWB_WARMING_LS_A                    0x00000021
#define MODALIX_ISP_CALIB_AWB_WARMING_LS_D50                  0x00000022
#define MODALIX_ISP_CALIB_AWB_WARMING_LS_D75                  0x00000023
#define MODALIX_ISP_CALIB_NOISE_PROFILE                       0x00000024
#define MODALIX_ISP_CALIB_DEMOSAIC                            0x00000025
#define MODALIX_ISP_CALIB_GAMMA                               0x00000026
#define MODALIX_ISP_CALIB_IRIDIX_ASYMMETRY                    0x00000027
#define MODALIX_ISP_CALIB_AWB_SCENE_PRESETS                   0x00000028
#define MODALIX_ISP_CALIB_WDR_NP_LUT                          0x00000029
#define MODALIX_ISP_CALIB_CA_FILTER_MEM                       0x0000002A
#define MODALIX_ISP_CALIB_CA_CORRECTION                       0x0000002B
#define MODALIX_ISP_CALIB_CA_CORRECTION_MEM                   0x0000002C
#define MODALIX_ISP_CALIB_SHADING_RADIAL_R                    0x0000002D
#define MODALIX_ISP_CALIB_SHADING_RADIAL_G                    0x0000002E
#define MODALIX_ISP_CALIB_SHADING_RADIAL_B                    0x0000002F
#define MODALIX_ISP_CALIB_SHADING_RADIAL_IR                   0x00000030
#define MODALIX_ISP_CALIB_SHADING_RADIAL_CENTRE_AND_MULT      0x00000031
#define MODALIX_ISP_CALIB_GAMMA_FE                            0x00000032
#define MODALIX_ISP_CALIB_AWB_WARMING_CCT                     0x00000033
#define MODALIX_ISP_CALIB_AWB_MIXED_LIGHT_PARAMETERS          0x00000034

/* ============================================================================
 * Dynamic calibration values
 * ============================================================================*/
#define MODALIX_ISP_CALIB_STITCHING_LM_MED_NOISE_INTENSITY    0x00000035
#define MODALIX_ISP_CALIB_EXPOSURE_RATIO_ADJUSTMENT           0x00000036
#define MODALIX_ISP_CALIB_SINTER_STRENGTH_MC_CONTRAST         0x00000037
#define MODALIX_ISP_CALIB_SINTER_PARAMS                       0x00000038
#define MODALIX_ISP_CALIB_SINTER_RADIAL_LUT                   0x00000039
#define MODALIX_ISP_CALIB_SINTER_RADIAL_PARAMS                0x0000003A
#define MODALIX_ISP_CALIB_AWB_BG_MAX_GAIN                     0x0000003B
#define MODALIX_ISP_CALIB_IRIDIX8_STRENGTH_DK_ENH_CONTROL     0x0000003C
#define MODALIX_ISP_CALIB_IRIDIX8_EXTENDED_CONTROL            0x0000003D
#define MODALIX_ISP_CALIB_CMOS_CONTROL                        0x0000003E
#define MODALIX_ISP_CALIB_DP_SLOPE                            0x0000003F
#define MODALIX_ISP_CALIB_DP_THRESHOLD                        0x00000040
#define MODALIX_ISP_CALIB_STITCHING_LM_MOV_MULT               0x00000041
#define MODALIX_ISP_CALIB_STITCHING_LM_NP                     0x00000042
#define MODALIX_ISP_CALIB_STITCHING_MS_MOV_MULT               0x00000043
#define MODALIX_ISP_CALIB_STITCHING_MS_NP                     0x00000044
#define MODALIX_ISP_CALIB_STITCHING_SVS_MOV_MULT              0x00000045
#define MODALIX_ISP_CALIB_STITCHING_SVS_NP                    0x00000046
#define MODALIX_ISP_CALIB_EVTOLUX_PROBABILITY_ENABLE          0x00000047
#define MODALIX_ISP_CALIB_AWB_AVG_COEF                        0x00000048
#define MODALIX_ISP_CALIB_IRIDIX_AVG_COEF                     0x00000049
#define MODALIX_ISP_CALIB_IRIDIX_STRENGTH_MAXIMUM             0x0000004A
#define MODALIX_ISP_CALIB_IRIDIX_MIN_MAX_STR                  0x0000004B
#define MODALIX_ISP_CALIB_IRIDIX_EV_LIM_FULL_STR              0x0000004C
#define MODALIX_ISP_CALIB_IRIDIX_EV_LIM_NO_STR                0x0000004D
#define MODALIX_ISP_CALIB_AE_CORRECTION                       0x0000004E
#define MODALIX_ISP_CALIB_AE_EXPOSURE_CORRECTION              0x0000004F
#define MODALIX_ISP_CALIB_SINTER_STRENGTH                     0x00000050
#define MODALIX_ISP_CALIB_SINTER_STRENGTH1                    0x00000051
#define MODALIX_ISP_CALIB_SINTER_STRENGTH4                    0x00000052
#define MODALIX_ISP_CALIB_SINTER_THRESH1                      0x00000053
#define MODALIX_ISP_CALIB_SINTER_THRESH4                      0x00000054
#define MODALIX_ISP_CALIB_SHARP_ALT_D                         0x00000055
#define MODALIX_ISP_CALIB_SHARP_ALT_UD                        0x00000056
#define MODALIX_ISP_CALIB_SHARP_ALT_DU                        0x00000057
#define MODALIX_ISP_CALIB_DEMOSAIC_UU_SLOPE                   0x00000058
#define MODALIX_ISP_CALIB_MESH_SHADING_STRENGTH               0x00000059
#define MODALIX_ISP_CALIB_SATURATION_STRENGTH                 0x0000005A
#define MODALIX_ISP_CALIB_CCM_ONE_GAIN_THRESHOLD              0x0000005B
#define MODALIX_ISP_CALIB_AE_ZONE_WGHT_HOR                    0x0000005C
#define MODALIX_ISP_CALIB_AE_ZONE_WGHT_VER                    0x0000005D
#define MODALIX_ISP_CALIB_AWB_ZONE_WGHT_HOR                   0x0000005E
#define MODALIX_ISP_CALIB_AWB_ZONE_WGHT_VER                   0x0000005F
#define MODALIX_ISP_CALIB_FS_MC_OFF                           0x00000060
#define MODALIX_ISP_CALIB_CMOS_EXP_PARTITION                  0x00000061
#define MODALIX_ISP_CALIB_DEFECT_PIXEL                        0x00000062
#define MODALIX_ISP_CALIB_GAMMA_BE0                           0x00000063
#define MODALIX_ISP_CALIB_GAMMA_BE1                           0x00000064
#define MODALIX_ISP_CALIB_DECOMPANDER_CONTROL                 0x00000065
#define MODALIX_ISP_CALIB_INPUT_FORMATTER                     0x00000066
#define MODALIX_ISP_CALIB_NEQ_LUT                             0x00000067
#define MODALIX_ISP_CALIB_SINTER_INTCONFIG                    0x00000068
#define MODALIX_ISP_CALIB_RGB2RGB_HS_CONVERSION               0x00000069
#define MODALIX_ISP_CALIB_RGB2RGB_HS_CONVERSION_B             0x0000006A
#define MODALIX_ISP_CALIB_RGB2RGB_S2_CONVERSION               0x0000006B
#define MODALIX_ISP_CALIB_RGB2RGB_S2_CONVERSION_B             0x0000006C
#define MODALIX_ISP_CALIB_AE_CONTROL_HDR_TARGET               0x0000006D
#define MODALIX_ISP_CALIB_AE_CONTROL                          0x0000006E
#define MODALIX_ISP_CALIB_COLOR_MATRIX_YUV_PRESETS            0x0000006F
#define MODALIX_ISP_CALIB_COLOR_MATRIX_B_YUV_PRESETS          0x00000070
#define MODALIX_ISP_CALIB_COLOR_MATRIX_LUV_PRESETS            0x00000071
#define MODALIX_ISP_CALIB_COLOR_MATRIX_B_LUV_PRESETS          0x00000072
#define MODALIX_ISP_CALIB_IRIDIX_GTM_LUT_X                    0x00000073
#define MODALIX_ISP_CALIB_IRIDIX_GTM_LUT_Y                    0x00000074
#define MODALIX_ISP_CALIB_DEMOSAIC_CONFIG                     0x00000075
#define MODALIX_ISP_CALIB_GAMMA_BLACK_LEVELS                  0x00000076
#define MODALIX_ISP_CALIB_RAW_FRONTEND_CONFIG                 0x00000077
#define MODALIX_ISP_CALIB_NOISE_PROFILE_CONFIG                0x00000078
#define MODALIX_ISP_CALIB_WDR_STITCH_CONFIG                   0x00000079
#define MODALIX_ISP_CALIB_STATISTICS_CONFIG                   0x0000007A
#define MODALIX_ISP_CALIB_CUSTOM_SETTINGS                     0x0000007B

/* Role-tagged illuminant table — chromaticity, temperature, and CCM
 * for each measured Planckian-locus illuminant. Three parallel arrays
 * of length N (≤ AWB_MAX_PLANCKIAN):
 *   ILLUMINANT_TEMPS_K  — N × u16              — CT in Kelvin
 *   ILLUMINANT_RG_BG    — N × 2 × u16          — [rg, bg] × 256
 *   ILLUMINANT_CCMS     — N × ISP_CCM_SIZE × u16 — per-illuminant CCM
 */
#define MODALIX_ISP_CALIB_ILLUMINANT_TEMPS_K                  0x0000007C
#define MODALIX_ISP_CALIB_ILLUMINANT_RG_BG                    0x0000007D
#define MODALIX_ISP_CALIB_ILLUMINANT_CCMS                     0x0000007E

/* Off-locus table — chromaticity, role, and optional per-role lux
 * gates. Four parallel arrays of length N (≤ AWB_MAX_OFF_LOCUS):
 *   OFF_LOCUS_RG_BG    — N × 2 × u16 — [rg, bg] chromaticity × 256
 *   OFF_LOCUS_ROLES    — N × u8      — AWB_ROLE_{CWF,SODIUM,MERCURY}
 *   OFF_LOCUS_LUX_LOW  — N × u16     — lux floor for LOTS-of-role
 *   OFF_LOCUS_LUX_HIGH — N × u16     — lux ceiling for LOTS-of-role
 * LUX_LOW/HIGH absent = use per-role hardcoded defaults.
 */
#define MODALIX_ISP_CALIB_OFF_LOCUS_RG_BG                     0x0000007F
#define MODALIX_ISP_CALIB_OFF_LOCUS_ROLES                     0x00000080
#define MODALIX_ISP_CALIB_OFF_LOCUS_LUX_LOW                   0x00000081
#define MODALIX_ISP_CALIB_OFF_LOCUS_LUX_HIGH                  0x00000082

/* Temporal biquad filters for the 3A loops. Each slot is one filter:
 * 1 row × BIQUAD_COEFFS_COLS (14) × u32 (Q16.16), laid out as
 *   [n_sections, seed_mode, {b0,b1,b2,a0,a1,a2} ×2 sections]
 * (see 3a/biquad_filter.h in the IPA). These are consumed ONLY by the
 * userspace IPA's 3A; the kernel carries the slots but never reads them.
 */
#define MODALIX_ISP_CALIB_AE_FILTER                           0x00000083
#define MODALIX_ISP_CALIB_WDR_EXPOSURE_RATIO_FILTER           0x00000084
#define MODALIX_ISP_CALIB_WHITE_BALANCE_GAIN_FILTER           0x00000085
#define MODALIX_ISP_CALIB_CCM_FILTER                          0x00000086

/* AWB tuning scalars (u16[14], IPA-consumed only). Optional: absent -> the IPA
 * uses its compiled defaults. Opaque ordered blob; the field meaning lives in
 * the IPA AWB source, not here. */
#define MODALIX_ISP_CALIB_AWB_TUNING                          0x00000087

#endif /* _UAPI_MODALIX_ISP_CALIB_H */
