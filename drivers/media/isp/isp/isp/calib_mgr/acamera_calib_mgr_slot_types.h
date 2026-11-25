/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2026 SiMa.ai
 */

#ifndef __ACAMERA_CALIB_MGR_SLOT_TYPES_H__
#define __ACAMERA_CALIB_MGR_SLOT_TYPES_H__

#include <linux/media/simaai/modalix_isp_calib.h>

/*
 * What the kernel expects each calibration slot to contain.
 *
 * The calibration blob arrives from user space, and bounds alone do not make
 * it safe to read: consumers size their reads in elements, so a descriptor
 * that declares the wrong element width sends an FSM past the end of its LUT
 * even though every offset is inside the blob. Nor is the width enough on its
 * own - a consumer that reads a fixed position walks off a LUT that is merely
 * short. Neither fact is a property of the blob; both are properties of the
 * code that reads the slot, so the boundary needs them written down.
 *
 * This table is that record, and it is mechanically derived from the driver:
 *
 *   - the widths come from every calib_mgr_u8/u16/u32/mod16/mod32_lut_get()
 *     call site, in .c and .h alike, with the index-holding variables
 *     resolved to the slots they can name;
 *   - the minimum element counts come from the constant subscripts those
 *     call sites apply, directly or through a local.
 *
 * The widths were cross-checked entry for entry against the shapes Arm
 * publishes for the Mali-C71AE calibration set, with no disagreements.
 *
 * The minimums deliberately are NOT Arm's declared column counts. Shipped
 * tunings legitimately carry shorter LUTs than Arm declares - 33-entry radial
 * shading against a declared 129, for one - and the consuming FSMs size-check
 * those themselves. Only what the kernel actually dereferences can be
 * required here; anything more would reject working tuning files.
 *
 * A slot with more than one width listed is read at either, and the consumer
 * picks by calib_mgr_lut_width() - iridix asymmetry is the only one today. A
 * slot absent from the table has no kernel consumer, so any valid width and
 * extent are accepted for it. A minimum of zero means every read of that slot
 * is bounded by calib_mgr_lut_len() or calib_mgr_lut_rows(), so the
 * descriptor's own extent is the bound.
 *
 * REQ marks a slot the kernel reads without first asking whether it is
 * there. Every tuning in tree provides all of them - the set was derived by
 * taking the slots no consumer guards with calib_mgr_lut_exists(), and
 * intersecting with what the IPA actually serialises for each shipped tuning.
 * A blob that omits one is refused, because the alternative is an FSM reading
 * a slot that is not there.
 *
 * MOD marks the modulation tables, whose entries are (x, y) pairs: the FSM
 * walks calib_mgr_lut_rows() entries of two columns, so the column count is
 * part of what has to hold. QUAD marks a LUT read in groups of four words.
 */

#define MODALIX_CALIB_SLOT_W1	BIT(0)
#define MODALIX_CALIB_SLOT_W2	BIT(1)
#define MODALIX_CALIB_SLOT_W4	BIT(2)
#define MODALIX_CALIB_SLOT_MOD	BIT(3)
#define MODALIX_CALIB_SLOT_QUAD	BIT(4)
#define MODALIX_CALIB_SLOT_REQ	BIT(5)

struct modalix_calib_slot_type {
	__u16 slot;
	__u16 flags;
	__u16 min_elems;
	/* 0 = no upper bound. Set for LUTs the FSM copies straight into a
	 * fixed hardware register array, where a longer LUT does not merely
	 * over-read the blob - it writes past the array into whatever ISP
	 * registers follow it. */
	__u16 max_elems;
};

#define W1	MODALIX_CALIB_SLOT_W1
#define W2	MODALIX_CALIB_SLOT_W2
#define W4	MODALIX_CALIB_SLOT_W4
#define MOD	MODALIX_CALIB_SLOT_MOD
#define QUAD	MODALIX_CALIB_SLOT_QUAD
#define REQ	MODALIX_CALIB_SLOT_REQ
#define SLOT(id, f, n)		{ .slot = (id), .flags = (f), .min_elems = (n) }
#define SLOT_EXACT(id, f, n)	{ .slot = (id), .flags = (f), .min_elems = (n), .max_elems = (n) }
#define SLOT_MAX(id, f, n, x)	{ .slot = (id), .flags = (f), .min_elems = (n), .max_elems = (x) }

static const struct modalix_calib_slot_type modalix_calib_slot_types[] = {
	SLOT(MODALIX_ISP_CALIB_AE_ZONE_WGHT_HOR, W2 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_AE_ZONE_WGHT_VER, W2 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_AWB_WARMING_LS_D50, W2 | REQ, 3),
	SLOT(MODALIX_ISP_CALIB_AWB_ZONE_WGHT_HOR, W2 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_AWB_ZONE_WGHT_VER, W2 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_BLACK_LEVEL_B, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_BLACK_LEVEL_GB, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_BLACK_LEVEL_GR, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_BLACK_LEVEL_R, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_CALIBRATION_BIT_DEPTH, W2, 1),
	SLOT(MODALIX_ISP_CALIB_CA_CORRECTION, W2, 3),
	SLOT(MODALIX_ISP_CALIB_CA_CORRECTION_MEM, W2, 20),
	SLOT_EXACT(MODALIX_ISP_CALIB_CA_FILTER_MEM, W4 | REQ, 32),
	SLOT(MODALIX_ISP_CALIB_CCM_ONE_GAIN_THRESHOLD, W2 | REQ, 1),
	/* 20 words: what cmos_control_param_t is and all the kernel reads. The
	 * producer sends more; requiring that many would reject a descriptor
	 * carrying every field. cmos_func.c asserts the struct still fits. */
	SLOT(MODALIX_ISP_CALIB_CMOS_CONTROL, W4 | REQ, 20),
	SLOT(MODALIX_ISP_CALIB_COLOR_MATRIX_B_LUV_PRESETS, W2 | REQ, 3),
	SLOT(MODALIX_ISP_CALIB_COLOR_MATRIX_B_YUV_PRESETS, W2 | REQ, 3),
	SLOT(MODALIX_ISP_CALIB_COLOR_MATRIX_LUV_PRESETS, W2 | REQ, 9),
	SLOT(MODALIX_ISP_CALIB_COLOR_MATRIX_YUV_PRESETS, W2 | REQ, 9),
	SLOT(MODALIX_ISP_CALIB_COLOR_TEMP, W2 | REQ, 1),
	SLOT(MODALIX_ISP_CALIB_CT_BG_POS_CALC, W2 | REQ, 1),
	SLOT(MODALIX_ISP_CALIB_CT_RG_POS_CALC, W2 | REQ, 1),
	SLOT(MODALIX_ISP_CALIB_CUSTOM_SETTINGS, W4 | QUAD, 4),
	SLOT(MODALIX_ISP_CALIB_DEFECT_PIXEL, W4 | REQ, 0),
	SLOT_EXACT(MODALIX_ISP_CALIB_DEMOSAIC, W1 | REQ, 128),
	SLOT(MODALIX_ISP_CALIB_DEMOSAIC_CONFIG, W2, 28),
	SLOT(MODALIX_ISP_CALIB_DEMOSAIC_UU_SLOPE, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_DP_SLOPE, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_DP_THRESHOLD, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_FS_MC_OFF, W2, 1),
	/* The decompander owns the input formatter, both gamma blocks and the
	 * linear source mux, and nothing else resets them. Absence used to
	 * leave the previous WDR mode programmed, so require it: the resolved
	 * blob must carry it even when a layer does not override it. */
	SLOT_EXACT(MODALIX_ISP_CALIB_DECOMPANDER_CONTROL, W1 | REQ, 8),
	SLOT_EXACT(MODALIX_ISP_CALIB_GAMMA, W4 | REQ, 257),
	SLOT(MODALIX_ISP_CALIB_GAMMA_BE0, W4 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_GAMMA_BE1, W4 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_GAMMA_BLACK_LEVELS, W4, 2),
	SLOT_EXACT(MODALIX_ISP_CALIB_GAMMA_FE, W4 | REQ, 4097),
	SLOT(MODALIX_ISP_CALIB_INPUT_FORMATTER, W2 | REQ, 17),
	SLOT(MODALIX_ISP_CALIB_IRIDIX8_EXTENDED_CONTROL, W4, 14),
	SLOT_MAX(MODALIX_ISP_CALIB_IRIDIX_ASYMMETRY, W2 | W4 | REQ, 1, 65),
	SLOT(MODALIX_ISP_CALIB_IRIDIX_AVG_COEF, W1 | REQ, 1),
	SLOT_EXACT(MODALIX_ISP_CALIB_IRIDIX_GTM_LUT_X, W4 | REQ, 33),
	SLOT_EXACT(MODALIX_ISP_CALIB_IRIDIX_GTM_LUT_Y, W4 | REQ, 33),
	SLOT(MODALIX_ISP_CALIB_IRIDIX_STRENGTH_MAXIMUM, W1 | REQ, 1),
	SLOT(MODALIX_ISP_CALIB_MESH_SHADING_STRENGTH, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_MT_ABSOLUTE_LS_A_CCM, W2 | REQ, 9),
	SLOT(MODALIX_ISP_CALIB_MT_ABSOLUTE_LS_D40_CCM, W2 | REQ, 9),
	SLOT(MODALIX_ISP_CALIB_MT_ABSOLUTE_LS_D50_CCM, W2 | REQ, 9),
	SLOT(MODALIX_ISP_CALIB_NEQ_LUT, W4 | MOD | REQ, 0),
	SLOT_EXACT(MODALIX_ISP_CALIB_NOISE_PROFILE, W1 | REQ, 128),
	SLOT(MODALIX_ISP_CALIB_NOISE_PROFILE_CONFIG, W2, 8),
	SLOT(MODALIX_ISP_CALIB_RAW_FRONTEND_CONFIG, W2, 8),
	SLOT(MODALIX_ISP_CALIB_RGB2RGB_HS_CONVERSION, W2 | REQ, 9),
	SLOT(MODALIX_ISP_CALIB_RGB2RGB_HS_CONVERSION_B, W2 | REQ, 3),
	SLOT(MODALIX_ISP_CALIB_RGB2RGB_S2_CONVERSION, W2 | REQ, 9),
	SLOT(MODALIX_ISP_CALIB_RGB2RGB_S2_CONVERSION_B, W2 | REQ, 3),
	SLOT(MODALIX_ISP_CALIB_SATURATION_STRENGTH, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_A_B, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_A_G, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_A_R, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_D65_B, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_D65_G, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_D65_R, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_TL84_B, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_TL84_G, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_LS_TL84_R, W1 | REQ, 1024),
	SLOT(MODALIX_ISP_CALIB_SHADING_RADIAL_B, W4 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_SHADING_RADIAL_CENTRE_AND_MULT, W4 | REQ, 6),
	SLOT(MODALIX_ISP_CALIB_SHADING_RADIAL_G, W4 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_SHADING_RADIAL_IR, W4 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_SHADING_RADIAL_R, W4 | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_SHARP_ALT_D, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SHARP_ALT_DU, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SHARP_ALT_UD, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SINTER_INTCONFIG, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SINTER_PARAMS, W2, 1),
	SLOT_EXACT(MODALIX_ISP_CALIB_SINTER_RADIAL_LUT, W1 | REQ, 33),
	SLOT(MODALIX_ISP_CALIB_SINTER_RADIAL_PARAMS, W2 | REQ, 4),
	SLOT(MODALIX_ISP_CALIB_SINTER_STRENGTH, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SINTER_STRENGTH1, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SINTER_STRENGTH4, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SINTER_STRENGTH_MC_CONTRAST, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SINTER_THRESH1, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_SINTER_THRESH4, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_STATIC_WB, W2 | REQ, 4),
	SLOT(MODALIX_ISP_CALIB_STATISTICS_CONFIG, W4, 6),
	SLOT(MODALIX_ISP_CALIB_STITCHING_LM_MED_NOISE_INTENSITY, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_STITCHING_LM_MOV_MULT, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_STITCHING_LM_NP, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_STITCHING_MS_MOV_MULT, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_STITCHING_MS_NP, W2 | MOD | REQ, 0),
	SLOT(MODALIX_ISP_CALIB_STITCHING_SVS_MOV_MULT, W2 | MOD, 0),
	SLOT(MODALIX_ISP_CALIB_STITCHING_SVS_NP, W2 | MOD, 0),
	SLOT_EXACT(MODALIX_ISP_CALIB_WDR_NP_LUT, W1 | REQ, 128),
	SLOT(MODALIX_ISP_CALIB_WDR_STITCH_CONFIG, W4, 12),
};

#undef SLOT_MAX
#undef SLOT_EXACT
#undef SLOT
#undef REQ
#undef QUAD
#undef MOD
#undef W4
#undef W2
#undef W1

#endif /* __ACAMERA_CALIB_MGR_SLOT_TYPES_H__ */
