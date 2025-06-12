/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 Synopsys, Inc.
 *
 * Synopsys DesignWare MIPI CSI-2 Host controller driver.
 * Supported bus formats
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#ifndef _DW_CSI_PLAT_H__
#define _DW_CSI_PLAT_H__

#include "dw_mipi_csi.h"

/* Video formats supported by the MIPI CSI-2 */
struct mipi_fmt dw_mipi_csi_formats[] = {
	{
		/* RAW 8 */
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.depth = 8,
	}, {
		/* RAW 10 */
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.depth = 10,
	}, {
		/* RAW 12 */
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.depth = 12,
	}, {
		/* RAW 14 */
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.depth = 14,
	}, {
		/* RAW 16 */
		.code = MEDIA_BUS_FMT_SRGGB16_1X16,
		.depth = 16,
	}, {
		/* RGB 666 */
		.code = MEDIA_BUS_FMT_RGB666_1X18,
		.depth = 18,
	}, {
		/* RGB 565 */
		.code = MEDIA_BUS_FMT_RGB565_2X8_BE,
		.depth = 16,
	}, {
		/* BGR 565 */
		.code = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.depth = 16,
	}, {
		/* RGB 555 */
		.code = MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE,
		.depth = 16,
	}, {
		/* BGR 555 */
		.code = MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.depth = 16,
	}, {
		/* RGB 444 */
		.code = MEDIA_BUS_FMT_RGB444_2X8_PADHI_BE,
		.depth = 16,
	}, {
		/* RGB 444 */
		.code = MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE,
		.depth = 16,
	}, {
		/* RGB 888 */
		.code = MEDIA_BUS_FMT_RGB888_2X12_LE,
		.depth = 24,
	}, {
		/* BGR 888 */
		.code = MEDIA_BUS_FMT_RGB888_2X12_BE,
		.depth = 24,
	}, {
		/* BGR 888 */
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.depth = 24,
	}, {
		/* YUV 422 8-bit */
		.code = MEDIA_BUS_FMT_VYUY8_1X16,
		.depth = 16,
	}, {
		/* YUV 422 10-bit */
		.code = MEDIA_BUS_FMT_VYUY10_2X10,
		.depth = 20,
	}, {
		/* YUV 420 8-bit LEGACY */
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.depth = 8,
	}, {
		/* YUV 420 10-bit */
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.depth = 10,
	},
};

#endif	/* _DW_CSI_PLAT_H__ */
