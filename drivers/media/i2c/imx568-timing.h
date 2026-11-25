/* SPDX-License-Identifier: GPL-2.0 */
/* Shared Sony IMX568 line-timing helpers. */
#ifndef _IMX568_TIMING_H_
#define _IMX568_TIMING_H_

#include <linux/kernel.h>
#include <linux/math64.h>

#define IMX568_LANE_RATE_594	594000000U
#define IMX568_LANE_RATE_891	891000000U
#define IMX568_LANE_RATE_1188	1188000000U

#define IMX568_HMAX_CLK_HZ		74250000ULL

#define IMX568_HMAX_594_2LANE_RAW12	1912U
#define IMX568_HMAX_1188_2LANE_RAW8	656U
#define IMX568_HMAX_1188_4LANE_RAW8	348U
#define IMX568_HMAX_1188_2LANE_RAW10	810U
#define IMX568_HMAX_1188_4LANE_RAW10	425U
#define IMX568_HMAX_1188_2LANE_RAW12	965U
#define IMX568_HMAX_1188_4LANE_RAW12	502U
#define IMX568_HMAX_1188_2LANE_FULL	1100U
#define IMX568_HMAX_1188_4LANE_FULL	573U

#define IMX568_GMRWT_1188_2LANE_RAW12	0x02
#define IMX568_GMTWT_1188_2LANE_RAW12	0x10
#define IMX568_GSDLY_1188_2LANE_RAW12	0x08

struct imx568_timing {
	u16 hmax;
	u8 gmrwt;
	u8 gmtwt;
	u8 gsdly;
};

/* IMX568-AAQJ-C data sheet table 18.3, for a 37.125 MHz INCK. */
static inline const struct imx568_timing *
imx568_timing_find(u32 lane_rate, u32 bpp, u32 lanes)
{
	static const struct imx568_timing timings[3][3][2] = {
		/* 594 Mbit/s/lane: RAW8, RAW10, RAW12; 2 lanes, 4 lanes. */
		{
			{ { 1297, 0x02, 0x0c, 0x06 }, { 682, 0x04, 0x16, 0x0a } },
			{ { 1603, 0x02, 0x0a, 0x04 }, { 835, 0x04, 0x12, 0x08 } },
			{
				{ IMX568_HMAX_594_2LANE_RAW12, 0x02, 0x08, 0x04 },
				{ 989, 0x02, 0x10, 0x08 },
			},
		},
		/* 891 Mbit/s/lane. */
		{
			{ { 870, 0x04, 0x12, 0x08 }, { 460, 0x06, 0x20, 0x0e } },
			{ { 1074, 0x02, 0x0e, 0x06 }, { 562, 0x04, 0x1a, 0x0c } },
			{ { 1280, 0x02, 0x0c, 0x06 }, { 664, 0x04, 0x16, 0x0a } },
		},
		/* 1188 Mbit/s/lane, also read by imx568_timing_hmax() for e-con. */
		{
			{
				{ IMX568_HMAX_1188_2LANE_RAW8, 0x04, 0x16, 0x0a },
				{ IMX568_HMAX_1188_4LANE_RAW8, 0x06, 0x2a, 0x12 },
			},
			{
				{ IMX568_HMAX_1188_2LANE_RAW10, 0x04, 0x12, 0x08 },
				{ IMX568_HMAX_1188_4LANE_RAW10, 0x06, 0x22, 0x10 },
			},
			{
				{ IMX568_HMAX_1188_2LANE_RAW12, IMX568_GMRWT_1188_2LANE_RAW12,
				  IMX568_GMTWT_1188_2LANE_RAW12, IMX568_GSDLY_1188_2LANE_RAW12 },
				{ IMX568_HMAX_1188_4LANE_RAW12, 0x04, 0x1e, 0x0e },
			},
		},
	};
	unsigned int rate_index;
	unsigned int depth_index;
	unsigned int lane_index;

	switch (lane_rate) {
	case IMX568_LANE_RATE_594:
		rate_index = 0;
		break;
	case IMX568_LANE_RATE_891:
		rate_index = 1;
		break;
	case IMX568_LANE_RATE_1188:
		rate_index = 2;
		break;
	default:
		return NULL;
	}

	switch (bpp) {
	case 8:
		depth_index = 0;
		break;
	case 10:
		depth_index = 1;
		break;
	case 12:
		depth_index = 2;
		break;
	default:
		return NULL;
	}

	if (lanes == 2)
		lane_index = 0;
	else if (lanes == 4)
		lane_index = 1;
	else
		return NULL;

	return &timings[rate_index][depth_index][lane_index];
}

/* Pixel rate and line length in the V4L2 pixel domain; the line rounds up. */
static inline u64 imx568_pixel_rate(u32 lane_rate, u32 lanes, u32 bpp)
{
	return div_u64((u64)lane_rate * lanes, bpp);
}

static inline u32 imx568_hmax_to_line_length(u32 hmax, u32 lane_rate,
					     u32 lanes, u32 bpp)
{
	return DIV_ROUND_UP_ULL((u64)hmax *
				imx568_pixel_rate(lane_rate, lanes, bpp),
				IMX568_HMAX_CLK_HZ);
}

/* e-con compatibility domain: HMAX in K x 74.25 MHz, separate from the V4L2 domain. */
#define IMX568_TIMING_K(lanes)		((lanes) == 4 ? 6u : 3u)
#define IMX568_TIMING_PIXEL_RATE(lanes)	(IMX568_HMAX_CLK_HZ * IMX568_TIMING_K(lanes))

/* Datasheet HMAX at 1188 Mbps/lane, by AD bit depth and lane count. */
static inline u32 imx568_timing_hmax(u32 bpp, u32 lanes)
{
	u32 compat_bpp, compat_lanes;

	/* Normalize to the compat helper's bit-depth and lane set. */
	compat_bpp = bpp == 12 ? 12 : (bpp == 10 ? 10 : 8);
	compat_lanes = lanes == 4 ? 4 : 2;
	/* Every 1188 Mbit/s row exists, so the lookup cannot fail. */
	return imx568_timing_find(IMX568_LANE_RATE_1188, compat_bpp,
				  compat_lanes)->hmax;
}

/* Line length in the K x 74.25 MHz domain. */
static inline u32 imx568_timing_line_px(u32 bpp, u32 lanes)
{
	return imx568_timing_hmax(bpp, lanes) * IMX568_TIMING_K(lanes);
}

/*
 * Same law keyed on a mode's two-lane HMAX, for callers that carry an HTS per
 * mode rather than a bit depth (the full-res mode is not in the depth table).
 */
static inline u32 imx568_timing_hmax_from_hts(u32 hts_2lane, u32 lanes)
{
	if (lanes != 4)
		return hts_2lane;
	switch (hts_2lane) {
	case IMX568_HMAX_1188_2LANE_RAW12:
		return IMX568_HMAX_1188_4LANE_RAW12;
	case IMX568_HMAX_1188_2LANE_FULL:
		return IMX568_HMAX_1188_4LANE_FULL;
	case IMX568_HMAX_1188_2LANE_RAW10:
		return IMX568_HMAX_1188_4LANE_RAW10;
	case IMX568_HMAX_1188_2LANE_RAW8:
		return IMX568_HMAX_1188_4LANE_RAW8;
	default:	return DIV_ROUND_UP(hts_2lane, 2);
	}
}

#endif /* _IMX568_TIMING_H_ */
