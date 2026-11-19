/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Metoak S315 SIMOR stereo payload (V4L2_PIX_FMT_METOAK_RGBD), from the S315
 * External Development Guide section 5. 640x360 x 24 bits, sent as RAW8 at
 * three samples per pixel (1920x360 bytes). Memory order B, G, R:
 *   B + G[3:0]  12-bit disparity
 *   G[7:4]      reference image UV, 2x2 subsampled, U/V by row/column parity
 *   R           reference image Y
 * Not a picture: the vendor forbids any ISP or image processing on it.
 */

#ifndef __UAPI_METOAK_RGBD_H__
#define __UAPI_METOAK_RGBD_H__

#include <linux/types.h>

/* Transport bytes per pixel, and the resulting bus-to-image width ratio. */
#define METOAK_RGBD_BPP			3

/* Image geometry. The CSI-2 line carries METOAK_RGBD_BPP times the width. */
#define METOAK_RGBD_WIDTH		640
#define METOAK_RGBD_HEIGHT		360

/* Byte offsets within one pixel triplet (memory order B, G, R). */
#define METOAK_RGBD_OFF_B		0
#define METOAK_RGBD_OFF_G		1
#define METOAK_RGBD_OFF_R		2

/*
 * Row 0 is metadata and rows 1..3 unusable: shift disparity up by
 * METOAK_RGBD_SKIP_ROWS rows and zero-pad the tail to align it with the image.
 */
#define METOAK_RGBD_META_ROW		0
#define METOAK_RGBD_SKIP_ROWS		4

/*
 * Metadata: 12-bit words packed like disparity, word i in pixel i. The
 * development guide numbers the same words D1..D640, one higher than the
 * indices here. Only these fields are known; the rest are unspecified.
 * The baseline field also carries Base and the focal length, unexposed.
 */
#define METOAK_RGBD_META_MAGIC_WORD	0	/* 4 words, see below */
#define METOAK_RGBD_META_BF_WORD	18	/* 4 words, see metoak_rgbd_meta_bf() */
#define METOAK_RGBD_META_FRAME_WORD	23	/* 3 words, big-endian */

/* Metadata magic in the first four 12-bit words of the metadata row. */
#define METOAK_RGBD_MAGIC0		0xACE
#define METOAK_RGBD_MAGIC1		0xBED
#define METOAK_RGBD_MAGIC2		0xCAB
#define METOAK_RGBD_MAGIC3		0xDEF

/* Fixed-point scales: disparity is 8.4 pixels, BF 20.12 millimetre-pixels. */
#define METOAK_RGBD_DISP_SUBPIXEL	16
#define METOAK_RGBD_BF_SCALE		4096

/* Specified sensing range (datasheet R2.1 section 3.1), 60 mm baseline. */
#define METOAK_RGBD_MIN_MM		200
#define METOAK_RGBD_MAX_MM		10000

static inline __u16 metoak_rgbd_disparity(const __u8 *px)
{
	return (__u16)((px[METOAK_RGBD_OFF_G] & 0x0F) << 8) |
	       px[METOAK_RGBD_OFF_B];
}

static inline __u8 metoak_rgbd_luma(const __u8 *px)
{
	return px[METOAK_RGBD_OFF_R];
}

/*
 * 4-bit chroma. Which parity carries U and which V is unspecified by the
 * vendor; establish it before relying on colour.
 */
static inline __u8 metoak_rgbd_chroma(const __u8 *px)
{
	return px[METOAK_RGBD_OFF_G] >> 4;
}

/* @row points at the first byte of the metadata row. */
static inline __u16 metoak_rgbd_meta_word(const __u8 *row, unsigned int i)
{
	return metoak_rgbd_disparity(row + (i * METOAK_RGBD_BPP));
}

static inline int metoak_rgbd_meta_valid(const __u8 *row)
{
	return metoak_rgbd_meta_word(row, METOAK_RGBD_META_MAGIC_WORD + 0) ==
		       METOAK_RGBD_MAGIC0 &&
	       metoak_rgbd_meta_word(row, METOAK_RGBD_META_MAGIC_WORD + 1) ==
		       METOAK_RGBD_MAGIC1 &&
	       metoak_rgbd_meta_word(row, METOAK_RGBD_META_MAGIC_WORD + 2) ==
		       METOAK_RGBD_MAGIC2 &&
	       metoak_rgbd_meta_word(row, METOAK_RGBD_META_MAGIC_WORD + 3) ==
		       METOAK_RGBD_MAGIC3;
}

/*
 * Baseline x focal length as 20.12 fixed point in millimetre-pixels, per
 * module and per frame. The four words are not a plain concatenation: the
 * field packs Base and the focal length ahead of it, so BF starts on a
 * nibble boundary inside the first word and ends inside the last.
 */
static inline __u32 metoak_rgbd_meta_bf(const __u8 *row)
{
	unsigned int w = METOAK_RGBD_META_BF_WORD;

	return ((__u32)(metoak_rgbd_meta_word(row, w) & 0x00F) << 28) |
	       ((__u32)metoak_rgbd_meta_word(row, w + 1) << 16) |
	       ((__u32)metoak_rgbd_meta_word(row, w + 2) << 4) |
	       (metoak_rgbd_meta_word(row, w + 3) >> 8);
}

/* 36-bit free-running frame counter, most significant word first. */
static inline __u64 metoak_rgbd_meta_frame(const __u8 *row)
{
	unsigned int w = METOAK_RGBD_META_FRAME_WORD;

	return ((__u64)metoak_rgbd_meta_word(row, w) << 24) |
	       ((__u64)metoak_rgbd_meta_word(row, w + 1) << 12) |
	       metoak_rgbd_meta_word(row, w + 2);
}

/* Zero disparity is "no measurement", and stays zero rather than dividing. */
static inline __u32 metoak_rgbd_depth_mm(__u16 disparity, __u32 bf)
{
	if (!disparity || !bf)
		return 0;

	/*
	 * (bf / METOAK_RGBD_BF_SCALE) / (disparity / METOAK_RGBD_DISP_SUBPIXEL)
	 * as one division: neither fraction is lost and bf cannot overflow.
	 */
	return bf / ((__u32)disparity *
		     (METOAK_RGBD_BF_SCALE / METOAK_RGBD_DISP_SUBPIXEL));
}

#endif /* __UAPI_METOAK_RGBD_H__ */
