/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2026 SiMa Technologies, Inc.
 */

#ifndef __SIMMAI_V4L2_VDMA_H__
#define __SIMMAI_V4L2_VDMA_H__

#include <linux/build_bug.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#define VDMA_CHANNELS			4

/*
 * The CSI/IPI appends one fixed-size metadata descriptor after each frame (a
 * SYNC_VIDEO_PKT/SYNC_EMBEDDED_PKT trailer). This is the single definition of
 * that size; it is reserved by sizeimage and excluded from the V4L2 payload,
 * and the scratch buffer inherits it via sizeimage. Distinct from the glue's
 * interlaced embedded-data regions (META_EMB_LD/EMB_TR), which are a separate
 * concern and 0 for progressive sensors.
 */
#define VDMA_FRAME_METADATA_SIZE	32
#define VDMA_FRAME_META_MAGIC		0x2c000453u

/* Layout of the per-frame metadata trailer (board-verified, little-endian). */
struct vdma_frame_meta {
	__le32	magic;		/* VDMA_FRAME_META_MAGIC when present */
	__le16	sequence;	/* hardware frame counter */
	__le16	status;		/* frame-valid flag (0 => bad frame) */
	__le16	width;
	__le16	height;
	__le16	stride;
	__le16	rsvd0;
	__le32	rsvd1;
	__le32	timestamp;
	__le32	rsvd2[2];
} __packed;
static_assert(sizeof(struct vdma_frame_meta) == VDMA_FRAME_METADATA_SIZE,
	      "frame metadata struct must match VDMA_FRAME_METADATA_SIZE");

struct vdma_channel;
struct vdma_buffer {
	struct vb2_v4l2_buffer		vb;
	size_t				size;
	struct vdma_channel		*channel;
};

struct vdma_channel *get_vdma_channel(struct v4l2_subdev *sd, unsigned int pad);
u64 get_vdma_channel_mask(struct vdma_channel *channel);
void vdma_buffer_queue(struct vb2_buffer *vb);

#endif
