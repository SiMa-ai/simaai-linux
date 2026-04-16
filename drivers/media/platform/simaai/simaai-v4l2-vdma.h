/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2026 SiMa Technologies, Inc.
 */

#ifndef __SIMMAI_V4L2_VDMA_H__
#define __SIMMAI_V4L2_VDMA_H__

#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#define VDMA_CHANNELS			4
#define VDMA_FRAME_METADATA_SIZE	32

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
