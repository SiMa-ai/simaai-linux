/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2026 SiMa Technologies, Inc.
 *
 * V4L2 META_OUTPUT params device — per-ISP-context vb2_queue +
 * video_device that receives the libcamera IPA's per-frame 3A
 * decisions (struct modalix_isp_params_buffer) from user-space and
 * hands them to the kernel's ISP apply path. This is the vb2-based
 * replacement for the legacy /dev/isp_sbuf<N> write() + mmap
 * "changed_flag" channel; pairs with isp-v4l2-meta-stats.c on the
 * other direction.
 *
 * Reference pattern: rkisp1's drivers/media/platform/rockchip/rkisp1/
 * rkisp1-params.c. Same vb2_ops shape; per-ISP-context lifetime; the
 * kernel apply path calls modalix_meta_params_consume() to take
 * ownership of the next user-queued buffer.
 */

#ifndef _ISP_V4L2_META_PARAMS_H_
#define _ISP_V4L2_META_PARAMS_H_

#include <linux/types.h>

struct v4l2_device;
struct modalix_meta_params_dev;
struct modalix_isp_params_buffer;

int  modalix_meta_params_init( uint32_t ctx_id,
                               struct v4l2_device *v4l2_dev,
                               struct modalix_meta_params_dev **out );
void modalix_meta_params_exit( struct modalix_meta_params_dev *dev );

/**
 * modalix_meta_params_lookup - find the meta-params device for an ISP
 * context.
 *
 * @ctx_id: ISP context id (0 .. FIRMWARE_CONTEXT_NUMBER - 1).
 *
 * Returns NULL if the context's meta-params device hasn't been
 * initialised yet (the caller should fall back to the legacy sbuf
 * write-back path in that case).
 */
struct modalix_meta_params_dev *modalix_meta_params_lookup( uint32_t ctx_id );

/**
 * modalix_meta_params_consume - pop the head of the user-queued
 * params FIFO into the caller-owned @out and complete the buffer.
 *
 * @dev: per-context meta-params device.
 * @out: caller-owned destination; on success this is filled with the
 *       payload the user-space QBUF'd. The kernel does not retain
 *       a reference to the buffer past return.
 *
 * Returns 0 on success, -ENODATA if the queue is empty or not
 * streaming. The kernel BH should treat ENODATA as "no fresh params
 * this frame" and keep last-known-good values, same semantics as the
 * legacy sbuf changed_flag=0 case.
 *
 * Safe to call from BH context. Holds dev->buf_lock with IRQs
 * disabled around the list pop; vb2_buffer_done is documented
 * IRQ-safe.
 */
int modalix_meta_params_consume( struct modalix_meta_params_dev *dev,
                                 struct modalix_isp_params_buffer *out );

/**
 * isp_apply_v4l2_params_drain - drain all currently-queued V4L2 params
 * for an ISP context and push them into the kernel apply path.
 *
 * Implemented in sbuf_func.c (the apply-path owner today) and called
 * from the meta-params buf_queue worker so apply runs as soon as the
 * IPA QBUFs a params buffer, independently of any sbuf chardev
 * activity.
 *
 * @ctx_id: ISP context id.
 *
 * Safe to call from a workqueue context. Returns the number of
 * buffers consumed (>= 0) on success, or a negative errno on hard
 * failure (e.g. unknown ctx).
 */
int isp_apply_v4l2_params_drain( uint32_t ctx_id );

#endif /* _ISP_V4L2_META_PARAMS_H_ */
