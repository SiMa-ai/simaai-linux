/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2026 SiMa Technologies, Inc.
 *
 * V4L2 META_CAPTURE stats device — per-ISP-context vb2_queue +
 * video_device that publishes the kernel's per-frame 3A statistics
 * (struct modalix_isp_stats_buffer) to user-space. This is the
 * vb2-based replacement for the legacy /dev/isp_sbuf<N> read()+mmap
 * protocol, scheduled to retire sbuf entirely in a later phase.
 *
 * Reference pattern: rkisp1's drivers/media/platform/rockchip/rkisp1/
 * rkisp1-stats.c. Same vb2_ops shape; per-ISP-context lifetime; the
 * BH stats-ready dispatch calls modalix_meta_stats_publish() with
 * the populated payload to release a buffer to user-space.
 */

#ifndef _ISP_V4L2_META_STATS_H_
#define _ISP_V4L2_META_STATS_H_

#include <linux/types.h>

struct v4l2_device;
struct modalix_meta_stats_dev;
struct modalix_isp_stats_buffer;

int  modalix_meta_stats_init( uint32_t ctx_id,
                              struct v4l2_device *v4l2_dev,
                              struct modalix_meta_stats_dev **out );
void modalix_meta_stats_exit( struct modalix_meta_stats_dev *dev );

/**
 * modalix_meta_stats_lookup - find the meta-stats device for an ISP
 * context.
 *
 * @ctx_id: ISP context id (0 .. FIRMWARE_CONTEXT_NUMBER - 1).
 *
 * Used by the kernel BH stats-ready dispatch to get a publish target
 * without taking a dependency on isp-v4l2's per-context state. Returns
 * NULL if the context's meta device hasn't been initialised yet (the
 * caller should fall back to the legacy sbuf path in that case).
 */
struct modalix_meta_stats_dev *modalix_meta_stats_lookup( uint32_t ctx_id );

/**
 * modalix_meta_stats_publish - hand a populated stats payload to the
 * V4L2 META_CAPTURE queue.
 *
 * @dev:     per-context meta-stats device.
 * @payload: caller-owned modalix_isp_stats_buffer to copy from. The
 *           function does not retain the pointer past return; the
 *           caller is free to reuse / stack-allocate the payload.
 *
 * Pops the head of the pending-buffer FIFO (filled by user-space
 * VIDIOC_QBUF), memcpys @payload into it, marks it
 * VB2_BUF_STATE_DONE so VIDIOC_DQBUF / poll() can return it.
 *
 * Returns 0 on success, -ENODATA if no pending buffer is available
 * (user-space hasn't queued one yet, or the queue is not streaming).
 * The kernel BH should treat that as a "drop this frame's stats"
 * event — same semantics as a kfifo overflow on the legacy IRQ
 * path.
 *
 * Safe to call from BH / softirq context. Holds dev->buf_lock with
 * IRQs disabled around the list pop; vb2_buffer_done itself is
 * documented IRQ-safe.
 */
int modalix_meta_stats_publish( struct modalix_meta_stats_dev *dev,
                                const struct modalix_isp_stats_buffer *payload );

/**
 * modalix_meta_stats_publish_ae - direct-from-FSM AE stats publisher.
 *
 * Looks up the per-context meta-stats device, allocates a transient
 * modalix_isp_stats_buffer, populates the AE section from @histogram
 * + @sum, and hands it to modalix_meta_stats_publish. No-op when the
 * context hasn't been initialised.
 *
 * @ctx_id:    ISP context id.
 * @histogram: pointer to histogram bin array (length must equal
 *             MODALIX_ISP_HISTOGRAM_BINS).
 * @sum:       total pixel count contributing to @histogram.
 *
 * Called directly by histogram_func.c on stats-ready, replacing the
 * legacy sbuf get_item / set_item dance that buffered the same data
 * for the chardev path.
 */
void modalix_meta_stats_publish_ae( uint32_t        ctx_id,
                                    const uint32_t *histogram,
                                    uint32_t        sum );

/**
 * modalix_meta_stats_publish_awb - direct-from-FSM AWB stats
 * publisher. Same shape as the AE helper; @zones is an opaque
 * pointer to the per-zone array (binary-compatible with
 * struct modalix_isp_awb_zone), @zone_count is bounded by
 * MODALIX_ISP_AWB_MAX_ZONES.
 */
void modalix_meta_stats_publish_awb( uint32_t    ctx_id,
                                     const void *zones,
                                     uint32_t    zone_count );

#endif /* _ISP_V4L2_META_STATS_H_ */
