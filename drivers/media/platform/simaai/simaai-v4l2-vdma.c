// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 SiMa Technologies, Inc.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/dmaengine.h>

#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#include "simaai-v4l2-vdma.h"

#define VDMA_NAME_LENGTH		16

struct vdma_channel {
	struct vdma_dev			*dev;
	char				name[VDMA_NAME_LENGTH];
	unsigned int			id;
	unsigned int			sequence;
	struct dma_chan			*dma;
	struct fwnode_handle		*ep_fwnode;
	struct v4l2_subdev		*ep_sd;
	unsigned int			ep_pad;
	struct v4l2_mbus_framefmt	fmt;
	struct mutex			lock;
	unsigned int 			buf_count;
	bool				streaming;
	bool				enabled;
	struct timer_list		fwd_timer;	/* frame watchdog */
	unsigned int			fwd_fires;	/* consecutive watchdog fires */
	struct dma_interleaved_template	xt;
	struct data_chunk 		sgl;
	/*
	 * frame_size is the live buffer size (incl. metadata); the DMA provider
	 * owns the throwaway scratch buffer it loops on when no user buffer is
	 * queued, so user buffers complete to userspace immediately.
	 */
	size_t				frame_size;
};

/* Provided by dw-axi-dmac (built-in); arms/disarms the per-channel scratch loop.
 * The provider allocates and owns the throwaway buffer -- pass the frame size. */
int dw_axi_dma_arm_scratch(struct dma_chan *dchan, size_t size);
void dw_axi_dma_disarm_scratch(struct dma_chan *dchan);
int dw_axi_dma_quiesce_for_overflow(struct dma_chan *dchan);
int dw_axi_dma_resume_after_overflow(struct dma_chan *dchan);
int dw_axi_dma_inject_halt(struct dma_chan *dchan);	/* test injector */

/* Provided by dwc-mipi-csi2 (the upstream CSI subdev); registers the DMA
 * quiesce/resume callbacks used by the CSI IPI-overflow recovery (SOCSW-5392). */
int dwc_csi_register_overflow_recovery(struct v4l2_subdev *sd,
				       void (*quiesce)(void *data),
				       void (*resume)(void *data), void *data);
void dwc_csi_trigger_overflow_recovery(struct v4l2_subdev *sd);

/* CSI IPI-overflow recovery parks the DMA across the CSI reset and restarts
 * it after, via these thunks. */
static void vdma_dma_quiesce(void *dma_chan)
{
	dw_axi_dma_quiesce_for_overflow((struct dma_chan *)dma_chan);
}

static void vdma_dma_resume(void *dma_chan)
{
	dw_axi_dma_resume_after_overflow((struct dma_chan *)dma_chan);
}

/* frame-watchdog timeout in ms, 0 = off. */
static unsigned int frame_watchdog_ms = 5000;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(frame_watchdog_ms, uint, 0644);
MODULE_PARM_DESC(frame_watchdog_ms, "frame-watchdog timeout in ms (0 = disabled)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

/* Max consecutive recovery attempts before giving up. 0 = infinite (never give
 * up, keep recovering as long as the stall persists). The counter is reset to 0
 * on every completed frame, so it counts *consecutive* failures, not lifetime. */
static unsigned int frame_watchdog_max_retry = 3;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(frame_watchdog_max_retry, uint, 0644);
MODULE_PARM_DESC(frame_watchdog_max_retry,
		 "max consecutive watchdog recovery attempts before giving up (0 = infinite)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

/*
 * Test knob: skip arming the DMA scratch fallback so the DMA runs out of
 * descriptors mid-stream and the CSI IPI overflows (SOCSW-5392) — used to
 * validate the IPI-overflow recovery. Default off.
 */
static bool scratch_disable;
#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
module_param(scratch_disable, bool, 0644);
MODULE_PARM_DESC(scratch_disable, "skip DMA scratch arm (force mid-stream IPI overflow; test only)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

/*
 * Test injector: writing to this param halts the active channel's DMA the way
 * descriptor-exhaustion does (clears the scratch LLI_VALID), deterministically
 * reproducing the wedge so IPI-only vs full recovery can be compared. The
 * active channel's dma_chan is published here at stream-on.
 */
static struct dma_chan *g_inject_dma;

#ifdef CONFIG_SIMAAI_CAMERA_INSTRUMENTATION
static int dma_inject_halt_set(const char *val, const struct kernel_param *kp)
{
	struct dma_chan *dc = READ_ONCE(g_inject_dma);

	if (!dc)
		return -ENODEV;
	return dw_axi_dma_inject_halt(dc);
}
static const struct kernel_param_ops dma_inject_halt_ops = {
	.set = dma_inject_halt_set,
};
module_param_cb(dma_inject_halt, &dma_inject_halt_ops, NULL, 0644);
MODULE_PARM_DESC(dma_inject_halt, "write to halt the active DMA (force the wedge; test only)");
#endif /* CONFIG_SIMAAI_CAMERA_INSTRUMENTATION */

struct vdma_dev {
	struct device			*dev;
	struct v4l2_subdev		sd;
	struct v4l2_async_notifier	nf;
	unsigned int			id;
	struct vdma_channel		channels[VDMA_CHANNELS];
	struct media_pad		pads[VDMA_CHANNELS * 2];
};

struct vdma_asc {
	struct v4l2_async_connection	base;
	struct vdma_channel		*channel;
};

static inline struct vdma_asc *
asc_to_vdma(struct v4l2_async_connection *asc)
{
	return container_of(asc, struct vdma_asc, base);
}

static inline struct vdma_dev *
subdev_to_vdma_dev(struct v4l2_subdev *s)
{
	return container_of(s, struct vdma_dev, sd);
}

static inline struct vdma_buffer *to_vdma_buffer(struct vb2_v4l2_buffer *vb2)
{
	return container_of(vb2, struct vdma_buffer, vb);
}

struct vdma_channel *get_vdma_channel(struct v4l2_subdev *sd, unsigned int pad)
{
	struct vdma_dev *dev = subdev_to_vdma_dev(sd);

	if ((pad < VDMA_CHANNELS)  || (pad >= 2 * VDMA_CHANNELS)) {
		dev_err(dev->dev, "Wrong pad number to get channel: %d", pad);
		return NULL;
	}

	return &dev->channels[pad - VDMA_CHANNELS];
}

u64 get_vdma_channel_mask(struct vdma_channel *channel)
{
	return BIT(channel->id);
}

static int vdma_get_fwnode_pad(struct media_entity *entity,
			      struct fwnode_endpoint *endpoint)
{
	return endpoint->id * VDMA_CHANNELS + endpoint->port;
}

static bool vdma_has_pad_interdep(struct media_entity *entity, unsigned int pad0,
				 unsigned int pad1)
{
	return ((pad0 + VDMA_CHANNELS == pad1) || (pad1 + VDMA_CHANNELS == pad0));
}

static const struct media_entity_operations vdma_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = vdma_get_fwnode_pad,
	.has_pad_interdep = vdma_has_pad_interdep,
};

static int vdma_notify_bound(struct v4l2_async_notifier *nf,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *base_asc)
{
	struct vdma_asc *asc = asc_to_vdma(base_asc);
	struct vdma_channel *chan = asc->channel;
	struct vdma_dev *dev = chan->dev;
	int ret;

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  chan->ep_fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	chan->ep_sd = subdev;
	chan->ep_pad = ret;

	ret = media_create_pad_link(&chan->ep_sd->entity, chan->ep_pad,
				    &dev->sd.entity, chan->id,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev->dev, "Unable to link %s:%u -> %s:%u\n",
			chan->ep_sd->name, chan->ep_pad, dev->sd.name, chan->id);
		return ret;
	}

	return 0;
}

static void vdma_notify_unbind(struct v4l2_async_notifier *nf,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	struct vdma_asc *asc = asc_to_vdma(base_asc);

	asc->channel->ep_sd = NULL;
}

static const struct v4l2_async_notifier_operations vdma_notify_ops = {
	.bound = vdma_notify_bound,
	.unbind = vdma_notify_unbind,
};

static int vdma_v4l2_notifier_register(struct vdma_dev *dev)
{
	unsigned int i;
	int ret;

	v4l2_async_subdev_nf_init(&dev->nf, &dev->sd);

	for (i = 0; i < VDMA_CHANNELS; i++) {
		struct vdma_channel *chan = &dev->channels[i];
		struct vdma_asc *asc;

		if (!chan->enabled)
			continue;

		if (!chan->ep_fwnode)
			continue;

		asc = v4l2_async_nf_add_fwnode(&dev->nf, chan->ep_fwnode,
					       struct vdma_asc);
		if (IS_ERR(asc)) {
			dev_err(dev->dev,
				"Failed to add subdev for source %u: %pe", i,
				asc);

			v4l2_async_nf_cleanup(&dev->nf);
			return PTR_ERR(asc);
		}

		asc->channel = chan;
	}
	dev->nf.ops = &vdma_notify_ops;

	ret = v4l2_async_nf_register(&dev->nf);
	if (ret) {
		dev_err(dev->dev, "Failed to register subdev notifier");
		v4l2_async_nf_cleanup(&dev->nf);
		return ret;
	}

	return 0;
}

static void vdma_v4l2_notifier_unregister(struct vdma_dev *dev)
{
	v4l2_async_nf_unregister(&dev->nf);
	v4l2_async_nf_cleanup(&dev->nf);
}

static void vdma_frame_watchdog(struct timer_list *t)
{
	struct vdma_channel *chan = timer_container_of(chan, t, fwd_timer);

	if (!chan->streaming || !frame_watchdog_ms)
		return;

	chan->fwd_fires++;

	/* Finite budget exhausted (max_retry == 0 means infinite -> never here). */
	if (frame_watchdog_max_retry &&
	    chan->fwd_fires > frame_watchdog_max_retry) {
		if (chan->fwd_fires == frame_watchdog_max_retry + 1)
			dev_err(chan->dev->dev,
				"ch%u: frame watchdog: stalled after %u recovery attempts; giving up (restart the stream)\n",
				chan->id, frame_watchdog_max_retry);
		return;	/* stop re-arming; a completed frame re-arms via fill_buffer */
	}

	dev_warn(chan->dev->dev,
		 "ch%u: frame watchdog: no frame in %u ms, recovery attempt #%u%s\n",
		 chan->id, frame_watchdog_ms, chan->fwd_fires,
		 frame_watchdog_max_retry ? "" : " (infinite)");

	if (chan->ep_sd)
		dwc_csi_trigger_overflow_recovery(chan->ep_sd);

	mod_timer(&chan->fwd_timer, jiffies + msecs_to_jiffies(frame_watchdog_ms));
}

static int vdma_enable_channel(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct vdma_dev *dev = subdev_to_vdma_dev(sd);
	struct vdma_channel *chan;
	struct vdma_channel *remote;
	u32 other_pad;
	u32 other_stream;
	int ret = 0;

	if ((pad < VDMA_CHANNELS)  || (pad >= 2 * VDMA_CHANNELS)) {
		dev_err(dev->dev, "Wrong pad number to enable stream: %d", pad);
		return -EINVAL;
	}

	chan = &dev->channels[pad - VDMA_CHANNELS];
	mutex_lock(&chan->lock);

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing,
			pad, pad - VDMA_CHANNELS, &other_pad, &other_stream);
	if (ret) {
		dev_err(dev->dev, "Cannot find opposite end: %d", ret);
		goto unlock;
	}

	/* Reset before the DMA/source start so the first buffer is sequence 0. */
	chan->sequence = 0;

	/*
	 * Arm the per-channel scratch buffer before the DMA starts and before the
	 * sensor pushes, so the channel always has a destination: it loops on the
	 * scratch when no user buffer is queued, which lets user buffers complete to
	 * userspace immediately (one frame each). The scratch is required for
	 * continuous capture without holding user buffers — fail the stream if it
	 * cannot be set up.
	 */
	if (scratch_disable)
		dev_warn(dev->dev, "scratch DISABLED (test): DMA will run out of "
			 "descriptors -> mid-stream IPI overflow expected\n");
	if (!scratch_disable && chan->frame_size) {
		ret = dw_axi_dma_arm_scratch(chan->dma, chan->frame_size);
		if (ret) {
			dev_err(dev->dev, "scratch arm failed: %d", ret);
			goto unlock;
		}
	}

	/*
	 * Let the upstream CSI's IPI-overflow recovery (SOCSW-5392) un-stick this
	 * DMA channel: register the resume callback now that dma + scratch are up.
	 */
	if (chan->ep_sd)
		dwc_csi_register_overflow_recovery(chan->ep_sd, vdma_dma_quiesce,
						   vdma_dma_resume, chan->dma);

	WRITE_ONCE(g_inject_dma, chan->dma);	/* test injector target */

	dma_async_issue_pending(chan->dma);

	/* The upstream feeding this source pad is the routed sink channel. */
	remote = &dev->channels[other_pad];
	ret = v4l2_subdev_enable_streams(remote->ep_sd, remote->ep_pad,
					 BIT(other_stream));
	if (ret) {
		dev_err(dev->dev, "Failed to enable stream on EP subdevice: %d", ret);
		/* DMA already issued above; drain it before vb2 frees the buffers
		 * (else fill_buffer completes freed buffers -> use-after-free).
		 * terminate_sync also tears down the scratch/RELOAD state and frees
		 * the provider-owned scratch buffer. */
		dmaengine_terminate_sync(chan->dma);
		cmpxchg(&g_inject_dma, chan->dma, NULL);	/* retract only our own */
		if (chan->ep_sd)
			dwc_csi_register_overflow_recovery(chan->ep_sd, NULL,
							   NULL, NULL);
		goto unlock;
	}

	chan->streaming = true;

	/* arm the frame watchdog; fill_buffer re-arms it on every frame. */
	chan->fwd_fires = 0;
	if (frame_watchdog_ms)
		mod_timer(&chan->fwd_timer,
			  jiffies + msecs_to_jiffies(frame_watchdog_ms));

unlock:
	mutex_unlock(&chan->lock);

	return ret;
}

static int vdma_disable_channel(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct vdma_dev *dev = subdev_to_vdma_dev(sd);
	struct vdma_channel *chan;
	struct vdma_channel *remote;
	u32 other_pad;
	u32 other_stream;
	int ret = 0;

	if ((pad < VDMA_CHANNELS)  || (pad >= 2 * VDMA_CHANNELS)) {
		dev_err(dev->dev, "Wrong pad number to disable stream: %d", pad);
		return -EINVAL;
	}

	chan = &dev->channels[pad - VDMA_CHANNELS];
	mutex_lock(&chan->lock);

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing,
			pad, pad - VDMA_CHANNELS, &other_pad, &other_stream);
	if (ret) {
		dev_err(dev->dev, "Cannot find opposite end: %d", ret);
		goto unlock;
	}

	/* disarm the watchdog before teardown so it can't fire while frames stop. */
	chan->streaming = false;
	timer_delete_sync(&chan->fwd_timer);

	/* The upstream feeding this source pad is the routed sink channel. */
	remote = &dev->channels[other_pad];
	ret = v4l2_subdev_disable_streams(remote->ep_sd, remote->ep_pad,
					  BIT(other_stream));
	if (ret) {
		dev_err(dev->dev, "Cannot disable streams: %d", ret);
		goto unlock;
	}

	/* terminate_sync stops the DMA, tears down the scratch/RELOAD state and
	 * frees the provider-owned scratch buffer. */
	dmaengine_terminate_sync(chan->dma);

	cmpxchg(&g_inject_dma, chan->dma, NULL);	/* retract only our own */

	/* Stop the CSI from resuming a channel that is going away. */
	if (chan->ep_sd)
		dwc_csi_register_overflow_recovery(chan->ep_sd, NULL, NULL, NULL);

unlock:
	mutex_unlock(&chan->lock);
	return ret;
};

static int vdma_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *sdformat)
{
	struct vdma_dev *dev = subdev_to_vdma_dev(sd);
	struct v4l2_mbus_framefmt *sink_fmt, *src_fmt;
	struct vdma_channel *chan;

	/*
	 * The VDMA can't transcode in any way, the source format can't be
	 * modified.
	 */
	if (sdformat->pad >= VDMA_CHANNELS)
		return v4l2_subdev_get_fmt(sd, sd_state, sdformat);

	sink_fmt = v4l2_subdev_state_get_format(sd_state, sdformat->pad,
						sdformat->stream);
	if (!sink_fmt)
		return -EINVAL;
	*sink_fmt = sdformat->format;

	/* Propagate the format from sink stream to source stream */
	src_fmt = v4l2_subdev_state_get_opposite_stream_format(sd_state,
							       sdformat->pad,
							       sdformat->stream);
	if (!src_fmt)
		return -EINVAL;
	*src_fmt = sdformat->format;

	if (sdformat->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	chan = &dev->channels[sdformat->pad];
	chan->fmt = sdformat->format;

	if (chan->ep_sd) {
		struct v4l2_subdev_format up = *sdformat;
		struct v4l2_subdev_state *ust;
		int ret = -ENODEV;

		ust = v4l2_subdev_lock_and_get_active_state(chan->ep_sd);
		if (ust) {
			ret = v4l2_subdev_routing_find_opposite_end(&ust->routing,
					chan->ep_pad, sdformat->stream,
					&up.pad, &up.stream);
			v4l2_subdev_unlock_state(ust);
		}
		if (!ret)
			v4l2_subdev_call_state_active(chan->ep_sd, pad, set_fmt, &up);
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops vdma_pad_ops = {
	.enable_streams = vdma_enable_channel,
	.disable_streams = vdma_disable_channel,

	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = vdma_set_fmt,
};

static const struct v4l2_subdev_ops vdma_subdev_ops = {
	.pad = &vdma_pad_ops,
};

static void fill_buffer(void *param, const struct dmaengine_result *result)
{
	struct vdma_buffer *buf = (struct vdma_buffer *) param;
	struct vdma_channel *chan = buf->channel;
	enum vb2_buffer_state done = VB2_BUF_STATE_DONE;

	if (unlikely(result->result != DMA_TRANS_NOERROR))
		done = VB2_BUF_STATE_ERROR;

	/*
	 * The CSI/IPI appends a per-frame metadata trailer (excluded from the
	 * payload). When present, its status field flags a bad frame — report it
	 * as an error rather than handing torn data to userspace.
	 */
	if (done == VB2_BUF_STATE_DONE &&
	    buf->size >= VDMA_FRAME_METADATA_SIZE) {
		void *vaddr = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);

		if (vaddr) {
			const struct vdma_frame_meta *meta =
				vaddr + buf->size - VDMA_FRAME_METADATA_SIZE;

			if (le32_to_cpu(meta->magic) == VDMA_FRAME_META_MAGIC &&
			    le16_to_cpu(meta->status) == 0) {
				dev_dbg_ratelimited(chan->dev->dev,
					"ch%u: bad frame (seq %u) per metadata status\n",
					chan->id, le16_to_cpu(meta->sequence));
				done = VB2_BUF_STATE_ERROR;
			}
		}
	}

	buf->vb.field = chan->fmt.field;
	buf->vb.sequence = chan->sequence++;
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, buf->size - VDMA_FRAME_METADATA_SIZE);
	vb2_buffer_done(&buf->vb.vb2_buf, done);

	/* a frame completed -> pipeline is alive: clear escalation + re-arm. */
	if (frame_watchdog_ms && chan->streaming) {
		chan->fwd_fires = 0;
		mod_timer(&chan->fwd_timer,
			  jiffies + msecs_to_jiffies(frame_watchdog_ms));
	}
}

void vdma_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vdma_buffer *buf = to_vdma_buffer(vbuf);
	struct vdma_channel *chan = buf->channel;
	struct dma_async_tx_descriptor *desc;

	mutex_lock(&chan->lock);
	chan->xt.dst_start = vb2_dma_contig_plane_dma_addr(vb, 0);
	chan->xt.sgl[0].size = buf->size;
	/* Remember the live buffer size so stream-on can size the scratch buffer. */
	chan->frame_size = buf->size;

	desc = dmaengine_prep_interleaved_dma(chan->dma, &chan->xt,
		DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(chan->dev->dev, "Fail to prepare DMA transfer");
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		mutex_unlock(&chan->lock);
		return;
	}

	desc->callback_result = fill_buffer;
	desc->callback_param = buf;

	dmaengine_submit(desc);

	if (vb2_is_streaming(vb->vb2_queue) && chan->streaming)
		dma_async_issue_pending(chan->dma);
	mutex_unlock(&chan->lock);
}

#define VDMA_INIT_ROUTE(n) { \
			.sink_pad = n, \
			.sink_stream = n, \
			.source_pad = n + VDMA_CHANNELS, \
			.source_stream = n, \
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE, \
		}

static int vdma_init_routing(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state)
{
	int ret;

	struct v4l2_subdev_route routes[] = {
		VDMA_INIT_ROUTE(0),
		VDMA_INIT_ROUTE(1),
		VDMA_INIT_ROUTE(2),
		VDMA_INIT_ROUTE(3),
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	if (routing.num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -EINVAL;

	ret = v4l2_subdev_routing_validate(sd, &routing,
			V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	return v4l2_subdev_set_routing(sd, sd_state, &routing);
}

static const struct v4l2_subdev_internal_ops vdma_internal_ops = {
	.init_state = vdma_init_routing,
};

static int vdma_v4l2_register(struct vdma_dev *dev)
{
	struct v4l2_subdev *sd = &dev->sd;
	unsigned int num_pads = ARRAY_SIZE(dev->pads);
	unsigned int i;
	int ret;

	v4l2_subdev_init(sd, &vdma_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "%s.%d", "vdma", dev->id);
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &vdma_media_ops;
	sd->internal_ops = &vdma_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	sd->dev = dev->dev;

	for (i = 0; i < num_pads; i++) {
		if (i < VDMA_CHANNELS)
			dev->pads[i].flags = MEDIA_PAD_FL_SINK;
		else
			dev->pads[i].flags = MEDIA_PAD_FL_SOURCE;
	}

	v4l2_set_subdevdata(sd, dev);

	ret = media_entity_pads_init(&sd->entity, num_pads, dev->pads);
	if (ret)
		return ret;

	ret = vdma_v4l2_notifier_register(dev);
	if (ret)
		goto err_media_entity_cleanup;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_nf_cleanup;

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_sd_cleanup;

	return 0;

err_sd_cleanup:
	v4l2_subdev_cleanup(sd);
err_nf_cleanup:
	vdma_v4l2_notifier_unregister(dev);
err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);

	return ret;
}

static void vdma_v4l2_unregister(struct vdma_dev *dev)
{
	struct v4l2_subdev *sd = &dev->sd;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	vdma_v4l2_notifier_unregister(dev);
	media_entity_cleanup(&sd->entity);
}

static void vdma_parse_sink_dt_endpoint(struct vdma_dev *dev, struct vdma_channel *chan)
{
	struct fwnode_handle *fwnode = dev_fwnode(dev->dev);
	struct fwnode_handle *ep;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, chan->id, 0, 0);
	if (!ep) {
		dev_err(dev->dev, "Failed to get local endpoint on port %u\n", chan->id);
		return;
	}

	chan->ep_fwnode = fwnode_graph_get_remote_endpoint(ep);
	fwnode_handle_put(ep);

	if (!chan->ep_fwnode) {
		dev_err(dev->dev, "Failed to get remote endpoint on port %u\n", chan->id);
		return;
	}

	chan->enabled = true;
}

static int vdma_probe(struct platform_device *pdev)
{
	struct device *dev_ = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct vdma_dev *dev;
	unsigned int i;
	int ret = 0;

	dev = devm_kzalloc(dev_, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = dev_;
	platform_set_drvdata(pdev, dev);

	of_property_read_u32(np, "id", &dev->id);

	ret = dma_set_mask_and_coherent(dev->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev->dev, "failed to set DMA mask\n");
		return ret;
	}

	for (i = 0; i < VDMA_CHANNELS; ++i) {
		struct vdma_channel *chan = &dev->channels[i];

		chan->id = i;
		chan->dev = dev;
		snprintf(chan->name, sizeof(chan->name), "channel-%d", i);
		chan->xt.dir = DMA_DEV_TO_MEM;
		chan->xt.src_sgl = false;
		chan->xt.dst_inc = false;
		chan->xt.dst_sgl = true;
		chan->xt.frame_size = 1;
		chan->xt.sgl[0].icg = 0;
		chan->xt.numf = 1;

		chan->dma = dma_request_chan(dev->dev, chan->name);
		if (IS_ERR(chan->dma)) {
			ret = PTR_ERR(chan->dma);
			chan->dma = NULL;
			dev_err(dev->dev, "Failed to initialize DMA channel %d: %d\n", i, ret);
			goto err_exit;
		}
		vdma_parse_sink_dt_endpoint(dev, chan);
		mutex_init(&chan->lock);
		timer_setup(&chan->fwd_timer, vdma_frame_watchdog, 0);
	}

	ret = vdma_v4l2_register(dev);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to initialize V4L2: %d\n", ret);
		goto err_exit;
	}

	dev_info(dev->dev, "Successfully initialized\n");

	return 0;

err_exit:
	for (i = 0; i < VDMA_CHANNELS; ++i) {
		if (dev->channels[i].dma)
			dma_release_channel(dev->channels[i].dma);
	}

	dev_err(dev->dev, "probe unsuccesfull\n");
	return ret;
}

static void vdma_remove(struct platform_device *pdev)
{
	struct vdma_dev *dev = platform_get_drvdata(pdev);
	unsigned int i;


	for (i = 0; i < VDMA_CHANNELS; ++i) {
		if (dev->channels[i].dma) {
			dmaengine_terminate_sync(dev->channels[i].dma);
			dma_release_channel(dev->channels[i].dma);
		}
		if (dev->channels[i].ep_fwnode)
			fwnode_handle_put(dev->channels[i].ep_fwnode);
	}

	vdma_v4l2_unregister(dev);
}

static const struct of_device_id vdma_of_match[] = {
	{ .compatible = "simaai,v4l2-vdma" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vdma_of_match);

static struct platform_driver simaai_v4l2_vdma = {
	.probe		= vdma_probe,
	.remove		= vdma_remove,
	.driver = {
		.of_match_table = vdma_of_match,
		.name		= "simaai-v4l2-vdma",
	}
};
module_platform_driver(simaai_v4l2_vdma);

MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai");
MODULE_DESCRIPTION("V4L2 driver for SiMa.ai vision DMA");
MODULE_LICENSE("GPL");
