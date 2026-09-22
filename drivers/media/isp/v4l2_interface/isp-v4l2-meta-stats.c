/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2026 SiMa Technologies, Inc.
 *
 * V4L2 META_CAPTURE stats device — V4L2 device registration.
 *
 * Phase 1b-3 commit: wraps the vb2_queue from 1b-2 in a video_device,
 * adds file_operations + v4l2_ioctl_ops for META_CAPTURE format
 * handling, and registers the device. After this commit user-space
 * can open the resulting /dev/videoN, call VIDIOC_REQBUFS / QBUF /
 * DQBUF — but no stats actually flow yet (the BH producer hook is
 * 1b-6). Per-context wiring that calls this init from the module
 * main paths is 1b-4.
 */

#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/list.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include <linux/media/simaai/modalix_isp_stats_buffer.h>
#include <linux/media/simaai/modalix_isp_v4l2.h>

#include "acamera_configuration.h"
#include "acamera_fsmgr_general_router.h"  /* for sizeof(acamera_cmd_sensor_info) */
#include "acamera_ctrl_channel.h"          /* modalix_isp_install_user_calibrations */
#include "isp-v4l2-meta-stats.h"
#include "acamera_logger.h"
#include "fw-interface.h"

/* Per-context lookup table populated by modalix_meta_stats_init() so
 * the kernel BH can find the publish target without depending on
 * isp-v4l2's per-context state. Reads are lock-less (atomic word-
 * sized pointer load); the BH never races init/exit because init
 * runs at module probe and exit at module remove. */
static struct modalix_meta_stats_dev *g_meta_devs[FIRMWARE_CONTEXT_NUMBER];

#define MODALIX_META_STATS_REQ_BUFS_MIN 2u
#define MODALIX_META_STATS_REQ_BUFS_MAX 8u

/* V4L2 meta pixel format for our stats payload. v4l2_fourcc('M','S',
 * 'T','S') = "modalix stats" — unique to this driver. Vendor-namespace
 * lives in <linux/media/simaai/modalix_isp_v4l2.h>; we don't reuse
 * V4L2_PIX_FMT_MODALIX_META there because that one is the generic
 * "opaque metadata" fourcc; this one is specifically tagged for the
 * 3A stats payload so user-space can tell the two apart on a system
 * that exposes both metadata paths. */
#define V4L2_META_FMT_MODALIX_ISP_STATS  v4l2_fourcc('M','S','T','S')

struct modalix_meta_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head       node;
};

struct modalix_meta_stats_dev {
	uint32_t              ctx_id;

	struct video_device   vdev;        /* exposes the queue to /dev/videoN */
	struct vb2_queue      q;

	struct mutex          queue_lock;
	spinlock_t            buf_lock;
	struct list_head      pending;

	struct v4l2_format    fmt;         /* current META format (g/s_fmt cache) */

	/* V4L2 controls exposing the kernel kf_info data the legacy
	 * fw_sbuf mmap used to surface. The IPA reads these via
	 * VIDIOC_G_EXT_CTRLS on this same fd. */
	struct v4l2_ctrl_handler ctrl_hdl;
};

/* ---------- vb2 callbacks (unchanged from 1b-2) ---------- */

static int modalix_meta_queue_setup( struct vb2_queue *vq,
                                     unsigned int *num_buffers,
                                     unsigned int *num_planes,
                                     unsigned int sizes[],
                                     struct device *alloc_devs[] )
{
	(void)alloc_devs;
	(void)vq;

	*num_planes  = 1;
	*num_buffers = clamp_t( unsigned int, *num_buffers,
	                        MODALIX_META_STATS_REQ_BUFS_MIN,
	                        MODALIX_META_STATS_REQ_BUFS_MAX );
	sizes[0] = sizeof( struct modalix_isp_stats_buffer );
	return 0;
}

static int modalix_meta_buf_prepare( struct vb2_buffer *vb )
{
	if ( vb2_plane_size( vb, 0 ) < sizeof( struct modalix_isp_stats_buffer ) ) {
		return -EINVAL;
	}
	vb2_set_plane_payload( vb, 0, sizeof( struct modalix_isp_stats_buffer ) );
	return 0;
}

static void modalix_meta_buf_queue( struct vb2_buffer *vb )
{
	struct vb2_v4l2_buffer        *vbuf = to_vb2_v4l2_buffer( vb );
	struct modalix_meta_buffer    *buf  = container_of( vbuf, struct modalix_meta_buffer, vb );
	struct vb2_queue              *vq   = vb->vb2_queue;
	struct modalix_meta_stats_dev *dev  = vb2_get_drv_priv( vq );
	unsigned long                  flags;

	spin_lock_irqsave( &dev->buf_lock, flags );
	list_add_tail( &buf->node, &dev->pending );
	spin_unlock_irqrestore( &dev->buf_lock, flags );
}

static void modalix_meta_stop_streaming( struct vb2_queue *vq )
{
	struct modalix_meta_stats_dev *dev = vb2_get_drv_priv( vq );
	struct modalix_meta_buffer    *buf;
	unsigned long                  flags;

	spin_lock_irqsave( &dev->buf_lock, flags );
	while ( !list_empty( &dev->pending ) ) {
		buf = list_first_entry( &dev->pending, struct modalix_meta_buffer, node );
		list_del( &buf->node );
		vb2_buffer_done( &buf->vb.vb2_buf, VB2_BUF_STATE_ERROR );
	}
	spin_unlock_irqrestore( &dev->buf_lock, flags );
}

static const struct vb2_ops modalix_meta_vb2_ops = {
	.queue_setup    = modalix_meta_queue_setup,
	.buf_prepare    = modalix_meta_buf_prepare,
	.buf_queue      = modalix_meta_buf_queue,
	.stop_streaming = modalix_meta_stop_streaming,
};

/* ---------- V4L2 ioctl callbacks ---------- */

static int modalix_meta_querycap( struct file *file, void *priv,
                                  struct v4l2_capability *cap )
{
	struct video_device *vdev = video_devdata( file );

	strscpy( cap->driver,   "modalix-isp",       sizeof( cap->driver ) );
	strscpy( cap->card,     vdev->name,           sizeof( cap->card ) );
	strscpy( cap->bus_info, "platform:simaai-isp", sizeof( cap->bus_info ) );

	return 0;
}

static int modalix_meta_enum_fmt_meta_cap( struct file *file, void *priv,
                                           struct v4l2_fmtdesc *f )
{
	struct video_device *vdev = video_devdata( file );

	if ( f->index > 0 || f->type != vdev->queue->type ) {
		return -EINVAL;
	}
	f->pixelformat = V4L2_META_FMT_MODALIX_ISP_STATS;
	return 0;
}

static int modalix_meta_g_fmt_meta_cap( struct file *file, void *priv,
                                        struct v4l2_format *f )
{
	struct video_device           *vdev = video_devdata( file );
	struct modalix_meta_stats_dev *dev  = video_get_drvdata( vdev );

	if ( f->type != vdev->queue->type ) {
		return -EINVAL;
	}

	/* Single fixed format. s_fmt is treated as a g_fmt (read-only)
	 * because the layout is dictated by the kernel BH producer, not
	 * negotiable. */
	memset( &f->fmt.meta, 0, sizeof( f->fmt.meta ) );
	f->fmt.meta.dataformat = V4L2_META_FMT_MODALIX_ISP_STATS;
	f->fmt.meta.buffersize = sizeof( struct modalix_isp_stats_buffer );
	dev->fmt = *f;
	return 0;
}

static const struct v4l2_ioctl_ops modalix_meta_ioctl_ops = {
	.vidioc_querycap          = modalix_meta_querycap,
	.vidioc_enum_fmt_meta_cap = modalix_meta_enum_fmt_meta_cap,
	.vidioc_g_fmt_meta_cap    = modalix_meta_g_fmt_meta_cap,
	.vidioc_s_fmt_meta_cap    = modalix_meta_g_fmt_meta_cap,
	.vidioc_try_fmt_meta_cap  = modalix_meta_g_fmt_meta_cap,

	.vidioc_reqbufs           = vb2_ioctl_reqbufs,
	.vidioc_querybuf          = vb2_ioctl_querybuf,
	.vidioc_create_bufs       = vb2_ioctl_create_bufs,
	.vidioc_qbuf              = vb2_ioctl_qbuf,
	.vidioc_dqbuf             = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf       = vb2_ioctl_prepare_buf,
	.vidioc_expbuf            = vb2_ioctl_expbuf,
	.vidioc_streamon          = vb2_ioctl_streamon,
	.vidioc_streamoff         = vb2_ioctl_streamoff,

	.vidioc_subscribe_event   = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations modalix_meta_fops = {
	.owner          = THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.poll           = vb2_fop_poll,
	.mmap           = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

/* ---------- V4L2 controls: kf_info readers ---------- */

static int modalix_meta_stats_g_volatile_ctrl( struct v4l2_ctrl *ctrl )
{
	struct modalix_meta_stats_dev *dev =
		container_of( ctrl->handler, struct modalix_meta_stats_dev, ctrl_hdl );
	int v = 0;
	int rc;

	switch ( ctrl->id ) {
	case MODALIX_ISP_V4L2_CID_SENSOR_INFO_BLOB:
		return fw_intf_get_sensor_info_blob( dev->ctx_id,
		                                     ctrl->p_new.p_u8,
		                                     ctrl->elems );
	case MODALIX_ISP_V4L2_CID_CMOS_MAX_EXPOSURE_LOG2:
		rc = fw_intf_get_cmos_max_exposure_log2( dev->ctx_id, &v );
		ctrl->val = v;
		return rc;
	case MODALIX_ISP_V4L2_CID_CMOS_AGAIN_LOG2:
		rc = fw_intf_get_cmos_again_log2( dev->ctx_id, &v );
		ctrl->val = v;
		return rc;
	case MODALIX_ISP_V4L2_CID_CMOS_DGAIN_LOG2:
		rc = fw_intf_get_cmos_dgain_log2( dev->ctx_id, &v );
		ctrl->val = v;
		return rc;
	case MODALIX_ISP_V4L2_CID_ISP_BYPASS_CONFIG: {
		uint32_t word = 0;
		rc = modalix_isp_read_pipeline_bypass( dev->ctx_id, &word );
		ctrl->val = (int32_t)word;
		return rc;
	}
	}
	return -EINVAL;
}

/* Write path: the IPA pushes a calibration blob via VIDIOC_S_EXT_CTRLS
 * on MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB. The framework copies the
 * payload from user-space and validates length against the ctrl's
 * dynamic-array bound (MODALIX_ISP_V4L2_CALIBRATION_BLOB_MAX) before
 * we see it here. We just hand the buffer to the calib bridge. */
static int modalix_meta_stats_s_ctrl( struct v4l2_ctrl *ctrl )
{
	struct modalix_meta_stats_dev *dev =
		container_of( ctrl->handler, struct modalix_meta_stats_dev, ctrl_hdl );

	switch ( ctrl->id ) {
	case MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB:
		return modalix_isp_install_user_calibrations( dev->ctx_id,
		                                              ctrl->p_new.p_u8,
		                                              ctrl->new_elems );
	case MODALIX_ISP_V4L2_CID_ISP_BYPASS_CONFIG:
		return modalix_isp_apply_pipeline_bypass( dev->ctx_id,
		                                          (uint32_t)ctrl->val );
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops modalix_meta_stats_ctrl_ops = {
	.g_volatile_ctrl = modalix_meta_stats_g_volatile_ctrl,
	.s_ctrl          = modalix_meta_stats_s_ctrl,
};

static const struct v4l2_ctrl_config modalix_meta_stats_ctrl_sensor_info_blob = {
	.ops      = &modalix_meta_stats_ctrl_ops,
	.id       = MODALIX_ISP_V4L2_CID_SENSOR_INFO_BLOB,
	.name     = "Modalix ISP sensor info blob",
	.type     = V4L2_CTRL_TYPE_U8,
	.flags    = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY |
	            V4L2_CTRL_FLAG_HAS_PAYLOAD,
	.min      = 0,
	.max      = 0xff,
	.step     = 1,
	.def      = 0,
	.dims     = { sizeof( acamera_cmd_sensor_info ) },
};

static const struct v4l2_ctrl_config modalix_meta_stats_ctrl_cmos_max_exposure_log2 = {
	.ops   = &modalix_meta_stats_ctrl_ops,
	.id    = MODALIX_ISP_V4L2_CID_CMOS_MAX_EXPOSURE_LOG2,
	.name  = "Modalix ISP cmos max exposure log2",
	.type  = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	/* Cap the announced range well inside int32 to dodge V4L2's
	 * range-validation quirks; the actual values for log2 gains
	 * always sit within ±1<<30 with plenty of headroom. */
	.min   = -(1 << 30),
	.max   = (1 << 30),
	.step  = 1,
	.def   = 0,
};

static const struct v4l2_ctrl_config modalix_meta_stats_ctrl_cmos_again_log2 = {
	.ops   = &modalix_meta_stats_ctrl_ops,
	.id    = MODALIX_ISP_V4L2_CID_CMOS_AGAIN_LOG2,
	.name  = "Modalix ISP cmos analog gain log2",
	.type  = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	/* Cap the announced range well inside int32 to dodge V4L2's
	 * range-validation quirks; the actual values for log2 gains
	 * always sit within ±1<<30 with plenty of headroom. */
	.min   = -(1 << 30),
	.max   = (1 << 30),
	.step  = 1,
	.def   = 0,
};

static const struct v4l2_ctrl_config modalix_meta_stats_ctrl_cmos_dgain_log2 = {
	.ops   = &modalix_meta_stats_ctrl_ops,
	.id    = MODALIX_ISP_V4L2_CID_CMOS_DGAIN_LOG2,
	.name  = "Modalix ISP cmos digital gain log2",
	.type  = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	/* Cap the announced range well inside int32 to dodge V4L2's
	 * range-validation quirks; the actual values for log2 gains
	 * always sit within ±1<<30 with plenty of headroom. */
	.min   = -(1 << 30),
	.max   = (1 << 30),
	.step  = 1,
	.def   = 0,
};

/* IPA → kernel calibration blob push. Declared as a U8 DYNAMIC_ARRAY so
 * variable-size payloads (one calibration blob per sensor, ~30–60 KiB
 * typically, never above MODALIX_ISP_V4L2_CALIBRATION_BLOB_MAX) flow
 * through VIDIOC_S_EXT_CTRLS without us having to negotiate the size
 * up-front. WRITE_ONLY + EXECUTE_ON_WRITE: the ctrl is one-shot —
 * userspace doesn't read it back, and every S_EXT_CTRLS dispatches
 * s_ctrl synchronously so by the time the ioctl returns the calib
 * bridge slot is live. */
static const struct v4l2_ctrl_config modalix_meta_stats_ctrl_calibration_blob = {
	.ops      = &modalix_meta_stats_ctrl_ops,
	.id       = MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB,
	.name     = "Modalix ISP calibration blob",
	.type     = V4L2_CTRL_TYPE_U8,
	.flags    = V4L2_CTRL_FLAG_WRITE_ONLY | V4L2_CTRL_FLAG_HAS_PAYLOAD |
	            V4L2_CTRL_FLAG_DYNAMIC_ARRAY | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	.min      = 0,
	.max      = 0xff,
	.step     = 1,
	.def      = 0,
	.dims     = { MODALIX_ISP_V4L2_CALIBRATION_BLOB_MAX },
};

/* IPA → kernel ISP pipeline bypass word (Mali reg 0xE040). Full s32
 * range so any bit pattern (incl. bit 31) round-trips. VOLATILE so
 * G_EXT_CTRLS reads the live register; EXECUTE_ON_WRITE so every
 * S_EXT_CTRLS applies even when the value is unchanged. */
static const struct v4l2_ctrl_config modalix_meta_stats_ctrl_isp_bypass = {
	.ops   = &modalix_meta_stats_ctrl_ops,
	.id    = MODALIX_ISP_V4L2_CID_ISP_BYPASS_CONFIG,
	.name  = "Modalix ISP pipeline bypass",
	.type  = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	.min   = S32_MIN,
	.max   = S32_MAX,
	.step  = 1,
	.def   = 0,
};

static int modalix_meta_stats_init_ctrls( struct modalix_meta_stats_dev *dev )
{
	struct v4l2_ctrl *c;
	int               rc;

	rc = v4l2_ctrl_handler_init( &dev->ctrl_hdl, 6 );
	if ( rc ) {
		return rc;
	}

	c = v4l2_ctrl_new_custom( &dev->ctrl_hdl,
	                          &modalix_meta_stats_ctrl_sensor_info_blob, NULL );
	if ( !c ) { goto fail; }
	c = v4l2_ctrl_new_custom( &dev->ctrl_hdl,
	                          &modalix_meta_stats_ctrl_cmos_max_exposure_log2, NULL );
	if ( !c ) { goto fail; }
	c = v4l2_ctrl_new_custom( &dev->ctrl_hdl,
	                          &modalix_meta_stats_ctrl_cmos_again_log2, NULL );
	if ( !c ) { goto fail; }
	c = v4l2_ctrl_new_custom( &dev->ctrl_hdl,
	                          &modalix_meta_stats_ctrl_cmos_dgain_log2, NULL );
	if ( !c ) { goto fail; }
	c = v4l2_ctrl_new_custom( &dev->ctrl_hdl,
	                          &modalix_meta_stats_ctrl_calibration_blob, NULL );
	if ( !c ) { goto fail; }
	c = v4l2_ctrl_new_custom( &dev->ctrl_hdl,
	                          &modalix_meta_stats_ctrl_isp_bypass, NULL );
	if ( !c ) { goto fail; }

	if ( dev->ctrl_hdl.error ) {
		rc = dev->ctrl_hdl.error;
		goto fail;
	}

	dev->vdev.ctrl_handler = &dev->ctrl_hdl;
	return 0;

fail:
	v4l2_ctrl_handler_free( &dev->ctrl_hdl );
	return rc < 0 ? rc : -EINVAL;
}

/* ---------- init / exit ---------- */

static int modalix_meta_stats_init_queue( struct modalix_meta_stats_dev *dev )
{
	struct vb2_queue *q = &dev->q;

	q->type              = V4L2_BUF_TYPE_META_CAPTURE;
	q->io_modes          = VB2_MMAP | VB2_READ;
	q->drv_priv          = dev;
	q->ops               = &modalix_meta_vb2_ops;
	q->mem_ops           = &vb2_vmalloc_memops;
	q->buf_struct_size   = sizeof( struct modalix_meta_buffer );
	q->timestamp_flags   = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock              = &dev->queue_lock;

	return vb2_queue_init( q );
}

int modalix_meta_stats_init( uint32_t ctx_id,
                             struct v4l2_device *v4l2_dev,
                             struct modalix_meta_stats_dev **out )
{
	struct modalix_meta_stats_dev *dev;
	struct video_device           *vdev;
	int                            rc;

	if ( !v4l2_dev || !out ) {
		return -EINVAL;
	}

	dev = kzalloc( sizeof( *dev ), GFP_KERNEL );
	if ( !dev ) {
		return -ENOMEM;
	}

	dev->ctx_id = ctx_id;
	mutex_init( &dev->queue_lock );
	spin_lock_init( &dev->buf_lock );
	INIT_LIST_HEAD( &dev->pending );

	rc = modalix_meta_stats_init_queue( dev );
	if ( rc ) {
		mutex_destroy( &dev->queue_lock );
		kfree( dev );
		return rc;
	}

	rc = modalix_meta_stats_init_ctrls( dev );
	if ( rc ) {
		LOG( LOG_ERR, "ctx %u: ctrls init failed, rc=%d", ctx_id, rc );
		vb2_queue_release( &dev->q );
		mutex_destroy( &dev->queue_lock );
		kfree( dev );
		return rc;
	}

	vdev = &dev->vdev;
	snprintf( vdev->name, sizeof( vdev->name ),
	          "modalix-isp-stats-ctx%u", ctx_id );
	vdev->v4l2_dev    = v4l2_dev;
	vdev->fops        = &modalix_meta_fops;
	vdev->ioctl_ops   = &modalix_meta_ioctl_ops;
	vdev->release     = video_device_release_empty;
	vdev->lock        = &dev->queue_lock;
	vdev->queue       = &dev->q;
	vdev->device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING;
	vdev->vfl_dir     = VFL_DIR_RX;
	video_set_drvdata( vdev, dev );

	rc = video_register_device( vdev, VFL_TYPE_VIDEO, -1 );
	if ( rc ) {
		LOG( LOG_ERR, "ctx %u: video_register_device failed, rc=%d",
		     ctx_id, rc );
		v4l2_ctrl_handler_free( &dev->ctrl_hdl );
		vb2_queue_release( &dev->q );
		mutex_destroy( &dev->queue_lock );
		kfree( dev );
		return rc;
	}

	if ( ctx_id < FIRMWARE_CONTEXT_NUMBER ) {
		g_meta_devs[ctx_id] = dev;
	}
	*out = dev;
	LOG( LOG_INFO, "ctx %u: /dev/video%d registered (%s)",
	     ctx_id, vdev->num, vdev->name );
	return 0;
}

void modalix_meta_stats_exit( struct modalix_meta_stats_dev *dev )
{
	if ( !dev ) {
		return;
	}

	if ( dev->ctx_id < FIRMWARE_CONTEXT_NUMBER &&
	     g_meta_devs[dev->ctx_id] == dev ) {
		g_meta_devs[dev->ctx_id] = NULL;
	}

	video_unregister_device( &dev->vdev );
	v4l2_ctrl_handler_free( &dev->ctrl_hdl );
	vb2_queue_release( &dev->q );
	mutex_destroy( &dev->queue_lock );
	LOG( LOG_INFO, "meta-stats ctx %u released", dev->ctx_id );
	kfree( dev );
}

struct modalix_meta_stats_dev *modalix_meta_stats_lookup( uint32_t ctx_id )
{
	if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
		return NULL;
	}
	return g_meta_devs[ctx_id];
}

void modalix_meta_stats_publish_ae( uint32_t        ctx_id,
                                    const uint32_t *histogram,
                                    uint32_t        sum )
{
	struct modalix_meta_stats_dev   *dev;
	struct modalix_isp_stats_buffer *p;

	dev = modalix_meta_stats_lookup( ctx_id );
	if ( !dev || !histogram ) {
		return;
	}

	p = kzalloc( sizeof( *p ), GFP_KERNEL );
	if ( !p ) {
		return;
	}

	p->version          = MODALIX_ISP_STATS_BUFFER_VERSION_V1;
	p->valid_mask       = MODALIX_ISP_STATS_VALID_AE;
	p->timestamp_ns     = ktime_get_ns();
	memcpy( p->ae.histogram, histogram, sizeof( p->ae.histogram ) );
	p->ae.histogram_sum = sum;

	(void)modalix_meta_stats_publish( dev, p );
	kfree( p );
}

void modalix_meta_stats_publish_awb( uint32_t    ctx_id,
                                     const void *zones,
                                     uint32_t    zone_count )
{
	struct modalix_meta_stats_dev   *dev;
	struct modalix_isp_stats_buffer *p;

	dev = modalix_meta_stats_lookup( ctx_id );
	if ( !dev || !zones ) {
		return;
	}
	if ( zone_count > MODALIX_ISP_AWB_MAX_ZONES ) {
		zone_count = MODALIX_ISP_AWB_MAX_ZONES;
	}

	p = kzalloc( sizeof( *p ), GFP_KERNEL );
	if ( !p ) {
		return;
	}

	p->version        = MODALIX_ISP_STATS_BUFFER_VERSION_V1;
	p->valid_mask     = MODALIX_ISP_STATS_VALID_AWB;
	p->timestamp_ns   = ktime_get_ns();
	memcpy( p->awb.zones, zones,
	        zone_count * sizeof( struct modalix_isp_awb_zone ) );
	p->awb.zone_count = zone_count;

	(void)modalix_meta_stats_publish( dev, p );
	kfree( p );
}

int modalix_meta_stats_publish( struct modalix_meta_stats_dev *dev,
                                const struct modalix_isp_stats_buffer *payload )
{
	struct modalix_meta_buffer *buf;
	void                       *vaddr;
	unsigned long               flags;

	if ( !dev || !payload ) {
		return -EINVAL;
	}

	/* Pop the next caller-queued buffer under the BH-safe spinlock.
	 * If user-space hasn't queued one yet, or the queue isn't
	 * streaming, the list is empty and we drop the frame's stats —
	 * same semantics as the kfifo-overflow path on the IRQ
	 * staging ring. */
	spin_lock_irqsave( &dev->buf_lock, flags );
	if ( list_empty( &dev->pending ) ) {
		spin_unlock_irqrestore( &dev->buf_lock, flags );
		return -ENODATA;
	}
	buf = list_first_entry( &dev->pending, struct modalix_meta_buffer, node );
	list_del( &buf->node );
	spin_unlock_irqrestore( &dev->buf_lock, flags );

	/* vb2's vmalloc memops gives us a linear kernel mapping of the
	 * buffer's payload; memcpy is safe and cheap (~10 KB at frame
	 * rate is a rounding error on the 33 ms frame budget). */
	vaddr = vb2_plane_vaddr( &buf->vb.vb2_buf, 0 );
	if ( vaddr ) {
		memcpy( vaddr, payload, sizeof( *payload ) );
	}

	/* Publish to user-space. vb2_buffer_done fires the poll() /
	 * DQBUF wakeup and transitions buffer state for refcounting. */
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence          = payload->frame_id;
	vb2_buffer_done( &buf->vb.vb2_buf, VB2_BUF_STATE_DONE );

	return 0;
}
