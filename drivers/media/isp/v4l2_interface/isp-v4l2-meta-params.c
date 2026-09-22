/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2026 SiMa Technologies, Inc.
 *
 * V4L2 META_OUTPUT params device — symmetric counterpart of the
 * META_CAPTURE stats device in isp-v4l2-meta-stats.c. User-space
 * QBUFs a populated struct modalix_isp_params_buffer; the kernel
 * apply path pulls it back through modalix_meta_params_consume().
 */

#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/workqueue.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include <linux/media/simaai/modalix_isp_params_buffer.h>

#include "acamera_command_api.h"
#include "acamera_configuration.h"
#include "acamera_fsmgr_general_router.h"
#include "acamera_isp_ctx.h"
#include "acamera_logger.h"

#if defined( ISP_HAS_COLOR_MATRIX_FSM )
#include "color_matrix_fsm.h"
#endif
#include "isp-v4l2-meta-params.h"

extern void *get_ctx_ptr_by_id( uint32_t ctx_id );

/* Per-context lookup table — same shape and lifetime guarantees as
 * the meta-stats g_meta_devs array. */
static struct modalix_meta_params_dev *g_params_devs[FIRMWARE_CONTEXT_NUMBER];

#define MODALIX_META_PARAMS_REQ_BUFS_MIN 2u
#define MODALIX_META_PARAMS_REQ_BUFS_MAX 8u

/* fourcc for the params payload. "MPRM" = "modalix params". Distinct
 * from V4L2_META_FMT_MODALIX_ISP_STATS ("MSTS") so user-space can
 * disambiguate the two metadata endpoints by format alone. */
#define V4L2_META_FMT_MODALIX_ISP_PARAMS  v4l2_fourcc('M','P','R','M')

struct modalix_meta_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head       node;
};

struct modalix_meta_params_dev {
	uint32_t              ctx_id;

	struct video_device   vdev;
	struct vb2_queue      q;

	struct mutex          queue_lock;
	spinlock_t            buf_lock;
	struct list_head      pending;     /* FIFO of user-queued buffers */

	struct v4l2_format    fmt;

	/* Apply worker. buf_queue runs under the vb2 q->lock; deferring
	 * the apply to a workqueue keeps that lock free of FSM-event /
	 * context-param updates (which take their own mutexes). */
	struct work_struct    apply_work;
};

/* ---------- vb2 callbacks ---------- */

static int modalix_params_queue_setup( struct vb2_queue *vq,
                                       unsigned int *num_buffers,
                                       unsigned int *num_planes,
                                       unsigned int sizes[],
                                       struct device *alloc_devs[] )
{
	(void)alloc_devs;
	(void)vq;

	*num_planes  = 1;
	*num_buffers = clamp_t( unsigned int, *num_buffers,
	                        MODALIX_META_PARAMS_REQ_BUFS_MIN,
	                        MODALIX_META_PARAMS_REQ_BUFS_MAX );
	sizes[0] = sizeof( struct modalix_isp_params_buffer );
	return 0;
}

static int modalix_params_buf_prepare( struct vb2_buffer *vb )
{
	if ( vb2_plane_size( vb, 0 ) < sizeof( struct modalix_isp_params_buffer ) ) {
		return -EINVAL;
	}
	/* For OUTPUT the caller (user-space) sets bytesused via QBUF; we
	 * just sanity-check it covers the header at minimum. */
	if ( vb2_get_plane_payload( vb, 0 ) < sizeof( struct modalix_isp_params_buffer ) ) {
		vb2_set_plane_payload( vb, 0, sizeof( struct modalix_isp_params_buffer ) );
	}
	return 0;
}

static void modalix_params_apply_work( struct work_struct *w )
{
	struct modalix_meta_params_dev *dev =
		container_of( w, struct modalix_meta_params_dev, apply_work );

	(void)isp_apply_v4l2_params_drain( dev->ctx_id );
}

static void modalix_params_buf_queue( struct vb2_buffer *vb )
{
	struct vb2_v4l2_buffer         *vbuf = to_vb2_v4l2_buffer( vb );
	struct modalix_meta_buffer     *buf  = container_of( vbuf, struct modalix_meta_buffer, vb );
	struct vb2_queue               *vq   = vb->vb2_queue;
	struct modalix_meta_params_dev *dev  = vb2_get_drv_priv( vq );
	unsigned long                   flags;

	spin_lock_irqsave( &dev->buf_lock, flags );
	list_add_tail( &buf->node, &dev->pending );
	spin_unlock_irqrestore( &dev->buf_lock, flags );

	/* Kick the apply worker — it will consume this and any other
	 * pending buffers via isp_apply_v4l2_params_drain. */
	schedule_work( &dev->apply_work );
}

static void modalix_params_stop_streaming( struct vb2_queue *vq )
{
	struct modalix_meta_params_dev *dev = vb2_get_drv_priv( vq );
	struct modalix_meta_buffer     *buf;
	unsigned long                   flags;

	spin_lock_irqsave( &dev->buf_lock, flags );
	while ( !list_empty( &dev->pending ) ) {
		buf = list_first_entry( &dev->pending, struct modalix_meta_buffer, node );
		list_del( &buf->node );
		vb2_buffer_done( &buf->vb.vb2_buf, VB2_BUF_STATE_ERROR );
	}
	spin_unlock_irqrestore( &dev->buf_lock, flags );
}

static const struct vb2_ops modalix_params_vb2_ops = {
	.queue_setup    = modalix_params_queue_setup,
	.buf_prepare    = modalix_params_buf_prepare,
	.buf_queue      = modalix_params_buf_queue,
	.stop_streaming = modalix_params_stop_streaming,
};

/* ---------- V4L2 ioctl callbacks ---------- */

static int modalix_params_querycap( struct file *file, void *priv,
                                    struct v4l2_capability *cap )
{
	struct video_device *vdev = video_devdata( file );

	strscpy( cap->driver,   "modalix-isp",        sizeof( cap->driver ) );
	strscpy( cap->card,     vdev->name,            sizeof( cap->card ) );
	strscpy( cap->bus_info, "platform:simaai-isp", sizeof( cap->bus_info ) );
	return 0;
}

static int modalix_params_enum_fmt_meta_out( struct file *file, void *priv,
                                             struct v4l2_fmtdesc *f )
{
	struct video_device *vdev = video_devdata( file );

	if ( f->index > 0 || f->type != vdev->queue->type ) {
		return -EINVAL;
	}
	f->pixelformat = V4L2_META_FMT_MODALIX_ISP_PARAMS;
	return 0;
}

static int modalix_params_g_fmt_meta_out( struct file *file, void *priv,
                                          struct v4l2_format *f )
{
	struct video_device            *vdev = video_devdata( file );
	struct modalix_meta_params_dev *dev  = video_get_drvdata( vdev );

	if ( f->type != vdev->queue->type ) {
		return -EINVAL;
	}
	memset( &f->fmt.meta, 0, sizeof( f->fmt.meta ) );
	f->fmt.meta.dataformat = V4L2_META_FMT_MODALIX_ISP_PARAMS;
	f->fmt.meta.buffersize = sizeof( struct modalix_isp_params_buffer );
	dev->fmt = *f;
	return 0;
}

static const struct v4l2_ioctl_ops modalix_params_ioctl_ops = {
	.vidioc_querycap          = modalix_params_querycap,
	.vidioc_enum_fmt_meta_out = modalix_params_enum_fmt_meta_out,
	.vidioc_g_fmt_meta_out    = modalix_params_g_fmt_meta_out,
	.vidioc_s_fmt_meta_out    = modalix_params_g_fmt_meta_out,
	.vidioc_try_fmt_meta_out  = modalix_params_g_fmt_meta_out,

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

static const struct v4l2_file_operations modalix_params_fops = {
	.owner          = THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.poll           = vb2_fop_poll,
	.mmap           = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

/* ---------- init / exit ---------- */

static int modalix_meta_params_init_queue( struct modalix_meta_params_dev *dev )
{
	struct vb2_queue *q = &dev->q;

	q->type              = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes          = VB2_MMAP | VB2_WRITE;
	q->drv_priv          = dev;
	q->ops               = &modalix_params_vb2_ops;
	q->mem_ops           = &vb2_vmalloc_memops;
	q->buf_struct_size   = sizeof( struct modalix_meta_buffer );
	q->timestamp_flags   = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->lock              = &dev->queue_lock;

	return vb2_queue_init( q );
}

int modalix_meta_params_init( uint32_t ctx_id,
                              struct v4l2_device *v4l2_dev,
                              struct modalix_meta_params_dev **out )
{
	struct modalix_meta_params_dev *dev;
	struct video_device            *vdev;
	int                             rc;

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
	INIT_WORK( &dev->apply_work, modalix_params_apply_work );

	rc = modalix_meta_params_init_queue( dev );
	if ( rc ) {
		mutex_destroy( &dev->queue_lock );
		kfree( dev );
		return rc;
	}

	vdev = &dev->vdev;
	snprintf( vdev->name, sizeof( vdev->name ),
	          "modalix-isp-params-ctx%u", ctx_id );
	vdev->v4l2_dev    = v4l2_dev;
	vdev->fops        = &modalix_params_fops;
	vdev->ioctl_ops   = &modalix_params_ioctl_ops;
	vdev->release     = video_device_release_empty;
	vdev->lock        = &dev->queue_lock;
	vdev->queue       = &dev->q;
	vdev->device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING;
	vdev->vfl_dir     = VFL_DIR_TX;
	video_set_drvdata( vdev, dev );

	rc = video_register_device( vdev, VFL_TYPE_VIDEO, -1 );
	if ( rc ) {
		LOG( LOG_ERR, "ctx %u: meta-params video_register_device failed, rc=%d",
		     ctx_id, rc );
		vb2_queue_release( &dev->q );
		mutex_destroy( &dev->queue_lock );
		kfree( dev );
		return rc;
	}

	if ( ctx_id < FIRMWARE_CONTEXT_NUMBER ) {
		g_params_devs[ctx_id] = dev;
	}
	*out = dev;
	LOG( LOG_INFO, "ctx %u: /dev/video%d registered (%s)",
	     ctx_id, vdev->num, vdev->name );
	return 0;
}

void modalix_meta_params_exit( struct modalix_meta_params_dev *dev )
{
	if ( !dev ) {
		return;
	}

	if ( dev->ctx_id < FIRMWARE_CONTEXT_NUMBER &&
	     g_params_devs[dev->ctx_id] == dev ) {
		g_params_devs[dev->ctx_id] = NULL;
	}

	/* Make sure the apply worker isn't still touching dev. */
	cancel_work_sync( &dev->apply_work );

	video_unregister_device( &dev->vdev );
	vb2_queue_release( &dev->q );
	mutex_destroy( &dev->queue_lock );
	LOG( LOG_INFO, "meta-params ctx %u released", dev->ctx_id );
	kfree( dev );
}

struct modalix_meta_params_dev *modalix_meta_params_lookup( uint32_t ctx_id )
{
	if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
		return NULL;
	}
	return g_params_devs[ctx_id];
}

int modalix_meta_params_consume( struct modalix_meta_params_dev *dev,
                                 struct modalix_isp_params_buffer *out )
{
	struct modalix_meta_buffer *buf;
	void                       *vaddr;
	unsigned long               flags;

	if ( !dev || !out ) {
		return -EINVAL;
	}

	spin_lock_irqsave( &dev->buf_lock, flags );
	if ( list_empty( &dev->pending ) ) {
		spin_unlock_irqrestore( &dev->buf_lock, flags );
		return -ENODATA;
	}
	buf = list_first_entry( &dev->pending, struct modalix_meta_buffer, node );
	list_del( &buf->node );
	spin_unlock_irqrestore( &dev->buf_lock, flags );

	vaddr = vb2_plane_vaddr( &buf->vb.vb2_buf, 0 );
	if ( vaddr ) {
		memcpy( out, vaddr, sizeof( *out ) );
	} else {
		memset( out, 0, sizeof( *out ) );
	}

	vb2_buffer_done( &buf->vb.vb2_buf, VB2_BUF_STATE_DONE );
	return 0;
}

/* ---------- apply path (formerly in sbuf_func.c) ---------- */

/*
 * Apply one queued modalix_isp_params_buffer for the given context.
 * Pops the head buffer via modalix_meta_params_consume, walks
 * valid_mask, and writes each populated section through the same
 * kernel paths sbuf_mgr_apply_new_param used to use. Returns 0 on
 * apply, -ENODATA when the queue is empty, -EINVAL on an unknown
 * payload version.
 */
static int apply_v4l2_params_once( uint32_t ctx_id )
{
	struct modalix_meta_params_dev   *dev;
	struct modalix_isp_params_buffer  p;
	acamera_isp_ctx_ptr_t             p_ictx;
	acamera_fsmgr_t                  *p_fsmgr;
	int                               rc;

	dev = modalix_meta_params_lookup( ctx_id );
	if ( !dev ) {
		return -ENODATA;
	}

	rc = modalix_meta_params_consume( dev, &p );
	if ( rc ) {
		return rc;
	}

	if ( p.version != MODALIX_ISP_PARAMS_BUFFER_VERSION_V1 ) {
		LOG( LOG_ERR, "ctx %u: unknown params buffer version %u",
		     ctx_id, p.version );
		return -EINVAL;
	}

	p_ictx  = get_ctx_ptr_by_id( ctx_id );
	if ( !p_ictx ) {
		return -ENODEV;
	}
	p_fsmgr = &p_ictx->fsmgr;

#if defined( ISP_HAS_AE_MANUAL_FSM )
	if ( p.valid_mask & MODALIX_ISP_PARAMS_VALID_AE ) {
		if ( get_context_param( p_ictx, SYSTEM_MANUAL_EXPOSURE_PARAM ) == 0 ) {
			p_fsmgr->AE_fsm.new_exposure_log2 = p.ae.exposure_log2;
		}
		if ( get_context_param( p_ictx, SYSTEM_MANUAL_EXPOSURE_RATIO_PARAM ) == 0 ) {
			p_fsmgr->AE_fsm.new_exposure_ratio = p.ae.exposure_ratio;
		}
		override_context_param( p_ictx, STATUS_INFO_LDR_GAIN_LOG2_ID_PARAM, p.ae.ldr_gain_log2 );
		override_context_param( p_ictx, STATUS_INFO_AE_HIST_MEAN_PARAM,    p.ae.hist_mean );

		fsm_raise_event( &p_fsmgr->AE_fsm, event_id_split_ae_data_ready );
	}
#endif

#if defined( ISP_HAS_AWB_MANUAL_FSM )
	if ( p.valid_mask & MODALIX_ISP_PARAMS_VALID_AWB ) {
		set_context_param( p_ictx, SYSTEM_AWB_RED_GAIN_PARAM,  p.awb.red_gain );
		set_context_param( p_ictx, SYSTEM_AWB_BLUE_GAIN_PARAM, p.awb.blue_gain );
		set_context_param( p_ictx, SYSTEM_AWB_CCT_PARAM,       p.awb.temperature_detected );

		p_fsmgr->AWB_fsm.light_source_candidate = p.awb.light_source_candidate;
		p_fsmgr->AWB_fsm.p_high                 = p.awb.p_high;

#if defined( ISP_HAS_COLOR_MATRIX_FSM )
		if ( p.awb.ccm_matrix_valid ) {
			color_matrix_set_user_matrix( &p_fsmgr->color_matrix_fsm,
			                              p.awb.ccm_matrix );
		}
#endif

		/* Warming offset is computed by the IPA; the kernel only writes it
		 * (awb_coeffs_write -> rgb2rgb_coef_b). */
		if ( p.awb.awb_warming_valid ) {
			unsigned int i;
			for ( i = 0; i < MODALIX_ISP_AWB_WARMING_LEN; i++ ) {
				p_fsmgr->AWB_fsm.awb_warming[i] = p.awb.awb_warming[i];
			}
		}
	}
#endif

#if defined( ISP_HAS_GAMMA_MANUAL_FSM )
	if ( p.valid_mask & MODALIX_ISP_PARAMS_VALID_GAMMA ) {
		size_t copy_len = sizeof( p.gamma.lut );
		if ( copy_len > sizeof( p_fsmgr->gamma_manual_fsm.lut_contrast ) ) {
			copy_len = sizeof( p_fsmgr->gamma_manual_fsm.lut_contrast );
		}
		p_fsmgr->gamma_manual_fsm.lut_length = p.gamma.lut_length;
		memcpy( p_fsmgr->gamma_manual_fsm.lut_contrast, p.gamma.lut, copy_len );
		fsm_raise_event( &p_fsmgr->gamma_manual_fsm, event_id_gamma_lut_ready );
	}
#endif

#if defined( ISP_HAS_IRIDIX8_MANUAL_FSM )
	if ( p.valid_mask & MODALIX_ISP_PARAMS_VALID_IRIDIX ) {
		p_fsmgr->iridix_fsm.strength_target = p.iridix.strength_target;
		p_fsmgr->iridix_fsm.dark_enh        = p.iridix.dark_enh;
		set_context_param( p_ictx, SYSTEM_IRIDIX_DIGITAL_GAIN_PARAM, p.iridix.digital_gain );
		p_fsmgr->iridix_fsm.iridix_contrast = p.iridix.contrast;
		override_context_param( p_ictx, STATUS_INFO_IRIDIX_CONTRAST_PARAM, p.iridix.contrast );
	}
#endif

	return 0;
}

int isp_apply_v4l2_params_drain( uint32_t ctx_id )
{
	int consumed = 0;
	int rc;

	if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
		return -EINVAL;
	}

	while ( ( rc = apply_v4l2_params_once( ctx_id ) ) == 0 ) {
		consumed++;
	}
	return ( rc == -ENODATA ) ? consumed : rc;
}
