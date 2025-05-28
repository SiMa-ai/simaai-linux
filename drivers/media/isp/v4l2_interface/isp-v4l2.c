/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2021 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>

#include "acamera_logger.h"

#include "fw-interface.h"
#include "isp-v4l2-common.h"
#include "isp-v4l2-ctrl.h"
#include "isp-v4l2-stream.h"
#include "isp-v4l2.h"
#include "isp-vb2.h"

#define ISP_V4L2_NUM_INPUTS 1
#define MIPI_ISP_INTEGRATION 1

// Default device capabilities for capture and memory-to-memory modes
#define ISP_V4L2_DEVICE_CAPS_CAP ( V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE )
#define ISP_V4L2_DEVICE_CAPS_M2M ( V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING )

/* isp_v4l2_dev_t to destroy video device */
static isp_v4l2_dev_t *g_isp_v4l2_devs[FIRMWARE_CONTEXT_NUMBER];

/**
 * @brief Payload data structure for V4L2_EVENT_ACAMERA_FRAME_READY event
 * 
 */
typedef struct isp_v4l2_event_frame_ready_data_t {
    uint32_t sequence;                     // Frame sequence
    uint32_t index;                        // VB2 buffer index
    uint32_t num_planes;                   // Number of planes
    uint32_t plane_crc[AFRAME_MAX_PLANES]; // Plane HW CRC
} isp_v4l2_event_frame_ready_data_t;

/* ----------------------------------------------------------------
 * V4L2 file operations
 */
static int isp_v4l2_cap_fop_open( struct file *file )
{
    int rc = 0;
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct video_device *video_dev = video_devdata( file );
    isp_v4l2_stream_t *pstream;
    int stream_type;

    const int stream_opened = atomic_read( &dev->opened );
    if ( stream_opened >= V4L2_STREAM_TYPE_MAX ) {
        LOG( LOG_ERR, "Too many open streams, stream_opened: %d, V4L2_STREAM_TYPE_MAX: %d", stream_opened, V4L2_STREAM_TYPE_MAX );
        return -EBUSY;
    }

    // Find matching video device and derive the stream type
    for ( stream_type = 0; stream_type < V4L2_STREAM_TYPE_MAX; stream_type++ ) {
        if ( video_dev == &dev->video_dev[stream_type] ) {
            break;
        }
    }

    if ( stream_type == V4L2_STREAM_TYPE_MAX ) {
        LOG( LOG_ERR, "Unable to find matching video device, video device: %p, file: %p", video_dev, file );
        return -EINVAL;
    }

    // Check if stream has been already opened
    if ( test_bit( stream_type, &dev->stream_open_mask ) != 0 ) {
        return -EINVAL;
    } else {
        set_bit( stream_type, &dev->stream_open_mask );
    }

    LOG( LOG_INFO, "%s, ctx_id: %d, called for stream type: %d", __func__, dev->ctx_id, stream_type );

    /* init stream */
    isp_v4l2_stream_init( &dev->pstreams[stream_type], stream_type, dev->ctx_id );
    pstream = dev->pstreams[stream_type];

    // Update stream parent device information
    pstream->parent_device_info.dev = dev->v4l2_dev->dev;
    pstream->parent_device_info.lock = &dev->mlock;

    // Init capture vb2 queue
    rc = isp_vb2_cap_queue_init( &pstream->vb2_q, pstream );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "Error, isp_vb2_cap_queue_init call failed, rc: %d", rc );
        goto vb2_q_fail;
    }

    /* update dma channel */
    //dev->pstreams[stream_type]->dma = dev->dma;
	//LOG (LOG_INFO, "DMA assigned");

    /* powering on MIPI CSI */
    if(stream_type == V4L2_STREAM_TYPE_RAW) {

		rc = (dev->sd[SD_MIPI_CSI])->ops->core->s_power(dev->sd[SD_MIPI_CSI], 1);
		if( rc < 0 ) {
			LOG( LOG_ERR, "ERROR : calling power on subdev operation on MIPI CSI: %d\n", rc);
		}
	}

    // Update file private data and initialize file handle
    file->private_data = pstream;

    v4l2_fh_init( &pstream->fh, video_dev );
    v4l2_fh_add( &pstream->fh );

    /* update open counter */
    atomic_add( 1, &dev->opened );

    return rc;

vb2_q_fail:
    isp_v4l2_stream_deinit( pstream, dev->stream_on_mask );
    isp_v4l2_stream_free( pstream );

    return rc;
}

static int isp_v4l2_m2m_fop_open( struct file *file )
{
    int rc = 0;
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct video_device *video_dev = video_devdata( file );
    isp_v4l2_stream_t *pstream;
    int stream_type;

    const int stream_opened = atomic_read( &dev->opened );
    if ( stream_opened >= V4L2_STREAM_TYPE_MAX ) {
        LOG( LOG_ERR, "Too many open streams, stream_opened: %d, V4L2_STREAM_TYPE_MAX: %d", stream_opened, V4L2_STREAM_TYPE_MAX );
        return -EBUSY;
    }

    // Find matching video device and derive the stream type
    for ( stream_type = 0; stream_type < V4L2_STREAM_TYPE_MAX; stream_type++ ) {
        if ( video_dev == &dev->video_dev[stream_type] ) {
            break;
        }
    }

    if ( stream_type == V4L2_STREAM_TYPE_MAX ) {
        LOG( LOG_ERR, "Unable to find matching video device, video device: %p, file: %p", video_dev, file );
        return -EINVAL;
    }

    // Check if stream has been already opened
    if ( test_bit( stream_type, &dev->stream_open_mask ) != 0 ) {
        return -EINVAL;
    } else {
        set_bit( stream_type, &dev->stream_open_mask );
    }

    LOG( LOG_INFO, "%s, ctx_id: %d, called for stream type: %d", __func__, dev->ctx_id, stream_type );

    /* init stream */
    isp_v4l2_stream_init( &dev->pstreams[stream_type], stream_type, dev->ctx_id );
    pstream = dev->pstreams[stream_type];

    // Update stream parent device information
    pstream->parent_device_info.dev = dev->v4l2_dev->dev;
    pstream->parent_device_info.lock = &dev->mlock;

    // Init m2m context and vb2 queues
    pstream->fh.m2m_ctx = v4l2_m2m_ctx_init( dev->v4l2_m2m_dev, pstream, &isp_vb2_m2m_queue_init );

    if ( IS_ERR( pstream->fh.m2m_ctx ) ) {
        rc = PTR_ERR( pstream->fh.m2m_ctx );
        LOG( LOG_ERR, "Error, v4l2_m2m_ctx_init call failed, rc: %d", rc );
        goto vb2_q_fail;
    }

    // Update file private data and initialize file handle
    file->private_data = pstream;

    v4l2_fh_init( &pstream->fh, video_dev );
    v4l2_fh_add( &pstream->fh );

    /* update open counter */
    atomic_add( 1, &dev->opened );

    return rc;

vb2_q_fail:
    isp_v4l2_stream_deinit( pstream, dev->stream_on_mask );
    isp_v4l2_stream_free( pstream );

    return rc;
}

static int isp_v4l2_cap_fop_release( struct file *file )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;
    int open_counter;
	int rc = -1;

    LOG( LOG_INFO, "%s, ctx_id: %d, called for stream type: %d", __func__, dev->ctx_id, pstream->stream_type );

    clear_bit( pstream->stream_type, &dev->stream_open_mask );
    clear_bit( pstream->stream_type, &dev->stream_on_mask );
    open_counter = atomic_sub_return( 1, &dev->opened );

    // Deinitialize stream, stop streams and release all buffers
    isp_v4l2_stream_deinit( pstream, dev->stream_on_mask );

    // Release vb2 queue
    if ( pstream->vb2_q.lock ) {
        mutex_lock( pstream->vb2_q.lock );
    }

    isp_vb2_queue_release( &pstream->vb2_q );

    if ( pstream->vb2_q.lock ) {
        mutex_unlock( pstream->vb2_q.lock );
    }

	/* powering off MIPI CSI */
	if (pstream->stream_type  == V4L2_STREAM_TYPE_RAW) {
		if((dev->sd[SD_MIPI_CSI])->ops->core->s_power) {
			rc = (dev->sd[SD_MIPI_CSI])->ops->core->s_power(dev->sd[SD_MIPI_CSI], 0);
			if( rc < 0 ) {
				LOG( LOG_ERR, "ERROR : calling power off subdev operation on MIPI CSI: %d\n", rc);
			}
		}	
	}

    // Update device stream pointers
    if ( pstream->stream_type < V4L2_STREAM_TYPE_MAX ) {
        dev->pstreams[pstream->stream_type] = NULL;
    }

    // Release file handle
    v4l2_fh_del( &pstream->fh );
    v4l2_fh_exit( &pstream->fh );

    // Free stream memory
    isp_v4l2_stream_free( pstream );

    return 0;
}

static int isp_v4l2_m2m_fop_release( struct file *file )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;
    int open_counter;

    LOG( LOG_INFO, "%s, ctx_id: %d, called for stream type: %d", __func__, dev->ctx_id, pstream->stream_type );

    clear_bit( pstream->stream_type, &dev->stream_open_mask );
    clear_bit( pstream->stream_type, &dev->stream_on_mask );
    open_counter = atomic_sub_return( 1, &dev->opened );

    // Deinitialize stream, stop streams and release all buffers
    isp_v4l2_stream_deinit( pstream, dev->stream_on_mask );

    // Update device stream pointers
    if ( pstream->stream_type < V4L2_STREAM_TYPE_MAX ) {
        dev->pstreams[pstream->stream_type] = NULL;
    }

    // Release vb2 queues
    v4l2_m2m_ctx_release( pstream->fh.m2m_ctx );

    // Release file handle
    v4l2_fh_del( &pstream->fh );
    v4l2_fh_exit( &pstream->fh );

    // Free stream memory
    isp_v4l2_stream_free( pstream );

    return 0;
}

static ssize_t isp_v4l2_cap_fop_read( struct file *file,
                                      char __user *buf, size_t count, loff_t *ppos )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    int rc = vb2_read( &pstream->vb2_q, buf, count, ppos, file->f_flags & O_NONBLOCK );

    return rc;
}

static unsigned int isp_v4l2_cap_fop_poll( struct file *file,
                                           struct poll_table_struct *wait )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    if ( pstream->vb2_q.lock && mutex_lock_interruptible( pstream->vb2_q.lock ) )
        return POLLERR;

    int rc = vb2_poll( &pstream->vb2_q, file, wait );

    if ( pstream->vb2_q.lock ) {
        mutex_unlock( pstream->vb2_q.lock );
    }

    return rc;
}

static int isp_v4l2_cap_fop_mmap( struct file *file, struct vm_area_struct *vma )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    int rc = vb2_mmap( &pstream->vb2_q, vma );

    return rc;
}

/**
 * @brief File operations for the standard capture V4L2 device node
 * 
 */
static const struct v4l2_file_operations isp_v4l2_cap_fops = {
    .owner = THIS_MODULE,
    .open = isp_v4l2_cap_fop_open,
    .release = isp_v4l2_cap_fop_release,
    .read = isp_v4l2_cap_fop_read,
    .poll = isp_v4l2_cap_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = isp_v4l2_cap_fop_mmap,
};

/**
 * @brief File operations for the m2m V4L2 device node
 * 
 */
static const struct v4l2_file_operations isp_v4l2_m2m_fops = {
    .owner = THIS_MODULE,
    .open = isp_v4l2_m2m_fop_open,
    .release = isp_v4l2_m2m_fop_release,
    .poll = v4l2_m2m_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = v4l2_m2m_fop_mmap,
};


/* ----------------------------------------------------------------
 * V4L2 ioctl operations
 */

/**
 * @brief Helper function to return stream name
 * 
 * @param stream_type Stream type
 * @return const char* Returns pointer to the stream name on success, "n/a" otherwise 
 */
static const char *isp_v4l2_get_stream_name( isp_v4l2_stream_type_t stream_type )
{
    static const char *stream_names[] = {
        [V4L2_STREAM_TYPE_RAW] = "raw",
        [V4L2_STREAM_TYPE_OUT] = "out",
        [V4L2_STREAM_TYPE_META] = "meta",
        [V4L2_STREAM_TYPE_M2M] = "m2m",
        [V4L2_STREAM_TYPE_MAX] = "n/a"};

    switch ( stream_type ) {
    case V4L2_STREAM_TYPE_RAW:  // fallthrough
    case V4L2_STREAM_TYPE_OUT:  // fallthrough
    case V4L2_STREAM_TYPE_META: // fallthrough
    case V4L2_STREAM_TYPE_M2M:  // fallthrough
        return stream_names[stream_type];
    default:
        return stream_names[V4L2_STREAM_TYPE_MAX];
    }
}

static int isp_v4l2_cap_querycap( struct file *file, void *priv, struct v4l2_capability *cap )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, dev: %p, file: %p, priv: %p", __func__, dev, file, priv );

    strcpy( cap->driver, "arm-camera-isp" );
    snprintf( cap->card, sizeof( cap->card ), "arm-isp-%s", isp_v4l2_get_stream_name( pstream->stream_type ) );
    snprintf( cap->bus_info, sizeof( cap->bus_info ), "platform: %s-%d-%d",
              dev->v4l2_dev->name, dev->ctx_id, pstream->stream_type );

    cap->device_caps = ISP_V4L2_DEVICE_CAPS_CAP;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

    return 0;
}

static int isp_v4l2_m2m_querycap( struct file *file, void *priv, struct v4l2_capability *cap )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, dev: %p, file: %p, priv: %p", __func__, dev, file, priv );

    strcpy( cap->driver, "arm-camera-isp" );
    snprintf( cap->card, sizeof( cap->card ), "arm-isp-%s", isp_v4l2_get_stream_name( pstream->stream_type ) );
    snprintf( cap->bus_info, sizeof( cap->bus_info ), "platform: %s-%d-%d",
              dev->v4l2_dev->name, dev->ctx_id, pstream->stream_type );

    cap->device_caps = ISP_V4L2_DEVICE_CAPS_M2M;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

    return 0;
}

static int isp_v4l2_g_fmt_vid_cap( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );

    isp_v4l2_stream_get_format( pstream, V4L2_STREAM_DIRECTION_CAP, f );

    LOG( LOG_DEBUG, "v4l2_format: type: %u, w: %u, h: %u, pixelformat: 0x%x, field: %u, colorspace: %u, sizeimage: %u, bytesperline: %u, flags: %u",
         f->type,
         f->fmt.pix.width,
         f->fmt.pix.height,
         f->fmt.pix.pixelformat,
         f->fmt.pix.field,
         f->fmt.pix.colorspace,
         f->fmt.pix.sizeimage,
         f->fmt.pix.bytesperline,
         f->fmt.pix.flags );

    return 0;
}

static int isp_v4l2_g_fmt_vid_out( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );

    isp_v4l2_stream_get_format( pstream, V4L2_STREAM_DIRECTION_OUT, f );

    LOG( LOG_DEBUG, "v4l2_format: type: %u, w: %u, h: %u, pixelformat: 0x%x, field: %u, colorspace: %u, sizeimage: %u, bytesperline: %u, flags: %u",
         f->type,
         f->fmt.pix.width,
         f->fmt.pix.height,
         f->fmt.pix.pixelformat,
         f->fmt.pix.field,
         f->fmt.pix.colorspace,
         f->fmt.pix.sizeimage,
         f->fmt.pix.bytesperline,
         f->fmt.pix.flags );

    return 0;
}

static int isp_v4l2_enum_fmt_vid_cap( struct file *file, void *priv, struct v4l2_fmtdesc *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );

    return isp_v4l2_stream_enum_format( pstream, V4L2_STREAM_DIRECTION_CAP, f );
}

static int isp_v4l2_enum_fmt_vid_out( struct file *file, void *priv, struct v4l2_fmtdesc *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );

    return isp_v4l2_stream_enum_format( pstream, V4L2_STREAM_DIRECTION_OUT, f );
}

static int isp_v4l2_try_fmt_vid_cap( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );

    return isp_v4l2_stream_try_format( pstream, V4L2_STREAM_DIRECTION_CAP, f );
}

static int isp_v4l2_try_fmt_vid_out( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );

    return isp_v4l2_stream_try_format( pstream, V4L2_STREAM_DIRECTION_OUT, f );
}

static int isp_v4l2_s_fmt_vid_cap( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;
	isp_v4l2_dev_t *dev = video_drvdata( file );
    struct vb2_queue *q;
	int rc =0;
	struct v4l2_subdev_format fmt;

    // Check stream type and get correct vb queue pointer
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, f->type );
    } else {
        q = &pstream->vb2_q;
    }

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );
    if ( vb2_is_busy( q ) ) {
        return -EBUSY;
    }

    rc = isp_v4l2_stream_set_format( pstream, V4L2_STREAM_DIRECTION_CAP, f );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "isp_v4l2_stream_set_format call failed, rc: %d", rc );
        return rc;
    }

    if (pstream->stream_type == V4L2_STREAM_TYPE_RAW) {
        fmt.format.width = f->fmt.pix.width;
        fmt.format.height = f->fmt.pix.height;
		fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		fmt.pad = 0; // TODO : hard coded to 0. Need better way to get this

		//LOG( LOG_INFO, "S_FMT RAW data \n" );
        if (f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_SRGGB8)
            fmt.format.code = MEDIA_BUS_FMT_SRGGB8_1X8;
        else if (f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_SRGGB12)
            fmt.format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
        else if (f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_SRGGB14)
            fmt.format.code = MEDIA_BUS_FMT_SRGGB14_1X14;
		else if (f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_SRGGB16)	
            fmt.format.code = MEDIA_BUS_FMT_SRGGB16_1X16;	
		else {
            LOG( LOG_ERR, "Unsupported isp_v4l2_stream_set_format RAW failed" );
	    	return -EINVAL;          
		}

		fmt.format.colorspace = V4L2_COLORSPACE_SRGB;

		rc = (dev->sd[SD_MIPI_CSI])->ops->pad->set_fmt(dev->sd[SD_MIPI_CSI], NULL, &fmt);
		if ( rc < 0) {
			LOG( LOG_ERR, "ERROR calling set_fmt direct on subdevice rc = %d, %#llx\n", rc, dev->sd[SD_MIPI_CSI]);
			return rc;
		}
		rc = (dev->sd[SD_CAMERA])->ops->pad->set_fmt(dev->sd[SD_CAMERA], NULL, &fmt);
		if ( rc < 0) {
			LOG( LOG_ERR, "ERROR calling set_fmt direct on subdevice rc = %d, %#llx\n", rc, dev->sd[SD_CAMERA]);
			return rc;
		}	
		LOG(LOG_INFO,"SUCCESS : invoking set_fmt on mipi subdev\n");
    }

    return 0;
}

static int isp_v4l2_s_fmt_vid_out( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_stream_t *pstream = file->private_data;
    struct vb2_queue *q;

    // Check stream type and get correct vb queue pointer
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, f->type );
    } else {
        q = &pstream->vb2_q;
    }

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );
    if ( vb2_is_busy( q ) ) {
        return -EBUSY;
    }

    int rc = isp_v4l2_stream_set_format( pstream, V4L2_STREAM_DIRECTION_OUT, f );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "isp_v4l2_stream_set_format call failed, rc: %d", rc );
        return rc;
    }

    return 0;
}

static int isp_v4l2_enum_framesizes( struct file *file, void *priv, struct v4l2_frmsizeenum *fsize )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_INFO, "%s, stream type: %d", __func__, pstream->stream_type );

    return isp_v4l2_stream_enum_framesizes( pstream, V4L2_STREAM_DIRECTION_CAP, fsize );
}

/* Per-stream control operations */
static inline bool isp_v4l2_is_q_busy( struct vb2_queue *queue, struct file *file )
{
    return queue->owner && queue->owner != file->private_data;
}

static int isp_v4l2_cap_streamon( struct file *file, void *priv, enum v4l2_buf_type buf_type )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;

    if ( isp_v4l2_is_q_busy( &pstream->vb2_q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_streamon( &pstream->vb2_q, buf_type );
    if ( rc != 0 ) {
        LOG( LOG_ERR, "vb2_streamon() call failed, rc: %d", rc );
        return rc;
    }

	LOG (LOG_INFO, "captured stream on called, open mask : %d", dev->stream_open_mask);
    /* Start hardware */
    rc = isp_v4l2_stream_on( pstream, V4L2_STREAM_DIRECTION_CAP, dev->stream_open_mask );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "isp_v4l2_stream_on() call failed, stream type: %d, stream direction: %d, stream open mask: 0x%lx, rc: %d",
             pstream->stream_type, V4L2_STREAM_DIRECTION_CAP, dev->stream_open_mask, rc );
        isp_v4l2_stream_off( pstream, V4L2_STREAM_DIRECTION_CAP, dev->stream_on_mask );
        return rc;
    }
	if (pstream->stream_type  == V4L2_STREAM_TYPE_OUT) {
		rc = (dev->sd[SD_CAMERA])->ops->video->s_stream(dev->sd[SD_CAMERA], 1);
		if ( rc < 0) {
		     LOG( LOG_ERR, "ERROR calling set_fmt direct on subdevice rc = %d, %#llx\n", rc, dev->sd[SD_CAMERA]);
	    	 return rc;
		}	
		LOG (LOG_INFO, "stream ON called on camera");
	}

    if ( rc > 0 ) {
        set_bit( pstream->stream_type, &dev->stream_on_mask );
    }

    return 0;
}

static int isp_v4l2_m2m_streamon( struct file *file, void *priv, enum v4l2_buf_type buf_type )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;

    struct vb2_queue *q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, buf_type );

    if ( isp_v4l2_is_q_busy( q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_streamon( q, buf_type );
    if ( rc != 0 ) {
        LOG( LOG_ERR, "vb2_streamon() call failed, rc: %d", rc );
        return rc;
    }

    const isp_v4l2_stream_direction_t stream_direction = ( V4L2_TYPE_IS_OUTPUT( buf_type ) ? V4L2_STREAM_DIRECTION_OUT : V4L2_STREAM_DIRECTION_CAP );

    /* Start hardware */
    rc = isp_v4l2_stream_on( pstream, stream_direction, dev->stream_open_mask );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "isp_v4l2_stream_on() call failed, stream type: %d, stream direction: %d, stream open mask: 0x%lx, rc: %d",
             pstream->stream_type, stream_direction, dev->stream_open_mask, rc );
        isp_v4l2_stream_off( pstream, stream_direction, dev->stream_on_mask );
        return rc;
    }

    if ( rc > 0 ) {
        set_bit( pstream->stream_type, &dev->stream_on_mask );
        v4l2_m2m_try_schedule( pstream->fh.m2m_ctx );
    }

    return 0;
}

static int isp_v4l2_cap_streamoff( struct file *file, void *priv, enum v4l2_buf_type buf_type )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;
	int rc = -1;

    if ( isp_v4l2_is_q_busy( &pstream->vb2_q, file ) ) {
        return -EBUSY;
    }

	if (pstream->stream_type  == V4L2_STREAM_TYPE_OUT) {
		rc = (dev->sd[SD_CAMERA])->ops->video->s_stream(dev->sd[SD_CAMERA], 0);
		if ( rc < 0) {
		     LOG( LOG_ERR, "ERROR calling set_fmt direct on subdevice rc = %d, %#llx\n", rc, dev->sd[SD_CAMERA]);
	    	 return rc;
		}	
		LOG (LOG_INFO, "stream OFF called on camera");
	}

    /* Stop hardware */
    rc = isp_v4l2_stream_off( pstream, V4L2_STREAM_DIRECTION_CAP, dev->stream_on_mask );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "isp_v4l2_stream_off() call failed, stream type: %d, stream direction: %d, stream on mask: 0x%lx, rc: %d",
             pstream->stream_type, V4L2_STREAM_DIRECTION_CAP, dev->stream_on_mask, rc );
        return rc;
    }

    if ( rc == 0 ) {
        clear_bit( pstream->stream_type, &dev->stream_on_mask );
    }

    /* vb streamoff */
    rc = vb2_streamoff( &pstream->vb2_q, buf_type );

    return rc;
}

static int isp_v4l2_m2m_streamoff( struct file *file, void *priv, enum v4l2_buf_type buf_type )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    isp_v4l2_stream_t *pstream = file->private_data;

    struct vb2_queue *q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, buf_type );

    if ( isp_v4l2_is_q_busy( q, file ) ) {
        return -EBUSY;
    }

    const isp_v4l2_stream_direction_t stream_direction = ( V4L2_TYPE_IS_OUTPUT( buf_type ) ? V4L2_STREAM_DIRECTION_OUT : V4L2_STREAM_DIRECTION_CAP );

    /* Stop hardware */
    int rc = isp_v4l2_stream_off( pstream, stream_direction, dev->stream_on_mask );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "isp_v4l2_stream_off() call failed, stream type: %d, stream direction: %d, stream on mask: 0x%lx, rc: %d",
             pstream->stream_type, V4L2_STREAM_DIRECTION_CAP, dev->stream_on_mask, rc );
        return rc;
    }

    if ( rc == 0 ) {
        clear_bit( pstream->stream_type, &dev->stream_on_mask );
    }

    /* v4l2 m2m streamoff */
    rc = v4l2_m2m_streamoff( file, pstream->fh.m2m_ctx, buf_type );

    return rc;
}

/* input control */
static int isp_v4l2_enum_input( struct file *file, void *fh, struct v4l2_input *input )
{
    /* currently only support general camera input */
    if ( input->index > 0 ) {
        return -EINVAL;
    }

    strlcpy( input->name, "camera", sizeof( input->name ) );
    input->type = V4L2_INPUT_TYPE_CAMERA;

    return 0;
}

static int isp_v4l2_g_input( struct file *file, void *fh, unsigned int *input )
{
    /* currently only support general camera input */
    *input = 0;

    return 0;
}

static int isp_v4l2_s_input( struct file *file, void *fh, unsigned int input )
{
    /* currently only support general camera input */
    return input == 0 ? 0 : -EINVAL;
}


/* vb2 customization for multi-stream support */
static int isp_v4l2_cap_reqbufs( struct file *file, void *priv,
                                 struct v4l2_requestbuffers *reqbufs )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_DEBUG, "%s, stream type: %d, ownermatch: %d", __func__, pstream->stream_type, isp_v4l2_is_q_busy( &pstream->vb2_q, file ) );
    if ( isp_v4l2_is_q_busy( &pstream->vb2_q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_reqbufs( &pstream->vb2_q, reqbufs );
    if ( rc == 0 ) {
        pstream->vb2_q.owner = ( reqbufs->count ? file->private_data : NULL );
        pstream->num_buffers_requested = reqbufs->count;
    }

    LOG( LOG_DEBUG, "%s, stream type: %d, req->type: %d, req->memory: %d, req->count: %d, rc: %d",
         __func__, pstream->stream_type, reqbufs->type, reqbufs->memory, reqbufs->count, rc );

    return rc;
}

static int isp_v4l2_m2m_reqbufs( struct file *file, void *priv,
                                 struct v4l2_requestbuffers *reqbufs )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    struct vb2_queue *q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, reqbufs->type );

    LOG( LOG_DEBUG, "%s, stream type: %d, ownermatch: %d", __func__, pstream->stream_type, isp_v4l2_is_q_busy( q, file ) );
    if ( isp_v4l2_is_q_busy( q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_reqbufs( q, reqbufs );
    if ( rc == 0 ) {
        q->owner = ( reqbufs->count ? file->private_data : NULL );
        pstream->num_buffers_requested = reqbufs->count;
    }

    LOG( LOG_DEBUG, "%s, stream type: %d, req->type: %d, req->memory: %d, req->count: %d, rc: %d",
         __func__, pstream->stream_type, reqbufs->type, reqbufs->memory, reqbufs->count, rc );

    return rc;
}

static int isp_v4l2_cap_expbuf( struct file *file, void *priv, struct v4l2_exportbuffer *expbuf )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    if ( isp_v4l2_is_q_busy( &pstream->vb2_q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_expbuf( &pstream->vb2_q, expbuf );
    LOG( LOG_DEBUG, "%s, stream type: %d, expbuf->type: %d, expbuf->index: %d, expbuf->plane: %d, rc: %d",
         __func__, pstream->stream_type, expbuf->type, expbuf->index, expbuf->plane, rc );

    return rc;
}

static int isp_v4l2_m2m_expbuf( struct file *file, void *priv, struct v4l2_exportbuffer *expbuf )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    struct vb2_queue *q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, expbuf->type );

    if ( isp_v4l2_is_q_busy( q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_expbuf( q, expbuf );
    LOG( LOG_DEBUG, "%s, stream type: %d, expbuf->type: %d, expbuf->index: %d, expbuf->plane: %d, rc: %d",
         __func__, pstream->stream_type, expbuf->type, expbuf->index, expbuf->plane, rc );

    return rc;
}

static int isp_v4l2_cap_querybuf( struct file *file, void *priv, struct v4l2_buffer *buf )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    int rc = vb2_querybuf( &pstream->vb2_q, buf );
    LOG( LOG_DEBUG, "%s, stream type: %d, buf->type: %d, buf->index: %d, rc: %d",
         __func__, pstream->stream_type, buf->type, buf->index, rc );

    return rc;
}

static int isp_v4l2_m2m_querybuf( struct file *file, void *priv, struct v4l2_buffer *buf )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    int rc = v4l2_m2m_querybuf( file, pstream->fh.m2m_ctx, buf );
    LOG( LOG_DEBUG, "%s, stream type: %d, buf->type: %d, buf->index: %d, rc: %d",
         __func__, pstream->stream_type, buf->type, buf->index, rc );

    return rc;
}

static int isp_v4l2_cap_qbuf( struct file *file, void *priv, struct v4l2_buffer *buf )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_DEBUG, "%s, stream type: %d, ownermatch: %d", __func__, pstream->stream_type, isp_v4l2_is_q_busy( &pstream->vb2_q, file ) );
    if ( isp_v4l2_is_q_busy( &pstream->vb2_q, file ) ) {
        return -EBUSY;
    }

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 20, 0 ) )
    int rc = vb2_qbuf( &pstream->vb2_q, NULL, buf );
#else
    int rc = vb2_qbuf( &pstream->vb2_q, buf );
#endif

    LOG( LOG_DEBUG, "%s, stream type: %d, buf->type: %d, buf->index: %d, rc: %d",
         __func__, pstream->stream_type, buf->type, buf->index, rc );

    return rc;
}

static int isp_v4l2_m2m_qbuf( struct file *file, void *priv, struct v4l2_buffer *buf )
{
    isp_v4l2_stream_t *pstream = file->private_data;
    struct vb2_queue *q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, buf->type );

    LOG( LOG_DEBUG, "%s, stream type: %d, ownermatch: %d", __func__, pstream->stream_type, isp_v4l2_is_q_busy( q, file ) );
    if ( isp_v4l2_is_q_busy( q, file ) ) {
        return -EBUSY;
    }

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 20, 0 ) )
    int rc = vb2_qbuf( q, NULL, buf );
#else
    int rc = vb2_qbuf( q, buf );
#endif

    LOG( LOG_DEBUG, "%s, stream type: %d, buf->type: %d, buf->index: %d, rc: %d",
         __func__, pstream->stream_type, buf->type, buf->index, rc );

    v4l2_m2m_try_schedule( pstream->fh.m2m_ctx );

    return rc;
}

static int isp_v4l2_cap_dqbuf( struct file *file, void *priv, struct v4l2_buffer *buf )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    LOG( LOG_DEBUG, "%s, stream type: %d, ownermatch: %d", __func__, pstream->stream_type, isp_v4l2_is_q_busy( &pstream->vb2_q, file ) );
    if ( isp_v4l2_is_q_busy( &pstream->vb2_q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_dqbuf( &pstream->vb2_q, buf, file->f_flags & O_NONBLOCK );
    LOG( LOG_DEBUG, "%s, stream type: %d, p->type: %d, p->index: %d, rc: %d", __func__, pstream->stream_type, buf->type, buf->index, rc );

    return rc;
}

static int isp_v4l2_m2m_dqbuf( struct file *file, void *priv, struct v4l2_buffer *buf )
{
    isp_v4l2_stream_t *pstream = file->private_data;

    struct vb2_queue *q = v4l2_m2m_get_vq( pstream->fh.m2m_ctx, buf->type );

    LOG( LOG_DEBUG, "%s, stream type: %d, ownermatch: %d", __func__, pstream->stream_type, isp_v4l2_is_q_busy( q, file ) );
    if ( isp_v4l2_is_q_busy( q, file ) ) {
        return -EBUSY;
    }

    int rc = vb2_dqbuf( q, buf, file->f_flags & O_NONBLOCK );
    LOG( LOG_DEBUG, "%s, stream type: %d, p->type: %d, p->index: %d, rc: %d", __func__, pstream->stream_type, buf->type, buf->index, rc );

    return rc;
}

static int isp_v4l2_subscribe_event( struct v4l2_fh *fh, const struct v4l2_event_subscription *sub )
{
    isp_v4l2_stream_t *pstream = container_of( fh, isp_v4l2_stream_t, fh );

    switch ( sub->type ) {
    case V4L2_EVENT_ACAMERA_FRAME_READY:
        return v4l2_event_subscribe( fh, sub, pstream->num_buffers_requested, NULL );

    // Only one event is currently supported, fall-through
    case V4L2_EVENT_ACAMERA_STREAM_OFF:
    default:
        return -EINVAL;
    }
}

/**
 * @brief V4L2 ioctl for the standard capture V4L2 device node
 * 
 */
static const struct v4l2_ioctl_ops isp_v4l2_cap_ioctl_ops = {
    .vidioc_querycap = isp_v4l2_cap_querycap,

    /* Per-stream config operations */
    .vidioc_g_fmt_vid_cap_mplane = isp_v4l2_g_fmt_vid_cap,
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 5, 3, 0 ) )
    .vidioc_enum_fmt_vid_cap = isp_v4l2_enum_fmt_vid_cap,
#else
    .vidioc_enum_fmt_vid_cap_mplane = isp_v4l2_enum_fmt_vid_cap,
#endif
    .vidioc_try_fmt_vid_cap_mplane = isp_v4l2_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap_mplane = isp_v4l2_s_fmt_vid_cap,
    .vidioc_enum_framesizes = isp_v4l2_enum_framesizes,

    /* Per-stream control operations */
    .vidioc_streamon = isp_v4l2_cap_streamon,
    .vidioc_streamoff = isp_v4l2_cap_streamoff,

    /* input control */
    .vidioc_enum_input = isp_v4l2_enum_input,
    .vidioc_g_input = isp_v4l2_g_input,
    .vidioc_s_input = isp_v4l2_s_input,

    /* vb2 customization for multi-stream support */
    .vidioc_reqbufs = isp_v4l2_cap_reqbufs,
    .vidioc_expbuf = isp_v4l2_cap_expbuf,
    .vidioc_querybuf = isp_v4l2_cap_querybuf,
    .vidioc_qbuf = isp_v4l2_cap_qbuf,
    .vidioc_dqbuf = isp_v4l2_cap_dqbuf,

    /* v4l2 event ioctls */
    .vidioc_log_status = v4l2_ctrl_log_status,
    .vidioc_subscribe_event = isp_v4l2_subscribe_event,
    .vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/**
 * @brief V4L2 ioctl for the m2m V4L2 device node
 * 
 */
static const struct v4l2_ioctl_ops isp_v4l2_m2m_ioctl_ops = {
    .vidioc_querycap = isp_v4l2_m2m_querycap,

    /* Per-stream config operations */
    .vidioc_g_fmt_vid_cap_mplane = isp_v4l2_g_fmt_vid_cap,
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 5, 3, 0 ) )
    .vidioc_enum_fmt_vid_cap = isp_v4l2_enum_fmt_vid_cap,
#else
    .vidioc_enum_fmt_vid_cap_mplane = isp_v4l2_enum_fmt_vid_cap,
#endif
    .vidioc_try_fmt_vid_cap_mplane = isp_v4l2_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap_mplane = isp_v4l2_s_fmt_vid_cap,

    .vidioc_g_fmt_vid_out_mplane = isp_v4l2_g_fmt_vid_out,
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 5, 3, 0 ) )
    .vidioc_enum_fmt_vid_out = isp_v4l2_enum_fmt_vid_out,
#else
    .vidioc_enum_fmt_vid_out_mplane = isp_v4l2_enum_fmt_vid_out,
#endif
    .vidioc_try_fmt_vid_out_mplane = isp_v4l2_try_fmt_vid_out,
    .vidioc_s_fmt_vid_out_mplane = isp_v4l2_s_fmt_vid_out,

    .vidioc_enum_framesizes = isp_v4l2_enum_framesizes,

    /* Per-stream control operations */
    .vidioc_streamon = isp_v4l2_m2m_streamon,
    .vidioc_streamoff = isp_v4l2_m2m_streamoff,

    /* input control */
    .vidioc_enum_input = isp_v4l2_enum_input,
    .vidioc_g_input = isp_v4l2_g_input,
    .vidioc_s_input = isp_v4l2_s_input,

    /* vb2 customization for multi-stream support */
    .vidioc_reqbufs = isp_v4l2_m2m_reqbufs,
    .vidioc_expbuf = isp_v4l2_m2m_expbuf,
    .vidioc_querybuf = isp_v4l2_m2m_querybuf,
    .vidioc_qbuf = isp_v4l2_m2m_qbuf,
    .vidioc_dqbuf = isp_v4l2_m2m_dqbuf,

    /* v4l2 event ioctls */
    .vidioc_log_status = v4l2_ctrl_log_status,
    .vidioc_subscribe_event = isp_v4l2_subscribe_event,
    .vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static void isp_v4l2_m2m_device_run( void *priv )
{
    if ( priv ) {
        const isp_v4l2_stream_t *pstream = priv;
        if ( fw_intf_m2m_process_request( pstream->ctx_id ) ) {
            LOG( LOG_ERR, "%s, failed to request frame process on context id: %d", __func__, pstream->ctx_id );
            return;
        }

        LOG( LOG_DEBUG, "%s, requesting a frame process on context id: %d", __func__, pstream->ctx_id );
    }
}

static void isp_v4l2_m2m_job_abort( void *priv )
{
    if ( priv ) {
        const isp_v4l2_stream_t *pstream = priv;
        if ( pstream->ctx_id < FIRMWARE_CONTEXT_NUMBER ) {
            v4l2_m2m_job_finish( pstream->fh.m2m_ctx->m2m_dev, pstream->fh.m2m_ctx );
            LOG( LOG_DEBUG, "%s called on context id: %d", __func__, pstream->ctx_id );
        } else {
            LOG( LOG_ERR, "%s, stream context id is out of range, context id: %d, stream type: %d",
                 __func__, pstream->ctx_id, pstream->stream_type );
        }
    }
}

static struct v4l2_m2m_ops isp_v4l2_m2m_ops = {
    .device_run = isp_v4l2_m2m_device_run,
    .job_abort = isp_v4l2_m2m_job_abort,
};

static int
subdev_notifier_bound(struct v4l2_async_notifier *notifier,
                     struct v4l2_subdev *subdev, struct v4l2_async_subdev *asd)
{
    isp_v4l2_dev_t *dev = container_of(notifier, isp_v4l2_dev_t, subdev_notifier);
    int i = 0;
    LOG( LOG_INFO, "BOUND : notification for subdev : %s, ctx_id: %u",
			subdev->name, dev->ctx_id);

    for (i = 0; i < SD_MAX; i++) {
        if (dev->asd[i].match.fwnode == of_fwnode_handle(subdev->dev->of_node)) {
            LOG( LOG_INFO, "BOUND : found match, %d, %#llx", i, of_fwnode_handle(subdev->dev->of_node));
           	dev->sd[i] = subdev;
			break;
        }
    }

    if (i == SD_MAX) {
        LOG( LOG_ERR, "BOUND : no match found");
        return -EINVAL;
    }


   return 0;
}

static int
subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
    isp_v4l2_dev_t *dev = container_of(notifier, isp_v4l2_dev_t, subdev_notifier);
    int i = 0;
    int rc = 0;
    struct video_device *vfd;
	
    LOG( LOG_INFO, "COMPLETE :  notification for context %u", dev->ctx_id);

	// Custom device names
    static char device_names[FIRMWARE_CONTEXT_NUMBER][V4L2_STREAM_TYPE_MAX][16];

    //-1 is because we dont want to register meta , gstreamer v4l2src probes it
    for ( i = 0; i < V4L2_STREAM_TYPE_MAX; ++i ) {

        vfd = &dev->video_dev[i];
        vfd->ctrl_handler = &dev->isp_v4l2_ctrl.ctrl_hdl_std_ctrl;

        snprintf( vfd->name, sizeof( vfd->name ), "isp_v4l2-vid-cap-%s", isp_v4l2_get_stream_name( i ) );
        snprintf( device_names[dev->ctx_id][i], sizeof( device_names[dev->ctx_id][i] ), "video%u%s", dev->ctx_id, isp_v4l2_get_stream_name( i ) );

        // Set appropriate file ops and ioctl depending on the stream type
        if ( i == V4L2_STREAM_TYPE_M2M ) {
            vfd->fops = &isp_v4l2_m2m_fops;
            vfd->ioctl_ops = &isp_v4l2_m2m_ioctl_ops;
            vfd->vfl_dir = VFL_DIR_M2M;
            vfd->device_caps = ISP_V4L2_DEVICE_CAPS_M2M;
        } else {
            vfd->fops = &isp_v4l2_cap_fops;
            vfd->ioctl_ops = &isp_v4l2_cap_ioctl_ops;
            vfd->vfl_dir = VFL_DIR_RX;
            vfd->device_caps = ISP_V4L2_DEVICE_CAPS_CAP;
        }

        vfd->release = video_device_release_empty;
        vfd->v4l2_dev = dev->v4l2_dev;
        vfd->queue = NULL; // queue will be customized in file handle
        vfd->tvnorms = 0;
        vfd->vfl_type = VFL_TYPE_VIDEO;
        vfd->dev.init_name = device_names[dev->ctx_id][i];

        /*
         * Provide a mutex to v4l2 core. It will be used to protect
         * all fops and v4l2 ioctls.
         */
        vfd->lock = &dev->mlock;
        video_set_drvdata( vfd, dev );

        /* videoX start number, -1 is autodetect */
        rc = video_register_device( vfd, VFL_TYPE_VIDEO, -1 );
        if ( rc < 0 ) {
            LOG( LOG_ERR, "ERROR : registerig video device for contex id: %u", dev->ctx_id);
			goto unreg_dev;
        }

        LOG( LOG_INFO, "V4L2 capture device for context id: %u, registered as: %s",
             dev->ctx_id, video_device_node_name( vfd ) );
    }

	rc = v4l2_device_register_subdev_nodes(dev->v4l2_dev);
	if (rc < 0) {
		LOG( LOG_ERR, "Failed to create subdev nodes\n");
		goto unreg_dev;
	}
	return 0;

unreg_dev:
    for ( i = 0; i < V4L2_STREAM_TYPE_MAX; ++i )
        video_unregister_device( &dev->video_dev[i] );

	return rc;
}
static const struct v4l2_async_notifier_operations subdev_notifier_ops = {
    .bound = subdev_notifier_bound,
    .complete = subdev_notifier_complete,
};

struct subdev_async_priv_data {
    struct v4l2_async_subdev asd;
    struct v4l2_subdev *sd;
};


static int isp_v4l2_init_dev( uint32_t ctx_id, struct v4l2_device *v4l2_dev )
{
    isp_v4l2_dev_t *dev;
    struct video_device *vfd;
    int rc = 0;
    int i;
    struct device_node *node = NULL;
    struct device_node *port = NULL;
    struct device_node *remote = NULL;
    struct v4l2_fwnode_endpoint endpoint;

    // Custom device names
    static char device_names[FIRMWARE_CONTEXT_NUMBER][V4L2_STREAM_TYPE_MAX][16];

    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Context id: %d is out of range [0;%d]",
             ctx_id, ( ( FIRMWARE_CONTEXT_NUMBER > 0 ) ? FIRMWARE_CONTEXT_NUMBER - 1 : 0 ) );
        return -EINVAL;
    }

    /* allocate main isp_v4l2 state structure */
    dev = kzalloc( sizeof( *dev ), GFP_KERNEL );
    if ( !dev )
        return -ENOMEM;

    memset( dev, 0x0, sizeof( isp_v4l2_dev_t ) );

#if MIPI_ISP_INTEGRATION
    v4l2_async_nf_init(&dev->subdev_notifier);

	port = of_graph_get_port_by_id(v4l2_dev->dev->of_node, ctx_id);
	if (!port) {
		LOG (LOG_ERR, "failed to get port by ctx id : %u", ctx_id);
		goto free_dev;
	}

	
    for_each_child_of_node(port, node) {
		LOG( LOG_INFO, "endpoint name : %s, full_name %s", node->name, node->full_name);
		remote = of_graph_get_remote_port_parent(node);
		if (!remote) {
			LOG( LOG_ERR, "ERROR : getting remote endpoint");
			goto free_dev; //Check this
	 	}
#if 0
		ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &endpoint);
		if ( ret < 0) {
			LOG (LOG_ERR, "Failed to parse remote endpoint");
			goto free_dev;
		}
		
		LOG( LOG_ERR, "endpoint id %d, endpoint port %d\n", endpoint.base.id, endpoint.base.port);
#endif

		struct subdev_async_priv_data *pd = v4l2_async_nf_add_fwnode(&dev->subdev_notifier,
											of_fwnode_handle(remote), struct subdev_async_priv_data);
		if(IS_ERR(pd)) {
			LOG( LOG_ERR, "ERROR : registering notifier for %s", remote->name);
			rc = PTR_ERR(pd);
			goto free_dev;
		}

		if(!strncmp(remote->name, "csi", 3)) {
			dev->asd[SD_MIPI_CSI].match.fwnode = of_fwnode_handle(remote);
		} else {
			dev->asd[SD_CAMERA].match.fwnode = of_fwnode_handle(remote);
		}
		LOG( LOG_INFO, "SUCCESS : registered remote endpoint for %s, fwnode : %#llx", remote->name,  of_fwnode_handle(remote));
	}

	of_node_put(port);
#endif

    /* register v4l2_device */
    dev->v4l2_dev = v4l2_dev;
    dev->ctx_id = ctx_id;

    /* init v4l2 controls */
    dev->isp_v4l2_ctrl.v4l2_dev = dev->v4l2_dev;
    rc = isp_v4l2_ctrl_init( ctx_id, &dev->isp_v4l2_ctrl );
    if ( rc ) {
        goto free_dev;
    }

    /* initialize locks */
    mutex_init( &dev->mlock );

    /* initialize open counter */
    atomic_set( &dev->opened, 0 );

    /* registering async notifier */
    dev->subdev_notifier.ops = &subdev_notifier_ops;
    dev->subdev_notifier.v4l2_dev = v4l2_dev;
    rc = v4l2_async_nf_register(v4l2_dev, &dev->subdev_notifier);
    if (rc) {
		LOG( LOG_ERR, "Error registering async notifier %#x", rc);
		goto free_dev;
    }

    // Initialise m2m driver structures
    dev->v4l2_m2m_dev = v4l2_m2m_init( &isp_v4l2_m2m_ops );
    if ( IS_ERR( dev->v4l2_m2m_dev ) ) {
        rc = PTR_ERR( dev->v4l2_m2m_dev );
        LOG( LOG_ERR, "Failed to initialize V4L2 m2m device, rc: %d", rc );

        goto free_dev;
    }

#if !MIPI_ISP_INTEGRATION
    /* finally start creating the device nodes */
    for ( i = 0; i < V4L2_STREAM_TYPE_MAX; ++i ) {
        vfd = &dev->video_dev[i];
        vfd->ctrl_handler = &dev->isp_v4l2_ctrl.ctrl_hdl_std_ctrl;

        snprintf( vfd->name, sizeof( vfd->name ), "isp_v4l2-vid-cap-%s", isp_v4l2_get_stream_name( i ) );
        snprintf( device_names[ctx_id][i], sizeof( device_names[ctx_id][i] ), "video%u%s", ctx_id, isp_v4l2_get_stream_name( i ) );

        // Set appropriate file ops and ioctl depending on the stream type
        if ( i == V4L2_STREAM_TYPE_M2M ) {
            vfd->fops = &isp_v4l2_m2m_fops;
            vfd->ioctl_ops = &isp_v4l2_m2m_ioctl_ops;
            vfd->vfl_dir = VFL_DIR_M2M;
            vfd->device_caps = ISP_V4L2_DEVICE_CAPS_M2M;
        } else {
            vfd->fops = &isp_v4l2_cap_fops;
            vfd->ioctl_ops = &isp_v4l2_cap_ioctl_ops;
            vfd->vfl_dir = VFL_DIR_RX;
            vfd->device_caps = ISP_V4L2_DEVICE_CAPS_CAP;
        }

        vfd->release = video_device_release_empty;
        vfd->v4l2_dev = dev->v4l2_dev;
        vfd->queue = NULL; // queue will be customized in file handle
        vfd->tvnorms = 0;
        vfd->dev.init_name = device_names[ctx_id][i];

        /*
        * Provide a mutex to v4l2 core. It will be used to protect
        * all fops and v4l2 ioctls.
        */
        vfd->lock = &dev->mlock;
        video_set_drvdata( vfd, dev );

        /* videoX start number, -1 is autodetect */
        rc = video_register_device( vfd, VFL_TYPE_GRABBER, -1 );
        if ( rc < 0 ) {
            goto unreg_dev;
        }

        LOG( LOG_INFO, "V4L2 device for context id: %u, registered as: %s",
             ctx_id, video_device_node_name( vfd ) );
    }
#endif

    /* store dev pointer to destroy later and find stream */
    g_isp_v4l2_devs[ctx_id] = dev;

    return rc;

#if !MIPI_ISP_INTEGRATION
unreg_dev:
    v4l2_m2m_release( dev->v4l2_m2m_dev );
    for ( i = 0; i < V4L2_STREAM_TYPE_MAX; ++i ) {
        video_unregister_device( &dev->video_dev[i] );
    }
#endif
    isp_v4l2_ctrl_deinit( &dev->isp_v4l2_ctrl );

free_dev:
    kfree( dev );

    return rc;
}


static void isp_v4l2_destroy_dev( int ctx_id )
{
    int i;

    if ( g_isp_v4l2_devs[ctx_id] ) {
        for ( i = 0; i < V4L2_STREAM_TYPE_MAX; ++i ) {
            LOG( LOG_INFO, "Unregistering video device: %s", video_device_node_name( &g_isp_v4l2_devs[ctx_id]->video_dev[i] ) );

            /* unregister video device */
            video_unregister_device( &g_isp_v4l2_devs[ctx_id]->video_dev[i] );
        }

        isp_v4l2_ctrl_deinit( &g_isp_v4l2_devs[ctx_id]->isp_v4l2_ctrl );
        v4l2_m2m_release( g_isp_v4l2_devs[ctx_id]->v4l2_m2m_dev );

        kfree( g_isp_v4l2_devs[ctx_id] );
        g_isp_v4l2_devs[ctx_id] = NULL;
    } else {
        LOG( LOG_INFO, "g_isp_v4l2_devs for context: %d is NULL, skipping", ctx_id );
    }
}


/* ----------------------------------------------------------------
 * V4L2 external interface for probe
 */
#if 0
int isp_v4l2_register_dma_channels(int ctx_id) {

    isp_v4l2_dev_t *dev = NULL;
    char dma_names[64];

    if( g_isp_v4l2_devs[ctx_id] == NULL) {
		LOG( LOG_ERR, "no isp v4l2 device for context id %d", ctx_id);
		return -EINVAL;
    }

    dev = g_isp_v4l2_devs[ctx_id];

    if(!(dev->v4l2_dev)) {
		LOG( LOG_ERR, "V4l2 device is not registered yet");
		return -EINVAL;
    }

    snprintf(dma_names,sizeof(dma_names)-1,"vdma%d",ctx_id);

    LOG( LOG_INFO, "DMA channel request name is %s", dma_names);

    dev->dma = dma_request_slave_channel(dev->v4l2_dev->dev, dma_names);
    if (dev->dma == NULL) {
		LOG( LOG_ERR, "no VDMA channel found by name vdma %s", dma_names);
		return -ENODEV;
    }
    
    LOG( LOG_INFO, "dma chan id %d", dev->dma->chan_id);

    return 0;
}
#endif

int isp_v4l2_create_instance( struct v4l2_device *v4l2_dev )
{
    int rc = 0;
    uint32_t ctx_id;

    if ( v4l2_dev == NULL ) {
        LOG( LOG_ERR, "Invalid parameter, v4l2_dev is NULL" );
        return -EINVAL;
    }

    /* initialize v4l2 layer devices */
    for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
        rc = isp_v4l2_init_dev( ctx_id, v4l2_dev );
        if ( rc ) {
            LOG( LOG_ERR, "isp_v4l2_init_dev for context id: %d failed.", ctx_id );
            goto unreg_dev;
        }

#if 0
		rc = isp_v4l2_register_dma_channels(ctx_id);
        if ( rc ) {
            LOG( LOG_ERR, "dma channel registration for context id: %d failed.",
							ctx_id );
            goto unreg_dev;
        }
#endif

    }

    /* initialize stream related resources to prepare for streaming.
     * It should be called after sensor initialized.
     */
    for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
        rc = isp_v4l2_stream_init_static_resources( ctx_id );
        if ( rc < 0 )
            goto unreg_dev;
    }

    return 0;

unreg_dev:
    for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
        isp_v4l2_destroy_dev( ctx_id );
    }

    return rc;
}

void isp_v4l2_destroy_instance( void )
{
    LOG( LOG_INFO, "%s: E", __func__ );

    int ctx_id;

    for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
        isp_v4l2_destroy_dev( ctx_id );
    }

    LOG( LOG_INFO, "%s: X", __func__ );
}


/* ----------------------------------------------------------------
 * stream finder utility function
 */
int isp_v4l2_find_stream( isp_v4l2_stream_t **ppstream,
                          int ctx_id, isp_v4l2_stream_type_t stream_type )
{
    *ppstream = NULL;

    if ( stream_type >= V4L2_STREAM_TYPE_MAX || stream_type < 0 || ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        return -EINVAL;
    }

    if ( g_isp_v4l2_devs[ctx_id] == NULL ) {
        return -EBUSY;
    }

    // Check if m2m node is opened to route raw and output stream requests
    if ( test_bit( V4L2_STREAM_TYPE_M2M, &g_isp_v4l2_devs[ctx_id]->stream_open_mask ) != 0 ) {
        if ( ( stream_type == V4L2_STREAM_TYPE_RAW ) || ( stream_type == V4L2_STREAM_TYPE_OUT ) ) {
            stream_type = V4L2_STREAM_TYPE_M2M;
        }
    }

    if ( g_isp_v4l2_devs[ctx_id]->pstreams[stream_type] == NULL ) {
        return -ENODEV;
    }

    *ppstream = g_isp_v4l2_devs[ctx_id]->pstreams[stream_type];

    return 0;
}

isp_v4l2_dev_t *isp_v4l2_get_dev( uint32_t ctx_number )
{
    return g_isp_v4l2_devs[ctx_number];
}

/* ----------------------------------------------------------------
 * event notifier utility function
 */
int isp_v4l2_notify_event( int ctx_id, int stream_type, aframe_t *frame, uint32_t event_type, isp_v4l2_stream_direction_t stream_direction )
{
    struct v4l2_event event;

    if ( g_isp_v4l2_devs[ctx_id] == NULL ) {
        return -EBUSY;
    }

    if ( g_isp_v4l2_devs[ctx_id]->pstreams[stream_type] == NULL ) {
        LOG( LOG_ERR, "Error, stream does not exist, context id: %d, stream type: %d, event type: %d", ctx_id, stream_type, event_type );
        return -EINVAL;
    }

    memset( &event, 0, sizeof( event ) );
    event.type = event_type;
    event.id = stream_direction;

    // Event specific payload
    if ( event_type == V4L2_EVENT_ACAMERA_FRAME_READY ) {
        // Check if event payload data fits into v4l2_event.u.data element
        if ( sizeof( isp_v4l2_event_frame_ready_data_t ) > sizeof( ( (struct v4l2_event *)0 )->u.data ) ) {
            LOG( LOG_ERR, "Error, isp_v4l2_event_frame_ready_data_t does not fit into v4l2_event.u.data" );
            return -EINVAL;
        }

        // Set event payload
        isp_v4l2_event_frame_ready_data_t *payload = (isp_v4l2_event_frame_ready_data_t *)event.u.data;
        payload->index = frame->frame_id;
        payload->sequence = frame->sequence;
        payload->num_planes = frame->num_planes;

        uint32_t i;
        for ( i = 0; i < frame->num_planes; i++ ) {
            payload->plane_crc[i] = frame->planes[i].crc;
        }
    }

    v4l2_event_queue_fh( &g_isp_v4l2_devs[ctx_id]->pstreams[stream_type]->fh, &event );

    return 0;
}
