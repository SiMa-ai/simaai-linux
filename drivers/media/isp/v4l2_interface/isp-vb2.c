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

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "acamera_logger.h"
#include "isp-v4l2-common.h"
#include "isp-v4l2-stream.h"
#include "isp-vb2.h"

/* ----------------------------------------------------------------
 * VB2 operations
 */

#define MIPI_FIR_REGISTER_SIZE (64)
#define ADDITIONAL_VB2_BUFFER_COUNT (15)

#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 ) )
static int isp_vb2_queue_setup( struct vb2_queue *vq,
                                unsigned int *nbuffers, unsigned int *nplanes,
                                unsigned int sizes[], struct device *alloc_devs[] )
#else
static int isp_vb2_queue_setup( struct vb2_queue *vq, const struct v4l2_format *fmt,
                                unsigned int *nbuffers, unsigned int *nplanes,
                                unsigned int sizes[], void *alloc_ctxs[] )
#endif
{
    static unsigned long cnt = 0;
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vq );
    struct v4l2_format vfmt;

    *nbuffers += ADDITIONAL_VB2_BUFFER_COUNT;

    LOG( LOG_INFO, "Setting up vb2 queue for stream type: %d, nplanes: %u, nbuffers: %u, fcall#: %lu.",
         pstream->stream_type, *nplanes, *nbuffers, cnt++ );
    LOG( LOG_INFO, "VB2 queue details, vq ptr: %p, vq->num_buffers: %u, vq->type: %u",
         vq, vq->num_buffers, vq->type );

    // Get stream direction based on the queue type
    isp_v4l2_stream_direction_t stream_direction;
    if ( V4L2_TYPE_IS_OUTPUT( vq->type ) ) {
        stream_direction = V4L2_STREAM_DIRECTION_OUT;
    } else {
        stream_direction = V4L2_STREAM_DIRECTION_CAP;
    }

    // Get current format based on the stream type and direction
    if ( isp_v4l2_stream_get_format( pstream, stream_direction, &vfmt ) < 0 ) {
        LOG( LOG_ERR, "Error. Failed to get stream format" );
        return -EBUSY;
    }

#if ( LINUX_VERSION_CODE < KERNEL_VERSION( 4, 8, 0 ) )
    if ( fmt )
        LOG( LOG_INFO, "Requested format, fmt: %p, width: %u, height: %u, sizeimage: %u.",
             fmt, fmt->fmt.pix.width, fmt->fmt.pix.height, fmt->fmt.pix.sizeimage );
#endif

    if ( ( vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE ) || ( vfmt.type == V4L2_BUF_TYPE_VIDEO_OUTPUT ) ) {
        *nplanes = 1;
        sizes[0] = vfmt.fmt.pix.sizeimage;
        LOG( LOG_INFO, "Effective nplanes: %u, size: %u", *nplanes, sizes[0] );
    } else if ( V4L2_TYPE_IS_MULTIPLANAR( vfmt.type ) ) {
        *nplanes = vfmt.fmt.pix_mp.num_planes;
        LOG( LOG_INFO, "Effective nplanes: %u", *nplanes );
        int i;
        for ( i = 0; i < vfmt.fmt.pix_mp.num_planes; i++ ) {
            sizes[i] = vfmt.fmt.pix_mp.plane_fmt[i].sizeimage;
            LOG( LOG_INFO, "  |--Plane %d. Effective size: %u", i, sizes[i] );
#if ( LINUX_VERSION_CODE < KERNEL_VERSION( 4, 8, 0 ) )
            alloc_ctxs[i] = 0;
#endif
        }
    } else {
        LOG( LOG_ERR, "Error. Wrong v4l2 buffer type supplied: %u", vfmt.type );
        return -EINVAL;
    }

    return 0;
}

static int isp_vb2_buf_prepare( struct vb2_buffer *vb )
{
    unsigned long size;
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vb->vb2_queue );
    static unsigned long cnt = 0;
    struct v4l2_format vfmt;

    LOG( LOG_INFO, "Preparing VB2 buffer for stream type: %d, queue type: %u, fcall#: %lu.",
         pstream->stream_type, vb->vb2_queue->type, cnt++ );

    // Get stream direction based on the queue type
    isp_v4l2_stream_direction_t stream_direction;
    if ( V4L2_TYPE_IS_OUTPUT( vb->vb2_queue->type ) ) {
        stream_direction = V4L2_STREAM_DIRECTION_OUT;
    } else {
        stream_direction = V4L2_STREAM_DIRECTION_CAP;
    }

    // Get current format based on the stream type and direction
    if ( isp_v4l2_stream_get_format( pstream, stream_direction, &vfmt ) < 0 ) {
        LOG( LOG_ERR, "Error. Failed to get stream format" );
        return -EBUSY;
    }

    if ( ( vfmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE ) || ( vfmt.type == V4L2_BUF_TYPE_VIDEO_OUTPUT ) ) {
        size = vfmt.fmt.pix.sizeimage;

        if ( vb2_plane_size( vb, 0 ) < size ) {
            LOG( LOG_ERR, "Error. Supplied buffer size (%lu) is too small, required at least: %lu", vb2_plane_size( vb, 0 ), size );
            return -EINVAL;
        }

        vb2_set_plane_payload( vb, 0, size );
        LOG( LOG_INFO, "VB2 buffer is single plane, effective payload size: %lu", size );
    } else if ( V4L2_TYPE_IS_MULTIPLANAR( vfmt.type ) ) {
        LOG( LOG_INFO, "VB2 buffer is multi plane:" );
        uint8_t i;
        for ( i = 0; i < vfmt.fmt.pix_mp.num_planes; i++ ) {
            size = vfmt.fmt.pix_mp.plane_fmt[i].sizeimage;
            if ( vb2_plane_size( vb, i ) < size ) {
                LOG( LOG_ERR, "  |--Plane %d. Supplied buffer size (%lu) is too small, required at least: %lu",
                     i, vb2_plane_size( vb, i ), size );
                return -EINVAL;
            }
            vb2_set_plane_payload( vb, i, size );
            LOG( LOG_INFO, "  |--Plane %d. Effective payload size: %lu", i, size );
        }
    }

    return 0;
}

#if 0
static void buffer_transfer_done(void *param) {

    isp_v4l2_stream_t *pstream = (isp_v4l2_stream_t *)param;

    //LOG( LOG_ERR, "MIPI buffer transfer done !!! channel %d", pstream->dma->chan_id);
#if 0
    int process = 0;

    
    spin_lock(&(pstream->vb2_stream_lock));
    if (pstream->vb2_queue_on) {
		process = 1;
	}
    spin_unlock(&(pstream->vb2_stream_lock));

    if (process) {
        LOG( LOG_DEBUG, "MIPI buffer process submit ctx: %d", pstream->ctx_id);	    
        fw_intf_process_input( pstream->ctx_id);
    } else {
        LOG( LOG_WARNING, "Drop MIPI buffer from process ctx: %d", pstream->ctx_id);
    }
#endif

	if (!schedule_work(&pstream->work)) {
		LOG( LOG_ERR, "Work is already scheduled !!!!");
	}

    return;
}

static int submit_dma_request(struct vb2_buffer *vb) {

    LOG( LOG_INFO, "MIPI submitting dma request");

    struct dma_async_tx_descriptor *desc;
    u32 flags;
    int32_t cookie = -1;
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vb->vb2_queue );

    flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
    pstream->xt.dir = DMA_DEV_TO_MEM;
    pstream->xt.dst_start = vb2_dma_contig_plane_dma_addr(vb, 0);
    LOG (LOG_INFO, "RAW : DMA address is %#llx", pstream->xt.dst_start);

    LOG (LOG_INFO, "Pixel width %d , height %d , pixel bits %d \n", 
		    pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].fmt.pix.width,
		    pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].fmt.pix.height,
		    pstream->pixelbits);

    pstream->xt.src_sgl = false;
    pstream->xt.dst_inc = false;
    pstream->xt.dst_sgl = true;
    pstream->xt.frame_size = 1;
    pstream->xt.sgl[0].size = 
	    ((pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].fmt.pix.width * pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].fmt.pix.height * pstream->pixelbits) / 8 + MIPI_FIR_REGISTER_SIZE);
    pstream->xt.sgl[0].icg = 0;
    pstream->xt.numf = pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].fmt.pix.height;

    desc = dmaengine_prep_interleaved_dma(pstream->dma, &pstream->xt, flags);
    if ( !desc) {
		LOG( LOG_ERR, "ERROR : creating descriptor\n");
		return -EINVAL;
    }

    desc->callback = buffer_transfer_done;
    desc->callback_param = pstream;

    cookie = dmaengine_submit(desc);
    LOG( LOG_INFO, "cookie submitted is %d", cookie);

    if (vb2_is_streaming(vb->vb2_queue) && pstream->start_dma_async) {
       LOG( LOG_INFO, "DMA async issue pending cookie:  %d", cookie);	    
       dma_async_issue_pending(pstream->dma);
    }

    LOG( LOG_INFO, "SUCCESS : MIPI submitting buffer");

    return 0;
}

#endif

static void isp_vb2_buf_queue( struct vb2_buffer *vb )
{
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vb->vb2_queue );
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer( vb );
    isp_v4l2_buffer_t *buf = container_of( vvb, isp_v4l2_buffer_t, vvb );
#else
    isp_v4l2_buffer_t *buf = container_of( vb, isp_v4l2_buffer_t, vb );
#endif
    static unsigned long cnt = 0;

    LOG( LOG_INFO, "Queueing buffer for stream type: %d, queue type: %u, fcall#: %lu.",
         pstream->stream_type, vb->vb2_queue->type, cnt++ );

#if 0
    //First buffer is used for creating bufset in ISP at stream start, but it doesn't
    //have MIPI DMA data by then
    if (pstream->stream_type == V4L2_STREAM_TYPE_RAW) {    
       LOG( LOG_INFO, "submit MIPI dma channel is %d", pstream->dma->chan_id);
       submit_dma_request(vb);	
       LOG( LOG_INFO, "SUCCESS : submit DMA Done");
    } else {
	  LOG (LOG_INFO, "OUT : DMA address is %#llx", vb2_dma_contig_plane_dma_addr(vb, 0));
    }
#endif

	LOG (LOG_INFO, "DMA address is %#llx", vb2_dma_contig_plane_dma_addr(vb, 0));


    // Capture stream VB2 buffers go to stream ready queue
    // M2M stream VB2 buffers go to m2m context internal ready queue
    if ( pstream->stream_type == V4L2_STREAM_TYPE_M2M ) {
        v4l2_m2m_buf_queue( pstream->fh.m2m_ctx, vvb );
    } else {
        spin_lock( &pstream->buffer_list[V4L2_STREAM_DIRECTION_CAP].lock );
        list_add_tail( &buf->list, &pstream->buffer_list[V4L2_STREAM_DIRECTION_CAP].ready.head );
        pstream->buffer_list[V4L2_STREAM_DIRECTION_CAP].ready.size++;
        spin_unlock( &pstream->buffer_list[V4L2_STREAM_DIRECTION_CAP].lock );
    }
}

#if 0
static int isp_start_streaming(struct vb2_queue *vq, unsigned int count) {

    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vq );

    LOG( LOG_INFO, "VB2 : Start streaming stream type %d", pstream->stream_type);   
    
    spin_lock(&(pstream->vb2_stream_lock));
    pstream->vb2_queue_on = 1;
    spin_unlock(&(pstream->vb2_stream_lock));

    LOG( LOG_INFO, "VB2 : Done Start streaming stream type %d", pstream->stream_type);

    return 0;
}

void isp_vb2_issue_dma_async_pending(isp_v4l2_stream_t *pstream)
{
   LOG( LOG_INFO, "VB2 : DMA async pending stream type %d", pstream->stream_type);   	
   
   if ((pstream != NULL) && (pstream->stream_type == V4L2_STREAM_TYPE_RAW)) { 
       LOG( LOG_INFO, "VB2 : Issue DMA async pending");
       spin_lock(&(pstream->vb2_stream_lock));       
       dma_async_issue_pending(pstream->dma);
       pstream->start_dma_async = 1;
       spin_unlock(&(pstream->vb2_stream_lock));
   }

   return;
}

static void isp_stop_streaming(struct vb2_queue *vq) {
    isp_v4l2_stream_t *pstream = vb2_get_drv_priv( vq );

    LOG( LOG_INFO, "VB2 : Stop streaming stream type %d",  pstream->stream_type);

    spin_lock(&(pstream->vb2_stream_lock));
    pstream->vb2_queue_on = 0;
    spin_unlock(&(pstream->vb2_stream_lock));
   
    if (pstream->stream_type == V4L2_STREAM_TYPE_RAW) {
        LOG(LOG_INFO, "VB2: initiate terminate DMA transfer");
		dmaengine_terminate_sync(pstream->dma);
    }

    LOG( LOG_INFO, "VB2 : Done Stop streaming stream type %d", pstream->stream_type);    
}
#endif

static const struct vb2_ops isp_vb2_ops = {
    .queue_setup = isp_vb2_queue_setup, // called from VIDIOC_REQBUFS
    .buf_prepare = isp_vb2_buf_prepare,
    .buf_queue = isp_vb2_buf_queue,
    //.start_streaming = isp_start_streaming,
    //.stop_streaming = isp_stop_streaming,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};


/* ----------------------------------------------------------------
 * VB2 external interface for isp-v4l2
 */
int isp_vb2_cap_queue_init( struct vb2_queue *q, isp_v4l2_stream_t *pstream )
{
    memset( q, 0, sizeof( struct vb2_queue ) );

    /* start creating the vb2 queues */

    q->type = pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].type;

    LOG( LOG_INFO, "isp_vb2_cap_queue_init for stream type: %d, queue type: %u.", pstream->stream_type, q->type );

    q->io_modes = VB2_MMAP | VB2_READ | VB2_USERPTR;
    q->drv_priv = pstream;
    q->buf_struct_size = sizeof( isp_v4l2_buffer_t );

    q->ops = &isp_vb2_ops;
    if ( pstream->stream_type == V4L2_STREAM_TYPE_META ) {
        //q->mem_ops = &vb2_vmalloc_memops;
        q->mem_ops = &vb2_dma_contig_memops;
    } else {
        q->mem_ops = &vb2_dma_contig_memops;
    }
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->min_buffers_needed = 1;
    q->lock = pstream->parent_device_info.lock;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 ) )
    q->dev = pstream->parent_device_info.dev;
#endif
    //pstream->start_dma_async = 0;
    //pstream->vb2_queue_on = 0;

    return vb2_queue_init( q );
}

int isp_vb2_m2m_queue_init( void *priv, struct vb2_queue *out_vq, struct vb2_queue *cap_vq )
{
    memset( cap_vq, 0, sizeof( struct vb2_queue ) );
    memset( out_vq, 0, sizeof( struct vb2_queue ) );

    isp_v4l2_stream_t *pstream = priv;
    int ret;

    /* start creating the vb2 queues */
    // ISP input stream, data to process
    out_vq->type = pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_OUT].type;

    LOG( LOG_INFO, "isp_vb2_m2m_queue_init for stream type: %d, direction: %d, queue type: %u.",
         pstream->stream_type, V4L2_STREAM_DIRECTION_OUT, out_vq->type );

    out_vq->io_modes = VB2_MMAP | VB2_READ | VB2_USERPTR;
    out_vq->drv_priv = pstream;
    out_vq->buf_struct_size = sizeof( isp_v4l2_buffer_t );

    out_vq->ops = &isp_vb2_ops;
    out_vq->mem_ops = &vb2_dma_contig_memops;
    out_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    out_vq->min_buffers_needed = 1;
    out_vq->lock = pstream->parent_device_info.lock;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 ) )
    out_vq->dev = pstream->parent_device_info.dev;
#endif

    ret = vb2_queue_init( out_vq );
    if ( ret ) {
        return ret;
    }

    // ISP output stream, processed data
    cap_vq->type = pstream->cur_v4l2_fmt[V4L2_STREAM_DIRECTION_CAP].type;

    LOG( LOG_INFO, "isp_vb2_m2m_queue_init for stream type: %d, direction: %d, queue type: %u.",
         pstream->stream_type, V4L2_STREAM_DIRECTION_CAP, cap_vq->type );

    cap_vq->io_modes = VB2_MMAP | VB2_READ | VB2_USERPTR;
    cap_vq->drv_priv = pstream;
    cap_vq->buf_struct_size = sizeof( isp_v4l2_buffer_t );

    cap_vq->ops = &isp_vb2_ops;
    cap_vq->mem_ops = &vb2_dma_contig_memops;
    cap_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    cap_vq->min_buffers_needed = 1;
    cap_vq->lock = pstream->parent_device_info.lock;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 ) )
    cap_vq->dev = pstream->parent_device_info.dev;
#endif

    return vb2_queue_init( cap_vq );
}

void isp_vb2_queue_release( struct vb2_queue *q )
{
    vb2_queue_release( q );
}
