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

#ifndef _ISP_V4L2_STREAM_H_
#define _ISP_V4L2_STREAM_H_

#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/dmaengine.h>
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-fh.h>
#endif

#include "isp-v4l2-common.h"

/* buffer for one video frame */
typedef struct _isp_v4l2_buffer {
/* vb or vvb (depending on kernel version) must be first */
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 4, 0 ) )
    struct vb2_v4l2_buffer vvb;
#else
    struct vb2_buffer vb;
#endif
    struct aframe_t frame;
    struct list_head list;
} isp_v4l2_buffer_t;

/**
 * struct isp_v4l2_stream_common
 */
typedef struct _isp_v4l2_frame_sizes {
    /* resolution table for FR stream */
    struct v4l2_frmsize_discrete frmsize[V4L2_SENSOR_INFO_MODES_MAX]; // for now this is same since FR path
                                                                      // doesn't have downscaler block
    uint8_t frmsize_num;
} isp_v4l2_frame_sizes;

typedef struct _isp_v4l2_stream_common {
    isp_v4l2_sensor_info sensor_info;
    isp_v4l2_frame_sizes snapshot_sizes;
} isp_v4l2_stream_common;

typedef struct _isp_v4l2_stream_buffer_list {
    struct {
        struct list_head head;
        uint32_t size;
    } ready, busy; // VB2 buffers: ready - available to ISP (free), busy - acquired by ISP (busy)
    spinlock_t lock;
} isp_v4l2_stream_buffer_list;

typedef struct isp_v4l2_stream_parent_device_info_t {
    struct device *dev; // device associated with v4l2 device parent to the stream
    struct mutex *lock; // device mutex (isp_v4l2_dev_t->mlock)
} isp_v4l2_stream_parent_device_info_t;

/**
 * struct isp_v4l2_stream_t - All internal data for one instance of ISP
 */
typedef struct _isp_v4l2_stream_t {
    // File handle associated with the stream
    struct v4l2_fh fh;

    // Parent device information. Required to initialise vb2 queues
    isp_v4l2_stream_parent_device_info_t parent_device_info;

    /* Control fields */
    uint32_t ctx_id;
    isp_v4l2_stream_type_t stream_type;

    // Per direction and global stream started flags
    int stream_direction_started[V4L2_STREAM_DIRECTION_MAX];
    int stream_started;

    /* Input stream */
    isp_v4l2_stream_common *stream_common;

    /* Stream formats */
    struct v4l2_format cur_v4l2_fmt[V4L2_STREAM_DIRECTION_MAX];

    // Video buffer lists. Capture streams use ready and busy queues.
    // M2M streams use only busy queues, ready queue is managed by m2m context
    isp_v4l2_stream_buffer_list buffer_list[V4L2_STREAM_DIRECTION_MAX];

    // Number of buffers requested for the stream
    uint32_t num_buffers_requested;

    // VB2 queue for the stream ( capture mode streams only )
    struct vb2_queue vb2_q;
	// SIMA.AI specific field
	//int start_dma_async;
	//struct work_struct work;
    //struct dma_chan *dma;
    //struct dma_interleaved_template xt;
    //struct data_chunk sgl[1];
} isp_v4l2_stream_t;


/* stream control interface */
int isp_v4l2_stream_init_static_resources( uint32_t ctx_id );
int isp_v4l2_stream_init( isp_v4l2_stream_t **ppstream, int stream_type, int ctx_num );
void isp_v4l2_stream_deinit( isp_v4l2_stream_t *pstream, unsigned long stream_on_mask );
void isp_v4l2_stream_free( isp_v4l2_stream_t *pstream );
int isp_v4l2_stream_on( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, unsigned long stream_open_mask );
int isp_v4l2_stream_off( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, unsigned long stream_on_mask );

/* stream configuration interface */
int isp_v4l2_stream_enum_framesizes( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_frmsizeenum *fsize );
int isp_v4l2_stream_enum_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_fmtdesc *f );
int isp_v4l2_stream_try_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_format *f );
int isp_v4l2_stream_get_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_format *f );
int isp_v4l2_stream_set_format( isp_v4l2_stream_t *pstream, isp_v4l2_stream_direction_t stream_direction, struct v4l2_format *f );

#endif
