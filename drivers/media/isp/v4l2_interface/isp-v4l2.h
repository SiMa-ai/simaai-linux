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

#ifndef _ISP_V4L2_H_
#define _ISP_V4L2_H_

#include <linux/mutex.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-async.h>
#include <linux/of_platform.h> 
#include <media/v4l2-fwnode.h>
#include <linux/of_graph.h>
#include <linux/dmaengine.h>

#include "acamera_aframe.h"

#include "isp-v4l2-common.h"
#include "isp-v4l2-ctrl.h"
#include "isp-v4l2-stream.h"

enum ISP_SUBDEVS {
	SD_CAMERA,
	SD_MIPI_CSI,
	SD_MAX
};

enum MIPI_CONTROLERS {
	MIPI_CONTROLER_0,
	MIPI_CONTROLER_1,
	MIPI_CONTROLER_2,
	MIPI_CONTROLER_3,
	MIPI_CONTROLLER_MAX
};


typedef struct _isp_v4l2_dev {
    /* device */
    uint32_t ctx_id;
    uint8_t v4l2_interface_mode;
    struct v4l2_device *v4l2_dev;
    struct v4l2_m2m_dev *v4l2_m2m_dev;
    struct video_device video_dev[V4L2_STREAM_TYPE_MAX];

    /* lock */
    struct mutex mlock;

    /* streams */
    isp_v4l2_stream_t *pstreams[V4L2_STREAM_TYPE_MAX];

    /* controls */
    isp_v4l2_ctrl_t isp_v4l2_ctrl;

    /* open counter and stream masks for stream type */
    atomic_t opened;
    volatile unsigned long stream_on_mask;
    volatile unsigned long stream_open_mask;

	//SIMA.AI specific
	struct v4l2_async_notifier subdev_notifier;
	//struct dma_chan *dma;
	struct v4l2_async_subdev asd[SD_MAX];
	struct v4l2_subdev *sd[SD_MAX];
} isp_v4l2_dev_t;


/* V4L2 external interface for probe */
int isp_v4l2_create_instance( struct v4l2_device *v4l2_dev );
void isp_v4l2_destroy_instance( void );

/* Stream finder */
int isp_v4l2_find_stream( isp_v4l2_stream_t **ppstream,
                          int ctx_id, isp_v4l2_stream_type_t stream_type );
isp_v4l2_dev_t *isp_v4l2_get_dev( uint32_t ctx_id );

/* Stream frame get/put functions */
int isp_v4l2_stream_get_frame( const unsigned int ctx_id, const aframe_type_t type, const aframe_state_t state, aframe_t **frame );
int isp_v4l2_stream_put_frame( aframe_t *frame );

/* Frame ready event */
int isp_v4l2_notify_event( int ctx_num, int stream_type, aframe_t *frame, uint32_t event_type, isp_v4l2_stream_direction_t stream_direction );

#endif
