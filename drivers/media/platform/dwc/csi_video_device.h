/*
 * Copyright (C) 2016 Synopsys, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIDEO_DEVICE_H_
#define VIDEO_DEVICE_H_

#include <linux/delay.h>
#include <linux/dma/xilinx_dma.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <media/dwc/csi_host_platform.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>


#define N_BUFFERS 5

#define VIDEO_DEVICE_NAME      "video-device"

#define FUNC_NAME __func__

struct rx_buffer {
       /** @short Buffer for video frames */
       struct vb2_v4l2_buffer vb;
       struct list_head list;

       dma_addr_t dma_addr;
       void *cpu_addr;
};

struct dmaqueue {
       struct list_head active;
       wait_queue_head_t wq;
};

/**
 * @short Structure to embed device driver information
 */
struct video_device_dev {
       struct platform_device *pdev;
       struct v4l2_device *v4l2_dev;
       struct v4l2_subdev subdev;
       struct media_pad vd_pad;
       struct media_pad subdev_pads[VIDEO_DEV_SD_PADS_NUM];
       struct mutex lock;
       spinlock_t slock;
       struct plat_csi_video_entity ve;
       struct v4l2_format format;
       struct v4l2_pix_format pix_format;
       const struct plat_csi_fmt *fmt;
       unsigned long *alloc_ctx;

       /* Buffer and DMA */
       struct vb2_queue vb_queue;
       int idx;
       int last_idx;
       struct dmaqueue vidq;
       struct rx_buffer dma_buf[N_BUFFERS];
       struct dma_chan *dma;
       struct dma_interleaved_template xt;
       struct data_chunk sgl[1];
};

#endif /* VIDEO_DEVICE_H_ */
