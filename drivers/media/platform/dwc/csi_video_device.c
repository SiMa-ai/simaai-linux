/*
 * CSI-2 Video platform video device driver
 *
 * Copyright (C) 2016 Synopsys, Inc. All rights reserved.
 * Author: Ramiro Oliveira <ramiro.olive...@synopsys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/of_reserved_mem.h>
#include "dw_mipi_csi.h"
#include "csi_video_device.h"

static const struct plat_csi_fmt vid_dev_formats[] = {
       {
               .name = "RGB888",
               .fourcc = V4L2_PIX_FMT_RGB24,
               .depth = 24,
               .mbus_code = MEDIA_BUS_FMT_RGB888_1X24,
       },{
               .name = "RAW8",
               .fourcc = V4L2_PIX_FMT_SRGGB8,
               .depth = 8,
               .mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
       },{
               .name = "RAW10",
               .fourcc = V4L2_PIX_FMT_SRGGB10,
               .depth = 10,
               .mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
       },{
               .name = "RAW12",
               .fourcc = V4L2_PIX_FMT_SRGGB12,
               .depth = 12,
               .mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
       },{
               .name = "RAW14",
               .fourcc = V4L2_PIX_FMT_SRGGB14,
               .depth = 14,
               .mbus_code = MEDIA_BUS_FMT_SBGGR14_1X14,
       },{
               .name = "RAW16",
               .fourcc = V4L2_PIX_FMT_SRGGB16,
               .depth = 16,
               .mbus_code = MEDIA_BUS_FMT_SRGGB16_1X16,
       },{
               .name = "YUV422",
               .fourcc = V4L2_PIX_FMT_YUYV,
               .depth = 16,
               .mbus_code = MEDIA_BUS_FMT_VYUY8_1X16,
       },{
               .name = "YUV420",
               .fourcc = V4L2_PIX_FMT_YUV420,
               .depth = 12,
               .mbus_code = MEDIA_BUS_FMT_Y8_1X8,
       }
};

static const struct plat_csi_fmt *vid_dev_find_format(struct v4l2_format *f)
{
       const struct plat_csi_fmt *fmt = NULL;
       unsigned int i;

       for (i = 0; i < ARRAY_SIZE(vid_dev_formats); ++i) {
               fmt = &vid_dev_formats[i];
               if (fmt->fourcc == f->fmt.pix.pixelformat)
                       return fmt;
       }
       return NULL;
}

/*
 * Video node ioctl operations
 */
static int
vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
       struct video_device_dev *vid_dev = video_drvdata(file);

       strlcpy(cap->driver, VIDEO_DEVICE_NAME, sizeof(cap->driver));
       strlcpy(cap->card, VIDEO_DEVICE_NAME, sizeof(cap->card));
       snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
                dev_name(&vid_dev->pdev->dev));
       return 0;
}

static int
vidioc_enum_fmt_vid_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
       const struct plat_csi_fmt *p_fmt;
	struct video_device_dev *dev = video_drvdata(file);

       if (f->index >= ARRAY_SIZE(vid_dev_formats))
               return -EINVAL;

	dev_err(&dev->pdev->dev, "INFO : %s\n", __FUNCTION__);
       p_fmt = &vid_dev_formats[f->index];

       f->pixelformat = p_fmt->fourcc;

       return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
                                       struct v4l2_format *f)
{
        struct video_device_dev *dev = video_drvdata(file);
	f->fmt.pix = dev->format.fmt.pix;
	return 0;
}

static int
vidioc_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct plat_csi_fmt *fmt;

	fmt = vid_dev_find_format(f);

	if (!fmt) {
                f->fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB8;
		fmt = vid_dev_find_format(f);
	}

	f->fmt.pix.field = V4L2_FIELD_NONE;
	v4l_bound_align_image(&f->fmt.pix.width, 48, MAX_WIDTH, 2,
                             &f->fmt.pix.height, 32, MAX_HEIGHT, 0, 0);

	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline + 64;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
                                       struct v4l2_format *f)
{
       struct video_device_dev *dev = video_drvdata(file);
       int ret;
       struct v4l2_subdev_format fmt;
       struct v4l2_pix_format *dev_fmt_pix = &dev->format.fmt.pix;

       if (vb2_is_busy(&dev->vb_queue))
               return -EBUSY;

       ret = vidioc_try_fmt_vid_cap(file, dev, f);
       if (ret)
               return ret;

       dev->fmt = vid_dev_find_format(f);
       dev_fmt_pix->pixelformat = f->fmt.pix.pixelformat;
       dev_fmt_pix->width = f->fmt.pix.width;
       dev_fmt_pix->height  = f->fmt.pix.height;
       dev_fmt_pix->bytesperline = (dev_fmt_pix->width * dev->fmt->depth) >> 3;
       dev_fmt_pix->sizeimage =
                       dev_fmt_pix->height * dev_fmt_pix->bytesperline + 64;

       fmt.format.colorspace = V4L2_COLORSPACE_SRGB;
       fmt.format.code = dev->fmt->mbus_code;

       fmt.format.width = dev_fmt_pix->width;
       fmt.format.height = dev_fmt_pix->height;

       ret = plat_csi_pipeline_call(&dev->ve, set_format, &fmt);

       return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
                      struct v4l2_frmsizeenum *fsize)
{
        static const struct v4l2_frmsize_stepwise sizes = {
                48, MAX_WIDTH, 4,
                32, MAX_HEIGHT, 1
        };
        int i;

        if (fsize->index)
                return -EINVAL;

        for (i = 0; i < ARRAY_SIZE(vid_dev_formats); i++)
                if (vid_dev_formats[i].fourcc == fsize->pixel_format)
                        break;
        if (i == ARRAY_SIZE(vid_dev_formats))
                return -EINVAL;
        fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
        fsize->stepwise = sizes;
        return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,
                       struct v4l2_input *input)
{
        if (input->index != 0)
                return -EINVAL;
        input->type = V4L2_INPUT_TYPE_CAMERA;
        input->std = 0;
        strcpy(input->name, "Camera");
        return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
       *i = 0;
       return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
       if (i != 0)
               return -EINVAL;
       return 0;
}

static int
vid_dev_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
        struct video_device_dev *vid_dev = video_drvdata(file);
        struct media_entity *entity = &vid_dev->ve.vdev.entity;
        int ret;

        ret = media_pipeline_start(entity->pads, &vid_dev->ve.pipe->mp);
        if (ret < 0)
                return ret;

        ret = vb2_ioctl_streamon(file, priv, type);
        if (!ret)
                return ret;

        media_pipeline_stop(entity->pads);
        return 0;
}

static int
vid_dev_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
        int ret;

        ret = vb2_ioctl_streamoff(file, priv, type);
        if (ret < 0)
                return ret;

        return 0;
}

static const struct v4l2_ioctl_ops vid_dev_ioctl_ops = {
        .vidioc_querycap = vidioc_querycap,
        .vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
        .vidioc_g_fmt_vid_cap = vidioc_g_fmt_vid_cap,
        .vidioc_s_fmt_vid_cap = vidioc_s_fmt_vid_cap,
        .vidioc_try_fmt_vid_cap = vidioc_try_fmt_vid_cap,
        .vidioc_enum_framesizes = vidioc_enum_framesizes,
        .vidioc_enum_input = vidioc_enum_input,
        .vidioc_g_input = vidioc_g_input,
        .vidioc_s_input = vidioc_s_input,

        .vidioc_reqbufs = vb2_ioctl_reqbufs,
        .vidioc_create_bufs = vb2_ioctl_create_bufs,
        .vidioc_prepare_buf = vb2_ioctl_prepare_buf,
        .vidioc_querybuf = vb2_ioctl_querybuf,
        .vidioc_qbuf = vb2_ioctl_qbuf,
        .vidioc_dqbuf = vb2_ioctl_dqbuf,
        .vidioc_streamon = vid_dev_streamon,
        .vidioc_streamoff = vid_dev_streamoff,
};

static int
vid_dev_open(struct file *file)
{
       struct video_device_dev *vid_dev = video_drvdata(file);
       struct media_entity *me = &vid_dev->ve.vdev.entity;
       int ret;

       mutex_lock(&vid_dev->lock);

       ret = v4l2_fh_open(file);
       if (ret < 0)
               goto unlock;

       if (!v4l2_fh_is_singular_file(file))
               goto unlock;

       mutex_lock(&me->graph_obj.mdev->graph_mutex);

       ret = plat_csi_pipeline_call(&vid_dev->ve, open, me, true);
       if (ret == 0)
               me->use_count++;

       mutex_unlock(&me->graph_obj.mdev->graph_mutex);

       if (!ret)
               goto unlock;

       v4l2_fh_release(file);
unlock:
       mutex_unlock(&vid_dev->lock);
       return ret;
}

static int
vid_dev_release(struct file *file)
{
       struct video_device_dev *vid_dev = video_drvdata(file);
       struct media_entity *entity = &vid_dev->ve.vdev.entity;

       mutex_lock(&vid_dev->lock);

       if (v4l2_fh_is_singular_file(file)) {
               plat_csi_pipeline_call(&vid_dev->ve, close);
               mutex_lock(&entity->graph_obj.mdev->graph_mutex);
               entity->use_count--;
               mutex_unlock(&entity->graph_obj.mdev->graph_mutex);
       }

       _vb2_fop_release(file, NULL);

       mutex_unlock(&vid_dev->lock);
       return 0;
}

static const struct v4l2_file_operations vid_dev_fops = {
       .owner = THIS_MODULE,
       .open = vid_dev_open,
       .release = vid_dev_release,
       .write = vb2_fop_write,
       .read = vb2_fop_read,
       .poll = vb2_fop_poll,
       .unlocked_ioctl = video_ioctl2,
       .mmap = vb2_fop_mmap,
};

/*
 * VideoBuffer2 operations
 */
#ifdef CONFIG_VIDEO_DWC_DMA_CONTIG
void fill_buffer(struct video_device_dev *dev, struct rx_buffer *buf,
                       int buf_num, unsigned long flags)
{
       buf->vb.field = dev->format.fmt.pix.field;
       buf->vb.sequence++;
       buf->vb.vb2_buf.timestamp = ktime_get_ns();
       vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
                       dev->format.fmt.pix.bytesperline*dev->format.fmt.pix.height);
       vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

}
#endif

#ifdef CONFIG_VIDEO_DWC_VMALLOC
static void fill_buffer(struct video_device_dev *dev, struct rx_buffer *buf,
                       int buf_num, unsigned long flags)
{
       int size = 0;
       void *vbuf = NULL;

       if (&buf->vb == NULL)
               return;

       size = vb2_plane_size(&buf->vb.vb2_buf, 0);
       vbuf = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);

       if (vbuf) {
               spin_unlock_irqrestore(&dev->slock, flags);

               memcpy(vbuf, dev->dma_buf[buf_num].cpu_addr, size);

               spin_lock_irqsave(&dev->slock, flags);

               buf->vb.field = dev->format.fmt.pix.field;
               buf->vb.sequence++;
               buf->vb.vb2_buf.timestamp = ktime_get_ns();
       }
       vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}
#endif

static void buffer_copy_process(void *param)
{
       struct video_device_dev *dev = (struct video_device_dev *) param;
       unsigned long flags;
       struct dmaqueue *dma_q = &dev->vidq;
       struct rx_buffer *buf = NULL;

       spin_lock_irqsave(&dev->slock, flags);

       if (!list_empty(&dma_q->active)) {
               buf = list_entry(dma_q->active.next, struct rx_buffer, list);
               list_del(&buf->list);
               fill_buffer(dev, buf, dev->last_idx, flags);
       }

       spin_unlock_irqrestore(&dev->slock, flags);
}

static inline struct rx_buffer *to_rx_buffer(struct vb2_v4l2_buffer *vb2)
{
       return container_of(vb2, struct rx_buffer, vb);
}

static int queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
                       unsigned int *nplanes, unsigned int sizes[],
                       struct device *alloc_devs[])
{
       struct video_device_dev *dev = vb2_get_drv_priv(vq);
       unsigned long size = 0;
#ifdef CONFIG_VIDEO_DWC_VMALLOC
       int i;
#endif

	size = dev->format.fmt.pix.sizeimage + 64;
	if (dev->format.fmt.pix.sizeimage  == 0) {
		dev_err(&dev->pdev->dev, "Configured Image size is zero\n");
		return -EINVAL;
	}
       *nbuffers = N_BUFFERS;
#ifdef CONFIG_VIDEO_DWC_VMALLOC
       for (i = 0; i < N_BUFFERS; i++) {
               dev->dma_buf[i].cpu_addr = dma_alloc_coherent(&dev->pdev->dev,
                               dev->format.fmt.pix.sizeimage,
                               &dev->dma_buf[i].dma_addr,
                               GFP_KERNEL);
       }
#endif
       *nplanes = 1;
       sizes[0] = size;
       return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
       struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
       struct rx_buffer *buf = to_rx_buffer(vbuf);
#ifdef CONFIG_VIDEO_DWC_VMALLOC
       int size = 0;

       if (vb == NULL) {
               pr_warn("%s:vb2_buffer is null\n", FUNC_NAME);
               return 0;
       }

       buf = to_rx_buffer(vbuf);

       size = vb2_plane_size(&buf->vb.vb2_buf, 0);
       vb2_set_plane_payload(&buf->vb.vb2_buf, 0, size);
#endif
       INIT_LIST_HEAD(&buf->list);
       return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
       struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
       struct video_device_dev *dev = NULL;
       struct rx_buffer *buf = NULL;
       struct dmaqueue *vidq = NULL;
       struct dma_async_tx_descriptor *desc;
       u32 flags;

       if (vb == NULL) {
               pr_warn("%s:vb2_buffer is null\n", FUNC_NAME);
               return;
       }

       dev = vb2_get_drv_priv(vb->vb2_queue);
       buf = to_rx_buffer(vbuf);
       vidq = &dev->vidq;
       flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
       dev->xt.dir = DMA_DEV_TO_MEM;
       dev->xt.src_sgl = false;
       dev->xt.dst_inc = false;
       dev->xt.dst_sgl = true;
#ifdef CONFIG_VIDEO_DWC_DMA_CONTIG
       dev->xt.dst_start = vb2_dma_contig_plane_dma_addr(vb, 0);
#else
       dev->xt.dst_start = dev->dma_buf[dev->idx].dma_addr;
#endif
       dev->last_idx = dev->idx;
       dev->idx++;
       if (dev->idx >= N_BUFFERS)
               dev->idx = 0;

       dev->xt.frame_size = 1;
       dev->xt.sgl[0].size = dev->format.fmt.pix.sizeimage + 64;
       dev->xt.sgl[0].icg = 0;
       dev->xt.numf = dev->format.fmt.pix.height;

       desc = dmaengine_prep_interleaved_dma(dev->dma, &dev->xt, flags);
       if (!desc) {
               pr_err("Fail to prepare DMA transfer!!!\n");
               vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
               return;
       }

       desc->callback = buffer_copy_process;
       desc->callback_param = dev;

       spin_lock(&dev->slock);
       list_add_tail(&buf->list, &vidq->active);
       spin_unlock(&dev->slock);

       dmaengine_submit(desc);

       if (vb2_is_streaming(&dev->vb_queue))
               dma_async_issue_pending(dev->dma);
	dev_dbg(&dev->pdev->dev, "Buffer queue done!!!\n");
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{

	struct video_device_dev *dev = vb2_get_drv_priv(vq);

	dev_dbg(&dev->pdev->dev, "Start streaming\n");
	dma_async_issue_pending(dev->dma);
	plat_csi_pipeline_call(&dev->ve, set_stream, true);

	return 0;
}

static void stop_streaming(struct vb2_queue *vq)
{
       struct video_device_dev *dev = vb2_get_drv_priv(vq);
       struct dmaqueue *dma_q = &dev->vidq;

	dev_dbg(&dev->pdev->dev, "Stop streaming\n");
	plat_csi_pipeline_call(&dev->ve, set_stream, false);

       /* Stop and reset the DMA engine. */
       dmaengine_terminate_all(dev->dma);

       while (!list_empty(&dma_q->active)) {
               struct rx_buffer *buf;

               buf = list_entry(dma_q->active.next, struct rx_buffer, list);
               if (buf) {
                       list_del(&buf->list);
                       vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
               }
       }
       list_del_init(&dev->vidq.active);
}

static const struct vb2_ops vb2_video_qops = {
       .queue_setup = queue_setup,
       .buf_prepare = buffer_prepare,
       .buf_queue = buffer_queue,
       .start_streaming = start_streaming,
       .stop_streaming = stop_streaming,
       .wait_prepare = vb2_ops_wait_prepare,
       .wait_finish = vb2_ops_wait_finish,
};

static int vid_dev_subdev_registered(struct v4l2_subdev *sd)
{
       struct video_device_dev *vid_dev = v4l2_get_subdevdata(sd);
       struct vb2_queue *q = &vid_dev->vb_queue;
       struct video_device *vfd = &vid_dev->ve.vdev;
       int ret;

       memset(vfd, 0, sizeof(*vfd));

       strlcpy(vfd->name, VIDEO_DEVICE_NAME, sizeof(vfd->name));

       vfd->fops = &vid_dev_fops;
       vfd->ioctl_ops = &vid_dev_ioctl_ops;
       vfd->v4l2_dev = sd->v4l2_dev;
       vfd->minor = -1;
       vfd->release = video_device_release_empty;
       vfd->queue = q;
       vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;


       INIT_LIST_HEAD(&vid_dev->vidq.active);
       init_waitqueue_head(&vid_dev->vidq.wq);
       memset(q, 0, sizeof(*q));
       q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
       q->io_modes = VB2_MMAP | VB2_USERPTR;

       q->ops = &vb2_video_qops;
#ifdef CONFIG_VIDEO_DWC_DMA_CONTIG
       q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
       q->mem_ops = &vb2_dma_contig_memops;
#else
       q->mem_ops = &vb2_vmalloc_memops;
       q->io_modes = VB2_MMAP | VB2_USERPTR |  VB2_READ;
#endif
       q->buf_struct_size = sizeof(struct rx_buffer);
       q->drv_priv = vid_dev;
       q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
       q->lock = &vid_dev->lock;
       q->dev = &vid_dev->pdev->dev;
	 q->min_buffers_needed = 1;

       ret = vb2_queue_init(q);
       if (ret < 0)
               return ret;

       vid_dev->vd_pad.flags = MEDIA_PAD_FL_SINK;
       ret = media_entity_pads_init(&vfd->entity, 1, &vid_dev->vd_pad);
       if (ret < 0)
               return ret;

       video_set_drvdata(vfd, vid_dev);
       vid_dev->ve.pipe = v4l2_get_subdev_hostdata(sd);

       ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
       if (ret < 0) {
               media_entity_cleanup(&vfd->entity);
               vid_dev->ve.pipe = NULL;
               return ret;
       }

       v4l2_info(sd->v4l2_dev, "Registered %s as /dev/%s\n",
                 vfd->name, video_device_node_name(vfd));
       return 0;
}

static void vid_dev_subdev_unregistered(struct v4l2_subdev *sd)
{
       struct video_device_dev *vid_dev = v4l2_get_subdevdata(sd);

       if (vid_dev == NULL)
               return;

       mutex_lock(&vid_dev->lock);

       if (video_is_registered(&vid_dev->ve.vdev)) {
               video_unregister_device(&vid_dev->ve.vdev);
               media_entity_cleanup(&vid_dev->ve.vdev.entity);
               vid_dev->ve.pipe = NULL;
       }

       mutex_unlock(&vid_dev->lock);
}

static const struct v4l2_subdev_internal_ops vid_dev_subdev_internal_ops = {
       .registered = vid_dev_subdev_registered,
       .unregistered = vid_dev_subdev_unregistered,
};

static struct v4l2_subdev_ops vid_dev_subdev_ops;

static int vid_dev_create_capture_subdev(struct video_device_dev *vid_dev)
{
       struct v4l2_subdev *sd = &vid_dev->subdev;
       int ret;

       v4l2_subdev_init(sd, &vid_dev_subdev_ops);
       sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
       snprintf(sd->name, sizeof(sd->name), "%s.%d", "capture_device", vid_dev->dma->chan_id);

       vid_dev->subdev_pads[VIDEO_DEV_SD_PAD_SINK_CSI].flags =
               MEDIA_PAD_FL_SINK;
       vid_dev->subdev_pads[VIDEO_DEV_SD_PAD_SOURCE_DMA].flags =
               MEDIA_PAD_FL_SOURCE;
       ret = media_entity_pads_init(&sd->entity, VIDEO_DEV_SD_PADS_NUM,
                                  vid_dev->subdev_pads);
       if (ret)
               return ret;

       sd->internal_ops = &vid_dev_subdev_internal_ops;
       sd->owner = THIS_MODULE;
       v4l2_set_subdevdata(sd, vid_dev);

       return 0;
}

static void vid_dev_unregister_subdev(struct video_device_dev *vid_dev)
{
       struct v4l2_subdev *sd = &vid_dev->subdev;

       v4l2_device_unregister_subdev(sd);
       media_entity_cleanup(&sd->entity);
       v4l2_set_subdevdata(sd, NULL);
}

static const struct of_device_id vid_dev_of_match[];

static int vid_dev_probe(struct platform_device *pdev)
{
       struct device *dev = &pdev->dev;
       const struct of_device_id *of_id;
       int ret = 0;
       struct video_device_dev *vid_dev;

       dev_dbg(dev, "Installing CSI Video Platform Video Device module\n");

       if (!dev->of_node)
               return -ENODEV;

       vid_dev = devm_kzalloc(dev, sizeof(*vid_dev), GFP_KERNEL);
       if (!vid_dev)
               return -ENOMEM;

       of_id = of_match_node(vid_dev_of_match, dev->of_node);
       if (WARN_ON(of_id == NULL))
               return -EINVAL;

	ret = of_reserved_mem_device_init(dev);
	if(ret) {
		dev_err(dev, "Could not get reserved memory\n");
		return ret;
	}

       vid_dev->pdev = pdev;

       spin_lock_init(&vid_dev->slock);
       mutex_init(&vid_dev->lock);

       vid_dev->dma = dma_request_slave_channel(&pdev->dev, "vdma");
       if (vid_dev->dma == NULL) {
               dev_err(&pdev->dev, "no VDMA channel found by name vdma\n");
               ret = -ENODEV;
               goto end;
       }

       ret = vid_dev_create_capture_subdev(vid_dev);
       if (ret)
               goto end;

       platform_set_drvdata(pdev, vid_dev);
#ifdef CONFIG_VIDEO_DWC_DMA_CONTIG
       vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));
       dev_info(dev, "VIDEOBUF2 DMA CONTIG\n");
#else
       dev_info(dev, "VIDEOBUF2 VMALLOC\n");
#endif
       dev_info(dev, "Video Device registered successfully\n");
       return 0;
end:
       dev_err(dev, "Video Device not registered\n");
       return ret;
}

static int vid_dev_remove(struct platform_device *pdev)
{
       struct video_device_dev *dev = platform_get_drvdata(pdev);

       vid_dev_unregister_subdev(dev);
       dev_info(&pdev->dev, "Driver removed\n");
       return 0;
}

static const struct of_device_id vid_dev_of_match[] = {
       {.compatible = "snps,video-device"},
       {}
};

MODULE_DEVICE_TABLE(of, vid_dev_of_match);

static struct platform_driver __refdata vid_dev_pdrv = {
       .remove = vid_dev_remove,
       .probe = vid_dev_probe,
       .driver = {
                  .name = VIDEO_DEVICE_NAME,
                  .owner = THIS_MODULE,
                  .of_match_table = vid_dev_of_match,
                  },
};

module_platform_driver(vid_dev_pdrv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramiro Oliveira <roliv...@synopsys.com>");
MODULE_DESCRIPTION("Driver for configuring DMA and Video Device");
