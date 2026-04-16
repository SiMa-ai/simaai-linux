// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 SiMa Technologies, Inc.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "simaai-v4l2-vdma.h"

#define N_BUFFERS			8
#define MAX_CAP_DEVICES			4

#define MAX_WIDTH			4096
#define MAX_HEIGHT			3280
#define MIN_WIDTH			640
#define MIN_HEIGHT			480

struct v4l2vid_dev {
	struct device			*dev;
	struct vb2_queue		vb_queue;
	struct video_device		vdev;
	struct fwnode_handle		*ep_fwnode;
	unsigned int			ep_pad;
	struct v4l2_subdev		*ep_sd;
	struct media_pad		pad;
	struct v4l2_format		format;
	const struct plat_csi_fmt	*fmt;
	struct vdma_channel		*vdma_channel;
	bool				enabled;
	const char			*label;
	struct mutex			lock;
};

struct v4l2vid_master_dev {
	struct device			*dev;
	struct media_device		media_dev;
	struct v4l2_device		v4l2_dev;
	struct v4l2_async_notifier	nf;
	struct v4l2vid_dev		devs[MAX_CAP_DEVICES];
};

struct v4l2vid_asc {
	struct v4l2_async_connection	base;
	struct v4l2vid_dev		*dev;
};

struct plat_csi_fmt {
	char				*name;
	u32				mbus_code;
	u32				fourcc;
	u8				depth;
};

static const struct plat_csi_fmt v4l2vid_formats[] = {
        /* YUV & RGB Formats */
        {
                .name = "UYVY",
                .fourcc = V4L2_PIX_FMT_UYVY,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
        }, {
                .name = "YUYV",
                .fourcc = V4L2_PIX_FMT_YUYV,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_VYUY8_1X16,
        }, {
                .name = "YUV420",
                .fourcc = V4L2_PIX_FMT_YUV420,
                .depth = 12,
                .mbus_code = MEDIA_BUS_FMT_Y8_1X8,
        }, {
                .name = "RGB565",
                .fourcc = V4L2_PIX_FMT_RGB565,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_RGB565_1X16,
        }, {
                .name = "RGB888",
                .fourcc = V4L2_PIX_FMT_RGB24,
                .depth = 24,
                .mbus_code = MEDIA_BUS_FMT_RGB888_1X24,
        },

        /* 8-bit RAW / Greyscale */
        {
                .name = "GREY",
                .fourcc = V4L2_PIX_FMT_GREY,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_Y8_1X8,
        }, {
                .name = "SBGGR8",
                .fourcc = V4L2_PIX_FMT_SBGGR8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SBGGR8_1X8,
        }, {
                .name = "SGBRG8",
                .fourcc = V4L2_PIX_FMT_SGBRG8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SGBRG8_1X8,
        }, {
                .name = "SGRBG8",
                .fourcc = V4L2_PIX_FMT_SGRBG8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8,
        }, {
                .name = "SRGGB8",
                .fourcc = V4L2_PIX_FMT_SRGGB8,
                .depth = 8,
                .mbus_code = MEDIA_BUS_FMT_SRGGB8_1X8,
        },

        /* 10-bit RAW / Greyscale */
        {
                .name = "Y10",
                .fourcc = V4L2_PIX_FMT_Y10,
                .depth = 10,
                .mbus_code = MEDIA_BUS_FMT_Y10_1X10,
        }, {
                .name = "SBGGR10",
                .fourcc = V4L2_PIX_FMT_SBGGR10,
                .depth = 10,
                .mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
        }, {
                .name = "SGBRG10",
                .fourcc = V4L2_PIX_FMT_SGBRG10,
                .depth = 10,
                .mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
        }, {
                .name = "SGRBG10",
                .fourcc = V4L2_PIX_FMT_SGRBG10,
                .depth = 10,
                .mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
        }, {
                .name = "SRGGB10",
                .fourcc = V4L2_PIX_FMT_SRGGB10,
                .depth = 10,
                .mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
        },

        /* 12-bit RAW / Greyscale */
        {
                .name = "Y12",
                .fourcc = V4L2_PIX_FMT_Y12,
                .depth = 12,
                .mbus_code = MEDIA_BUS_FMT_Y12_1X12,
        }, {
                .name = "SBGGR12",
                .fourcc = V4L2_PIX_FMT_SBGGR12,
                .depth = 12,
                .mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
        }, {
                .name = "SGBRG12",
                .fourcc = V4L2_PIX_FMT_SGBRG12,
                .depth = 12,
                .mbus_code = MEDIA_BUS_FMT_SGBRG12_1X12,
        }, {
                .name = "SGRBG12",
                .fourcc = V4L2_PIX_FMT_SGRBG12,
                .depth = 12,
                .mbus_code = MEDIA_BUS_FMT_SGRBG12_1X12,
        }, {
                .name = "SRGGB12",
                .fourcc = V4L2_PIX_FMT_SRGGB12,
                .depth = 12,
                .mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
        },

        /* 14-bit RAW */
        {
                .name = "SBGGR14",
                .fourcc = V4L2_PIX_FMT_SBGGR14,
                .depth = 14,
                .mbus_code = MEDIA_BUS_FMT_SBGGR14_1X14,
        }, {
                .name = "SGBRG14",
                .fourcc = V4L2_PIX_FMT_SGBRG14,
                .depth = 14,
                .mbus_code = MEDIA_BUS_FMT_SGBRG14_1X14,
        }, {
                .name = "SGRBG14",
                .fourcc = V4L2_PIX_FMT_SGRBG14,
                .depth = 14,
                .mbus_code = MEDIA_BUS_FMT_SGRBG14_1X14,
        }, {
                .name = "SRGGB14",
                .fourcc = V4L2_PIX_FMT_SRGGB14,
                .depth = 14,
                .mbus_code = MEDIA_BUS_FMT_SRGGB14_1X14,
        },

        /* 16-bit RAW */
        {
                .name = "SBGGR16",
                .fourcc = V4L2_PIX_FMT_SBGGR16,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SBGGR16_1X16,
        }, {
                .name = "SGBRG16",
                .fourcc = V4L2_PIX_FMT_SGBRG16,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SGBRG16_1X16,
        }, {
                .name = "SGRBG16",
                .fourcc = V4L2_PIX_FMT_SGRBG16,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SGRBG16_1X16,
        }, {
                .name = "SRGGB16",
                .fourcc = V4L2_PIX_FMT_SRGGB16,
                .depth = 16,
                .mbus_code = MEDIA_BUS_FMT_SRGGB16_1X16,
        }
};

static inline struct v4l2vid_asc *
asc_to_v4l2vid(struct v4l2_async_connection *asc)
{
	return container_of(asc, struct v4l2vid_asc, base);
}

static inline struct v4l2vid_master_dev *
nf_to_v4l2vid_master_dev(struct v4l2_async_notifier *n)
{
	return container_of(n, struct v4l2vid_master_dev, nf);
}

static inline struct vdma_buffer *to_vdma_buffer(struct vb2_v4l2_buffer *vb2)
{
	return container_of(vb2, struct vdma_buffer, vb);
}

static const struct plat_csi_fmt *v4l2vid_find_format(struct v4l2_format *f)
{
	const struct plat_csi_fmt *fmt = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(v4l2vid_formats); ++i) {
		fmt = &v4l2vid_formats[i];
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
	struct v4l2vid_dev *dev = video_drvdata(file);

	snprintf(cap->driver, sizeof(cap->driver), "%s", dev->label);
	snprintf(cap->card, sizeof(cap->card), "%s", dev->label);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(dev->dev));
	return 0;
}

static int
vidioc_enum_fmt_vid_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	const struct plat_csi_fmt *p_fmt;

	if (f->index >= ARRAY_SIZE(v4l2vid_formats))
		return -EINVAL;

	p_fmt = &v4l2vid_formats[f->index];

	f->pixelformat = p_fmt->fourcc;

	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct v4l2vid_dev *dev = video_drvdata(file);

	f->fmt.pix = dev->format.fmt.pix;
	return 0;
}

static int
vidioc_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct plat_csi_fmt *fmt;

	fmt = v4l2vid_find_format(f);

	if (!fmt) {
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB8;
		fmt = v4l2vid_find_format(f);
	}

	f->fmt.pix.field = V4L2_FIELD_NONE;
	v4l_bound_align_image(&f->fmt.pix.width, 48, MAX_WIDTH, 2,
			     &f->fmt.pix.height, 32, MAX_HEIGHT, 0, 0);

	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline + VDMA_FRAME_METADATA_SIZE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct v4l2vid_dev *dev = video_drvdata(file);
	struct v4l2_subdev_format fmt;
	struct v4l2_pix_format *dev_fmt_pix = &dev->format.fmt.pix;
	int ret;

	if (vb2_is_busy(&dev->vb_queue))
		return -EBUSY;

	ret = vidioc_try_fmt_vid_cap(file, dev, f);
	if (ret)
		return ret;

	dev->fmt = v4l2vid_find_format(f);
	dev_fmt_pix->pixelformat = f->fmt.pix.pixelformat;
	dev_fmt_pix->width = f->fmt.pix.width;
	dev_fmt_pix->height  = f->fmt.pix.height;
	dev_fmt_pix->field = f->fmt.pix.field;
	dev_fmt_pix->bytesperline = (dev_fmt_pix->width * dev->fmt->depth) >> 3;
	dev_fmt_pix->sizeimage =
			dev_fmt_pix->height * dev_fmt_pix->bytesperline + VDMA_FRAME_METADATA_SIZE;

	fmt.format.colorspace = V4L2_COLORSPACE_RAW;
	fmt.format.code = dev->fmt->mbus_code;

	fmt.format.width = dev_fmt_pix->width;
	fmt.format.height = dev_fmt_pix->height;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = 0;

	return ret;
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

	for (i = 0; i < ARRAY_SIZE(v4l2vid_formats); i++)
		if (v4l2vid_formats[i].fourcc == fsize->pixel_format)
			break;
	if (i == ARRAY_SIZE(v4l2vid_formats))
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
	strscpy(input->name, "Camera");
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
v4l2vid_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct v4l2vid_dev *dev = video_drvdata(file);
	int ret;

	ret = video_device_pipeline_start(&dev->vdev, &dev->vdev.pipe);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to start pipeline %d", ret);
		return ret;
	}

	return vb2_ioctl_streamon(file, priv, type);
}

static int
v4l2vid_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct v4l2vid_dev *dev = video_drvdata(file);

	vb2_ioctl_streamoff(file, priv, type);
	if (dev->pad.pipe)
		video_device_pipeline_stop(&dev->vdev);

	return 0;
}

static const struct v4l2_ioctl_ops v4l2vid_ioctl_ops = {
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
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = v4l2vid_streamon,
	.vidioc_streamoff = v4l2vid_streamoff,
};

static int
v4l2vid_open(struct file *file)
{
	struct v4l2vid_dev *dev = video_drvdata(file);
	int ret;

	mutex_lock(&dev->lock);

	ret = v4l2_fh_open(file);
	if (ret < 0)
		goto unlock;

	if (!v4l2_fh_is_singular_file(file))
		goto unlock;

	if (!ret)
		goto unlock;

	v4l2_fh_release(file);
unlock:
	mutex_unlock(&dev->lock);
	return ret;
}

static int
v4l2vid_release(struct file *file)
{
	struct v4l2vid_dev *dev = video_drvdata(file);
	struct media_entity *entity = &dev->vdev.entity;

	mutex_lock(&dev->lock);

	if (v4l2_fh_is_singular_file(file)) {
		mutex_lock(&entity->graph_obj.mdev->graph_mutex);
		entity->use_count--;
		mutex_unlock(&entity->graph_obj.mdev->graph_mutex);
	}

	_vb2_fop_release(file, NULL);

	mutex_unlock(&dev->lock);
	return 0;
}

static const struct v4l2_file_operations v4l2vid_fops = {
	.owner = THIS_MODULE,
	.open = v4l2vid_open,
	.release = v4l2vid_release,
	.write = vb2_fop_write,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

/*
 * VideoBuffer2 operations
 */
static int queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			unsigned int *nplanes, unsigned int sizes[],
			struct device *alloc_devs[])
{
	struct v4l2vid_dev *dev = vb2_get_drv_priv(vq);
	unsigned long size = 0;

	size = dev->format.fmt.pix.sizeimage;
	if (dev->format.fmt.pix.sizeimage  == 0) {
		dev_err(dev->dev, "Configured Image size is zero\n");
		return -EINVAL;
	}
	*nbuffers = N_BUFFERS;
	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vdma_buffer *buf = to_vdma_buffer(vbuf);
	struct v4l2vid_dev *dev = vb2_get_drv_priv(vb->vb2_queue);

	buf->channel = dev->vdma_channel;
	buf->size = dev->format.fmt.pix.sizeimage;

	return 0;
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct v4l2vid_dev *dev = vb2_get_drv_priv(vq);
	int ret;

	ret = v4l2_subdev_enable_streams(dev->ep_sd, dev->ep_pad,
		get_vdma_channel_mask(dev->vdma_channel));
	if (ret) {
		dev_err(dev->dev, "Failed to enable stream on EP subdevice: %d", ret);
		return ret;
	}

	return 0;
}

static void stop_streaming(struct vb2_queue *vq)
{

	struct v4l2vid_dev *dev = vb2_get_drv_priv(vq);

	v4l2_subdev_disable_streams(dev->ep_sd, dev->ep_pad,
		get_vdma_channel_mask(dev->vdma_channel));
}

static const struct vb2_ops vb2_video_qops = {
	.queue_setup = queue_setup,
	.buf_prepare = buffer_prepare,
	.buf_queue = vdma_buffer_queue,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int v4l2vid_register_video_dev(struct v4l2vid_dev *dev, struct v4l2vid_master_dev *master_dev)
{
	struct vb2_queue *q;
	struct video_device *vfd;
	int ret;

	vfd = &dev->vdev;
	q = &dev->vb_queue;

	snprintf(vfd->name, sizeof(vfd->name), "%s", dev->label);

	vfd->fops = &v4l2vid_fops;
	vfd->ioctl_ops = &v4l2vid_ioctl_ops;
	vfd->v4l2_dev = &master_dev->v4l2_dev;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->queue = q;
	vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;
	video_set_drvdata(vfd, dev);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;

	q->ops = &vb2_video_qops;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct vdma_buffer);
	q->drv_priv = dev;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &dev->lock;
	q->dev = master_dev->dev;
	q->min_queued_buffers = 4;
	q->min_reqbufs_allocation = 4;

	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to init vb2 queue: %d\n", ret);
		goto vb2_err;
	}

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to register video device: %d\n", ret);
		goto cleanup;
	}

	dev_info(dev->dev, "Registered %s as /dev/%s\n", vfd->name, video_device_node_name(vfd));

	return 0;

cleanup:
	media_entity_cleanup(&vfd->entity);
vb2_err:
	return ret;
}

static int v4l2vid_notify_complete(struct v4l2_async_notifier *nf)
{
	struct v4l2vid_master_dev *master_dev = nf_to_v4l2vid_master_dev(nf);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&master_dev->v4l2_dev);
	if (ret < 0)
		dev_err(master_dev->dev, "Failed to register subdev nodes: %d\n", ret);
	return ret;
}
static int v4l2vid_notify_bound(struct v4l2_async_notifier *nf,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *base_asc)
{
	struct v4l2vid_dev *dev = asc_to_v4l2vid(base_asc)->dev;
	int ret;

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  dev->ep_fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	dev->ep_sd = subdev;
	dev->ep_pad = ret;

	dev->vdma_channel = get_vdma_channel(dev->ep_sd, dev->ep_pad);
	if (dev->vdma_channel == NULL) {
		dev_err(dev->dev, "Unable to find vdma channel for pad %s:%u\n",
			dev->ep_sd->name, dev->ep_pad);
		return -ENOENT;
	}

	ret = media_create_pad_link(&dev->ep_sd->entity, dev->ep_pad,
				    &dev->vdev.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev->dev, "Unable to link %s:%u -> %s:%u\n",
			dev->ep_sd->name, dev->ep_pad, dev->vdev.name, 0);
		return ret;
	}

	return 0;
}

static void v4l2vid_notify_unbind(struct v4l2_async_notifier *nf,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	struct v4l2vid_dev *dev = asc_to_v4l2vid(base_asc)->dev;

	dev->ep_sd = NULL;
	dev->vdma_channel = NULL;

	if (video_is_registered(&dev->vdev)) {
		video_unregister_device(&dev->vdev);
		media_entity_cleanup(&dev->vdev.entity);
	}
}

static int v4l2vid_link_validate(struct media_link *link)
{
	struct media_entity *entity = link->sink->entity;
	struct v4l2_subdev *source_sd;
	struct v4l2_subdev_format source_fmt = { 0 };
	int ret;
	struct video_device *vdev = media_entity_to_video_device(entity);
	struct v4l2vid_dev *dev = video_get_drvdata(vdev);

	if (!is_media_entity_v4l2_subdev(link->source->entity)) {
		dev_err(dev->dev, "Remote is not a subdev\n");
		return -EINVAL;
	}

	source_sd = media_entity_to_v4l2_subdev(link->source->entity);

	source_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	source_fmt.pad = link->source->index;

	ret = v4l2_subdev_call(source_sd, pad, get_fmt, NULL, &source_fmt);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to get source format\n");
		return ret;
	}

	if (source_fmt.format.width != dev->format.fmt.pix.width ||
		source_fmt.format.height != dev->format.fmt.pix.height) {
		dev_err(dev->dev, "Resolution mismatch: Source %dx%d vs Capture %dx%d\n",
			source_fmt.format.width, source_fmt.format.height,
			dev->format.fmt.pix.width, dev->format.fmt.pix.height);
		return -EPIPE;
	}

	if (source_fmt.format.field != dev->format.fmt.pix.field &&
		dev->format.fmt.pix.field != V4L2_FIELD_NONE) {
		dev_err(dev->dev, "Field mismatch: Source %d vs Capture %d\n",
			source_fmt.format.field, dev->format.fmt.pix.field);
		return -EPIPE;
	}

	if (source_fmt.format.code != dev->fmt->mbus_code) {
		dev_err(dev->dev, "Code mismatch: Source %d vs Capture %d\n",
			source_fmt.format.code, dev->fmt->mbus_code);
		return -EPIPE;
	}

	return 0;
}

static int v4l2vid_get_fwnode_pad(struct media_entity *entity,
			      struct fwnode_endpoint *endpoint)
{
	return endpoint->port;
}

static const struct media_entity_operations v4l2vid_media_ops = {
	.link_validate = v4l2vid_link_validate,
	.get_fwnode_pad = v4l2vid_get_fwnode_pad,
};

static const struct v4l2_async_notifier_operations v4l2vid_notify_ops = {
	.bound = v4l2vid_notify_bound,
	.unbind = v4l2vid_notify_unbind,
	.complete = v4l2vid_notify_complete,
};

static int v4l2vid_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_format *sdformat)
{
	struct v4l2_mbus_framefmt *fmt;

	/*
	 * The VDMA can't transcode in any way, the source format can't be
	 * modified.
	 */
	if (sdformat->pad != 0)
		return -EINVAL;

	fmt = v4l2_subdev_state_get_format(sd_state, sdformat->pad,
			sdformat->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = sdformat->format;

	/* Propagate the format from sink stream to source stream */
	fmt = v4l2_subdev_state_get_opposite_stream_format(sd_state, sdformat->pad,
			sdformat->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = sdformat->format;

	return 0;
}

static const struct v4l2_subdev_pad_ops v4l2vid_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = v4l2vid_set_fmt,
};

static const struct v4l2_subdev_ops v4l2vid_subdev_ops = {
	.pad = &v4l2vid_pad_ops,
};

static void v4l2vid_notifier_unregister(struct v4l2vid_master_dev *dev)
{
	v4l2_async_nf_unregister(&dev->nf);
	v4l2_async_nf_cleanup(&dev->nf);
}


static int v4l2vid_init_vdev_entity(struct v4l2vid_dev *dev,
	struct v4l2vid_master_dev *master_dev)
{
	struct media_entity *entity = &dev->vdev.entity;
	struct v4l2vid_asc *asc;
	int ret;

	dev->pad.flags = MEDIA_PAD_FL_SINK;
	mutex_init(&dev->lock);

	ret = media_entity_pads_init(entity, 1, &dev->pad);
	if (ret)
		return ret;

	entity->ops = &v4l2vid_media_ops;

	asc = v4l2_async_nf_add_fwnode(&master_dev->nf, dev->ep_fwnode, struct v4l2vid_asc);
	if (IS_ERR(asc)) {
		dev_err(dev->dev, "Failed to add subdev: %pe", asc);
		ret = PTR_ERR(asc);
	} else
		asc->dev = dev;

	return ret;
}

static void v4l2vid_cleanup_capture_dev(struct v4l2vid_dev *dev)
{
	struct media_entity *entity = &dev->vdev.entity;

	media_entity_cleanup(entity);
}

static int v4l2vid_master_init(struct v4l2vid_master_dev *master_dev)
{
	struct media_device *media_dev = &master_dev->media_dev;
	struct v4l2_device *v4l2_dev = &master_dev->v4l2_dev;
	int ret;

	/* Initialize media device. */
	strscpy(media_dev->model, "SiMa.ai Capture Media Device", sizeof(media_dev->model));
	media_dev->dev = master_dev->dev;
	media_device_init(media_dev);
	ret = media_device_register(media_dev);
	if (ret < 0) {
		dev_err(master_dev->dev, "Failed to register media device: %d\n", ret);
		return ret;
	}
	v4l2_async_nf_init(&master_dev->nf, &master_dev->v4l2_dev);
	master_dev->nf.ops = &v4l2vid_notify_ops;

	/* Initialize and register the V4L2 device. */
	v4l2_dev->mdev = media_dev;
	strscpy(v4l2_dev->name, "v4l2vid-master", sizeof(v4l2_dev->name));
	ret = v4l2_device_register(master_dev->dev, v4l2_dev);
	if (ret < 0) {
		media_device_unregister(media_dev);
		media_device_cleanup(media_dev);
		dev_err(master_dev->dev, "Failed to register V4L2 device: %d\n", ret);
	}

	return ret;
}

static void v4l2vid_parse_sink_dt_endpoints(struct v4l2vid_master_dev *master_dev)
{
	struct fwnode_handle *fwnode = dev_fwnode(master_dev->dev);
	struct v4l2vid_dev *dev;
	struct fwnode_handle *ep;
	int i;

	for (i = 0; i < MAX_CAP_DEVICES; i++) {
		dev = &master_dev->devs[i];
		ep = fwnode_graph_get_endpoint_by_id(fwnode, i, 0, 0);
		if (!ep || !fwnode_device_is_available(ep)) {
			dev_dbg(master_dev->dev, "No local endpoint 0 on port %u\n", i);
			dev->enabled = false;
			continue;
		}

		if (fwnode_property_read_string(ep, "label", &dev->label)) {
			dev_dbg(master_dev->dev, "Failed to get label on port %u\n", i);
			dev->label = "v4l2vid";
		}

		dev->ep_fwnode = fwnode_graph_get_remote_endpoint(ep);
		fwnode_handle_put(ep);
		if (!dev->ep_fwnode) {
			dev_dbg(master_dev->dev, "Failed to get remote endpoint on port %u\n", i);
			continue;
		}

		dev->dev = master_dev->dev;
		dev->enabled = true;
	}
}

static void v4l2vid_v4l2_cleanup(struct v4l2vid_master_dev *dev)
{
	int i;

	for (i = 0; i < MAX_CAP_DEVICES; i++) {
		if (!dev->devs[i].enabled)
			continue;

		v4l2vid_cleanup_capture_dev(&dev->devs[i]);
	}

	v4l2vid_notifier_unregister(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	media_device_unregister(&dev->media_dev);
	media_device_cleanup(&dev->media_dev);
}

static int v4l2vid_probe(struct platform_device *pdev)
{
	struct device *dev_ = &pdev->dev;
	struct v4l2vid_master_dev *master_dev;
	int ret = 0;
	int i;

	master_dev = devm_kzalloc(dev_, sizeof(*master_dev), GFP_KERNEL);
	if (!master_dev)
		return -ENOMEM;

	master_dev->dev = dev_;
	platform_set_drvdata(pdev, master_dev);

	ret = dma_set_mask_and_coherent(master_dev->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(master_dev->dev, "Failed to set DMA mask\n");
		return ret;
	}

	ret = v4l2vid_master_init(master_dev);
	if (ret < 0) {
		dev_err(master_dev->dev, "Failed to initialize master device: %d\n", ret);
		return ret;
	}

	v4l2vid_parse_sink_dt_endpoints(master_dev);
	for (i = 0; i < MAX_CAP_DEVICES; i++) {
		if (!master_dev->devs[i].enabled)
			continue;

		ret = v4l2vid_init_vdev_entity(&master_dev->devs[i], master_dev);
		if (ret < 0) {
			dev_err(master_dev->dev, "Failed to initialize V4L2: %d\n", ret);
			master_dev->devs[i].enabled = false;
			continue;
		}

		ret = v4l2vid_register_video_dev(&master_dev->devs[i], master_dev);
		if (ret < 0) {
			dev_err(master_dev->dev, "Failed to register video device: %d\n", ret);
			master_dev->devs[i].enabled = false;
			continue;
		}
	}

	ret = v4l2_async_nf_register(&master_dev->nf);
	if (ret) {
		dev_err(master_dev->dev, "Failed to register master notifier");
		v4l2vid_v4l2_cleanup(master_dev);
		return ret;
	}

	dev_info(master_dev->dev, "Successfully initialized\n");

	return 0;
}

static void v4l2vid_remove(struct platform_device *pdev)
{
	struct v4l2vid_master_dev *dev = platform_get_drvdata(pdev);

	v4l2vid_v4l2_cleanup(dev);
}

static const struct of_device_id simaai_v4l2vid_of_match[] = {
	{ .compatible = "simaai,v4l2-video" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, simaai_v4l2vid_of_match);

static struct platform_driver simaai_v4l2vid_driver = {
	.probe		= v4l2vid_probe,
	.remove		= v4l2vid_remove,
	.driver = {
		.of_match_table = simaai_v4l2vid_of_match,
		.name		= "simaai-v4l2-video",
	}
};
module_platform_driver(simaai_v4l2vid_driver);

MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai");
MODULE_DESCRIPTION("SiMa.ai V4L2 capture video driver");
MODULE_LICENSE("GPL");
