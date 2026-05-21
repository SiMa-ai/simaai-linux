// SPDX-License-Identifier: GPL-2.0
/*
 * li-fpga.c - Leopard Imaging FPGA V4L2 subdevice bridge driver
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2026, SiMa Technologies, Inc.
 *
 * The Leopard Imaging FPGA sits between a camera sensor and the SoC's CSI
 * receiver. The original driver exposed li_fpga_write_reg() which every
 * sensor driver had to call from its start_streaming() and stop_streaming()
 * callbacks. This driver instead registers the FPGA as a V4L2 sub-device
 * with two pads (sink from the sensor, source to the CSI receiver). It
 * uses the V4L2 async framework to discover its upstream sensor from the
 * OF graph and programs the FPGA on .enable_streams / .disable_streams.
 * The sensor driver does not need to know anything about the FPGA.
 *
 * Expected OF-graph topology:
 *
 *   sensor  ---port@0--->  li-fpga  ---port@1--->  csi
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define LI_FPGA_PAD_SINK	0
#define LI_FPGA_PAD_SOURCE	1
#define LI_FPGA_NUM_PADS	2

/*
 * FPGA register map:
 *   0x01 : line length high byte  (width * bpp / 8, big endian)
 *   0x02 : line length low  byte
 *   0x03 : stream enable (1 = enable, 0 = disable)
 */
#define LI_FPGA_REG_LINE_LEN_HI	0x01
#define LI_FPGA_REG_LINE_LEN_LO	0x02
#define LI_FPGA_REG_STREAM	0x03

#define LI_FPGA_STREAM_DISABLE	0x00
#define LI_FPGA_STREAM_ENABLE	0x01

struct li_fpga {
	struct i2c_client		*client;
	struct v4l2_subdev		sd;
	struct media_pad		pads[LI_FPGA_NUM_PADS];

	struct v4l2_async_notifier	notifier;
	struct fwnode_handle		*sensor_ep_fwnode;
	struct v4l2_subdev		*sensor_sd;
	unsigned int			sensor_pad;

	struct v4l2_ctrl_handler	ctrls;
	struct v4l2_ctrl		*link_freq;
	struct v4l2_ctrl		*pixel_rate;
	u64				*link_frequencies;
	unsigned int			nr_link_frequencies;

	struct mutex			lock;
	bool				streaming;
	u32				stream_on_delay_us;
};

struct li_fpga_asc {
	struct v4l2_async_connection	base;
	struct li_fpga			*fpga;
};

static inline struct li_fpga *sd_to_li_fpga(struct v4l2_subdev *sd)
{
	return container_of(sd, struct li_fpga, sd);
}

static int li_fpga_write_reg(struct li_fpga *fpga, u8 addr, u8 val)
{
	struct i2c_client *client = fpga->client;
	u8 data[2] = { addr, val };
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= sizeof(data),
		.buf	= data,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev,
			"i2c write failed: reg 0x%02x = 0x%02x (%d)\n",
			addr, val, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static unsigned int li_fpga_mbus_bpp(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return 8;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return 12;
	case MEDIA_BUS_FMT_SBGGR14_1X14:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return 14;
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
		return 16;
	default:
		return 0;
	}
}

static int li_fpga_configure(struct li_fpga *fpga,
			     const struct v4l2_mbus_framefmt *fmt)
{
	unsigned int bpp;
	u32 line_bytes;
	int ret;

	bpp = li_fpga_mbus_bpp(fmt->code);
	if (!bpp) {
		dev_err(&fpga->client->dev,
			"unsupported mbus format 0x%04x\n", fmt->code);
		return -EINVAL;
	}

	if ((fmt->width * bpp) % 8) {
		dev_err(&fpga->client->dev,
			"width %u at %u bpp is not byte-aligned\n",
			fmt->width, bpp);
		return -EINVAL;
	}

	line_bytes = (fmt->width * bpp) / 8;

	/* Start from a known (disabled) state before reprogramming. */
	ret = li_fpga_write_reg(fpga, LI_FPGA_REG_STREAM,
				LI_FPGA_STREAM_DISABLE);
	if (ret)
		return ret;

	ret = li_fpga_write_reg(fpga, LI_FPGA_REG_LINE_LEN_HI,
				(line_bytes >> 8) & 0xff);
	if (ret)
		return ret;

	return li_fpga_write_reg(fpga, LI_FPGA_REG_LINE_LEN_LO,
				 line_bytes & 0xff);
}

static int li_fpga_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *sink_fmt, *source_fmt;

	/* The FPGA is a transparent bridge; source format just mirrors sink. */
	if (fmt->pad == LI_FPGA_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, fmt);

	sink_fmt = v4l2_subdev_state_get_format(state, LI_FPGA_PAD_SINK, 0);
	source_fmt = v4l2_subdev_state_get_format(state, LI_FPGA_PAD_SOURCE, 0);
	if (!sink_fmt || !source_fmt)
		return -EINVAL;

	*sink_fmt = fmt->format;
	*source_fmt = fmt->format;
	return 0;
}

static int li_fpga_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct li_fpga *fpga = sd_to_li_fpga(sd);
	struct v4l2_mbus_framefmt *sink_fmt;
	int ret;

	if (pad != LI_FPGA_PAD_SOURCE)
		return -EINVAL;

	if (!fpga->sensor_sd) {
		dev_err(&fpga->client->dev,
			"no upstream sensor bound, cannot start streaming\n");
		return -ENODEV;
	}

	sink_fmt = v4l2_subdev_state_get_format(state, LI_FPGA_PAD_SINK, 0);
	if (!sink_fmt)
		return -EINVAL;

	mutex_lock(&fpga->lock);

	if (fpga->streaming) {
		ret = 0;
		goto unlock;
	}

	/*
	 * Sequence: program line length with FPGA disabled, start the
	 * upstream sensor, wait for it to settle, then enable the FPGA
	 * output. Preserved from the original sensor-driver code.
	 */
	ret = li_fpga_configure(fpga, sink_fmt);
	if (ret)
		goto unlock;

	ret = v4l2_subdev_enable_streams(fpga->sensor_sd, fpga->sensor_pad,
					 BIT(0));
	if (ret) {
		dev_err(&fpga->client->dev,
			"failed to enable upstream sensor stream: %d\n", ret);
		goto unlock;
	}

	if (fpga->stream_on_delay_us)
		fsleep(fpga->stream_on_delay_us);

	ret = li_fpga_write_reg(fpga, LI_FPGA_REG_STREAM,
				LI_FPGA_STREAM_ENABLE);
	if (ret) {
		v4l2_subdev_disable_streams(fpga->sensor_sd, fpga->sensor_pad,
					    BIT(0));
		goto unlock;
	}

	fpga->streaming = true;

unlock:
	mutex_unlock(&fpga->lock);
	return ret;
}

static int li_fpga_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct li_fpga *fpga = sd_to_li_fpga(sd);
	int ret, err;

	if (pad != LI_FPGA_PAD_SOURCE)
		return -EINVAL;

	mutex_lock(&fpga->lock);

	if (!fpga->streaming) {
		ret = 0;
		goto unlock;
	}

	/*
	 * Reverse the enable order: stop the sensor first, then disable
	 * the FPGA. Don't bail out early on error - still try to leave the
	 * FPGA in a sane state.
	 */
	ret = 0;
	if (fpga->sensor_sd) {
		ret = v4l2_subdev_disable_streams(fpga->sensor_sd,
						  fpga->sensor_pad, BIT(0));
		if (ret)
			dev_err(&fpga->client->dev,
				"failed to disable upstream sensor stream: %d\n",
				ret);
	}

	err = li_fpga_write_reg(fpga, LI_FPGA_REG_STREAM,
				LI_FPGA_STREAM_DISABLE);
	if (err && !ret)
		ret = err;

	fpga->streaming = false;

unlock:
	mutex_unlock(&fpga->lock);
	return ret;
}

static const struct v4l2_subdev_pad_ops li_fpga_pad_ops = {
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= li_fpga_set_fmt,
	.enable_streams		= li_fpga_enable_streams,
	.disable_streams	= li_fpga_disable_streams,
};

static const struct v4l2_subdev_ops li_fpga_subdev_ops = {
	.pad = &li_fpga_pad_ops,
};

static const struct media_entity_operations li_fpga_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int li_fpga_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *sink_fmt, *source_fmt;

	sink_fmt = v4l2_subdev_state_get_format(state, LI_FPGA_PAD_SINK, 0);
	source_fmt = v4l2_subdev_state_get_format(state, LI_FPGA_PAD_SOURCE, 0);
	if (!sink_fmt || !source_fmt)
		return -EINVAL;

	/* Safe default; sensor will overwrite via set_fmt. */
	sink_fmt->width		= 1920;
	sink_fmt->height	= 1080;
	sink_fmt->code		= MEDIA_BUS_FMT_SRGGB12_1X12;
	sink_fmt->field		= V4L2_FIELD_NONE;
	sink_fmt->colorspace	= V4L2_COLORSPACE_RAW;
	*source_fmt = *sink_fmt;

	return 0;
}

static const struct v4l2_subdev_internal_ops li_fpga_internal_ops = {
	.init_state = li_fpga_init_state,
};

static int li_fpga_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *asc)
{
	struct li_fpga *fpga =
		container_of(notifier, struct li_fpga, notifier);
	int pad, ret;

	pad = media_entity_get_fwnode_pad(&subdev->entity,
					  fpga->sensor_ep_fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(&fpga->client->dev,
			"failed to find source pad on %s: %d\n",
			subdev->name, pad);
		return pad;
	}

	fpga->sensor_sd = subdev;
	fpga->sensor_pad = pad;

	ret = media_create_pad_link(&subdev->entity, pad,
				    &fpga->sd.entity, LI_FPGA_PAD_SINK,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(&fpga->client->dev,
			"failed to link %s:%u -> %s:%u: %d\n",
			subdev->name, pad,
			fpga->sd.name, LI_FPGA_PAD_SINK, ret);
		fpga->sensor_sd = NULL;
		return ret;
	}

	dev_dbg(&fpga->client->dev, "bound upstream sensor %s pad %u\n",
		subdev->name, pad);
	return 0;
}

static void li_fpga_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *asc)
{
	struct li_fpga *fpga =
		container_of(notifier, struct li_fpga, notifier);

	fpga->sensor_sd = NULL;
}

static const struct v4l2_async_notifier_operations li_fpga_notify_ops = {
	.bound	= li_fpga_notify_bound,
	.unbind	= li_fpga_notify_unbind,
};

static int li_fpga_parse_of(struct li_fpga *fpga)
{
	struct device *dev = &fpga->client->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct fwnode_handle *local_ep;
	int nr, ret;

	local_ep = fwnode_graph_get_endpoint_by_id(fwnode, LI_FPGA_PAD_SINK,
						   0, 0);
	if (!local_ep) {
		dev_err(dev, "no sink endpoint (port@%d) in DT\n",
			LI_FPGA_PAD_SINK);
		return -ENODEV;
	}

	fpga->sensor_ep_fwnode = fwnode_graph_get_remote_endpoint(local_ep);
	fwnode_handle_put(local_ep);
	if (!fpga->sensor_ep_fwnode) {
		dev_err(dev, "sink endpoint has no remote - missing sensor link\n");
		return -ENODEV;
	}

	/*
	 * Read link-frequencies from the sensor endpoint so we can publish
	 * V4L2_CID_LINK_FREQ on our own subdev at probe time.
	 */
	nr = fwnode_property_count_u64(fpga->sensor_ep_fwnode,
				       "link-frequencies");
	if (nr <= 0) {
		dev_err(dev,
			"sensor endpoint has no 'link-frequencies' property (%d)\n",
			nr);
		ret = -EINVAL;
		goto err_put;
	}

	fpga->link_frequencies = devm_kcalloc(dev, nr,
					      sizeof(*fpga->link_frequencies),
					      GFP_KERNEL);
	if (!fpga->link_frequencies) {
		ret = -ENOMEM;
		goto err_put;
	}

	ret = fwnode_property_read_u64_array(fpga->sensor_ep_fwnode,
					     "link-frequencies",
					     fpga->link_frequencies, nr);
	if (ret) {
		dev_err(dev, "failed to read link-frequencies: %d\n", ret);
		goto err_put;
	}

	fpga->nr_link_frequencies = nr;

	fwnode_property_read_u32(fwnode, "li,stream-on-delay-us",
				 &fpga->stream_on_delay_us);

	return 0;

err_put:
	fwnode_handle_put(fpga->sensor_ep_fwnode);
	fpga->sensor_ep_fwnode = NULL;
	return ret;
}

static int li_fpga_register_notifier(struct li_fpga *fpga)
{
	struct li_fpga_asc *asc;
	int ret;

	v4l2_async_subdev_nf_init(&fpga->notifier, &fpga->sd);

	asc = v4l2_async_nf_add_fwnode(&fpga->notifier,
				       fpga->sensor_ep_fwnode,
				       struct li_fpga_asc);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		dev_err(&fpga->client->dev,
			"failed to add async subdev: %d\n", ret);
		v4l2_async_nf_cleanup(&fpga->notifier);
		return ret;
	}
	asc->fpga = fpga;

	fpga->notifier.ops = &li_fpga_notify_ops;

	ret = v4l2_async_nf_register(&fpga->notifier);
	if (ret) {
		dev_err(&fpga->client->dev,
			"failed to register async notifier: %d\n", ret);
		v4l2_async_nf_cleanup(&fpga->notifier);
		return ret;
	}

	return 0;
}

static void li_fpga_unregister_notifier(struct li_fpga *fpga)
{
	v4l2_async_nf_unregister(&fpga->notifier);
	v4l2_async_nf_cleanup(&fpga->notifier);
}

static int li_fpga_init_ctrls(struct li_fpga *fpga)
{
	struct v4l2_ctrl_handler *hdl = &fpga->ctrls;
	s64 link_freq_hz = fpga->link_frequencies[0];
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 2);
	if (ret)
		return ret;

	fpga->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL,
			V4L2_CID_LINK_FREQ,
			fpga->nr_link_frequencies - 1, 0,
			(const s64 *)fpga->link_frequencies);

	/*
	 * PIXEL_RATE here is a coarse placeholder (2 * link_freq). Anything
	 * that needs the real value will get it from the sensor's own
	 * control handler via the media pipeline.
	 */
	fpga->pixel_rate = v4l2_ctrl_new_std(hdl, NULL,
			V4L2_CID_PIXEL_RATE,
			link_freq_hz * 2, link_freq_hz * 2, 1,
			link_freq_hz * 2);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	if (fpga->link_freq)
		fpga->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (fpga->pixel_rate)
		fpga->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	fpga->sd.ctrl_handler = hdl;
	return 0;
}

static void li_fpga_free_ctrls(struct li_fpga *fpga)
{
	v4l2_ctrl_handler_free(&fpga->ctrls);
}

static int li_fpga_register_subdev(struct li_fpga *fpga)
{
	struct v4l2_subdev *sd = &fpga->sd;
	int ret;

	v4l2_i2c_subdev_init(sd, fpga->client, &li_fpga_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &li_fpga_media_ops;
	sd->internal_ops = &li_fpga_internal_ops;

	fpga->pads[LI_FPGA_PAD_SINK].flags   = MEDIA_PAD_FL_SINK;
	fpga->pads[LI_FPGA_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = li_fpga_init_ctrls(fpga);
	if (ret)
		return ret;

	ret = media_entity_pads_init(&sd->entity, LI_FPGA_NUM_PADS, fpga->pads);
	if (ret)
		goto err_free_ctrls;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_entity_cleanup;

	ret = li_fpga_register_notifier(fpga);
	if (ret)
		goto err_sd_cleanup;

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_nf_unregister;

	return 0;

err_nf_unregister:
	li_fpga_unregister_notifier(fpga);
err_sd_cleanup:
	v4l2_subdev_cleanup(sd);
err_entity_cleanup:
	media_entity_cleanup(&sd->entity);
err_free_ctrls:
	li_fpga_free_ctrls(fpga);
	return ret;
}

static void li_fpga_unregister_subdev(struct li_fpga *fpga)
{
	v4l2_async_unregister_subdev(&fpga->sd);
	li_fpga_unregister_notifier(fpga);
	v4l2_subdev_cleanup(&fpga->sd);
	media_entity_cleanup(&fpga->sd.entity);
	li_fpga_free_ctrls(fpga);
}

static int li_fpga_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct li_fpga *fpga;
	int ret;

	fpga = devm_kzalloc(dev, sizeof(*fpga), GFP_KERNEL);
	if (!fpga)
		return -ENOMEM;

	fpga->client = client;
	mutex_init(&fpga->lock);
	i2c_set_clientdata(client, fpga);

	ret = li_fpga_parse_of(fpga);
	if (ret)
		return ret;

	ret = li_fpga_register_subdev(fpga);
	if (ret) {
		dev_err(dev, "failed to register V4L2 subdev: %d\n", ret);
		fwnode_handle_put(fpga->sensor_ep_fwnode);
		return ret;
	}

	dev_info(dev, "Leopard Imaging FPGA bridge registered\n");
	return 0;
}

static void li_fpga_remove(struct i2c_client *client)
{
	struct li_fpga *fpga = i2c_get_clientdata(client);

	li_fpga_unregister_subdev(fpga);
	fwnode_handle_put(fpga->sensor_ep_fwnode);
}

static const struct i2c_device_id li_fpga_id[] = {
	{ "fpga", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, li_fpga_id);

static const struct of_device_id li_fpga_of_match[] = {
	{ .compatible = "nvidia,fpga" },
	{ }
};
MODULE_DEVICE_TABLE(of, li_fpga_of_match);

static struct i2c_driver li_fpga_i2c_driver = {
	.driver = {
		.name		= "li-fpga",
		.of_match_table	= li_fpga_of_match,
	},
	.probe		= li_fpga_probe,
	.remove		= li_fpga_remove,
	.id_table	= li_fpga_id,
};
module_i2c_driver(li_fpga_i2c_driver);

MODULE_DESCRIPTION("Leopard Imaging FPGA V4L2 subdevice bridge driver");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_AUTHOR("SiMa Technologies, Inc.");
MODULE_LICENSE("GPL v2");
