// SPDX-License-Identifier: GPL-2.0
/*
 * simor I2C camera sensor driver for V4L2 / Media Controller bring-up.
 *
 * Derived from Metoak camera S315 sensor driver.                                                                                                                                                                                                       
 *                                                                                                                                                                                                                                                      
 * Copyright (C) Metoak                                                                                                                                                                                                                                 
 * Copyright (C) 2026 Sima.ai
 *
 *
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* ---- sensor width and height------------------------------ */
#define SIMOR_WIDTH 1920
#define SIMOR_HEIGHT 360
/*
 * MIPI bus format. For RGB888 sensor data sent over 8 bit raw :
 * MEDIA_BUS_FMT_SBGGR8_1X8  (8 bit raw)
 * 
 */
#define SIMOR_MBUS_CODE MEDIA_BUS_FMT_SBGGR8_1X8

#define SIMOR_NUM_LANES 2
#define SIMOR_BPP 8 /* SBGGR8 = 8 bits/pixel on the bus */

/* PIXEL_RATE in pixels/sec, as seen by the receiver */
#define SIMOR_PIXEL_RATE(freq, lanes) (((u64)(freq)*2 * (lanes)) / SIMOR_BPP)

/* ------------------------------------------------------------------------- */

struct simor_sensor {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
	struct i2c_client *client;

	unsigned int num_data_lanes;
	u64 link_freq;
	unsigned int link_freq_index;
	u64 pixel_rate;
};

static const s64 simor_link_freqs[] = {
	450000000,
	480000000,
	540000000,
};

static inline struct simor_sensor *to_simor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct simor_sensor, sd);
}

/* ===== Pad ops: format negotiation ====================================== */

static int simor_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = SIMOR_MBUS_CODE;
	return 0;
}

static int simor_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != SIMOR_MBUS_CODE)
		return -EINVAL;
	fse->min_width = fse->max_width = SIMOR_WIDTH;
	fse->min_height = fse->max_height = SIMOR_HEIGHT;
	return 0;
}

static int simor_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *fmt)
{
	/* We support exactly one format - clamp to it */
	fmt->format.code = SIMOR_MBUS_CODE;
	fmt->format.width = SIMOR_WIDTH;
	fmt->format.height = SIMOR_HEIGHT;
	fmt->format.field = V4L2_FIELD_NONE;

	fmt->format.colorspace   = V4L2_COLORSPACE_DEFAULT;    
        fmt->format.ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;     
        fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;   /* = 0 */
        fmt->format.xfer_func    = V4L2_XFER_FUNC_DEFAULT;

	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	return 0;
}

/* ===== Video ops: stream on/off (NO-OP, by design) ====================== */




static int simor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct simor_sensor *simor = container_of(sd, struct simor_sensor, sd);

	mutex_lock(&simor->lock);

	/* Intentional no-op: userspace handles sensor streaming over I2C. */
	dev_dbg(sd->dev, "s_stream(%d) - no-op\n", enable);

        if (enable) {
                /*
                 * Apply default & customized values
                 * and then start streaming.
                 */  
             
        } else {
                
        }

	mutex_unlock(&simor->lock);
	return 0;
}

/* ===== Op tables ======================================================== */

static const struct v4l2_subdev_video_ops simor_video_ops = {
	.s_stream = simor_s_stream,
};

static const struct v4l2_subdev_pad_ops simor_pad_ops = {
	.enum_mbus_code = simor_enum_mbus_code,
	.enum_frame_size = simor_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt, /* core helper, reads state */
	.set_fmt = simor_set_fmt,
};

static const struct v4l2_subdev_ops simor_subdev_ops = {
	.video = &simor_video_ops,
	.pad = &simor_pad_ops,
};

static const struct media_entity_operations simor_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ===== init_state: default format applied to fresh subdev state ========= */

static int simor_init_state(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, 0);

	fmt->code = SIMOR_MBUS_CODE;
	fmt->width = SIMOR_WIDTH;
	fmt->height = SIMOR_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	return 0;
}

static const struct v4l2_subdev_internal_ops simor_internal_ops = {
	.init_state = simor_init_state,
};

/* ===== Controls (mandatory for many CSI bridges) ======================== */
static int simor_find_link_freq_index(u64 freq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(simor_link_freqs); i++) {
		if (freq == simor_link_freqs[i])
			return i;
	}

	return -EINVAL;
}

static int simor_parse_hw_config(struct simor_sensor *s)
{
	struct device *dev = s->dev;
	struct fwnode_handle *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -ENXIO, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		s->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
		break;

	default:
		ret = dev_err_probe(dev, -EINVAL,
				    "invalid CSI-2 data lanes: %u\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(dev, -EINVAL,
				    "no link-frequencies defined\n");
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		int idx;

		idx = simor_find_link_freq_index(bus_cfg.link_frequencies[i]);
		if (idx < 0)
			continue;

		s->link_freq = bus_cfg.link_frequencies[i];
		s->link_freq_index = idx;

		dev_info(dev, "CSI-2 lanes=%u link_freq=%llu Hz index=%u\n",
			 s->num_data_lanes, s->link_freq, s->link_freq_index);

		ret = 0;
		goto done_endpoint_free;
	}

	ret = dev_err_probe(dev, -EINVAL,
			    "DTS link-frequencies do not match driver table\n");

done_endpoint_free:
	s->pixel_rate = SIMOR_PIXEL_RATE(s->link_freq, s->num_data_lanes);

	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int simor_init_controls(struct simor_sensor *s)
{
	struct v4l2_ctrl_handler *hdl = &s->ctrls;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 2);
	if (ret)
		return ret;

	/* LINK_FREQ - many CSI bridges need this to configure the PHY */
	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(simor_link_freqs) - 1,
				      s->link_freq_index, simor_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* PIXEL_RATE - some bridges/userspace tools require this */
	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, s->pixel_rate,
				 s->pixel_rate, 1, s->pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	s->sd.ctrl_handler = hdl;
	return 0;
}

/* ===== Probe / remove =================================================== */

static int simor_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct simor_sensor *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->client = client;
	s->dev = dev;

	mutex_init(&s->lock);

	/* Initialize subdev (this sets sd->dev, sd->name, owner, etc.) */
	v4l2_i2c_subdev_init(&s->sd, client, &simor_subdev_ops);
	s->sd.internal_ops = &simor_internal_ops;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = simor_parse_hw_config(s);
	if (ret)
		return dev_err_probe(dev, ret, "parse hw config  failed\n");

	/* Controls */
	ret = simor_init_controls(s);
	if (ret)
		return dev_err_probe(dev, ret, "ctrl init failed\n");

	/* Media pad: one source pad (sensor output) */
	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s->sd.entity.ops = &simor_media_ops;
	ret = media_entity_pads_init(&s->sd.entity, 1, &s->pad);
	if (ret)
		goto err_ctrls;

	/* Allocate per-handle subdev state machinery */
	s->sd.state_lock = &s->lock;
	ret = v4l2_subdev_init_finalize(&s->sd);
	if (ret)
		goto err_entity;

	/* Register with async core - bridge driver binds when DT graph matches */
	ret = v4l2_async_register_subdev_sensor(&s->sd);
	if (ret)
		goto err_finalize;

	/* Runtime PM (no-op on this driver but keeps frameworks happy) */
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	dev_info(dev, "simor sensor registered: %dx%d, mbus=0x%x\n",
		 SIMOR_WIDTH, SIMOR_HEIGHT, SIMOR_MBUS_CODE);
	return 0;

err_finalize:
	v4l2_subdev_cleanup(&s->sd);
err_entity:
	media_entity_cleanup(&s->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s->ctrls);
	return ret;
}

static void simor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct simor_sensor *s = to_simor(sd);

	pm_runtime_disable(&client->dev);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->ctrls);
}

/* ===== DT / I2C boilerplate ============================================= */

static const struct of_device_id simor_of_match[] = { { .compatible ="Metoak,simor" },
						      {} };
MODULE_DEVICE_TABLE(of, simor_of_match);

static const struct i2c_device_id simor_id[] = { { "simor_metoak", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, simor_id);

static struct i2c_driver simor_i2c_driver = {
    .driver = {
        .name           = "simor_metoak",
        .of_match_table = simor_of_match,
    },
    .probe    = simor_probe,
    .remove   = simor_remove,
    .id_table = simor_id,
};
module_i2c_driver(simor_i2c_driver);

MODULE_AUTHOR("Sagar Kontam <sagar.kontam@sima.ai>");
MODULE_DESCRIPTION("Metoak simor sensor driver");
MODULE_LICENSE("GPL v2");
