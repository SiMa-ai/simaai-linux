// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2021 Sima ai
 *
 * Author: Nileshkumar Raghuvanshi <nilesh.r@sima.ai>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/media-entity.h>

#include <asm/io.h>
#include "zebu_camera_config.h"

struct zebu_camera {
	struct device *dev;
	struct v4l2_subdev sd;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *image_type;
	struct v4l2_ctrl *frame_count;
	struct v4l2_ctrl *vc_mask;
	struct v4l2_ctrl *wrapper_mask;
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_pad pads;
#endif

	unsigned char frame_type;
	unsigned short nr_frames;
	unsigned char vc_disable_mask;
	unsigned char wrapper_disable_mask;

};

static void dump_ocm_config(struct zebu_camera *cam) {

	dev_err(cam->dev, "Frame type : %#x\n", cam->frame_type);
	dev_err(cam->dev, "Frame count : %#x\n", cam->nr_frames);
	dev_err(cam->dev, "VC disable mask : %#x\n", cam->vc_disable_mask);
	dev_err(cam->dev, "wrapper disable mask : %#x\n", cam->wrapper_disable_mask);

}

static int zebu_camera_set_stream(struct v4l2_subdev *sd, int enable) {

	struct device *dev = v4l2_get_subdevdata(sd);
	struct zebu_camera *cam = container_of(sd, struct zebu_camera, sd);
	unsigned char ocm_config = 0;

	/* OCM config memory pointer for zebu transactor */
	void __iomem* ocm_config_add;

	if(enable) {
		dev_err(dev, "start streaming\n");

		dump_ocm_config(cam);
		/*
		** OCM config space for zebu camera
		*/

		ocm_config_add = ioremap(FRAME_STREAM_ADD, 8);
		if (!ocm_config_add) {
			dev_err(dev, "ERROR : mapping %#x\n", FRAME_STREAM_ADD);
			return -EBUSY;
		}
	
		dev_err(dev, "Zebu camera : OCM config space mapped\n");

		ocm_config = (((cam->nr_frames & 0x3FF ) << 16) |
						((cam->wrapper_disable_mask & 0xF) << 12) |
						((cam->vc_disable_mask & 0xF) << 8) |
						(cam->frame_type & 0xFF));

		dev_err(dev, "ocm config %#x\n", ocm_config);
	
		writel(ocm_config, ocm_config_add);
	
		dev_err(cam->dev, "INFO : SUCCESS writing OCM config\n");

		iounmap(ocm_config_add);

	} else
		dev_err(dev, "stop streaming\n");


	return 0;
}

static int zebu_camera_set_ctrl(struct v4l2_ctrl *ctrl) {

	struct zebu_camera *cam = container_of(ctrl->handler, struct zebu_camera,
											ctrls);

	dev_err(cam->dev, "set ctrls invoked\n");
	switch (ctrl->id) {
		case  V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH:
			dev_err(cam->dev, "INFO : frame type set to %#x\n", ctrl->val);
			cam->frame_type = ctrl->val;
			break;
		case  V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH + 1:
			dev_err(cam->dev, "INFO : frame count set to %#x\n", ctrl->val);
			cam->nr_frames = ctrl->val;
			break;
		case  V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH + 2:
			dev_err(cam->dev, "INFO : VC disable mask set to %#x\n", ctrl->val);
			cam->vc_disable_mask = ctrl->val;
			break;
		case  V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH + 3:
			dev_err(cam->dev, "INFO : wrapper disable mask set to %#x\n", ctrl->val);
			cam->wrapper_disable_mask = ctrl->val;
			break;
		default :
			dev_err(cam->dev, "INFO : unknown ctrl %#x\n", ctrl->id);
	}

	return 0;

}

static int zebu_camera_get_volatile_ctrl(struct v4l2_ctrl *ctrl) {

	int ret = 0;
	struct zebu_camera *cam = container_of(ctrl->handler, struct zebu_camera,
											ctrls);
	dev_err(cam->dev, "get ctrls invoked\n");

	return 0;

}

static const struct v4l2_ctrl_ops zebu_camera_ctrl_ops = {
	.g_volatile_ctrl = zebu_camera_get_volatile_ctrl,
	.s_ctrl = zebu_camera_set_ctrl,
};

static const struct v4l2_ctrl_config zebu_camera_ctrl_image_type= {
    .ops        = &zebu_camera_ctrl_ops,
    .id     	= V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH,
    .type       = V4L2_CTRL_TYPE_INTEGER,
    .name       = "Image type",
    .min        = 0,
    .max        = 1023,
    .step       = 1,
    .def        = 0,
    .flags      = 0,
};

static const struct v4l2_ctrl_config zebu_camera_ctrl_frame_count = {
    .ops        = &zebu_camera_ctrl_ops,
    .id     	= V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH + 1,
    .type       = V4L2_CTRL_TYPE_INTEGER,
    .name       = "Frame count",
    .min        = 0,
    .max        = 1023,
    .step       = 1,
    .def        = 1,
    .flags      = 0,
};

static const struct v4l2_ctrl_config zebu_camera_ctrl_vc_mask = {
    .ops        = &zebu_camera_ctrl_ops,
    .id     	= V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH + 2,
    .type       = V4L2_CTRL_TYPE_INTEGER,
    .name       = "VC Disable mask",
    .min        = 0,
    .max        = 0xF,
    .step       = 1,
    .def        = 0,
    .flags      = 0,
};

static const struct v4l2_ctrl_config zebu_camera_ctrl_wrapper_mask = {
    .ops        = &zebu_camera_ctrl_ops,
    .id     	= V4L2_CID_SIMAAI_ZEBU_CAMERA_IMAGE_WIDTH + 3,
    .type       = V4L2_CTRL_TYPE_INTEGER,
    .name       = "Wrapper disable mask",
    .min        = 0,
    .max        = 0xF,
    .step       = 1,
    .def        = 0,
    .flags      = 0,
};

static int zebu_camera_get_fmt(struct v4l2_subdev *sd,
               struct v4l2_subdev_state *sd_state,
               struct v4l2_subdev_format *format) {

	struct zebu_camera *cam = container_of(sd, struct zebu_camera, sd);

	dev_err(cam->dev, "INFO : received get fmt call\n");

	return 0;
}

static int zebu_camera_set_fmt(struct v4l2_subdev *sd,
               struct v4l2_subdev_state *sd_state,
               struct v4l2_subdev_format *format) {

	struct zebu_camera *cam = container_of(sd, struct zebu_camera, sd);

	dev_err(cam->dev, "INFO : s_fmt : which %u, pad %u, w: %d, h: %d\n",
			format->which, format->pad, format->format.width, format->format.height);

	
	//writel((RAW12_8_1920_1080_FULLHD << 16 | 9), cam->ocm_config_add);

	return 0;

}
static int zebu_camera_init_cfg(struct v4l2_subdev *sd,  
							 struct v4l2_subdev_state *state) {

	struct zebu_camera *cam = container_of(sd, struct zebu_camera, sd);

	dev_err(cam->dev, "INFO : received init cfg call\n");

	return 0;
}


static const struct v4l2_subdev_video_ops zebu_camera_video_ops = {
	.s_stream = zebu_camera_set_stream,
};

static const struct v4l2_subdev_pad_ops zebu_camera_pad_ops = { 
	.init_cfg = zebu_camera_init_cfg,
	.get_fmt = zebu_camera_get_fmt,
	.set_fmt = zebu_camera_set_fmt,
};

static const struct v4l2_subdev_ops zebu_camera_subdev_ops = {
	.video = &zebu_camera_video_ops,
	.pad = &zebu_camera_pad_ops,
};

static int zebu_camera_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zebu_camera *cam = NULL;

	int ret = 0;

	dev_err(dev, "Zebu camera probe called\n");

	cam = devm_kzalloc(dev, sizeof(*cam), GFP_KERNEL);
	if(!cam) {
		dev_err(dev, "ERROR : allocating zebu camera driver structure\n");
		return -ENOMEM;
	}

	cam->dev = dev;

	/*
	** V4L2 sub device registeration
	*/

	v4l2_subdev_init(&cam->sd, &zebu_camera_subdev_ops);
	v4l2_ctrl_handler_init(&cam->ctrls, 4);

	cam->image_type = v4l2_ctrl_new_custom(&cam->ctrls,
			&zebu_camera_ctrl_image_type, NULL);

	cam->frame_count = v4l2_ctrl_new_custom(&cam->ctrls,
			&zebu_camera_ctrl_frame_count, NULL);

	cam->vc_mask = v4l2_ctrl_new_custom(&cam->ctrls,
			&zebu_camera_ctrl_vc_mask, NULL);

	cam->wrapper_mask = v4l2_ctrl_new_custom(&cam->ctrls,
			&zebu_camera_ctrl_wrapper_mask, NULL);

	cam->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	cam->sd.owner = THIS_MODULE;
	snprintf(cam->sd.name, sizeof(cam->sd.name), "%s", pdev->name);

	dev_err(dev, "name of sd is %s\n", cam->sd.name);

	cam->sd.dev = dev;
	cam->sd.ctrl_handler = &cam->ctrls;

#if defined(CONFIG_MEDIA_CONTROLLER)
	cam->pads.flags = MEDIA_PAD_FL_SOURCE;
	cam->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&cam->sd.entity, 1, &cam->pads);
	if(ret < 0) {
		dev_err(dev, "ERROR : failed to initialize pads\n");
		goto err_pads_init;
	}
	dev_err(dev, "INFO : media entity registered %#x\n", &(cam->sd.entity));
#endif

	v4l2_set_subdevdata(&cam->sd, pdev);

	ret = v4l2_async_register_subdev(&cam->sd);
	if (ret < 0) {
		dev_err(dev, "ERROR : registerting to async framework %d\n", ret);
		goto err_async_reg;
	}

	dev_err(dev,"zebu camera subdev name %s\n", cam->sd.name);
	platform_set_drvdata(pdev, cam); 
	dev_err(dev, "zebu camera probe completed\n");
	
	return 0;

err_async_reg:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&cam->sd.entity);
#endif
err_pads_init:
	v4l2_ctrl_handler_free(&cam->ctrls);

	return ret;
}

static int zebu_camera_remove(struct platform_device *pdev)
{
	struct zebu_camera *cam = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_err(dev, "Zebu camera remove called\n");

	v4l2_ctrl_handler_free(&cam->ctrls);

#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&cam->sd.entity);
#endif

	v4l2_async_unregister_subdev(&cam->sd);

	return 0;
}

static const struct of_device_id zebu_camera_match[] = {
	{ .compatible = "simaai,zebu-camera" },
	{},
};

MODULE_DEVICE_TABLE(of, zebu_camera_match);

static struct platform_driver zebu_camera_driver = {
	.probe	= zebu_camera_probe,
	.remove	= zebu_camera_remove,
	.driver	= {
		.name	= "zebu-camera",
		.of_match_table	= zebu_camera_match,
	},
};

module_platform_driver(zebu_camera_driver);

MODULE_AUTHOR("Nileshkumar Raghuvanshi <nilesh.r@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai camera driver for Michelangelo zebu emulation");
MODULE_LICENSE("Dual MIT/GPL");
