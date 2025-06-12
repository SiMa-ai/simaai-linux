/*
 *
 * DWC MIPI CSI-2 Host Video Platform device driver
 *
 * Based on S5P/EXYNOS4 SoC series camera host interface media device Driver
 * Copyright (C) 2011 - 2013 Samsung Electronics Co., Ltd.
 * Author: Sylwester Nawrocki <s.nawro...@samsung.com>
 *
 * Copyright (C) 2016 Synopsys, Inc. All rights reserved.
 * Author: Ramiro Oliveira <ramiro.olive...@synopsys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <media/v4l2-async.h>
#include "csi_video_plat.h"

struct subdev_async_priv_data {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *sd;
};

static int
__plat_csi_pipeline_s_format(struct plat_csi_media_pipeline *ep,
                            struct v4l2_subdev_format *fmt)
{
       struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
       static const u8 seq[IDX_MAX] = {IDX_SENSOR, IDX_CSI, IDX_VDEV};

       fmt->pad = 0;
       fmt->which = V4L2_SUBDEV_FORMAT_ACTIVE;
       v4l2_subdev_call(p->subdevs[seq[IDX_CSI]], pad, set_fmt, NULL, fmt);
       v4l2_subdev_call(p->subdevs[seq[IDX_SENSOR]], pad, set_fmt, NULL, fmt);

       return 0;
}

static void
plat_csi_pipeline_prepare(struct plat_csi_pipeline *p, struct media_entity *me)
{
       struct v4l2_subdev *sd;
       unsigned int i = 0;

       for (i = 0; i < IDX_MAX; i++)
               p->subdevs[i] = NULL;

       while (1) {
               struct media_pad *pad = NULL;

               for (i = 0; i < me->num_pads; i++) {
                       struct media_pad *spad = &me->pads[i];

                       if (!(spad->flags & MEDIA_PAD_FL_SINK))
                               continue;

                       pad = media_pad_remote_pad_first(spad);
                       if (pad)
                               break;
               }
               if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
                       break;

               sd = media_entity_to_v4l2_subdev(pad->entity);

               switch (sd->grp_id) {
               case GRP_ID_SENSOR:
                       p->subdevs[IDX_SENSOR] = sd;
                       break;
               case GRP_ID_CSI:
                       p->subdevs[IDX_CSI] = sd;
                       break;
               case GRP_ID_VIDEODEV:
                       p->subdevs[IDX_VDEV] = sd;
                       break;
               default:
                       break;
               }
               me = &sd->entity;
               if (me->num_pads == 1)
                       break;
       }
}

static int __subdev_set_power(struct v4l2_subdev *sd, int on)
{
       int *use_count;
       int ret;

       if (sd == NULL) {
               pr_err("Null subdev\n");
               return -ENXIO;
       }
       use_count = &sd->entity.use_count;
       if (on && (*use_count)++ > 0)
               return 0;
       else if (!on && (*use_count == 0 || --(*use_count) > 0))
               return 0;

       ret = v4l2_subdev_call(sd, core, s_power, on);

       return ret != -ENOIOCTLCMD ? ret : 0;
}

static int plat_csi_pipeline_s_power(struct plat_csi_pipeline *p, bool on)
{
       static const u8 seq[IDX_MAX] = {IDX_CSI, IDX_SENSOR, IDX_VDEV};
       int i, ret = 0;

       for (i = 0; i < IDX_MAX; i++) {
               unsigned int idx = seq[i];

               if (p->subdevs[idx] == NULL)
                       pr_info("No device registered on %d\n", idx);
               else {
                       ret = __subdev_set_power(p->subdevs[idx], on);
                       if (ret < 0 && ret != -ENXIO)
                               goto error;
               }
       }
       return 0;
error:
       for (; i >= 0; i--) {
               unsigned int idx = seq[i];

               __subdev_set_power(p->subdevs[idx], !on);
       }
       return ret;
}

static int
__plat_csi_pipeline_open(struct plat_csi_media_pipeline *ep,
                        struct media_entity *me, bool prepare)
{
       struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
       int ret;

       if (WARN_ON(p == NULL || me == NULL))
               return -EINVAL;

       if (prepare)
               plat_csi_pipeline_prepare(p, me);

       ret = plat_csi_pipeline_s_power(p, 1);
       if (!ret)
               return 0;

       return ret;
}

static int __plat_csi_pipeline_close(struct plat_csi_media_pipeline *ep)
{
       struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
       int ret;

       ret = plat_csi_pipeline_s_power(p, 0);

       return ret == -ENXIO ? 0 : ret;
}

static int
__plat_csi_pipeline_s_stream(struct plat_csi_media_pipeline *ep, bool on)
{
       static const u8 seq[IDX_MAX] = {IDX_SENSOR, IDX_CSI, IDX_VDEV};
       struct plat_csi_pipeline *p = to_plat_csi_pipeline(ep);
       int i, ret = 0;

       for (i = 0; i < IDX_MAX; i++) {
               unsigned int idx = seq[i];

               if (p->subdevs[idx] == NULL)
                       pr_debug("s_Stream : No device registered on %d\n", idx);
               else {
                       ret =
                           v4l2_subdev_call(p->subdevs[idx], video, s_stream,
                                            on);

                       if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
                               goto error;
               }
       }
       return 0;
error:
       for (; i >= 0; i--) {
               unsigned int idx = seq[i];

               v4l2_subdev_call(p->subdevs[idx], video, s_stream, !on);
       }
       return ret;
}

static const struct plat_csi_media_pipeline_ops plat_csi_pipeline_ops = {
       .open = __plat_csi_pipeline_open,
       .close = __plat_csi_pipeline_close,
       .set_format = __plat_csi_pipeline_s_format,
       .set_stream = __plat_csi_pipeline_s_stream,
};

static struct plat_csi_media_pipeline *
plat_csi_pipeline_create(struct plat_csi_dev *plat_csi)
{
       struct plat_csi_pipeline *p;

       p = kzalloc(sizeof(*p), GFP_KERNEL);
       if (!p)
               return NULL;

       list_add_tail(&p->list, &plat_csi->pipelines);

       p->ep.ops = &plat_csi_pipeline_ops;
       return &p->ep;
}

static void
plat_csi_pipelines_free(struct plat_csi_dev *plat_csi)
{
       while (!list_empty(&plat_csi->pipelines)) {
               struct plat_csi_pipeline *p;

               p = list_entry(plat_csi->pipelines.next, typeof(*p), list);
               list_del(&p->list);
               kfree(p);
       }
}

static int plat_csi_register_sensor_entities(struct plat_csi_dev *plat_csi)
{
	struct device_node *parent = plat_csi->pdev->dev.of_node;
	struct device_node *child;
	struct device_node *node;
	struct device *dev = &plat_csi->pdev->dev;
	struct device_node *remote = NULL;
	struct v4l2_fwnode_endpoint endpoint;
	struct plat_csi_source_info *pd = NULL;
        struct subdev_async_priv_data *subdev_async_pd = NULL;
	int index = 0;
	int ret;

	for_each_available_child_of_node(parent, child) {
                for_each_endpoint_of_node(child, node) {
                        pd = &plat_csi->sensor[index].pdata;
			remote = of_graph_get_remote_port_parent(node);
			if (!remote) {
				dev_err(dev, "ERROR : getting remote port\n");
				ret = -EINVAL;
				goto err_cleanup;
			}
			dev_dbg(dev, "Found remote port %s\n", remote->name);
			ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &endpoint);
			if( ret < 0) {
				dev_err(dev, "ERROR : endpoint parsing failed\n");
				return ret;
			}

			dev_dbg(dev, "INFO : endpoint id %d, endpoint port %d\n", endpoint.base.id,
							endpoint.base.port);
			pd->mux_id = endpoint.base.port - 1;
			subdev_async_pd = v4l2_async_nf_add_fwnode(
                                        &plat_csi->subdev_notifier,
                                        of_fwnode_handle(remote),
                                        struct subdev_async_priv_data);
                        if(IS_ERR(subdev_async_pd)) {
                                dev_err(dev, "ERROR : registering notifier for %s\n", remote->name);
				ret = PTR_ERR(subdev_async_pd);
				goto err_cleanup;
			}
			plat_csi->sensor[index].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
			plat_csi->sensor[index].asd.match.fwnode  = of_fwnode_handle(remote);
			plat_csi->async_subdevs[index] = &plat_csi->sensor[index].asd;
			dev_dbg(dev, "SUCCESS : registered remote endpoint at index %d\n", index);
			index++;
		}
	}

	return 0;

err_cleanup:

	return ret;
}

static int register_videodev_entity(struct plat_csi_dev *plat_csi,
                        struct video_device_dev *vid_dev)
{
	struct v4l2_subdev *sd;
	struct plat_csi_media_pipeline *ep;
	int ret;

	sd = &vid_dev->subdev;
	sd->grp_id = GRP_ID_VIDEODEV;

	ep = plat_csi_pipeline_create(plat_csi);
	if (!ep) {
		dev_err(&plat_csi->pdev->dev, "ERROR : creating csi pipeline\n");
		return -ENOMEM;
	}

	v4l2_set_subdev_hostdata(sd, ep);

	ret = v4l2_device_register_subdev(&plat_csi->v4l2_dev, sd);
	if (ret < 0) {
		v4l2_err(&plat_csi->v4l2_dev, "Failed to register Video Device\n");
		return ret;
	}

	if(!vid_dev->dma) {
		dev_err(&plat_csi->pdev->dev, "ERROR : dma channel is not configured\n");
		return -EINVAL;
	}

	dev_err(&plat_csi->pdev->dev, "INFO registering dma channel %d\n", vid_dev->dma->chan_id);

	plat_csi->vid_dev[vid_dev->dma->chan_id] = vid_dev;

	dev_err(&plat_csi->pdev->dev, "SUCCESS : registering video entity\n");

	return ret;
}

static int register_mipi_csi_entity(struct plat_csi_dev *plat_csi,
                        struct platform_device *pdev, struct mipi_csi_dev *mipi_csi)
{
	int ret;
	struct device_node *parent = pdev->dev.of_node;
	struct device_node *child_port = NULL;
	unsigned int id = 0;

	dev_err(&plat_csi->pdev->dev, "parent %s\n", parent->name);
	for_each_available_child_of_node(parent, child_port) { 
		dev_err(&plat_csi->pdev->dev, "child %s\n", child_port->name);
		of_property_read_u32(child_port, "reg", &id);	

		id = id - 1;
		dev_err(&plat_csi->pdev->dev, "INFO : id is %u\n", id);

		if (WARN_ON(id < 0 || id >= CSI_MAX_ENTITIES)) {
			dev_err(&plat_csi->pdev->dev, "INFO : id is beyond limit %u\n", CSI_MAX_ENTITIES);
			return -ENOENT;
		}

		if (WARN_ON(plat_csi->mipi_csi[id].sd)) {
			dev_err(&plat_csi->pdev->dev, "subdevice is already set for id %u !!!\n", id);
			return -EBUSY;
		}

		if(!mipi_csi) {
			dev_err(&plat_csi->pdev->dev, "ERROR : invalid mipi csi pdev\n");
			return -ENOENT;
		}

		mipi_csi->sd[id].grp_id = GRP_ID_CSI;	

		ret = v4l2_device_register_subdev(&plat_csi->v4l2_dev, &mipi_csi->sd[id]);
		if ( ret < 0) {
			dev_err(&plat_csi->pdev->dev, "ERROR registering mipi-csi entity\n");
			return ret;
		}

		if (!ret)
			plat_csi->mipi_csi[id].sd = &mipi_csi->sd[id];
		else
			v4l2_err(&plat_csi->v4l2_dev, "Failed to register MIPI-CSI.%d (%d)\n",
					id, ret);
	}

	dev_err(&plat_csi->pdev->dev, "SUCCESS registering mipi-csi entity\n");

	return 0;
}

static int plat_csi_register_platform_entity(struct plat_csi_dev *plat_csi,
                               struct platform_device *pdev, int plat_entity)
{
       struct device *dev = &pdev->dev;
       int ret = -EPROBE_DEFER;
       void *drvdata;

       device_lock(dev);

	if (!dev->driver || !try_module_get(dev->driver->owner)) {
		dev_err(&plat_csi->pdev->dev, "Module %s is not loaded\n", dev_name(dev));
		goto dev_unlock;
	}

	drvdata = dev_get_drvdata(dev);

       if (drvdata) {
               switch (plat_entity) {
               case IDX_VDEV:
                       ret = register_videodev_entity(plat_csi, drvdata);
                       break;
               case IDX_CSI:
                       ret = register_mipi_csi_entity(plat_csi, pdev, drvdata);
                       break;
               default:
                       ret = -ENODEV;
               }
       } else {
			dev_err(&plat_csi->pdev->dev, "%s no drvdata\n", dev_name(dev));
		}

       module_put(dev->driver->owner);
dev_unlock:
       device_unlock(dev);
       if (ret == -EPROBE_DEFER)
               dev_err(&plat_csi->pdev->dev,
                        "deferring %s device registration\n", dev_name(dev));
       else if (ret < 0)
               dev_err(&plat_csi->pdev->dev,
                       "%s device registration failed (%d)\n", dev_name(dev),
                       ret);
       return ret;
}

static int
plat_csi_register_platform_entities(struct plat_csi_dev *plat_csi,
                                   struct device_node *parent)
{
	struct device_node *node;
	int ret = 0;

	for_each_available_child_of_node(parent, node) {
		struct platform_device *pdev;
		int plat_entity = -1;

		dev_err(&plat_csi->pdev->dev, "INFO : processing %s\n", node->name);

		pdev = of_find_device_by_node(node);
		if (!pdev) {
			dev_err(&plat_csi->pdev->dev, "pdev not found for %s\n", node->name);
			continue;
		}

		if (!strncmp(node->name, VIDEODEV_OF_NODE_NAME, strlen(VIDEODEV_OF_NODE_NAME)))
        	plat_entity = IDX_VDEV;
		else if (!strncmp(node->name, CSI_OF_NODE_NAME, strlen(CSI_OF_NODE_NAME)))
        	plat_entity = IDX_CSI;

		if (plat_entity >= 0)
        	ret = plat_csi_register_platform_entity(plat_csi, pdev, plat_entity);
		else 
			dev_err(&plat_csi->pdev->dev, "INFO : ignoring node %s\n", node->name);

		put_device(&pdev->dev);

		if (ret < 0) {
			dev_err(&plat_csi->pdev->dev, "ERROR : registering platform entity %s\n",
							node->name);
			break;
		}
    }

	return ret;
}

static void
plat_csi_unregister_entities(struct plat_csi_dev *plat_csi)
{
	int i;

	for (i = 0; i < CSI_MAX_ENTITIES; i++) {

		if (plat_csi->mipi_csi[i].sd != NULL) {
			v4l2_device_unregister_subdev(plat_csi->mipi_csi[i].sd);
			plat_csi->mipi_csi[i].sd = NULL;
		}

		if (plat_csi->vid_dev[i] != NULL) {
			v4l2_device_unregister_subdev(&plat_csi->vid_dev[i]->subdev);
       		plat_csi->vid_dev[i]->ve.pipe = NULL;	
       		plat_csi->vid_dev[i] = NULL;
		}
	}

	dev_dbg(plat_csi->dev, "Unregistered all entities\n");
}

static int
__plat_csi_create_videodev_sink_links(struct plat_csi_dev *plat_csi,
                                     struct media_entity *source,
                                     int pad, int index)
{
       struct media_entity *sink;
       int ret = 0;
       struct video_device_dev *vid_dev = plat_csi->vid_dev[index];

	if (!vid_dev) {
		dev_err(&plat_csi->pdev->dev, "Video dev initialization fail for VC %d\n", index);
		return 0;
	}

       sink = &vid_dev->subdev.entity;
       dev_dbg(&plat_csi->pdev->dev, "Creating sink link [%s] -> [%s]\n",
                 source->name, sink->name);

       ret = media_create_pad_link(source, pad, sink,
                                   VIDEO_DEV_SD_PAD_SINK_CSI, MEDIA_LNK_FL_ENABLED);
       if (ret)
               return ret;

       dev_dbg(&plat_csi->pdev->dev, "Created sink link [%s] -> [%s]\n",
                 source->name, sink->name);

       return 0;
}

static int
__plat_csi_create_videodev_source_links(struct plat_csi_dev *plat_csi, int index)
{
       struct media_entity *source, *sink;
       int ret = 0;

       struct video_device_dev *vid_dev = plat_csi->vid_dev[index];

       if (vid_dev == NULL) {
               dev_err(&plat_csi->pdev->dev, "VC : %d Video device not present\n", index);
               return 0;
	   }

       source = &vid_dev->subdev.entity;
       sink = &vid_dev->ve.vdev.entity;

       dev_dbg(&plat_csi->pdev->dev, "Creating source link [%s] -> [%s]\n",
                 source->name, sink->name);

       ret = media_create_pad_link(source, VIDEO_DEV_SD_PAD_SOURCE_DMA,
                                   sink, 0, MEDIA_LNK_FL_ENABLED);
       if (ret)
               return ret;

       dev_dbg(&plat_csi->pdev->dev, "Created source link [%s] -> [%s]\n",
                 source->name, sink->name);

       return ret;
}

static int
plat_csi_create_links(struct plat_csi_dev *plat_csi)
{
	struct v4l2_subdev *csi_sensor[CSI_MAX_ENTITIES] = { NULL };
	struct v4l2_subdev *sensor, *csi;
	struct media_entity *source;
	struct plat_csi_source_info *pdata;
	int i, pad, ret = 0;

	for (i = 0; i < plat_csi->num_sensors; i++) {
		if (plat_csi->sensor[i].subdev == NULL) {
			dev_err(&plat_csi->pdev->dev, "for sensor %d, subdev is invalid\n",i);
        	continue;
		}

		sensor = plat_csi->sensor[i].subdev;
		pdata = v4l2_get_subdev_hostdata(sensor);
		if (!pdata) {
			dev_err(&plat_csi->pdev->dev, "creating links, pdata is not valid for %d\n",i);
			continue;
		}

		source = NULL;
		csi = plat_csi->mipi_csi[pdata->mux_id].sd;

		if (WARN(csi == NULL, "dw-mipi-csi module is not loaded!\n"))
        	return -EINVAL;

		pad = sensor->entity.num_pads - 1;
		ret = media_create_pad_link(&sensor->entity, pad,
                            &csi->entity, CSI_PAD_SINK,
                            MEDIA_LNK_FL_IMMUTABLE |
                            MEDIA_LNK_FL_ENABLED);
		if (ret < 0) {
			dev_err(&plat_csi->pdev->dev, "failed to create link\n");
			return ret;
		}
		csi_sensor[pdata->mux_id] = sensor;
	}

	for (i = 0; i < CSI_MAX_ENTITIES; i++) {
		if (plat_csi->mipi_csi[i].sd == NULL) {
			dev_err(&plat_csi->pdev->dev, "no link for VC %d\n", i);
			continue;
		}

		source = &plat_csi->mipi_csi[i].sd->entity;
		pad  = CSI_PAD_SOURCE;

		ret = __plat_csi_create_videodev_sink_links(plat_csi, source, pad, i);
		if( ret < 0) {
			dev_err(&plat_csi->pdev->dev, "ERROR : failed to create videodev sink link\n");
			return ret;
		}
	}

	for (i = 0; i < CSI_MAX_ENTITIES; i++) {

		ret = __plat_csi_create_videodev_source_links(plat_csi, i);
		if (ret < 0) {
			dev_err(&plat_csi->pdev->dev, "ERROR : failed to create videodev source link\n");
			return ret;
		}
	}

	return ret;
}

static int __plat_csi_modify_pipeline(struct media_entity *entity, bool enable)
{
       struct plat_csi_video_entity *ve;
       struct plat_csi_pipeline *p;
       struct video_device *vdev;
       int ret;

       vdev = media_entity_to_video_device(entity);

       if (vdev->entity.use_count == 0)
               return 0;

       ve = vdev_to_plat_csi_video_entity(vdev);
       p = to_plat_csi_pipeline(ve->pipe);

       if (enable)
               ret = __plat_csi_pipeline_open(ve->pipe, entity, true);
       else
               ret = __plat_csi_pipeline_close(ve->pipe);

       if (ret == 0 && !enable)
               memset(p->subdevs, 0, sizeof(p->subdevs));

       return ret;
}

static int
__plat_csi_modify_pipelines(struct media_entity *entity, bool enable,
                           struct media_graph *graph)
{
       struct media_entity *entity_err = entity;
       int ret;

       media_graph_walk_start(graph, entity);

       while ((entity = media_graph_walk_next(graph))) {
               if (!is_media_entity_v4l2_video_device(entity))
                       continue;

               ret = __plat_csi_modify_pipeline(entity, enable);

               if (ret < 0)
                       goto err;
       }

       return 0;

err:
       media_graph_walk_start(graph, entity_err);

       while ((entity_err = media_graph_walk_next(graph))) {
               if (!is_media_entity_v4l2_video_device(entity_err))
                       continue;

               __plat_csi_modify_pipeline(entity_err, !enable);

               if (entity_err == entity)
                       break;
       }

       return ret;
}

static int
plat_csi_link_notify(struct media_link *link, unsigned int flags,
                    unsigned int notification)
{
       struct media_graph *graph =
           &container_of(link->graph_obj.mdev, struct plat_csi_dev,
                         media_dev)->link_setup_graph;
       struct media_entity *sink = link->sink->entity;
       int ret = 0;

       if (notification == MEDIA_DEV_NOTIFY_PRE_LINK_CH) {
               ret = media_graph_walk_init(graph, link->graph_obj.mdev);
               if (ret)
                       return ret;
               if (!(flags & MEDIA_LNK_FL_ENABLED))
                       ret = __plat_csi_modify_pipelines(sink, false, graph);

       } else if (notification == MEDIA_DEV_NOTIFY_POST_LINK_CH) {
               if (link->flags & MEDIA_LNK_FL_ENABLED)
                       ret = __plat_csi_modify_pipelines(sink, true, graph);
               media_graph_walk_cleanup(graph);
       }

       return ret ? -EPIPE : 0;
}

static const struct media_device_ops plat_csi_media_ops = {
       .link_notify = plat_csi_link_notify,
};

static int
subdev_notifier_bound(struct v4l2_async_notifier *notifier,
                     struct v4l2_subdev *subdev, struct v4l2_async_subdev *asd)
{
	struct plat_csi_dev *plat_csi = notifier_to_plat_csi(notifier);
	struct plat_csi_sensor_info *si = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(plat_csi->sensor); i++) {
		if (plat_csi->sensor[i].asd.match.fwnode ==
						of_fwnode_handle(subdev->dev->of_node)) {
			dev_err(&plat_csi->pdev->dev, "BOUND : found match, %d\n", i);
			si = &plat_csi->sensor[i];
		}
	}

	if (si == NULL) {
		dev_err(&plat_csi->pdev->dev, "BOUND : no match found\n");
		return -EINVAL;
	}

   v4l2_set_subdev_hostdata(subdev, &si->pdata);
   si->subdev = subdev;
   subdev->grp_id = GRP_ID_SENSOR;

   dev_err(&plat_csi->pdev->dev, "BOUND : notification for sensor subdevice: %s (%d)\n",
             subdev->name, plat_csi->num_sensors);

   plat_csi->num_sensors++;

   return 0;
}

static int
subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct plat_csi_dev *plat_csi = notifier_to_plat_csi(notifier);
	int ret;

	mutex_lock(&plat_csi->media_dev.graph_mutex);

	ret = plat_csi_create_links(plat_csi);
	if (ret < 0) {
		dev_err(&plat_csi->pdev->dev, "ERROR : failed to create link\n");
		goto unlock;
	}

	ret = v4l2_device_register_subdev_nodes(&plat_csi->v4l2_dev);

unlock:
	mutex_unlock(&plat_csi->media_dev.graph_mutex);
	if (ret < 0) {
		dev_err(&plat_csi->pdev->dev, "ERROR : failed to register subdev, in complete notification\n");
        return ret;
	}

	return media_device_register(&plat_csi->media_dev);

}

static const struct v4l2_async_notifier_operations subdev_notifier_ops = {
	.bound = subdev_notifier_bound,
	.complete = subdev_notifier_complete,
};

static int plat_csi_probe(struct platform_device *pdev)
{
       struct device *dev = &pdev->dev;
       struct v4l2_device *v4l2_dev;
       struct plat_csi_dev *plat_csi;
       int ret;

       plat_csi = devm_kzalloc(dev, sizeof(*plat_csi), GFP_KERNEL);
       if (!plat_csi)
               return -ENOMEM;

       plat_csi->num_sensors = 0;

       spin_lock_init(&plat_csi->slock);
       INIT_LIST_HEAD(&plat_csi->pipelines);
       plat_csi->pdev = pdev;

       strlcpy(plat_csi->media_dev.model, "CSI Video Platform",
               sizeof(plat_csi->media_dev.model));
       plat_csi->media_dev.ops = &plat_csi_media_ops;
       plat_csi->media_dev.dev = dev;

       v4l2_dev = &plat_csi->v4l2_dev;
       v4l2_dev->mdev = &plat_csi->media_dev;
       strlcpy(v4l2_dev->name, "plat-csi", sizeof(v4l2_dev->name));

       media_device_init(&plat_csi->media_dev);

       ret = v4l2_device_register(dev, &plat_csi->v4l2_dev);
       if (ret < 0) {
               v4l2_err(v4l2_dev, "Fail to register v4l2 device: %d\n", ret);
               return ret;
       }

       platform_set_drvdata(pdev, plat_csi);
	
       plat_csi->v4l2_dev.dev = &pdev->dev;
	
       v4l2_async_nf_init(&plat_csi->subdev_notifier);
       plat_csi->subdev_notifier.ops = &subdev_notifier_ops;
       plat_csi->subdev_notifier.v4l2_dev = &plat_csi->v4l2_dev;

       ret = plat_csi_register_platform_entities(plat_csi, dev->of_node);
       if (ret)
               goto err_m_ent;

       ret = plat_csi_register_sensor_entities(plat_csi);
       if (ret) {
               dev_err(dev, "Fail to regiter sensor entity\n");
               goto err_m_ent;
	}

	ret = v4l2_async_nf_register(&plat_csi->v4l2_dev, &plat_csi->subdev_notifier);
	if (ret) {
		dev_err(dev, "Fail to register async notifier\n");
		goto err_m_ent;
	}
	
	dev_info(dev, "CSI video platform driver registered successfully!!!\n");

	return 0;

err_m_ent:

	plat_csi_unregister_entities(plat_csi);
	media_device_unregister(&plat_csi->media_dev);
	media_device_cleanup(&plat_csi->media_dev);
	v4l2_device_unregister(&plat_csi->v4l2_dev);

	return ret;
}

static int plat_csi_remove(struct platform_device *pdev)
{
       struct plat_csi_dev *dev = platform_get_drvdata(pdev);

       v4l2_async_nf_unregister(&dev->subdev_notifier);
       v4l2_device_unregister(&dev->v4l2_dev);
       plat_csi_unregister_entities(dev);
       plat_csi_pipelines_free(dev);
       media_device_unregister(&dev->media_dev);
       media_device_cleanup(&dev->media_dev);
       dev_info(&pdev->dev, "CSI video platform driver un-registered successfully!!!\n");
       return 0;
}

/**
 * @short of_device_id structure
 */
static const struct of_device_id plat_csi_of_match[] = {
       {.compatible = "snps,plat-csi"},
       {}
};

MODULE_DEVICE_TABLE(of, plat_csi_of_match);

/**
 * @short Platform driver structure
 */
static struct platform_driver plat_csi_pdrv = {
       .remove = plat_csi_remove,
       .probe = plat_csi_probe,
       .driver = {
                  .name = "snps,plat-csi",
                  .owner = THIS_MODULE,
                  .of_match_table = of_match_ptr(plat_csi_of_match),
                  },
};

static int __init
plat_csi_init(void)
{
	int rc = request_module("dw-mipi-csi");
	if (rc != 0) {
		printk (KERN_CRIT "Failed to load dw-mipi-csi module\n");
	}

	return platform_driver_register(&plat_csi_pdrv);
}

static void __exit
plat_csi_exit(void)
{
       platform_driver_unregister(&plat_csi_pdrv);
}

module_init(plat_csi_init);
module_exit(plat_csi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramiro Oliveira <roliv...@synopsys.com>");
MODULE_DESCRIPTION("Video Platform driver for MIPI CSI-2 Host");
