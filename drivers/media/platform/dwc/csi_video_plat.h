/*
 * Copyright (C) 2016 Synopsys, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PLAT_CSI_H_
#define PLAT_CSI_H_

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/dwc/csi_host_platform.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

#include "dw_mipi_csi.h"
#include "csi_video_device.h"

#define VIDEODEV_OF_NODE_NAME  "video-device"
#define CSI_OF_NODE_NAME       "csi"

enum plat_csi_subdev_index {
       IDX_SENSOR,
       IDX_CSI,
       IDX_VDEV,
       IDX_MAX,
};

struct plat_csi_sensor_info {
       struct plat_csi_source_info pdata;
       struct v4l2_async_subdev asd;
       struct v4l2_subdev *subdev;
       struct mipi_csi_dev *host;
};

struct plat_csi_pipeline {
       struct plat_csi_media_pipeline ep;
       struct list_head list;
       struct media_entity *vdev_entity;
       struct v4l2_subdev *subdevs[IDX_MAX];
};

#define to_plat_csi_pipeline(_ep)\
        container_of(_ep, struct plat_csi_pipeline, ep)

struct mipi_csi_info {
       struct v4l2_subdev *sd;
       int id;
};

/**
 * @short Structure to embed device driver information
 */
struct plat_csi_dev {
       struct mipi_csi_info            mipi_csi[CSI_MAX_ENTITIES];
       struct video_device_dev         *vid_dev[CSI_MAX_ENTITIES];
       struct device                   *dev;
       struct media_device             media_dev;
       struct v4l2_device              v4l2_dev;
       struct platform_device          *pdev;
       struct plat_csi_sensor_info     sensor[PLAT_MAX_SENSORS];
       struct v4l2_async_notifier      subdev_notifier;
       struct v4l2_async_subdev        *async_subdevs[PLAT_MAX_SENSORS];
       spinlock_t                      slock;
       struct list_head                pipelines;
       int                             num_sensors;
       struct media_graph      link_setup_graph;
};

static inline struct plat_csi_dev *
entity_to_plat_csi_mdev(struct media_entity *me)
{
       return me->graph_obj.mdev == NULL ? NULL :
           container_of(me->graph_obj.mdev, struct plat_csi_dev, media_dev);
}

static inline struct plat_csi_dev *
notifier_to_plat_csi(struct v4l2_async_notifier *n)
{
       return container_of(n, struct plat_csi_dev, subdev_notifier);
}

static inline void
plat_csi_graph_unlock(struct plat_csi_video_entity *ve)
{
       mutex_unlock(&ve->vdev.entity.graph_obj.mdev->graph_mutex);
}

#endif /* PLAT_CSI_H_ */
