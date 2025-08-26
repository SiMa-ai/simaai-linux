/*
 * Copyright (C) 2016 Synopsys, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SNPS_CSI_VIDEO_PLAT_INCLUDES_H_
#define SNPS_CSI_VIDEO_PLAT_INCLUDES_H_

#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

/*
 * The subdevices' group IDs.
 */

#define MAX_WIDTH      4096
#define MAX_HEIGHT     3280

#define MIN_WIDTH      640
#define MIN_HEIGHT     480

#define GRP_ID_SENSOR          (10)
#define GRP_ID_CSI             (20)
#define GRP_ID_VIDEODEV                (30)

#define CSI_MAX_ENTITIES       4
#define PLAT_MAX_SENSORS       4

enum video_dev_pads {
       VIDEO_DEV_SD_PAD_SINK_CSI = 0,
       VIDEO_DEV_SD_PAD_SOURCE_DMA = 1,
       VIDEO_DEV_SD_PADS_NUM = 2,
};

enum mipi_csi_pads {
       CSI_PAD_SINK = 0,
       CSI_PAD_SOURCE = 1,
       CSI_PADS_NUM = 2,
};

struct plat_csi_source_info {
       u16 flags;
       u16 mux_id;
};

struct plat_csi_fmt {
       char *name;
       u32 mbus_code;
       u32 fourcc;
       u8 depth;
};

struct plat_csi_media_pipeline;

/*
 * Media pipeline operations to be called from within a video node,  i.e. the
 * last entity within the pipeline. Implemented by related media device driver.
 */
struct plat_csi_media_pipeline_ops {
       int (*prepare)(struct plat_csi_media_pipeline *p,
                       struct media_entity *me);
       int (*unprepare)(struct plat_csi_media_pipeline *p);
       int (*open)(struct plat_csi_media_pipeline *p,
                       struct media_entity *me, bool resume);
       int (*close)(struct plat_csi_media_pipeline *p);
       int (*set_stream)(struct plat_csi_media_pipeline *p, bool state);
       int (*set_format)(struct plat_csi_media_pipeline *p,
                       struct v4l2_subdev_format *fmt);
};

struct plat_csi_video_entity {
       struct video_device vdev;
       struct plat_csi_media_pipeline *pipe;
};

struct plat_csi_media_pipeline {
       struct media_pipeline mp;
       const struct plat_csi_media_pipeline_ops *ops;
};

static inline struct plat_csi_video_entity *
vdev_to_plat_csi_video_entity(struct video_device *vdev)
{
       return container_of(vdev, struct plat_csi_video_entity, vdev);
}

#define plat_csi_pipeline_call(ent, op, args...)\
       (!(ent) ? -ENOENT : (((ent)->pipe->ops && (ent)->pipe->ops->op) ? \
       (ent)->pipe->ops->op(((ent)->pipe), ##args) : -ENOIOCTLCMD))      \


#endif                         /* SNPS_CSI_VIDEO_PLAT_INCLUDES_H_ */
