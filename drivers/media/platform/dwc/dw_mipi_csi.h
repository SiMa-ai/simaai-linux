/*
 * Copyright (C) 2016 Synopsys, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

 #ifndef DW_MIPI_CSI_H_
 #define DW_MIPI_CSI_H_

 #include <linux/delay.h>
 #include <linux/errno.h>
 #include <linux/io.h>
 #include <linux/interrupt.h>
 #include <linux/kernel.h>
 #include <linux/module.h>
 #include <linux/of_irq.h>
 #include <linux/of_graph.h>
 #include <linux/phy/phy.h>
 #include <linux/platform_device.h>
 #include <linux/ratelimit.h>
 #include <linux/reset.h>
 #include <linux/string.h>
 #include <linux/types.h>
 #include <linux/videodev2.h>
 #include <linux/wait.h>
 #include <media/dwc/csi_host_platform.h>
 #include <media/v4l2-device.h>
 #include <media/v4l2-dv-timings.h>
 #include <media/v4l2-fwnode.h>

 #define CSI_DEVICE_NAME        "dw-mipi-csi"
 #define MAX_VC_PER_CSI			4

/** @short DWC MIPI CSI-2 register addresses*/
enum register_addresses {
       R_CSI2_VERSION = 0x00,
       R_CSI2_N_LANES = 0x04,
       R_CSI2_CTRL_RESETN = 0x08,
       R_CSI2_INTERRUPT = 0x0C,
       R_CSI2_DATA_IDS_1 = 0x10,
       R_CSI2_DATA_IDS_2 = 0x14,
       R_CSI2_PHY_SHUTDOWNZ = 0x40,
       R_CSI2_DPHY_RESETZ = 0x44,
       R_CSI2_IPI_MODE = 0x80,
       R_CSI2_IPI_VCID = 0x84,
       R_CSI2_IPI_DATA_TYPE = 0x88,
       R_CSI2_IPI_MEM_FLUSH = 0x8C,
       R_CSI2_IPI_HSA_TIME = 0x90,
       R_CSI2_IPI_HBP_TIME = 0x94,
       R_CSI2_IPI_HSD_TIME = 0x98,
       R_CSI2_IPI_HLINE_TIME = 0x9C,
       R_CSI2_IPI_SOFT_RESET = 0xA0,
       R_CSI2_IPI_ADV_FEATURE = 0xAC,
       R_CSI2_IPI_VSA_LINES = 0xB0,
       R_CSI2_IPI_VBP_LINES = 0xB4,
       R_CSI2_IPI_VFP_LINES = 0xB8,
       R_CSI2_IPI_VACTIVE_LINES = 0xBC,
       R_CSI2_VIRTUAL_CHANNEL_EXT = 0xC8,
       R_CSI2_INT_PHY_FATAL = 0xe0,
       R_CSI2_MASK_INT_PHY_FATAL = 0xe4,
       R_CSI2_FORCE_INT_PHY_FATAL = 0xe8,
       R_CSI2_INT_PKT_FATAL = 0xf0,
       R_CSI2_MASK_INT_PKT_FATAL = 0xf4,
       R_CSI2_FORCE_INT_PKT_FATAL = 0xf8,
       R_CSI2_INT_FRAME_FATAL = 0x100,
       R_CSI2_MASK_INT_FRAME_FATAL = 0x104,
       R_CSI2_FORCE_INT_FRAME_FATAL = 0x108,
       R_CSI2_INT_PHY = 0x110,
       R_CSI2_MASK_INT_PHY = 0x114,
       R_CSI2_FORCE_INT_PHY = 0x118,
       R_CSI2_INT_PKT = 0x120,
       R_CSI2_MASK_INT_PKT = 0x124,
       R_CSI2_FORCE_INT_PKT = 0x128,
       R_CSI2_INT_LINE = 0x130,
       R_CSI2_MASK_INT_LINE = 0x134,
       R_CSI2_FORCE_INT_LINE = 0x138,
       R_CSI2_INT_IPI = 0x140,
       R_CSI2_MASK_INT_IPI = 0x144,
       R_CSI2_FORCE_INT_IPI = 0x148,
       R_CSI2_IPI2_MODE = 0x200,
       R_CSI2_IPI2_VCID = 0x204,
       R_CSI2_IPI2_DATA_TYPE = 0x208,
       R_CSI2_IPI2_MEM_FLUSH = 0x20c,
       R_CSI2_IPI2_HSA_TIME = 0x210,
       R_CSI2_IPI2_HBP_TIME = 0x214,
       R_CSI2_IPI2_HSD_TIME = 0x218,
       R_CSI2_IPI2_ADV_FEATURE = 0x21c,
       R_CSI2_IPI3_MODE = 0x220,
       R_CSI2_IPI3_VCID = 0x224,
       R_CSI2_IPI3_DATA_TYPE = 0x228,
       R_CSI2_IPI3_MEM_FLUSH = 0x22c,
       R_CSI2_IPI3_HSA_TIME = 0x230,
       R_CSI2_IPI3_HBP_TIME = 0x234,
       R_CSI2_IPI3_HSD_TIME = 0x238,
       R_CSI2_IPI3_ADV_FEATURE = 0x23c,
       R_CSI2_IPI4_MODE = 0x240,
       R_CSI2_IPI4_VCID = 0x244,
       R_CSI2_IPI4_DATA_TYPE = 0x248,
       R_CSI2_IPI4_MEM_FLUSH = 0x24c,
       R_CSI2_IPI4_HSA_TIME = 0x250,
       R_CSI2_IPI4_HBP_TIME = 0x254,
       R_CSI2_IPI4_HSD_TIME = 0x258,
       R_CSI2_IPI4_ADV_FEATURE = 0x25c
};

/** @short IPI Data Types */
enum data_type {
       CSI_2_YUV420_8 = 0x18,
       CSI_2_YUV420_10 = 0x19,
       CSI_2_YUV420_8_LEG = 0x1A,
       CSI_2_YUV420_8_SHIFT = 0x1C,
       CSI_2_YUV420_10_SHIFT = 0x1D,
       CSI_2_YUV422_8 = 0x1E,
       CSI_2_YUV422_10 = 0x1F,
       CSI_2_RGB444 = 0x20,
       CSI_2_RGB555 = 0x21,
       CSI_2_RGB565 = 0x22,
       CSI_2_RGB666 = 0x23,
       CSI_2_RGB888 = 0x24,
       CSI_2_RAW6 = 0x28,
       CSI_2_RAW7 = 0x29,
       CSI_2_RAW8 = 0x2A,
       CSI_2_RAW10 = 0x2B,
       CSI_2_RAW12 = 0x2C,
       CSI_2_RAW14 = 0x2D,
       CSI_2_RAW16 = 0x2E,
};

/** @short Interrupt Masks */
enum interrupt_type {
       CSI2_INT_PHY_FATAL = 1 << 0,
       CSI2_INT_PKT_FATAL = 1 << 1,
       CSI2_INT_FRAME_FATAL = 1 << 2,
       CSI2_INT_PHY = 1 << 16,
       CSI2_INT_PKT = 1 << 17,
       CSI2_INT_LINE = 1 << 18,
       CSI2_INT_IPI = 1 << 19,

};

/** @short DWC MIPI CSI-2 output types*/
enum output_type {
       IPI_OUT = 0,
       IDI_OUT = 1,
       BOTH_OUT = 2
};

/** @short IPI output types*/
enum ipi_output_type {
       CAMERA_TIMING = 0,
       AUTO_TIMING = 1
};

/**
 * @short Format template
 */
struct mipi_fmt {
       u32 code;
       u8 depth;
};

struct csi_hw {

       uint32_t num_lanes;
       uint32_t output_type;

       /*IPI Info */
       uint32_t ipi_mode;
       uint32_t ipi_color_mode;
       uint32_t ipi_auto_flush;
       uint32_t virtual_ch;

       uint32_t hsa;
       uint32_t hbp;
       uint32_t hsd;
       uint32_t htotal;

       uint32_t vsa;
       uint32_t vbp;
       uint32_t vfp;
       uint32_t vactive;
};

/**
 * @short Structure to embed device driver information
 */
struct mipi_csi_dev {
       struct v4l2_subdev sd[MAX_VC_PER_CSI];
       struct video_device vdev;

       struct mutex lock;
       spinlock_t slock;
       struct media_pad pads[MAX_VC_PER_CSI][CSI_PADS_NUM];
       struct platform_device *pdev;
       u8 index;

       /** Store current format */
       const struct mipi_fmt *fmt;
       struct v4l2_mbus_framefmt format;

       /** Device Tree Information */
       void __iomem *csi_reg0_add;
       void __iomem *csi_reg1_add;
       void __iomem *glue_reg0_add;
       void __iomem *glue_reg1_add;
       uint32_t ctrl_irq_number;

       struct csi_hw hw;
       struct phy *phy;
       struct reset_control *rst;
       struct task_struct *dw_phy_power;
};

 #endif /* DW_MIPI_CSI */
