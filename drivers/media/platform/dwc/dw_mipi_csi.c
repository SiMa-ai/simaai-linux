/*
 * DWC MIPI CSI-2 Host device driver
 *
 * Copyright (C) 2016 Synopsys, Inc. All rights reserved.
 * Author: Ramiro Oliveira <ramiro.olive...@synopsys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include "dw_mipi_csi.h"
#include "dw_csi_plat.h"

static void dw_mipi_csi_write(struct mipi_csi_dev *dev, unsigned int address,
	    unsigned int data)
{
  if (address < 0x40)
    iowrite32(data, dev->csi_reg0_add + address);
  else
    iowrite32(data, dev->csi_reg1_add + address - 0x58);
}

static u32 dw_mipi_csi_read(struct mipi_csi_dev *dev, unsigned long address)
{
  if (address < 0x40)
    return ioread32(dev->csi_reg0_add + address);
  else
    return ioread32(dev->csi_reg1_add + address - 0x58);

}

static void dw_mipi_csi_glue_write(struct mipi_csi_dev *dev,
	   unsigned int offset, unsigned int data)
{
  if (offset < 0x20)
    iowrite32(data, dev->glue_reg0_add + offset);
  else
    iowrite32(data, dev->glue_reg1_add + offset - 0x34);

}

static const struct mipi_fmt *
find_dw_mipi_csi_format(struct v4l2_mbus_framefmt *mf)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dw_mipi_csi_formats); i++) {
		if (mf->code == dw_mipi_csi_formats[i].code) {
			printk(KERN_CRIT "Match found at index %d\n", i);
			return &dw_mipi_csi_formats[i];
		}
	}

	return NULL;
}

static void dw_mipi_csi_glue_reset(struct mipi_csi_dev *dev)
{
	dw_mipi_csi_glue_write(dev, 0xB8, 0x0);
	dw_mipi_csi_glue_write(dev, 0xDC, 0x0);
	dw_mipi_csi_glue_write(dev, 0x100, 0x0);
	dw_mipi_csi_glue_write(dev, 0x124, 0x0);
}

static void dw_mipi_csi_reset(struct mipi_csi_dev *dev)
{
       dw_mipi_csi_write(dev, R_CSI2_CTRL_RESETN, 0);
       dw_mipi_csi_write(dev, R_CSI2_CTRL_RESETN, 1);
}

static int dw_mipi_csi_mask_irq_power_off(struct mipi_csi_dev *dev)
{
       /* set only one lane (lane 0) as active (ON) */
       dw_mipi_csi_write(dev, R_CSI2_N_LANES, 0x0);

       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PHY_FATAL, 0);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PKT_FATAL, 0);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_FRAME_FATAL, 0);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PHY, 0);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PKT, 0);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_LINE, 0);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_IPI, 0);

       dw_mipi_csi_write(dev, R_CSI2_CTRL_RESETN, 0);

       return 0;

}

static int dw_mipi_csi_hw_stdby(struct mipi_csi_dev *dev)
{
       /* set only one lane (lane 0) as active (ON) */
       dw_mipi_csi_reset(dev);

       dw_mipi_csi_write(dev, R_CSI2_N_LANES, 0x0);

       phy_init(dev->phy);

       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PHY_FATAL, 0xFFFFFFFF);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PKT_FATAL, 0xFFFFFFFF);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_FRAME_FATAL, 0xFFFFFFFF);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PHY, 0xFFFFFFFF);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_PKT, 0xFFFFFFFF);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_LINE, 0xFFFFFFFF);
       dw_mipi_csi_write(dev, R_CSI2_MASK_INT_IPI, 0xFFFFFFFF);

       return 0;

}

static void dw_mipi_csi_set_ipi_fmt(struct mipi_csi_dev *csi_dev)
{
       struct device *dev = &csi_dev->pdev->dev;

       switch (csi_dev->fmt->code) {
	   /* RGB 666 */
       case MEDIA_BUS_FMT_RGB666_1X18:
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, CSI_2_RGB666);
	break;
       /* RGB 565 */
       case MEDIA_BUS_FMT_RGB565_2X8_BE:
       case MEDIA_BUS_FMT_RGB565_2X8_LE:
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, CSI_2_RGB565);
	break;
       /* RGB 555 */
       case MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE:
       case MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE:
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, CSI_2_RGB555);
	break;
       /* RGB 444 */
       case MEDIA_BUS_FMT_RGB444_2X8_PADHI_BE:
       case MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE:
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, CSI_2_RGB444);
	break;
       /* RGB 888 */
	break;
       case MEDIA_BUS_FMT_RGB888_2X12_LE:
       case MEDIA_BUS_FMT_RGB888_2X12_BE:
       case MEDIA_BUS_FMT_RGB888_1X24:
	       dev_info(dev, "Configuring data type to RGB888");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, 0x100 | CSI_2_RGB888);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_RGB888);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_RGB888);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_RGB888);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8, 0x2400);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC, 0x2400);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x2400);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x2400);
	break;
       /* RAW 10 */
       case MEDIA_BUS_FMT_SRGGB10_1X10:
       case MEDIA_BUS_FMT_SBGGR10_1X10:
       case MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE:
	       dev_info(dev, "Configuring data type to RAW10");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE,  0x100 | CSI_2_RAW10);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_RAW10);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_RAW10);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_RAW10);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8,  0x2B00);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC,  0x2B00);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x2B00);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x2B00);
	break;
       /* RAW 12 */
       case MEDIA_BUS_FMT_SRGGB12_1X12:
	       dev_info(dev, "Configuring data type to RAW12");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, 0x100 | CSI_2_RAW12);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_RAW12);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_RAW12);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_RAW12);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8, 0x2C00);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC, 0x2C00);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x2C00);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x2C00);
		break;
       /* RAW 14 */
       case MEDIA_BUS_FMT_SBGGR14_1X14:
	       dev_info(dev, "Configuring data type to RAW14");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE,  0x100 | CSI_2_RAW14);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_RAW14);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_RAW14);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_RAW14);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8,  0x2D00);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC,  0x2D00);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x2D00);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x2D00);
	break;
       /* RAW 16 */
       case MEDIA_BUS_FMT_SRGGB16_1X16:
	       dev_info(dev, "Configuring data type to RAW16");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, 0x100 | CSI_2_RAW16);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_RAW16);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_RAW16);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_RAW16);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8, 0x2E00);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC, 0x2E00);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x2E00);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x2E00);
	break;
       /* RAW 8 */
       case MEDIA_BUS_FMT_SRGGB8_1X8:
	       dev_info(dev, "Configuring data type to RAW8");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, 0x100 | CSI_2_RAW8);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_RAW8);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_RAW8);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_RAW8);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8, 0x2A00);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC, 0x2A00);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x2A00);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x2A00);
	break;
       /* YUV 422 8-bit */
       case MEDIA_BUS_FMT_YVYU8_2X8:
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, CSI_2_RAW8);
	break;
       /* YUV 422 8-bit */
       case MEDIA_BUS_FMT_VYUY8_1X16:
	       dev_info(dev, "Configuring data type to YUV422");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, 0x100 | CSI_2_YUV422_8);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_YUV422_8);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_YUV422_8);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_YUV422_8);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8, 0x1E00);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC, 0x1E00);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x1E00);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x1E00);
	break;
       /* YUV 420 8-bit LEGACY */
       case MEDIA_BUS_FMT_Y8_1X8:
	       dev_info(dev, "Configuring data type to YUV420Legacy");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, 0x100 | CSI_2_YUV420_8_LEG);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_DATA_TYPE, 0x100 | CSI_2_YUV420_8_LEG);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_DATA_TYPE, 0x100 | CSI_2_YUV420_8_LEG);
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_DATA_TYPE, 0x100 | CSI_2_YUV420_8_LEG);
	       dw_mipi_csi_glue_write(csi_dev, 0xC8, 0x1A00);
	       dw_mipi_csi_glue_write(csi_dev, 0xEC, 0x1A00);
	       dw_mipi_csi_glue_write(csi_dev, 0x110, 0x1A00);
	       dw_mipi_csi_glue_write(csi_dev, 0x134, 0x1A00);
	break;
       /* YUV 420 10-bit */
       case MEDIA_BUS_FMT_Y10_1X10:
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, CSI_2_RAW8);
	break;
       default:
	       dev_info(dev, "Configuring default data type to RAW8");
	       dw_mipi_csi_write(csi_dev, R_CSI2_IPI_DATA_TYPE, CSI_2_RAW8);
	break;
       }
}

static void __dw_mipi_csi_fill_timings(struct mipi_csi_dev *dev,
			  const struct v4l2_bt_timings *bt)
{
       if (bt == NULL)
	return;

       dev->hw.hsa = bt->hsync;
       dev->hw.hbp = bt->hbackporch;
       dev->hw.hsd = bt->hsync;
       dev->hw.htotal = bt->height + bt->vfrontporch +
	   bt->vsync + bt->vbackporch;
       dev->hw.vsa = bt->vsync;
       dev->hw.vbp = bt->vbackporch;
       dev->hw.vfp = bt->vfrontporch;
       dev->hw.vactive = bt->height;
}

static void __dw_mipi_csi_start(struct mipi_csi_dev *csi_dev)
{
	struct device *dev = &csi_dev->pdev->dev;

	dw_mipi_csi_write(csi_dev, R_CSI2_N_LANES, 0x3);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_HLINE_TIME, 0x2134);

	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_MODE, 0x1010000);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_VCID, 0x0);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_MEM_FLUSH, 0x100);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_HSA_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_HBP_TIME, 0xc8);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_HSD_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_ADV_FEATURE, 0x270000);

	dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_MODE, 0x1010000);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_VCID, 0x1);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_MEM_FLUSH, 0x100);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_HSA_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_HBP_TIME, 0xc8);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_HSD_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI2_ADV_FEATURE, 0x270000);

	dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_MODE, 0x1010000);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_VCID, 0x2);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_MEM_FLUSH, 0x100);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_HSA_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_HBP_TIME, 0xc8);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_HSD_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI3_ADV_FEATURE, 0x270000);

	dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_MODE, 0x1010000);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_VCID, 0x3);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_MEM_FLUSH, 0x100);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_HSA_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_HBP_TIME, 0xc8);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_HSD_TIME, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI4_ADV_FEATURE, 0x270000);

	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_VSA_LINES, 0x2);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_VBP_LINES, 0x2);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_VFP_LINES, 0xF);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_VACTIVE_LINES, 0x10);
	dw_mipi_csi_write(csi_dev, R_CSI2_VIRTUAL_CHANNEL_EXT, 0x0);

	dw_mipi_csi_glue_write(csi_dev, 0x10, 0x80000008);
	dw_mipi_csi_glue_write(csi_dev, 0xB8, 0x8801);
	dw_mipi_csi_glue_write(csi_dev, 0xC0, 0xF00);
	dw_mipi_csi_glue_write(csi_dev, 0xDC, 0x8801);
	dw_mipi_csi_glue_write(csi_dev, 0xE4, 0xF00);
	dw_mipi_csi_glue_write(csi_dev, 0x100, 0x8801);
	dw_mipi_csi_glue_write(csi_dev, 0x108, 0xF00);
	dw_mipi_csi_glue_write(csi_dev, 0x124, 0x8801);
	dw_mipi_csi_glue_write(csi_dev, 0x12C, 0xF00);

	dw_mipi_csi_write(csi_dev, R_CSI2_CTRL_RESETN, 0);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_SOFT_RESET, 0);
	dw_mipi_csi_write(csi_dev, R_CSI2_CTRL_RESETN, 1);
	dw_mipi_csi_write(csi_dev, R_CSI2_IPI_SOFT_RESET, 0x1111);
	dev_info(dev, "CSI : Reset Done!!!\n");
}

static int dw_mipi_csi_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_mbus_code_enum *code)
{
       if (code->index >= ARRAY_SIZE(dw_mipi_csi_formats))
	return -EINVAL;

       code->code = dw_mipi_csi_formats[code->index].code;
       return 0;
}

static const struct mipi_fmt *
dw_mipi_csi_try_format(struct v4l2_mbus_framefmt *mf)
{
	struct mipi_fmt const *fmt;

	fmt = find_dw_mipi_csi_format(mf);
	if (fmt == NULL) {
		printk(KERN_CRIT "No match found data type defaulting to RAW8");
		fmt = &dw_mipi_csi_formats[0];
	}
	mf->code = fmt->code;
	return fmt;
}

static struct v4l2_mbus_framefmt *
__dw_mipi_csi_get_format(struct mipi_csi_dev *dev,
			struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			enum v4l2_subdev_format_whence which)
{
       if (which == V4L2_SUBDEV_FORMAT_TRY)
	return state ? v4l2_subdev_get_try_format(sd, state,
						       0) : NULL;

       return &dev->format;
}

static int
dw_mipi_csi_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		   struct v4l2_subdev_format *fmt)
{
       struct mipi_csi_dev *dev = v4l2_get_subdevdata(sd);
       struct mipi_fmt const *dev_fmt;
       struct v4l2_mbus_framefmt *mf;
       unsigned int glue_frame_dim;

       mf = __dw_mipi_csi_get_format(dev, sd, state, fmt->which);

       dev_fmt = dw_mipi_csi_try_format(&fmt->format);
       if (dev_fmt) {
	       *mf = fmt->format;
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		       dev->fmt = dev_fmt;
	       dw_mipi_csi_set_ipi_fmt(dev);
	       dev_dbg(&dev->pdev->dev, "Configuring width: %d, height: %d\n",
			       fmt->format.width, fmt->format.height);
               glue_frame_dim = (fmt->format.width & 0x1FFF) |
		       ((fmt->format.height & 0xFFF) << 16);
	       dw_mipi_csi_glue_write(dev, 0xC4, glue_frame_dim);
	       dw_mipi_csi_glue_write(dev, 0xE8, glue_frame_dim);
	       dw_mipi_csi_glue_write(dev, 0x10C, glue_frame_dim);
	       dw_mipi_csi_glue_write(dev, 0x130, glue_frame_dim);

       }
       dev_dbg(&dev->pdev->dev, "Frame size configured!!!\n");

       return 0;

}

static int
dw_mipi_csi_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		   struct v4l2_subdev_format *fmt)
{
       struct mipi_csi_dev *dev = v4l2_get_subdevdata(sd);
       struct v4l2_mbus_framefmt *mf;

       mf = __dw_mipi_csi_get_format(dev, sd, state, fmt->which);
       if (!mf)
	return -EINVAL;

       mutex_lock(&dev->lock);
       fmt->format = *mf;
       dev_dbg(&dev->pdev->dev, "Get width: %#x, height: %#x\n", mf->width, mf->height);
       mutex_unlock(&dev->lock);
       return 0;
}

static int
dw_mipi_csi_s_power(struct v4l2_subdev *sd, int on)
{
	struct mipi_csi_dev *dev = v4l2_get_subdevdata(sd);

	if (on) {
		dev_dbg(&dev->pdev->dev, "Power on\n");
		dw_mipi_csi_hw_stdby(dev);
		__dw_mipi_csi_start(dev);
		phy_power_on(dev->phy);
	} else {
		dev_dbg(&dev->pdev->dev, "Power off\n");
		phy_power_off(dev->phy);
		dw_mipi_csi_glue_reset(dev);
		dw_mipi_csi_mask_irq_power_off(dev);
	}

	return 0;
}

static int
dw_mipi_csi_init_cfg(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
       struct v4l2_mbus_framefmt *format =
	   v4l2_subdev_get_try_format(sd, state, 0);

       format->colorspace = V4L2_COLORSPACE_SRGB;
       format->code = dw_mipi_csi_formats[0].code;
       format->width = MIN_WIDTH;
       format->height = MIN_HEIGHT;
       format->field = V4L2_FIELD_NONE;

       return 0;
}

static struct v4l2_subdev_core_ops dw_mipi_csi_core_ops = {
       .s_power = dw_mipi_csi_s_power,
};

static struct v4l2_subdev_pad_ops dw_mipi_csi_pad_ops = {
       .init_cfg = dw_mipi_csi_init_cfg,
       .enum_mbus_code = dw_mipi_csi_enum_mbus_code,
       .get_fmt = dw_mipi_csi_get_fmt,
       .set_fmt = dw_mipi_csi_set_fmt,
};

static struct v4l2_subdev_ops dw_mipi_csi_subdev_ops = {
       .core = &dw_mipi_csi_core_ops,
       .pad = &dw_mipi_csi_pad_ops,
};

static irqreturn_t
dw_mipi_csi_irq1(int irq, void *dev_id)
{
       struct mipi_csi_dev *csi_dev = dev_id;
       u32 global_int_status, i_sts;
       unsigned long flags;
       struct device *dev = &csi_dev->pdev->dev;

       global_int_status = dw_mipi_csi_read(csi_dev, R_CSI2_INTERRUPT);
       spin_lock_irqsave(&csi_dev->slock, flags);

       if (global_int_status & CSI2_INT_PHY_FATAL) {
	       i_sts = dw_mipi_csi_read(csi_dev, R_CSI2_INT_PHY_FATAL);
	       dev_dbg_ratelimited(dev, "CSI INT PHY FATAL: %08X\n", i_sts);
       }

       if (global_int_status & CSI2_INT_PKT_FATAL) {
	       i_sts = dw_mipi_csi_read(csi_dev, R_CSI2_INT_PKT_FATAL);
	       dev_dbg_ratelimited(dev, "CSI INT PKT FATAL: %08X\n", i_sts);
       }

       if (global_int_status & CSI2_INT_FRAME_FATAL) {
	       i_sts = dw_mipi_csi_read(csi_dev, R_CSI2_INT_FRAME_FATAL);
	       dev_dbg_ratelimited(dev, "CSI INT FRAME FATAL: %08X\n", i_sts);
       }

       if (global_int_status & CSI2_INT_PHY) {
	       i_sts = dw_mipi_csi_read(csi_dev, R_CSI2_INT_PHY);
	       dev_dbg_ratelimited(dev, "CSI INT PHY: %08X\n", i_sts);
       }

       if (global_int_status & CSI2_INT_PKT) {
	       i_sts = dw_mipi_csi_read(csi_dev, R_CSI2_INT_PKT);
	       dev_dbg_ratelimited(dev, "CSI INT PKT: %08X\n", i_sts);
       }

       if (global_int_status & CSI2_INT_LINE) {
	       i_sts = dw_mipi_csi_read(csi_dev, R_CSI2_INT_LINE);
	       dev_dbg_ratelimited(dev, "CSI INT LINE: %08X\n", i_sts);
       }

       if (global_int_status & CSI2_INT_IPI) {
	       i_sts = dw_mipi_csi_read(csi_dev, R_CSI2_INT_IPI);
	       dev_dbg_ratelimited(dev, "CSI INT IPI: %08X\n", i_sts);
       }
       spin_unlock_irqrestore(&csi_dev->slock, flags);
       return IRQ_HANDLED;
}

static int
dw_mipi_csi_parse_dt(struct platform_device *pdev, struct mipi_csi_dev *dev)
{
       struct device_node *node = pdev->dev.of_node;
       int ret = 0;

       ret = of_property_read_u32(node, "output-type", &dev->hw.output_type);
       if (ret) {
	       dev_err(&pdev->dev, "Couldn't read output-type\n");
	return ret;
       }

       ret = of_property_read_u32(node, "ipi-mode", &dev->hw.ipi_mode);
       if (ret) {
	       dev_err(&pdev->dev, "Couldn't read ipi-mode\n");
	return ret;
       }

       ret = of_property_read_u32(node, "ipi-auto-flush",
				&dev->hw.ipi_auto_flush);
       if (ret) {
	       dev_err(&pdev->dev, "Couldn't read ipi-auto-flush\n");
	return ret;
       }

       ret = of_property_read_u32(node, "ipi-color-mode",
				&dev->hw.ipi_color_mode);
       if (ret) {
	       dev_err(&pdev->dev, "Couldn't read ipi-color-mode\n");
	return ret;
       }

       ret = of_property_read_u32(node, "virtual-channel",
			       &dev->hw.virtual_ch);
       if (ret) {
	       dev_err(&pdev->dev, "Couldn't read virtual-channel\n");
	return ret;
       }

       node = of_graph_get_next_endpoint(node, NULL);
       if (!node) {
	       dev_err(&pdev->dev, "No port node at %s\n",
			       pdev->dev.of_node->full_name);
	return -EINVAL;
       }

       return ret;
}

static const struct of_device_id dw_mipi_csi_of_match[];

/**
 * @short Initialization routine - Entry point of the driver
 * @param[in] pdev pointer to the platform device structure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
static int mipi_csi_probe(struct platform_device *pdev)
{
       const struct of_device_id *of_id;
       struct device *dev = &pdev->dev;
       struct resource *res = NULL;
       struct mipi_csi_dev *mipi_csi;
       int ret = -ENOMEM;
       u32 isp_connected = 0;
       unsigned int i = 0;

       mipi_csi = devm_kzalloc(dev, sizeof(*mipi_csi), GFP_KERNEL);
       if (!dev)
	return -ENOMEM;

       mutex_init(&mipi_csi->lock);
       spin_lock_init(&mipi_csi->slock);
       mipi_csi->pdev = pdev;

       of_id = of_match_node(dw_mipi_csi_of_match, dev->of_node);
       if (WARN_ON(of_id == NULL))
	return -EINVAL;

       ret = dw_mipi_csi_parse_dt(pdev, mipi_csi);
       if (ret < 0)
	return ret;

       mipi_csi->phy = devm_of_phy_get(dev, dev->of_node, NULL);
       if (IS_ERR(mipi_csi->phy)) {
	       dev_err(dev, "No DPHY available\n");
	return -EPROBE_DEFER;
       }

       res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csi-reg0");
       mipi_csi->csi_reg0_add = devm_ioremap_resource(dev, res);
       if (IS_ERR(mipi_csi->csi_reg0_add)) {
	 dev_err(dev, "csi reg0 not set");
	return PTR_ERR(mipi_csi->csi_reg0_add);
       }

       res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csi-reg1");
       mipi_csi->csi_reg1_add = devm_ioremap_resource(dev, res);
       if (IS_ERR(mipi_csi->csi_reg1_add)) {
	 dev_err(dev, "csi reg1 not set");
	return PTR_ERR(mipi_csi->csi_reg1_add);
       }

       res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "glue-reg0");
       mipi_csi->glue_reg0_add = devm_ioremap_resource(dev, res);
       if (IS_ERR(mipi_csi->glue_reg0_add)) {
	 dev_err(dev, "glue reg0 not set");
	return PTR_ERR(mipi_csi->glue_reg0_add);
       }

       res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "glue-reg1");
       mipi_csi->glue_reg1_add = devm_ioremap_resource(dev, res);
       if (IS_ERR(mipi_csi->glue_reg1_add)) {
	 dev_err(dev, "glue reg1 not set");
	return PTR_ERR(mipi_csi->glue_reg1_add);
       }

       mipi_csi->ctrl_irq_number = platform_get_irq(pdev, 0);
       if (mipi_csi->ctrl_irq_number <= 0) {
	       dev_err(dev, "IRQ number not set.\n");
	return mipi_csi->ctrl_irq_number;
       }

       ret = devm_request_irq(dev, mipi_csi->ctrl_irq_number,
			      dw_mipi_csi_irq1, IRQF_SHARED,
			      dev_name(dev), mipi_csi);
       if (ret) {
	       dev_err(dev, "IRQ failed\n");
	goto end;
       }

       mipi_csi->rst = devm_reset_control_get_optional_shared(dev, NULL);
       if (IS_ERR(mipi_csi->rst))
	       mipi_csi->rst = NULL;

	ret = of_property_read_u32(pdev->dev.of_node, "isp-connected", &isp_connected);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't read isp-connnected\n");
		return ret;
	}

	for(i = 0; i < mipi_csi->hw.virtual_ch ; i++) {
		v4l2_subdev_init(&mipi_csi->sd[i], &dw_mipi_csi_subdev_ops);
		mipi_csi->sd[i].owner = THIS_MODULE;
		snprintf(mipi_csi->sd[i].name, sizeof(mipi_csi->sd[i].name), "%s.%d",
		CSI_DEVICE_NAME, i);
		mipi_csi->sd[i].dev = dev;

		mipi_csi->sd[i].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

		mipi_csi->sd[i].entity.function = MEDIA_ENT_F_IO_V4L;
		mipi_csi->pads[i][CSI_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
		mipi_csi->pads[i][CSI_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

		ret = media_entity_pads_init(&mipi_csi->sd[i].entity,
				    CSI_PADS_NUM, mipi_csi->pads[i]);

		if (ret < 0) {
			dev_err(dev, "Media Entity init for VC %d failed\n", i);
			goto entity_cleanup;

		}

		/* This allows to retrieve the platform device id by the host driver */
		v4l2_set_subdevdata(&mipi_csi->sd[i], mipi_csi);
        if (isp_connected) {
            ret = v4l2_async_register_subdev(&mipi_csi->sd[i]);
            if (ret < 0) {
                dev_err(dev, "ERROR : registering mipi subdev\n");
                goto entity_cleanup;
            }
            dev_info(dev, "VC : %d async subdev %#llx and media registration success !!!\n",
                i, &mipi_csi->sd[i]);
        } else {
            dev_info(
                dev,
                "VC %d subdev %#llx and media entity registration success\n",
                i, &mipi_csi->sd[i]);
    	}
	}

	mipi_csi->fmt = &dw_mipi_csi_formats[0];
	mipi_csi->format.code = dw_mipi_csi_formats[0].code;
	mipi_csi->format.width = MIN_WIDTH;
	mipi_csi->format.height = MIN_HEIGHT;

	/* .. and a pointer to the subdev. */
	platform_set_drvdata(pdev, mipi_csi);

	if (mipi_csi->rst)
		reset_control_deassert(mipi_csi->rst);

	dw_mipi_csi_mask_irq_power_off(mipi_csi);

	dev_info(dev, "DW MIPI CSI-2 Host registered successfully\n");

	return 0;

entity_cleanup:
	for (i = 0; i < mipi_csi->hw.virtual_ch; i++)
		media_entity_cleanup(&mipi_csi->sd[i].entity);

end:
       return ret;
}

/**
 * @short Exit routine - Exit point of the driver
 * @param[in] pdev pointer to the platform device structure
 * @return 0 on success and a negative number on failure
 * Refer to Linux errors.
 */
static int mipi_csi_remove(struct platform_device *pdev)
{
	struct mipi_csi_dev *mipi_csi = platform_get_drvdata(pdev);
	unsigned int i = 0;

	dev_dbg(&pdev->dev, "Removing MIPI CSI-2 module\n");

	if (mipi_csi->rst)
		reset_control_assert(mipi_csi->rst);

	for (i = 0; i < mipi_csi->hw.virtual_ch; i++)
		media_entity_cleanup(&mipi_csi->sd[i].entity);

	return 0;
}

/**
 * @short of_device_id structure
 */
static const struct of_device_id dw_mipi_csi_of_match[] = {
       {.compatible = "snps,dw-mipi-csi"},
       { /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, dw_mipi_csi_of_match);

/**
 * @short Platform driver structure
 */
static struct platform_driver __refdata dw_mipi_csi_pdrv = {
       .remove = mipi_csi_remove,
       .probe = mipi_csi_probe,
       .driver = {
		  .name = CSI_DEVICE_NAME,
		  .owner = THIS_MODULE,
		  .of_match_table = of_match_ptr(dw_mipi_csi_of_match),
		  },
};

module_platform_driver(dw_mipi_csi_pdrv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramiro Oliveira <roliv...@synopsys.com>");
MODULE_DESCRIPTION("Synopsys DW MIPI CSI-2 Host driver");
