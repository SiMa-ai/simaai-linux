// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Sony IMX415 CMOS Image Sensor.
 *
 * Copyright (C) 2023 WolfVision GmbH.
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX415_NATIVE_WIDTH	  3864
#define IMX415_NATIVE_HEIGHT	  2192
#define IMX415_EXPOSURE_OFFSET	  8

#define IMX415_PIXEL_RATE_74_25MHZ	891000000
#define IMX415_PIXEL_RATE_72MHZ		864000000

#define IMX415_NUM_CLK_PARAM_REGS 11

#define IMX415_MODE		  CCI_REG8(0x3000)
#define IMX415_MODE_OPERATING	  (0)
#define IMX415_MODE_STANDBY	  BIT(0)
#define IMX415_REGHOLD		  CCI_REG8(0x3001)
#define IMX415_REGHOLD_INVALID	  (0)
#define IMX415_REGHOLD_VALID	  BIT(0)
#define IMX415_XMSTA		  CCI_REG8(0x3002)
#define IMX415_XMSTA_START	  (0)
#define IMX415_XMSTA_STOP	  BIT(0)
#define IMX415_BCWAIT_TIME	  CCI_REG16_LE(0x3008)
#define IMX415_CPWAIT_TIME	  CCI_REG16_LE(0x300a)
#define IMX415_WINMODE		  CCI_REG8(0x301c)
#define IMX415_ADDMODE		  CCI_REG8(0x3022)
#define IMX415_REVERSE		  CCI_REG8(0x3030)
#define IMX415_HREVERSE_SHIFT	  (0)
#define IMX415_VREVERSE_SHIFT	  BIT(0)
#define IMX415_ADBIT		  CCI_REG8(0x3031)
#define IMX415_MDBIT		  CCI_REG8(0x3032)
#define IMX415_ADBIT1		  CCI_REG8(0x3701)
#define IMX415_SYS_MODE		  CCI_REG8(0x3033)
#define IMX415_OUTSEL		  CCI_REG8(0x30c0)
#define IMX415_DRV		  CCI_REG8(0x30c1)
#define IMX415_VMAX		  CCI_REG24_LE(0x3024)
#define IMX415_VMAX_MAX		  0xfffff
#define IMX415_HMAX		  CCI_REG16_LE(0x3028)
#define IMX415_HMAX_MAX		  0xffff
#define IMX415_HMAX_MULTIPLIER	  12
#define IMX415_SHR0		  CCI_REG24_LE(0x3050)
#define IMX415_GAIN_PCG_0	  CCI_REG16_LE(0x3090)
#define IMX415_AGAIN_MIN	  0
#define IMX415_AGAIN_MAX	  100
#define IMX415_AGAIN_STEP	  1
#define IMX415_BLKLEVEL		  CCI_REG16_LE(0x30e2)
#define IMX415_BLKLEVEL_DEFAULT	  50
#define IMX415_TPG_EN_DUOUT	  CCI_REG8(0x30e4)
#define IMX415_TPG_PATSEL_DUOUT	  CCI_REG8(0x30e6)
#define IMX415_TPG_COLORWIDTH	  CCI_REG8(0x30e8)
#define IMX415_TESTCLKEN_MIPI	  CCI_REG8(0x3110)
#define IMX415_INCKSEL1		  CCI_REG8(0x3115)
#define IMX415_INCKSEL2		  CCI_REG8(0x3116)
#define IMX415_INCKSEL3		  CCI_REG16_LE(0x3118)
#define IMX415_INCKSEL4		  CCI_REG16_LE(0x311a)
#define IMX415_INCKSEL5		  CCI_REG8(0x311e)
#define IMX415_DIG_CLP_MODE	  CCI_REG8(0x32c8)
#define IMX415_WRJ_OPEN		  CCI_REG8(0x3390)
#define IMX415_SENSOR_INFO	  CCI_REG16_LE(0x3f12)
#define IMX415_SENSOR_INFO_MASK	  0xfff
#define IMX415_CHIP_ID		  0x514
#define IMX415_LANEMODE		  CCI_REG16_LE(0x4001)
#define IMX415_LANEMODE_2	  1
#define IMX415_LANEMODE_4	  3
#define IMX415_TXCLKESC_FREQ	  CCI_REG16_LE(0x4004)
#define IMX415_INCKSEL6		  CCI_REG8(0x400c)
#define IMX415_TCLKPOST		  CCI_REG16_LE(0x4018)
#define IMX415_TCLKPREPARE	  CCI_REG16_LE(0x401a)
#define IMX415_TCLKTRAIL	  CCI_REG16_LE(0x401c)
#define IMX415_TCLKZERO		  CCI_REG16_LE(0x401e)
#define IMX415_THSPREPARE	  CCI_REG16_LE(0x4020)
#define IMX415_THSZERO		  CCI_REG16_LE(0x4022)
#define IMX415_THSTRAIL		  CCI_REG16_LE(0x4024)
#define IMX415_THSEXIT		  CCI_REG16_LE(0x4026)
#define IMX415_TLPX		  CCI_REG16_LE(0x4028)
#define IMX415_INCKSEL7		  CCI_REG8(0x4074)
#define IMX415_PIX_HST  	  CCI_REG16_LE(0x3040)
#define IMX415_PIX_HWIDTH  	  CCI_REG16_LE(0x3042)
#define IMX415_PIX_VST  	  CCI_REG16_LE(0x3044)
#define IMX415_PIX_VWIDTH  	  CCI_REG16_LE(0x3046)
#define IMX415_EMB_DATA  	  CCI_REG8(0x3338)
#define IMX415_OB_DATA_1  	  CCI_REG8(0x3038)
#define IMX415_OB_DATA_2  	  CCI_REG8(0x3337)
#define IMX415_ACTIVE_LOW  	  CCI_REG8(0x303A)
#define IMX415_ACTIVE_HIGH  	  CCI_REG8(0x303B)

static const char *const imx415_supply_names[] = {
	"dvdd",
	"ovdd",
	"avdd",
};

/*
 * The IMX415 data sheet uses lane rates but v4l2 uses link frequency to
 * describe MIPI CSI-2 speed. This driver uses lane rates wherever possible
 * and converts them to link frequencies by a factor of two when needed.
 */
static const s64 link_freq_menu_items[] = {
	594000000 / 2,	720000000 / 2,	891000000 / 2,
	1440000000 / 2, 1485000000 / 2,
};

struct imx415_clk_params {
	u64 lane_rate;
	u64 inck;
	struct cci_reg_sequence regs[IMX415_NUM_CLK_PARAM_REGS];
};

/* INCK Settings - includes all lane rate and INCK dependent registers */
static const struct imx415_clk_params imx415_clk_params[] = {
	{
		.lane_rate = 594000000UL,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x7 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x084 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 594000000UL,
		.inck = 37125000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x07F },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x05B },
		.regs[2] = { IMX415_SYS_MODE, 0x7 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x24 },
		.regs[5] = { IMX415_INCKSEL3, 0x080 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x24 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0984 },
	},
	{
		.lane_rate = 594000000UL,
		.inck = 74250000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0FF },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B6 },
		.regs[2] = { IMX415_SYS_MODE, 0x7 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x080 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1290 },
	},
	{
		.lane_rate = 720000000UL,
		.inck = 24000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x054 },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x03B },
		.regs[2] = { IMX415_SYS_MODE, 0x9 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0B4 },
		.regs[6] = { IMX415_INCKSEL4, 0x0FC },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0600 },
	},
	{
		.lane_rate = 720000000UL,
		.inck = 72000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0F8 },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B0 },
		.regs[2] = { IMX415_SYS_MODE, 0x9 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x0A0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1200 },
	},
	{
		.lane_rate = 891000000UL,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x5 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0C6 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 891000000UL,
		.inck = 37125000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x07F },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x05B },
		.regs[2] = { IMX415_SYS_MODE, 0x5 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x24 },
		.regs[5] = { IMX415_INCKSEL3, 0x0C0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x24 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0948 },
	},
	{
		.lane_rate = 891000000UL,
		.inck = 74250000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0FF },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B6 },
		.regs[2] = { IMX415_SYS_MODE, 0x5 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x0C0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1290 },
	},
	{
		.lane_rate = 1440000000UL,
		.inck = 24000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x054 },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x03B },
		.regs[2] = { IMX415_SYS_MODE, 0x8 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0B4 },
		.regs[6] = { IMX415_INCKSEL4, 0x0FC },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0600 },
	},
	{
		.lane_rate = 1440000000UL,
		.inck = 72000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0F8 },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B0 },
		.regs[2] = { IMX415_SYS_MODE, 0x8 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x0A0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1200 },
	},
	{
		.lane_rate = 1485000000UL,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x8 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0A5 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 1485000000UL,
		.inck = 37125000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x07F },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x05B },
		.regs[2] = { IMX415_SYS_MODE, 0x8 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x24 },
		.regs[5] = { IMX415_INCKSEL3, 0x0A0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x24 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0948 },
	},
	{
		.lane_rate = 1485000000UL,
		.inck = 74250000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0FF },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B6 },
		.regs[2] = { IMX415_SYS_MODE, 0x8 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x0A0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1290 },
	},
	{
		.lane_rate = 1782000000UL,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x4 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0C6 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 1782000000UL,
		.inck = 37125000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x07F },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x05B },
		.regs[2] = { IMX415_SYS_MODE, 0x4 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x24 },
		.regs[5] = { IMX415_INCKSEL3, 0x0C0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x24 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0948 },
	},
	{
		.lane_rate = 1782000000UL,
		.inck = 74250000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0FF },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B6 },
		.regs[2] = { IMX415_SYS_MODE, 0x4 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x0C0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1290 },
	},
	{
		.lane_rate = 2079000000UL,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x2 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0E7 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 2079000000UL,
		.inck = 37125000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x07F },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x05B },
		.regs[2] = { IMX415_SYS_MODE, 0x2 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x24 },
		.regs[5] = { IMX415_INCKSEL3, 0x0E0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x24 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0948 },
	},
	{
		.lane_rate = 2079000000UL,
		.inck = 74250000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0FF },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B6 },
		.regs[2] = { IMX415_SYS_MODE, 0x2 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x0E0 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1290 },
	},
	{
		.lane_rate = 2376000000UL,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x0 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x108 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 2376000000UL,
		.inck = 37125000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x07F },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x05B },
		.regs[2] = { IMX415_SYS_MODE, 0x0 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x24 },
		.regs[5] = { IMX415_INCKSEL3, 0x100 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x24 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0948 },
	},
	{
		.lane_rate = 2376000000UL,
		.inck = 74250000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x0FF },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x0B6 },
		.regs[2] = { IMX415_SYS_MODE, 0x0 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x28 },
		.regs[5] = { IMX415_INCKSEL3, 0x100 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E0 },
		.regs[7] = { IMX415_INCKSEL5, 0x28 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x1290 },
	},
};

/* 720 Mbps CSI configuration */
static const struct cci_reg_sequence imx415_linkrate_720mbps[] = {
	{ IMX415_TCLKPOST, 0x006F },
	{ IMX415_TCLKPREPARE, 0x002F },
	{ IMX415_TCLKTRAIL, 0x002F },
	{ IMX415_TCLKZERO, 0x00BF },
	{ IMX415_THSPREPARE, 0x002F },
	{ IMX415_THSZERO, 0x0057 },
	{ IMX415_THSTRAIL, 0x002F },
	{ IMX415_THSEXIT, 0x004F },
	{ IMX415_TLPX, 0x0027 },
};

/* 1440 Mbps CSI configuration */
static const struct cci_reg_sequence imx415_linkrate_1440mbps[] = {
	{ IMX415_TCLKPOST, 0x009F },
	{ IMX415_TCLKPREPARE, 0x0057 },
	{ IMX415_TCLKTRAIL, 0x0057 },
	{ IMX415_TCLKZERO, 0x0187 },
	{ IMX415_THSPREPARE, 0x005F },
	{ IMX415_THSZERO, 0x00A7 },
	{ IMX415_THSTRAIL, 0x005F },
	{ IMX415_THSEXIT, 0x0097 },
	{ IMX415_TLPX, 0x004F },
};

/* 891 Mbps CSI configuration */
static const struct cci_reg_sequence imx415_linkrate_891mbps[] = {
	{ IMX415_TCLKPOST, 0x007F },
	{ IMX415_TCLKPREPARE, 0x0037 },
	{ IMX415_TCLKTRAIL, 0x0037 },
	{ IMX415_TCLKZERO, 0x00F7 },
	{ IMX415_THSPREPARE, 0x003F },
	{ IMX415_THSZERO, 0x006F },
	{ IMX415_THSTRAIL, 0x003F },
	{ IMX415_THSEXIT, 0x005F },
	{ IMX415_TLPX, 0x002F },
};

/* 594 Mbps CSI configuration */
static const struct cci_reg_sequence imx415_linkrate_594mbps[] = {
	{ IMX415_TCLKPOST, 0x0067 },
	{ IMX415_TCLKPREPARE, 0x0027 },
	{ IMX415_TCLKTRAIL, 0x0027 },
	{ IMX415_TCLKZERO, 0x00B7 },
	{ IMX415_THSPREPARE, 0x002F },
	{ IMX415_THSZERO, 0x004F },
	{ IMX415_THSTRAIL, 0x002F },
	{ IMX415_THSEXIT, 0x0047 },
	{ IMX415_TLPX, 0x0027 },
};

struct imx415_mode_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct imx415_mode {
	u64 lane_rate;
	/*
	 * Per-rate HMAX minima from the datasheet mode tables, indexed
	 * [0]=2-lane, [1]=4-lane. The 12-bit values are discrete per-mode
	 * settings, not a scaling of the 10-bit ones; 0 means the datasheet
	 * defines no 12-bit setting for that rate/lane combination.
	 */
	u32 hmax_min[2];
	u32 hmax_min_12[2];
	struct imx415_mode_reg_list reg_list;
};

/* mode configs */
static const struct imx415_mode supported_modes[] = {
	{
		.lane_rate = 720000000,
		.hmax_min = { 2032, 1066 },
		.hmax_min_12 = { 0, 1280 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_linkrate_720mbps),
			.regs = imx415_linkrate_720mbps,
		},
	},
	{
		.lane_rate = 1440000000,
		.hmax_min = { 1066, 533 },
		.hmax_min_12 = { 0, 1066 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_linkrate_1440mbps),
			.regs = imx415_linkrate_1440mbps,
		},
	},
	{
		.lane_rate = 891000000,
		.hmax_min = { 2200, 1100 },
		.hmax_min_12 = { 2200, 1100 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_linkrate_891mbps),
			.regs = imx415_linkrate_891mbps,
		},
	},
	{
		.lane_rate = 594000000,
		.hmax_min = { 3300, 1320 },
		.hmax_min_12 = { 3300, 1650 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_linkrate_594mbps),
			.regs = imx415_linkrate_594mbps,
		},
	},
};

/*
 * Readout window (output resolution) configuration, these select the sensor
 * readout window. The H period is fixed to the base mode, so only the H/V
 * blanking and exposure control ranges change between windows.
 */
struct imx415_readout_mode {
	u32 width;
	u32 height;
	u32 crop_top;
	u32 crop_left;
	u32 vblank_min;
	/* Target fps, [0]=2-lane, [1]=4-lane; 0 keeps vblank_min. */
	u32 fps_target[2];
	struct imx415_mode_reg_list reg_list;
};

/* All-pixel readout: full 3864x2192 effective array. */
static const struct cci_reg_sequence imx415_window_all_pixel[] = {
	{ IMX415_WINMODE, 0x00 },
};

/* Window cropping: 3840x2160 (UHD) centred in the effective array. */
static const struct cci_reg_sequence imx415_window_crop_uhd[] = {
	{ IMX415_WINMODE, 0x04 },
	{ IMX415_PIX_HST, 12 },		/* (3864 - 3840) / 2, multiple of 2 */
	{ IMX415_PIX_HWIDTH, 3840 },	/* multiple of 24 */
	{ IMX415_PIX_VST, 32 },		/* 16 lines x 2, multiple of 4 */
	{ IMX415_PIX_VWIDTH, 4320 },	/* 2160 lines x 2, multiple of 4 */
	/* active lines, need to be set as per crop window when OB is enabled */
	{ IMX415_ACTIVE_LOW, 0x70 },	/* 2160 */
	{ IMX415_ACTIVE_HIGH, 0x08 },
};

/* Window cropping: 1920x1080 (FHD) centred in the effective array. */
static const struct cci_reg_sequence imx415_window_crop_fhd[] = {
	{ IMX415_WINMODE, 0x04 },
	{ IMX415_PIX_HST, 972 },	/* (3864 - 1920) / 2, multiple of 2 */
	{ IMX415_PIX_HWIDTH, 1920 },	/* multiple of 24 */
	{ IMX415_PIX_VST, 1112 },	/* 556 lines x 2, multiple of 4 */
	{ IMX415_PIX_VWIDTH, 2160 },	/* 1080 lines x 2, multiple of 4 */
	/* active lines, need to be set as per crop window when OB is enabled */
	{ IMX415_ACTIVE_LOW, 0x38 },	/* 1080 */
	{ IMX415_ACTIVE_HIGH, 0x04 },
};

#define IMX415_DEFAULT_MODE 0

static const struct imx415_readout_mode imx415_readout_modes[] = {
	/* [0] 3840x2160 (UHD) window crop, centred; VMAX comes from fps_target. */
	{
		.width = 3840,
		.height = 2160,
		.crop_top = 16,
		.crop_left = 12,
		.vblank_min = 90,
		.fps_target = { 10, 30 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_window_crop_uhd),
			.regs = imx415_window_crop_uhd,
		},
	},
	/* [1] Full 3864x2192 effective array, all-pixel readout. */
	{
		.width = IMX415_NATIVE_WIDTH,
		.height = IMX415_NATIVE_HEIGHT,
		.crop_top = 0,
		.crop_left = 0,
		.vblank_min = 58,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_window_all_pixel),
			.regs = imx415_window_all_pixel,
		},
	},
	/* [2] 1920x1080 (FHD) window crop, centred; VMAX comes from fps_target. */
	{
		.width = 1920,
		.height = 1080,
		.crop_top = 556,
		.crop_left = 972,
		.vblank_min = 46,
		.fps_target = { 30, 30 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_window_crop_fhd),
			.regs = imx415_window_crop_fhd,
		},
	},
};

/*
 * Supported output formats. Bit depth is orthogonal to the readout-window
 * configuration: it selects ADBIT/MDBIT (0 = 10-bit, 1 = 12-bit), the
 * matching AD converter precision ADBIT1 (00h = 10-bit, 03h = 12-bit) and
 * the CSI-2 data type. SYS_MODE and the D-PHY timing are driven by the lane
 * rate; the HMAX line minima are per-(lane, rate, depth) mode-table values.
 */
struct imx415_pixfmt {
	u32 code;
	u8 bit_setting;		/* ADBIT/MDBIT register value */
	u8 adbit1;		/* ADBIT1 (AD converter bits) register value */
	u8 csi_dt;		/* MIPI CSI-2 data type */
};

static const struct imx415_pixfmt imx415_formats[] = {
	{ MEDIA_BUS_FMT_SGBRG10_1X10, 0x00, 0x00, MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, 0x01, 0x03, MIPI_CSI2_DT_RAW12 },
};

/* Look up a format by mbus code, defaulting to 10-bit for an unknown code. */
static const struct imx415_pixfmt *imx415_find_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx415_formats); i++)
		if (imx415_formats[i].code == code)
			return &imx415_formats[i];

	return &imx415_formats[0];
}

static const char *const imx415_test_pattern_menu[] = {
	"disabled",
	"solid black",
	"solid white",
	"solid dark gray",
	"solid light gray",
	"stripes light/dark grey",
	"stripes dark/light grey",
	"stripes black/dark grey",
	"stripes dark grey/black",
	"stripes black/white",
	"stripes white/black",
	"horizontal color bar",
	"vertical color bar",
};

struct imx415 {
	struct device *dev;
	struct clk *clk;
	unsigned long pixel_rate;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx415_supply_names)];
	struct gpio_desc *reset;
	struct regmap *regmap;

	const struct imx415_clk_params *clk_params;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *exposure;

	unsigned int cur_mode;		/* lane-rate / CSI-timing index */
	unsigned int num_data_lanes;
};

/*
 * This table includes fixed register settings and a bunch of undocumented
 * registers that have to be set to another value than default.
 */
static const struct cci_reg_sequence imx415_init_table[] = {
	/* no binning, no flip; the readout window is selected per mode */
	{ IMX415_ADDMODE, 0x00 },
	{ IMX415_REVERSE, 0x00 },
	/* ADBIT/MDBIT (RAW bit depth) are set per the active format in setup */
	/* output VSYNC on XVS and low on XHS */
	{ IMX415_OUTSEL, 0x22 },
	{ IMX415_DRV, 0x00 },

	/* Disable embedded and optical black data */
	{ IMX415_EMB_DATA, 0x00 },
	{ IMX415_OB_DATA_1, 0x00 },
	{ IMX415_OB_DATA_2, 0x80 },

	/* SONY magic registers */
	{ CCI_REG8(0x32D4), 0x21 },
	{ CCI_REG8(0x32EC), 0xA1 },
	{ CCI_REG8(0x3452), 0x7F },
	{ CCI_REG8(0x3453), 0x03 },
	{ CCI_REG8(0x358A), 0x04 },
	{ CCI_REG8(0x35A1), 0x02 },
	{ CCI_REG8(0x36BC), 0x0C },
	{ CCI_REG8(0x36CC), 0x53 },
	{ CCI_REG8(0x36CD), 0x00 },
	{ CCI_REG8(0x36CE), 0x3C },
	{ CCI_REG8(0x36D0), 0x8C },
	{ CCI_REG8(0x36D1), 0x00 },
	{ CCI_REG8(0x36D2), 0x71 },
	{ CCI_REG8(0x36D4), 0x3C },
	{ CCI_REG8(0x36D6), 0x53 },
	{ CCI_REG8(0x36D7), 0x00 },
	{ CCI_REG8(0x36D8), 0x71 },
	{ CCI_REG8(0x36DA), 0x8C },
	{ CCI_REG8(0x36DB), 0x00 },
	{ CCI_REG8(0x3724), 0x02 },
	{ CCI_REG8(0x3726), 0x02 },
	{ CCI_REG8(0x3732), 0x02 },
	{ CCI_REG8(0x3734), 0x03 },
	{ CCI_REG8(0x3736), 0x03 },
	{ CCI_REG8(0x3742), 0x03 },
	{ CCI_REG8(0x3862), 0xE0 },
	{ CCI_REG8(0x38CC), 0x30 },
	{ CCI_REG8(0x38CD), 0x2F },
	{ CCI_REG8(0x395C), 0x0C },
	{ CCI_REG8(0x3A42), 0xD1 },
	{ CCI_REG8(0x3A4C), 0x77 },
	{ CCI_REG8(0x3AE0), 0x02 },
	{ CCI_REG8(0x3AEC), 0x0C },
	{ CCI_REG8(0x3B00), 0x2E },
	{ CCI_REG8(0x3B06), 0x29 },
	{ CCI_REG8(0x3B98), 0x25 },
	{ CCI_REG8(0x3B99), 0x21 },
	{ CCI_REG8(0x3B9B), 0x13 },
	{ CCI_REG8(0x3B9C), 0x13 },
	{ CCI_REG8(0x3B9D), 0x13 },
	{ CCI_REG8(0x3B9E), 0x13 },
	{ CCI_REG8(0x3BA1), 0x00 },
	{ CCI_REG8(0x3BA2), 0x06 },
	{ CCI_REG8(0x3BA3), 0x0B },
	{ CCI_REG8(0x3BA4), 0x10 },
	{ CCI_REG8(0x3BA5), 0x14 },
	{ CCI_REG8(0x3BA6), 0x18 },
	{ CCI_REG8(0x3BA7), 0x1A },
	{ CCI_REG8(0x3BA8), 0x1A },
	{ CCI_REG8(0x3BA9), 0x1A },
	{ CCI_REG8(0x3BAC), 0xED },
	{ CCI_REG8(0x3BAD), 0x01 },
	{ CCI_REG8(0x3BAE), 0xF6 },
	{ CCI_REG8(0x3BAF), 0x02 },
	{ CCI_REG8(0x3BB0), 0xA2 },
	{ CCI_REG8(0x3BB1), 0x03 },
	{ CCI_REG8(0x3BB2), 0xE0 },
	{ CCI_REG8(0x3BB3), 0x03 },
	{ CCI_REG8(0x3BB4), 0xE0 },
	{ CCI_REG8(0x3BB5), 0x03 },
	{ CCI_REG8(0x3BB6), 0xE0 },
	{ CCI_REG8(0x3BB7), 0x03 },
	{ CCI_REG8(0x3BB8), 0xE0 },
	{ CCI_REG8(0x3BBA), 0xE0 },
	{ CCI_REG8(0x3BBC), 0xDA },
	{ CCI_REG8(0x3BBE), 0x88 },
	{ CCI_REG8(0x3BC0), 0x44 },
	{ CCI_REG8(0x3BC2), 0x7B },
	{ CCI_REG8(0x3BC4), 0xA2 },
	{ CCI_REG8(0x3BC8), 0xBD },
	{ CCI_REG8(0x3BCA), 0xBD },
};

static inline struct imx415 *to_imx415(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx415, subdev);
}

static const struct imx415_readout_mode *imx415_find_mode(u32 width, u32 height)
{
	return v4l2_find_nearest_size(imx415_readout_modes,
				      ARRAY_SIZE(imx415_readout_modes),
				      width, height, width, height);
}

/* A half-width window on 2 lanes needs no more line than full width on 4. */
static u32 imx415_hmax_min(const struct imx415 *sensor,
			   const struct imx415_readout_mode *mode, bool raw12)
{
	const struct imx415_mode *cur = &supported_modes[sensor->cur_mode];
	unsigned int idx = sensor->num_data_lanes == 2 ? 0 : 1;

	if (idx == 0 && mode->width * 2 <= IMX415_NATIVE_WIDTH)
		idx = 1;

	/*
	 * 12-bit line minima are discrete per-mode datasheet values, not a
	 * scaling of the 10-bit ones (e.g. 4-lane 1440 Mbps doubles).
	 */
	if (raw12 && cur->hmax_min_12[idx])
		return cur->hmax_min_12[idx];

	return cur->hmax_min[idx];
}

/* Hold the frame rate with HMAX, so VMAX and the line burst stay minimal. */
static u32 imx415_hblank_def(const struct imx415 *sensor,
			     const struct imx415_readout_mode *mode,
			     u32 hblank_min)
{
	u32 fps = mode->fps_target[sensor->num_data_lanes == 2 ? 0 : 1];
	u32 hts;

	if (!fps || !sensor->pixel_rate)
		return hblank_min;

	/* Round down: a short HMAX overshoots the target, a long one misses. */
	hts = sensor->pixel_rate / ((mode->height + mode->vblank_min) * fps);
	hts -= hts % IMX415_HMAX_MULTIPLIER;

	if (hts < hblank_min + mode->width)
		return hblank_min;

	return hts - mode->width;
}

/*
 * Update the blanking and exposure control ranges to match the selected
 * readout window. Called from .set_fmt with the subdev state (== control
 * handler) lock held, so the __v4l2_ctrl_modify_range() variants are used.
 */
static void imx415_update_controls(struct imx415 *sensor,
				   const struct imx415_readout_mode *mode,
				   u32 code)
{
	u32 hblank_min, hblank_max, hblank_def, exposure_max;
	u32 hmax_min = imx415_hmax_min(sensor, mode,
				       imx415_find_format(code)->csi_dt ==
				       MIPI_CSI2_DT_RAW12);

	hblank_min = (hmax_min * IMX415_HMAX_MULTIPLIER) - mode->width;
	hblank_max = (IMX415_HMAX_MAX * IMX415_HMAX_MULTIPLIER) - mode->width;
	hblank_def = imx415_hblank_def(sensor, mode, hblank_min);
	__v4l2_ctrl_modify_range(sensor->hblank, hblank_min, hblank_max,
				 IMX415_HMAX_MULTIPLIER, hblank_def);
	/* modify_range moves the default only; the value must follow too. */
	__v4l2_ctrl_s_ctrl(sensor->hblank, hblank_def);

	__v4l2_ctrl_modify_range(sensor->vblank, mode->vblank_min,
				 IMX415_VMAX_MAX - mode->height, 1,
				 mode->vblank_min);
	__v4l2_ctrl_s_ctrl(sensor->vblank, mode->vblank_min);

	exposure_max = mode->height + mode->vblank_min - IMX415_EXPOSURE_OFFSET;
	__v4l2_ctrl_modify_range(sensor->exposure, sensor->exposure->minimum,
				 exposure_max, sensor->exposure->step,
				 exposure_max);
}

static int imx415_set_testpattern(struct imx415 *sensor, int val)
{
	int ret = 0;

	if (val) {
		cci_write(sensor->regmap, IMX415_BLKLEVEL, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_TPG_EN_DUOUT, 0x01, &ret);
		cci_write(sensor->regmap, IMX415_TPG_PATSEL_DUOUT,
			  val - 1, &ret);
		cci_write(sensor->regmap, IMX415_TPG_COLORWIDTH, 0x01, &ret);
		cci_write(sensor->regmap, IMX415_TESTCLKEN_MIPI, 0x20, &ret);
		cci_write(sensor->regmap, IMX415_DIG_CLP_MODE, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_WRJ_OPEN, 0x00, &ret);
	} else {
		cci_write(sensor->regmap, IMX415_BLKLEVEL,
			  IMX415_BLKLEVEL_DEFAULT, &ret);
		cci_write(sensor->regmap, IMX415_TPG_EN_DUOUT, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_TESTCLKEN_MIPI, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_DIG_CLP_MODE, 0x01, &ret);
		cci_write(sensor->regmap, IMX415_WRJ_OPEN, 0x01, &ret);
	}
	return 0;
}

static int imx415_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx415 *sensor = container_of(ctrl->handler, struct imx415,
					     ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	u32 exposure_max;
	unsigned int vmax;
	unsigned int flip;
	int ret;

	state = v4l2_subdev_get_locked_active_state(&sensor->subdev);
	format = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = format->height + ctrl->val -
			       IMX415_EXPOSURE_OFFSET;
		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 exposure_max, sensor->exposure->step,
					 sensor->exposure->default_value);
	}

	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = cci_write(sensor->regmap, IMX415_VMAX,
				format->height + ctrl->val, NULL);
		if (ret)
			return ret;
		/*
		 * Exposure is set based on VMAX which has just changed, so
		 * program exposure register as well
		 */
		ctrl = sensor->exposure;
		fallthrough;
	case V4L2_CID_EXPOSURE:
		/* clamp the exposure value to VMAX. */
		vmax = format->height + sensor->vblank->cur.val;
		ctrl->val = min_t(int, ctrl->val, vmax);
		ret = cci_write(sensor->regmap, IMX415_SHR0,
				vmax - ctrl->val, NULL);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		/* analogue gain in 0.3 dB step size */
		ret = cci_write(sensor->regmap, IMX415_GAIN_PCG_0,
				ctrl->val, NULL);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		flip = (sensor->hflip->val << IMX415_HREVERSE_SHIFT) |
		       (sensor->vflip->val << IMX415_VREVERSE_SHIFT);
		ret = cci_write(sensor->regmap, IMX415_REVERSE, flip, NULL);
		break;

	case V4L2_CID_TEST_PATTERN:
		ret = imx415_set_testpattern(sensor, ctrl->val);
		break;

	case V4L2_CID_HBLANK:
		ret = cci_write(sensor->regmap, IMX415_HMAX,
				(format->width + ctrl->val) /
						IMX415_HMAX_MULTIPLIER,
				NULL);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx415_ctrl_ops = {
	.s_ctrl = imx415_s_ctrl,
};

static int imx415_ctrls_init(struct imx415 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	const struct imx415_mode *cur_mode = &supported_modes[sensor->cur_mode];
	const struct imx415_readout_mode *mode =
		&imx415_readout_modes[IMX415_DEFAULT_MODE];
	u64 lane_rate = cur_mode->lane_rate;
	u32 hblank_min = (imx415_hmax_min(sensor, mode, false) *
			  IMX415_HMAX_MULTIPLIER) - mode->width;
	u32 hblank_max = (IMX415_HMAX_MAX * IMX415_HMAX_MULTIPLIER) -
			 mode->width;
	u32 hblank_def = imx415_hblank_def(sensor, mode, hblank_min);
	u32 exposure_max = mode->height + mode->vblank_min -
			   IMX415_EXPOSURE_OFFSET;
	unsigned int i;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 10);

	for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); ++i) {
		if (lane_rate == link_freq_menu_items[i] * 2)
			break;
	}
	if (i == ARRAY_SIZE(link_freq_menu_items)) {
		return dev_err_probe(sensor->dev, -EINVAL,
				     "lane rate %llu not supported\n",
				     lane_rate);
	}

	ctrl = v4l2_ctrl_new_int_menu(&sensor->ctrls, &imx415_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq_menu_items) - 1, i,
				      link_freq_menu_items);

	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					     V4L2_CID_EXPOSURE, 4,
					     exposure_max, 1, exposure_max);

	v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, IMX415_AGAIN_MIN,
			  IMX415_AGAIN_MAX, IMX415_AGAIN_STEP,
			  IMX415_AGAIN_MIN);

	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					   V4L2_CID_HBLANK, hblank_min,
					   hblank_max, IMX415_HMAX_MULTIPLIER,
					   hblank_def);

	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					   V4L2_CID_VBLANK, mode->vblank_min,
					   IMX415_VMAX_MAX - mode->height,
					   1, mode->vblank_min);

	v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  sensor->pixel_rate, sensor->pixel_rate, 1,
			  sensor->pixel_rate);

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &imx415_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx415_test_pattern_menu) - 1,
				     0, 0, imx415_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &imx415_ctrl_ops,
					&props);

	if (sensor->ctrls.error) {
		dev_err_probe(sensor->dev, sensor->ctrls.error,
			      "failed to add controls\n");
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return sensor->ctrls.error;
	}
	sensor->subdev.ctrl_handler = &sensor->ctrls;

	return 0;
}

static int imx415_set_mode(struct imx415 *sensor, int mode)
{
	int ret = 0;

	if (mode >= ARRAY_SIZE(supported_modes)) {
		dev_err(sensor->dev, "Mode %d not supported\n", mode);
		return -EINVAL;
	}

	cci_multi_reg_write(sensor->regmap,
			    supported_modes[mode].reg_list.regs,
			    supported_modes[mode].reg_list.num_of_regs,
			    &ret);

	cci_multi_reg_write(sensor->regmap,
			    sensor->clk_params->regs,
			    IMX415_NUM_CLK_PARAM_REGS,
			    &ret);

	ret = cci_write(sensor->regmap, IMX415_LANEMODE,
			sensor->num_data_lanes == 2 ? IMX415_LANEMODE_2 :
						      IMX415_LANEMODE_4,
			NULL);

	return ret;
}

static int imx415_setup(struct imx415 *sensor, struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format =
		v4l2_subdev_state_get_format(state, 0);
	const struct imx415_readout_mode *mode =
		imx415_find_mode(format->width, format->height);
	const struct imx415_pixfmt *pixfmt = imx415_find_format(format->code);
	int ret;

	ret = cci_multi_reg_write(sensor->regmap,
				  imx415_init_table,
				  ARRAY_SIZE(imx415_init_table),
				  NULL);
	if (ret)
		return ret;

	/* Select the RAW bit depth (10- or 12-bit) for the active format:
	 * output/bus width (ADBIT/MDBIT) and the matching AD converter
	 * precision (ADBIT1). */
	ret = 0;
	cci_write(sensor->regmap, IMX415_ADBIT, pixfmt->bit_setting, &ret);
	cci_write(sensor->regmap, IMX415_MDBIT, pixfmt->bit_setting, &ret);
	cci_write(sensor->regmap, IMX415_ADBIT1, pixfmt->adbit1, &ret);
	if (ret)
		return ret;

	/* Select the readout window (all-pixel or window cropping). */
	ret = cci_multi_reg_write(sensor->regmap,
				  mode->reg_list.regs,
				  mode->reg_list.num_of_regs,
				  NULL);
	if (ret)
		return ret;

	return imx415_set_mode(sensor, sensor->cur_mode);
}

static int imx415_wakeup(struct imx415 *sensor)
{
	int ret;

	ret = cci_write(sensor->regmap, IMX415_MODE,
			IMX415_MODE_OPERATING, NULL);
	if (ret)
		return ret;

	/*
	 * According to the datasheet we have to wait at least 63 us after
	 * leaving standby mode. But this doesn't work even after 30 ms.
	 * So probably this should be 63 ms and therefore we wait for 80 ms.
	 */
	msleep(80);

	return 0;
}

static int imx415_stream_on(struct imx415 *sensor)
{
	int ret;

	ret = imx415_wakeup(sensor);
	return cci_write(sensor->regmap, IMX415_XMSTA,
			 IMX415_XMSTA_START, &ret);
}

static int imx415_stream_off(struct imx415 *sensor)
{
	int ret;

	ret = cci_write(sensor->regmap, IMX415_XMSTA,
			IMX415_XMSTA_STOP, NULL);
	return cci_write(sensor->regmap, IMX415_MODE,
			 IMX415_MODE_STANDBY, &ret);
}

static int imx415_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx415 *sensor = to_imx415(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = imx415_stream_off(sensor);

		pm_runtime_put_autosuspend(sensor->dev);

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = imx415_setup(sensor, state);
	if (ret)
		goto err_pm;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret < 0)
		goto err_pm;

	ret = imx415_stream_on(sensor);
	if (ret)
		goto err_pm;

	ret = 0;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;

err_pm:
	/*
	 * In case of error, turn the power off synchronously as the device
	 * likely has no other chance to recover.
	 */
	pm_runtime_put_sync(sensor->dev);

	goto unlock;
}

/*
 * Whether the active rate/lane combination can carry @fmt. 12-bit needs an
 * HMAX minimum the mode may not define, and both enumerations and set_format
 * have to agree about that - a frame size offered for a code the mbus
 * enumeration hides is an invitation to build a pipeline that then
 * silently negotiates something else.
 */
static bool imx415_format_supported(struct imx415 *sensor,
				    const struct imx415_pixfmt *fmt)
{
	if (fmt->csi_dt != MIPI_CSI2_DT_RAW12)
		return true;

	return supported_modes[sensor->cur_mode]
		       .hmax_min_12[sensor->num_data_lanes == 2 ? 0 : 1] != 0;
}

static int imx415_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx415 *sensor = to_imx415(sd);

	if (code->index >= ARRAY_SIZE(imx415_formats))
		return -EINVAL;

	if (!imx415_format_supported(sensor, &imx415_formats[code->index]))
		return -EINVAL;

	code->code = imx415_formats[code->index].code;

	return 0;
}

static int imx415_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx415 *sensor = to_imx415(sd);
	const struct imx415_pixfmt *fmt = imx415_find_format(fse->code);

	if (fse->index >= ARRAY_SIZE(imx415_readout_modes) ||
	    fmt->code != fse->code || !imx415_format_supported(sensor, fmt))
		return -EINVAL;

	fse->min_width = imx415_readout_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx415_readout_modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static int imx415_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	u32 code;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, 0, 0)->code;
	v4l2_subdev_unlock_state(state);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry->pixelcode = code;
	fd->entry->stream = 0;
	fd->entry->bus.csi2.vc = 0; //Read from device tree
	fd->entry->bus.csi2.dt = imx415_find_format(code)->csi_dt;

	return 0;
}

static int imx415_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct imx415 *sensor = to_imx415(sd);
	const struct imx415_readout_mode *mode;
	const struct imx415_pixfmt *pixfmt;
	struct v4l2_mbus_framefmt *format;

	mode = imx415_find_mode(fmt->format.width, fmt->format.height);

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	format->width = mode->width;
	format->height = mode->height;
	pixfmt = imx415_find_format(fmt->format.code);
	/*
	 * Fall back to 10-bit where the datasheet defines no 12-bit setting
	 * for the active rate/lane combination.
	 */
	if (pixfmt->csi_dt == MIPI_CSI2_DT_RAW12 &&
	    !supported_modes[sensor->cur_mode]
		     .hmax_min_12[sensor->num_data_lanes == 2 ? 0 : 1])
		pixfmt = &imx415_formats[0];
	format->code = pixfmt->code;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		imx415_update_controls(sensor, mode, format->code);

	return 0;
}

static int imx415_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	const struct v4l2_mbus_framefmt *format;
	const struct imx415_readout_mode *mode;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		format = v4l2_subdev_state_get_format(sd_state, 0);
		mode = imx415_find_mode(format->width, format->height);
		sel->r.top = mode->crop_top;
		sel->r.left = mode->crop_left;
		sel->r.width = mode->width;
		sel->r.height = mode->height;

		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX415_NATIVE_WIDTH;
		sel->r.height = IMX415_NATIVE_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx415_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = imx415_readout_modes[IMX415_DEFAULT_MODE].width,
			.height = imx415_readout_modes[IMX415_DEFAULT_MODE].height,
		},
	};

	imx415_set_format(sd, state, &format);

	return 0;
}

static const struct v4l2_subdev_video_ops imx415_subdev_video_ops = {
	.s_stream = imx415_s_stream,
};

static const struct v4l2_subdev_pad_ops imx415_subdev_pad_ops = {
	.get_frame_desc = imx415_get_frame_desc,
	.enum_mbus_code = imx415_enum_mbus_code,
	.enum_frame_size = imx415_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx415_set_format,
	.get_selection = imx415_get_selection,
};

static const struct v4l2_subdev_ops imx415_subdev_ops = {
	.video = &imx415_subdev_video_ops,
	.pad = &imx415_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx415_internal_ops = {
	.init_state = imx415_init_state,
};

static int imx415_subdev_init(struct imx415 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &imx415_subdev_ops);
	sensor->subdev.internal_ops = &imx415_internal_ops;

	ret = imx415_ctrls_init(sensor);
	if (ret)
		return ret;

	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	sensor->subdev.state_lock = sensor->subdev.ctrl_handler->lock;
	v4l2_subdev_init_finalize(&sensor->subdev);

	return 0;
}

static void imx415_subdev_cleanup(struct imx415 *sensor)
{
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

static int imx415_power_on(struct imx415 *sensor)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret < 0)
		return ret;

	gpiod_set_value_cansleep(sensor->reset, 0);

	udelay(1);

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto err_reset;

	/*
	 * Data sheet states that 20 us are required before communication start,
	 * but this doesn't work in all cases. Use 100 us to be on the safe
	 * side.
	 */
	usleep_range(100, 200);

	return 0;

err_reset:
	gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	return ret;
}

static void imx415_power_off(struct imx415 *sensor)
{
	clk_disable_unprepare(sensor->clk);
	gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

static int imx415_identify_model(struct imx415 *sensor)
{
	int model, ret;
	u64 chip_id;

	/*
	 * While most registers can be read when the sensor is in standby, this
	 * is not the case of the sensor info register :-(
	 */
	ret = imx415_wakeup(sensor);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get sensor out of standby\n");

	ret = cci_read(sensor->regmap, IMX415_SENSOR_INFO, &chip_id, NULL);
	if (ret < 0) {
		dev_err_probe(sensor->dev, ret,
			      "failed to read sensor information\n");
		goto done;
	}

	model = chip_id & IMX415_SENSOR_INFO_MASK;

	switch (model) {
	case IMX415_CHIP_ID:
		dev_info(sensor->dev, "Detected IMX415 image sensor\n");
		break;
	default:
		ret = dev_err_probe(sensor->dev, -ENODEV,
				    "invalid device model 0x%04x\n", model);
		goto done;
	}

	ret = 0;

done:
	cci_write(sensor->regmap, IMX415_MODE, IMX415_MODE_STANDBY, &ret);
	return ret;
}

static int imx415_check_inck(unsigned long inck, u64 link_frequency)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx415_clk_params); ++i) {
		if ((imx415_clk_params[i].lane_rate == link_frequency * 2) &&
		    imx415_clk_params[i].inck == inck)
			break;
	}

	if (i == ARRAY_SIZE(imx415_clk_params))
		return -EINVAL;
	else
		return 0;
}

static int imx415_parse_hw_config(struct imx415 *sensor)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	u64 lane_rate;
	unsigned long inck;
	unsigned int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); ++i)
		sensor->supplies[i].supply = imx415_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get supplies\n");

	sensor->reset = devm_gpiod_get_optional(sensor->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	sensor->clk = devm_v4l2_sensor_clk_get(sensor->dev, NULL);
	if (IS_ERR(sensor->clk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->clk),
				     "failed to get clock\n");

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		sensor->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "invalid number of CSI2 data lanes %d\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no link frequencies defined");
		goto done_endpoint_free;
	}

	/*
	 * Check if there exists a sensor mode defined for current INCK,
	 * number of lanes and given lane rates.
	 */
	inck = clk_get_rate(sensor->clk);
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; ++i) {
		if (imx415_check_inck(inck, bus_cfg.link_frequencies[i])) {
			dev_dbg(sensor->dev,
				"INCK %lu Hz not supported for this link freq",
				inck);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(supported_modes); ++j) {
			if (bus_cfg.link_frequencies[i] * 2 !=
			    supported_modes[j].lane_rate)
				continue;
			sensor->cur_mode = j;
			break;
		}
		if (j < ARRAY_SIZE(supported_modes))
			break;
	}
	if (i == bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no valid sensor mode defined\n");
		goto done_endpoint_free;
	}
	switch (inck) {
	case 27000000:
	case 37125000:
	case 74250000:
		sensor->pixel_rate = IMX415_PIXEL_RATE_74_25MHZ;
		break;
	case 24000000:
	case 72000000:
		sensor->pixel_rate = IMX415_PIXEL_RATE_72MHZ;
		break;
	}

	lane_rate = supported_modes[sensor->cur_mode].lane_rate;
	for (i = 0; i < ARRAY_SIZE(imx415_clk_params); ++i) {
		if (lane_rate == imx415_clk_params[i].lane_rate &&
		    inck == imx415_clk_params[i].inck) {
			sensor->clk_params = &imx415_clk_params[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(imx415_clk_params)) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "Mode %d not supported\n",
				    sensor->cur_mode);
		goto done_endpoint_free;
	}

	ret = 0;
	dev_dbg(sensor->dev, "clock: %lu Hz, lane_rate: %llu bps, lanes: %d\n",
		inck, lane_rate, sensor->num_data_lanes);

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx415_probe(struct i2c_client *client)
{
	struct imx415 *sensor;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	ret = imx415_parse_hw_config(sensor);
	if (ret)
		return ret;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = imx415_power_on(sensor);
	if (ret)
		return ret;

	ret = imx415_identify_model(sensor);
	if (ret)
		goto err_power;

	ret = imx415_subdev_init(sensor);
	if (ret)
		goto err_power;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->subdev);
	if (ret < 0)
		goto err_pm;

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	dev_info(sensor->dev, "Probe is successful\n");
	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	imx415_subdev_cleanup(sensor);
err_power:
	imx415_power_off(sensor);
	return ret;
}

static void imx415_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx415 *sensor = to_imx415(subdev);

	v4l2_async_unregister_subdev(subdev);

	imx415_subdev_cleanup(sensor);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		imx415_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static int imx415_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx415 *sensor = to_imx415(subdev);

	return imx415_power_on(sensor);
}

static int imx415_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx415 *sensor = to_imx415(subdev);

	imx415_power_off(sensor);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx415_pm_ops, imx415_runtime_suspend,
				 imx415_runtime_resume, NULL);

static const struct of_device_id imx415_of_match[] = {
	{ .compatible = "sony,imx415" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, imx415_of_match);

static struct i2c_driver imx415_driver = {
	.probe = imx415_probe,
	.remove = imx415_remove,
	.driver = {
		.name = "imx415",
		.of_match_table = imx415_of_match,
		.pm = pm_ptr(&imx415_pm_ops),
	},
};

module_i2c_driver(imx415_driver);

MODULE_DESCRIPTION("Sony IMX415 image sensor driver");
MODULE_AUTHOR("Gerald Loacker <gerald.loacker@wolfvision.net>");
MODULE_AUTHOR("Michael Riesch <michael.riesch@wolfvision.net>");
MODULE_LICENSE("GPL");
