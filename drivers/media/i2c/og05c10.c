// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony OG05C10 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

static int dpc_enable = 1;
module_param(dpc_enable, int, 0644);
MODULE_PARM_DESC(dpc_enable, "Enable on-sensor DPC");

static int trigger_mode;
module_param(trigger_mode, int, 0644);
MODULE_PARM_DESC(trigger_mode, "Set vsync trigger mode: 1=source, 2=sink");

#define OG05C10_REG_VALUE_08BIT		1
#define OG05C10_REG_VALUE_16BIT		2
#define OG05C10_REG_VALUE_24BIT		3

/* Chip ID */
#define OG05C10_REG_CHIP_ID		0x300a
#define OG05C10_CHIP_ID			0x530543

#define OG05C10_REG_MODE_SELECT		0x0100

#define OG05C10_MODE_STOP_STREAMING	0x00
#define OG05C10_MODE_START_STREAMING	0x01

#define OG05C10_XCLK_FREQ		24000000

#define OG05C10_DEFAULT_LINK_FREQ	424000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define OG05C10_PIXEL_RATE		848000000

#define EXPOSURE_FACTOR			1000000
#define GAIN_FACTOR				10



/* V_TIMING internal */
#define OG05C10_REG_FRAME_LENGTH	0x0340
#define OG05C10_FRAME_LENGTH_MAX	0xffdc

/* H_TIMING internal */
#define OG05C10_REG_LINE_LENGTH		0x0342
#define OG05C10_LINE_LENGTH_MAX		0xfff0

/* Long exposure multiplier */
#define OG05C10_LONG_EXP_SHIFT_MAX	7

/* Exposure control */
#define OG05C10_REG_EXP_HI		0x3500
#define OG05C10_REG_EXP_MID		0x3501
#define OG05C10_REG_EXP_LO		0x3502
#define OG05C10_EXP_MIN			0

/* Analog gain control */
#define OG05C10_REG_GAIN_HI		0x3509
#define OG05C10_REG_GAIN_LO		0x3508

#define OG05C10_VBLANK_MIN		24
#define OG05C10_VTS_MAX			32767
#define OG05C10_VTS			(0x0868)

#define OG05C10_HTS_REG_VAL		(0x0C58) //number of clocks per line
#define OG05C10_HTS				(OG05C10_HTS_REG_VAL*2*2) //OG05C10_HTS_REG_VAL*(number of lanes)*2
#define OG05C10_HTS_MAX			0x7fff

#define OG05C10_AGAIN_MIN			0
#define OG05C10_AGAIN_STEP			1
#define OG05C10_AGAIN_DEFAULT			0
#define OG05C10_AGAIN_MAX			(32 << 18)

/* Embedded metadata stream structure */
#define OG05C10_EMBEDDED_LINE_WIDTH 16384
#define OG05C10_NUM_EMBEDDED_LINES 1


#define OG05C10_EXPOSURE_MIN		(0x10+4)
#define OG05C10_EXPOSURE_STEP	1
#define OG05C10_EXPOSURE_DEFAULT	(0x70)
#define OG05C10_EXPOSURE_MAX		(OG05C10_VTS / 2 - 10)

#define OG05C10_DELAY_REG 0

#define OG05C10_DEBUG
#if defined(OG05C10_DEBUG)
#define OG05C10_TAG				       "og05c10: "
#define OG05C10_FUN(f)               	printk(OG05C10_TAG"%s\n", __FUNCTION__)
#define OG05C10_ERR(fmt, args...)    	printk(OG05C10_TAG"%s[%d] " fmt, __FUNCTION__, __LINE__, ##args)
#define OG05C10_LOG(fmt, args...)    	printk(OG05C10_TAG"%s[%d] " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define OG05C10_FUN()
#define OG05C10_LOG(fmt, args...)
#define OG05C10_ERR(fmt, args...)
#endif

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* OG05C10 native and active pixel array size. */
#define OG05C10_NATIVE_WIDTH		2048U
#define OG05C10_NATIVE_HEIGHT		1080U
#define OG05C10_PIXEL_ARRAY_LEFT		0U
#define OG05C10_PIXEL_ARRAY_TOP		0U
#define OG05C10_PIXEL_ARRAY_WIDTH	1920U
#define OG05C10_PIXEL_ARRAY_HEIGHT	1080U

struct og05c10_reg {
	u16 address;
	u8 val;
};

struct og05c10_reg_list {
	unsigned int num_of_regs;
	const struct og05c10_reg *regs;
};

/* Mode : resolution and related config&values */
struct og05c10_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* H-timing in pixels */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	struct v4l2_fract timeperframe_min;

	/* Default framerate. */
	struct v4l2_fract timeperframe_default;

	/* Default register values */
	struct og05c10_reg_list reg_list;
};

static const s64 og05c10_link_freq_menu[] = {
	OG05C10_DEFAULT_LINK_FREQ,
};

static const struct og05c10_reg mode_1920x1080_regs[] = {
//OG05C10_1920x1080_Lin12_30fps_848M_2laneMIPI
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x0109, 0x01},
	{0x0104, 0x02},
	{0x0104, 0x00},
	{0x0109, 0x00},
	{0x4004, 0x01},
	{0x4005, 0x00},
	{0x402e, 0x01},
	{0x402f, 0x00},
	{0x3616, 0x06},
	{0x3617, 0xaa},
	{0x361c, 0x06},
	{0x361d, 0xaa},
	{0x3626, 0x80},
	{0x450f, 0x82},
	{0x450e, 0x64},
	{0x3a2b, 0x00},
	{0x3a2c, 0x14},
	{0x3a2d, 0x02},
	{0x3a2e, 0x90},
	{0x301f, 0xe9},
	{0x465b, 0x03},
	{0x301f, 0xe1},
	{0x3208, 0x02},
	{0x301b, 0xb5},
	{0x3017, 0xf5},
	{0x3869, 0x0c},
	{0x301f, 0xe9},
	{0x465b, 0x03},
	{0x301f, 0xe1},
	{0x320d, 0x8a},
	{0x320e, 0xaa},
	{0x322f, 0x00},
	{0x301b, 0xb4},
	{0x3017, 0xf0},
	{0x641d, 0x4e},
	{0x3208, 0x12},
	{0x3208, 0x03},
	{0x301b, 0xb5},
	{0x3017, 0xf5},
	{0x301f, 0xe9},
	{0x3869, 0x00},
	{0x3878, 0x00},
	{0x387e, 0x80},
	{0x301b, 0xb4},
	{0x3017, 0xf0},
	{0x3016, 0xf0},
	{0x3016, 0xf0},
	{0x3016, 0xf0},
	{0x301f, 0xe1},
	{0x320d, 0x0b},
	{0x322f, 0x80},
	{0x641d, 0x30},
	{0x301f, 0xe9},
	{0x465b, 0x01},
	{0x301f, 0xe1},
	{0x3208, 0x13},
	{0x3208, 0x00},
	{0x374b, 0x00},
	{0x6474, 0x28},
	{0x3208, 0x10},
	{0x3208, 0x01},
	{0x374b, 0x01},
	{0x6474, 0x1f},
	{0x3208, 0x11},
	{0x4604, 0x44},
	{0x362e, 0xfb},
	{0x3637, 0x11},
	{0x3638, 0x11},
	{0x3698, 0x1f},
	{0x3699, 0x1f},
	{0x369a, 0x1f},
	{0x369b, 0x1f},
	{0x369c, 0x1f},
	{0x369d, 0x1f},
	{0x369e, 0x1f},
	{0x369f, 0x1f},
	{0x36c6, 0x1f},
	{0x36c9, 0x00},
	{0x36ca, 0x00},
	{0x36cb, 0x00},
	{0x36cc, 0x00},
	{0x36cd, 0x00},
	{0x36ce, 0x00},
	{0x36cf, 0x00},
	{0x36d0, 0x00},
	{0x6432, 0x26},
	{0x6433, 0x00},
	{0x6434, 0x67},
	{0x6435, 0x30},
	{0x6436, 0x01},
	{0x6437, 0x69},
	{0x6438, 0xb8},
	{0x643a, 0x6b},
	{0x643b, 0x76},
	{0x643d, 0x6d},
	{0x643e, 0x1c},
	{0x643f, 0x00},
	{0x6440, 0x6f},
	{0x6441, 0x30},
	{0x6442, 0x00},
	{0x6443, 0x71},
	{0x6444, 0x30},
	{0x6445, 0x05},
	{0x6446, 0x73},
	{0x6447, 0x8b},
	{0x6448, 0x05},
	{0x6449, 0x75},
	{0x644a, 0x9a},
	{0x644b, 0x00},
	{0x644c, 0x81},
	{0x644d, 0xf8},
	{0x644f, 0x8d},
	{0x6450, 0xbd},
	{0x6452, 0x8e},
	{0x6453, 0xbc},
	{0x6455, 0x9a},
	{0x6456, 0x9f},
	{0x6457, 0x00},
	{0x6458, 0xba},
	{0x6459, 0x12},
	{0x645b, 0xd0},
	{0x645e, 0xd2},
	{0x645f, 0x08},
	{0x6461, 0xd4},
	{0x6464, 0xd6},
	{0x6465, 0x08},
	{0x6466, 0x01},
	{0x6467, 0xd8},
	{0x6468, 0xa9},
	{0x646a, 0xda},
	{0x646b, 0x4e},
	{0x646c, 0x00},
	{0x646d, 0xdc},
	{0x646e, 0x30},
	{0x646f, 0x05},
	{0x6470, 0xde},
	{0x6471, 0x8b},
	{0x6472, 0x05},
	{0x6473, 0xe0},
	{0x6474, 0x9a},
	{0x6476, 0xe2},
	{0x6479, 0xe4},
	{0x647a, 0xf8},
	{0x647c, 0xe6},
	{0x647f, 0xe8},
	{0x6480, 0x26},
	{0x6482, 0xea},
	{0x6483, 0x09},
	{0x6484, 0x00},
	{0x6485, 0xeb},
	{0x6486, 0x26},
	{0x6488, 0xed},
	{0x6489, 0x26},
	{0x648b, 0xef},
	{0x648c, 0x26},
	{0x648d, 0x00},
	{0x648e, 0xf1},
	{0x648f, 0x12},
	{0x6490, 0x00},
	{0x6491, 0xf3},
	{0x6492, 0x08},
	{0x6494, 0xf5},
	{0x6495, 0x26},
	{0x6497, 0xf7},
	{0x6498, 0x43},
	{0x649a, 0x00},
	{0x649b, 0x00},
	{0x649d, 0x00},
	{0x649e, 0x00},
	{0x64a0, 0x00},
	{0x64a1, 0x00},
	{0x64a3, 0x00},
	{0x64a4, 0x00},
	{0x64a6, 0x00},
	{0x64a7, 0x00},
	{0x64a8, 0x00},
	{0x64a9, 0x00},
	{0x64aa, 0x00},
	{0x64ac, 0x00},
	{0x64ad, 0x00},
	{0x64af, 0x00},
	{0x64b0, 0x00},
	{0x64b1, 0x00},
	{0x64b2, 0x00},
	{0x64b3, 0x00},
	{0x64b4, 0x00},
	{0x64b5, 0x00},
	{0x64b6, 0x00},
	{0x64b8, 0x00},
	{0x64b9, 0x00},
	{0x64bb, 0x00},
	{0x64bc, 0x00},
	{0x64be, 0x00},
	{0x64bf, 0x00},
	{0x64c1, 0x00},
	{0x64c2, 0x00},
	{0x64c4, 0x00},
	{0x64c5, 0x00},
	{0x64c7, 0x00},
	{0x64c8, 0x00},
	{0x64ca, 0x00},
	{0x64cb, 0x00},
	{0x64cd, 0x00},
	{0x64ce, 0x00},
	{0x64d0, 0x00},
	{0x64d1, 0x00},
	{0x64d3, 0x00},
	{0x64d4, 0x00},
	{0x64f1, 0xf1},
	{0x6507, 0xf1},
	{0x6508, 0x67},
	{0x6509, 0xf1},
	{0x650d, 0x69},
	{0x6510, 0x67},
	{0x6511, 0xf1},
	{0x6515, 0x69},
	{0x6518, 0xe0},
	{0x6519, 0xe6},
	{0x651e, 0xe0},
	{0x651f, 0xe6},
	{0x6524, 0x75},
	{0x6525, 0x83},
	{0x652a, 0x75},
	{0x652b, 0x83},
	{0x6532, 0xda},
	{0x6533, 0xde},
	{0x6538, 0xda},
	{0x6539, 0xde},
	{0x653e, 0x6d},
	{0x653f, 0x73},
	{0x6544, 0x6d},
	{0x6545, 0x73},
	{0x654a, 0xdc},
	{0x654b, 0xe4},
	{0x6550, 0xdc},
	{0x6551, 0xe4},
	{0x6556, 0x71},
	{0x6557, 0x81},
	{0x655c, 0x71},
	{0x655d, 0x81},
	{0x6560, 0x81},
	{0x6561, 0x83},
	{0x6562, 0xe4},
	{0x6563, 0xe6},
	{0x6564, 0x00},
	{0x6565, 0x00},
	{0x6566, 0x00},
	{0x6567, 0x00},
	{0x656e, 0x77},
	{0x656f, 0x8d},
	{0x6570, 0xe2},
	{0x6571, 0xe8},
	{0x6572, 0x00},
	{0x6573, 0x00},
	{0x6574, 0x00},
	{0x6575, 0x00},
	{0x657c, 0xf1},
	{0x6586, 0xf1},
	{0x658c, 0xf1},
	{0x658e, 0xf1},
	{0x6591, 0xf1},
	{0x65b2, 0xf1},
	{0x65bb, 0xeb},
	{0x65cb, 0xf1},
	{0x65db, 0xea},
	{0x65ea, 0x9a},
	{0x65ec, 0xf5},
	{0x65ed, 0xba},
	{0x65f0, 0x90},
	{0x65f2, 0xf3},
	{0x65f3, 0xd2},
	{0x65f6, 0x40},
	{0x65f7, 0xed},
	{0x65fa, 0xf3},
	{0x6600, 0xf3},
	{0x6606, 0xf5},
	{0x660c, 0xf5},
	{0x661a, 0x5d},
	{0x661b, 0xef},
	{0x6620, 0x90},
	{0x6621, 0xd4},
	{0x374d, 0xfc},
	{0x6624, 0xf3},
	{0x6625, 0x6b},
	{0x662a, 0x00},
	{0x662b, 0x00},
	{0x6630, 0xf7},
	{0x6631, 0xed},
	{0x6636, 0x00},
	{0x6637, 0x00},
	{0x6648, 0xbc},
	{0x6649, 0xd8},
	{0x664e, 0xd6},
	{0x664f, 0xc6},
	{0x6654, 0xd4},
	{0x6655, 0xd0},
	{0x6665, 0x6f},
	{0x6666, 0x6f},
	{0x6667, 0xf1},
	{0x666a, 0x00},
	{0x666b, 0x00},
	{0x666c, 0x42},
	{0x666d, 0x8e},
	{0x666e, 0x8e},
	{0x666f, 0xf3},
	{0x6670, 0x00},
	{0x6671, 0x00},
	{0x66ba, 0x67},
	{0x66bb, 0xf1},
	{0x601d, 0xc4},
	{0x6027, 0x02},
	{0x6029, 0x8d},
	{0x6080, 0x01},
	{0x6081, 0x0a},
	{0x6082, 0x01},
	{0x6083, 0x1d},
	{0x6088, 0x01},
	{0x6089, 0x0a},
	{0x608a, 0x01},
	{0x608b, 0x1d},
	{0x6090, 0x01},
	{0x6091, 0x0a},
	{0x6092, 0x01},
	{0x6093, 0x1d},
	{0x6094, 0x00},
	{0x6095, 0x00},
	{0x6096, 0x00},
	{0x6097, 0x00},
	{0x609c, 0x00},
	{0x609d, 0x00},
	{0x609e, 0x00},
	{0x609f, 0x00},
	{0x60a8, 0x01},
	{0x60a9, 0x0a},
	{0x60aa, 0x01},
	{0x60ab, 0x1d},
	{0x60ac, 0x00},
	{0x60ad, 0x00},
	{0x60ae, 0x00},
	{0x60af, 0x00},
	{0x60ec, 0x01},
	{0x60ed, 0x06},
	{0x60ee, 0x01},
	{0x60ef, 0x3b},
	{0x6108, 0x00},
	{0x610a, 0x00},
	{0x610c, 0x01},
	{0x610d, 0x06},
	{0x610e, 0x01},
	{0x610f, 0x3b},
	{0x616c, 0x01},
	{0x616d, 0x24},
	{0x616e, 0x01},
	{0x616f, 0x3b},
	{0x6178, 0x00},
	{0x617a, 0x00},
	{0x617c, 0x01},
	{0x617d, 0x24},
	{0x617e, 0x01},
	{0x617f, 0x3b},
	{0x618c, 0x01},
	{0x618d, 0x24},
	{0x618e, 0x01},
	{0x618f, 0x3b},
	{0x6198, 0x00},
	{0x619a, 0x00},
	{0x619c, 0x01},
	{0x619d, 0x24},
	{0x619e, 0x01},
	{0x619f, 0x3b},
	{0x61c8, 0x00},
	{0x61cc, 0x01},
	{0x61cd, 0x41},
	{0x61ce, 0x01},
	{0x61cf, 0x05},
	{0x61d0, 0x00},
	{0x61d4, 0x01},
	{0x61d5, 0x41},
	{0x61d6, 0x01},
	{0x61d7, 0x05},
	{0x65da, 0x01},
	{0x65db, 0x00},
	{0x6604, 0x00},
	{0x6605, 0x00},
	{0x6606, 0x00},
	{0x6607, 0x00},
	{0x660a, 0x00},
	{0x660b, 0x00},
	{0x660c, 0x00},
	{0x660d, 0x00},
	{0x6630, 0x00},
	{0x6631, 0x00},
	{0x6636, 0x00},
	{0x6637, 0x00},
	{0x650c, 0x00},
	{0x650d, 0x00},
	{0x6514, 0x00},
	{0x6515, 0x00},
	{0x6592, 0x00},
	{0x6593, 0x00},
	{0x65a2, 0x00},
	{0x65a3, 0x00},
	{0x6674, 0x00},
	{0x6675, 0x00},
	{0x6648, 0x00},
	{0x6649, 0x00},
	{0x664a, 0x00},
	{0x664b, 0x00},
	{0x664e, 0x00},
	{0x664f, 0x00},
	{0x6651, 0x00},
	{0x6652, 0x00},
	{0x6654, 0x00},
	{0x6655, 0x00},
	{0x6656, 0x00},
	{0x6657, 0x00},
	{0x3785, 0x10},
	{0x6532, 0x00},
	{0x6533, 0x00},
	{0x6538, 0x00},
	{0x6539, 0x00},
	{0x653e, 0x00},
	{0x653f, 0x00},
	{0x6544, 0x00},
	{0x6545, 0x00},
	{0x650c, 0x00},
	{0x650d, 0x00},
	{0x6514, 0x00},
	{0x6515, 0x00},
	{0x657c, 0x00},
	{0x657d, 0x00},
	{0x6586, 0x00},
	{0x6587, 0x00},
	{0x658c, 0x00},
	{0x658d, 0x00},
	{0x658e, 0x00},
	{0x658f, 0x00},
	{0x6664, 0x00},
	{0x6665, 0x00},
	{0x6666, 0x00},
	{0x6667, 0x00},
	{0x6668, 0x00},
	{0x6669, 0x00},
	{0x6002, 0x50},
	{0x607d, 0x00},
	{0x607f, 0x00},
	{0x6081, 0x00},
	{0x6083, 0x00},
	{0x6085, 0x00},
	{0x6087, 0x00},
	{0x6089, 0x00},
	{0x608b, 0x00},
	{0x608d, 0x00},
	{0x608f, 0x00},
	{0x6091, 0x00},
	{0x6093, 0x00},
	{0x6095, 0x00},
	{0x6097, 0x00},
	{0x6099, 0x00},
	{0x609b, 0x00},
	{0x609d, 0x00},
	{0x609f, 0x00},
	{0x60a1, 0x00},
	{0x60a3, 0x00},
	{0x0105, 0x03},
	{0x3216, 0x01},
	{0x3218, 0x00},
	{0x0303, 0x04},
	{0x0304, 0x02},
	{0x0305, 0x7c},
	{0x0306, 0x00},
	{0x0307, 0x02},
	{0x030b, 0x01},
	{0x030c, 0x01},
	{0x0317, 0x6a},
	{0x0318, 0x00},
	{0x0328, 0x24},
	{0x032b, 0x03},
	{0x3002, 0x00},
	{0x3012, 0x11},
	{0x3018, 0xf0},
	{0x301a, 0xe0},
	{0x301e, 0xd0},
	{0x301f, 0x61},
	{0x3106, 0x90},
	{0x3500, 0x00},
	{0x3501, 0x00},
	{0x3502, 0x40},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x3540, 0x00},
	{0x3541, 0x00},
	{0x3542, 0x10},
	{0x3548, 0x01},
	{0x3549, 0x00},
	{0x3600, 0x00},
	{0x3602, 0x00},
	{0x3603, 0x00},
	{0x360e, 0x00},
	{0x3614, 0x08},
	{0x361a, 0x08},
	{0x3627, 0x17},
	{0x363b, 0x40},
	{0x3640, 0x40},
	{0x3641, 0x40},
	{0x3642, 0x00},
	{0x3643, 0xb0},
	{0x3644, 0x40},
	{0x3645, 0x40},
	{0x3646, 0x00},
	{0x3647, 0xb0},
	{0x3648, 0x77},
	{0x3649, 0x67},
	{0x364a, 0x77},
	{0x364b, 0x67},
	{0x3660, 0x02},
	{0x3661, 0x07},
	{0x3662, 0x00},
	{0x3663, 0x08},
	{0x3666, 0x00},
	{0x366f, 0x00},
	{0x3670, 0x08},
	{0x3679, 0x00},
	{0x367c, 0x08},
	{0x3688, 0x12},
	{0x3689, 0xff},
	{0x368a, 0x00},
	{0x368b, 0x43},
	{0x3b1e, 0x01},
	{0x3b20, 0xff},
	{0x3b23, 0x00},
	{0x3b24, 0x00},
	{0x3b29, 0x00},
	{0x3b2a, 0x00},
	{0x3b2b, 0x00},
	{0x3b2c, 0x00},
	{0x3b2f, 0x47},
	{0x3d8c, 0x77},
	{0x3d8d, 0xb0},
	{0x4000, 0x79},
	{0x400a, 0x08},
	{0x400b, 0xff},
	{0x4010, 0x00},
	{0x4011, 0x00},
	{0x4028, 0x5f},
	{0x402c, 0x08},
	{0x402d, 0x00},
	{0x4032, 0x8f},
	{0x4288, 0xcf},
	{0x4289, 0x01},
	{0x42d1, 0x00},
	{0x4388, 0x00},
	{0x438b, 0x0f},
	{0x438c, 0xff},
	{0x438d, 0x00},
	{0x438e, 0x00},
	{0x4390, 0xff},
	{0x4391, 0xff},
	{0x4392, 0x00},
	{0x4393, 0x00},
	{0x4394, 0x04},
	{0x4800, 0x04},
	{0x4803, 0x10},
	{0x481f, 0x30},
	{0x4829, 0x80},
	{0x4833, 0x27},
	{0x4837, 0x0d},
	{0x4843, 0x00},
	{0x4844, 0x00},
	{0x4845, 0x00},
	{0x484b, 0x07},
	{0x484c, 0x00},
	{0x484f, 0x00},
	{0x4850, 0x19},
	{0x4d00, 0x4c},
	{0x4d01, 0x88},
	{0x4d02, 0xba},
	{0x4d03, 0x65},
	{0x4d04, 0x47},
	{0x4d05, 0x0a},
	{0x4d07, 0x30},
	{0x4d09, 0x0a},
	{0x5000, 0x5f},
	{0x5006, 0x01},
	{0x5074, 0x00},
	{0x507a, 0x3f},
	{0x507b, 0xff},
	{0x5080, 0x00},
	{0x5081, 0x01},
	{0x50c0, 0x00},
	{0x50c1, 0x01},
	{0x5200, 0x00},
	{0x5201, 0x10},
	{0x5202, 0x06},
	{0x5203, 0x9f},
	{0x5240, 0x00},
	{0x5241, 0x10},
	{0x5242, 0x06},
	{0x5243, 0x9f},
	{0x53a3, 0x01},
	{0x5423, 0x01},
	{0x5780, 0x00},
	{0x5781, 0x58},
	{0x5800, 0x29},
	{0x5801, 0x0c},
	{0x5802, 0x06},
	{0x5803, 0x06},
	{0x5804, 0x07},
	{0x5805, 0x08},
	{0x5806, 0x09},
	{0x5807, 0x0a},
	{0x5808, 0x0b},
	{0x5809, 0x0c},
	{0x580a, 0x0d},
	{0x5826, 0x00},
	{0x5827, 0x40},
	{0x582a, 0x00},
	{0x582b, 0x40},
	{0x582e, 0x00},
	{0x582f, 0x80},
	{0x5832, 0x01},
	{0x5833, 0x00},
	{0x5836, 0x01},
	{0x5837, 0x00},
	{0x583a, 0x01},
	{0x583b, 0x00},
	{0x583e, 0x02},
	{0x583f, 0x00},
	{0x5842, 0x04},
	{0x5843, 0x00},
	{0x5846, 0x06},
	{0x5847, 0x00},
	{0x58ab, 0x0f},
	{0x3700, 0x84},
	{0x3714, 0x02},
	{0x371c, 0x00},
	{0x371d, 0x00},
	{0x372f, 0x00},
	{0x3744, 0x08},
	{0x3745, 0x08},
	{0x3746, 0xff},
	{0x3747, 0x44},
	{0x3748, 0x44},
	{0x374b, 0x00},
	{0x374d, 0xf8},
	{0x374e, 0x00},
	{0x3775, 0xbc},
	{0x3782, 0x80},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x01},
	{0x3803, 0xe4},
	{0x3804, 0x09},
	{0x3805, 0x9f},
	{0x3806, 0x06},
	{0x3807, 0x2b},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x0c},
	{0x380d, 0x58},
	{0x380e, 0x08},
	{0x380f, 0x30},
	{0x3810, 0x00},
	{0x3811, 0xd0},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x80},
	{0x3821, 0x04},
	{0x3822, 0x08},
	{0x3823, 0x00},
	{0x3828, 0x00},
	{0x382c, 0xff},
	{0x382e, 0x01},
	{0x3856, 0x00},
	{0x3857, 0x00},
	{0x385a, 0x04},
	{0x385b, 0x40},
	{0x385c, 0x00},
	{0x385d, 0x00},
	{0x3869, 0x00},
	{0x386b, 0x10},
	{0x3873, 0x02},
	{0x3875, 0x0c},
	{0x3876, 0x0c},
	{0x3878, 0x04},
	{0x3879, 0x00},
	{0x387b, 0x00},
	{0x387c, 0x00},
	{0x387e, 0x81},
	{0x388a, 0x00},
	{0x388b, 0x1c},
	{0x388c, 0x00},
	{0x390b, 0x33},
	{0x3918, 0x00},
	{0x450a, 0x12},
	{0x450b, 0xf8},
	{0x4510, 0x02},
	{0x4512, 0x00},
	{0x4513, 0x00},
	{0x4514, 0x10},
	{0x4515, 0x1f},
	{0x4516, 0x14},
	{0x4517, 0x01},
	{0x4e00, 0x00},
	{0x4e23, 0x55},
	{0x4e24, 0x11},
	{0x4e25, 0x54},
	{0x4e26, 0x57},
	{0x4e27, 0x00},
	{0x4e28, 0x08},
	{0x4e2a, 0x55},
	{0x4e2b, 0x55},
	{0x4e2c, 0x15},
	{0x4e2d, 0x00},
	{0x4e2e, 0x00},
	{0x60ce, 0x01},
	{0x60cf, 0x00},
	{0x64fe, 0x00},
	{0x64ff, 0x00},
	{0x6530, 0x00},
	{0x6531, 0x00},
	{0x6536, 0x00},
	{0x6537, 0x00},
	{0x653c, 0x00},
	{0x653d, 0x00},
	{0x6542, 0x00},
	{0x6543, 0x00},
	{0x6548, 0x00},
	{0x6549, 0x00},
	{0x654e, 0x00},
	{0x654f, 0x00},
	{0x6554, 0x00},
	{0x6555, 0x00},
	{0x655a, 0x00},
	{0x655b, 0x00},
	{0x6586, 0x00},
	{0x6587, 0x00},
	{0x665a, 0x00},
	{0x665b, 0x00},
	{0x6682, 0x01},
	{0x6683, 0x00},
	{0x6684, 0x00},
	{0x6685, 0x00},
	{0x6686, 0x00},
	{0x6687, 0x00},
	{0x6688, 0x00},
	{0x6689, 0x00},
	{0x668a, 0x00},
	{0x668b, 0x00},
	{0x668c, 0x00},
	{0x668d, 0x00},
	{0x3822, 0x0c},
	{0x3873, 0x04},
	{0x3875, 0x0c},
	{0x3876, 0x0c},
	{0x387b, 0x00},
	{0x387c, 0x00},
	{0x4008, 0x00},
	{0x4009, 0x17},
	{0x4050, 0x04},
	{0x4051, 0x0f},
	{0x4032, 0x9f},
	{0x4052, 0x00},
	{0x4053, 0x43},
	{0x4054, 0x00},
	{0x4055, 0x43},
	{0x4056, 0x00},
	{0x4057, 0x43},
	{0x4058, 0x00},
	{0x4059, 0x43},
	{0x4034, 0x00},
	{0x4035, 0x80},
	{0x4036, 0x00},
	{0x4037, 0x80},
	{0x4038, 0x00},
	{0x4039, 0x80},
	{0x403a, 0x00},
	{0x403b, 0x80},
	{0x6508, 0x00},
	{0x6509, 0x00},
	{0x6518, 0x00},
	{0x6519, 0x00},
	{0x6524, 0x00},
	{0x6525, 0x00},
	{0x6548, 0x00},
	{0x6549, 0x00},
	{0x654a, 0x00},
	{0x654b, 0x00},
	{0x6554, 0x00},
	{0x6555, 0x00},
	{0x6556, 0x00},
	{0x6557, 0x00},
	{0x60e8, 0x00},
	{0x60e9, 0x00},
	{0x60ea, 0x00},
	{0x60eb, 0x00},
	{0x60ec, 0x00},
	{0x60ed, 0x00},
	{0x60ee, 0x00},
	{0x60ef, 0x00},
	{0x6168, 0x00},
	{0x6169, 0x00},
	{0x616a, 0x00},
	{0x616b, 0x00},
	{0x616c, 0x00},
	{0x616d, 0x00},
	{0x616e, 0x00},
	{0x616f, 0x00},
	{0x6188, 0x00},
	{0x6189, 0x00},
	{0x618a, 0x00},
	{0x618b, 0x00},
	{0x618c, 0x00},
	{0x618d, 0x00},
	{0x618e, 0x00},
	{0x618f, 0x00},
	{0x388a, 0x00},
	{0x380e, 0x08},
	{0x380f, 0x68},
	{0x3500, 0x00},
	{0x3501, 0x00},
	{0x3502, 0x3b},
	{0x3540, 0x00},
	{0x3541, 0x00},
	{0x3542, 0x3b},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x3506, 0x01},
	{0x3507, 0xf4},
	{0x64f0, 0x00},
	{0x6510, 0x2f},
	{0x66ba, 0x00},
	{0x666c, 0x2f},
	{0x65f6, 0x2f},
	{0x6506, 0x2f},
	{0x65ba, 0x2f},
	{0x65ca, 0x2f},
	{0x65b3, 0x30},
	{0x6618, 0x00},
	{0x6619, 0x00},
	{0x661e, 0x00},
	{0x661f, 0x00},
	{0x65f8, 0x00},
	{0x65f9, 0x00},
	{0x65fe, 0x00},
	{0x65ff, 0x00},
	{0x65ea, 0x00},
	{0x65eb, 0x00},
	{0x65ec, 0x00},
	{0x65ed, 0x00},
	{0x661a, 0x58},
	{0x6625, 0x6b},
	{0x6403, 0x00},
	{0x6405, 0x51},
	{0x6427, 0x00},
	{0x6429, 0x00},
	{0x6436, 0x00},
	{0x6438, 0x18},
	{0x6439, 0x00},
	{0x643b, 0x1c},
	{0x6448, 0x04},
	{0x644a, 0x11},
	{0x644b, 0x00},
	{0x644d, 0x18},
	{0x3636, 0x00},
	{0x362c, 0x8c},
	{0x6451, 0x00},
	{0x6453, 0x00},
	{0x6454, 0x00},
	{0x6456, 0x01},
	{0x6457, 0x00},
	{0x6459, 0x00},
	{0x645a, 0x00},
	{0x645c, 0x00},
	{0x645d, 0x00},
	{0x645f, 0x40},
	{0x374d, 0xfc},
	{0x382c, 0x29},
	{0x6472, 0x04},
	{0x6474, 0x5d},
	{0x6478, 0x00},
	{0x647a, 0x80},
	{0x6496, 0x00},
	{0x6498, 0x80},
	{0x3747, 0xff},
	{0x3748, 0xff},
	{0x3744, 0x2f},
	{0x3745, 0x10},
	{0x362f, 0xc9},
	{0x3630, 0xcb},
	{0x3611, 0x00},
	{0x371c, 0x6e},
	{0x371d, 0x00},
	{0x6424, 0x00},
	{0x6426, 0x00},
	{0x6400, 0x00},
	{0x6402, 0x00},
	{0x6406, 0x00},
	{0x6408, 0x00},
	{0x6409, 0x00},
	{0x640b, 0x00},
	{0x6412, 0x00},
	{0x6414, 0x00},
	{0x6415, 0x00},
	{0x6417, 0x00},
	{0x6418, 0x00},
	{0x641a, 0x00},
	{0x641b, 0x00},
	{0x641d, 0x00},
	{0x641e, 0x00},
	{0x6420, 0x00},
	{0x6421, 0x00},
	{0x6423, 0x00},
	{0x642a, 0x00},
	{0x642c, 0x00},
	{0x642d, 0x00},
	{0x642f, 0x00},
	{0x6430, 0x00},
	{0x6432, 0x00},
	{0x6433, 0x00},
	{0x6435, 0x00},
	{0x643c, 0x00},
	{0x643e, 0x00},
	{0x643f, 0x00},
	{0x6441, 0x00},
	{0x6442, 0x00},
	{0x6444, 0x00},
	{0x6445, 0x00},
	{0x6447, 0x00},
	{0x644e, 0x00},
	{0x6450, 0x00},
	{0x6460, 0x00},
	{0x6462, 0x00},
	{0x6463, 0x00},
	{0x6465, 0x00},
	{0x6466, 0x00},
	{0x6468, 0x00},
	{0x6469, 0x00},
	{0x646b, 0x00},
	{0x646c, 0x00},
	{0x646e, 0x00},
	{0x646f, 0x00},
	{0x6471, 0x00},
	{0x6475, 0x00},
	{0x6477, 0x00},
	{0x647b, 0x00},
	{0x647d, 0x00},
	{0x647e, 0x00},
	{0x6480, 0x00},
	{0x6481, 0x00},
	{0x6483, 0x00},
	{0x6484, 0x00},
	{0x6486, 0x00},
	{0x6487, 0x00},
	{0x6489, 0x00},
	{0x648a, 0x00},
	{0x648c, 0x00},
	{0x648d, 0x00},
	{0x648f, 0x00},
	{0x6490, 0x00},
	{0x6492, 0x00},
	{0x6493, 0x00},
	{0x6495, 0x00},
	{0x640c, 0x00},
	{0x640e, 0xfc},
	{0x640f, 0x04},
	{0x6411, 0x6b},
	{0x6412, 0x04},
	{0x6414, 0x60},
	{0x6415, 0x04},
	{0x6417, 0x25},
	{0x641b, 0x00},
	{0x641d, 0x30},
	{0x65b3, 0x2f},
	{0x665a, 0x2f},
	{0x665b, 0x32},
	{0x654e, 0x2f},
	{0x654f, 0x35},
	{0x655a, 0x2f},
	{0x655b, 0x35},
	{0x3208, 0x0a},
	{0x3739, 0x02},
	{0x373b, 0x1e},
	{0x3734, 0x42},
	{0x65ca, 0xf1},
	{0x65cb, 0x2f},
	{0x6590, 0xf1},
	{0x6591, 0x25},
	{0x60e6, 0x3b},
	{0x60e7, 0x07},
	{0x6169, 0x01},
	{0x6179, 0x3b},
	{0x6189, 0x01},
	{0x6199, 0x3b},
	{0x616b, 0x00},
	{0x617b, 0x24},
	{0x618b, 0x00},
	{0x619b, 0x24},
	{0x616d, 0x01},
	{0x617d, 0x3b},
	{0x618d, 0x01},
	{0x619d, 0x3b},
	{0x616f, 0x00},
	{0x617f, 0x24},
	{0x618f, 0x00},
	{0x619f, 0x24},
	{0x3208, 0x1a},
	{0x3208, 0x0b},
	{0x3739, 0x00},
	{0x373b, 0x00},
	{0x3734, 0x00},
	{0x65ca, 0x2f},
	{0x65cb, 0xf1},
	{0x6590, 0x25},
	{0x6591, 0xf1},
	{0x60e6, 0x07},
	{0x60e7, 0x3b},
	{0x6169, 0x00},
	{0x6179, 0x24},
	{0x6189, 0x00},
	{0x6199, 0x24},
	{0x616b, 0x01},
	{0x617b, 0x3b},
	{0x618b, 0x01},
	{0x619b, 0x3b},
	{0x616d, 0x00},
	{0x617d, 0x24},
	{0x618d, 0x00},
	{0x619d, 0x24},
	{0x616f, 0x01},
	{0x617f, 0x3b},
	{0x618f, 0x01},
	{0x619f, 0x3b},
	{0x3208, 0x1b},
	{0x3239, 0x01},
	{0x323a, 0x01},
	{0x3840, 0x08},
	{0x3854, 0x05},
	{0x3855, 0xda},
	{0x3853, 0x03},
	{0x322f, 0x80},
	{0x5a02, 0x04},
	{0x5a03, 0x00}
};

/* Mode configs */
static const struct og05c10_mode supported_modes_12bit[] = {
	{
		.width = 1920,
		.height = 1080,
		.line_length_pix = 0x1866,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 1920,
			.height = 1080,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 3000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
	}
};

static const struct og05c10_mode supported_modes_10bit[] = {

};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

static const char * const og05c10_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

/* regulator supplies */
static const char * const og05c10_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define OG05C10_NUM_SUPPLIES ARRAY_SIZE(og05c10_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define OG05C10_XCLR_MIN_DELAY_US	8000
#define OG05C10_XCLR_DELAY_RANGE_US	1000

struct og05c10_compatible_data {
	unsigned int chip_id;
	struct og05c10_reg_list extra_regs;
};

struct og05c10 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OG05C10_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct og05c10_mode *mode;

	/* Trigger mode */
	int trigger_mode_of;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK */
	unsigned int long_exp_shift;

	/* Any extra information related to different compatible sensors */
	const struct og05c10_compatible_data *compatible_data;

	int again;
	uint32_t integration_time;
};

static inline struct og05c10 *to_og05c10(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct og05c10, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct og05c10_mode **mode_list,
				  unsigned int *num_modes)
{
	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;
		*num_modes = ARRAY_SIZE(supported_modes_12bit);
		break;
	/* 10-bit */
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*mode_list = supported_modes_10bit;
		*num_modes = ARRAY_SIZE(supported_modes_10bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
static int og05c10_read_reg(struct og05c10 *og05c10, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int og05c10_write_reg(struct og05c10 *og05c10, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int og05c10_write_regs(struct og05c10 *og05c10,
			     const struct og05c10_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		if (regs[i].address == OG05C10_DELAY_REG){
			mdelay(regs[i].val);
			continue;
		}

		ret = og05c10_write_reg(og05c10, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 og05c10_get_format_code(struct og05c10 *og05c10, u32 code)
{
	unsigned int i;

	for(i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
		i = 0;

	return codes[i];
}

static void og05c10_set_default_format(struct og05c10 *og05c10)
{
	/* Set default mode to max resolution */
	og05c10->mode = &supported_modes_12bit[0];
	og05c10->fmt_code = MEDIA_BUS_FMT_SBGGR12_1X12;
}

static int og05c10_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct og05c10 *og05c10 = to_og05c10(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_state_get_format(fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&og05c10->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes_12bit[0].width;
	try_fmt_img->height = supported_modes_12bit[0].height;
	try_fmt_img->code = og05c10_get_format_code(og05c10,
						   MEDIA_BUS_FMT_SBGGR12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = OG05C10_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = OG05C10_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_state_get_crop(fh->state, IMAGE_PAD);
	try_crop->left = OG05C10_PIXEL_ARRAY_LEFT;
	try_crop->top = OG05C10_PIXEL_ARRAY_TOP;
	try_crop->width = OG05C10_PIXEL_ARRAY_WIDTH;
	try_crop->height = OG05C10_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&og05c10->mutex);

	return 0;
}

static void og05c10_adjust_exposure_range(struct og05c10 *og05c10)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = og05c10->mode->height + og05c10->vblank->val -
		       4;
	exposure_def = min(exposure_max, OG05C10_EXPOSURE_DEFAULT);
	__v4l2_ctrl_modify_range(og05c10->exposure, og05c10->exposure->minimum,
				 exposure_max, og05c10->exposure->step,
				 exposure_def);
}

int32_t cam_set_exposure(struct og05c10 *og05c10, uint64_t exp)
{
	int ret = 0;

	if(exp < OG05C10_EXP_MIN)
		exp = OG05C10_EXP_MIN;
	if(exp > OG05C10_EXPOSURE_MAX)
		exp = OG05C10_EXPOSURE_MAX;

	ret = og05c10_write_reg(og05c10, OG05C10_REG_EXP_HI, OG05C10_REG_VALUE_08BIT, (exp >> 16) & 0xff);
	if (ret)
		return ret;

	ret = og05c10_write_reg(og05c10, OG05C10_REG_EXP_MID, OG05C10_REG_VALUE_08BIT, (exp >> 8) & 0xff);
	if (ret)
		return ret;

	ret = og05c10_write_reg(og05c10, OG05C10_REG_EXP_LO, OG05C10_REG_VALUE_08BIT, exp & 0xff);

	return ret;
}

int32_t cam_set_gain(struct og05c10 *og05c10, uint16_t gain)
{
	int ret = 0;

	ret = og05c10_write_reg(og05c10, OG05C10_REG_GAIN_LO,
				OG05C10_REG_VALUE_08BIT, (gain >> 4) & 0x3f);

	if (ret)
		return ret;

	ret = og05c10_write_reg(og05c10, OG05C10_REG_GAIN_HI,
				       OG05C10_REG_VALUE_08BIT, gain & 0xf);

	return ret;
}

static int32_t set_analogue_gain( struct og05c10 *og05c10, uint32_t gain )
{
	u32 i = gain >> 18;
	u32 f = gain & 0x3FFFF;
	u64 f_sq;
	u64 approx_f;
	u32 linear_q18;
	f_sq = (u64)f * f;

	approx_f = (1ULL << 18) + (((u64)f * 171804) >> 18) + ((f_sq * 90338) >> 36);

	if (i >= 5)
		linear_q18  = 0x01FF;
	else
		linear_q18 = ((u32)(approx_f << i)) >> 14;

	return cam_set_gain (og05c10, (uint16_t)linear_q18);
}

static int og05c10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og05c10 *og05c10 =
		container_of(ctrl->handler, struct og05c10, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	int ret = 0;
	int exposure_max, exposure_def;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK) {

		/* Update max exposure while meeting expected vblanking */
		exposure_max = og05c10->mode->height + ctrl->val - 4;
		exposure_def = min(exposure_max, OG05C10_EXPOSURE_DEFAULT);
		__v4l2_ctrl_modify_range(og05c10->exposure, og05c10->exposure->minimum,
				 exposure_max, og05c10->exposure->step,
				 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = set_analogue_gain(og05c10, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cam_set_exposure(og05c10, ctrl->val);
		break;
	default:
		dev_dbg(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops og05c10_ctrl_ops = {
	.s_ctrl = og05c10_set_ctrl,
};

static int og05c10_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct og05c10 *og05c10 = to_og05c10(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = og05c10_get_format_code(og05c10,
						    codes[code->index]);
		if(code->code == 0)
			return -EINVAL;
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int og05c10_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct og05c10 *og05c10 = to_og05c10(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct og05c10_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != og05c10_get_format_code(og05c10, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = OG05C10_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = OG05C10_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void og05c10_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void og05c10_update_image_pad_format(struct og05c10 *og05c10,
					   const struct og05c10_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	og05c10_reset_colorspace(&fmt->format);
}

static void og05c10_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = OG05C10_EMBEDDED_LINE_WIDTH;
	fmt->format.height = OG05C10_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int og05c10_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct og05c10 *og05c10 = to_og05c10(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&og05c10->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				og05c10_get_format_code(og05c10, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			og05c10_update_image_pad_format(og05c10, og05c10->mode,
						       fmt);
			fmt->format.code =
			       og05c10_get_format_code(og05c10, og05c10->fmt_code);
		} else {
			og05c10_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&og05c10->mutex);
	return 0;
}

static
unsigned int og05c10_get_frame_length(const struct og05c10_mode *mode,
				     const struct v4l2_fract *timeperframe)
{
	u64 frame_length;

	frame_length = (u64)timeperframe->numerator * OG05C10_PIXEL_RATE;
	do_div(frame_length,
	       (u64)timeperframe->denominator * mode->line_length_pix);

	if (WARN_ON(frame_length > OG05C10_FRAME_LENGTH_MAX))
		frame_length = OG05C10_FRAME_LENGTH_MAX;

	return max_t(unsigned int, frame_length, mode->height);
}

static void og05c10_set_framing_limits(struct og05c10 *og05c10)
{
	unsigned int frm_length_min, frm_length_default, hblank_min;
	const struct og05c10_mode *mode = og05c10->mode;

	frm_length_min = og05c10_get_frame_length(mode, &mode->timeperframe_min);
	frm_length_default =
		     og05c10_get_frame_length(mode, &mode->timeperframe_default);

	/* Default to no long exposure multiplier. */
	og05c10->long_exp_shift = 0;

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(og05c10->vblank, frm_length_min - mode->height,
				 ((1 << OG05C10_LONG_EXP_SHIFT_MAX) *
					OG05C10_FRAME_LENGTH_MAX) - mode->height,
				 1, frm_length_default - mode->height);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(og05c10->vblank, frm_length_default - mode->height);

	hblank_min = mode->line_length_pix - mode->width;
	__v4l2_ctrl_modify_range(og05c10->hblank, hblank_min,
				 OG05C10_LINE_LENGTH_MAX, 1, hblank_min);
	__v4l2_ctrl_s_ctrl(og05c10->hblank, hblank_min);
}

static int og05c10_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct og05c10_mode *mode;
	struct og05c10 *og05c10 = to_og05c10(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&og05c10->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct og05c10_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = og05c10_get_format_code(og05c10,
							  fmt->format.code);

		get_mode_table(fmt->format.code, &mode_list, &num_modes);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		og05c10_update_image_pad_format(og05c10, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		} else {
			og05c10->mode = mode;
			og05c10->fmt_code = fmt->format.code;
			og05c10_set_framing_limits(og05c10);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			og05c10_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&og05c10->mutex);

	return 0;
}

static const struct v4l2_rect *
__og05c10_get_pad_crop(struct og05c10 *og05c10,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &og05c10->mode->crop;
	}

	return NULL;
}

static int og05c10_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct og05c10 *og05c10 = to_og05c10(sd);

		mutex_lock(&og05c10->mutex);
		sel->r = *__og05c10_get_pad_crop(og05c10, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&og05c10->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OG05C10_NATIVE_WIDTH;
		sel->r.height = OG05C10_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = OG05C10_PIXEL_ARRAY_LEFT;
		sel->r.top = OG05C10_PIXEL_ARRAY_TOP;
		sel->r.width = OG05C10_PIXEL_ARRAY_WIDTH;
		sel->r.height = OG05C10_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int og05c10_start_streaming(struct og05c10 *og05c10)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	const struct og05c10_reg_list *reg_list;
	int ret;


	/* Apply default values of current mode */
	reg_list = &og05c10->mode->reg_list;
	ret = og05c10_write_regs(og05c10, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(og05c10->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */

	return og05c10_write_reg(og05c10, OG05C10_REG_MODE_SELECT,
				OG05C10_REG_VALUE_08BIT, OG05C10_MODE_START_STREAMING);
}

/* Stop streaming */
static void og05c10_stop_streaming(struct og05c10 *og05c10)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	int ret;

	/* set stream off register */

	/* Stop driving XVS out (there is still a weak pull-up) */
	og05c10_write_reg(og05c10, OG05C10_REG_MODE_SELECT,
			 OG05C10_REG_VALUE_08BIT, OG05C10_MODE_STOP_STREAMING);
}

static int og05c10_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct og05c10 *og05c10 = to_og05c10(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&og05c10->mutex);
	if (og05c10->streaming == enable) {
		mutex_unlock(&og05c10->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = og05c10_start_streaming(og05c10);
		if (ret)
			goto err_rpm_put;
	} else {
		og05c10_stop_streaming(og05c10);
		pm_runtime_put(&client->dev);
	}

	og05c10->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(og05c10->vflip, enable);
	__v4l2_ctrl_grab(og05c10->hflip, enable);

	mutex_unlock(&og05c10->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&og05c10->mutex);

	return ret;
}

/* Power/clock management functions */
static int og05c10_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og05c10 *og05c10 = to_og05c10(sd);
	int ret;

	ret = regulator_bulk_enable(OG05C10_NUM_SUPPLIES,
				    og05c10->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	if(og05c10->reset_gpio != NULL){
		gpiod_set_value_cansleep(og05c10->reset_gpio, 0);
		mdelay(100);
		gpiod_set_value_cansleep(og05c10->reset_gpio, 1);
	}

	usleep_range(OG05C10_XCLR_MIN_DELAY_US,
		     OG05C10_XCLR_MIN_DELAY_US + OG05C10_XCLR_DELAY_RANGE_US);

	return 0;

}

static int og05c10_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og05c10 *og05c10 = to_og05c10(sd);

	if(og05c10->reset_gpio != NULL)
		gpiod_set_value_cansleep(og05c10->reset_gpio, 0);
	regulator_bulk_disable(OG05C10_NUM_SUPPLIES, og05c10->supplies);

	/* Force reprogramming of the common registers when powered up again. */
	og05c10->common_regs_written = false;

	return 0;
}

static int __maybe_unused og05c10_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og05c10 *og05c10 = to_og05c10(sd);

	if (og05c10->streaming)
		og05c10_stop_streaming(og05c10);

	return 0;
}

static int __maybe_unused og05c10_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og05c10 *og05c10 = to_og05c10(sd);
	int ret;

	if (og05c10->streaming) {
		ret = og05c10_start_streaming(og05c10);
		if (ret)
			goto error;
	}

	return 0;

error:
	og05c10_stop_streaming(og05c10);
	og05c10->streaming = 0;
	return ret;
}

static int og05c10_get_regulators(struct og05c10 *og05c10)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	unsigned int i;

	for (i = 0; i < OG05C10_NUM_SUPPLIES; i++)
		og05c10->supplies[i].supply = og05c10_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       OG05C10_NUM_SUPPLIES,
				       og05c10->supplies);
}

static const struct v4l2_subdev_core_ops og05c10_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops og05c10_video_ops = {
	.s_stream = og05c10_set_stream,
};

static const struct v4l2_subdev_pad_ops og05c10_pad_ops = {
	.enum_mbus_code = og05c10_enum_mbus_code,
	.get_fmt = og05c10_get_pad_format,
	.set_fmt = og05c10_set_pad_format,
	.get_selection = og05c10_get_selection,
	.enum_frame_size = og05c10_enum_frame_size,
};

static const struct v4l2_subdev_ops og05c10_subdev_ops = {
	.core = &og05c10_core_ops,
	.video = &og05c10_video_ops,
	.pad = &og05c10_pad_ops,
};

static const struct v4l2_subdev_internal_ops og05c10_internal_ops = {
	.open = og05c10_open,
};

/* Initialize control handlers */
static int og05c10_init_controls(struct og05c10 *og05c10)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &og05c10->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&og05c10->mutex);
	ctrl_hdlr->lock = &og05c10->mutex;

	/* By default, PIXEL_RATE is read only */
	og05c10->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       OG05C10_PIXEL_RATE,
					       OG05C10_PIXEL_RATE, 1,
					       OG05C10_PIXEL_RATE);
	if (og05c10->pixel_rate)
		og05c10->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* LINK_FREQ is also read only */
	og05c10->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &og05c10_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(og05c10_link_freq_menu) - 1, 0,
				       og05c10_link_freq_menu);
	if (og05c10->link_freq)
		og05c10->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the og05c10_set_framing_limits() call below.
	 */
	og05c10->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					   V4L2_CID_VBLANK, OG05C10_VBLANK_MIN, OG05C10_VTS_MAX - og05c10->mode->height, 1, OG05C10_VTS - - og05c10->mode->height);
	og05c10->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					   V4L2_CID_HBLANK, OG05C10_HTS - og05c10->mode->width, OG05C10_HTS_MAX - og05c10->mode->width, 1, OG05C10_HTS - og05c10->mode->width);

	og05c10->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     OG05C10_EXPOSURE_MIN,
					     OG05C10_EXPOSURE_MAX,
					     OG05C10_EXPOSURE_STEP,
					     OG05C10_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OG05C10_AGAIN_MIN, OG05C10_AGAIN_MAX,
			  OG05C10_AGAIN_STEP, OG05C10_AGAIN_DEFAULT);

	og05c10->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (og05c10->hflip)
		og05c10->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	og05c10->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &og05c10_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (og05c10->vflip)
		og05c10->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &og05c10_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(og05c10_test_pattern_menu) - 1,
				     0, 0, og05c10_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &og05c10_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	og05c10->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&og05c10->mutex);

	return ret;
}

static void og05c10_free_controls(struct og05c10 *og05c10)
{
	v4l2_ctrl_handler_free(og05c10->sd.ctrl_handler);
	mutex_destroy(&og05c10->mutex);
}

static int og05c10_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != OG05C10_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct og05c10_compatible_data og05c10_compatible = {
	.chip_id = 0x477,
	.extra_regs = {
		.num_of_regs = 0,
		.regs = NULL
	}
};

static const struct of_device_id og05c10_dt_ids[] = {
	{ .compatible = "OV,og05c10", .data = &og05c10_compatible },
	{ /* sentinel */ }
};


static int og05c10_identify_module(struct og05c10 *og05c10)
{
	int ret;
	u32 val = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&og05c10->sd);

	ret = og05c10_read_reg(og05c10, OG05C10_REG_CHIP_ID,
			OG05C10_REG_VALUE_24BIT, &val);
	if (ret)
		return ret;

	if (val != OG05C10_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x",
				OG05C10_CHIP_ID, val);
		return -ENXIO;
	}

	dev_dbg(&client->dev, "chip id : %x", val);

	return 0;
}

static int og05c10_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct og05c10 *og05c10;
	const struct of_device_id *match;
	int ret;
	u32 tm_of;

	og05c10 = devm_kzalloc(&client->dev, sizeof(*og05c10), GFP_KERNEL);
	if (!og05c10)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&og05c10->sd, client, &og05c10_subdev_ops);

	match = &og05c10_dt_ids[0];
	if (!match)
		return -ENODEV;
	og05c10->compatible_data =
		(const struct og05c10_compatible_data *)match->data;

	/* Check the hardware configuration in device tree */
	if (og05c10_check_hwcfg(dev))
		return -EINVAL;

	/* Default the trigger mode from OF to -1, which means invalid */
	ret = of_property_read_u32(dev->of_node, "trigger-mode", &tm_of);
	og05c10->trigger_mode_of = (ret == 0) ? tm_of : -1;

	/* Get system clock (xclk) */
	og05c10->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(og05c10->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(og05c10->xclk);
	}

	og05c10->xclk_freq = clk_get_rate(og05c10->xclk);
	if (og05c10->xclk_freq != OG05C10_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			og05c10->xclk_freq);
		return -EINVAL;
	}

	ret = og05c10_get_regulators(og05c10);
	if (ret) {
		dev_err(dev, "failed to get regulators  %d\n", ret);
		return ret;
	}

	/* Request optional enable pin */
	og05c10->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for og05c10_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = og05c10_power_on(dev);
	if (ret)
		return ret;

	/* Initialize default format */
	og05c10_set_default_format(og05c10);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = og05c10_identify_module(og05c10);
	if (ret)
		return ret;

	/* This needs the pm runtime to be registered. */
	ret = og05c10_init_controls(og05c10);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	og05c10->sd.internal_ops = &og05c10_internal_ops;
	og05c10->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	og05c10->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	og05c10->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	og05c10->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&og05c10->sd.entity, NUM_PADS, og05c10->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&og05c10->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	dev_info(dev, "Probe is successful\n");
	return 0;

error_media_entity:
	media_entity_cleanup(&og05c10->sd.entity);

error_handler_free:
	og05c10_free_controls(og05c10);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	og05c10_power_off(&client->dev);

	return ret;
}

static void og05c10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og05c10 *og05c10 = to_og05c10(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	og05c10_free_controls(og05c10);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		og05c10_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

MODULE_DEVICE_TABLE(of, og05c10_dt_ids);

static const struct dev_pm_ops og05c10_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(og05c10_suspend, og05c10_resume)
	SET_RUNTIME_PM_OPS(og05c10_power_off, og05c10_power_on, NULL)
};

static struct i2c_driver og05c10_i2c_driver = {
	.driver = {
		.name = "og05c10",
		.of_match_table	= og05c10_dt_ids,
		.pm = &og05c10_pm_ops,
	},
	.probe = og05c10_probe,
	.remove = og05c10_remove,
};

module_i2c_driver(og05c10_i2c_driver);

MODULE_AUTHOR("Naushir Patuck <naush@raspberrypi.com>");
MODULE_DESCRIPTION("Sony OG05C10 sensor driver");
MODULE_LICENSE("GPL v2");
