// SPDX-License-Identifier: GPL-2.0
/*
* A V4L2 driver for OnSemi AR0234 cameras.
* Copyright (C) 2021, Raspberry Pi (Trading) Ltd
*
* Based on Sony imx219 camera driver
* Copyright (C) 2019, Raspberry Pi (Trading) Ltd
*
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/mipi-csi2.h>

/* Chip ID */
#define AR0234_REG_CHIP_ID		0x3000
#define AR0234_CHIP_ID			0x0a56
#define AR0234_CHIP_ID_MONO		0x1a56

#define AR0234_REG_RESET          0x301A
/* Bit 0 is reset */
/* Bit 2 is stream on/off */
#define AR0234_REG_RESET_RESET       0x00D9
#define AR0234_REG_RESET_STREAM_OFF  0x2058
#define AR0234_REG_RESET_STREAM_ON   0x205C

/* External clock frequency is 24.0M */
#define AR0234_XCLK_FREQ		27000000ULL

/* Pixel rate is fixed at 180M for all the modes */
#define AR0234_PIXEL_RATE		550000000ULL

#define AR0234_DEFAULT_LINK_FREQ	270000000ULL

#define LINE_LENGTH_PCK			0x300C

/* V_TIMING internal */
#define AR0234_REG_VTS			0x300a
#define AR0234_VTS_30FPS		0x04c4
#define AR0234_VTS_60FPS		0x0dc6 //fixme
#define AR0234_VTS_MAX			0xffff

#define AR0234_VBLANK_MIN		16

/*Frame Length Line*/
#define AR0234_FLL_MIN			0x08a6
#define AR0234_FLL_MAX			0xffff
#define AR0234_FLL_STEP			1
#define AR0234_FLL_DEFAULT		0x0c98

/* HBLANK control - read only */
#define AR0234_PPL_DEFAULT		2448

/* Exposure control */
#define AR0234_REG_EXPOSURE_COARSE	0x3012
#define AR0234_REG_EXPOSURE_FINE	0x3014
#define AR0234_EXPOSURE_MIN		4
#define AR0234_EXPOSURE_STEP		1
#define AR0234_EXPOSURE_DEFAULT		0x640
#define AR0234_EXPOSURE_MAX		65535

/* Analog gain control (R0x3060: coarse[6:4]=2^s s=0..4, fine[3:0]; 7-bit
 * context-A field). 0x40 = s4 = 16x; reg is monotonic 1x..16x over 0..0x40. */
#define AR0234_REG_ANALOG_GAIN		0x3060
#define AR0234_ANA_GAIN_MIN		0
#define AR0234_ANA_GAIN_MAX		0x40
#define AR0234_ANA_GAIN_STEP		1
#define AR0234_ANA_GAIN_DEFAULT		0x0

/* Digital gain control (R0x305E: 11-bit xxxx.yyyyyyy Q7; 0x080 = 1.0x,
 * 0x7FF = 15.99x). */
#define AR0234_REG_DIGITAL_GAIN		0x305e
#define AR0234_DGTL_GAIN_MIN		0x0080
#define AR0234_DGTL_GAIN_MAX		0x07ff
#define AR0234_DGTL_GAIN_DEFAULT	0x0080
#define AR0234_DGTL_GAIN_STEP		1

#define AR0234_REG_ORIENTATION		0x3040
#define AR0234_REG_ORIENTATION_HFLIP	BIT(14)
#define AR0234_REG_ORIENTATION_VFLIP	BIT(15)

/* SMIA_TEST: bit 8 enables the 2 embedded-data lines; clear to disable */
#define AR0234_REG_SMIA_TEST		0x3064
#define AR0234_SMIA_TEST_EMB_DATA	BIT(8)

#define AR0234_REG_OUTPUT_DEPTH		0x31AC

/* Test Pattern Control */
#define AR0234_REG_TEST_PATTERN		0x0600
#define AR0234_TEST_PATTERN_DISABLE	0
#define AR0234_TEST_PATTERN_SOLID_COLOR	1
#define AR0234_TEST_PATTERN_COLOR_BARS	2
#define AR0234_TEST_PATTERN_GREY_COLOR	3
#define AR0234_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define AR0234_REG_TESTP_RED		0x0602
#define AR0234_REG_TESTP_GREENR		0x0604
#define AR0234_REG_TESTP_BLUE		0x0606
#define AR0234_REG_TESTP_GREENB		0x0608
#define AR0234_TESTP_COLOUR_MIN		0
#define AR0234_TESTP_COLOUR_MAX		0x03ff
#define AR0234_TESTP_COLOUR_STEP	1
#define AR0234_TESTP_RED_DEFAULT	AR0234_TESTP_COLOUR_MAX
#define AR0234_TESTP_GREENR_DEFAULT	0
#define AR0234_TESTP_BLUE_DEFAULT	0
#define AR0234_TESTP_GREENB_DEFAULT	0

enum pad_types {
	IMAGE_PAD,
	NUM_PADS
};

/* AR0234 native and active pixel array size. */
#define AR0234_NATIVE_WIDTH		1484U
#define AR0234_NATIVE_HEIGHT		856U
#define AR0234_PIXEL_ARRAY_LEFT		6U
#define AR0234_PIXEL_ARRAY_TOP		10U
#define AR0234_PIXEL_ARRAY_WIDTH	1920U
#define AR0234_PIXEL_ARRAY_HEIGHT	1080U

struct ar0234_reg {
	u16 address;
	u16 val;
};

struct ar0234_reg_list {
	unsigned int num_of_regs;
	const struct ar0234_reg *regs;
};

/* Mode : resolution and related config&values */
struct ar0234_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* V-timing */
	unsigned int vts_def;

	/* Default register values */
	struct ar0234_reg_list reg_list;
};

#define VT_PIX_CLK_DIV     0x302A
#define VT_SYS_CLK_DIV     0x302C
#define PRE_PLL_CLK_DIV    0x302E
#define PLL_MULTIPLIER     0x3030
#define OP_PIX_CLK_DIV     0x3036
#define OP_SYS_CLK_DIV     0x3038
#define DIGITAL_TEST       0x30B0

#define DELAY 0xffff	/* Delay for specified number of ms */

static const struct ar0234_reg mode_1920x1080_regs[] = {
	// [1920x1080]
	{0x302A, 0x0006},		//VT_PIX_CLK_DIV = 6
	{0x302C, 0x0002},		//VT_SYS_CLK_DIV = 2
	{0x302E, 0x0002},		//PRE_PLL_CLK_DIV = 2
	{0x3030, 0x0028},		//PLL_MULTIPLIER = 40
	{0x3036, 0x000C},		//OP_PIX_CLK_DIV = 12
	{0x3038, 0x0002},		//OP_SYS_CLK_DIV = 2
	{0x31B0, 0x0059},		//FRAME_PREAMBLE = 89
	{0x31B2, 0x003B},		//LINE_PREAMBLE = 59
	{0x31B4, 0x31C5},		//MIPI_TIMING_0 = 12741
	{0x31B6, 0x214C},		//MIPI_TIMING_1 = 8524
	{0x31B8, 0x5048},		//MIPI_TIMING_2 = 20552
	{0x31BA, 0x0186},		//MIPI_TIMING_3 = 390
	{0x31BC, 0x8805},		//MIPI_TIMING_4 = 2053
	{0x3354, 0x002C},		//MIPI_CNTRL = 44


	//[Timing_settings]
	{0x301A, 0x2058},		//RESET_REGISTER = 8280
	{0x31AE, 0x0202},		//SERIAL_FORMAT = 514
	{0x3002, 0x0008},		//Y_ADDR_START = 8
	{0x3004, 0x0008},		//X_ADDR_START = 8
	{0x3006, 0x043F},		//Y_ADDR_END = 1087
	{0x3008, 0x0787},		//X_ADDR_END = 1927
	{0x300A, 0x04C4},		//FRAME_LENGTH_LINES = 1220
	{0x300C, 0x0264},		//LINE_LENGTH_PCK = 612
	{0x3012, 0x016F},		//COARSE_INTEGRATION_TIME = 367
	{0x31AC, 0x0C0C},		//DATA_FORMAT_BITS = 3084
	{0x306E, 0x9010},		//DATAPATH_SELECT = 36880
	{0x30A2, 0x0001},		//X_ODD_INC = 1
	{0x30A6, 0x0001},		//Y_ODD_INC = 1
	{0x3082, 0x0003},		//OPERATION_MODE_CTRL = 3
	{0x3040, 0x0000},		//READ_MODE = 0
	{0x31D0, 0x0000},		//COMPANDING = 0
	{0x301A, 0x205C},		//RESET_REGISTER = 8284
};

static const struct ar0234_reg mode_1920x1200_60_regs[] = {
	// [1920x1200 60fps]
	/* Cropping / output size */
	{0x302A, 0x0006},		//VT_PIX_CLK_DIV = 6
	{0x302C, 0x0002},		//VT_SYS_CLK_DIV = 2
	{0x302E, 0x0002},		//PRE_PLL_CLK_DIV = 2
	{0x3030, 0x0028},		//PLL_MULTIPLIER = 40
	{0x3036, 0x000C},		//OP_PIX_CLK_DIV = 12
	{0x3038, 0x0002},		//OP_SYS_CLK_DIV = 2
//BITFIELD = 0x30B0, 0x4000, 0x0000	//DIGITAL_TEST, bits 0x4000 = 0
	{0x31B0, 0x0059},		//FRAME_PREAMBLE = 89
	{0x31B2, 0x003B},		//LINE_PREAMBLE = 59
	{0x31B4, 0x31C5},		//MIPI_TIMING_0 = 12741
	{0x31B6, 0x214C},		//MIPI_TIMING_1 = 8524
	{0x31B8, 0x5048},		//MIPI_TIMING_2 = 20552
	{0x31BA, 0x0186},		//MIPI_TIMING_3 = 390
	{0x31BC, 0x8805},		//MIPI_TIMING_4 = 2053
	{0x3354, 0x002C},		//MIPI_CNTRL = 44


//[Timing_settings]
	{0x301A, 0x2058},		//RESET_REGISTER = 8280
	{0x31AE, 0x0202},		//SERIAL_FORMAT = 514
	{0x3002, 0x0008},		//Y_ADDR_START = 8
	{0x3004, 0x0008},		//X_ADDR_START = 8
	{0x3006, 0x04B7},		//Y_ADDR_END = 1207
	{0x3008, 0x0787},		//X_ADDR_END = 1927
	{0x300A, 0x04C4},		//FRAME_LENGTH_LINES = 1220
	{0x300C, 0x0264},		//LINE_LENGTH_PCK = 612
	{0x3012, 0x016F},		//COARSE_INTEGRATION_TIME = 367
	{0x31AC, 0x0C0C},		//DATA_FORMAT_BITS = 3084
	{0x306E, 0x9010},		//DATAPATH_SELECT = 36880
	{0x30A2, 0x0001},		//X_ODD_INC = 1
	{0x30A6, 0x0001},		//Y_ODD_INC = 1
	{0x3082, 0x0003},		//OPERATION_MODE_CTRL = 3
	{0x3040, 0x0000},		//READ_MODE = 0
	{0x31D0, 0x0000},		//COMPANDING = 0
	{0x301A, 0x205C},		//RESET_REGISTER = 8284

//	{0x3028, 0x0010}, //ROW_SPEED
};

static const char * const ar0234_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int ar0234_test_pattern_val[] = {
	AR0234_TEST_PATTERN_DISABLE,
	AR0234_TEST_PATTERN_COLOR_BARS,
	AR0234_TEST_PATTERN_SOLID_COLOR,
	AR0234_TEST_PATTERN_GREY_COLOR,
	AR0234_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const ar0234_supply_name[] = {
	/* Supplies can be enabled in any order */
	"vana",  /* Analog (2.8V) supply */
	"vdig",  /* Digital Core (1.8V) supply */
	"vddl",  /* IF (1.2V) supply */
};

#define AR0234_NUM_SUPPLIES ARRAY_SIZE(ar0234_supply_name)

#define AR0234_XCLR_MIN_DELAY_US	6200
#define AR0234_XCLR_DELAY_RANGE_US	1000

static const u32 bayer_codes[] = {
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SGRBG12_1X12,
};

static const s64 link_freq[] = {
	AR0234_DEFAULT_LINK_FREQ,
};

/*
* There is an inherent assumption that there will be the same number of codes
* for the Bayer and monochrome sensors
*/
#define NUM_CODES ARRAY_SIZE(bayer_codes)

/* Mode configs */
static const struct ar0234_mode supported_modes[] = {
	{
		/* 1920x1080 mode */
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = AR0234_PIXEL_ARRAY_LEFT,
			.top = AR0234_PIXEL_ARRAY_TOP,
			.width = 1920,
			.height = 1080
		},
		.vts_def = AR0234_VTS_30FPS,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
	},
	{
		/* 1280x800 60fps mode */
		.width = 1920,
		.height = 1200,
		.crop = {
			.left = AR0234_PIXEL_ARRAY_LEFT,
			.top = AR0234_PIXEL_ARRAY_TOP,
			.width = 1920,
			.height = 1200
		},
		.vts_def = AR0234_VTS_30FPS,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1200_60_regs),
			.regs = mode_1920x1200_60_regs,
		},
	},
};

struct ar0234 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct regmap *regmap;

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to AR0234 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[AR0234_NUM_SUPPLIES];

	bool monochrome;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct ar0234_mode *mode;

	/*
	* Mutex for serialized access:
	* Protect sensor module set pad format and start/stop streaming safely.
	*/
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct ar0234 *to_ar0234(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct ar0234, sd);
}

/* Read a 16-bit register */
static int ar0234_read_reg(struct ar0234 *ar0234, u16 reg, u64 *val)
{
	int ret = 0;

	cci_read(ar0234->regmap, CCI_REG16(reg), val, &ret);

	return ret;
}

/* Write a 16-bit register */
static int ar0234_write_reg(struct ar0234 *ar0234, u16 reg, u64 val)
{
	int ret = 0;

	cci_write(ar0234->regmap, CCI_REG16(reg), val, &ret);

	return ret;
}

/* Write a list of registers */
static int ar0234_write_regs(struct ar0234 *ar0234,
				const struct ar0234_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		if (regs[i].address == DELAY) {
			usleep_range(regs[i].val * 1000,
					(regs[i].val + 1) * 1000);
			continue;
		}

		ret = ar0234_write_reg(ar0234, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
						"Failed to write reg 0x%4.4x. error = %d\n",
						regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

static const u32 *ar0234_get_codes(struct ar0234 *ar0234)
{
		return bayer_codes;
}

/* Get bayer order based on flip setting. */
static u32 ar0234_get_format_code(struct ar0234 *ar0234, u32 code)
{
	const u32 *codes = ar0234_get_codes(ar0234);
	unsigned int i;

	lockdep_assert_held(&ar0234->mutex);

	for (i = 0; i < NUM_CODES; i++)
		if (codes[i] == code)
			break;

	if (i >= NUM_CODES)
		i = 0;

	return codes[i];
}

static void ar0234_set_default_format(struct ar0234 *ar0234)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &ar0234->fmt;
	fmt->code = ar0234_get_codes(ar0234)[0];

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							fmt->colorspace,
							fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int ar0234_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt);

static int ar0234_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	ar0234_set_pad_format(sd, state, &format);

	return 0;
}

static int ar0234_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0234 *ar0234 =
		container_of(ctrl->handler, struct ar0234, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = ar0234->mode->height + ctrl->val - 4;
		exposure_def = (exposure_max < AR0234_EXPOSURE_DEFAULT) ?
			exposure_max : AR0234_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(ar0234->exposure,
					ar0234->exposure->minimum,
					exposure_max, ar0234->exposure->step,
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
		dev_dbg(&client->dev, "DBG: ar0234 analogue gain = %d\n", ctrl->val);
		ret = ar0234_write_reg(ar0234, AR0234_REG_ANALOG_GAIN,
					ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ar0234_write_reg(ar0234, AR0234_REG_EXPOSURE_COARSE,
					ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		dev_dbg(&client->dev, "DBG: ar0234 digital gain = %d\n", ctrl->val);
		ret = ar0234_write_reg(ar0234, AR0234_REG_DIGITAL_GAIN,
				       ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret =  ar0234_write_reg(ar0234, AR0234_REG_TEST_PATTERN,
				       ar0234_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	{
		u64 reg;

		ret = ar0234_read_reg(ar0234, AR0234_REG_ORIENTATION, &reg);
		if (ret)
			break;

		reg &= ~(AR0234_REG_ORIENTATION_HFLIP |
			AR0234_REG_ORIENTATION_VFLIP);
		if (ar0234->hflip->val)
			reg |= AR0234_REG_ORIENTATION_HFLIP;
		if (ar0234->vflip->val)
			reg |= AR0234_REG_ORIENTATION_VFLIP;

		ret = ar0234_write_reg(ar0234, AR0234_REG_ORIENTATION, reg);
		break;
	}
	case V4L2_CID_VBLANK:
		ret = ar0234_write_reg(ar0234, AR0234_REG_VTS,
					ar0234->mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = 0;//ar0234_write_reg(ar0234, AR0234_REG_TESTP_RED,
			//	       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = 0;//ar0234_write_reg(ar0234, AR0234_REG_TESTP_GREENR,
			//	       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = 0;//ar0234_write_reg(ar0234, AR0234_REG_TESTP_BLUE,
		//		       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = 0;//ar0234_write_reg(ar0234, AR0234_REG_TESTP_GREENB,
			//	       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			"ctrl(id:0x%x,val:0x%x) is not handled\n",
			ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0234_ctrl_ops = {
	.s_ctrl = ar0234_set_ctrl,
};

static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	const u32 *codes = ar0234_get_codes(ar0234);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->index >= NUM_CODES)
		return -EINVAL;

	code->code = ar0234_get_format_code(ar0234, codes[code->index]);
	if (code->code == 0)
		return -EINVAL;

	return 0;
}

static int ar0234_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != ar0234_get_format_code(ar0234, fse->code))
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void ar0234_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							fmt->colorspace,
							fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void ar0234_update_image_pad_format(struct ar0234 *ar0234,
					const struct ar0234_mode *mode,
					struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	ar0234_reset_colorspace(&fmt->format);
}

static int __ar0234_get_pad_format(struct ar0234 *ar0234,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = ar0234_get_format_code(ar0234, try_fmt->code);
		fmt->format = *try_fmt;
	} else {
		ar0234_update_image_pad_format(ar0234, ar0234->mode, fmt);
		fmt->format.code =
			ar0234_get_format_code(ar0234, ar0234->fmt.code);
	}

	return 0;
}

static int ar0234_get_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	int ret;

	mutex_lock(&ar0234->mutex);
	ret = __ar0234_get_pad_format(ar0234, sd_state, fmt);
	mutex_unlock(&ar0234->mutex);

	return ret;
}

static int ar0234_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	const struct ar0234_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	int exposure_max, exposure_def, hblank;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&ar0234->mutex);

	if (fmt->pad == IMAGE_PAD) {
		fmt->format.code = ar0234_get_format_code(ar0234,
							fmt->format.code);

		mode = v4l2_find_nearest_size(supported_modes,
						ARRAY_SIZE(supported_modes),
						width, height,
						fmt->format.width,
						fmt->format.height);
		ar0234_update_image_pad_format(ar0234, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
								fmt->pad);
			*framefmt = fmt->format;
		} else {
			ar0234->fmt = fmt->format;
			ar0234->mode = mode;
			/* Update limits and set FPS to default */
			__v4l2_ctrl_modify_range(ar0234->vblank,
						AR0234_VBLANK_MIN,
						AR0234_VTS_MAX - mode->height,
						1,
						mode->vts_def - mode->height);
			__v4l2_ctrl_s_ctrl(ar0234->vblank,
					mode->vts_def - mode->height);
			/*
			* Update max exposure while meeting
			* expected vblanking
			*/
			exposure_max = mode->vts_def - 4;
			exposure_def =
				(exposure_max < AR0234_EXPOSURE_DEFAULT) ?
					exposure_max : AR0234_EXPOSURE_DEFAULT;
			__v4l2_ctrl_modify_range(ar0234->exposure,
						ar0234->exposure->minimum,
						exposure_max,
						ar0234->exposure->step,
						exposure_def);
			/*
			* Currently PPL is fixed to AR0234_PPL_DEFAULT, so
			* hblank depends on mode->width only, and is not
			* changeble in any way other than changing the mode.
			*/
			hblank = AR0234_PPL_DEFAULT - mode->width;
			__v4l2_ctrl_modify_range(ar0234->hblank, hblank, hblank,
						1, hblank);
		}
	}

	mutex_unlock(&ar0234->mutex);

	return 0;
}

static int ar0234_set_framefmt(struct ar0234 *ar0234)
{
	switch (ar0234->fmt.code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return ar0234_write_reg(ar0234, 0x31AC, 0x0A0A);
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		return ar0234_write_reg(ar0234, 0x31AC, 0x0C0C);
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__ar0234_get_pad_crop(struct ar0234 *ar0234, struct v4l2_subdev_state *sd_state,
			unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0234->mode->crop;
	}

	return NULL;
}

static int ar0234_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct ar0234 *ar0234 = to_ar0234(sd);

		mutex_lock(&ar0234->mutex);
		sel->r = *__ar0234_get_pad_crop(ar0234, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&ar0234->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = AR0234_NATIVE_WIDTH;
		sel->r.height = AR0234_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = AR0234_PIXEL_ARRAY_TOP;
		sel->r.left = AR0234_PIXEL_ARRAY_LEFT;
		sel->r.width = AR0234_PIXEL_ARRAY_WIDTH;
		sel->r.height = AR0234_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int ar0234_start_streaming(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	const struct ar0234_reg_list *reg_list;
	u64 val;
	int ret;


	/* Apply default values of current mode */
	reg_list = &ar0234->mode->reg_list;
	ret = ar0234_write_regs(ar0234, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Disable the sensor's embedded-data lines */
	ret = ar0234_read_reg(ar0234, AR0234_REG_SMIA_TEST, &val);
	if (!ret)
		ret = ar0234_write_reg(ar0234, AR0234_REG_SMIA_TEST,
				       val & ~AR0234_SMIA_TEST_EMB_DATA);
	if (ret) {
		dev_err(&client->dev, "%s failed to disable embedded data\n",
			__func__);
		return ret;
	}

	ret = ar0234_set_framefmt(ar0234);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(ar0234->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return ar0234_write_reg(ar0234, AR0234_REG_RESET,
				AR0234_REG_RESET_STREAM_ON);
}

static void ar0234_stop_streaming(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	int ret;

	/* set stream off register */
	ret = ar0234_write_reg(ar0234, AR0234_REG_RESET,
				AR0234_REG_RESET_STREAM_OFF);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int ar0234_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&ar0234->mutex);
	if (ar0234->streaming == enable) {
		mutex_unlock(&ar0234->mutex);
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
		ret = ar0234_start_streaming(ar0234);
		if (ret)
			goto err_rpm_put;
	} else {
		ar0234_stop_streaming(ar0234);
		pm_runtime_put(&client->dev);
	}

	ar0234->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(ar0234->vflip, enable);
	__v4l2_ctrl_grab(ar0234->hflip, enable);

	mutex_unlock(&ar0234->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&ar0234->mutex);

	return ret;
}

/* Power/clock management functions */
static int ar0234_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);
	int ret;

	ret = regulator_bulk_enable(AR0234_NUM_SUPPLIES,
					ar0234->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(ar0234->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(ar0234->reset_gpio, 1);
	usleep_range(AR0234_XCLR_MIN_DELAY_US,
			AR0234_XCLR_MIN_DELAY_US + AR0234_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(AR0234_NUM_SUPPLIES, ar0234->supplies);

	return ret;
}

static int ar0234_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	gpiod_set_value_cansleep(ar0234->reset_gpio, 0);
	regulator_bulk_disable(AR0234_NUM_SUPPLIES, ar0234->supplies);
	clk_disable_unprepare(ar0234->xclk);

	return 0;
}

static int ar0234_get_regulators(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	unsigned int i;

	for (i = 0; i < AR0234_NUM_SUPPLIES; i++)
		ar0234->supplies[i].supply = ar0234_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
					AR0234_NUM_SUPPLIES,
					ar0234->supplies);
}

/* Verify chip ID */
static int ar0234_identify_module(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	unsigned int retry;
	int ret;
	u64 val;

	for (retry = 0; retry < 5; retry++) {
		ret = ar0234_read_reg(ar0234, AR0234_REG_CHIP_ID, &val);
		if (ret) {
			dev_warn(&client->dev,
				 "failed to read chip id %d (attempt %u/5)\n",
				 ret, retry + 1);
			continue;
		}

		if (val == AR0234_CHIP_ID || val == AR0234_CHIP_ID_MONO)
			break;

		dev_warn(&client->dev,
			 "chip id mismatch: %x!=%llx (attempt %u/5)\n",
			 AR0234_CHIP_ID, val, retry + 1);
		ret = -EIO;
	}
	if (ret) {
		dev_err(&client->dev, "failed to identify chip id %d\n",
			ret);
		return ret;
	}

	dev_info(&client->dev, "Success reading chip id: %llx\n", val);

	if (val == AR0234_CHIP_ID_MONO)
		ar0234->monochrome = true;

	return 0;
}

static const struct v4l2_subdev_core_ops ar0234_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ar0234_video_ops = {
	.s_stream = ar0234_set_stream,
};

static int ar0234_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	u32 code;

	if (pad != IMAGE_PAD)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, IMAGE_PAD, 0)->code;
	v4l2_subdev_unlock_state(state);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry->pixelcode = code;
	fd->entry->stream = 0;
	fd->entry->bus.csi2.vc = 0; /* Read from device tree */
	fd->entry->bus.csi2.dt = MIPI_CSI2_DT_RAW12;

	return 0;
}

static const struct v4l2_subdev_pad_ops ar0234_pad_ops = {
	.enum_mbus_code = ar0234_enum_mbus_code,
	.get_fmt = ar0234_get_pad_format,
	.set_fmt = ar0234_set_pad_format,
	.get_selection = ar0234_get_selection,
	.enum_frame_size = ar0234_enum_frame_size,
	.get_frame_desc = ar0234_get_frame_desc,
};

static const struct v4l2_subdev_ops ar0234_subdev_ops = {
	.core = &ar0234_core_ops,
	.video = &ar0234_video_ops,
	.pad = &ar0234_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0234_internal_ops = {
	.init_state = ar0234_init_state,
};

/* Initialize control handlers */
static int ar0234_init_controls(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	unsigned int height = ar0234->mode->height;
	int exposure_max, exposure_def, hblank;
	struct v4l2_ctrl *ctrl;
	int i, ret;

	ctrl_hdlr = &ar0234->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&ar0234->mutex);
	ctrl_hdlr->lock = &ar0234->mutex;

	/* By default, PIXEL_RATE is read only */
	ar0234->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						AR0234_PIXEL_RATE,
						AR0234_PIXEL_RATE, 1,
						AR0234_PIXEL_RATE);
	if (ar0234->pixel_rate)
		ar0234->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initial vblank/hblank/exposure parameters based on current mode */
	ar0234->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_VBLANK, AR0234_VBLANK_MIN,
					AR0234_VTS_MAX - height, 1,
					ar0234->mode->vts_def - height);
	hblank = AR0234_PPL_DEFAULT - ar0234->mode->width;
	ar0234->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_HBLANK, hblank, hblank,
					1, hblank);
	if (ar0234->hblank)
		ar0234->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = ar0234->mode->vts_def - 4;
	exposure_def = (exposure_max < AR0234_EXPOSURE_DEFAULT) ?
		exposure_max : AR0234_EXPOSURE_DEFAULT;
	ar0234->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
						V4L2_CID_EXPOSURE,
						AR0234_EXPOSURE_MIN, exposure_max,
						AR0234_EXPOSURE_STEP,
						exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			AR0234_ANA_GAIN_MIN, AR0234_ANA_GAIN_MAX,
			AR0234_ANA_GAIN_STEP, AR0234_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			AR0234_DGTL_GAIN_MIN, AR0234_DGTL_GAIN_MAX,
			AR0234_DGTL_GAIN_STEP, AR0234_DGTL_GAIN_DEFAULT);

	ar0234->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_HFLIP, 0, 1, 1, 0);

	ar0234->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(ar0234_test_pattern_menu) - 1,
					0, 0, ar0234_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		* The assumption is that
		* V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		* V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		* V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		*/
		v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
				V4L2_CID_TEST_PATTERN_RED + i,
				AR0234_TESTP_COLOUR_MIN,
				AR0234_TESTP_COLOUR_MAX,
				AR0234_TESTP_COLOUR_STEP,
				AR0234_TESTP_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &ar0234_ctrl_ops,
					V4L2_CID_LINK_FREQ, 0, 0,
					link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ar0234_ctrl_ops,
						&props);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ar0234->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&ar0234->mutex);

	return ret;
}

static void ar0234_free_controls(struct ar0234 *ar0234)
{
	v4l2_ctrl_handler_free(ar0234->sd.ctrl_handler);
	mutex_destroy(&ar0234->mutex);
}

static int ar0234_check_hwcfg(struct device *dev)
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
		ep_cfg.link_frequencies[0] != AR0234_DEFAULT_LINK_FREQ) {
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

static int ar0234_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0234 *ar0234;
	int ret;

	ar0234 = devm_kzalloc(&client->dev, sizeof(*ar0234), GFP_KERNEL);
	if (!ar0234)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ar0234->sd, client, &ar0234_subdev_ops);

	ar0234->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ar0234->regmap)) {
		ret = PTR_ERR(ar0234->regmap);
		dev_err(dev, "failed to initialise CCI: %d\n", ret);
		return ret;
	}

	/* Check the hardware configuration in device tree */
	if (ar0234_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	ar0234->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(ar0234->xclk)) {
		if (PTR_ERR(ar0234->xclk) != -EPROBE_DEFER)
			dev_err(dev, "failed to get xclk %ld\n", PTR_ERR(ar0234->xclk));
		return PTR_ERR(ar0234->xclk);
	}

	ar0234->xclk_freq = clk_get_rate(ar0234->xclk);
	if (ar0234->xclk_freq != AR0234_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			ar0234->xclk_freq);
		return -EINVAL;
	}

	ret = ar0234_get_regulators(ar0234);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	ar0234->reset_gpio = devm_gpiod_get_optional(dev, "reset",
							GPIOD_OUT_HIGH);

	/*
	* The sensor must be powered for ar0234_identify_module()
	* to be able to read the CHIP_ID register
	*/
	ret = ar0234_power_on(dev);
	if (ret)
		return ret;

	ret = ar0234_identify_module(ar0234);
	if (ret)
		goto error_power_off;

	/* Set default mode to max resolution */
	ar0234->mode = &supported_modes[0];

	/* sensor doesn't enter LP-11 state upon power up until and unless
	* streaming is started, so upon power up switch the modes to:
	* streaming -> standby
	*/
	ret = ar0234_write_reg(ar0234, AR0234_REG_RESET,
				AR0234_REG_RESET_STREAM_ON);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	/* put sensor back to standby mode */
	ret = ar0234_write_reg(ar0234, AR0234_REG_RESET,
				AR0234_REG_RESET_STREAM_OFF);
	if (ret < 0)
		goto error_power_off;
	usleep_range(100, 110);

	ret = ar0234_init_controls(ar0234);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	ar0234->sd.internal_ops = &ar0234_internal_ops;
	ar0234->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	ar0234->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	ar0234->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	ar0234_set_default_format(ar0234);

	ret = media_entity_pads_init(&ar0234->sd.entity, NUM_PADS, ar0234->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	/*
	 * Leave sd.state_lock unset (independent of ar0234->mutex): the format
	 * pad ops take ar0234->mutex themselves, so sharing the state lock with
	 * it would deadlock when the core locks the state around set_fmt.
	 */
	ret = v4l2_subdev_init_finalize(&ar0234->sd);
	if (ret) {
		dev_err(dev, "failed to init subdev state: %d\n", ret);
		goto error_media_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&ar0234->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_subdev_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_subdev_cleanup:
	v4l2_subdev_cleanup(&ar0234->sd);

error_media_entity:
	media_entity_cleanup(&ar0234->sd.entity);

error_handler_free:
	ar0234_free_controls(ar0234);

error_power_off:
	ar0234_power_off(dev);

	return ret;
}

static void ar0234_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	ar0234_free_controls(ar0234);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ar0234_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id ar0234_dt_ids[] = {
	{ .compatible = "OV,ar0234" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0234_dt_ids);

static const struct dev_pm_ops ar0234_pm_ops = {
	SET_RUNTIME_PM_OPS(ar0234_power_off, ar0234_power_on, NULL)
};

static struct i2c_driver ar0234_i2c_driver = {
	.driver = {
		.name = "ar0234",
		.of_match_table	= ar0234_dt_ids,
		.pm = &ar0234_pm_ops,
	},
	.probe = ar0234_probe,
	.remove = ar0234_remove,
};

module_i2c_driver(ar0234_i2c_driver);

MODULE_AUTHOR("zihongl@leopardimaging.com");
MODULE_DESCRIPTION("OnSemi AR0234 sensor driver");
MODULE_LICENSE("GPL");
