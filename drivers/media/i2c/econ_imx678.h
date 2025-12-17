/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Based on V4L2 IMX678 Image Sensor driver
 * Copyright (C) 2025 e-con systems
 *
 */

#include <linux/v4l2-controls.h>
#include <linux/types.h>

#ifndef _CAM_FIRMWARE_H
#define _CAM_FIRMWARE_H

/*   Local Defines */
#define MAX_BUF_LEN 2048

#define MAX_PAGES 			512
#define TOTAL_PAGES 		1536
#define NUM_ERASE_CYCLES	(TOTAL_PAGES / MAX_PAGES)

#define FLASH_START_ADDRESS 0x08000000
#define FLASH_SIZE 			192*1024
#define FLASH_READ_LEN		256

#define CR 13                   /*   Carriage return */
#define LF 10                   /*   Line feed */

/* Only necessary commands added */
enum _i2c_cmds
{
	BL_GET_VERSION = 0x01,
	BL_GO = 0x21,
	BL_READ_MEM = 0x11,
	BL_WRITE_MEM = 0x31,
	BL_WRITE_MEM_NS = 0x32,
	BL_ERASE_MEM = 0x44,
	BL_ERASE_MEM_NS = 0x45,
};

enum _i2c_resp
{
	RESP_ACK = 0x79,
	RESP_NACK = 0x1F,
	RESP_BUSY = 0x76,
};

enum
{
	NUM_LANES_1 = 0x01,
	NUM_LANES_2 = 0x02,
	NUM_LANES_4 = 0x04,
	NUM_LANES_UNKWN = 0xFF,
};

enum _ihex_rectype
{
	/*   Normal data */
	REC_TYPE_DATA = 0x00,
	/*  End of File */
	REC_TYPE_EOF = 0x01,

	/*   Extended Segment Address */
	REC_TYPE_ESA = 0x02,
	/*   Start Segment Address */
	REC_TYPE_SSA = 0x03,

	/*   Extended Linear Address */
	REC_TYPE_ELA = 0x04,
	/*   Start Linear Address */
	REC_TYPE_SLA = 0x05,
};

typedef struct __attribute__ ((packed)) _ihex_rec {
	unsigned char datasize;
	unsigned short int addr;
	unsigned char rectype;
	unsigned char recdata[];
} IHEX_RECORD;

unsigned int g_bload_flashaddr = 0x0000;
unsigned int g_num_lanes = 0x00;

/*   Buffer to Send Bootloader CMDs */
unsigned char g_bload_buf[MAX_BUF_LEN] = { 0 };

#endif                          //_CAM_FIRMWARE_H

#define IMX678_XCLK_FREQ		24000000
#define IMX678_DEFAULT_LINK_FREQ	450000000

/* Embedded metadata stream structure - Not Implemented */
#define IMX678_EMBEDDED_LINE_WIDTH 16384
#define IMX678_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};
/* Defines related to MCU */

#define CMD_SIGNATURE			0x43
#define TX_LEN_PKT			5
#define RX_LEN_PKT			6
#define HEADER_FOOTER_SIZE		4
#define CMD_STATUS_MSG_LEN		7
#define MAX_CTRL_DATA_LEN 		100
#define MAX_CTRL_UI_STRING_LEN 		32
#define MAX_CTRL_MENU_ELEM 		20
#define MAX_NUM_FRATES			10
#define MAX_NUM_FMTS			20
#define NUM_CTRLS			10
#define VERSION_SIZE			32
#define MAX_ATTEMPTS			5
#define CAM_MASTER_MODE			0x00
#define CAM_SLAVE_MODE			0x01
#define EXPOSURE_30HZ			33333
#define EXPOSURE_60HZ			16667

#define MCU_CMD_STATUS_SUCCESS		0x0000
#define MCU_CMD_STATUS_PENDING		0xF000
#define MCU_CMD_STATUS_ISP_PWDN		0x0FF0
#define MCU_CMD_STATUS_ISP_UNINIT	0x0FF1
#define EXTENDED_CTRL_LENGTH		32
#define EXTENDED_CTRL_SIZE		8

#define DEBUG_CONTROLS_ENABLE 		1

// Added to load mcu firmware bin from package
#define VERSION_FILE_OFFSET			99

static const s64 imx678_link_freq_menu[] = {
	IMX678_DEFAULT_LINK_FREQ,
};

struct camera_common_frmfmt {
        struct v4l2_frmsize_discrete    size;
        const int       *framerates;
        int     num_framerates;
        bool    hdr_en;
        int     mode;
};

/* BS: Added Minimum camera_common_data structure
*/
struct camera_common_data {
	struct v4l2_ctrl_handler		*ctrl_handler;
	struct device				*dev;
	struct v4l2_subdev			subdev;
	int	numlanes;
	int	mode;
	int	def_mode, def_width, def_height;
	int	def_clk_freq;
	int	fmt_width, fmt_height;
	void	*imx678;
};

typedef enum _errno
{
        ERRCODE_SUCCESS = 0x00,
        ERRCODE_BUSY = 0x01,
        ERRCODE_INVAL = 0x02,
        ERRCODE_PERM = 0x03,
        ERRCODE_NODEV = 0x04,
        ERRCODE_IO = 0x05,
        ERRCODE_HW_SPEC = 0x06,
        ERRCODE_AGAIN = 0x07,
        ERRCODE_ALREADY = 0x08,
        ERRCODE_NOTIMPL = 0x09,
        ERRCODE_RANGE = 0x0A,

        /*   Reserved 0x0B - 0xFE */

        ERRCODE_UNKNOWN = 0xFF,
} RETCODE;

typedef enum _cmd_id
{
        CMD_ID_VERSION = 0x00,
        CMD_ID_GET_SENSOR_ID = 0x01,
        CMD_ID_GET_STREAM_INFO = 0x02,
        CMD_ID_GET_CTRL_INFO = 0x03,
        CMD_ID_INIT_CAM = 0x04,
        CMD_ID_GET_STATUS = 0x05,
        CMD_ID_DE_INIT_CAM = 0x06,
        CMD_ID_STREAM_ON = 0x07,
        CMD_ID_STREAM_OFF = 0x08,
        CMD_ID_STREAM_CONFIG = 0x09,
	CMD_ID_GET_CTRL_UI_INFO = 0x0A,

        /* Reserved 0x0B to 0x0F */

        CMD_ID_GET_CTRL = 0x10,
        CMD_ID_SET_CTRL = 0x11,
        CMD_ID_ISP_READ = 0x12,
        CMD_ID_ISP_WRITE = 0x13,
        CMD_ID_FW_UPDT = 0x14,
        CMD_ID_ISP_PDOWN = 0x15,
        CMD_ID_ISP_PUP = 0x16,

	/* Configuring MIPI Lanes */
	CMD_ID_LANE_CONFIG = 0x17,
	CMD_ID_MIPI_CLK_CONFIG = 0x18,
        /* Reserved - 0x1E to 0xFE (except 0x43) */
	
        CMD_ID_UNKNOWN = 0xFF,

} HOST_CMD_ID;

enum
{
        FRAME_RATE_DISCRETE = 0x01,
        FRAME_RATE_CONTINOUS = 0x02,
};

enum
{
        CTRL_STANDARD = 0x01,
        CTRL_EXTENDED = 0x02,
};

enum
{
/*  0x01 - Integer (32bit)
		0x02 - Long Int (64 bit)
		0x03 - String
		0x04 - Pointer to a 1-Byte Array
		0x05 - Pointer to a 2-Byte Array
		0x06 - Pointer to a 4-Byte Array
		0x07 - Pointer to Generic Data (custom Array)
*/

        EXT_CTRL_TYPE_INTEGER = 0x01,
        EXT_CTRL_TYPE_LONG = 0x02,
        EXT_CTRL_TYPE_STRING = 0x03,
        EXT_CTRL_TYPE_PTR8 = 0x04,
        EXT_CTRL_TYPE_PTR16 = 0x05,
        EXT_CTRL_TYPE_PTR32 = 0x06,
        EXT_CTRL_TYPE_VOID = 0x07,
};

typedef struct _isp_stream_info
{
        uint32_t fmt_fourcc;
        uint16_t width;
        uint16_t height;
        uint8_t frame_rate_type;
        union
        {
                struct
                {
                        uint16_t frame_rate_num;
                        uint16_t frame_rate_denom;
                } disc;
                struct
                {
                        uint16_t frame_rate_min_num;
                        uint16_t frame_rate_min_denom;
                        uint16_t frame_rate_max_num;
                        uint16_t frame_rate_max_denom;
                        uint16_t frame_rate_step_num;
                        uint16_t frame_rate_step_denom;
                } cont;
        } frame_rate;
} ISP_STREAM_INFO;


typedef struct _isp_ctrl_ui_info {
	struct {
		char ctrl_name[MAX_CTRL_UI_STRING_LEN];
		uint8_t ctrl_ui_type;
		uint8_t ctrl_ui_flags;
	}ctrl_ui_info;

	/* This Struct is valid only if ctrl_ui_type = 0x03 */
	struct {
		uint8_t num_menu_elem;
		char **menu;
		long *menu_int;
	}ctrl_menu_info;
} ISP_CTRL_UI_INFO;



typedef struct _isp_ctrl_info_std
{
        uint32_t ctrl_id;
        uint8_t ctrl_type;
        union
        {
                struct
                {
                        int32_t ctrl_min;
                        int32_t ctrl_max;
                        int32_t ctrl_def;
                        int32_t ctrl_step;
                } std;
                struct
                {
                        uint8_t val_type;
                        uint32_t val_length;
                        // This size may vary according to ctrl types
						uint64_t ctrl_min;
						uint64_t ctrl_max;
						uint64_t ctrl_def;
						uint64_t ctrl_step;
				
						uint8_t val_data[MAX_CTRL_DATA_LEN];
				} ext;
        } ctrl_data;
	ISP_CTRL_UI_INFO ctrl_ui_data;

} ISP_CTRL_INFO;

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
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* regulator supplies */
static const char * const imx678_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (3.3V) supply */
	"VDIG",  /* Digital Core (1.1V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define IMX678_NUM_SUPPLIES ARRAY_SIZE(imx678_supply_name)

struct imx678 {
	struct v4l2_ctrl_handler ctrl_handler;
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct camera_common_data *s_data;
	ISP_STREAM_INFO			*stream_info;
	ISP_CTRL_INFO 			*cam_ctrl_info;
	uint32_t 			*ctrldb;
	struct camera_common_frmfmt 	*cam_frmfmt;
	int 				num_ctrls;
	int 				*streamdb;
	int				frmfmt_mode;
	uint32_t 			format_fourcc;
	int 				frate_index;
	uint32_t			mipi_lane_config;
	uint32_t			mipi_clock_config;
	bool				use_dol_wdr_mode;
	uint32_t			reset_gpio;
	uint32_t			boot_gpio;
	bool				use_sensor_mode_id;
	bool				mipi_clk_configurable;
	int 				frm_fmt_size;

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct regulator_bulk_data supplies[IMX678_NUM_SUPPLIES];

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;
};

// Added to load MCU firmware bin along with package
const char *cam_fw_name = NULL;
const struct firmware *cam_fw = NULL;
static uint8_t is_fw_loaded = 0;
unsigned char *cam_fw_buf = NULL;
static uint8_t stream_status = 0;

static int cam_read(struct i2c_client *client, u8 * val, u32 count);
static int cam_write(struct i2c_client *client, u8 * val, u32 count);
static int cam_list_fmts(struct i2c_client *client, struct imx678 *imx678,
			  ISP_STREAM_INFO *stream_info,int *frm_fmt_size);
static int cam_list_ctrls(struct i2c_client *client, struct imx678 *imx678,
                          ISP_CTRL_INFO * cam_ctrl_info);
unsigned char errorcheck(char *data, unsigned int len);
static int is_fw_update_required(struct i2c_client *client, struct imx678 *imx678,
		unsigned char * fw_version, unsigned char *bin_fw_version);
static int cam_get_cmd_status(struct i2c_client *client, uint8_t * cmd_id,
                              uint16_t * cmd_status, uint8_t * ret_code);
static int cam_init(struct i2c_client *client);
static int cam_stream_config(struct i2c_client *client, struct imx678 *priv, uint32_t format,
                             int mode, int frate_index);
static int cam_fw_update(struct i2c_client *client, unsigned char *cam_fw_version);

static int cam_stream_on(struct i2c_client *client, struct imx678 *priv);
static int cam_stream_off(struct i2c_client *client, struct imx678 *priv);
