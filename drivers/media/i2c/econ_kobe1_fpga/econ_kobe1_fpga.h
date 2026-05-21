/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Based on V4L2 FPGA Driver
 * Copyright (C) 2026 e-con systems
 *
 */

#include <linux/v4l2-controls.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>

// Custom CIDs for FPGA Control
#define V4L2_CID_FPGA_BASE      (V4L2_CTRL_CLASS_USER | 0x2000)

#define V4L2_CID_FPGA_MIPI_TX_CLK_MODE	(V4L2_CID_FPGA_BASE + 0)
#define V4L2_CID_FPGA_MIPI_SPEED	(V4L2_CID_FPGA_BASE + 1)
#define V4L2_CID_FPGA_MIPI_FILTER_DT1	(V4L2_CID_FPGA_BASE + 2)
#define V4L2_CID_FPGA_MIPI_FILTER_DT2	(V4L2_CID_FPGA_BASE + 3)
#define V4L2_CID_FPGA_MIPI_FILTER_DT3	(V4L2_CID_FPGA_BASE + 4)
#define V4L2_CID_FPGA_MIPI_FILTER_DT4	(V4L2_CID_FPGA_BASE + 5)
#define V4L2_CID_FPGA_TPG_MODE		(V4L2_CID_FPGA_BASE + 6)
#define V4L2_CID_FPGA_TPG_ENABLE	(V4L2_CID_FPGA_BASE + 7)
#define V4L2_CID_FPGA_READ_RX_STATUS	(V4L2_CID_FPGA_BASE + 8)
#define V4L2_CID_FPGA_READ_TX_STATUS	(V4L2_CID_FPGA_BASE + 9)
#define V4L2_CID_FPGA_MIPI_DT_FILTER_FIRST	(V4L2_CID_FPGA_BASE + 10)
#define V4L2_CID_FPGA_MIPI_DT_FILTER_LAST	(V4L2_CID_FPGA_BASE + 11)
#define V4L2_CID_FPGA_MIPI_RX_CLK_MODE		(V4L2_CID_FPGA_BASE + 12)

// FPGA RX 
#define V4L2_CID_FPGA_RX_READ_DT1		(V4L2_CID_FPGA_BASE + 13)
#define V4L2_CID_FPGA_RX_READ_DT2		(V4L2_CID_FPGA_BASE + 14)
#define V4L2_CID_FPGA_RX_READ_DT3		(V4L2_CID_FPGA_BASE + 15)
#define V4L2_CID_FPGA_RX_READ_DT4		(V4L2_CID_FPGA_BASE + 16)
#define V4L2_CID_FPGA_RX_READ_DT5		(V4L2_CID_FPGA_BASE + 17)
#define V4L2_CID_FPGA_RX_READ_DT6		(V4L2_CID_FPGA_BASE + 18)
#define V4L2_CID_FPGA_RX_READ_DT7		(V4L2_CID_FPGA_BASE + 19)
#define V4L2_CID_FPGA_RX_READ_DT8		(V4L2_CID_FPGA_BASE + 20)
#define V4L2_CID_FPGA_RX_READ_FPS		(V4L2_CID_FPGA_BASE + 21)
#define V4L2_CID_FPGA_RX_READ_CRC		(V4L2_CID_FPGA_BASE + 22)
#define V4L2_CID_FPGA_RX_READ_ECC		(V4L2_CID_FPGA_BASE + 23)
#define V4L2_CID_FPGA_RX_READ_LC		(V4L2_CID_FPGA_BASE + 24)
#define V4L2_CID_FPGA_RX_READ_WC		(V4L2_CID_FPGA_BASE + 25)

// FPGA TX status
#define V4L2_CID_FPGA_TX_READ_DT1		(V4L2_CID_FPGA_BASE + 26)
#define V4L2_CID_FPGA_TX_READ_DT2		(V4L2_CID_FPGA_BASE + 27)
#define V4L2_CID_FPGA_TX_READ_DT3		(V4L2_CID_FPGA_BASE + 28)
#define V4L2_CID_FPGA_TX_READ_DT4		(V4L2_CID_FPGA_BASE + 29)
#define V4L2_CID_FPGA_TX_READ_FIFO_OVERFLOW		(V4L2_CID_FPGA_BASE + 30)
#define V4L2_CID_FPGA_TX_READ_FPS		(V4L2_CID_FPGA_BASE + 31)
#define V4L2_CID_FPGA_TX_READ_LC		(V4L2_CID_FPGA_BASE + 32)
#define V4L2_CID_FPGA_TX_READ_WC		(V4L2_CID_FPGA_BASE + 33)

#define V4L2_CID_FPGA_TX_STREAMING_CTRL		(V4L2_CID_FPGA_BASE + 34)
#define V4L2_CID_FPGA_VC_SELECT		(V4L2_CID_FPGA_BASE + 35)

#define FPGA_RST_HANDLE 	1
#define FPGA_FIRMWARE_UPDATE	1
#define FPGA_MAJOR_FW_VERSION	0x00001
#define FPGA_MINOR_FW_VERSION	0x00000
#define FPGA_SLAVE_ADDR 	0x0C

#if FPGA_FIRMWARE_UPDATE
short int fpga_init(struct i2c_client *client);
int write_spi_data_to_fpga(struct i2c_client *client);
#endif

s32 fpga_write_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val);
s32 fpga_write_32byte_reg(struct i2c_client *client, u16 sladdr, u16 reg, char *bytearray);
s32 fpga_read_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 * val);
short int ispProcessI2C(struct i2c_client *client);

void toggle_gpio_fpga(struct gpio_desc *gpio, int value);

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

struct fpga {
	struct v4l2_ctrl_handler ctrl_handler;
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_device v4l2_dev;
	uint8_t curr_tp_mode;
};


typedef struct FpgaTestPatternModes {
	uint8_t tp_fps;
	uint16_t tp_width;
	uint16_t tp_height;
	uint16_t tp_speed;
} FPGA_TP_MODES;

const FPGA_TP_MODES fpga_tp_modes[] = {
	{ // Mode 0
		.tp_fps = 10,
		.tp_width = 3840,
		.tp_height = 2160,
		.tp_speed = 594,
	},
	{ // Mode 1
		.tp_fps = 30,
		.tp_width = 1920,
		.tp_height = 1080,
		.tp_speed = 594,
	},
	{ // Mode 2
		.tp_fps = 60,
		.tp_width = 1280,
		.tp_height = 720,
		.tp_speed = 594,
	},
	{ // Mode 3
		.tp_fps = 20,
		.tp_width = 3840,
		.tp_height = 2160,
		.tp_speed = 1188,
	},
	{ // Mode 4
		.tp_fps = 60,
		.tp_width = 1920,
		.tp_height = 1080,
		.tp_speed = 1188,
	},
	{ // Mode 5
		.tp_fps = 60,
		.tp_width = 1280,
		.tp_height = 720,
		.tp_speed = 1188,
	},
	{ // Mode 6
		.tp_fps = 45,
		.tp_width = 3840,
		.tp_height = 2160,
		.tp_speed = 2376,
	},
	{ // Mode 7
		.tp_fps = 60,
		.tp_width = 1920,
		.tp_height = 1080,
		.tp_speed = 2376,
	},
	{ // Mode 8
		.tp_fps = 60,
		.tp_width = 1280,
		.tp_height = 720,
		.tp_speed = 2376,
	},
};

// MIPI Speed Calculations
struct ConfigRegisters{
    uint8_t reg1;
    uint8_t reg2;
    uint8_t reg3;
    uint8_t reg4;
    uint8_t reg5;
};

struct MipiTx_TimingParams_t{
	uint8_t t_lpx;
	uint8_t t_clkprep;
	uint8_t t_clk_hszero;
	uint8_t t_clkpre;
	uint8_t t_clkpost;
	uint8_t t_clktrail;
	uint8_t t_clkexit;

	uint8_t t_datprep;
	uint8_t t_dat_hszero;
	uint8_t t_dattrail;
	uint8_t t_datexit;

	uint8_t t_skewcal_hszero;
	uint16_t t_skewcal_init;
	uint16_t t_skewcal_period;
};

struct PllRegisters{
    uint8_t reg_0B;
    uint8_t reg_0C;
    uint8_t reg_0D;
    uint8_t reg_0E;

    uint8_t reg_15;
    uint8_t reg_16;
    uint8_t reg_05;
    uint8_t reg_06;

    uint8_t reg_13;
    uint8_t reg_14;

    uint8_t reg_29;
    uint8_t reg_2A;
};

int get_pll_coefficients(uint8_t ref_clk, uint16_t target_rate, uint8_t *best_n, uint8_t *best_m, uint8_t *best_o, uint16_t *best_r);
int get_config_registers(int CN, int CM, int CO, struct ConfigRegisters *regs);
int calculate_tLPX(uint8_t byte_clk_mhz, uint8_t *result);
int calculate_tCLKPREP(uint8_t byte_clk_mhz, uint8_t *result);
int calculate_tCLK_HSZERO(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result);
int calculate_tCLKPRE(uint8_t gear, uint8_t *result);
int calculate_tCLKPOST(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result);
int calculate_tCLKTRAIL(uint8_t byte_clk_mhz, uint8_t *result);
int calculate_tCLKEXIT(uint8_t byte_clk_mhz, uint8_t *result);
int calculate_tDATPREP(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result);
int calculate_tDAT_HSZERO(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result);
int calculate_tDATTRAIL(uint8_t byte_clk_mhz, uint16_t tx_line_rate_mbps, uint8_t gear, uint8_t *result);
int calculate_tDATEXIT(uint8_t byte_clk_mhz, uint8_t *result);
int calculate_tSKEWCAL_INIT(uint8_t byte_clk_mhz, uint16_t tx_line_rate_mbps, uint8_t gear, uint16_t *result);
int calculate_tSKEWCAL_PERIOD(uint8_t gear, uint16_t *result);
int calculate_tSKEWCAL_HSZERO(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result);
int calculate_tclk_settle(uint8_t syncclk_mhz, uint8_t ns_target, uint8_t *result);
int calculate_tHS_SETTLE(uint8_t gear, uint8_t byteclk_mhz, uint8_t syncclk_mhz, uint8_t *result);
int calculate_m_value(uint8_t reg_0B, uint8_t reg_0C, uint8_t reg_0D, uint8_t reg_0E, uint8_t *m_val);
int calculate_n_value(uint8_t reg_0B, uint8_t reg_15, uint8_t reg_16, uint8_t reg_05, uint8_t reg_06, uint16_t *n_val);
int calculate_f_value(uint8_t reg_0B, uint8_t reg_13, uint8_t reg_14, uint16_t *f_val);
int calculate_o_value(uint8_t reg_29, uint8_t reg_2A, uint8_t *o_val);
int calculate_vco_frequency(uint16_t ref_clk, uint8_t m_val, uint16_t n_val, uint16_t f_val, uint8_t o_val, uint16_t *vco_freq);
int calculate_best_o_value(uint16_t vco_freq_mhz, uint8_t required_freq_mhz, uint8_t *new_o_val);
int calculate_o_reg_value(uint8_t o_val, uint8_t reg_29, uint8_t reg_2A, uint8_t *reg_29_val, uint8_t *reg_2A_val);
void calculate_tp_parameters(uint8_t tp_freq, uint8_t tp_fps, uint16_t tp_width, uint16_t tp_height, uint8_t bit_size, uint16_t *tp_front_porch, uint16_t *tp_back_porch, uint16_t *tp_h_blanking, uint32_t *tp_v_blanking);
void calculate_rx_data_settle_cyc(uint8_t byteclk_mhz, uint8_t gear, uint8_t is_queue_fifo, uint8_t *dsettle_cnt);
static int fpga_config_mipi_speed(struct i2c_client *client, uint16_t val);

struct ConfigRegisters config_registers;
struct MipiTx_TimingParams_t mipitx_parameters;
struct PllRegisters pll_reg;


uint8_t tx_ref_clk = 72;
uint8_t rx_ref_clk = 72;
uint8_t pll_ref_clk = 24;
uint8_t gear = 16;
uint8_t lanes = 2;

uint8_t ns_target = 150;

uint16_t tx_target_speed = 1188;
uint16_t rx_target_speed = 1188;
bool test_pattern = 0;

uint8_t tp_fps = 60;
uint16_t tp_width = 1920;
uint16_t tp_height = 1080;
uint8_t tx_byte_clk;
