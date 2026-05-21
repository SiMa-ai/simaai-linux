// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for econ FPGA
 * Copyright (C) 2026, e-con Systems India Pvt Ltd
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

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include "econ_kobe1_fpga.h"

struct gpio_desc *fpga_nprogram_gpio = NULL;
struct gpio_desc *fpga_nreset_gpio = NULL;

void toggle_gpio_fpga(struct gpio_desc *gpio, int val)
{
	if (gpiod_cansleep(gpio)){
		gpiod_direction_output(gpio,val);
		gpiod_set_value_cansleep(gpio, val);
	} else{
		gpiod_direction_output(gpio,val);
		gpiod_set_value(gpio, val);
	}
}

static inline struct fpga *to_fpga(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct fpga, sd);
}

static int fpga_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

//==============================================================
//		MIPI TX PLL Calculations
//==============================================================
int get_pll_coefficients(uint8_t ref_clk, uint16_t target_rate, uint8_t *best_n, uint8_t *best_m, uint8_t *best_o, uint16_t *best_r) {

	const uint8_t O_VALS[5] = {1, 2, 4, 8, 16};

	uint32_t pre_dist = 0xFFFFFFFF;
	int found_solution = 0;

	uint8_t final_n = 0;
	uint8_t final_m = 0;
	uint8_t final_o = 0;
	uint32_t final_rate = 0;

	for (uint32_t n_div = 1; n_div <= 32; n_div++) {
		if ((uint32_t)ref_clk >= (24 * n_div) && (uint32_t)ref_clk <= (50 * n_div)) {
			for (int odiv_idx = 0; odiv_idx < 5; odiv_idx++) {
				uint32_t o_div = O_VALS[odiv_idx];
				uint32_t m_div = ((uint32_t)target_rate * n_div * o_div + (uint32_t)ref_clk - 1) / (uint32_t)ref_clk;

				if (m_div >= 16 && m_div <= 255) {
					uint32_t fvco = ((uint32_t)ref_clk * m_div) / n_div;

					if (fvco >= 1250 && fvco <= 2500) {

						uint32_t cur_rate = ((uint32_t)ref_clk * m_div) / (n_div * o_div);

						if (cur_rate >= (uint32_t)target_rate) {
							uint32_t cur_dist = cur_rate - (uint32_t)target_rate;

							if (cur_dist < pre_dist) {
								pre_dist = cur_dist;

								final_n = (uint8_t)n_div;
								final_m = (uint8_t)m_div;
								final_o = (uint8_t)o_div;
								final_rate = cur_rate;

								found_solution = 1;
							}
						}
					}
				}
			}
		}
	}

	if (found_solution) {
		*best_n = final_n;
		*best_m = final_m;
		*best_o = final_o;
		*best_r = (uint16_t)final_rate;
		return 0;
	}
	return 1;
}

int get_config_registers(int CN, int CM, int CO, struct ConfigRegisters *regs) {
	uint8_t n_encoded = 0;
	uint8_t m_encoded = 0;
	uint8_t o_encoded = 0;

	static const uint8_t n_lookup[33] = {
		0x00, 0x1F, 0x00, 0x10, 0x18,
		0x1C, 0x0E, 0x07, 0x13, 0x09,
		0x04, 0x02, 0x11, 0x08, 0x14,
		0x0A, 0x15, 0x1A, 0x1D, 0x1E,
		0x0F, 0x17, 0x1B, 0x0D, 0x16,
		0x0B, 0x05, 0x12, 0x19, 0x0C,
		0x06, 0x03, 0x01
	};

	static const uint8_t o_lookup[17] = {
		0x00, 0x00, 0x01, 0x00,
		0x02, 0x00, 0x00, 0x00,
		0x03, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x07
	};

	static const uint8_t m_lookup[256] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
		0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
		0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
		0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
		0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
		0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
		0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
		0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
		0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
		0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
		0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
		0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
		0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
		0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
		0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
		0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
		0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
		0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
		0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
		0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
		0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
		0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
		0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
		0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
		0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F
	};

	if (CN >= 1 && CN <= 32)   n_encoded = n_lookup[CN];
	if (CM >= 16 && CM <= 255) m_encoded = m_lookup[CM];
	if (CO >= 1 && CO <= 16)   o_encoded = o_lookup[CO];

	regs->reg1 = ((n_encoded & 0x01) << 3) | 0x04;
	regs->reg2 = (n_encoded >> 1) & 0x0F;
	regs->reg3 = m_encoded & 0x0F;
	regs->reg4 = (m_encoded >> 4) & 0x0F;
	regs->reg5 = 0x08 | (o_encoded & 0x07);

	return 0;
}

//==============================================================
//		MIPI TX Calculations
//==============================================================

//===================== tLPX ===========================//
int calculate_tLPX(uint8_t byte_clk_mhz, uint8_t *result) {

	uint32_t val = ((uint32_t)byte_clk_mhz / 20) + 1;

	if (val > 255) {
		*result = 255;
		return 1;
	} else if (val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = (uint8_t)val;
		return 0;
	}
}
//======================================================//

//===================== tCLKPREP ===========================//
int calculate_tCLKPREP(uint8_t byte_clk_mhz, uint8_t *result) {

	uint32_t calc = ((uint32_t)byte_clk_mhz * 19) / 500;
	uint32_t val = calc + 1;

	if (val > 255) {
		*result = 255;
		return 1;
	} else if (val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = (uint8_t)val;
		return 0;
	}
}
//======================================================//

//====================== tCLK_HSZERO ========================//
int calculate_tCLK_HSZERO(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result) {

	uint32_t numerator = (uint32_t)byte_clk_mhz * 131;
	uint32_t denominator = 500;

	uint32_t clk_hszero_gui_cycles = numerator / denominator;
	uint32_t remainder = numerator % denominator;

	if (remainder > 0) {
		clk_hszero_gui_cycles = clk_hszero_gui_cycles + 1;
	}

	uint32_t final_val;

	if (gear == 16) {
		final_val = clk_hszero_gui_cycles + 1;
	} else {
		final_val = clk_hszero_gui_cycles;
	}

	if (final_val > 255) {
		*result = 255;
		return 1;
	}
	else if (final_val < 1) {
		*result = 1;
		return 1;
	}
	else {
		*result = (uint8_t)final_val;
		return 0;
	}
}
//=============================================================================//

//============================= tCLKPRE ===================================//
int calculate_tCLKPRE(uint8_t gear, uint8_t *result) {
	uint32_t calc = 8 / (uint32_t)gear;
	uint32_t rem = 8 % (uint32_t)gear;

	if (rem > 0) {
		calc = calc + 1;
	}
	uint8_t term1_ceil = (uint8_t)(calc + 1);

	uint8_t clkpre_min = (gear == 16) ? 2 : 1;
	uint8_t final_val = (term1_ceil > clkpre_min) ? (term1_ceil - clkpre_min) : 1;
	uint8_t register_min = (gear == 16) ? 2 : 1;

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < register_min) {
		*result = register_min;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//============================= tCLKPOST ===================================//
int calculate_tCLKPOST(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result) {
	uint32_t denominator = 50 * (uint32_t)gear;
	uint32_t numerator = ((uint32_t)byte_clk_mhz * 3 * (uint32_t)gear) + (52 * 50);

	uint8_t term_floor = (uint8_t)(numerator / denominator) + 1;
	uint8_t final_val = term_floor + 1;
	uint8_t register_min = (gear == 8) ? 2 : 1;

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < register_min) {
		*result = register_min;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= tCLKTRAIL ====================================//
int calculate_tCLKTRAIL(uint8_t byte_clk_mhz, uint8_t *result) {
	uint32_t term_floor = ((uint32_t)byte_clk_mhz * 3) / 50;
	uint8_t final_val = (uint8_t)term_floor + 2;

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < 2) {
		*result = 2;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= tCLKEXIT ====================================//
int calculate_tCLKEXIT(uint8_t byte_clk_mhz, uint8_t *result) {
	uint32_t term_floor = (uint32_t)byte_clk_mhz / 10;
	uint8_t final_val = (uint8_t)term_floor + 2;

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_DATPREP ====================================//
int calculate_tDATPREP(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result) {
	uint32_t denominator = 25 * (uint32_t)gear;
	uint32_t numerator = ((uint32_t)byte_clk_mhz * (uint32_t)gear) + (4 * 25);

	uint32_t calc = numerator / denominator;
	uint32_t rem = numerator % denominator;

	if (rem > 0) {
		calc = calc + 1;
	}

	uint8_t final_val = (uint8_t)calc;

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_DAT_HSZERO ====================================//
int calculate_tDAT_HSZERO(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result) {
	uint32_t denominator = 200 * (uint32_t)gear;
	uint32_t numerator = ((uint32_t)byte_clk_mhz * 21 * (uint32_t)gear) + (6 * 200);

	uint32_t term_ceil = numerator / denominator;
	uint32_t rem = numerator % denominator;

	if (rem > 0) {
		term_ceil = term_ceil + 1;
	}

	uint8_t final_val = (uint8_t)(term_ceil + 1);

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_DATTRAIL ====================================//
int calculate_tDATTRAIL(uint8_t byte_clk_mhz, uint16_t tx_line_rate_mbps, uint8_t gear, uint8_t *result) {
	uint32_t denominator = 25 * (uint32_t)tx_line_rate_mbps;
	uint32_t numerator = (uint32_t)byte_clk_mhz * ((2 * (uint32_t)tx_line_rate_mbps) + 828);

	uint32_t e_cycles = numerator / denominator;
	uint32_t rem = numerator % denominator;

	if (rem > 0) {
		e_cycles = e_cycles + 1;
	}

	uint8_t rtl_adjustment = (gear == 16) ? 1 : 2;
	uint8_t final_val = (uint8_t)(e_cycles + rtl_adjustment);

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_DATEXIT ====================================//
int calculate_tDATEXIT(uint8_t byte_clk_mhz, uint8_t *result) {
	uint32_t term_floor = (uint32_t)byte_clk_mhz / 10;
	uint8_t final_val = (uint8_t)term_floor + 2;

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_SKEWCAL_INIT ====================================//
int calculate_tSKEWCAL_INIT(uint8_t byte_clk_mhz, uint16_t tx_line_rate_mbps, uint8_t gear, uint16_t *result) {
	uint16_t final_val = (32768 / (uint16_t)gear) + 1;
	uint16_t min_bound;
	uint16_t max_bound;

	if (tx_line_rate_mbps > 1500) {
		min_bound = final_val;
		max_bound = ((uint16_t)byte_clk_mhz * 100) - 1;
	} else {
		min_bound = 1;
		max_bound = final_val;
	}

	if (final_val > max_bound) {
		*result = max_bound;
		return 1;
	} else if (final_val < min_bound) {
		*result = min_bound;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_SKEWCAL_PERIOD ====================================//
int calculate_tSKEWCAL_PERIOD(uint8_t gear, uint16_t *result) {
	uint16_t final_val = (1024 / (uint16_t)gear) + 1;
	uint16_t min_bound = 1;

	if (final_val < min_bound) {
		*result = min_bound;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_SKEWCAL_HSZERO ====================================//
int calculate_tSKEWCAL_HSZERO(uint8_t byte_clk_mhz, uint8_t gear, uint8_t *result) {
	uint32_t denominator = 200 * (uint32_t)gear;
	uint32_t numerator = ((uint32_t)byte_clk_mhz * 21 * (uint32_t)gear) + (6 * 200);

	uint32_t term_ceil = numerator / denominator;
	uint32_t rem = numerator % denominator;

	if (rem > 0) {
		term_ceil = term_ceil + 1;
	}

	uint8_t final_val = (term_ceil > 2) ? (uint8_t)(term_ceil - 2) : 0;

	if (final_val > 255) {
		*result = 255;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_CLK_SETTLE ====================================//
int calculate_tclk_settle(uint8_t syncclk_mhz, uint8_t ns_target, uint8_t *result) {
	uint32_t final_val = ((uint32_t)ns_target * (uint32_t)syncclk_mhz) / 1000;

	if (final_val > 63) {
		*result = 63;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = (uint8_t)final_val;
		return 0;
	}
}
//=============================================================================//

//================================= T_HS_SETTLE ====================================//
int calculate_tHS_SETTLE(uint8_t gear, uint8_t byteclk_mhz, uint8_t syncclk_mhz, uint8_t *result) {
	// Target NS formula: 100 + 8000 / (byteclk * gear)
	uint32_t ns_denominator = (uint32_t)byteclk_mhz * (uint32_t)gear;
	uint32_t ns_numerator = (100 * ns_denominator) + 8000;

	uint32_t t_data_settle_ns = ns_numerator / ns_denominator;
	uint32_t ns_rem = ns_numerator % ns_denominator;

	if (ns_rem > 0) {
		t_data_settle_ns = t_data_settle_ns + 1;
	}

	uint32_t final_val = (t_data_settle_ns * (uint32_t)syncclk_mhz) / 1000;

	if (final_val > 63) {
		*result = 255;
		return 1;
	} else if (final_val < 1) {
		*result = 1;
		return 1;
	} else {
		*result = (uint8_t)final_val;
		return 0;
	}
}
//=============================================================================//


int calculate_m_value(uint8_t reg_0B, uint8_t reg_0C, uint8_t reg_0D, uint8_t reg_0E, uint8_t *m_val)
{
	uint8_t m_value = 0;

	if(((reg_0B & 0x08) >> 3)){
		m_value = ((reg_0C & 0xF0) >> 4) | ((reg_0D & 0x0F) << 4);
	} else {
		m_value = (reg_0D & 0xF0) | (reg_0E & 0x0F);
	}

	*m_val = m_value;
	return 0;
}

int calculate_n_value(uint8_t reg_0B, uint8_t reg_15, uint8_t reg_16, uint8_t reg_05, uint8_t reg_06, uint16_t *n_val)
{
	uint16_t n_value = 0;

	if(((reg_0B & 0x04) >> 2)){
		n_value = ((reg_06 & 0x7F) << 1) | ((reg_05 & 0x80) >> 7) ;
	} else {
		n_value = ((reg_16 & 0x01) << 8) | (reg_15) ;
	}

	*n_val = n_value;
	return 0;
}

int calculate_f_value(uint8_t reg_0B, uint8_t reg_13, uint8_t reg_14, uint16_t *f_val)
{
	uint16_t f_value = 0;

	if(((reg_0B & 0x04) >> 2)){
		f_value = 0;
	} else {
		f_value = (reg_14 << 7) | (reg_13 >> 1) ;
	}

	*f_val = f_value;
	return 0;
}

int calculate_o_value(uint8_t reg_29, uint8_t reg_2A, uint8_t *o_val)
{
	uint8_t o_value = 0;

	o_value = ((reg_2A & 0x0F) << 3) | ((reg_29 & 0xE0) >> 5) ;

	*o_val = o_value;
	return 0;
}

int calculate_vco_frequency(uint16_t ref_clk, uint8_t m_val, uint16_t n_val, uint16_t f_val, uint8_t o_val, uint16_t *vco_freq)
{
	if (m_val == 0) {
		*vco_freq = 0;
		return -1;
	}

	uint32_t fractional_multiplier = ((uint32_t)n_val * 4096) + (uint32_t)f_val;
	uint32_t numerator = (uint32_t)ref_clk * fractional_multiplier;

	uint32_t denominator = (uint32_t)m_val * 4096;
	*vco_freq = (uint16_t)(numerator / denominator);

	return 0;
}

int calculate_best_o_value(uint16_t vco_freq_mhz, uint8_t required_freq_mhz, uint8_t *new_o_val)
{
	if (required_freq_mhz == 0) {
		*new_o_val = 127;
		return 0;
	}

	uint32_t numerator = (uint32_t)vco_freq_mhz;
	uint32_t denominator = (uint32_t)required_freq_mhz;

	uint32_t calculated_o = (numerator + (denominator / 2)) / denominator;

	if (calculated_o < 1) {
		*new_o_val = 1;
	} else if (calculated_o > 127) {
		*new_o_val = 127;
	} else {
		*new_o_val = (uint8_t)calculated_o;
	}

	return 0;
}

int calculate_o_reg_value(uint8_t o_val, uint8_t reg_29, uint8_t reg_2A, uint8_t *reg_29_val, uint8_t *reg_2A_val)
{
	*reg_29_val = ((o_val & 0x07) << 5) | (reg_29 & 0x1F);
	*reg_2A_val = (reg_2A & 0xF0) | ((o_val & 0xF8) >> 3);

	return 0;
}


void calculate_tp_parameters(uint8_t tp_freq, uint8_t tp_fps, uint16_t tp_width, uint16_t tp_height, uint8_t bit_size, uint16_t *tp_front_porch, uint16_t *tp_back_porch, uint16_t *tp_h_blanking, uint32_t *tp_v_blanking)
{
	uint32_t effective_freq_hz = (uint32_t)(tp_freq) * 1000000;

	uint32_t line_active_clocks = (uint32_t)(tp_width * 10) / bit_size;
	uint32_t h_blank_clocks     = line_active_clocks / 4;
	uint32_t total_line_clocks  = line_active_clocks + h_blank_clocks;

	uint32_t worst_case_budget = effective_freq_hz / tp_fps;

	uint32_t active_frame_cycles = total_line_clocks * tp_height;
	*tp_front_porch = 250;
	*tp_back_porch  = 250;
	uint32_t porch_overhead = *tp_front_porch + *tp_back_porch;

	if (worst_case_budget > (active_frame_cycles + porch_overhead)) {
		*tp_v_blanking = worst_case_budget - active_frame_cycles - porch_overhead;
	} else {
		*tp_v_blanking = total_line_clocks;
	}

	*tp_h_blanking = (uint16_t)h_blank_clocks;
}

void calculate_rx_data_settle_cyc(uint8_t byteclk_mhz, uint8_t gear, uint8_t is_queue_fifo, uint8_t *dsettle_cnt) {
	uint8_t dsettle_tmp = 0;
	uint8_t dsettle_cnt_internal = 0;

	if (gear == 8) {
		if (byteclk_mhz > 12 && byteclk_mhz <= 16)       dsettle_tmp = 1;
		else if (byteclk_mhz > 16 && byteclk_mhz < 26)   dsettle_tmp = 2;
		else if (byteclk_mhz >= 26 && byteclk_mhz < 33)  dsettle_tmp = 3;
		else if (byteclk_mhz >= 33 && byteclk_mhz < 40)  dsettle_tmp = 4;
		else if (byteclk_mhz >= 40 && byteclk_mhz < 50)  dsettle_tmp = 5;
		else if (byteclk_mhz >= 50 && byteclk_mhz < 55)  dsettle_tmp = 6;
		else if (byteclk_mhz >= 55 && byteclk_mhz < 65)  dsettle_tmp = 7;
		else if (byteclk_mhz >= 65 && byteclk_mhz < 70)  dsettle_tmp = 8;
		else if (byteclk_mhz >= 70 && byteclk_mhz < 80)  dsettle_tmp = 9;
		else if (byteclk_mhz >= 80 && byteclk_mhz < 89)  dsettle_tmp = 10;
		else if (byteclk_mhz >= 89 && byteclk_mhz < 100) dsettle_tmp = 11;
		else if (byteclk_mhz >= 100 && byteclk_mhz < 112) dsettle_tmp = 12;
		else if (byteclk_mhz >= 112 && byteclk_mhz < 125) dsettle_tmp = 13;
		else if (byteclk_mhz >= 125 && byteclk_mhz < 150) dsettle_tmp = 14;
		else if (byteclk_mhz >= 150 && byteclk_mhz < 156) dsettle_tmp = 15;
		else if (byteclk_mhz >= 156 && byteclk_mhz < 167) dsettle_tmp = 16;
		else if (byteclk_mhz >= 167 && byteclk_mhz < 179) dsettle_tmp = 17;
		else if (byteclk_mhz >= 179 && byteclk_mhz < 190) dsettle_tmp = 18;
		else if (byteclk_mhz >= 190 && byteclk_mhz < 210) dsettle_tmp = 19;
		else if (byteclk_mhz >= 210 && byteclk_mhz < 220) dsettle_tmp = 20;
		else if (byteclk_mhz >= 220)                     dsettle_tmp = 21;
	} else {
		if (byteclk_mhz >= 9 && byteclk_mhz < 19)        dsettle_tmp = 1;
		else if (byteclk_mhz >= 19 && byteclk_mhz < 31)  dsettle_tmp = 2;
		else if (byteclk_mhz >= 31 && byteclk_mhz < 41)  dsettle_tmp = 3;
		else if (byteclk_mhz >= 41 && byteclk_mhz < 50)  dsettle_tmp = 4;
		else if (byteclk_mhz >= 50 && byteclk_mhz < 60)  dsettle_tmp = 5;
		else if (byteclk_mhz >= 60 && byteclk_mhz < 70)  dsettle_tmp = 6;
		else if (byteclk_mhz >= 70 && byteclk_mhz < 80)  dsettle_tmp = 7;
		else if (byteclk_mhz >= 80 && byteclk_mhz < 85)  dsettle_tmp = 8;
		else if (byteclk_mhz >= 85 && byteclk_mhz < 93)  dsettle_tmp = 9;
		else if (byteclk_mhz >= 93 && byteclk_mhz < 105) dsettle_tmp = 10;
		else if (byteclk_mhz >= 105 && byteclk_mhz < 113) dsettle_tmp = 11;
		else if (byteclk_mhz >= 113 && byteclk_mhz < 125) dsettle_tmp = 12;
		else if (byteclk_mhz >= 125 && byteclk_mhz < 136) dsettle_tmp = 13;
		else if (byteclk_mhz >= 136)                     dsettle_tmp = 14;
	}

	dsettle_cnt_internal = dsettle_tmp;
	/*
	   if (dsettle_tmp < 3) {
	   dsettle_cnt_internal = 0;
	   } else {
	   dsettle_cnt_internal = dsettle_tmp - 3;
	   }
	   */

	if (byteclk_mhz <= 30 && is_queue_fifo) {
		*dsettle_cnt = dsettle_cnt_internal + 3;
	} else {
		*dsettle_cnt = dsettle_cnt_internal;
	}
}

static int fpga_config_mipi_speed(struct i2c_client *client, uint16_t val)
{
	uint8_t CN;
	uint8_t CM;
	uint8_t CO;

	uint8_t tclk_settle;
	uint8_t ths_settle;

	uint16_t tx_target_speed = val;
	uint16_t tx_actual_speed = 0;

	uint8_t data8b = 0;
	uint16_t rx_target_speed = tx_target_speed;
	uint8_t rx_speed_control0;
	uint8_t rx_speed_control1;
	uint8_t rx_byte_clk = rx_target_speed/gear;
	uint8_t tclk_settle0;
	uint8_t tclk_settle1;
	uint8_t tclk_settle2;
	uint8_t ths_settle0;
	uint8_t ths_settle1;
	uint8_t required_pll_freq = test_pattern ? tx_byte_clk : rx_byte_clk ;

	uint16_t n_val;
	uint16_t f_val;
	uint8_t m_val;
	uint8_t o_val;
	uint16_t vco_freq;
	uint8_t new_o_val;
	uint8_t reg_29_val;
	uint8_t reg_2A_val;

	// TX Speed
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x08, tx_target_speed & 0xFF);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x09, tx_target_speed >> 8);

	get_pll_coefficients(tx_ref_clk, tx_target_speed, &CN, &CM, &CO, &tx_actual_speed);

	tx_byte_clk = (tx_actual_speed / gear) ;
	get_config_registers(CN, CM, CO, &config_registers);

	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0A, tx_actual_speed & 0xFF);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0B, tx_actual_speed >> 8);

	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0A, &data8b);
	tx_actual_speed = data8b;
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0B, &data8b);
	tx_actual_speed = tx_actual_speed | (data8b << 8);

	printk("FPGA TX actual speed: %d\n", tx_actual_speed);

	//config_registers.reg1 -> TX PLL Parameter 1 -> 0x20
	//config_registers.reg2 -> TX PLL Parameter 2 -> 0x21
	//config_registers.reg3 -> TX PLL Parameter 3 -> 0x22
	//config_registers.reg4 -> TX PLL Parameter 4 -> 0x23
	//config_registers.reg5 -> TX PLL Parameter 5 -> 0x24

	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x20, config_registers.reg1);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x21, config_registers.reg2);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x22, config_registers.reg3);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x23, config_registers.reg4);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x24, config_registers.reg5);

	calculate_tLPX(tx_byte_clk, &mipitx_parameters.t_lpx);
	calculate_tCLKPREP(tx_byte_clk, &mipitx_parameters.t_clkprep);
	calculate_tCLK_HSZERO(tx_byte_clk, gear, &mipitx_parameters.t_clk_hszero);
	calculate_tCLKPRE(gear,&mipitx_parameters.t_clkpre);
	calculate_tCLKPOST(tx_byte_clk, gear, &mipitx_parameters.t_clkpost);
	calculate_tCLKTRAIL(tx_byte_clk, &mipitx_parameters.t_clktrail);
	calculate_tCLKEXIT(tx_byte_clk, &mipitx_parameters.t_clkexit);
	calculate_tDATPREP(tx_byte_clk, gear, &mipitx_parameters.t_datprep);
	calculate_tDAT_HSZERO(tx_byte_clk, gear, &mipitx_parameters.t_dat_hszero);
	calculate_tDATTRAIL(tx_byte_clk, tx_target_speed, gear, &mipitx_parameters.t_dattrail);
	calculate_tDATEXIT(tx_byte_clk, &mipitx_parameters.t_datexit);
	calculate_tSKEWCAL_INIT(tx_byte_clk, tx_target_speed, gear, &mipitx_parameters.t_skewcal_init);
	calculate_tSKEWCAL_PERIOD(gear,&mipitx_parameters.t_skewcal_period);
	calculate_tSKEWCAL_HSZERO(tx_byte_clk, gear, &mipitx_parameters.t_skewcal_hszero);

	// mipitx_parameters.t_lpx                   -> TX_LPX              -> 0x25
	// mipitx_parameters.t_clkprep               -> TX_CLKPREP          -> 0x26
	// mipitx_parameters.t_clk_hszero            -> TX_CLK_HSZERO       -> 0x27
	// mipitx_parameters.t_clkpre                -> TX_CLKPRE           -> 0x28
	// mipitx_parameters.t_clkpost               -> TX_CLKPOST          -> 0x29
	// mipitx_parameters.t_clktrail              -> TX_CLKTRAIL         -> 0x2A
	// mipitx_parameters.t_clkexit               -> TX_CLKEXIT          -> 0x2B
	// mipitx_parameters.t_datprep               -> TX_DATPREP          -> 0x2C
	// mipitx_parameters.t_dat_hszero            -> TX_DATA_HSZERO      -> 0x2D
	// mipitx_parameters.t_dattrail              -> TX_DATTRAIL         -> 0x2E
	// mipitx_parameters.t_datexit               -> TX_DATEXIT          -> 0x2F
	// mipitx_parameters.t_skewcal_init & 0xFF   -> TX_SKEWCAL_INIT_0   -> 0x30
	// mipitx_parameters.t_skewcal_init >> 8     -> TX_SKEWCAL_INIT_1   -> 0x31
	// mipitx_parameters.t_skewcal_period & 0xFF -> TX_SKEWCAL_PERIOD_0 -> 0x32
	// mipitx_parameters.t_skewcal_period >> 8   -> TX_SKEWCAL_PERIOD_1 -> 0x33
	// mipitx_parameters.t_skewcal_hszero        -> TX_SKEWCAL_HSZERO   -> 0x34



	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x25, mipitx_parameters.t_lpx);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x26, mipitx_parameters.t_clkprep);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x27, mipitx_parameters.t_clk_hszero);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x28, mipitx_parameters.t_clkpre);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x29, mipitx_parameters.t_clkpost);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x2A, mipitx_parameters.t_clktrail);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x2B, mipitx_parameters.t_clkexit);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x2C, mipitx_parameters.t_datprep);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x2D, mipitx_parameters.t_dat_hszero);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x2E, mipitx_parameters.t_dattrail);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x2F, mipitx_parameters.t_datexit);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x30, (mipitx_parameters.t_skewcal_init & 0xFF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x31, (mipitx_parameters.t_skewcal_init >> 8));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x32, (mipitx_parameters.t_skewcal_period & 0xFF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x33, (mipitx_parameters.t_skewcal_period >> 8));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x34, mipitx_parameters.t_skewcal_hszero);
	// TX speed

	// RX speed
	calculate_tclk_settle(rx_ref_clk, ns_target, &tclk_settle);
	calculate_tHS_SETTLE(gear, rx_byte_clk, rx_ref_clk, &ths_settle);

	rx_speed_control0 = (rx_target_speed <= 1500) ? 2 : 0 ;
	rx_speed_control1 = (rx_target_speed <= 1500) ? 8 : 0 ;

	tclk_settle0 = (((tclk_settle & 1)<<3) | ((lanes-1)<<1));
	tclk_settle1 = ((tclk_settle >> 1) & 15);
	tclk_settle2 = ((tclk_settle >> 5) & 1);
	ths_settle0  = ((ths_settle & 3) << 2);
	ths_settle1  = (ths_settle >> 2);

	// tclk_settle0      -> RX_CLK_SETTLE [7:0]     -> 0x50
	// tclk_settle1      -> RX_CLK_SETTLE [15:8]    -> 0x51
	// tclk_settle2      -> RX_CLK_SETTLE [23:16]   -> 0x52
	// ths_settle0       -> RX_HS_SETTLE [7:0]      -> 0x53
	// ths_settle1       -> RX_HS_SETTLE [15:8]     -> 0x54
	// rx_speed_control0 -> RX_SPEED_CONTROL [7:0]  -> 0x55
	// rx_speed_control1 -> RX_SPEED_CONTROL [15:8] -> 0x56

	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x38, rx_target_speed & 0x00FF);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x39, rx_target_speed >> 8);

	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x50, tclk_settle0);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x51, tclk_settle1);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x52, tclk_settle2);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x53, ths_settle0);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x54, ths_settle1);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x55, rx_speed_control0);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x56, rx_speed_control1);

	// Update shadow register
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x07, 0x1);

	// Delay required after writing shadow register
	msleep(50);

	return 0;
}

static int configure_test_pattern_mode(struct i2c_client *client, uint8_t mode)
{
	uint8_t tp_freq = tx_byte_clk;

	uint16_t tp_front_porch;
	uint16_t tp_back_porch;
	uint16_t tp_h_blanking;
	uint32_t tp_v_blanking;
	uint8_t bit_size = 32;

	// Configure test pattern MIPI speed
	fpga_config_mipi_speed(client, fpga_tp_modes[mode].tp_speed);

	tp_freq = tx_byte_clk;
	calculate_tp_parameters(tp_freq, fpga_tp_modes[mode].tp_fps, fpga_tp_modes[mode].tp_width, fpga_tp_modes[mode].tp_height, bit_size, &tp_front_porch, &tp_back_porch, &tp_h_blanking, &tp_v_blanking);

	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x60, fpga_tp_modes[mode].tp_fps);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x62, (fpga_tp_modes[mode].tp_width & 0x00FF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x63, (fpga_tp_modes[mode].tp_width >> 8));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x64, (fpga_tp_modes[mode].tp_height & 0x00FF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x65, (fpga_tp_modes[mode].tp_height >> 8));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x66, (tp_front_porch & 0x00FF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x67, (tp_front_porch >> 8));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x68, (tp_back_porch & 0x00FF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x69, (tp_back_porch >> 8));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x6A, (tp_h_blanking & 0x00FF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x6B, (tp_h_blanking >> 8));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x6C, (tp_v_blanking & 0x000000FF));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x6D, (tp_v_blanking >> 8) & 0x000000FF);
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x6E, (((tp_v_blanking >> 16) & 0x000000FF)));
	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x6F, (((tp_v_blanking >> 24) & 0x000000FF)));

	fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x07, 0x1);

	return 0;
}
static void read_fpga_tx_status(struct i2c_client * client)
{
	uint8_t DT_1, DT_2, DT_3, DT_4;
	uint8_t tx_fifo_overflow;
	uint8_t data8b;
	uint8_t tx_fps;
	uint16_t tx_lc; // TX line Count
	uint32_t tx_wc; // TX Word Count

	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1B, &DT_1);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1C, &DT_2);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1D, &DT_3);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1E, &DT_4);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1A, &tx_fifo_overflow);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x13, &tx_fps);

	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x14, &data8b);
	tx_lc = data8b & 0xFF;
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x15, &data8b);
	tx_lc = tx_lc | (data8b << 8);

	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x16, &data8b);
	tx_wc = data8b & 0xFF;
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x17, &data8b);
	tx_wc = tx_wc | (data8b << 8);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x18, &data8b);
	tx_wc = tx_wc | (data8b << 16);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x19, &data8b);
	tx_wc = tx_wc | (data8b << 24);

	printk("1st unique data type: 0x%X\n", DT_1);
	printk("2nd unique data type: 0x%X\n", DT_2);
	printk("3rd unique data type: 0x%X\n", DT_3);
	printk("4th unique data type: 0x%X\n", DT_4);
	printk("TX FPS: %d\n", tx_fps);
	printk("TX Overflow: %d\n", tx_fifo_overflow);
	printk("TX Line count: 0x%X\n", tx_lc);
	printk("TX Word count: 0x%X\n", tx_wc);
}

static void read_fpga_rx_status(struct i2c_client *client)
{
	uint8_t DT_1, DT_2, DT_3, DT_4, DT_5, DT_6, DT_7, DT_8;
	uint8_t rx_crc, rx_ecc;
	uint8_t data8b;
	uint8_t rx_fps;
	uint16_t rx_lc; // RX line Count
	uint32_t rx_wc; // RX Word Count

	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3B, &DT_1);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3C, &DT_2);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3D, &DT_3);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3E, &DT_4);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3F, &DT_5);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x40, &DT_6);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x41, &DT_7);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x42, &DT_8);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x43, &rx_crc);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x44, &rx_ecc);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x45, &rx_fps);

	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x46, &data8b);
	rx_lc = data8b & 0xFF;
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x47, &data8b);
	rx_lc = rx_lc | (data8b << 8);

	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x48, &data8b);
	rx_wc = data8b & 0xFF;
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x49, &data8b);
	rx_wc = rx_wc | (data8b << 8);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x4A, &data8b);
	rx_wc = rx_wc | (data8b << 16);
	fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x4B, &data8b);
	rx_wc = rx_wc | (data8b << 24);

	printk("1st unique data type: 0x%X\n", DT_1);
	printk("2nd unique data type: 0x%X\n", DT_2);
	printk("3rd unique data type: 0x%X\n", DT_3);
	printk("4th unique data type: 0x%X\n", DT_4);
	printk("5th unique data type: 0x%X\n", DT_5);
	printk("6th unique data type: 0x%X\n", DT_6);
	printk("7th unique data type: 0x%X\n", DT_7);
	printk("8th unique data type: 0x%X\n", DT_8);
	printk("RX CRC: %d\n", rx_crc);
	printk("RX ECC: %d\n", rx_ecc);
	printk("RX FPS: %d\n", rx_fps);
	printk("RX Line count: 0x%X\n", rx_lc);
	printk("RX Word count: 0x%X\n", rx_wc);
}

static int fpga_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct fpga *fpga =
		container_of(ctrl->handler, struct fpga, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&fpga->sd);
	int ret = 0;
	uint8_t data8b;

	switch (ctrl->id) {
		case V4L2_CID_FPGA_MIPI_TX_CLK_MODE:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0C, &data8b);
			ctrl->val = data8b;
			break;

		case V4L2_CID_FPGA_MIPI_RX_CLK_MODE:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3A, &data8b);
			ctrl->val = data8b;
			break;

		case V4L2_CID_FPGA_MIPI_SPEED:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0A, &data8b);
			ctrl->val = data8b & 0xFF;
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0B, &data8b);
			ctrl->val = ctrl->val | (data8b << 8);
			break;

		case V4L2_CID_FPGA_MIPI_FILTER_DT1:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0D, &data8b);
			ctrl->val = data8b;
			break;

		case V4L2_CID_FPGA_MIPI_FILTER_DT2:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0E, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_MIPI_FILTER_DT3:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0F, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_MIPI_FILTER_DT4:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x10, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_TPG_MODE:
			ctrl->val = fpga->curr_tp_mode;
			break;
		case V4L2_CID_FPGA_TPG_ENABLE:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x05, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_READ_RX_STATUS:
			read_fpga_rx_status(client);
			break;
		case V4L2_CID_FPGA_READ_TX_STATUS:
			read_fpga_tx_status(client);
			break;
		case V4L2_CID_FPGA_MIPI_DT_FILTER_FIRST:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x11, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_MIPI_DT_FILTER_LAST:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x12, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT1:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3B, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT2:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3C, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT3:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3D, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT4:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3E, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT5:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x3F, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT6:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x40, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT7:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x41, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_DT8:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x42, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_FPS:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x45, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_CRC:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x43, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_ECC:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x44, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_RX_READ_LC:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x46, &data8b);
			ctrl->val = data8b & 0xFF;
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x47, &data8b);
			ctrl->val = ctrl->val | (data8b << 8);
			break;
		case V4L2_CID_FPGA_RX_READ_WC:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x48, &data8b);
			ctrl->val = data8b & 0xFF;
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x49, &data8b);
			ctrl->val = ctrl->val | (data8b << 8);
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x4A, &data8b);
			ctrl->val = ctrl->val | (data8b << 16);
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x4B, &data8b);
			ctrl->val = ctrl->val | (data8b << 24);
			break;
		case V4L2_CID_FPGA_TX_READ_DT1:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1B, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_TX_READ_DT2:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1C, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_TX_READ_DT3:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1D, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_TX_READ_DT4:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1E, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_TX_READ_FPS:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x13, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_TX_READ_FIFO_OVERFLOW:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x1A, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_TX_READ_LC:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x14, &data8b);
			ctrl->val = data8b & 0xFF;
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x15, &data8b);
			ctrl->val = ctrl->val | (data8b << 8);
			break;
		case V4L2_CID_FPGA_TX_READ_WC:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x16, &data8b);
			ctrl->val = data8b & 0xFF;
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x17, &data8b);
			ctrl->val = ctrl->val | (data8b << 8);
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x18, &data8b);
			ctrl->val = ctrl->val | (data8b << 16);
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x19, &data8b);
			ctrl->val = ctrl->val | (data8b << 24);
			break;
		case V4L2_CID_FPGA_TX_STREAMING_CTRL:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x03, &data8b);
			ctrl->val = data8b;
			break;
		case V4L2_CID_FPGA_VC_SELECT:
			fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x04, &data8b);
			ctrl->val = data8b;
			break;
		default:
			return -EINVAL;
	}
	return ret;
}

static int fpga_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct fpga *fpga =
		container_of(ctrl->handler, struct fpga, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&fpga->sd);
	int ret = 0;

	switch (ctrl->id) {
		case V4L2_CID_FPGA_MIPI_TX_CLK_MODE:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0C, ctrl->val);
			// Update shadow register
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x07, 0x1);
			break;

		case V4L2_CID_FPGA_MIPI_RX_CLK_MODE:
			// Write MIPI Speed registers and then clock mode
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x3A, ctrl->val);
			// Update shadow register
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x07, 0x1);
			break;

		case V4L2_CID_FPGA_MIPI_SPEED:
			fpga_config_mipi_speed(client, ctrl->val);
			break;

		case V4L2_CID_FPGA_MIPI_FILTER_DT1:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0D, ctrl->val);
			break;

		case V4L2_CID_FPGA_MIPI_FILTER_DT2:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0E, ctrl->val);
			break;
		case V4L2_CID_FPGA_MIPI_FILTER_DT3:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0F, ctrl->val);
			break;
		case V4L2_CID_FPGA_MIPI_FILTER_DT4:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x10, ctrl->val);
			break;
		case V4L2_CID_FPGA_TPG_MODE:
			configure_test_pattern_mode(client, ctrl->val);
			fpga->curr_tp_mode = ctrl->val;
			break;
		case V4L2_CID_FPGA_TPG_ENABLE:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x05, ctrl->val);
			break;
		case V4L2_CID_FPGA_READ_RX_STATUS:
			//read_fpga_rx_status(client);
			break;
		case V4L2_CID_FPGA_READ_TX_STATUS:
			//read_fpga_tx_status(client);
			break;
		case V4L2_CID_FPGA_MIPI_DT_FILTER_FIRST:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x11, ctrl->val);
			break;
		case V4L2_CID_FPGA_MIPI_DT_FILTER_LAST:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x12, ctrl->val);
			break;
		case V4L2_CID_FPGA_TX_STREAMING_CTRL:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x03, ctrl->val);
			break;
		case V4L2_CID_FPGA_VC_SELECT:
			fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x04, ctrl->val);
			break;
		default:
			return -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops fpga_ctrl_ops = {
	.s_ctrl = fpga_set_ctrl,
	.g_volatile_ctrl = fpga_get_ctrl,
};

static const struct v4l2_subdev_video_ops fpga_video_ops = {
	//.s_stream = fpga_set_stream,
};

static const struct v4l2_subdev_ops fpga_subdev_ops = {
	.video = &fpga_video_ops,
};

static const struct v4l2_subdev_internal_ops fpga_internal_ops = {
	.open = fpga_open,
};


static const struct of_device_id fpga_dt_ids[] = {
	{ .compatible = "econ,kobe1_fpga"},
	{ /* sentinel */ }
};

static const struct v4l2_ctrl_config fpga_mipi_ctrls[] = {
	{ // MIPI Clock Mode
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_TX_CLK_MODE,
		.name = "MIPI TX Clock",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0, // Non-continuous
		.max = 1, // Continuous
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI Data rate Mbps/lane
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_SPEED,
		.name = "MIPI Speed",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 2500,
		.step = 1,
		.def = 1188,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI Data type filter 1
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_FILTER_DT1,
		.name = "MIPI Filter DT1",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0x92,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI Data type filter 2
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_FILTER_DT2,
		.name = "MIPI Filter DT2",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0x93,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI Data type filter 3
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_FILTER_DT3,
		.name = "MIPI Filter DT3",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0x7F,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI Data type filter 4
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_FILTER_DT4,
		.name = "MIPI Filter DT4",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0x7F,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI FPGA TPG Mode
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TPG_MODE,
		.name = "MIPI TPG Mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 9,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI TPG Enable
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TPG_ENABLE,
		.name = "MIPI TPG Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // FPGA RX Status
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_READ_RX_STATUS,
		.name = "FPGA RX Status",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 255,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // FPGA TX status
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_READ_TX_STATUS,
		.name = "FPGA TX Status",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 255,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI Data type range filter first
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_DT_FILTER_FIRST,
		.name = "MIPI DT Range Filter First",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0xB0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI data type range filter last
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_DT_FILTER_LAST,
		.name = "MIPI DT Range Filter Last",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0x3F,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{ // MIPI RX Clock Mode
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_MIPI_RX_CLK_MODE,
		.name = "MIPI RX Clock",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0, // Non-Continuous
		.max = 1, // Continuous
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	// RX status read
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT1,
		.name = "MIPI RX DT1",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT2,
		.name = "MIPI RX DT2",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT3,
		.name = "MIPI RX DT3",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT4,
		.name = "MIPI RX DT4",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT5,
		.name = "MIPI RX DT5",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT6,
		.name = "MIPI RX DT6",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT7,
		.name = "MIPI RX DT7",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_DT8,
		.name = "MIPI RX DT8",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_FPS,
		.name = "MIPI RX FPS",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_CRC,
		.name = "MIPI RX CRC",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_ECC,
		.name = "MIPI RX ECC",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_LC,
		.name = "MIPI RX Linecount",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFFFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_RX_READ_WC,
		.name = "MIPI RX Wordcount",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFFFFFFFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	// FPGA TX Status
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_DT1,
		.name = "MIPI TX DT1",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_DT2,
		.name = "MIPI TX DT2",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_DT3,
		.name = "MIPI TX DT3",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_DT4,
		.name = "MIPI TX DT4",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_FPS,
		.name = "MIPI TX FPS",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_FIFO_OVERFLOW,
		.name = "MIPI TX FIFO OVERFLOW",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_LC,
		.name = "MIPI TX Linecount",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFFFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_READ_WC,
		.name = "MIPI TX Wordcount",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFFFFFFFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_TX_STREAMING_CTRL,
		.name = "MIPI TX Streaming",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	// VC select - to see the respective VC's TX and RX stats
	{
		.ops = &fpga_ctrl_ops,
		.id = V4L2_CID_FPGA_VC_SELECT,
		.name = "MIPI VC Select",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 4,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
};

static int fpga_mipi_init (struct i2c_client *client, struct fpga *fpga)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;

	uint32_t rx_mipi_filter_dt1 = 0, rx_mipi_filter_dt2 = 0, rx_mipi_filter_dt3 = 0;
	uint32_t rx_mipi_filter_dt4 = 0;
	uint32_t rx_mipi_speed = 0;
	uint32_t rx_mipi_clockmode = 0;
	uint32_t tx_mipi_clockmode = 0;
	int ret = 0;

	ret = of_property_read_u32(node, "mipi_tx_clockmode", &tx_mipi_clockmode);
	if (ret < 0) {
	    dev_err(dev, "Error in getting mipi tx clock mode\n");
	}

	ret = of_property_read_u32(node, "mipi_speed", &rx_mipi_speed);
	if (ret < 0) {
	    dev_err(dev, "Error in getting mipi speed\n");
	}

	// Configure TX clock mode
	ret = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0C, tx_mipi_clockmode);
	if (ret < 0) {
		return ret;
	}

	ret = of_property_read_u32(node, "mipi_rx_clockmode", &rx_mipi_clockmode);
	if (ret < 0) {
	    dev_err(dev, "Error in getting mipi rx clock mode\n");
	}

	// Configure RX clock mode
	ret = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x3A, rx_mipi_clockmode);
	if (ret < 0) {
		return ret;
	}

	// Configure RX & TX MIPI datarate
	ret = fpga_config_mipi_speed(client, rx_mipi_speed);
	if (ret < 0) {
		return ret;
	}

	// Configure MIPI data type filters
	ret = of_property_read_u32(node, "mipi_filter_dt1", &rx_mipi_filter_dt1);
	if (ret < 0) {
	    dev_err(dev, "Error in getting mipi filter dt1\n");
	}

	ret = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0D, rx_mipi_filter_dt1);
	if (ret < 0) {
		return ret;
	}

	ret = of_property_read_u32(node, "mipi_filter_dt2", &rx_mipi_filter_dt2);
	if (ret < 0) {
	    dev_err(dev, "Error in getting mipi filter dt2\n");
	}

	ret = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0E, rx_mipi_filter_dt2);
	if (ret < 0) {
		return ret;
	}

	ret = of_property_read_u32(node, "mipi_filter_dt3", &rx_mipi_filter_dt3);
	if (ret < 0) {
	    dev_err(dev, "Error in getting mipi filter dt3\n");
	}

	ret = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0F, rx_mipi_filter_dt3);
	if (ret < 0) {
		return ret;
	}

	ret = of_property_read_u32(node, "mipi_filter_dt4", &rx_mipi_filter_dt4);
	if (ret < 0) {
	    dev_err(dev, "Error in getting mipi filter dt4\n");
	}

	ret = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x10, rx_mipi_filter_dt4);
	if (ret < 0) {
		return ret;
	}

	// Update shadow register
	ret = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x07, 0x1);
	if (ret < 0)
		return ret;

	// Delay required after writing shadow register
	msleep(50);

	return 0;
}
/* Initialize control handlers */
static int fpga_init_controls(struct fpga *fpga)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	unsigned int i;
	int ret;

	ctrl_hdlr = &fpga->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	for (i=0; i < ARRAY_SIZE(fpga_mipi_ctrls); i++) {
		v4l2_ctrl_new_custom(&fpga->ctrl_handler,
				&fpga_mipi_ctrls[i],
				NULL);
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		printk( "%s control init failed (%d)\n",
				__func__, ret);
		goto error;
	}
	fpga->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}
static int fpga_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct fpga *fpga;
	int ret = 0, err = 0;

	struct gpio_desc *fpga_sid_sel0 = NULL;
	struct gpio_desc *fpga_sid_sel1 = NULL;
	uint8_t fpga_major_ver = 0, fpga_minor_ver = 0;

	fpga_nprogram_gpio = devm_gpiod_get(dev, "fpga_nprogram", GPIOD_OUT_LOW);
	if (IS_ERR(fpga_nprogram_gpio))
        return dev_err_probe(dev, PTR_ERR(fpga_nprogram_gpio), "Failed to get FPGA nprogram GPIO\n");

	fpga_nreset_gpio = devm_gpiod_get(dev, "fpga_nreset", GPIOD_OUT_LOW);
	if (IS_ERR(fpga_nreset_gpio))
        return dev_err_probe(dev, PTR_ERR(fpga_nreset_gpio), "Failed to get FPGA nreset GPIO\n");

	fpga_sid_sel0 = devm_gpiod_get(dev, "fpga_slaveid_sel0", GPIOD_OUT_LOW);
	if(IS_ERR(fpga_sid_sel0))
		return dev_err_probe(dev, PTR_ERR(fpga_sid_sel0), "Failed to get FPGA slave ID select0 GPIO\n");

	fpga_sid_sel1 = devm_gpiod_get(dev, "fpga_slaveid_sel1", GPIOD_OUT_LOW);
	if(IS_ERR(fpga_sid_sel1))
		return dev_err_probe(dev, PTR_ERR(fpga_sid_sel1), "Failed to get FPGA slave ID select1 GPIO\n");

	fpga = devm_kzalloc(dev, sizeof(*fpga), GFP_KERNEL);
	if (!fpga)
		return -ENOMEM;

	fpga->i2c_client = client;

	/* FPGA I2C slave address select
	 * Sel0 Sel1 Slave ID
	 * 0    0    0x0C
	 * 0    1    0x0D
	 * 1    0    0x0E
	 * 1    1    0x0F
	 * Currently it is fixed to 00 - 0x0C, others are not tested
	 */
	toggle_gpio_fpga(fpga_sid_sel0, 0);
	msleep(10);
	toggle_gpio_fpga(fpga_sid_sel1, 0);
	msleep(10);

	// nProgram Pin
	toggle_gpio_fpga(fpga_nprogram_gpio, 0);
	msleep(10);
	toggle_gpio_fpga(fpga_nprogram_gpio, 1);
	msleep(1000);

	// nReset Pin
	toggle_gpio_fpga(fpga_nreset_gpio, 0);
	msleep(10);
	toggle_gpio_fpga(fpga_nreset_gpio, 1);

	err = fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x01, &fpga_major_ver);
	if (err)
	{
		dev_err(&client->dev, "Unable to read FPGA Major FW Version %s(%d)",__func__, __LINE__);
		goto fpga_fw_update;
	}
	err = fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x02, &fpga_minor_ver);
	if (err)
	{
		dev_err(&client->dev, "Unable to read FPGA Minor FW Version %s(%d)",__func__, __LINE__);
		goto fpga_fw_update;
	}

	dev_info(&client->dev, "Econ Kobe1 FPGA Firmware version: V%d.%d\n",fpga_major_ver, fpga_minor_ver);

	if(fpga_major_ver != FPGA_MAJOR_FW_VERSION || fpga_minor_ver != FPGA_MINOR_FW_VERSION)
	{
fpga_fw_update:
		dev_info(&client->dev, "FPGA Firmware version Mismatch, Programming FPGA via I2C\n");

		/* Assreting PROGRAMN Pin to LOW for I2C Boot */
		dev_info(&client->dev, "Asserting FPGA_nPROGRAM low");
		gpio_direction_output(fpga_nprogram_gpio,0);
		msleep(100);

		err = fpga_init(client);
		if(err != 0)
		{
			dev_err(&client->dev, "FPGA Init Failed - %d\n", err);
			return -ENODEV;
		}
		else
			dev_info(&client->dev, "FPGA Booted Successfully\n");

		msleep(100);

		dev_info(&client->dev, "Writing Data for SPI Boot\n");
		err =  write_spi_data_to_fpga(client);
		if(err)
		{
			dev_err(&client->dev, "FPGA SPI Data Write Failed - %d\n", err);

			toggle_gpio_fpga(fpga_nprogram_gpio, 1);
			msleep(100);
			return -EINVAL;
		}
		dev_info(&client->dev, "Spi Write Completed\n");

		toggle_gpio_fpga(fpga_nprogram_gpio, 1);
		msleep(100);
		toggle_gpio_fpga(fpga_nprogram_gpio, 0);
		msleep(10);
		toggle_gpio_fpga(fpga_nprogram_gpio, 1);
		msleep(1000);

		fpga_major_ver = 0;
		fpga_minor_ver = 0;
		err = fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0001, &fpga_major_ver);
		if (err)
		{
			dev_err(&client->dev, "Unable to read FPGA Major FW Version %s(%d)",__func__, __LINE__);
			return err;
		}
		err = fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0002, &fpga_minor_ver);
		if (err)
		{
			dev_err(&client->dev, "Unable to read FPGA Minor FW Version %s(%d)",__func__, __LINE__);
			return err;
		}

		dev_info(&client->dev, "Econ Kobe1 FPGA Firmware version: V%d.%d\n",fpga_major_ver, fpga_minor_ver);
	}

	v4l2_i2c_subdev_init(&fpga->sd, client, &fpga_subdev_ops);

	ret = fpga_init_controls(fpga);
	if (ret)
		return ret;

	fpga->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	fpga->sd.internal_ops = &fpga_internal_ops;
	fpga->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	fpga->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&fpga->sd.entity, 1, &fpga->pad);
	if (ret)
		goto err_ctrl_init;

	/* Register v4l2_device */
	ret = v4l2_device_register(dev, &fpga->v4l2_dev);
	if (ret)
		goto err_entity_init;

	ret = v4l2_device_register_subdev(&fpga->v4l2_dev, &fpga->sd);
	if (ret)
		goto err_v4l2_register;

	ret = v4l2_device_register_subdev_nodes(&fpga->v4l2_dev);
	if (ret)
		goto err_subdev_register;

	// FPGA Configuration
	ret = fpga_mipi_init(client, fpga);
	if (ret)
		return ret;

	dev_info(dev, "Detected econ_FPGA\n");
	return 0;

err_subdev_register:
	v4l2_device_unregister_subdev(&fpga->sd);
err_v4l2_register:
	v4l2_device_unregister(&fpga->v4l2_dev);
err_entity_init:
	media_entity_cleanup(&fpga->sd.entity);
err_ctrl_init:
	v4l2_ctrl_handler_free(&fpga->ctrl_handler);
	return ret;
}


static void fpga_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct fpga *fpga = to_fpga(sd);

	if (!fpga)
		return;

	// Unregister subdev from v4l2_device
	v4l2_device_unregister_subdev(&fpga->sd);

	// Unregister v4l2_device
	v4l2_device_unregister(&fpga->v4l2_dev);

	// Cleanup media entity
	media_entity_cleanup(&fpga->sd.entity);

	// Cleanup control handler
	v4l2_ctrl_handler_free(&fpga->ctrl_handler);
}

MODULE_DEVICE_TABLE(of, fpga_dt_ids);

static struct i2c_driver fpga_i2c_driver = {
	.driver = {
		.name = "fpga",
		.of_match_table	= fpga_dt_ids,
	},
	.probe = fpga_probe,
	.remove = fpga_remove,
};

module_i2c_driver(fpga_i2c_driver);

MODULE_AUTHOR("Kishore Kumar <kishore.kumar@e-consystems.com>");
MODULE_DESCRIPTION("e-con FPGA driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.3");
