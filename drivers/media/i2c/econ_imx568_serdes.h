/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 e-con Systems Pvt Ltd, Inc. All Rights Reserved.
 */

#ifndef __SERDES_H
#define __SERDES_H

#define SER1_ADDR \
	0x40 /* MAX9295A alias on MAX96716A (native 0x40 mapped to 0x44 by DES alias pool) */
#define DES_ADDR 0x28 /* MAX96716A on Modalix HHHLV2 */

#define PHY_A 'A'
#define PHY_B 'B'
#define PHY_C 'C'
#define PHY_D 'D'

uint8_t ser_status;

struct serdes_parse {
	uint16_t reg;
	uint8_t val;
};

struct serdes_parse SER1_CONF[] = {
	// MAX9295
	{ 0x0330, 0x00 }, { 0x0331, 0x33 }, // 4lane, 0x13 -> 2lane
	{ 0x0332, 0xE0 }, { 0x0333, 0x04 }, { 0x0334, 0x00 }, { 0x0335, 0x00 },
	{ 0x0308, 0x7D }, { 0x0311, 0x15 }, { 0x0314, 0x6C }, { 0x0315, 0x00 },
	{ 0x0313, 0x10 }, { 0x031C, 0x38 }, { 0x0002, 0x13 }, { 0x0053, 0x10 },
};

// MAX96716
struct serdes_parse DSER_CONF[] = {
	// Video Pipes And Routing Configuration
	{ 0x0161, 0x20 },
	// Pipe to Controller Mapping Configuration (Pipe Y)
	{ 0x044B, 0x07 },
	{ 0x044C, 0x00 },
	{ 0x044D, 0x2C },
	{ 0x044E, 0x2C },
	{ 0x044F, 0x00 },
	{ 0x0450, 0x00 },
	{ 0x0451, 0x01 },
	{ 0x0452, 0x01 },
	{ 0x046D, 0x2A },
	// Double Mode Configuration
	{ 0x0473, 0x01 },
	{ 0x0160, 0x01 },
	{ 0x0483, 0x00 },
	{ 0x0484, 0x00 },
	{ 0x0332, 0xC4 },
	{ 0x04B3, 0x01 },
	// MIPI D-PHY Configuration (CSI PHY 1)
	{ 0x0330, 0x04 }, // PHY 2x4 mode
	// LANE configuration 0xC0 -> 4lane, 0x50 -> 2lane
	{ 0x040A, 0xC0 },
	{ 0x044A, 0xC0 },
	{ 0x048A, 0xC0 },
	{ 0x04CA, 0xC0 },
	{ 0x0333, 0x4E },
	{ 0x0335, 0x00 },
	{ 0x1D00, 0xF4 },
	{ 0x1D00, 0xF4 },
	{ 0x0323, 0x2C }, // 1200 Mbps/lane
	{ 0x1D00, 0xF5 },
	// CSI Output Enable
	{ 0x0325, 0x80 }, // This will resolve the first frame corruption
	{ 0x0313, 0x02 },
};

struct serdes_parse SER1_I2C_CONF[] = {
	{ 0x0042, 0x86 },
	{ 0x0043, 0x84 },
};

int serdes_write_i2c(struct i2c_client *client, u16 sladdr, u8 *val, u32 count);
int serdes_read_i2c(struct i2c_client *client, u16 sladdr, u8 *val, u32 count);
s32 serdes_read_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 *val);
s32 serdes_write_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 val);
s32 serdes_read_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg,
			u8 *val);
s32 serdes_write_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg,
			 u8 val);
static s32 serdes_parse_regdata(struct i2c_client *client, struct serdes_parse *, u32,
				u8);
#endif /* __SERDES_H */
