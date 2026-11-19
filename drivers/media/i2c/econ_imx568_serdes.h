/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 e-con Systems Pvt Ltd, Inc. All Rights Reserved.
 */

#ifndef __SERDES_H
#define __SERDES_H

/* Both MAX9295A serializers' factory-default address. Same value as
 * SER_NATIVE_ADDR / SER_B_ADDR below; kept because the link B bring-up
 * path still refers to it. */
#define SER1_ADDR	0x40
#define DES_ADDR 0x28 /* MAX96716A on Modalix HHHLV2 */

/*
 * DUAL-CAMERA ADDRESS MAP (see serdes_config_init()).
 *
 * Both MAX9295A serializers and both camera MCUs are identical parts and
 * boot at the same addresses, so with both links up every I2C write is
 * broadcast to both and every read comes back as the wire-AND of two
 * devices. They are disambiguated by bringing up ONE link at a time and
 * reprogramming each serializer while it is the only one reachable.
 *
 *   deserializer          0x28   fixed, local on the SoC bus
 *   serializer A (link A) 0x41   reassigned from 0x40 via DEV_ADDR
 *   serializer B (link B) 0x40   left at the factory default
 *   camera A MCU          0x44   translated -> native 0x42
 *   camera B MCU          0x43   translated -> native 0x42
 *
 * Verified on hardware 2026-08-19, both links up, probing each address
 * directly:  r1@0x43 -> ACK (camera B),  r1@0x44 -> ACK (camera A).
 *
 * DO NOT USE i2cdetect TO CHECK THIS. It does not list 0x43 or 0x44
 * even while both ACK, which cost a long detour chasing an alias bug
 * that did not exist. Use:
 *     i2ctransfer -y -f 0 r1@0x44
 * A NAK prints "Remote I/O error"; an ACK returns a byte.
 *
 * 0x42 still responds - both MCUs pass through untranslated there, so
 * a read at 0x42 is the wire-AND of the two and is meaningless. Neither
 * camera's DT node may use it.
 */
#define SER_NATIVE_ADDR	0x40 /* factory default of BOTH serializers */
#define SER_A_ADDR	0x41 /* link A serializer, after DEV_ADDR rewrite */
#define SER_B_ADDR	0x40 /* link B serializer, left at factory default */
#define CAM_A_ALIAS	0x44 /* translated -> 0x42 by SER1_I2C_CONF slot 0 */
#define CAM_B_ALIAS	0x43

struct serdes_parse {
	uint16_t reg;
	uint8_t val;
};

struct serdes_parse SER1_CONF[] = {
	// MAX9295
	{ 0x0330, 0x00 }, { 0x0331, 0x33 }, // 0x331 = D-PHY timing, NOT lane count
	{ 0x0332, 0xE0 }, { 0x0333, 0x04 }, { 0x0334, 0x00 }, { 0x0335, 0x00 },
	{ 0x0308, 0x7D }, { 0x0311, 0x15 }, { 0x0314, 0x6C }, { 0x0315, 0x00 },
	{ 0x0313, 0x10 }, { 0x031C, 0x38 }, { 0x0002, 0x13 }, { 0x0053, 0x10 },
};

/*
 * Identical to SER1_CONF, since this is the same physical camera module -
 * only the I2C translation target differs, not the serializer's own config.
 */
struct serdes_parse SER2_CONF[] = {
	// MAX9295
	{ 0x0330, 0x00 }, { 0x0331, 0x33 }, // 0x331 = D-PHY timing, NOT lane count
	{ 0x0332, 0xE0 }, { 0x0333, 0x04 }, { 0x0334, 0x00 }, { 0x0335, 0x00 },
	{ 0x0308, 0x7D }, { 0x0311, 0x15 }, { 0x0314, 0x6C }, { 0x0315, 0x00 },
	{ 0x0313, 0x10 }, { 0x031C, 0x38 }, { 0x0002, 0x13 }, { 0x0053, 0x10 },
};

/*
 * Pipe Y - link A's video path, VC0. Applied AFTER DSER_PIPE_Z_CONF so
 * the proven Pipe Z sequence stays byte-identical; this table only adds
 * Pipe Y on top and then enables both pipes together.
 *
 * Pipe Y's mapping is an IDENTITY map - SRC == DST - because link A's
 * camera stays on VC0. Compare DSER_PIPE_Z_CONF, where DST carries
 * +0x40 in each entry to move link B onto VC1.
 */
struct serdes_parse DSER_PIPE_Y_CONF[] = {
	{ 0x044B, 0x07 }, // MAP_EN_L: mappings 0,1,2
	{ 0x044C, 0x00 }, // MAP_EN_H
	{ 0x044D, 0x2C }, // MAP_SRC_0: VC0, RAW12
	{ 0x044E, 0x2C }, // MAP_DST_0: VC0, RAW12 (identity)
	{ 0x044F, 0x00 }, // MAP_SRC_1: VC0, frame start
	{ 0x0450, 0x00 }, // MAP_DST_1: VC0
	{ 0x0451, 0x01 }, // MAP_SRC_2: VC0, frame end
	{ 0x0452, 0x01 }, // MAP_DST_2: VC0
	{ 0x046D, 0x2A }, // MAP_DPHY_DEST: DPHY2, same as Pipe Z
	{ 0x0473, 0x01 }, // Pipe Y's counterpart of Pipe Z's 0x04B3
	/*
	 * VIDEO_PIPE_EN: enable Pipe Y *and* Pipe Z. Written last, once
	 * both pipes' mappings are in place. DSER_PIPE_Z_CONF sets this
	 * to 0x02 (Z only); this supersedes it when link A is present.
	 */
	{ 0x0160, 0x03 },
};

// MAX96716

/*
 * Chip-wide configuration, applied ahead of DSER_PIPE_Z_CONF: D-PHY mode,
 * per-port lane config, the data-rate sequence and the CSI output enable.
 * Excludes the per-pipe mapping blocks, so Pipe Z gets a working
 * D-PHY/CSI-output path without disturbing Pipe Y.
 *
 * Two entries are here because their scope is not obvious from the address:
 *  - 0x0332 (MIPI_PHY2, D-PHY timing) belongs to the same MIPI_PHYx family
 *    as 0x0330/0x0333/0x0335 and is chip-wide, not per-pipe.
 *  - 0x04B3 is MIPI_TX51 (register doc p.237/242). The MIPI_TX50-55 family
 *    is defined twice at a fixed +0x40 offset (0x0472-0x0477 and
 *    0x04B2-0x04B7), the same per-controller duplication used for the Pipe
 *    Y/Z mapping blocks (0x044B <-> 0x048B), so 0x04B3 is Pipe Z's own
 *    ALT_MEM_MAP12 bit, distinct from Pipe Y's 0x0473. It controls 12-bit
 *    DT memory packing, which this SRGGB12 stream depends on.
 */
/*
 * Rate- and lane-dependent settings are NOT here: they are programmed by
 * dser_configure_dphy() from the device tree, so the deserializer's output
 * and the SoC's receiver are driven by the same two properties and cannot
 * disagree. This array holds only what is identical at every rate.
 */
struct serdes_parse DSER_PIPE_Z_SHARED_CONF[] = {
	// MIPI D-PHY Configuration (CSI PHY 1)
	{ 0x0330, 0x04 }, // PHY 2x4 mode
	// LANE configuration 0xC0 -> 4lane, 0x50 -> 2lane
	/*
	 * D-PHY transmit timing, widened for 1800 Mbps/lane. Everything
	 * specified in fixed ns stays put while the UI shrinks 833 -> 556 ps,
	 * so the POR values sat only 6-9 ns above the MIPI minimums here.
	 * These fields have minimums but no maximums, so widening is safe.
	 *
	 *  0x0331 = 0xD1  t_hs_przero 0b11 -> 200ns+24UI (was 146ns+24UI)
	 *                 t_hs_prep   0b01 ->  53.4ns+4UI (was 46.7ns+4UI)
	 *                 t_clk_przero 0b01 -> 600ns      (was 306ns, min 300)
	 *  0x0332 = 0xC5  phy_Stdby_n 0b1100 (PHY2+PHY3 active, unchanged)
	 *                 t_lpx       0b01 -> 106.7ns     (unchanged)
	 *                 t_hs_trail  0b01 ->  80ns+8UI  (was 66.7ns+8UI)
	 *  0x0335 = 0x40  t_clk_prep  0b01 ->  46.7ns     (was 40ns, min 38)
	 */
	{ 0x0333, 0x4E },
	/*
	 * Hold the CSI TX PLL in soft reset across the rate change, then
	 * release it. DPLL_0 bit0 is config_soft_rst_n (active low), so
	 * 0xF4 asserts reset and 0xF5 releases it.
	 *
	 * There is one DPLL block per PLL - 0x1C00, 0x1D00, 0x1E00, 0x1F00 -
	 * and BACKTOP1 (0x0308) describes CSIPLLZ_LOCK as the "PLL for MIPI
	 * Port B in 2x4 mode". Port B (PHY2+PHY3) is the port every pipe
	 * here goes out of, so the PLL carrying the data is PLL 2, whose
	 * block is 0x1E00. The original flow reset only 0x1D00 - Port A's
	 * PLL - so the PLL that actually carries the data was never held in
	 * reset while its divider changed, nor relocked afterwards.
	 * 1200 Mbps/lane tolerated that; 1800 did not train at all.
	 */
	// CSI Output Enable
	{ 0x04B3, 0x01 }, // Pipe Z's ALT_MEM_MAP12 (MIPI_TX51) - see note above
	/*
	 * Lane deskew. Mandatory at this rate, and it takes BOTH ends:
	 *
	 *  - The SoC's Synopsys D-PHY RX arms its deskew calibration by
	 *    itself above 1.5 Gbps ("enabled by default for operating
	 *    frequencies > 1.5 Gbps"), but only runs it "if the RX D-PHY is
	 *    triggered through a defined pattern sent by the TX" (D-PHY
	 *    databook 7.2.5). So the deserializer has to emit the pattern;
	 *    nothing on the SoC side can compensate for its absence.
	 *  - The initial pattern is sent once, before the first HS burst.
	 *    If the RX is not yet powered up and in STOPSTATE at that
	 *    instant it is missed, and without periodic deskew the link
	 *    never recovers - which is what an initial-only attempt looked
	 *    like here: continuous PHY and CRC errors, zero frames.
	 *
	 * Values are the user guide's own deskew example (p.26), moved from
	 * its controller 1 / Port A addresses to Port B's controller 2:
	 *   DESKEW_INIT 0x80 - auto initial deskew on, minimum 32k UI width
	 *   DESKEW_PER  0x91 - periodic deskew on, 2k UI, every 2 frames
	 *
	 * Periodic deskew also needs a continuous clock with the clock lane
	 * held in high-speed mode.
	 */

};

/*
 * Pipe Z's own mapping block, mirroring Pipe Y's 0x044B block exactly but
 * at Pipe Z's base address (0x048B, per the official MAX96716A/F user
 * guide's pipe register layout). The 0x0160 entry below is superseded at
 * runtime by the final VIDEO_PIPE_EN write, which is derived from whichever
 * links are actually populated. The deliberate content change vs Pipe Y is
 * MAP_DST_0 = 0x6C instead of 0x2C -
 * bits[7:6]=01 (VC1) instead of 00 (VC0), bits[5:0]=0x2C (RAW12 DT,
 * unchanged) - that is the VC-remap this pipe exists to perform, and
 * it is active below; everything else here just mirrors Pipe Y's own
 * pattern.
 *
 * These MAP_DST_* bytes are the single source of truth for which VC
 * Pipe Z stamps on the wire. Exactly two things must agree or no frame
 * reaches userspace:
 *   1. these bytes                      (deserializer stamps VC1)
 *   2. the overlay's sink port          (csi31in = the host's VC1 pad,
 *                                        which selects IPI2, whose
 *                                        VCID is hardwired to VC1 in
 *                                        dwc-mipi-csi2.c)
 * Nothing validates that pairing at runtime: a mismatch is a silent
 * black frame, not an error. The camera endpoint's sima,virtual-channel
 * property records the same VC a third time, but is NOT a third
 * requirement - it only feeds .get_frame_desc, which nothing in this
 * pipeline currently calls. Do not rely on it to enforce anything.
 */
struct serdes_parse DSER_PIPE_Z_CONF[] = {
	/*
	 * ROOT CAUSE #1 (was missing entirely): VIDEO_PIPE_SEL (0x0161,
	 * register doc p.143) selects which GMSL link AND which serializer
	 * stream ID each video pipe accepts. Pipe Z is bits[5:3], POR
	 * 0b110 = "Link B stream ID 10".
	 *
	 * But the serializer transmits stream ID 00: SER1_CONF/SER2_CONF
	 * write 0x0053 = 0x10, and TX3 (0x53) bits[1:0] = TX_STR_SEL =
	 * 0b00 (MAX9295A reg doc p.114).
	 *
	 * So at POR, Pipe Z filters for stream ID 10 while the camera
	 * sends 00 -> every video packet is discarded at the pipe INPUT ->
	 * VIDEO_LOCK(Z) can never assert -> the MIPI TX PLL never begins
	 * its locking sequence (CSIPLLx_LOCK all 0) -> no CSI output ->
	 * SoC VDMA frame watchdog times out. This also explains why every
	 * fix aimed at the pipe's OUTPUT side (mapping bytes, D-PHY
	 * timing, PLL/link resets) changed nothing: the failure is
	 * upstream of all of them.
	 *
	 * 0x20 maps both pipes correctly - Y <- Link A stream 00 and
	 * Z <- Link B stream 00 - so it is used verbatim here rather than a
	 * Pipe-Z-only variant.
	 *
	 * Must be programmed BEFORE VIDEO_PIPE_EN below, matching the
	 * working flow's ordering.
	 */
	{ 0x0161, 0x20 },
	{ 0x0160, 0x02 }, // superseded by the runtime VIDEO_PIPE_EN write
	// Pipe to Controller Mapping Configuration (Pipe Z)
	{ 0x048B, 0x07 },
	{ 0x048C, 0x00 },
	{ 0x048D, 0x2C },
	{ 0x048E, 0x6C }, // MAP_DST_0: VC1 (bits[7:6]=01), RAW12 DT (0x2C) unchanged
	{ 0x048F, 0x00 },
	{ 0x0490, 0x40 }, // MAP_DST_1: VC1 tag on frame-start
	{ 0x0491, 0x01 },
	{ 0x0492, 0x41 }, // MAP_DST_2: VC1 tag on frame-end
	/*
	 * ROOT CAUSE #2 (off-by-one): this is MAP_DPHY_DEST, which routes
	 * each enabled mapping to a physical D-PHY. Pipe Y's copy is
	 * MIPI_TX45 at 0x046D (register doc p.235); the per-controller
	 * blocks are based at 0x0400/0x0440/0x0480/0x04C0 with register
	 * MIPI_TXn at base+n, so Pipe Z's copy is 0x0480 + 45 = 0x04AD.
	 *
	 * 0x04AC is easy to reach for by mistake and is MAP_DST_15 - a
	 * mapping that MAP_EN_L (0x048B = 0x07) never enables, so a write
	 * was inert, and the real MAP_DPHY_DEST stayed at POR 0x00 =
	 * "all mappings -> DPHY0". The board drives DPHY2 - that is why
	 * 0x048A, DPHY2's CSI2_LANE_CNT, is the one carrying the lane
	 * count - so Pipe Z's output was aimed at the wrong physical port.
	 *
	 * 0x2A = 0b00_10_10_10 -> mappings 0/1/2 all to DPHY2, identical
	 * to Pipe Y's 0x046D value.
	 */
	{ 0x04AD, 0x2A }, // MAP_DPHY_DEST: same DPHY2 as Pipe Y (was 0x04AC)
	/*
	 * CSI_OUT_EN (BACKTOP12 0x0313 bit1, register doc p.193). POR is
	 * already 1, so this is belt-and-braces, but the working flow
	 * writes it explicitly as its final deserializer step and the
	 * datasheet start-up sequence (user guide p.11, step 19) calls
	 * for enabling CSI output after all MIPI TX configuration.
	 */
	{ 0x0313, 0x02 },
};

/*
 * Camera A's MCU alias: SRC 0x44 -> DST 0x42, in translator slot 0
 * (regs 0x0042/0x0043; slot x lives at 0x42 + 2x, per the upstream
 * max96717 driver's MAX96717_I2C_2(x)/MAX96717_I2C_3(x)). Values are
 * the 7-bit address shifted left by one - bits[7:1].
 *
 * Camera B gets the same treatment on serializer B via SER2_I2C_CONF,
 * SRC 0x43 -> DST 0x42, but in slot 1. Different slots is incidental,
 * not required; slot 0 alone was verified sufficient here.
 *
 * Both MCUs are natively 0x42 and both serializers ALSO pass 0x42
 * through untranslated, so 0x42 has two responders once both links are
 * up and any read there is the wire-AND of both. That is why each
 * camera needs its own alias and why neither DT node may use 0x42.
 *
 * Use i2ctransfer, not i2cdetect, to check this - see the address map at
 * the top of this file.
 */
struct serdes_parse SER1_I2C_CONF[] = {
	{ 0x0042, 0x88 }, /* slot 0 SRC = 0x44 (CAM_A_ALIAS) */
	{ 0x0043, 0x84 }, /* slot 0 DST = 0x42 (MCU native)  */
};

/*
 * Camera B's MCU alias: SRC 0x43 -> DST 0x42, in translator slot 1
 * (regs 0x0044/0x0045). Mirror of SER1_I2C_CONF above, which does the
 * same for camera A with SRC 0x44 in slot 0. Values are the 7-bit
 * address shifted left by one.
 *
 * This is NOT the serializer chip's own control address - that stays
 * at SER_NATIVE_ADDR/0x40 for serializer B. This targets the camera
 * MCU behind it, natively 0x42 like camera A's.
 */
struct serdes_parse SER2_I2C_CONF[] = {
	{ 0x0044, 0x86 },
	{ 0x0045, 0x84 },
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
