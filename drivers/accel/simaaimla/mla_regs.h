/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 SiMa.ai
 *
 * MLA register offsets within the AXI window (mdev->regs base).
 * Lifted from /workspace/mla_regs.h, with select aliases mirroring
 * mla_rt/src/mlaip/mosaic_modalix.h. All values here are OFFSETS
 * relative to MLA_AXI_BASE (== mdev->regs).
 */
#ifndef _MLA_REGS_H_
#define _MLA_REGS_H_

/* SYS_REG */
#define MLA_REG_MLA_ACTIVE_DUR_CNT_CTRL   0x000000a8
#define MLA_REG_MLA_ACTIVE_DUR_CNT_MEAS0  0x000000ac

/* TLC_REG: per-quad and quadgroup setup */
#define MLA_REG_QUAD0_SETUP_REG0          0x00001800
#define MLA_REG_QUAD0_DBG_REG0            0x00001804
#define MLA_REG_QUAD1_SETUP_REG0          0x00001808
/* stride between QUADn and QUADn+1 SETUP_REG0 (and DBG_REG0) */
#define MLA_REG_QUAD_STRIDE \
	(MLA_REG_QUAD1_SETUP_REG0 - MLA_REG_QUAD0_SETUP_REG0)

#define MLA_REG_QUADGRP0_SETUP_REG0       0x00001820
#define MLA_REG_QUADGRP0_SETUP_REG1       0x00001824   /* GO bit */

#define MLA_REG_TLC_NON_FTL_INTR_SRC      0x000019e4

/* DMC_REG: per-DMA-unit DFM config */
#define MLA_REG_DMA0_DFM_CONFIG0          0x0000f448
/* stride between DMA_REGn and DMA_REGn+1 (any reg) */
#define MLA_REG_DMA_STRIDE                0x00000200

/* DMC_REG: global semaphore state + per-DMA-unit deadlock-timeout regs.
 * DMA_SEMA_REG layout (dma_sema_reg_s, msc_reg_defs_ma.h:1557):
 *   bits  0..7   m4_push_sema      (M4-side push counter)
 *   bits  8..15  m4_pop_sema       (M4-side pop  counter)
 *   bits 16..23  push_sema_map_field
 *   bits 24..31  pop_sema_map_field
 * Confirmed in msc_addr_map_ma.h:2233 — DMA_SEMA_REG is a single
 * global at 0xf09c (NOT per-unit), covers all four DMA controllers
 * via the map_field bits. Deadlock-timeout is per-unit at stride
 * MLA_REG_DMA_STRIDE. Read for post-mortem when a job times out.
 */
#define MLA_REG_DMA_SEMA_REG              0x0000f09c
#define MLA_REG_DMA0_SEMA_DEADLOCK_TMO    0x0000f430

/* MSC / TLC / MIM diagnostic offsets — needed by mla_hw_dump_status.
 * These are read-only from our side and pulled from
 * mla_rt/include/ma_hw/msc_addr_map_ma.h.
 */
#define MLA_REG_SYS_FTL_INTR_SNAPSHOT     0x0000007c
#define MLA_REG_SYS_SRAM_F_SNAPSHOT       0x0000008c
#define MLA_REG_MIM_SNAPSHOT_F_INFO       0x00002330
#define MLA_REG_TLC_FTL_INTR_SNAPSHOT     0x000019c0
#define MLA_REG_TLC_TMX_QUAD0_FTL_SRC     0x00001988
#define MLA_REG_TLC_TMX_QUAD0_FTL_SNAP    0x00001990
#define MLA_REG_TLC_TMX_QUAD0_NON_FTL_SRC 0x000019c4
/* Per-quad group strides. FTL triples (SRC/EN/SNAP) at 12B stride;
 * NON_FTL pairs (SRC/EN) at 8B stride. Confirmed in
 * mla_rt/include/ma_hw/msc_addr_map_ma.h:355-380 —
 *   FTL_SRC:   0x1988, 0x1994, 0x19a0, 0x19ac       (stride 0xc)
 *   FTL_SNAP:  0x1990, 0x199c, 0x19a8, 0x19b4       (stride 0xc)
 *   NF_SRC:    0x19c4, 0x19cc, 0x19d4, 0x19dc       (stride 0x8)
 * Using the wrong stride causes SError on unmapped offsets.
 */
#define MLA_REG_TLC_TMX_QUAD_FTL_STRIDE   0x0000000c
#define MLA_REG_TLC_TMX_QUAD_NF_STRIDE    0x00000008

/* L2G FTL source registers (4 L2 cache groups, stride 0x400).
 * Offsets from mla_rt/include/ma_hw/msc_addr_map_ma.h.
 */
#define MLA_REG_L2G0_FTL_INTR_SRC0       0x00000890
#define MLA_REG_L2G_STRIDE                0x00000400

/* Quad debug-reg0 status select values */
#define MLA_QUAD_DBG_PROGRAM_DONE         1u

/* QUADGRP0 setup-reg0 layout (tmx_quadgrp_setup_reg0_s, ma_hw):
 *   bits  0..2  quadgrp_sts (status; not written)
 *   bits  3..6  quadgrp_dfm_map  — 0xf = use all 4 DMAs
 *   bits  7..10 quadgrp_quad_map — 0xf = use all 4 quads
 *   bit  11     mla_tile_fuse_opt_rdbk
 *   bits 12..17 quadgrp_flush_len — set to 30
 *   bits 18..31 reserved
 *
 * mla_hw_program_and_start computes this dynamically because DMA-only
 * jobs leave quad_map=0 and tile-only jobs leave dfm_map=0; this is the
 * default "tiles + DMA" pattern.
 */
#define MLA_QG0_DFM_MAP_SHIFT             3u
#define MLA_QG0_QUAD_MAP_SHIFT            7u
#define MLA_QG0_FLUSH_LEN_SHIFT           12u
#define MLA_QG0_FLUSH_LEN_DEFAULT         30u

/* APB→AXI translation. tile_info[i].base_reg is APB-space; subtract
 * APB_BASE to get the AXI-window offset that mdev->regs uses.
 */
#define MLA_APB_BASE                      0x20000000u
#define MLA_AXI_BASE                      0x05000000u
#define MLA_REGS_SIZE                     (64 * 1024)

/* Per-job watchdog defaults */
#define MLA_JOB_DEFAULT_TIMEOUT_MS        2000u
#define MLA_JOB_MAX_TIMEOUT_MS            30000u

/* All 4 quads must signal program-done before a job is considered done. */
#define MLA_QUADS_ACTIVE                  4

/* =====================================================================
 *  Slice 4.5: NF/FTL IRQ enable hierarchy + DMA burst + PRC reset
 * =====================================================================
 */

/* MSC top-level non-FTL aggregator (msc_addr_map_ma.h:38..40) */
#define MLA_REG_SYS_NON_FTL_INTR_STS      0x00000080
#define MLA_REG_SYS_NON_FTL_INTR_FORCE    0x00000084
#define MLA_REG_SYS_NON_FTL_INTR_EN       0x00000088

/* MSC top-level FTL aggregator (msc_ftl_intr_en_s layout):
 *   bit 0 msc_sram_2b_ecc_f_int    mosaic-class fatal
 *   bit 1 mcr_f_int                mosaic-class fatal
 *   bit 2 tlc_f_int                quad/tile-class fatal
 *   bit 3 mim_f_int                memory-interface-class fatal
 *   bit 4 dmc_f_int                DMA-class fatal
 *   bit 5 quadgrp_tmo_f_int        mosaic-class fatal (timeout)
 */
#define MLA_REG_SYS_FTL_INTR_STS          0x00000070
#define MLA_REG_SYS_FTL_INTR_FORCE        0x00000074
#define MLA_REG_SYS_FTL_INTR_EN           0x00000078

/* FTL aggregator bit positions (msc_ftl_intr_en_s) */
#define MLA_FTL_BIT_MSC_SRAM_2B_ECC       0u
#define MLA_FTL_BIT_MCR                   1u
#define MLA_FTL_BIT_TLC                   2u
#define MLA_FTL_BIT_MIM                   3u
#define MLA_FTL_BIT_DMC                   4u
#define MLA_FTL_BIT_QUADGRP_TMO           5u
#define MLA_FTL_MASK_ALL                  0x3fu

/* FTL leaf status registers (per-block). Read after the aggregator
 * tells us the class, to attribute the fault for diagnostic logging.
 */
#define MLA_REG_MIM_F_INT                 0x00002324
#define MLA_REG_DMC_F_INT                 0x0000f090
#define MLA_REG_TLC_QUAD0_FTL_INTR_SRC    0x00001988
/* QUAD1=0x1994, QUAD2=0x19a0, QUAD3=0x19ac — stride 0xc */
#define MLA_REG_TLC_QUAD_FTL_STRIDE       0xc

/* TLC leaf NF. SRC at 0x19e4, EN at 0x19e8, FORCE at 0x19ec — order
 * confirmed against /workspace/mla_regs.h (the auto-generated
 * authoritative map). Writing to an offset outside this exact set
 * causes an SError on real silicon.
 */
#define MLA_REG_TLC_NON_FTL_INTR_EN       0x000019e8
#define MLA_REG_TLC_NON_FTL_INTR_FORCE    0x000019ec

/* TLC NF source bits (tlc_non_ftl_intr_src_s in msc_reg_defs_ma.h):
 *   sync_ctl_intr     bit  0
 *   quadgrp_halt_trig bits 9..12 (4 bits)
 *   quadgrp_prog_done bits 13..16 (4 bits)
 *
 * The EN register (tlc_non_ftl_intr_en_s) uses DIFFERENT bit
 * positions even though it's the same logical signals — the kernel
 * needs separate constants for src vs en or it will write to the
 * wrong bits. mla-rt's user-space code sidesteps this by using
 * named-bitfield assignments; we use explicit shifts.
 */
#define MLA_TLC_NF_SRC_SYNC_CTL_INTR_BIT      0
#define MLA_TLC_NF_SRC_HALT_TRIG_SHIFT        9
#define MLA_TLC_NF_SRC_PROG_DONE_SHIFT        13
#define MLA_TLC_NF_SRC_HALT_TRIG_MASK         (0xfu << MLA_TLC_NF_SRC_HALT_TRIG_SHIFT)
#define MLA_TLC_NF_SRC_PROG_DONE_MASK         (0xfu << MLA_TLC_NF_SRC_PROG_DONE_SHIFT)

/* Aliases for the names mla_hw.c uses; refer to the SRC layout above. */
#define MLA_TLC_NF_SYNC_CTL_INTR_BIT          MLA_TLC_NF_SRC_SYNC_CTL_INTR_BIT
#define MLA_TLC_NF_QUADGRP_HALT_TRIG_SHIFT    MLA_TLC_NF_SRC_HALT_TRIG_SHIFT
#define MLA_TLC_NF_QUADGRP_PROG_DONE_SHIFT    MLA_TLC_NF_SRC_PROG_DONE_SHIFT
#define MLA_TLC_NF_QUADGRP_HALT_TRIG_MASK     MLA_TLC_NF_SRC_HALT_TRIG_MASK
#define MLA_TLC_NF_QUADGRP_PROG_DONE_MASK     MLA_TLC_NF_SRC_PROG_DONE_MASK

/* TLC NF EN-register bit positions (tlc_non_ftl_intr_en_s):
 *   quad_intr         bits  0..3
 *   l2_group_intr     bits  4..7
 *   sync_ctl_intr     bit   8
 *   quadgrp_halt_trig bits  9..12
 *   quadgrp_prog_done bits 13..16
 *
 * Coincidentally the halt_trig + prog_done positions match SRC's
 * bits 9..16, but sync_ctl_intr is at bit 8 in EN vs bit 0 in SRC,
 * and EN has additional fields (quad_intr, l2_group_intr) at bits
 * 0..7 that don't exist in SRC.
 */
#define MLA_TLC_NF_EN_SYNC_CTL_BIT            8
#define MLA_TLC_NF_EN_HALT_TRIG_SHIFT         9
#define MLA_TLC_NF_EN_PROG_DONE_SHIFT         13
#define MLA_TLC_NF_EN_PROG_DONE_MASK          (0xfu << MLA_TLC_NF_EN_PROG_DONE_SHIFT)

/* MSC top-level NF EN bits (msc_non_ftl_intr_en_s) that mla_hw_enable_irqs()
 * programs. Bit 3 is the one that gates progdone propagation out of the TLC
 * leaf: with it clear the MLA latches progdone in TLC_NON_FTL_INTR_SRC and
 * never drives the GIC line.
 */
#define MLA_MSC_NF_EN_MCR_BIT                 2
#define MLA_MSC_NF_EN_TLC_BIT                 3
#define MLA_MSC_NF_EN_DMC_BIT                 4
#define MLA_MSC_NF_EN_KERNEL_SET  ((1u << MLA_MSC_NF_EN_MCR_BIT) | \
				   (1u << MLA_MSC_NF_EN_TLC_BIT) | \
				   (1u << MLA_MSC_NF_EN_DMC_BIT))

/* MSC top-level NF source bits the kernel must keep enabled. */
#define MLA_MSC_NON_FTL_INTR_EN_ALL           0xffffffffu

/* DMA burst-config registers — one per DMA controller (4 total) at
 * stride MLA_REG_DMA_STRIDE. The base offset is in msc_addr_map_ma.h
 * as MSC_REG__DMC_REG__DMA_REG0__DMA_CFG_REG_ADDR.
 */
#define MLA_REG_DMA0_DMA_CFG_REG          0x0000f438

/* dma_cfg_reg_s bit layout (msc_reg_defs_ma.h):
 *   bits 0..2  dma_wr_burst_size
 *   bits 3..5  dma_rd_burst_size
 */
#define MLA_DMA_CFG_WR_BURST_SHIFT        0
#define MLA_DMA_CFG_RD_BURST_SHIFT        3
#define MLA_DMA_CFG_BURST_MASK            0x7u

/* Burst size encodings (BurstSize enum in MLAControl.hh). */
#define MLA_BURST_256B                    0u
#define MLA_BURST_512B                    1u
#define MLA_BURST_1KB                     2u
#define MLA_BURST_2KB                     3u
#define MLA_BURST_4KB                     4u

/* Operating defaults. mla_rt picks (1KB write, 512B read) for normal
 * models; semaphored models REQUIRE 256B WR.
 */
#define MLA_DEFAULT_RD_BURST              MLA_BURST_512B
#define MLA_DEFAULT_WR_BURST              MLA_BURST_1KB
#define MLA_SEMA_WR_BURST                 MLA_BURST_256B
#define MLA_HW_RESET_BURST                MLA_BURST_256B

/* PRC clock/reset register region. NOT in the MLA AXI window — must
 * be ioremapped separately. mla_ck_rst is at MLA_PRC_REGS_BASE +
 * MLA_PRC_MLA_CK_RST_OFFSET.
 */
#define MLA_PRC_REGS_BASE                 0x000ff00000ULL
#define MLA_PRC_MLA_CK_RST_OFFSET         0x00000594
#define MLA_PRC_REGS_SIZE                 0x1000

/* TLC sync-controller perf counter block.
 *
 * 8 counter slots of 8 bytes each (CTRL+CNTR pair) starting at offset
 * 0x1948. CTRL layout (ctrl_s in msc_reg_defs_ma.h:617):
 *   bit  0     cntr_en
 *   bit  1     duration_mode
 *   bit  3     cntr_ovfl
 *   bits 4..23 src_sel  (low 3 bits = signal id; bit 3 = quad_group_sel)
 *   bits 24..31 cnt_hi   (top 8 bits of 40-bit counter; CNTR holds low 32)
 *
 * src_sel signal ids (slice-local copies of the enum in
 * mla_rt/externals/m4_mla/hw_include/msc_reg_defs.h:25-36):
 *   0  tiled_lat            (tile_d_runtime — full QG runtime)
 *   1  tiles_runtime        (per-tile runtime)
 *   2  tiles_active         (active compute cycles)
 *   3  sync_freeze_or_pwr
 *   4  tile_pwr_ctrl
 *   5  sync_freeze
 *   6  iq_sync_freeze       (instruction-queue stall)
 *   7  d_sync_freeze        (data stall — DRAM/L2 wait)
 */
#define MLA_REG_TLC_SCT_PCT_CTRL          0x00001948
#define MLA_REG_TLC_SCT_PCT_CNTR          0x0000194c
#define MLA_REG_TLC_SCT_PCT_STRIDE        8

#define MLA_PCT_CTRL_CNTR_EN              (1u << 0)
#define MLA_PCT_CTRL_DURATION_MODE        (1u << 1)
#define MLA_PCT_CTRL_CNTR_OVFL            (1u << 3)
#define MLA_PCT_CTRL_SRC_SEL_SHIFT        4
#define MLA_PCT_CTRL_SRC_SEL_MASK         (0xfffffu << MLA_PCT_CTRL_SRC_SEL_SHIFT)
#define MLA_PCT_CTRL_CNT_HI_SHIFT         24
#define MLA_PCT_CTRL_CNT_HI_MASK          (0xffu << MLA_PCT_CTRL_CNT_HI_SHIFT)

/* Usable counter slots. FOUR, not the eight CTRL/CNTR pairs the block
 * exposes: per the src_sel notes above, slots 0-3 serve quadgroup 0/1 and
 * slots 4-7 serve quadgroup 2/3, and this SoC is permanently QG0-only.
 * Must equal MLA_PERFCTR_MAX_SLOTS in the uapi header; the two are kept
 * separate only to avoid a circular include.
 *
 * Which signal each slot carries is chosen by userspace through
 * MLA_OP_PERFCTR_CONFIG — the driver wires nothing by default. The
 * authoritative signal list is the table above plus the uapi's
 * MLA_OP_PERFCTR_CONFIG block.
 */
#define MLA_PCT_NUM_SLOTS                 4u

/* mla_ck_rst bit layout (mla_ck_rst_s in prc_reg_defs.h:1454-1462). */
#define MLA_CK_RST_MLA_CLK_EN             (1u << 0)
#define MLA_CK_RST_MLA_DBG_CLK_EN         (1u << 1)
#define MLA_CK_RST_MLA_XXX_RESET_N        (1u << 2)
#define MLA_CK_RST_MLA_UCTRL_POR_RST      (1u << 3)
#define MLA_CK_RST_MLA_UCTRL_SYS_RST      (1u << 4)
#define MLA_CK_RST_MLA_DBG_RSTN           (1u << 5)

/* MLA PLL control/status, same PRC region as MLA_PRC_MLA_CK_RST. The
 * driver only ever reads this register (writes are mla-rt's job).
 * Layout from pll_ctrl_stat_s (ma_hw/prc_reg_defs.h:13-26):
 *   bit  0      pll_enpll
 *   bit  1      pll_reset_n
 *   bit  2      pll_byp
 *   bit  3      reserved
 *   bits 4..9   pll_refdiv (6)
 *   bit  10     reserved
 *   bits 11..22 pll_fbdiv (12)
 *   bits 23..25 pll_postdiv1 (3)
 *   bit  26     reserved
 *   bits 27..29 pll_postdiv2 (3)
 *   bit  30     pll_lock
 *   bit  31     timeout
 */
#define MLA_PRC_MLA_PLL_CTRL_OFFSET       0x00000014
#define MLA_PLL_REFDIV_SHIFT              4
#define MLA_PLL_REFDIV_MASK               0x3fu
#define MLA_PLL_FBDIV_SHIFT               11
#define MLA_PLL_FBDIV_MASK                0xfffu
#define MLA_PLL_POSTDIV1_SHIFT            23
#define MLA_PLL_POSTDIV1_MASK             0x7u
#define MLA_PLL_POSTDIV2_SHIFT            27
#define MLA_PLL_POSTDIV2_MASK             0x7u

/*
 * Descriptor DRAM-address field.
 *
 * Shared by the two places that patch it: the per-job patcher
 * (mla_hw_apply_relocs) and the MODEL_DEFINE static-reloc splicer
 * (v2_apply_static_section_relocs). One wire format, one definition,
 * so the two cannot drift apart silently.
 *
 * NOT related to the 40-bit perfctr cycle counter in
 * mla_hw_perfctr_read_us() despite the matching width — that is a counter
 * saturation value, not an address.
 *
 * The 40-bit reach is also why MLA_OP_INFO advertising DMA_BIT_MASK(64) is a
 * simplification; see the dma_coerce_mask_and_coherent() call in mla_probe().
 */
#define MLA_DESC_SIZE_BYTES               32
#define MLA_DESC_DRAM_ADDR_WORD           1     /* word[1] = bytes 8..15 */
#define MLA_DESC_DRAM_ADDR_SHIFT          22u   /* bit position in word[1] */
#define MLA_DESC_DRAM_ADDR_BITS           40u
#define MLA_DESC_DRAM_ADDR_MASK           ((1ULL << MLA_DESC_DRAM_ADDR_BITS) - 1)

#endif /* _MLA_REGS_H_ */
