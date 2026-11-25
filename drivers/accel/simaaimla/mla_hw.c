// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// MLA hardware programming: per-job register-poke sequence (init_dfm +
// load_tile_instructions + GO), clock/reset, IRQ enable masks, DMA
// burst-size config, stale-IRQ clear, and per-submit cache sync. Mirrors
// mla_rt's src/mlaip/modalix_driver.cc.
//
// Register I/O routes through mdev->regs (AXI window). The clock/reset
// register is in a different physical region (PRC), reached through
// mdev->prc_regs which is ioremap'd separately at probe.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include "mla_drv.h"
#include "mla_image_fmt.h"
#include "mla_regs.h"

/**
 * mla_hw_reset() — assert POR + SYS reset and release, via PRC MMIO
 * @mdev: target device
 *
 * Always drives the PRC MLA_CK_RST register directly: asserts
 * POR + SYS reset (bits 3+4) with clocks enabled, then releases
 * mla_xxx_reset_n. The clk-framework gate toggle only touches
 * bits 0+1 (ck_en + xxx_reset_n) and does not clear DMA semaphore
 * counters / QG state that a stalled semaphored job leaves behind —
 * so we use PRC MMIO unconditionally.
 *
 * Caller must already hold sched_lock or otherwise ensure no jobs are
 * running across the reset.
 *
 * Return: 0 on success; -ENODEV if the PRC region was not mapped at
 * probe.
 */
int mla_hw_reset(struct mla_device *mdev)
{
	if (!mdev->prc_regs)
		return -ENODEV;

	iowrite32(MLA_CK_RST_MLA_CLK_EN |
		  MLA_CK_RST_MLA_DBG_CLK_EN |
		  MLA_CK_RST_MLA_UCTRL_POR_RST |
		  MLA_CK_RST_MLA_UCTRL_SYS_RST,
		  mdev->prc_regs + MLA_PRC_MLA_CK_RST_OFFSET);
	udelay(10);

	iowrite32(MLA_CK_RST_MLA_CLK_EN |
		  MLA_CK_RST_MLA_DBG_CLK_EN |
		  MLA_CK_RST_MLA_XXX_RESET_N |
		  MLA_CK_RST_MLA_UCTRL_POR_RST |
		  MLA_CK_RST_MLA_UCTRL_SYS_RST,
		  mdev->prc_regs + MLA_PRC_MLA_CK_RST_OFFSET);
	udelay(10);

	/* The block is now out of reset and has executed nothing. DMA_CFG
	 * latches only out of reset, so mla_sched_apply_burst() may program
	 * the burst directly from here until a GO write clears this.
	 */
	WRITE_ONCE(mdev->hw_fresh_reset, true);
	return 0;
}

/**
 * mla_hw_read_pll_freq_mhz() — compute MLA core clock from PRC PLL
 * @mdev: device whose PRC region we read
 *
 * Reads the MLA PLL control/status register and computes the
 * post-divider frequency. Mirrors mla-rt's MLAControl::ma_getMlaFreq:
 *   mhz = (100 / 3 * fbdiv) / (refdiv * postdiv1 * postdiv2)
 *
 * Read-only PRC access — the "PRC writes cause lockups" constraint
 * applies only to writes, not reads.
 *
 * Return: clock in MHz; 0 if prc_regs unmapped or any divider is 0
 * (the PLL hasn't been programmed yet — we don't try to recover,
 * just report unknown so callers can fall back).
 */
u32 mla_hw_read_pll_freq_mhz(struct mla_device *mdev)
{
	u32 v, refdiv, fbdiv, postdiv1, postdiv2;
	u64 num, den;

	if (!mdev->prc_regs)
		return 0;

	v = ioread32(mdev->prc_regs + MLA_PRC_MLA_PLL_CTRL_OFFSET);
	refdiv   = (v >> MLA_PLL_REFDIV_SHIFT)   & MLA_PLL_REFDIV_MASK;
	fbdiv    = (v >> MLA_PLL_FBDIV_SHIFT)    & MLA_PLL_FBDIV_MASK;
	postdiv1 = (v >> MLA_PLL_POSTDIV1_SHIFT) & MLA_PLL_POSTDIV1_MASK;
	postdiv2 = (v >> MLA_PLL_POSTDIV2_SHIFT) & MLA_PLL_POSTDIV2_MASK;

	if (!refdiv || !postdiv1 || !postdiv2)
		return 0;

	num = 100ULL * fbdiv;
	den = 3ULL * refdiv * postdiv1 * postdiv2;
	return (u32)((num + den / 2) / den);
}

/* ---------------------------------------------------------------------- *
 *  IRQ enables
 *
 *  After reset, all IRQ enable masks are zero. Without these the
 *  hardware sets source bits internally but never drives the GIC
 *  line, producing a silent "GO written, no IRQ ever fires" hang.
 *
 *  Mirrors enable_non_fatal_irqs() in
 *  src/mlaip/mla_non_fatal_irqs.cc:291-361 plus the FTL aggregator
 *  enable from src/mlaip/mla_fatal_irqs.cc.
 * ----------------------------------------------------------------------
 */

/* TLC sync-controller performance counters.
 *
 * Programs slot @idx to source signal @signal_id. quad_group_sel is fixed
 * at 0 since Modalix is permanently QG0-only. duration_mode=0 (default)
 * means "count only while QG is active" — which gives us per-job deltas
 * without explicit reset. mla-rt's MIP path uses the exact same setup
 * (mla_perfctrs.cc:set_tlc_sct_perf_counter).
 *
 * Only slots 0..3 are usable. The block has eight CTRL/CNTR pairs, but
 * per msc_reg_defs.h the low four serve quadgroup 0/1 and the high four
 * serve quadgroup 2/3, and this SoC only ever runs QG0.
 *
 * Which signals are programmed is chosen by userspace via
 * MLA_OP_PERFCTR_CONFIG; nothing is enabled until it asks.
 */
static void mla_hw_perfctr_setup_one(struct mla_device *mdev, u32 idx,
				     u32 signal_id)
{
	u32 off = MLA_REG_TLC_SCT_PCT_CTRL +
		  idx * MLA_REG_TLC_SCT_PCT_STRIDE;
	u32 v;

	v = ioread32(mdev->regs + off);
	v &= ~MLA_PCT_CTRL_SRC_SEL_MASK;
	v |= MLA_PCT_CTRL_CNTR_EN;
	v &= ~MLA_PCT_CTRL_DURATION_MODE;            /* count while QG active */
	v |= ((signal_id & 0xfffffu) << MLA_PCT_CTRL_SRC_SEL_SHIFT);
	iowrite32(v, mdev->regs + off);
}

/* Clear cntr_en on slot @idx. A disabled slot is what makes
 * mla_hw_perfctr_read_us's "report nothing" state real rather than
 * cosmetic: leaving a slot enabled but unread would keep it accumulating,
 * and a later read would return a window nobody asked to measure.
 */
static void mla_hw_perfctr_disable_one(struct mla_device *mdev, u32 idx)
{
	u32 off = MLA_REG_TLC_SCT_PCT_CTRL +
		  idx * MLA_REG_TLC_SCT_PCT_STRIDE;
	u32 v;

	v = ioread32(mdev->regs + off);
	v &= ~MLA_PCT_CTRL_CNTR_EN;
	iowrite32(v, mdev->regs + off);
}

/**
 * mla_hw_perfctr_apply_locked() — push mdev->perfctr_* into the hardware
 * @mdev: device whose AXI window we'll write
 *
 * Programs slot i to mdev->perfctr_signals[i] for i < mdev->perfctr_count
 * and disables every slot at or above the count, so the register state
 * always matches what the response will claim.
 *
 * Caller must hold mdev->sched_lock, and must only call this while the
 * device is idle — MLA_OP_PERFCTR_CONFIG enforces both. Changing src_sel
 * under a running job would give that job a measurement window split
 * across two different signals.
 */
void mla_hw_perfctr_apply_locked(struct mla_device *mdev)
{
	u32 i;

	if (!mdev->regs)
		return;

	for (i = 0; i < MLA_PCT_NUM_SLOTS; i++) {
		if (i < mdev->perfctr_count)
			mla_hw_perfctr_setup_one(mdev, i,
						 mdev->perfctr_signals[i]);
		else
			mla_hw_perfctr_disable_one(mdev, i);
	}
}

/**
 * mla_hw_perfctr_disable_all() — return to the unconfigured state
 * @mdev: device whose AXI window we'll write
 *
 * Clears the stored selection and disables all four slots. This is the
 * probe-time state (counters are opt-in, so a fresh module reports
 * nothing). NOTE: mla_hw_recover() clears the selection in software only
 * instead of calling this, because doing MMIO there can take a synchronous
 * external abort under sched_lock. This function's own callers (probe and
 * context shutdown) run with the device up, not mid-reset.
 *
 * Explicitly disabling at probe rather than simply not touching the
 * registers matters — a previous module load may have left slots enabled
 * with its own selection, and inheriting that would report signals this
 * load never agreed to.
 *
 * Takes sched_lock itself, so callers must not already hold it. All three
 * call sites are safe by construction: probe, mla_recovery_work_fn (a
 * workqueue), and mla_sched_device_reset (documented process-context-only)
 * — and mla_hw_recover flushes a workqueue, so it can never be reached
 * with a spinlock held in the first place.
 */
void mla_hw_perfctr_disable_all(struct mla_device *mdev)
{
	unsigned long flags;

	spin_lock_irqsave(&mdev->sched_lock, flags);
	mdev->perfctr_count = 0;
	memset(mdev->perfctr_signals, 0, sizeof(mdev->perfctr_signals));
	mdev->perfctr_gen++;
	mla_hw_perfctr_apply_locked(mdev);
	spin_unlock_irqrestore(&mdev->sched_lock, flags);
}

/**
 * mla_hw_perfctr_read_us() — sample the configured slots, in microseconds
 * @mdev: device whose AXI window we'll read
 * @out:  array of MLA_PCT_NUM_SLOTS u32, filled with per-slot µs
 *
 * out[i] is the i-th signal the caller selected, so the order it listed
 * them in MLA_OP_PERFCTR_CONFIG is the order it reads them back. Slots at
 * or above the configured count are zeroed rather than read.
 *
 * With nothing configured this zeroes @out and touches no registers at
 * all. That skip is deliberate: it is the common case, it runs in hard
 * IRQ, and reading four disabled slots would cost 8 MMIO accesses per job
 * to produce values nobody asked for.
 *
 * Reads the 32-bit CNTR + 8-bit cnt_hi (from CTRL) for each slot,
 * forms the 40-bit cycle count, divides by mdev->mla_mhz to convert
 * to microseconds. Saturates at 2^40-1 cycles per the hardware
 * convention (cntr_ovfl flag in CTRL).
 *
 * No reset performed — counters auto-clear at next QG active edge per
 * duration_mode=0 setup.
 */
void mla_hw_perfctr_read_us(struct mla_device *mdev, u32 out[4])
{
	u32 mhz = mdev->mla_mhz ? mdev->mla_mhz : 1000;
	u32 count = READ_ONCE(mdev->perfctr_count);
	u32 i;

	memset(out, 0, MLA_PCT_NUM_SLOTS * sizeof(out[0]));

	if (!count || !mdev->regs)
		return;

	for (i = 0; i < count; i++) {
		u32 ctrl_off = MLA_REG_TLC_SCT_PCT_CTRL +
			       i * MLA_REG_TLC_SCT_PCT_STRIDE;
		u32 cntr_off = MLA_REG_TLC_SCT_PCT_CNTR +
			       i * MLA_REG_TLC_SCT_PCT_STRIDE;
		u32 ctrl = ioread32(mdev->regs + ctrl_off);
		u32 lo   = ioread32(mdev->regs + cntr_off);
		u64 cyc;

		if (ctrl & MLA_PCT_CTRL_CNTR_OVFL) {
			cyc = (1ULL << 40) - 1;
		} else {
			u32 hi = (ctrl & MLA_PCT_CTRL_CNT_HI_MASK) >>
				 MLA_PCT_CTRL_CNT_HI_SHIFT;
			cyc = ((u64)hi << 32) | lo;
		}
		out[i] = (u32)(cyc / mhz);
	}
}

/**
 * mla_hw_enable_irqs() — program TLC + MSC + FTL interrupt enables
 * @mdev: device whose AXI window we'll write
 *
 * Sets the TLC leaf EN bits (quadgrp_prog_done[3:0], halt_trig[3:0],
 * sync_ctl), the MSC top-level aggregator EN (tlc/mcr/dmc nf_int —
 * specifically bit 3 which gates progdone propagation), and enables
 * all FTL aggregator sources for fault attribution.
 *
 * Mirrors mla-rt's enable_non_fatal_irqs() and the FTL aggregator
 * enable. Without these masks set, the MLA latches source bits
 * internally but never drives the GIC line — the symptom looks like
 * "GO written, no IRQ ever fires."
 */
void mla_hw_enable_irqs(struct mla_device *mdev)
{
	u32 tlc_en;

	/* TLC leaf EN register (tlc_non_ftl_intr_en_s — different layout
	 * from SRC). Set quadgrp_prog_done[3:0] + quadgrp_halt_trig[3:0]
	 * + sync_ctl_intr bit. Read-modify-write to preserve any other
	 * bits that may already be set.
	 */
	tlc_en  = ioread32(mdev->regs + MLA_REG_TLC_NON_FTL_INTR_EN);
	tlc_en |= 0xfu << MLA_TLC_NF_EN_PROG_DONE_SHIFT;   /* 4 quadgroups */
	tlc_en |= 0xfu << MLA_TLC_NF_EN_HALT_TRIG_SHIFT;
	tlc_en |= 1u   << MLA_TLC_NF_EN_SYNC_CTL_BIT;
	iowrite32(tlc_en, mdev->regs + MLA_REG_TLC_NON_FTL_INTR_EN);

	/* MSC top-level aggregator (msc_non_ftl_intr_en_s layout):
	 *   bit 0 msc_sram_1b_ecc_nf_int
	 *   bit 1 mcr_mailbox_nf_int
	 *   bit 2 mcr_nf_int
	 *   bit 3 tlc_nf_int     ← gates progdone propagation
	 *   bit 4 dmc_nf_int     ← gates DMA-done propagation
	 * mla_rt's enable_non_fatal_irqs sets tlc_nf_int + mcr_nf_int +
	 * dmc_nf_int → bits 2,3,4 = 0x1c. WITHOUT bit 3 set, TLC source
	 * events (including progdone) are blocked at the aggregator and
	 * never drive the GIC line — board appears to hang silently.
	 */
	iowrite32(MLA_MSC_NF_EN_KERNEL_SET,
		  mdev->regs + MLA_REG_SYS_NON_FTL_INTR_EN);

	iowrite32(0xffffffffu, mdev->regs + MLA_REG_SYS_FTL_INTR_EN);

	/* Published AFTER the writes land, so a concurrent
	 * mla_hw_repair_irq_masks() never sees "armed" describing masks that
	 * are not programmed yet.
	 */
	WRITE_ONCE(mdev->irqs_armed, true);
}

/**
 * mla_hw_init() — one-shot reset + IRQ-enable at probe
 * @mdev: device to initialize
 *
 * Calls mla_hw_reset() (only effective when prc_regs is mapped),
 * then unconditionally calls mla_hw_enable_irqs() — the latter is
 * needed even without reset, since enabling IRQs on an already-running
 * MLA is required for the IRQ path to work.
 *
 * Re-invoked from the scheduler when the burst-size dance fires and
 * from the recovery worker after a fatal fault.
 *
 * Return: 0 if reset succeeded; the reset's errno (typically -ENODEV
 * when prc_regs is unmapped) otherwise. IRQ enables run regardless.
 */
int mla_hw_init(struct mla_device *mdev)
{
	int ret = mla_hw_reset(mdev);

	/* Even if reset failed (no PRC region), enabling IRQs against
	 * an MLA that's already running is fine and actually required
	 * for the IRQ path to work.
	 */
	mla_hw_enable_irqs(mdev);

	/* A reset wipes DFM/tile/QG0 register state, so the cached
	 * "last programmed model" is no longer valid. Force the next
	 * dispatch to reprogram.
	 */
	mdev->last_programmed_model = NULL;
	mdev->last_programmed_bank_idx = MLA_BANK_UNASSIGNED;
	return ret;
}

/* ---------------------------------------------------------------------- *
 *  DMA burst size
 *
 *  Writes the same DMA_CFG value to all 4 DMA controllers. After a
 *  reset the HW default is 256B/256B (slowest); mla_rt resets to
 *  (1KB write, 512B read) for normal models. Semaphored models force
 *  256B WR — see mla_sched_pick_locked for the dispatch.
 * ----------------------------------------------------------------------
 */

/**
 * mla_hw_set_dma_burst_size() — program DMA burst size on all 4 DMA units
 * @mdev: device whose DMA controllers we'll program
 * @rd:   read burst encoding (MLA_BURST_*)
 * @wr:   write burst encoding (MLA_BURST_*)
 *
 * Writes the same DMA_CFG value to all 4 DMA units. Default model
 * mix uses (rd=512B, wr=1KB); semaphore-based models force wr=256B
 * (see mla_sched_apply_burst). Caches @rd and @wr in @mdev so the
 * scheduler can detect when the next job needs a different size.
 *
 * No-op if mdev->regs isn't mapped (debug mode).
 */
void mla_hw_set_dma_burst_size(struct mla_device *mdev, u32 rd, u32 wr)
{
	u32 i, val;

	if (!mdev->regs)
		return;

	val = ((rd & MLA_DMA_CFG_BURST_MASK) << MLA_DMA_CFG_RD_BURST_SHIFT) |
	      ((wr & MLA_DMA_CFG_BURST_MASK) << MLA_DMA_CFG_WR_BURST_SHIFT);

	for (i = 0; i < 4; i++)
		iowrite32(val,
			  mdev->regs + MLA_REG_DMA0_DMA_CFG_REG +
			  i * MLA_REG_DMA_STRIDE);

	mdev->cur_burst_rd = (s8)rd;
	mdev->cur_burst_wr = (s8)wr;

	dev_dbg(mdev->dev, "dma burst set: rd=%u wr=%u (val=0x%x)\n",
		rd, wr, val);
}

/**
 * mla_hw_quiesce_irqs() — disable all MLA-level IRQ enable masks
 * @mdev: device whose AXI window we'll write
 *
 * Writes 0 to TLC_NON_FTL_INTR_EN, SYS_NON_FTL_INTR_EN, and
 * SYS_FTL_INTR_EN so no further IRQ sources can drive the GIC line
 * while the kernel runs fault recovery. After this returns the
 * hardware will not raise any IRQ until mla_hw_enable_irqs() runs
 * again.
 *
 * Safe to call from any context; only does iowrite32.
 */
void mla_hw_quiesce_irqs(struct mla_device *mdev)
{
	if (!mdev->regs)
		return;
	/* Published BEFORE the writes, so a concurrent
	 * mla_hw_repair_irq_masks() cannot observe "armed" and re-arm masks
	 * we are in the middle of tearing down.
	 */
	WRITE_ONCE(mdev->irqs_armed, false);
	iowrite32(0, mdev->regs + MLA_REG_TLC_NON_FTL_INTR_EN);
	iowrite32(0, mdev->regs + MLA_REG_SYS_NON_FTL_INTR_EN);
	iowrite32(0, mdev->regs + MLA_REG_SYS_FTL_INTR_EN);
}

/**
 * mla_hw_repair_irq_masks() — re-arm IRQ masks an external agent cleared
 * @mdev: device to check
 *
 * mla-rt's MIP backend drives this MLA from user space with the kernel
 * driver still loaded, and its mla_init() calls disable_non_fatal_irqs(),
 * which clears quadgrp_prog_done in the TLC leaf EN and tlc_nf_int in the
 * MSC aggregator. MIP polls for completion, so it neither needs nor
 * restores them. This driver is interrupt-driven, so the next JOB_SUBMIT
 * programs the model, writes GO, the MLA runs the job to completion and
 * latches progdone in TLC_NON_FTL_INTR_SRC — and no IRQ is ever delivered.
 * The job dies on the watchdog with sys_sts=0, i.e. no fault of any kind;
 * mla_hw_recover() then re-arms the masks as a side effect, which is why
 * only the FIRST kernel-driver run after a MIP run fails.
 *
 * Repairs only when this driver believes the masks are armed. When
 * irqs_armed is false they are off because we turned them off (the fatal
 * IRQ handler quiesces in hard-IRQ context and defers the reset, or
 * mla_hw_recover() is mid-flight) and mla_hw_recover() owns restoring
 * them. Re-arming here would enable IRQs on a block that is about to be
 * reset: the resulting fault costs an extra slot against the 3-per-60 s
 * recovery budget, and reading TLC registers across a reset is what took
 * the synchronous external abort described in mla_sched_apply_burst().
 *
 * Tests the specific bits rather than "EN is non-zero". MIP leaves
 * quadgrp_halt_trig at 0xf, so TLC_NON_FTL_INTR_EN reads 0x1e00, and EN
 * additionally has quad_intr / l2_group_intr at bits 0..7 that SRC does
 * not — a non-zero test concludes the masks are healthy and repairs
 * nothing.
 *
 * Context: process, from mla_open(). Two MMIO reads, and no writes at all
 * in the normal case.
 */
void mla_hw_repair_irq_masks(struct mla_device *mdev)
{
	u32 tlc_en, sys_en;

	if (!mdev->regs || !READ_ONCE(mdev->irqs_armed))
		return;

	tlc_en = ioread32(mdev->regs + MLA_REG_TLC_NON_FTL_INTR_EN);
	sys_en = ioread32(mdev->regs + MLA_REG_SYS_NON_FTL_INTR_EN);

	if ((tlc_en & MLA_TLC_NF_EN_PROG_DONE_MASK) &&
	    (sys_en & (1u << MLA_MSC_NF_EN_TLC_BIT)))
		return;                 /* still armed — nothing to do */

	atomic_inc(&mdev->irq_masks_repaired);
	dev_warn_once(mdev->dev,
		      "non-fatal IRQ masks cleared by an external agent (TLC_EN=0x%08x SYS_EN=0x%08x) — re-arming\n",
		      tlc_en, sys_en);
	mla_hw_enable_irqs(mdev);
}

/* ---------------------------------------------------------------------- *
 *  mla_hw_dump_status() — post-mortem MLA register snapshot.
 *
 *  Mirrors mla_rt's mla_dump_all_status (mla_util.cc) for the kernel-
 *  driver path where user space has no MMIO access. Called on job
 *  watchdog timeout + recovery-budget-exceeded to give us evidence at
 *  the moment the pipeline stalled.
 *
 *  Reads only. Safe from any context that mdev->regs is valid in
 *  (soft-IRQ / process). Total: ~28 register reads (~few µs).
 * ----------------------------------------------------------------------
 */
void mla_hw_dump_status(struct mla_device *mdev, const char *tag)
{
	u32 sys_ftl_sts, sys_ftl_snap, sram_snap;
	u32 dmc_f, mim_f, mim_snap;
	u32 tlc_ftl_snap, tlc_nf_src;
	u32 qg0_setup, quad_dbg[MLA_QUADS_ACTIVE];
	u32 q_ftl_src[MLA_QUADS_ACTIVE], q_ftl_snap[MLA_QUADS_ACTIVE];
	u32 q_nf_src[MLA_QUADS_ACTIVE];
	u32 dma_sema, dma_deadlock[4];
	int i;

	if (!mdev->regs) {
		dev_warn(mdev->dev, "dump[%s]: regs not mapped\n", tag ?: "?");
		return;
	}

	sys_ftl_sts  = ioread32(mdev->regs + MLA_REG_SYS_FTL_INTR_STS);
	sys_ftl_snap = ioread32(mdev->regs + MLA_REG_SYS_FTL_INTR_SNAPSHOT);
	sram_snap    = ioread32(mdev->regs + MLA_REG_SYS_SRAM_F_SNAPSHOT);
	dmc_f        = ioread32(mdev->regs + MLA_REG_DMC_F_INT);
	mim_f        = ioread32(mdev->regs + MLA_REG_MIM_F_INT);
	mim_snap     = ioread32(mdev->regs + MLA_REG_MIM_SNAPSHOT_F_INFO);
	tlc_ftl_snap = ioread32(mdev->regs + MLA_REG_TLC_FTL_INTR_SNAPSHOT);
	tlc_nf_src   = ioread32(mdev->regs + MLA_REG_TLC_NON_FTL_INTR_SRC);
	qg0_setup    = ioread32(mdev->regs + MLA_REG_QUADGRP0_SETUP_REG0);

	for (i = 0; i < MLA_QUADS_ACTIVE; i++) {
		quad_dbg[i] = ioread32(mdev->regs +
				       MLA_REG_QUAD0_DBG_REG0 +
				       i * MLA_REG_QUAD_STRIDE);
		q_ftl_src[i] = ioread32(mdev->regs +
					MLA_REG_TLC_TMX_QUAD0_FTL_SRC +
					i * MLA_REG_TLC_TMX_QUAD_FTL_STRIDE);
		q_ftl_snap[i] = ioread32(mdev->regs +
					 MLA_REG_TLC_TMX_QUAD0_FTL_SNAP +
					 i * MLA_REG_TLC_TMX_QUAD_FTL_STRIDE);
		q_nf_src[i] = ioread32(mdev->regs +
				       MLA_REG_TLC_TMX_QUAD0_NON_FTL_SRC +
				       i * MLA_REG_TLC_TMX_QUAD_NF_STRIDE);
	}

	dma_sema = ioread32(mdev->regs + MLA_REG_DMA_SEMA_REG);
	for (i = 0; i < 4; i++) {
		dma_deadlock[i] = ioread32(mdev->regs + MLA_REG_DMA0_SEMA_DEADLOCK_TMO +
					   i * MLA_REG_DMA_STRIDE);
	}

	dev_err(mdev->dev,
		"dump[%s]: SYS_FTL sts=0x%08x snap=0x%08x sram_snap=0x%08x\n",
		tag ?: "?", sys_ftl_sts, sys_ftl_snap, sram_snap);
	dev_err(mdev->dev,
		"dump[%s]: DMC_F=0x%08x MIM_F=0x%08x MIM_snap=0x%08x TLC_FTL_snap=0x%08x TLC_NF_src=0x%08x\n",
		tag ?: "?", dmc_f, mim_f, mim_snap, tlc_ftl_snap, tlc_nf_src);
	dev_err(mdev->dev,
		"dump[%s]: QG0_setup=0x%08x QuadDBG q0=0x%08x q1=0x%08x q2=0x%08x q3=0x%08x\n",
		tag ?: "?", qg0_setup,
		quad_dbg[0], quad_dbg[1], quad_dbg[2], quad_dbg[3]);
	dev_err(mdev->dev,
		"dump[%s]: TMX_Q FTL_src [0]=0x%08x [1]=0x%08x [2]=0x%08x [3]=0x%08x\n",
		tag ?: "?", q_ftl_src[0], q_ftl_src[1], q_ftl_src[2], q_ftl_src[3]);
	dev_err(mdev->dev,
		"dump[%s]: TMX_Q FTL_snap [0]=0x%08x [1]=0x%08x [2]=0x%08x [3]=0x%08x\n",
		tag ?: "?", q_ftl_snap[0], q_ftl_snap[1], q_ftl_snap[2], q_ftl_snap[3]);
	dev_err(mdev->dev,
		"dump[%s]: TMX_Q NF_src   [0]=0x%08x [1]=0x%08x [2]=0x%08x [3]=0x%08x\n",
		tag ?: "?", q_nf_src[0], q_nf_src[1], q_nf_src[2], q_nf_src[3]);
	dev_err(mdev->dev,
		"dump[%s]: DMA_SEMA(global)=0x%08x (m4_push=%u m4_pop=%u push_map=0x%02x pop_map=0x%02x)\n",
		tag ?: "?", dma_sema,
		dma_sema & 0xff, (dma_sema >> 8) & 0xff,
		(dma_sema >> 16) & 0xff, (dma_sema >> 24) & 0xff);
	dev_err(mdev->dev,
		"dump[%s]: DMA_DEADLOCK d0=0x%08x d1=0x%08x d2=0x%08x d3=0x%08x\n",
		tag ?: "?",
		dma_deadlock[0], dma_deadlock[1], dma_deadlock[2], dma_deadlock[3]);
}

/**
 * mla_hw_recover() — fault-recovery sequence
 * @mdev: device to recover
 *
 * Quiesces IRQs, drives the PRC clock-and-reset register, re-enables
 * IRQ masks, and clears stale IRQ source bits.
 *
 * Callers are mla_recovery_work_fn() (workqueue, after a HW fault) and
 * mla_sched_device_reset() (MLA_OP_DEVICE_RESET, process context), so this is
 * allowed to sleep across the udelays inside mla_hw_reset(). Note the second
 * caller means this is reachable from a supported op, not only from a fault.
 *
 * The caller is responsible for the precondition mla_hw_reset() documents —
 * no jobs running across the reset. mla_sched_device_reset() drains the
 * in-flight job before it gets here; a wedged job can still time out, so
 * nothing in this function may do MMIO that could abort under a spinlock.
 *
 * Return: 0 on success; negative errno on failure (in which case the
 * caller marks the device fatal).
 */
int mla_hw_recover(struct mla_device *mdev)
{
	unsigned long flags;
	int ret;

	if (!mdev->regs)
		return -ENODEV;

	/* Wait for any in-flight prep to finish before we touch descriptor
	 * blobs / reset HW. Prep worker only reads/writes DRAM (no MMIO),
	 * so reset itself isn't racing MMIO, but flushing keeps
	 * prep_in_flight accounting clean and ensures no async prep is
	 * patching a blob concurrently with recovery.
	 */
	if (mdev->prep_wq)
		flush_workqueue(mdev->prep_wq);

	mla_hw_quiesce_irqs(mdev);

	/* Any failure here aborts recovery. mla_hw_reset() returns -ENODEV
	 * only when !mdev->prc_regs, i.e. we cannot reset the block at all —
	 * which should never happen, and if it does, continuing on to enable
	 * IRQs, clear stale sources, and touch the perfctr window on a block
	 * that just failed to reset would only compound the problem. Callers
	 * already handle failure correctly: mla_recovery_work_fn sets
	 * device_fatal and mla_sched_device_reset reports to userspace.
	 */
	ret = mla_hw_reset(mdev);
	if (ret) {
		dev_err(mdev->dev, "recover: hw_reset failed: %d\n", ret);
		return ret;
	}

	mla_hw_enable_irqs(mdev);

	/* Clear any stale source bits that may have been latched
	 * during the fault. We only clear bits we recognize so the
	 * write doesn't poke unmapped offsets.
	 */
	mla_hw_clear_stale_irqs(mdev);

	/* PRC reset wipes the DMA burst-size config in the MLA. Mark
	 * our cached values as "unknown" so the scheduler's burst-dance
	 * will re-program them on the next dispatch.
	 */
	mdev->cur_burst_rd = -1;
	mdev->cur_burst_wr = -1;

	/* DFM/tile/QG0 register state is wiped by the reset. */
	mdev->last_programmed_model = NULL;
	mdev->last_programmed_bank_idx = MLA_BANK_UNASSIGNED;

	/* Perf counters: the hard reset wiped the CTRL registers, and we do
	 * NOT re-apply the caller's selection. Counters measure a healthy
	 * run; if a job faulted badly enough to land here, the numbers are
	 * meaningless and the question is why the model failed, not how long
	 * its tiles were active. Handing back post-recovery measurements
	 * would point the investigation at the wrong thing.
	 *
	 * Dropping the selection also keeps this path honest about what the
	 * registers now hold, rather than having two behaviours to choose
	 * between based on state. Userspace reconfigures if it still wants
	 * counters; per-context fatal state means it has to notice the fault
	 * regardless.
	 *
	 * Cleared in SOFTWARE ONLY — deliberately not via
	 * mla_hw_perfctr_disable_all(), which would read-modify-write the four
	 * TLC perfctr CTRL registers under spin_lock_irqsave(&sched_lock). The
	 * reset above already wiped those registers, so the register I/O is
	 * redundant, and it is actively dangerous here: an MMIO read in this
	 * window can take a synchronous external abort when the reset landed
	 * under live DMA. That abort kills the task with sched_lock held and
	 * IRQs disabled, which leaks the lock — RCU stalls, then mmc0 (the
	 * rootfs) stops being serviced, and the board answers ping while
	 * refusing ssh. mla_sched_device_reset() drains the hardware first,
	 * but a wedged job can still time out there, so this path must not
	 * be able to take the board down.
	 */
	spin_lock_irqsave(&mdev->sched_lock, flags);
	mdev->perfctr_count = 0;
	memset(mdev->perfctr_signals, 0, sizeof(mdev->perfctr_signals));
	/* Bump the generation even though this is a software-only clear: the
	 * configuring context must be able to tell that its selection is gone,
	 * and mla_perfctr_release() must not disable slots a LATER owner
	 * installed after this recovery. Cheap, and it keeps every mutation of
	 * perfctr_* observable through one field.
	 */
	mdev->perfctr_gen++;
	spin_unlock_irqrestore(&mdev->sched_lock, flags);

	dev_info(mdev->dev, "recover: complete\n");
	return 0;
}

/* ---------------------------------------------------------------------- *
 *  Stale-IRQ clear
 *
 *  Run before each GO. A leftover bit from a previous job, reset, or
 *  spurious event can otherwise cause the next IRQ to look like a
 *  premature progdone.
 *
 *  Mirrors mla_rt's poll-loop ack at modalix_driver.cc:486-487, BUT
 *  only clears the specific bits we know about. Writing arbitrary
 *  bits back (e.g. level-sensitive system-status bits) on this
 *  silicon can re-arm sources we don't intend to touch.
 * ----------------------------------------------------------------------
 */

/**
 * mla_hw_clear_stale_irqs() — W1C-clear leftover non-fatal source bits
 * @mdev: device whose TLC source register we'll write
 *
 * Run before each GO and from the recovery worker. A leftover bit
 * from a previous job, reset, or spurious event can otherwise cause
 * the next IRQ to look like a premature progdone.
 *
 * Only clears bits we explicitly recognize (progdone, halt_trig,
 * sync_ctl) — writing arbitrary bits back to the source register
 * caused RCU stalls in silicon validation. The MSC
 * aggregator clear is deliberately skipped here: it's a read-only
 * shadow of the leaf-EN gating, and writing it without the
 * corresponding EN bits set is unsafe on this silicon.
 */
void mla_hw_clear_stale_irqs(struct mla_device *mdev)
{
	u32 v, mask;

	/* Only clear progdone + halt_trig + sync_ctl bits. */
	mask = MLA_TLC_NF_QUADGRP_PROG_DONE_MASK |
	       MLA_TLC_NF_QUADGRP_HALT_TRIG_MASK |
	       (1u << MLA_TLC_NF_SYNC_CTL_INTR_BIT);

	v = ioread32(mdev->regs + MLA_REG_TLC_NON_FTL_INTR_SRC);
	v &= mask;
	if (v)
		iowrite32(v, mdev->regs + MLA_REG_TLC_NON_FTL_INTR_SRC);

	/* The MSC aggregator status is a read-only shadow of the leaf-en
	 * gating; writing it without the corresponding EN bits set produced
	 * RCU stalls in validation. So we clear only the TLC leaf here —
	 * enable_irqs sets the aggregator EN mask, and the IRQ handler
	 * W1C-acks aggregator status when it fires.
	 */
}

/* ---------------------------------------------------------------------- *
 *  IFM/OFM cache maintenance is NOT done here — it is the caller's job
 *
 *  There is deliberately no per-submit IFM/OFM cache sync in this driver.
 *  Cache maintenance is required at the moment the CPU touches a buffer,
 *  not once per job dispatch, and only the caller knows when that was.
 *
 *  Per gst-api.h's mla_alloc_dmabuf contract, the caller must bracket its
 *  own CPU access with DMA_BUF_IOCTL_SYNC — END|WRITE after writing an
 *  IFM or memsetting an OFM, START|READ before reading an OFM. For an
 *  mla-exported buffer that lands in mla_dmabuf_{begin,end}_cpu_access
 *  (mla_dmabuf.c); for a carveout-heap buffer, in the heap's own
 *  cpu-access ops.
 *
 *  Separately, the driver does its own maintenance for descriptor-blob
 *  and reloc data (mla_pool_sync_for_device, mla_pool_sync_range_for_device,
 *  and the reloc range flush in mla_hw_apply_relocs), because that memory
 *  is the driver's own write and syncing it is the driver's obligation.
 * ----------------------------------------------------------------------
 */

/* ---------------------------------------------------------------------- *
 *  Per-job register programming
 * ----------------------------------------------------------------------
 */

/*
 * Compose the QG0 setup register value: flush length + quad/DFM map
 * bits per the model's active-tile / DMA-descriptor counts. Modalix
 * is QG0-only, so this is the entire quad-map programming.
 */
static u32 mla_hw_compose_qg0_reg0(const struct mla_model *m)
{
	u32 v = MLA_QG0_FLUSH_LEN_DEFAULT << MLA_QG0_FLUSH_LEN_SHIFT;

	if (m->num_active_tiles)
		v |= 0xfu << MLA_QG0_QUAD_MAP_SHIFT;
	if (m->num_dma_descriptors)
		v |= 0xfu << MLA_QG0_DFM_MAP_SHIFT;
	return v;
}

/* ---------------------------------------------------------------------- *
 *  MODEL_DEFINE-shaped DFM + tile loaders.
 *
 *  Models carry tile_info_typed[] and dma_info_typed[] directly; the
 *  loaders below resolve section-relative addresses against
 *  section_bufs[] and run the iowrite sequences that program the MLA
 *  registers.
 * ----------------------------------------------------------------------
 */

static void mla_hw_init_dfm(struct mla_device *mdev,
			    const struct mla_model *m,
			    u8 bank_idx)
{
	const struct mla_dma_unit_desc *du = m->dma_info_typed;
	const struct mla_buffer *bank_blob;
	int i;

	/* All four DMA units point at the same dma.descs section (their
	 * desc_section_index values agree; validated at MODEL_DEFINE).
	 * When banks > 0, this specific job's bank owns a distinct
	 * copy of that section — DFM must fetch from bank_idx, not from
	 * the primary section_bufs slot.
	 */
	bank_blob = mla_model_bank_blob(m, bank_idx);

	for (i = 0; i < MLA_DMA_UNIT_COUNT_SIZE; i++) {
		u32 base = MLA_REG_DMA0_DFM_CONFIG0 + i * MLA_REG_DMA_STRIDE;
		const struct mla_buffer *blob;
		u64 fetch_addr;
		u32 dfm0, dfm1, dfm2;

		/* Bounds-check before deref — bogus user input must not
		 * fault the kernel. Skip the unit (zero its config) on
		 * invalid input rather than panic.
		 */
		if (du[i].desc_section_index >= m->num_sections) {
			dev_warn_ratelimited(mdev->dev,
				"init_dfm: dma[%d] desc_section_index %u >= num_sections %u; zeroing config\n",
				i, du[i].desc_section_index, m->num_sections);
			writel_relaxed(0, mdev->regs + base + 0);
			writel_relaxed(0, mdev->regs + base + 4);
			writel_relaxed(0, mdev->regs + base + 8);
			continue;
		}
		blob = bank_blob;
		if (!blob) {
			writel_relaxed(0, mdev->regs + base + 0);
			writel_relaxed(0, mdev->regs + base + 4);
			writel_relaxed(0, mdev->regs + base + 8);
			continue;
		}
		/* fetch_addr is the section's kernel-chosen phys plus the
		 * per-unit blob offset — never a user-space-supplied address.
		 */
		fetch_addr = (u64)blob->dma_addr + du[i].desc_blob_offset;
		dfm0 = upper_32_bits(fetch_addr) << 24;
		dfm1 = lower_32_bits(fetch_addr);
		dfm2 = (du[i].dfm_fetch_map_type << 1) |
		       (du[i].desc_count           << 12);

		if (mla_debug & MLA_DBG_HOT)
			dev_info(mdev->dev,
				 "v2_trace: dfm[%d] sec_idx=%u sec_phys=0x%llx blob_off=0x%llx fetch_addr=0x%llx desc_count=%u type=%u dfm0=0x%08x dfm1=0x%08x dfm2=0x%08x\n",
				 i, du[i].desc_section_index,
				 (unsigned long long)blob->dma_addr,
				 (unsigned long long)du[i].desc_blob_offset,
				 (unsigned long long)fetch_addr,
				 du[i].desc_count, du[i].dfm_fetch_map_type,
				 dfm0, dfm1, dfm2);

		writel_relaxed(dfm0, mdev->regs + base + 0);
		writel_relaxed(dfm1, mdev->regs + base + 4);
		writel_relaxed(dfm2, mdev->regs + base + 8);
	}
}

static void mla_hw_load_tiles(struct mla_device *mdev,
			      const struct mla_model *m)
{
	const struct mla_tile_desc *td = m->tile_info_typed;
	u32 i;
	int q;

	for (i = 0; i < m->num_active_tiles; i++) {
		const struct mla_buffer *instr_buf;
		u32 axi_off = td[i].mim_base_reg - MLA_APB_BASE;
		u64 instr_phys;
		u32 mim_base_val;

		if (td[i].instr_section_index >= m->num_sections) {
			dev_warn_ratelimited(mdev->dev,
				"load_tiles: tile[%u] instr_section_index %u >= num_sections %u; skipping\n",
				i, td[i].instr_section_index, m->num_sections);
			continue;
		}
		instr_buf = m->section_bufs[td[i].instr_section_index];
		if (!instr_buf)
			continue;
		/* base_val is the per-tile instruction blob phys right-shifted
		 * 9 bits — the MIM hardware register expects a 41-bit phys
		 * encoded as the top 32 bits. Kernel-chosen INSTR section
		 * phys + per-tile instr_offset, shifted right 9. The
		 * user-space mim_base_val field is ignored; only
		 * mim_base_reg + instr_section_index + instr_offset are
		 * authoritative inputs.
		 */
		instr_phys = (u64)instr_buf->dma_addr + td[i].instr_offset;
		mim_base_val = (u32)(instr_phys >> 9);

		if (mla_debug & MLA_DBG_HOT)
			dev_info(mdev->dev,
				 "v2_trace: tile[%u] sec_idx=%u sec_phys=0x%llx instr_off=0x%x instr_phys=0x%llx mim_reg=0x%x mim_val=0x%08x\n",
				 i, td[i].instr_section_index,
				 (unsigned long long)instr_buf->dma_addr,
				 td[i].instr_offset,
				 (unsigned long long)instr_phys,
				 td[i].mim_base_reg, mim_base_val);

		writel_relaxed(mim_base_val, mdev->regs + axi_off);
	}

	for (q = 0; q < 4; q++) {
		u32 setup_off = MLA_REG_QUAD0_SETUP_REG0 + q * MLA_REG_QUAD_STRIDE;
		u32 dbg_off   = MLA_REG_QUAD0_DBG_REG0   + q * MLA_REG_QUAD_STRIDE;

		writel_relaxed(m->quad_tile_bitmask[q], mdev->regs + setup_off);
		writel_relaxed(MLA_QUAD_DBG_PROGRAM_DONE, mdev->regs + dbg_off);
	}

	writel_relaxed(mla_hw_compose_qg0_reg0(m),
		       mdev->regs + MLA_REG_QUADGRP0_SETUP_REG0);
}

/* ---------------------------------------------------------------------- *
 *  Kernel-side DMA descriptor patching.
 *
 *  Each DMA descriptor (32 B, Dma_Desc_Michelangelo from
 *  externals/mla-toolchain/michelangelo/arch/dma_desc.h) has its
 *  dram_addr field as a 40-bit bitfield spanning bits 86..125 (byte
 *  10 bit 6 through byte 15 bit 5). In the union word[4] model that's
 *  bits 22..61 of word[1] (8 bytes at descriptor offset 8).
 *
 *  Userspace ships MODEL_DEFINE with placeholder zone bases (typically
 *  zero) and a side-table of {desc location, zone, port,
 *  offset-within-zone}. Section-tagged entries are resolved once at
 *  MODEL_DEFINE; IFM/OFM/RELOC-tagged entries are resolved per-job at
 *  JOB_SUBMIT.
 *
 *  Reads section_bufs[dma_info_typed[0].desc_section_index] (mla-rt
 *  concatenates all four DMA units' blobs into one section;
 *  MODEL_DEFINE validation cross-checks that all four
 *  desc_section_index values agree). Runs from softirq context —
 *  no sleeping, no allocations.
 * ----------------------------------------------------------------------
 */

/* MLA_DESC_* descriptor-field macros live in mla_regs.h — one definition
 * shared with v2_apply_static_section_relocs(), which patches the same field.
 */

static int mla_hw_apply_relocs(struct mla_job *job, u8 bidx, bool from_worker)
{
	struct mla_model *m = job->model;
	struct mla_buffer *blob_buf;
	void *base;
	size_t blob_size;
	size_t dirty_lo = SIZE_MAX;   /* min offset dirtied by this pass */
	size_t dirty_hi = 0;          /* max offset+8 dirtied by this pass */
	bool stolen = false;
	u32 i;

	if (!m->reloc_list || !m->reloc_count)
		return 0;
	if (!m->section_bufs || !m->dma_info_typed)
		return -EFAULT;

	/* Descriptor blob's CPU vaddr. mla-rt-K places all four DMA
	 * units' descriptors in the same section (just at different
	 * offsets); we use unit 0's section as the canonical base. The
	 * bank accessor picks between the primary section (bank[0]
	 * sentinel) and a private per-bank copy for jobs whose peer
	 * currently holds bank[0].
	 */
	if (m->dma_info_typed[0].desc_section_index >= m->num_sections) {
		dev_err(job->ctx->mdev->dev,
			"apply_relocs: desc_section_index %u >= num_sections %u\n",
			m->dma_info_typed[0].desc_section_index, m->num_sections);
		return -EINVAL;
	}
	/* bidx is passed in, never re-read from job->bank_idx: finish_job clears
	 * that field on the owning job, and mla_model_bank_blob() answers an
	 * out-of-range index with bank[0], so a worker re-reading it mid-loop
	 * would silently redirect its writes into another job's bank.
	 */
	blob_buf = mla_model_bank_blob(m, bidx);
	if (!blob_buf || !blob_buf->cpu_vaddr)
		return -EFAULT;
	base      = blob_buf->cpu_vaddr;
	blob_size = blob_buf->size;

	if (mla_debug & MLA_DBG_TRACE)
		dev_info(job->ctx->mdev->dev,
			 "v2_trace: model=\"%s\" apply_relocs reloc_count=%u static=%u dynamic_start=%u num_ifm=%u num_ofm=%u num_sections=%u blob_phys=0x%llx blob_size=%zu\n",
			 m->name,
			 m->reloc_count, m->static_reloc_count,
			 m->static_reloc_count,
			 job->num_ifm, job->num_ofm,
			 m->num_sections, (unsigned long long)blob_buf->dma_addr,
			 blob_size);

	/* Stamp AFTER the trace/section-lookup setup, BEFORE the per-reloc
	 * loop, so trace_reloc_loop_us isolates the loop body cost from
	 * function prologue / desc-section resolution / MLA_DBG_TRACE emit.
	 */
	job->ts_reloc_loop_start = ktime_get();

	/* Static-tag prefix [0..static_reloc_count) was already spliced
	 * into the blob at MODEL_DEFINE time — skip on every job.
	 */
	for (i = m->static_reloc_count; i < m->reloc_count; i++) {
		const struct mla_reloc_entry *r = &m->reloc_list[i];
		struct mla_buffer *b = NULL;
		u64 dram_addr;
		__le64 *qw;
		u64 v;
		size_t off;

		/* Optimisation only: the writer ref held across this loop is what
		 * keeps the bank unclaimable however long we stay. Bailing just
		 * stops spending descriptor RMWs once dispatch has taken over and
		 * shortens the drain. Every 32nd iteration keeps the test off the
		 * hot path.
		 *
		 * Suppressed while the delay injection is armed, because that knob
		 * exists to hold a worker inside this loop across a completion,
		 * which is exactly what bailing early would prevent.
		 */
		if (from_worker && !mla_inject_prep_delay_us &&
		    (i & 0x1f) == 0 &&
		    atomic_read(&m->banks[bidx].state) != MLA_BANK_PREPARING) {
			stolen = true;
			break;
		}

		/* Stalling on the first dynamic reloc leaves every write in this
		 * loop stale, which is what exposes a bank released and re-claimed
		 * while its worker is still inside it. A stall before this function
		 * exposes nothing: dispatch sets job->prepped on the way past, so a
		 * late worker returns without writing.
		 *
		 * from_worker is required because dispatch reaches this function
		 * from hard IRQ, where sleeping panics the board.
		 */
		if (unlikely(mla_inject_prep_delay_us && from_worker &&
			     i == m->static_reloc_count)) {
			static atomic_t nth = ATOMIC_INIT(0);

			/* Alternating keeps a peer worker publishing READY, so a
			 * victim exists to corrupt. Skipping only the sleep keeps a
			 * test hook from changing which descriptors get written.
			 */
			if (!WARN_ON_ONCE(in_interrupt()) &&
			    (atomic_inc_return(&nth) & 1)) {
				uint d = mla_inject_prep_delay_us;

				usleep_range(d, d + (d >> 2) + 1);
			}
		}

		switch (r->zone_tag) {
		case MLA_RELOC_ZONE_IFM:
			if (!job->ifm_bufs || r->zone_index >= job->num_ifm)
				return -EINVAL;
			b = job->ifm_bufs[r->zone_index];
			if (!b)
				return -EINVAL;
			dram_addr = (u64)b->dma_addr + r->placeholder_offset;
			if (job->ifm_offsets)
				dram_addr += job->ifm_offsets[r->zone_index];
			break;
		case MLA_RELOC_ZONE_OFM:
			if (!job->ofm_bufs || r->zone_index >= job->num_ofm)
				return -EINVAL;
			b = job->ofm_bufs[r->zone_index];
			if (!b)
				return -EINVAL;
			dram_addr = (u64)b->dma_addr + r->placeholder_offset;
			if (job->ofm_offsets)
				dram_addr += job->ofm_offsets[r->zone_index];
			break;
		case MLA_RELOC_ZONE_SECTION:
			if (r->zone_index >= m->num_sections)
				return -EINVAL;
			b = m->section_bufs[r->zone_index];
			if (!b)
				return -EINVAL;
			dram_addr = (u64)b->dma_addr + r->placeholder_offset;
			break;
		default:
			/* MLA_RELOC_ZONE_RELOC lands here. A LoRA port's address
			 * does not change per job, so its descriptors are already
			 * spliced — at MODEL_DEFINE with the ELF's default binding,
			 * and again by MLA_OP_MODEL_UPDATE_RELOC, which patches
			 * EVERY bank under bank_grow_lock while the model is
			 * quiesced. Do not "fix" this by patching reloc ports here:
			 * that would put 278-394 extra descriptor RMWs on every
			 * dispatch (~28-40k per token on gemma) to reproduce bytes
			 * that are already in the blob.
			 *
			 * Per-JOB reloc bindings (MLA_CAP_RELOC_PER_JOB) are what
			 * would need an arm here, driven by a generation compare
			 * rather than an unconditional walk.
			 */
			continue;
		}

		/* The field is 40 bits and the mask below TRUNCATES, which would
		 * program the DMA at a wrong address rather than fail. Judged
		 * unreachable on Modalix — DMS0 is 40 GiB at 0x14_0000_0000, so
		 * ~38 bits are in use and nothing approaches 1 TiB — but that is
		 * inference, not a guarantee, and MLA_OP_INFO advertises
		 * DMA_BIT_MASK(64). Make it audible if it ever becomes reachable.
		 *
		 * WARN_ONCE, not dev_warn: this runs in softirq, once per reloc
		 * entry, and a warn in this loop would be a hot-path printk.
		 */
		WARN_ONCE(dram_addr > MLA_DESC_DRAM_ADDR_MASK,
			  "mla: reloc dram_addr 0x%llx exceeds the %u-bit descriptor field; truncating\n",
			  (unsigned long long)dram_addr,
			  (unsigned int)MLA_DESC_DRAM_ADDR_BITS);
		dram_addr &= MLA_DESC_DRAM_ADDR_MASK;
		off = (size_t)r->dma_offset +
		      (size_t)r->desc_index * MLA_DESC_SIZE_BYTES;
		if (off + MLA_DESC_SIZE_BYTES > blob_size) {
			dev_err(job->ctx->mdev->dev,
				"apply_relocs[%u]: off %zu past blob_size %zu\n",
				i, off, blob_size);
			return -EINVAL;
		}
		qw = (__le64 *)((u8 *)base + off +
				MLA_DESC_DRAM_ADDR_WORD * 8);
		v = le64_to_cpu(*qw);
		if (mla_debug & MLA_DBG_HOT)
			dev_info(job->ctx->mdev->dev,
				 "v2_trace: reloc[%u] tag=%u idx=%u dma_off=%u desc_idx=%u placeholder=0x%llx -> dma_addr=0x%llx (was qw=0x%llx)\n",
				 i, r->zone_tag, r->zone_index, r->dma_offset,
				 r->desc_index,
				 (unsigned long long)r->placeholder_offset,
				 (unsigned long long)dram_addr,
				 (unsigned long long)v);
		v &= ~(MLA_DESC_DRAM_ADDR_MASK << MLA_DESC_DRAM_ADDR_SHIFT);
		v |= dram_addr << MLA_DESC_DRAM_ADDR_SHIFT;
		*qw = cpu_to_le64(v);

		/* Track the range this pass dirtied so the outer sync flushes
		 * only [dirty_lo, dirty_hi). The reloc list on typical models
		 * (~100-300 dynamic relocs on cache_token2047; 2 on resnet_50)
		 * dirties a tiny fraction of a multi-MB descriptor blob;
		 * flushing the whole blob would cost 100s of µs. No-op on
		 * cma_backed=false.
		 */
		{
			size_t qw_off = off + MLA_DESC_DRAM_ADDR_WORD * 8;

			if (qw_off < dirty_lo)
				dirty_lo = qw_off;
			if (qw_off + 8 > dirty_hi)
				dirty_hi = qw_off + 8;
		}
	}

	/* Publish only the dirtied cache lines to memory. The precomputed
	 * flush_regions list was built at MODEL_DEFINE from the same
	 * dynamic-reloc set this loop just wrote, so it always covers
	 * exactly the cache lines touched here (one line per reloc, runs
	 * of adjacent lines coalesced) — unlike a single [dirty_lo,
	 * dirty_hi) span, which degrades to the whole blob once relocs are
	 * spread across a multi-MB descriptor blob.
	 *
	 * Fallback to full-buffer sync when flush_regions is NULL — either
	 * mla_reloc_range_flush=0 (module-param kill switch) or the
	 * MODEL_DEFINE-time alloc failed. Correctness preserved either way.
	 */
	(void)dirty_lo; (void)dirty_hi;
	/* Nothing to publish once dispatch owns the bank: it ran its own full
	 * pass and flushed before GO. Skipping also keeps this out of a bank the
	 * hardware is actively fetching, which a writeback of identical bytes
	 * should survive but which there is no reason to enter.
	 */
	if (stolen)
		return -ECANCELED;
	if (m->flush_regions && m->num_flush_regions) {
		u32 k;

		for (k = 0; k < m->num_flush_regions; k++)
			mla_pool_sync_range_for_device(job->ctx->mdev, blob_buf,
						       m->flush_regions[k].offset,
						       m->flush_regions[k].len);
	} else {
		mla_pool_sync_for_device(job->ctx->mdev, blob_buf);
	}
	return 0;
}

/**
 * mla_hw_prep_work_fn() — async prep-ahead worker, per-bank
 * @work: pointer to job->prep_work
 *
 * Runs on mdev->prep_wq while HW is executing another job. The bank
 * assigned to this job by mla_sched_find_prep_candidate_locked is in
 * PREPARING state; the worker's job is to run apply_relocs + flush
 * and advance the bank to READY.
 *
 * Does NOT own the bank for its whole run: dispatch may take it at any point
 * via PREPARING -> ACTIVE. What the worker holds is the writer ref taken with
 * the claim, which forbids the bank being handed to a different job until the
 * worker is out of the descriptor loop. So a steal costs this worker its
 * publish, never the next owner's descriptors.
 *
 * Exit path always drops the job fence ref taken at queue time, resets
 * mdev->prep_in_flight so the next candidate can be prepped, and restarts
 * dispatch in case a bank shortage refused one while we held ours.
 */
void mla_hw_prep_work_fn(struct work_struct *work)
{
	struct mla_job *job = container_of(work, struct mla_job, prep_work);
	struct mla_device *mdev = job->ctx->mdev;
	/* prep_bank_idx, not bank_idx: dispatch may already have stolen the bank,
	 * run the job and had finish_job clear bank_idx before this worker was
	 * ever scheduled, and the writer ref still has to come back.
	 */
	u8 bidx = job->prep_bank_idx;
	struct mla_desc_bank *bank;
	int ret, inject;

	/* No bank means no writer ref was taken for us at claim time, so this is
	 * the one exit that must not drop one.
	 */
	if (bidx == MLA_BANK_UNASSIGNED)
		goto out;
	bank = &job->model->banks[bidx];

	/* Bail only when the job is genuinely finished -- cancelled, faulted or
	 * completed -- or the device/context is fatal, freeing the bank so a
	 * later walker can pick it up.
	 *
	 * MLA_JOB_RUNNING must NOT bail: mla_sched_pick_reserve_locked() sets
	 * RUNNING before dispatch runs, so a healthy job the scheduler has just
	 * picked is RUNNING while this worker is still finishing. Bailing would
	 * free the bank dispatch is about to mark ACTIVE and clear the bank_idx
	 * it needs to locate the blob this worker patched, so dispatch would
	 * issue GO against a buffer still holding the peer job's address.
	 *
	 * A racing dispatch needs no state test here: the cmpxchg below fails
	 * when dispatch has taken the bank.
	 */
	if (READ_ONCE(mdev->device_fatal) ||
	    READ_ONCE(job->ctx->fatal) ||
	    READ_ONCE(job->state) == MLA_JOB_DONE) {
		/* cmpxchg, not atomic_set: dispatch may already have stolen this
		 * bank and published ACTIVE, and stomping FREE over that would
		 * hand a running job's bank to the next claimer. Losing means
		 * dispatch owns it, so leave bank_idx alone for it to resolve.
		 *
		 * Publishing FREE while still holding the writer ref is safe --
		 * a claimer refuses a FREE bank with writers, so nothing can take
		 * it until out_release below.
		 */
		if (atomic_cmpxchg(&bank->state, MLA_BANK_PREPARING,
				   MLA_BANK_FREE) == MLA_BANK_PREPARING)
			job->bank_idx = MLA_BANK_UNASSIGNED;
		goto out_release;
	}

	inject = mla_inject_prep_fail;
	if (inject < 0) {
		/* Worker-site arm of the pre-GO prep-failure injection
		 * (negative values; positive values arm the dispatch site —
		 * see mla_hw_program_and_start). This site exists because the
		 * worker's failure handling is a DIFFERENT path from
		 * dispatch's: this one must decline to publish rather than
		 * return an error to a caller that finalizes the job.
		 *
		 * Consume by ASSIGNING inject+1, not by ++: up to
		 * MLA_MAX_BANKS-1 workers run concurrently, and two racing
		 * increments from -1 would land on +1 — silently re-arming the
		 * dispatch site instead of disarming. Assigning means racers
		 * converge on 0 and the count can never cross zero. The
		 * consequence of the race is that the number of injections is a
		 * lower bound of |N|, not exactly |N|.
		 *
		 * Deliberately SILENT here — the report is emitted after the
		 * cmpxchg below. A dev_warn between this decision and the
		 * cmpxchg costs milliseconds at 115200 baud, which hands the
		 * bank to dispatch and means the branch under test is never
		 * reached.
		 */
		mla_inject_prep_fail = inject + 1;
		ret = -EINVAL;
	} else {
		ret = mla_hw_prep_job(job, bidx, true);
	}

	if (ret == -ECANCELED)
		atomic_inc(&mdev->prep_aborted);

	/* mla_hw_prep_job() has already published job->prepped with release
	 * semantics on success, which is what lets the state transition below
	 * mean "these descriptors are complete".
	 *
	 * On failure the bank stays owned by this job as READY with
	 * prepped == false, so dispatch re-runs prep inline, hits the same
	 * deterministic error, and finalizes the job through
	 * mla_sched_dispatch()'s pre-GO contract. Releasing to FREE here
	 * instead would require clearing bank_idx too, and the two writes
	 * cannot be made atomic together: dispatch reading a still-assigned
	 * bank_idx against a FREE bank would publish ACTIVE over a bank the
	 * prep walker may already have handed to another job.
	 */
	if (atomic_cmpxchg(&bank->state, MLA_BANK_PREPARING,
			   MLA_BANK_READY) == MLA_BANK_PREPARING) {
		/* Finalization can land anywhere inside prep, so the bail above
		 * is not sufficient on its own. Re-test AFTER publishing:
		 * finish_job leaves a PREPARING bank alone, so a job cancelled
		 * mid-prep would otherwise strand this bank READY with nothing
		 * to reclaim it.
		 */
		if ((READ_ONCE(mdev->device_fatal) ||
		     READ_ONCE(job->ctx->fatal) ||
		     READ_ONCE(job->state) == MLA_JOB_DONE) &&
		    atomic_cmpxchg(&bank->state, MLA_BANK_READY,
				   MLA_BANK_FREE) == MLA_BANK_READY) {
			job->bank_idx = MLA_BANK_UNASSIGNED;
			/* out_release, not out: this path still owes the writer
			 * ref. Skipping it strands the bank FREE with a writer
			 * forever, and mla_bank_try_claim() refuses that for the
			 * life of the model.
			 */
			goto out_release;
		}
		if (ret) {
			atomic_inc(&mdev->prep_failures);
			/* After the cmpxchg: a printk before it costs milliseconds
			 * at console baud rates, long enough to change what the
			 * branch under test observes.
			 */
			dev_warn(mdev->dev,
				 "prep worker: job %u prep failed (%d)%s; bank kept READY, dispatch will re-prep and finalize\n",
				 job->id, ret, inject < 0 ? " [FAULT_INJECT]" : "");
		}
	} else {
		/* Dispatch took the bank while we were writing it. Legal: it reran
		 * the whole reloc pass itself, and the writer ref held across the
		 * loop kept the bank out of any other job's hands until we were
		 * done. Nothing of ours to publish or release.
		 */
		atomic_inc(&mdev->prep_stolen);
	}

out_release:
	/* Ordered after the publish above, not before it: dropping the ref first
	 * would let the bank go FREE, be re-claimed by another job, and absorb
	 * our cmpxchg(PREPARING -> READY) as if that job's prep were finished.
	 * The barrier stops our descriptor writes being reordered past the
	 * release, since a claimer that observes zero may start writing at once.
	 */
	smp_mb__before_atomic();
	atomic_dec(&bank->writers);

out:
	atomic_dec(&mdev->prep_in_flight);
	mla_job_put(job);
	/* Unconditional: the bank this worker just released may be the one a
	 * dispatch was refused for, and with no completion pending nothing else
	 * would retry. Must follow the decrement and the put -- holding the prep
	 * slot across dispatch blocks the next prep, and @job may already be
	 * freed.
	 */
	mla_sched_dispatch_next(mdev);
}

/**
 * mla_hw_prep_job() — reloc + cache flush for one job
 * @job: job to prepare
 *
 * Runs the two pieces of dispatch that don't touch device MMIO and thus
 * don't require exclusive HW ownership: patch DMA descriptors with
 * runtime IFM/OFM phys addresses, then flush the dirtied cache lines.
 * Idempotent via job->prepped.
 *
 * Return: 0 on success; -ENODEV if regs/model are unmapped; reloc
 * errors propagate from mla_hw_apply_relocs().
 */
int mla_hw_prep_job(struct mla_job *job, u8 bidx, bool from_worker)
{
	struct mla_device *mdev = job->ctx->mdev;
	struct mla_model  *m    = job->model;
	int ret;

	if (!mdev->regs || !m)
		return -ENODEV;
	/* Acquire pairs with the release below. Dispatch may reach this while the
	 * prep worker for the same job is still running, so observing the flag
	 * has to also guarantee that worker's descriptor writes are visible.
	 */
	if (smp_load_acquire(&job->prepped))
		return 0;

	/* Patch DMA descriptors with the actual phys_addrs of the supplied
	 * IFM/OFM (and SECTION) buffers before the DFM/tile programming
	 * reads them.
	 *
	 * apply_relocs does its own range-scoped cache flush for the exact
	 * bytes it dirtied (see the dirty_lo/dirty_hi tracking there). No
	 * blob-wide sync here — a dma_sync_single_for_device over a
	 * multi-MB blob would swamp the savings from the WB alias.
	 *
	 * No IRQ-source clear here — mla_hw_clear_stale_irqs() runs
	 * elsewhere in the dispatch sequence; mla_rt only acks inside the
	 * IRQ handler, never preemptively, and the kernel driver matches
	 * that.
	 */
	ret = mla_hw_apply_relocs(job, bidx, from_worker);
	if (ret)
		return ret;

	/* The single publish point for both callers. Timestamp stamps live at the
	 * dispatch call site so that a late worker run doesn't overwrite
	 * dispatch's krlc/kdma numbers.
	 */
	/* release: a dispatch observing the flag must see the writes above too */
	smp_store_release(&job->prepped, true);
	return 0;
}

/**
 * mla_hw_program_and_start() — program the MLA for one job and write GO
 * @job: job to dispatch (must be in mdev->in_flight)
 *
 * Sequence:
 *  1. Patch DMA descriptors with runtime IFM/OFM phys addresses (via prep)
 *  2. Sync IFM/OFM dma-bufs for device (when mla_do_dma_sync=1) (via prep)
 *  3. Program DFM and tile registers
 *  4. Write GO via QUADGRP0_SETUP_REG1 (the cycle_count register)
 *  5. Arm the watchdog timer
 *
 * Return: 0 on dispatch (PROGDONE will arrive via IRQ later); -ENODEV
 * if regs/model are unmapped; reloc errors propagate from
 * mla_hw_apply_relocs().
 */
int mla_hw_program_and_start(struct mla_job *job)
{
	struct mla_device *mdev = job->ctx->mdev;
	struct mla_model  *m    = job->model;
	struct mla_desc_bank *bank;
	bool inline_claim = false;
	u8 bidx;
	int ret;

	if (!mdev->regs || !m)
		return -ENODEV;

	/* Bank ownership. An assigned bank is taken from READY, or from PREPARING
	 * by stealing it from its prep worker; an unassigned job claims a FREE
	 * bank inline. Stealing is safe only because the worker holds a writer ref
	 * across its descriptor loop, so the bank cannot be released to another
	 * job until the worker is out of it.
	 */
	bidx = job->bank_idx;
	if (bidx != MLA_BANK_UNASSIGNED) {
		struct mla_desc_bank *b = &m->banks[bidx];
		int st;

		/* Two claims, then one decision: st ends up holding whichever
		 * state actually blocked us, which is what the counters need.
		 */
		st = atomic_cmpxchg(&b->state, MLA_BANK_READY, MLA_BANK_ACTIVE);
		if (st != MLA_BANK_READY)
			st = atomic_cmpxchg(&b->state, MLA_BANK_PREPARING,
					    MLA_BANK_ACTIVE);

		if (st == MLA_BANK_READY) {
			/* The worker finished; this acquire pairs with its release,
			 * so its descriptor bytes are visible to us.
			 */
		} else if (st == MLA_BANK_PREPARING) {
			atomic_inc(&mdev->dispatch_stole_preparing);
			/* job->prepped is left alone deliberately: it is published
			 * release/acquire, so if the worker got far enough to set it
			 * we can use its work, and otherwise mla_hw_prep_job() below
			 * does the pass inline. Either way the worker's remaining
			 * writes are this same job's addresses, identical to ours.
			 */
		} else {
			/* The bank is not ours: something released it between the
			 * pick and here. One route is a fatalized context, since
			 * mla_sched_pick_reserve_locked() gates on
			 * mdev->device_fatal but NOT on job->ctx->fatal, so a
			 * queued job whose context just died is still dispatched
			 * while the prep walker skips it and its worker bails and
			 * frees the bank. The counters below split the cases so a
			 * release with no fatal set is distinguishable from one
			 * this branch already explains.
			 *
			 * Drop the stale assignment and claim a bank properly
			 * below. Must NOT return -EAGAIN: the pre-GO contract
			 * finalizes the job on error, so a transient loss here
			 * would fail a perfectly good job.
			 */
			atomic_inc(&mdev->dispatch_bank_not_ready);
			if (READ_ONCE(mdev->device_fatal) ||
			    READ_ONCE(job->ctx->fatal))
				atomic_inc(&mdev->dispatch_bnr_fatal);
			else
				atomic_inc(&mdev->dispatch_bnr_unexplained);
			if (st == MLA_BANK_ACTIVE)
				atomic_inc(&mdev->dispatch_bnr_was_active);
			job->bank_idx = MLA_BANK_UNASSIGNED;
			bidx = MLA_BANK_UNASSIGNED;
		}
	}
	if (bidx == MLA_BANK_UNASSIGNED) {
		u32 k, n = READ_ONCE(m->num_banks);

		for (k = 0; k < n; k++) {
			struct mla_desc_bank *b = &m->banks[k];

			if (mla_bank_try_claim(m, k)) {
				job->bank_idx = (u8)k;
				bidx = (u8)k;
				break;
			}
			/* FREE-with-writers only; see the matching test in
			 * mla_sched_find_prep_candidate_locked() for why the state
			 * test is what makes this counter mean anything.
			 */
			if (atomic_read(&b->state) == MLA_BANK_FREE &&
			    atomic_read(&b->writers))
				atomic_inc(&mdev->dispatch_bank_draining);
		}
		if (bidx == MLA_BANK_UNASSIGNED) {
			/* Borrowing bank[0] would write this job's descriptors
			 * into a bank owned by another job's worker or by the
			 * running job. With no bank to run in, refusing is the
			 * only safe answer; mla_model_grow_banks() is
			 * best-effort, so ENOMEM plus a higher-priority job
			 * jumping one that holds a bank reaches this.
			 */
			atomic_inc(&mdev->dispatch_no_free_bank);
			dev_warn_ratelimited(mdev->dev,
				"dispatch: no free bank for model=\"%.20s\"; refusing job %u\n",
				m->name, job->id);
			return -EAGAIN;
		}
		/* Whatever a prep worker may have written into a bank we no longer
		 * hold is not ours to trust, so this claim always re-preps.
		 */
		job->prepped = false;
		inline_claim = true;
	}
	bank = &m->banks[bidx];

	/* Count at the DISPATCH site only. mla_hw_prep_job() itself is
	 * also called by the async prep worker, so instrumenting inside it
	 * conflates "worker did the work off the critical path" with
	 * "dispatch had to do it on the critical path" -- the exact
	 * distinction we are trying to measure. job->prepped here means the
	 * worker finished first and dispatch skips apply_relocs, i.e. the
	 * reloc cost was hidden behind the previous job's ~674 us of tile
	 * execution.
	 */
	if (job->prepped)
		atomic_inc(&mdev->prep_claimed);
	else
		atomic_inc(&mdev->prep_inline);

	if (mla_inject_prep_fail > 0) {
		/* Fault injection for the PRE-GO error path. mla_inject_fault
		 * cannot reach it: that one corrupts a descriptor after prep
		 * and lets the job reach GO, producing an FTL/watchdog fault
		 * instead. This arm returns before GO and before
		 * mla_job_arm_timeout() — a discarded error here must not
		 * leave the job parked in in_flight with nothing able to
		 * complete it.
		 *
		 * Injected at this call site rather than inside
		 * mla_hw_prep_job() so the SITE is selectable: which of the two
		 * callers fails is otherwise timing-dependent, and the two have
		 * different failure handling. Positive values arm here;
		 * negative values arm the async prep worker
		 * (mla_hw_prep_work_fn). Skipping the real prep also matches
		 * the shape of a genuine early failure: job->prepped stays
		 * false and no relocs are applied.
		 */
		dev_warn(mdev->dev,
			 "FAULT_INJECT: forcing pre-GO prep failure for job %u (%d remaining)\n",
			 job->id, mla_inject_prep_fail);
		mla_inject_prep_fail--;
		ret = -EINVAL;
	} else {
		ret = mla_hw_prep_job(job, bidx, false);
	}
	/* Between mla_sched_pick_reserve_locked() reserving this job and here, a
	 * cancel or device reset can finalize it: mla_sched_cancel_ctx_jobs()
	 * holds sched_lock only to READ in_flight, and dispatch runs unlocked
	 * (mla_sched_enqueue drops the lock before dispatching, deliberately,
	 * because programming may udelay). Without this test we publish ACTIVE
	 * and write GO for a job that no longer exists, into a block a reset is
	 * about to take down, and strand the bank with no owner to reclaim it.
	 *
	 * Reporting it as an error is safe even though the job is already
	 * finalized: mla_sched_finish_job() returns early on job->finalized,
	 * claimed under sched_lock by whichever path retired the job first, so
	 * the caller's finalize is a no-op. The block below then releases the bank
	 * this dispatch claimed.
	 */
	if (!ret && READ_ONCE(job->state) == MLA_JOB_DONE) {
		atomic_inc(&mdev->dispatch_cancelled_before_go);
		ret = -ECANCELED;
	}
	if (ret) {
		/* Dispatch claimed this bank itself and is failing before GO, so
		 * dispatch releases it: finish_job reclaims only ACTIVE and
		 * READY because a PREPARING bank belongs to a prep worker, and
		 * there is no worker here. Leaving it would leak the bank for
		 * the lifetime of the model.
		 */
		if (inline_claim &&
		    atomic_cmpxchg(&bank->state, MLA_BANK_PREPARING,
				   MLA_BANK_FREE) == MLA_BANK_PREPARING)
			job->bank_idx = MLA_BANK_UNASSIGNED;
		return ret;
	}
	/* Inline claims only: a bank claimed through READY is already ACTIVE.
	 * cmpxchg rather than atomic_set so ACTIVE can never be published over
	 * a bank owned by another job's worker.
	 */
	if (inline_claim &&
	    atomic_cmpxchg(&bank->state, MLA_BANK_PREPARING,
			   MLA_BANK_ACTIVE) != MLA_BANK_PREPARING) {
		atomic_inc(&mdev->dispatch_inline_claim_lost);
		/* The observed state names the writer: FREE means a stale
		 * release of a bank this dispatch owned, ACTIVE means a second
		 * dispatcher reached it.
		 */
		WARN_ONCE(1, "mla: inline claim lost bank %u (state=%d)\n",
			  bidx, atomic_read(&bank->state));
	}
	/* Timestamps stamped here (not inside prep_job) so a late-running
	 * worker doesn't corrupt them after inline prep completed.
	 */
	job->ts_after_relocs = ktime_get();
	job->ts_after_sync   = job->ts_after_relocs;

	/* Split MMIO reprogram: DFM registers hold per-bank descriptor
	 * fetch addresses (banks[bank_idx].blob->dma_addr), while tile
	 * MIM/QG0 registers hold model-static instruction section
	 * addresses. Bank change invalidates DFM only; model change
	 * invalidates both.
	 *
	 *   last_model != m                       -> reprogram DFM + tiles
	 *   last_model == m, last_bank != bank    -> reprogram DFM only
	 *   last_model == m, last_bank == bank    -> skip everything
	 *
	 * apply_relocs above patched per-job IFM/OFM phys into this
	 * bank's dma-descriptor blob; DFM/tile registers themselves are
	 * unchanged by that.  mla_hw_recover() and the burst-change
	 * reset both clear last_programmed_{model,bank_idx} so the
	 * next dispatch reprograms everything.
	 */
	if (mdev->last_programmed_model != m || mla_reprog_always) {
		mla_hw_init_dfm(mdev, m, job->bank_idx);
		mla_hw_load_tiles(mdev, m);
		mdev->last_programmed_model    = m;
		mdev->last_programmed_bank_idx = job->bank_idx;
	} else if (mdev->last_programmed_bank_idx != job->bank_idx) {
		mla_hw_init_dfm(mdev, m, job->bank_idx);
		mdev->last_programmed_bank_idx = job->bank_idx;
	}

	/* Fault injection: corrupt the first descriptor qword so the DMA
	 * engine fetches a bogus address, triggering FTL/quadgroup-timeout.
	 * Restores the original value AFTER the fault fires (via the reloc
	 * re-patch on the next prep_job call for this model).
	 */
	if (mla_inject_fault) {
		struct mla_buffer *desc_buf;

		desc_buf = mla_model_bank_blob(m, job->bank_idx);
		if (desc_buf && desc_buf->cpu_vaddr) {
			u64 *p = (u64 *)desc_buf->cpu_vaddr;

			dev_warn(mdev->dev,
				 "FAULT_INJECT: corrupting desc[0] qword (was 0x%llx) for job %u (%u remaining)\n",
				 *p, job->id, mla_inject_fault);
			*p = 0xDEAD0000DEAD0000ULL;
			/* Backing-aware flush: dma_sync_single would fault
			 * on a carveout-backed blob (no linear-map alias).
			 */
			mla_pool_sync_range_for_device(mdev, desc_buf, 0, 64);
		}
		mla_inject_fault--;
	}

	job->ts_go = ktime_get();
	if (mla_skip_hw) {
		/* Debug short-circuit: everything up to here ran (reloc
		 * patching, dma sync, DFM/tile/QG0 register programming,
		 * v2_trace prints), but skip GO and synthesize a clean
		 * completion. Lets us probe MODEL_DEFINE/JOB_SUBMIT
		 * bookkeeping (refcounts, reloc lists, dedup-shared
		 * sections) without involving live silicon.
		 */
		dev_info_ratelimited(mdev->dev,
			"v2_trace: skip_hw=1 — bypassing GO for job %u\n",
			job->id);
		mla_sched_complete(mdev, job, 0);
		return 0;
	}
	/* Arm the watchdog BEFORE the GO write, never after.
	 *
	 * The completion IRQ can only fire once the hardware is started, so
	 * arming first guarantees that whichever path finalizes this job finds a
	 * pending timer and disarms it. Arming afterwards left a window in which
	 * completion finalized the job first — its disarm found nothing pending
	 * and no-opped — and the arm then landed on a finished job that nothing
	 * would ever disarm. With CONFIG_PREEMPT=y this thread is in a sleepable
	 * context here, so that window is bounded by scheduling latency rather
	 * than by the instruction count.
	 *
	 * No error returns between here and the GO write, so this cannot leave a
	 * job armed but never started; the pre-GO failure paths all return above
	 * and reach mla_sched_finish_job, whose disarm is unconditional.
	 */
	mla_job_arm_timeout(job);

	/* Test hook: skip ONLY the GO write. The job is fully programmed and its
	 * watchdog armed, but no hardware work is ever started, so no PROGDONE
	 * can arrive and nothing will complete it — the watchdog expires for
	 * real. Distinct from mla_skip_hw, which also skips GO but then
	 * synthesizes a clean completion, hiding the timeout path entirely.
	 *
	 * Countdown, like mla_inject_fault. Each injection costs the caller a
	 * -ETIMEDOUT, a fatalized context and one recovery-budget slot.
	 */
	if (unlikely(mla_inject_no_complete)) {
		atomic_inc(&mdev->inject_no_completes);
		dev_warn(mdev->dev,
			 "FAULT_INJECT: withholding GO for job %u (%u remaining) — nothing will complete it, the watchdog must fire\n",
			 job->id, mla_inject_no_complete);
		mla_inject_no_complete--;
	} else {
		iowrite32(job->cycle_count,
			  mdev->regs + MLA_REG_QUADGRP0_SETUP_REG1);
		/* The block is now executing, so DMA_CFG will no longer latch
		 * a new burst value without another reset. Deliberately inside
		 * the else: when the injection withholds GO nothing runs, so
		 * the block is still freshly reset and the flag must stand.
		 */
		WRITE_ONCE(mdev->hw_fresh_reset, false);
	}

	return 0;
}
