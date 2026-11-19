/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 SiMa.ai */
#ifndef _MLA_DRV_H_
#define _MLA_DRV_H_

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/plist.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>

#include <linux/dma-heap.h>
#include <linux/notifier.h>

#include <linux/simaai/mla.h>

#define MLA_DRV_NAME                    "mla"
/*
 * 4: MLA_OP_JOB_ASYNC publishes resp.job_id at submit + MLA_CAP_JOB_CANCEL.
 *    Additive — MLA_ABI_VERSION stays 2.
 * 5: MLA_OP_DEVICE_RESET (op 18) + MLA_CAP_DEVICE_RESET. Additive — a new
 *    op code at MLA_OP_MAX, so MLA_ABI_VERSION stays 2.
 * 6: MLA_OP_PERFCTR_CONFIG (op 19) + MLA_CAP_PERFCTR_CONFIG. New op code,
 *    so MLA_ABI_VERSION stays 2 — but note this one CHANGES THE MEANING of
 *    an existing field rather than only adding: mla_job_submit_resp.
 *    perf_ctr_us[] is caller-selected via slot config and all-zero until
 *    configured, so a v5-era caller that reads those four slots
 *    positionally gets zeros, not wrong numbers.
 * 7: LoRA reloc ports — MLA_OP_MODEL_UPDATE_RELOC (op 20) +
 *    MLA_CAP_RELOC_UPDATE, MODEL_DEFINE_V3 accepts MLA_RELOC_ZONE_RELOC
 *    entries, and mla_model_define_v3_req grew an optional tail
 *    (reloc_defaults_user_ptr) behind a req_len >= offsetofend(..., name)
 *    check. Additive in both directions: a v6-era caller sends the old
 *    size with num_reloc_ports zero, which is exactly a model with no
 *    LoRA ports. MLA_ABI_VERSION stays 2.
 * 8: Optional cold prepared-bank count and MLA_CAP_PREPARED_BANKS.
 *    Existing callers retain lazy growth. MLA_ABI_VERSION stays 2.
 */
#define MLA_DRV_VERSION                 8

/* Hardware-constant values lifted from mla_regs.h (Modalix). */
#define MLA_TILE_ROWS                   10
#define MLA_TILE_COLS                   10
#define MLA_NUM_DMA_UNITS               4

/* Implementation-default policy knobs, reported via MLA_OP_INFO. */
#define MLA_MAX_CONTEXTS                64
#define MLA_MAX_MODELS_PER_CTX          16
/* MLA_MAX_INFLIGHT_PER_CTX (the per-context async-job cap, enforced by
 * MLA_IOCTL_SUBMIT_ASYNC and advertised via MLA_OP_INFO) lives in the uapi
 * header so userspace sizes its reap buffer to the same value.
 */
/* Cap on mla_job_submit_req.num_ifm and num_ofm. Both sync and async
 * paths heap-allocate their tensor-descriptor arrays, so there is no
 * stack-frame concern at this limit.
 */
#define MLA_MAX_IOTENSORS               256
/*
 * Flattening multiplier for the two-dimensional context priority:
 * key = group_prio * MLA_PRIO_GROUP_SHIFT + intra_group_prio, lower
 * wins (task_nice() convention). Group occupies the high bits so it
 * strictly outranks intra-group — 1024 is far above the [-20,19] range
 * CTX_CONFIGURE accepts, so an intra spread can never bridge one group
 * step.
 *
 * Lives here rather than in mla_sched.c so the debugfs dumper prints
 * the same key the scheduler orders on instead of duplicating the
 * literal. mla_sched_prio_key() is the only place it should be applied
 * on the hot path.
 */
#define MLA_PRIO_GROUP_SHIFT            1024
#define MLA_DEFAULT_ALIGN               (64 * 1024)   /* 64 KiB, conservative */
#define MLA_DEFAULT_MAX_BUFFER          (1ULL << 33)  /* 8 GiB */

/* IRQs: ftl (fatal), non_ftl (completion), mailbox. */
enum mla_irq {
	MLA_IRQ_FTL,
	MLA_IRQ_NON_FTL,
	MLA_IRQ_MAILBOX,
	MLA_IRQ_COUNT
};

/* Single device instance: one MLA per SoC. */
struct mla_job;
struct mla_section_dedup_entry;

/* Cache-line-aligned run of dirtied bytes inside a descriptor blob.
 * Precomputed at MODEL_DEFINE; consumed by mla_hw_apply_relocs_v2 to
 * avoid a full-buffer dma_sync on every JOB_SUBMIT.
 */
struct mla_flush_region {
	size_t  offset;    /* byte offset within blob, 64-B aligned */
	size_t  len;       /* byte length, multiple of 64 B */
};

/* Slice 4.7.2 — content-hash dedup table.
 *
 * Bucket count sized for hundreds of unique sections across many models
 * (each model contributes ~3-5 dedupable sections; 100 models on a busy
 * device → ~300-500 entries). 512 buckets keeps load factor <1 even at
 * the high end. Lookup walks one bucket comparing full sha256 — small
 * loops, no hash collisions matter for correctness.
 */
#define MLA_SECTION_DEDUP_BUCKETS  512

struct mla_section_dedup {
	struct hlist_head  bucket[MLA_SECTION_DEDUP_BUCKETS];
	spinlock_t         lock;
};

struct mla_device {
	struct device      *dev;
	struct miscdevice   miscdev;

	void __iomem       *regs;          /* mapped from "reg" property */
	resource_size_t     regs_size;

	int                 irq[MLA_IRQ_COUNT];

	struct mutex        ctx_list_lock;
	struct list_head    contexts;      /* all open mla_context */

	/* Two different populations; conflating them silently breaks any
	 * permission gate that means to ask "is someone else using the
	 * device?".
	 *
	 *   num_contexts  : contexts ALIVE. Incremented in mla_context_alloc,
	 *                   decremented in mla_context_destroy — i.e. on the
	 *                   LAST reference, not at fd close. A model, job or
	 *                   buffer routinely outlives the fd (see
	 *                   docs/CTX_LIFETIME_DESIGN.md), and for an
	 *                   uncommitted BUF_ALLOC buffer fput is deferred to
	 *                   the delayed_fput workqueue, so this can stay
	 *                   elevated for a while after close(). It is also
	 *                   what MLA_MAX_CONTEXTS caps.
	 *   contexts_open : contexts whose fd is still open, i.e. exactly the
	 *                   membership of @contexts above, since
	 *                   mla_context_shutdown does the list_del. This is
	 *                   the number any "is someone else using the device?"
	 *                   test wants.
	 *
	 * num_contexts - contexts_open = contexts draining after close. A
	 * non-zero difference is normal and transient; one that never returns
	 * to zero is a leaked context reference.
	 */
	atomic_t            num_contexts;
	atomic_t            contexts_open;

	u32                 soc_family;    /* MLA_SOC_* */

	/* Allocation policy may differ from the original DMS heap. Keep DMS
	 * identity to recognize delegated legacy no-map buffers individually.
	 */
	struct dma_heap    *heap;
	struct dma_heap    *dms_heap;
	bool                dms_is_carveout;
	bool                reserved_mem_attached;

	/* Annotates the OOM report with MLA's pool footprint. Name and size are
	 * cached from DT because cma_get_name()/cma_get_size() carry no
	 * EXPORT_SYMBOL and cannot link in a module.
	 */
	struct notifier_block oom_nb;
	bool                oom_nb_registered;
	const char         *pool_name;
	u64                 pool_size;

	/* Pool bytes clients have imported. Distinct from
	 * outstanding_alloc_bytes, which counts only driver-internal
	 * allocations: clients allocate from the heap in userspace.
	 */
	atomic64_t          imported_heap_bytes;

	/* ---------- Slice 4: scheduler + in-flight job tracking ----------
	 * Single in-flight job at a time. The scheduler lock protects the
	 * run queue and the `in_flight` slot.
	 */
	spinlock_t          sched_lock;
	struct plist_head   sched_queue;
	struct mla_job     *in_flight;
	atomic64_t          fence_seqno;     /* monotonic dma-fence seqno */
	u64                 fence_context;   /* dma_fence_context_alloc() */

	/* ---------- Slice 4.5: clock/reset + DMA burst-size tracking ----
	 * Two paths coexist:
	 *
	 * Native OF probe: clk framework consumers from DT. mlaclk is the
	 *   gate (BIT(0)|BIT(1) on PRC reg +0x594, covers both ck_en and
	 *   xxx_reset_n). mlapll is the rate-controllable parent. Both
	 *   resolve to provider simaai,clkc-prc. Toggling mlaclk
	 *   disable/enable is the reset sequence.
	 *
	 * Takeover (legacy DTB without clkc consumer): falls back to a
	 *   raw ioremap of the PRC region (mla_ck_rst at +0x594, PLL ctrl
	 *   at +0x14) for the same operations.
	 */
	struct clk         *mlaclk;        /* native path; NULL on takeover */
	struct clk         *mlapll;        /* native path; NULL on takeover */
	struct devfreq     *df;            /* native path; NULL on takeover */
	void __iomem       *prc_regs;      /* takeover path; NULL on native */

	/* MLA core-clock frequency in MHz. On the native path, derived
	 * from clk_get_rate(mlapll); on takeover, from the PLL register
	 * via mla_hw_read_pll_freq_mhz(). The IRQ handler uses this to
	 * convert the MLA_ACTIVE_DUR_CNT cycle counter to microseconds;
	 * 0 falls back to a 1000 MHz assumption (Modalix's actual
	 * silicon frequency). Reported via MLA_OP_INFO.
	 */
	u32                 mla_mhz;

	/* Cached DMA burst-size state. Set by mla_hw_set_dma_burst_size;
	 * compared against per-model desired values to decide whether
	 * the per-job burst-change-via-reset dance is needed. -1 means
	 * "unknown / never set".
	 */
	s8                  cur_burst_rd;
	s8                  cur_burst_wr;

	/* True while the MLA block has been reset and has not executed
	 * anything since. Set by mla_hw_reset() on success, cleared by the
	 * GO write in mla_hw_program_and_start(). The MLA latches DMA_CFG
	 * only out of reset, so mla_sched_apply_burst() uses this to tell
	 * "the burst write needs a reset first" from "a reset just happened,
	 * write it directly" — the latter must NOT reset again, because that
	 * touches MLA MMIO from the dispatch path and can race the recovery
	 * worker's reset into a synchronous external abort.
	 *
	 * Accessed with READ_ONCE/WRITE_ONCE, same discipline as device_fatal
	 * and for the same reason: the writers are mla_hw_reset() (recovery
	 * worker, probe, MLA_OP_DEVICE_RESET) and the GO write on the dispatch
	 * path, while the reader mla_sched_apply_burst() runs OUTSIDE
	 * sched_lock. There is no lock to take here — a stale read is benign in
	 * one direction (a redundant reset) and the point is only to stop the
	 * compiler caching or tearing the access.
	 */
	bool                hw_fresh_reset;

	/* True while mla_hw_enable_irqs() has armed the MLA's non-fatal IRQ
	 * masks and nothing in THIS driver has quiesced them since. Writers
	 * are mla_hw_enable_irqs() (probe, recovery, burst dance) and
	 * mla_hw_quiesce_irqs() (the fatal IRQ handler and mla_hw_recover()).
	 *
	 * Exists so mla_hw_repair_irq_masks() can tell "an agent outside this
	 * driver cleared our masks" from "we cleared them deliberately and
	 * mla_hw_recover() will restore them". The register value alone cannot
	 * distinguish the two: mla-rt's MIP backend clears quadgrp_prog_done
	 * but leaves quadgrp_halt_trig at 0xf, so TLC_NON_FTL_INTR_EN reads
	 * 0x1e00 — non-zero — while progdone delivery is dead.
	 *
	 * Accessed with READ_ONCE/WRITE_ONCE, same discipline as
	 * hw_fresh_reset above and for the same reason: there is no lock
	 * covering both the MMIO and this flag, and a stale read is benign in
	 * both directions (a redundant enable, or a skipped repair that the
	 * next open retries).
	 */
	bool                irqs_armed;

	/* Cached "last model whose DFM/tile/QG0 setup is currently
	 * programmed into the registers". When the next job uses the
	 * same model AND the same bank, we skip the ~30us of MMIO writes
	 * for DFM, tile, and QG0 — only apply_relocs (which patches
	 * per-job IFM/OFM phys addrs into the dma descriptors) and the
	 * GO write are required. Bank comparison matters because DFM's
	 * fetch address is banks[idx].blob->dma_addr, which differs
	 * across banks. Cleared on burst-change reset, hardware recovery,
	 * and model unload. NULL means "registers are stale, reprogram".
	 * last_programmed_bank_idx == MLA_BANK_UNASSIGNED forces reprogram.
	 */
	const struct mla_model *last_programmed_model;
	u8                      last_programmed_bank_idx;

	/* Slice 4.7.2: per-device content-hash dedup table for
	 * MLA_OP_MODEL_LOAD_SECTIONS. Initialized at probe, torn down at
	 * remove. Shared across all contexts on this device.
	 */
	struct mla_section_dedup  sec_ddp;

	/* Sum of currently-live pool-owned buffer sizes (bytes). Bumped in
	 * mla_pool_alloc, decremented in mla_pool_free. Exposed via
	 * MLA_OP_INFO for leak detection in tests.
	 */
	atomic64_t          outstanding_alloc_bytes;

	/* ---------- Slice 5: fault containment + reset --------------- */

	/* Workqueue item that runs mla_hw_recover() outside any
	 * IRQ/softirq context. Triggered by mla_sched_complete() on a
	 * non-zero result (timeout / FTL fault / cancel-via-fault).
	 */
	struct work_struct  recovery_work;

	/* Set to 1 by mla_sched_complete to coalesce duplicate triggers
	 * (multiple in-flight failures could schedule recovery many
	 * times before the worker runs once). Cleared by the worker.
	 */
	atomic_t            recovery_pending;

	/* Once true, all submissions return -EHOSTDOWN. Set when the
	 * recovery work fails or when the recovery rate-limit budget
	 * is exceeded. Cleared only by module reload.
	 */
	bool                device_fatal;

	/* Sliding-window reset budget. Protects against a misbehaving
	 * tenant triggering endless resets.
	 */
	spinlock_t          recovery_lock;
	u32                 recovery_count;
	unsigned long       recovery_window_start; /* jiffies */

	/* v2.5 Phase 2: async prep-ahead. prep_wq runs mla_hw_prep_job on
	 * an eligible head-of-plist job while HW is executing the current
	 * in-flight job. Bounded to one outstanding prep via prep_in_flight
	 * (cmpxchg 0→1 to queue; worker resets 0 on exit). max_active=1 is
	 * belt-and-braces — only one prep runs at a time regardless.
	 * Allocated in mla_sched_init(), destroyed in mla_remove().
	 */
	struct workqueue_struct *prep_wq;
	atomic_t                 prep_in_flight;

	/* Deferred buffer teardown. A job binding's reference can be the LAST
	 * one, and it is released from mla_fence_release, which can run in
	 * softirq (mla_sched_complete via the IRQ handler, mla_job_timeout_fn).
	 * mla_buffer_drop's teardown may sleep — dma_buf_put can call the
	 * exporter's release op and dma_buf_detach takes a mutex — so
	 * mla_buffer_drop_async() hands only the last-ref teardown here.
	 *
	 * A dedicated queue rather than system_wq because a pending item holds
	 * a work function pointer into this module: destroy_workqueue() in
	 * mla_sched_exit() flushes it before the module can go. Deliberately
	 * NOT prep_wq, which is the hot prep-ahead path and gets
	 * flush_workqueue'd during recovery.
	 *
	 * Rarely used: the common case is refcount_dec_and_test returning
	 * false, which never touches the queue. See
	 * docs/BUFFER_LIFETIME_DESIGN.md §6.
	 */
	struct workqueue_struct *buf_release_wq;

	/* Prep-ahead effectiveness, read via /sys/kernel/debug/mla/dump:
	 *
	 *   prep_claimed : dispatch found the async worker had already run
	 *                  apply_relocs + flush, so it skipped them
	 *   prep_inline  : dispatch had to do them on the critical path
	 *
	 * claimed/(claimed+inline) is the only number that says whether
	 * the bank pool + prep-ahead machinery is actually doing anything
	 * for a given workload.
	 *
	 * Do NOT instrument this with printks instead: serial-console
	 * logging is slow enough to drain the run queue between
	 * submissions, destroying the queue occupancy that prep-ahead
	 * depends on and making it look like prep-ahead never engages.
	 */
	atomic_t                 prep_claimed;
	atomic_t                 prep_inline;
	/* A claim skipped a bank that was FREE but still had writers. Anti-vacuity
	 * counter for the drain scheme: at zero, no bank was released while its
	 * worker was still inside it, so a green run proves nothing.
	 *
	 * Both increment sites test FREE as well as writers, because a bank
	 * PREPARING under its own worker also fails the claim with writers
	 * non-zero and counting ordinary occupancy would swamp the signal. A
	 * large value on a workload with no steals therefore means a leaked ref.
	 */
	atomic_t                 dispatch_bank_draining;
	/* Dispatch took a bank from its live prep worker, PREPARING -> ACTIVE.
	 * Legal, and the reason writers exists.
	 */
	atomic_t                 dispatch_stole_preparing;
	/* A prep worker found its bank already taken when it went to publish.
	 * The mirror of dispatch_stole_preparing, counted on the losing side.
	 */
	atomic_t                 prep_stolen;
	/* A prep worker left its write loop early because dispatch had taken
	 * the bank. Pure optimisation, so zero is correct-but-slower.
	 */
	atomic_t                 prep_aborted;
	/* Dispatch found its assigned bank in neither READY nor PREPARING, so
	 * something released it, and re-claimed a bank instead. Benign; not a
	 * failure path. The three below split it by explanation, since the total
	 * alone cannot say whether the cause is understood:
	 *
	 *   _fatal       : device_fatal or ctx->fatal was set, which explains it
	 *   _unexplained : neither was, so the release has no known route
	 *   _was_active  : the bank was ACTIVE, not FREE, meaning a second
	 *                  writer reached it. Worse than a stale release.
	 */
	atomic_t                 dispatch_bank_not_ready;
	atomic_t                 dispatch_bnr_fatal;
	atomic_t                 dispatch_bnr_unexplained;
	atomic_t                 dispatch_bnr_was_active;
	/* Banks mla_sched_device_reset() found off FREE and reclaimed. Non-zero
	 * proves a job can be cancelled mid-dispatch and leave its bank behind.
	 */
	atomic_t                 reset_banks_reclaimed;
	/* Of those, banks still carrying a writer. Reachable by design, because
	 * mla_sched_try_prep_ahead() can make a worker visible after
	 * mla_hw_recover() flushed prep_wq; see the reclaim loop.
	 */
	atomic_t                 reset_bank_had_writer;
	/* Dispatch found its job already finalized before writing GO, so it
	 * refused instead of running a cancelled job and stranding its bank.
	 * Non-zero means a cancel or reset really does race dispatch.
	 */
	atomic_t                 dispatch_cancelled_before_go;
	/* A model had no free bank at dispatch. Reachable when bank growth hit
	 * ENOMEM and a higher-priority job jumped one already holding a bank.
	 */
	atomic_t                 dispatch_no_free_bank;
	/* Dispatch's own FREE -> PREPARING claim did not survive to ACTIVE,
	 * so a bank it owned was republished under it.
	 */
	atomic_t                 dispatch_inline_claim_lost;
	/* Async prep worker declined to publish a bank because
	 * mla_hw_prep_job() failed. Non-zero proves the worker's error path
	 * actually ran, since a green run otherwise looks identical to one
	 * that never reached the path.
	 */
	atomic_t                 prep_failures;
	/* BUF_IMPORTs of a FOREIGN dma-buf whose exporter supplies neither
	 * begin_cpu_access nor end_cpu_access. For those the caller's
	 * DMA_BUF_IOCTL_SYNC is a silent no-op (dma_buf_end_cpu_access()
	 * returns 0 when the op is NULL), and this driver does no per-job
	 * IFM/OFM maintenance, so nothing cleans a CPU write to that buffer.
	 *
	 * Counted, not refused: see the dev_warn_once in mla_op_buf_import()
	 * for why rejecting is wrong. Paired with that warning because
	 * warn_once goes quiet after the first hit — this is what answers
	 * "has this ever actually happened on a real board?" months later.
	 *
	 * A non-zero value is not by itself a bug: a caller that never
	 * CPU-touches the buffer (pure device-to-device zero-copy, the usual
	 * reason to import a camera frame) is perfectly safe. It means
	 * "an unverifiable caller obligation is in play here".
	 */
	atomic64_t               foreign_imports_no_cpu_ops;
	/* mla_sched_pick_reserve_locked() refused to reserve the in_flight
	 * slot because the device was fatal. Counts refusals from BOTH
	 * dispatch paths, so a non-zero value proves the guard was exercised
	 * — it does NOT prove the specific reset-vs-enqueue interleaving it
	 * exists to stop. The real evidence for that is the external-abort
	 * rate across many resets of a BUSY device. Treat this as "the gate
	 * is reachable", not as "the race happened".
	 */
	atomic_t                 dispatch_refused_fatal;
	atomic_t                 inject_no_completes;

	/* Burst-dance accounting. burst_reprograms counts every DMA_CFG
	 * reprogram; burst_resets counts the subset that still needed a PRC
	 * reset first. These prove the fresh-reset skip was actually
	 * exercised: a run where burst_reprograms is 0 never reached the
	 * path at all, and post-recovery reprograms must show up as
	 * reprograms WITHOUT a matching reset.
	 */
	atomic_t                 burst_reprograms;
	atomic_t                 burst_resets;

	/* Times mla_open() found our non-fatal IRQ masks cleared by an agent
	 * outside this driver and re-armed them. Non-zero means something
	 * else drove the MLA between opens, such as mla-rt's MIP backend.
	 * Silent counter plus a dev_warn_once, and
	 * deliberately not an error: the repair is sufficient, and refusing
	 * the open would break a caller that did nothing wrong.
	 *
	 * Also proves the fix works in both directions — it must be non-zero
	 * after a MIP run and must stay zero across SWMLA-v2-recovery, whose
	 * quiesce windows must NOT be repaired.
	 */
	atomic_t                 irq_masks_repaired;

	/* LoRA reloc accounting. reloc_ports_bound counts every port binding
	 * installed, at define (defaults) and by MLA_OP_MODEL_UPDATE_RELOC;
	 * reloc_binds counts update calls that named at least one port;
	 * reloc_patch_passes counts descriptor-patch passes, one per bank per
	 * update plus one per define. Used in every LoRA test to confirm the
	 * adapter was actually applied.
	 */
	atomic_t                 reloc_binds;
	atomic_t                 reloc_ports_bound;
	atomic_t                 reloc_patch_passes;
	/* Updates that found outstanding != 0 and had to wait, split by
	 * outcome. Without these there is no way to tell whether the wait
	 * ever ran, whether it did any good, or whether a -EBUSY was a real
	 * refusal rather than a wait that gave up.
	 */
	atomic_t                 reloc_update_waits;
	atomic_t                 reloc_update_wait_timeouts;
	/* Jobs mla_sched_enqueue() refused because their model was mid-update.
	 * Proves the quiescence rule's refusal path actually ran: a test
	 * asserting "submit during an update is rejected" needs more than an
	 * -EBUSY reaching userspace — it needs to know THIS path produced it.
	 */
	atomic_t                 reloc_submits_refused;


	/* Debug invariant counters — every refcount_inc / final
	 * refcount_dec_and_test on a struct mla_buffer ticks these. At
	 * teardown, alive-buf-refcount-sum should equal (inc-dec). Used by
	 * the debugfs /sys/kernel/debug/mla/dump handler.
	 */
	atomic64_t          buf_inc_ops;
	atomic64_t          buf_dec_ops;

	/* The same invariant for jobs, which buf_inc_ops/buf_dec_ops cannot
	 * see. struct mla_job has no refcount_t — its embedded dma-fence IS
	 * its refcount — so these tick once per job created
	 * (mla_job_init_internal) and once per job destroyed
	 * (mla_fence_release, the fence destructor, which is the only point
	 * that runs on every free path including the err_* unwinds).
	 *
	 * (inc - dec) is therefore the number of jobs currently alive, and it
	 * must return to 0 once a workload has drained. Job retention is
	 * invisible to the buffer delta alone, so this counter exists to
	 * catch a leaked job registry entry that a buffer-only check would
	 * miss.
	 */
	atomic64_t          job_inc_ops;
	atomic64_t          job_dec_ops;

	/* Debugfs anchor for the leak-tracing dumper. NULL when
	 * CONFIG_DEBUG_FS is disabled or debugfs_create_dir failed.
	 */
	struct dentry      *debugfs_root;

	/* Caller-selected TLC perf counters, backing MLA_OP_PERFCTR_CONFIG.
	 *
	 * perfctr_count == 0 means "nothing configured": no slot has
	 * cntr_en set, the completion IRQ skips the register read, and
	 * perf_ctr_us[] goes back to userspace all zeros. That is the state
	 * at probe and after any recovery.
	 *
	 * perfctr_signals[i] is the hardware src_sel programmed into slot i,
	 * and slot i is what lands in perf_ctr_us[i] — the order the caller
	 * listed is the order it reads back.
	 *
	 * Written only under sched_lock, and only while the device is idle
	 * (the op refuses otherwise), which is what keeps a reconfigure from
	 * racing a job that is mid-measurement. Read from the completion IRQ
	 * under the same lock discipline. Deliberately plain integers and
	 * not a pointer to whoever configured them: a stale pointer that the
	 * slab handed back for a new object could compare equal to the freed
	 * one. Ownership for cleanup purposes lives on the context instead,
	 * as mla_context.owns_perfctr.
	 */
	u32                 perfctr_count;
	u8                  perfctr_signals[MLA_PERFCTR_MAX_SLOTS];

	/* Bumped under sched_lock on EVERY change to perfctr_count /
	 * perfctr_signals[], by all three mutators: MLA_OP_PERFCTR_CONFIG,
	 * mla_hw_perfctr_disable_all() and mla_hw_recover()'s software clear.
	 *
	 * This is what lets a holder tell "my selection is still the live one"
	 * from "something replaced or wiped it", without the stale-pointer
	 * problem the comment above describes: a generation number that the
	 * slab cannot hand back. Starts at 0, so 0 also reliably means "never
	 * configured" and a context that never called op 19 can never match.
	 *
	 * Two bugs need it. (1) mla_context_shutdown() decrements contexts_open
	 * (mla_drv.c:193) BEFORE the job/model/buffer drain that precedes
	 * mla_perfctr_release() (:219), so a new context can legitimately open,
	 * pass op 19's sole-context gate and install its own selection while the
	 * dying one is still draining — whose release then wiped the newcomer's
	 * counters. (2) mla_hw_recover() clears the selection but cannot clear
	 * some other context's owns_perfctr, leaving it claiming an ownership it
	 * no longer has.
	 */
	u32                 perfctr_gen;

	/* Legacy /dev/simaai-mlamem (simaai_mlamem.c); NULL if not published. */
	struct simaai_mlamem *mlamem;
};

/* Recovery rate limit: at most this many resets in this window.
 * Exceeding it escalates to device_fatal.
 */
#define MLA_RECOVERY_BUDGET_COUNT   3u
#define MLA_RECOVERY_BUDGET_WINDOW  (60 * HZ)

/* Default grace period MLA_OP_MODEL_UPDATE_RELOC waits for a model to go
 * quiescent before answering -EBUSY. Overridable via
 * /sys/module/mla/parameters/mla_reloc_quiesce_grace_ms.
 *
 * Sized for the SKEW it exists to absorb, NOT for a caller's workload: the
 * window between dma_fence_signal() and the atomic_dec of model->outstanding in
 * mla_sched_finish_job() is the tail of that function — single-digit
 * microseconds, widened only by preemption or an IRQ landing in between. 2 ms
 * is ~20x a typical preemption latency and still two orders below the drain
 * time of any real job queue.
 *
 * A grace period sized to absorb a genuine job drain, rather than just the
 * fence-signal/decrement skew, stops being a grace period and becomes a
 * silent wait: an update that should be refused with -EBUSY succeeds
 * instead, and SWMLA-v2-lora-reloc's update-side -EBUSY case is exactly
 * what that would hide.
 */
#define MLA_RELOC_QUIESCE_GRACE_MS  2u

/*
 * One physically-contiguous DMA buffer. Owned by the context that
 * allocated or imported it.
 *
 * Three flavors:
 *   - allocated by us from the carveout heap (heap_backed): we own a
 *     dmabuf ref + attachment/sgt (->dma_addr from the single sg
 *     entry) + vmap (->cpu_vaddr); ->imported = false.
 *   - allocated by us from CMA for CPU_RMW sections (cma_backed):
 *     ->cpu_vaddr/->dma_addr from dma_alloc_noncoherent; the
 *     mla_buffer lives in the exported dma-buf's priv.
 *   - imported from another exporter via MLA_OP_BUF_IMPORT: we hold an
 *     attachment + mapped sg_table; ->imported = true. The single sg
 *     entry's dma_address is recorded as ->dma_addr.
 */
struct mla_buffer {
	struct mla_context        *ctx;        /* owning context */
	u32                        id;         /* internal_buf_id (idr key) */
	u32                        flags;      /* MLA_HEAP_FLAG_* + internal */

	struct dma_buf            *dmabuf;     /* the dma-buf (NULL for pool-owned) */
	bool                       imported;
	bool                       phys_attached; /* slice 4.7: BUF_ATTACH_PHYS */
	bool                       pool_owned;    /* slice 4.7.1: section-load pool;
						   * kernel-side identity with no
						   * userspace fd. mla_buffer_drop
						   * routes to mla_pool_free.
						   */
	bool                       dedup_alias;   /* slice 4.7.2: shell mla_buffer that
						   * mirrors a dedup-shared primary's
						   * cpu_vaddr/dma_addr. drop frees only
						   * the shell — the primary is owned by
						   * the dedup entry.
						   */
	bool                       cma_backed;    /* krlc attack: backed by
						   * dma_alloc_noncoherent (WB linear-map
						   * alias, faster CPU RMW at the cost of
						   * explicit dma_sync_single_for_device
						   * on every DMA handoff). Set for
						   * CPU_RMW descriptor-blob sections
						   * (pool banks and SECTION_ALLOC).
						   */
	/* Owns the heap dma-buf plus attachment/map/vmap. Both reusable CMA
	 * and legacy carveout allocations use exporter CPU-access hooks.
	 */
	bool                       heap_backed;
	enum dma_data_direction    dma_dir;       /* Direction passed to
						   * dma_alloc_noncoherent; needed for
						   * dma_sync + dma_free. Defined only
						   * when cma_backed is true.
						   */
	/* Legacy no-map backing has no linear-map alias: only the
	 * exporter's CPU-access hooks may maintain its caches. Reusable CMA
	 * has normal pages and must not inherit this flag just by heap name.
	 */
	bool                       no_cpu_alias;

	/* Set when counted in mdev->imported_heap_bytes, so the decrement in
	 * mla_buffer_free() balances on whichever teardown path runs.
	 */
	bool                       counted_heap_bytes;

	/* For allocated buffers */
	void                      *cpu_vaddr;
	dma_addr_t                 dma_addr;
	size_t                     size;
	unsigned long              alignment;

	/* For imported buffers */
	struct dma_buf_attachment *attach;
	struct sg_table           *sgt;

	/* Slice 4.7 phys_attached only: the kernel virtual mapping of the
	 * fixed phys range (memremap'd). cpu_vaddr is set to this so the
	 * normal vmap/mmap paths work.
	 */
	void __iomem              *iomap;

	/* Slice 4.7.2: when non-NULL, this buffer participates in
	 * content-hash dedup. For a primary buffer (the first context
	 * to insert this hash), dedup points at the entry that HOLDS A
	 * COUNTED REFERENCE on it; the entry's last put clears this field
	 * and drops that reference via mla_buffer_drop.
	 * For an alias buffer (dedup_alias=true), dedup points at the
	 * same entry and the alias owns one reference on it; dropping the
	 * alias hands that back and frees only the shell, while the
	 * entry's refcount governs the underlying memory.
	 */
	struct mla_section_dedup_entry *dedup;

	/* SECTION_ALLOC/SECTION_COMMIT plumbing. Only meaningful for
	 * dma-buf-backed buffers produced by mla_op_section_alloc. When
	 * userspace called SECTION_ALLOC with MLA_SECTION_FLAG_DEDUP_OK
	 * and a non-zero sha256, we memoize the request here until
	 * SECTION_COMMIT publishes into sec_ddp. Zero-cleared on
	 * non-dedup buffers. dedup_requested==true implies pending_sha[]
	 * is meaningful.
	 *
	 * committed transitions false->true at SECTION_COMMIT and means
	 * "userspace has finished writing this section". It carries NO
	 * lifetime meaning: the buffer already survives userspace fd close
	 * from the moment SECTION_ALLOC registered it, because that is where
	 * the kernel's single counted dma-buf reference is taken. Validation
	 * is the only consumer (v3_resolve_section_refs rejects uncommitted
	 * ids). See docs/BUFFER_LIFETIME_DESIGN.md.
	 */
	bool                       dedup_requested;
	bool                       committed;
	u8                         pending_sha[32];

	refcount_t                 refcount;

	/* Owning device, cached to save a hop through ->ctx on hot and
	 * completion paths.
	 *
	 * Correctness does NOT depend on this field — a buffer holds a
	 * counted reference on its context (see mla_buffer_register), so
	 * buf->ctx->mdev is valid for the buffer's whole life even when the
	 * buffer outlives the /dev/mla fd. It is a convenience, and it also
	 * covers buffers that never registered in a context at all (pool and
	 * heap allocations).
	 *
	 * Safe to hold unconditionally: mla_device is devm_kzalloc'd at
	 * probe and lives as long as the driver is loaded, which is
	 * strictly longer than any buffer or context.
	 *
	 * NOTE new members go at the END of this struct — a mid-struct
	 * insertion shifts every later offset and a partially-deployed
	 * userspace binary then reads the wrong field.
	 */
	struct mla_device         *mdev;

	/* Deferred-teardown work item, used ONLY by mla_buffer_drop_async()
	 * when a job binding drops the last reference from a context that may
	 * not sleep. INIT_WORK happens there rather than at the eight
	 * allocation sites, which is safe because a buffer's last reference
	 * drops exactly once, so this is queued at most once per buffer.
	 *
	 * Costs 32 bytes: sizeof(struct mla_buffer) goes 152 -> 184, so the
	 * struct still lands in kmalloc-192 and the slab-triage note in
	 * mla_rt/skills/mla-kernel-slab-debug/SKILL.md stays correct. Measured,
	 * not assumed — check it again before adding another member.
	 */
	struct work_struct         release_work;
};

/* Internal flags (not in uapi) */
#define MLA_BUF_FLAG_IMPORTED   (1u << 16)
#define MLA_BUF_FLAG_VM_GUEST   (1u << 17)  /* phys_addr hidden in responses */

/*
 * One loaded model. Refcounted because host-kind contexts may share
 * a single backing image via content-hash dedup.
 *
 * Models are loaded through MLA_OP_MODEL_DEFINE_V3: user-space allocates
 * each section itself via SECTION_ALLOC / SECTION_COMMIT and passes
 * section_ids[] plus tile / DMA-unit / reloc arrays; the kernel patches
 * SECTION-tagged relocs at MODEL_DEFINE and IFM/OFM/RELOC-tagged
 * relocs at JOB_SUBMIT.
 */

#define MLA_MAX_BANKS         3
#define MLA_BANK_UNASSIGNED   0xff   /* sentinel for mla_job.bank_idx */

enum mla_bank_state {
	MLA_BANK_FREE      = 0,
	MLA_BANK_PREPARING = 1,
	MLA_BANK_READY     = 2,
	MLA_BANK_ACTIVE    = 3,
};

struct mla_desc_bank {
	/* NULL => bank[0] sentinel: use section_bufs[desc_section_index].
	 * Non-NULL => private kmalloc'd copy for banks[1..num_banks-1].
	 */
	struct mla_buffer *blob;
	atomic_t           state;  /* enum mla_bank_state */

	/* Async prep workers that may still be inside this bank's descriptor
	 * write loop. A bank is claimable only at zero: dispatch is allowed to
	 * steal a PREPARING bank, so the owning job can run, complete and
	 * release the bank while its worker is still writing. Reuse before the
	 * worker leaves is what corrupts the next owner's descriptors.
	 */
	atomic_t           writers;
};

struct mla_model {
	struct mla_context        *ctx;        /* owning context */
	u32                        id;         /* model_id (xarray key) */
	refcount_t                 refcount;

	/* Parsed geometry (echoed to userspace in model_define_resp). */
	u32                        num_active_tiles;
	u32                        num_dma_descriptors;
	u32                        quad_tile_bitmask[4];

	/* Submission contract. */
	u32                        batch_size;
	u32                        num_inputs;
	u32                        num_outputs;

	u32                        flags;       /* MLA_MODEL_* + internal */

	/* Semaphored models REQUIRE 256B WR burst or they deadlock.
	 * Detected by walking the typed DMA descriptor blob at
	 * MODEL_DEFINE time.
	 */
	bool                       using_semaphores;

	/* Structured relocation list. Copied from userspace at
	 * MODEL_DEFINE; walked + applied at JOB_SUBMIT. Each entry
	 * patches one descriptor's dram_addr field with the actual
	 * phys_addr of an IFM/OFM internal_buf_id at submit time.
	 *
	 * Static-reloc split: when static_reloc_count > 0, entries
	 * [0..static_reloc_count) were already spliced into the
	 * descriptor blob at MODEL_DEFINE time (section-static phys
	 * doesn't change between jobs). The per-job patcher only walks
	 * [static_reloc_count..reloc_count) — which carries IFM/OFM/
	 * RELOC tags whose phys can change per job. Zero means "no
	 * partition" — patcher walks every entry.
	 */
	struct mla_reloc_entry    *reloc_list;
	u32                        reloc_count;
	u32                        static_reloc_count;

	/* ---- LoRA / reloc-port bindings ---------------------------------
	 *
	 * The analogue of the per-job ifm_bufs/ifm_offsets pair below, but
	 * living on the model rather than the job because reloc state is
	 * STICKY: a binding installed by MLA_OP_MODEL_UPDATE_RELOC applies
	 * to every subsequent job until another update replaces it.
	 *
	 * num_reloc_ports comes from MODEL_DEFINE_V3 and bounds the zone_index
	 * of every MLA_RELOC_ZONE_RELOC entry. Both arrays are NULL when it is
	 * 0, which is every non-LoRA model.
	 *
	 * reloc_bufs[p] is NEVER NULL once define has succeeded — it starts at
	 * the port's ELF-supplied default section and is only ever replaced,
	 * so no descriptor is ever dispatched holding a placeholder. Each entry
	 * owns exactly one counted reference, whether it points at the default
	 * section or at caller memory; uniform ownership is deliberate, as the
	 * alternative (alias the default without a reference, since the model
	 * already holds one as a section) needs a per-port "is this still the
	 * default?" flag that every update and teardown path has to consult.
	 *
	 * reloc_offsets[p] is the binding's offset within its buffer, so one
	 * caller allocation can back several ports. The patched address is
	 * reloc_bufs[p]->dma_addr + reloc_offsets[p] + placeholder_offset,
	 * exactly as the IFM arm computes it.
	 */
	u32                        num_reloc_ports;
	struct mla_buffer        **reloc_bufs;      /* [num_reloc_ports] */
	u64                       *reloc_offsets;   /* [num_reloc_ports] */

	/* Which ports the model's reloc list actually NAMES, i.e. which ones
	 * mla_model_patch_reloc_ports() will rewrite a descriptor for.
	 *
	 * num_reloc_ports is the number of RELOC sections the ELF ships, not
	 * the number the descriptors reference — mla-rt derives it from its
	 * reloc_defaults_ and cross-checks it against the ELF's RELOC section
	 * count, and nothing there consults the reloc list. So a declared port
	 * that no descriptor names is a shape we have not proven impossible,
	 * and it matters: re-pointing such a port rewrites nothing, so the
	 * adapter silently does not apply. Recorded here at define so
	 * MLA_OP_MODEL_UPDATE_RELOC can refuse exactly that request instead of
	 * reporting a swap it did not perform.
	 *
	 * Fixed 32 bytes (MLA_MAX_IOTENSORS bits), so no allocation and no
	 * teardown. Meaningless when num_reloc_ports == 0.
	 */
	DECLARE_BITMAP(reloc_ports_refd, MLA_MAX_IOTENSORS);

	/* Set by MLA_OP_MODEL_UPDATE_RELOC across its patch, tested by the
	 * submit path under mdev->sched_lock (which enqueue already holds) so a
	 * job cannot be queued against a descriptor set that is mid-rewrite.
	 * Written under bank_grow_lock AND sched_lock; read under sched_lock.
	 */
	bool                       reloc_update_in_progress;

	/* Woken on the 0-transition of `outstanding`, so
	 * MLA_OP_MODEL_UPDATE_RELOC can wait out the skew between
	 * dma_fence_signal() and the atomic_dec that follows it in
	 * mla_sched_finish_job().
	 *
	 * Without it, a caller that waited on its completion_fence_fd — the
	 * protocol the uapi mandates — could still observe outstanding != 0 and
	 * be refused with a SPURIOUS -EBUSY, intermittently and only on the
	 * fence-fd path (sync JOB_SUBMIT and the async CQE both notify after
	 * the decrement, so neither reproduces it).
	 */
	wait_queue_head_t          reloc_quiesce_wq;

	/* Debug name (from MODEL_DEFINE req.name); NUL-padded. Empty if
	 * userspace didn't fill it. Used only in v2_trace prints so a debug
	 * session can tell which model each section[] / JOB_SUBMIT line
	 * belongs to. Capacity matches the directive name max so the same
	 * format specifier works for both.
	 */
	char                       name[MLA_SECTION_NAME_MAX];

	/* Per-section buffer references (parallel to the directive[]
	 * array supplied at MODEL_DEFINE). Each is a primary or alias
	 * mla_buffer registered in the owning context. Held with a
	 * refcount via mla_buffer_register; destroyed by stepping each
	 * through mla_buffer_drop at model destroy time.
	 */
	struct mla_buffer        **section_bufs;
	u32                        num_sections;

	/* Typed tile + DMA-unit descriptors. Allocated via kmalloc at
	 * MODEL_DEFINE; freed at mla_model_destroy. The kernel programs
	 * MLA registers from these arrays directly — no header parsing,
	 * no vmap'd image to read.
	 */
	struct mla_tile_desc      *tile_info_typed;
	struct mla_dma_unit_desc  *dma_info_typed;

	/* krlc range-scoped flush.
	 *
	 * Precomputed at MODEL_DEFINE from the dynamic-reloc range
	 * [static_reloc_count, reloc_count). Each entry is a 64-B-aligned
	 * cache-line run inside the descriptor blob; adjacent lines are
	 * coalesced into a single run. mla_hw_apply_relocs_v2 iterates
	 * this list to sync only the dirtied cache lines instead of the
	 * whole multi-MB blob (100s of µs on cache_token2047).
	 *
	 * Set to NULL / 0 when either (a) no dynamic relocs exist, or (b)
	 * mla_reloc_range_flush=0 (kill switch — fall back to full-buffer
	 * sync). The hot path picks the fallback in either case.
	 */
	struct mla_flush_region   *flush_regions;
	u32                        num_flush_regions;

	/*
	 * Descriptor-bank pool.
	 *
	 * The dma.descs blob (section_bufs[dma_info_typed[0].desc_section_index])
	 * is CPU-RMW'd by apply_relocs on every job to patch that job's
	 * IFM/OFM addresses. When multiple jobs of the SAME model queue up,
	 * a per-bank generation lets each job's prep run into its own bank
	 * copy — dispatch consumes a READY bank; PROGDONE releases it back
	 * to FREE. Target workload: JOB_ASYNC clients that submit N
	 * inferences of the same model with different IFM/OFM slots (e.g.
	 * multi-buffered video pipelines) — with banks, the scheduler can
	 * overlap each job's prep with its predecessor's HW execution.
	 *
	 * Bank[0].blob == NULL is a sentinel: the accessor
	 * mla_model_bank_blob() falls back to
	 * section_bufs[dma_info_typed[0].desc_section_index] — the primary
	 * section blob shared with static-reloc splice and OP_INFO. Every
	 * other bank owns its own kmalloc'd copy allocated by mla_model_grow_banks(),
	 * either during cold preparation or on demand for legacy models.
	 *
	 * outstanding is the enqueue-time counter (increment on
	 * mla_sched_enqueue, decrement on mla_sched_complete): when it
	 * exceeds num_banks, legacy models attempt to grow the pool. Prepared
	 * models keep fixed backing and use the existing bank backpressure.
	 */
	struct mla_desc_bank      *banks;       /* array of MLA_MAX_BANKS */
	u32                        num_banks;
	/* Immutable after DEFINE: no enqueue-time backing growth. */
	bool                       banks_prepared;
	atomic_t                   outstanding;
	struct mutex               bank_grow_lock; /* protects num_banks + banks[] grow */

	/* Deferred teardown for mla_model_put_async(). mla_model_destroy()
	 * blocks (mla_buffer_drop, kvfree, mla_context_put), but the last
	 * model reference is dropped from mla_fence_release(), which can run
	 * in softirq — so the destroy is handed to mdev->buf_release_wq.
	 * Queued at most once per model, since a last reference drops once.
	 */
	struct work_struct         release_work;
};

/**
 * mla_model_bank_blob() — descriptor-blob buffer for a specific bank
 * @m:       model
 * @bank_idx: bank index [0..num_banks-1]
 *
 * Returns the mla_buffer * carrying the descriptor blob CPU vaddr for
 * this bank. Bank[0] is aliased to the primary section blob
 * (section_bufs[dma_info_typed[0].desc_section_index]); every other
 * bank has its own private allocation via mla_model_grow_banks.
 *
 * Callers: apply_relocs, program_and_start (fault-inject path), and
 * any future prep worker. Consolidating the lookup here keeps the
 * "which blob is dma.descs" convention in exactly one place.
 */
static inline struct mla_buffer *
mla_model_bank_blob(const struct mla_model *m, u8 bank_idx)
{
	if (bank_idx >= m->num_banks || !m->banks[bank_idx].blob)
		return m->section_bufs[m->dma_info_typed[0].desc_section_index];
	return m->banks[bank_idx].blob;
}

/*
 * Claim banks[k] for a new owner, FREE -> PREPARING. The only claim
 * primitive; both call sites (the prep-ahead walker and dispatch's inline
 * scan) must go through it so the drain rule cannot diverge between them.
 *
 * Refuses a FREE bank that still has writers. Testing writers before the
 * cmpxchg is safe rather than racy: writers is only ever incremented by the
 * claimer of a FREE bank, so for a bank that is already FREE the count is
 * monotonically decreasing and an observed zero cannot go back up. Two
 * claimers racing are arbitrated by the cmpxchg, not by this test.
 *
 * Return: true if the caller now owns banks[k] as PREPARING.
 */
static inline bool mla_bank_try_claim(struct mla_model *m, u32 k)
{
	if (atomic_read(&m->banks[k].writers))
		return false;
	return atomic_cmpxchg(&m->banks[k].state, MLA_BANK_FREE,
			      MLA_BANK_PREPARING) == MLA_BANK_FREE;
}

#define MLA_MODEL_FLAG_DEDUPED   (1u << 16)

/*
 * One inference. Lives on the scheduler queue (sched_node) until the
 * hardware is free; while running it sits in mdev->in_flight. Owned by
 * its context's `jobs` xarray for lookup and cancel.
 *
 * The dma-fence is the externally visible completion signal. It signals
 * OK on hardware progdone, or with an error on timeout / cancel /
 * fault. The fence holds a reference on the job; the job is freed when
 * both the fence reference and the xarray reference are dropped.
 */
struct mla_job {
	struct mla_context  *ctx;
	struct mla_model    *model;
	u32                  id;
	u32                  cycle_count;
	u32                  timeout_ms;
	u32                  num_ifm;
	u32                  num_ofm;

	/* Per-tensor bindings; resolved into mla_buffers at submit time
	 * and pinned to the job for its lifetime.
	 */
	struct mla_buffer  **ifm_bufs;     /* num_ifm entries */
	struct mla_buffer  **ofm_bufs;     /* num_ofm entries */

	/* Per-tensor sub-buffer offsets (from mla_iotensor.offset). Added
	 * to bufs[i]->dma_addr during apply_relocs_v2 so userspace can
	 * present a sub-page-aligned phys to the model when the underlying
	 * BUF_ATTACH_PHYS region was rounded down to a page boundary.
	 */
	u64                 *ifm_offsets;  /* num_ifm entries, NULL if all zero */
	u64                 *ofm_offsets;  /* num_ofm entries, NULL if all zero */

	/* Hot-path completion state. Only the IRQ writes prog_done_count;
	 * the scheduler reads/clears it under sched_lock.
	 */
	u32                  prog_done_count;
	u32                  result;       /* 0 / -ETIMEDOUT / -EIO */
	u64                  latency_us;

	/* Fault snapshot — filled by FTL IRQ or timeout path. */
	u32                  fault_class;
	u32                  fault_sys_sts;
	u32                  fault_l2g_sts[4];

	/* dma-fence */
	struct dma_fence     fence;
	spinlock_t           fence_lock;
	bool                 fence_signaled;

	/* Scheduling — plist node, priority = mla_sched_prio_key(ctx) at
	 * insert time, encoding (group_prio, intra_group_prio). plist
	 * preserves FIFO among equal-priority nodes.
	 */
	struct plist_node    sched_node;
	enum {
		MLA_JOB_QUEUED,
		MLA_JOB_RUNNING,
		MLA_JOB_DONE,
	}                    state;
	/* Set under sched_lock by the one mla_sched_finish_job() call that
	 * retires the job. Not the same as state == MLA_JOB_DONE, which
	 * mla_sched_dequeue() sets just before its caller finalizes.
	 */
	bool                 finalized;

	/* Watchdog: armed at start, cancelled at progdone. */
	struct timer_list    timeout;

	/* Sync-mode (MLA_CMD_FLAG_WAIT) wakeup */
	bool                 sync_wait;
	struct completion    sync_done;

	/* Async-completion state. The stashed out_ptr / out_len are the
	 * caller's user-space resp buffer (validated by mla_async_build_job at
	 * submit time); MLA_IOCTL_REAP fills resp there at the terminal point.
	 * Zero on the sync JOB_SUBMIT path.
	 */
	u64                  async_out_ptr;
	u32                  async_out_len;
	ktime_t              ts_op_enter;   /* stamped at async build entry */

	/* v2.5 Phase 1: set by mla_hw_prep_job() after apply_relocs +
	 * pre_dma_sync succeed. Dispatch skips those steps when true.
	 * The async prep worker sets it only after winning
	 * cmpxchg(bank PREPARING -> READY) in mla_hw_prep_work_fn; dispatch's
	 * paired acquire cmpxchg(READY -> ACTIVE) is what publishes the
	 * descriptor bytes before HW fetches them. The bank state, not this
	 * flag, is the worker-vs-dispatch handshake.
	 */
	bool                 prepped;

	/* v2.6 descriptor-bank pool: which bank of job->model->banks[] this
	 * job's descriptor patches were written into. MLA_BANK_UNASSIGNED
	 * (0xff) means the walker hasn't picked a bank yet; dispatch's
	 * inline prep path assigns one on the fly. Any value < num_banks
	 * is a legit bank index owned by this job until PROGDONE.
	 */
	u8                   bank_idx;

	/* The bank whose writer ref the prep worker must drop. Separate from
	 * bank_idx because finish_job clears that on the owning job: dispatch can
	 * steal the bank and run the job to completion before the worker is ever
	 * scheduled, and a worker finding MLA_BANK_UNASSIGNED would return without
	 * releasing its ref, leaving the bank unclaimable for the model's life.
	 * Set with the claim under sched_lock; cleared only by the queue_work
	 * rollback, which drops the ref in the worker's place.
	 */
	u8                   prep_bank_idx;

	/* v2.5 Phase 2: async prep-ahead worker. When
	 * mla_sched_try_prep_ahead finds a queued job that can claim a FREE
	 * descriptor bank, it queues prep_work onto mdev->prep_wq; the worker
	 * runs mla_hw_apply_relocs + pre_dma_sync out from under the critical
	 * path. Worker-vs-dispatch is resolved entirely by the bank state
	 * machine (mla_desc_bank.state FREE/PREPARING/READY/ACTIVE) plus the
	 * mdev->prep_in_flight fan-out bound — see mla_hw_prep_work_fn. The
	 * worker holds a job fence ref for its lifetime so mla_fence_release
	 * can't kfree the job out from under the bank->state / bank_idx
	 * accesses.
	 */
	struct work_struct   prep_work;

	/* Optional phase trace (filled when MLA_OP_JOB_SUBMIT caller
	 * passed an extended resp). All ktime stamps; deltas computed at
	 * the end. ts_submit is set in mla_op_job_submit just before
	 * enqueue; ts_go in mla_hw_program_and_start right before the GO
	 * write; ts_irq in mla_irq_non_ftl on entry; ts_complete in
	 * mla_sched_complete on entry; ts_wake right before the
	 * completion's `complete()`.
	 */
	ktime_t              ts_submit;
	ktime_t              ts_dispatch;
	ktime_t              ts_reloc_loop_start;   /* apply_relocs_v2: after setup, before loop */
	ktime_t              ts_after_relocs;
	ktime_t              ts_after_sync;
	ktime_t              ts_go;
	ktime_t              ts_irq;
	ktime_t              ts_complete;
	ktime_t              ts_wake;

	/* Slice 4.7.x diagnostic: TLC sync-controller perf counters
	 * sampled in the non-FTL IRQ right after PROGDONE. Indexed by
	 * slot id (0..3), values are microseconds (CNTR cycles divided
	 * by mla_mhz). Filled by mla_hw_perfctr_read_us — never zeros
	 * unless the counters are disabled, in which case the JOB_SUBMIT
	 * resp leaves them as 0.
	 */
	u32                  perf_ctr_us[4];

	/* ioctl-async completion carrier. Used when the job was
	 * submitted via MLA_IOCTL_SUBMIT_ASYNC (sync_wait false).
	 * mla_sched_notify_done does nothing for this job — its dma-fence is
	 * signalled by mla_sched_finish_job and ioctl_cb wakes the reaper.
	 * Collected by MLA_IOCTL_REAP, which does the resp copy + ref drops.
	 * ioctl_link is on ctx->ioctl_inflight.
	 */
	bool                 ioctl_async;
	u64                  ioctl_cookie;
	struct list_head     ioctl_link;
	struct dma_fence_cb  ioctl_cb;
	/* Set by ioctl_cb when the fence signals; read by the reaper to pick
	 * this job without walking fence state. The callback also bumps
	 * ctx->ioctl_signalled so the reap predicate is O(1).
	 */
	bool                 ioctl_done;
};

/* Per-fd context. Allocated in mla_open(), freed in mla_release(). */
struct mla_context {
	struct mla_device  *mdev;
	struct list_head    node;          /* on mdev->contexts */

	/* identity */
	pid_t               creator_pid;
	kuid_t              uid;
	kgid_t              gid;
	u32                 kind;          /* MLA_CTX_* */
	u64                 vm_token;

	/* Scheduling. group_prio and intra_group_prio ARE honored — they
	 * are flattened by mla_sched_prio_key() into the plist key the run
	 * queue orders on. Defaults come from the caller's nice value at
	 * open; MLA_OP_CTX_CONFIGURE retunes them under ctx->lock.
	 *
	 * weight and realtime are stored and validated but not consumed by
	 * the scheduler (see mla_sched.c); do not expose them through a
	 * userspace API before they are.
	 */
	s8                  group_prio;
	s8                  intra_group_prio;
	u32                 weight;
	bool                realtime;

	/* state */
	struct mutex        lock;

	/* Context lifetime. A context can OUTLIVE its /dev/mla fd:
	 * buffers, models and jobs all hold a ->ctx back-pointer and all
	 * have independent lifetimes (see docs/CTX_LIFETIME_DESIGN.md).
	 *
	 * One ref is held by the fd itself, dropped in mla_release. Each
	 * child object takes one at creation and drops it when destroyed.
	 * The last put runs mla_context_destroy.
	 *
	 * The refcount is what prevents `fatal` from being read through a
	 * dangling pointer after teardown.
	 */
	refcount_t          refcount;
	bool                fatal;

	/* Set by mla_context_shutdown: the fd is closed and the registries
	 * are drained, but children may still hold refs. Anything reached
	 * from a surviving child must treat the context as inert — no new
	 * jobs, no new registrations. Distinct from `fatal`, which means
	 * the HARDWARE wedged.
	 */
	bool                shutdown;

	/* Slice 2: per-context buffer registry.
	 *
	 * xarray (XA_FLAGS_ALLOC1) handles its own internal locking, so
	 * no separate mutex is needed for alloc / load / erase.
	 * IDs start at 1; 0 is reserved as "invalid".
	 */
	struct xarray       buffers;       /* internal_buf_id -> mla_buffer */

	/* Slice 3: per-context model registry */
	struct xarray       models;        /* model_id -> mla_model */

	/* Slice 4: per-context job registry.
	 *
	 * Async jobs are erased at their terminal point (MLA_IOCTL_REAP), so
	 * this holds only genuinely-outstanding work. Sync jobs are never
	 * registered here at all — see the comment in mla_op_job_submit.
	 */
	struct xarray       jobs;          /* job_id -> mla_job */

	/* Cursor for xa_alloc_cyclic on `jobs`. Job IDs must NOT be reused
	 * promptly: userspace can hold a job_id across the job's
	 * completion (it cancels by ID), and plain xa_alloc would hand the
	 * freed ID straight to the next submit — so a stale cancel would
	 * silently kill an unrelated job. Same failure shape as the
	 * mla_model pointer-reuse bug; see docs/JOB_LIFETIME_DESIGN.md §4.4.
	 *
	 * This BOUNDS the hazard (2^32 submits) rather than removing it;
	 * the userspace `reaped` guard covers the common case.
	 *
	 * New members go at the END of this struct.
	 */
	u32                 next_job_id;

	/* This context installed the current perf-counter selection, so its
	 * shutdown must clear it (mdev->perfctr_* and the CTRL registers).
	 *
	 * A flag on the context rather than a context pointer on the device
	 * on purpose: there is no cross-object pointer to go stale, and no
	 * comparison against a possibly-freed address. It carries no
	 * arbitration meaning — it exists only so cleanup knows whose job
	 * it is.
	 *
	 * Do NOT read this flag alone as "the device selection is mine". It
	 * cannot be: mla_context_shutdown decrements mdev->contexts_open
	 * BEFORE the job/model/buffer drain and only calls
	 * mla_perfctr_release() after it, so inside that window a peer context
	 * sees contexts_open == 1, passes both the sole-context and idle gates,
	 * and installs its own selection. The flag says "I configured
	 * something once"; @perfctr_gen below says whether it is still live.
	 * mla_perfctr_release() requires both.
	 *
	 * The sole-context gate does NOT make a second owner impossible.
	 */
	bool                owns_perfctr;

	/* mdev->perfctr_gen as it stood when THIS context's op 19 installed
	 * its selection. Equal to mdev->perfctr_gen means our selection is
	 * still the live one; unequal means a peer replaced it or a recovery
	 * wiped it, and we must not disable counters we no longer own.
	 * Meaningless unless owns_perfctr is set.
	 */
	u32                 perfctr_gen;

	/* Ioctl-async transport state, strictly per-context so concurrent
	 * /dev/mla openers never touch each other's reap queue. A job submitted
	 * via MLA_IOCTL_SUBMIT_ASYNC stays on @ioctl_inflight until MLA_IOCTL_REAP
	 * (or mla_context_shutdown) collects it; its fence callback wakes
	 * @ioctl_wq.
	 */
	struct list_head    ioctl_inflight;
	u32                 ioctl_inflight_count; /* unreaped async jobs, capped at MLA_MAX_INFLIGHT_PER_CTX */
	spinlock_t          ioctl_lock;    /* guards @ioctl_inflight and @ioctl_inflight_count */
	wait_queue_head_t   ioctl_wq;
	/* Count of inflight jobs whose fence has signalled but which the reaper
	 * has not yet collected. Bumped lock-free in ioctl_cb, so the reap
	 * waitqueue predicate is O(1) instead of walking @ioctl_inflight.
	 */
	atomic_t            ioctl_signalled;
};

/*
 * Module parameters — storage + module_param() declarations live in
 * mla_drv.c. Referenced from mla_ops.c / mla_hw.c / mla_sched.c hot
 * paths; the externs live here so checkpatch's AVOID_EXTERNS rule is
 * satisfied and there's a single point of truth for the surface.
 */
extern bool mla_do_dma_sync;
extern uint mla_desc_cma_enable;
extern uint mla_reloc_range_flush;
extern uint mla_debug;
extern bool mla_no_recover;
extern bool mla_reprog_always;
extern bool mla_skip_hw;
extern bool mla_defer_notify;
extern uint mla_inject_fault;
extern int mla_inject_prep_fail;
extern uint mla_inject_no_complete;
extern uint mla_inject_reloc_delay_ms;
extern uint mla_inject_prep_delay_us;
extern uint mla_reloc_quiesce_grace_ms;

/*
 * mla_debug — verbosity bitmask, writable via
 * /sys/module/mla/parameters/mla_debug at runtime.
 *
 * Bit 0 (MLA_DBG_TRACE): one-shot per-event traces (MODEL_DEFINE,
 *   JOB_SUBMIT header, PROGDONE, enqueue, burst-dance, prep-ahead).
 *   Bounded rate; low volume.
 * Bit 1 (MLA_DBG_HOT):   per-item inner-loop traces (per-tile,
 *   per-DFM, per-reloc, per-iotensor, per-section). Volume scales
 *   with model size — 2000+ lines per job on cache_token2047.
 *   Combined with serial-console printk (~15 ms/line) this can
 *   trip the 2s JOB_SUBMIT timeout — debug-only, do NOT enable
 *   during perf runs.
 *
 * Bits 0x4..0x80 are reserved.
 */
#define MLA_DBG_TRACE  (1u << 0)
#define MLA_DBG_HOT    (1u << 1)

/* From mla_ctx.c */
extern const struct file_operations mla_fops;
struct mla_context *mla_context_alloc(struct mla_device *mdev);

/* Drain the registries and mark the context inert. Called once, from
 * mla_release on fd close. Does NOT free — see mla_context_put.
 */
void mla_context_shutdown(struct mla_context *ctx);

/* Drop a context reference; frees on the last put.
 *
 * May run from atomic / IRQ-adjacent context (the last put can come from
 * mla_fence_release), so the free path must not sleep.
 */
void mla_context_put(struct mla_context *ctx);

/* Take a context reference. Every object that stores a ->ctx
 * back-pointer must hold one for as long as that pointer is readable.
 */
static inline struct mla_context *mla_context_get(struct mla_context *ctx)
{
	if (ctx)
		refcount_inc(&ctx->refcount);
	return ctx;
}

/* From mla_ops.c */
int mla_op_info(struct mla_context *ctx,
		const struct mla_uring_cmd_sqe *sqe,
		const struct mla_cmd_hdr *hdr);
int mla_op_ctx_configure(struct mla_context *ctx,
			 const struct mla_uring_cmd_sqe *sqe,
			 const struct mla_cmd_hdr *hdr);

/* From mla_pool.c — DMA-heap consumer glue. Prefer the shared simaai,mla
 * heap; use the original DMS heap when the shared heap is unavailable.
 */
int  mla_heap_init(struct mla_device *mdev);
bool mla_dmabuf_is_dms(struct mla_device *mdev, struct dma_buf *dmabuf);
bool mla_dmabuf_no_cpu_alias(struct mla_device *mdev, struct dma_buf *dmabuf);
struct mla_buffer *mla_heap_alloc(struct mla_device *mdev, size_t size,
				      u32 flags);
void mla_heap_free(struct mla_device *mdev, struct mla_buffer *buf);

/* From mla_dmabuf.c — exporter for WB-cached CMA section buffers
 * (SECTION_ALLOC with MLA_SECTION_FLAG_CPU_RMW). Falls back to
 * mla_heap_alloc when the shared heap is selected or mla_desc_cma_enable is 0.
 */
struct mla_buffer *mla_secbuf_alloc_cached(struct mla_device *mdev,
					   size_t size);

/* NOTE: Per-job IFM/OFM cache maintenance is the caller's obligation,
 * discharged with DMA_BUF_IOCTL_SYNC on its own mmap; see the block
 * comment in mla_hw.c for why. Do not reintroduce a per-dispatch sync
 * here — if a consumer appears to need one, the bug is a missing bracket
 * in that consumer, and it is equally broken on the MIP transport, which
 * never had a driver in the dispatch path.
 *
 * Buffer-class predicates are needed for DESCRIPTOR maintenance, which is
 * the driver's job; that lives in mla_pool.c and keys on cma_backed /
 * heap_backed directly.
 */

/* From mla_ops_buf.c (Slice 2) */
int  mla_op_buf_alloc(struct mla_context *ctx,
		      const struct mla_uring_cmd_sqe *sqe,
		      const struct mla_cmd_hdr *hdr);
int  mla_op_buf_import(struct mla_context *ctx,
		       const struct mla_uring_cmd_sqe *sqe,
		       const struct mla_cmd_hdr *hdr);
int  mla_op_buf_info(struct mla_context *ctx,
		     const struct mla_uring_cmd_sqe *sqe,
		     const struct mla_cmd_hdr *hdr);
int  mla_op_buf_attach_phys(struct mla_context *ctx,
			    const struct mla_uring_cmd_sqe *sqe,
			    const struct mla_cmd_hdr *hdr);
int  mla_op_buf_release(struct mla_context *ctx,
			const struct mla_uring_cmd_sqe *sqe,
			const struct mla_cmd_hdr *hdr);
void mla_context_buffers_release(struct mla_context *ctx);

/* Buffer-registry helpers, exposed so other op handlers can register
 * their own kernel-side allocations (notably section-load pool buffers
 * in slice 4.7.1) without going through the dma-buf-export path.
 */
int  mla_buffer_register(struct mla_context *ctx, struct mla_buffer *buf);
struct mla_buffer *mla_buffer_find(struct mla_context *ctx, u32 id);
void mla_buffer_drop(struct mla_buffer *buf);
/* Same as mla_buffer_drop but defers the (possibly sleeping) last-reference
 * teardown to mdev->buf_release_wq. For callers that may be in softirq
 * (e.g. mla_job_put_bindings via mla_fence_release).
 */
void mla_buffer_drop_async(struct mla_buffer *buf);

/* kfree an mla_buffer and release its context reference. THE ONLY place
 * an mla_buffer is freed — every free path must route here so the
 * mla_context_put from mla_buffer_register cannot be missed.
 */
void mla_buffer_free(struct mla_buffer *buf);

int  mla_op_model_release(struct mla_context *ctx,
			  const struct mla_uring_cmd_sqe *sqe,
			  const struct mla_cmd_hdr *hdr);
void mla_context_models_release(struct mla_context *ctx);

/* From mla_pool.c (Slice 4.7.1) — kernel-owned pool helpers, used by
 * the section-load handler. Pool buffers are carveout-heap dma-bufs
 * (mla_heap_alloc) without a userspace fd; the mla_buffer is the
 * only owner and mla_pool_free tears the backing down.
 */
struct mla_buffer *mla_pool_alloc(struct mla_device *mdev, size_t size);

/* Allocate a pool-owned WB descriptor buffer. Uses the shared heap when
 * available, otherwise dma_alloc_noncoherent. Caller must publish CPU
 * writes with mla_pool_sync_for_device before every DMA handoff. Setting
 * mla_desc_cma_enable=0 uses heap exporter full-buffer flushes instead.
 */
struct mla_buffer *mla_pool_alloc_cached(struct mla_device *mdev, size_t size);

/* Publish CPU writes to a WB-cached buffer before a DMA hand-off.
 * cma_backed → dma_sync_single_for_device; heap_backed → the
 * exporter's end_cpu_access (VA-based clean over the heap's vmap;
 * dma_sync_* must never run on no-map carveout memory). No-op for
 * anything else, so callers can call unconditionally over a mixed
 * section_bufs[] array.
 */
void              mla_pool_sync_for_device(struct mla_device *mdev,
					   struct mla_buffer *buf);

/* Range-scoped variant. Flushes only [offset, offset+size). Callers
 * that dirtied a small region of a large blob (e.g. per-job reloc
 * patcher touching a few dozen u64s in a multi-MB descriptor blob)
 * MUST use this — dma_sync_single_for_device over the whole buffer
 * walks every cache line and costs 100s of µs on typical blobs.
 * Shared-heap page-backed buffers retain range sync through their SG
 * mapping; legacy no-map DMS and mla_desc_cma_enable=0 use the exporter.
 */
void              mla_pool_sync_range_for_device(struct mla_device *mdev,
						 struct mla_buffer *buf,
						 size_t offset, size_t size);

void              mla_pool_free(struct mla_device *mdev, struct mla_buffer *buf);

/* Slice 4.7.2: content-hash dedup. The entry is opaque to most of the
 * driver — only mla_pool.c (where the table lives) and the
 * section-load handler (which does lookup/insert) touch fields. The
 * dedup table is initialized at probe, torn down at remove.
 */
struct mla_section_dedup_entry;

void mla_section_dedup_init(struct mla_device *mdev);
void mla_section_dedup_destroy(struct mla_device *mdev);

/* Look up by content hash AND size. Both are part of the key: the hash alone
 * let a section with a matching SHA but a different size receive an alias to
 * the other one's buffer, at the other one's size. @size is normalised with
 * PAGE_ALIGN internally, so callers pass the raw requested size.
 *
 * NOTE the hash itself is supplied by userspace and is NOT verified in the
 * kernel, and the table is device-wide. A client that can open /dev/mla can
 * therefore claim another client's section by presenting its hash. Bounded
 * only by /dev/mla being mode 0660 — every opener is trusted. This must be
 * closed before multi-tenancy ships; see docs/SECTION_ALLOC_ABI.md.
 */
struct mla_section_dedup_entry *
mla_section_dedup_lookup(struct mla_device *mdev, const u8 sha256[32],
			 size_t size);

/* Insert a freshly-allocated, freshly-populated buffer into the table.
 * On success, returns an entry whose ->buf is either @primary (we won
 * the insert race) or some other buffer (a concurrent inserter beat us
 * — the caller must free their @primary). The entry's refcount is
 * always >= 1, with one of those references owned by the caller (drop
 * via mla_section_dedup_put when the per-context handle is dropped).
 *
 * On failure, returns ERR_PTR(-ENOMEM) — the caller still owns
 * @primary and must free it.
 */
struct mla_section_dedup_entry *
mla_section_dedup_insert(struct mla_device *mdev,
			 const u8 sha256[32],
			 struct mla_buffer *primary);

/* Drop one reference. When the last reference goes away the entry is
 * unlinked and the underlying primary mla_buffer is freed via
 * mla_pool_free (RCU-deferred).
 */
void mla_section_dedup_put(struct mla_device *mdev,
			   struct mla_section_dedup_entry *e);

/* Accessor — needed by the section-load handler to compare the
 * returned entry's buf against the buf it tried to insert. Other
 * callers should not poke into the entry struct directly.
 */
struct mla_buffer *mla_section_dedup_entry_buf(
	const struct mla_section_dedup_entry *e);
const u8 *mla_section_dedup_entry_sha256(
	const struct mla_section_dedup_entry *e);

/* Debug walker: invoke @cb for every live dedup entry on @mdev. Holds the
 * device dedup spinlock across the entire walk — @cb must not sleep and
 * must not re-enter dedup functions. Used by the debugfs dumper.
 */
typedef void (*mla_dedup_walker_cb)(struct mla_section_dedup_entry *e,
				    unsigned int refcount, void *arg);
void mla_section_dedup_walk(struct mla_device *mdev,
			    mla_dedup_walker_cb cb, void *arg);

/* Debug refcount for a dedup entry (spin-lock protected snapshot). */
unsigned int mla_section_dedup_entry_refcount(
	const struct mla_section_dedup_entry *e);

/* Debugfs anchor lifecycle. Cheap no-ops when !CONFIG_DEBUG_FS. */
void mla_debugfs_init(struct mla_device *mdev);
void mla_debugfs_exit(struct mla_device *mdev);

/* simaai_mlamem.c — legacy /dev/simaai-mlamem for pre-/dev/mla mla-rt */
int  simaai_mlamem_init(struct mla_device *mdev);
void simaai_mlamem_exit(struct mla_device *mdev);

/* SECTION_ALLOC_ABI Phase 1 op handlers. */
int  mla_op_section_alloc(struct mla_context *ctx,
			  const struct mla_uring_cmd_sqe *sqe,
			  const struct mla_cmd_hdr *hdr);
int  mla_op_section_commit(struct mla_context *ctx,
			   const struct mla_uring_cmd_sqe *sqe,
			   const struct mla_cmd_hdr *hdr);

/* Create an alias mla_buffer over a dedup entry and register it in @ctx.
 * Sole caller is SECTION_ALLOC's dedup HIT path.
 */
int  mla_section_alloc_alias(struct mla_context *ctx,
			     struct mla_section_dedup_entry *entry,
			     struct mla_buffer **out_buf);

int  mla_op_model_define_v3(struct mla_context *ctx,
			    const struct mla_uring_cmd_sqe *sqe,
			    const struct mla_cmd_hdr *hdr);
int  mla_op_model_update_reloc(struct mla_context *ctx,
			       const struct mla_uring_cmd_sqe *sqe,
			       const struct mla_cmd_hdr *hdr);

/* Semaphore detection walks the DMA descriptor blob via section_bufs[]
 * + dma_info_typed[]. If any descriptor sets chk_sema or set_sema the
 * scheduler needs the 256B WR burst before dispatch.
 */
bool mla_model_uses_semaphores_v2(struct mla_model *m);
void mla_job_init_internal(struct mla_job *job, struct mla_device *mdev);
/* Tear down an mla_model. Reusable by mla_op_model_define's error paths. */
void mla_model_destroy(struct mla_model *m);
/* Drop a model reference from a possibly-atomic context (mla_fence_release).
 * mla_model_destroy() blocks, so the last put is deferred to buf_release_wq.
 */
void mla_model_put_async(struct mla_model *m);

/* v2.6 descriptor-bank pool: grow banks[] by one slot. Called at enqueue
 * when outstanding > num_banks and num_banks < MLA_MAX_BANKS. Allocates
 * a new dma.descs blob via mla_pool_alloc_cached, copies the primary
 * blob into it, and bumps num_banks. Takes bank_grow_lock and re-checks
 * num_banks under the lock (TOCTOU-safe). Returns 0 on success, negative
 * errno on allocation failure or if the ceiling was already reached
 * (which is not itself an error — the caller runs on whatever's
 * available).
 */
int mla_model_grow_banks(struct mla_model *m);

/* From mla_hw.c (Slice 4 + 4.5 + 5) */
int  mla_hw_prep_job(struct mla_job *job, u8 bidx, bool from_worker);
void mla_hw_prep_work_fn(struct work_struct *work);
int  mla_hw_program_and_start(struct mla_job *job);
int  mla_hw_init(struct mla_device *mdev);
int  mla_hw_reset(struct mla_device *mdev);
void mla_hw_enable_irqs(struct mla_device *mdev);
void mla_hw_set_dma_burst_size(struct mla_device *mdev, u32 rd, u32 wr);
void mla_hw_clear_stale_irqs(struct mla_device *mdev);
void mla_hw_quiesce_irqs(struct mla_device *mdev);
void mla_hw_repair_irq_masks(struct mla_device *mdev);
int  mla_hw_recover(struct mla_device *mdev);
u32  mla_hw_read_pll_freq_mhz(struct mla_device *mdev);
void mla_hw_dump_status(struct mla_device *mdev, const char *tag);

/* TLC sync-controller perf counters, backing MLA_OP_PERFCTR_CONFIG.
 *
 * Opt-in: nothing is programmed until userspace selects signals, so probe
 * and recovery both call _disable_all() and _read_us() then returns zeros
 * without touching the hardware. Values are microseconds. The slot count
 * must match MLA_PCT_NUM_SLOTS in mla_regs.h — MLA_PERFCTR_MAX_SLOTS in
 * the uapi is the same number, kept separate to avoid a circular include
 * between mla_drv.h and mla_regs.h.
 *
 * _apply_locked() expects mdev->sched_lock held and mdev->perfctr_*
 * already updated; it does the MMIO. _disable_all() takes the lock itself
 * and is safe to call from probe and from the recovery path.
 */
void mla_hw_perfctr_apply_locked(struct mla_device *mdev);
void mla_hw_perfctr_disable_all(struct mla_device *mdev);
void mla_hw_perfctr_read_us(struct mla_device *mdev, u32 out[4]);

/* Fatal-IRQ classification used by FTL handler + recovery. */
enum mla_ftl_class {
	MLA_FTL_CLASS_NONE = 0,    /* no fatal bit set (spurious) */
	MLA_FTL_CLASS_MOSAIC,      /* msc_sram_2b_ecc_f / mcr_f / quadgrp_tmo */
	MLA_FTL_CLASS_TLC,         /* tlc_f */
	MLA_FTL_CLASS_MIM,         /* mim_f */
	MLA_FTL_CLASS_DMC,         /* dmc_f */
};

struct mla_ftl_decode {
	u32                 sys_ftl_sts;   /* raw aggregator value */
	enum mla_ftl_class  class;
	const char         *desc;          /* short text for dmesg */
};

/* IRQ handlers live in mla_drv.c alongside probe / fops. */
irqreturn_t mla_irq_non_ftl(int irq, void *data);
irqreturn_t mla_irq_ftl(int irq, void *data);
int         mla_irq_ftl_decode(struct mla_device *mdev,
			       struct mla_ftl_decode *out);

/* From mla_sched.c (Slice 4 + 5) */
void mla_sched_init(struct mla_device *mdev);
void mla_sched_exit(struct mla_device *mdev);
void mla_sched_enqueue(struct mla_job *job);
/* v2.5 Phase 2: try to queue prep for a plist head-of-queue candidate
 * (§3.2.1 walk). Caller must NOT hold sched_lock. No-op if in_flight
 * is NULL, no candidate is prep-eligible, or another prep is already
 * in flight.
 */
void mla_sched_try_prep_ahead(struct mla_device *mdev);
/* Exported so the prep worker can restart a queue the ownership gate
 * refused while the worker held the bank.
 */
void mla_sched_dispatch_next(struct mla_device *mdev);
void mla_sched_complete(struct mla_device *mdev, struct mla_job *job, int result);
struct mla_job *mla_sched_in_flight(struct mla_device *mdev);
bool mla_sched_dequeue(struct mla_job *job);
void mla_sched_fault_context(struct mla_context *ctx);
/* Backs MLA_OP_DEVICE_RESET: cancels every in-flight and queued job on
 * every context, resets the hardware, then clears mdev->device_fatal, the
 * recovery budget and every ctx->fatal. Process context only — takes
 * ctx_list_lock and flushes a workqueue.
 */
int  mla_sched_device_reset(struct mla_device *mdev, u32 *jobs_cancelled,
			    u32 *ctxs_cleared, bool *was_device_fatal);

/* From mla_job.c (Slice 4) */
void mla_job_get(struct mla_job *job);
void mla_job_put(struct mla_job *job);
void mla_job_arm_timeout(struct mla_job *job);
/* Release the per-tensor buffer references taken at submit-time resolve.
 * Idempotent; safe from softirq.
 */
void mla_job_put_bindings(struct mla_job *job);
void mla_job_cancel_timeout(struct mla_job *job);
void mla_job_cancel_timeout_sync(struct mla_job *job);

int  mla_op_job_submit(struct mla_context *ctx,
		       const struct mla_uring_cmd_sqe *sqe,
		       const struct mla_cmd_hdr *hdr);
int  mla_op_job_cancel(struct mla_context *ctx,
		       const struct mla_uring_cmd_sqe *sqe,
		       const struct mla_cmd_hdr *hdr);
int  mla_op_perfctr_config(struct mla_context *ctx,
			   const struct mla_uring_cmd_sqe *sqe,
			   const struct mla_cmd_hdr *hdr);
/* Drop this context's perf-counter selection, if it owns one, and disable
 * the hardware slots. Called from mla_context_shutdown and from the
 * recovery path. Idempotent; no-op when the context owns nothing.
 */
void mla_perfctr_release(struct mla_context *ctx);
int  mla_op_device_reset(struct mla_context *ctx,
			 const struct mla_uring_cmd_sqe *sqe,
			 const struct mla_cmd_hdr *hdr);
void mla_context_jobs_release(struct mla_context *ctx);

/* Ioctl-async transport. mla_context_ioctl_drain MUST run at fd close before
 * mla_context_jobs_release (see its definition for why).
 */
int  mla_ioctl_submit_async(struct mla_context *ctx, struct mla_ioctl_submit *s);
int  mla_ioctl_reap(struct mla_context *ctx, struct mla_ioctl_reap *r);
void mla_context_ioctl_drain(struct mla_context *ctx);

#endif /* _MLA_DRV_H_ */
