// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// Platform driver for the SiMa MLA. Binds to "simaai,mla-modalix" device
// tree nodes, maps the MLA register window, hooks the FTL / non-FTL
// interrupts, and publishes /dev/mla as a misc char device whose op
// surface flows through ioctl (see mla_ops.c).

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/cred.h>
#include <linux/debugfs.h>
#include <linux/devfreq.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/oom.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mla_drv.h"
#include "mla_regs.h"

/* ---------- devfreq clock framework consumer --------------------------- *
 * devfreq target/get_cur_freq drive mlapll rate via clk_set_rate. OPP
 * table comes from DT (operating-points-v2 = <&mla_opp_table>; entries
 * at 100..1000 MHz on Modalix, filtered by opp-supported-hw to 100..600 MHz
 * when the PLL boots at 600 MHz).
 */

static int mla_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct mla_device *mdev = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long rate = *freq;
	int ret;

	opp = devfreq_recommended_opp(dev, &rate, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	rate = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	ret = clk_set_rate(mdev->mlapll, rate);
	if (ret)
		return ret;

	/*
	 * devfreq sets LEAST_UPPER_BOUND only when it clamped the request
	 * to max_freq (1000 MHz or 600 MHz in case of a low-power boot).
	 */
	if (flags & DEVFREQ_FLAG_LEAST_UPPER_BOUND)
		dev_warn_ratelimited(dev,
				     "requested rate above max_freq, capped at %lu MHz\n",
				     rate / 1000000);

	mdev->mla_mhz = clk_get_rate(mdev->mlapll) / 1000000;
	*freq = rate;
	return 0;
}

static int mla_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct mla_device *mdev = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long rate = clk_get_rate(mdev->mlapll);

	opp = dev_pm_opp_find_freq_ceil(dev, &rate);
	if (!IS_ERR(opp))
		dev_pm_opp_put(opp);
	*freq = rate;
	return 0;
}

/* Both are defined at the bottom of this file, next to the platform
 * driver and module_init. mla_probe() below stamps .initial_freq into
 * the devfreq profile and matches against the OF table.
 */
static struct devfreq_dev_profile mla_devfreq_profile;
static const struct of_device_id mla_of_match[];

/* ============================ Per-fd context ============================ *
 *
 * struct mla_context is allocated on /dev/mla open, freed on release. It
 * holds this fd's private buffer/model/job registries, priority state
 * from MLA_OP_CTX_CONFIGURE, and the fatality flag consulted by the
 * ioctl dispatcher.
 */

/**
 * mla_context_alloc() — allocate a per-fd context on /dev/mla open
 * @mdev: device this context attaches to
 *
 * Allocates a struct mla_context, initializes its per-context
 * registries (buffers, models, jobs as xarrays with XA_FLAGS_ALLOC1),
 * caches the caller's pid/uid/gid for accounting and audit, and
 * inserts the context into mdev->contexts.
 *
 * Default scheduling priority is the caller's process nice value at
 * open time. Weight defaults to 1024 (the round-robin neutral). Both
 * can be retuned later via MLA_OP_CTX_CONFIGURE.
 *
 * Return: pointer to the new context; ERR_PTR(-EMFILE) if mdev already
 * has MLA_MAX_CONTEXTS; ERR_PTR(-ENOMEM) on allocation failure.
 */
struct mla_context *mla_context_alloc(struct mla_device *mdev)
{
	struct mla_context *ctx;

	if (atomic_inc_return(&mdev->num_contexts) > MLA_MAX_CONTEXTS) {
		atomic_dec(&mdev->num_contexts);
		return ERR_PTR(-EMFILE);
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		atomic_dec(&mdev->num_contexts);
		return ERR_PTR(-ENOMEM);
	}

	/* A job, fence or exported buffer can retain the context after
	 * /dev/mla closes. Keep the driver and its DMA device binding alive.
	 */
	if (!try_module_get(THIS_MODULE)) {
		kfree(ctx);
		atomic_dec(&mdev->num_contexts);
		return ERR_PTR(-ENODEV);
	}

	ctx->mdev          = mdev;
	ctx->creator_pid   = task_pid_nr(current);
	ctx->uid           = current_uid();
	ctx->gid           = current_gid();
	ctx->kind          = MLA_CTX_HOST_PROCESS;

	ctx->group_prio       = task_nice(current);
	ctx->intra_group_prio = ctx->group_prio;
	ctx->weight           = 1024;
	ctx->realtime         = false;

	mutex_init(&ctx->lock);
	xa_init_flags(&ctx->buffers, XA_FLAGS_ALLOC1);
	xa_init_flags(&ctx->models,  XA_FLAGS_ALLOC1);
	xa_init_flags(&ctx->jobs,    XA_FLAGS_ALLOC1);
	/* Per-context ioctl-async reap queue. */
	INIT_LIST_HEAD(&ctx->ioctl_inflight);
	spin_lock_init(&ctx->ioctl_lock);
	init_waitqueue_head(&ctx->ioctl_wq);
	atomic_set(&ctx->ioctl_signalled, 0);
	/* One reference for the fd. Children take their own. */
	refcount_set(&ctx->refcount, 1);
	ctx->shutdown = false;

	/* Bumped here rather than beside num_contexts above because this
	 * counts fds that are actually open, and the two error paths above
	 * return without ever producing one. Balanced in
	 * mla_context_shutdown, which is also where the list_del happens —
	 * the counter and @contexts must agree.
	 */
	atomic_inc(&mdev->contexts_open);

	mutex_lock(&mdev->ctx_list_lock);
	list_add_tail(&ctx->node, &mdev->contexts);
	mutex_unlock(&mdev->ctx_list_lock);

	dev_dbg(mdev->dev,
		"ctx alloc pid=%d uid=%u prio=%d\n",
		ctx->creator_pid, __kuid_val(ctx->uid), ctx->group_prio);
	return ctx;
}

/**
 * mla_context_shutdown() — drain a context on /dev/mla close
 * @ctx: context previously returned by mla_context_alloc()
 *
 * Removes the context from mdev->contexts, then drains its three
 * registries in dependency order: jobs first (so any in-flight job
 * gets cancelled before its model goes away), then models (which may
 * hold dma-buf refs from buffers), then buffers. Each *_release
 * helper handles its own teardown — including unmapping dma-bufs and
 * dropping fence references — so the registries are safe to xa_destroy
 * once those return.
 *
 * Does NOT free the context. Buffers, models and jobs can all outlive
 * the fd (an uncommitted BUF_ALLOC buffer belongs to the dma-buf
 * refcount and fput is deferred; a running job holds a model ref), and
 * every one of them dereferences its ->ctx back-pointer afterwards.
 * Freeing here is what produced the UAF documented in
 * docs/CTX_LIFETIME_DESIGN.md. The context dies on the last
 * mla_context_put instead.
 *
 * Runs exactly once, from mla_release.
 */
void mla_context_shutdown(struct mla_context *ctx)
{
	struct mla_device *mdev = ctx->mdev;

	mutex_lock(&mdev->ctx_list_lock);
	list_del(&ctx->node);
	mutex_unlock(&mdev->ctx_list_lock);

	/* The fd is going away, so drop out of the open population now. The
	 * context itself may live on for a while yet (jobs, models and
	 * buffers hold their own references) — that residual lifetime is what
	 * num_contexts tracks, and it is decremented in mla_context_destroy.
	 */
	atomic_dec(&mdev->contexts_open);

	/* Off the device list, so nothing new can find us by iteration.
	 * Mark inert before draining: a child that survives this must not
	 * be able to submit new work through a half-torn-down context.
	 */
	ctx->shutdown = true;

	/* Release the extra fence ref on any ioctl-async job still queued. MUST
	 * precede mla_context_jobs_release: while the xarray still holds each
	 * job's init ref, dropping the extra ref here can never be the last put.
	 */
	mla_context_ioctl_drain(ctx);

	mla_context_jobs_release(ctx);
	xa_destroy(&ctx->jobs);
	mla_context_models_release(ctx);
	xa_destroy(&ctx->models);
	mla_context_buffers_release(ctx);
	xa_destroy(&ctx->buffers);

	/* Drop any perf-counter selection this context installed, bounding a
	 * custom selection to the fd's lifetime so a context that opens later
	 * cannot silently inherit signals chosen by a process that has already
	 * exited. No-op unless this context configured them.
	 *
	 * Deliberately AFTER the job drain, not before: disabling a counter
	 * mid-flight would only spoil the measurement of a job whose owner is
	 * already exiting (clearing cntr_en cannot disturb the MLA's compute),
	 * but there is no reason to hand back a corrupted number when draining
	 * first costs nothing.
	 */
	mla_perfctr_release(ctx);

	dev_dbg(mdev->dev, "ctx shutdown pid=%d refs=%u\n",
		ctx->creator_pid, refcount_read(&ctx->refcount));
}

/**
 * mla_context_destroy() — final free, on the last reference
 * @ctx: context whose refcount just hit zero
 *
 * MUST NOT SLEEP. The last put can come from mla_fence_release, which
 * runs from the dma_fence completion path (softirq / IRQ-adjacent). So:
 * no allocation, no mutex_lock, no mutex_destroy that could sleep, no
 * dev_dbg with a mutex behind it. kfree + atomic_dec only.
 *
 * The xarrays were already xa_destroy'd in mla_context_shutdown, and
 * ctx->lock has no waiters by construction — every path that takes it
 * holds a context reference, so nobody can be blocked on it here.
 */
static void mla_context_destroy(struct mla_context *ctx)
{
	struct mla_device *mdev = ctx->mdev;

	kfree(ctx);
	atomic_dec(&mdev->num_contexts);
	module_put(THIS_MODULE);
}

/**
 * mla_context_put() — drop a context reference
 * @ctx: context to release a reference on; NULL is tolerated
 *
 * Frees the context when the last reference goes. Callers may be in
 * atomic context — see mla_context_destroy.
 */
void mla_context_put(struct mla_context *ctx)
{
	if (ctx && refcount_dec_and_test(&ctx->refcount))
		mla_context_destroy(ctx);
}

/* ============================ /dev/mla fops ============================ *
 *
 * Only the two lifecycle hooks (open/release) and the ioctl
 * dispatcher. No read/write — the entire op surface flows
 * through ioctl.
 */

/**
 * mla_open() — fops .open handler for /dev/mla
 * @inode: VFS inode for /dev/mla
 * @filp:  the just-opened struct file
 *
 * Pulls the owning struct mla_device out of the misc-device parent,
 * allocates a fresh context for this fd, and stashes it in
 * filp->private_data so the ioctl entry point can find it.
 *
 * Return: 0 on success; -EMFILE / -ENOMEM via mla_context_alloc().
 */
static int mla_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;     /* set by misc core */
	struct mla_device *mdev = container_of(misc, struct mla_device, miscdev);
	struct mla_context *ctx;
	int ret;

	/* Ungate the MLA clock for the lifetime of this fd */
	ret = pm_runtime_resume_and_get(mdev->dev);
	if (ret)
		return ret;

	ctx = mla_context_alloc(mdev);
	if (IS_ERR(ctx)) {
		pm_runtime_put(mdev->dev);
		return PTR_ERR(ctx);
	}

	/* A user-space backend may have driven this MLA since our last open
	 * and left our non-fatal IRQ masks disarmed — mla-rt --connect mip
	 * does exactly that, and the first JOB_SUBMIT afterwards would
	 * otherwise complete in hardware and die on the watchdog with no
	 * interrupt and no fault. No-op unless the masks really
	 * were clobbered.
	 */
	mla_hw_repair_irq_masks(mdev);

	filp->private_data = ctx;
	return 0;
}

/**
 * mla_release() — fops .release handler for /dev/mla
 * @inode: VFS inode (unused)
 * @filp:  the file being closed
 *
 * Frees the per-fd context. All in-flight jobs, models, and buffers
 * owned by this context are torn down here as a side effect of
 * mla_context_shutdown().
 *
 * Return: 0 always (close() doesn't have a sensible failure path).
 */
static int mla_release(struct inode *inode, struct file *filp)
{
	struct mla_context *ctx = filp->private_data;
	struct device *dev = ctx->mdev->dev;

	mla_context_shutdown(ctx);

	/* Drop the fd's reference. This frees the context only if no child
	 * outlived us; a leaked dma-buf or a still-running job legitimately
	 * keeps it alive until it drops its own ref.
	 */
	mla_context_put(ctx);

	pm_runtime_put(dev);
	return 0;
}

/*
 * Upper bound on any *_req struct. Conservative; bump as ops grow.
 * Used to bound copy_from_user and reject malformed req_len early.
 */
#define MLA_REQ_MAX_BYTES   1024

/*
 * mla_cmd_prologue() — validation for the command surface. @sqe carries a
 * {req_user_ptr, req_len} descriptor (a local copy built from the ioctl
 * argument). Copies the request
 * header out, bounds-checks the descriptor, enforces the reserved-field
 * contract, and applies the fatal-state gate. On success returns 0 with @hdr
 * populated; otherwise a negative errno the caller surfaces unchanged:
 *   -EHOSTDOWN  — device has been marked fatal (recovery exhausted)
 *   -EOWNERDEAD — this context's previous job faulted; close the fd and reopen
 *
 * MLA_OP_DEVICE_RESET is exempt from the fatal gate — it is the only op that
 * clears either fatal flag, and gating it would make a device-fatal board
 * unrecoverable without a module reload. Any future recovery op belongs in
 * this exemption too, but keep the list short and explicit, never a range test.
 */
static int mla_cmd_prologue(struct mla_context *ctx,
			    const struct mla_uring_cmd_sqe *sqe,
			    struct mla_cmd_hdr *hdr)
{
	if (!sqe->req_user_ptr ||
	    sqe->req_len < sizeof(*hdr) ||
	    sqe->req_len > MLA_REQ_MAX_BYTES)
		return -EINVAL;

	if (copy_from_user(hdr, u64_to_user_ptr(sqe->req_user_ptr), sizeof(*hdr)))
		return -EFAULT;

	if (hdr->op >= MLA_OP_MAX)
		return -EINVAL;

	/* Reserved fields must be zero so future ABI extensions can safely
	 * repurpose them; without enforcement userspace could rely on garbage
	 * being ignored, then break when a real meaning is assigned. The uapi
	 * header documents this contract.
	 */
	if (sqe->reserved || hdr->reserved[0] || hdr->reserved[1])
		return -EINVAL;

	if (hdr->op != MLA_OP_DEVICE_RESET) {
		if (READ_ONCE(ctx->mdev->device_fatal))
			return -EHOSTDOWN;
		if (READ_ONCE(ctx->fatal))
			return -EOWNERDEAD;
	}

	return 0;
}

/*
 * mla_dispatch_cmd() — run one synchronous op and return its result. Used by
 * MLA_IOCTL_SUBMIT_SYNC. The prologue has already validated @hdr.
 *
 * MLA_OP_JOB_ASYNC is deliberately not dispatched to the async handler here: it
 * needs to own a completion object (the reap machinery for
 * MLA_IOCTL_SUBMIT_ASYNC), which this synchronous path cannot provide. A sync
 * caller that asks for it is refused with -EINVAL.
 */
static int mla_dispatch_cmd(struct mla_context *ctx,
			    const struct mla_uring_cmd_sqe *sqe,
			    const struct mla_cmd_hdr *hdr)
{
	switch (hdr->op) {
	case MLA_OP_INFO:
		return mla_op_info(ctx, sqe, hdr);
	case MLA_OP_CTX_CONFIGURE:
		return mla_op_ctx_configure(ctx, sqe, hdr);
	case MLA_OP_BUF_ALLOC:
		return mla_op_buf_alloc(ctx, sqe, hdr);
	case MLA_OP_BUF_IMPORT:
		return mla_op_buf_import(ctx, sqe, hdr);
	case MLA_OP_BUF_INFO:
		return mla_op_buf_info(ctx, sqe, hdr);
	case MLA_OP_BUF_ATTACH_PHYS:
		return mla_op_buf_attach_phys(ctx, sqe, hdr);
	case MLA_OP_BUF_RELEASE:
		return mla_op_buf_release(ctx, sqe, hdr);
	case MLA_OP_MODEL_RELEASE:
		return mla_op_model_release(ctx, sqe, hdr);
	case MLA_OP_MODEL_DEFINE:
		/* Retired; use MLA_OP_MODEL_DEFINE_V3, which takes section_ids[]
		 * from prior SECTION_ALLOC / SECTION_COMMIT calls. Answered
		 * explicitly rather than falling into default's bare -EOPNOTSUPP
		 * so a stale caller gets told what to do. Op number 13 stays
		 * burned — never reuse it.
		 */
		dev_warn_once(ctx->mdev->dev,
			      "MLA_OP_MODEL_DEFINE (op 13) is retired; use MLA_OP_MODEL_DEFINE_V3 (op 16)\n");
		return -EOPNOTSUPP;
	case MLA_OP_SECTION_ALLOC:
		return mla_op_section_alloc(ctx, sqe, hdr);
	case MLA_OP_SECTION_COMMIT:
		return mla_op_section_commit(ctx, sqe, hdr);
	case MLA_OP_MODEL_DEFINE_V3:
		return mla_op_model_define_v3(ctx, sqe, hdr);
	case MLA_OP_JOB_SUBMIT:
		return mla_op_job_submit(ctx, sqe, hdr);
	case MLA_OP_JOB_ASYNC:
		return -EINVAL;
	case MLA_OP_JOB_CANCEL:
		return mla_op_job_cancel(ctx, sqe, hdr);
	case MLA_OP_DEVICE_RESET:
		return mla_op_device_reset(ctx, sqe, hdr);
	case MLA_OP_PERFCTR_CONFIG:
		return mla_op_perfctr_config(ctx, sqe, hdr);
	case MLA_OP_MODEL_UPDATE_RELOC:
		return mla_op_model_update_reloc(ctx, sqe, hdr);
	default:
		return -EOPNOTSUPP;
	}
}

/* ============================ IRQ handlers ============================ *
 *
 * Modalix is permanently QG0-only silicon: hardware programs and runs
 * exactly one quadgroup per job. The non-fatal handler recognizes the
 * QG0 progdone source and completes the in-flight job — multi-quadgroup
 * completion counting is not a concern here.
 *
 * Both handlers read+ack the TLC leaf source AND the MSC top-level
 * aggregator status. The aggregator EN masks are set unconditionally
 * at probe (via mla_hw_enable_irqs), so both must be W1C-acked or the
 * GIC line stays asserted and we get exactly one IRQ then silence.
 */

/**
 * mla_irq_non_ftl() — non-fatal IRQ handler (progdone + recoverable sources)
 * @irq:  Linux IRQ number
 * @data: cookie; the struct mla_device *
 *
 * Called from hard-IRQ context. Reads (and W1C-acks) the TLC leaf
 * source register plus the MSC top-level aggregator status. On any
 * non-zero source with an in-flight job, captures the active-duration
 * counter (cycles → µs) and completes the job with result=0 — which
 * signals the dma-fence, unblocks any sync waiter, and dispatches the
 * next queued job.
 *
 * Modalix runs only QG0 per job (silicon constraint), so a single
 * progdone equals full job completion. Other TLC sources (halt_trig,
 * sync_ctl, per-tile L1 ECC) are acked by the same write but do not
 * change semantics.
 *
 * Return: IRQ_HANDLED if a TLC/MSC source was acked; IRQ_NONE if
 * neither register had any source bits set (shared-line spurious).
 */
irqreturn_t mla_irq_non_ftl(int irq, void *data)
{
	struct mla_device *mdev = data;
	struct mla_job *job;
	u32 tlc_src, msc_sts;
	ktime_t ts_irq_entry = ktime_get();

	tlc_src = ioread32(mdev->regs + MLA_REG_TLC_NON_FTL_INTR_SRC);
	msc_sts = ioread32(mdev->regs + MLA_REG_SYS_NON_FTL_INTR_STS);
	if (!tlc_src && !msc_sts)
		return IRQ_NONE;

	if (tlc_src)
		iowrite32(tlc_src, mdev->regs + MLA_REG_TLC_NON_FTL_INTR_SRC);
	if (msc_sts)
		iowrite32(msc_sts, mdev->regs + MLA_REG_SYS_NON_FTL_INTR_STS);

	job = mla_sched_in_flight(mdev);
	if (!job) {
		dev_warn_ratelimited(mdev->dev,
			"non-FTL IRQ tlc=%#x msc=%#x with no in-flight job\n",
			tlc_src, msc_sts);
		return IRQ_HANDLED;
	}

	/* Modalix is permanently QG0-only — silicon never programs QG1-3
	 * (see mla_hw_compose_qg0_reg0 in mla_hw.c). Any TLC IRQ source
	 * we get here is the QG0 progdone (or one of its peer per-tile
	 * sources we ack but don't count). One progdone = job done.
	 */
	job->prog_done_count++;
	dev_dbg(mdev->dev,
		"irq job %u tlc_src=%#x msc_sts=%#x count=%u\n",
		job->id, tlc_src, msc_sts, job->prog_done_count);

	if (job->prog_done_count == 0) {
		dma_fence_put(&job->fence);
		return IRQ_HANDLED;
	}

	/* MLA_ACTIVE_DUR_CNT_MEAS0 increments at MLA core-clock rate.
	 * mla-rt converts via fscale = 1000/freq_MHz and reports in
	 * microseconds (see MLAStats.cc tilesLatency). We do the same:
	 * divide cycle count by mla_mhz to get microseconds directly.
	 *
	 * mla_mhz is populated at probe from the PRC PLL register. If
	 * that read failed (PRC unmapped or PLL unprogrammed) it stays
	 * 0; fall back to 1000 MHz, which matches Modalix's actual silicon.
	 */
	{
		u32 cycles = ioread32(mdev->regs + MLA_REG_MLA_ACTIVE_DUR_CNT_MEAS0);
		u32 mhz    = mdev->mla_mhz ? mdev->mla_mhz : 1000;

		job->latency_us = cycles / mhz;
	}

	/* Sample the configured TLC perf counters now, before the scheduler
	 * dispatches the next job and resets the MLA's QG-active window.
	 * Output is microseconds, slot i being the i-th signal userspace
	 * selected via MLA_OP_PERFCTR_CONFIG. mla_op_job_submit copies these
	 * into the resp on the waiter's behalf.
	 *
	 * Counters are opt-in: with nothing configured this zeroes the array
	 * and touches no registers, which is the common case and keeps 8 MMIO
	 * reads per job out of hard IRQ.
	 */
	mla_hw_perfctr_read_us(mdev, job->perf_ctr_us);

	job->ts_irq = ts_irq_entry;
	if (mla_debug & MLA_DBG_TRACE)
		dev_info(mdev->dev,
			"v2_trace: PROGDONE job %u path=irq ctx=%p prio=(%d,%d) latency_us=%llu tile=%u tact=%u iqfrz=%u dfrz=%u\n",
			job->id, job->ctx,
			job->ctx->group_prio, job->ctx->intra_group_prio,
			job->latency_us,
			job->perf_ctr_us[0], job->perf_ctr_us[1],
			job->perf_ctr_us[2], job->perf_ctr_us[3]);
	mla_sched_complete(mdev, job, 0);
	/* May be the last put, freeing the job from hard IRQ; mla_fence_release
	 * is written for that. Outside sched_lock, which it takes.
	 */
	dma_fence_put(&job->fence);
	return IRQ_HANDLED;
}

/**
 * mla_irq_ftl_decode() — read aggregator + leaf FTL status, classify
 * @mdev: device whose AXI window to read
 * @out:  caller-provided buffer for the decoded result
 *
 * Reads SYS_FTL_INTR_STS to find which fatal class fired, then
 * (for diagnostic logging only) reads the corresponding leaf status
 * register. Does NOT clear; that happens in the caller after the
 * decode is logged.
 *
 * The class mapping is one-to-one with the aggregator bit positions;
 * if multiple bits are set we return the most severe (mosaic-class
 * first, then DMC, MIM, TLC).
 *
 * Return: 0 on success and fills @out; -EAGAIN if no fatal bit was
 * set (spurious interrupt or shared-line situation).
 */
int mla_irq_ftl_decode(struct mla_device *mdev, struct mla_ftl_decode *out)
{
	u32 sts;

	if (!mdev->regs)
		return -ENODEV;

	sts = ioread32(mdev->regs + MLA_REG_SYS_FTL_INTR_STS);
	out->sys_ftl_sts = sts;

	if (!(sts & MLA_FTL_MASK_ALL)) {
		out->class = MLA_FTL_CLASS_NONE;
		out->desc  = "spurious";
		return -EAGAIN;
	}

	/* Mosaic-class bits are most severe (whole-device fault). Check
	 * them first so a multi-bit fault is attributed to mosaic.
	 */
	if (sts & ((1u << MLA_FTL_BIT_MSC_SRAM_2B_ECC) |
		   (1u << MLA_FTL_BIT_MCR) |
		   (1u << MLA_FTL_BIT_QUADGRP_TMO))) {
		out->class = MLA_FTL_CLASS_MOSAIC;
		if (sts & (1u << MLA_FTL_BIT_MSC_SRAM_2B_ECC))
			out->desc = "mosaic: SRAM 2-bit ECC";
		else if (sts & (1u << MLA_FTL_BIT_MCR))
			out->desc = "mosaic: MCR fault";
		else
			out->desc = "mosaic: quadgroup timeout";
		return 0;
	}
	if (sts & (1u << MLA_FTL_BIT_DMC)) {
		out->class = MLA_FTL_CLASS_DMC;
		out->desc  = "DMC: DMA fault";
		return 0;
	}
	if (sts & (1u << MLA_FTL_BIT_MIM)) {
		out->class = MLA_FTL_CLASS_MIM;
		out->desc  = "MIM: memory-interface fault";
		return 0;
	}
	if (sts & (1u << MLA_FTL_BIT_TLC)) {
		out->class = MLA_FTL_CLASS_TLC;
		out->desc  = "TLC: tile/quad fault";
		return 0;
	}
	out->class = MLA_FTL_CLASS_NONE;
	out->desc  = "unknown";
	return -EAGAIN;
}

/**
 * mla_irq_ftl() — fatal interrupt handler
 * @irq:  Linux IRQ number
 * @data: cookie; the struct mla_device *
 *
 * Decodes the fault, logs it once with leaf-bit attribution, acks
 * the aggregator, fails the in-flight job with -EIO, and triggers
 * the recovery workqueue. The handler runs in hard-IRQ context so
 * it does the minimum and defers reset to mla_recovery_work_fn().
 *
 * Return: IRQ_HANDLED if a fault was attributed; IRQ_NONE if no
 * fatal source is set (spurious / shared line).
 */
irqreturn_t mla_irq_ftl(int irq, void *data)
{
	struct mla_device *mdev = data;
	struct mla_job *job;
	struct mla_ftl_decode dec;
	int ret;

	ret = mla_irq_ftl_decode(mdev, &dec);
	if (ret == -EAGAIN)
		return IRQ_NONE;

	/* For mosaic-class faults read the leaf for additional log
	 * detail; cheap and helps post-mortem. dev_err_ratelimited so
	 * a fault storm doesn't hose the console.
	 */
	switch (dec.class) {
	case MLA_FTL_CLASS_MIM:
		dev_err_ratelimited(mdev->dev,
			"FTL: %s (sys=%#x mim=%#x)\n", dec.desc, dec.sys_ftl_sts,
			ioread32(mdev->regs + MLA_REG_MIM_F_INT));
		break;
	case MLA_FTL_CLASS_DMC:
		dev_err_ratelimited(mdev->dev,
			"FTL: %s (sys=%#x dmc=%#x)\n", dec.desc, dec.sys_ftl_sts,
			ioread32(mdev->regs + MLA_REG_DMC_F_INT));
		break;
	default:
		dev_err_ratelimited(mdev->dev,
			"FTL: %s (sys=%#x)\n", dec.desc, dec.sys_ftl_sts);
		break;
	}

	/* CRITICAL: quiesce IRQs immediately, in this IRQ context.
	 *
	 * If we only clear the aggregator and let the recovery worker
	 * disable IRQs later, the leaf source (e.g. DMC F_INT) keeps
	 * re-asserting and we get an IRQ storm — millions of FTL
	 * callbacks per second, which blocks the worker from ever
	 * running and effectively wedges the system.
	 *
	 * Disable EN masks first (no more new IRQs), THEN clear leaf
	 * status (DMC/MIM where applicable), THEN ack the aggregator.
	 * After this point the GIC line will deassert and stay
	 * deasserted until the recovery worker re-enables EN masks.
	 */
	mla_hw_quiesce_irqs(mdev);

	/* Clear the relevant leaf so when the recovery worker re-enables
	 * masks the latch isn't already pending.
	 */
	switch (dec.class) {
	case MLA_FTL_CLASS_DMC: {
		u32 leaf = ioread32(mdev->regs + MLA_REG_DMC_F_INT);

		iowrite32(leaf, mdev->regs + MLA_REG_DMC_F_INT);
		break;
	}
	case MLA_FTL_CLASS_MIM: {
		u32 leaf = ioread32(mdev->regs + MLA_REG_MIM_F_INT);

		iowrite32(leaf, mdev->regs + MLA_REG_MIM_F_INT);
		break;
	}
	default:
		break;
	}

	/* W1C ack of the aggregator. */
	iowrite32(dec.sys_ftl_sts, mdev->regs + MLA_REG_SYS_FTL_INTR_STS);

	mla_hw_dump_status(mdev, "ftl");

	job = mla_sched_in_flight(mdev);
	if (job) {
		static const u32 class_to_uapi[] = {
			[MLA_FTL_CLASS_NONE]   = MLA_FAULT_NONE,
			[MLA_FTL_CLASS_MOSAIC] = MLA_FAULT_MOSAIC,
			[MLA_FTL_CLASS_TLC]    = MLA_FAULT_TLC,
			[MLA_FTL_CLASS_MIM]    = MLA_FAULT_MIM,
			[MLA_FTL_CLASS_DMC]    = MLA_FAULT_DMC,
		};
		int i;

		job->fault_class   = class_to_uapi[dec.class];
		job->fault_sys_sts = dec.sys_ftl_sts;
		for (i = 0; i < 4; i++)
			job->fault_l2g_sts[i] = ioread32(mdev->regs +
				MLA_REG_L2G0_FTL_INTR_SRC0 + i * MLA_REG_L2G_STRIDE);
		mla_sched_complete(mdev, job, -EIO);
		dma_fence_put(&job->fence);
	}
	return IRQ_HANDLED;
}

/*
 * Request the two named IRQs from DT (ftl, non_ftl) and stash the
 * Linux IRQ numbers in mdev->irq[]. mla_recovery_work_fn (mla_sched.c)
 * is wired into mdev->recovery_work by mla_sched_init(). Any M4 mailbox
 * IRQ in DT is not used by this driver.
 */
static int mla_request_irqs(struct mla_device *mdev,
			    struct platform_device *pdev)
{
	static const struct {
		const char    *name;
		irq_handler_t  handler;
		int            slot;
	} irqs[] = {
		{ "ftl",     mla_irq_ftl,     MLA_IRQ_FTL     },
		{ "non_ftl", mla_irq_non_ftl, MLA_IRQ_NON_FTL },
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(irqs); i++) {
		int irq = platform_get_irq_byname(pdev, irqs[i].name);

		if (irq < 0) {
			dev_err(mdev->dev, "missing IRQ '%s': %d\n",
				irqs[i].name, irq);
			return irq;
		}
		ret = devm_request_irq(mdev->dev, irq, irqs[i].handler,
				       IRQF_SHARED, dev_name(mdev->dev), mdev);
		if (ret) {
			dev_err(mdev->dev, "request_irq '%s' failed: %d\n",
				irqs[i].name, ret);
			return ret;
		}
		mdev->irq[irqs[i].slot] = irq;
	}
	return 0;
}

/* ---- Probe / remove ---- */

/**
 * mla_oom_notify() - name MLA's CMA footprint in the OOM report
 * @nb:   embedded &struct mla_device.oom_nb
 * @val:  unused; the OOM chain passes no meaningful value
 * @data: unused; points at a "pages freed" counter we do not touch
 *
 * The standard OOM report names the invoking task and the victim, so a kill
 * caused by MLA holding a reusable CMA pool reads as unexplained. printk is
 * appropriate on this already-failing path.
 *
 * Return: NOTIFY_OK always. Reclaims nothing, so *data is left alone.
 */
static int mla_oom_notify(struct notifier_block *nb, unsigned long val,
			  void *data)
{
	struct mla_device *mdev = container_of(nb, struct mla_device, oom_nb);
	s64 imported = atomic64_read(&mdev->imported_heap_bytes);
	s64 internal = atomic64_read(&mdev->outstanding_alloc_bytes);
	u64 occupied = dma_heap_allocated_bytes(mdev->dms_heap);

	if (mdev->heap != mdev->dms_heap) {
		pr_info("mla: OOM: allocator \"%s\": %llu MiB in the original DMS heap, %llu MiB in fallback-owned Linux/CMA buffers; tracked MLA imports %lld MiB + pool buffers %lld MiB. The DMS region size is not the allocator's total capacity\n",
			dma_heap_get_name(mdev->heap), occupied >> 20,
			dma_heap_allocated_bytes(mdev->heap) >> 20,
			imported >> 20, internal >> 20);
		pr_info("mla: OOM: system CmaFree %lu MiB\n",
			(global_zone_page_state(NR_FREE_CMA_PAGES) << PAGE_SHIFT) >> 20);
	} else if (mdev->reserved_mem_attached) {
		/* Occupancy leads: the pool is consumed through the heap fd, so
		 * most of it is held by buffers this driver never sees.
		 */
		pr_info("mla: OOM: DMS pool \"%s\": %llu MiB of %llu MiB allocated via the dma-heap; MLA has %lld MiB mapped for the device + %lld MiB driver-internal. This memory IS part of Linux's RAM, so MLA's pool is a plausible contributor to this OOM\n",
			mdev->pool_name ? mdev->pool_name : "?",
			occupied >> 20, mdev->pool_size >> 20,
			imported >> 20, internal >> 20);
		/* A gap is the expected transient state during a model load, and
		 * one sample cannot distinguish that from a client holding
		 * unused buffers, so the wording must not assert hoarding.
		 */
		if ((u64)(imported + internal) < occupied)
			pr_info("mla: OOM: %llu MiB of the pool is allocated but not currently mapped to the MLA. This is expected while a model is loading; it indicates a client holding unused buffers only if it persists (watch mla/pool_usage)\n",
				(occupied - (u64)(imported + internal)) >> 20);
		pr_info("mla: OOM: system CmaFree %lu MiB -- free CMA pages count toward MemFree/MemAvailable but cannot satisfy unmovable allocations, so an application can see free memory it is unable to obtain\n",
			(global_zone_page_state(NR_FREE_CMA_PAGES) << PAGE_SHIFT) >> 20);
	} else {
		/* A no-map carveout is outside Linux's memory, so this rules MLA
		 * out rather than leaving it a suspect.
		 */
		pr_info("mla: OOM: MLA holds %lld MiB from no-map carveout \"%s\", which is outside Linux's memory and cannot have caused this OOM\n",
			(imported + internal) >> 20,
			mdev->pool_name ? mdev->pool_name : "?");
		pr_info("mla: OOM: system CmaFree %lu MiB\n",
			(global_zone_page_state(NR_FREE_CMA_PAGES) << PAGE_SHIFT) >> 20);
	}

	return NOTIFY_OK;
}

/**
 * mla_probe() — native DT probe handler
 * @pdev: platform device that matched mla_of_match
 *
 * Allocates struct mla_device, resolves the carveout dma-heap for its
 * memory-region, ioremap's the register window, requests the ftl and
 * non_ftl IRQs, and registers the misc device that publishes
 * /dev/mla. Used when the system DT contains a proper mla node owned
 * by this driver — not the takeover path.
 *
 * Return: 0 on success; negative errno from any of the underlying
 * platform/devm/misc calls. Errors propagate via dev_err_probe so
 * they show up in the deferred-probe trace.
 */
static int mla_probe(struct platform_device *pdev)
{
	struct mla_device *mdev;
	struct resource *res;
	const struct of_device_id *match;
	int ret;
	u32 supported_hw;

	mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, mdev);

	mutex_init(&mdev->ctx_list_lock);
	INIT_LIST_HEAD(&mdev->contexts);
	atomic_set(&mdev->num_contexts, 0);
	atomic_set(&mdev->contexts_open, 0);
	atomic64_set(&mdev->outstanding_alloc_bytes, 0);
	atomic64_set(&mdev->imported_heap_bytes, 0);
	atomic64_set(&mdev->buf_inc_ops, 0);
	atomic64_set(&mdev->buf_dec_ops, 0);
	atomic64_set(&mdev->job_inc_ops, 0);
	atomic64_set(&mdev->job_dec_ops, 0);
	mla_sched_init(mdev);
	mla_section_dedup_init(mdev);
	mla_debugfs_init(mdev);

	match = of_match_device(mla_of_match, &pdev->dev);
	mdev->soc_family = match ? (uintptr_t)match->data : MLA_SOC_UNKNOWN;

	/* DRAM DMA addresses must be representable by the MLA, not merely
	 * by dma_addr_t. Imported mapped extents are validated separately.
	 */
	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
	if (ret)
		goto err_debugfs;

	/* Platform devices provide dma_parms. MLA accepts contiguous DMA
	 * spans up to the SG length limit, not the generic 64-KiB default.
	 */
	dma_set_max_seg_size(&pdev->dev, UINT_MAX);

	/* Resolve the DT-owned heap and, for reusable DMS, bind descriptor
	 * DMA allocations to that same CMA area before any allocation.
	 */
	ret = mla_heap_init(mdev);
	if (ret)
		goto err_debugfs;

	/* After heap resolution so the notifier can name its backing. Failure
	 * costs only the diagnostic, so it must not fail probe.
	 */
	mdev->oom_nb.notifier_call = mla_oom_notify;
	if (register_oom_notifier(&mdev->oom_nb))
		dev_warn(&pdev->dev,
			 "cannot register OOM notifier; an application OOM caused by MLA's pool will not be annotated\n");
	else
		mdev->oom_nb_registered = true;

	mdev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mdev->regs)) {
		ret = PTR_ERR(mdev->regs);
		goto err_debugfs;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mdev->regs_size = resource_size(res);

	/* PRC clock/reset region (0xff00000). Read to derive the MLA
	 * core clock frequency at probe; written by mla_hw_reset to
	 * assert POR + SYS reset (bits 3+4 of MLA_CK_RST — the clk
	 * framework's gate only reaches bits 0+1 and does not clear DMA
	 * semaphore counters after a stalled semaphored job). One page,
	 * standalone from the MLA AXI window.
	 */
	mdev->prc_regs = ioremap(MLA_PRC_REGS_BASE, MLA_PRC_REGS_SIZE);
	if (!mdev->prc_regs)
		dev_warn(&pdev->dev,
			 "ioremap PRC 0x%llx failed; reset path unavailable\n",
			 (unsigned long long)MLA_PRC_REGS_BASE);

	/* Clock framework consumer (simaai,clkc-prc provider).
	 *   "pll":  rate-controllable parent (devfreq scales this)
	 *   "gate": MLA + debug clock enables on PRC +0x594
	 * Optional: legacy DTBs without the consumer wiring fall through
	 * to NULL clks.
	 */
	mdev->mlapll = devm_clk_get_optional(&pdev->dev, "pll");
	if (IS_ERR(mdev->mlapll)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(mdev->mlapll),
				    "clk_get(pll)\n");
		goto err_unmap_prc;
	}
	mdev->mlaclk = devm_clk_get_optional(&pdev->dev, "gate");
	if (IS_ERR(mdev->mlaclk)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(mdev->mlaclk),
				    "clk_get(gate)\n");
		goto err_unmap_prc;
	}

	if (mdev->mlaclk) {
		ret = clk_prepare_enable(mdev->mlaclk);
		if (ret) {
			ret = dev_err_probe(&pdev->dev, ret,
					    "clk_prepare_enable(gate)\n");
			goto err_unmap_prc;
		}
	}

	/*
	 * mlapll carries CLK_SET_RATE_GATE, so the devfreq knob below can
	 * only change the rate while every gate consumer has it unprepared.
	 * Releasing the gate on runtime suspend is what lets that interlock
	 * open when the MLA is idle.
	 */
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	if (mdev->mlapll) {
		mdev->mla_mhz = clk_get_rate(mdev->mlapll) / 1000000;
		dev_info(&pdev->dev, "MLA core clock: %u MHz (via clk)\n",
			 mdev->mla_mhz);

		/*
		 * Boot PLL of 600 MHz limits the OPPs to 100..600 MHz, a
		 * 1000 MHz boot PLL allows 100..1000 MHz. Must be set before
		 * the OPP table is parsed. mla_mhz is still the boot rate
		 * here: nothing has reprogrammed the PLL yet.
		 */
		supported_hw = (mdev->mla_mhz && mdev->mla_mhz <= 600) ?
				BIT(0) : BIT(1);
		ret = devm_pm_opp_set_supported_hw(&pdev->dev, &supported_hw, 1);
		if (ret) {
			dev_warn(&pdev->dev, "OPP supported-hw: %d\n", ret);
		} else {
			ret = devm_pm_opp_of_add_table(&pdev->dev);
			if (ret && ret != -ENODEV)
				dev_warn(&pdev->dev, "OPP table add: %d\n", ret);
		}

		if (!ret) {
			mla_devfreq_profile.initial_freq =
				clk_get_rate(mdev->mlapll);
			mdev->df = devm_devfreq_add_device(&pdev->dev,
				&mla_devfreq_profile, DEVFREQ_GOV_USERSPACE,
				NULL);
			if (IS_ERR(mdev->df)) {
				dev_warn(&pdev->dev, "devfreq add: %ld\n",
					 PTR_ERR(mdev->df));
				mdev->df = NULL;
			}
		}
	}

	ret = mla_request_irqs(mdev, pdev);
	if (ret)
		goto err_clk;

	/* Misc device — publishes /dev/mla */
	mdev->miscdev.minor  = MISC_DYNAMIC_MINOR;
	mdev->miscdev.name   = MLA_DRV_NAME;
	mdev->miscdev.fops   = &mla_fops;
	mdev->miscdev.parent = &pdev->dev;
	mdev->miscdev.mode   = 0660;

	ret = misc_register(&mdev->miscdev);
	if (ret) {
		ret = dev_err_probe(&pdev->dev, ret, "misc_register failed\n");
		goto err_clk;
	}

	/* Hardware bring-up. In OF-probe mode we are the first owner of
	 * the MLA window (no prior SDK-image bind has touched it), so
	 * explicitly reset + enable IRQs + clear stale source bits.
	 * Without this the very first JOB_SUBMIT after a fresh insmod
	 * times out, then recover() runs and subsequent jobs work;
	 * calling it preemptively skips that wasted first-job timeout.
	 */
	{
		int rret = mla_hw_init(mdev);

		if (rret == -ENODEV)
			dev_warn(mdev->dev,
				 "PRC region not mapped; reset skipped\n");
	}
	mla_hw_recover(mdev);    /* preemptive warmup; subsumes enable_irqs */
	mla_hw_set_dma_burst_size(mdev,
				  MLA_DEFAULT_RD_BURST,
				  MLA_DEFAULT_WR_BURST);

	/* TLC perf counters are opt-in via MLA_OP_PERFCTR_CONFIG, so start
	 * with all four slots disabled and report nothing until userspace
	 * selects signals. Disabling explicitly rather than leaving the
	 * registers alone is deliberate: a previous module load may have left
	 * slots enabled with its own selection, and inheriting that would
	 * report signals this load never agreed to.
	 */
	mla_hw_perfctr_disable_all(mdev);

	dev_info(&pdev->dev,
		 "MLA driver loaded: soc=%u regs=%pa size=0x%llx dma_sync=%d\n",
		 mdev->soc_family, &res->start, (u64)mdev->regs_size,
		 mla_do_dma_sync);

	/* Legacy /dev/simaai-mlamem. Optional: /dev/mla works without it. */
	ret = simaai_mlamem_init(mdev);
	if (ret)
		dev_warn(&pdev->dev, "simaai-mlamem not available (%d)\n", ret);

	/* Drop probe's usage count, with no /dev/mla fd open the device
	 * idles and the gate clock is released.
	 */
	pm_runtime_put(&pdev->dev);
	return 0;

	/* Unwind in reverse acquisition order, mirroring mla_remove(). Only
	 * the non-devm resources need it: mdev, regs and the IRQs are
	 * devres-tracked and released once we return an error. recovery_work
	 * is never queued on an error path — nothing dispatches before
	 * misc_register() succeeds — so no cancel_work_sync() is needed here.
	 *
	 * The debugfs unwind is needed so -EPROBE_DEFER from mla_heap_init()
	 * can be safely retried; otherwise the global debugfs parent retains a
	 * stale directory reference across retries.
	 */
err_clk:
	/* The usage count held since enable kept the device from suspending,
	 * so the gate is still prepared here.
	 */
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	if (mdev->mlaclk)
		clk_disable_unprepare(mdev->mlaclk);
err_unmap_prc:
	if (mdev->prc_regs) {
		iounmap(mdev->prc_regs);
		mdev->prc_regs = NULL;
	}
err_debugfs:
	/* Guarded: mla_heap_init()'s failure jumps here before registration. */
	if (mdev->oom_nb_registered) {
		unregister_oom_notifier(&mdev->oom_nb);
		mdev->oom_nb_registered = false;
	}
	if (mdev->reserved_mem_attached)
		of_reserved_mem_device_release(&pdev->dev);
	mla_debugfs_exit(mdev);
	mla_section_dedup_destroy(mdev);
	mla_sched_exit(mdev);
	return ret;
}

/**
 * mla_remove() — native DT remove handler
 * @pdev: platform device being removed
 *
 * Cancels any pending recovery work and deregisters the misc device.
 * devm-tracked allocations (mdev, regs, IRQs) are released by devres
 * after this returns. The carveout heap is a kernel-lifetime object;
 * nothing to tear down for it.
 */
static void mla_remove(struct platform_device *pdev)
{
	struct mla_device *mdev = platform_get_drvdata(pdev);

	/* First: the notifier dereferences mdev, so it must not outlive it. */
	if (mdev->oom_nb_registered) {
		unregister_oom_notifier(&mdev->oom_nb);
		mdev->oom_nb_registered = false;
	}
	simaai_mlamem_exit(mdev);
	if (cancel_work_sync(&mdev->recovery_work))
		pm_runtime_put_noidle(&pdev->dev);	/* the cancelled work's reference */
	mla_sched_exit(mdev);
	misc_deregister(&mdev->miscdev);
	mla_debugfs_exit(mdev);
	mla_section_dedup_destroy(mdev);
	/* Balance probe's gate prepare only if runtime suspend hasn't
	 * already dropped it (e.g. unbind while /dev/mla is still open).
	 */
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		clk_disable_unprepare(mdev->mlaclk);
	pm_runtime_set_suspended(&pdev->dev);
	if (mdev->prc_regs) {
		iounmap(mdev->prc_regs);
		mdev->prc_regs = NULL;
	}
	/* Normal unload is blocked while any context or exported section
	 * remains. Release CMA association only after all backing is freed.
	 */
	if (mdev->reserved_mem_attached)
		of_reserved_mem_device_release(&pdev->dev);
	dev_info(&pdev->dev, "MLA driver unloaded\n");
}

/* ======================== Driver-wide op tables ======================== *
 *
 * The /dev/mla file ops, the devfreq profile, the OF match table and the
 * platform driver itself are grouped here at the bottom, ahead of the
 * module parameters and module_init, so the file reads as
 * implementation-then-registration.
 */

/*
 * mla_ioctl() — the ioctl command surface for /dev/mla.
 *
 * The sole op transport. Uses ioctl (not io_uring) so Neat can run under
 * Docker's default seccomp profile, which blocks io_uring_* but allows ioctl:
 *   MLA_IOCTL_SUBMIT_SYNC  — one synchronous op of any kind (via
 *                            mla_cmd_prologue + mla_dispatch_cmd), returning
 *                            the op's errno directly.
 *   MLA_IOCTL_SUBMIT_ASYNC — enqueue N jobs; completions collected via REAP.
 *   MLA_IOCTL_REAP         — drain completed async jobs.
 * Submit paths apply the prologue's fatal gate; reap is always allowed so
 * completions can still be drained after the device goes fatal.
 */
static long mla_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mla_context *ctx = filp->private_data;
	void __user *uarg = (void __user *)arg;
	int ret;

	if (!ctx)
		return -EBADF;


	switch (cmd) {
	case MLA_IOCTL_SUBMIT_SYNC: {
		struct mla_ioctl_submit_sync s;
		struct mla_uring_cmd_sqe sqe;
		struct mla_cmd_hdr hdr;

		if (copy_from_user(&s, uarg, sizeof(s)))
			return -EFAULT;

		/* Build the descriptor the shared dispatch expects.
		 * mla_cmd_prologue re-checks s.reserved (as sqe.reserved) and
		 * applies the fatal gate; the op's own out_ptr (not this struct)
		 * receives any response, so there is nothing to copy back — just
		 * return the op's errno.
		 */
		sqe.req_user_ptr = s.req_user_ptr;
		sqe.req_len      = s.req_len;
		sqe.reserved     = s.reserved;

		ret = mla_cmd_prologue(ctx, &sqe, &hdr);
		if (ret)
			return ret;

		return mla_dispatch_cmd(ctx, &sqe, &hdr);
	}
	case MLA_IOCTL_SUBMIT_ASYNC: {
		struct mla_ioctl_submit s;

		if (READ_ONCE(ctx->mdev->device_fatal))
			return -EHOSTDOWN;
		if (READ_ONCE(ctx->fatal))
			return -EOWNERDEAD;
		if (copy_from_user(&s, uarg, sizeof(s)))
			return -EFAULT;
		if (s.reserved)
			return -EINVAL;
		ret = mla_ioctl_submit_async(ctx, &s);
		if (copy_to_user(uarg, &s, sizeof(s)))
			return -EFAULT;
		return ret;
	}
	case MLA_IOCTL_REAP: {
		struct mla_ioctl_reap r;

		if (copy_from_user(&r, uarg, sizeof(r)))
			return -EFAULT;
		if (r.reserved)
			return -EINVAL;
		ret = mla_ioctl_reap(ctx, &r);
		if (copy_to_user(uarg, &r, sizeof(r)))
			return -EFAULT;
		return ret;
	}
	default:
		return -ENOTTY;
	}
}

const struct file_operations mla_fops = {
	.owner          = THIS_MODULE,
	.open           = mla_open,
	.release        = mla_release,
	.unlocked_ioctl = mla_ioctl,
	/* Every ioctl arg struct is __u64/__u32 and naturally aligned, so the
	 * 32-bit layout is identical; compat_ptr_ioctl just widens the pointer.
	 */
	.compat_ioctl   = compat_ptr_ioctl,
};

/* devfreq target/get_cur_freq drive mlapll rate via clk_set_rate. The OPP
 * table comes from DT (operating-points-v2 = <&mla_opp_table>; entries at
 * 100..1000 MHz on Modalix, filtered by opp-supported-hw to 100..600 MHz
 * when the PLL boots at 600 MHz). .initial_freq is stamped by mla_probe()
 * from the PLL's rate at probe time.
 */
static struct devfreq_dev_profile mla_devfreq_profile = {
	.target       = mla_devfreq_target,
	.get_cur_freq = mla_devfreq_get_cur_freq,
	.polling_ms   = 0,
};

static int __maybe_unused mla_runtime_suspend(struct device *dev)
{
	struct mla_device *mdev = dev_get_drvdata(dev);

	clk_disable_unprepare(mdev->mlaclk);
	return 0;
}

static int __maybe_unused mla_runtime_resume(struct device *dev)
{
	struct mla_device *mdev = dev_get_drvdata(dev);

	return clk_prepare_enable(mdev->mlaclk);
}

static const struct dev_pm_ops mla_pm_ops = {
	SET_RUNTIME_PM_OPS(mla_runtime_suspend, mla_runtime_resume, NULL)
};

static const struct of_device_id mla_of_match[] = {
	{ .compatible = "simaai,mla-modalix", .data = (void *)MLA_SOC_MODALIX },
	/* The next-generation SoC family will append a new compatible
	 * string here when that driver support lands (uapi MLA_SOC_*
	 * value 2 is reserved).
	 */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mla_of_match);

static struct platform_driver mla_platform_driver = {
	.driver = {
		.name           = MLA_DRV_NAME,
		.of_match_table = mla_of_match,
		.pm             = &mla_pm_ops,
		/* This fixed platform device is not hot-removable. An exported
		 * DMA buffer may outlive /dev/mla: unbind must not free its DMA
		 * device state. Normal module unload is reference-counted.
		 */
		.suppress_bind_attrs = true,
	},
	.probe  = mla_probe,
	.remove = mla_remove,
};


/* ============================ Module params + init ============================ *
 *
 * Note: Only knobs that are used at runtime (either for diagnostics or as
 * kill switches) are here. Anything that must always be on for the driver
 * to work correctly (IRQs, DMA burst config, reset at probe) is baked in.
 *
 * Storage definitions sit immediately below each module_param so the
 * knob, its type and its documentation stay together.
 */

bool mla_do_dma_sync = true;
uint mla_desc_cma_enable = 1;
uint mla_reloc_range_flush = 1;
uint mla_debug;
bool mla_no_recover;
bool mla_reprog_always;
bool mla_skip_hw;
uint mla_inject_fault;
int mla_inject_prep_fail;
uint mla_inject_no_complete;
uint mla_inject_reloc_delay_ms;
uint mla_inject_prep_delay_us;
uint mla_reloc_quiesce_grace_ms = MLA_RELOC_QUIESCE_GRACE_MS;

/* Runtime-tunable via /sys/module/mla/parameters/.
 *
 * Gates only MODEL-side flush-region construction at MODEL_DEFINE_V3
 * (v2_build_flush_regions) — maintenance over memory the driver itself
 * writes. IFM/OFM cache maintenance is the caller's obligation
 * (DMA_BUF_IOCTL_SYNC on its own mmap; see the block comment in mla_hw.c).
 * Descriptor-blob and reloc maintenance in mla_pool.c is NOT gated by
 * this and must not become so.
 *
 * The name stays despite not matching IFM/OFM because board scripts and
 * SWMLA-v2-foreign-import read this exact sysfs path.
 */
module_param(mla_do_dma_sync, bool, 0644);
MODULE_PARM_DESC(mla_do_dma_sync,
		 "Build model flush regions at MODEL_DEFINE. Does NOT affect IFM/OFM (caller-owned). Default 1.");

/* Read-once at insmod. Flipping mid-life would require every
 * already-loaded model's descriptor-blob section to swap backing.
 */
module_param(mla_desc_cma_enable, uint, 0444);
MODULE_PARM_DESC(mla_desc_cma_enable,
		 "Use WB descriptor buffers with range-scoped flushes. Shared-heap mode uses the selected heap; otherwise dma_alloc_noncoherent. Zero uses heap exporter full-buffer flushes. MODEL_DEFINE hard-fails on -ENOMEM. Default 1.");

/* Read at MODEL_DEFINE; changing after models are loaded has no
 * effect until the next MODEL_DEFINE.
 */
module_param(mla_reloc_range_flush, uint, 0644);
MODULE_PARM_DESC(mla_reloc_range_flush,
		 "Precompute range-scoped flush regions at MODEL_DEFINE so per-job krlc flushes only dirtied cache lines (~2 µs vs ~23 µs full-buffer). Kill switch: 0 falls back to full-buffer flush. Default 1.");

module_param(mla_debug, uint, 0644);
MODULE_PARM_DESC(mla_debug,
		 "Debug verbosity bitmask. Bit 0 (0x1, MLA_DBG_TRACE): one-shot per-event traces (MODEL_DEFINE, JOB_SUBMIT header, PROGDONE, enqueue, burst dance, prep-ahead). Bit 1 (0x2, MLA_DBG_HOT): per-item inner-loop traces (per-tile, per-DFM, per-reloc, per-iotensor, per-section) — heavy (~15ms/line printk × 2000+ lines/job on cache_token2047), can trip the 2s JOB_SUBMIT timeout; HOT is useful only combined with TRACE (0x3). Bits 0x4..0x80 reserved. Runtime-toggleable via /sys/module/mla/parameters/mla_debug.");

/* Post-mortem: freeze state on job fault. Skips mla_sched_fault_context
 * (ctx not fatalized → next submits proceed instead of returning
 * -EOWNERDEAD) and the recovery workqueue. Fence still signals with
 * the error result.
 */
module_param(mla_no_recover, bool, 0644);
MODULE_PARM_DESC(mla_no_recover,
		 "On job fault, skip ctx fatalize and HW recovery; return errno to userspace with device state preserved for debugging. Default 0.");

/* Perf measurement: bypasses last_programmed_model cache. Forces the
 * full ~30µs of MMIO writes on every JOB_SUBMIT.
 */
module_param(mla_reprog_always, bool, 0644);
MODULE_PARM_DESC(mla_reprog_always,
		 "Always reprogram DFM/tile/QG0 on every JOB_SUBMIT. Perf measurement only. Default 0.");

/* Sched/fence isolation: run everything but the GO write, fake PROGDONE.
 * Lets sched priority / fence-ref accounting be validated without HW.
 */
module_param(mla_skip_hw, bool, 0644);
MODULE_PARM_DESC(mla_skip_hw,
		 "Run reloc/dma-sync/DFM/tile programming but skip GO write and fake PROGDONE. HW isolation for debug. Default 0.");

/* Move the completed job's wakeup (sync completion) to AFTER the next job
 * has been dispatched and given GO.
 *
 * mla_sched_complete runs inline in hard IRQ, so everything it does before
 * mla_sched_dispatch_next() is dead silicon time: the device sits idle from
 * PROGDONE until the next GO write. Waking the completed job's owner is the
 * one step in that prefix whose result nothing in the dispatch path consumes,
 * so it can be deferred without reordering anything the next job depends on.
 *
 * Only ever deferred on a CLEAN completion (result == 0). On a fault or
 * timeout the caller may fatalize the context and drain the queue, so the
 * waiter must be woken with the error before any of that runs.
 *
 * The slot release, watchdog cancel, fence signal and bank release all stay
 * BEFORE dispatch: the next job cannot be picked while in_flight is set,
 * cannot run without a FREE bank, and a watchdog firing in the gap would
 * double-finalize a job whose fence was not yet signalled.
 *
 * Default 0 — opt-in until it has a clean correctness gate on hardware.
 */
bool mla_defer_notify;
module_param(mla_defer_notify, bool, 0644);
MODULE_PARM_DESC(mla_defer_notify,
		 "Defer the completed job's wakeup until after the next dispatch reaches GO. Cuts inter-job device idle. Default 0.");

module_param(mla_inject_fault, uint, 0644);
MODULE_PARM_DESC(mla_inject_fault,
		 "Fault injection: corrupt the first descriptor qword before GO for the next N dispatches (triggers FTL/quadgroup-timeout). Write N>0 to arm; decrements on each dispatch. Default 0.");

/* Distinct from mla_inject_fault, which faults AFTER GO. This one makes
 * mla_hw_program_and_start return before GO and before the watchdog is
 * armed — the only window in which a dropped dispatch error can strand a
 * job in mdev->in_flight with no completion possible.
 */
module_param(mla_inject_prep_fail, int, 0644);
MODULE_PARM_DESC(mla_inject_prep_fail,
		 "Fault injection: fail job prep before the GO write. SITE-SELECTABLE, because the two prep callers handle failure differently. N>0 fails the next N *dispatch-site* preps (exercises the pre-GO error return, which must finalize the job rather than wedge the queue). N<0 fails the next |N| *async prep-worker* preps (exercises the worker declining to publish the bank READY; count via counters/prep_failures). Magnitude decrements toward 0 on each injection. Default 0.");

module_param(mla_inject_no_complete, uint, 0644);
MODULE_PARM_DESC(mla_inject_no_complete,
		 "Test hook: withhold the GO write for the next N dispatches so nothing ever completes the job and its watchdog expires for real. Write N>0 to arm; decrements on each dispatch. counters/inject_no_completes counts them. Default 0.");

/* Production policy, not a test hook: exposed because the right value
 * depends on preemption behaviour. Also lets SWMLA-v2-lora-reloc pin the
 * grace period to widen an otherwise microseconds-long window, so the
 * submit-side half of the reloc quiescence gate can be tested
 * deterministically instead of by racing. 0 restores pure fail-fast
 * (-EBUSY on any outstanding job).
 */
module_param(mla_reloc_quiesce_grace_ms, uint, 0644);
MODULE_PARM_DESC(mla_reloc_quiesce_grace_ms,
		 "Milliseconds MLA_OP_MODEL_UPDATE_RELOC waits for a model to go quiescent before returning -EBUSY. Absorbs the skew between dma_fence_signal() and the outstanding decrement, so a caller that waited on its fence fd is not refused spuriously. 0 = fail fast. counters/reloc_update_waits and reloc_update_wait_timeouts report the two outcomes. Default 2.");

module_param(mla_inject_reloc_delay_ms, uint, 0644);
MODULE_PARM_DESC(mla_inject_reloc_delay_ms,
		 "Test hook: sleep this many ms inside MLA_OP_MODEL_UPDATE_RELOC while reloc_update_in_progress is set, so a concurrent submit reliably lands in the window and is refused with -EBUSY (counters/reloc_submits_refused). STICKY, not a countdown -- it applies to every update until set back to 0. Default 0.");

module_param(mla_inject_prep_delay_us, uint, 0644);
MODULE_PARM_DESC(mla_inject_prep_delay_us,
		 "Test hook: stall the async prep worker this many MICROSECONDS on its first dynamic reloc, holding its bank in MLA_BANK_PREPARING with the descriptor set unwritten. Reproduces the bank-reuse window; a value near one job's tile latency is the useful range, and much larger values stop the race occurring at all because the spare banks stay pinned and every job takes the inline FREE claim. Applies to every second worker so a peer still publishes READY. Distinct from mla_inject_reloc_delay_ms, which delays MLA_OP_MODEL_UPDATE_RELOC. STICKY, not a countdown -- applies until set back to 0. Default 0.");

static int __init mla_module_init(void)
{
	return platform_driver_register(&mla_platform_driver);
}
module_init(mla_module_init);

static void __exit mla_module_exit(void)
{
	platform_driver_unregister(&mla_platform_driver);
	/* Dedup entry frees may still be waiting for an RCU grace period. */
	rcu_barrier();
}
module_exit(mla_module_exit);

MODULE_AUTHOR("SiMa.ai");
MODULE_DESCRIPTION("SiMa MLA accelerator driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");
MODULE_IMPORT_NS("DMA_BUF");
MODULE_IMPORT_NS("DMA_BUF_HEAP");
