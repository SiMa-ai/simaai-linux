// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// mla_job lifecycle + dma-fence ops + watchdog timer.
//
// Refcount model: a freshly-allocated job has refcount 1 (owned by the
// xarray entry). The dma-fence inside the job carries a separate
// reference (see dma_fence_get(&job->fence) in mla_op_job_submit). The
// job is freed when both go away — i.e. when the xarray entry is
// erased AND the last fence reference is dropped (sync_file fd closed,
// kernel-side waiters released).

#include <linux/dma-fence.h>
#include <linux/jiffies.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include "mla_drv.h"
#include "mla_plist.h"
#include "mla_regs.h"

/* ---------------------------------------------------------------------- *
 *  dma-fence ops
 * ----------------------------------------------------------------------
 */

/* dma-fence ops vtable. Driver name + timeline appear in the
 * /sys/kernel/debug/sync trace and in dma-fence error messages.
 */
static const char *mla_fence_get_driver_name(struct dma_fence *fence)
{
	return "mla";
}

static const char *mla_fence_get_timeline_name(struct dma_fence *fence)
{
	return "mla.queue0";
}

/**
 * mla_job_put_bindings() — release the per-tensor buffer references
 * @job: job whose IFM/OFM bindings should be released
 *
 * Drops the reference mla_resolve_iotensors() took for each bound tensor.
 * Uses mla_buffer_drop_async() because this runs from mla_fence_release(),
 * which can be reached from softirq — mla_sched_complete() via the IRQ
 * handler, and mla_job_timeout_fn() below — while the last-reference
 * teardown may sleep.
 *
 * Safe to call more than once, and safe at any point in JOB_SUBMIT's error
 * unwind: each entry is NULLed as it is dropped, the array pointers are
 * NULL once the unwind has freed them, and mla_resolve_iotensors guarantees
 * an all-NULL array when it fails. Without that, the unwind's kfree+NULL of
 * ifm_bufs/ofm_bufs happens BEFORE the fence put, so every reference taken
 * on an error path would leak (delta > 0).
 */
void mla_job_put_bindings(struct mla_job *job)
{
	u32 i;

	if (job->ifm_bufs) {
		for (i = 0; i < job->num_ifm; i++) {
			if (job->ifm_bufs[i]) {
				mla_buffer_drop_async(job->ifm_bufs[i]);
				job->ifm_bufs[i] = NULL;
			}
		}
	}
	if (job->ofm_bufs) {
		for (i = 0; i < job->num_ofm; i++) {
			if (job->ofm_bufs[i]) {
				mla_buffer_drop_async(job->ofm_bufs[i]);
				job->ofm_bufs[i] = NULL;
			}
		}
	}
}

/*
 * Final destructor for an mla_job: release the per-tensor buffer
 * references, free the IFM/OFM binding arrays, and free the job itself.
 * Called by dma_fence subsystem when the last reference drops — which
 * happens after both the xarray entry has been erased and any sync_file fd
 * holding the fence is closed.
 */
static void mla_fence_release(struct dma_fence *fence)
{
	struct mla_job *job = container_of(fence, struct mla_job, fence);

	/* Buffer references were taken at submit-time resolve; drop them here,
	 * when the fence — which always outlives the job's queue/run state —
	 * is released. This is the point that makes the reference correct: it
	 * is strictly after the last mla_hw read of b->dma_addr.
	 *
	 * MUST come before the kfree below: put_bindings reads through
	 * ifm_bufs and ofm_bufs.
	 */
	mla_job_put_bindings(job);

	/* Drop the model reference taken under xa_lock at submit time. Same
	 * reasoning as the bindings above: this destructor is the one place that
	 * runs on every path that frees a job, so it is the only drop point that
	 * cannot leak — including the err_* unwinds and jobs that never reached
	 * hardware.
	 *
	 * MUST be the _async variant: this can run in softirq (via
	 * mla_sched_complete / mla_job_timeout_fn) and mla_model_destroy()
	 * blocks — mla_buffer_drop(), kvfree() on a possibly-vmalloc'd
	 * reloc_list, and a final mla_context_put(). The deferred teardown goes
	 * to mdev->buf_release_wq, exactly like mla_buffer_drop_async().
	 *
	 * NULLed after the put so this is idempotent, matching put_bindings.
	 */
	if (job->model) {
		mla_model_put_async(job->model);
		job->model = NULL;
	}

	/* Backstop: never free a job with its watchdog still armed.
	 *
	 * This is the destructor, so it is the one place guaranteed to run on
	 * EVERY path that frees a job — including paths that never reached a
	 * qualifying mla_sched_finish_job (the sync waiter's belt, and every
	 * err_* unwind that drops the last fence ref). A live timer on freed
	 * memory makes the next call_timer_fn branch into a recycled object.
	 *
	 * Free when the timer was never armed or is already gone, and safe in
	 * softirq: timer_delete_sync_try never waits. It also returns -1
	 * rather than deadlocking if we somehow got here from inside
	 * mla_job_timeout_fn.
	 *
	 * The primary disarm stays in mla_sched_finish_job — this is defence
	 * in depth, not a replacement for finalizing properly.
	 */
	mla_job_cancel_timeout(job);

	/* Backstop for the run-queue invariant: a job must never be freed while
	 * its sched_node is still linked on mdev->sched_queue.
	 *
	 * plist membership is a weak reference — mla_sched_enqueue() links
	 * without taking a fence ref — so a job freed while linked leaves the
	 * freed object reachable from the queue, and the next walk in
	 * mla_sched_pick_reserve_locked() or
	 * mla_sched_find_prep_candidate_locked() reads through it. That is a
	 * silent UAF: it only turns into an oops once the allocator hands the
	 * page to something else, so it needs an assertion at the free rather
	 * than a crash somewhere downstream.
	 *
	 * mla_sched_finish_job() is the primary unlink and covers every
	 * finalizing path; this is the same defence-in-depth shape as the
	 * watchdog disarm above. Unlink here too rather than only warning —
	 * leaving the freed node linked would be the actual memory-corruption
	 * bug, and a WARN that then crashes anyway is no help.
	 */
	{
		struct mla_device *mdev = job->ctx->mdev;
		unsigned long sflags;
		bool still_linked;

		spin_lock_irqsave(&mdev->sched_lock, sflags);
		still_linked = !plist_node_empty(&job->sched_node);
		if (still_linked)
			mla_plist_del(&job->sched_node, &mdev->sched_queue);
		spin_unlock_irqrestore(&mdev->sched_lock, sflags);

		WARN_ONCE(still_linked,
			  "mla: job %u freed while still linked on sched_queue (state=%d sync_wait=%d result=%d)\n",
			  job->id, job->state, job->sync_wait, job->result);
	}

	kfree(job->ifm_bufs);
	kfree(job->ofm_bufs);
	kfree(job->ifm_offsets);
	kfree(job->ofm_offsets);

	{
		struct mla_context *ctx = job->ctx;
		struct mla_device *mdev = ctx->mdev;

		/* Death half of the job-lifetime invariant — see job_inc_ops in
		 * mla_drv.h. Counted in the destructor rather than at any
		 * terminal/completion point because this is the only place that
		 * runs on every path that frees a job: the err_* unwinds, the
		 * sync waiter's belt expiry, and jobs that never reached
		 * hardware. mdev is read before the context put below, since the
		 * device outlives the context but ctx does not outlive its last
		 * reference.
		 */
		atomic64_inc(&mdev->job_dec_ops);

		job->ctx = NULL;
		kfree(job);
		/* Release the context reference taken at job creation. A job
		 * routinely outlives the fd — its fence keeps it alive past
		 * mla_context_shutdown — and mla_hw.c dereferences
		 * job->ctx->mdev from the dispatch/completion path, so the
		 * context must stay alive until exactly here.
		 *
		 * NOTE this can run in softirq/IRQ-adjacent context, which is
		 * why mla_context_put's free path must not sleep.
		 */
		mla_context_put(ctx);
	}
}

static const struct dma_fence_ops mla_fence_ops = {
	.get_driver_name   = mla_fence_get_driver_name,
	.get_timeline_name = mla_fence_get_timeline_name,
	.release           = mla_fence_release,
};

/* ---------------------------------------------------------------------- *
 *  Refcount helpers (delegate to the embedded fence)
 * ----------------------------------------------------------------------
 */

/**
 * mla_job_get() — take a reference to a job
 * @job: job to retain
 *
 * Delegates to dma_fence_get on the embedded fence — that's the
 * single refcount the job follows.
 */
void mla_job_get(struct mla_job *job)
{
	dma_fence_get(&job->fence);
}

/**
 * mla_job_put() — drop a reference to a job
 * @job: job to release
 *
 * On the last drop, mla_fence_release() runs and frees the job.
 */
void mla_job_put(struct mla_job *job)
{
	dma_fence_put(&job->fence);
}

/* ---------------------------------------------------------------------- *
 *  Watchdog
 *
 *  Fires from soft-IRQ (timer) context.
 *
 *  CRITICAL: mla_job_cancel_timeout uses try_to_del_timer_sync() instead
 *  of del_timer_sync(). The completion path (mla_sched_complete) calls
 *  cancel_timeout, and this can be invoked from inside the timer
 *  callback itself (when a timeout fires and the timer fn calls
 *  mla_sched_complete → cancel_timeout). del_timer_sync() in that
 *  recursion deadlocks: it waits for the timer to finish, but the
 *  timer IS running this code. try_to_del_timer_sync returns -1 if
 *  the timer is currently running (i.e. from inside its own
 *  callback) which is exactly what we want to be a no-op.
 * ----------------------------------------------------------------------
 */

/*
 * Watchdog callback: fired in soft-IRQ context when a job exceeds
 * its timeout. Logs the event and completes the job with -ETIMEDOUT,
 * which triggers the scheduler-side fault path (fatalizes the
 * offending context and schedules recovery).
 */
static void mla_job_timeout_fn(struct timer_list *t)
{
	struct mla_job *job = timer_container_of(job, t, timeout);
	struct mla_device *mdev = job->ctx->mdev;
	struct mla_job *waiter;
	unsigned long flags;
	unsigned int n_queued = 0;
	int pm;

	dev_warn(mdev->dev, "job %u timed out after %u ms\n",
		 job->id, job->timeout_ms);

	/* Dump the outstanding scheduler queue (jobs that never got to
	 * dispatch). Runs in soft-IRQ context so spin_lock_irqsave is
	 * required.
	 */
	spin_lock_irqsave(&mdev->sched_lock, flags);
	dev_warn(mdev->dev,
		"timeout: in_flight=%u(ctx=%p prio=(%d,%d)) prog_done_count=%u\n",
		job->id, job->ctx,
		job->ctx->group_prio, job->ctx->intra_group_prio,
		job->prog_done_count);
	plist_for_each_entry(waiter, &mdev->sched_queue, sched_node) {
		dev_warn(mdev->dev,
			"timeout: queued[%u] job %u ctx=%p prio=(%d,%d) state=%d\n",
			n_queued, waiter->id, waiter->ctx,
			waiter->ctx->group_prio,
			waiter->ctx->intra_group_prio,
			waiter->state);
		n_queued++;
	}
	dev_warn(mdev->dev, "timeout: total_queued=%u\n", n_queued);
	spin_unlock_irqrestore(&mdev->sched_lock, flags);

	/* The progdone IRQ can retire this job while this callback runs, and
	 * its put may drop the last runtime-PM reference. Only touch registers
	 * while the block is still clocked. -EINVAL means runtime PM is not
	 * managing the gate, so read as before.
	 */
	pm = pm_runtime_get_if_active(mdev->dev);
	if (!pm)
		goto complete;

	/* Snapshot the MLA register state BEFORE mla_sched_complete may
	 * fatalize the ctx and schedule recovery (which does a reset that
	 * wipes the diagnostic state we want to see). Runs in soft-IRQ
	 * context — read-only MMIO, cheap.
	 */
	mla_hw_dump_status(mdev, "job-timeout");

	/* Read against the TLC_NF_src value dump_status just printed: a
	 * latched quadgrp_prog_done (bits 13..16) while irqs_armed is 1 means
	 * the job COMPLETED in hardware and its interrupt was never delivered,
	 * which is a different failure from a stalled pipeline.
	 *
	 * Deliberately a software read and NOT an ioread32 of the EN masks:
	 * mla_hw_dump_status() is also called from mla_irq_ftl() in HARD IRQ
	 * during an active fault, where extra MMIO and extra printk are both
	 * hazardous, so the EN registers are not read on any fault path.
	 */
	dev_warn(mdev->dev, "timeout: irqs_armed=%d\n",
		 READ_ONCE(mdev->irqs_armed));

	/* Stash fault info for userspace resp. No FTL IRQ fired (watchdog
	 * expired first), so read the aggregator snapshot for any latched
	 * FTL bits that the IRQ handler didn't catch.
	 */
	job->fault_class   = MLA_FAULT_TIMEOUT;
	job->fault_sys_sts = ioread32(mdev->regs + MLA_REG_SYS_FTL_INTR_STS);
	{
		int i;

		for (i = 0; i < 4; i++)
			job->fault_l2g_sts[i] = ioread32(mdev->regs +
				MLA_REG_L2G0_FTL_INTR_SRC0 +
				i * MLA_REG_L2G_STRIDE);
	}

complete:
	mla_sched_complete(mdev, job, -ETIMEDOUT);
	if (pm > 0)
		pm_runtime_put(mdev->dev);

	/* Drop the reference the armed timer held (mla_job_arm_timeout).
	 * @job may be freed here — do not touch it again.
	 */
	dma_fence_put(&job->fence);
}

/**
 * mla_job_arm_timeout() — start the watchdog for an in-flight job
 * @job: job whose timeout to arm
 *
 * Uses the per-job timeout_ms (defaulting to MLA_JOB_DEFAULT_TIMEOUT_MS,
 * capped at MLA_JOB_MAX_TIMEOUT_MS) to schedule mla_job_timeout_fn
 * via mod_timer. Called from mla_hw_program_and_start just after GO
 * is written.
 */
void mla_job_arm_timeout(struct mla_job *job)
{
	u32 ms = job->timeout_ms ? job->timeout_ms : MLA_JOB_DEFAULT_TIMEOUT_MS;

	if (ms > MLA_JOB_MAX_TIMEOUT_MS)
		ms = MLA_JOB_MAX_TIMEOUT_MS;

	/* An armed timer owns a job reference, so the job cannot be freed while
	 * the timer is pending or its callback is running.
	 *
	 * Take it BEFORE mod_timer. Once mod_timer returns, the timer is pending
	 * and visible to every other CPU, so a concurrent canceller sees it
	 * pending, returns 1, and drops a reference that has not been taken yet
	 * — freeing the job under us, or leaving one nobody will ever drop.
	 * mla_sched_cancel_ctx_jobs is exactly such a canceller: op 18 and fd
	 * teardown both run it against jobs that are already RUNNING, so this is
	 * not hypothetical. Getting first and dropping the surplus below makes
	 * "pending implies referenced" true at every instant.
	 */
	dma_fence_get(&job->fence);
	if (mod_timer(&job->timeout, jiffies + msecs_to_jiffies(ms)))
		dma_fence_put(&job->fence);   /* already pending: earlier arm's ref stands */
}

/**
 * mla_job_cancel_timeout() — cancel an armed watchdog (deadlock-safe)
 * @job: job whose watchdog to cancel
 *
 * Uses try_to_del_timer_sync() rather than del_timer_sync() because
 * mla_sched_complete (which calls this) can itself be reached from
 * inside mla_job_timeout_fn — the recursion would deadlock under
 * del_timer_sync, which waits for the timer to finish while the
 * timer callback is running this code.
 *
 * The return value says who owns the reference the armed timer took:
 * 1 means we removed a still-pending timer and inherit its reference,
 * while 0 (not pending) and -1 (callback running — on this CPU or on
 * any other) mean the callback owns it and will drop it itself.
 */
void mla_job_cancel_timeout(struct mla_job *job)
{
	/* NEVER waits — unlike del_timer_sync, which would deadlock when
	 * the callback is what got us here.
	 */
	if (timer_delete_sync_try(&job->timeout) == 1)
		dma_fence_put(&job->fence);
}

/**
 * mla_job_cancel_timeout_sync() — cancel the watchdog and WAIT for it
 * @job: job whose watchdog to quiesce
 *
 * Same reference rule as mla_job_cancel_timeout — a removed pending timer
 * hands us the reference it held — but this form waits for a callback that is
 * already running instead of reporting -1 and returning.
 *
 * Wrong tool for every other caller: mla_sched_complete can be reached from
 * inside mla_job_timeout_fn, where waiting would deadlock the callback on
 * itself. Only call this from a context that is definitely not the callback,
 * in process context, holding no locks.
 *
 * Exists for the sync JOB_SUBMIT waiter — see the call site in
 * mla_op_job_submit for why it has to wait rather than cancel.
 */
void mla_job_cancel_timeout_sync(struct mla_job *job)
{
	if (timer_delete_sync(&job->timeout) == 1)
		dma_fence_put(&job->fence);
}

/* ---------------------------------------------------------------------- *
 *  Initialization helper used by mla_op_job_submit
 * ----------------------------------------------------------------------
 */

/**
 * mla_job_init_internal() — finish-init an mla_job after kzalloc + caller fields
 * @job:  partially-populated job (caller has set ctx, model, num_*, etc.)
 * @mdev: device to associate the embedded dma-fence with
 *
 * Initializes the spinlock the dma-fence ops require, the dma-fence
 * itself (with a per-device monotonic seqno), the watchdog timer,
 * sched_node plist node, sync-mode completion, and sets state to
 * QUEUED. After this returns, the job is ready for mla_sched_enqueue.
 * mla_sched_enqueue re-initializes the plist node with the resolved
 * priority key — this init is just so plist_node_empty() is well
 * defined before insert and after dequeue.
 */
void mla_job_init_internal(struct mla_job *job, struct mla_device *mdev)
{
	spin_lock_init(&job->fence_lock);
	dma_fence_init(&job->fence, &mla_fence_ops, &job->fence_lock,
		       mdev->fence_context,
		       atomic64_inc_return(&mdev->fence_seqno));
	timer_setup(&job->timeout, mla_job_timeout_fn, 0);
	plist_node_init(&job->sched_node, 0);
	init_completion(&job->sync_done);
	job->state = MLA_JOB_QUEUED;

	/* The prep_work function pointer must be initialized before it can
	 * be queue_work'd from mla_sched_try_prep_ahead().
	 */
	INIT_WORK(&job->prep_work, mla_hw_prep_work_fn);

	/* No bank picked yet. The walker (via prep-ahead) or dispatch (via
	 * inline prep) assigns a bank before apply_relocs runs.
	 */
	job->bank_idx = MLA_BANK_UNASSIGNED;
	job->prep_bank_idx = MLA_BANK_UNASSIGNED;

	/* Birth half of the job-lifetime invariant. One site covers both
	 * submit paths, since sync JOB_SUBMIT and JOB_ASYNC both route
	 * through here. Balanced in mla_fence_release.
	 */
	atomic64_inc(&mdev->job_inc_ops);
}
