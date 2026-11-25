// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// Scheduler: strict-priority FIFO, single in-flight job.
//
// Today's silicon serializes work — the hardware runs exactly one
// quadgroup-0 program at a time. The scheduler's job is to pick the
// next queued job by (group_prio, intra_group_prio) and start it as
// soon as the previous one completes.
//
// Run queue is a Linux plist (priority-sorted doubly-linked list).
// plist is O(K) insert (K = distinct priorities currently present),
// O(1) at the head, preserves FIFO among equal-priority nodes — all
// the invariants this scheduler relies on. The two-dimensional context
// priority (group_prio, intra_group_prio) is flattened to a single int
// in mla_sched_prio_key(); group occupies the high bits so it strictly
// outranks intra-group.
//
//
// Locking: sched_lock (spinlock_t in struct mla_device) protects
// sched_queue and in_flight. dma_fence_signal() is called outside the
// lock to avoid holding it across a potentially long callback chain
// from waiters.

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/plist.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include "mla_drv.h"
#include "mla_plist.h"
#include "mla_regs.h"

/**
 * mla_recovery_work_fn() — workqueue handler for fault recovery
 * @work: pointer to mdev->recovery_work
 *
 * Calls mla_hw_recover() to quiesce + (optionally) reset + re-enable
 * the MLA after a fault. On failure marks the device fatal so all
 * subsequent submissions return -EHOSTDOWN. The pending flag is
 * cleared on entry so a subsequent fault during recovery causes the
 * worker to be re-scheduled (mla_sched_complete will atomic_xchg
 * the flag back to 1 and schedule again).
 */
static void mla_recovery_work_fn(struct work_struct *work)
{
	struct mla_device *mdev = container_of(work, struct mla_device,
					       recovery_work);
	int ret;

	atomic_set(&mdev->recovery_pending, 0);

	/* Runs on the PM reference mla_sched_finish_job() queued with this
	 * work, so the block stays clocked from the fault until here.
	 */
	if (!READ_ONCE(mdev->device_fatal)) {
		ret = mla_hw_recover(mdev);
		if (ret) {
			dev_err(mdev->dev, "recovery failed (%d) — device fatal\n", ret);
			WRITE_ONCE(mdev->device_fatal, true);
		}
	}

	pm_runtime_put(mdev->dev);
}

/**
 * mla_sched_init() — one-shot scheduler + recovery state initialization
 * @mdev: device to initialize
 *
 * Sets up the run queue, the dma-fence context for this device, and
 * the recovery workqueue + rate-limit budget. Called from probe and
 * from the debug/takeover self-registration paths so all three end up
 * with the same initialized state.
 */
void mla_sched_init(struct mla_device *mdev)
{
	spin_lock_init(&mdev->sched_lock);
	plist_head_init(&mdev->sched_queue);
	mdev->in_flight = NULL;
	atomic64_set(&mdev->fence_seqno, 0);
	mdev->fence_context = dma_fence_context_alloc(1);

	INIT_WORK(&mdev->recovery_work, mla_recovery_work_fn);
	atomic_set(&mdev->recovery_pending, 0);
	mdev->device_fatal           = false;
	spin_lock_init(&mdev->recovery_lock);
	mdev->recovery_count         = 0;
	mdev->recovery_window_start  = jiffies;

	/* Prep-ahead workqueue. WQ_UNBOUND spreads across cores without
	 * pinning, WQ_HIGHPRI so prep isn't starved by other kernel work.
	 * max_active caps concurrent workers to the number
	 * of non-ACTIVE banks (MLA_MAX_BANKS - 1) — the ACTIVE bank is
	 * always in-flight and can't be prepped. This matches the bank
	 * pool's structural bound; prep_in_flight below enforces the
	 * same cap atomically.
	 */
	atomic_set(&mdev->prep_in_flight, 0);
	mdev->prep_wq = alloc_workqueue("mla-prep",
					WQ_UNBOUND | WQ_HIGHPRI,
					MLA_MAX_BANKS - 1);
	/* NULL prep_wq disables prep-ahead but doesn't fail probe — the
	 * inline dispatch path in mla_hw_program_and_start still works.
	 */

	/* Deferred buffer teardown (mla_buffer_drop_async). Ordered so two
	 * teardowns can't interleave, and not HIGHPRI — this is cleanup, it
	 * must not compete with prep or dispatch. A NULL queue is survivable:
	 * mla_buffer_drop_async falls back to inline teardown with a WARN.
	 */
	mdev->buf_release_wq = alloc_ordered_workqueue("mla-bufrel", 0);
}

/**
 * mla_sched_exit() — release prep_wq and buf_release_wq (recovery_work is
 * handled by mla_remove).
 *
 * destroy_workqueue() drains pending items, which matters for
 * buf_release_wq: a queued mla_buffer_release_work_fn holds a function
 * pointer into this module and a counted reference on an mla_context, so
 * both must be gone before the module can unload.
 */
void mla_sched_exit(struct mla_device *mdev)
{
	if (mdev->prep_wq) {
		destroy_workqueue(mdev->prep_wq);
		mdev->prep_wq = NULL;
	}
	if (mdev->buf_release_wq) {
		destroy_workqueue(mdev->buf_release_wq);
		mdev->buf_release_wq = NULL;
	}
}

/*
 * Flatten (group_prio, intra_group_prio) into a single plist key.
 * Lower (more negative) wins, matching task_nice() conventions.
 *
 * MLA_PRIO_GROUP_SHIFT lives in mla_drv.h so the debugfs per-ctx dump
 * can report the same key without duplicating the literal; see the
 * comment there for why group strictly outranks intra.
 */
static inline int mla_sched_prio_key(const struct mla_context *ctx)
{
	return ctx->group_prio * MLA_PRIO_GROUP_SHIFT + ctx->intra_group_prio;
}

/*
 * Insert @job into the run queue maintaining priority order. plist
 * walks at most K entries (K = distinct priorities present) and
 * preserves FIFO among equal-priority nodes — the invariants this
 * scheduler needs. Caller holds sched_lock.
 */
static void mla_sched_insert_locked(struct mla_device *mdev,
				    struct mla_job *job)
{
	plist_node_init(&job->sched_node, mla_sched_prio_key(job->ctx));
	mla_plist_add(&job->sched_node, &mdev->sched_queue);
}

/*
 * Pick the next runnable job under sched_lock and reserve the
 * in_flight slot. Returns the picked job (state==RUNNING, sched_node
 * detached) or NULL if the queue is empty or a job is already running.
 *
 * Hardware programming and the burst-size dance happen OUTSIDE the
 * lock — they may sleep (udelay in mla_hw_reset) and they touch
 * device registers extensively.
 */
static struct mla_job *mla_sched_pick_reserve_locked(struct mla_device *mdev)
{
	struct mla_job *next;

	/* The authoritative device_fatal gate, and the reason it belongs HERE
	 * rather than at each caller: this is the only place in_flight is
	 * reserved, so a test under sched_lock is the only one a reset cannot
	 * race past. Without it, a submitter could reserve the slot and write
	 * GO while mla_sched_device_reset() is mid PRC-reset — Modalix has no
	 * abort bit and GO is write-only, so the quadgroup keeps mastering
	 * and the resulting MMIO abort lands with sched_lock held and IRQs
	 * off, leaking the lock.
	 *
	 * Nothing is stranded by refusing here. mla_sched_device_reset() clears
	 * device_fatal and then calls mla_sched_dispatch_next() for exactly
	 * this case — a job that raced the cancel walk.
	 */
	if (READ_ONCE(mdev->device_fatal)) {
		atomic_inc(&mdev->dispatch_refused_fatal);
		return NULL;
	}

	if (mdev->in_flight)
		return NULL;
	if (plist_head_empty(&mdev->sched_queue))
		return NULL;

	next = plist_first_entry(&mdev->sched_queue, struct mla_job, sched_node);

	/* No bank-state gate here on purpose. Holding the head back while its
	 * prep worker finishes idles the device for the rest of that worker's
	 * loop and makes dispatch latency a function of workqueue scheduling
	 * delay, which costs measurable throughput on a CPU-saturated host.
	 * Dispatch takes the bank from the worker instead; mla_desc_bank.writers
	 * is what keeps that safe.
	 */
	mla_plist_del(&next->sched_node, &mdev->sched_queue);
	next->state = MLA_JOB_RUNNING;
	next->prog_done_count = 0;
	mdev->in_flight = next;
	/* in_flight owns a reference for the whole running lifetime. Without it
	 * the job has only its registry (init) and ioctl-reap refs between here
	 * and mla_job_arm_timeout(), and a concurrent fd close drops both while
	 * mla_sched_dispatch() is still programming the hardware — freeing the
	 * job mid-dispatch. Dropped in mla_sched_finish_job() when it clears
	 * in_flight. (io_uring could not hit this: a pending command pinned the
	 * file, so close could not run while an async job was outstanding.)
	 */
	dma_fence_get(&next->fence);
	return next;
}

/*
 * Bounded walk of the plist head looking for a prep-ahead candidate.
 * Skips entries whose model matches in_flight (§3.4 S1) or whose ctx
 * is fatalized (S3). Returns the first eligible job or NULL. Bounded
 * to keep sched_lock hold time small. Caller must hold sched_lock.
 * Caller must have in_flight != NULL (otherwise the head is
 * dispatch-ready, not prep-ahead-ready).
 */
#define MLA_PREP_WALK_LIMIT 4

static struct mla_job *
mla_sched_find_prep_candidate_locked(struct mla_device *mdev)
{
	struct mla_job *iter;
	unsigned int walked = 0;

	if (!mdev->in_flight)
		return NULL;

	plist_for_each_entry(iter, &mdev->sched_queue, sched_node) {
		u32 k, n;

		if (walked++ >= MLA_PREP_WALK_LIMIT)
			break;
		/* S3: fatalized context — dispatch will short-circuit to
		 * -EOWNERDEAD; prep bytes would be wasted.
		 */
		if (READ_ONCE(iter->ctx->fatal))
			continue;
		/* Already owns a bank — walker (or prior dispatch inline)
		 * assigned bank_idx already. No re-assignment needed.
		 */
		if (iter->bank_idx != MLA_BANK_UNASSIGNED)
			continue;
		/* Look for a FREE bank in this model's pool. FREE → PREPARING
		 * transition atomically claims the bank; if we lose the race,
		 * try the next slot. Multiple same-model queued jobs can each
		 * grab a distinct FREE bank in sequence.
		 *
		 * S6 (per-model prep-ahead serialisation) and S1 (same-model
		 * as in_flight) are both dissolved by per-bank isolation:
		 * whichever bank the in_flight job holds is ACTIVE, so this
		 * loop won't pick it; another same-model queued job that
		 * finds a different FREE bank can prep in parallel.
		 */
		n = READ_ONCE(iter->model->num_banks);
		for (k = 0; k < n; k++) {
			struct mla_desc_bank *b = &iter->model->banks[k];

			if (mla_bank_try_claim(iter->model, k)) {
				/* Taken here, under sched_lock, together with the
				 * claim -- not in the worker. Dispatch may steal
				 * this bank the moment it is PREPARING, and a ref
				 * taken once the worker runs would leave the bank
				 * releasable and re-claimable in between.
				 * Dropped at out_release in mla_hw_prep_work_fn.
				 */
				atomic_inc(&b->writers);
				iter->bank_idx = (u8)k;
				iter->prep_bank_idx = (u8)k;
				return iter;
			}
			/* Only a FREE bank held off by a live writer is the drain
			 * case. Without the state test this also counts every bank
			 * legitimately PREPARING under its own worker, and the READY
			 * window before that worker drops its ref -- both ordinary,
			 * and both far more common than a real drain, which would
			 * leave the counter useless as evidence that the drain
			 * protection engaged.
			 */
			if (atomic_read(&b->state) == MLA_BANK_FREE &&
			    atomic_read(&b->writers))
				atomic_inc(&mdev->dispatch_bank_draining);
		}
		/* No FREE bank available for this model. Skip and try the
		 * next candidate; a later grow (from the enqueue path) or
		 * a completion FREEing a bank will make this eligible.
		 */
	}
	return NULL;
}

/**
 * mla_sched_try_prep_ahead() — queue prep for a head-of-plist candidate
 * @mdev: device to try prep-ahead on
 *
 * Called from mla_sched_enqueue (after in_flight is confirmed busy) and
 * from mla_sched_complete's dispatch pick path (after the new in_flight
 * is set). Fast path: reads in_flight, walks a bounded prefix of plist,
 * claims a prep_in_flight slot, cmpxchg-claims a FREE descriptor bank for
 * the candidate, then queues the prep worker. Any race loses cleanly and
 * falls back to inline reloc on dispatch.
 */
void mla_sched_try_prep_ahead(struct mla_device *mdev)
{
	struct mla_job *cand;
	unsigned long flags;

	if (!mdev->prep_wq)
		return;
	if (READ_ONCE(mdev->device_fatal))
		return;
	/* Allow up to (MLA_MAX_BANKS - 1) concurrent
	 * prep workers, matching the pool's structural cap (ACTIVE bank
	 * excluded). atomic_inc_return + rollback on ceiling is TOCTOU-safe
	 * — a peer that lost the race sees inc_return > cap, decrements,
	 * and bails. The per-bank cmpxchg in the walker is the actual
	 * bank-claim primitive; this counter just bounds worker fan-out
	 * so we don't queue N prep_work items on the workqueue where
	 * only N-1 banks can be claimed.
	 */
	if (atomic_inc_return(&mdev->prep_in_flight) > MLA_MAX_BANKS - 1) {
		atomic_dec(&mdev->prep_in_flight);
		return;
	}

	spin_lock_irqsave(&mdev->sched_lock, flags);
	cand = mla_sched_find_prep_candidate_locked(mdev);
	/* Walker already atomically transitioned banks[cand->bank_idx]
	 * FREE → PREPARING and stashed the index in cand->bank_idx. Any
	 * subsequent walker call for the same job sees bank_idx != UNASSIGNED
	 * and skips it. The bank cmpxchg is the only claim primitive needed.
	 */
	spin_unlock_irqrestore(&mdev->sched_lock, flags);

	if (!cand) {
		atomic_dec(&mdev->prep_in_flight);
		return;
	}

	/* Worker holds a fence ref for its lifetime so mla_fence_release
	 * can't kfree the job out from under the bank->state / bank_idx
	 * accesses inside apply_relocs.
	 */
	mla_job_get(cand);
	if (mla_debug & MLA_DBG_TRACE)
		dev_info(mdev->dev,
			 "v2_trace: prep-ahead PREPARING model=\"%.20s\" bank=%u\n",
			 cand->model ? cand->model->name : "?",
			 cand->bank_idx);
	if (!queue_work(mdev->prep_wq, &cand->prep_work)) {
		/* Already queued — should be impossible with our own reservation,
		 * but recover cleanly: release the bank and drop the reservation.
		 * The writer ref goes back too, since no worker will reach
		 * out_release for this claim.
		 */
		struct mla_desc_bank *b = &cand->model->banks[cand->bank_idx];

		mla_job_put(cand);
		atomic_set(&b->state, MLA_BANK_FREE);
		cand->prep_bank_idx = MLA_BANK_UNASSIGNED;
		atomic_dec(&b->writers);
		cand->bank_idx = MLA_BANK_UNASSIGNED;
		atomic_dec(&mdev->prep_in_flight);
		/* Releasing the bank can unblock a dispatch that found none free,
		 * and no completion is pending to restart the queue.
		 */
		mla_sched_dispatch_next(mdev);
	}
}

/*
 * Per-job burst-size dance. If the next model needs a different DMA
 * burst configuration than the current one, reset the MLA, re-enable
 * IRQs, and write the new burst before GO. Semaphored models require
 * a 256 B WR burst (default 1 KB deadlocks their DMA descriptors).
 */
static void mla_sched_apply_burst(struct mla_device *mdev,
				  const struct mla_job *job)
{
	const struct mla_model *m = job->model;
	u32 desired_wr, desired_rd;

	if (!mdev->regs || !m)
		return;

	desired_wr = m->using_semaphores ? MLA_SEMA_WR_BURST :
					   MLA_DEFAULT_WR_BURST;
	desired_rd = MLA_DEFAULT_RD_BURST;

	if (mdev->cur_burst_wr == (s8)desired_wr &&
	    mdev->cur_burst_rd == (s8)desired_rd)
		return;

	if (mla_debug & MLA_DBG_TRACE) {
		dev_info(mdev->dev,
			"burst dance: rd %d→%u, wr %d→%u (semaphores=%d)\n",
			mdev->cur_burst_rd, desired_rd,
			mdev->cur_burst_wr, desired_wr,
			m->using_semaphores);
	}

	/* The MLA latches DMA_CFG only out of reset, so changing the burst on
	 * a block that has run something since its last reset needs one. When
	 * hw_fresh_reset is already set there is nothing to redo — resetting
	 * again here would race the recovery worker's own PRC reset and can
	 * trigger a synchronous external abort reading TLC_NON_FTL_INTR_EN.
	 *
	 * Skipping the reset in that case is still safe for the latch
	 * requirement: mla_hw_reset() has just run, so the write below is
	 * landing on a freshly reset block, the condition the hardware wants.
	 */
	if (!READ_ONCE(mdev->hw_fresh_reset) &&
	    (mdev->mlaclk || mdev->prc_regs)) {
		(void)mla_hw_init(mdev);    /* reset + re-enable IRQs */
		atomic_inc(&mdev->burst_resets);
	}
	mla_hw_set_dma_burst_size(mdev, desired_rd, desired_wr);
	atomic_inc(&mdev->burst_reprograms);
}

/* Both defined next to mla_sched_complete below. Forward-declared
 * because mla_sched_enqueue has to unwind a pre-GO dispatch failure.
 */
static bool mla_sched_finish_job(struct mla_device *mdev, struct mla_job *job,
				 int result, bool hw_started);
static void mla_sched_notify_done(struct mla_job *job);

/*
 * Drive the hardware to start the picked job. Runs without sched_lock
 * so it can sleep (cache sync, register I/O loops). The job is
 * already in mdev->in_flight from the reserve step, so the IRQ path
 * will find it the moment progdone fires.
 *
 * Returns 0 once the job is running — GO written and the watchdog
 * armed — or a negative errno if it failed BEFORE GO.
 *
 * A negative return is load-bearing and must not be discarded. Such a
 * job is still parked in mdev->in_flight (put there by the reserve
 * step) with no timer armed, no GO written and therefore no interrupt
 * ever coming: nothing will complete it. Since
 * mla_sched_pick_reserve_locked() refuses to pick while in_flight is
 * non-NULL, the queue would stop dispatching permanently — and not
 * even fd close recovers it, because mla_sched_dequeue() only touches
 * jobs in MLA_JOB_QUEUED while this one is already RUNNING. The caller
 * MUST hand a failure to mla_sched_finish_job(); both call sites do so
 * via mla_sched_dispatch_next().
 */
static int mla_sched_dispatch(struct mla_device *mdev, struct mla_job *job)
{
	if (!mdev->regs)
		return -ENODEV;    /* debug mode: no real hardware */

	/* Stamp dispatch start HERE, not at the call sites: both callers
	 * (mla_sched_enqueue's inline dispatch and mla_sched_dispatch_next)
	 * funnel through here, so one stamp covers both and can't drift out
	 * of sync with a new call site. A job left with ts_dispatch == 0
	 * (the kzalloc default) reports garbage in the derived
	 * ksub.lat / krlc.lat trace fields exposed to userspace.
	 */
	job->ts_dispatch = ktime_get();

	mla_sched_apply_burst(mdev, job);
	return mla_hw_program_and_start(job);
}

/*
 * True if some bank of @m could be claimed now: FREE with no writer left,
 * the test mla_bank_try_claim() applies. Unlocked and advisory -- a stale
 * answer only means one bank too many or too few gets grown.
 */
static bool mla_model_has_claimable_bank(struct mla_model *m)
{
	u32 k, n = READ_ONCE(m->num_banks);

	for (k = 0; k < n; k++)
		if (!atomic_read(&m->banks[k].writers) &&
		    atomic_read(&m->banks[k].state) == MLA_BANK_FREE)
			return true;
	return false;
}

/**
 * mla_sched_enqueue() — add a job to the run queue and dispatch if idle
 * @job: prepared job (state==QUEUED, prog_done_count=0)
 *
 * Inserts under sched_lock in priority order, then atomically
 * reserves the in_flight slot if free. Hardware programming runs
 * outside the lock — it may sleep.
 *
 * Called from MLA_OP_JOB_SUBMIT after the job's dma-fence has been
 * created and (in async mode) attached to the ioctl reap carrier.
 */
void mla_sched_enqueue(struct mla_job *job)
{
	struct mla_device *mdev = job->ctx->mdev;
	struct mla_job *to_dispatch = NULL;
	struct mla_job *in_flight;
	unsigned long flags;
	int now;

	/* Keep the gate on until this job is retired. The fd's runtime-PM
	 * reference is not enough: an async job keeps running in hardware
	 * after its fd is closed (mla_context_jobs_release() only cancels
	 * queued jobs). Dropped in mla_sched_finish_job(), which every
	 * enqueued job reaches exactly once. The submitting fd holds the
	 * device active here, so no resume is needed.
	 */
	pm_runtime_get_noresume(mdev->dev);

	/* Cold-provisioned models never allocate backing here. Legacy models
	 * retain best-effort growth. atomic_inc_return returns the new
	 * value; if it exceeds current num_banks (i.e. another job of
	 * this model is already alive) AND we haven't hit the ceiling,
	 * grow one bank. mla_model_grow_banks re-checks under its own
	 * mutex so two concurrent enqueuers don't double-allocate.
	 * Growth is best-effort — a failed grow (ENOMEM) leaves us with
	 * fewer banks than ideal; the new job still runs against
	 * whatever bank was available at dispatch time.
	 *
	 * Also grow when no bank is claimable right now, even with only this
	 * job outstanding. A sync submitter has one job alive at a time, so
	 * the count test alone never grows its model past bank[0] — and that
	 * bank can be FREE with a writer still on it: a prep worker claimed
	 * it for the previous job, dispatch stole it, and the job completed
	 * before the worker ever ran. mla_bank_try_claim() refuses such a
	 * bank, so on a busy host this job would be refused at dispatch with
	 * -EAGAIN ("no free bank"). Growing here, where sleeping is allowed,
	 * gives it a bank instead.
	 */
	now = atomic_inc_return(&job->model->outstanding);
	if (!job->model->banks_prepared &&
	    READ_ONCE(job->model->num_banks) < MLA_MAX_BANKS &&
	    (now > (int)READ_ONCE(job->model->num_banks) ||
	     !mla_model_has_claimable_bank(job->model)))
		(void)mla_model_grow_banks(job->model);

	spin_lock_irqsave(&mdev->sched_lock, flags);
	/* Refuse to queue against a model whose descriptors are being rewritten
	 * by MLA_OP_MODEL_UPDATE_RELOC. Tested here, under the lock that also
	 * publishes the flag and that the update op reads `outstanding` under,
	 * so there is no window between the test and the insert: either we see
	 * the flag, or the update sees the atomic_inc_return above. A check in
	 * the submit handler instead would leave exactly that window open, and
	 * what fits through it is the device reading a half-rewritten descriptor
	 * set — an FTL fault or quadgroup timeout, not a clean error.
	 *
	 * Reported through mla_sched_finish_job() rather than by failing the
	 * enqueue: the job is fully built by now (fence created, bindings
	 * resolved, and in async mode a sync_file fd already installed in the
	 * caller's table), so finalizing it the same way a pre-GO dispatch
	 * failure is finalized reuses the one path that gets all of that right.
	 * Sync callers read -EBUSY from job->result; async callers get it in
	 * their CQE.
	 */
	if (unlikely(job->model->reloc_update_in_progress)) {
		spin_unlock_irqrestore(&mdev->sched_lock, flags);
		atomic_inc(&mdev->reloc_submits_refused);
		dev_warn_ratelimited(mdev->dev,
				     "job %u refused: model %u has a reloc update in progress; wait for MLA_OP_MODEL_UPDATE_RELOC to return\n",
				     job->id, job->model->id);
		/* Non-zero result, so finish_job cannot defer the wakeup. It
		 * also pairs the atomic_dec for the increment above.
		 */
		(void)mla_sched_finish_job(mdev, job, -EBUSY, false);
		return;
	}
	mla_sched_insert_locked(mdev, job);
	in_flight = mdev->in_flight;
	to_dispatch = mla_sched_pick_reserve_locked(mdev);
	spin_unlock_irqrestore(&mdev->sched_lock, flags);

	if (mla_debug & MLA_DBG_TRACE)
		dev_info(mdev->dev,
			"v2_trace: enqueue job %u ctx=%p prio=(%d,%d) %s in_flight=%u(%s)\n",
			job->id, job->ctx,
			job->ctx->group_prio, job->ctx->intra_group_prio,
			to_dispatch ? "DISPATCH" : "QUEUED",
			in_flight ? in_flight->id : 0,
			in_flight ? "busy" : "idle");

	if (to_dispatch) {
		int ret;

		ret = mla_sched_dispatch(mdev, to_dispatch);
		if (ret) {
			/* Failed before GO: release the in_flight slot this
			 * job reserved, then drive whatever is still queued.
			 * Skipping this wedges the queue permanently — see
			 * the contract on mla_sched_dispatch().
			 */
			/* Non-zero result, so finish_job cannot defer the wakeup. */
			(void)mla_sched_finish_job(mdev, to_dispatch, ret, false);
			mla_sched_dispatch_next(mdev);
		}
		/* No try_prep_ahead on the success path, deliberately:
		 * dispatching into an idle device is what keeps the first
		 * dispatches of a run inline, and the measured prep
		 * hit_rate depends on that shape.
		 */
	} else if (in_flight) {
		/* HW busy — try to prep this (or another head-of-plist)
		 * job ahead of dispatch.
		 */
		mla_sched_try_prep_ahead(mdev);
	}
}

/**
 * mla_sched_in_flight() — read the currently-running job pointer
 * @mdev: device to query
 *
 * Used by the IRQ handlers to find the job to attribute a progdone or
 * fault to. The caller gets a fence reference and must dma_fence_put() it.
 *
 * mdev->in_flight owns a reference (taken in mla_sched_pick_reserve_locked,
 * dropped by finish_job), so a job in the slot is always alive even in the
 * pick→arm_timeout window before the watchdog holds its own ref. The get is
 * safe under sched_lock: finish_job clears the slot under this lock before it
 * drops the reference, so a job still found here still holds the in_flight
 * ref and the get cannot start from zero.
 *
 * Return: referenced in-flight job, or NULL if the slot is empty (benign — a
 * non-fatal IRQ without a job is logged and dropped, not an error).
 */
struct mla_job *mla_sched_in_flight(struct mla_device *mdev)
{
	struct mla_job *job;
	unsigned long flags;

	spin_lock_irqsave(&mdev->sched_lock, flags);
	job = mdev->in_flight;
	if (job)
		dma_fence_get(&job->fence);
	spin_unlock_irqrestore(&mdev->sched_lock, flags);
	return job;
}

/**
 * mla_sched_dequeue() — remove a queued (not yet running) job
 * @job: job to dequeue
 *
 * Used by MLA_OP_JOB_CANCEL to drop a job before it runs and by
 * mla_sched_fault_context to drain queued jobs after a fault.
 *
 * Atomically transitions the job to MLA_JOB_DONE if and only if it
 * was MLA_JOB_QUEUED. Running and already-done jobs are not touched
 * — those are completed via the normal IRQ path.
 *
 * Return: true if the job was queued and has now been removed; false
 * if it wasn't queued (already running, already done, or never
 * enqueued).
 */
bool mla_sched_dequeue(struct mla_job *job)
{
	struct mla_device *mdev = job->ctx->mdev;
	unsigned long flags;
	bool removed = false;

	spin_lock_irqsave(&mdev->sched_lock, flags);
	if (job->state == MLA_JOB_QUEUED && !plist_node_empty(&job->sched_node)) {
		mla_plist_del(&job->sched_node, &mdev->sched_queue);
		job->state = MLA_JOB_DONE;
		removed = true;
	}
	spin_unlock_irqrestore(&mdev->sched_lock, flags);
	return removed;
}

/**
 * mla_sched_fault_context() — fatalize a context after one of its jobs faulted
 * @ctx: context to fatalize
 *
 * Marks the context fatal so the command dispatcher returns
 * -EOWNERDEAD on subsequent ops, and drains the context's pending
 * job xarray, completing each queued (not-yet-running) job with
 * -ECANCELED so callers waiting on those fences get an error
 * instead of a timeout.
 *
 * The currently-running (in-flight) job is not touched here — the
 * caller (mla_sched_complete) has already completed it with the
 * fault errno.
 *
 * Safe from any context: WRITE_ONCE for the flag, xa_for_each takes
 * its own lock, and the only sleeping calls (dma_fence_signal
 * callbacks) are reached via mla_sched_complete which is itself
 * IRQ-safe.
 */
void mla_sched_fault_context(struct mla_context *ctx)
{
	struct mla_job *job;
	unsigned long index;

	WRITE_ONCE(ctx->fatal, true);

	/* Drain queued jobs.
	 *
	 * The lookup must publish a fence reference atomically with itself,
	 * which is why this is an open loop rather than xa_for_each: async
	 * jobs are erased and freed at their terminal point (MLA_IOCTL_REAP),
	 * so a bare iteration hands back a pointer a concurrent completion
	 * can kfree before the job->state read below. xa_for_each drops its
	 * RCU lock before the body runs, and struct mla_job is not
	 * RCU-freed, so RCU is no protection here.
	 *
	 * The fence is the job's refcount (no separate refcount_t); the ref
	 * is what keeps mla_sched_dequeue + mla_sched_complete safe. The put
	 * is outside the xarray lock because mla_sched_complete re-enters
	 * the scheduler and may dispatch.
	 */
	for (index = 0; ; index++) {
		xa_lock(&ctx->jobs);
		job = xa_find(&ctx->jobs, &index, ULONG_MAX, XA_PRESENT);
		if (job)
			dma_fence_get(&job->fence);
		xa_unlock(&ctx->jobs);

		if (!job)
			break;

		if (job->state == MLA_JOB_QUEUED &&
		    mla_sched_dequeue(job)) {
			/* dequeued from run queue; complete it */
			mla_sched_complete(ctx->mdev, job, -ECANCELED);
		}

		dma_fence_put(&job->fence);
	}
}

/**
 * mla_sched_should_recover() — recovery-budget rate limiter
 * @mdev: device to check/charge
 *
 * Keeps a sliding-window counter of recoveries on @mdev. If the
 * count exceeds MLA_RECOVERY_BUDGET_COUNT within
 * MLA_RECOVERY_BUDGET_WINDOW jiffies, returns false (caller should
 * treat the device as unrecoverable). Otherwise increments the
 * counter and returns true.
 */
static bool mla_sched_should_recover(struct mla_device *mdev)
{
	unsigned long flags, now = jiffies;
	bool ok;

	spin_lock_irqsave(&mdev->recovery_lock, flags);
	if (time_after(now,
		       mdev->recovery_window_start + MLA_RECOVERY_BUDGET_WINDOW)) {
		mdev->recovery_window_start = now;
		mdev->recovery_count        = 0;
	}
	mdev->recovery_count++;
	ok = mdev->recovery_count <= MLA_RECOVERY_BUDGET_COUNT;
	spin_unlock_irqrestore(&mdev->recovery_lock, flags);
	return ok;
}

/*
 * Finalize exactly one job: release the in-flight slot, cancel the
 * watchdog, sync OFM back, signal the fence, release the bank, wake the
 * waiter and charge the fault budget.
 *
 * Split out of mla_sched_complete() so the pre-GO dispatch failure paths
 * can finalize a job WITHOUT kicking the queue, which is what lets
 * mla_sched_dispatch_next() stay a loop instead of recursing.
 *
 * @hw_started is false only on those pre-GO paths, where the job never
 * reached the GO write. It suppresses the fault/recovery escalation: see
 * the comment on that block for why charging a software error to the
 * hardware recovery budget is the wrong answer.
 */
/*
 * Wake whoever is waiting on @job: the synchronous JOB_SUBMIT caller.
 * Split out of mla_sched_finish_job so mla_sched_complete can run it AFTER
 * the next dispatch when mla_defer_notify is set -- see that param's comment
 * in mla_drv.c.
 *
 * Async jobs (MLA_IOCTL_SUBMIT_ASYNC) are NOT woken here: their completion
 * carrier is the job's dma-fence, already signalled by mla_sched_finish_job
 * before this runs, and MLA_IOCTL_REAP collects them off the fence. So for an
 * async job this is a no-op; deferring only delays the sync notification,
 * never the fence.
 */
static void mla_sched_notify_done(struct mla_job *job)
{
	if (job->sync_wait) {
		job->ts_wake = ktime_get();
		complete(&job->sync_done);
	}
}

static bool mla_sched_finish_job(struct mla_device *mdev, struct mla_job *job,
				 int result, bool hw_started)
{
	unsigned long flags;
	bool was_in_flight;

	job->ts_complete = ktime_get();

	dev_dbg(mdev->dev,
		"sched_complete: job %u result=%d in_irq=%d sync_wait=%d\n",
		job->id, result, !!in_interrupt(), job->sync_wait);

	spin_lock_irqsave(&mdev->sched_lock, flags);
	was_in_flight = (mdev->in_flight == job);
	if (was_in_flight)
		mdev->in_flight = NULL;

	/* Unlink from the run queue if this job is still on it.
	 *
	 * plist membership is a WEAK reference: mla_sched_enqueue() links the
	 * node without taking a fence ref, so the only thing keeping a queued
	 * job alive is whoever else holds a ref. Finalizing therefore MUST
	 * unlink, or the next holder to drop its last ref frees an object that
	 * is still reachable from mdev->sched_queue — and every later walk
	 * (mla_sched_pick_reserve_locked, mla_sched_find_prep_candidate_locked)
	 * then dereferences freed memory.
	 *
	 * It lives HERE rather than at each caller because this is the single
	 * point every finalization funnels through, so it cannot be forgotten.
	 * mla_sched_cancel_ctx_jobs() dequeues before calling us and the
	 * pre-GO failure paths were already unlinked by the reserve step, so
	 * for them this is a no-op — plist_node_empty() is true and
	 * mla_plist_del() is skipped.
	 *
	 * The caller that actually needs it: the sync waiter's +500 ms belt in
	 * mla_op_job_submit. A sync job is NOT in ctx->jobs (submit skips
	 * xa_alloc for sync mode), so neither mla_sched_fault_context() nor
	 * mla_sched_cancel_ctx_jobs() can see it to dequeue — the belt is its
	 * only finalizer, and it drops both fence refs and frees the job right
	 * after, so a still-linked job would be freed while still reachable
	 * from sched_queue.
	 *
	 * Before the signaled check below on purpose: unlinking is idempotent
	 * and must happen on every finalizing path, including the one that
	 * returns early.
	 */
	if (!plist_node_empty(&job->sched_node))
		mla_plist_del(&job->sched_node, &mdev->sched_queue);

	/* If someone else finalized this job first (an FTL fault racing the
	 * watchdog on the same job), skip the double set_error + double signal
	 * that would trip the dma-fence WARN and cascade into a machine check —
	 * and with it every other once-only step below: the bank release, the
	 * outstanding decrement, the waiter wake, the recovery budget and the
	 * runtime-PM put.
	 *
	 * Tested and claimed here under sched_lock. The fence's signaled bit
	 * cannot serve: dma_fence_signal() sets it only after the lock is
	 * dropped, so two racing finalizers could both see it clear.
	 *
	 * Returning here does NOT skip the next dispatch: mla_sched_complete()
	 * calls mla_sched_dispatch_next() unconditionally, so an
	 * already-finalized completion still restarts the queue instead of
	 * leaving the hardware parked behind a non-empty plist.
	 */
	if (job->finalized) {
		spin_unlock_irqrestore(&mdev->sched_lock, flags);
		/* The path that got here first may not have disarmed this
		 * watchdog: an armed timer on a job about to be freed is
		 * what leaves a dangling timer_list on the timer wheel.
		 */
		mla_job_cancel_timeout(job);
		/* Whoever finalized this job first owns its wakeup too. */
		return false;
	}
	job->finalized = true;

	/* Release the descriptor bank under the same lock that cleared the
	 * in_flight slot, so a dispatcher observing the free slot also observes
	 * the free bank. Published separately, dispatch could find the slot
	 * free while every bank of that model still read ACTIVE, and have no
	 * bank to run the job in.
	 *
	 * UNASSIGNED means the job never reached a bank. Safe here because the
	 * hardware is done with these descriptors by the time any caller
	 * finalizes, and the finalized early return above keeps it
	 * once-only.
	 */
	if (job->bank_idx != MLA_BANK_UNASSIGNED &&
	    job->bank_idx < job->model->num_banks) {
		atomic_t *st = &job->model->banks[job->bank_idx].state;

		/* Reclaim only a bank this job owns for execution. A PREPARING
		 * bank still belongs to its prep worker: a cancel, context
		 * fault or device reset finalizes queued jobs, so this can run
		 * while that worker is mid-write. The worker releases it after
		 * publishing instead, and whichever side observes the other's
		 * transition does the release exactly once.
		 *
		 * Releasing an ACTIVE bank whose worker is still writing is safe
		 * and expected once dispatch is allowed to steal: the bank goes
		 * FREE with writers non-zero, which mla_bank_try_claim() refuses
		 * until that worker leaves.
		 */
		if (atomic_cmpxchg(st, MLA_BANK_ACTIVE, MLA_BANK_FREE)
			    == MLA_BANK_ACTIVE ||
		    atomic_cmpxchg(st, MLA_BANK_READY, MLA_BANK_FREE)
			    == MLA_BANK_READY)
			job->bank_idx = MLA_BANK_UNASSIGNED;
	}
	job->state  = MLA_JOB_DONE;
	job->result = result;
	spin_unlock_irqrestore(&mdev->sched_lock, flags);

	/* Disarm unconditionally, NOT gated on was_in_flight: a job can reach
	 * here with an armed timer while no longer being mdev->in_flight (the
	 * sync waiter's +500 ms belt finalizing a job the watchdog has already
	 * dequeued from in_flight, for one), and any job freed with a live
	 * timer turns a later call_timer_fn into a branch through recycled
	 * memory. Cancelling a timer that was never armed is free:
	 * timer_delete_sync_try returns 0 for "not pending" and -1 for
	 * "running its callback", and never waits, so this is also safe on
	 * the self-recursive mla_job_timeout_fn -> mla_sched_complete -> here
	 * path.
	 */
	mla_job_cancel_timeout(job);

	/* No OFM cache maintenance here. Invalidating stale CPU lines over an
	 * OFM is the caller's obligation, discharged with
	 * DMA_BUF_IOCTL_SYNC(START|READ) before it reads the buffer — see the
	 * block comment in mla_hw.c.
	 */

	if (result)
		dma_fence_set_error(&job->fence, result);
	dma_fence_signal(&job->fence);

	/* Pairs with mla_sched_enqueue's atomic_inc. Done
	 * after fence_signal so a peer reading outstanding sees "this job
	 * is finished" only after any external waiter has been released.
	 * The finalized check above (early return) means this
	 * runs exactly once per job regardless of PROGDONE / timeout /
	 * fault / cancel path.
	 *
	 * dec_and_test, so the wake fires only on the 0-transition — exactly the
	 * condition MLA_OP_MODEL_UPDATE_RELOC waits on — rather than on every
	 * completion. wake_up() is IRQ-safe, which this path needs: it runs in
	 * hard-IRQ context on the progdone route.
	 */
	if (atomic_dec_and_test(&job->model->outstanding))
		wake_up(&job->model->reloc_quiesce_wq);

	/* With mla_defer_notify set, the caller owes this job a
	 * mla_sched_notify_done() once the next dispatch has reached GO. Never
	 * deferred on a fault/timeout/cancel: the recovery block below may
	 * fatalize the context and drain the queue, and the waiter must see
	 * the error before any of that runs.
	 */
	const bool defer = mla_defer_notify && result == 0;

	if (!defer)
		mla_sched_notify_done(job);

	/* On fault, fatalize the offending context and
	 * schedule device recovery on a workqueue. The work is
	 * coalesced via recovery_pending so a fault storm doesn't
	 * spam the queue. The rate limiter inside the worker
	 * decides whether to escalate to device-fatal.
	 */
	if (result && result != -ECANCELED && was_in_flight) {
		if (!hw_started) {
			/* Failed before the GO write, so no hardware
			 * operation was ever started: there is nothing to
			 * reset, and nothing that should be charged to the
			 * recovery budget. Escalating here would let a single
			 * malformed model fatalize its context and — after
			 * MLA_RECOVERY_BUDGET_COUNT submits — mark the whole
			 * device fatal, taking every other process down with
			 * it. That is the wrong response to what is really a
			 * software validation error in the reloc walker.
			 * Report it and let the caller see the errno.
			 */
			dev_warn(mdev->dev,
				 "job %u failed before GO (rc=%d) — completed without HW recovery\n",
				 job->id, result);
		} else if (mla_no_recover) {
			/* Debug mode: freeze state on fault. Return errno to
			 * userspace via the already-signaled fence, but do not
			 * fatalize the ctx and do not schedule HW recovery.
			 * Preserves post-mortem state (registers, in-flight
			 * descriptors, section blobs) for inspection.
			 */
			dev_warn(mdev->dev,
				 "job %u faulted (rc=%d) — recovery skipped (mla_no_recover=1)\n",
				 job->id, result);
		} else {
			mla_sched_fault_context(job->ctx);
			if (!mla_sched_should_recover(mdev)) {
				dev_err(mdev->dev,
					"recovery budget exceeded; device fatal\n");
				/* Final post-mortem — this is the last chance
				 * to see MLA state before device_fatal locks
				 * out all subsequent submits.
				 */
				mla_hw_dump_status(mdev, "budget-exceeded");
				WRITE_ONCE(mdev->device_fatal, true);
			} else if (atomic_xchg(&mdev->recovery_pending, 1) == 0) {
				/* The work owns a PM reference from here until
				 * it runs or is cancelled. Taken while this job's
				 * own reference still holds the device active;
				 * otherwise that put, below, can gate the clock
				 * before the worker resumes it.
				 */
				pm_runtime_get_noresume(mdev->dev);
				if (!schedule_work(&mdev->recovery_work))
					pm_runtime_put_noidle(mdev->dev);
			}
		}
	}

	/* Pairs with mla_sched_enqueue(). Dropped last so that all MMIO done
	 * for this job (IRQ handler, watchdog, the budget-exceeded dump above)
	 * ran with the gate held. The async put is safe in hard IRQ.
	 */
	pm_runtime_put(mdev->dev);

	/* Release the reference mla_sched_pick_reserve_locked() took when it
	 * put this job in in_flight. Paired with was_in_flight so it runs
	 * exactly once (the finalized early-return above already saw in_flight
	 * cleared and skipped it). Must be the LAST touch of @job here: it can
	 * be the final put and free the job. Every mla_sched_complete() caller
	 * holds its own ref across the call (IRQ via mla_sched_in_flight, the
	 * watchdog via the timer, cancel paths at lookup), so this never frees
	 * a job the caller still uses.
	 */
	if (was_in_flight)
		dma_fence_put(&job->fence);

	return defer;
}

/*
 * Start the next queued job, if any. Reserve under sched_lock, program
 * the hardware outside it.
 *
 * Iterative rather than recursive, deliberately. A job that fails before
 * GO has to be finalized and the next one tried; doing that by calling
 * back into mla_sched_complete() would nest one
 * complete->dispatch->complete frame per queued entry, so a model whose
 * relocs fail for every job (a bad desc_index, say) would recurse to the
 * full depth of the plist on a 16K kernel stack. The loop keeps it flat.
 */
void mla_sched_dispatch_next(struct mla_device *mdev)
{
	unsigned long flags;

	for (;;) {
		struct mla_job *next;
		int ret;

		/* Skip dispatch if device is fatal — there's no point
		 * starting a job that the (broken) hardware can't run;
		 * mla_sched_dispatch would just hit -ENODEV.
		 */
		if (READ_ONCE(mdev->device_fatal))
			return;

		spin_lock_irqsave(&mdev->sched_lock, flags);
		next = mla_sched_pick_reserve_locked(mdev);
		spin_unlock_irqrestore(&mdev->sched_lock, flags);

		if (!next)
			return;

		ret = mla_sched_dispatch(mdev, next);
		if (!ret) {
			/* With new in_flight running, try prep for the
			 * *following* head-of-plist entry.
			 */
			mla_sched_try_prep_ahead(mdev);
			return;
		}

		/* Failed before GO: this job holds the in_flight slot and
		 * nothing will ever complete it. Finalize it here, then try
		 * the next queued entry.
		 */
		/* Non-zero result, so finish_job cannot defer the wakeup. */
		(void)mla_sched_finish_job(mdev, next, ret, false);
	}
}

/**
 * mla_sched_complete() — finalize a job and pick the next one
 * @mdev:   device that owns the scheduler
 * @job:    job to complete
 * @result: 0 on success; negative errno on fault/timeout/cancel
 *
 * Removes @job from the in-flight slot, signals its dma-fence
 * (with @result as the fence error code on failure), syncs OFM
 * dma-bufs FROM_DEVICE on success, wakes any sync-mode waiter, and
 * picks the next queued job for dispatch.
 *
 * On non-zero @result this also:
 *   - fatalizes the job's context (further ops return -EOWNERDEAD)
 *   - schedules mla_recovery_work_fn() on the device workqueue
 *   - charges the recovery rate-limit budget; if exceeded, marks
 *     the device fatal so all submissions return -EHOSTDOWN.
 *
 * Safe to call from hard-IRQ, softirq, or process context. The
 * heavy work (PRC reset, IRQ re-enable) runs on a workqueue so this
 * function never blocks.
 *
 * The next-dispatch step runs even when @job turns out to have been
 * finalized already by a racing completer: the hardware is idle either
 * way, so something still has to restart the queue.
 *
 * Callers:
 *   - mla_irq_non_ftl on progdone (result == 0)
 *   - mla_job_timeout_fn on watchdog (result == -ETIMEDOUT)
 *   - mla_irq_ftl on fault (result == -EIO)
 *   - mla_op_job_cancel / context teardown (result == -ECANCELED)
 */
void mla_sched_complete(struct mla_device *mdev, struct mla_job *job,
			int result)
{
	const bool owed = mla_sched_finish_job(mdev, job, result, true);

	/* Hold @job across the dispatch below. finish_job has already signalled
	 * the fence and cancelled the watchdog (dropping the reference an armed
	 * timer owns), so on a sync job the only references left belong to the
	 * submitter blocked in wait_for_completion_timeout() — and its
	 * timeout_ms + 500 ms belt can expire inside this window, drop both refs
	 * and free the job before we reach notify_done(), which writes
	 * job->ts_wake and completes job->sync_done.
	 *
	 * Only when a wakeup is actually owed: on the common !mla_defer_notify
	 * path notify_done already ran inside finish_job, so this costs nothing.
	 *
	 * Every caller now holds its own reference across this call (the IRQ
	 * handlers take one in mla_sched_in_flight(), the watchdog holds the
	 * timer's until after we return, cancel paths take one at lookup), so
	 * this get never starts from zero; it is kept as a local guarantee.
	 */
	if (owed)
		dma_fence_get(&job->fence);

	mla_sched_dispatch_next(mdev);

	/* Off the device-idle path now: the next job already has GO. */
	if (owed) {
		mla_sched_notify_done(job);
		/* May be the last put, freeing the job from hard IRQ. That is an
		 * established path here (mla_job_cancel_timeout does it) and
		 * mla_fence_release is written for it. Must stay outside
		 * sched_lock, which mla_fence_release takes.
		 */
		dma_fence_put(&job->fence);
	}
}

/*
 * Cancel every job registered on @ctx, queued or running, with
 * -ECANCELED. Returns the number finalized.
 *
 * This is mla_sched_fault_context's drain loop with two deliberate
 * differences: it does NOT set ctx->fatal (the caller is clearing fatal
 * state, not creating it), and it finalizes the in-flight job too. The
 * fault path can skip in-flight because its caller has already
 * completed that job with the fault errno; a reset has no such caller,
 * and leaving a job in mdev->in_flight across mla_hw_recover() would
 * reset the hardware out from under a job whose watchdog is still armed
 * — the stranded-in_flight shape that wedges the queue permanently.
 *
 * The xa_find-with-fence-ref loop is load-bearing, not styling:
 * xa_for_each drops its internal RCU lock before the body runs and
 * struct mla_job is not RCU-freed, so a concurrent terminal completion
 * (MLA_IOCTL_REAP erases and frees at the async terminal point) can kfree
 * the iterator's pointer before the job->state read. The fence IS the
 * job's refcount, so the get must happen under xa_lock.
 */
static u32 mla_sched_cancel_ctx_jobs(struct mla_context *ctx)
{
	struct mla_device *mdev = ctx->mdev;
	struct mla_job *job;
	unsigned long index;
	u32 cancelled = 0;

	for (index = 0; ; index++) {
		unsigned long flags;
		bool in_flight;

		xa_lock(&ctx->jobs);
		job = xa_find(&ctx->jobs, &index, ULONG_MAX, XA_PRESENT);
		if (job)
			dma_fence_get(&job->fence);
		xa_unlock(&ctx->jobs);

		if (!job)
			break;

		/* Finalize whether it is queued or running. Dequeue first so
		 * a job sitting on the plist leaves it; if it is the
		 * in-flight one there is nothing to dequeue (dispatch already
		 * removed it) and finish_job clears mdev->in_flight and
		 * disarms the watchdog via its was_in_flight path.
		 */
		spin_lock_irqsave(&mdev->sched_lock, flags);
		in_flight = (mdev->in_flight == job);
		spin_unlock_irqrestore(&mdev->sched_lock, flags);

		if (mla_sched_dequeue(job) || in_flight) {
			/* hw_started=false: a reset is a software cancel, so
			 * it must not charge the hardware recovery budget —
			 * otherwise resetting a device three times would
			 * itself mark it fatal.
			 *
			 * finish_job rather than mla_sched_complete: we do
			 * NOT want to dispatch the next queued job into
			 * hardware we are about to reset. The queue is kicked
			 * once at the end of mla_sched_device_reset().
			 */
			/* Non-zero result, so finish_job cannot defer the wakeup. */
			(void)mla_sched_finish_job(mdev, job, -ECANCELED, false);
			cancelled++;
		}

		dma_fence_put(&job->fence);
	}

	return cancelled;
}

/**
 * mla_sched_device_reset() — return the device to probe-equivalent state
 * @mdev:             device to reset
 * @jobs_cancelled:   out, jobs finalized -ECANCELED across all contexts
 * @ctxs_cleared:     out, contexts that had ctx->fatal set
 * @was_device_fatal: out, mdev->device_fatal as observed on entry
 *
 * Backs MLA_OP_DEVICE_RESET. Besides mla_sched_init() (probe), this is the
 * only other place that clears mdev->device_fatal; without it, a
 * device-fatal board can only recover via rmmod + re-insmod, since
 * mla_recovery_work_fn bails out once the flag is set.
 *
 * Destructive and device-wide, in this order:
 *   0. drain the job the HARDWARE is running (bounded by its watchdog), so
 *      the PRC reset does not land under live DMA — see step 0 in the body
 *   1. cancel every in-flight and queued job on every context
 *   2. mla_hw_recover() — quiesce, PRC reset, re-enable IRQs, clear
 *      stale sources, invalidate the burst + last-programmed-model cache
 *   3. clear mdev->device_fatal, reset the recovery budget window, and
 *      clear every ctx->fatal
 *   4. kick the queue, in case a submit raced step 1
 *
 * Fatal state is cleared only AFTER the hardware is back, so nothing
 * can dispatch into a half-reset device.
 *
 * Context: process context only — takes ctx_list_lock (a mutex) and
 * mla_hw_recover flushes a workqueue. Never call from IRQ or softirq.
 *
 * Return: 0 on success, or mla_hw_recover()'s error. On failure the
 * jobs are still cancelled and the device is left fatal.
 */
int mla_sched_device_reset(struct mla_device *mdev, u32 *jobs_cancelled,
			   u32 *ctxs_cleared, bool *was_device_fatal)
{
	struct mla_context *ctx;
	unsigned long flags;
	u32 n_jobs = 0, n_ctxs = 0, n_banks = 0;
	bool prep_quiesced;
	u32 i;
	int ret;

	*was_device_fatal = READ_ONCE(mdev->device_fatal);

	/* Gate dispatch FIRST, or the drain below never converges. On completion
	 * mla_sched_complete() calls mla_sched_dispatch_next(), which starts
	 * the next queued job immediately — so against a continuous
	 * submitter, waiting for one job just hands the hardware another and
	 * the device is never idle. device_fatal is the existing gate:
	 * mla_sched_dispatch_next returns early on it and so does the
	 * prep-ahead path.
	 *
	 * *was_device_fatal is already captured above, so what userspace is
	 * told about the entry state is unaffected, and step 3 below clears
	 * this again.
	 *
	 * A submit that races the reset window gets -EHOSTDOWN rather than
	 * being queued to run after the reset — consistent with an op whose
	 * whole purpose is to cancel everything in flight, but callers should
	 * be ready for this errno.
	 */
	WRITE_ONCE(mdev->device_fatal, true);

	/* STEP 0 — let the HARDWARE finish before anything resets it.
	 *
	 * This must happen BEFORE the cancel walk below, and the ordering is
	 * the whole point. mla_sched_cancel_ctx_jobs() finalizes the in-flight
	 * job too, so after the cancel software believes the device is idle
	 * even though Modalix has no way to stop the MLA mastering short of a
	 * PRC reset (no abort bit, GO is write-only) — the quadgroup is still
	 * running and moving data. Resetting under that live DMA takes a
	 * synchronous external abort when the interconnect refuses the next
	 * MMIO read. Waiting on mdev->in_flight AFTER the cancel would wait
	 * on nothing — the cancel has already NULLed it.
	 *
	 * Bounded by the job watchdog: the fence is signalled either by
	 * completion or by the timeout path, so this cannot wait forever. A
	 * genuinely wedged job still times out here and we reset under live
	 * DMA anyway — that residual case is why mla_hw_recover() must not do
	 * abort-capable MMIO under a spinlock (see mla_hw.c).
	 */
	{
		struct mla_job *hw_job = NULL;

		spin_lock_irqsave(&mdev->sched_lock, flags);
		if (mdev->in_flight) {
			hw_job = mdev->in_flight;
			dma_fence_get(&hw_job->fence);
		}
		spin_unlock_irqrestore(&mdev->sched_lock, flags);

		if (hw_job) {
			long left = dma_fence_wait_timeout(&hw_job->fence, false,
					msecs_to_jiffies(MLA_JOB_DEFAULT_TIMEOUT_MS + 500));

			if (left <= 0)
				dev_warn(mdev->dev,
					 "device_reset: in-flight job did not drain (%ld); resetting under live DMA\n",
					 left);
			dma_fence_put(&hw_job->fence);
		}
	}

	/* One walk under ctx_list_lock covers both the drain and the
	 * ctx->fatal clear. Holding it is what keeps each ctx alive: a
	 * context is removed from this list by mla_context_shutdown under
	 * this same mutex, so anything we see here still has its fd open.
	 * Safe to hold across finish_job — the only lock the fence-signal
	 * path can reach is sched_lock, and mla_context_destroy (the last
	 * put, reachable from mla_fence_release) is kfree + atomic_dec with
	 * no mutex.
	 */
	mutex_lock(&mdev->ctx_list_lock);
	list_for_each_entry(ctx, &mdev->contexts, node) {
		n_jobs += mla_sched_cancel_ctx_jobs(ctx);

		if (READ_ONCE(ctx->fatal))
			n_ctxs++;
	}
	mutex_unlock(&mdev->ctx_list_lock);

	/* Retire any recovery worker BEFORE touching hardware or clearing
	 * device_fatal.
	 *
	 * mla_recovery_work_fn's only bail-out is "device_fatal is already
	 * set" (:69). Clearing that flag below therefore RE-ARMS a worker
	 * that was scheduled by the fault which fatalized us: it would run
	 * mla_hw_recover() a second time, asynchronously, after this op has
	 * returned and userspace has been told the device is healthy — a
	 * hard PRC reset landing under a freshly dispatched job, whose
	 * completion IRQ then never arrives and whose watchdog fires into
	 * the timer path. cancel_work_sync covers both cases: it waits out a
	 * worker already running and dequeues one merely pending.
	 *
	 * recovery_pending is the coalescing flag the scheduler tests with
	 * atomic_xchg before scheduling (:794); clear it so the next genuine
	 * fault can still queue a worker.
	 */
	if (cancel_work_sync(&mdev->recovery_work))
		pm_runtime_put(mdev->dev);	/* the cancelled work's reference */
	atomic_set(&mdev->recovery_pending, 0);

	/* Hardware first, fatal flags second. */
	ret = mla_hw_recover(mdev);
	if (ret) {
		dev_err(mdev->dev,
			"device_reset: hw_recover failed (%d) — device left fatal\n",
			ret);
		*jobs_cancelled = n_jobs;
		*ctxs_cleared   = 0;
		WRITE_ONCE(mdev->device_fatal, true);
		return ret;
	}

	/* Every descriptor bank back to FREE, as part of the probe-equivalent
	 * state this op promises. A job cancelled mid-dispatch strands its bank:
	 * mla_sched_finish_job() reclaims only ACTIVE and READY, and deliberately
	 * leaves PREPARING to its prep worker, so a bank whose owning job is gone
	 * has nothing left to reclaim it and is lost for the model's lifetime.
	 * Nothing else in this function touched bank state.
	 *
	 * Ordering is load-bearing in both directions:
	 *
	 *  - It must FOLLOW mla_hw_recover(), which PRC-resets the block and only
	 *    returns 0 on success. Before that reset the quadgroup may still be
	 *    mastering -- Modalix has no abort bit -- so a bank released there
	 *    could be handed to a new job while hardware still fetched from it.
	 *    After it, nothing is mastering and the descriptors are nobody's.
	 *  - It must PRECEDE the device_fatal clear below, which is still set
	 *    here, so mla_sched_pick_reserve_locked() refuses to dispatch and no
	 *    job can claim a bank while this walk publishes FREE.
	 *
	 * num_banks cannot grow under this walk either: mla_model_grow_banks() is
	 * reached from the submit path, refused while device_fatal is set, and a
	 * newly grown bank is FREE anyway.
	 */

	/* Drain prep_in_flight before the walk, so writer refs can be judged.
	 *
	 * mla_hw_recover() above flushed prep_wq, but that alone is not enough:
	 * mla_sched_try_prep_ahead() takes the writer ref under sched_lock and
	 * queues its worker only after dropping it, so a caller that passed the
	 * device_fatal check before we set it can still make a worker visible
	 * after the flush returned. prep_in_flight is incremented BEFORE that
	 * claim and decremented by the worker's exit, so it spans that gap where
	 * flush_workqueue() cannot.
	 *
	 * Once it reads zero, no prep worker exists and none can appear, because
	 * device_fatal is still set and try_prep_ahead returns early on it.
	 */
	for (i = 0; i < 100 && atomic_read(&mdev->prep_in_flight); i++)
		usleep_range(500, 1000);
	prep_quiesced = !atomic_read(&mdev->prep_in_flight);
	if (!prep_quiesced)
		dev_warn(mdev->dev,
			 "device_reset: prep did not quiesce (in_flight=%d); leaving writer refs alone\n",
			 atomic_read(&mdev->prep_in_flight));

	mutex_lock(&mdev->ctx_list_lock);
	list_for_each_entry(ctx, &mdev->contexts, node) {
		struct mla_model *m;
		unsigned long id;

		xa_for_each(&ctx->models, id, m) {
			u32 k;

			for (k = 0; k < READ_ONCE(m->num_banks); k++) {
				struct mla_desc_bank *b = &m->banks[k];

				/* xchg, not set: a bank that was not already FREE
				 * was genuinely stranded, and that is worth
				 * counting rather than silently repairing.
				 */
				if (atomic_xchg(&b->state, MLA_BANK_FREE) !=
				    MLA_BANK_FREE)
					n_banks++;

				if (atomic_read(&b->writers)) {
					atomic_inc(&mdev->reset_bank_had_writer);
					/* Clear only once prep is quiesced. A
					 * LIVE ref must be left alone: zeroing
					 * it hands the bank to another job while
					 * that worker is still in the descriptor
					 * loop, the corruption this protocol
					 * exists to prevent.
					 *
					 * Quiesced there is no worker, so a
					 * non-zero count is a leak -- an
					 * invariant violation rather than a
					 * reachable race, hence the warn.
					 */
					if (prep_quiesced) {
						WARN_ONCE(1, "mla: bank %u writer ref leaked (%d)\n",
							  k, atomic_read(&b->writers));
						atomic_set(&b->writers, 0);
					}
				}
			}
		}
	}
	mutex_unlock(&mdev->ctx_list_lock);
	atomic_add(n_banks, &mdev->reset_banks_reclaimed);

	spin_lock_irqsave(&mdev->recovery_lock, flags);
	mdev->recovery_count        = 0;
	mdev->recovery_window_start = jiffies;
	spin_unlock_irqrestore(&mdev->recovery_lock, flags);

	WRITE_ONCE(mdev->device_fatal, false);

	mutex_lock(&mdev->ctx_list_lock);
	list_for_each_entry(ctx, &mdev->contexts, node)
		WRITE_ONCE(ctx->fatal, false);
	mutex_unlock(&mdev->ctx_list_lock);

	/* A submit could have raced step 1 on a device that was not fatal.
	 * Kick the queue so such a job is not stranded behind the reset —
	 * the same reason mla_sched_complete always calls this.
	 */
	mla_sched_dispatch_next(mdev);

	*jobs_cancelled = n_jobs;
	*ctxs_cleared   = n_ctxs;

	dev_info(mdev->dev,
		 "device_reset: complete (was_fatal=%d jobs_cancelled=%u ctxs_cleared=%u banks_reclaimed=%u)\n",
		 *was_device_fatal, n_jobs, n_ctxs, n_banks);

	return 0;
}

