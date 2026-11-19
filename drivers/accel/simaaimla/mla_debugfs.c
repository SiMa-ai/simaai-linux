// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// Diagnostic-only debugfs surface under /sys/kernel/debug/mla/, all
// read-only, one file per topic:
//
//   counters/     — one integer per file, every scalar the driver keeps.
//                   Use these for anything automated: one open, one read,
//                   no parsing.
//   section_dedup — the device-wide section-dedup table, one row per entry,
//                   plus the aggregate cross-check of that same walk. This
//                   is what turns a non-zero counters/dedup_mismatches into
//                   an actionable diagnosis: the counter says how many rows
//                   failed, the rows say which and why.
//   contexts      — one row per open context: owner, scheduling priority,
//                   and the buffers it holds broken down by kind.
//
// The tables stay tables. One-value-per-file is a *sysfs* convention
// (Documentation/filesystems/sysfs.rst) and a per-entry inventory cannot be
// expressed that way; cf. dma-buf's own bufinfo, same shape for the same
// reason.
//
// Note /sys/kernel/debug is 0700 root, so none of this is readable by an
// unprivileged test process — elevate for the observation only.

#include <linux/debugfs.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/xarray.h>

#include "mla_drv.h"

#ifdef CONFIG_DEBUG_FS

struct dump_entry_row {
	struct mla_section_dedup_entry *entry;
	struct mla_buffer              *primary_buf;
	unsigned int                    entry_rc;
	size_t                          size;
	u8                              sha[32];
};

/* Collected during the dedup-lock-held walk; consumed after unlock so
 * seq_printf never runs with the spinlock held.
 */
#define MLA_DUMP_MAX_ROWS  4096

struct dump_ctx {
	struct dump_entry_row rows[MLA_DUMP_MAX_ROWS];
	unsigned int          n_rows;
	unsigned int          dropped;
};

static void dedup_collect_cb(struct mla_section_dedup_entry *e,
			     unsigned int refcount, void *arg)
{
	struct dump_ctx *dc = arg;
	struct dump_entry_row *r;
	const u8 *sha;

	if (dc->n_rows >= MLA_DUMP_MAX_ROWS) {
		dc->dropped++;
		return;
	}
	r = &dc->rows[dc->n_rows++];
	r->entry = e;
	r->primary_buf = mla_section_dedup_entry_buf(e);
	r->entry_rc = refcount;
	r->size = r->primary_buf ? r->primary_buf->size : 0;
	sha = mla_section_dedup_entry_sha256(e);
	if (sha)
		memcpy(r->sha, sha, 32);
	else
		memset(r->sha, 0, 32);
}

/* Feeds the invariant check: entry->refcount must equal the number of
 * buffers holding a reference, counting both the xarray and section_bufs.
 */
static void count_bufs_for_entry(struct mla_context *ctx,
				 struct mla_section_dedup_entry *entry,
				 unsigned int *n_bufs,
				 unsigned int *sum_buf_rc,
				 unsigned int *n_primary,
				 unsigned int *n_alias,
				 unsigned int *n_via_sbufs)
{
	struct mla_buffer *buf;
	struct mla_model  *m;
	unsigned long index;

	xa_for_each(&ctx->buffers, index, buf) {
		if (!buf || buf->dedup != entry)
			continue;
		(*n_bufs)++;
		*sum_buf_rc += refcount_read(&buf->refcount);
		if (buf->dedup_alias)
			(*n_alias)++;
		else
			(*n_primary)++;
	}

	/* Also scan every model's section_bufs[] — released buffers still hold
	 * references even when gone from ctx->buffers. Count only those not in
	 * the xarray to avoid double-counting the overlap when models are resident.
	 *
	 * Compare POINTER, not id: ids recycle, so a stale id can name a live
	 * unrelated buffer (mla_op_buf_release clears buf->id to 0 for this).
	 */
	xa_for_each(&ctx->models, index, m) {
		u32 i;

		if (!m || !m->section_bufs)
			continue;
		for (i = 0; i < m->num_sections; i++) {
			struct mla_buffer *sb = m->section_bufs[i];

			if (!sb || sb->dedup != entry)
				continue;
			if (sb->id && xa_load(&ctx->buffers, sb->id) == sb)
				continue;   /* already counted above */
			(*n_via_sbufs)++;
		}
	}
}

/* Per-context buffer breakdown, so a leak can be attributed to one context
 * instead of only showing up in the device-wide totals.
 */
struct ctx_totals {
	unsigned int n_pool;
	unsigned int n_dedup_primary;
	unsigned int n_dedup_alias;
	unsigned int n_imported;
	unsigned int n_phys;
	unsigned int n_other;
	unsigned long sum_rc;
};

static void ctx_scan(struct mla_context *ctx, struct ctx_totals *t)
{
	struct mla_buffer *buf;
	unsigned long index;

	memset(t, 0, sizeof(*t));
	xa_for_each(&ctx->buffers, index, buf) {
		if (!buf)
			continue;
		t->sum_rc += refcount_read(&buf->refcount);
		if (buf->imported)
			t->n_imported++;
		else if (buf->phys_attached)
			t->n_phys++;
		else if (buf->dedup_alias)
			t->n_dedup_alias++;
		else if (buf->dedup)
			t->n_dedup_primary++;
		else if (buf->pool_owned)
			t->n_pool++;
		else
			t->n_other++;
	}
}

/* Result of one pass over the dedup table. @mismatches is a count of ROWS
 * failing the invariant, not the signed difference of the two totals: two
 * rows wrong in opposite directions cancel in the difference and would
 * report clean.
 */
struct dedup_scan_result {
	unsigned int entries;
	unsigned int truncated;
	unsigned int mismatches;
	unsigned int total_entry_rc;
	unsigned int total_bufs_via_entries;
};

/* mla_dedup_scan() — walk the dedup table, optionally rendering it
 * @mdev: device to scan
 * @s:    seq_file to print the entry table into, or NULL to only count
 * @res:  out-param, zeroed here
 *
 * This is the ONLY expression of the lock order this file needs — the
 * dedup spinlock (taken and dropped inside mla_section_dedup_walk, so no
 * seq_printf ever runs under it) then mdev->ctx_list_lock then the
 * per-context xarray internal locks. Both `dump` and every
 * counters/dedup_* file go through here precisely so that a second reader
 * cannot invert it. **Do not open-code another walk.**
 *
 * The table and the per-context section are two separate ctx_list_lock
 * critical sections rather than one, so they are snapshots of slightly
 * different moments. This trades cross-snapshot consistency for a shorter
 * lock hold; the two snapshots are never cross-referenced.
 *
 * Return: 0, or -ENOMEM if the row snapshot could not be allocated.
 */
static int mla_dedup_scan(struct mla_device *mdev, struct seq_file *s,
			  struct dedup_scan_result *res)
{
	struct mla_context *ctx;
	struct dump_ctx *dc;
	unsigned int i;

	memset(res, 0, sizeof(*res));

	/* kvzalloc, not kzalloc: struct dump_ctx is ~256 KB (64 B/row x
	 * MLA_DUMP_MAX_ROWS), which is an order-6 allocation that can fail
	 * under fragmentation.
	 */
	dc = kvzalloc(sizeof(*dc), GFP_KERNEL);
	if (!dc)
		return -ENOMEM;

	mla_section_dedup_walk(mdev, dedup_collect_cb, dc);

	res->entries   = dc->n_rows;
	res->truncated = dc->dropped;

	if (s)
		seq_printf(s, "=== dedup entries (%u total%s) ===\n",
			   dc->n_rows,
			   dc->dropped ? " [truncated]" : "");
	if (s)
		seq_printf(s, "%-16s %-16s %-3s %-16s %-10s %s\n",
			   "entry", "primary_buf", "rc", "sha[0..7]", "size",
			   "per-ctx buf counts");

	mutex_lock(&mdev->ctx_list_lock);
	for (i = 0; i < dc->n_rows; i++) {
		struct dump_entry_row *r = &dc->rows[i];
		unsigned int total_n = 0, total_sum = 0;
		unsigned int total_p = 0, total_a = 0;
		unsigned int total_sb = 0;
		bool ok;

		list_for_each_entry(ctx, &mdev->contexts, node)
			count_bufs_for_entry(ctx, r->entry,
					     &total_n, &total_sum,
					     &total_p, &total_a, &total_sb);

		res->total_entry_rc += r->entry_rc;
		res->total_bufs_via_entries += total_n + total_sb;

		ok = (r->entry_rc == total_n + total_sb);
		if (!ok)
			res->mismatches++;

		if (s)
			seq_printf(s,
				   "%016lx %016lx %3u %02x%02x%02x%02x%02x%02x%02x%02x %10zu xa=%u(p=%u a=%u) sb=%u sum_rc=%u %s\n",
				   (unsigned long)r->entry,
				   (unsigned long)r->primary_buf,
				   r->entry_rc,
				   r->sha[0], r->sha[1], r->sha[2], r->sha[3],
				   r->sha[4], r->sha[5], r->sha[6], r->sha[7],
				   r->size,
				   total_n, total_p, total_a, total_sb,
				   total_sum, ok ? "OK" : "MISMATCH");
	}
	mutex_unlock(&mdev->ctx_list_lock);

	kvfree(dc);
	return 0;
}

/* Exists because a per-entry inventory cannot be one-integer-per-file, and
 * because it turns a non-zero counters/dedup_mismatches into an actionable
 * diagnosis: the counter says how many rows failed, these rows say which and
 * why (compare rc against xa= + sb=).
 */
static int mla_section_dedup_show(struct seq_file *s, void *unused)
{
	struct mla_device *mdev = s->private;
	struct dedup_scan_result res;
	int err;

	err = mla_dedup_scan(mdev, s, &res);
	if (err)
		return err;

	seq_printf(s, "\nsum(entry->refcount) = %u\n", res.total_entry_rc);
	seq_printf(s, "sum(bufs pointing at any entry, all ctxs) = %u\n",
		   res.total_bufs_via_entries);
	/* Assert on mismatched_rows, not delta: two rows wrong in opposite
	 * directions cancel in the delta and read clean.
	 */
	seq_printf(s, "invariant: they must match. delta=%d mismatched_rows=%u %s\n",
		   (int)res.total_entry_rc - (int)res.total_bufs_via_entries,
		   res.mismatches,
		   (res.mismatches == 0 &&
		    res.total_entry_rc == res.total_bufs_via_entries) ?
			"OK" : "MISMATCH");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mla_section_dedup);

/*
 * heap_allocated spans every consumer of the heap; imported_to_mla only what a
 * client mapped for the device. A gap is expected while a model loads, so
 * sample over time rather than reading once. heap_allocated is a floor: this
 * device's direct dma_alloc_* also draws on the region, bypassing the heap.
 */
static int mla_pool_usage_show(struct seq_file *s, void *unused)
{
	struct mla_device *mdev = s->private;
	u64 dms = dma_heap_allocated_bytes(mdev->dms_heap);
	u64 fallback = mdev->heap != mdev->dms_heap ?
		       dma_heap_allocated_bytes(mdev->heap) : 0;
	u64 occupied = dms + fallback;
	s64 imported = atomic64_read(&mdev->imported_heap_bytes);
	s64 internal = atomic64_read(&mdev->outstanding_alloc_bytes);

	seq_printf(s, "allocator         %s\n",
		   mdev->heap ? dma_heap_get_name(mdev->heap) : "<none>");
	seq_printf(s, "region            %s\n",
		   mdev->pool_name ? mdev->pool_name : "<none>");
	seq_printf(s, "backing           %s\n",
		   mdev->reserved_mem_attached ? "reusable CMA" : "no-map carveout");
	seq_printf(s, "region_bytes      %llu\n", mdev->pool_size);
	seq_printf(s, "heap_allocated    %llu\n", occupied);
	seq_printf(s, "imported_to_mla   %lld\n", imported);
	seq_printf(s, "driver_internal   %lld\n", internal);
	seq_printf(s, "unmapped_by_mla   %lld\n",
		   (s64)occupied - imported - internal);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mla_pool_usage);

/* Prints both contexts_open and num_contexts because they measure different
 * things: open are active fds, alive includes contexts still draining after
 * close.
 */
static int mla_contexts_show(struct seq_file *s, void *unused)
{
	struct mla_device *mdev = s->private;
	struct mla_context *ctx;
	unsigned int ctx_idx = 0;

	seq_printf(s, "=== contexts (%d open, %d alive) ===\n",
		   atomic_read(&mdev->contexts_open),
		   atomic_read(&mdev->num_contexts));

	mutex_lock(&mdev->ctx_list_lock);
	list_for_each_entry(ctx, &mdev->contexts, node) {
		struct ctx_totals t;

		ctx_scan(ctx, &t);
		/* prio is (group, intra) as fed to mla_sched_prio_key(); key is
		 * the flattened value the plist actually orders on. Shown per
		 * ctx because "why did the high-priority workload not run
		 * first?" needs every context's priority at once — which a
		 * per-process MLA_OP_INFO read-back cannot give you.
		 */
		seq_printf(s,
			   "ctx[%u] pid=%d prio=(%d,%d) key=%d bufs: pool=%u dedup_primary=%u dedup_alias=%u imported=%u phys=%u other=%u sum_rc=%lu\n",
			   ctx_idx++, ctx->creator_pid,
			   ctx->group_prio, ctx->intra_group_prio,
			   ctx->group_prio * MLA_PRIO_GROUP_SHIFT +
				ctx->intra_group_prio,
			   t.n_pool, t.n_dedup_primary, t.n_dedup_alias,
			   t.n_imported, t.n_phys, t.n_other, t.sum_rc);
	}
	mutex_unlock(&mdev->ctx_list_lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mla_contexts);

/* ---------------------------------------------------------------------- *
 *  counters/ — one integer per file
 *
 *  One open, one read, no parsing. Deliberately NOT exported: any derived
 *  value (delta, hit_rate, etc.). Export raw counters; let userspace do
 *  arithmetic and avoid precision loss and invented names for things the
 *  caller can trivially form.
 * ----------------------------------------------------------------------
 */

/* include/linux/debugfs.h has no atomic64 counterpart to
 * debugfs_create_atomic_t, so every atomic64_t counter shares this one
 * read-only attribute rather than getting a getter each.
 *
 * "%lld" also serves the signed outstanding_alloc_bytes: simple_attr_read()
 * formats with scnprintf(.., attr->fmt, (unsigned long long)val), so a
 * negative value round-trips through the u64 and still prints signed.
 */
static int mla_dbg_atomic64_get(void *data, u64 *val)
{
	*val = (u64)atomic64_read((atomic64_t *)data);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mla_dbg_atomic64_fops, mla_dbg_atomic64_get, NULL,
			 "%lld\n");

static void dedup_count_cb(struct mla_section_dedup_entry *e,
			   unsigned int refcount, void *arg)
{
	unsigned int *n = arg;

	(*n)++;
}

/* Entry count only; takes the cheap path without the ~256 KB row snapshot.
 * Do NOT reroute through mla_dedup_scan() — that would allocate per read
 * for a number that does not need it.
 */
static int mla_dbg_dedup_entries_get(void *data, u64 *val)
{
	struct mla_device *mdev = data;
	unsigned int n = 0;

	mla_section_dedup_walk(mdev, dedup_count_cb, &n);
	*val = n;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mla_dbg_dedup_entries_fops, mla_dbg_dedup_entries_get,
			 NULL, "%llu\n");

/* Rows that individually fail entry->refcount == (buffers pointing at that
 * entry). Needs the full walk plus the per-context scan, so unlike the other
 * counters this one is not free to read.
 */
static int mla_dbg_dedup_mismatches_get(void *data, u64 *val)
{
	struct dedup_scan_result res;
	int err;

	err = mla_dedup_scan(data, NULL, &res);
	if (err)
		return err;

	*val = res.mismatches;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mla_dbg_dedup_mismatches_fops,
			 mla_dbg_dedup_mismatches_get, NULL, "%llu\n");

/* Entries the snapshot could not hold (MLA_DUMP_MAX_ROWS). Exported so a
 * dedup_mismatches read of 0 cannot be mistaken for "clean" when it
 * actually means "we did not look at all of it" — assert this is 0
 * alongside any mismatch assertion.
 */
static int mla_dbg_dedup_truncated_get(void *data, u64 *val)
{
	struct dedup_scan_result res;
	int err;

	err = mla_dedup_scan(data, NULL, &res);
	if (err)
		return err;

	*val = res.truncated;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mla_dbg_dedup_truncated_fops,
			 mla_dbg_dedup_truncated_get, NULL, "%llu\n");

static void mla_debugfs_counters_init(struct mla_device *mdev,
				      struct dentry *parent)
{
	struct dentry *d;

	d = debugfs_create_dir("counters", parent);
	if (IS_ERR(d))
		return;

	debugfs_create_file_unsafe("buf_inc_ops", 0444, d,
				   &mdev->buf_inc_ops, &mla_dbg_atomic64_fops);
	debugfs_create_file_unsafe("buf_dec_ops", 0444, d,
				   &mdev->buf_dec_ops, &mla_dbg_atomic64_fops);
	debugfs_create_file_unsafe("job_inc_ops", 0444, d,
				   &mdev->job_inc_ops, &mla_dbg_atomic64_fops);
	debugfs_create_file_unsafe("job_dec_ops", 0444, d,
				   &mdev->job_dec_ops, &mla_dbg_atomic64_fops);
	debugfs_create_file_unsafe("outstanding_alloc_bytes", 0444, d,
				   &mdev->outstanding_alloc_bytes,
				   &mla_dbg_atomic64_fops);
	/* Driver-internal only; see pool_usage for total occupancy. */
	debugfs_create_file_unsafe("imported_heap_bytes", 0444, d,
				   &mdev->imported_heap_bytes,
				   &mla_dbg_atomic64_fops);

	debugfs_create_atomic_t("prep_claimed", 0444, d, &mdev->prep_claimed);
	debugfs_create_atomic_t("prep_inline", 0444, d, &mdev->prep_inline);
	debugfs_create_atomic_t("dispatch_bank_draining", 0444, d,
				&mdev->dispatch_bank_draining);
	debugfs_create_atomic_t("dispatch_stole_preparing", 0444, d,
				&mdev->dispatch_stole_preparing);
	debugfs_create_atomic_t("prep_stolen", 0444, d, &mdev->prep_stolen);
	debugfs_create_atomic_t("prep_aborted", 0444, d, &mdev->prep_aborted);
	debugfs_create_atomic_t("dispatch_bank_not_ready", 0444, d,
				&mdev->dispatch_bank_not_ready);
	debugfs_create_atomic_t("dispatch_bnr_fatal", 0444, d,
				&mdev->dispatch_bnr_fatal);
	debugfs_create_atomic_t("dispatch_bnr_unexplained", 0444, d,
				&mdev->dispatch_bnr_unexplained);
	debugfs_create_atomic_t("dispatch_bnr_was_active", 0444, d,
				&mdev->dispatch_bnr_was_active);
	debugfs_create_atomic_t("reset_banks_reclaimed", 0444, d,
				&mdev->reset_banks_reclaimed);
	debugfs_create_atomic_t("reset_bank_had_writer", 0444, d,
				&mdev->reset_bank_had_writer);
	debugfs_create_atomic_t("dispatch_cancelled_before_go", 0444, d,
				&mdev->dispatch_cancelled_before_go);
	debugfs_create_atomic_t("dispatch_no_free_bank", 0444, d,
				&mdev->dispatch_no_free_bank);
	debugfs_create_atomic_t("dispatch_inline_claim_lost", 0444, d,
				&mdev->dispatch_inline_claim_lost);
	debugfs_create_atomic_t("prep_failures", 0444, d, &mdev->prep_failures);
	debugfs_create_file_unsafe("foreign_imports_no_cpu_ops", 0444, d,
				   &mdev->foreign_imports_no_cpu_ops,
				   &mla_dbg_atomic64_fops);
	debugfs_create_atomic_t("dispatch_refused_fatal", 0444, d,
				&mdev->dispatch_refused_fatal);
	debugfs_create_atomic_t("inject_no_completes", 0444, d,
				&mdev->inject_no_completes);
	debugfs_create_atomic_t("burst_reprograms", 0444, d,
				&mdev->burst_reprograms);
	debugfs_create_atomic_t("burst_resets", 0444, d, &mdev->burst_resets);
	debugfs_create_atomic_t("irq_masks_repaired", 0444, d,
				&mdev->irq_masks_repaired);

	/* LoRA reloc ports. reloc_patch_passes proves an adapter was actually
	 * applied: a run claiming success with this at 0 applied nothing.
	 */
	debugfs_create_atomic_t("reloc_binds", 0444, d, &mdev->reloc_binds);
	debugfs_create_atomic_t("reloc_ports_bound", 0444, d,
				&mdev->reloc_ports_bound);
	debugfs_create_atomic_t("reloc_patch_passes", 0444, d,
				&mdev->reloc_patch_passes);
	debugfs_create_atomic_t("reloc_submits_refused", 0444, d,
				&mdev->reloc_submits_refused);
	debugfs_create_atomic_t("reloc_update_waits", 0444, d,
				&mdev->reloc_update_waits);
	debugfs_create_atomic_t("reloc_update_wait_timeouts", 0444, d,
				&mdev->reloc_update_wait_timeouts);

	/* Both context populations — see the comment on num_contexts in
	 * mla_drv.h. A test asking "is this device idle?" wants
	 * contexts_open; one checking for leaked context references wants
	 * num_contexts.
	 */
	debugfs_create_atomic_t("num_contexts", 0444, d, &mdev->num_contexts);
	debugfs_create_atomic_t("contexts_open", 0444, d, &mdev->contexts_open);

	/* Machine-readable fatal state, so a test does not have to scrape
	 * dmesg for "recovery budget exceeded; device fatal". Both are read
	 * without sched_lock: a torn read is impossible for a naturally
	 * aligned u32/bool on this architecture, and a diagnostic counter
	 * does not justify serialising the fault path. device_fatal reads as
	 * "Y"/"N", not 0/1 — debugfs_create_bool's format.
	 */
	debugfs_create_u32("recoveries", 0444, d, &mdev->recovery_count);
	debugfs_create_bool("device_fatal", 0444, d, &mdev->device_fatal);

	debugfs_create_file_unsafe("dedup_entries", 0444, d, mdev,
				   &mla_dbg_dedup_entries_fops);
	debugfs_create_file_unsafe("dedup_mismatches", 0444, d, mdev,
				   &mla_dbg_dedup_mismatches_fops);
	debugfs_create_file_unsafe("dedup_truncated", 0444, d, mdev,
				   &mla_dbg_dedup_truncated_fops);
}

void mla_debugfs_init(struct mla_device *mdev)
{
	mdev->debugfs_root = debugfs_create_dir("mla", NULL);
	if (IS_ERR(mdev->debugfs_root)) {
		mdev->debugfs_root = NULL;
		return;
	}

	debugfs_create_file("section_dedup", 0444, mdev->debugfs_root, mdev,
			    &mla_section_dedup_fops);
	debugfs_create_file("contexts", 0444, mdev->debugfs_root, mdev,
			    &mla_contexts_fops);
	debugfs_create_file("pool_usage", 0444, mdev->debugfs_root, mdev,
			    &mla_pool_usage_fops);

	/* No matching teardown: mla_debugfs_exit's debugfs_remove_recursive
	 * on the root takes the subtree with it.
	 */
	mla_debugfs_counters_init(mdev, mdev->debugfs_root);
}

void mla_debugfs_exit(struct mla_device *mdev)
{
	debugfs_remove_recursive(mdev->debugfs_root);
	mdev->debugfs_root = NULL;
}

#else /* !CONFIG_DEBUG_FS */

void mla_debugfs_init(struct mla_device *mdev)  { }
void mla_debugfs_exit(struct mla_device *mdev)  { }

#endif
