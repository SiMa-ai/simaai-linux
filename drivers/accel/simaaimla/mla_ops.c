// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 SiMa.ai
//
// MLA op-handler surface. One file for the four op families whose
// entries the uring_cmd dispatcher in mla_drv.c calls into:
//
//   MLA_OP_INFO / MLA_OP_CTX_CONFIGURE   — pure-software ops
//   MLA_OP_MODEL_DEFINE_V3               — structured model ingest
//   MLA_OP_MODEL_RELEASE                 — model teardown
//   MLA_OP_JOB_SUBMIT / MLA_OP_JOB_CANCEL — runtime dispatch
//
// Buffer ops (MLA_OP_BUF_*) live in mla_ops_buf.c — that file is really
// the buffer registry module and its op handlers are one facet of it,
// so it stays separate. Everything here is per-op logic that reads
// user-space input, walks registries built elsewhere, and drives the
// scheduler / hardware programmer.
//
// See docs/DESIGN.md §6.4 and §7.6 for the structured-input ABI.

#include <linux/bitmap.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/iosys-map.h>
#include <linux/jiffies.h>
#include <linux/overflow.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/xarray.h>

#include "mla_drv.h"
#include "mla_image_fmt.h"
#include "mla_regs.h"
#include "mla_prepared_banks.h"

/* ====================================================================== *
 *  MLA_OP_INFO
 * ======================================================================
 */

/**
 * mla_op_info() — handle MLA_OP_INFO
 * @ctx: context the op is being invoked on
 * @sqe: io_uring SQE inline payload (16/80 bytes)
 * @hdr: already-copied request header
 *
 * Reports device identity (soc_family, tile grid, DMA-unit count),
 * resource limits, alignment requirements, and the priority range
 * accepted by MLA_OP_CTX_CONFIGURE. Used by libraries probing
 * capability before issuing other ops.
 *
 * Return: 0 on success; -EINVAL if request shape is malformed;
 * -EFAULT on user-pointer copy failures.
 */
int mla_op_info(struct mla_context *ctx,
		const struct mla_uring_cmd_sqe *sqe,
		const struct mla_cmd_hdr *hdr)
{
	struct mla_info_req  req;
	struct mla_info_resp resp;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;

	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;

	if (!req.out_ptr || req.out_len == 0)
		return -EINVAL;

	memset(&resp, 0, sizeof(resp));
	resp.abi_version              = MLA_ABI_VERSION;
	resp.driver_version           = MLA_DRV_VERSION;
	resp.soc_family               = ctx->mdev->soc_family;
	resp.tile_rows                = MLA_TILE_ROWS;
	resp.tile_cols                = MLA_TILE_COLS;
	resp.num_dma_units            = MLA_NUM_DMA_UNITS;
	resp.max_contexts             = MLA_MAX_CONTEXTS;
	resp.max_models_per_ctx       = MLA_MAX_MODELS_PER_CTX;
	resp.max_inflight_jobs_per_ctx = MLA_MAX_INFLIGHT_PER_CTX;
	resp.model_image_alignment    = MLA_DEFAULT_ALIGN;
	resp.ifm_alignment            = MLA_DEFAULT_ALIGN;
	resp.ofm_alignment            = MLA_DEFAULT_ALIGN;
	resp.max_buffer_size          = MLA_DEFAULT_MAX_BUFFER;
	resp.dma_mask                 = DMA_BIT_MASK(40);
	/* MLA_OP_MODEL_DEFINE (op 13) is retired and answers -EOPNOTSUPP;
	 * it never had a capability bit to clear. Bits 4 and 5 are reserved.
	 *
	 * MLA_CAP_SECTION_ALLOC (bit 6): SECTION_ALLOC_ABI.md Phase 1 —
	 * MLA_OP_SECTION_ALLOC / MLA_OP_SECTION_COMMIT are available.
	 *
	 * MLA_CAP_MODEL_DEFINE_V3 (bit 7): SECTION_ALLOC_ABI.md §4.3 —
	 * MLA_OP_MODEL_DEFINE_V3 accepts pre-allocated section_ids[]
	 * (from prior SECTION_ALLOC + SECTION_COMMIT calls) instead of
	 * an image_fd + section_directive[] shape. mla-rt gates its
	 * SECTION_ALLOC-driven load path on both this bit and
	 * MLA_CAP_SECTION_ALLOC.
	 *
	 * MLA_CAP_JOB_ASYNC (bit 8): MLA_OP_JOB_ASYNC is available — the
	 * pure-async twin of JOB_SUBMIT. Batching callers switch to
	 * JOB_ASYNC to avoid parking an io_wq worker per outstanding job.
	 *
	 * MLA_CAP_JOB_CANCEL (bit 9): JOB_ASYNC publishes resp.job_id at
	 * submit, so MLA_OP_JOB_CANCEL can name a still-queued job.
	 *
	 * MLA_CAP_DEVICE_RESET (bit 10): MLA_OP_DEVICE_RESET is available —
	 * clears device_fatal, the recovery budget and every ctx->fatal
	 * without a module reload. Gate on this bit rather than probing the
	 * op: a driver without it answers -EOPNOTSUPP, which is
	 * indistinguishable from a reset that failed.
	 *
	 * MLA_CAP_PERFCTR_CONFIG (bit 11): MLA_OP_PERFCTR_CONFIG is
	 * available, so perf_ctr_us[] is caller-selected and opt-in. Gate on
	 * this bit, never on a zero-valued field: 0 is both a legal signal id
	 * and a legal counter reading. A driver WITHOUT this bit populates
	 * perf_ctr_us[] with the hardwired {tiled_lat, tiles_active,
	 * iq_sync_freeze, d_sync_freeze} on every job; a driver WITH it
	 * returns zeros until asked. Same information, opposite defaults, so
	 * a caller that cares must check.
	 *
	 * MLA_CAP_RELOC_UPDATE (bit 13): MODEL_DEFINE_V3 accepts
	 * MLA_RELOC_ZONE_RELOC entries, and MLA_OP_MODEL_UPDATE_RELOC can
	 * re-point a loaded model's LoRA ports. A model that declares ports
	 * must also supply their default bindings, so gating on this bit is
	 * also how a caller knows the define request's reloc tail is read.
	 *
	 * MLA_CAP_RELOC_PER_JOB (bit 14) is deliberately NOT set: a submit
	 * request cannot carry its own reloc bindings.
	 *
	 * MLA_CAP_PREPARED_BANKS: a nonzero DEFINE_V3 prepared_bank_count
	 * provisions backing before model publication and forbids hot growth.
	 * max_prepared_banks reports the accepted upper bound.
	 */
	resp.capabilities             = MLA_CAP_SECTION_ALLOC |
					MLA_CAP_MODEL_DEFINE_V3 |
					MLA_CAP_JOB_ASYNC |
					MLA_CAP_JOB_CANCEL |
					MLA_CAP_DEVICE_RESET |
					MLA_CAP_PERFCTR_CONFIG |
					MLA_CAP_RELOC_UPDATE |
					MLA_CAP_PREPARED_BANKS;
	resp.max_prepared_banks       = MLA_MAX_BANKS;
	resp.priority_lo              = -20;
	resp.priority_hi              =  19;
	resp.mla_freq_mhz             = ctx->mdev->mla_mhz;
	resp.outstanding_alloc_bytes  =
		(u64)atomic64_read(&ctx->mdev->outstanding_alloc_bytes);

	/* This context's live priority (MLA_DRV_VERSION 3). Under ctx->lock
	 * because that is what mla_op_ctx_configure() writes them under —
	 * so a concurrent reconfigure cannot be observed half-applied.
	 */
	mutex_lock(&ctx->lock);
	resp.cur_group_prio           = ctx->group_prio;
	resp.cur_intra_group_prio     = ctx->intra_group_prio;
	mutex_unlock(&ctx->lock);

	/* Additive-field forward compat: copy only what the caller asked
	 * for. Old callers (out_len < sizeof(resp)) get the leading
	 * fields; new callers get everything.
	 */
	{
		size_t n = min_t(size_t, (size_t)req.out_len, sizeof(resp));

		if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, n))
			return -EFAULT;
	}

	return 0;
}

/* ====================================================================== *
 *  MLA_OP_CTX_CONFIGURE
 *
 *  Applies whichever fields are present in req.fields. Capability checks
 *  gate the privileged transitions.
 * ======================================================================
 */

/*
 * Permission gate for context-kind transitions. CONTAINER and
 * VM_GUEST are reserved for cgroup integration / vhost-mla, neither
 * of which this driver implements; only HOST_PROCESS is allowed.
 */
static bool mla_can_set_kind(u32 kind)
{
	return kind == MLA_CTX_HOST_PROCESS;
}

/**
 * mla_op_ctx_configure() — handle MLA_OP_CTX_CONFIGURE
 * @ctx: context to reconfigure
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Applies whichever fields are set in req.fields:
 *   GROUP_PRIO/INTRA_PRIO  — scheduler priority tuple, [-20..19]
 *   WEIGHT                 — round-robin weight, [1..65535]
 *   REALTIME               — RT class flag (CAP_SYS_NICE-gated)
 *   KIND                   — context kind (HOST_PROCESS only)
 *   VM_TOKEN               — reserved for vhost-mla; rejected
 *
 * Raising priority (more negative number) and toggling REALTIME
 * require CAP_SYS_NICE; lowering priority is unprivileged.
 *
 * Return: 0 on success; -EINVAL on out-of-range values; -EPERM on
 * privilege violations; -EFAULT on copy failures.
 */
int mla_op_ctx_configure(struct mla_context *ctx,
			 const struct mla_uring_cmd_sqe *sqe,
			 const struct mla_cmd_hdr *hdr)
{
	struct mla_ctx_configure_req req;
	int ret = 0;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;

	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;

	mutex_lock(&ctx->lock);

	if (req.fields & MLA_CFG_GROUP_PRIO) {
		if (req.group_prio < -20 || req.group_prio > 19) {
			ret = -EINVAL;
			goto out;
		}
		/* Raising priority (more negative) requires CAP_SYS_NICE. */
		if (req.group_prio < ctx->group_prio && !capable(CAP_SYS_NICE)) {
			ret = -EPERM;
			goto out;
		}
		ctx->group_prio = req.group_prio;
	}

	if (req.fields & MLA_CFG_INTRA_PRIO) {
		if (req.intra_group_prio < -20 || req.intra_group_prio > 19) {
			ret = -EINVAL;
			goto out;
		}
		ctx->intra_group_prio = req.intra_group_prio;
	}

	if (req.fields & MLA_CFG_WEIGHT) {
		if (req.weight == 0 || req.weight > 65535) {
			ret = -EINVAL;
			goto out;
		}
		ctx->weight = req.weight;
	}

	if (req.fields & MLA_CFG_REALTIME) {
		if (req.realtime && !capable(CAP_SYS_NICE)) {
			ret = -EPERM;
			goto out;
		}
		ctx->realtime = !!req.realtime;
	}

	if (req.fields & MLA_CFG_KIND) {
		if (!mla_can_set_kind(req.kind)) {
			ret = -EPERM;
			goto out;
		}
		ctx->kind = req.kind;
	}

	/* MLA_CFG_VM_TOKEN names a vhost-mla guest, which this driver does
	 * not support; reject explicitly rather than silently ignore it.
	 */
	if (req.fields & MLA_CFG_VM_TOKEN) {
		ret = -EPERM;
		goto out;
	}

out:
	mutex_unlock(&ctx->lock);
	return ret;
}

/* ====================================================================== *
 *  Structured model-load handling, shared by MODEL_DEFINE_V3.
 *  (Legacy MLA_OP_MODEL_DEFINE, op 13, is retired.)
 *
 *  User-space (mla-rt) emits typed input from the ELF; there is no
 *  m4ctrl synthesis:
 *    - mla_section_directive[]   — per-section payload + role + sha256
 *    - mla_tile_desc[]           — per-tile MIM register + INSTR slice
 *    - mla_dma_unit_desc[]       — per-DMA-unit configuration
 *    - mla_reloc_entry[]         — symbolic descriptor refs
 *
 *  The kernel allocates each section from its DRAM pool, resolves all
 *  SECTION-tagged reloc entries here at load time (sections are
 *  model-static), and registers an mla_model that the per-job path
 *  uses at JOB_SUBMIT. IFM/OFM/RELOC-tagged reloc entries are resolved
 *  per-job by the patcher in mla_hw.c.
 *
 *  See docs/DESIGN.md §6.4 and §7.6 for the design; this file is the
 *  authoritative source for the on-wire ABI shape.
 * ======================================================================
 */

/* ---- Per-section helpers ---------------------------------------------- *
 *  Called from the MODEL_DEFINE loader below as well as any future
 *  direct-callers of individual sections.
 */

/* Build a fresh alias mla_buffer that shares @primary's pool memory.
 * The alias gets registered in @ctx's xarray with its own
 * internal_buf_id, but its drop path returns memory ownership to the
 * dedup entry (not mla_pool_free). Caller has already taken one
 * reference on @entry; on success that reference is owned by the new
 * alias buffer. On failure, this function drops it.
 */
int mla_section_alloc_alias(struct mla_context *ctx,
			    struct mla_section_dedup_entry *entry,
			    struct mla_buffer **out_buf)
{
	struct mla_buffer *primary = mla_section_dedup_entry_buf(entry);
	struct mla_buffer *alias;
	int ret;

	if (!primary) {
		mla_section_dedup_put(ctx->mdev, entry);
		return -EINVAL;
	}

	alias = kzalloc(sizeof(*alias), GFP_KERNEL);
	if (!alias) {
		mla_section_dedup_put(ctx->mdev, entry);
		return -ENOMEM;
	}
	refcount_set(&alias->refcount, 1);
	atomic64_inc(&ctx->mdev->buf_inc_ops);
	alias->size      = primary->size;
	alias->alignment = PAGE_SIZE;
	alias->cpu_vaddr = primary->cpu_vaddr;
	alias->dma_addr  = primary->dma_addr;
	alias->dedup     = entry;
	alias->dedup_alias = true;

	ret = mla_buffer_register(ctx, alias);
	if (ret < 0) {
		kfree(alias);
		mla_section_dedup_put(ctx->mdev, entry);
		return ret;
	}
	*out_buf = alias;
	return 0;
}

/*
 * v2_validate_tile_desc / _dma_unit_desc / _reloc_entry:
 *  Bounds-check a tile/DMA-unit/reloc descriptor against a resolved
 *  section table. MODEL_DEFINE_V3 is the only caller.
 *
 *  section_sizes[]: mandatory. section_sizes[i] is the payload size
 *      (bytes) of the i'th resolved section — section_bufs[i]->size.
 *  section_roles[]: optional; V3 always passes NULL, since
 *      SECTION_ALLOC_ABI.md §3.5 has no role field. Kept so the
 *      instr_section_index == MLA_SECTION_ROLE_INSTR enforcement can
 *      be reinstated if a role column returns.
 */
static int v2_validate_tile_desc(const struct mla_tile_desc *t,
				 u32 num_sections,
				 const u64 *section_sizes,
				 const u32 *section_roles)
{
	u64 instr_sz;
	u32 axi_off;
	int k;

	if (t->tile_index >= MLA_MAX_ACTIVE_TILES)
		return -EINVAL;
	if (t->quadgroup >= 4)
		return -EINVAL;
	if (t->instr_section_index >= num_sections)
		return -EINVAL;
	instr_sz = section_sizes[t->instr_section_index];
	if (section_roles &&
	    section_roles[t->instr_section_index] != MLA_SECTION_ROLE_INSTR)
		return -EINVAL;
	if ((u64)t->instr_offset + (u64)t->instr_size > instr_sz)
		return -EINVAL;

	/* MIM base reg: APB-space addr; subtract MLA_APB_BASE to get
	 * AXI offset; validate it (and the 4 bytes we'd write at it)
	 * lies inside the MLA AXI window.
	 */
	if (t->mim_base_reg < MLA_APB_BASE)
		return -EINVAL;
	axi_off = t->mim_base_reg - MLA_APB_BASE;
	if (axi_off + 4 > MLA_REGS_SIZE)
		return -EINVAL;

	for (k = 0; k < (int)ARRAY_SIZE(t->reserved); k++)
		if (t->reserved[k])
			return -EINVAL;
	return 0;
}

static int v2_validate_dma_unit_desc(const struct mla_dma_unit_desc *u,
				     u32 num_sections,
				     const u64 *section_sizes)
{
	u64 end;

	if (u->unit_index >= MLA_DMA_UNIT_COUNT_SIZE)
		return -EINVAL;
	if (u->reserved || u->reserved2)
		return -EINVAL;
	if (u->desc_section_index >= num_sections)
		return -EINVAL;
	end = (u64)u->desc_blob_offset + (u64)u->desc_count * 32u;
	if (end > section_sizes[u->desc_section_index])
		return -EINVAL;
	/* Caller picks dfm_fetch_map_type 1=RO, 2=WO, 3=mixed. Reject
	 * anything else — the hardware would silently do something
	 * wrong otherwise.
	 *
	 * An EMPTY unit is the exception: with desc_count == 0 there is
	 * nothing to fetch, and type 0 is what both userspace paths supply
	 * for it and what mla_hw_init_dfm() wants (it programs
	 * dfm2 = (type << 1) | (desc_count << 12), so 0/0 writes a zeroed
	 * "fetches nothing" config). Demanding 1..3 unconditionally would
	 * reject any model that leaves a DMA unit idle.
	 */
	if (u->dfm_fetch_map_type > 3)
		return -EINVAL;
	if (u->desc_count && u->dfm_fetch_map_type == 0)
		return -EINVAL;
	return 0;
}

static int v2_validate_reloc_entry(const struct mla_reloc_entry *r,
				   u32 num_sections,
				   u32 num_inputs, u32 num_outputs,
				   u32 num_reloc_ports)
{
	u64 desc_off;

	if (r->zone_tag >= MLA_RELOC_ZONE_MAX)
		return -EINVAL;

	switch (r->zone_tag) {
	case MLA_RELOC_ZONE_IFM:
		if (r->zone_index >= num_inputs)
			return -EINVAL;
		break;
	case MLA_RELOC_ZONE_OFM:
		if (r->zone_index >= num_outputs)
			return -EINVAL;
		break;
	case MLA_RELOC_ZONE_RELOC:
		/* A LoRA port index. Every port is bound at define to the
		 * section the ELF shipped (reloc_defaults), so the descriptor
		 * is spliced before the model can be submitted and the caller
		 * can never reach a state where a placeholder address is
		 * dispatched. See mla_op_model_define_v3's reloc block.
		 */
		if (r->zone_index >= num_reloc_ports)
			return -EINVAL;
		break;
	case MLA_RELOC_ZONE_SECTION:
		if (r->zone_index >= num_sections)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	/* Bounds-check that the descriptor we're about to patch lies
	 * inside *some* section. We don't yet know which section holds
	 * the descriptor blob — that's via dma_unit_desc[].
	 * desc_section_index. The descriptor patcher will catch
	 * out-of-bounds at apply time. We only sanity-check here that
	 * dma_offset + desc_index*32 doesn't overflow.
	 */
	desc_off = (u64)r->dma_offset + (u64)r->desc_index * 32u;
	if (desc_off + 32 < desc_off)            /* overflow */
		return -EINVAL;

	return 0;
}


#define MLA_CACHE_LINE_BYTES  64u

/* ---- Static-reloc resolution at MODEL_DEFINE time --------------------- *
 *
 *  When user-space supplies a static_reloc_count > 0, the leading
 *  static_reloc_count entries are all SECTION-tagged (caller's
 *  guarantee, re-validated here) and their target section phys is
 *  invariant across jobs. Splice them into the descriptor blob right
 *  here so the per-job patcher only needs to revisit IFM/OFM/RELOC.
 *
 *  Wire-level: identical to mla_hw_apply_relocs's SECTION case.
 *  Bounds were already validated by v2_validate_reloc_entry. Anything
 *  unexpected (non-SECTION in the static region, missing section,
 *  off-by-one) returns -EINVAL.
 */

/* MLA_DESC_* descriptor-field macros live in mla_regs.h. */

static int v2_apply_static_section_relocs(struct mla_model *m,
					  const struct mla_dma_unit_desc *dma_units)
{
	struct mla_buffer *blob_buf;
	void *base;
	size_t blob_size;
	size_t dirty_lo = SIZE_MAX;
	size_t dirty_hi = 0;
	u32 i;

	if (!m->static_reloc_count)
		return 0;
	if (!m->reloc_list || !m->section_bufs)
		return -EFAULT;

	if (dma_units[0].desc_section_index >= m->num_sections)
		return -EINVAL;
	blob_buf = m->section_bufs[dma_units[0].desc_section_index];
	if (!blob_buf || !blob_buf->cpu_vaddr)
		return -EFAULT;
	base      = blob_buf->cpu_vaddr;
	blob_size = blob_buf->size;

	for (i = 0; i < m->static_reloc_count; i++) {
		const struct mla_reloc_entry *r = &m->reloc_list[i];
		struct mla_buffer *b;
		u64 dram_addr;
		__le64 *qw;
		u64 v;
		size_t off;

		if (r->zone_tag != MLA_RELOC_ZONE_SECTION) {
			pr_warn("%s: static_relocs[%u] tag=%u (expected SECTION); rejecting\n",
				__func__, i, r->zone_tag);
			return -EINVAL;
		}
		if (r->zone_index >= m->num_sections)
			return -EINVAL;
		b = m->section_bufs[r->zone_index];
		if (!b)
			return -EINVAL;
		dram_addr = (u64)b->dma_addr + r->placeholder_offset;
		/* Same truncation hazard as mla_hw_apply_relocs; same rationale
		 * for making it audible rather than silent. WARN_ONCE for
		 * symmetry with that site, even though this one runs in process
		 * context and could return -EINVAL — the two patchers must not
		 * disagree about what an over-range address means.
		 */
		WARN_ONCE(dram_addr > MLA_DESC_DRAM_ADDR_MASK,
			  "mla: static reloc dram_addr 0x%llx exceeds the %u-bit descriptor field; truncating\n",
			  (unsigned long long)dram_addr,
			  (unsigned int)MLA_DESC_DRAM_ADDR_BITS);
		dram_addr &= MLA_DESC_DRAM_ADDR_MASK;

		off = (size_t)r->dma_offset +
		      (size_t)r->desc_index * MLA_DESC_SIZE_BYTES;
		if (off + MLA_DESC_SIZE_BYTES > blob_size) {
			pr_warn("%s: static_relocs[%u] off %zu past blob_size %zu\n",
				__func__, i, off, blob_size);
			return -EINVAL;
		}
		qw = (__le64 *)((u8 *)base + off +
				MLA_DESC_DRAM_ADDR_WORD * 8);
		v = le64_to_cpu(*qw);
		v &= ~(MLA_DESC_DRAM_ADDR_MASK << MLA_DESC_DRAM_ADDR_SHIFT);
		v |= dram_addr << MLA_DESC_DRAM_ADDR_SHIFT;
		*qw = cpu_to_le64(v);
		{
			size_t qw_off = off + MLA_DESC_DRAM_ADDR_WORD * 8;

			if (qw_off < dirty_lo)
				dirty_lo = qw_off;
			if (qw_off + 8 > dirty_hi)
				dirty_hi = qw_off + 8;
		}
	}

	if (mla_debug & MLA_DBG_TRACE)
		dev_info(m->ctx->mdev->dev,
			 "v2_trace: model=\"%s\" MODEL_DEFINE static_relocs resolved count=%u (per-job will only walk %u dynamic)\n",
			 m->name,
			 m->static_reloc_count,
			 m->reloc_count - m->static_reloc_count);

	/* Publish the dirtied range to memory before the first JOB_SUBMIT
	 * reads it. Sync the whole buffer, matching the JOB_SUBMIT
	 * apply_relocs site — a range-scoped flush here is not yet
	 * correctness-safe.
	 *
	 * TODO: replace with a range-scoped flush over [dirty_lo, dirty_hi)
	 * once that is fixed. Keep both sites in sync.
	 */
	(void)dirty_lo; (void)dirty_hi;
	mla_pool_sync_for_device(m->ctx->mdev, blob_buf);
	return 0;
}

/* Defined with the submit path, where its per-job callers live; MODEL_DEFINE
 * needs it too, to resolve the reloc ports' default bindings.
 */
static int mla_resolve_iotensors(struct mla_context *ctx,
				 const struct mla_iotensor *user_arr,
				 u32 count,
				 struct mla_buffer **bufs);

/* ---- LoRA / reloc-port splicing ---------------------------------------- *
 *
 *  RELOC-tagged entries live in the DYNAMIC partition of the reloc list
 *  (mla-rt sorts SECTION-tagged entries first, so they land after
 *  static_reloc_count), but unlike IFM/OFM their address does not change
 *  per job: a port keeps its binding until MLA_OP_MODEL_UPDATE_RELOC
 *  re-points it. So they are spliced HERE — once at MODEL_DEFINE with the
 *  ELF's own default bindings, and again per bank on each update — and the
 *  per-job patcher in mla_hw_apply_relocs() leaves them alone.
 *
 *  Wire-level identical to the other two patchers; the third copy of the
 *  read-modify-write is deliberate rather than factored out, because the
 *  three run under different locking (define: none needed, model not yet
 *  published; update: bank_grow_lock; per-job: softirq), and merging them
 *  risks calling into the wrong context.
 *
 *  @patched_out receives the number of descriptors actually rewritten, which
 *  is NOT the same as "this call returned 0": both early bails below are
 *  successful no-ops. Callers that report a patch count to userspace must
 *  count on this, not on the return value — see the comment on
 *  mla_model_update_reloc_resp.patch_passes.
 */
static int mla_model_patch_reloc_ports(struct mla_model *m, u32 bank_idx,
				       u32 *patched_out)
{
	struct mla_buffer *blob_buf;
	void *base;
	size_t blob_size;
	u32 i, patched = 0;

	*patched_out = 0;

	if (!m->num_reloc_ports || !m->reloc_list || !m->reloc_count)
		return 0;
	if (!m->reloc_bufs || !m->section_bufs || !m->dma_info_typed)
		return -EFAULT;
	if (m->dma_info_typed[0].desc_section_index >= m->num_sections) {
		dev_err(m->ctx->mdev->dev,
			"reloc_ports: desc_section_index %u >= num_sections %u\n",
			m->dma_info_typed[0].desc_section_index, m->num_sections);
		return -EINVAL;
	}

	blob_buf = mla_model_bank_blob(m, (u8)bank_idx);
	if (!blob_buf || !blob_buf->cpu_vaddr)
		return -EFAULT;
	base      = blob_buf->cpu_vaddr;
	blob_size = blob_buf->size;

	for (i = m->static_reloc_count; i < m->reloc_count; i++) {
		const struct mla_reloc_entry *r = &m->reloc_list[i];
		struct mla_buffer *b;
		u64 dram_addr;
		__le64 *qw;
		u64 v;
		size_t off;

		if (r->zone_tag != MLA_RELOC_ZONE_RELOC)
			continue;
		/* Validated at define against num_reloc_ports, and
		 * reloc_bufs[] is never sparse once define succeeded — so
		 * either of these firing means internal corruption, not a
		 * caller error. Check anyway: this walks userspace-supplied
		 * indices into a kernel array.
		 */
		if (r->zone_index >= m->num_reloc_ports ||
		    !m->reloc_bufs[r->zone_index]) {
			dev_err(m->ctx->mdev->dev,
				"reloc_ports[%u]: port %u unusable (num_reloc_ports=%u buf=%p)\n",
				i, r->zone_index, m->num_reloc_ports,
				r->zone_index < m->num_reloc_ports ?
					m->reloc_bufs[r->zone_index] : NULL);
			return -EINVAL;
		}
		b = m->reloc_bufs[r->zone_index];

		dram_addr = (u64)b->dma_addr + r->placeholder_offset;
		if (m->reloc_offsets)
			dram_addr += m->reloc_offsets[r->zone_index];

		/* Same truncation hazard, and the same choice to make it
		 * audible, as the other two patchers.
		 */
		WARN_ONCE(dram_addr > MLA_DESC_DRAM_ADDR_MASK,
			  "mla: reloc-port dram_addr 0x%llx exceeds the %u-bit descriptor field; truncating\n",
			  (unsigned long long)dram_addr,
			  (unsigned int)MLA_DESC_DRAM_ADDR_BITS);
		dram_addr &= MLA_DESC_DRAM_ADDR_MASK;

		off = (size_t)r->dma_offset +
		      (size_t)r->desc_index * MLA_DESC_SIZE_BYTES;
		if (off + MLA_DESC_SIZE_BYTES > blob_size) {
			dev_err(m->ctx->mdev->dev,
				"reloc_ports[%u]: off %zu past blob_size %zu (bank %u)\n",
				i, off, blob_size, bank_idx);
			return -EINVAL;
		}
		qw = (__le64 *)((u8 *)base + off + MLA_DESC_DRAM_ADDR_WORD * 8);
		v = le64_to_cpu(*qw);
		v &= ~(MLA_DESC_DRAM_ADDR_MASK << MLA_DESC_DRAM_ADDR_SHIFT);
		v |= dram_addr << MLA_DESC_DRAM_ADDR_SHIFT;
		*qw = cpu_to_le64(v);
		patched++;
	}

	if (!patched)
		return 0;

	/* Whole-buffer sync, matching what MODEL_DEFINE's static-reloc splice
	 * does: a range-scoped flush here is not yet correctness-safe, and
	 * this path is rare (load, and adapter swap) rather than per job.
	 * The precomputed flush_regions are not usable here anyway — they
	 * cover the union of dynamic-reloc cache lines, a superset, but the
	 * hot path owns that list.
	 */
	mla_pool_sync_for_device(m->ctx->mdev, blob_buf);
	atomic_inc(&m->ctx->mdev->reloc_patch_passes);
	*patched_out = patched;

	if (mla_debug & MLA_DBG_TRACE)
		dev_info(m->ctx->mdev->dev,
			 "v2_trace: model=\"%s\" reloc ports spliced bank=%u descs=%u ports=%u\n",
			 m->name, bank_idx, patched, m->num_reloc_ports);
	return 0;
}

/* ---- Precomputed range-scoped flush regions for the per-job krlc path * *
 *
 *  Each dynamic reloc entry (index [static_reloc_count, reloc_count))
 *  patches exactly one 8-byte descriptor field at qw_off =
 *    r->dma_offset + r->desc_index * MLA_DESC_SIZE_BYTES
 *                                    + MLA_DESC_DRAM_ADDR_WORD * 8
 *
 *  That write dirties one 64-byte cache line (qw_off & ~63). The set of
 *  dirty cache-line offsets is fixed for the model's life — only the
 *  addresses being spliced in change per job. Precomputing sorted,
 *  coalesced runs of adjacent cache lines lets mla_hw_apply_relocs
 *  sync a few bytes on resnet_50 (2 relocs → 1-2 runs × 64 B) instead of
 *  a full multi-MB blob.
 *
 *  Failure to allocate the region array is not fatal: on error we leave
 *  flush_regions=NULL and the hot path falls back to full-buffer sync
 *  (correctness preserved, just slower). Same fallback path is taken
 *  when mla_reloc_range_flush=0 (module-param kill switch).
 */

static int u64_cmp(const void *a, const void *b)
{
	u64 x = *(const u64 *)a;
	u64 y = *(const u64 *)b;

	return (x < y) ? -1 : (x > y);
}

static int v2_build_flush_regions(struct mla_model *m)
{
	u64 *lines;
	u32 dyn_count, uniq = 0;
	u32 i, j;
	size_t desc_blob_size;
	struct mla_buffer *blob_buf;

	m->flush_regions = NULL;
	m->num_flush_regions = 0;

	if (!mla_reloc_range_flush)
		return 0;

	if (!m->reloc_list || m->reloc_count <= m->static_reloc_count)
		return 0;
	dyn_count = m->reloc_count - m->static_reloc_count;

	if (!m->dma_info_typed || !m->section_bufs)
		return 0;
	if (m->dma_info_typed[0].desc_section_index >= m->num_sections)
		return 0;
	blob_buf = m->section_bufs[m->dma_info_typed[0].desc_section_index];
	if (!blob_buf)
		return 0;
	desc_blob_size = blob_buf->size;

	lines = kvmalloc_array(dyn_count, sizeof(*lines), GFP_KERNEL);
	if (!lines)
		return -ENOMEM;

	for (i = m->static_reloc_count, j = 0; i < m->reloc_count; i++) {
		const struct mla_reloc_entry *r = &m->reloc_list[i];
		u64 qw_off;

		qw_off = (u64)r->dma_offset +
			 (u64)r->desc_index * MLA_DESC_SIZE_BYTES +
			 (u64)MLA_DESC_DRAM_ADDR_WORD * 8u;
		if (qw_off + 8 > desc_blob_size)
			continue; /* would fail apply_relocs bounds check */
		lines[j++] = qw_off & ~((u64)MLA_CACHE_LINE_BYTES - 1);
	}
	if (!j) {
		kvfree(lines);
		return 0;
	}

	sort(lines, j, sizeof(*lines), u64_cmp, NULL);

	/* Dedup in-place. */
	uniq = 1;
	for (i = 1; i < j; i++)
		if (lines[i] != lines[uniq - 1])
			lines[uniq++] = lines[i];

	/* Coalesce adjacent cache lines into runs. */
	{
		struct mla_flush_region *regs;
		u32 rcount = 0;

		regs = kcalloc(uniq, sizeof(*regs), GFP_KERNEL);
		if (!regs) {
			kvfree(lines);
			return -ENOMEM;
		}
		regs[0].offset = lines[0];
		regs[0].len    = MLA_CACHE_LINE_BYTES;
		rcount = 1;
		for (i = 1; i < uniq; i++) {
			struct mla_flush_region *tail = &regs[rcount - 1];

			if (lines[i] == tail->offset + tail->len) {
				tail->len += MLA_CACHE_LINE_BYTES;
			} else {
				regs[rcount].offset = lines[i];
				regs[rcount].len    = MLA_CACHE_LINE_BYTES;
				rcount++;
			}
		}
		m->flush_regions     = regs;
		m->num_flush_regions = rcount;
	}
	kvfree(lines);

	if (mla_debug & MLA_DBG_TRACE) {
		u32 k;

		dev_info(m->ctx->mdev->dev,
			 "v2_trace: model=\"%s\" flush_regions built count=%u (from %u dynamic relocs, blob_size=%zu)\n",
			 m->name, m->num_flush_regions, dyn_count,
			 desc_blob_size);
		for (k = 0; k < m->num_flush_regions; k++)
			dev_info(m->ctx->mdev->dev,
				 "v2_trace: model=\"%s\" flush_regions[%u] off=0x%zx len=%zu (line=0x%zx-0x%zx)\n",
				 m->name, k,
				 m->flush_regions[k].offset,
				 m->flush_regions[k].len,
				 m->flush_regions[k].offset,
				 m->flush_regions[k].offset +
				 m->flush_regions[k].len - 1);
	}
	return 0;
}

/* ---- Build mla_model from typed input --------------------------------- */

static struct mla_model *
v2_build_model(struct mla_context *ctx,
	       const struct mla_model_define_req *req,
	       struct mla_buffer **section_bufs,
	       struct mla_tile_desc *tile_descs,
	       struct mla_dma_unit_desc *dma_unit_descs)
{
	struct mla_model *m;
	u32 i;

	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return ERR_PTR(-ENOMEM);

	refcount_set(&m->refcount, 1);
	atomic_set(&m->outstanding, 0);
	init_waitqueue_head(&m->reloc_quiesce_wq);
	mutex_init(&m->bank_grow_lock);
	m->banks = kcalloc(MLA_MAX_BANKS, sizeof(*m->banks), GFP_KERNEL);
	if (!m->banks) {
		kfree(m);
		return ERR_PTR(-ENOMEM);
	}
	/* Bank[0]: sentinel — blob == NULL routes the accessor to
	 * section_bufs[dma_info_typed[0].desc_section_index]. No separate
	 * allocation. State starts FREE for the future state machine.
	 */
	atomic_set(&m->banks[0].state, MLA_BANK_FREE);
	m->num_banks     = 1;
	/* Counted reference: a model outlives the fd whenever an in-flight
	 * job still holds a ref on it (mla_context_models_release documents
	 * exactly that), and mla_model_grow_banks / mla_model_destroy both
	 * dereference m->ctx. Released in mla_model_destroy.
	 */
	m->ctx           = mla_context_get(ctx);
	m->batch_size    = req->batch_size ? req->batch_size : 1;
	m->num_inputs    = req->num_inputs;
	m->num_outputs   = req->num_outputs;
	m->flags         = req->flags;
	memcpy(m->quad_tile_bitmask, req->quad_tile_bitmask,
	       sizeof(m->quad_tile_bitmask));

	m->num_sections      = req->num_sections;
	m->section_bufs      = section_bufs;
	m->tile_info_typed   = tile_descs;
	m->dma_info_typed    = dma_unit_descs;
	m->num_active_tiles  = req->num_tiles;

	/* Total descriptor count = sum across DMA units; informational
	 * to user-space (echoed in resp).
	 */
	m->num_dma_descriptors = 0;
	for (i = 0; i < req->num_dma_units; i++)
		m->num_dma_descriptors += dma_unit_descs[i].desc_count;

	/* Hold an extra refcount on each section's mla_buffer so an
	 * early MLA_OP_BUF_RELEASE on its id can't pull memory out from
	 * under JOB_SUBMIT. mla_model_destroy reverses these.
	 */
	for (i = 0; i < req->num_sections; i++) {
		if (section_bufs[i]) {
			refcount_inc(&section_bufs[i]->refcount);
			atomic64_inc(&ctx->mdev->buf_inc_ops);
		}
	}

	return m;
}


/* ====================================================================== *
 *  MLA_OP_MODEL_DEFINE_V3 — section_ids[]-based MODEL_DEFINE
 *  (SECTION_ALLOC_ABI.md §4.3)
 *
 *  Same downstream flow as V2 (validators, v2_build_model,
 *  apply_static_section_relocs, build_flush_regions, xa_alloc), but
 *  the section prologue is a resolve-section_ids-from-ctx-xarray step
 *  instead of an image_fd + directive-batch loader. Reuses V2 helpers
 *  wherever possible.
 *
 *  Sections must already be SECTION_ALLOC'd + SECTION_COMMIT'd by
 *  the caller. v3_resolve_section_ids validates each ID resolves to
 *  a committed buf and bumps the section_bufs[] refcount that
 *  mla_model_destroy will drop.
 * ======================================================================
 */

static void v3_release_section_refs(struct mla_context *ctx,
				    struct mla_buffer **bufs, u32 n)
{
	u32 i;

	for (i = 0; i < n; i++) {
		struct mla_buffer *b = bufs[i];

		if (!b)
			continue;
		if (refcount_dec_and_test(&b->refcount)) {
			/* Should never happen: the ctx-xarray registration still
			 * holds a ref. Log and forget rather than crash.
			 */
			dev_warn(ctx->mdev->dev,
				 "%s: buf=%p unexpectedly dropped to 0\n",
				 __func__, b);
		}
		atomic64_inc(&ctx->mdev->buf_dec_ops);
	}
}

static int v3_resolve_section_ids(struct mla_context *ctx,
				  const u32 *section_ids, u32 num_sections,
				  struct mla_buffer **bufs,
				  u64 *section_sizes)
{
	u32 i;

	for (i = 0; i < num_sections; i++) {
		struct mla_buffer *b = mla_buffer_find(ctx, section_ids[i]);

		if (!b) {
			v3_release_section_refs(ctx, bufs, i);
			return -ENOENT;
		}
		/* Must be a SECTION_ALLOC-produced dma-buf-backed buffer
		 * that has been committed, OR a dedup-alias whose primary
		 * was committed (aliases inherit committedness implicitly).
		 * Anything else is a caller error (attaching an ATTACH_PHYS
		 * or IMPORT'd id, for example).
		 */
		if (b->imported || b->phys_attached || b->pool_owned) {
			v3_release_section_refs(ctx, bufs, i);
			return -EINVAL;
		}
		if (!b->committed && !b->dedup_alias) {
			v3_release_section_refs(ctx, bufs, i);
			return -EBADF;
		}
		refcount_inc(&b->refcount);
		atomic64_inc(&ctx->mdev->buf_inc_ops);
		bufs[i]          = b;
		section_sizes[i] = b->size;
	}
	return 0;
}

int mla_op_model_define_v3(struct mla_context *ctx,
			   const struct mla_uring_cmd_sqe *sqe,
			   const struct mla_cmd_hdr *hdr_unused)
{
	struct mla_model_define_v3_req    req;
	struct mla_model_define_v3_resp   resp;
	struct mla_model_define_req       shim_req;   /* for v2_build_model */
	u32                              *section_ids   = NULL;
	struct mla_buffer               **bufs          = NULL;
	struct mla_tile_desc             *tile_descs    = NULL;
	struct mla_dma_unit_desc         *dma_descs     = NULL;
	u64                              *section_sizes = NULL;
	struct mla_model                 *m             = NULL;
	u32 id, i;
	int ret;

	(void)hdr_unused;

	/* Additive tail (MLA_DRV_VERSION 7): accept anything from the original
	 * struct — which ended at name[] — upward, zero-filling what the caller
	 * did not send and ignoring what this kernel does not know. This lets a
	 * new submit-time field grow the struct without becoming a whole new op.
	 */
	if (sqe->req_len < offsetofend(struct mla_model_define_v3_req, name)) {
		dev_err(ctx->mdev->dev,
			"MODEL_DEFINE_V3: req_len=%u below the minimum %zu\n",
			sqe->req_len,
			offsetofend(struct mla_model_define_v3_req, name));
		return -EINVAL;
	}
	memset(&req, 0, sizeof(req));
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr),
			   min_t(size_t, (size_t)sqe->req_len, sizeof(req))))
		return -EFAULT;

	ret = mla_prepared_banks_validate(sqe->req_len,
					  req.prepared_bank_count,
					  req.prepared_reserved, MLA_MAX_BANKS);
	if (ret) {
		dev_err(ctx->mdev->dev,
			"MODEL_DEFINE_V3: invalid prepared banks: req_len=%u count=%u reserved=%u maximum=%u\n",
			sqe->req_len, req.prepared_bank_count,
			req.prepared_reserved, MLA_MAX_BANKS);
		return ret;
	}

	/* Every request-shape rejection names the field it tripped on: a bare
	 * -EINVAL leaves no dmesg line and nothing in the errno to say which
	 * bound was violated, which is a poor trade on a path that runs once
	 * per model, not once per job.
	 */
	{
		const char *bad = NULL;

		if (req.reserved2)
			bad = "reserved2 != 0";
		else if (req.num_reloc_ports > MLA_MAX_IOTENSORS)
			bad = "num_reloc_ports > MLA_MAX_IOTENSORS";
		/* A model declaring LoRA ports must name a default binding for
		 * each, and a caller that sends only the pre-v7 struct cannot:
		 * num_reloc_ports occupies what was reserved_pad, so for such a
		 * caller it is 0.
		 */
		else if (req.num_reloc_ports && !req.reloc_defaults_user_ptr)
			bad = "num_reloc_ports > 0 with no reloc_defaults_user_ptr";
		else if (!req.num_sections ||
			 req.num_sections > MLA_SECTION_MAX_PER_LOAD)
			bad = "num_sections";
		else if (!req.num_tiles || req.num_tiles > MLA_MAX_ACTIVE_TILES)
			bad = "num_tiles";
		else if (req.num_dma_units != MLA_DMA_UNIT_COUNT_SIZE)
			bad = "num_dma_units != MLA_DMA_UNIT_COUNT_SIZE";
		else if (req.reloc_list_count > MLA_RELOC_MAX_ENTRIES)
			bad = "reloc_list_count > MLA_RELOC_MAX_ENTRIES";
		else if (req.static_reloc_count > req.reloc_list_count)
			bad = "static_reloc_count > reloc_list_count";
		else if (!req.section_ids_user_ptr)
			bad = "section_ids_user_ptr == NULL";
		else if (!req.tile_descs_user_ptr)
			bad = "tile_descs_user_ptr == NULL";
		else if (!req.dma_unit_descs_user_ptr)
			bad = "dma_unit_descs_user_ptr == NULL";
		else if (!req.out_ptr)
			bad = "out_ptr == NULL";
		else if (req.out_len < sizeof(resp))
			bad = "out_len < sizeof(resp)";

		if (bad) {
			dev_err(ctx->mdev->dev,
				"MODEL_DEFINE_V3: rejected on %s (req_len=%u sizeof=%zu num_sections=%u num_tiles=%u num_dma_units=%u reloc=%u static=%u num_in=%u num_out=%u ports=%u out_len=%u resp=%zu)\n",
				bad, sqe->req_len, sizeof(req),
				req.num_sections, req.num_tiles,
				req.num_dma_units, req.reloc_list_count,
				req.static_reloc_count, req.num_inputs,
				req.num_outputs, req.num_reloc_ports,
				req.out_len, sizeof(resp));
			return -EINVAL;
		}
	}

	req.name[sizeof(req.name) - 1] = '\0';

	section_ids   = kcalloc(req.num_sections, sizeof(*section_ids),   GFP_KERNEL);
	bufs          = kcalloc(req.num_sections, sizeof(*bufs),          GFP_KERNEL);
	section_sizes = kcalloc(req.num_sections, sizeof(*section_sizes), GFP_KERNEL);
	tile_descs    = kcalloc(req.num_tiles,    sizeof(*tile_descs),    GFP_KERNEL);
	dma_descs     = kcalloc(req.num_dma_units, sizeof(*dma_descs),    GFP_KERNEL);
	if (!section_ids || !bufs || !section_sizes || !tile_descs || !dma_descs) {
		ret = -ENOMEM;
		goto out_free;
	}

	if (copy_from_user(section_ids,
			   u64_to_user_ptr(req.section_ids_user_ptr),
			   req.num_sections * sizeof(*section_ids)) ||
	    copy_from_user(tile_descs,
			   u64_to_user_ptr(req.tile_descs_user_ptr),
			   req.num_tiles * sizeof(*tile_descs)) ||
	    copy_from_user(dma_descs,
			   u64_to_user_ptr(req.dma_unit_descs_user_ptr),
			   req.num_dma_units * sizeof(*dma_descs))) {
		ret = -EFAULT;
		goto out_free;
	}

	/* Pass 1: resolve every section_id + bump section_bufs refcount. */
	ret = v3_resolve_section_ids(ctx, section_ids, req.num_sections,
				     bufs, section_sizes);
	if (ret)
		goto out_free;

	/* Pass 2: validate tile/dma_unit/reloc descriptors against the
	 * resolved section sizes. Reused verbatim from V2 (roles=NULL —
	 * SECTION_ALLOC has no role field per §3.5).
	 */
	for (i = 0; i < req.num_tiles; i++) {
		ret = v2_validate_tile_desc(&tile_descs[i],
					    req.num_sections,
					    section_sizes,
					    /*section_roles=*/NULL);
		if (ret) {
			dev_err(ctx->mdev->dev,
				"MODEL_DEFINE_V3: tile[%u] rejected (rc=%d): instr_section_index=%u instr_offset=%u instr_size=%u mim_base_reg=0x%x\n",
				i, ret, tile_descs[i].instr_section_index,
				tile_descs[i].instr_offset,
				tile_descs[i].instr_size,
				tile_descs[i].mim_base_reg);
			goto out_release_refs;
		}
	}
	for (i = 0; i < req.num_dma_units; i++) {
		ret = v2_validate_dma_unit_desc(&dma_descs[i],
						req.num_sections,
						section_sizes);
		if (ret) {
			dev_err(ctx->mdev->dev,
				"MODEL_DEFINE_V3: dma_unit[%u] rejected (rc=%d): unit_index=%u desc_section_index=%u desc_blob_offset=0x%llx desc_count=%u fetch_map_type=%u section_size=%llu\n",
				i, ret, dma_descs[i].unit_index,
				dma_descs[i].desc_section_index,
				(unsigned long long)dma_descs[i].desc_blob_offset,
				dma_descs[i].desc_count,
				dma_descs[i].dfm_fetch_map_type,
				dma_descs[i].desc_section_index < req.num_sections ?
					(unsigned long long)section_sizes[dma_descs[i].desc_section_index] : 0ULL);
			goto out_release_refs;
		}
	}

	/* Pass 3: reloc list. Sized to the caller's actual count, not to
	 * MLA_RELOC_MAX_ENTRIES — at the 1 Mi cap that would be 24 MiB.
	 * kvcalloc because a large model's list exceeds what the page
	 * allocator will hand out contiguously (a 472,937-entry model is
	 * ~10.8 MiB), and the matching free is the kvfree() in
	 * mla_model_destroy() — this list lives as long as the model, it is
	 * NOT freed after the copy_from_user below.
	 */
	{
		struct mla_reloc_entry *list = NULL;

		if (req.reloc_list_count) {
			list = kvcalloc(req.reloc_list_count, sizeof(*list), GFP_KERNEL);
			if (!list) {
				ret = -ENOMEM;
				goto out_release_refs;
			}
			if (copy_from_user(list,
					   u64_to_user_ptr(req.reloc_list_user_ptr),
					   req.reloc_list_count * sizeof(*list))) {
				kvfree(list);
				ret = -EFAULT;
				goto out_release_refs;
			}
			for (i = 0; i < req.reloc_list_count; i++) {
				ret = v2_validate_reloc_entry(&list[i],
							req.num_sections,
							req.num_inputs,
							req.num_outputs,
							req.num_reloc_ports);
				if (ret) {
					/* Name the entry and every bound it was
					 * checked against: a bare -EINVAL makes a
					 * zone/index mismatch indistinguishable
					 * from any other malformed field, and
					 * userspace cannot see the reloc list it
					 * built from the ELF.
					 */
					dev_err(ctx->mdev->dev,
						"MODEL_DEFINE_V3: reloc[%u] rejected (rc=%d): zone_tag=%u zone_index=%u dma_offset=%u desc_index=%u placeholder=0x%llx; bounds: num_sections=%u num_inputs=%u num_outputs=%u num_reloc_ports=%u\n",
						i, ret, list[i].zone_tag,
						list[i].zone_index,
						list[i].dma_offset,
						list[i].desc_index,
						(unsigned long long)list[i].placeholder_offset,
						req.num_sections, req.num_inputs,
						req.num_outputs, req.num_reloc_ports);
					kvfree(list);
					goto out_release_refs;
				}
			}
		}

		/* Build the model. v2_build_model takes a
		 * struct mla_model_define_req; assemble a shim carrying
		 * only the fields it reads (num_sections, batch/num_in/out,
		 * flags, quad_tile_bitmask, num_tiles, num_dma_units).
		 * v2_build_model does its own refcount_inc on section_bufs;
		 * our v3_resolve_section_ids ref was already bumped, so
		 * transfer ownership: after v2_build_model succeeds, our
		 * refs merge into m->section_bufs[] and mla_model_destroy
		 * drops them.
		 */
		memset(&shim_req, 0, sizeof(shim_req));
		shim_req.num_sections   = req.num_sections;
		shim_req.num_tiles      = req.num_tiles;
		shim_req.num_dma_units  = req.num_dma_units;
		shim_req.batch_size     = req.batch_size;
		shim_req.num_inputs     = req.num_inputs;
		shim_req.num_outputs    = req.num_outputs;
		shim_req.flags          = req.flags;
		memcpy(shim_req.quad_tile_bitmask, req.quad_tile_bitmask,
		       sizeof(shim_req.quad_tile_bitmask));
		memcpy(shim_req.name, req.name, sizeof(shim_req.name));

		m = v2_build_model(ctx, &shim_req, bufs, tile_descs, dma_descs);
		if (IS_ERR(m)) {
			ret = PTR_ERR(m);
			m = NULL;
			kvfree(list);
			goto out_release_refs;
		}
		/* v2_build_model took its own refs on section_bufs.
		 * Release ours (the ones v3_resolve_section_ids bumped).
		 */
		v3_release_section_refs(ctx, bufs, req.num_sections);

		m->reloc_list         = list;
		m->reloc_count        = req.reloc_list_count;
		m->static_reloc_count = req.static_reloc_count;
		memcpy(m->name, req.name, sizeof(m->name));
	}
	/* tile_descs / dma_descs now owned by m; NULL out so error path
	 * doesn't double-free.
	 */
	tile_descs = NULL;
	dma_descs  = NULL;

	ret = v2_apply_static_section_relocs(m, m->dma_info_typed);
	if (ret)
		goto out_destroy;

	/* Default bindings for the LoRA ports, and their first splice.
	 *
	 * This is what makes "defined but never updated" a supported
	 * configuration rather than a descriptor pointing at whatever its
	 * compile-time placeholder address happens to name. The caller has
	 * already SECTION_ALLOC'd + SECTION_COMMIT'd the adapter sections the
	 * ELF shipped (all-zero filters, real scales, for every LoRA model
	 * measured), so binding to them yields base-model behaviour by
	 * construction — no assumption about how the graph consumes the
	 * adapter, and in particular no assumption that a zeroed scale is
	 * equivalent to the real one.
	 */
	if (req.num_reloc_ports) {
		struct mla_iotensor *defs;
		u32 p, spliced = 0, refd;

		/* Record which declared ports the reloc list actually names.
		 *
		 * Scanned over EXACTLY the range mla_model_patch_reloc_ports()
		 * walks — [static_reloc_count, reloc_count) — so the two agree by
		 * construction. (A RELOC entry in the static prefix is separately
		 * rejected by v2_apply_static_section_relocs(), which admits only
		 * SECTION-tagged entries there.) Every zone_index was already
		 * validated < num_reloc_ports, so __set_bit cannot run off the
		 * end.
		 *
		 * Not fatal either way. An unreferenced port is only harmful if a
		 * caller later tries to re-point it, and op 20 is where that is
		 * caught, per port; rejecting the LOAD here would refuse a model
		 * that runs correctly on base weights.
		 */
		bitmap_zero(m->reloc_ports_refd, MLA_MAX_IOTENSORS);
		for (i = m->static_reloc_count; i < m->reloc_count; i++)
			if (m->reloc_list[i].zone_tag == MLA_RELOC_ZONE_RELOC)
				__set_bit(m->reloc_list[i].zone_index,
					  m->reloc_ports_refd);
		refd = (u32)bitmap_weight(m->reloc_ports_refd,
					  req.num_reloc_ports);
		if (refd != req.num_reloc_ports)
			dev_warn(ctx->mdev->dev,
				 "MODEL_DEFINE_V3: %u of %u reloc ports are named by no reloc entry (first: port %u) — re-pointing one of those would rewrite nothing\n",
				 req.num_reloc_ports - refd, req.num_reloc_ports,
				 (u32)find_first_zero_bit(m->reloc_ports_refd,
							  req.num_reloc_ports));

		defs = kcalloc(req.num_reloc_ports, sizeof(*defs), GFP_KERNEL);
		m->reloc_bufs    = kcalloc(req.num_reloc_ports,
					   sizeof(*m->reloc_bufs), GFP_KERNEL);
		m->reloc_offsets = kcalloc(req.num_reloc_ports,
					   sizeof(*m->reloc_offsets), GFP_KERNEL);
		if (!defs || !m->reloc_bufs || !m->reloc_offsets) {
			kfree(defs);
			ret = -ENOMEM;
			goto out_destroy;
		}
		/* Publish the count only once both arrays exist: every consumer
		 * (patcher, update op, destroy) treats it as "reloc_bufs[] has
		 * this many live entries".
		 */
		m->num_reloc_ports = req.num_reloc_ports;

		if (copy_from_user(defs,
				   u64_to_user_ptr(req.reloc_defaults_user_ptr),
				   req.num_reloc_ports * sizeof(*defs))) {
			kfree(defs);
			ret = -EFAULT;
			goto out_destroy;
		}
		for (p = 0; p < req.num_reloc_ports; p++) {
			if (defs[p].internal_buf_id && !defs[p].reserved)
				continue;
			/* Rejecting here, not at submit, is deliberate: it is
			 * checked once, the caller can still act on it, and a
			 * job can never reach a port that has no binding.
			 */
			dev_err(ctx->mdev->dev,
				"MODEL_DEFINE_V3: reloc port %u has no default binding (buf_id=%u reserved=%u)\n",
				p, defs[p].internal_buf_id, defs[p].reserved);
			kfree(defs);
			ret = -EINVAL;
			goto out_destroy;
		}
		ret = mla_resolve_iotensors(ctx, defs, req.num_reloc_ports,
					    m->reloc_bufs);
		if (ret) {
			kfree(defs);
			goto out_destroy;
		}
		for (p = 0; p < req.num_reloc_ports; p++)
			m->reloc_offsets[p] = defs[p].offset;
		kfree(defs);

		atomic_add(req.num_reloc_ports, &ctx->mdev->reloc_ports_bound);

		/* Bank 0 only: it is the primary blob, and every bank grown
		 * later is a memcpy of it (mla_model_grow_banks), so the splice
		 * is inherited exactly as the static-reloc splice is.
		 */
		ret = mla_model_patch_reloc_ports(m, 0, &spliced);
		if (ret)
			goto out_destroy;
		/* refd ports exist and every port is bound, so the patcher must
		 * have rewritten something. Zero here is an internal
		 * inconsistency, not a caller error — hence WARN. Guarded on refd
		 * because refd == 0 legitimately splices nothing, and that case
		 * is a dev_warn above rather than a failed load.
		 */
		if (WARN_ON_ONCE(refd && !spliced)) {
			dev_err(ctx->mdev->dev,
				"MODEL_DEFINE_V3: %u reloc ports bound, %u referenced, but no descriptor spliced\n",
				req.num_reloc_ports, refd);
			ret = -EINVAL;
			goto out_destroy;
		}
	}

	m->using_semaphores = mla_model_uses_semaphores_v2(m);

	if (mla_do_dma_sync) {
		int rc = v2_build_flush_regions(m);

		if (rc && rc != -ENOMEM) {
			ret = rc;
			goto out_destroy;
		}
	}

	/* Admission is still private: static and default LoRA relocations are
	 * installed before copying the primary into extra banks. Reuse the
	 * established bank publication/cleanup protocol, not a second pool.
	 */
	if (req.prepared_bank_count) {
		while (m->num_banks < req.prepared_bank_count) {
			u32 previous = m->num_banks;

			ret = mla_model_grow_banks(m);
			if (ret)
				goto out_destroy;
			if (WARN_ON_ONCE(m->num_banks != previous + 1)) {
				ret = -EIO;
				goto out_destroy;
			}
		}
		m->banks_prepared = true;
	}

	ret = xa_alloc(&ctx->models, &id, m, xa_limit_32b, GFP_KERNEL);
	if (ret < 0)
		goto out_destroy;
	m->id = id;

	memset(&resp, 0, sizeof(resp));
	resp.model_id            = m->id;
	resp.num_active_tiles    = m->num_active_tiles;
	resp.num_dma_descriptors = m->num_dma_descriptors;

	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp))) {
		xa_erase(&ctx->models, m->id);
		ret = -EFAULT;
		goto out_destroy;
	}

	dev_dbg(ctx->mdev->dev,
		"MODEL_DEFINE_V3: model %u sections=%u tiles=%u dma_descs=%u using_sema=%u\n",
		m->id, req.num_sections, req.num_tiles,
		m->num_dma_descriptors, m->using_semaphores);

	/* NOTE: do NOT kfree(bufs) — v2_build_model stored this pointer
	 * as m->section_bufs and mla_model_destroy will kfree it there.
	 * Same for tile_descs / dma_descs: they're owned by m after
	 * v2_build_model.
	 */
	kfree(section_sizes);
	kfree(section_ids);
	return 0;

out_destroy:
	/* m owns section_bufs, tile_info_typed, dma_info_typed, reloc_list,
	 * flush_regions. mla_model_destroy handles them all.
	 */
	if (m)
		mla_model_destroy(m);
	/* Our v3_resolve_section_ids refs were already released above. */
	kfree(section_sizes);
	kfree(section_ids);
	return ret;

out_release_refs:
	v3_release_section_refs(ctx, bufs, req.num_sections);
out_free:
	kfree(dma_descs);
	kfree(tile_descs);
	kfree(section_sizes);
	kfree(bufs);
	kfree(section_ids);
	return ret;
}

/* ====================================================================== *
 *  MLA_OP_MODEL_RELEASE + model lifecycle
 * ======================================================================
 */

/**
 * mla_model_grow_banks() — allocate the next descriptor-blob bank
 * @m: model whose bank pool is being grown
 *
 * Adds one more bank at m->banks[m->num_banks]. The bank owns its own
 * mla_pool_alloc_cached'd buffer (WB linear-map alias, matches the
 * primary section's CPU_RMW routing so range-scoped flush stays cheap).
 * The buffer is initialised to a byte-for-byte copy of the primary
 * dma.descs blob at grow time — subsequent per-job apply_relocs will
 * patch its dynamic-reloc window; static relocs were resolved once at
 * MODEL_DEFINE and are already in the memcpy'd bytes.
 *
 * Concurrency:
 *  * Multiple enqueuers may all see outstanding > num_banks and race
 *    to call this. Serialised on bank_grow_lock; the second entrant
 *    re-checks num_banks and returns 0 without work if the first
 *    already grew.
 *  * Readers of banks[i].blob outside the lock rely on the release
 *    ordering of atomic_set(&bank->state) + smp_store_release on
 *    num_banks below.
 *
 * Return: 0 on success or "already at ceiling / already grown by a
 * peer" (no error either way); -ENOMEM on allocation failure.
 */
int mla_model_grow_banks(struct mla_model *m)
{
	struct mla_buffer *primary, *new_blob;
	struct mla_device *mdev;
	u32 sec_idx;
	u32 new_idx;
	int ret = 0;

	/* m->ctx is valid here: the model holds a counted reference on its
	 * context (taken at model creation, released in mla_model_destroy),
	 * so a model outliving the fd — which happens whenever an in-flight
	 * job still holds a model ref, and this IS the enqueue path for such
	 * jobs — keeps its context alive too. See
	 * docs/CTX_LIFETIME_DESIGN.md.
	 *
	 * The NULL check stays as cheap defence for a partially-constructed
	 * model reaching here; it cannot catch a dangling (freed but
	 * non-NULL) pointer, so it is not a substitute for the context
	 * reference above.
	 */
	if (!m->ctx || !m->ctx->mdev)
		return -EINVAL;
	mdev = m->ctx->mdev;

	if (READ_ONCE(m->num_banks) >= MLA_MAX_BANKS)
		return 0;

	mutex_lock(&m->bank_grow_lock);

	if (m->num_banks >= MLA_MAX_BANKS)
		goto out_unlock;   /* another enqueuer grew it while we waited */

	/* Bank[0] is the sentinel-aliased primary section blob; copy from
	 * section_bufs[dma_info_typed[0].desc_section_index]. The primary's
	 * static-reloc window has already been spliced by MODEL_DEFINE, so
	 * the memcpy captures the correct baseline for every new bank.
	 */
	if (!m->dma_info_typed || !m->section_bufs) {
		ret = -EINVAL;
		goto out_unlock;
	}
	/* desc_section_index is userspace-supplied. MODEL_DEFINE validates
	 * it, and every other consumer re-checks anyway (see :723, :844,
	 * :1896 and mla_hw.c init_dfm) -- this was the one site that
	 * indexed section_bufs[] on trust. Re-check for symmetry: a
	 * one-compare cost on a path that already takes a mutex.
	 */
	sec_idx = m->dma_info_typed[0].desc_section_index;
	if (sec_idx >= m->num_sections) {
		ret = -EINVAL;
		goto out_unlock;
	}
	primary = m->section_bufs[sec_idx];
	if (!primary || !primary->cpu_vaddr) {
		ret = -EINVAL;
		goto out_unlock;
	}

	new_blob = mla_pool_alloc_cached(mdev, primary->size);
	if (IS_ERR(new_blob)) {
		ret = PTR_ERR(new_blob);
		goto out_unlock;
	}
	/* so mla_buffer_drop → mla_pool_free routes correctly. Counted, like
	 * every other ->ctx back-pointer: this blob is freed via
	 * mla_pool_free → mla_buffer_free, which does the matching put.
	 * Note this buffer is NOT in ctx->buffers (never registered), so it
	 * gets its reference here rather than in mla_buffer_register.
	 */
	new_blob->ctx = mla_context_get(m->ctx);
	memcpy(new_blob->cpu_vaddr, primary->cpu_vaddr, primary->size);
	/* Publish the freshly-copied bytes to memory so the first prep
	 * worker's apply_relocs sees the static-reloc baseline in DMA-visible
	 * form. Subsequent per-job flushes handle only their dynamic
	 * patches.
	 */
	mla_pool_sync_for_device(mdev, new_blob);

	new_idx = m->num_banks;
	m->banks[new_idx].blob = new_blob;
	atomic_set(&m->banks[new_idx].state, MLA_BANK_FREE);
	/* Publish new_idx before bumping num_banks so a reader that sees
	 * num_banks > new_idx also sees banks[new_idx].blob populated.
	 */
	smp_store_release(&m->num_banks, new_idx + 1);

out_unlock:
	mutex_unlock(&m->bank_grow_lock);
	return ret;
}

/*
 * Free a model and all attached resources — from the last refcount drop
 * (mla_model_put / mla_model_put_async), or directly from a
 * MODEL_DEFINE_V3 error path before the model is ever published. Walks
 * section_bufs[] to release each per-directive buffer (dedup-aliased or
 * freshly allocated) and frees the typed tile / DMA-unit / reloc /
 * flush-region arrays.
 */
void mla_model_destroy(struct mla_model *m)
{
	/* Invalidate the DFM/tile/QG0 reprogram-skip cache if it still
	 * points at this model. mla_hw_program_and_start compares
	 * mdev->last_programmed_model by POINTER, and we are about to
	 * kfree(m) — the very next kzalloc of a struct mla_model can return
	 * this same address (fixed-size slab; xa_alloc can also recycle the
	 * same m->id), making a genuinely different model compare equal and
	 * skip both mla_hw_init_dfm and mla_hw_load_tiles.
	 *
	 * The dangerous half is the skipped load_tiles: the tile MIM/QG0
	 * registers would still point at the freed model's instruction
	 * sections, which are never re-allocated to match, so the hardware
	 * executes dead instruction memory until the watchdog fires:
	 *   JOB_SUBMIT: HW fault class=mosaic sys_sts=0x20(quadgrp_tmo)
	 * DFM happens to stay correct because the descriptor blob is
	 * reallocated at the same phys address, but that is incidental.
	 * See UNITTEST.reprog_skip_cache_two_models.
	 */
	if (m->ctx && m->ctx->mdev &&
	    m->ctx->mdev->last_programmed_model == m) {
		m->ctx->mdev->last_programmed_model = NULL;
		m->ctx->mdev->last_programmed_bank_idx = MLA_BANK_UNASSIGNED;
	}

	/* kvfree: reloc_list may be vmalloc-backed when the model has
	 * many relocs (gemma3-siglip448 hits ~186k entries).
	 */
	kvfree(m->reloc_list);

	/* LoRA port bindings, one counted reference each, whether the port
	 * still points at its ELF default section or at caller memory that a
	 * later MLA_OP_MODEL_UPDATE_RELOC bound. Dropped BEFORE the section
	 * loop below, in the same order the update op uses (patch, then drop
	 * the displaced reference): a default binding and its section are the
	 * same mla_buffer, and the section loop is the one that also
	 * xa_erase()s it, so it must run last.
	 *
	 * mla_buffer_drop (not _async) is correct here: mla_model_destroy
	 * already blocks — kvfree, mla_context_put — and mla_model_put_async()
	 * is what keeps it off the softirq paths.
	 */
	if (m->reloc_bufs) {
		u32 p;

		for (p = 0; p < m->num_reloc_ports; p++)
			if (m->reloc_bufs[p])
				mla_buffer_drop(m->reloc_bufs[p]);
		kfree(m->reloc_bufs);
		m->reloc_bufs = NULL;
	}
	kfree(m->reloc_offsets);
	m->reloc_offsets = NULL;

	/* MODEL_DEFINE bumped the refcount by 1 per section on top of
	 * the ref that mla_pool_alloc initialized to 1 (used by
	 * mla_buffer_register — xa insert doesn't bump). Total ref
	 * per unshared section = 2 (xarray + section_bufs). To
	 * release memory at MODEL_RELEASE we need to drop BOTH: the
	 * section_bufs ref AND the xarray ref. Otherwise the xarray
	 * keeps the buffer alive until fd close.
	 *
	 * Dedup-alias sections have refcount=1 (alias buf), released
	 * via the dedup entry refcount inside mla_buffer_drop. Their
	 * xarray entry is per-context and independent.
	 */
	if (m->section_bufs) {
		u32 i;

		for (i = 0; i < m->num_sections; i++) {
			struct mla_buffer *b = m->section_bufs[i];
			u32 id;

			if (!b)
				continue;

			id = b->id;

			/* Drop the section_bufs[] ref that
			 * v2_build_model bumped.
			 */
			mla_buffer_drop(b);

			/* Drop the xarray ref by xa_erase +
			 * mla_buffer_drop.
			 */
			if (m->ctx && id) {
				struct mla_buffer *erased =
					xa_erase(&m->ctx->buffers, id);
				if (erased && erased == b) {
					mla_buffer_drop(b);
				} else if (erased && erased != b && m->ctx->mdev) {
					/* Anomaly: xa slot holds a DIFFERENT
					 * buffer than the section_bufs pointer.
					 * We just yanked someone else's live
					 * registration out from under them.
					 * This is either the id-recycle bug or
					 * a real double-free waiting to happen.
					 */
					dev_warn(m->ctx->mdev->dev,
						 "MDROP2_MISMATCH: model=%u i=%u buf=%p erased=%p id=%u — orphaned prior tenant\n",
						 m->id, i, b, erased, id);
				}
			}
		}
		kfree(m->section_bufs);
	}
	kfree(m->tile_info_typed);
	kfree(m->dma_info_typed);
	kfree(m->flush_regions);
	/* v2.6 bank pool: banks[1..num_banks-1].blob are private allocations
	 * from mla_model_grow_banks (step 2c); free them before the array
	 * itself. Bank[0].blob is NULL (sentinel), so the loop skips it —
	 * the aliased section blob is freed via section_bufs above.
	 */
	if (m->banks) {
		u32 i;

		for (i = 0; i < m->num_banks; i++) {
			if (m->banks[i].blob)
				mla_buffer_drop(m->banks[i].blob);
		}
		kfree(m->banks);
	}

	{
		struct mla_context *ctx = m->ctx;

		m->ctx = NULL;
		kfree(m);
		/* Release the context reference taken at model creation. Last,
		 * and after kfree(m): this put may free the context, so nothing
		 * may touch m or m->ctx afterwards.
		 */
		mla_context_put(ctx);
	}
}

/* Descriptor-layout constants used by the semaphore-detection walker.
 * Each DMA descriptor is 32 bytes; the first word carries per-descriptor
 * fields including the chk_sema (bits 4..11) and set_sema (bits 12..19)
 * lanes. Any non-zero value in either mask means the model uses
 * semaphores.
 */
#define MLA_DMA_DESC_SIZE_BYTES         32u
#define MLA_DMA_DESC_CHK_SEMA_MASK      0x00000ff0u  /* bits 4..11  */
#define MLA_DMA_DESC_SET_SEMA_MASK      0x000ff000u  /* bits 12..19 */

/*
 * Walk the DMA descriptor blob via section_bufs[] + dma_info_typed[].
 * If any descriptor sets chk_sema or set_sema, the scheduler needs to
 * program the 256B WR burst before dispatch — semaphored models
 * deadlock on the default 1KB write burst.
 *
 * Conservative on missing/short blobs: if a unit's section is unmapped
 * or the blob extent overflows the section, skip that unit — the
 * worst case is a wrong burst (slow but correct), and validation at
 * MODEL_DEFINE already rejected over-range blobs.
 */
bool mla_model_uses_semaphores_v2(struct mla_model *m)
{
	u32 unit, i;

	if (!m->dma_info_typed || !m->section_bufs)
		return false;

	for (unit = 0; unit < MLA_DMA_UNIT_COUNT_SIZE; unit++) {
		const struct mla_dma_unit_desc *u = &m->dma_info_typed[unit];
		const struct mla_buffer *blob_buf;
		const u8 *p;
		u64 end;

		if (!u->desc_count)
			continue;
		if (u->desc_section_index >= m->num_sections)
			continue;
		blob_buf = m->section_bufs[u->desc_section_index];
		if (!blob_buf || !blob_buf->cpu_vaddr)
			continue;

		end = (u64)u->desc_blob_offset +
		      (u64)u->desc_count * MLA_DMA_DESC_SIZE_BYTES;
		if (end > blob_buf->size)
			continue;

		p = (const u8 *)blob_buf->cpu_vaddr + u->desc_blob_offset;
		for (i = 0; i < u->desc_count; i++) {
			u32 first_word = *(const u32 *)
				(p + i * MLA_DMA_DESC_SIZE_BYTES);
			if (first_word &
			    (MLA_DMA_DESC_CHK_SEMA_MASK |
			     MLA_DMA_DESC_SET_SEMA_MASK))
				return true;
		}
	}
	return false;
}

static void mla_model_put(struct mla_model *m)
{
	if (refcount_dec_and_test(&m->refcount))
		mla_model_destroy(m);
}

static void mla_model_release_work_fn(struct work_struct *work)
{
	struct mla_model *m = container_of(work, struct mla_model, release_work);

	mla_model_destroy(m);
}

/**
 * mla_model_put_async() — drop a model reference from a context that may sleep-
 *                         forbidden
 * @m: model to put
 *
 * Same accounting as mla_model_put(), but the last-reference teardown is handed
 * to mdev->buf_release_wq instead of running inline. Required because a job
 * holds a model reference and drops it from mla_fence_release(), which runs in
 * softirq via mla_sched_complete() / mla_job_timeout_fn(), while
 * mla_model_destroy() blocks: it calls mla_buffer_drop() (not the _async
 * variant), kvfree() on a possibly-vmalloc'd reloc_list, and a final
 * mla_context_put().
 *
 * Mirrors mla_buffer_drop_async(). Only the free is deferred, never the
 * refcount, so no accounting is timing-dependent.
 */
void mla_model_put_async(struct mla_model *m)
{
	struct mla_device *mdev = m->ctx ? m->ctx->mdev : NULL;

	if (!refcount_dec_and_test(&m->refcount))
		return;

	/* No queue (probe partially failed, or mla_sched_exit already tore it
	 * down): fall back to inline teardown. A caller reaching here from
	 * softirq would be a latent sleep-in-atomic, so say so rather than
	 * hide it — same contract as mla_buffer_drop_async().
	 */
	if (!mdev || !mdev->buf_release_wq) {
		WARN_ONCE(!mdev || !mdev->buf_release_wq,
			  "mla: no buf_release_wq; tearing down model inline\n");
		mla_model_destroy(m);
		return;
	}

	/* Safe to init here: a model's last reference drops exactly once, so
	 * this work item is queued at most once and cannot already be pending.
	 */
	INIT_WORK(&m->release_work, mla_model_release_work_fn);
	queue_work(mdev->buf_release_wq, &m->release_work);
}

/**
 * mla_op_model_release() — handle MLA_OP_MODEL_RELEASE
 * @ctx: context that owns the model
 * @sqe: io_uring SQE inline payload
 * @hdr_unused: request header (unused)
 *
 * Removes the model from this context's xarray and drops the
 * reference. The actual destroy runs from mla_model_put() when the
 * last reference (jobs holding it) drops.
 *
 * Return: 0 on success; -ENOENT if model_id unknown to this context.
 */
int mla_op_model_release(struct mla_context *ctx,
			 const struct mla_uring_cmd_sqe *sqe,
			 const struct mla_cmd_hdr *hdr_unused)
{
	struct mla_model_release_req req;
	struct mla_model *m;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;

	m = xa_erase(&ctx->models, req.model_id);
	if (!m)
		return -ENOENT;

	mla_model_put(m);
	return 0;
}

/**
 * mla_context_models_release() — drop all models on context teardown
 * @ctx: context whose model xarray is being destroyed
 *
 * Walks the model xarray and drops each. Any surviving reference held
 * by an in-flight job keeps the model alive until that job completes.
 */
void mla_context_models_release(struct mla_context *ctx)
{
	struct mla_model *m;
	unsigned long id;

	xa_for_each(&ctx->models, id, m) {
		xa_erase(&ctx->models, id);
		mla_model_put(m);
	}
}

/* ====================================================================== *
 *  MLA_OP_JOB_SUBMIT / MLA_OP_JOB_CANCEL
 *
 *  Sync (MLA_CMD_FLAG_WAIT) and async paths share the enqueue path.
 *  Async returns immediately with a completion_fence_fd (sync_file) the
 *  caller polls. Sync waits inside the kernel for the job's completion
 *  before returning.
 * ======================================================================
 */

/*
 * Resolve a userspace mla_iotensor[] array (referencing buffers by
 * internal_buf_id) into a caller-provided array of struct mla_buffer
 * pointers owned by @ctx, taking one counted reference per entry.
 * Validates that each id exists in this context's xarray and that the
 * requested offset+length lies within the buffer.
 *
 * The reference is what keeps the buffer — and its DMA mapping — alive for
 * the life of the job: a job routinely outlives the BUF_RELEASE and the fd
 * close of its own tensors, and mla_hw_apply_relocs reads b->dma_addr from
 * the dispatch path long after submit returned. Released by
 * mla_job_put_bindings(). See docs/BUFFER_LIFETIME_DESIGN.md.
 *
 * The buf_inc_ops bump pairs with the one mla_buffer_drop* does on
 * buf_dec_ops — that metric counts references TAKEN and drop CALLS, not
 * buffers, so a per-tensor reference must be counted per tensor. A
 * duplicated buf_id in one job takes (and releases) two references, which
 * needs no dedupe search.
 *
 * @bufs must have room for @count entries; allocation is the caller's
 * responsibility (stack for sync-mode small counts, kcalloc for async).
 *
 * POSTCONDITION on failure: every entry of @bufs is NULL and this function
 * holds no references. @bufs is zeroed up front rather than relying on the
 * caller to have done so. That is what lets mla_job_put_bindings() be called
 * at any point in the error unwind, and twice, without leaking or
 * double-dropping.
 *
 * Returns 0 on success; -ENOENT if any id is unknown to this context,
 * -EINVAL on out-of-range offset/length.
 */
static int mla_resolve_iotensors(struct mla_context *ctx,
				 const struct mla_iotensor *user_arr,
				 u32 count,
				 struct mla_buffer **bufs)
{
	u32 i;

	memset(bufs, 0, count * sizeof(*bufs));

	for (i = 0; i < count; i++) {
		struct mla_buffer *b = xa_load(&ctx->buffers,
					       user_arr[i].internal_buf_id);
		u64 end;

		if (!b) {
			dev_info(ctx->mdev->dev,
				 "resolve_iotensors: ENOENT i=%u buf_id=%u (not in ctx->buffers)\n",
				 i, user_arr[i].internal_buf_id);
			goto err_unwind;
		}
		/* For BUF_ALLOC buffers, b->size is the authoritative extent of
		 * the underlying dma-buf and offset+length MUST fit. For
		 * BUF_ATTACH_PHYS buffers, b->size is only the window the
		 * caller registered; the underlying physical region (e.g. a
		 * llima KV-cache CMA carve-out) is typically much larger and
		 * holds many tensor slices at different offsets. The caller
		 * had CAP_SYS_RAWIO when they registered, so we trust their
		 * offset/length here -- but never a window whose end wraps, which
		 * would name an address below the buffer.
		 */
		if (check_add_overflow(user_arr[i].offset, user_arr[i].length, &end) ||
		    (!b->phys_attached && end > b->size)) {
			dev_info(ctx->mdev->dev,
				 "resolve_iotensors: EINVAL i=%u buf_id=%u offset=0x%llx length=0x%llx b->size=0x%zx\n",
				 i, user_arr[i].internal_buf_id,
				 (unsigned long long)user_arr[i].offset,
				 (unsigned long long)user_arr[i].length,
				 (size_t)b->size);
			goto err_unwind_einval;
		}
		refcount_inc(&b->refcount);
		atomic64_inc(&ctx->mdev->buf_inc_ops);
		bufs[i] = b;
	}
	return 0;

err_unwind_einval:
	while (i--)
		mla_buffer_drop(bufs[i]);
	memset(bufs, 0, count * sizeof(*bufs));
	return -EINVAL;

err_unwind:
	while (i--)
		mla_buffer_drop(bufs[i]);
	memset(bufs, 0, count * sizeof(*bufs));
	return -ENOENT;
}

/**
 * mla_op_model_update_reloc() — handle MLA_OP_MODEL_UPDATE_RELOC (op 20)
 * @ctx: context that owns the model and the adapter buffers
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header (unused; no flags apply)
 *
 * Re-points some or all of a model's LoRA ports and rewrites the affected
 * descriptors in every descriptor bank before returning. Synchronous and
 * eager on purpose: the caller's contract is "update_rel, then run", so a
 * 0 return has to mean the next job already sees the new adapter. Nothing
 * is deferred to submit or prep, and no per-job state is created.
 *
 * Quiescence is enforced, not assumed. The op takes the model's
 * bank_grow_lock (the only other writer of a bank blob) and, in one
 * sched_lock section, sets reloc_update_in_progress and reads
 * model->outstanding. That pairing is what closes the window a bare
 * check-then-patch would leave: mla_sched_enqueue() increments outstanding
 * before it takes sched_lock and tests the flag under it, so one of the two
 * always loses — either we see the job's increment and answer -EBUSY, or it
 * sees our flag and its submit fails with -EBUSY. Patching under a live job
 * would let the device read a descriptor set that is half old adapter and
 * half new, or a descriptor mid-RMW — a torn 40-bit address pointing
 * anywhere, which surfaces as an FTL fault or a quadgroup timeout rather
 * than as an error anyone can attribute.
 *
 * Return: 0 on success; -ENOENT unknown model_id or buffer id; -EINVAL
 * malformed request or num_reloc != the model's port count; -ENODATA if a port
 * being re-pointed is named by no reloc entry, so no descriptor would be
 * rewritten; -EBUSY if the model has outstanding work; -ENOMEM.
 */
int mla_op_model_update_reloc(struct mla_context *ctx,
			      const struct mla_uring_cmd_sqe *sqe,
			      const struct mla_cmd_hdr *hdr)
{
	struct mla_model_update_reloc_req  req;
	struct mla_model_update_reloc_resp resp;
	struct mla_iotensor *user_arr = NULL;
	struct mla_buffer  **newbufs  = NULL;
	struct mla_buffer  **oldbufs  = NULL;
	u64                 *oldoffs  = NULL;
	struct mla_device *mdev = ctx->mdev;
	struct mla_model *m;
	unsigned long flags;
	u32 p, bank, updated = 0, passes = 0;
	int ret;

	(void)hdr;

	if (sqe->req_len <
	    offsetofend(struct mla_model_update_reloc_req, reserved2))
		return -EINVAL;
	memset(&req, 0, sizeof(req));
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr),
			   min_t(size_t, (size_t)sqe->req_len, sizeof(req))))
		return -EFAULT;
	if (req.reserved || req.reserved2)
		return -EINVAL;
	/* struct_size is the caller's declared length, and the uapi documents it
	 * as authoritative ("the handler accepts struct_size >= offsetofend(...,
	 * reserved2) and truncates to what it understands"). Enforce that
	 * contract here rather than leaving it decorative. The truncation itself
	 * is already done by the min_t() copy above.
	 */
	if (req.struct_size <
	    offsetofend(struct mla_model_update_reloc_req, reserved2) ||
	    req.struct_size > sqe->req_len) {
		dev_err(mdev->dev,
			"MODEL_UPDATE_RELOC: struct_size=%u outside [%zu, req_len=%u]\n",
			req.struct_size,
			offsetofend(struct mla_model_update_reloc_req, reserved2),
			sqe->req_len);
		return -EINVAL;
	}
	if (!req.reloc_ptr || !req.num_reloc ||
	    req.num_reloc > MLA_MAX_IOTENSORS)
		return -EINVAL;
	if (!req.out_ptr ||
	    req.out_len < offsetofend(struct mla_model_update_reloc_resp,
				      patch_passes))
		return -EINVAL;

	/* Model reference under xa_lock, so a concurrent MODEL_RELEASE cannot
	 * destroy it between lookup and increment. Same protocol as JOB_SUBMIT.
	 */
	xa_lock(&ctx->models);
	m = xa_load(&ctx->models, req.model_id);
	if (m && !refcount_inc_not_zero(&m->refcount))
		m = NULL;
	xa_unlock(&ctx->models);
	if (!m) {
		dev_err(mdev->dev,
			"MODEL_UPDATE_RELOC: ENOENT model_id=%u\n", req.model_id);
		return -ENOENT;
	}

	/* Indexed by port, not packed — so the count has to match exactly,
	 * and a caller that wants to touch one port sends the full array with
	 * internal_buf_id 0 in the others.
	 */
	if (req.num_reloc != m->num_reloc_ports) {
		dev_err(mdev->dev,
			"MODEL_UPDATE_RELOC: model %u has %u reloc ports, request carries %u\n",
			m->id, m->num_reloc_ports, req.num_reloc);
		ret = -EINVAL;
		goto out_put_model;
	}
	if (!m->reloc_bufs || !m->reloc_offsets) {
		ret = -EINVAL;
		goto out_put_model;
	}

	user_arr = kcalloc(req.num_reloc, sizeof(*user_arr), GFP_KERNEL);
	newbufs  = kcalloc(req.num_reloc, sizeof(*newbufs),  GFP_KERNEL);
	oldbufs  = kcalloc(req.num_reloc, sizeof(*oldbufs),  GFP_KERNEL);
	oldoffs  = kcalloc(req.num_reloc, sizeof(*oldoffs),  GFP_KERNEL);
	if (!user_arr || !newbufs || !oldbufs || !oldoffs) {
		ret = -ENOMEM;
		goto out_free;
	}
	if (copy_from_user(user_arr, u64_to_user_ptr(req.reloc_ptr),
			   req.num_reloc * sizeof(*user_arr))) {
		ret = -EFAULT;
		goto out_free;
	}
	for (p = 0; p < req.num_reloc; p++) {
		if (user_arr[p].reserved) {
			ret = -EINVAL;
			goto out_free;
		}
		/* Refuse to "swap" an adapter on a port no descriptor in this
		 * model names. Nothing would be rewritten, so the swap cannot
		 * take effect — and returning 0 with that port counted in
		 * ports_updated is precisely the silent-wrong-answer this op
		 * exists to prevent (bindings recorded, job runs, fence signals
		 * success, output is the old adapter's).
		 *
		 * Enforced HERE rather than at MODEL_DEFINE deliberately.
		 * num_reloc_ports is the ELF's RELOC section count, not the set
		 * of ports the descriptors reference, so an unreferenced port is
		 * a shape we cannot yet rule out — and rejecting the define would
		 * refuse a model that runs correctly today on base weights, for a
		 * problem that only exists once someone tries to re-point that
		 * particular port. Note this does NOT constrain partial updates:
		 * leaving a port alone (internal_buf_id == 0) is always legal,
		 * referenced or not.
		 */
		if (user_arr[p].internal_buf_id &&
		    !test_bit(p, m->reloc_ports_refd)) {
			dev_err(mdev->dev,
				"MODEL_UPDATE_RELOC: model %u port %u is named by no reloc entry — re-pointing it would rewrite no descriptor, so the adapter could not take effect\n",
				m->id, p);
			ret = -ENODATA;
			goto out_free;
		}
	}

	/* Resolve per port rather than in one mla_resolve_iotensors call:
	 * internal_buf_id == 0 means "leave this port alone" (never a valid id
	 * — ctx->buffers is XA_FLAGS_ALLOC1), which that helper would report as
	 * -ENOENT. One-entry calls keep its validation, refcounting and
	 * buf_inc_ops accounting verbatim.
	 */
	for (p = 0; p < req.num_reloc; p++) {
		if (!user_arr[p].internal_buf_id)
			continue;
		ret = mla_resolve_iotensors(ctx, &user_arr[p], 1, &newbufs[p]);
		if (ret)
			goto out_drop_new;
		updated++;
	}
	if (!updated) {
		/* Legal no-op — every port said "leave alone". Say so in the
		 * response rather than in the return code, which a caller
		 * cannot tell from a real update.
		 */
		ret = 0;
		goto out_respond;
	}

	/* Wait out the fence-signal / outstanding-decrement skew before
	 * declaring the caller non-quiescent.
	 *
	 * mla_sched_finish_job() signals the job's fence and only THEN
	 * decrements model->outstanding, so a caller released by its
	 * completion_fence_fd can arrive here while the completing CPU has not
	 * yet run the decrement. Reading `outstanding` bare therefore refused
	 * correct callers with a spurious -EBUSY.
	 *
	 * This is a short GRACE PERIOD, not a wait for the caller's work: the
	 * gap it covers is the tail of finish_job, microseconds. A model with a
	 * genuinely running job still gets -EBUSY — the fail-fast contract is
	 * unchanged — it just costs this bounded wait first.
	 *
	 * Taken with NO LOCK HELD, deliberately. Waiting under bank_grow_lock
	 * would deadlock against an enqueuer: mla_sched_enqueue() increments
	 * outstanding BEFORE calling mla_model_grow_banks(), which blocks on
	 * that same mutex — so the very job we are waiting for could never
	 * reach its decrement.
	 */
	if (atomic_read(&m->outstanding) != 0) {
		atomic_inc(&mdev->reloc_update_waits);
		if (mla_reloc_quiesce_grace_ms &&
		    !wait_event_timeout(m->reloc_quiesce_wq,
					atomic_read(&m->outstanding) == 0,
					msecs_to_jiffies(mla_reloc_quiesce_grace_ms)))
			atomic_inc(&mdev->reloc_update_wait_timeouts);
	}

	/* bank_grow_lock: serialise against mla_model_grow_banks(), the only
	 * other writer of a bank blob. Taken before sched_lock everywhere, and
	 * mla_sched_enqueue()'s grow attempt uses the same order.
	 *
	 * The re-check under sched_lock below remains the authoritative gate —
	 * the wait above only removes the spurious refusals, it does not
	 * establish quiescence.
	 */
	mutex_lock(&m->bank_grow_lock);
	spin_lock_irqsave(&mdev->sched_lock, flags);
	if (atomic_read(&m->outstanding) != 0) {
		spin_unlock_irqrestore(&mdev->sched_lock, flags);
		mutex_unlock(&m->bank_grow_lock);
		dev_err(mdev->dev,
			"MODEL_UPDATE_RELOC: model %u has %d outstanding job(s) — wait for their fences first\n",
			m->id, atomic_read(&m->outstanding));
		ret = -EBUSY;
		goto out_drop_new;
	}
	m->reloc_update_in_progress = true;
	spin_unlock_irqrestore(&mdev->sched_lock, flags);

	/* Test hook only. The window between setting the flag and clearing it
	 * is a few microseconds of descriptor RMW, which no userspace test can
	 * reliably aim a submit at; widening it on demand is what makes the
	 * submit-refusal half of this gate testable rather than assumed. Sleeps
	 * with the flag SET and bank_grow_lock held, which is exactly the state
	 * a racing submit has to meet. Never set in production.
	 */
	if (unlikely(mla_inject_reloc_delay_ms)) {
		dev_warn(mdev->dev,
			 "MODEL_UPDATE_RELOC: mla_inject_reloc_delay_ms=%u — holding the update window open on model %u\n",
			 mla_inject_reloc_delay_ms, m->id);
		msleep(mla_inject_reloc_delay_ms);
	}

	for (p = 0; p < req.num_reloc; p++) {
		oldbufs[p] = m->reloc_bufs[p];
		oldoffs[p] = m->reloc_offsets[p];
		if (!newbufs[p])
			continue;
		m->reloc_bufs[p]    = newbufs[p];
		m->reloc_offsets[p] = user_arr[p].offset;
	}

	/* passes counts banks whose descriptors were actually REWRITTEN, not
	 * banks walked. mla_model_patch_reloc_ports() returns 0 for a walk
	 * that touched nothing too, so counting iterations made
	 * resp.patch_passes >= 1 unconditionally (num_banks is never 0) —
	 * which made the zero the uapi tells callers to assert on
	 * unreachable, and left it disagreeing with the debugfs
	 * reloc_patch_passes counter the tests gate on.
	 */
	for (bank = 0; bank < m->num_banks; bank++) {
		u32 n = 0;

		ret = mla_model_patch_reloc_ports(m, bank, &n);
		if (ret)
			break;
		if (n)
			passes++;
	}

	if (ret) {
		/* Only reachable through internal inconsistency — every bank is
		 * a same-sized copy of the primary, and the bounds these checks
		 * make were already satisfied when MODEL_DEFINE spliced bank 0.
		 * Roll back anyway: some banks now hold the new addresses and
		 * some the old, and dropping the displaced references in that
		 * state could free memory a descriptor still names.
		 */
		dev_err(mdev->dev,
			"MODEL_UPDATE_RELOC: model %u patch failed on bank %u (rc=%d) — rolling back\n",
			m->id, bank, ret);
		for (p = 0; p < req.num_reloc; p++) {
			m->reloc_bufs[p]    = oldbufs[p];
			m->reloc_offsets[p] = oldoffs[p];
		}
		for (bank = 0; bank < m->num_banks; bank++) {
			u32 n = 0;
			int rb = mla_model_patch_reloc_ports(m, bank, &n);

			if (rb)
				dev_err(mdev->dev,
					"MODEL_UPDATE_RELOC: model %u rollback of bank %u failed (rc=%d) — descriptors may name a stale adapter\n",
					m->id, bank, rb);
		}
	}

	spin_lock_irqsave(&mdev->sched_lock, flags);
	m->reloc_update_in_progress = false;
	spin_unlock_irqrestore(&mdev->sched_lock, flags);
	mutex_unlock(&m->bank_grow_lock);

	if (ret)
		goto out_drop_new;

	/* Drop the displaced references LAST, after the patch and the flush.
	 * The other order frees memory the descriptors still name whenever the
	 * displaced buffer was at its last reference, and the very next job
	 * then DMAs from freed pages. A default binding survives this because
	 * the model also holds it as a section, but nothing here relies on
	 * that.
	 */
	for (p = 0; p < req.num_reloc; p++)
		if (newbufs[p] && oldbufs[p])
			mla_buffer_drop(oldbufs[p]);

	atomic_inc(&mdev->reloc_binds);
	atomic_add(updated, &mdev->reloc_ports_bound);

	dev_dbg(mdev->dev,
		"MODEL_UPDATE_RELOC: model %u ports_updated=%u banks_patched=%u\n",
		m->id, updated, passes);

out_respond:
	memset(&resp, 0, sizeof(resp));
	resp.ports_updated = updated;
	resp.patch_passes  = passes;
	{
		size_t n = min_t(size_t, (size_t)req.out_len, sizeof(resp));

		/* Report what the kernel actually WROTE, not sizeof(resp).
		 * out_len may legally be as small as offsetofend(resp,
		 * patch_passes), and a caller trusting struct_size to say which
		 * fields were populated would otherwise read its own
		 * uninitialised memory as kernel-supplied data — including the
		 * reserved[] the uapi specifies as must-be-0.
		 */
		resp.struct_size = (u32)n;
		if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, n))
			ret = -EFAULT;
	}
	/* The bindings are installed either way: a failed copy_to_user does
	 * not un-patch the descriptors, and pretending otherwise would need a
	 * second rollback for a case the caller can simply re-issue.
	 */
	goto out_free;

out_drop_new:
	for (p = 0; p < req.num_reloc; p++)
		if (newbufs[p])
			mla_buffer_drop(newbufs[p]);
out_free:
	kfree(oldoffs);
	kfree(oldbufs);
	kfree(newbufs);
	kfree(user_arr);
out_put_model:
	mla_model_put(m);
	return ret;
}

/**
 * mla_op_job_submit() — handle MLA_OP_JOB_SUBMIT
 * @ctx: context that owns the model and buffers
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Builds a struct mla_job from the user request, resolves IFM/OFM
 * tensor descriptors to in-context buffer pointers, allocates a
 * dma-fence and (in async mode) a sync_file fd backed by it, then
 * enqueues the job in the priority-ordered run queue.
 *
 * Sync mode (MLA_CMD_FLAG_WAIT): blocks in
 * wait_for_completion_interruptible_timeout until the job completes
 * or the timeout fires; the request returns the job's result errno.
 *
 * Async mode: returns immediately. Caller polls completion_fence_fd
 * (a sync_file) for the result.
 *
 * Return: 0 on success (sync mode: 0 means PROGDONE; async: 0 means
 * accepted onto the queue); -ENOENT if model_id unknown; -EINVAL for
 * malformed args; -EBUSY/-ENOMEM/-ETIMEDOUT for the obvious cases.
 */
int mla_op_job_submit(struct mla_context *ctx,
		      const struct mla_uring_cmd_sqe *sqe,
		      const struct mla_cmd_hdr *hdr)
{
	struct mla_job_submit_req  req;
	struct mla_job_submit_resp resp;
	struct mla_iotensor *ifm_user = NULL, *ofm_user = NULL;
	struct mla_model *model;
	struct mla_job   *job;
	struct sync_file *sync;
	int fence_fd = -1;
	u32 id;
	int ret;
	bool sync_mode;
	ktime_t ts_op_enter = ktime_get();
	ktime_t ts_op_exit;
	ktime_t ts_after_cfu  = ts_op_enter;
	ktime_t ts_after_resv = ts_op_enter;
	ktime_t ts_after_offs = ts_op_enter;

	/* Additive tail: accept the pre-v7 struct (which ended at reserved2)
	 * and anything longer, zero-filling what the caller did not send. See
	 * the tail comment on struct mla_job_submit_req.
	 */
	if (sqe->req_len < offsetofend(struct mla_job_submit_req, reserved2)) {
		dev_info(ctx->mdev->dev,
			 "JOB_SUBMIT: EINVAL sqe->req_len=%u min=%zu\n",
			 sqe->req_len,
			 offsetofend(struct mla_job_submit_req, reserved2));
		return -EINVAL;
	}
	memset(&req, 0, sizeof(req));
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr),
			   min_t(size_t, (size_t)sqe->req_len, sizeof(req))))
		return -EFAULT;
	if (req.reserved3)
		return -EINVAL;
	if (req.num_reloc) {
		/* Per-job reloc bindings are ABI-complete but not implemented:
		 * accepting them silently is the exact failure mode
		 * MLA_RELOC_ZONE_RELOC already produced once (job succeeds,
		 * output is garbage). MLA_OP_MODEL_UPDATE_RELOC does the same
		 * job between submissions.
		 */
		dev_err_ratelimited(ctx->mdev->dev,
				    "JOB_SUBMIT: per-job reloc bindings (num_reloc=%u) need MLA_CAP_RELOC_PER_JOB; use MLA_OP_MODEL_UPDATE_RELOC\n",
				    req.num_reloc);
		return -EOPNOTSUPP;
	}
	if (!req.out_ptr || req.out_len < sizeof(resp)) {
		dev_info(ctx->mdev->dev,
			 "JOB_SUBMIT: EINVAL out_ptr=0x%llx out_len=%u sizeof(resp)=%zu\n",
			 (unsigned long long)req.out_ptr, req.out_len, sizeof(resp));
		return -EINVAL;
	}
	if (req.num_ifm > MLA_MAX_IOTENSORS || req.num_ofm > MLA_MAX_IOTENSORS) {
		dev_info(ctx->mdev->dev,
			 "JOB_SUBMIT: EINVAL num_ifm=%u num_ofm=%u (cap %u)\n",
			 req.num_ifm, req.num_ofm, MLA_MAX_IOTENSORS);
		return -EINVAL;
	}
	if (req.wait_fence_fd != -1)
		return -EOPNOTSUPP;

	sync_mode = !!(hdr->flags & MLA_CMD_FLAG_WAIT);

	/* Take the model reference UNDER xa_lock, so a concurrent MODEL_RELEASE
	 * cannot erase and destroy it between the lookup and the increment.
	 * refcount_inc_not_zero (not refcount_inc) so a model already at zero
	 * and awaiting its deferred destroy reads as gone rather than being
	 * resurrected. Same protocol as JOB_CANCEL's fence get under
	 * xa_lock(&ctx->jobs). Dropped either by mla_fence_release once the job
	 * owns it, or by the err_free_ifm unwind below if we never get there.
	 */
	xa_lock(&ctx->models);
	model = xa_load(&ctx->models, req.model_id);
	if (model && !refcount_inc_not_zero(&model->refcount))
		model = NULL;
	xa_unlock(&ctx->models);
	if (!model) {
		dev_info(ctx->mdev->dev,
			 "JOB_SUBMIT: ENOENT model_id=%u not in ctx->models\n",
			 req.model_id);
		return -ENOENT;
	}
	/* Userspace tensor arrays, heap-allocated: at MLA_MAX_IOTENSORS these
	 * arrays are large enough that putting them on the stack would blow
	 * the frame-size warning and risk overflow.
	 */
	if (req.num_ifm) {
		ifm_user = kcalloc(req.num_ifm, sizeof(*ifm_user),
				   GFP_KERNEL);
		if (!ifm_user) {
			/* Route through the unwind rather than returning
			 * directly: the model reference taken above still
			 * has to be dropped. kfree(NULL) there is fine.
			 */
			ret = -ENOMEM;
			goto err_free_ifm;
		}
		if (copy_from_user(ifm_user, u64_to_user_ptr(req.ifm_ptr),
				   req.num_ifm * sizeof(*ifm_user))) {
			ret = -EFAULT;
			goto err_free_ifm;
		}
	}
	if (req.num_ofm) {
		ofm_user = kcalloc(req.num_ofm, sizeof(*ofm_user),
				   GFP_KERNEL);
		if (!ofm_user) {
			ret = -ENOMEM;
			goto err_free_ifm;
		}
		if (copy_from_user(ofm_user, u64_to_user_ptr(req.ofm_ptr),
				   req.num_ofm * sizeof(*ofm_user))) {
			ret = -EFAULT;
			goto err_free_ofm;
		}
	}
	ts_after_cfu = ktime_get();

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job) {
		ret = -ENOMEM;
		goto err_free_ofm;
	}
	/* Counted ref: a running job outlives the fd (its fence holds it),
	 * and job->ctx->mdev is dereferenced from the HW/completion path.
	 * Released in mla_fence_release.
	 */
	job->ctx            = mla_context_get(ctx);
	job->model          = model;
	job->cycle_count    = req.cycle_count;
	job->timeout_ms     = req.timeout_ms;
	job->num_ifm        = req.num_ifm;
	job->num_ofm        = req.num_ofm;
	job->sync_wait      = sync_mode;
	mla_job_init_internal(job, ctx->mdev);
	/* Ownership of the model reference has transferred to the job, and the
	 * fence now exists, so every failure from here on unwinds through
	 * err_free_job's dma_fence_put → mla_fence_release, which drops it.
	 * NULL the local so the err_free_ifm put below cannot double-put: the
	 * unwind labels chain downward, so err_free_job's fence put is followed
	 * by err_free_ofm and err_free_ifm on the very same path.
	 */
	model = NULL;

	/* Bind IFM/OFM buffer arrays; heap-allocated for the same frame-size
	 * reason as the tensor arrays above.
	 */
	if (req.num_ifm) {
		job->ifm_bufs = kcalloc(req.num_ifm,
					sizeof(*job->ifm_bufs),
					GFP_KERNEL);
		if (!job->ifm_bufs) {
			ret = -ENOMEM;
			goto err_free_job;
		}
	}
	if (req.num_ofm) {
		job->ofm_bufs = kcalloc(req.num_ofm,
					sizeof(*job->ofm_bufs),
					GFP_KERNEL);
		if (!job->ofm_bufs) {
			ret = -ENOMEM;
			goto err_free_ifm_bufs;
		}
	}
	if (req.num_ifm) {
		ret = mla_resolve_iotensors(ctx, ifm_user, req.num_ifm,
					    job->ifm_bufs);
		if (ret)
			goto err_free_ofm_bufs;
	}
	if (req.num_ofm) {
		ret = mla_resolve_iotensors(ctx, ofm_user, req.num_ofm,
					    job->ofm_bufs);
		if (ret)
			goto err_put_bindings;
	}
	ts_after_resv = ktime_get();

	/* Stash per-tensor offsets for reloc patching. mla_hw_apply_relocs
	 * treats NULL as "all zero" and skips the per-tensor add — kcalloc
	 * only when needed.
	 */
	{
		u32 ti;
		bool any_ifm_off = false, any_ofm_off = false;

		for (ti = 0; ti < req.num_ifm; ti++) {
			if (ifm_user[ti].offset) {
				any_ifm_off = true;
				break;
			}
		}
		for (ti = 0; ti < req.num_ofm; ti++) {
			if (ofm_user[ti].offset) {
				any_ofm_off = true;
				break;
			}
		}
		if (any_ifm_off) {
			job->ifm_offsets = kcalloc(req.num_ifm,
						   sizeof(*job->ifm_offsets),
						   GFP_KERNEL);
			if (!job->ifm_offsets) {
				ret = -ENOMEM;
				goto err_put_bindings;
			}
			for (ti = 0; ti < req.num_ifm; ti++)
				job->ifm_offsets[ti] = ifm_user[ti].offset;
		}
		if (any_ofm_off) {
			job->ofm_offsets = kcalloc(req.num_ofm,
						   sizeof(*job->ofm_offsets),
						   GFP_KERNEL);
			if (!job->ofm_offsets) {
				ret = -ENOMEM;
				goto err_free_ifm_offsets;
			}
			for (ti = 0; ti < req.num_ofm; ti++)
				job->ofm_offsets[ti] = ofm_user[ti].offset;
		}
	}
	ts_after_offs = ktime_get();

	if (mla_debug & MLA_DBG_TRACE)
		dev_info(ctx->mdev->dev,
			 "v2_trace: JOB_SUBMIT model_id=%u num_ifm=%u num_ofm=%u num_sections=%u reloc_count=%u static_reloc_count=%u\n",
			 req.model_id, req.num_ifm, req.num_ofm,
			 job->model->num_sections,
			 job->model->reloc_count,
			 job->model->static_reloc_count);
	if (mla_debug & MLA_DBG_HOT) {
		u32 ti;

		for (ti = 0; ti < req.num_ifm; ti++)
			dev_info(ctx->mdev->dev,
				 "v2_trace:   ifm[%u] buf_id=%u phys=0x%llx size=%zu\n",
				 ti, ifm_user[ti].internal_buf_id,
				 (unsigned long long)job->ifm_bufs[ti]->dma_addr,
				 (size_t)job->ifm_bufs[ti]->size);
		for (ti = 0; ti < req.num_ofm; ti++)
			dev_info(ctx->mdev->dev,
				 "v2_trace:   ofm[%u] buf_id=%u phys=0x%llx size=%zu\n",
				 ti, ofm_user[ti].internal_buf_id,
				 (unsigned long long)job->ofm_bufs[ti]->dma_addr,
				 (size_t)job->ofm_bufs[ti]->size);
		if (job->model->section_bufs) {
			for (ti = 0; ti < job->model->num_sections; ti++) {
				struct mla_buffer *b = job->model->section_bufs[ti];

				if (!b) {
					dev_info(ctx->mdev->dev,
						 "v2_trace:   sec[%u] = NULL\n", ti);
					continue;
				}
				dev_info(ctx->mdev->dev,
					 "v2_trace:   sec[%u] buf_id=%u phys=0x%llx size=%zu cpu_vaddr=%p\n",
					 ti, b->id,
					 (unsigned long long)b->dma_addr,
					 (size_t)b->size, b->cpu_vaddr);
			}
		}
	}

	/* Register in xarray only for async mode. Sync-mode jobs are not
	 * externally observed by userspace (no cancel via job_id, no fd) —
	 * the syscall thread holds a live pointer via the sync_done wait
	 * and mla_sched_complete looks the job up via mdev->in_flight, not
	 * via the xarray. Skipping xa_alloc saves ~2-3µs off kprep on the
	 * common path. Async keeps the xarray entry so job_cancel and
	 * context_jobs_release can find pending work.
	 */
	if (!sync_mode) {
		ret = xa_alloc(&ctx->jobs, &id, job, xa_limit_32b, GFP_KERNEL);
		if (ret < 0)
			goto err_put_bindings;
		job->id = id;

		sync = sync_file_create(&job->fence);
		if (!sync) {
			ret = -ENOMEM;
			goto err_xa_erase;
		}
		fence_fd = get_unused_fd_flags(O_CLOEXEC);
		if (fence_fd < 0) {
			fput(sync->file);   /* drops fence ref */
			ret = fence_fd;
			goto err_xa_erase;
		}
		fd_install(fence_fd, sync->file);
	}

	/* Hold an extra fence reference for the kernel side until the
	 * job leaves the run queue / completes.
	 */
	dma_fence_get(&job->fence);

	job->ts_submit = ktime_get();
	mla_sched_enqueue(job);

	/* Sync mode: wait in-kernel until PROGDONE IRQ signals the fence.
	 * TASK_UNINTERRUPTIBLE (not _interruptible_) so a pending
	 * TIF_NOTIFY_SIGNAL doesn't abort the wait with -ERESTARTSYS.
	 * The kernel watchdog timer (mla_job_arm_timeout, default 2000 ms)
	 * bounds the wait on the HW side; the +500 ms here is a belt
	 * around it.
	 */
	if (sync_mode) {
		long w = wait_for_completion_timeout(
			&job->sync_done,
			msecs_to_jiffies((job->timeout_ms ? job->timeout_ms :
					  MLA_JOB_DEFAULT_TIMEOUT_MS) + 500));
		if (w == 0) {
			/* The +500 ms belt expired without the watchdog having
			 * finalized this job. Finalize it HERE rather than just
			 * reporting -ETIMEDOUT: this path drops both fence refs
			 * below, and dropping the last one frees the job. Without
			 * a finish, that free leaves mdev->in_flight pointing at
			 * freed memory, the fence unsignalled, and — the fatal
			 * part — the watchdog timer still armed, so call_timer_fn
			 * later branches into a recycled object (LIST_POISON2 in
			 * x23, __run_timers in the trace).
			 *
			 * Safe if the watchdog or IRQ won the race concurrently:
			 * mla_sched_finish_job short-circuits on an
			 * already-finalized job, so this is at worst a no-op.
			 *
			 * The job reaching here is very often still QUEUED — the
			 * belt expires precisely when nothing dispatched it — so
			 * it is still linked on mdev->sched_queue. Unlinking is
			 * mla_sched_finish_job's job and must stay there: plist
			 * membership takes no fence reference, so the two puts
			 * below would otherwise free a job the next queue walk
			 * still reaches. A sync job is invisible to the other
			 * drains (submit skips xa_alloc for sync mode, so it is
			 * not in ctx->jobs), which makes this the ONLY finalizer
			 * it ever gets.
			 */
			mla_sched_complete(ctx->mdev, job, -ETIMEDOUT);
		}
		/* Read the result AFTER finalizing, so a completion that raced
		 * the belt reports its real terminal value rather than a
		 * blanket -ETIMEDOUT. finish_job sets state and result together
		 * before signalling, so a signalled fence implies result is set.
		 */
		ret = job->result;

		/* Quiesce the watchdog before this frame goes away.
		 *
		 * mla_fence_release walks ifm_bufs/ofm_bufs via
		 * mla_job_put_bindings. That is only safe while release runs
		 * in this thread, which holds as long as this thread performs
		 * the last fence put.
		 *
		 * On a normal completion it does: finish_job cancels a still-
		 * pending timer and drops its reference before waking us, so no
		 * timer reference is outstanding here. On a TIMEOUT the callback
		 * is what completed the job — it woke us from inside itself and
		 * still holds the reference it drops on the way out. Cancelling
		 * cannot help: the timer already fired, so
		 * timer_delete_sync_try just reports -1. If we returned now and
		 * that put landed last, release would run in softirq against a
		 * popped frame.
		 *
		 * So wait. Short in practice — the callback's printks and
		 * register dump all precede the wake, leaving only bookkeeping,
		 * dispatch_next and its put. Recovery itself runs on a
		 * workqueue, not here.
		 */
		mla_job_cancel_timeout_sync(job);
	}

	ts_op_exit = ktime_get();
	memset(&resp, 0, sizeof(resp));
	resp.job_id              = job->id;
	resp.completion_fence_fd = fence_fd;
	resp.latency_us          = job->latency_us;

	/* Phase trace deltas, only meaningful in sync mode after a
	 * successful run (otherwise some stamps are zero). out_len
	 * gates the write — older callers stay binary-compatible.
	 */
	if (sync_mode && ret == 0 &&
	    req.out_len >= sizeof(resp)) {
		resp.trace_submit_to_dispatch_us =
			ktime_to_us(ktime_sub(job->ts_dispatch, job->ts_submit));
		resp.trace_dispatch_to_go_us =
			ktime_to_us(ktime_sub(job->ts_go, job->ts_dispatch));
		resp.trace_go_to_irq_us =
			ktime_to_us(ktime_sub(job->ts_irq, job->ts_go));
		resp.trace_irq_to_wake_us =
			ktime_to_us(ktime_sub(job->ts_wake, job->ts_irq));
		resp.trace_relocs_us =
			ktime_to_us(ktime_sub(job->ts_after_relocs, job->ts_dispatch));
		resp.trace_dma_sync_us =
			ktime_to_us(ktime_sub(job->ts_after_sync, job->ts_after_relocs));
		resp.trace_program_us =
			ktime_to_us(ktime_sub(job->ts_go, job->ts_after_sync));
		/* TLC perf counters sampled in IRQ. */
		resp.perf_ctr_us[0] = job->perf_ctr_us[0];
		resp.perf_ctr_us[1] = job->perf_ctr_us[1];
		resp.perf_ctr_us[2] = job->perf_ctr_us[2];
		resp.perf_ctr_us[3] = job->perf_ctr_us[3];
		/* Op-entry/exit wrapper split — where does the JOB_SUBMIT-
		 * specific overhead (beyond ksub/kdsp/kgo/kwk) live between
		 * the command handler entry and the ts_submit stamp /
		 * ts_wake stamp and copy_to_user?
		 */
		resp.trace_op_prep_us =
			ktime_to_us(ktime_sub(job->ts_submit, ts_op_enter));
		resp.trace_op_post_us =
			ktime_to_us(ktime_sub(ts_op_exit, job->ts_wake));
		/* krlc split: setup (function prologue + desc-section
		 * resolution + optional trace print) vs. per-reloc loop.
		 */
		if (job->model->reloc_count) {
			resp.trace_reloc_setup_us =
				ktime_to_us(ktime_sub(job->ts_reloc_loop_start,
						      job->ts_dispatch));
			resp.trace_reloc_loop_us =
				ktime_to_us(ktime_sub(job->ts_after_relocs,
						      job->ts_reloc_loop_start));
		}
		/* kprep sub-split. See mla.h. */
		resp.trace_kprep_cfu_us =
			ktime_to_us(ktime_sub(ts_after_cfu,  ts_op_enter));
		resp.trace_kprep_resv_us =
			ktime_to_us(ktime_sub(ts_after_resv, ts_after_cfu));
		resp.trace_kprep_offs_us =
			ktime_to_us(ktime_sub(ts_after_offs, ts_after_resv));
		resp.trace_kprep_xa_us =
			ktime_to_us(ktime_sub(job->ts_submit, ts_after_offs));
	}

	/* Fault diagnostics — unconditional (zero on success). */
	resp.fault_class   = job->fault_class;
	resp.fault_sys_sts = job->fault_sys_sts;
	resp.fault_l2g_sts[0] = job->fault_l2g_sts[0];
	resp.fault_l2g_sts[1] = job->fault_l2g_sts[1];
	resp.fault_l2g_sts[2] = job->fault_l2g_sts[2];
	resp.fault_l2g_sts[3] = job->fault_l2g_sts[3];

	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp))) {
		dma_fence_put(&job->fence);
		ret = -EFAULT;
	} else {
		/* Drop our extra reference now that response is in user space. */
		dma_fence_put(&job->fence);
	}
	/* Sync-mode jobs have no external observer (no xarray entry, no
	 * sync_file fd) — the original fence ref from dma_fence_init must
	 * be dropped here or the mla_job (including its arm_timeout timer)
	 * leaks for the module's lifetime. Async mode releases this ref via
	 * the xarray-erase path in MLA_OP_JOB_WAIT or on ctx teardown.
	 */
	if (sync_mode)
		dma_fence_put(&job->fence);
	/* Both modes heap-allocate these, so the free is unconditional on
	 * sync_mode. kfree(NULL) is defined, so num_ifm/num_ofm == 0 needs no
	 * guard.
	 */
	kfree(ifm_user);
	kfree(ofm_user);
	return ret;

	/* DOUBLE FREE, and it corrupts the slab freelist, if not nulled:
	 * err_free_job below does dma_fence_put(), which drops the last ref
	 * and runs mla_fence_release() -- and that frees these SAME four
	 * pointers again. Nulling as we go makes the second pass a no-op
	 * (kfree(NULL) is defined) instead of a second free of a live object.
	 * Do NOT drop the nulling: this unwind and the fence release are
	 * two independent owners of the same pointers by construction.
	 */
err_xa_erase:
	xa_erase(&ctx->jobs, job->id);
	kfree(job->ofm_offsets);
	job->ofm_offsets = NULL;
err_free_ifm_offsets:
	kfree(job->ifm_offsets);
	job->ifm_offsets = NULL;
err_put_bindings:
	/* Release the per-tensor references mla_resolve_iotensors took. MUST be
	 * before the kfree+NULL below: those lines are why a put_bindings in
	 * mla_fence_release alone cannot cover this path — by the time the fence
	 * put at err_free_job runs, the arrays are already gone and every
	 * reference taken on an error path would leak (delta > 0). Idempotent,
	 * so the second call from mla_fence_release is a no-op.
	 *
	 * Reached by every failure AFTER a successful resolve, including the
	 * OFM-resolve failure (IFM references are held by then). The IFM-resolve
	 * failure deliberately jumps PAST this to err_free_ofm_bufs because it
	 * holds no references: mla_resolve_iotensors drops every reference it
	 * took and re-zeroes the array before returning an error, so walking a
	 * zeroed heap array here would be a harmless no-op anyway.
	 */
	mla_job_put_bindings(job);
err_free_ofm_bufs:
	kfree(job->ofm_bufs);
	job->ofm_bufs = NULL;
err_free_ifm_bufs:
	kfree(job->ifm_bufs);
	job->ifm_bufs = NULL;
err_free_job:
	dma_fence_put(&job->fence);   /* drops the original ref → release */
err_free_ofm:
	kfree(ofm_user);
err_free_ifm:
	kfree(ifm_user);
	/* Model reference from the xa_lock'd lookup, for the failures that
	 * happen BEFORE the job exists. Non-NULL only on those paths: it is
	 * NULLed at mla_job_init_internal once the job owns it, after which
	 * mla_fence_release is the single drop point. Guarding on the pointer
	 * rather than adding a separate label is what keeps this correct
	 * despite the chained labels — the same "two independent owners of one
	 * pointer" shape that produced the binding double-free.
	 */
	if (model)
		mla_model_put(model);
	return ret;
}

/*
 * mla_async_fill_resp() — build a mla_job_submit_resp for JOB_ASYNC.
 * Mirrors the sync-mode response fill from mla_op_job_submit's tail;
 * the code path is separated because JOB_ASYNC runs it from task_work
 * context, not from the syscall thread. Kept memcpy-simple so a future
 * range-scoped resp layout change lands in one place.
 */
static void mla_async_fill_resp(struct mla_job *job, int result,
			       struct mla_job_submit_resp *resp)
{
	memset(resp, 0, sizeof(*resp));
	resp->job_id              = job->id;
	resp->completion_fence_fd = -1;
	resp->latency_us          = job->latency_us;

	if (result == 0) {
		resp->trace_submit_to_dispatch_us =
			ktime_to_us(ktime_sub(job->ts_dispatch, job->ts_submit));
		resp->trace_dispatch_to_go_us =
			ktime_to_us(ktime_sub(job->ts_go, job->ts_dispatch));
		resp->trace_go_to_irq_us =
			ktime_to_us(ktime_sub(job->ts_irq, job->ts_go));
		resp->trace_irq_to_wake_us =
			ktime_to_us(ktime_sub(job->ts_wake, job->ts_irq));
		resp->trace_relocs_us =
			ktime_to_us(ktime_sub(job->ts_after_relocs, job->ts_dispatch));
		resp->trace_dma_sync_us =
			ktime_to_us(ktime_sub(job->ts_after_sync, job->ts_after_relocs));
		resp->trace_program_us =
			ktime_to_us(ktime_sub(job->ts_go, job->ts_after_sync));
		resp->perf_ctr_us[0] = job->perf_ctr_us[0];
		resp->perf_ctr_us[1] = job->perf_ctr_us[1];
		resp->perf_ctr_us[2] = job->perf_ctr_us[2];
		resp->perf_ctr_us[3] = job->perf_ctr_us[3];
		resp->trace_op_prep_us =
			ktime_to_us(ktime_sub(job->ts_submit, job->ts_op_enter));
		/* trace_op_post_us: task_work runs after ts_wake so include
		 * the wake→callback gap. Approximate with ktime_get() at
		 * fill time.
		 */
		resp->trace_op_post_us =
			ktime_to_us(ktime_sub(ktime_get(), job->ts_wake));
		if (job->model->reloc_count) {
			resp->trace_reloc_setup_us =
				ktime_to_us(ktime_sub(job->ts_reloc_loop_start,
						      job->ts_dispatch));
			resp->trace_reloc_loop_us =
				ktime_to_us(ktime_sub(job->ts_after_relocs,
						      job->ts_reloc_loop_start));
		}
	}

	resp->fault_class     = job->fault_class;
	resp->fault_sys_sts   = job->fault_sys_sts;
	resp->fault_l2g_sts[0] = job->fault_l2g_sts[0];
	resp->fault_l2g_sts[1] = job->fault_l2g_sts[1];
	resp->fault_l2g_sts[2] = job->fault_l2g_sts[2];
	resp->fault_l2g_sts[3] = job->fault_l2g_sts[3];
}

/*
 * mla_async_build_job() — async-job builder for the MLA_IOCTL_SUBMIT_ASYNC
 * path. Validates and copies the request, resolves bindings, allocates the job
 * in ctx->jobs (init fence ref live in the xarray) and publishes job_id to the
 * caller's out_ptr. On success *out_job holds the job with ioctl_async false,
 * no extra fence ref taken and NOT yet enqueued — the caller attaches its
 * completion carrier (the ioctl reap queue), takes the extra ref, stamps
 * ts_submit and calls mla_sched_enqueue. On error (<0) the job is fully
 * unwound and *out_job is untouched.
 */
static int mla_async_build_job(struct mla_context *ctx,
			       u64 req_user_ptr, u32 req_len,
			       struct mla_job **out_job)
{
	struct mla_job_submit_req  req;
	struct mla_iotensor *ifm_user = NULL, *ofm_user = NULL;
	struct mla_model *model;
	struct mla_job   *job;
	u32 id;
	int ret;
	ktime_t ts_op_enter = ktime_get();

	/* Same additive-tail rule as JOB_SUBMIT — the two share the struct. */
	if (req_len < offsetofend(struct mla_job_submit_req, reserved2)) {
		dev_info(ctx->mdev->dev,
			 "%s: EINVAL req_len=%u min=%zu\n",
			 __func__, req_len,
			 offsetofend(struct mla_job_submit_req, reserved2));
		return -EINVAL;
	}
	memset(&req, 0, sizeof(req));
	if (copy_from_user(&req, u64_to_user_ptr(req_user_ptr),
			   min_t(size_t, (size_t)req_len, sizeof(req))))
		return -EFAULT;
	if (req.reserved3)
		return -EINVAL;
	if (req.num_reloc) {
		dev_err_ratelimited(ctx->mdev->dev,
				    "JOB_ASYNC: per-job reloc bindings (num_reloc=%u) need MLA_CAP_RELOC_PER_JOB; use MLA_OP_MODEL_UPDATE_RELOC\n",
				    req.num_reloc);
		return -EOPNOTSUPP;
	}
	if (!req.out_ptr || req.out_len < sizeof(struct mla_job_submit_resp)) {
		dev_info(ctx->mdev->dev,
			 "%s: EINVAL out_ptr=0x%llx out_len=%u sizeof(resp)=%zu\n",
			 __func__, (unsigned long long)req.out_ptr,
			 req.out_len,
			 sizeof(struct mla_job_submit_resp));
		return -EINVAL;
	}
	if (req.num_ifm > MLA_MAX_IOTENSORS || req.num_ofm > MLA_MAX_IOTENSORS)
		return -EINVAL;
	if (req.wait_fence_fd != -1)
		return -EOPNOTSUPP;

	/* Reference taken under xa_lock — see the matching comment in
	 * mla_op_job_submit. The window matters more here than on the sync
	 * path: an async job outlives this syscall entirely, so a MODEL_RELEASE
	 * arriving at any point during the job's life would otherwise destroy a
	 * model the scheduler and mla_hw still dereference.
	 */
	xa_lock(&ctx->models);
	model = xa_load(&ctx->models, req.model_id);
	if (model && !refcount_inc_not_zero(&model->refcount))
		model = NULL;
	xa_unlock(&ctx->models);
	if (!model)
		return -ENOENT;

	if (req.num_ifm) {
		ifm_user = kcalloc(req.num_ifm, sizeof(*ifm_user), GFP_KERNEL);
		if (!ifm_user) {
			/* Unwind the model reference, as below. */
			ret = -ENOMEM;
			goto err_free_ifm;
		}
		if (copy_from_user(ifm_user, u64_to_user_ptr(req.ifm_ptr),
				   req.num_ifm * sizeof(*ifm_user))) {
			ret = -EFAULT;
			goto err_free_ifm;
		}
	}
	if (req.num_ofm) {
		ofm_user = kcalloc(req.num_ofm, sizeof(*ofm_user), GFP_KERNEL);
		if (!ofm_user) {
			ret = -ENOMEM;
			goto err_free_ifm;
		}
		if (copy_from_user(ofm_user, u64_to_user_ptr(req.ofm_ptr),
				   req.num_ofm * sizeof(*ofm_user))) {
			ret = -EFAULT;
			goto err_free_ofm;
		}
	}

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job) {
		ret = -ENOMEM;
		goto err_free_ofm;
	}
	/* Counted ref: a running job outlives the fd (its fence holds it),
	 * and job->ctx->mdev is dereferenced from the HW/completion path.
	 * Released in mla_fence_release.
	 */
	job->ctx            = mla_context_get(ctx);
	job->model          = model;
	job->cycle_count    = req.cycle_count;
	job->timeout_ms     = req.timeout_ms;
	job->num_ifm        = req.num_ifm;
	job->num_ofm        = req.num_ofm;
	job->sync_wait      = false;
	/* The ioctl carrier (ioctl_async / ioctl_cookie) is attached by the
	 * caller after build; kzalloc leaves ioctl_async false until then.
	 */
	job->async_out_ptr   = req.out_ptr;
	job->async_out_len   = req.out_len;
	job->ts_op_enter    = ts_op_enter;
	mla_job_init_internal(job, ctx->mdev);
	/* Model reference now owned by the job and dropped by
	 * mla_fence_release; NULL the local so the err_free_ifm unwind cannot
	 * double-put through the chained unwind labels. See mla_op_job_submit.
	 */
	model = NULL;

	if (req.num_ifm) {
		job->ifm_bufs = kcalloc(req.num_ifm, sizeof(*job->ifm_bufs),
					GFP_KERNEL);
		if (!job->ifm_bufs) {
			ret = -ENOMEM;
			goto err_free_job;
		}
	}
	if (req.num_ofm) {
		job->ofm_bufs = kcalloc(req.num_ofm, sizeof(*job->ofm_bufs),
					GFP_KERNEL);
		if (!job->ofm_bufs) {
			ret = -ENOMEM;
			goto err_free_ifm_bufs;
		}
	}
	if (req.num_ifm) {
		ret = mla_resolve_iotensors(ctx, ifm_user, req.num_ifm,
					    job->ifm_bufs);
		if (ret)
			goto err_free_ofm_bufs;
	}
	if (req.num_ofm) {
		ret = mla_resolve_iotensors(ctx, ofm_user, req.num_ofm,
					    job->ofm_bufs);
		if (ret)
			goto err_put_bindings;
	}

	{
		u32 ti;
		bool any_ifm_off = false, any_ofm_off = false;

		for (ti = 0; ti < req.num_ifm; ti++) {
			if (ifm_user[ti].offset) {
				any_ifm_off = true;
				break;
			}
		}
		for (ti = 0; ti < req.num_ofm; ti++) {
			if (ofm_user[ti].offset) {
				any_ofm_off = true;
				break;
			}
		}
		if (any_ifm_off) {
			job->ifm_offsets = kcalloc(req.num_ifm,
						   sizeof(*job->ifm_offsets),
						   GFP_KERNEL);
			if (!job->ifm_offsets) {
				ret = -ENOMEM;
				goto err_put_bindings;
			}
			for (ti = 0; ti < req.num_ifm; ti++)
				job->ifm_offsets[ti] = ifm_user[ti].offset;
		}
		if (any_ofm_off) {
			job->ofm_offsets = kcalloc(req.num_ofm,
						   sizeof(*job->ofm_offsets),
						   GFP_KERNEL);
			if (!job->ofm_offsets) {
				ret = -ENOMEM;
				goto err_free_ifm_offsets;
			}
			for (ti = 0; ti < req.num_ofm; ti++)
				job->ofm_offsets[ti] = ofm_user[ti].offset;
		}
	}

	/* Cyclic, not xa_alloc: the ID outlives the job in userspace's
	 * hands (it cancels by ID), so prompt reuse would let a stale
	 * cancel hit an unrelated job. See mla_context.next_job_id.
	 */
	ret = xa_alloc_cyclic(&ctx->jobs, &id, job, xa_limit_32b,
			      &ctx->next_job_id, GFP_KERNEL);
	if (ret < 0)
		goto err_put_bindings;
	job->id = id;

	/* Publish job_id NOW so the caller can name this job to
	 * MLA_OP_JOB_CANCEL while it is still queued. mla_async_fill_resp
	 * writes the same value again at completion, which is where a
	 * caller would otherwise first learn it — far too late to cancel.
	 *
	 * Writing to userspace here is in-context: the submit path runs
	 * synchronously in the ioctl() caller, exactly like the three
	 * copy_from_user's this handler already does at the same point.
	 * req.out_ptr / out_len were validated at entry.
	 *
	 * Advertised by MLA_CAP_JOB_CANCEL; only the leading job_id field
	 * is touched, so the write is safe for any out_len we accepted.
	 */
	if (put_user(job->id,
		     &((struct mla_job_submit_resp __user *)
		       u64_to_user_ptr(req.out_ptr))->job_id)) {
		ret = -EFAULT;
		goto err_xa_erase;
	}

	/* Built. The caller attaches its completion carrier, takes the extra
	 * fence ref, stamps ts_submit and enqueues. The init fence ref is live
	 * in ctx->jobs and is released via the xarray-erase path on teardown or
	 * at the completion's terminal point (MLA_IOCTL_REAP).
	 */
	*out_job = job;
	kfree(ifm_user);
	kfree(ofm_user);
	return 0;

	/* Same double free as the sync path above -- see the comment there.
	 * dma_fence_put() at err_free_job runs mla_fence_release(), which
	 * frees these same pointers a second time. Null as we go.
	 */
err_xa_erase:
	/* Reached only from the put_user failure above, i.e. after
	 * xa_alloc_cyclic published the job. Remove it before the fence
	 * put below frees it, or the registry keeps a dangling entry that
	 * a subsequent JOB_CANCEL would happily dereference.
	 *
	 * next_job_id is deliberately NOT rewound: the cursor only has to
	 * move forward to keep IDs from being reused promptly, and rewinding
	 * would reintroduce exactly the reuse this design avoids.
	 */
	xa_erase(&ctx->jobs, job->id);
err_free_ifm_offsets:
	kfree(job->ifm_offsets);
	job->ifm_offsets = NULL;
err_put_bindings:
	/* Release the per-tensor references mla_resolve_iotensors took. MUST be
	 * before the kfree+NULL below: those lines are why a put_bindings in
	 * mla_fence_release alone cannot cover this path — by the time the fence
	 * put at err_free_job runs, the arrays are already gone and every
	 * reference taken on an error path would leak (delta > 0). Idempotent,
	 * so the second call from mla_fence_release is a no-op.
	 *
	 * Reached by every failure AFTER a successful resolve, including the
	 * OFM-resolve failure (IFM references are held by then). The IFM-resolve
	 * failure deliberately jumps PAST this to err_free_ofm_bufs: it holds no
	 * references, and ofm_bufs is a zeroed heap array at that point, so
	 * walking it here would be a harmless no-op.
	 */
	mla_job_put_bindings(job);
err_free_ofm_bufs:
	kfree(job->ofm_bufs);
	job->ofm_bufs = NULL;
err_free_ifm_bufs:
	kfree(job->ifm_bufs);
	job->ifm_bufs = NULL;
err_free_job:
	dma_fence_put(&job->fence);   /* drops the original init ref */
err_free_ofm:
	kfree(ofm_user);
err_free_ifm:
	kfree(ifm_user);
	/* Model reference, for failures before the job existed. NULL once the
	 * job owns it — see mla_job_init_internal above.
	 */
	if (model)
		mla_model_put(model);
	return ret;
}

/*
 * ===========================================================================
 * Ioctl-based async transport
 * ===========================================================================
 *
 * MLA_IOCTL_SUBMIT_ASYNC builds an async job whose completion carrier is the
 * job's dma-fence: a one-shot fence callback wakes ctx->ioctl_wq, and
 * MLA_IOCTL_REAP drains jobs whose fence has signalled. This never touches the
 * hard-IRQ completion path — for an async job (!sync_wait) mla_sched_notify_done
 * is a no-op and the fence is signalled by mla_sched_finish_job as for any job.
 */

/* Fence completion callback: mark the job done and wake the reaper. Runs from
 * the fence-signal context (possibly hard-IRQ) with fence->lock held, so it
 * takes no ctx->ioctl_lock and does not sleep — only a lock-free counter bump,
 * flag store and wakeup. The job and thus its ctx are alive here (the reap/drain
 * extra ref is still held). One-shot. The inc precedes the wakeup, so a reaper
 * that sleeps on an empty predicate is always re-woken once this fires.
 */
static void mla_ioctl_fence_cb(struct dma_fence *f, struct dma_fence_cb *cb)
{
	struct mla_job *job = container_of(cb, struct mla_job, ioctl_cb);

	(void)f;
	atomic_inc(&job->ctx->ioctl_signalled);
	WRITE_ONCE(job->ioctl_done, true);
	wake_up(&job->ctx->ioctl_wq);
}

/* True if any job on ctx->ioctl_inflight has completed. Used as the reap
 * waitqueue predicate — O(1): ioctl_cb bumps the counter as each fence signals.
 */
static bool mla_ioctl_any_signaled(struct mla_context *ctx)
{
	return atomic_read(&ctx->ioctl_signalled) > 0;
}

/**
 * mla_ioctl_submit_async() — enqueue a batch of async jobs in one syscall.
 * @s->count jobs are built from @s->descs; @s->count is overwritten with the
 * number actually enqueued. Stops at the first build error; returns that errno
 * only if nothing was enqueued, otherwise 0 (the caller sees the short count).
 */
int mla_ioctl_submit_async(struct mla_context *ctx, struct mla_ioctl_submit *s)
{
	struct mla_ioctl_submit_desc __user *udesc = u64_to_user_ptr(s->descs);
	u32 i, enqueued = 0;
	int ret = 0;

	if (!s->descs || !s->count)
		return -EINVAL;

	for (i = 0; i < s->count; i++) {
		struct mla_ioctl_submit_desc desc;
		struct mla_job *job;
		unsigned long flags;

		if (copy_from_user(&desc, &udesc[i], sizeof(desc))) {
			ret = -EFAULT;
			break;
		}
		if (desc.reserved) {
			ret = -EINVAL;
			break;
		}

		/* Reserve an inflight slot before building. The cap bounds the
		 * unreaped async jobs a context can pin — without it a client
		 * that submits without ever reaping drives unbounded allocation.
		 * Reserving before the (sleeping) build also caps the jobs in
		 * flight across concurrent submitters on one fd. Released below
		 * on build failure; otherwise the live job inherits the slot.
		 */
		spin_lock_irqsave(&ctx->ioctl_lock, flags);
		if (ctx->ioctl_inflight_count >= MLA_MAX_INFLIGHT_PER_CTX) {
			spin_unlock_irqrestore(&ctx->ioctl_lock, flags);
			ret = -EBUSY;
			break;
		}
		ctx->ioctl_inflight_count++;
		spin_unlock_irqrestore(&ctx->ioctl_lock, flags);

		ret = mla_async_build_job(ctx, desc.req_user_ptr, desc.req_len,
					  &job);
		if (ret < 0) {
			spin_lock_irqsave(&ctx->ioctl_lock, flags);
			ctx->ioctl_inflight_count--;
			spin_unlock_irqrestore(&ctx->ioctl_lock, flags);
			break;
		}

		job->ioctl_async  = true;
		job->ioctl_cookie = desc.cookie;
		/* Extra fence ref held by the reap queue; dropped in
		 * mla_ioctl_reap (or mla_context_ioctl_drain).
		 */
		dma_fence_get(&job->fence);

		/* Publish on the reap list and register the wakeup BEFORE
		 * enqueue, so the job cannot run (and signal) until it is both
		 * reapable and has a callback. At this point the fence is
		 * guaranteed unsignalled (nothing dispatches an unenqueued job),
		 * so add_callback cannot fail. The submit thread must not touch
		 * @job after mla_sched_enqueue: once enqueued it may complete and
		 * be reaped (and freed) on another CPU at any time.
		 */
		spin_lock_irqsave(&ctx->ioctl_lock, flags);
		list_add_tail(&job->ioctl_link, &ctx->ioctl_inflight);
		spin_unlock_irqrestore(&ctx->ioctl_lock, flags);

		dma_fence_add_callback(&job->fence, &job->ioctl_cb,
				       mla_ioctl_fence_cb);

		job->ts_submit = ktime_get();
		mla_sched_enqueue(job);

		enqueued++;
	}

	s->count = enqueued;
	return (enqueued == 0) ? ret : 0;
}

/**
 * mla_ioctl_reap() — collect completed async jobs for this context.
 * Waits per @r->timeout_ms for at least one completion, then drains up to
 * @r->max signalled jobs into @r->events, doing the async resp copy
 * and reference drops. @r->count is set to the number collected.
 */
int mla_ioctl_reap(struct mla_context *ctx, struct mla_ioctl_reap *r)
{
	struct mla_ioctl_event __user *uev = u64_to_user_ptr(r->events);
	struct mla_job *job, *tmp;
	LIST_HEAD(done);
	unsigned long flags;
	u32 collected = 0, freed = 0;
	int ret = 0;

	if (!r->events || !r->max)
		return -EINVAL;


	if (r->timeout_ms != 0) {
		if (r->timeout_ms < 0) {
			ret = wait_event_interruptible(ctx->ioctl_wq,
				ctx->shutdown || mla_ioctl_any_signaled(ctx));
			if (ret)
				return ret;   /* -ERESTARTSYS */
		} else {
			long tv = wait_event_interruptible_timeout(ctx->ioctl_wq,
				ctx->shutdown || mla_ioctl_any_signaled(ctx),
				msecs_to_jiffies(r->timeout_ms));
			if (tv < 0)
				return tv;    /* -ERESTARTSYS */
			if (tv == 0) {        /* timed out */
				r->count = 0;
				return 0;
			}
		}
	}

	/* Splice up to @max signalled jobs off the inflight list under the
	 * lock, then process them unlocked (copy_to_user can fault/sleep).
	 */
	spin_lock_irqsave(&ctx->ioctl_lock, flags);
	list_for_each_entry_safe(job, tmp, &ctx->ioctl_inflight, ioctl_link) {
		if (collected >= r->max)
			break;
		/* Pick on the cb-set flag, not the fence bit: the flag and
		 * ctx->ioctl_signalled are set together in ioctl_cb, so the
		 * counter the predicate reads stays consistent with what is
		 * reaped here.
		 */
		if (!READ_ONCE(job->ioctl_done))
			continue;
		list_del(&job->ioctl_link);
		list_add_tail(&job->ioctl_link, &done);
		collected++;
	}
	spin_unlock_irqrestore(&ctx->ioctl_lock, flags);

	collected = 0;
	list_for_each_entry_safe(job, tmp, &done, ioctl_link) {
		struct mla_job_submit_resp resp;
		struct mla_ioctl_event ev;
		int result = job->result;
		u32 id = job->id;

		/* Read every job field BEFORE the ref drops below — the last put
		 * runs mla_fence_release which NULLs ctx and kfrees the job.
		 */
		if (job->async_out_ptr &&
		    job->async_out_len >= sizeof(resp)) {
			mla_async_fill_resp(job, result, &resp);
			if (copy_to_user(u64_to_user_ptr(job->async_out_ptr),
					 &resp, sizeof(resp)))
				result = -EFAULT;
		}
		ev.cookie   = job->ioctl_cookie;
		ev.result   = result;
		ev.reserved = 0;
		if (copy_to_user(&uev[collected], &ev, sizeof(ev))) {
			/* Event buffer faulted. Leave this job and the rest of
			 * the drained batch reapable rather than freeing them
			 * here: splice them back so a later REAP delivers their
			 * completions. Events already copied (0..collected) stay
			 * delivered and their jobs freed below the loop.
			 */
			spin_lock_irqsave(&ctx->ioctl_lock, flags);
			list_splice_tail(&done, &ctx->ioctl_inflight);
			spin_unlock_irqrestore(&ctx->ioctl_lock, flags);
			ret = -EFAULT;
			break;
		}
		collected++;

		list_del(&job->ioctl_link);
		/* Terminal drops for an async job: whoever erases the
		 * registry entry owns the init ref; then drop the extra ref.
		 */
		if (xa_erase(&ctx->jobs, id))
			dma_fence_put(&job->fence);
		dma_fence_put(&job->fence);
		freed++;
	}

	/* Release the inflight reservations the freed jobs held, and drop their
	 * signalled count. The faulted batch spliced back above keeps both — its
	 * jobs are still live and still signalled, so a later reap re-collects
	 * them (the predicate stays true).
	 */
	spin_lock_irqsave(&ctx->ioctl_lock, flags);
	ctx->ioctl_inflight_count -= freed;
	spin_unlock_irqrestore(&ctx->ioctl_lock, flags);
	atomic_sub(freed, &ctx->ioctl_signalled);

	r->count = collected;
	return ret;
}

/**
 * mla_context_ioctl_drain() — release ioctl jobs still inflight at fd close.
 * Called from mla_context_shutdown() BEFORE mla_context_jobs_release(), so the
 * xarray still holds each job's init ref — dropping the extra ioctl ref here is
 * therefore never the last put on a live job. The fence callback is removed so
 * it cannot fire against a torn-down context; benign if it already ran.
 */
void mla_context_ioctl_drain(struct mla_context *ctx)
{
	struct mla_job *job, *tmp;
	LIST_HEAD(local);
	unsigned long flags;

	spin_lock_irqsave(&ctx->ioctl_lock, flags);
	list_splice_init(&ctx->ioctl_inflight, &local);
	ctx->ioctl_inflight_count = 0;   /* fd teardown; no submit/reap can race */
	atomic_set(&ctx->ioctl_signalled, 0);
	spin_unlock_irqrestore(&ctx->ioctl_lock, flags);

	list_for_each_entry_safe(job, tmp, &local, ioctl_link) {
		list_del(&job->ioctl_link);
		dma_fence_remove_callback(&job->fence, &job->ioctl_cb);
		dma_fence_put(&job->fence);   /* extra ioctl ref */
	}
}

/**
 * mla_op_job_cancel() — handle MLA_OP_JOB_CANCEL
 * @ctx: context that owns the job
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Drops a job from the run queue if it hasn't yet started running.
 * Once the hardware has been programmed for a job, the only way to
 * cancel is via the watchdog or fault paths.
 *
 * The terminal completion stays authoritative: this only dequeues. The
 * caller must still reap the job to collect its result and release the
 * handle — a cancel consumes nothing.
 *
 * Return: 0 if the job was dequeued and cancelled; -ENOENT if job_id
 * unknown to this context; -EBUSY if the job is already running OR
 * already terminal. Those two share an errno deliberately: hardware is
 * not preemptible mid-job, so both call for the same caller action —
 * reap and read the real result. See docs/JOB_LIFETIME_DESIGN.md §5.
 */
int mla_op_job_cancel(struct mla_context *ctx,
		      const struct mla_uring_cmd_sqe *sqe,
		      const struct mla_cmd_hdr *hdr)
{
	struct mla_job_cancel_req req;
	struct mla_job *job;
	int ret = -EBUSY;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;
	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;
	if (req.reserved)
		return -EINVAL;

	/* Take a fence reference while holding the xarray lock. Async jobs
	 * are erased and freed at their terminal point (MLA_IOCTL_REAP), so a
	 * bare xa_load would hand back a pointer that a concurrent
	 * completion can kfree before mla_sched_dequeue dereferences it.
	 *
	 * The fence IS the job's refcount — struct mla_job has no separate
	 * refcount_t, and mla_fence_release is its destructor. dma_fence_get
	 * is a plain atomic inc, so it is safe under the xarray spinlock.
	 */
	xa_lock(&ctx->jobs);
	job = xa_load(&ctx->jobs, req.job_id);
	if (job)
		dma_fence_get(&job->fence);
	xa_unlock(&ctx->jobs);

	if (!job)
		return -ENOENT;

	if (mla_sched_dequeue(job)) {
		mla_sched_complete(ctx->mdev, job, -ECANCELED);
		ret = 0;
	}

	/* Paired with the get above. Never the last put while the registry
	 * entry survives, and safe if it is: this runs in process context
	 * from the uring_cmd path.
	 */
	dma_fence_put(&job->fence);
	return ret;
}

/**
 * mla_op_device_reset() — handle MLA_OP_DEVICE_RESET
 * @ctx: context issuing the reset
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Returns the device to probe-equivalent state without a module reload:
 * cancels every in-flight and queued job on every open context with
 * -ECANCELED, resets the hardware, and clears mdev->device_fatal, the
 * recovery budget and every ctx->fatal.
 *
 * Permission: unprivileged when the caller is the ONLY open context,
 * CAP_SYS_ADMIN otherwise. A sole caller can only destroy its own work,
 * which is what lets an unprivileged test harness reset between cases;
 * with a peer attached the caller is destroying work it does not own.
 * Deliberately NOT CAP_SYS_RAWIO (MLA_OP_BUF_ATTACH_PHYS's gate) — that
 * would force root on every test that needs a reset.
 *
 * This is the one op reachable while the device is fatal; every other
 * op short-circuits in mla_cmd_prologue before its handler runs.
 *
 * Note the resp counters are what a test must assert on: resetting an
 * already-clean device is a legal no-op that also returns 0, so rc alone
 * cannot distinguish "recovered" from "was never broken".
 *
 * Return: 0 on success; -EINVAL on malformed request; -EFAULT on copy
 * failure; -EPERM if another context is open and the caller lacks
 * CAP_SYS_ADMIN; -ENODEV or mla_hw_recover()'s error if the hardware
 * did not come back (device left fatal).
 */
int mla_op_device_reset(struct mla_context *ctx,
			const struct mla_uring_cmd_sqe *sqe,
			const struct mla_cmd_hdr *hdr)
{
	struct mla_device_reset_req  req;
	struct mla_device_reset_resp resp;
	u32 jobs_cancelled = 0, ctxs_cleared = 0;
	bool was_device_fatal = false;
	int ret;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;

	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;

	if (req.reserved)
		return -EINVAL;

	/* Full-size response buffer required, matching every other op that
	 * returns counters (:890, :1675, :2379). This op's contract is that
	 * the COUNTERS, not rc, are what a caller asserts on (see above:
	 * resetting an already-clean device is a legal no-op that also
	 * returns 0), so a silently truncated response would be
	 * indistinguishable from a clean no-op reset.
	 *
	 * Rejected BEFORE mla_sched_device_reset() below, so a caller that got
	 * -EINVAL knows nothing was cancelled.
	 *
	 * MLA_OP_INFO's min_t() copyout (:154) is deliberately NOT changed:
	 * there the short write IS the version-negotiation contract.
	 */
	if (!req.out_ptr || req.out_len < sizeof(resp))
		return -EINVAL;

	/* > 1 open fd means someone else's jobs are about to be cancelled, so
	 * require CAP_SYS_ADMIN for that. Racy by nature — a context can open
	 * immediately after the check — but the race only widens the set of
	 * victims, and a caller that legitimately shares the device is
	 * expected to hold CAP_SYS_ADMIN anyway.
	 *
	 * This MUST read contexts_open, not num_contexts. num_contexts counts
	 * contexts still ALIVE, which includes any whose fd is already closed
	 * but whose jobs, models or buffers have not finished draining — and
	 * for an uncommitted BUF_ALLOC buffer that drain waits on the
	 * delayed_fput workqueue. Gating on it would return -EPERM to a caller
	 * that genuinely is the sole context, intermittently, precisely in
	 * the aftermath-of-a-crash case this op exists to clean up.
	 */
	if (atomic_read(&ctx->mdev->contexts_open) > 1 &&
	    !capable(CAP_SYS_ADMIN))
		return -EPERM;

	ret = mla_sched_device_reset(ctx->mdev, &jobs_cancelled,
				     &ctxs_cleared, &was_device_fatal);
	if (ret)
		return ret;

	memset(&resp, 0, sizeof(resp));
	resp.jobs_cancelled   = jobs_cancelled;
	resp.contexts_cleared = ctxs_cleared;
	resp.was_device_fatal = was_device_fatal ? 1 : 0;

	/* Full copy: out_len was validated at entry. No rollback on failure —
	 * unlike op 19's selection, a reset cannot be undone, and the entry
	 * state the caller cares about was captured in *was_device_fatal
	 * before anything changed.
	 */
	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp)))
		return -EFAULT;

	return 0;
}

/**
 * mla_op_perfctr_config() — handle MLA_OP_PERFCTR_CONFIG
 * @ctx: context issuing the request
 * @sqe: io_uring SQE inline payload
 * @hdr: already-copied request header
 *
 * Selects which TLC sync-controller signals subsequent jobs report in
 * mla_job_submit_resp.perf_ctr_us[], mirroring mla-rt's `-e N` flag.
 * signal_ids[i] is reported in perf_ctr_us[i]; count == 0 disables all
 * four slots and returns to the probe-time "report nothing" state.
 *
 * The counters are one device-wide register block with four usable slots
 * (the other four belong to quadgroup 2, which this SoC never runs), so
 * there is no per-context copy to hand out. Two gates make that safe
 * without inventing an arbitration protocol:
 *
 *   -EPERM  unless contexts_open == 1. Same counter as
 *           MLA_OP_DEVICE_RESET, and for the same reason it must read
 *           contexts_open rather than num_contexts — the latter counts
 *           contexts still alive including ones merely draining, which
 *           would return intermittent -EPERM to a genuinely sole caller.
 *
 *           Unlike op 18 there is deliberately NO CAP_SYS_ADMIN override.
 *           Root overriding a reset destroys work visibly and reports what
 *           it destroyed; root overriding a counter selection would
 *           silently give a peer context measurements of signals it never
 *           chose. Nothing is gained by permitting that, so the gate binds
 *           root as well.
 *   -EBUSY  unless the device is idle. Reprogramming src_sel under a
 *           running or queued job would give that job a window split
 *           across two different signals, with no way for the caller to
 *           tell, since perf_ctr_us[] carries no echo of what it measured.
 *
 *           ALSO -EBUSY when a recovery worker is pending or running. That
 *           is not redundant with the idle test: recovery_pending is set
 *           after the faulting job has already left in_flight, so the
 *           device can read idle while a reset is imminent, and
 *           mla_hw_recover() clears the selection. Installing one in that
 *           window returns success for work that is about to be discarded.
 *
 * Because the selection can only be installed while idle, the register
 * writes happen here and the per-job dispatch path is untouched.
 *
 * The idle test and the MMIO are done together under sched_lock so a
 * submit cannot slip in between them.
 *
 * resp.gen carries mdev->perfctr_gen as it stood after this install. The
 * kernel bumps that on every subsequent change to the selection — including
 * a recovery's software clear and a disable by a closing context — so a
 * caller can detect a silent wipe by comparing it against a later call's.
 * perf_ctr_us[] cannot express that itself: 0 is a legal reading.
 *
 * Return: 0 on success; -EINVAL on a malformed request (count > 4, signal
 * id > 7, duplicate ids, non-zero flags/reserved); -EFAULT on copy
 * failure; -EPERM with a peer context open; -EBUSY if the device is busy
 * or a recovery is pending.
 */
int mla_op_perfctr_config(struct mla_context *ctx,
			  const struct mla_uring_cmd_sqe *sqe,
			  const struct mla_cmd_hdr *hdr)
{
	struct mla_device *mdev = ctx->mdev;
	struct mla_perfctr_config_req  req;
	struct mla_perfctr_config_resp resp;
	u8 prev_signals[MLA_PERFCTR_MAX_SLOTS];
	unsigned long flags;
	u32 i, j, gen, prev_count;
	bool prev_owns;

	if (sqe->req_len != sizeof(req))
		return -EINVAL;

	if (copy_from_user(&req, u64_to_user_ptr(sqe->req_user_ptr), sizeof(req)))
		return -EFAULT;

	if (req.flags || req.reserved)
		return -EINVAL;

	/* Full-size response buffer required — same rationale as op 18 above,
	 * and here the echo is the whole point: without it a caller cannot
	 * distinguish "measured what I asked for" from "measured whatever was
	 * already programmed", and a truncated echo silently reintroduces that.
	 */
	if (!req.out_ptr || req.out_len < sizeof(resp))
		return -EINVAL;

	if (req.count > MLA_PERFCTR_MAX_SLOTS)
		return -EINVAL;

	/* Validate every id, and reject duplicates rather than silently
	 * collapsing them. Two slots on one signal is legal in hardware but
	 * makes count disagree with the number of distinct measurements, and
	 * the caller almost certainly meant something else.
	 */
	for (i = 0; i < req.count; i++) {
		if (req.signal_ids[i] > MLA_PERFCTR_SIGNAL_MAX)
			return -EINVAL;
		for (j = 0; j < i; j++) {
			if (req.signal_ids[j] == req.signal_ids[i])
				return -EINVAL;
		}
	}

	/* Racy in the same benign way as op 18's gate: a context can open
	 * immediately after the check. It cannot corrupt anything — the
	 * newcomer just also sees these counters, which is the documented
	 * limitation of a debugging facility, not a correctness problem.
	 */
	if (atomic_read(&mdev->contexts_open) > 1)
		return -EPERM;

	/* A recovery worker already queued or running will clear the selection
	 * out from under this call (mla_hw_recover's software clear), so
	 * installing one now is guaranteed-stale work that reports success.
	 * The idle test below cannot see this: recovery_pending is set from
	 * mla_sched_complete's fault path AFTER the faulting job has left
	 * in_flight, so the device can be idle and a reset still imminent.
	 *
	 * device_fatal is already refused at op entry (mla_drv.c:405), so it
	 * needs no test here.
	 */
	if (atomic_read(&mdev->recovery_pending))
		return -EBUSY;

	spin_lock_irqsave(&mdev->sched_lock, flags);

	if (mdev->in_flight || !plist_head_empty(&mdev->sched_queue)) {
		spin_unlock_irqrestore(&mdev->sched_lock, flags);
		return -EBUSY;
	}

	/* Snapshot for the copyout-failure rollback below. Taken under the same
	 * lock as the overwrite so it cannot capture a torn state.
	 */
	prev_count = mdev->perfctr_count;
	memcpy(prev_signals, mdev->perfctr_signals, sizeof(prev_signals));

	mdev->perfctr_count = req.count;
	memset(mdev->perfctr_signals, 0, sizeof(mdev->perfctr_signals));
	for (i = 0; i < req.count; i++)
		mdev->perfctr_signals[i] = req.signal_ids[i];

	mla_hw_perfctr_apply_locked(mdev);

	/* Publish the new generation in the same critical section as the state
	 * and the MMIO it describes, and take our copy of it from inside the
	 * lock — sampling mdev->perfctr_gen after the unlock could read a
	 * value a racing mutator had already moved past.
	 */
	mdev->perfctr_gen++;
	gen = mdev->perfctr_gen;

	spin_unlock_irqrestore(&mdev->sched_lock, flags);

	/* Remember who to clean up after, and WHICH selection. The flag alone
	 * is not an ownership proof: the sole-context gate does NOT make a
	 * second owner impossible, because mla_context_shutdown decrements
	 * contexts_open (mla_drv.c:193) before the drain that precedes
	 * mla_perfctr_release (:219). The generation is what makes cleanup
	 * safe — see mla_perfctr_release.
	 */
	prev_owns = ctx->owns_perfctr;
	ctx->owns_perfctr = (req.count > 0);
	ctx->perfctr_gen  = gen;

	memset(&resp, 0, sizeof(resp));
	resp.count = req.count;
	for (i = 0; i < req.count; i++)
		resp.signal_ids[i] = req.signal_ids[i];
	resp.gen = gen;

	if (copy_to_user(u64_to_user_ptr(req.out_ptr), &resp, sizeof(resp))) {
		/* The caller is being told this call failed, so the device must
		 * not be left reprogrammed. Put the previous selection back and
		 * re-apply it, so the registers, mdev->perfctr_* and what the
		 * caller believes all agree.
		 *
		 * Re-doing MMIO here is safe, unlike in mla_hw_recover(): op 19
		 * only runs on a healthy idle device (-EBUSY otherwise, and
		 * device_fatal is refused at op entry), never mid-reset, so
		 * there is no window in which a register read can take a
		 * synchronous external abort.
		 *
		 * The rollback bumps the generation again rather than restoring
		 * the old value: the selection genuinely changed twice, and a
		 * holder that saw the intermediate generation must not be told
		 * its selection is live again.
		 */
		spin_lock_irqsave(&mdev->sched_lock, flags);
		mdev->perfctr_count = prev_count;
		memcpy(mdev->perfctr_signals, prev_signals,
		       sizeof(mdev->perfctr_signals));
		mdev->perfctr_gen++;
		mla_hw_perfctr_apply_locked(mdev);
		spin_unlock_irqrestore(&mdev->sched_lock, flags);

		ctx->owns_perfctr = prev_owns;
		return -EFAULT;
	}

	return 0;
}

/**
 * mla_perfctr_release() — drop this context's perf-counter selection
 * @ctx: context being torn down
 *
 * Disables all four slots and clears the stored selection if @ctx is the
 * one that installed it, so the exposure of a custom selection is bounded
 * by the configuring context's fd lifetime. Without this a context that
 * opened later would keep reading signals chosen by a process that has
 * already exited, and would have no way to tell.
 *
 * Idempotent and cheap: a no-op for the overwhelming majority of contexts,
 * which never configure anything.
 *
 * Disables ONLY if this context's selection is still the live one. It often
 * is not: mla_context_shutdown decrements mdev->contexts_open
 * (mla_drv.c:193) before draining jobs, models and buffers, and only calls
 * this afterwards (:219) — a drain that waits on the delayed_fput workqueue
 * for an uncommitted BUF_ALLOC buffer. Inside that window a NEW context
 * legitimately opens, sees contexts_open == 1, passes op 19's sole-context
 * and idle gates, and installs its own selection. Disabling unconditionally
 * then wiped the newcomer's counters, leaving it with owns_perfctr still set
 * and perf_ctr_us[] reading all zeros with no way to notice — 0 is a legal
 * counter value, so nothing in the data can act as a sentinel. A recovery's
 * software clear (mla_hw_recover) has the same effect.
 *
 * Moving the contexts_open decrement down here instead would be wrong: it
 * recreates the intermittent -EPERM that gating on num_contexts caused,
 * documented at mla_op_device_reset above.
 *
 * Context: called from mla_context_shutdown (process context, fd close).
 * Must not be called with sched_lock held.
 */
void mla_perfctr_release(struct mla_context *ctx)
{
	struct mla_device *mdev = ctx->mdev;
	unsigned long flags;

	if (!ctx->owns_perfctr)
		return;

	ctx->owns_perfctr = false;

	/* Test the generation and act on it in ONE critical section. Split
	 * across two — compare under the lock, then call
	 * mla_hw_perfctr_disable_all() which takes the lock again and wipes
	 * unconditionally — a peer's op 19 fits in the gap. It passes the
	 * sole-context gate right now, for the reason described above, installs
	 * its selection and bumps the generation, and we then wipe the
	 * newcomer: exactly the bug this generation check exists to prevent,
	 * surviving in a narrower window.
	 *
	 * Body is mla_hw_perfctr_disable_all()'s, gated. That function stays for
	 * probe's "start with all four slots disabled" call.
	 */
	spin_lock_irqsave(&mdev->sched_lock, flags);
	if (ctx->perfctr_gen == mdev->perfctr_gen) {
		mdev->perfctr_count = 0;
		memset(mdev->perfctr_signals, 0, sizeof(mdev->perfctr_signals));
		mdev->perfctr_gen++;
		mla_hw_perfctr_apply_locked(mdev);
	}
	spin_unlock_irqrestore(&mdev->sched_lock, flags);
}

/**
 * mla_context_jobs_release() — drain all jobs on context teardown
 * @ctx: context whose job xarray is being destroyed
 *
 * Walks the per-context jobs xarray, dequeuing any jobs still on the
 * run queue (completing them with -ECANCELED) and dropping the
 * xarray's implicit fence reference. Any job currently running on
 * the hardware has its own outstanding refs that keep the fence
 * alive until the IRQ path reaches it.
 *
 * Called from mla_context_shutdown() — fd close path.
 */
void mla_context_jobs_release(struct mla_context *ctx)
{
	struct mla_job *job;
	unsigned long index;

	xa_for_each(&ctx->jobs, index, job) {
		/* Use the pointer xa_erase RETURNS, never the one the
		 * iterator handed us. Only the caller that actually removes
		 * an entry owns the registry's fence reference, and
		 * MLA_IOCTL_REAP erases too (an async job's terminal point) —
		 * it can run concurrently with fd close, because a job
		 * routinely outlives its fd. So the iterator's pointer may
		 * already be freed by the time we get here, and an
		 * unconditional put would also be a double put.
		 *
		 * Reassigning `job` keeps every deref below on the pointer we
		 * atomically claimed.
		 */
		job = xa_erase(&ctx->jobs, index);
		if (!job)
			continue;

		if (mla_sched_dequeue(job))
			mla_sched_complete(ctx->mdev, job, -ECANCELED);
		/* The xarray held an implicit fence reference (the
		 * original from dma_fence_init). Drop it now that the
		 * entry is erased. If the job is still running, the
		 * scheduler's in_flight reference (taken in
		 * mla_sched_pick_reserve_locked) keeps it alive until
		 * finish_job drops it — so this is never the last put on
		 * a job that is mid-dispatch.
		 */
		dma_fence_put(&job->fence);
	}
}
