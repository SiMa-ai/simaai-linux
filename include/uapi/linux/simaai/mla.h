/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* Copyright (c) 2026 SiMa.ai */
#ifndef _UAPI_LINUX_SIMAAI_MLA_H
#define _UAPI_LINUX_SIMAAI_MLA_H

#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * MLA kernel/userspace ABI.
 *
 * This header is the single source of truth for the ioctl command
 * interface exposed by /dev/mla. It is consumed by libmla, mla-rt's
 * kernel-driver connection, and any direct ioctl user.
 *
 * Versioning rules:
 *   - MLA_ABI_VERSION is bumped on any incompatible change. v1 is the
 *     baseline shipped with mla-rt's `Connect_kernel` integration.
 *   - Adding a NEW op code at the next free slot (MLA_OP_MAX, which is
 *     then bumped) does not bump ABI version; libmla checks
 *     mla_info_resp.abi_version and uses MLA_OP_INFO's capabilities
 *     bitmap to discover new features.
 *   - Adding a NEW field at the END of an existing struct (using one
 *     of the reservedN slots) does not bump ABI version provided the
 *     struct's size in mla_info_resp's *_size fields is still
 *     consulted. Renaming/reordering existing fields DOES bump.
 *   - Removing or repurposing an op code DOES bump.
 *
 * Reserved fields are always zeroed by the kernel and must be zeroed
 * by userspace. The kernel rejects requests where any reserved byte
 * is non-zero so future ABI extensions can repurpose them safely.
 */
#define MLA_ABI_VERSION                 2

/* ---- Op codes ----
 *
 * Op codes 5 and 12 (MLA_OP_MODEL_LOAD, MLA_OP_MODEL_LOAD_SECTIONS)
 * were removed at ABI v2 in favour of MLA_OP_MODEL_DEFINE, which is
 * itself now retired in favour of MLA_OP_MODEL_DEFINE_V3. The op is
 * a u32; new ops append at MLA_OP_MAX (which is then bumped).
 */
#define MLA_OP_INFO                     0
#define MLA_OP_CTX_CONFIGURE            1
#define MLA_OP_BUF_ALLOC                2
#define MLA_OP_BUF_IMPORT               3
#define MLA_OP_BUF_INFO                 4
/* 5 reserved (was MLA_OP_MODEL_LOAD, removed at ABI v2) */
#define MLA_OP_MODEL_RELEASE            6
#define MLA_OP_JOB_SUBMIT               7
#define MLA_OP_JOB_CANCEL               8
#define MLA_OP_STATS                    9   /* not yet implemented */
#define MLA_OP_BUF_ATTACH_PHYS          10  /* CAP_SYS_RAWIO */
#define MLA_OP_BUF_RELEASE              11
/* 12 reserved (was MLA_OP_MODEL_LOAD_SECTIONS, removed at ABI v2) */
/*
 * RETIRED. Was the structured image_fd + section_directive[] ingest, where
 * the kernel allocated and copied every section itself. The driver now
 * answers it with -EOPNOTSUPP; use MLA_OP_MODEL_DEFINE_V3 (op 16), which
 * takes section_ids[] from prior SECTION_ALLOC / SECTION_COMMIT calls.
 * The number stays defined so existing callers still compile and get a
 * clean errno rather than a mystery, and stays BURNED — never reuse 13.
 */
#define MLA_OP_MODEL_DEFINE             13  /* retired; -EOPNOTSUPP */
#define MLA_OP_SECTION_ALLOC            14  /* SECTION_ALLOC_ABI.md Phase 1 */
#define MLA_OP_SECTION_COMMIT           15  /* SECTION_ALLOC_ABI.md Phase 1 */
#define MLA_OP_MODEL_DEFINE_V3          16  /* SECTION_ALLOC_ABI.md §4.3 — section_ids[]-based */
/* pure-async twin of JOB_SUBMIT (no WAIT); CQE carries result */
#define MLA_OP_JOB_ASYNC                 17
#define MLA_OP_DEVICE_RESET             18  /* clear device_fatal + recovery budget without rmmod */
#define MLA_OP_PERFCTR_CONFIG           19  /* select which TLC perf counters a job reports */
#define MLA_OP_MODEL_UPDATE_RELOC       20  /* re-point a loaded model's LoRA ports */
#define MLA_OP_MAX                      21

/* ---- Command flags ---- */
#define MLA_CMD_FLAG_WAIT               (1u << 0)
#define MLA_CMD_FLAG_NOFENCE            (1u << 1)

/* ---- Context kinds ---- */
#define MLA_CTX_HOST_PROCESS            0
#define MLA_CTX_CONTAINER               1
#define MLA_CTX_VM_GUEST                2

/* ---- CTX_CONFIGURE field bitmap ---- */
#define MLA_CFG_GROUP_PRIO              (1u << 0)
#define MLA_CFG_INTRA_PRIO              (1u << 1)
#define MLA_CFG_WEIGHT                  (1u << 2)
#define MLA_CFG_REALTIME                (1u << 3)
#define MLA_CFG_KIND                    (1u << 4)
#define MLA_CFG_VM_TOKEN                (1u << 5)

/* ---- INFO capability bitmap ---- */
#define MLA_CAP_SMMU_PRESENT            (1u << 0)
#define MLA_CAP_MODEL_DEDUP             (1u << 1)
#define MLA_CAP_VFIO_MDEV               (1u << 2)
#define MLA_CAP_OCM_HEAP                (1u << 3)
/* bit 4 reserved (was MLA_CAP_SECTION_LOAD; removed at ABI v2) */
/* bit 5 reserved (was MLA_CAP_MODEL_DEFINE_V2; MODEL_DEFINE is unconditional in ABI v2) */
/* SECTION_ALLOC / SECTION_COMMIT ops available */
#define MLA_CAP_SECTION_ALLOC           (1u << 6)
/* MODEL_DEFINE_V3 (section_ids[]) op available */
#define MLA_CAP_MODEL_DEFINE_V3         (1u << 7)
#define MLA_CAP_JOB_ASYNC                (1u << 8)  /* MLA_OP_JOB_ASYNC — pure-async submission */
/*
 * MLA_OP_JOB_CANCEL is usable: MLA_OP_JOB_ASYNC publishes resp.job_id at
 * SUBMIT time (before the job is enqueued), not only at completion, so a
 * caller can name a still-queued job. The op itself has existed since ABI
 * v2; without this bit it was unreachable in practice.
 *
 * Gate on this bit, NOT on resp.job_id != 0. Zero is a usable sentinel
 * today only because IDs start at 1 (XA_FLAGS_ALLOC1) and userspace
 * happens to zero the resp buffer first; neither is a contract.
 */
#define MLA_CAP_JOB_CANCEL              (1u << 9)  /* JOB_CANCEL reachable (see above) */
#define MLA_CAP_DEVICE_RESET            (1u << 10) /* MLA_OP_DEVICE_RESET available */
#define MLA_CAP_PERFCTR_CONFIG          (1u << 11) /* MLA_OP_PERFCTR_CONFIG available */
/* bit 12 reserved for MLA_CAP_OVERLAPPED_OFM_SYNC (origin/codex/mla-overlapped-ofm-sync) */
/* LoRA / reloc-zone support, two bits because the two halves ship
 * separately and a single bit could not be probed for either meaning:
 *
 *   MLA_CAP_RELOC_UPDATE (bit 13): MODEL_DEFINE_V3 accepts
 *   MLA_RELOC_ZONE_RELOC entries and MLA_OP_MODEL_UPDATE_RELOC
 *   (op 20) can re-point a loaded model's LoRA ports between jobs.
 *
 *   MLA_CAP_RELOC_PER_JOB (bit 14): a submit request may carry its own
 *   reloc bindings, applied for that job onwards (sticky). NOT SET yet —
 *   the driver rejects a submit carrying reloc bindings with -EOPNOTSUPP.
 *
 * Gate on the bit, never on a non-zero field value.
 */
#define MLA_CAP_RELOC_UPDATE            (1u << 13) /* MLA_OP_MODEL_UPDATE_RELOC available */
#define MLA_CAP_RELOC_PER_JOB           (1u << 14) /* per-job reloc bindings honoured */
/* DEFINE_V3 can provision fixed descriptor-bank backing before publication. */
#define MLA_CAP_PREPARED_BANKS          (1u << 15)

/* TLC sync-controller perf counters, for MLA_OP_PERFCTR_CONFIG.
 *
 * MLA_PERFCTR_MAX_SLOTS is a HARDWARE limit, not a policy choice. The
 * block has eight CTRL/CNTR pairs, but slots 0-3 serve quadgroup 0/1 and
 * slots 4-7 serve quadgroup 2/3, and Modalix is permanently QG0-only — so
 * only four are reachable. Do not "raise" it.
 */
#define MLA_PERFCTR_MAX_SLOTS           4
#define MLA_PERFCTR_SIGNAL_MAX          7   /* signal ids are 0..7 */

/* ---- Heap allocation flag bitmap (passed via heap_flags) ---- */
#define MLA_HEAP_FLAG_CACHED            (1u << 0)
#define MLA_HEAP_FLAG_RDONLY            (1u << 1)
#define MLA_HEAP_FLAG_OCM               (1u << 2)

/* ---- Buffer "intended use" hints for IMPORT ----
 * Advisory labels only. The driver never reads intended_use, and it does not
 * select any caching or cache-maintenance behaviour: maintenance for an
 * imported buffer is the caller's, unconditionally. See MLA_OP_BUF_IMPORT.
 */
#define MLA_BUF_IFM                     0u
#define MLA_BUF_OFM                     1u
#define MLA_BUF_WEIGHTS                 2u
#define MLA_BUF_MODEL_IMAGE             3u

/*
 * The 16-byte command descriptor. A single, uniform layout across all ops: a
 * user pointer to the op-specific request struct, and its length. The first 24
 * bytes of every op-specific struct are struct mla_cmd_hdr; the kernel reads
 * the header to decide how many more bytes to copy from userspace.
 *
 * MLA_IOCTL_SUBMIT_SYNC passes one of these; MLA_IOCTL_SUBMIT_ASYNC passes an
 * array of mla_ioctl_submit_desc, each carrying the same {req_user_ptr,
 * req_len} pair plus a cookie. Forward-extensible: the kernel copies req_len
 * bytes, so a longer *_req struct needs no ABI change.
 */
struct mla_uring_cmd_sqe {
	__u64 req_user_ptr;   /* userspace pointer to struct mla_*_req */
	__u32 req_len;        /* exact size of the *_req struct */
	__u32 reserved;       /* must be 0; kernel returns -EINVAL otherwise */
};

/* ---- Command header (first member of every *_req) ---- */
struct mla_cmd_hdr {
	__u32 op;             /* MLA_OP_* */
	__u32 flags;          /* MLA_CMD_FLAG_* */
	__u64 cookie;         /* opaque user value; echoed in async-mode response */
	__u32 reserved[2];    /* must be 0; kernel returns -EINVAL otherwise */
};

/* =====================================================================
 *  MLA_OP_INFO
 * =====================================================================
 */

struct mla_info_req {
	struct mla_cmd_hdr hdr;
	__u64 out_ptr;        /* userspace ptr to struct mla_info_resp */
	__u32 out_len;        /* sizeof(struct mla_info_resp) at caller */
	__u32 reserved;       /* must be 0 */
};

struct mla_info_resp {
	__u32 abi_version;             /* MLA_ABI_VERSION; bumps on incompatible changes */
	__u32 driver_version;          /* MLA_DRV_VERSION; bumps on non-ABI driver changes */
	__u32 soc_family;              /* MLA_SOC_*; treat unknown values as UNKNOWN */
	__u32 tile_rows;               /* hw tile-grid rows; Modalix=10 */
	__u32 tile_cols;               /* hw tile-grid cols; Modalix=10 */
	__u32 num_dma_units;           /* Modalix=4 */
	__u32 max_contexts;            /* per-device cap on concurrent /dev/mla fds */
	__u32 max_models_per_ctx;      /* per-fd cap on live model_ids */
	__u32 max_inflight_jobs_per_ctx; /* per-fd cap on queued+running jobs */
	__u32 model_image_alignment;   /* required alignment for MODEL_LOAD's image_fd backing */
	__u32 ifm_alignment;           /* required alignment for IFM buffers */
	__u32 ofm_alignment;           /* required alignment for OFM buffers */
	__u64 max_buffer_size;         /* per-buffer size cap, bytes */
	__u64 dma_mask;                /* MLA's DMA-reachable address-space mask */
	__u32 capabilities;            /* MLA_CAP_* bitmap */
	__s8  priority_lo;             /* lowest accepted group_prio (most negative) */
	__s8  priority_hi;             /* highest accepted group_prio */
	__u8  reserved0[2];            /* must be 0 */
	__u32 mla_freq_mhz;            /* MLA core clock; 0 if unknown */
	/* Additive since MLA_DRV_VERSION=2. Old callers that pass a
	 * smaller out_len still succeed; kernel copies min(out_len,
	 * sizeof(resp)) bytes.
	 */
	__u64 outstanding_alloc_bytes; /* sum of live pool-owned buffer sizes */
	/* Additive since MLA_DRV_VERSION=3. The CALLING context's live
	 * scheduler priority — what mla_sched_prio_key() keys on right
	 * now, including the value inherited from the caller's nice value
	 * at open if CTX_CONFIGURE was never issued. Distinct from
	 * priority_lo/priority_hi above, which are the static accepted
	 * RANGE and say nothing about this context.
	 *
	 * Read-back exists because 0 is a legal priority and userspace has
	 * no other way to see the inherited default: there is no
	 * CTX_CONFIGURE read-back op, and debugfs is root-only.
	 *
	 * Gate on driver_version >= 3 before trusting these. They are 0 on
	 * an older kernel, and 0 is a VALID priority — so a zero value
	 * cannot be used to mean "absent".
	 */
	__s8  cur_group_prio;          /* this ctx's group_prio, -20..19 */
	__s8  cur_intra_group_prio;    /* this ctx's intra_group_prio */
	__u8  reserved1[2];            /* must be 0 */

	/* Present only with MLA_CAP_PREPARED_BANKS. This is a descriptor-bank
	 * capability, not a limit on queued jobs. Old INFO out_len is valid.
	 */
	__u32 reserved2;               /* preserve original INFO tail padding */
	__u32 max_prepared_banks;
	__u32 reserved3;               /* must be 0 */
};

/* SoC family values for mla_info_resp.soc_family.
 *
 * Values are stable across ABI v1. New SoC families append at the
 * next free slot — adding a value does not bump MLA_ABI_VERSION
 * (additive change per the versioning rules above).
 *
 * Value 2 is reserved for the next-generation SiMa.ai SoC family.
 * The exact name will be assigned when that silicon's driver
 * support lands; consumers should treat unknown values as
 * MLA_SOC_UNKNOWN until they explicitly support the value they see.
 */
#define MLA_SOC_UNKNOWN                 0
#define MLA_SOC_MODALIX                 1
/* 2 reserved for next-generation SoC family */

/* =====================================================================
 *  MLA_OP_BUF_ALLOC
 *
 *  Allocate a physically-contiguous, MLA-reachable buffer and return a
 *  dma-buf fd plus a per-context internal_buf_id. The dma-buf fd's
 *  lifetime owns the underlying memory: close(fd) frees the allocation
 *  (the kernel reclaims when the last reference goes away).
 *
 *  phys_addr is returned to host-kind contexts (HOST_PROCESS,
 *  CONTAINER) so userspace can relocate ELF model images against it.
 *  For VM_GUEST contexts the field is set to 0.
 * =====================================================================
 */

struct mla_buf_alloc_req {
	struct mla_cmd_hdr hdr;
	__u64  size;              /* requested bytes; rounded up to page */
	__u32  flags;             /* MLA_HEAP_FLAG_* */
	__u32  alignment;         /* 0 = MLA default; else power-of-two bytes */
	__u64  out_ptr;           /* user ptr to struct mla_buf_alloc_resp */
	__u32  out_len;           /* sizeof(struct mla_buf_alloc_resp) */
	__u32  reserved;          /* must be 0 */
};

struct mla_buf_alloc_resp {
	__s32  dmabuf_fd;         /* dma-buf fd; close() to free the buffer */
	__u32  internal_buf_id;   /* per-context xarray key for tensor refs */
	__u64  phys_addr;         /* HOST/CONTAINER kinds only; 0 for VM_GUEST */
	__u64  size_actual;       /* page-aligned actual size (>= req.size) */
	__u32  flags;             /* echoed/derived from request */
	__u32  reserved;          /* must be 0 */
};

/* =====================================================================
 *  MLA_OP_BUF_ATTACH_PHYS
 *
 *  Privileged op: register a fixed physical address range with the
 *  context without exporting a DMA-BUF or allocating an fd. Used for
 *  parity testing against the legacy mla-rt path where the runtime pre-allocated buffers via
 *  /dev/simaai-memory and we want our test to reuse those same phys
 *  addresses (so the model image's pre-relocated DMA descriptors point
 *  at memory that's actually filled with valid data).
 *
 *  Requires CAP_SYS_RAWIO and rejects any overlap with System RAM,
 *  including reusable CMA pages. For non-RAM legacy carveouts, the caller
 *  must own the entire range; this op does not track conflicting owners.
 *  It is a diagnostic compatibility path, not an allocator. Consumers of
 *  reusable memory must allocate/import DMA-BUFs and retain their handles.
 * =====================================================================
 */

struct mla_buf_attach_phys_req {
	struct mla_cmd_hdr hdr;
	__u64  phys_addr;            /* physical address (page-aligned) */
	__u64  size;                 /* byte length (page-aligned) */
	__u32  flags;                /* MLA_HEAP_FLAG_CACHED, etc. */
	__u32  reserved;             /* must be 0 */
	__u64  out_ptr;              /* user ptr to mla_buf_attach_phys_resp */
	__u32  out_len;              /* sizeof(resp) */
	__u32  reserved2;            /* must be 0 */
};

struct mla_buf_attach_phys_resp {
	__s32  dmabuf_fd;            /* always -1: this attachment is fd-less */
	__u32  internal_buf_id;      /* per-context xarray key for tensor refs */
	__u64  phys_addr;            /* echoed back for confirmation */
	__u64  size;                 /* echoed back, page-aligned */
};

/* =====================================================================
 *  MLA_OP_BUF_IMPORT
 *
 *  Take an existing dma-buf fd from another exporter (e.g. camera) and
 *  register it with this context. The driver verifies the underlying
 *  mapped DMA extent is one contiguous, MLA-reachable range. Adjacent
 *  scatterlist entries are supported, including allocations above 4 GiB;
 *  gaps, invalid lengths and out-of-range addresses are rejected.
 *
 *  CACHE MAINTENANCE IS THE CALLER'S, AND THE DRIVER CANNOT DO IT FOR YOU.
 *
 *  This driver performs no per-job cache maintenance over IFM/OFM buffers.
 *  If you touch an imported buffer with the CPU you must bracket that
 *  access yourself:
 *    - after writing an IFM (or memsetting an OFM):
 *        DMA_BUF_IOCTL_SYNC, DMA_BUF_SYNC_END   | DMA_BUF_SYNC_WRITE
 *    - before reading an OFM:
 *        DMA_BUF_IOCTL_SYNC, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ
 *
 *  Be aware of a sharp edge in that contract: DMA_BUF_IOCTL_SYNC is
 *  dispatched to the EXPORTER's begin/end_cpu_access ops, and the dma-buf
 *  core returns success when an exporter supplies none. Several in-tree
 *  exporters supply none, or supply no-op stubs, so the ioctl can appear
 *  to succeed while doing nothing at all. The MLA is not cache-coherent,
 *  so on such a buffer a CPU write is never written back and the device
 *  reads stale memory — silently, with no error from any call you made.
 *
 *  Buffers exported by the SoC's own carveout dma-heap
 *  (/dev/dma_heap/simaai,dms) do implement real cpu-access ops, so the
 *  bracket above is sufficient for them. For a foreign exporter, verify
 *  it implements them; if it does not, the only safe use is one where the
 *  CPU never touches the buffer (pure device-to-device zero-copy). The
 *  driver warns once and counts such imports in debugfs
 *  counters/foreign_imports_no_cpu_ops rather than refusing them, because
 *  it cannot see whether you intend to CPU-touch the memory.
 *
 *  intended_use does NOT change any of this. It is a hint only; the driver
 *  does not read it, and it selects no caching or sync policy.
 * =====================================================================
 */

struct mla_buf_import_req {
	struct mla_cmd_hdr hdr;
	__s32  dmabuf_fd;         /* external dma-buf to import */
	__u32  intended_use;      /* MLA_BUF_IFM | MLA_BUF_OFM | ... — hint only:
				   * never read by the driver, selects no caching
				   * or sync policy. See the section comment. */
	__u64  out_ptr;           /* user ptr to struct mla_buf_import_resp */
	__u32  out_len;           /* sizeof(resp) */
	__u32  reserved;          /* must be 0 */
};

struct mla_buf_import_resp {
	__u32  internal_buf_id;   /* per-context xarray key */
	__u32  flags;             /* MLA_BUF_RESP_* */
	__u64  phys_addr;         /* HOST/CONTAINER kinds only; 0 for VM_GUEST */
	__u64  size;              /* dma-buf size, bytes */
};

/* =====================================================================
 *  MLA_OP_BUF_INFO
 *
 *  Look up an internal_buf_id in this context and return its metadata.
 * =====================================================================
 */

struct mla_buf_info_req {
	struct mla_cmd_hdr hdr;
	__u32  internal_buf_id;   /* the id to look up in this context */
	__u32  reserved0;         /* must be 0 */
	__u64  out_ptr;           /* user ptr to struct mla_buf_info_resp */
	__u32  out_len;           /* sizeof(resp) */
	__u32  reserved1;         /* must be 0 */
};

struct mla_buf_info_resp {
	__u32  internal_buf_id;   /* echoed back */
	__u32  flags;             /* mirrored from alloc/import */
	__u64  size;              /* bytes */
	__u64  phys_addr;         /* HOST/CONTAINER kinds only; 0 for VM_GUEST */
	__u64  alignment;         /* alignment guaranteed by allocator */
};

/* =====================================================================
 *  MLA_OP_BUF_RELEASE
 *
 *  Drop the per-context registration of an internal_buf_id and the
 *  kernel's reference to the underlying dma-buf. The dma-buf itself
 *  stays alive as long as userspace still holds its fd (or any other
 *  attachment is still alive); close(dmabuf_fd) frees the memory.
 *
 *  Symmetric with MLA_OP_MODEL_RELEASE. After a successful release
 *  the internal_buf_id is invalidated; subsequent JOB_SUBMIT calls
 *  that reference the released id return -ENOENT.
 *
 *  Releasing a buffer that's still bound to an in-flight or queued
 *  job is safe: the job has already resolved the buf_id to a kernel
 *  pointer at submit time and no longer consults the xarray. The job
 *  completes normally; only future submissions are affected.
 * =====================================================================
 */

struct mla_buf_release_req {
	struct mla_cmd_hdr hdr;
	__u32  internal_buf_id;   /* id from BUF_ALLOC/IMPORT/ATTACH_PHYS */
	__u32  reserved;          /* must be 0 */
};

/* =====================================================================
 *  Structured relocation list (used by MLA_OP_MODEL_DEFINE_V3).
 *
 *  Userspace emits typed tile/DMA-unit descriptors and a symbolic reloc
 *  list; the kernel rewrites `dram_addr` fields in DMA descriptors at
 *  JOB_SUBMIT time using the actual phys_addr of each IFM/OFM
 *  internal_buf_id (SECTION-tagged entries are resolved once at
 *  MODEL_DEFINE). This keeps phys_addr entirely inside the kernel —
 *  host / container / VM_GUEST all use the same path.
 *
 *  One entry per descriptor that references a zone-relative address.
 * =====================================================================
 */

struct mla_reloc_entry {
	__u32 dma_offset;            /* byte offset of the DMA-unit's
				      * descriptor blob within its section
				      */
	__u32 desc_index;            /* descriptor index within that blob
				      * (each descriptor is 32 B)
				      */
	__u32 zone_tag;              /* MLA_RELOC_ZONE_* */
	__u32 zone_index;            /* IFM/OFM port index (matches
				      * mla_iotensor[] index at submit),
				      * RELOC port index (matches the
				      * mla_iotensor[] index at define /
				      * MODEL_UPDATE_RELOC), or section_ids[]
				      * index for SECTION
				      */
	__u64 placeholder_offset;    /* offset within the chosen zone buffer.
				      * The kernel splices
				      *   buf->phys + iotensor.offset
				      *            + placeholder_offset
				      * into the descriptor's dram_addr, where
				      * iotensor.offset is the binding's own
				      * offset (0 for a caller that hands over a
				      * whole buffer). SECTION-tagged entries
				      * have no iotensor and use
				      * buf->phys + placeholder_offset.
				      */
};

#define MLA_RELOC_ZONE_IFM         0u
#define MLA_RELOC_ZONE_OFM         1u
/* 2 reserved (was MLA_RELOC_ZONE_WEIGHTS; v1-only) */
/*
 * 3: LoRA-style adapter weights. zone_index is a RELOC PORT index, valid iff
 * < mla_model_define_v3_req.num_reloc_ports.
 *
 * Unlike IFM/OFM, a reloc port is bound at MODEL_DEFINE — to the section the
 * ELF itself shipped, named by reloc_defaults_user_ptr — and stays bound until
 * MLA_OP_MODEL_UPDATE_RELOC (op 20) re-points it. So a model that is never
 * updated is a valid configuration that runs with base-model weights; there is
 * no unbound state, and a descriptor is never dispatched still holding its
 * compile-time placeholder address.
 *
 * That guarantee is the whole point. Before MLA_CAP_RELOC_UPDATE existed the
 * kernel accepted these entries and then SKIPPED them in the descriptor
 * patcher: the descriptor kept its placeholder, the job ran to completion, the
 * fence signalled success and the OFM was garbage — no error, no dmesg line, no
 * counter. That is why MODEL_DEFINE answered -EOPNOTSUPP in the interim rather
 * than accepting an entry it could not honour, and why the default binding is
 * mandatory rather than best-effort.
 *
 * Per-JOB reloc bindings (a submit request carrying its own adapter addresses)
 * are a separate capability, MLA_CAP_RELOC_PER_JOB, and are not implemented
 * yet.
 */
#define MLA_RELOC_ZONE_RELOC       3u
#define MLA_RELOC_ZONE_SECTION     4u  /* model-static section ref */
#define MLA_RELOC_ZONE_MAX         5u

/* Per-model reloc cap. Observed sizes: resnet_50 ~1500;
 * gemma3-siglip448 n1_post_layer33_stage1 ~186k; and a model needing
 * 472,937 was rejected by the previous 262144 cap (2026-08-28), which is
 * why this is now 1 Mi. Raised straight to 2^20 rather than 2^19 because
 * 524288 would have left only 11% headroom over a model we already have.
 *
 * The cap does NOT itself cost memory: the kernel allocates
 * reloc_list_count * 24 B, not the maximum. It bounds the worst case a
 * caller can ask for. What the cap permits is the real cost, and it is
 * NOT transient (an earlier version of this comment claimed "freed after
 * copy_from_user" — wrong): the list is retained for the model's whole
 * lifetime as mla_model.reloc_list and only released by kvfree() in
 * mla_model_destroy(). So a 472,937-entry model holds ~10.8 MiB resident
 * per live model_id, and a multi-stage LLM pipeline holds that per stage.
 * kvcalloc, so the allocation falls back to vmalloc and page-allocator
 * fragmentation is not an issue at these sizes.
 *
 * Per-job cost is unaffected by the static prefix: the descriptor patcher
 * walks only [static_reloc_count, reloc_count), because SECTION-tagged
 * entries are spliced once at MODEL_DEFINE. A large mostly-static model
 * therefore pays at load, not per job.
 *
 * MODEL_DEFINE's handler validates against this bound; mla-rt's
 * elfMLALoader reads the SAME symbol from this header
 * (MLALoader.cc build_dma_descs_and_relocs), so both sides move together
 * — but mla-rt must be REBUILT against a changed header, or it keeps
 * rejecting at the old value.
 */
#define MLA_RELOC_MAX_ENTRIES      1048576u

/* =====================================================================
 *  MLA_OP_MODEL_RELEASE
 *
 *  Release a model_id from a context. Drops the dedup-table refcount
 *  for shared models; frees the underlying image when the last
 *  reference goes away.
 * =====================================================================
 */

struct mla_model_release_req {
	struct mla_cmd_hdr hdr;
	__u32  model_id;             /* id from MODEL_DEFINE response */
	__u32  reserved;             /* must be 0 */
};

/* Model flags */
#define MLA_MODEL_RDONLY                (1u << 0)

/* MODEL_DEFINE response flags */
#define MLA_MODEL_RESP_DEDUPED          (1u << 0)

/* =====================================================================
 *  Section directive shared types. mla_model_define_req survives as
 *  MODEL_DEFINE_V3's internal shim; op 13 itself is retired.
 *
 *  User-space builds a directive list with per-section {role,
 *  offset_in_image, size, sha256, flags} and hands the kernel a single
 *  source fd plus the directive array. The kernel allocates each
 *  section from its DMA pool (memory-region bound at probe —
 *  dms0_reserved on Modalix), copies the section bytes in, and
 *  registers each allocation as an internal_buf_id in the caller's
 *  context xarray.
 *
 *  IFM/OFM stay on BUF_ALLOC: those are per-inference, user-written,
 *  and not part of the model. WEIGHTS/INSTR/DEFAULT are the
 *  model-static sections.
 *
 *  Dedup: callers set MLA_SECTION_FLAG_DEDUP_OK on read-only sections
 *  (typically WEIGHTS). If a previously-registered section on the same
 *  device matches sha256, the kernel returns its existing
 *  internal_buf_id and the binding's flags carry
 *  MLA_SECTION_BINDING_FLAG_DEDUP_HIT.
 * =====================================================================
 */

#define MLA_SECTION_NAME_MAX        64
#define MLA_SECTION_MAX_PER_LOAD    4096

/* Section role tag — what the section means to the MLA. */
/* 0 reserved (was MLA_SECTION_ROLE_M4CTRL; v1-only) */
#define MLA_SECTION_ROLE_WEIGHTS    1  /* model weights / DRAM section */
#define MLA_SECTION_ROLE_INSTR      2  /* tile instruction blob */
#define MLA_SECTION_ROLE_DEFAULT    3  /* zero-init or scratch */
#define MLA_SECTION_ROLE_MAX        4

/* Section flags (per directive) */
#define MLA_SECTION_FLAG_DEDUP_OK   (1u << 0)  /* opt-in to content-hash dedup */
#define MLA_SECTION_FLAG_RDONLY     (1u << 1)  /* MLA never writes; eligible to share */
#define MLA_SECTION_FLAG_CPU_RMW    (1u << 2)  /* CPU RMWs per-job → cached WB alloc */

struct mla_section_directive {
	char    name[MLA_SECTION_NAME_MAX];   /* informational; NUL-padded */
	__u64   src_offset;                   /* into image_fd, must be page-aligned */
	__u64   size;                         /* bytes, must be page-aligned */
	__u32   role;                         /* MLA_SECTION_ROLE_* */
	__u32   flags;                        /* MLA_SECTION_FLAG_* */
	__u8    sha256[32];                   /* content hash (full SHA-256) */
	__u8    reserved[8];                  /* must be 0 */
};

/* Per-directive response binding. Caller indexes by directive position;
 * this carries the kernel-assigned internal_buf_id plus a flag bitmap
 * indicating whether the kernel deduped against an existing section.
 */
#define MLA_SECTION_BINDING_FLAG_DEDUP_HIT  (1u << 0)

struct mla_section_binding {
	char    name[MLA_SECTION_NAME_MAX];   /* echoed from directive */
	__u32   internal_buf_id;              /* registered in caller's context */
	__u32   flags;                        /* MLA_SECTION_BINDING_FLAG_* */
};

/* =====================================================================
 *  MLA_OP_MODEL_DEFINE  (RETIRED — kept as MODEL_DEFINE_V3's shim)
 *
 *  Structured model-load ABI. Typed inputs replace v1's
 *  m4ctrl-as-wire-format:
 *
 *    - mla_section_directive[]        — section payloads
 *    - mla_tile_desc[]                — per-tile MIM register + INSTR slice
 *    - mla_dma_unit_desc[]            — per-DMA-unit configuration
 *    - mla_reloc_entry[]              — symbolic descriptor refs
 *
 *  No fully-relocated m4ctrl image. The kernel allocates section memory,
 *  patches DMA descriptors with kernel-chosen physes at MODEL_DEFINE
 *  time (for SECTION-tagged reloc entries) and at JOB_SUBMIT time (for
 *  IFM/OFM/RELOC tags). User-space never sees a phys.
 *
 *  OCM section role is out of scope; future additive ABI extension will
 *  add an OCM kernel pool.
 * =====================================================================
 */

#define MLA_MAX_ACTIVE_TILES    100u    /* 10x10 grid */

struct mla_tile_desc {
	__u32   tile_index;                /* 0..99 */
	__u32   quadgroup;                 /* 0..3 (Modalix uses 0 only) */
	/* INSTR is aggregated: one section holds every active tile's
	 * instruction blob laid out contiguously. Each tile_desc indexes
	 * into it via instr_offset / instr_size.
	 */
	__u32   instr_section_index;       /* index into the directive[] array */
	__u32   instr_offset;              /* byte offset within the INSTR section */
	__u32   instr_size;                /* bytes for this tile */
	/* MIM base-register programming (was tile_reg_info.base_reg/val
	 * inside the m4ctrl image). Kernel writes this register at
	 * register-program time; validates that base_reg is within the
	 * MLA AXI window.
	 */
	__u32   mim_base_reg;
	__u32   mim_base_val;
	__u32   reserved[3];               /* must be 0 */
};

struct mla_dma_unit_desc {
	__u32   unit_index;                /* 0..3 (DMA_UNIT_COUNT_SIZE) */
	__u32   desc_count;                /* number of descriptors in the blob */
	__u32   dfm_fetch_map_type;        /* 1=RO, 2=WO, 3=mixed */
	__u32   reserved;                  /* must be 0 */
	__u32   desc_section_index;        /* directive[] index of the section
					    * holding the descriptor blob
					    */
	__u32   reserved2;                 /* must be 0 */
	__u64   desc_blob_offset;          /* byte offset within that section */
};

struct mla_model_define_req {
	struct mla_cmd_hdr hdr;

	/* Section payloads. All sections allocated from the kernel's
	 * reserved-memory pool.
	 */
	__s32   image_fd;                  /* dma-buf or regular file fd */
	__u32   num_sections;              /* 1..MLA_SECTION_MAX_PER_LOAD */
	__u64   sections_user_ptr;         /* mla_section_directive[] */

	/* Tile + DMA unit descriptors. */
	__u32   num_tiles;                 /* 1..MLA_MAX_ACTIVE_TILES */
	__u32   num_dma_units;             /* must be 4 on Modalix */
	__u64   tile_descs_user_ptr;       /* mla_tile_desc[num_tiles] */
	__u64   dma_unit_descs_user_ptr;   /* mla_dma_unit_desc[num_dma_units] */

	/* Symbolic reloc list (replaces user-space pre-relocation).
	 * Entries with zone_tag == MLA_RELOC_ZONE_SECTION are resolved
	 * once at MODEL_DEFINE time; IFM/OFM/RELOC tags are resolved
	 * per-job at JOB_SUBMIT.
	 */
	__u64   reloc_list_user_ptr;       /* mla_reloc_entry[]; NULL if count=0 */
	__u32   reloc_list_count;          /* 0..MLA_RELOC_MAX_ENTRIES */
	/* Partition hint for the reloc list. When non-zero, the caller
	 * guarantees entries [0..static_reloc_count) carry only SECTION-
	 * tag zones and are safe to resolve once at MODEL_DEFINE time;
	 * entries [static_reloc_count..reloc_list_count) carry IFM/OFM/
	 * RELOC tags that the per-job patcher must visit on every
	 * JOB_SUBMIT. Zero means "no partition supplied" — kernel walks
	 * every entry on every job, same as v2.1c behavior.
	 */
	__u32   static_reloc_count;

	/* Submission contract. */
	__u32   batch_size;                /* 1 if no batching */
	__u32   num_inputs;                /* IFM port count */
	__u32   num_outputs;               /* OFM port count */
	__u32   flags;                     /* MLA_MODEL_RDONLY etc. */

	/* Quad-tile config. */
	__u32   quad_tile_bitmask[4];

	__u64   out_ptr;                   /* mla_model_define_resp + bindings[] */
	__u32   out_len;                   /* size of resp + bindings buffer */
	__u32   reserved2;                 /* must be 0 */

	/* Debug/observability: model name for kernel v2_trace correlation.
	 * NUL-padded; informational only. Userspace fills with the ELF
	 * filename basename (or "" if unavailable). Kernel reads at most
	 * MLA_SECTION_NAME_MAX-1 bytes and forces NUL termination.
	 */
	char    name[MLA_SECTION_NAME_MAX];
};

/* Layout of the response buffer:
 *   struct mla_model_define_resp resp;
 *   struct mla_section_binding   bindings[num_bindings];
 * out_len must be >= sizeof(resp) + num_sections * sizeof(binding).
 * num_bindings == num_sections.
 */
struct mla_model_define_resp {
	__u32   model_id;                  /* per-context xarray key */
	__u32   flags;                     /* MLA_MODEL_RESP_* */
	__u32   num_active_tiles;          /* echo of num_tiles */
	__u32   num_dma_descriptors;       /* total across all DMA units */
	__u32   num_bindings;              /* echo of num_sections */
	__u32   reserved;                  /* must be 0 */
};

/* =====================================================================
 *  MLA_OP_MODEL_DEFINE_V3  (SECTION_ALLOC_ABI.md §4.3)
 *
 *  Structured model-load ABI, section-IDs form. Userspace has already
 *  allocated + written + committed each section via SECTION_ALLOC /
 *  SECTION_COMMIT; here we pass the resolved section_ids so the kernel
 *  never sees section bytes at MODEL_DEFINE time. No image_fd, no
 *  section_directive[] — the corresponding fields are simply gone.
 *
 *  Tile / DMA-unit / reloc descriptors are unchanged from V2. Reloc
 *  entries with zone_tag == MLA_RELOC_ZONE_SECTION carry a zone_index
 *  that points into this request's section_ids[]; the kernel resolves
 *  those to mla_buffer* via ctx->buffers[section_id] and splices at
 *  MODEL_DEFINE. IFM/OFM/RELOC tags still resolve per-job.
 *
 *  Advertised via MLA_CAP_MODEL_DEFINE_V3 in mla_info_resp.capabilities.
 * =====================================================================
 */

struct mla_model_define_v3_req {
	struct mla_cmd_hdr hdr;

	/* Pre-committed section IDs. section_ids[i] must resolve to a
	 * SECTION_ALLOC-produced buf in ctx->buffers whose SECTION_COMMIT
	 * has run. i is the index used by tile_desc.instr_section_index,
	 * dma_unit_desc.desc_section_index, and reloc.zone_index (SECTION
	 * tag).
	 */
	__u64   section_ids_user_ptr;      /* __u32[num_sections] */
	__u32   num_sections;              /* 1..MLA_SECTION_MAX_PER_LOAD */

	/* Tile + DMA unit descriptors. Same shape as V2. */
	__u32   num_tiles;                 /* 1..MLA_MAX_ACTIVE_TILES */
	__u32   num_dma_units;             /* must be MLA_DMA_UNIT_COUNT_SIZE */

	/* Number of LoRA / reloc ports this model has: the bound that a
	 * MLA_RELOC_ZONE_RELOC entry's zone_index is validated against, and
	 * the length of both reloc_defaults_user_ptr here and the array
	 * MLA_OP_MODEL_UPDATE_RELOC carries. 0 = model has no LoRA ports,
	 * which is what every pre-MLA_CAP_RELOC_UPDATE caller sends: this word
	 * was `reserved_pad`, specified must-be-0, so the struct size is
	 * unchanged and an old caller cannot accidentally declare ports.
	 *
	 * Every MLA_RELOC_ZONE_RELOC entry's zone_index must be
	 * < num_reloc_ports, else -EINVAL.
	 *
	 * The reverse direction is NOT a define-time error: a declared port that
	 * no reloc entry names is only logged. Such a port has no descriptor to
	 * splice, so the model still loads and runs correctly on base weights —
	 * but MLA_OP_MODEL_UPDATE_RELOC will reject an attempt to re-point that
	 * particular port with -ENODATA, rather than report a swap that could
	 * not take effect. Padding this count therefore costs you nothing at
	 * load and fails loudly only if you later try to use the padded ports.
	 */
	__u32   num_reloc_ports;           /* 0..MLA_MAX_IOTENSORS */
	__u64   tile_descs_user_ptr;       /* mla_tile_desc[num_tiles] */
	__u64   dma_unit_descs_user_ptr;   /* mla_dma_unit_desc[num_dma_units] */

	/* Symbolic reloc list. Same partitioning contract as V2. */
	__u64   reloc_list_user_ptr;
	__u32   reloc_list_count;
	__u32   static_reloc_count;

	/* Submission contract. */
	__u32   batch_size;
	__u32   num_inputs;
	__u32   num_outputs;
	__u32   flags;                     /* MLA_MODEL_RDONLY etc. */

	/* Quad-tile config. */
	__u32   quad_tile_bitmask[4];

	__u64   out_ptr;                   /* mla_model_define_v3_resp */
	__u32   out_len;                   /* size of resp */
	__u32   reserved2;                 /* must be 0 */

	/* Debug: model name for kernel v2_trace correlation. NUL-padded. */
	char    name[MLA_SECTION_NAME_MAX];

	/* ---- Fields below this line are OPTIONAL TAIL. ------------------
	 * The handler accepts any req_len >= offsetofend(..., name) and
	 * zero-fills the rest, so appending here is additive: a caller built
	 * against an older header still works, and a caller built against a
	 * newer one than the kernel gets its tail ignored rather than -EINVAL.
	 * (The check used to be req_len == sizeof(req), which is why op 20 and
	 * anything else needing a new submit-time field had to be a new op.)
	 */

	/* Default binding for each reloc port: mla_iotensor[num_reloc_ports],
	 * indexed by port. Required (and must be non-NULL with every
	 * internal_buf_id valid) whenever num_reloc_ports > 0.
	 *
	 * Each entry names the section carrying that port's ELF-supplied
	 * adapter data — typically one of this request's own section_ids[],
	 * SECTION_ALLOC'd and SECTION_COMMIT'd with the ELF's bytes like any
	 * other section — plus the offset of the payload within it. That is
	 * what makes "loaded but never updated" mean base-model behaviour
	 * instead of a descriptor pointing at whatever its placeholder address
	 * happens to name.
	 *
	 * The kernel takes a counted reference per port, splices each port's
	 * descriptors at define, and holds the reference until the port is
	 * re-pointed by MLA_OP_MODEL_UPDATE_RELOC or the model is released. So
	 * an early MLA_OP_BUF_RELEASE on a default section's id is safe, for
	 * the same reason it is safe on any other section.
	 */
	__u64   reloc_defaults_user_ptr;   /* mla_iotensor[num_reloc_ports] */

	/* Requires MLA_CAP_PREPARED_BANKS. Absent/zero preserves legacy lazy
	 * growth. Nonzero provisions exactly this many descriptor banks before
	 * publishing the model and disables enqueue-time backing allocation.
	 * Values above INFO.max_prepared_banks are rejected, not clamped; any
	 * allocation failure aborts model definition. More queued jobs still
	 * use normal bank availability/backpressure, not extra allocations.
	 *
	 * Send both words when using this tail. An older kernel may ignore
	 * them: callers MUST negotiate the capability before relying on the
	 * no-enqueue-allocation guarantee (including a count of one).
	 */
	__u32   prepared_bank_count;
	__u32   prepared_reserved;         /* must be 0 */
};

struct mla_model_define_v3_resp {
	__u32   model_id;
	__u32   flags;                     /* echoed MLA_MODEL_RESP_* */
	__u32   num_active_tiles;
	__u32   num_dma_descriptors;
	__u32   reserved[4];               /* must be 0 */
};

/* =====================================================================
 *  MLA_OP_MODEL_UPDATE_RELOC
 *
 *  Re-point some or all of a loaded model's reloc (LoRA) ports at
 *  caller-supplied buffers, and patch the affected descriptors. This is
 *  what a caller uses to swap an adapter on a model it has already
 *  defined; the bindings it installs are STICKY — every subsequent job
 *  runs against them until another update changes them.
 *
 *  SYNCHRONOUS and EAGER, by design: the descriptors of every descriptor
 *  bank are rewritten before the op returns, so a caller that gets 0 back
 *  knows the next job will use the new adapter. Nothing is deferred to
 *  submit or prep.
 *
 *  ⚠ THE CALLER MUST BE QUIESCENT. The op returns -EBUSY if the model has
 *  any outstanding job (queued, prepped-ahead or in flight), and a submit
 *  that races an update in progress also gets -EBUSY. This is not
 *  politeness: the descriptors being rewritten are the ones the hardware
 *  reads, so patching under live work can hand the device a half-updated
 *  descriptor set, or a torn 40-bit address pointing anywhere. That
 *  presents as an FTL fault or a quadgroup timeout, not as a clean error.
 *  Wait for your fences, then update.
 *
 *  Note the same obligation applies to memory the caller writes itself: a
 *  memset of a bound adapter buffer (the "unbind" that userspace actually
 *  performs) is equally a mutation of live device-visible memory, and the
 *  driver cannot see it to enforce anything. It also needs
 *  DMA_BUF_IOCTL_SYNC bracketing, like any other CPU write to a buffer the
 *  MLA reads.
 *
 *  Ports not named by an update keep their current binding, so a caller
 *  may re-point one port and leave the rest alone. There is no encoding
 *  for "revert to the ELF default" — no caller needs one; zeroing the
 *  bound buffer is what removes an adapter's effect.
 *
 *  A port the model's reloc list never names has no descriptor to rewrite,
 *  so re-pointing it is refused with -ENODATA rather than counted as
 *  updated. Leaving such a port alone (internal_buf_id == 0) is always
 *  legal — this constrains only what you ask to CHANGE.
 *
 *  Advertised via MLA_CAP_RELOC_UPDATE in mla_info_resp.capabilities.
 * =====================================================================
 */

struct mla_model_update_reloc_req {
	struct mla_cmd_hdr hdr;
	/* Size of this struct as the CALLER knows it. The handler accepts
	 * struct_size >= offsetofend(..., reserved2) and truncates to what it
	 * understands, so the next field appended here is a version bump
	 * rather than a fourth model op.
	 */
	__u32  struct_size;
	__u32  model_id;                /* from MODEL_DEFINE_V3 response */
	/* Must equal the model's num_reloc_ports. The array is indexed BY
	 * PORT, not packed: to leave a port alone, set its
	 * internal_buf_id to 0, which is never a valid buffer id.
	 */
	__u32  num_reloc;
	__u32  reserved;                /* must be 0 */
	__u64  reloc_ptr;               /* user ptr to mla_iotensor[num_reloc] */
	__u64  out_ptr;                 /* struct mla_model_update_reloc_resp * */
	__u32  out_len;
	__u32  reserved2;               /* must be 0 */
};

struct mla_model_update_reloc_resp {
	__u32  struct_size;             /* size the KERNEL filled in */
	/* How many ports this call actually re-pointed (entries with a
	 * non-zero internal_buf_id). A caller cannot infer this from rc: a
	 * request naming no ports at all is a legal no-op that also returns 0.
	 */
	__u32  ports_updated;
	/* Descriptor-patch passes performed, one per bank the model has grown.
	 * Zero with ports_updated > 0 would mean the bindings were recorded
	 * without any descriptor being rewritten — the exact silent failure
	 * this op exists to prevent, so it is worth asserting on.
	 */
	__u32  patch_passes;
	__u32  reserved[5];             /* must be 0 */
};

/* =====================================================================
 *  MLA_OP_JOB_SUBMIT
 *
 *  Submit one inference. Async-by-default: the SQE returns 0 once the
 *  job is queued, with `job_id` and a sync_file fd in the response.
 *  poll/epoll the fence fd for completion; read its status via the
 *  standard sync_file ioctl.
 *
 *  Slice 4 (this slice) provides a synchronous fallback gated by
 *  MLA_CMD_FLAG_WAIT: when set, the SQE blocks until the hardware
 *  finishes (or times out) and the CQE.res reports the final status.
 *  No fence fd is returned in that mode.
 * =====================================================================
 */

struct mla_iotensor {
	__u32 internal_buf_id;       /* from BUF_ALLOC / BUF_IMPORT / BUF_ATTACH_PHYS */
	__u32 reserved;              /* must be 0 */
	__u64 offset;                /* byte offset into buffer */
	__u64 length;                /* byte length used by this tensor */
};

struct mla_job_submit_req {
	struct mla_cmd_hdr hdr;      /* set MLA_CMD_FLAG_WAIT for sync mode */
	__u32  model_id;             /* from MODEL_LOAD response */
	__u32  num_ifm;              /* must match model's num_inputs */
	__u32  num_ofm;              /* must match model's num_outputs */
	__u32  cycle_count;          /* 0 = run-until-done; else hw cycle budget */
	__u64  ifm_ptr;              /* user ptr to mla_iotensor[num_ifm] */
	__u64  ofm_ptr;              /* user ptr to mla_iotensor[num_ofm] */
	__u32  timeout_ms;           /* 0 = driver default (5000 ms) */
	__u32  flags;                /* MLA_JOB_FLAG_* */
	__s32  wait_fence_fd;        /* -1 = none; else dma-fence fd to wait on */
	__u32  reserved;             /* must be 0 */
	__u64  out_ptr;              /* user ptr to mla_job_submit_resp */
	__u32  out_len;              /* sizeof(resp) */
	__u32  reserved2;            /* must be 0 */

	/* ---- Fields below this line are OPTIONAL TAIL. ------------------
	 * Both JOB_SUBMIT and JOB_ASYNC accept any
	 * req_len >= offsetofend(..., reserved2) and zero-fill the rest, so a
	 * caller built against an older header keeps working unchanged and a
	 * newer field is a version bump rather than a new op number. (The check
	 * was req_len == sizeof(req) until MLA_DRV_VERSION 7, which is why
	 * MLA_OP_PERFCTR_CONFIG had to become a separate op instead of a submit
	 * field, and why a pair of JOB_SUBMIT_V2 / JOB_ASYNC_V2 ops was
	 * considered for the reloc bindings below. Neither is needed.)
	 */

	/* Per-job reloc (LoRA) bindings: mla_iotensor[num_reloc] indexed by
	 * reloc port, exactly as MLA_OP_MODEL_UPDATE_RELOC takes them, with
	 * internal_buf_id == 0 meaning "leave this port alone".
	 *
	 * Semantics when implemented: the bindings become the MODEL's current
	 * bindings at submit time — they are STICKY, not per-job-scoped, so a
	 * later job that carries none runs with whatever the last binding-
	 * carrying job installed. That is the behaviour the callers want (bind
	 * an adapter once, then run many tokens) and it is why these live on
	 * the model rather than the job.
	 *
	 * NOT IMPLEMENTED YET: a request with num_reloc != 0 is rejected with
	 * -EOPNOTSUPP unless MLA_CAP_RELOC_PER_JOB is set. Use
	 * MLA_OP_MODEL_UPDATE_RELOC, which does the same thing between jobs.
	 */
	__u32  num_reloc;            /* 0, or the model's num_reloc_ports */
	__u32  reserved3;            /* must be 0 */
	__u64  reloc_ptr;            /* user ptr to mla_iotensor[num_reloc] */
};

struct mla_job_submit_resp {
	__u32  job_id;               /* per-context job key */
	__s32  completion_fence_fd;  /* sync_file fd in async mode; -1 in WAIT mode */
	__u64  latency_us;           /* populated in WAIT mode after run; microseconds */

	/* Optional kernel-side phase trace, all in microseconds. Only
	 * populated when the caller sized resp >= sizeof(mla_job_submit_resp)
	 * (i.e. these fields are present); otherwise the kernel touches only
	 * the leading three fields above. Useful for splitting the "where
	 * does the 80us overhead live" question between submit dispatch,
	 * pre-GO programming, IRQ wakeup, and completion.
	 */
	__u32  trace_submit_to_dispatch_us; /* sched_enqueue -> dispatch start */
	__u32  trace_dispatch_to_go_us;     /* dispatch -> GO write */
	__u32  trace_go_to_irq_us;          /* GO write -> non-FTL IRQ entry */
	__u32  trace_irq_to_wake_us;        /* IRQ entry -> sync_wait wakeup */

	/* Sub-split of dispatch->GO so we can see whether apply_relocs,
	 * pre-GO cache sync, or DFM/tile programming dominates.
	 */
	__u32  trace_relocs_us;             /* dispatch -> after apply_relocs */
	__u32  trace_dma_sync_us;           /* after relocs -> after pre-GO sync */
	__u32  trace_program_us;            /* after sync -> GO (DFM+tile+QG0) */

	/* TLC sync-controller perf counters sampled post-PROGDONE, in
	 * microseconds. Slot i carries the i-th signal the caller selected
	 * via MLA_OP_PERFCTR_CONFIG, in the order it listed them.
	 *
	 * OPT-IN as of MLA_DRV_VERSION 6. With no selection configured the
	 * driver programs nothing, skips the read entirely, and returns all
	 * zeros here. Zero is ALSO a legal counter reading, so do not infer
	 * "not configured" from a zero — the caller knows what it asked for,
	 * and mla_perfctr_config_resp.count is the authoritative answer.
	 * Slots >= the configured count are undefined, returned as zero.
	 *
	 * Before MLA_DRV_VERSION 6 these were hardwired to
	 * {tiled_lat, tiles_active, iq_sync_freeze, d_sync_freeze} and were
	 * populated on every job whether the caller wanted them or not.
	 *
	 * A recovery CLEARS the selection, so a run that faulted reports
	 * zeros until it reconfigures — see MLA_OP_PERFCTR_CONFIG for why.
	 */
	__u32  perf_ctr_us[4];

	/* Op-entry / op-exit wrappers around the existing dispatch stamps.
	 *   trace_op_prep_us = ts_submit - ts_op_enter
	 *     Covers pre-enqueue work: iotensor copy_from_user, kzalloc,
	 *     resolve_iotensors, xa_alloc, sync_file (async), dma_fence_get.
	 *   trace_op_post_us = ts_op_exit - ts_wake
	 *     Covers post-wait work: resp fill, copy_to_user, fence_put.
	 * Older kernels leave these zero.
	 */
	__u32  trace_op_prep_us;
	__u32  trace_op_post_us;

	/* Sub-split of trace_relocs_us: setup (function prologue,
	 * desc-section resolution) vs. the per-reloc patch loop body.
	 *   trace_reloc_setup_us = ts_reloc_loop_start - ts_dispatch
	 *   trace_reloc_loop_us  = ts_after_relocs      - ts_reloc_loop_start
	 * Older kernels leave these zero.
	 */
	__u32  trace_reloc_setup_us;
	__u32  trace_reloc_loop_us;

	/* Reserved (busy-poll path retired). Kernel always writes 0. */
	__u32  trace_poll_us;
	__u32  reserved3;

	/* Sub-split of trace_op_prep_us (2026-07-09). Splits the
	 * kernel-side kprep window (ts_op_enter -> ts_submit) into four
	 * phases so profiling can localize the dominant cost:
	 *   kprep_cfu_us    = ts_after_cfu    - ts_op_enter
	 *       req header copy_from_user + iotensor kcalloc/copy_from_user
	 *       for both IFM and OFM arrays.
	 *   kprep_resv_us   = ts_after_resv   - ts_after_cfu
	 *       job kzalloc + mla_resolve_iotensors (2 inner kcallocs, 2
	 *       xa_load loops).
	 *   kprep_offs_us   = ts_after_offs   - ts_after_resv
	 *       per-tensor offset scan + optional ifm/ofm_offsets kcallocs.
	 *   kprep_xa_us     = ts_submit       - ts_after_offs
	 *       xa_alloc(&ctx->jobs) + sync_file/get_unused_fd (async only)
	 *       + dma_fence_get.
	 * All zero on older kernels or when out_len is too small.
	 */
	__u32  trace_kprep_cfu_us;
	__u32  trace_kprep_resv_us;
	__u32  trace_kprep_offs_us;
	__u32  trace_kprep_xa_us;

	/* Fault diagnostics — populated on job failure (result != 0).
	 * Zero on success. Only filled when out_len covers these fields.
	 *
	 * fault_class: 0=none, 1=mosaic, 2=DMC, 3=MIM, 4=TLC, 5=timeout
	 * fault_sys_sts: raw SYS_FTL_INTR_STS aggregator register at fault
	 * fault_l2g_sts[4]: L2G_FTL_INTR_SRC0 for each L2 cache group (0-3)
	 */
	__u32  fault_class;
	__u32  fault_sys_sts;
	__u32  fault_l2g_sts[4];
};

/* Fault class values for mla_job_submit_resp.fault_class */
#define MLA_FAULT_NONE      0
#define MLA_FAULT_MOSAIC    1  /* mosaic: quadgroup timeout / SRAM ECC / MCR */
#define MLA_FAULT_DMC       2  /* DMA-class fault */
#define MLA_FAULT_MIM       3  /* memory-interface fault */
#define MLA_FAULT_TLC       4  /* tile/quad-class fault */
#define MLA_FAULT_TIMEOUT   5  /* job watchdog expired, no FTL IRQ */

/* Job flags */
#define MLA_JOB_FLAG_HIGH_PRIO_BOOST    (1u << 0)

/* =====================================================================
 *  MLA_OP_JOB_ASYNC
 *
 *  Pure-async twin of MLA_OP_JOB_SUBMIT. Shares struct mla_job_submit_req
 *  and struct mla_job_submit_resp verbatim — the opcode is the sole
 *  difference. MLA_CMD_FLAG_WAIT is IGNORED (the op is always async).
 *
 *  Behaviour: issued via MLA_IOCTL_SUBMIT_ASYNC, which enqueues the job and
 *  returns immediately (nothing is waited on). The job's dma-fence is
 *  signalled after PROGDONE (or fault/timeout); MLA_IOCTL_REAP then fills
 *  resp, copy_to_user's it, and reports the per-job result by cookie.
 *
 *  Rationale: JOB_SUBMIT|WAIT blocks a thread for the entire HW execution,
 *  so overlapping N jobs would need N blocked threads. JOB_ASYNC keeps all N
 *  in-flight without any blocked kernel threads. HW still runs one job at a
 *  time, so wall-clock is unchanged; kernel-side cost drops significantly (no
 *  blocked thread per outstanding job).
 *
 *  Advertised via MLA_CAP_JOB_ASYNC in mla_info_resp.capabilities.
 * =====================================================================
 */

/* =====================================================================
 *  MLA_OP_JOB_CANCEL
 * =====================================================================
 */

struct mla_job_cancel_req {
	struct mla_cmd_hdr hdr;
	__u32  job_id;                /* job to cancel */
	__u32  reserved;              /* must be 0 */
};

/* =====================================================================
 *  MLA_OP_CTX_CONFIGURE
 * =====================================================================
 */

struct mla_ctx_configure_req {
	struct mla_cmd_hdr hdr;
	__u32  fields;                /* MLA_CFG_* bitmap of which fields apply */
	__s8   group_prio;            /* -20..19; CAP_SYS_NICE required to raise */
	__s8   intra_group_prio;      /* -20..19 */
	__u8   reserved0[2];          /* must be 0 */
	__u32  weight;                /* 1..65535; default 1024 (round-robin neutral) */
	__u32  realtime;              /* 0/1; CAP_SYS_NICE required to set */
	__u32  kind;                  /* MLA_CTX_*; HOST_PROCESS only today */
	__u64  vm_token;              /* reserved for vhost-mla; rejected today */
};

/* =====================================================================
 *  MLA_OP_SECTION_ALLOC / MLA_OP_SECTION_COMMIT
 *
 *  Direct per-section allocation for the SECTION_ALLOC_ABI.md path.
 *  Userspace hands the kernel one section at a time; on MISS the
 *  kernel returns a dma-buf fd userspace can mmap and memcpy into,
 *  followed by DMA_BUF_IOCTL_SYNC(SYNC_END|WRITE) and
 *  MLA_OP_SECTION_COMMIT to publish the section into the device-wide
 *  dedup table.
 *
 *  On dedup HIT (existing sec_ddp entry with matching SHA256) the
 *  kernel returns dmabuf_fd=-1 and the caller skips write + commit —
 *  bytes are already in place, refcount was bumped, section is
 *  immediately usable.
 *
 *  Dedup atomicity across concurrent callers is handled by an
 *  in-flight "pending_shas" table alongside sec_ddp; a second caller
 *  hitting the same SHA blocks on the first caller's SECTION_COMMIT
 *  before allocating.
 *
 *  Advertised via MLA_CAP_SECTION_ALLOC in mla_info_resp.capabilities.
 * =====================================================================
 */

struct mla_section_alloc_req {
	struct mla_cmd_hdr hdr;
	__u32  zone;                          /* MLAMemoryZone (informational) */
	__u32  flags;                         /* MLA_SECTION_FLAG_* */
	__u64  size;                          /* page-aligned */
	__u8   name[MLA_SECTION_NAME_MAX];    /* NUL-padded; informational */
	__u8   sha256[32];                    /* all-zero => dedup skipped */
	__u64  out_ptr;                       /* struct mla_section_alloc_resp * */
	__u32  out_len;
	__u32  reserved;                      /* must be 0 */
};

struct mla_section_alloc_resp {
	__u32  section_id;                    /* per-ctx internal_buf_id */
	__s32  dmabuf_fd;                     /* -1 on dedup HIT (no write needed) */
	__u64  size_actual;                   /* PAGE_ALIGN(req.size) */
	__u32  dedup_hit;                     /* 0 or 1 */
	__u32  reserved[3];                   /* must be 0 */
};

struct mla_section_commit_req {
	struct mla_cmd_hdr hdr;
	__u32  section_id;                    /* from SECTION_ALLOC resp */
	__u32  reserved;                      /* must be 0 */
};

/* =====================================================================
 *  MLA_OP_DEVICE_RESET
 *
 *  Returns the device to probe-equivalent state without rmmod/insmod.
 *
 *  Motivation: mdev->device_fatal (set when the recovery budget is
 *  exceeded or recovery itself fails) was clearable in exactly one
 *  place — mla_sched_init(), which runs only at probe. The recovery
 *  worker returns early once the flag is set, so it can never clear
 *  it. Until this op existed, a device-fatal board required rmmod +
 *  re-insmod, which made per-test isolation impractical for the
 *  fatal-state cases.
 *
 *  This op is DESTRUCTIVE and DEVICE-WIDE. It force-cancels every
 *  in-flight and queued job on every open context with -ECANCELED,
 *  resets the hardware, and clears all fatal state — mdev->device_fatal,
 *  the recovery budget, and every context's ctx->fatal. Callers waiting
 *  on a cancelled job's fence observe -ECANCELED, not a timeout.
 *
 *  Permission: no capability is required when the caller is the ONLY
 *  open context (contexts_open == 1) — the common case for a test
 *  binary, and it lets an unprivileged harness reset between cases.
 *  With any other context open the caller is destroying work it does
 *  not own, which requires CAP_SYS_ADMIN. Deliberately NOT
 *  CAP_SYS_RAWIO (MLA_OP_BUF_ATTACH_PHYS's gate): that would force
 *  root on every test needing a reset.
 *
 *  This op is accepted while the device is fatal — it is the only one.
 *  Every other op short-circuits to -EHOSTDOWN (device) or
 *  -EOWNERDEAD (context) before reaching its handler.
 *
 *  Advertised via MLA_CAP_DEVICE_RESET in mla_info_resp.capabilities.
 * =====================================================================
 */

struct mla_device_reset_req {
	struct mla_cmd_hdr hdr;
	__u64  out_ptr;                       /* struct mla_device_reset_resp * */
	__u32  out_len;
	__u32  reserved;                      /* must be 0 */
};

struct mla_device_reset_resp {
	/* What the reset actually had to undo. A test asserting that a
	 * reset did something must check these rather than just rc == 0:
	 * resetting an already-clean device is a legal no-op that also
	 * returns 0, so rc alone cannot distinguish "recovered" from
	 * "never broken".
	 */
	__u32  jobs_cancelled;                /* in-flight + queued, all ctxs */
	__u32  contexts_cleared;              /* ctxs that had ctx->fatal set */
	__u32  was_device_fatal;              /* 0 or 1, value before the reset */
	__u32  reserved[5];                   /* must be 0 */
};

/* =====================================================================
 *  MLA_OP_PERFCTR_CONFIG
 *
 *  Select which TLC sync-controller signals subsequent jobs report in
 *  mla_job_submit_resp.perf_ctr_us[]. Mirrors mla-rt's `-e N` CLI flag,
 *  which accepts up to four counter ids per run.
 *
 *  Signal ids (mla-rt's `-e` numbering, and the hardware's src_sel):
 *    0  tiled_lat       QG tiles+DMA running time (active+freeze)
 *    1  tile_lat        QG tiles running time
 *    2  tile_active     QG tiles active compute
 *    3  sync_frz/pwr    sync freeze OR tile local power-control asserted
 *    4  tile_pwr-c      tile local power-control asserted
 *    5  sync_frz        sync freeze
 *    6  iq_sync_frz     instruction-queue sync freeze
 *    7  d_sync_frz      data (DMA) sync freeze
 *
 *  Up to MLA_PERFCTR_MAX_SLOTS (4) at a time. That is the hardware
 *  ceiling for QG0, not a policy limit — see the constant's comment.
 *
 *  OPT-IN. Until this op is issued the driver programs nothing, the
 *  completion IRQ skips the counter read, and perf_ctr_us[] is all zeros.
 *  count == 0 returns to exactly that state.
 *
 *  ORDER IS THE CONTRACT: signal_ids[i] is reported in perf_ctr_us[i].
 *  Slots >= count are undefined and returned as zero.
 *
 *  The counters are a DEVICE-WIDE resource — one register block, four
 *  usable slots, no per-context copy — so the op is gated twice:
 *
 *    -EPERM  unless the caller is the only open context
 *            (contexts_open == 1 — the same counter as
 *            MLA_OP_DEVICE_RESET, and it must be that counter and not
 *            num_contexts; see below). These counters exist for debugging
 *            model performance, where there is normally one context.
 *
 *            NOTE this gate has NO CAP_SYS_ADMIN escape, which is a
 *            deliberate difference from MLA_OP_DEVICE_RESET. Root
 *            overriding a reset destroys work visibly and reports in the
 *            response what it destroyed; root overriding a counter
 *            selection would silently hand a peer context measurements of
 *            signals it never chose, with nothing anywhere reporting an
 *            error. There is no diagnostic value in permitting that, so
 *            the gate applies to root too.
 *    -EBUSY  unless the device is idle — nothing in flight and nothing
 *            queued. Reconfiguring under live jobs would leave jobs
 *            straddling the change measuring different signal sets, with
 *            no way to tell which, since perf_ctr_us[] carries no echo.
 *
 *  Because a selection can only be installed while the device is idle,
 *  the register writes happen here and NOTHING is added to the per-job
 *  dispatch path.
 *
 *  A RECOVERY CLEARS THE SELECTION. Counters measure a healthy run; if a
 *  job faulted badly enough to reset the hardware, the numbers are
 *  meaningless and the question is why the model failed, not how long its
 *  tiles were active. Returning post-recovery measurements would invite
 *  the wrong investigation. Reconfigure after a fault if you still want
 *  counters. The selection is likewise dropped when the configuring
 *  context closes its fd.
 *
 *  KNOWN LIMITATION: the sole-context gate is checked at configure time.
 *  A context that opens LATER will read the configured signals in its own
 *  perf_ctr_us[] and cannot detect that it did, because the echo lives
 *  only in this op's response. Bounded by the drop-on-close above. This
 *  is a debugging facility, not an isolated per-context measurement.
 *
 *  Advertised via MLA_CAP_PERFCTR_CONFIG in mla_info_resp.capabilities.
 *  Gate on that bit, never on a zero-valued field: 0 is both a legal
 *  signal id and a legal counter reading, so it cannot be a sentinel.
 * =====================================================================
 */

struct mla_perfctr_config_req {
	struct mla_cmd_hdr hdr;
	__u32  count;                         /* 0..4; 0 disables all */
	__u8   signal_ids[MLA_PERFCTR_MAX_SLOTS];  /* each 0..7; first `count` used */
	__u32  flags;                         /* must be 0 */
	__u64  out_ptr;                       /* struct mla_perfctr_config_resp * */
	__u32  out_len;
	__u32  reserved;                      /* must be 0 */
};

struct mla_perfctr_config_resp {
	/* What the block now actually holds. Echoed rather than assumed:
	 * without it a caller cannot distinguish "measured what I asked
	 * for" from "measured whatever was already programmed", and both
	 * yield four plausible microsecond values.
	 */
	__u32  count;                         /* slots populated; rest undefined */
	__u8   signal_ids[MLA_PERFCTR_MAX_SLOTS];

	/* Generation of the device-wide selection this call installed.
	 *
	 * The kernel bumps it on EVERY change to the selection, including the
	 * software clear inside a recovery (see perf_ctr_us[] above) and a
	 * disable by a closing context. So a caller that remembers the value
	 * returned here and re-reads it from a later op-19 call can tell
	 * "still measuring what I asked for" from "something wiped it and
	 * perf_ctr_us[] has been zeros since" — which perf_ctr_us[] itself
	 * cannot express, 0 being a legal reading.
	 *
	 * Additive: this word was `reserved` and specified as must-be-0, so no
	 * caller can have been relying on a value, and the struct size is
	 * unchanged. Older kernels leave it 0, which is never a valid live
	 * generation.
	 */
	__u32  gen;
	__u32  reserved[1];                   /* must be 0 */
};

/*
 * ============================================================================
 * Ioctl-based async submit/reap transport
 * ============================================================================
 *
 * The async submission surface for /dev/mla: ioctl (not io_uring) so Neat
 * containers run under Docker's DEFAULT seccomp profile, which blocks
 * io_uring_setup/enter/register but permits ioctl. MLA command/job semantics
 * are unchanged — these ops build the same async job as MLA_OP_JOB_ASYNC and
 * reuse struct mla_job_submit_req / struct mla_job_submit_resp verbatim;
 * completion is reaped by draining jobs whose dma-fence has signalled.
 */

/* One queued job in an MLA_IOCTL_SUBMIT_ASYNC batch. @req_user_ptr points at a
 * struct mla_job_submit_req exactly as JOB_ASYNC expects (including out_ptr /
 * out_len for the per-job struct mla_job_submit_resp). @cookie is opaque and is
 * echoed back in struct mla_ioctl_event at reap, mirroring the io_uring
 * user_data demux.
 */
struct mla_ioctl_submit_desc {
	__u64 req_user_ptr;
	__u32 req_len;
	__u32 reserved;        /* must be 0 */
	__u64 cookie;
};

/* MLA_IOCTL_SUBMIT_ASYNC argument. Enqueues @count jobs in one syscall; the
 * kernel writes back the number actually enqueued in @count (equal to the
 * input on full success, or the count submitted before the first enqueue
 * error). Each job completes asynchronously and is collected via
 * MLA_IOCTL_REAP.
 */
struct mla_ioctl_submit {
	__u64 descs;           /* userspace ptr to mla_ioctl_submit_desc[count] */
	__u32 count;           /* in: #descs; out: #enqueued */
	__u32 reserved;        /* must be 0 */
};

/* One completed job reported by MLA_IOCTL_REAP. @result is the job's terminal
 * errno (0 on success), the same value the io_uring CQE res carries. The full
 * struct mla_job_submit_resp has already been written to that job's out_ptr.
 */
struct mla_ioctl_event {
	__u64 cookie;
	__s32 result;
	__u32 reserved;        /* 0 */
};

/* MLA_IOCTL_REAP argument. Blocks per @timeout_ms until at least one job of
 * this context has completed, then drains up to @max completed jobs into
 * @events, writing the number collected into @count.
 *   timeout_ms < 0  block indefinitely
 *   timeout_ms == 0 non-blocking (drain whatever is ready, may return count 0)
 *   timeout_ms > 0  bounded wait in milliseconds
 */
struct mla_ioctl_reap {
	__u64 events;          /* userspace ptr to mla_ioctl_event[max] */
	__u32 max;             /* in: capacity of @events */
	__u32 count;           /* out: #events written */
	__s32 timeout_ms;
	__u32 reserved;        /* must be 0 */
};

/* MLA_IOCTL_SUBMIT_SYNC argument. The synchronous twin of mla_uring_cmd: a
 * blocking ioctl that runs ONE command of any synchronous op (INFO,
 * CTX_CONFIGURE, BUF_*, MODEL_*, SECTION_*, JOB_SUBMIT, JOB_CANCEL,
 * DEVICE_RESET, PERFCTR_CONFIG, MODEL_UPDATE_RELOC) inline and returns its
 * errno directly. @req_user_ptr / @req_len are exactly as the uring SQE carries
 * them — a userspace pointer to the op's struct mla_*_req, whose own out_ptr
 * (if any) receives the response. JOB_ASYNC is rejected here; use
 * MLA_IOCTL_SUBMIT_ASYNC. Layout-identical to struct mla_uring_cmd_sqe by
 * design, so both transports share one dispatch path in the driver.
 */
struct mla_ioctl_submit_sync {
	__u64 req_user_ptr;    /* userspace ptr to struct mla_*_req */
	__u32 req_len;         /* exact size of the *_req struct */
	__u32 reserved;        /* must be 0 */
};

/* Per-context cap on async jobs submitted via MLA_IOCTL_SUBMIT_ASYNC but not
 * yet collected by MLA_IOCTL_REAP: the driver rejects an over-cap submit with
 * -EBUSY, and userspace sizes its reap buffer to it to drain a ctx in one call.
 */
#define MLA_MAX_INFLIGHT_PER_CTX        512

#define MLA_IOC_MAGIC 'M'
#define MLA_IOCTL_SUBMIT_ASYNC  _IOWR(MLA_IOC_MAGIC, 1, struct mla_ioctl_submit)
#define MLA_IOCTL_REAP          _IOWR(MLA_IOC_MAGIC, 2, struct mla_ioctl_reap)
#define MLA_IOCTL_SUBMIT_SYNC   _IOWR(MLA_IOC_MAGIC, 3, struct mla_ioctl_submit_sync)

#endif /* _UAPI_LINUX_SIMAAI_MLA_H */
