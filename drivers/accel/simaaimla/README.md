# mla-driver

Linux kernel driver for the SiMa.ai MLA (Machine Learning Accelerator)
on Modalix-class silicon. Exposes `/dev/mla` and serves a uniform
io_uring-based ABI to host processes, containers, and (eventually)
VM guests.

## What's implemented

| ABI op | Description |
|---|---|
| `MLA_OP_INFO` | Capability query (tile dims, alignments, dma_mask) |
| `MLA_OP_CTX_CONFIGURE` | Per-fd priority/weight/kind mutation |
| `MLA_OP_BUF_ALLOC` | Allocate physically-contiguous dma-buf |
| `MLA_OP_BUF_IMPORT` | Import external dma-buf (camera, ISP, etc.) |
| `MLA_OP_BUF_INFO` | Look up buffer metadata by id |
| `MLA_OP_BUF_ATTACH_PHYS` | Wrap fixed phys range as dma-buf (CAP_SYS_RAWIO) |
| `MLA_OP_BUF_RELEASE` | Drop buffer reference |
| `MLA_OP_MODEL_DEFINE` | **Retired** — returns `-EOPNOTSUPP`; use `MLA_OP_MODEL_DEFINE_V3` |
| `MLA_OP_MODEL_RELEASE` | Drop model reference |
| `MLA_OP_JOB_SUBMIT` | Submit inference (sync via `MLA_CMD_FLAG_WAIT`, async with sync_file fd) |
| `MLA_OP_JOB_CANCEL` | Cancel queued job |

The driver also patches DMA descriptors in-kernel using a
caller-supplied relocation list, so user-space (and future VM guests)
never needs to know host-physical addresses.

## Build

Out-of-tree against a prepared kernel source tree. This `Makefile` carries only
kbuild directives, so drive kbuild directly. `CONFIG_SIMAAI_MLA=m` must be set
on the command line — it is not in the build kernel's `.config`, and without it
kbuild exits 0 having built nothing:

```
make -C <path-to-prepared-simaai-linux-tree> M=$(pwd) \
     ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
     CONFIG_SIMAAI_MLA=m modules
```

The `mla_rt` repository wraps that invocation at `scripts/Makefile.oot`, which
defaults both paths and supplies `CONFIG_SIMAAI_MLA` for you.

The userspace tests link `liburing`:

```
cd test && make
```

## Bring-up

Requires a DTB with `compatible = "simaai,mla-modalix"` at
`mla@5000000`. Board.53 has it built in; board.46 gets it via the
DTBO overlay in `/workspace/mla-kernel-bundle/dts-overlays/`.

```bash
# 1. Prepare DMS memory carveout (SDK-provided).
sudo bash /usr/bin/init_mla_memory.sh

# 2. Load the module.
sudo insmod mla.ko mla_desc_cma_enable=1

# 3. Open /dev/mla for non-root.
sudo chmod a+rw /dev/mla
```

`setup-kernel-driver.sh` in the kernel bundle wraps all three.

## Module parameters

| Param | Default | Perm | Description |
|---|---|---|---|
| `mla_do_dma_sync` | 1 | 0644 | Per-submit dma_sync of IFM/OFM/model |
| `mla_desc_cma_enable` | 1 | 0444 | Route v2 descriptor blobs to WB linear-map alias (WC fallback if 0) |
| `mla_reloc_range_flush` | 1 | 0644 | Range-scoped krlc flush (full-buffer fallback if 0) |
| `mla_debug` | 0 | 0644 | Verbosity bitmask. Bit 0 (`MLA_DBG_TRACE=0x1`) enables one-shot per-event traces (MODEL_DEFINE / JOB_SUBMIT / PROGDONE / enqueue / burst-dance / prep-ahead). Bit 1 (`MLA_DBG_HOT=0x2`) enables per-item inner-loop traces (per-tile / per-DFM / per-reloc / per-iotensor / per-section) — heavy, can trip the 2s JOB_SUBMIT timeout on cache_token2047 (~2000+ lines/job × ~15ms/line serial printk). Use `mla_debug=3` for full trace. Runtime-toggleable via `/sys/module/mla/parameters/mla_debug`. |
| `mla_no_recover` | 0 | 0644 | Freeze state on job fault (skip fatalize + recovery) for post-mortem |
| `mla_reprog_always` | 0 | 0644 | Bypass last_programmed_model cache — perf measurement only |
| `mla_skip_hw` | 0 | 0644 | Skip GO + fake PROGDONE for sched/fence isolation (no HW) |

Reset + burst-size programming + IRQ-enable run unconditionally at
probe; no module knobs.

## Source layout

The public ABI header lives with all other simaai uapi headers at the
kernel tree root: `include/uapi/linux/simaai/mla.h`.

Driver sources under `drivers/accel/simaaimla/`:

```
mla_drv.c        probe/remove + module params + fops + uring_cmd dispatcher + IRQ handlers
mla_ops.c        INFO / CTX_CONFIGURE / MODEL_DEFINE / MODEL_RELEASE / JOB_SUBMIT / JOB_CANCEL
mla_ops_buf.c    BUF_ALLOC/IMPORT/INFO/ATTACH_PHYS + buffer registry
mla_dmabuf.c     dma-buf exporter for WB CMA section buffers (CPU_RMW)
mla_pool.c       carveout-heap consumer glue + kernel-owned pool + section dedup table
mla_job.c        job lifecycle, dma-fence ops, watchdog
mla_sched.c      strict-priority FIFO scheduler, single in-flight
mla_hw.c         DMC/tile/quad register programming + GO + reloc patcher
mla_plist.c      vendored plist copy (until upstream exports plist_add/del)
mla_drv.h        internal types
mla_regs.h       MLA register offsets
mla_image_fmt.h  shared DMA-unit constants

test/
├── test_mla_info.c            ABI smoke
├── test_mla_buf.c             buffer ops
└── test_mla_section_define.c  MODEL_DEFINE end-to-end
```

## Architecture plans

The ABI design and the multi-slice implementation roadmap live in:

- `~/.claude/plans/we-have-our-existing-lucky-lerdorf.md` — overall
  architecture (kernel driver + dma-buf + dma-fence + virtio-mla).
- `~/.claude/plans/mla-driver-abi-sketch.md` — public ABI spec.
- `~/.claude/plans/mla-driver-abi-sketch_implementation.md` — slice
  breakdown.
- `~/.claude/plans/mla-driver-mlart-integration.md` — how mla-rt
  maps onto the new ABI.
