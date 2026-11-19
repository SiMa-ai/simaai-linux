.. SPDX-License-Identifier: GPL-2.0

===================
SiMa MLA DMA memory
===================

The standard Modalix SoM device trees use reusable DMS memory, available to
Linux while MLA does not own allocations. DMS placement and size depend on
the board; legacy profiles may retain no-map carveout backing.

Memory ownership
================

The board's DMS interval is included in System RAM and declared as a
non-default, reusable ``shared-dma-pool`` with the
``simaai,mla-cma`` compatible. It has neither ``no-map`` nor ``carved-out``.
The existing low CMA area and EV/Allegro reservations are unchanged.

The existing ``/dev/dma_heap/simaai,dms`` name now exports this CMA area.
There is one allocator, not a carveout gen_pool layered over Linux pages.
The MLA device also attaches to this same reserved-memory area before
allocating descriptor backing. Required options are ``CONFIG_DMA_CMA``,
``CONFIG_DMABUF_HEAPS_CMA`` and ``CONFIG_SIMAAI_MLA``.

Linux may use idle CMA pages for movable allocations. A DMA allocation
migrates movable occupants as needed and retains exclusive page ownership
until the final DMA-BUF/backing reference is released. This is not a
promise that any requested contiguous size can always be obtained:
long-term pins, memory pressure and fragmentation can cause failure.
Cold allocation can be expensive. Extra System RAM also needs kernel
metadata, so the increase in MemTotal is less than the DMS region size.

Applications must use allocated DMA-BUF handles, not infer ownership from
a physical address. ``BUF_ATTACH_PHYS`` rejects any interval overlapping
System RAM even for privileged callers; privileged legacy non-RAM
attachments remain supported. Old runtimes using raw DMS addresses must
migrate before using reusable DMS.

Shared MLA fallback heap
========================

Modalix registers ``/dev/dma_heap/simaai,mla`` automatically, and the MLA
driver uses it for model sections and descriptor backing. Each allocation
tries ``simaai,dms``, then physically contiguous ordinary Linux RAM,
then default CMA shared with EV74 and other devices.

Userspace tensor allocators must open ``simaai,mla`` to use this policy;
other heap endpoints retain their allocation behavior. Contiguous allocation
is best effort, with no reserved Linux or EV74 headroom.

``/sys/kernel/debug/mla_heap/stats`` reports live Linux and CMA fallback
bytes. Run ``tools/testing/selftests/dmabuf-heaps/mla-heap`` for heap checks;
real-model validation below is still required.

DMA and cache contract
======================

* Import validates the entire mapped DMA extent: nonzero segment lengths,
  adjacent addresses, coverage, alignment and the 40-bit DRAM address limit.
  A physically contiguous allocation can have multiple scatterlist entries;
  an entry's length field cannot represent a whole allocation above 4 GiB.
* Both legacy carveout and CMA heap buffers use their exporter's CPU-access
  hooks. A matching heap name does not imply no-map backing.
* Descriptor allocations retain their write-back CPU mapping. Page-backed
  buffers use range-scoped cache maintenance; legacy no-map buffers use
  exporter hooks. The shared heap's fallback order also applies to descriptors
  and their private copies, including default CMA.
* Userspace must pair CPU access with ``DMA_BUF_IOCTL_SYNC`` and wait for the
  relevant device completion before reading outputs or releasing ownership.
  Allocation or a CPU mapping alone is not cache synchronization.
* Open contexts, retained jobs/fences and MLA-exported buffers retain module
  ownership. Userspace driver bind/unbind is disabled for this fixed platform
  device. Forced module unload and dynamic platform-device removal are not
  supported lifetime mechanisms. Device CMA association is released only
  during normal, drained driver teardown.

Cold descriptor-bank preparation
================================

``MLA_CAP_PREPARED_BANKS`` and the additive INFO field
``max_prepared_banks`` negotiate optional cold provisioning.
After checking the capability, userspace sends the complete optional
``MODEL_DEFINE_V3`` tail:

.. code-block:: c

   request.prepared_bank_count = required_banks;
   request.prepared_reserved = 0;

An absent or zero count preserves legacy lazy growth. A nonzero count
provisions exactly that many descriptor banks, after static/default LoRA
relocations and before model publication. Allocation failure aborts
definition and frees partial state. Unsupported counts and malformed
partial tails are rejected.

After successful strict definition, enqueue never grows descriptor backing.
Extra jobs use existing bank availability and backpressure. The count is
a preparation-overlap budget, not a job queue limit: do not allocate the
maximum for every model by default. This guarantee is specifically about
descriptor-bank backing, not every small allocation in the submission path.

The runtime must include the preparation policy in its model-cache identity,
or otherwise verify that a reused model satisfies the request. An older kernel
can ignore unknown request tails; a successful DEFINE without capability
negotiation does not establish the strict guarantee. No runtime changes are
provided by the Linux implementation.

Validation and rollout
======================

Compile the target board DTs and validate the binding. Exercise DMA
extent validation, request compatibility and partial-allocation cleanup with
separately maintained tests. Host-only checks do not prove cache coherency or
actual CMA migration.

Qualify a matching kernel/DT and V3 runtime on the physical board:

#. Confirm the DMS interval is System RAM and dedicated CMA, with no duplicate
   carveout heap or change to low CMA. Allocate movable Linux data into the
   area before requesting MLA memory; MemTotal alone is insufficient evidence.
#. Run real MLA models after migration, including multi-segment allocations
   above 4 GiB where workload capacity permits. Check dirty CPU/device data at
   boundaries, offsets and tails.
#. Inject bank-allocation failures and verify no model publication or leaked
   backing; then retry. Trace strict warm submissions and verify no descriptor
   backing allocations.
#. Exercise concurrent jobs, cancellation, reset, process exit and retained
   output DMA-BUFs. Memory must return only after the final owner and hardware
   access have ended. Combine with the separately maintained bank-ownership
   fixes; provisioning does not replace their synchronization protocol.
#. Stress aged memory and alternating model sizes alongside EV/Allegro. Record
   allocation success, cold preparation latency/CPU cost, warm median latency,
   and returned memory separately. Restore the original kernel/DT after trials.

Changing DT selects that profile's DMS backing; the shared fallback heap
remains automatic with either reusable or legacy carveout DMS.
