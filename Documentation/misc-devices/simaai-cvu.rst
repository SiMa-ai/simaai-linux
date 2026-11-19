.. SPDX-License-Identifier: GPL-2.0

======================================
SiMa.ai CVU/EV74 job submission driver
======================================

:Author: Nileshkumar Raghuvanshi <nilesh.r@sima.ai>

Overview
========

The CVU is an EV74 vector-DSP remote processor on the SiMa.ai Modalix SoC. It
runs image-processing graphs on buffers that live in DRAM. This driver,
``simaai-cvu``, lets userspace submit a graph to the CVU without ever handling
a physical or bus address: the application passes dma-buf file descriptors and
the kernel resolves them to the addresses the EV74 needs.

The driver is an rpmsg client. It binds the name-service channel the EV74
firmware announces once it has booted, and exposes a single misc device,
``/dev/cvu``. Because the channel only appears after the firmware is running,
so does the device node.

Why the kernel owns the addresses
=================================

A CVU graph reads its inputs and writes its outputs by DMA. The firmware
descriptor for a graph therefore needs the EV74 *bus* address of every buffer.
Historically userspace obtained those addresses (via a dma-buf ioctl) and wrote
them into the descriptor itself. That leaked the physical memory map to
applications and, worse, let a buggy or malicious application point the EV74 -
a bus master - at arbitrary physical memory, since the firmware dereferences
whatever addresses the descriptor contains.

With this driver the application only ever provides dma-buf fds. For each
buffer the kernel:

* resolves the fd to a CPU physical address;
* translates it to the EV74 bus address through the EVXX STU (system
  translation unit);
* bounds-checks the offset against the buffer and the descriptor size;
* writes the result into the descriptor at a caller-supplied offset.

The EV74 can thus only reach buffers the caller legitimately holds an fd to.

Submitting a job
================

A job is described by ``struct cvu_submit`` (see
``include/uapi/linux/simaai/simaai_cvu.h``):

* ``graph_id`` - the graph to run on the CVU;
* ``desc_fd`` - dma-buf fd of the per-graph config descriptor;
* ``relocs`` / ``num_relocs`` - an array of ``struct cvu_buf_reloc``, one per
  buffer, each naming a dma-buf fd (``buf_fd``), the byte offset within the
  descriptor at which its bus address must be written (``desc_offset``) and an
  offset added to that address for sub-buffer access (``buf_offset``);
* ``request_id`` - an opaque id echoed back for tracing.

The application fills the descriptor's scalar fields (image dimensions and so
on), leaving the buffer-address slots to the kernel, and issues::

    ioctl(fd, CVU_IOC_SUBMIT, &submit);

By default the ioctl is synchronous: it dispatches the job and blocks until the
CVU replies (or ``cvu_timeout_ms`` elapses). ``fw_status`` returns the raw
firmware reply flags.

Asynchronous submission
=======================

Set ``CVU_SUBMIT_NONBLOCK`` in ``flags`` to return as soon as the job is
dispatched. The caller then waits for completion with ``poll()`` (``POLLIN``)
and collects the outcome with ``read()`` as a ``struct cvu_completion``::

    submit.flags = CVU_SUBMIT_NONBLOCK;
    ioctl(fd, CVU_IOC_SUBMIT, &submit);

    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    poll(&pfd, 1, timeout_ms);

    struct cvu_completion done;
    read(fd, &done, sizeof(done));      /* done.result: 0 or negative errno */

``read()`` returns ``-EAGAIN`` on a non-blocking fd with no result ready, and
``-ENODATA`` if nothing was submitted.

Multiple clients
================

Each ``open()`` of ``/dev/cvu`` gets its own rpmsg endpoint. The endpoint
address is carried in the request so the firmware routes the reply back to
exactly that endpoint. Separate open file descriptors - separate processes, or
threads that each open their own fd - therefore submit concurrently and receive
their own replies with no shared lock. A single fd carries at most one request
in flight; a second submit on a busy fd returns ``-EBUSY``.

The EV74 executes graphs one at a time, so concurrent submitters are serviced
in turn rather than in parallel; the driver adds no serialization of its own.

Module parameters
=================

``cvu_timeout_ms``
    Completion timeout for a synchronous submit, in milliseconds
    (default 5000).
