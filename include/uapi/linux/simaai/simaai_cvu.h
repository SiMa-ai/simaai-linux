/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Userspace ABI for the SiMa.ai CVU/EV74 job submission driver.
 *
 * A job is submitted as a graph id plus a per-graph config descriptor
 * (a dma-buf) and a list of buffer relocations. Each relocation names a
 * dma-buf fd and the byte offset inside the descriptor where the buffer's
 * EV74 bus address must be written. The kernel resolves the fds to bus
 * addresses (via the EVXX STU), patches them into the descriptor and
 * dispatches the job to the remote core. Userspace never sees a physical
 * or bus address.
 *
 * Copyright (c) 2025 SiMa.ai
 */
#ifndef _UAPI_LINUX_SIMAAI_CVU_H
#define _UAPI_LINUX_SIMAAI_CVU_H

#include <linux/types.h>
#include <linux/ioctl.h>

/**
 * struct cvu_buf_reloc - one buffer address to patch into the descriptor
 * @desc_offset: byte offset within the config descriptor at which the
 *		 resolved bus address (a __u64) is written
 * @buf_fd:	 dma-buf fd of the input/output buffer to resolve
 * @buf_offset:	 offset added to the buffer's bus address (sub-buffer access)
 */
struct cvu_buf_reloc {
	__u32 desc_offset;
	__s32 buf_fd;
	__u64 buf_offset;
};

/**
 * struct cvu_submit - a synchronous CVU/EV74 job submission
 * @graph_id:	 graph/kernel to run on the remote core
 * @desc_fd:	 dma-buf fd of the per-graph config descriptor
 * @num_relocs:	 number of entries in @relocs, at most CVU_MAX_RELOCS
 * @flags:	 reserved, must be 0
 * @relocs:	 userspace pointer to a struct cvu_buf_reloc[@num_relocs]
 * @request_id:	 opaque id echoed by the caller for tracing
 * @fw_status:	 out: raw firmware ipc_flags of the reply
 * @reserved:	 reserved, must be 0
 */
struct cvu_submit {
	__u32 graph_id;
	__s32 desc_fd;
	__u32 num_relocs;
	__u32 flags;		/* CVU_SUBMIT_* */
	__u64 relocs;
	__u64 request_id;
	__u32 fw_status;	/* out (synchronous submit only) */
	__u32 reserved;
};

/*
 * Upper bound on cvu_submit.num_relocs. A graph names one buffer per pointer
 * field in its config descriptor. The limit is deliberately equal to the
 * driver's default per-fd cache size, so a job can never bind more buffers
 * than the driver can hold resolved at once. Submitting more returns -EINVAL.
 */
#define CVU_MAX_RELOCS		64

/*
 * cvu_submit.flags: by default CVU_IOC_SUBMIT blocks until the EV74 replies.
 * With CVU_SUBMIT_NONBLOCK it returns as soon as the request is dispatched;
 * the caller then waits with poll() (EPOLLIN) and retrieves the outcome with
 * read() as a struct cvu_completion. One request may be in flight per open fd.
 */
#define CVU_SUBMIT_NONBLOCK	(1u << 0)

/* Outcome of an asynchronous submit, returned by read() on the /dev/cvu fd. */
struct cvu_completion {
	__u64 request_id;	/* echoes the cvu_submit.request_id */
	__u32 fw_status;	/* raw firmware ipc_flags of the reply */
	__s32 result;		/* 0 on success, negative errno on failure */
};

#define CVU_IOC_MAGIC	'V'
#define CVU_IOC_SUBMIT	_IOWR(CVU_IOC_MAGIC, 1, struct cvu_submit)

#endif /* _UAPI_LINUX_SIMAAI_CVU_H */
