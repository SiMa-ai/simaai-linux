/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * SiMa.ai HPI user-space ABI (/dev/simaai-hpi).
 *
 * GET_MEM + mmap expose the shared DDR so user space can stage
 * payloads and read tRoot's responses in place.
 *
 * Copyright (c) 2026 SiMa ai
 */
#ifndef _UAPI_LINUX_SIMAAI_HPI_H
#define _UAPI_LINUX_SIMAAI_HPI_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define SIMAAI_HPI_IOC_MAGIC	'H'

struct simaai_hpi_msg {
	__u32 p0;
	__u32 p1;
};

struct simaai_hpi_mem {
	__u64 phys;
	__u64 size;
};

#define SIMAAI_HPI_SEND		_IOW(SIMAAI_HPI_IOC_MAGIC, 1, struct simaai_hpi_msg)
#define SIMAAI_HPI_RECV		_IOR(SIMAAI_HPI_IOC_MAGIC, 2, struct simaai_hpi_msg)
#define SIMAAI_HPI_XFER		_IOWR(SIMAAI_HPI_IOC_MAGIC, 3, struct simaai_hpi_msg)
#define SIMAAI_HPI_GET_MEM	_IOR(SIMAAI_HPI_IOC_MAGIC, 4, struct simaai_hpi_mem)

#endif /* _UAPI_LINUX_SIMAAI_HPI_H */

