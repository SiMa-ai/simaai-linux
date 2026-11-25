/* SPDX-License-Identifier: (GPL-2.0+ WITH Linux-syscall-note) OR MIT */
/*
 * Copyright (c) 2021 Sima ai
 */

#ifndef _SIMAAI_MEMORY_IOCTL_UAPI_H
#define _SIMAAI_MEMORY_IOCTL_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define MAX_SEGMENTS					(16)
/* Allocated buffer info provided by the driver */
struct simaai_memory_info {
	/* buffer target */
	__u32 target;
	/* buffer size requested by user */
	__u32 size;
	/* allocated buffer size aligned to page boundary */
	__u32 aligned_size;
	/* buffer flags */
	__u32 flags;
	/* buffer starting physical address */
	__u64 phys_addr;
	/* buffer starting bus address */
	__u64 bus_addr;
	/* offset from the parent segment */
	__u64 offset;
};

/* Allocate buffer */
struct simaai_alloc_args {
	/* buffer target */
	__u32 target;
	/* buffer size requested by user */
	__u32 size[MAX_SEGMENTS];
	/* allocated buffer size aligned to page boundary */
	__u32 aligned_size;
	/* buffer flags */
	__u32 flags;
	/* buffer starting physical address */
	__u64 phys_addr[MAX_SEGMENTS];
	/* buffer starting bus address */
	__u64 bus_addr[MAX_SEGMENTS];
	/* num of segments */
	__u32 num_of_segments;
	/* offset from parent segment */
	__u64 offset[MAX_SEGMENTS];
};

/* Free buffer */
struct simaai_free_args {
	/* buffer ID assigned by kernel */
	__u64 phys_addr[MAX_SEGMENTS];
	/* num of segments */
	__u32 num_of_segments;
};

/* memcpy args */
struct simaai_memcpy_args {
	/* src phy addr */
	__u64 src_addr;
	/* src buffer offset */
	__u64 src_offset;
	/* dst phy addr */
	__u64 dst_addr;
	/* dst buffer offset */
	__u64 dst_offset;
	/*  copy buffer size  */
	__u32 size;
};

/*
 * Target mask to indicate from which target, memory is allocated
 */
#define SIMAAI_TARGET_ALLOCATOR_DRAM		(0)
#define SIMAAI_TARGET_ALLOCATOR_OCM			(1)
#define SIMAAI_TARGET_ALLOCATOR_DMS0	 	(2)
#define SIMAAI_TARGET_ALLOCATOR_DMS1	 	(3)
#define SIMAAI_TARGET_ALLOCATOR_DMS2	 	(4)
#define SIMAAI_TARGET_ALLOCATOR_DMS3	 	(5)
#define SIMAAI_TARGET_ALLOCATOR_EV74	 	(6)

#define SIMAAI_BUFFER_FLAG_CACHED		BIT(0)
#define SIMAAI_BUFFER_FLAG_RDONLY		BIT(1)

/*
 * Allocate memory buffer from DMA32 pool
 *
 * Takes a buffer size.
 * Binds allocated buffer to the driver file descriptor.
 */
#define SIMAAI_IOC_MEM_ALLOC_COHERENT	_IOWR('S', 0, struct simaai_alloc_args)

/*
 * Free a memory buffer bound to the driver file descriptor.
 */
#define SIMAAI_IOC_MEM_FREE		_IOW('S', 1, struct simaai_free_args)

/*
 * Get allocated memory buffer information
 */
#define SIMAAI_IOC_MEM_INFO		_IOWR('S', 2, struct simaai_memory_info)

/*
 * memcpy through sdma
 */
#define SIMAAI_IOC_MEMCPY		_IOWR('S', 3, struct simaai_memcpy_args)

#endif /* _SIMAAI_MEMORY_IOCTL_UAPI_H */
