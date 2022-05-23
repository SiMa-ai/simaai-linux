/* SPDX-License-Identifier: (GPL-2.0+ WITH Linux-syscall-note) OR MIT */
/*
 * Copyright (c) 2021 Sima ai
 */

#ifndef _SIMAAI_MEMORY_IOCTL_UAPI_H
#define _SIMAAI_MEMORY_IOCTL_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* Allocated buffer info provided by the driver */
struct simaai_memory_info {
	/* buffer ID assigned by kernel */
	__u32 id;
	/* buffer size requested by user */
	__u32 size;
	/* allocated buffer size aligned to page boundary */
	__u32 aligned_size;
	/* buffer starting physical address */
	__u64 phys_addr;
};

/*
 * Target mask to indicate from which target, memory is allocated
 * Topmost bit is used to indicate target memory block
 * No of bits to be used can be expanded in future to add more source
 * Bit(31): 0 -> DRAM
 *			1 -> OCM
 */

#define SIMAAI_TARGET_ALLOCATOR_DRAM 	(0)
#define SIMAAI_TARGET_ALLOCATOR_OCM	 	(1)
#define SIMAAI_TARGET_ALLOCATOR_DMS0	 	(2)
#define SIMAAI_TARGET_ALLOCATOR_DMS1	 	(3)
#define SIMAAI_TARGET_ALLOCATOR_DMS2	 	(4)
#define SIMAAI_TARGET_ALLOCATOR_DMS3	 	(5)
#define SIMAAI_TARGET_ALLOCATOR_EV74	 	(6)
#define SIMAAI_SET_TARGET_ALLOCATOR(x) (((x) & 0xf) << 28)
#define SIMAAI_GET_TARGET_ALLOCATOR(x) (((x) >> 28) & 0xf)

/*
 * Allocate a memory buffer from generic pool
 *
 *  Takes a buffer size.
 *  Binds allocated buffer to the driver file descriptor.
 */
#define SIMAAI_IOC_MEM_ALLOC_GENERIC	_IOW('S', 0, unsigned int)

/*
 * Allocate memory buffer from DMA32 pool
 *
 * Takes a buffer size.
 * Binds allocated buffer to the driver file descriptor.
 */
#define SIMAAI_IOC_MEM_ALLOC_COHERENT	_IOW('S', 1, unsigned int)

/*
 * Free a memory buffer bound to the driver file descriptor.
 */
#define SIMAAI_IOC_MEM_FREE		_IO('S', 2)

/*
 * Get allocated memory buffer by ID
 *
 * Takes a buffer ID.
 * Binds existing buffer to the driver file descriptor.
 */
#define SIMAAI_IOC_MEM_GET		_IOW('S', 3, unsigned int)

/*
 * Get allocated memory buffer information
 */
#define SIMAAI_IOC_MEM_INFO		_IOR('S', 4, struct simaai_memory_info)

#endif /* _SIMAAI_MEMORY_IOCTL_UAPI_H */
