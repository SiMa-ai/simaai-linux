/* SPDX-License-Identifier: (GPL-2.0+ WITH Linux-syscall-note) OR MIT */
/*
 * Copyright (c) 2021 Sima ai
 */

#ifndef _SIMAAI_MLAMEM_IOCTL_UAPI_H
#define _SIMAAI_MLAMEM_IOCTL_UAPI_H

#include <linux/types.h>

struct simaai_reg_info {
	/* register index */
	__u32 reg_idx[4];
	/* register value */
	__u32 reg_value[4];
	/* number of regs to be r/w */
	__u32 num_of_regs;
	/* driver revision */
	__u32 rev;
};

/*
 * Writes to HW register
 */
#define SIMAAI_IOC_WRITE_REG	_IOW('M', 0, struct simaai_reg_info)

/*
 * Reads from HW register
 */
#define SIMAAI_IOC_READ_REG		_IOR('M', 1, struct simaai_reg_info)

/*
 * Gets driver revision
 */
#define SIMAAI_IOC_GET_REV		_IOR('M', 2, struct simaai_reg_info)

#endif /* _SIMAAI_MLAMEM_IOCTL_UAPI_H */
