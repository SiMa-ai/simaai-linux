/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright SiMa.ai (C) 2021. All rights reserved
 */

#ifndef _LINUX_SIMMAI_MAILBOX_H_
#define _LINUX_SIMMAI_MAILBOX_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define SIMAAI_MAX_DATA_SIZE	(0x800)
#define SIMAAI_MAX_MSG_SIZE	(sizeof(size_t) + SIMAAI_MAX_DATA_SIZE)
/**
 * struct simaai_mbmsg - SiMa.ai mailbox message structure
 * @len:  Length of message
 * @data: message payload
 *
 * This is the structure for data used in mbox_send_message
 * the maximum length of data buffer is fixed to 4 kilobytes.
 * Client is supposed to be aware of this.
 */
struct simaai_mbmsg {
	size_t len;
	u8 data[];
};


#endif /* _LINUX_SIMMAI_MAILBOX_H_ */
