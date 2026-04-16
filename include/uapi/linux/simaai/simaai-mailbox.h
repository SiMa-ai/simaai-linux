/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright SiMa.ai (C) 2021. All rights reserved
 */

#ifndef _UAPI_LINUX_SIMAMAILBOX_H
#define _UAPI_LINUX_SIMAMAILBOX_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

struct sima_ipc_command {
	union {
		struct {
			uint8_t ipc_class;
			uint8_t ipc_flags;
			uint16_t ipc_code;
		};
		uint32_t v;
	};
} __attribute__((packed));

struct sima_ipc_message {
	uint16_t version;
	uint16_t sequence_id;
	struct sima_ipc_command message_type;
	union {
		struct {
			uint32_t address;
			uint32_t size;
		};
		uint8_t data[8];
	} payload;
} __attribute__((packed));

enum sima_ipc_command_class {
	SIMA_IPC_COMMAND_COMMON = 1,
	SIMA_IPC_COMMAND_STATUS,
	SIMA_IPC_COMMAND_SYNC,
	SIMA_IPC_COMMAND_BOOT,
	SIMA_IPC_COMMAND_SECURITY,
	SIMA_IPC_COMMAND_DEBUG,
	SIMA_IPC_COMMAND_MLA,
	SIMA_IPC_COMMAND_LAST,
};

#endif /* _UAPI_LINUX_SIMAMAILBOX_H */
