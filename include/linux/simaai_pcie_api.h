// SPDX-License-Identifier: GPL-2.0
/**
 * PCIe EP driver for SiMa.ai SoC
 *
 * Copyright (C) 2025 SiMa.ai
 * Author:
 */

/*
 * Any data types or definition that is shared with the userspace goes
 * in this file. The libraries should only need to include this file.
 */

#ifndef _SIMA_PCIE_API_H_
#define _SIMA_PCIE_API_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#ifndef __aligned
#define __aligned(x) __attribute__((aligned(x)))
#endif
#ifndef __packed
#define __packed __attribute__((packed))
#endif

#define SI_MAX_DQS 8

#define SI_CMD	'S'

/*
 * Host Driver ioctls
 */
/* Commands that interacts with soc */
#define SI_SOC_GET_STATUS _IOWR(SI_CMD, 0x01, struct si_soc_status)
#define SI_SOC_GET_CAPS _IOW(SI_CMD, 0x02, __u32)
#define SI_SOC_SET_TIME	_IOW(SI_CMD, 0x03, struct si_mcmd_timesync_params)
#define SI_SOC_REBOOT	_IO(SI_CMD, 0x04)
#define SI_SOC_INSTALL_IMG _IOWR(SI_CMD, 0x05, struct si_io)

/* Commands that host driver serves */
#define SI_BUFF_GET _IOR(SI_CMD, 0x20, struct si_buffer)
#define SI_BUFF_PUT _IOW(SI_CMD, 0x21, struct si_buffer)
#define SI_DEV_REM_RESCAN _IO(SI_CMD, 0x22)
#define SI_DEV_SBRESET _IO(SI_CMD, 0x23)
#define SI_HW_GETINFO _IOR(SI_CMD, 0x24, struct si_hwinfo)

/* Commands that has work on both sides */
#define SI_CREATE_MWQ _IOW(SI_CMD, 0x10, struct si_qcmd_param)
#define SI_CREATE_DWQ _IOW(SI_CMD, 0x11, struct si_qcmd_param)
#define SI_CREATE_MRQ _IOW(SI_CMD, 0x12, struct si_qcmd_param)
#define SI_CREATE_DRQ _IOW(SI_CMD, 0x13, struct si_qcmd_param)
#define SI_CREATE_MCQ _IOW(SI_CMD, 0x14, struct si_qcmd_param)
#define SI_CREATE_DCQ _IOW(SI_CMD, 0x15, struct si_qcmd_param)

#define SI_DELETE_MWQ _IOW(SI_CMD, 0x16, __u32)
#define SI_DELETE_DWQ _IOW(SI_CMD, 0x17, __u32)
#define SI_DELETE_MRQ _IOW(SI_CMD, 0x18, __u32)
#define SI_DELETE_DRQ _IOW(SI_CMD, 0x19, __u32)
#define SI_DELETE_MCQ _IOW(SI_CMD, 0x1a, __u32)
#define SI_DELETE_DCQ _IOW(SI_CMD, 0x1b, __u32)

#define SI_QUEUE_FLUSH _IOW(SI_CMD, 0x1c, __u32)

/*
 * SoC Driver ioctls
 */
#define SI_GET_DATA _IOR(SI_CMD, 0x50, struct si_data_param)

#define SI_MAX_MCMD_TIMEOUT 300 * 1000

/* Image transfer block size */
#define SI_MGMT_BUFF_SIZE (512 * 1024)

/* Different types of frames that can be sent to SoC */
#define SI_IO_TYPE_DATA		0x1000
#define SI_IO_TYPE_CAPS		0x1001
#define SI_IO_TYPE_EOS		0x1002
#define SI_IO_TYPE_FILE		0x2003
#define SI_IO_TYPE_FFRAG	0x2004
#define SI_IO_TYPE_JSON		0x2005

/*
 * Management and Data requests
 */
#define SI_FRAME_PROCESS	0x01
#define SI_DATA_SEND		0x02
#define SI_DATA_RECEIVE		0x03

#define SI_FILE_SET_NAME	0x05
#define SI_FILE_SEND		0x06
#define SI_FILE_RECEIVE		0x07

/* Response codes */
enum si_resp {
	SI_RES_INVALID = 0xFFFFFFFF,
	SI_RES_SUCCESS = 0x7fffffa5,
	SI_RES_FAILURE = 0x7fffffff,
	SI_RES_ABORTED = 0x7ffffffe,
};

/*
 * Types of queues we use
 */
enum si_qtype {
	SI_ALL,	/* Placeholder for all queues */
	SI_MWQ,	/* Management work queue */
	SI_MCQ,	/* Management completion queue */
	SI_MRQ,	/* Management receive queue */
	SI_DWQ,	/* Data work queue */
	SI_DCQ,	/* Data completion queue */
	SI_DRQ,	/* Data receive queue */
	SI_QTYPE_MAX
} __packed;

/*
 * These structures should fit the parameter areas of si_wqe, si_cqe, and si_rqe
 * They should be accessible to userspace applications and rest of *qes are not
 * visible to userspace.
 *
 * We limit the size of queue entries to 64 bytes. This will leave us with 32
 * bytes for parametes.
 */
struct si_io_desc {
	__u32 type;
	__u32 resp;
	__s32 err;
	__u32 frame_id_l;
	__u32 frame_id_h;
	__u32 stream_id;
	__u32 frame_idx;
	__u32 rsvd;
} __aligned(4) __packed;

struct si_io_chunk {
	__u32 type;
	__u32 resp;
	__s32 err;
	__u32 seq_id;
	__u32 cur_off_l;
	__u32 cur_off_h;
	__u32 tot_len_l;
	__u32 tot_len_h;
} __aligned(4) __packed;

struct si_qcmd_param {
	enum si_qtype qtype;
	__u32 qid;
	__u32 qsize;
	__u32 qdepth;
	__u32 bsize;
	__u32 timeout;
} __aligned(4) __packed;

struct si_mcmd_timesync_params {
	__u32 time_sec_lo;
	__u32 time_sec_hi;
	__u32 time_nsec_lo;
	__u32 time_nsec_hi;
	__u32 tz_minuteswest;
	__u32 tz_dsttime;
} __packed __aligned(4);

enum si_buffer_type {
	SI_BUF_TYPE_INVALID,
	SI_BUF_TYPE_TX,
	SI_BUF_TYPE_RX,
	SI_BUF_TYPE_MAX
};

enum si_buffer_addr_type {
	SI_BUF_ADDR_INVALID,
	SI_BUF_ADDR_NONE,
	SI_BUF_ADDR_USER,
	SI_BUF_ADDR_PHYSICAL,
	SI_BUF_ADDR_KVIRTUAL,
	SI_BUF_ADDR_SGE,
	SI_BUF_ADDR_SGE_LIST,
	SI_BUF_ADDR_MAX
};

#define SI_NUM_BUFS_MAX	16

/*
 * Buffer types and requirements are different in SOC and host due to the
 * availability of simaai-mem on soc and the way things are handled. Aim
 * is to have a uniform structure
 */
struct si_buffer_id {
	__u32 btype:2;
	__u32 qid:8;
	__u32 idx:10;
	__u32 rsvd:12;
};

struct si_buffer {
	union {
		struct si_buffer_id bid;
		__u32 bid_int;
	};
	__u32 addrtype:4;
	__u32 tsize;
	__u32 nents;
	union {
		size_t size;
		size_t *sizes;
	};
	union {
		void *addr;
		void **addrs;
	};
	__u32 qidx;
}__aligned(4) __packed;

struct si_soc_status {
	__u32 status0;
	__u32 status1;
	__u32 status2;
	__u32 status3;
};

struct si_hwinfo {
	__u8 bus_name[64];
};

struct si_file_info {
	__u8 name[512];
	off_t size;
	mode_t mode;
};

struct si_io {
	__u32 cmd;
	enum si_resp resp; /* Remove this */
	union {
		struct si_io_desc desc;
		struct si_io_chunk chunk;
	};
	struct si_buffer buffer;
};

struct si_data_param {
	struct si_buffer src;
	struct si_buffer dst;
};

#endif

