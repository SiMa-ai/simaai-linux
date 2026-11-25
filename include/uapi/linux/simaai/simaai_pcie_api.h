/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/**
 * PCIe EP driver for SiMa.ai SoC
 *
 * Copyright (C) 2025 SiMa.ai
 * Author: SiMa.ai SoC SW Team
 */

/*
 * Any data types or definition that is shared with the userspace goes
 * in this file. The libraries should only need to include this file.
 */

#ifndef _SIMA_PCIE_API_H_
#define _SIMA_PCIE_API_H_

#include <linux/types.h>
#include <linux/ioctl.h>

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
#define SI_GET_BUF_ALLOC_MODE _IOR(SI_CMD, 0x25, __u32)
#define SI_BUFF_ENROLL _IOW(SI_CMD, 0x26, struct si_buf_enroll)

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
 * Query the parameters of an existing queue, so a process joining a queue
 * another process created can size its buffers to what was actually
 * allocated rather than to what it would have asked for.
 *
 * On the host, issue it on the card node with qtype and (for data queues)
 * qid filled in. On the SoC, issue it on a queue device node; the queue is
 * implied by the node and the input fields are ignored. Both sides fill in
 * qsize (entries), qdepth and bsize (bytes per ring buffer, as allocated)
 * from the live queue context and return -ENOENT when the queue does not
 * exist yet.
 *
 * The WQ/RQ/CQ triple of a queue set is created with one geometry, so
 * querying the WQ describes the set.
 */
#define SI_QUEUE_GETINFO _IOWR(SI_CMD, 0x1d, struct si_qcmd_param)

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
 * Frame types of the converged interface (libsimaaipcie / libsimaaipep).
 *
 * These occupy the low 16 bits of si_io_desc.type / si_io_chunk.type. The
 * upper 16 bits carry a magic byte and per-frame flags, which is what tells
 * the two ends apart from the types above: a frame of the older protocol has
 * zero in its upper 16 bits and is never mistaken for one of these.
 *
 * Nothing in either driver interprets any of this. The values live here
 * because both userspace libraries have to agree on them and this is the
 * only header they already share.
 */
#define SI_IO_TYPE_CMD		0x3000	/* command with payload, one reply */
#define SI_IO_TYPE_MANIFEST	0x3001	/* opens a bulk transfer */
#define SI_IO_TYPE_ITEM		0x3002	/* one file, directory or symlink */
#define SI_IO_TYPE_CHUNK	0x3003	/* one slice of an item's content */
#define SI_IO_TYPE_END		0x3004	/* closes an item or a transfer */
#define SI_IO_TYPE_PULL		0x3005	/* host polls for SoC-to-host data */
#define SI_IO_TYPE_FRAME	0x3006	/* data buffer submitted for work */
#define SI_IO_TYPE_RESULT	0x3007	/* data buffer returned after work */

/*
 * Layout of the type word in the converged interface.
 *   31..24  SI_IO_PROTO_MAGIC, which also encodes the version
 *   23..16  per-frame flags, private to the libraries
 *   15..0   one of the SI_IO_TYPE_* values above
 */
#define SI_IO_PROTO_MAGIC	0x51	/* 'Q', version 1 */
#define SI_IO_PROTO_MAGIC_SHIFT	24
#define SI_IO_PROTO_FLAG_SHIFT	16
#define SI_IO_PROTO_TYPE_MASK	0xffff

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
} __attribute__((packed));

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
} __attribute__((aligned(4))) __attribute__((packed));

struct si_io_chunk {
	__u32 type;
	__u32 resp;
	__s32 err;
	__u32 seq_id;
	__u32 cur_off_l;
	__u32 cur_off_h;
	__u32 tot_len_l;
	__u32 tot_len_h;
} __attribute__((aligned(4))) __attribute__((packed));

struct si_qcmd_param {
	enum si_qtype qtype;
	__u32 qid;
	__u32 qsize;
	__u32 qdepth;
	__u32 bsize;
	__u32 timeout;
} __attribute__((aligned(4))) __attribute__((packed));

struct si_mcmd_timesync_params {
	__u32 time_sec_lo;
	__u32 time_sec_hi;
	__u32 time_nsec_lo;
	__u32 time_nsec_hi;
	__u32 tz_minuteswest;
	__u32 tz_dsttime;
} __attribute__((packed)) __attribute__((aligned(4)));

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
	/*
	 * addr carries a dma-buf fd (one per entry when nents > 1). The driver
	 * attaches to the dma-buf instead of pinning user pages, so it works
	 * with buffers whose CPU mapping is VM_PFNMAP (dma-heaps, udmabuf).
	 */
	SI_BUF_ADDR_DMABUF,
	SI_BUF_ADDR_MAX
};

/*
 * Source of the ring buffers backing the queues. Reported to userspace via
 * SI_GET_BUF_ALLOC_MODE so the library knows whether it must allocate and
 * enroll its own buffers.
 */
enum si_buf_alloc_mode {
	SI_BUF_ALLOC_KERNEL = 0,
	SI_BUF_ALLOC_USER = 1,
};

/*
 * Enroll a userspace-allocated dma-buf (e.g. udmabuf) as one of the ring
 * buffers of a queue. Only valid when the driver is in SI_BUF_ALLOC_USER mode.
 *
 * The slot identified by (qid, idx) is filled with the buffer behind fd. The
 * driver finalises the queue (RQE setup if RX, SoC create command) on enroll
 * of the last buffer.
 */
struct si_buf_enroll {
	__u32 qid;	/* Target queue id */
	__u32 idx;	/* Buffer slot index, in [0, ctx->entries) */
	__u32 btype;	/* One of enum si_buffer_type */
	__s32 fd;	/* Userspace dma-buf / udmabuf file descriptor */
	__u32 size;	/* Buffer size in bytes, must match queue bsize */
} __attribute__((aligned(4))) __attribute__((packed));

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
	/*
	 * When nents == 1, size and addr hold the buffer size and address.
	 * When nents > 1, each holds a userspace pointer to an array of
	 * nents __u64 sizes/addresses respectively.
	 */
	__u64 size;
	__u64 addr;
	__u32 qidx;
} __attribute__((aligned(4))) __attribute__((packed));

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
	__s64 size;
	__u32 mode;
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

