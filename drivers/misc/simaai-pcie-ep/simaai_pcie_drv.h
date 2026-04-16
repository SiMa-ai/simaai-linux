// SPDX-License-Identifier: GPL-2.0
/**
 * Host PCIe Host and  EP driver for SiMa.ai SoCs
 *
 * Copyright (C) 2021-2025 SiMa.ai
 * Author: PCIe SW Team
 */

#ifndef _SIMA_DRV_COMMON_H_
#define _SIMA_DRV_COMMON_H_

#include <linux/simaai_pcie_api.h>
#include "simaai_pcie_net.h"

#define STOP_BUILD_IF_NOT(expr) extern const char failed[(!!(expr) ? 1 : (-1))]

#define SI_MCMD_SETUP_SOC 0x7f01

#define SI_MCMD_CREATE_MWQS 0x7f02
#define SI_MCMD_CREATE_DWQS 0x7f03
#define SI_MCMD_CREATE_MRQS 0x7f04
#define SI_MCMD_CREATE_DRQS 0x7f05
#define SI_MCMD_CREATE_MCQS 0x7f06
#define SI_MCMD_CREATE_DCQS 0x7f07
#define SI_MCMD_DELETE_MWQ 0x7f08
#define SI_MCMD_DELETE_DWQ 0x7f09
#define SI_MCMD_DELETE_MRQ 0x7f0a
#define SI_MCMD_DELETE_DRQ 0x7f0b
#define SI_MCMD_DELETE_MCQ 0x7f0c
#define SI_MCMD_DELETE_DCQ 0x7f0d

#define SI_MCMD_ABORT_REQ	0x7f0e
#define SI_MCMD_FLUSH_REQS	0x7f0f

#define SI_MCMD_REBOOT_SOC	0x7f11
#define SI_MCMD_SET_TIME	0x7f12
#define SI_MCMD_SET_CARD_NUM	0x7f13
#define SI_MCMD_SET_NETHWADDR	0x7f14
#define SI_MCMD_SET_NETIPADDR	0x7f15

#define SI_MCQ_MSI		0
#define SI_DCQ_MSI_START	1
#define SI_SMCMD_DONE_MSI	(SI_DCQ_MSI_START + SI_MAX_DQS)
#define SI_ALERT_MSI		(SI_SMCMD_DONE_MSI + 1)
#define SI_HOST_MSI_MAX		(SI_ALERT_MSI + 1)

/*
 * Queue status definitions
 */
enum si_qstatus {
	SI_INVALID,	/* Queue is not created */
	SI_ACTIVE,	/* Queue is created and ready to use */
	SI_FROZEN,	/* Queue is temporarily unavailable */
	SI_FLUSHING,	/* Queue entries are being flushed */
	SI_QSTATUS_COUNT_MAX
} __packed;

/*
 * Queue Entry ID: This will be at the top of every queue entry.
 *
 * idx: Index of this entry in the queue
 * qid: ID of the queue this entry is part of
 * qtype: One of enum si_qtype
 * inuse: Whether the entry is currently in use
 */
struct si_qe_id {
	u32 qtype:3;
	u32 qid:5;
	u32 idx:10;
	u32 inuse:1;
	u32 rsvd:13;
} __packed __aligned(4);

enum si_sge_type {
	SGE_TYPE_NONE,		/* There are no SGEs. No data */
	SGE_TYPE_SINGLE,	/* There is one SGE. This one */
	SGE_TYPE_LIST,		/* This one points to a list of SGEs */
	SGE_TYPE_MAX
} __packed;

enum si_sge_flags {
	SGE_FLAGS_NONE,		/* Nothing special about this */
	SGE_FLAGS_FIRSTSGE,	/* First one in a list */
	SGE_FLAGS_LASTSGE,	/* This is the last one in a list */
	SGE_FLAGS_MAX
} __packed;

/*
 * This represents how the data is in memory.
 *
 * When the type says SGE_TYPE_LIST,
 * the address would point to a list of struct si_sge s with size indicating
 * the number of entries in the list. The total size of list would be (size *
 * sizeof(struct si_sge)). Each entry in the list will have an index. The last
 * entry in the list would have SGE_FLAG_LASTSGE set in flags. Others would
 * have SGE_FLAGS_NONE.
 *
 * When the type says SGE_TYPE_SINGLE, the address would point to data. And the
 * size would indicate the size of data in bytes. Index is not used in this case
 * and flags are optional. Recommended value is SGE_FLAGS_NONE.
 *
 * If the type is SGE_TYPE_NONE, it would mean there is no data. All other
 * fields are invalid in this case.
 */
struct si_sge {
	u32 type:2;	/* One of enum si_sge_type */
	u32 flags:2;	/* One of enum si_sge_flags */
	u32 idx:8;	/* Index of this SGE in a list */
	u32 bidx:20;	/* Index of the buffer this belongs to */
	u32 size;	/* Size of data in this SGE. Or list size */
	u32 addr_l;	/* Upper 32 bits of the data/list address */
	u32 addr_h;	/* Lower 32 bits of the data/list address */
} __packed __aligned(4);

/*
 * All requests are sent from host to SoC via a wqe. This represents how it
 * would look like
 */
struct si_wqe {
	struct si_qe_id qidx;	/* Identification of this queue entry */
	struct si_sge sge;	/* An SGE or a list of SGEs pointing to data */
	u32 seqid;		/* Driver sequence ID. Not passed to apps */
	u32 data_size;		/* Total size of data associated with req */
	u32 cmd;		/* Command, indicates what to do with data */
	u32 params[8];		/* Any additional parametes for the command */
} __packed __aligned(4);

/*
 * All data that is sent to the host will have an rqe associated.
 */
struct si_rqe {
	struct si_qe_id qidx;	/* Identification of this queue entry */
	struct si_sge sge;	/* An SGE or a list of SGEs pointing to data */
	u32 data_size;		/* Total size of data  */
	u32 rsvd[10];		/* Padding */
} __packed __aligned(4);

/*
 * The completion of all commands is notified to the host via a cqe
 */
struct si_cqe {
	struct si_qe_id qidx;	/* Identification of this queue entry */
	struct si_qe_id rqidx;	/* Id for associated receive queue entry */
	u32 seqid;		/* Driver sequence ID. Not passed to apps */
	u32 cmd;		/* The command to which we are responding to */
	u32 resp;		/* Status of the command */
	u32 params[11];		/* Any additional parameters to the response */
} __packed __aligned(4);

STOP_BUILD_IF_NOT(sizeof(struct si_wqe) == 64);
STOP_BUILD_IF_NOT(sizeof(struct si_rqe) == 64);
STOP_BUILD_IF_NOT(sizeof(struct si_cqe) == 64);

/*
 * Definition or context for a queue
 */
struct si_qctx {
	u8 qid;				/* Queue ID */
	enum si_qtype qtype;		/* Type of the queue */
	u32 entries;			/* Number of entries in the queue */
	u32 qdepth;			/* Max number of outstanding reqs */
	enum si_qstatus status;		/* Queue status */
	ktime_t *enq_times;		/* Times when the entries are sent */
	u32 bufsize;			/* Buffer size for each entries */
	size_t size;
	dma_addr_t ctx_paddr;		/* DMA handle for this context */
	dma_addr_t qe_pb;		/* DMA handle for queue entries base */
	union {
		struct si_wqe *wqe;	/* List of work queue entries */
		struct si_cqe *cqe;	/* List of completion queue entries */
		struct si_rqe *rqe;	/* List of receive queue entries */
		void *qe_vb;		/* Virtual address of queue entry base */
	};
	atomic_t head;			/* Current head of the queue */
	atomic_t tail;			/* Current tail of the queue */
} __aligned(4);

/*
 * Various states the SoC Can be
 */
enum si_soc_state {
	SOC_STATE_INVALID = 0,	/* Not initialized */
	SOC_STATE_EINIT,	/* Troot is up */
	SOC_STATE_INIT,		/* Linux driver is up */
	SOC_STATE_RUN,		/* Soc is configured and ready */
	SOC_STATE_MAX
};

enum si_bar_val_types {
	wq_head,
	cq_tail,
	rq_tail,
	wq_tail,
	cq_head,
	rq_head
};

enum si_net_val_types {
        rx_head,
        rx_tail,
        tx_head,
        tx_tail
};

/*
 * Structure of management commands that are used to configure the SoC
 */
struct si_mcmd {
	u32 cmd;		/* Management command */
	u32 seq_id;		/* Command sequence ID */
	struct si_sge sge;	/* If there is data */
	u32 param_size;		/* Size of parameters */
	u8 params[128 - 28];	/* Any additional parameters for command */
} __aligned(4);
STOP_BUILD_IF_NOT(sizeof(struct si_mcmd) == 128);

struct si_mcmd_res {
	u32 res;		/* Result of the command */
	s32 errno;		/* Reason for failure */
	u32 cmd;		/* Original command */
	u32 seq_id;		/* Sequence ID of command */
	struct si_sge sge;	/* If there is data */
	u32 param_size;		/* Size of parameters */
	u8 params[64 - 36];	/* Additional parameters for response */
} __aligned(4);
STOP_BUILD_IF_NOT(sizeof(struct si_mcmd_res) == 64);

enum si_alert_types {
	SI_ALERT_REBOOT_SOC,
	SI_ALERT_LINK_DOWN,
	SI_ALERT_KERNEL_CRASH,
	SI_ALERT_MAX
};

struct si_alert {
	u8 type;
	u8 sub_type;
	u8 data[126];
} __aligned(4);

/*
 * Internal driver commands, not exposed to applications. Those which are 
 * exposed to applications should be defined in sima_pcie_api.h
 */
struct si_mcmd_set_nethw_addr_params {
	u32 nethw_addr_low;
	u32 nethw_addr_high;
} __aligned(4);

struct si_mcmd_set_ipaddr_params {
	__be32 net_addr;
	__be32 net_mask;
} __aligned(4);

struct si_setup_soc_params {
	u32 alert_addr_low;
	u32 alert_addr_high;
} __aligned(4);

#endif
