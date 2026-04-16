// SPDX-License-Identifier: GPL-2.0
/**
 * Platform PCIe EP driver for SiMa.ai Davinci SoCs
 *
 * Copyright (C) 2021-2022 SiMa.ai
 * Author:
 */

#ifndef _SIMA_PEP_DRV_H_
#define _SIMA_PEP_DRV_H_

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/compiler_types.h>
#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/irqreturn.h>
#include <linux/kmsg_dump.h>

#include <linux/simaai_pcie_api.h>
#include "simaai_pcie_drv.h"
#include "simaai_pcie_net.h"
#include "simaai_pep_net_drv.h"

#define MAX_TIMESTAMPS 10

#define MAX_DMA_CHANNELS 16

extern int dbg_level;
#define si_err_common(dev, fmt, args...)				\
	do {								\
		dev_err((dev), "CPU%d:%s: " fmt "\n",			\
			 smp_processor_id(), __func__, ##args);		\
	} while (0)

#define si_err(pep, fmt, args...) si_err_common((pep->dev), fmt, ##args)
#define si_qerr(qdev, fmt, args...)					\
	si_err_common((qdev->pep->dev), "%s queue %d, " fmt,		\
		si_pep_qtype_str(qdev->qtype), qdev->qid, ##args)	\

#define si_dbg_common(pep, fmt, args...)				\
	dev_info((pep->dev), "CPU%d:%s: " fmt "\n",			\
		smp_processor_id(), __func__, ##args)

#define si_dbg_cond(bit, pep, fmt, args...)				\
	do {								\
		if ((dbg_level & bit) && printk_ratelimit()) {		\
			si_dbg_common(pep, fmt, ##args);		\
		}							\
	} while (0)

#define si_info(pep, fmt, args...) si_dbg_cond(1U, pep, fmt, ##args)
#define si_minfo(pep, fmt, args...) si_dbg_cond(2U, pep, fmt, ##args)
#define si_dinfo(pep, fmt, args...) si_dbg_cond(4U, pep, fmt, ##args)
#define si_dbg(pep, fmt, args...) si_dbg_cond(8U, pep, fmt, ##args)
#define si_dbg_l(pep, fmt, args...) si_dbg_cond(16U, pep, fmt, ##args)
#define si_dbg_net(pep, fmt, args...) si_dbg_cond(32U, pep, fmt, ##args)
#define si_dbg_dump(pep, fmt, args...) si_dbg_cond(64U, pep, fmt, ##args)
#define si_dbg_dma(pep, fmt, args...) si_dbg_cond(128U, pep, fmt, ##args)

#define si_qdbg_common(bit, qdev, fmt, args...)			\
	si_dbg_cond(bit, (qdev->pep), "%s queue %d, " fmt,		\
		si_pep_qtype_str(qdev->qtype), qdev->qid, ##args)	\

#define si_qinfo(qdev, fmt, args...) si_qdbg_common(1U, qdev, fmt, ##args)
#define si_qminfo(qdev, fmt, args...) si_qdbg_common(2U, qdev,, fmt, ##args)
#define si_qdinfo(qdev, fmt, args...) si_qdbg_common(4U, qdev, fmt, ##args)
#define si_qdbg(qdev, fmt, args...) si_qdbg_common(8U, qdev, fmt, ##args)
#define si_qdbg_l(qdev, fmt, args...) si_qdbg_common(16U, qdev, fmt, ##args)

#define SI_DWQ_CACHE_SIZE 512 /* Per queue cache */
#define SI_MWQ_CACHE_SIZE 128

/**
 * The cache area layout in OCM
 * +=============+============================+================================+
 * |   Used for  |         Size of 1          |       Number of Elements       |
 * +=============+============================+================================+
 * | MCMD        | sizeof(struct si_mcmd)     | 1                              |
 * +-------------+----------------------------+--------------------------------+
 * | MRESP       | sizeof(struct si_mcmd_res) | 1                              |
 * +-------------+----------------------------+--------------------------------+
 * | MWQ Ctx     | sizeof(struct si_qctx)     | 1                              |
 * +-------------+----------------------------+--------------------------------+
 * | MCQ Ctx     | sizeof(struct si_qctx)     | 1                              |
 * +-------------+----------------------------+--------------------------------+
 * | MRQ Ctx     | sizeof(struct si_qctx)     | 1                              |
 * +-------------+----------------------------+--------------------------------+
 * | DWQ Ctx     | sizeof(struct si_qctx)     | SI_MAX_DQS                     |
 * +-------------+----------------------------+--------------------------------+
 * | DCQ Ctx     | sizeof(struct si_qctx)     | SI_MAX_DQS                     |
 * +-------------+----------------------------+--------------------------------+
 * | DRQ Ctx     | sizeof(struct si_qctx)     | SI_MAX_DQS                     |
 * +-------------+----------------------------+--------------------------------+
 * | Temp MCQ    | sizeof(struct si_cqe)      | 1                              |
 * +-------------+----------------------------+--------------------------------+
 * | Temp DCQ    | sizeof(struct si_cqe)      | SI_MAX_DQS                     |
 * +-------------+----------------------------+--------------------------------+
 * | MREQ Cache  | sizeof(struct si_wqe)      | SI_MWQ_CACHE_SIZE              |
 * +-------------+----------------------------+--------------------------------+
 * | DREQ Cache  | sizeof(struct si_wqe)      | SI_DWQ_CACHE_SIZE * SI_MAX_DQS |
 * +-------------+----------------------------+--------------------------------+
 * | Alerts      | sizeof(struct si_alert)    | 1                              |
 * +-------------+----------------------------+--------------------------------+
 */
struct si_pep_cache {
	struct si_mcmd mcmd;
	struct si_mcmd_res mcmd_res;
	struct si_qctx mwq_ctx;
	struct si_qctx mcq_ctx;
	struct si_qctx mrq_ctx;
	struct si_qctx dwq_ctx[SI_MAX_DQS];
	struct si_qctx dcq_ctx[SI_MAX_DQS];
	struct si_qctx drq_ctx[SI_MAX_DQS];
	struct si_cqe mcqe_tmp;
	struct si_cqe dcqe_tmps[SI_MAX_DQS];
	struct si_wqe mreq_cache[SI_MWQ_CACHE_SIZE];
	struct si_wqe dreq_caches[SI_MAX_DQS][SI_DWQ_CACHE_SIZE];
	struct si_alert alert;
} __aligned(4);
/* This needs to fit in 1MB OCM */
STOP_BUILD_IF_NOT(sizeof(struct si_pep_cache) <= (1 * 1024 * 1024));

#define DREQ_CACHE_SIZE	(sizeof(struct si_wqe) * SI_DWQ_CACHE_SIZE)
#define MREQ_CACHE_SIZE	(sizeof(struct si_wqe) * SI_MWQ_CACHE_SIZE)

#define si_pep_cache_phys(pep, elem) \
	(phys_addr_t)(&(((struct si_pep_cache *)(pep->cache_pbase))->elem))
#define si_pep_cache_virt(pep, elem) &((pep->cache_base)->elem)
/*
 * Hardware endpoint structures
 */
struct si_pep_of_data {
	int data;
	char *name;
};

struct si_pep_irq_handlers {
	char *name;
	irqreturn_t (*handler)(int irq, void *arg);
};

struct si_pep_req_cache_hdr {
	u16 nreqs;
	u16 maxreqs;
	u16 head;
	u16 tail;
} __packed;

/**
 * struct si_pep_timestamps - Timestamp structure
 * @start: Start timestamp.
 * @stop: Stop Timestamp.
 * @min_time: Minimun time differnce bettwen entry timepoint and exit timepoint.
 * @max_time: Maximun time differnce bettwen entry timepoint and exit timepoint.
 * @ts_index: Timestamp array index
 */
struct si_pep_timestamps {
	ktime_t start[MAX_TIMESTAMPS];
	ktime_t stop[MAX_TIMESTAMPS];
	ktime_t min_time;
	ktime_t max_time;
	u32 ts_index;
};

struct si_pep_stats {
	/* bytes txed from SoC */
	uint64_t tx_bytes;
	/* bytes rxed from host */
	uint64_t rx_bytes;
	/* Number of reqs recevied from Host */
	uint32_t rx_reqs;
	/* Number of resps txed to Host */
	uint32_t tx_resps;
	/* Number of reqs that were aborted successfully */
	uint32_t abort_reqs_suc;
	/* Number of reqs that was consumed by application before aborting */
	uint32_t abort_reqs_fail;
	/* Time stamp of last last resp sent to Host */
	ktime_t last_resp;
};

struct si_pep_thread_param {
	struct si_pep_dev *pep;
	enum si_qtype qtype;
	u32 qid;
};

struct si_pep_edma_irq_param {
	int ch;
	int dir;
};

struct si_pep_db_int_desc {
	u32 index;
	u32 offset;
};

struct si_pep_userbuf_map {
	struct page **pages;
	unsigned int nr_pages;
	struct sg_table sgt;
	unsigned int nr_sgents;
	dma_addr_t *dma_addrs;
	size_t total_size;
};

struct si_pep_userbuf_ctx {
	struct si_pep_userbuf_map *userbuf_maps;
	unsigned int nr_bufs;
};

struct si_pep_dmabuf {
	dma_addr_t addr;
	size_t size;
};

struct si_pep_drv_ops {
	void (*mq_set_val)(struct si_pep_dev*, u16, enum si_bar_val_types);
	u16 (*mq_get_val)(struct si_pep_dev*, enum si_bar_val_types);
	void (*dq_set_val)(struct si_pep_dev*, u32, u16, enum si_bar_val_types);
	u16 (*dq_get_val)(struct si_pep_dev*, u32, enum si_bar_val_types);
	int (*check_mcmd)(struct si_pep_dev*, struct si_mcmd*);
	int (*resp_mcmd)(struct si_pep_dev*, struct si_mcmd_res*);
	void (*set_mlsoc_caps)(struct si_pep_dev*, u32, u32);
	int (*queue_created_callback)(struct si_pep_dev*, enum si_qtype, int);
	int (*queue_deleted_callback)(struct si_pep_dev*, enum si_qtype, int);
	int (*platform_init)(struct si_pep_dev*);
	void (*platform_deinit)(struct si_pep_dev*);
	int (*local_init)(struct si_pep_dev*);
	void (*local_deinit)(struct si_pep_dev*);
	void (*set_soc_state)(struct si_pep_dev*, enum si_soc_state);
	/* Net device operations */
	void (*net_release)(struct si_pep_dev*);
	int (*net_open)(struct si_pep_dev*);
	void (*net_set_val)(struct si_pep_dev *, enum si_net_val_types, u16);
	u16 (*net_get_val)(struct si_pep_dev *, enum si_net_val_types);
};

struct si_pep_qdev {
	u32 qid;
	enum si_qtype qtype;
	char devname[32];
	struct mutex ioctl_mutex;
	struct si_pep_dev *pep;
	atomic_t seqid;
	/* Timestamps structures per queue */
	struct si_pep_timestamps cache_dreq_ts;
	struct si_pep_timestamps read_ts;
	struct si_pep_timestamps write_ts;
	/* Statistics per queue*/
	struct si_pep_stats stats;
	/* Timepoints per queue */
	ktime_t *timepoint_1;
	ktime_t *timepoint_2;
	ktime_t *timepoint_3;
	ktime_t *timepoint_4;
};

struct si_pep_dev {
	dev_t devt;
	struct platform_device *pdev;
	struct cdev cdev;
	struct class *class;
	struct device *dev;
	struct fasync_struct *fasync;
	struct task_struct *loopback_hdlr[SI_MAX_DQS];
	struct kmsg_dumper kdump;
	struct notifier_block reboot_handler;
	struct delayed_work link_down_handler;
	struct kobject *kobj_stats;

	const struct si_pep_drv_ops *drv_ops;
	struct si_pep_net_dev net_dev;
	struct si_pep_qdev qdev[SI_MAX_DQS + 1];

	struct si_qctx *mwq_ctx;
	struct si_qctx *mcq_ctx;
	struct si_qctx *mrq_ctx;
	struct si_qctx *dwq_ctx[SI_MAX_DQS];
	struct si_qctx *dcq_ctx[SI_MAX_DQS];
	struct si_qctx *drq_ctx[SI_MAX_DQS];
	phys_addr_t p_mwq_ctx;
	phys_addr_t p_mcq_ctx;
	phys_addr_t p_mrq_ctx;
	phys_addr_t p_dwq_ctx[SI_MAX_DQS];
	phys_addr_t p_dcq_ctx[SI_MAX_DQS];
	phys_addr_t p_drq_ctx[SI_MAX_DQS];

	struct si_pep_req_cache_hdr dchdr[SI_MAX_DQS];
	struct si_pep_req_cache_hdr mchdr;
	struct si_wqe *dreq_caches[SI_MAX_DQS];
	phys_addr_t p_dreq_caches[SI_MAX_DQS];
	struct si_wqe *mreq_cache;
	phys_addr_t p_mreq_cache;
	struct mutex dreq_cache_mutex[SI_MAX_DQS];
	struct mutex mreq_cache_mutex;
	u32 dreq_cached[SI_MAX_DQS];
	u32 mreq_cached;
	wait_queue_head_t dreq_wait[SI_MAX_DQS];
	wait_queue_head_t mreq_wait;

	struct si_rqe *drqe_caches[SI_MAX_DQS];
	dma_addr_t p_drqe_caches[SI_MAX_DQS];
	size_t s_drqe_caches[SI_MAX_DQS];
	struct si_rqe *mrqe_cache;
	dma_addr_t p_mrqe_cache;
	size_t s_mrqe_cache;

	spinlock_t dma_write_lock[MAX_DMA_CHANNELS];
	spinlock_t dma_read_lock[MAX_DMA_CHANNELS];

	atomic_t dwq_prev_heads[SI_MAX_DQS];
	atomic_t mwq_prev_head;

	u32 msg_addr_lower;
	u32 msg_addr_upper;
	u64 host_alert_addr;

	u8 __iomem *dbi_base;	/* PCIe DBI register base */
	size_t dbi_size;
	u8 __iomem *dbi2_base;	/* PCIe DBI2 register base */
	size_t dbi2_size;
	u8 __iomem *glue_logic_base;	/* PCIe Glue register base */
	size_t glue_logic_size;
	u8 __iomem *ocm_base;		/* SoC Reserved OCM Base address */
	phys_addr_t ocm_phys_base;	/* SoC Reserved OCM Physical address */
	size_t ocm_size;
	u8 __iomem *asr_base;
	phys_addr_t asr_phys_base;
	size_t asr_size;
	u8 __iomem *hdma_base;
	struct si_pep_cache __iomem *cache_base;
	phys_addr_t cache_pbase;

	int card_num;
	bool modalix;
	enum si_soc_state state;
};

#define qdev_to_pep_dev(x) (x->pep)

#define kmsg_dumper_to_pep_dev(x) container_of((x), struct si_pep_dev, kdump)

#define link_down_work_to_pep_dev(x) \
	container_of((x), struct si_pep_dev, link_down_handler.work)

#define reboot_notifier_to_pep_dev(x) \
	container_of((x), struct si_pep_dev, reboot_handler)

#define irqwork_to_pep_dev(x) \
	container_of((x), struct si_pep_dev, irq_handler.work)

#define reqwork_to_pep_dev(x) \
	container_of((x), struct si_pep_dev, req_handler.work)

#define queue_is_bad(x) ((u8)(x) >= (SI_MAX_DQS + 1))
#define queue_is_data(qdev) \
			((((struct si_pep_qdev *)qdev)->qtype >= SI_DWQ) && \
	 		(((struct si_pep_qdev *)qdev)->qtype <= SI_DRQ))

#define MCMD_DMA_CH		0
#define MQ_DMA_CH		1
#define DQ_DMA_CH_START		2
#define DQ_DMA_CH_END		((DQ_DMA_CH_START + SI_MAX_DQS) - 1)
#define ETH_DATA_DMA_CH		(DQ_DMA_CH_END + 1)
#define ETH_DESC_DMA_CH		(ETH_DATA_DMA_CH + 1)
#define ALERT_DMA_CH		(ETH_DESC_DMA_CH + 1)
#define MWQ_CACHE_DMA_CH	(ALERT_DMA_CH + 1)
#define DWQ_CACHE_DMA_CH	(MWQ_CACHE_DMA_CH + 1)

#define DQ_DMA_CH(q) (DQ_DMA_CH_START + q)
#define Q_DMA_CH(x) ((queue_is_data(x))?\
			DQ_DMA_CH((((struct si_pep_qdev *)x)->qid)):MQ_DMA_CH)

#define hexdump(prefix, buf, len)                                             \
	print_hex_dump(KERN_INFO, prefix, DUMP_PREFIX_OFFSET, 4, 4, buf, len, \
		       false)

#define mq_set_wq_tail(pep, val) pep->drv_ops->mq_set_val(pep, val, wq_tail)
#define mq_set_cq_head(pep, val) pep->drv_ops->mq_set_val(pep, val, cq_head)
#define mq_set_rq_head(pep, val) pep->drv_ops->mq_set_val(pep, val, rq_head)

#define dq_set_wq_tail(pep, qid, val) \
	pep->drv_ops->dq_set_val(pep, qid, val, wq_tail)
#define dq_set_cq_head(pep, qid, val) \
	pep->drv_ops->dq_set_val(pep, qid, val, cq_head)
#define dq_set_rq_head(pep, qid, val) \
	pep->drv_ops->dq_set_val(pep, qid, val, rq_head)

#define mq_get_wq_head(pep) pep->drv_ops->mq_get_val(pep, wq_head)
#define mq_get_cq_head(pep) pep->drv_ops->mq_get_val(pep, cq_head)
#define mq_get_rq_head(pep) pep->drv_ops->mq_get_val(pep, rq_head)
#define mq_get_wq_tail(pep) pep->drv_ops->mq_get_val(pep, wq_tail)
#define mq_get_cq_tail(pep) pep->drv_ops->mq_get_val(pep, cq_tail)
#define mq_get_rq_tail(pep) pep->drv_ops->mq_get_val(pep, rq_tail)

#define dq_get_wq_head(pep, qid) pep->drv_ops->dq_get_val(pep, qid, wq_head)
#define dq_get_cq_head(pep, qid) pep->drv_ops->dq_get_val(pep, qid, cq_head)
#define dq_get_rq_head(pep, qid) pep->drv_ops->dq_get_val(pep, qid, rq_head)
#define dq_get_wq_tail(pep, qid) pep->drv_ops->dq_get_val(pep, qid, wq_tail)
#define dq_get_cq_tail(pep, qid) pep->drv_ops->dq_get_val(pep, qid, cq_tail)
#define dq_get_rq_tail(pep, qid) pep->drv_ops->dq_get_val(pep, qid, rq_tail)

#define net_get_rxtail(pep) pep->drv_ops->net_get_val(pep, rx_tail)
#define net_get_txtail(pep) pep->drv_ops->net_get_val(pep, tx_tail)
#define net_get_rxhead(pep) pep->drv_ops->net_get_val(pep, rx_head)
#define net_get_txhead(pep) pep->drv_ops->net_get_val(pep, tx_head)

#define net_set_txhead(pep, val) pep->drv_ops->net_set_val(pep, tx_head, val)
#define net_set_rxtail(pep, val) pep->drv_ops->net_set_val(pep, rx_tail, val)

/* Functions that are provided by the common driver */
int si_pep_copy_from_host(struct si_pep_dev *, u64, u64, u32, int);
int si_pep_copy_to_host(struct si_pep_dev *, u64, u64, u32, int, int);
void si_pep_handle_mcmd(struct si_pep_dev *, struct si_mcmd *);
void si_pep_get_reqs_from_host(struct si_pep_dev *, enum si_qtype, int);
void si_pep_recycle_qes(struct si_pep_dev *, enum si_qtype, int);
void si_pep_iomem_read_rep(void *, void __iomem *, size_t);
void si_pep_iomem_write_rep(void __iomem *, void *, size_t);
void si_pep_iomem_memset(void __iomem *, u8 val, size_t);
int si_pep_register_irq_handler(struct si_pep_dev*, char*,
		irqreturn_t (*)(int, void*), void*);
irqreturn_t si_pep_dma_irq_handler(int, void*);
irqreturn_t si_pep_irq_link_down(int, void*);
inline bool si_pep_is_data_queue(u32);
inline bool si_pep_is_mgmt_queue(u32);
char *si_pep_qtype_str(enum si_qtype);
char *si_pep_qstatus_string(enum si_qstatus);

extern struct si_pep_edma_irq_param w_irq_param[];
extern struct si_pep_edma_irq_param r_irq_param[];

extern struct si_pep_dev *g_pep_dev;

#endif /* _SIMA_PEP_DRV_H_ */
