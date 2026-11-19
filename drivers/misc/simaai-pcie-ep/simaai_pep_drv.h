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

#include <uapi/linux/simaai/simaai_pcie_api.h>
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
/* For what a process can make happen as often as it likes */
#define si_err_ratelimited(pep, fmt, args...)				\
	do {								\
		static DEFINE_RATELIMIT_STATE(_rs,			\
				DEFAULT_RATELIMIT_INTERVAL,		\
				DEFAULT_RATELIMIT_BURST);		\
									\
		if (__ratelimit(&_rs))					\
			si_err(pep, fmt, ##args);			\
	} while (0)
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
	/* dma-buf attachment cache lookups that reused an existing mapping */
	uint32_t dbuf_cache_hits;
	/* dma-buf attachment cache lookups that had to attach and map */
	uint32_t dbuf_cache_misses;
	/* Time stamp of last last resp sent to Host */
	ktime_t last_resp;
};

struct si_pep_msi_config {
	u32 msg_addr_l;
	u32 msg_addr_h;
	u32 msg_nvecs;
	u16 msg_data;
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

struct dma_buf;
struct dma_buf_attachment;

/*
 * Number of dma-buf attachments a queue keeps mapped. Producers recycle a
 * small pool of buffers (v4l2 typically 4-8, GStreamer 4-16), so this covers
 * realistic reuse without holding on to attachments that will never be seen
 * again. Validate against a real pipeline with the hit/miss counters in
 * struct si_pep_stats before changing it.
 */
#define SI_PEP_DBUF_CACHE_SIZE 16

/**
 * struct si_pep_dbuf_cache_ent - one cached dma-buf attachment
 * @dbuf: imported buffer; the cache holds a reference for as long as the
 *	  entry is populated. Doubles as the "slot occupied" flag.
 * @attach: attachment of @dbuf to this device.
 * @sgt: mapping owned by the exporter, valid until the entry is released.
 * @dir: direction @sgt was mapped with. Part of the lookup key: a mapping made
 *	 for DMA_TO_DEVICE cannot serve a DMA_FROM_DEVICE transfer.
 * @inuse: number of transfers currently using @sgt. An entry is only evictable
 *	   at zero, otherwise eviction would unmap under an in-flight DMA.
 * @stamp: value of si_pep_qdev.dbuf_cache_clock when the entry was last handed
 *	   out; the lowest stamp among idle entries is evicted first.
 */
struct si_pep_dbuf_cache_ent {
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	enum dma_data_direction dir;
	unsigned int inuse;
	u64 stamp;
};

struct si_pep_userbuf_map {
	struct page **pages;
	unsigned int nr_pages;
	struct sg_table sgt;
	unsigned int nr_sgents;
	dma_addr_t *dma_addrs;
	size_t total_size;
	/*
	 * Set only for SI_BUF_ADDR_DMABUF entries: the entry is backed by an
	 * imported dma-buf and dbuf_sgt (owned by the exporter) is used in
	 * place of pinned pages and sgt.
	 */
	struct dma_buf *dbuf;
	struct dma_buf_attachment *dbuf_attach;
	struct sg_table *dbuf_sgt;
	/*
	 * When set, the three fields above are borrowed from the queue's
	 * attachment cache and this transfer only holds a use count on the
	 * entry: unmapping must release that count, not tear the mapping down.
	 * NULL means the mapping is private to this transfer (either a pinned
	 * user pointer, or a dma-buf imported while every cache entry was
	 * busy) and is unmapped when the transfer completes.
	 */
	struct si_pep_dbuf_cache_ent *dbuf_ent;
	/*
	 * Direction this buffer was dma-mapped with. The DMA API requires
	 * unmap to use the same direction as map, so it is recorded here
	 * rather than assumed.
	 */
	enum dma_data_direction dir;
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
	/*
	 * Arm and disarm the host doorbells. Optional; a SoC with no
	 * doorbell to mask leaves both NULL.
	 *
	 * platform_init() installs the handlers but leaves every doorbell
	 * masked, because the handler needs the workqueue local_init()
	 * allocates and the queue contexts probe sets up after that.
	 * Nothing is armed until probe has finished, and everything is
	 * disarmed again before local_deinit() takes the workqueue away.
	 */
	void (*enable_doorbells)(struct si_pep_dev*);
	void (*disable_doorbells)(struct si_pep_dev*);
	void (*set_soc_state)(struct si_pep_dev*, enum si_soc_state);
	/* Net device operations */
	void (*net_release)(struct si_pep_dev*);
	int (*net_open)(struct si_pep_dev*);
	void (*net_set_val)(struct si_pep_dev *, enum si_net_val_types, u16);
	u16 (*net_get_val)(struct si_pep_dev *, enum si_net_val_types);
	void (*net_enable_rxint)(struct si_pep_dev *);
	void (*net_disable_rxint)(struct si_pep_dev *);
};

struct si_pep_qdev {
	u32 qid;
	enum si_qtype qtype;
	char devname[32];
	struct mutex ioctl_mutex;
	struct si_pep_dev *pep;
	/*
	 * Cached DMA linked-list buffer. dma_alloc_coherent() is far too
	 * expensive to do per transfer, so the buffer is kept and reused,
	 * growing on demand. ll_busy serialises access without introducing a
	 * lock on the submission path: a concurrent transfer on the same
	 * queue simply falls back to its own allocation.
	 */
	void *ll_cache;
	dma_addr_t ll_cache_dma;
	size_t ll_cache_size;
	unsigned long ll_busy;
	/*
	 * Cached scratch buffer for the remote SGE list copied in from the
	 * host by si_pep_dma_buflist_from_sge(). Same rationale as ll_cache:
	 * dma_alloc_coherent() per transfer is too expensive to repeat for
	 * every buffer, so the queue keeps one buffer and reuses it, growing
	 * on demand. sge_busy claims it without a lock on the submission
	 * path; a concurrent transfer on the same queue falls back to its own
	 * allocation.
	 */
	void *sge_cache;
	dma_addr_t sge_cache_dma;
	size_t sge_cache_size;
	unsigned long sge_busy;
	/*
	 * Cached dma-buf attachments. Importing a dma-buf costs an attach and a
	 * dma-map on every transfer, which dominates the transfer itself for
	 * the buffer pools userspace actually recycles, so the mappings are
	 * kept here and reused. The cache lives on the queue because an
	 * application stays on one queue, and its idle entries are flushed on
	 * every close of the queue device -- opens are not refcounted, so a
	 * second holder of the same queue loses its warm entries when the
	 * first one closes.
	 */
	struct mutex dbuf_cache_lock;
	struct si_pep_dbuf_cache_ent dbuf_cache[SI_PEP_DBUF_CACHE_SIZE];
	u64 dbuf_cache_clock;
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

	/*
	 * Serialise completion and receive slot allocation.
	 *
	 * The queue contexts these protect live in the OCM window, which is
	 * ioremapped and so is Device memory. Atomic read-modify-write
	 * instructions are not supported there on arm64 -- a cmpxchg on
	 * ctx->head raises a synchronous external abort -- so the allocation
	 * cannot be made atomic in place and has to be serialised by a lock
	 * held in ordinary memory instead. The accesses inside stay plain
	 * loads and stores.
	 *
	 * Index 0 is the management queue and 1..SI_MAX_DQS are data queue
	 * qid + 1, matching the qdev[] convention.
	 */
	spinlock_t cqe_alloc_lock[SI_MAX_DQS + 1];
	spinlock_t rqe_alloc_lock[SI_MAX_DQS + 1];
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
	u8 __iomem *pciesys_base;
	phys_addr_t pciesys_phys_base;
	size_t pciesys_size;

	int card_num;
	/* Completion interrupts of the veth DMA channel, if any */
	int net_tx_irq;
	int net_rx_irq;
	/* The veth is up, so its doorbell may schedule its poll */
	bool net_rx_up;
	bool modalix;
	enum si_soc_state state;

	bool recovery_mode;
	struct task_struct *recovery_thread;

	/*
	 * Serialises the link-down handler, the recovery thread's rebuild
	 * and si_pep_remove() against one another, and guards 'initialized':
	 * true from the moment probe has built everything the link-down
	 * path tears down until remove() starts taking it apart. A link-down
	 * event that arrives outside that window has nothing to clean up
	 * and is ignored.
	 */
	struct mutex state_lock;
	bool initialized;
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
#define net_get_host_rxint(pep) pep->drv_ops->net_get_val(pep, host_rxint)

#define net_set_txhead(pep, val) pep->drv_ops->net_set_val(pep, tx_head, val)
#define net_set_rxtail(pep, val) pep->drv_ops->net_set_val(pep, rx_tail, val)

/* Functions that are provided by the common driver */
int si_pep_copy_from_host(struct si_pep_dev *, u64, u64, u32, int);
int si_pep_copy_to_host(struct si_pep_dev *, u64, u64, u32, int, int);
int si_pep_copy_to_host_quiet(struct si_pep_dev *pep, u64 dest, u64 src,
			      u32 size, int ch, int msi);
struct si_pep_dma_ll_ctx;
int si_pep_copy_do_ll(struct si_pep_dev *, struct si_pep_dma_ll_ctx *, int, int);
void si_pep_dma_ll_program(struct si_pep_dev *pep, int ch, int dir,
			   dma_addr_t ll_p);
bool si_pep_msi_has_vector(struct si_pep_dev *, int);
void si_pep_handle_mcmd(struct si_pep_dev *, struct si_mcmd *);
int si_pep_get_reqs_from_host(struct si_pep_dev *, enum si_qtype, int);
void si_pep_recycle_qes(struct si_pep_dev *, enum si_qtype, int);
void si_pep_iomem_read_rep(void *, void __iomem *, size_t);
void si_pep_iomem_write_rep(void __iomem *, void *, size_t);
void si_pep_iomem_memset(void __iomem *, u8 val, size_t);
int si_pep_register_irq_handler(struct si_pep_dev*, char*,
		irqreturn_t (*)(int, void*), void*);
irqreturn_t si_pep_dma_irq_handler(int, void*);
irqreturn_t si_pep_irq_link_down(int, void*);
int si_pep_irq_link_down_enable(struct si_pep_dev *);
int si_pep_irq_link_down_disable(struct si_pep_dev *pep);
inline bool si_pep_is_data_queue(u32);
inline bool si_pep_is_mgmt_queue(u32);
char *si_pep_qtype_str(enum si_qtype);
char *si_pep_qstatus_string(enum si_qstatus);

extern struct si_pep_edma_irq_param w_irq_param[];
extern struct si_pep_edma_irq_param r_irq_param[];

extern struct si_pep_dev *g_pep_dev;

#endif /* _SIMA_PEP_DRV_H_ */
