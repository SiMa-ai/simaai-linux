// SPDX-License-Identifier: GPL-2.0
/*
 * Platform PCIe EP driver for SiMa.ai SoCs
 *
 * Copyright (C) 2024 SiMa.ai
 *
 */

#include <asm/io.h>
#include <linux/kthread.h>
#include <linux/refcount.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "simaai_pep_net_drv.h"
#include "simaai_pep_drv_davinci.h"
#include "simaai_pcie_davinci.h"

#define SI_DAV_MAX_DQS		6
#define SI_DAV_MAX_EDMA_INTS	1

static spinlock_t dwq_tail_locks[2];
static spinlock_t dcq_head_locks[2];
static spinlock_t drq_head_locks[2];
static spinlock_t net_reg_lock;

static struct task_struct *mcmd_checker;
static struct task_struct *mreq_checker;
static struct task_struct *dreq_checker[SI_DAV_MAX_DQS];
static struct task_struct *mqe_recycler;
static struct task_struct *dqe_recycler;
static struct task_struct *net_rx_poller;
static struct si_pep_thread_param dt_params[SI_DAV_MAX_DQS];
static struct si_pep_thread_param mt_params;
atomic_t dq_ref;
atomic_t mq_ref;

static u8 __iomem *dma_reg_base;
static u64 host_mcmd_addr;

#define si_pep_dma_reg_write(offset, value) \
	writel(value, dma_reg_base + offset)
#define si_pep_dma_reg_read(offset) readl(dma_reg_base + offset)

static u16 si_pep_dav_mq_get_val(struct si_pep_dev *pep,
		enum si_bar_val_types type)
{
	u32 offset;

	switch (type) {
	case wq_head:
		offset = DAV_OFF_MWQ_HEAD;
		break;
	case wq_tail:
		offset = DAV_OFF_MWQ_TAIL;
		break;
	case cq_head:
		offset = DAV_OFF_MCQ_HEAD;
		break;
	case cq_tail:
		offset = DAV_OFF_MCQ_TAIL;
		break;
	case rq_head:
		offset = DAV_OFF_MRQ_HEAD;
		break;
	case rq_tail:
		offset = DAV_OFF_MRQ_TAIL;
		break;
	default:
		return 0;
	}

	return ((si_pep_dma_reg_read(offset) & 0xff));
}

static void si_pep_dav_mq_set_val(struct si_pep_dev *pep, u16 val,
		enum si_bar_val_types type)
{
	u32 offset;

	switch (type) {
	case wq_head:
		offset = DAV_OFF_MWQ_HEAD;
		break;
	case wq_tail:
		offset = DAV_OFF_MWQ_TAIL;
		break;
	case cq_head:
		offset = DAV_OFF_MCQ_HEAD;
		break;
	case cq_tail:
		offset = DAV_OFF_MCQ_TAIL;
		break;
	case rq_head:
		offset = DAV_OFF_MRQ_HEAD;
		break;
	case rq_tail:
		offset = DAV_OFF_MRQ_TAIL;
		break;
	default:
		return;
	}

	si_pep_dma_reg_write(offset, val);
}

/*
 * 6 queues in total. Each queue is allocated 9 bits for head/tail,
 * across two registers. First 3 queues in first register. Others in second
 */
static u16 si_pep_dav_dq_get_val(struct si_pep_dev *pep, u32 qid,
		enum si_bar_val_types type)
{
	u32 offset;
	u32 mask_start, mask_end;

	mask_start = ((qid % 3) * 10) + 9;
	mask_end = mask_start - 9;

	if (qid >= SIMA_DAV_MAX_DQS)
		return 0;

	switch (type) {
	case wq_head:
		offset = (qid < 3)?DAV_OFF_DWQ012_HEAD:DAV_OFF_DWQ345_HEAD;
		break;
	case wq_tail:
		offset = (qid < 3)?DAV_OFF_DWQ012_TAIL:DAV_OFF_DWQ345_TAIL;
		break;
	case cq_head:
		offset = (qid < 3)?DAV_OFF_DCQ012_HEAD:DAV_OFF_DCQ345_HEAD;
		break;
	case cq_tail:
		offset = (qid < 3)?DAV_OFF_DCQ012_TAIL:DAV_OFF_DCQ345_TAIL;
		break;
	case rq_head:
		offset = (qid < 3)?DAV_OFF_DRQ012_HEAD:DAV_OFF_DRQ345_HEAD;
		break;
	case rq_tail:
		offset = (qid < 3)?DAV_OFF_DRQ012_TAIL:DAV_OFF_DRQ345_TAIL;
		break;
	default:
		return 0;
	}

	return ((si_pep_dma_reg_read(offset) & GENMASK(mask_start, mask_end)) >>
			mask_end);
}

static void si_pep_dav_dq_set_val(struct si_pep_dev *pep, u32 qid, u16 val,
		enum si_bar_val_types type)
{
	u32 reg;
	u32 offset;
	u32 mask_start, mask_end;
	spinlock_t *lock;

	if (qid >= SIMA_DAV_MAX_DQS)
		return;

	mask_start = ((qid % 3) * 10) + 9;
	mask_end = mask_start - 9;

	if (type == wq_tail) {
		offset = (qid < 3)?DAV_OFF_DWQ012_TAIL:DAV_OFF_DWQ345_TAIL;
		lock = (qid < 3)?&dwq_tail_locks[0]:&dwq_tail_locks[1];
	} else if (type == cq_head) {
		offset = (qid < 3)?DAV_OFF_DCQ012_HEAD:DAV_OFF_DCQ345_HEAD;
		lock = (qid < 3)?&dcq_head_locks[0]:&dcq_head_locks[1];
	} else if (type == rq_head) {
		offset = (qid < 3)?DAV_OFF_DRQ012_HEAD:DAV_OFF_DRQ345_HEAD;
		lock = (qid < 3)?&drq_head_locks[0]:&drq_head_locks[1];
	} else
		return;

	spin_lock(lock);
	reg = si_pep_dma_reg_read(offset);
	si_pep_dma_reg_write(offset, (reg & ~(GENMASK(mask_start, mask_end))) |
			((val & GENMASK(9, 0)) << mask_end));
	spin_unlock(lock);
}

static void si_pep_dav_set_soc_state(struct si_pep_dev *pep,
		enum si_soc_state state)
{
	pep->state = state;
	si_pep_dma_reg_write(DAV_OFF_SOC_STATUS, state);
}

static int si_pep_dav_check_mcmd(struct si_pep_dev *pep, struct si_mcmd *mcmd)
{
	int ret;
	volatile u32 reg;
	u32 mcmd_addr_low;
	u32 mcmd_addr_high;
	u64 mcmd_addr;
	u64 mcmd_phys_base;
	struct si_mcmd *mcmd_base;

	mcmd_base = si_pep_cache_virt(pep, mcmd);
	mcmd_phys_base = si_pep_cache_phys(pep, mcmd);

	if (mcmd == NULL)
		return -1;
	/*
	 * If the host driver has been reloaded, the mcmd address can
	 * change. It will be safer to get the address each time. Else
	 * the soc driver will also need to be reloaded.
	 */
	mcmd_addr_low = si_pep_dma_reg_read(DAV_OFF_SMCMD_LOW);
	mcmd_addr_high = si_pep_dma_reg_read(DAV_OFF_SMCMD_HIGH);
	mcmd_addr = ((u64)mcmd_addr_high << 32) | mcmd_addr_low;
	if (mcmd_addr == 0) {
		host_mcmd_addr = 0;
		si_pep_dav_set_soc_state(pep, SOC_STATE_INIT);
		return -1;
	}

	if (host_mcmd_addr != mcmd_addr) {
		host_mcmd_addr = mcmd_addr;
		si_pep_dav_set_soc_state(pep, SOC_STATE_RUN);
	}

	reg = si_pep_dma_reg_read(DAV_OFF_SMCMD);
	if (reg) {
		ret = si_pep_copy_from_host(pep, mcmd_phys_base, host_mcmd_addr,
				sizeof(struct si_mcmd), MCMD_DMA_CH);
		if (ret)
			return -1;

		memcpy(mcmd, mcmd_base, sizeof(struct si_mcmd));
		si_pep_iomem_memset(mcmd_base, 0, sizeof(struct si_mcmd));
		return 0;
	}

	return -1;
}

static int si_pep_dav_resp_mcmd(struct si_pep_dev *pep, struct si_mcmd_res *resp)
{
	u64 dst, src;
	struct si_mcmd_res *mcmd_res;

	if (host_mcmd_addr == 0) {
		si_err(pep, "Unable to send mcmd response");
		return -1;
	}

	mcmd_res = si_pep_cache_virt(pep, mcmd_res);
	memcpy(mcmd_res, resp, sizeof(struct si_mcmd_res));

	src = si_pep_cache_phys(pep, mcmd_res);
	dst = host_mcmd_addr + sizeof(struct si_mcmd);
	si_pep_copy_to_host(pep, dst, src, sizeof(struct si_mcmd_res),
			MCMD_DMA_CH, SI_SMCMD_DONE_MSI);
	si_pep_dma_reg_write(DAV_OFF_SMRSP, 1);
	/* Mark mcmd as completed */
	si_pep_dma_reg_write(DAV_OFF_SMCMD, 0);

	return 0;
}

static int si_pep_dav_check_mcmd_t(void *arg)
{
	int ret;
	struct si_mcmd mcmd = { 0 };
	struct si_pep_dev *pep = (struct si_pep_dev *)arg;

	while (!kthread_should_stop()) {
		ret = si_pep_dav_check_mcmd(pep, &mcmd);
		if (ret == 0 && mcmd.cmd != 0)
			si_pep_handle_mcmd(pep, &mcmd);

		usleep_range(200, 250);
	}

	return 0;
}

static int si_pep_dav_check_req_t(void *arg)
{
	struct si_pep_thread_param *param = (struct si_pep_thread_param *)arg;
	struct si_pep_dev *pep = param->pep;
	struct si_qctx *ctx;
	enum si_qtype t = param->qtype;
	int qid = param->qid;

	if (t == SI_MWQ)
		ctx = pep->mwq_ctx;
	else if (t == SI_DWQ)
		ctx = pep->dwq_ctx[qid];
	else {
		si_err(pep, "Invalid queue type for request monitor thread");
		return 0;
	}

	while (!kthread_should_stop()) {
		usleep_range(100, 150);

		if (pep->state != SOC_STATE_RUN)
			continue;

		if (ctx->status == SI_ACTIVE)
			si_pep_get_reqs_from_host(pep, t, qid);
	}

	return 0;
}

static int si_pep_dav_mqe_recycler_t(void *arg)
{
	struct si_pep_dev *pep = (struct si_pep_dev*)arg;
	struct si_qctx *ctx;

	while (!kthread_should_stop()) {
		usleep_range(50, 100);

		ctx = pep->mcq_ctx;
		if (ctx && ctx->status == SI_ACTIVE)
			si_pep_recycle_qes(pep, SI_MCQ, 0);

		ctx = pep->mrq_ctx;
		if (ctx && ctx->status == SI_ACTIVE)
			si_pep_recycle_qes(pep, SI_MRQ, 0);
	}

	return 0;
}

static int si_pep_dav_dqe_recycler_t(void *arg)
{
	int qid;
	struct si_pep_dev *pep = (struct si_pep_dev*)arg;
	struct si_qctx *ctx;

	while (!kthread_should_stop()) {
		usleep_range(50, 100);

		for (qid = 0; qid < SI_DAV_MAX_DQS; qid++) {
			ctx = pep->drq_ctx[qid];
			if (ctx && ctx->status == SI_ACTIVE)
				si_pep_recycle_qes(pep, SI_DRQ, qid);

			ctx = pep->dcq_ctx[qid];
			if (ctx && ctx->status == SI_ACTIVE)
				si_pep_recycle_qes(pep, SI_DCQ, qid);
		}
	}

	return 0;
}

static void si_pep_dav_mq_cleanup(struct si_pep_dev *pep)
{
	if (mreq_checker) {
		kthread_stop(mreq_checker);
		mreq_checker = NULL;
	}

	if (mqe_recycler) {
		kthread_stop(mqe_recycler);
		mqe_recycler = NULL;
	}
}

static void si_pep_dav_dq_cleanup(struct si_pep_dev *pep)
{
	if (dqe_recycler) {
		kthread_stop(dqe_recycler);
		dqe_recycler = NULL;
	}
}

static int si_pep_dav_queue_created_callback(struct si_pep_dev *pep,
		enum si_qtype qtype, int qid)
{
	si_dbg(pep, "%s queue %d is created", si_pep_qtype_str(qtype), qid);
	if (qtype == SI_MWQ) {
		if (atomic_inc_return(&mq_ref) == 1) {
			mt_params.qid = 0;
			mt_params.qtype = SI_MWQ;
			mt_params.pep = pep;
			mreq_checker = kthread_create(si_pep_dav_check_req_t,
					&mt_params, "sima-mreq-mon");
			wake_up_process(mreq_checker);
			mqe_recycler = kthread_create(si_pep_dav_mqe_recycler_t,
					pep, "sima-mqe-recycler");
			wake_up_process(mqe_recycler);
		}
		return 0;
	} else if (qtype == SI_DWQ) {
		if (atomic_inc_return(&dq_ref) == 1) {
			dqe_recycler = kthread_create(si_pep_dav_dqe_recycler_t,
					pep, "sima-dqe-recycler");
			wake_up_process(dqe_recycler);
		}
		dt_params[qid].qid = qid;
		dt_params[qid].qtype = SI_DWQ;
		dt_params[qid].pep = pep;
		dreq_checker[qid] = kthread_create(si_pep_dav_check_req_t,
				&dt_params[qid], "sima-dreq%d-mon", qid);
		wake_up_process(dreq_checker[qid]);
	}

	return 0;
}

static int si_pep_dav_queue_deleted_callback(struct si_pep_dev *pep,
		enum si_qtype qtype, int qid)
{
	si_dbg(pep, "%s queue %d is deleted", si_pep_qtype_str(qtype), qid);
	if (qtype == SI_MWQ) {
		if (atomic_dec_and_test(&mq_ref))
			si_pep_dav_mq_cleanup(pep);
	} else if (qtype == SI_DWQ) {
		if (dreq_checker[qid]) {
			kthread_stop(dreq_checker[qid]);
			dreq_checker[qid] = NULL;
		}

		if (atomic_dec_and_test(&dq_ref))
			si_pep_dav_dq_cleanup(pep);
	}

	return 0;
}

static int si_pep_dav_plat_init(struct si_pep_dev *pep)
{
	struct platform_device *pdev = pep->pdev;
	struct device *dev = pep->dev;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "glue_logic");
	if (res == NULL) {
		si_err(pep, "Unable to find glue logic in device tree");
		return -ENOENT;
	}

	pep->glue_logic_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pep->glue_logic_base)) {
		si_err(pep, "Failed to map 'glue_logic' resources");
		return PTR_ERR(pep->glue_logic_base);
	}
	pep->glue_logic_size = resource_size(res);

	if (pep->hdma_base == 0)
		dma_reg_base = pep->dbi2_base + SIMA_DAV_PF0_DMA_REGS;
	else
		dma_reg_base = pep->hdma_base;

	si_pep_register_irq_handler(pep, "hdma_wch0", si_pep_dma_irq_handler,
			&w_irq_param[0]);
	si_pep_register_irq_handler(pep, "hdma_rch0", si_pep_dma_irq_handler,
			&r_irq_param[0]);
	si_pep_register_irq_handler(pep, "link_down", si_pep_dma_irq_handler,
			pep);

	return 0;
}

static void si_pep_dav_plat_deinit(struct si_pep_dev *pep)
{
}

static int si_pep_dav_local_init(struct si_pep_dev *pep)
{
	spin_lock_init(&dwq_tail_locks[0]);
	spin_lock_init(&dwq_tail_locks[1]);
	spin_lock_init(&dcq_head_locks[0]);
	spin_lock_init(&dcq_head_locks[1]);
	spin_lock_init(&drq_head_locks[0]);
	spin_lock_init(&drq_head_locks[1]);
	spin_lock_init(&net_reg_lock);

	/* Clear all head and tail registers */
        si_pep_dma_reg_write(DAV_OFF_MWQ_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_MWQ_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_MCQ_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_MCQ_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_MRQ_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_MRQ_TAIL, 0);

        si_pep_dma_reg_write(DAV_OFF_DWQ012_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DWQ345_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DWQ012_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_DWQ345_TAIL, 0);

        si_pep_dma_reg_write(DAV_OFF_DCQ012_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DCQ345_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DCQ012_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_DCQ345_TAIL, 0);

        si_pep_dma_reg_write(DAV_OFF_DRQ012_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DRQ345_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DRQ012_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_DRQ345_TAIL, 0);

	si_pep_dav_set_soc_state(pep, SOC_STATE_INIT);

	atomic_set(&dq_ref, 0);
	atomic_set(&mq_ref, 0);

	mcmd_checker = kthread_create(si_pep_dav_check_mcmd_t, pep,
			"sima-mcmd-mon");
	wake_up_process(mcmd_checker);

	return 0;
}

static void si_pep_dav_local_deinit(struct si_pep_dev *pep)
{
	int i;

	/* Clear all head and tail registers */
        si_pep_dma_reg_write(DAV_OFF_MWQ_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_MWQ_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_MCQ_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_MCQ_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_MRQ_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_MRQ_TAIL, 0);

        si_pep_dma_reg_write(DAV_OFF_DWQ012_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DWQ345_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DWQ012_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_DWQ345_TAIL, 0);

        si_pep_dma_reg_write(DAV_OFF_DCQ012_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DCQ345_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DCQ012_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_DCQ345_TAIL, 0);

        si_pep_dma_reg_write(DAV_OFF_DRQ012_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DRQ345_HEAD, 0);
        si_pep_dma_reg_write(DAV_OFF_DRQ012_TAIL, 0);
        si_pep_dma_reg_write(DAV_OFF_DRQ345_TAIL, 0);

	si_pep_dma_reg_write(DAV_OFF_SOC_STATUS, SOC_STATE_INIT);

	for (i = 0; i < SI_DAV_MAX_DQS; i++) {
		if (dreq_checker[i]) {
			kthread_stop(dreq_checker[i]);
			dreq_checker[i] = NULL;
		}
	}

	if (mqe_recycler) {
		kthread_stop(mqe_recycler);
		mqe_recycler = NULL;
	}

	if (dqe_recycler) {
		kthread_stop(dqe_recycler);
		dqe_recycler = NULL;
	}

	if (mreq_checker) {
		kthread_stop(mreq_checker);
		mreq_checker = NULL;
	}

	if (mcmd_checker) {
		kthread_stop(mcmd_checker);
		mcmd_checker = NULL;
	}

	atomic_set(&dq_ref, 0);
	atomic_set(&mq_ref, 0);
}

static int si_pep_dav_net_poller(void *arg)
{
	u32 head;
	u32 tail;
	struct si_pep_dev *pep = (struct si_pep_dev *)arg;
	struct si_pep_net_dev *net = &pep->net_dev;

	while (!kthread_should_stop()) {
		usleep_range(50, 60);

		if (kthread_should_stop())
			return 0;

		if (atomic_read(&net->rx_en) != 1)
			continue;

		if (net->created == 0) {
			BUG();
			return 0;
		}

		if (net->host_hwdata_p == 0)
			continue;

		atomic_set(&net->rx_en, 0);
		head = net_get_rxhead(pep);
		tail = net_get_rxtail(pep);
		si_pep_check_packets(pep, head, tail);
	}

	return 0;
}

static void si_pep_dav_net_set_val(struct si_pep_dev *pep,
		enum si_net_val_types type, u16 val)
{
	u32 offset;
	u32 reg;
	u32 lower;

	switch (type) {
	case tx_head:
		offset = DAV_OFF_NET_RXHEAD;
		lower = 0;
		break;
	case rx_tail:
		offset = DAV_OFF_NET_TXTAIL;
		lower = 1;
		break;
	default:
		return;
	}

	spin_lock(&net_reg_lock);
	reg = si_pep_dma_reg_read(offset);
	if (!lower)
		reg = (reg & 0x0000ffff) | ((u32)val << 16);
	else
		reg = (reg & 0xffff0000) | val;
	si_pep_dma_reg_write(offset, reg);
	spin_unlock(&net_reg_lock);
}

static u16 si_pep_dav_net_get_val(struct si_pep_dev *pep,
		enum si_net_val_types type)
{
	u32 offset;
	u32 reg;
	u32 lower;

	switch (type) {
	case tx_head:
		offset = DAV_OFF_NET_RXHEAD;
		lower = 0;
		break;
	case tx_tail:
		offset = DAV_OFF_NET_RXTAIL;
		lower = 1;
		break;
	case rx_head:
		offset = DAV_OFF_NET_TXHEAD;
		lower = 0;
		break;
	case rx_tail:
		offset = DAV_OFF_NET_TXTAIL;
		lower = 1;
		break;
	default:
		return 0;
	}

	reg = si_pep_dma_reg_read(offset);
	if (lower)
		return lower_16_bits(reg);

	return upper_16_bits(reg);
}

static int si_pep_dav_net_open(struct si_pep_dev *pep)
{
	if (!net_rx_poller) {
		net_rx_poller = kthread_create(si_pep_dav_net_poller, pep,
				"sima-net-rxpoller");
	}
	wake_up_process(net_rx_poller);

	return 0;
}

static void si_pep_dav_net_release(struct si_pep_dev *pep)
{
	if (net_rx_poller) {
		kthread_stop(net_rx_poller);
		net_rx_poller = NULL;
	}
}

const struct si_pep_drv_ops davinci_drv_ops = {
	.mq_set_val = si_pep_dav_mq_set_val,
	.mq_get_val = si_pep_dav_mq_get_val,
	.dq_set_val = si_pep_dav_dq_set_val,
	.dq_get_val = si_pep_dav_dq_get_val,
	.check_mcmd = si_pep_dav_check_mcmd,
	.resp_mcmd = si_pep_dav_resp_mcmd,
	.set_mlsoc_caps = NULL,
	.queue_created_callback = si_pep_dav_queue_created_callback,
	.queue_deleted_callback = si_pep_dav_queue_deleted_callback,
	.platform_init = si_pep_dav_plat_init,
	.local_init = si_pep_dav_local_init,
	.platform_deinit = si_pep_dav_plat_deinit,
	.local_deinit = si_pep_dav_local_deinit,
	.set_soc_state = si_pep_dav_set_soc_state,
	.net_set_val = si_pep_dav_net_set_val,
	.net_get_val = si_pep_dav_net_get_val,
	.net_open = si_pep_dav_net_open,
	.net_release = si_pep_dav_net_release
};

