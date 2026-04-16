// SPDX-License-Identifier: GPL-2.0
/*
 * Platform PCIe EP driver for SiMa.ai SoCs
 *
 * Copyright (C) 2024 SiMa.ai
 *
 */

#include <asm/io.h>

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "simaai_pcie_modalix.h"
#include "simaai_pep_drv_modalix.h"

struct delayed_work mcmd_handler;
struct delayed_work d_dreq_cachers[SI_MAX_DQS];
struct delayed_work d_mreq_cacher;
struct workqueue_struct *irq_work;

extern struct si_pep_dev *g_pep_dev;

static struct si_pep_db_int_desc db_int_descs[] = {
	{0, 0xfc}, {1, 0x150}, {2, 0x194}, {3, 0x1d4}, {4, 0x210}, {5, 0x254},
	{6, 0x294}, {7, 0x310}, {8, 0x354}, {9, 0x394}, {10, 0x410}, {11, 0x454},
	{12, 0x494}, {13, 0x510}, {14, 0x554}, {15, 0x594}, {16, 0x610},
	{17, 0x654}, {18, 0x694}, {19, 0x710}, {20, 0x754}, {21, 0x794},
	{22, 0x810}, {23, 0x854}, {24, 0x894}, {25, 0x910}, {26, 0x954},
	{27, 0x994}, {28, 0x50}, {29, 0x54}, {30, 0x58}, {31, 0x5c},
	{32, 0x60}, {33, 0x64}, {34, 0x68}, {35, 0x6c}
};
static int n_db_irqs __attribute__((unused)) = ARRAY_SIZE(db_int_descs);

static irqreturn_t si_pep_mod_doorbell_int_handler(int, void*);
static irqreturn_t si_pep_mod_net_rxhead_int_handler(int, void*);
static irqreturn_t si_pep_mod_net_rxhead_int_handler_t(int, void*);
static void si_pep_mod_delayed_mcmd_handler(struct work_struct*);

typedef void (*si_pep_dreq_cacher)(struct work_struct *);

static void si_pep_mod_delayed_mreq_cacher(struct work_struct *qwork)
{
	si_pep_get_reqs_from_host(g_pep_dev, SI_MWQ, 0);
}

#define DEF_DQ_REQ_CHECKERS \
	RC(0) RC(1) RC(2) RC(3) RC(4) RC(5) RC(6) RC(7) RC(8)

#define RC(q) \
static void si_pep_mod_delayed_dreq_cacher_##q(struct work_struct *w) \
{ \
	si_pep_get_reqs_from_host(g_pep_dev, SI_DWQ, q); \
} \

DEF_DQ_REQ_CHECKERS
#undef RC
static si_pep_dreq_cacher dreq_cachers[] = {
#define RC(q) \
	&si_pep_mod_delayed_dreq_cacher_##q,
DEF_DQ_REQ_CHECKERS
#undef RC
};

static __always_inline u32 si_pep_asr_reg_read(struct si_pep_dev *pep,
					       uint32_t offset)
{
	return readl(pep->asr_base + offset);
}

static __always_inline void si_pep_asr_reg_write(struct si_pep_dev *pep,
						 uint32_t offset, uint32_t val)
{
	writel(val, pep->asr_base + offset);
}

static u16 si_pep_mod_mq_get_val(struct si_pep_dev *pep,
		enum si_bar_val_types type)
{
	u32 offset;

	switch (type) {
	case wq_head:
		offset = MOD_OFF_MWQ_HEAD_DB;
		break;
	case wq_tail:
		offset = MOD_OFF_MWQ_TAIL;
		break;
	case cq_head:
		offset = MOD_OFF_MCQ_HEAD;
		break;
	case cq_tail:
		offset = MOD_OFF_MCQ_TAIL_DB;
		break;
	case rq_head:
		offset = MOD_OFF_MRQ_HEAD;
		break;
	case rq_tail:
		offset = MOD_OFF_MRQ_TAIL_DB;
		break;
	default:
		return 0;
	}

	return ((si_pep_asr_reg_read(pep, offset) & 0xffff));
}

static void si_pep_mod_mq_set_val(struct si_pep_dev *pep, u16 val,
		enum si_bar_val_types type)
{
	u32 offset;
	u32 reg;

	switch (type) {
	case wq_head:
		offset = MOD_OFF_MWQ_HEAD_DB;
		break;
	case wq_tail:
		offset = MOD_OFF_MWQ_TAIL;
		break;
	case cq_head:
		offset = MOD_OFF_MCQ_HEAD;
		break;
	case cq_tail:
		offset = MOD_OFF_MCQ_TAIL_DB;
		break;
	case rq_head:
		offset = MOD_OFF_MRQ_HEAD;
		break;
	case rq_tail:
		offset = MOD_OFF_MRQ_TAIL_DB;
		break;
	default:
		return;
	}

	reg = si_pep_asr_reg_read(pep, offset);
	si_pep_asr_reg_write(pep, offset, (reg & 0xffff0000) | val);
}

static u16 si_pep_mod_dq_get_val(struct si_pep_dev *pep, u32 qid,
		enum si_bar_val_types type)
{
	u32 offset;

	switch (type) {
	case wq_head:
		offset = MOD_OFF_WQ_HEAD_DB(qid);
		break;
	case wq_tail:
		offset = MOD_OFF_WQ_TAIL(qid);
		break;
	case cq_head:
		offset = MOD_OFF_CQ_HEAD(qid);
		break;
	case cq_tail:
		offset = MOD_OFF_CQ_TAIL_DB(qid);
		break;
	case rq_head:
		offset = MOD_OFF_RQ_HEAD(qid);
		break;
	case rq_tail:
		offset = MOD_OFF_RQ_TAIL_DB(qid);
		break;
	default:
		return 0;
	}

	return ((si_pep_asr_reg_read(pep, offset) & 0xffff));
}

static void si_pep_mod_dq_set_val(struct si_pep_dev *pep, u32 qid, u16 val,
		enum si_bar_val_types type)
{
	u32 offset;
	u32 reg;

	switch (type) {
	case wq_head:
		offset = MOD_OFF_WQ_HEAD_DB(qid);
		break;
	case wq_tail:
		offset = MOD_OFF_WQ_TAIL(qid);
		break;
	case cq_head:
		offset = MOD_OFF_CQ_HEAD(qid);
		break;
	case cq_tail:
		offset = MOD_OFF_CQ_TAIL_DB(qid);
		break;
	case rq_head:
		offset = MOD_OFF_RQ_HEAD(qid);
		break;
	case rq_tail:
		offset = MOD_OFF_RQ_TAIL_DB(qid);
		break;
	default:
		return;
	}

	reg = si_pep_asr_reg_read(pep, offset);
	si_pep_asr_reg_write(pep, offset, (reg & 0xffff0000) | val);
}

static void si_pep_mod_set_soc_state(struct si_pep_dev *pep,
		enum si_soc_state state)
{
	pep->state = state;
	si_pep_asr_reg_write(pep, MOD_OFF_MLSOC_STATUS, state);
}

static int si_pep_mod_check_mcmd(struct si_pep_dev *pep, struct si_mcmd *mcmd)
{
	u8 __iomem *src = pep->asr_base + MOD_OFF_MCMD;
	u32 reg;

	reg = si_pep_asr_reg_read(pep, MOD_OFF_MCMD);
	if (reg == 0)
		return -1;

	si_pep_iomem_read_rep(mcmd, src, sizeof(struct si_mcmd));
	return 0;
}

static int si_pep_mod_resp_mcmd(struct si_pep_dev *pep, struct si_mcmd_res *resp)
{
	struct si_mcmd __iomem *mcmd = si_pep_cache_virt(pep, mcmd);
	struct si_mcmd_res __iomem *mresp = si_pep_cache_virt(pep, mcmd_res);
	u64 daddr = si_pep_asr_reg_read(pep, MOD_OFF_MCMD_ADDR_LOW) |
		((u64)si_pep_asr_reg_read(pep, MOD_OFF_MCMD_ADDR_HIGH) << 32);
	u64 saddr = si_pep_cache_phys(pep, mcmd_res);
	u8 __iomem *mcmd_asr_base = pep->asr_base + MOD_OFF_MCMD;
	u8 __iomem *mresp_asr_base = pep->asr_base + MOD_OFF_MCMD_RESP;

	si_pep_iomem_memset(mcmd_asr_base, 0, sizeof(struct si_mcmd) - 4);
	si_pep_iomem_memset(mcmd, 0, sizeof(struct si_mcmd));
	si_pep_iomem_write_rep(mresp_asr_base, resp, sizeof(struct si_mcmd_res));
	si_pep_iomem_write_rep(mresp, resp, sizeof(struct si_mcmd_res));

	daddr = daddr + sizeof(struct si_mcmd);
	si_pep_copy_to_host(pep, daddr, saddr, 4, MCMD_DMA_CH, SI_SMCMD_DONE_MSI);
	si_pep_asr_reg_write(pep, MOD_OFF_MCMD_DB, 0);

	return 0;
}

static void si_pep_mod_set_mlsoc_caps(struct si_pep_dev *pep, u32 cap, u32 val)
{
	si_pep_asr_reg_write(pep, cap, val);
}

static int si_pep_mod_queue_created_callback(struct si_pep_dev *pep,
		enum si_qtype qtype, int qid)
{
	return 0;
}

static int si_pep_mod_queue_deleted_callback(struct si_pep_dev *pep,
		enum si_qtype qtype, int qid)
{
	return 0;
}

static int si_pep_mod_plat_init(struct si_pep_dev *pep)
{
	int i;
	int irq;
	struct platform_device *pdev = pep->pdev;
	struct device *dev = pep->dev;
	struct resource *res;
	char irqname[32];

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hdma");
	if (res == NULL) {
		si_err(pep, "HDMA base address is not found in device tree");
		return -ENOENT;
	}

	pep->hdma_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pep->hdma_base)) {
		si_err(pep, "Failed to map 'hdma' resources");
		return PTR_ERR(pep->hdma_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "asr");
	if (res == NULL) {
		si_err(pep, "ASR base address is not found in device tree");
		return -ENOENT;
	}

	pep->asr_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pep->hdma_base)) {
		si_err(pep, "Failed to map 'asr' resources");
		return PTR_ERR(pep->asr_base);
	}

	for (i = MOD_MCMD_DB; i <= MOD_DQ_DB_END; i++) {
		snprintf(irqname, sizeof(irqname), "doorbell_%d", i);
		si_pep_register_irq_handler(pep, irqname,
			si_pep_mod_doorbell_int_handler, &db_int_descs[i]);
	}

	snprintf(irqname, sizeof(irqname), "doorbell_%d", MOD_NET_TXHEAD_DB);
	irq = platform_get_irq_byname(pdev, irqname);
	if (irq < 0) {
		si_err(pep, "Failed to find IRQ for '%s'", irqname);
		return irq;
	}

	i = devm_request_threaded_irq(dev, irq,
			si_pep_mod_net_rxhead_int_handler,
			si_pep_mod_net_rxhead_int_handler_t, IRQF_ONESHOT,
			pep->dev->driver->name,
			&db_int_descs[MOD_NET_TXHEAD_DB]);
	if (i < 0) {
		si_err(pep, "Failed to register handler for '%s'", irqname);
		return i;
	}
#if 0
	/* This will cause nested interrupts */
	for (i = 0; i < 16; i++) {
		snprintf(irqname, sizeof(irqname), "hdma_wch%d", i);
		si_pep_register_irq_handler(pep, irqname, si_pep_dma_irq_handler,
				&w_irq_param[0]);

		snprintf(irqname, sizeof(irqname), "hdma_rch%d", i);
		si_pep_register_irq_handler(pep, irqname, si_pep_dma_irq_handler,
				&r_irq_param[0]);
	}
#endif

	return 0;
}

static void si_pep_mod_plat_deinit(struct si_pep_dev *pep)
{
}

static int si_pep_mod_local_init(struct si_pep_dev *pep)
{
	int i;

	irq_work = alloc_workqueue("sima-mod", WQ_MEM_RECLAIM | WQ_HIGHPRI |
			WQ_UNBOUND, 0);
	if (!irq_work) {
		si_err(pep, "Failed to allocate a workqueue for irq");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&mcmd_handler, si_pep_mod_delayed_mcmd_handler);
	for (i = 0; i < SI_MAX_DQS; i++)
		INIT_DELAYED_WORK(&d_dreq_cachers[i], dreq_cachers[i]);
	INIT_DELAYED_WORK(&d_mreq_cacher, si_pep_mod_delayed_mreq_cacher);
	si_pep_mod_set_soc_state(pep, SOC_STATE_INIT);

	return 0;
}

static void si_pep_mod_local_deinit(struct si_pep_dev *pep)
{
	int i;

	si_pep_mod_set_soc_state(pep, SOC_STATE_INIT);
	cancel_delayed_work(&mcmd_handler);
	cancel_delayed_work(&d_mreq_cacher);
	for (i = 0; i < SI_MAX_DQS; i++)
		cancel_delayed_work(&d_dreq_cachers[i]);
	destroy_workqueue(irq_work);
}

static void si_pep_mod_net_set_val(struct si_pep_dev *pep,
		enum si_net_val_types type, u16 val)
{
	u32 offset;

	switch (type) {
	case tx_head:
		offset = MOD_OFF_NET_RXHEAD;
		break;
	case rx_tail:
		offset = MOD_OFF_NET_TXTAIL;
		break;
	default:
		return;
	}

	si_pep_asr_reg_write(pep, offset, 0ul | val);
}

static u16 si_pep_mod_net_get_val(struct si_pep_dev *pep,
		enum si_net_val_types type)
{
	u32 offset;

	switch (type) {
	case tx_head:
		offset = MOD_OFF_NET_RXHEAD;
		break;
	case tx_tail:
		offset = MOD_OFF_NET_RXTAIL;
		break;
	case rx_head:
		offset = MOD_OFF_NET_TXHEAD;
		break;
	case rx_tail:
		offset = MOD_OFF_NET_TXTAIL;
		break;
	default:
		return 0;
	}

	return (u16)si_pep_asr_reg_read(pep, offset);
}

static int si_pep_mod_net_open(struct si_pep_dev *pep)
{
	return 0;
}

static void si_pep_mod_net_release(struct si_pep_dev *pep)
{
}

static void si_pep_mod_delayed_mcmd_handler(struct work_struct *iwork)
{
	struct si_mcmd *mcmd = si_pep_cache_virt(g_pep_dev, mcmd);
	if (mcmd->cmd)
		si_pep_handle_mcmd(g_pep_dev, mcmd);
}

static irqreturn_t si_pep_mod_net_rxhead_int_handler(int irq, void *arg)
{
	struct si_pep_db_int_desc *desc;
	struct si_pep_dev *pep = g_pep_dev;
	struct si_pep_net_dev *net = &pep->net_dev;
	u32 reg;

	desc = (struct si_pep_db_int_desc*)arg;
	reg = si_pep_asr_reg_read(pep, desc->offset);
	reg = (reg | MOD_DOORBELL_INT_DIS) & (~(MOD_DOORBELL_INT));
	si_pep_asr_reg_write(pep, desc->offset, reg);
	if (atomic_read(&net->rx_en) == 1) {
		atomic_set(&net->rx_en, 0);
		return IRQ_WAKE_THREAD;
	}

	reg = si_pep_asr_reg_read(pep, desc->offset);
	reg = reg & (~MOD_DOORBELL_INT_DIS);
	si_pep_asr_reg_write(pep, desc->offset, reg);

	return IRQ_HANDLED;
}

static irqreturn_t si_pep_mod_net_rxhead_int_handler_t(int irq, void *arg)
{
	u16 head;
	u16 tail;

	head = net_get_rxhead((g_pep_dev));
	tail = net_get_rxtail((g_pep_dev));
	si_pep_check_packets(g_pep_dev, head, tail);

	return IRQ_HANDLED;
}

static irqreturn_t si_pep_mod_doorbell_int_handler(int irq, void *arg)
{
	struct si_pep_db_int_desc *desc;
	struct si_pep_dev *pep = g_pep_dev;
	struct si_mcmd *mcmd = si_pep_cache_virt(pep, mcmd);
	u32 reg;
	u16 val;
	int type;
	int qid;
	int spdbno = 0;

	if (!pep->modalix)
		return IRQ_HANDLED;

	desc = (struct si_pep_db_int_desc*)arg;
	reg = readl(pep->asr_base + desc->offset);
	val = MOD_DOORBELL_VAL(reg);
	si_dbg_l(pep, "Interrupt %d, Doorbell Interrupt %d, Ofset 0x%x, "
			"Reg 0x%x, Value 0x%x",
			irq, desc->index, desc->offset, reg, val);
	/* Disable the interrupt and mask further interrupts until we are done */
	reg = (reg | MOD_DOORBELL_INT_DIS) & (~(MOD_DOORBELL_INT));
	writel(reg, pep->asr_base + desc->offset);

	if (desc->index >= MOD_DQ_DB_START && desc->index <= MOD_DQ_DB_END) {
		/* Data queue doorbells */
		qid = (desc->index - MOD_DQ_DB_START) / 3;
		type = (desc->index - MOD_DQ_DB_START) % 3;
		switch (type) {
		case MOD_DWQ_DB_REM: /* wqe head */
			queue_work(irq_work, &d_dreq_cachers[qid].work);
			break;
		case MOD_DCQ_DB_REM: /* cqe tail */
			si_pep_recycle_qes(pep, SI_DCQ, qid);
			break;
		case MOD_DRQ_DB_REM: /* rqe tail */
			si_pep_recycle_qes(pep, SI_DRQ, qid);
			break;
		default:
			/* This should not happen, right? */
			BUG();
		}
		goto handled;
	} else if (desc->index >= MOD_SPARE_DB_START &&
			desc->index <= MOD_SPARE_DB_END) {
		spdbno = desc->index - MOD_SPARE_DB_START;
		switch (spdbno) {
		default:
			si_dbg(pep, "Spare doorbell %d. No action defined",
					spdbno);
		}
		goto handled;
	}

	switch (desc->index) {
	case MOD_MCMD_DB:
		if (pep->drv_ops->check_mcmd(pep, mcmd) == 0) {
			reg = reg | 0xA5A5;
			writel(reg, pep->asr_base + desc->offset);
			/* Management command handlers do non-int-context-
			 * friendly stuff. Can't handle them here. Defer
			 */
			queue_work(irq_work, &mcmd_handler.work);
		}
		break;
	case MOD_MWQ_DB:	/* MWQ head */
		queue_work(irq_work, &d_mreq_cacher.work);
		break;
	case MOD_MCQ_DB:	/* MCQ tail */
		si_pep_recycle_qes(pep, SI_MCQ, 0);
		break;
	case MOD_MRQ_DB:	/* MRQ tail */
		si_pep_recycle_qes(pep, SI_MRQ, 0);
		break;
	}

handled:
	reg = readl(pep->asr_base + desc->offset);
	reg = reg & (~(MOD_DOORBELL_INT_DIS));
	writel(reg, pep->asr_base + desc->offset);

	return IRQ_HANDLED;
}

const struct si_pep_drv_ops modalix_drv_ops = {
	.mq_set_val = si_pep_mod_mq_set_val,
	.mq_get_val = si_pep_mod_mq_get_val,
	.dq_set_val = si_pep_mod_dq_set_val,
	.dq_get_val = si_pep_mod_dq_get_val,
	.check_mcmd = si_pep_mod_check_mcmd,
	.resp_mcmd = si_pep_mod_resp_mcmd,
	.set_mlsoc_caps = si_pep_mod_set_mlsoc_caps,
	.queue_created_callback = si_pep_mod_queue_created_callback,
	.queue_deleted_callback = si_pep_mod_queue_deleted_callback,
	.platform_init = si_pep_mod_plat_init,
	.local_init = si_pep_mod_local_init,
	.platform_deinit = si_pep_mod_plat_deinit,
	.local_deinit = si_pep_mod_local_deinit,
	.set_soc_state = si_pep_mod_set_soc_state,
	.net_set_val = si_pep_mod_net_set_val,
	.net_get_val = si_pep_mod_net_get_val,
	.net_open = si_pep_mod_net_open,
	.net_release = si_pep_mod_net_release
};

