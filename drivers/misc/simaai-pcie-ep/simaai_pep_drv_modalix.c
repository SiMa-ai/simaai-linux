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
#include <linux/interrupt.h>

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
static int n_db_irqs = ARRAY_SIZE(db_int_descs);

static irqreturn_t si_pep_mod_doorbell_int_handler(int, void*);
static irqreturn_t si_pep_mod_net_rxhead_int_handler(int, void*);
static void si_pep_mod_delayed_mcmd_handler(struct work_struct*);
static void si_pep_asr_db_enable_int(struct si_pep_dev *, u32);

typedef void (*si_pep_dreq_cacher)(struct work_struct *);

/*
 * Fetch what the host queued, then re-enable the doorbell interrupt, then
 * look again -- and keep going while each pass makes progress.
 *
 * The interrupt is masked from the moment the handler takes it until this
 * runs, and a doorbell written during that window updates the register
 * without raising anything. Fetching once and enabling once therefore
 * loses any request the host queued between the fetch and the enable:
 * nothing will ask again, because the only thing that asks is a doorbell,
 * and a host that is now blocked waiting for the reply to that very
 * request will not write another. The transfer stops dead with no error
 * on either side.
 *
 * Re-checking after the enable closes it. A request that arrives after the
 * last check raises an interrupt, because by then the interrupt is on. The
 * progress test is what keeps a request cache that has genuinely run out
 * of room from spinning here.
 */
static void si_pep_mod_cache_reqs(struct si_pep_dev *pep, enum si_qtype t,
				  int qid, u32 db_off)
{
	/*
	 * Enable the interrupt before fetching, never after.
	 *
	 * The doorbell is masked from the moment the handler takes it until
	 * this runs, and a doorbell written while it is masked raises
	 * nothing. Fetch-then-enable therefore always leaves a window: a
	 * request queued between the fetch and the enable is not in what we
	 * just read, and will not raise an interrupt either, so nothing ever
	 * looks again -- and a host blocked waiting for the reply to that
	 * request will not write another doorbell to shake it loose.
	 *
	 * Enabling first inverts the two cases and leaves no gap between
	 * them. Anything queued before the enable is picked up by the fetch
	 * below, which reads the live head register rather than anything
	 * latched at interrupt time. Anything queued after it raises an
	 * interrupt, which queues this work again. The cost is at most one
	 * redundant run, which finds nothing and returns.
	 */
	si_pep_asr_db_enable_int(pep, db_off);
	si_pep_get_reqs_from_host(pep, t, qid);
}

static void si_pep_mod_delayed_mreq_cacher(struct work_struct *qwork)
{
	si_pep_mod_cache_reqs(g_pep_dev, SI_MWQ, 0, MOD_OFF_MWQ_HEAD_DB);
}

#define DEF_DQ_REQ_CHECKERS \
	RC(0) RC(1) RC(2) RC(3) RC(4) RC(5) RC(6) RC(7) RC(8)

#define RC(q) \
static void si_pep_mod_delayed_dreq_cacher_##q(struct work_struct *w) \
{ \
	si_pep_mod_cache_reqs(g_pep_dev, SI_DWQ, q, \
			      MOD_OFF_WQ_HEAD_DB(q)); \
} \

DEF_DQ_REQ_CHECKERS
#undef RC
static si_pep_dreq_cacher dreq_cachers[] = {
#define RC(q) \
	&si_pep_mod_delayed_dreq_cacher_##q,
DEF_DQ_REQ_CHECKERS
#undef RC
};

static __always_inline u16 si_pep_asr_reg_readw(struct si_pep_dev *pep,
		u32 offset)
{
	return readw(pep->asr_base + offset);
}

static __always_inline void si_pep_asr_reg_writew(struct si_pep_dev *pep,
						 u32 offset, u16 val)
{
	writew(val, pep->asr_base + offset);
}

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

/*
 * +----------+----------------+-----------+-------+
 * |  31:18   |       17       |     16    |  15:0 |
 * +----------+----------------+-----------+-------+
 * | Reserved | Interrupt Mask | Interrupt | Index |
 * +----------+----------------+-----------+-------+
 */
static void si_pep_asr_db_clear_and_mask_int(struct si_pep_dev *pep, u32 off)
{
	u16 val = si_pep_asr_reg_readw(pep, off + 2);

	val = (val | BIT(1)) & (~(BIT(0)));

	si_pep_asr_reg_writew(pep, off + 2, val);
}

static void si_pep_asr_db_enable_int(struct si_pep_dev *pep, u32 off)
{
	u16 val = si_pep_asr_reg_readw(pep, off + 2);

	val = val & ~(BIT(1));

	si_pep_asr_reg_writew(pep, off + 2, val);
}

static void si_pep_asr_db_disable_int(struct si_pep_dev *pep, u32 off)
{
	u16 val = si_pep_asr_reg_readw(pep, off + 2);

	val = val | BIT(1);

	si_pep_asr_reg_writew(pep, off + 2, val);
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

	/*
	 * The reply the host actually reads is the one written into the ASR
	 * window above. The DMA below is a second copy into the host's
	 * coherent buffer, and it carries the MSI that releases the host from
	 * its wait.
	 *
	 * Its destination comes from the host, which writes MCMD_ADDR when it
	 * sees the SoC reach SOC_STATE_INIT. A host that missed that
	 * transition leaves the register at zero, and DMAing to zero plus the
	 * offset below is a write into whatever sits at host physical address
	 * 0x80 -- an IOMMU fault at best, silent corruption of the host's low
	 * memory at worst. Skip it. The host falls back to polling the
	 * doorbell cleared below, which costs it a timeout but is correct.
	 */
	if (daddr == 0) {
		if (printk_ratelimit())
			si_err(pep, "Host has not set the mgmt response address");
	} else {
		daddr = daddr + sizeof(struct si_mcmd);
		si_pep_copy_to_host(pep, daddr, saddr, 4, MCMD_DMA_CH,
				SI_SMCMD_DONE_MSI);
	}

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
	if (IS_ERR(pep->asr_base)) {
		si_err(pep, "Failed to map 'asr' resources");
		return PTR_ERR(pep->asr_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pcie_sys");
	if (res == NULL) {
		si_err(pep, "PCIeSYS base address is not found in device tree");
		return -ENOENT;
	}

	pep->pciesys_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pep->pciesys_base)) {
		si_err(pep, "Failed to map 'PCIeSYS' resources");
		return PTR_ERR(pep->pciesys_base);
	}

	/*
	 * Install the link-down handler but leave the event masked, for the
	 * same reason the doorbells below are masked: the handler tears down
	 * the workqueue, the queues and the veth, none of which probe has
	 * built yet. Some hosts retrain the link while the SoC is still
	 * probing, and that event used to reach local_deinit() with no
	 * workqueue to destroy. si_pep_probe() lifts the mask once there is
	 * something to tear down.
	 *
	 * Masking first, rather than after the request, keeps an enable left
	 * behind by a previous boot from firing the moment the line is on.
	 */
	si_pep_irq_link_down_disable(pep);
	i = si_pep_register_irq_handler(pep, "pcie0_lde", si_pep_irq_link_down,
			pep);
	if (i != 0) {
		si_err(pep, "Failed to register handler for pcie0_ldeinterrupt. Error %d\n", i);
		return -EINVAL;
	}


	/*
	 * Mask every doorbell before a single handler is installed.
	 *
	 * The mask lives in the ASR register rather than in the GIC, so it
	 * survives what the kernel knows about. A doorbell left unmasked by
	 * a previous boot -- the host keeps running across an SoC reset, and
	 * goes on writing doorbells throughout it -- fires the instant
	 * devm_request_irq() enables the line, which is long before the
	 * workqueue and the queue contexts the handler needs exist.
	 *
	 * si_pep_mod_enable_doorbells() lifts the mask once probe has built
	 * all of that.
	 */
	for (i = 0; i < n_db_irqs; i++)
		si_pep_asr_db_clear_and_mask_int(pep, db_int_descs[i].offset);

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

	i = devm_request_irq(dev, irq, si_pep_mod_net_rxhead_int_handler, 0,
			pep->dev->driver->name,
			&db_int_descs[MOD_NET_TXHEAD_DB]);
	if (i < 0) {
		si_err(pep, "Failed to register handler for '%s'", irqname);
		return i;
	}

	/*
	 * The completion interrupts of the DMA channel the network interface
	 * transmits and receives on. Nothing depends on them: without one,
	 * the transfers in its direction are waited for. They are left
	 * disabled here and enabled for as long as there is an interface.
	 */
	snprintf(irqname, sizeof(irqname), "hdma_wch%d", ETH_DATA_DMA_CH);
	irq = platform_get_irq_byname_optional(pdev, irqname);
	if (irq > 0 && devm_request_irq(dev, irq, si_pep_net_tx_irq,
				IRQF_NO_AUTOEN, "simaai_pep_net_tx", pep) == 0)
		pep->net_tx_irq = irq;
	else
		si_err(pep, "No interrupt for '%s'", irqname);

	snprintf(irqname, sizeof(irqname), "hdma_rch%d", ETH_DATA_DMA_CH);
	irq = platform_get_irq_byname_optional(pdev, irqname);
	if (irq > 0 && devm_request_irq(dev, irq, si_pep_net_rx_irq,
				IRQF_NO_AUTOEN, "simaai_pep_net_rx", pep) == 0)
		pep->net_rx_irq = irq;
	else
		si_err(pep, "No interrupt for '%s'", irqname);

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
	/* Install handler for pcie link down interrupt */
	return 0;
}

static void si_pep_mod_plat_deinit(struct si_pep_dev *pep)
{
}

/*
 * The network doorbell is deliberately left out. Its interrupt belongs to
 * the netdev, which arms it in net_enable_rxint() when the interface comes
 * up and masks it again on the way down; arming it here would deliver
 * packets to an interface that is not open.
 *
 * So is the register the host keeps its receive interrupt enable in. It is
 * a doorbell only by address: its value is read when a packet is sent and
 * nothing is ever meant to ring it.
 */
static void si_pep_mod_enable_doorbells(struct si_pep_dev *pep)
{
	int i;

	for (i = 0; i < n_db_irqs; i++) {
		if (i == MOD_NET_TXHEAD_DB || i == MOD_NET_RXINT_DB)
			continue;

		si_pep_asr_db_enable_int(pep, db_int_descs[i].offset);
	}
}

static void si_pep_mod_disable_doorbells(struct si_pep_dev *pep)
{
	int i;

	for (i = 0; i < n_db_irqs; i++)
		si_pep_asr_db_disable_int(pep, db_int_descs[i].offset);
}

static int si_pep_mod_local_init(struct si_pep_dev *pep)
{
	struct workqueue_struct *wq;
	int i;

	wq = alloc_workqueue("sima-mod", WQ_MEM_RECLAIM | WQ_HIGHPRI |
			WQ_UNBOUND, 0);
	if (!wq) {
		si_err(pep, "Failed to allocate a workqueue for irq");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&mcmd_handler, si_pep_mod_delayed_mcmd_handler);
	for (i = 0; i < SI_MAX_DQS; i++)
		INIT_DELAYED_WORK(&d_dreq_cachers[i], dreq_cachers[i]);
	INIT_DELAYED_WORK(&d_mreq_cacher, si_pep_mod_delayed_mreq_cacher);

	/*
	 * Publish the workqueue only once the work items it will be handed
	 * are initialised. irq_work is what the doorbell handler tests before
	 * it queues anything, so the store has to be the last of the group to
	 * become visible -- otherwise the handler can find a usable workqueue
	 * and hand it a work_struct that INIT_DELAYED_WORK has not reached,
	 * which is the same NULL func it would have crashed on before.
	 */
	smp_store_release(&irq_work, wq);
	si_pep_mod_set_soc_state(pep, SOC_STATE_INIT);

	return 0;
}

static void si_pep_mod_local_deinit(struct si_pep_dev *pep)
{
	struct workqueue_struct *wq = irq_work;
	int i;

	/*
	 * Nothing to take down if local_init() never ran, or already undid
	 * this: the work items are uninitialised and destroy_workqueue()
	 * does not tolerate NULL.
	 */
	if (!wq)
		return;

	si_pep_mod_set_soc_state(pep, SOC_STATE_INIT);

	/*
	 * Withdraw the workqueue before freeing it, not after.
	 *
	 * Callers disarm the doorbells before getting here, so the handler
	 * should already be quiet. But a freed workqueue whose pointer still
	 * looks valid is the one mistake on this path with no recovery, and
	 * the order that prevents it costs nothing: the handler cannot pick
	 * up what it can no longer see.
	 */
	WRITE_ONCE(irq_work, NULL);

	cancel_delayed_work(&mcmd_handler);
	cancel_delayed_work(&d_mreq_cacher);
	for (i = 0; i < SI_MAX_DQS; i++)
		cancel_delayed_work(&d_dreq_cachers[i]);
	destroy_workqueue(wq);
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
	case host_rxint:
		offset = MOD_OFF_NET_RXINT;
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

static void si_pep_mod_net_enable_rxint(struct si_pep_dev *pep)
{
	const u32 offset = MOD_OFF_NET_TXHEAD;

	si_pep_asr_db_enable_int(pep, offset);
}

static void si_pep_mod_net_disable_rxint(struct si_pep_dev *pep)
{
	const u32 offset = MOD_OFF_NET_TXHEAD;

	si_pep_asr_db_disable_int(pep, offset);
}

static void si_pep_mod_delayed_mcmd_handler(struct work_struct *iwork)
{
	struct si_mcmd *mcmd = si_pep_cache_virt(g_pep_dev, mcmd);
	if (mcmd->cmd)
		si_pep_handle_mcmd(g_pep_dev, mcmd);

	si_pep_asr_reg_writew(g_pep_dev, MOD_OFF_MCMD_DB, 0);
	si_pep_asr_db_enable_int(g_pep_dev, MOD_OFF_MCMD_DB);

}

/*
 * The host published a packet. Mask the doorbell and leave the rest to the
 * poll, which fetches what the host has queued and turns the doorbell back
 * on when it has caught up.
 */
static irqreturn_t si_pep_mod_net_rxhead_int_handler(int irq, void *arg)
{
	struct si_pep_dev *pep = g_pep_dev;
	struct si_pep_db_int_desc *desc;

	desc = (struct si_pep_db_int_desc*)arg;
	si_pep_asr_db_clear_and_mask_int(pep, desc->offset);
	si_pep_net_rx_schedule(pep);

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
	/* Disable the interrupt and mask further interrupts until we are done */
	si_pep_asr_db_clear_and_mask_int(pep, desc->offset);

	/*
	 * Probe has not reached local_init() yet, or teardown has already
	 * passed local_deinit(). Either way there is no workqueue to defer to
	 * and no queue context to act on, so leave the doorbell masked and
	 * drop this one. enable_doorbells() re-arms it, and the host re-reads
	 * the live head register when it does.
	 */
	if (!smp_load_acquire(&irq_work))
		return IRQ_HANDLED;
	
	val = MOD_DOORBELL_VAL(reg);
	si_dbg_l(pep, "Interrupt %d, Doorbell Interrupt %d, Ofset 0x%x, "
			"Reg 0x%x, Value 0x%x",
			irq, desc->index, desc->offset, reg, val);

	if (desc->index >= MOD_DQ_DB_START && desc->index <= MOD_DQ_DB_END) {
		/* Data queue doorbells */
		qid = (desc->index - MOD_DQ_DB_START) / 3;
		type = (desc->index - MOD_DQ_DB_START) % 3;
		switch (type) {
		case MOD_DWQ_DB_REM: /* wqe head */
			queue_work(irq_work, &d_dreq_cachers[qid].work);
			return IRQ_HANDLED;
			break;
		case MOD_DCQ_DB_REM: /* cqe tail */
			si_pep_recycle_qes(pep, SI_DCQ, qid);
			si_pep_asr_db_enable_int(pep, desc->offset);
			/*
			 * Again with the interrupt back on. These doorbells
			 * carry a position, not an event, so one written
			 * while the interrupt was masked raises nothing and
			 * would otherwise leave this end believing the host
			 * has consumed less than it has -- and refusing to
			 * produce completions into a queue it thinks is
			 * full. Re-reading costs a register access.
			 */
			si_pep_recycle_qes(pep, SI_DCQ, qid);
			return IRQ_HANDLED;
		case MOD_DRQ_DB_REM: /* rqe tail */
			si_pep_recycle_qes(pep, SI_DRQ, qid);
			si_pep_asr_db_enable_int(pep, desc->offset);
			si_pep_recycle_qes(pep, SI_DRQ, qid);
			return IRQ_HANDLED;
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
			writew(0x10ad, pep->asr_base + desc->offset);
			/* Management command handlers do non-int-context-
			 * friendly stuff. Can't handle them here. Defer
			 */
			queue_work(irq_work, &mcmd_handler.work);
			return IRQ_HANDLED;
		}
		break;
	case MOD_MWQ_DB:	/* MWQ head */
		queue_work(irq_work, &d_mreq_cacher.work);
		return IRQ_HANDLED;
		break;
	case MOD_MCQ_DB:	/* MCQ tail */
		si_pep_recycle_qes(pep, SI_MCQ, 0);
		si_pep_asr_db_enable_int(pep, desc->offset);
		si_pep_recycle_qes(pep, SI_MCQ, 0);
		return IRQ_HANDLED;
	case MOD_MRQ_DB:	/* MRQ tail */
		si_pep_recycle_qes(pep, SI_MRQ, 0);
		si_pep_asr_db_enable_int(pep, desc->offset);
		si_pep_recycle_qes(pep, SI_MRQ, 0);
		return IRQ_HANDLED;
	}

handled:
	si_pep_asr_db_enable_int(pep, desc->offset);

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
	.enable_doorbells = si_pep_mod_enable_doorbells,
	.disable_doorbells = si_pep_mod_disable_doorbells,
	.set_soc_state = si_pep_mod_set_soc_state,
	.net_set_val = si_pep_mod_net_set_val,
	.net_get_val = si_pep_mod_net_get_val,
	.net_open = si_pep_mod_net_open,
	.net_release = si_pep_mod_net_release,
	.net_enable_rxint = si_pep_mod_net_enable_rxint,
	.net_disable_rxint = si_pep_mod_net_disable_rxint
};

