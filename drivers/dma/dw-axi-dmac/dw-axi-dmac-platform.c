// SPDX-License-Identifier: GPL-2.0
// (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)

/*
 * Synopsys DesignWare AXI DMA Controller driver.
 *
 * Author: Eugeniy Paltsev <Eugeniy.Paltsev@synopsys.com>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "dw-axi-dmac.h"
#include "../dmaengine.h"
#include "../virt-dma.h"

struct axi_dma_desc *writeback_desc = NULL;
struct axi_dma_desc *shadow_reg_desc = NULL;
void shadow_reg_work(void);
/*
 * The set of bus widths supported by the DMA controller. DW AXI DMAC supports
 * master data bus width up to 512 bits (for both AXI master interfaces), but
 * it depends on IP block configuration.
 */
#define AXI_DMA_BUSWIDTHS		  \
	(DMA_SLAVE_BUSWIDTH_1_BYTE	| \
	DMA_SLAVE_BUSWIDTH_2_BYTES	| \
	DMA_SLAVE_BUSWIDTH_4_BYTES	| \
	DMA_SLAVE_BUSWIDTH_8_BYTES	| \
	DMA_SLAVE_BUSWIDTH_16_BYTES	| \
	DMA_SLAVE_BUSWIDTH_32_BYTES	| \
	DMA_SLAVE_BUSWIDTH_64_BYTES)

static inline void
axi_dma_iowrite32(struct axi_dma_chip *chip, u32 reg, u32 val)
{
	iowrite32(val, chip->regs + reg);
}

static inline u32 axi_dma_ioread32(struct axi_dma_chip *chip, u32 reg)
{
	return ioread32(chip->regs + reg);
}

static inline u64 axi_dma_ioread64(struct axi_dma_chip *chip, u32 reg)
{
	return ioread64(chip->regs + reg);
}

static inline void
axi_chan_iowrite32(struct axi_dma_chan *chan, u32 reg, u32 val)
{
	iowrite32(val, chan->chan_regs + reg);
}

static inline u32 axi_chan_ioread32(struct axi_dma_chan *chan, u32 reg)
{
	return ioread32(chan->chan_regs + reg);
}

static inline u64 axi_chan_ioread64(struct axi_dma_chan *chan, u32 reg)
{
	return ioread64(chan->chan_regs + reg);
}

static inline void
axi_chan_iowrite64(struct axi_dma_chan *chan, u32 reg, u64 val)
{
	/*
	 * We split one 64 bit write for two 32 bit write as some HW doesn't
	 * support 64 bit access.
	 */
	iowrite32(lower_32_bits(val), chan->chan_regs + reg);
	iowrite32(upper_32_bits(val), chan->chan_regs + reg + 4);
}

static inline void axi_chan_config_write(struct axi_dma_chan *chan,
					 struct axi_dma_chan_config *config)
{
	u32 cfg_lo, cfg_hi;

	cfg_lo = (config->dst_multblk_type << CH_CFG_L_DST_MULTBLK_TYPE_POS |
				config->src_multblk_type << CH_CFG_L_SRC_MULTBLK_TYPE_POS | 
				config->wr_uid << CH_CFG_L_WR_UID_POS);
	if (chan->chip->dw->hdata->reg_map_8_channels) {
		cfg_hi = config->tt_fc << CH_CFG_H_TT_FC_POS |
			 config->hs_sel_src << CH_CFG_H_HS_SEL_SRC_POS |
			 config->hs_sel_dst << CH_CFG_H_HS_SEL_DST_POS |
			 config->src_per << CH_CFG_H_SRC_PER_POS |
			 config->dst_per << CH_CFG_H_DST_PER_POS |
			 config->prior << CH_CFG_H_PRIORITY_POS |
			 config->dst_osr_limit << CH_CFG_H_DST_OSR_LIMIT_POS;
	} else {
		cfg_lo |= config->src_per << CH_CFG2_L_SRC_PER_POS |
			  config->dst_per << CH_CFG2_L_DST_PER_POS;
		cfg_hi = config->tt_fc << CH_CFG2_H_TT_FC_POS |
			 config->hs_sel_src << CH_CFG2_H_HS_SEL_SRC_POS |
			 config->hs_sel_dst << CH_CFG2_H_HS_SEL_DST_POS |
			 config->prior << CH_CFG2_H_PRIORITY_POS;
	}
	cfg_hi |= (0xf << CH_CFG_H_SRC_OSR_LIMIT_POS) | (0xf << CH_CFG_H_DST_OSR_LIMIT_POS);
	axi_chan_iowrite32(chan, CH_CFG_L, cfg_lo);
	axi_chan_iowrite32(chan, CH_CFG_H, cfg_hi);
}

static inline void axi_dma_disable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val &= ~DMAC_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_dma_enable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val |= DMAC_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_dma_irq_disable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val &= ~INT_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_dma_irq_enable(struct axi_dma_chip *chip)
{
	u32 val;

	val = axi_dma_ioread32(chip, DMAC_CFG);
	val |= INT_EN_MASK;
	axi_dma_iowrite32(chip, DMAC_CFG, val);
}

static inline void axi_chan_irq_disable(struct axi_dma_chan *chan, u32 irq_mask)
{
	u32 val;

	if (likely(irq_mask == DWAXIDMAC_IRQ_ALL)) {
		axi_chan_iowrite32(chan, CH_INTSTATUS_ENA, DWAXIDMAC_IRQ_NONE);
	} else {
		val = axi_chan_ioread32(chan, CH_INTSTATUS_ENA);
		val &= ~irq_mask;
		axi_chan_iowrite32(chan, CH_INTSTATUS_ENA, val);
	}
}

static inline void axi_chan_irq_set(struct axi_dma_chan *chan, u64 irq_mask)
{
	axi_chan_iowrite64(chan, CH_INTSTATUS_ENA, irq_mask);
}

static inline void axi_chan_irq_sig_set(struct axi_dma_chan *chan, u64 irq_mask)
{
	axi_chan_iowrite64(chan, CH_INTSIGNAL_ENA, irq_mask);
}

static inline void axi_chan_irq_clear(struct axi_dma_chan *chan, u64 irq_mask)
{
	axi_chan_iowrite64(chan, CH_INTCLEAR, irq_mask);
}

static inline u64 axi_chan_irq_read(struct axi_dma_chan *chan)
{
	return axi_chan_ioread64(chan, CH_INTSTATUS);
}

static inline void axi_chan_disable(struct axi_dma_chan *chan)
{
	u32 val;
	u32 offset = chan->id < 16 ? DMAC_CHEN_L : DMAC_CHEN_H;

	val = axi_dma_ioread32(chan->chip, offset);
	val &= ~(BIT(chan->id & 0xf) << DMAC_CHAN_EN_SHIFT);
	if (chan->chip->dw->hdata->reg_map_8_channels)
		val |=   BIT(chan->id) << DMAC_CHAN_EN_WE_SHIFT;
	else
		val |=   BIT(chan->id & 0xf) << DMAC_CHAN_EN2_WE_SHIFT;
	axi_dma_iowrite32(chan->chip, offset, val);
}

static inline void axi_chan_enable(struct axi_dma_chan *chan)
{
	u32 val;
	u32 offset = chan->id < 16 ? DMAC_CHEN_L : DMAC_CHEN_H;

	val = axi_dma_ioread32(chan->chip, offset);
	if (chan->chip->dw->hdata->reg_map_8_channels)
		val |= BIT(chan->id) << DMAC_CHAN_EN_SHIFT |
			BIT(chan->id) << DMAC_CHAN_EN_WE_SHIFT;
	else
		val |= BIT(chan->id & 0xf) << DMAC_CHAN_EN_SHIFT |
			BIT(chan->id & 0xf) << DMAC_CHAN_EN2_WE_SHIFT;
	axi_dma_iowrite32(chan->chip, offset, val);
}

static inline bool axi_chan_is_hw_enable(struct axi_dma_chan *chan)
{
	u32 val;
	u32 offset = chan->id < 16 ? DMAC_CHEN_L : DMAC_CHEN_H;

	val = axi_dma_ioread32(chan->chip, offset);

	return !!(val & (BIT(chan->id & 0xf) << DMAC_CHAN_EN_SHIFT));
}

static void axi_dma_hw_init(struct axi_dma_chip *chip)
{
	int ret;
	u32 i;

	for (i = 0; i < chip->dw->hdata->nr_channels; i++) {
		axi_chan_irq_disable(&chip->dw->chan[i], DWAXIDMAC_IRQ_ALL);
		axi_chan_disable(&chip->dw->chan[i]);
	}
	ret = dma_set_mask_and_coherent(chip->dev, DMA_BIT_MASK(64));
	if (ret)
		dev_warn(chip->dev, "Unable to set coherent mask\n");
}

static u32 axi_chan_get_xfer_width(struct axi_dma_chan *chan, dma_addr_t src,
				   dma_addr_t dst, size_t len)
{
	u32 max_width = chan->chip->dw->hdata->m_data_width;

	return __ffs(src | dst | len | BIT(max_width));
}

static inline const char *axi_chan_name(struct axi_dma_chan *chan)
{
	return dma_chan_name(&chan->vc.chan);
}

static struct axi_dma_desc *axi_desc_alloc(u32 num)
{
	struct axi_dma_desc *desc;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->hw_desc = kcalloc(num, sizeof(*desc->hw_desc), GFP_NOWAIT);
	if (!desc->hw_desc) {
		kfree(desc);
		return NULL;
	}

	INIT_LIST_HEAD(&desc->node);

	return desc;
}

static struct axi_dma_lli *axi_desc_get(struct axi_dma_chan *chan,
					dma_addr_t *addr)
{
	struct axi_dma_lli *lli;
	dma_addr_t phys;

	lli = dma_pool_zalloc(chan->desc_pool, GFP_NOWAIT, &phys);
	if (unlikely(!lli)) {
		dev_err(chan2dev(chan), "%s: not enough descriptors available\n",
			axi_chan_name(chan));
		return NULL;
	}

	atomic_inc(&chan->descs_allocated);
	*addr = phys;

	return lli;
}

static void axi_desc_put(struct axi_dma_desc *desc)
{
	struct axi_dma_chan *chan = desc->chan;
	int count = atomic_read(&chan->descs_allocated);
	struct axi_dma_hw_desc *hw_desc;
	int descs_put;

	for (descs_put = 0; descs_put < count; descs_put++) {
		hw_desc = &desc->hw_desc[descs_put];
		dma_pool_free(chan->desc_pool, hw_desc->lli, hw_desc->llp);
	}

	kfree(desc->hw_desc);
	kfree(desc);
	atomic_sub(descs_put, &chan->descs_allocated);
	dev_vdbg(chan2dev(chan), "%s: %d descs put, %d still allocated\n",
		axi_chan_name(chan), descs_put,
		atomic_read(&chan->descs_allocated));
}

static void axi_vdesc_put(struct axi_dma_desc *desc)
{
	struct axi_dma_chan *chan = desc->chan;
	struct axi_dma_hw_desc *hw_desc;

	hw_desc = &desc->hw_desc[0];
	dma_pool_free(chan->desc_pool, hw_desc->lli, hw_desc->llp);

	atomic_sub(1, &chan->descs_allocated);
	dev_dbg(chan2dev(chan), "%s: desc %#llx deleted\n", axi_chan_name(chan), hw_desc->lli);

	list_del(&desc->node);
	kfree(desc->hw_desc);
	kfree(desc);
}

static void vchan_desc_put(struct virt_dma_desc *vdesc)
{
	struct axi_dma_desc *desc = vd_to_axi_desc(vdesc);
	struct axi_dma_chan *chan = NULL;

	if(!desc)
		return;

	chan = desc->chan;

	if(chan->is_video_mode) {
		axi_vdesc_put(desc);
	} else {
		axi_desc_put(desc);
	}
}

static enum dma_status
dma_chan_tx_status(struct dma_chan *dchan, dma_cookie_t cookie,
		  struct dma_tx_state *txstate)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	u32 completed_length;
	unsigned long flags;
	u32 completed_blocks;
	size_t bytes = 0;
	u32 length;
	u32 len;

	status = dma_cookie_status(dchan, cookie, txstate);
	if (status == DMA_COMPLETE || !txstate)
		return status;

	spin_lock_irqsave(&chan->vc.lock, flags);

	vdesc = vchan_find_desc(&chan->vc, cookie);
	if (vdesc) {
		length = vd_to_axi_desc(vdesc)->length;
		completed_blocks = vd_to_axi_desc(vdesc)->completed_blocks;
		len = vd_to_axi_desc(vdesc)->hw_desc[0].len;
		completed_length = completed_blocks * len;
		bytes = length - completed_length;
	}

	spin_unlock_irqrestore(&chan->vc.lock, flags);
	dma_set_residue(txstate, bytes);

	return status;
}

static void write_desc_llp(struct axi_dma_hw_desc *desc, dma_addr_t adr)
{
	desc->lli->llp = cpu_to_le64(adr);
}

static void write_chan_llp(struct axi_dma_chan *chan, dma_addr_t adr)
{
	axi_chan_iowrite64(chan, CH_LLP, adr);
}

static void dw_axi_dma_set_byte_halfword(struct axi_dma_chan *chan, bool set)
{
	u32 offset = DMAC_APB_BYTE_WR_CH_EN;
	u32 reg_width, val;

	if (!chan->chip->apb_regs) {
		dev_dbg(chan->chip->dev, "apb_regs not initialized\n");
		return;
	}

	reg_width = __ffs(chan->config.dst_addr_width);
	if (reg_width == DWAXIDMAC_TRANS_WIDTH_16)
		offset = DMAC_APB_HALFWORD_WR_CH_EN;

	val = ioread32(chan->chip->apb_regs + offset);

	if (set)
		val |= BIT(chan->id);
	else
		val &= ~BIT(chan->id);

	iowrite32(val, chan->chip->apb_regs + offset);
}

static void mark_hwdesc_done(struct axi_dma_desc *dma_desc)
{
	int i = 0;
	u32 val;

	do {
		if(!dma_desc->hw_desc[i].lli) {
			pr_err("NULL LLI POINTER\n");
			return;
		}
		val = le32_to_cpu(dma_desc->hw_desc[i].lli->ctl_hi);
		if(dma_desc->hw_desc[i].lli->reserved_lo != DMAC_DESC_COMPLETED) {
			dma_desc->hw_desc[i].lli->reserved_lo = DMAC_DESC_COMPLETED;
			return;
		}
		i++;
	} while(!(val & CH_CTL_H_LLI_LAST));
}

static int get_next_hwdesc_number(struct axi_dma_desc *dma_desc)
{
	int i = 0;
	u32 val;

	do {
		if(!dma_desc->hw_desc[i].lli) {
			pr_err("NULL LLI POINTER\n");
			return -1;
		}
		val = le32_to_cpu(dma_desc->hw_desc[i].lli->ctl_hi);
		if(dma_desc->hw_desc[i].lli->reserved_lo != DMAC_DESC_COMPLETED)
			return i;
		i++;
	} while(!(val & CH_CTL_H_LLI_LAST));

	return -1;
}

static void axi_chan_dump_lli(struct axi_dma_chan *chan,
			      struct axi_dma_hw_desc *desc)
{
	if (!desc->lli) {
		dev_err(dchan2dev(&chan->vc.chan), "LLI is empty\n");
		return;
	}

	dev_info(dchan2dev(&chan->vc.chan), "SAR: 0x%llx DAR: 0x%llx LLP: 0x%llx BTS 0x%x "
		"CTL: 0x%x:%08x SSTAT: %#x DSTAT: %#x STATUS: %#x:%08x",
		le64_to_cpu(desc->lli->sar), le64_to_cpu(desc->lli->dar), le64_to_cpu(desc->lli->llp),
		le32_to_cpu(desc->lli->block_ts_lo), le32_to_cpu(desc->lli->ctl_hi),
		le32_to_cpu(desc->lli->ctl_lo), le32_to_cpu(desc->lli->sstat),
		le32_to_cpu(desc->lli->dstat), le32_to_cpu(desc->lli->status_hi),
		le32_to_cpu(desc->lli->status_lo));
}

static void axi_chan_list_dump_lli(struct axi_dma_chan *chan,
				   struct axi_dma_desc *desc_head)
{
	int count = atomic_read(&chan->descs_allocated);
	int i;

	for (i = 0; i < count; i++)
		axi_chan_dump_lli(chan, &desc_head->hw_desc[i]);
}

/* Called in chan locked context */
static void axi_chan_block_xfer_start(struct axi_dma_chan *chan,
				      struct axi_dma_desc *first)
{
	u32 priority = chan->chip->dw->hdata->priority[chan->id];
	struct axi_dma_chan_config config = {};
	u64 irq_mask;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */
	u32 i;
	lms = chan->chip->dw->hdata->lms_axi_master;

	if (unlikely(axi_chan_is_hw_enable(chan))) {
		dev_dbg(chan2dev(chan), "%s is non-idle!\n",
			axi_chan_name(chan));

		return;
	}

	axi_dma_enable(chan->chip);
	axi_dma_irq_enable(chan->chip);
	if (chan->is_video_mode) {
		dev_info(chan->chip->dev, "video mode setting SSTATA");
		axi_chan_iowrite64(chan, CH_SSTATAR, (chan->id * CH_LLI_SRC_START_ADDR) +
			CH_SSTATAR_START_ADDR);
	}

	config.dst_multblk_type = chan->chip->dw->hdata->xfer_mode;
	config.src_multblk_type = chan->chip->dw->hdata->xfer_mode;
	config.tt_fc = DWAXIDMAC_TT_FC_MEM_TO_MEM_DMAC;
	config.prior = priority;
	config.hs_sel_dst = DWAXIDMAC_HS_SEL_HW;
	config.hs_sel_src = DWAXIDMAC_HS_SEL_HW;
	switch (chan->direction) {
	case DMA_MEM_TO_DEV:
		dw_axi_dma_set_byte_halfword(chan, true);
		config.tt_fc = chan->config.device_fc ?
				DWAXIDMAC_TT_FC_MEM_TO_PER_DST :
				DWAXIDMAC_TT_FC_MEM_TO_PER_DMAC;
		if (chan->chip->apb_regs)
			config.dst_per = chan->id;
		else
			config.dst_per = chan->hw_handshake_num;
		break;
	case DMA_DEV_TO_MEM:
		config.tt_fc = chan->config.device_fc ?
				DWAXIDMAC_TT_FC_PER_TO_MEM_SRC :
				DWAXIDMAC_TT_FC_PER_TO_MEM_DMAC;
		if (chan->chip->apb_regs)
			config.src_per = chan->id;
		else
			config.src_per = chan->hw_handshake_num;

		if (chan->is_video_mode) {
			dev_info(chan->chip->dev, "hs sel tt func video mode");
			config.hs_sel_dst = DWAXIDMAC_HS_SEL_SW;
			config.hs_sel_src = DWAXIDMAC_HS_SEL_HW;
			config.tt_fc = DWAXIDMAC_TT_FC_PER_TO_MEM_SRC;
			config.src_per = chan->id;
			config.wr_uid = CH_CFG_L_WR_UID_VAL;
			config.dst_osr_limit = CH_CFG_H_DST_OST_LIMIT_VAL; 
		}
		break;
	default:
		break;
	}

	axi_chan_config_write(chan, &config);

	if (chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_CONTIGUOUS) {
		i = get_next_hwdesc_number(first);
		if(i < 0) {
			dev_err(chan->chip->dev, "No blocks for current descriptor\n");
			return;
		}
		axi_chan_iowrite64(chan, CH_BLOCK_TS, first->hw_desc[i].lli->block_ts_lo);
		axi_chan_iowrite32(chan, CH_CTL_L, first->hw_desc[i].lli->ctl_lo);
		axi_chan_iowrite32(chan, CH_CTL_H, first->hw_desc[i].lli->ctl_hi);
		axi_chan_iowrite64(chan, CH_SAR, first->hw_desc[i].lli->sar);
		axi_chan_iowrite64(chan, CH_DAR, first->hw_desc[i].lli->dar);
		irq_mask = DWAXIDMAC_IRQ_BLOCK_TRF | DWAXIDMAC_IRQ_DMA_TRF | DWAXIDMAC_IRQ_ALL_ERR;
	} else {
		write_chan_llp(chan, first->hw_desc[0].llp | lms);
		if (chan->is_video_mode)
			irq_mask = DWAXIDMAC_IRQ_ALL & (~DWAXIDMAC_IRQ_SRC_TRAN);
		else
			irq_mask = DWAXIDMAC_IRQ_DMA_TRF | DWAXIDMAC_IRQ_ALL_ERR;
	}

	axi_chan_irq_sig_set(chan, irq_mask);

	/* Generate 'suspend' status but don't generate interrupt */
	if (chan->is_video_mode)
		irq_mask &= (~DWAXIDMAC_IRQ_DST_TRAN);
	else
		irq_mask |= DWAXIDMAC_IRQ_SUSPENDED;

	axi_chan_irq_set(chan, irq_mask);

	if (chan->chip->dw->hdata->xfer_mode != DWAXIDMAC_MBLK_TYPE_SHADOW_REG)
                axi_chan_enable(chan);

        if (chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_SHADOW_REG)
                shadow_reg_work();
}

static void axi_chan_start_first_queued(struct axi_dma_chan *chan)
{
	struct axi_dma_desc *desc;
	struct virt_dma_desc *vd;

	vd = vchan_next_desc(&chan->vc);
	if (!vd)
		return;

	desc = vd_to_axi_desc(vd);
	dev_vdbg(chan2dev(chan), "%s: started %u\n", axi_chan_name(chan),
		vd->tx.cookie);
	axi_chan_block_xfer_start(chan, desc);
}

static void dma_chan_issue_pending(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc))
		axi_chan_start_first_queued(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void dw_axi_dma_synchronize(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	vchan_synchronize(&chan->vc);
}

static int dma_chan_alloc_chan_resources(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	/* ASSERT: channel is idle */
	if (axi_chan_is_hw_enable(chan)) {
		dev_err(chan2dev(chan), "%s is non-idle!\n",
			axi_chan_name(chan));
		return -EBUSY;
	}

	/* LLI address must be aligned to a 64-byte boundary */
	chan->desc_pool = dma_pool_create(dev_name(chan2dev(chan)),
					  chan->chip->dev,
					  sizeof(struct axi_dma_lli),
					  64, 0);
	if (!chan->desc_pool) {
		dev_err(chan2dev(chan), "No memory for descriptors\n");
		return -ENOMEM;
	}
	dev_vdbg(dchan2dev(dchan), "%s: allocating\n", axi_chan_name(chan));

	pm_runtime_get(chan->chip->dev);

	return 0;
}

static void dma_chan_free_chan_resources(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	/* ASSERT: channel is idle */
	if (axi_chan_is_hw_enable(chan))
		dev_err(dchan2dev(dchan), "%s is non-idle!\n",
			axi_chan_name(chan));

	axi_chan_disable(chan);
	axi_chan_irq_disable(chan, DWAXIDMAC_IRQ_ALL);

	vchan_free_chan_resources(&chan->vc);

	dma_pool_destroy(chan->desc_pool);
	chan->desc_pool = NULL;
	dev_vdbg(dchan2dev(dchan),
		 "%s: free resources, descriptor still allocated: %u\n",
		 axi_chan_name(chan), atomic_read(&chan->descs_allocated));

	pm_runtime_put(chan->chip->dev);
}

static void dw_axi_dma_set_hw_channel(struct axi_dma_chan *chan, bool set)
{
	struct axi_dma_chip *chip = chan->chip;
	unsigned long reg_value, val;

	if (!chip->apb_regs) {
		dev_vdbg(chip->dev, "apb_regs not initialized\n");
		return;
	}

	/*
	 * An unused DMA channel has a default value of 0x3F.
	 * Lock the DMA channel by assign a handshake number to the channel.
	 * Unlock the DMA channel by assign 0x3F to the channel.
	 */
	if (set)
		val = chan->hw_handshake_num;
	else
		val = UNUSED_CHANNEL;

	reg_value = lo_hi_readq(chip->apb_regs + DMAC_APB_HW_HS_SEL_0);

	/* Channel is already allocated, set handshake as per channel ID */
	/* 64 bit write should handle for 8 channels */

	reg_value &= ~(DMA_APB_HS_SEL_MASK <<
			(chan->id * DMA_APB_HS_SEL_BIT_SIZE));
	reg_value |= (val << (chan->id * DMA_APB_HS_SEL_BIT_SIZE));
	lo_hi_writeq(reg_value, chip->apb_regs + DMAC_APB_HW_HS_SEL_0);

	return;
}

/*
 * If DW_axi_dmac sees CHx_CTL.ShadowReg_Or_LLI_Last bit of the fetched LLI
 * as 1, it understands that the current block is the final block in the
 * transfer and completes the DMA transfer operation at the end of current
 * block transfer.
 */
static void set_desc_last(struct axi_dma_hw_desc *desc)
{
	u32 val;

	val = le32_to_cpu(desc->lli->ctl_hi);
	val |= CH_CTL_H_LLI_LAST;
	desc->lli->ctl_hi = cpu_to_le32(val);
}

static void unset_desc_last(struct axi_dma_hw_desc *desc)
{
	u32 val;

	val = le32_to_cpu(desc->lli->ctl_hi);
	val &= ~CH_CTL_H_LLI_LAST;
	desc->lli->ctl_hi = cpu_to_le32(val);
}

static void write_desc_sar(struct axi_dma_hw_desc *desc, dma_addr_t adr)
{
	desc->lli->sar = cpu_to_le64(adr);
}

static void write_desc_dar(struct axi_dma_hw_desc *desc, dma_addr_t adr)
{
	desc->lli->dar = cpu_to_le64(adr);
}

static void set_desc_src_master(struct axi_dma_hw_desc *desc)
{
	u32 val;

	/* Select AXI0 for source master */
	val = le32_to_cpu(desc->lli->ctl_lo);
	val &= ~CH_CTL_L_SRC_MAST;
	desc->lli->ctl_lo = cpu_to_le32(val);
}

static void set_desc_dest_master(struct axi_dma_hw_desc *hw_desc,
				 struct axi_dma_desc *desc)
{
	u32 val;

	/* Select AXI1 for source master if available */
	val = le32_to_cpu(hw_desc->lli->ctl_lo);
	if (desc->chan->chip->dw->hdata->nr_masters > 1)
		val |= CH_CTL_L_DST_MAST;
	else
		val &= ~CH_CTL_L_DST_MAST;

	hw_desc->lli->ctl_lo = cpu_to_le32(val);
}

static int dw_axi_dma_set_hw_desc(struct axi_dma_chan *chan,
				  struct axi_dma_hw_desc *hw_desc,
				  dma_addr_t mem_addr, size_t len)
{
	unsigned int data_width = BIT(chan->chip->dw->hdata->m_data_width);
	unsigned int reg_width;
	unsigned int mem_width;
	dma_addr_t device_addr;
	size_t axi_block_ts;
	size_t block_ts;
	u32 ctllo, ctlhi;
	u32 burst_len;

	axi_block_ts = chan->chip->dw->hdata->block_size[chan->id];

	mem_width = __ffs(data_width | mem_addr | len);
	if (mem_width > DWAXIDMAC_TRANS_WIDTH_32)
		mem_width = DWAXIDMAC_TRANS_WIDTH_32;

	if (!IS_ALIGNED(mem_addr, 4)) {
		dev_err(chan->chip->dev, "invalid buffer alignment\n");
		return -EINVAL;
	}

	switch (chan->direction) {
	case DMA_MEM_TO_DEV:
		reg_width = __ffs(chan->config.dst_addr_width);
		device_addr = chan->config.dst_addr & chan->chip->dw->hdata->dev_addr_mask;
		ctllo = reg_width << CH_CTL_L_DST_WIDTH_POS |
			mem_width << CH_CTL_L_SRC_WIDTH_POS |
			DWAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_DST_INC_POS |
			DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS;
		block_ts = len >> mem_width;
		break;
	case DMA_DEV_TO_MEM:
		reg_width = __ffs(chan->config.src_addr_width);
		device_addr = chan->config.src_addr & chan->chip->dw->hdata->dev_addr_mask;
		ctllo = reg_width << CH_CTL_L_SRC_WIDTH_POS |
			mem_width << CH_CTL_L_DST_WIDTH_POS |
			DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
			DWAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_SRC_INC_POS;
		block_ts = len >> reg_width;
		break;
	default:
		return -EINVAL;
	}

	if (block_ts > axi_block_ts)
		return -EINVAL;

	hw_desc->lli = axi_desc_get(chan, &hw_desc->llp);
	if (unlikely(!hw_desc->lli))
		return -ENOMEM;

	ctlhi = CH_CTL_H_LLI_VALID;

	if (chan->chip->dw->hdata->restrict_axi_burst_len) {
		burst_len = chan->chip->dw->hdata->axi_rw_burst_len;
		ctlhi |= CH_CTL_H_ARLEN_EN | CH_CTL_H_AWLEN_EN |
			 burst_len << CH_CTL_H_ARLEN_POS |
			 burst_len << CH_CTL_H_AWLEN_POS;
	}

	hw_desc->lli->ctl_hi = cpu_to_le32(ctlhi);

	if (chan->direction == DMA_MEM_TO_DEV) {
		write_desc_sar(hw_desc, mem_addr);
		write_desc_dar(hw_desc, device_addr);
	} else {
		write_desc_sar(hw_desc, device_addr);
		write_desc_dar(hw_desc, mem_addr);
	}

	hw_desc->lli->block_ts_lo = cpu_to_le32(block_ts - 1);

	ctllo |= DWAXIDMAC_BURST_TRANS_LEN_32 << CH_CTL_L_DST_MSIZE_POS |
		 DWAXIDMAC_BURST_TRANS_LEN_8 << CH_CTL_L_SRC_MSIZE_POS;
	hw_desc->lli->ctl_lo = cpu_to_le32(ctllo);

	set_desc_src_master(hw_desc);

	hw_desc->len = len;
	return 0;
}

static size_t calculate_block_len(struct axi_dma_chan *chan,
				  dma_addr_t dma_addr, size_t buf_len,
				  enum dma_transfer_direction direction)
{
	u32 data_width, reg_width, mem_width;
	size_t axi_block_ts, block_len;

	axi_block_ts = chan->chip->dw->hdata->block_size[chan->id];

	switch (direction) {
	case DMA_MEM_TO_DEV:
		data_width = BIT(chan->chip->dw->hdata->m_data_width);
		mem_width = __ffs(data_width | dma_addr | buf_len);
		if (mem_width > DWAXIDMAC_TRANS_WIDTH_32)
			mem_width = DWAXIDMAC_TRANS_WIDTH_32;

		block_len = axi_block_ts << mem_width;
		break;
	case DMA_DEV_TO_MEM:
		reg_width = __ffs(chan->config.src_addr_width);
		block_len = axi_block_ts << reg_width;
		break;
	default:
		block_len = 0;
	}

	return block_len;
}

static struct dma_async_tx_descriptor *
dw_axi_dma_chan_prep_interleaved(struct dma_chan *dchan,
									struct dma_interleaved_template *xt,
									unsigned long flags)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct axi_dma_hw_desc *hw_desc = NULL;
	struct axi_dma_desc *desc = NULL;

	unsigned int i;
	int status;
	u64 llp = 0;
	unsigned int iter = 0;

	dma_addr_t device_addr;
	size_t axi_block_ts;
	size_t block_ts;
	u32 ctllo, ctlhi;
	u32 burst_len;
	struct axi_dma_desc *desc_iter = NULL;
	struct axi_dma_desc *prev_desc = NULL;

	if (!is_slave_direction(xt->dir)) {
		dev_err(chan->chip->dev, "slave direction is wrong\n");
		return NULL;
	}

	if (!xt->numf || !xt->sgl[0].size) {
		dev_err(chan->chip->dev, "Failed in numf %u and size %u\n", xt->numf, xt->sgl[0].size);
		return NULL;
	}

	if(xt->frame_size != 1) {
		dev_err(chan->chip->dev, "Frame size is not 1\n");
		return NULL;
	}

	desc = axi_desc_alloc(1);
	if (unlikely(!desc)) {
		dev_err(chan->chip->dev, "axi desc alloc failed\n");
		goto err_desc_get;
	}

	if (chan->desc == NULL) {
		dev_info(chan->chip->dev, "First time init of descriptor list\n");
		chan->desc = desc;
	}

	chan->direction = xt->dir;
	desc->chan = chan;
	desc->length = 0;
	chan->is_video_mode = true;
	hw_desc = &desc->hw_desc[0];

	hw_desc->lli = axi_desc_get(chan, &hw_desc->llp);
	if (unlikely(!hw_desc->lli))
		return -ENOMEM;

	write_desc_sar(hw_desc, (chan->id) * CH_LLI_SRC_START_ADDR);
	write_desc_dar(hw_desc, xt->dst_start);

	block_ts = (xt->sgl[0].size + FRAME_METADATA_SIZE)/8 -1;
	hw_desc->lli->block_ts_lo = cpu_to_le32(block_ts);

	ctllo =	CH_CTL_L_LAST_WRITE_EN |
			DWAXIDMAC_BURST_TRANS_LEN_32 << CH_CTL_L_DST_MSIZE_POS |
			DWAXIDMAC_BURST_TRANS_LEN_8 << CH_CTL_L_SRC_MSIZE_POS |
 			chan->chip->dw->hdata->m_data_width << CH_CTL_L_SRC_WIDTH_POS |
			chan->chip->dw->hdata->m_data_width << CH_CTL_L_DST_WIDTH_POS |
			DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
			DWAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_SRC_INC_POS |
			DWAXIDMAC_CH_CTL_L_MAST_2_INTF << CH_CTL_L_DST_MAST_POS |
			DWAXIDMAC_CH_CTL_L_MAST_1_INTF << CH_CTL_L_SRC_MAST_POS;

	hw_desc->lli->ctl_lo = cpu_to_le32(ctllo);

	ctlhi = CH_CTL_H_LLI_VALID |
			CH_CTL_H_IOC_BLK_TFR_EN |	
			CH_CTL_H_SRC_STAT_EN |
			(DWAXIDMAC_ARWLEN_32 << CH_CTL_H_AWLEN_POS) |
			CH_CTL_H_AWLEN_EN |
			(DWAXIDMAC_ARWLEN_8 << CH_CTL_H_ARLEN_POS) |
			CH_CTL_H_ARLEN_EN;
 
	hw_desc->lli->ctl_hi = cpu_to_le32(ctlhi);
	desc->length += hw_desc->len;

	//axi_chan_dump_lli(chan, hw_desc);
	list_add_tail(&desc->node, &chan->desc_list);

	list_for_each_entry(desc_iter, &chan->desc_list, node) {
		if(prev_desc != NULL) {
			llp = desc_iter->hw_desc->llp;
			write_desc_llp(prev_desc->hw_desc, llp);
		}
		prev_desc = desc_iter;
	}

	dw_axi_dma_set_hw_channel(chan, true);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_desc_get:
	if (desc)
		axi_desc_put(desc);

	return NULL;
}

static struct dma_async_tx_descriptor *
dw_axi_dma_chan_prep_cyclic(struct dma_chan *dchan, dma_addr_t dma_addr,
			    size_t buf_len, size_t period_len,
			    enum dma_transfer_direction direction,
			    unsigned long flags)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct axi_dma_hw_desc *hw_desc = NULL;
	struct axi_dma_desc *desc = NULL;
	dma_addr_t src_addr = dma_addr;
	u32 num_periods, num_segments;
	size_t axi_block_len;
	u32 total_segments;
	u32 segment_len;
	unsigned int i;
	int status;
	u64 llp = 0;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	num_periods = buf_len / period_len;

	axi_block_len = calculate_block_len(chan, dma_addr, buf_len, direction);
	if (axi_block_len == 0)
		return NULL;

	num_segments = DIV_ROUND_UP(period_len, axi_block_len);
	segment_len = DIV_ROUND_UP(period_len, num_segments);

	total_segments = num_periods * num_segments;

	desc = axi_desc_alloc(total_segments);
	if (unlikely(!desc))
		goto err_desc_get;

	chan->direction = direction;
	desc->chan = chan;
	chan->cyclic = true;
	desc->length = 0;
	desc->period_len = period_len;

	for (i = 0; i < total_segments; i++) {
		hw_desc = &desc->hw_desc[i];

		status = dw_axi_dma_set_hw_desc(chan, hw_desc, src_addr,
						segment_len);
		if (status < 0)
			goto err_desc_get;

		desc->length += hw_desc->len;
		/* Set end-of-link to the linked descriptor, so that cyclic
		 * callback function can be triggered during interrupt.
		 */
		set_desc_last(hw_desc);

		src_addr += segment_len;
	}

	llp = desc->hw_desc[0].llp;

	/* Managed transfer list */
	do {
		hw_desc = &desc->hw_desc[--total_segments];
		write_desc_llp(hw_desc, llp | lms);
		llp = hw_desc->llp;
	} while (total_segments);

	dw_axi_dma_set_hw_channel(chan, true);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_desc_get:
	if (desc)
		axi_desc_put(desc);

	return NULL;
}

static struct dma_async_tx_descriptor *
dw_axi_dma_chan_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
			      unsigned int sg_len,
			      enum dma_transfer_direction direction,
			      unsigned long flags, void *context)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct axi_dma_hw_desc *hw_desc = NULL;
	struct axi_dma_desc *desc = NULL;
	u32 num_segments, segment_len;
	unsigned int loop = 0;
	struct scatterlist *sg;
	size_t axi_block_len;
	u32 len, num_sgs = 0;
	unsigned int i;
	dma_addr_t mem;
	int status;
	u64 llp = 0;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	if (unlikely(!is_slave_direction(direction) || !sg_len))
		return NULL;

	mem = sg_dma_address(sgl);
	len = sg_dma_len(sgl);

	axi_block_len = calculate_block_len(chan, mem, len, direction);
	if (axi_block_len == 0)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i)
		num_sgs += DIV_ROUND_UP(sg_dma_len(sg), axi_block_len);

	desc = axi_desc_alloc(num_sgs);
	if (unlikely(!desc))
		goto err_desc_get;

	desc->chan = chan;
	desc->length = 0;
	chan->direction = direction;

	for_each_sg(sgl, sg, sg_len, i) {
		mem = sg_dma_address(sg);
		len = sg_dma_len(sg);
		num_segments = DIV_ROUND_UP(sg_dma_len(sg), axi_block_len);
		segment_len = DIV_ROUND_UP(sg_dma_len(sg), num_segments);

		do {
			hw_desc = &desc->hw_desc[loop++];
			status = dw_axi_dma_set_hw_desc(chan, hw_desc, mem, segment_len);
			if (status < 0)
				goto err_desc_get;

			desc->length += hw_desc->len;
			len -= segment_len;
			mem += segment_len;
		} while (len >= segment_len);
	}

	/* Set end-of-link to the last link descriptor of list */
	set_desc_last(&desc->hw_desc[num_sgs - 1]);

	/* Managed transfer list */
	do {
		hw_desc = &desc->hw_desc[--num_sgs];
		write_desc_llp(hw_desc, llp | lms);
		llp = hw_desc->llp;
	} while (num_sgs);

	dw_axi_dma_set_hw_channel(chan, true);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_desc_get:
	if (desc)
		axi_desc_put(desc);

	return NULL;
}

void shadow_reg_work(void)
{
        struct axi_dma_hw_desc *hw_desc = NULL;
        u32 num = 0;
        hw_desc = &shadow_reg_desc->hw_desc[num];
        while(!(hw_desc->lli->ctl_hi & CH_CTL_H_LLI_LAST)) {
                axi_chan_iowrite64(shadow_reg_desc->chan, CH_BLOCK_TS, hw_desc->lli->block_ts_lo);
                axi_chan_iowrite64(shadow_reg_desc->chan, CH_SAR, hw_desc->lli->sar);
                axi_chan_iowrite64(shadow_reg_desc->chan, CH_DAR, hw_desc->lli->dar);
                axi_chan_iowrite32(shadow_reg_desc->chan, CH_CTL_L, hw_desc->lli->ctl_lo);
                axi_chan_iowrite32(shadow_reg_desc->chan, CH_CTL_H, hw_desc->lli->ctl_hi);
                /*in shadow reg mode channel will be automatically disabled by DMAC after every block transfer
                 * hence enable the channel
                 */
                axi_chan_enable(shadow_reg_desc->chan);
                axi_chan_iowrite32(shadow_reg_desc->chan, CH_BLK_TFR_RESUMEREQ, 0x1);

                while((axi_chan_ioread32(shadow_reg_desc->chan, CH_CTL_H) & CH_CTL_H_LLI_VALID) !=0);
                num++;
                hw_desc = &shadow_reg_desc->hw_desc[num];
        }
        /* complete last block */
        axi_chan_iowrite64(shadow_reg_desc->chan, CH_BLOCK_TS, hw_desc->lli->block_ts_lo);
        axi_chan_iowrite64(shadow_reg_desc->chan, CH_SAR, hw_desc->lli->sar);
        axi_chan_iowrite64(shadow_reg_desc->chan, CH_DAR, hw_desc->lli->dar);
        axi_chan_iowrite32(shadow_reg_desc->chan, CH_CTL_L, hw_desc->lli->ctl_lo);
        axi_chan_iowrite32(shadow_reg_desc->chan, CH_CTL_H, hw_desc->lli->ctl_hi);
        /*in shadow reg mode channel will be automatically disabled by DMAC after every block transfer
         * hence enable the channel
         */
        axi_chan_enable(shadow_reg_desc->chan);
        axi_chan_iowrite32(shadow_reg_desc->chan, CH_BLK_TFR_RESUMEREQ, 0x1);

        while((axi_chan_ioread32(shadow_reg_desc->chan, CH_CTL_H) & CH_CTL_H_LLI_VALID) !=0);
}

static struct dma_async_tx_descriptor *
dma_chan_prep_dma_memcpy(struct dma_chan *dchan, dma_addr_t dst_adr,
			 dma_addr_t src_adr, size_t len, unsigned long flags)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	size_t block_ts, max_block_ts, xfer_len;
	struct axi_dma_hw_desc *hw_desc = NULL;
	struct axi_dma_desc *desc = NULL;
	u32 xfer_width, reg, num;
	u64 llp = 0;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	dev_dbg(chan2dev(chan), "%s: memcpy: src: %pad dst: %pad length: %zd flags: %#lx",
		axi_chan_name(chan), &src_adr, &dst_adr, len, flags);

	max_block_ts = chan->chip->dw->hdata->block_size[chan->id];
	lms = chan->chip->dw->hdata->lms_axi_master;
	xfer_width = axi_chan_get_xfer_width(chan, src_adr, dst_adr, len);
	num = DIV_ROUND_UP(len, max_block_ts << xfer_width);
	desc = axi_desc_alloc(num);
	if (unlikely(!desc))
		goto err_desc_get;

	desc->chan = chan;
	num = 0;
	desc->length = 0;
	while (len) {
		xfer_len = len;

		hw_desc = &desc->hw_desc[num];
		/*
		 * Take care for the alignment.
		 * Actually source and destination widths can be different, but
		 * make them same to be simpler.
		 */
		if(chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_CONTIGUOUS)
			xfer_width = chan->chip->dw->hdata->m_data_width;
		else
			xfer_width = axi_chan_get_xfer_width(chan, src_adr, dst_adr, xfer_len);

		/*
		 * block_ts indicates the total number of data of width
		 * to be transferred in a DMA block transfer.
		 * BLOCK_TS register should be set to block_ts - 1
		 */
		block_ts = xfer_len >> xfer_width;
		if (block_ts > max_block_ts) {
			block_ts = max_block_ts;
			xfer_len = max_block_ts << xfer_width;
		}

		hw_desc->lli = axi_desc_get(chan, &hw_desc->llp);
		if (unlikely(!hw_desc->lli))
			goto err_desc_get;

		write_desc_sar(hw_desc, src_adr);
		write_desc_dar(hw_desc, dst_adr);
		hw_desc->lli->block_ts_lo = cpu_to_le32(block_ts - 1);

		reg = (chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_CONTIGUOUS) ?
			0 : CH_CTL_H_LLI_VALID;
		if (chan->chip->dw->hdata->restrict_axi_burst_len) {
			u32 burst_len = chan->chip->dw->hdata->axi_rw_burst_len;

			reg |= (CH_CTL_H_ARLEN_EN |
				burst_len << CH_CTL_H_ARLEN_POS |
				CH_CTL_H_AWLEN_EN |
				burst_len << CH_CTL_H_AWLEN_POS);
		}
		hw_desc->lli->ctl_hi = cpu_to_le32(reg);

		if(chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_CONTIGUOUS) {
			reg = (xfer_width << CH_CTL_L_DST_WIDTH_POS  |
				xfer_width << CH_CTL_L_SRC_WIDTH_POS);
			hw_desc->lli->reserved_lo = DMAC_DESC_SUBMITTED;
			hw_desc->lli->reserved_hi = 0;
		} else
			reg = (DWAXIDMAC_BURST_TRANS_LEN_256 << CH_CTL_L_DST_MSIZE_POS |
			       DWAXIDMAC_BURST_TRANS_LEN_256 << CH_CTL_L_SRC_MSIZE_POS |
			       xfer_width << CH_CTL_L_DST_WIDTH_POS |
			       xfer_width << CH_CTL_L_SRC_WIDTH_POS |
			       DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
			       DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS);
		hw_desc->lli->ctl_lo = cpu_to_le32(reg);

		set_desc_src_master(hw_desc);
		set_desc_dest_master(hw_desc, desc);

		hw_desc->len = xfer_len;
		desc->length += hw_desc->len;
		/* update the length and addresses for the next loop cycle */
		len -= xfer_len;
		dst_adr += xfer_len;
		src_adr += xfer_len;
		num++;
	}

	/* Set end-of-link to the last link descriptor of list */
	set_desc_last(&desc->hw_desc[num - 1]);
	/* Managed transfer list */
	do {
		hw_desc = &desc->hw_desc[--num];
		write_desc_llp(hw_desc, llp | lms);
		llp = hw_desc->llp;
	} while (num);

	if(chan->chip->dw->hdata->rdwr_back_feature)
                writeback_desc = desc;
	if(chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_SHADOW_REG) {
                shadow_reg_desc = desc;
        }
	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_desc_get:
	if (desc)
		axi_desc_put(desc);
	return NULL;
}

static int dw_axi_dma_chan_slave_config(struct dma_chan *dchan,
					struct dma_slave_config *config)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	memcpy(&chan->config, config, sizeof(*config));

	return 0;
}

static noinline void axi_chan_handle_err(struct axi_dma_chan *chan, u32 status)
{
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	axi_chan_disable(chan);

	/* The bad descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	if (!vd) {
		dev_err(chan2dev(chan), "BUG: %s, IRQ with no descriptors\n",
			axi_chan_name(chan));
		goto out;
	}
	/* Remove the completed descriptor from issued list */
	list_del(&vd->node);

	/* WARN about bad descriptor */
	dev_err(chan2dev(chan),
		"Bad descriptor submitted for %s, cookie: %d, irq: 0x%08x\n",
		axi_chan_name(chan), vd->tx.cookie, status);
	axi_chan_list_dump_lli(chan, vd_to_axi_desc(vd));

	vchan_cookie_complete(vd);

	/* Try to restart the controller */
	axi_chan_start_first_queued(chan);

out:
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void axi_chan_dma_xfer_complete(struct axi_dma_chan *chan)
{
	int count = atomic_read(&chan->descs_allocated);
	struct axi_dma_hw_desc *hw_desc;
	struct axi_dma_desc *desc;
	struct virt_dma_desc *vd;
	unsigned long flags;
	u64 llp;
	int i;

	dev_dbg(chan2dev(chan), "DMA XFER complete");

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (unlikely(axi_chan_is_hw_enable(chan))) {
		dev_err(chan2dev(chan), "BUG: %s caught DWAXIDMAC_IRQ_DMA_TRF, but channel not idle!\n",
			axi_chan_name(chan));
		axi_chan_disable(chan);
	}

	/* The completed descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	if (!vd) {
		dev_err(chan2dev(chan), "BUG: %s, IRQ with no descriptors\n",
			axi_chan_name(chan));
		goto out;
	}

	desc = vd_to_axi_desc(vd);
	if (chan->cyclic) {
		if (desc) {
			llp = lo_hi_readq(chan->chan_regs + CH_LLP);
			for (i = 0; i < count; i++) {
				hw_desc = &desc->hw_desc[i];
				if (hw_desc->llp == llp) {
					axi_chan_irq_clear(chan, hw_desc->lli->status_lo);
					hw_desc->lli->ctl_hi |= CH_CTL_H_LLI_VALID;
					desc->completed_blocks = i;

					if (((hw_desc->len * (i + 1)) % desc->period_len) == 0)
						vchan_cyclic_callback(vd);
					break;
				}
			}

			axi_chan_enable(chan);
		}
	} else {
		if (chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_CONTIGUOUS)
			mark_hwdesc_done(desc);

		if((chan->chip->dw->hdata->xfer_mode != DWAXIDMAC_MBLK_TYPE_CONTIGUOUS) ||
			((chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_CONTIGUOUS) &&
			(get_next_hwdesc_number(desc) < 0)))
		{
			/* Remove the completed descriptor from issued list before completing */
			list_del(&vd->node);
			vchan_cookie_complete(vd);
		}

		/* Submit queued descriptors after processing the completed ones */
		if (!((chan->chip->dw->hdata->xfer_mode == DWAXIDMAC_MBLK_TYPE_CONTIGUOUS)
		       && (chan->is_paused)))
			axi_chan_start_first_queued(chan);
	}

out:
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}
static void axi_chan_block_xfer_complete(struct axi_dma_chan *chan)
{
	struct virt_dma_desc *vd;
	unsigned long flags;

	dev_dbg(chan2dev(chan), "block transfer complete\n");

	spin_lock_irqsave(&chan->vc.lock, flags);

	/* The completed descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	if (!vd) {
		dev_err(chan2dev(chan), "BUG: %s, IRQ with no descriptors\n",
			axi_chan_name(chan));
		goto out;
	}

	if(chan->chip->dw->hdata->xfer_mode != DWAXIDMAC_MBLK_TYPE_CONTIGUOUS) {
		/* Remove the completed descriptor from issued list before completing */
		list_del(&vd->node);
		vchan_cookie_complete(vd);
	}
out:
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void dw_axi_dma_process_interrupt(struct axi_dma_chan *chan)
{
	u64 status;

	/* Poll, clear and process this channel interrupt status */
	status = axi_chan_irq_read(chan);

	if(chan->chip->dw->hdata->rdwr_back_feature)
	{
                if (status) {
                        dev_dbg(chan2dev(chan) ,"size of lli:%d \n", sizeof(struct axi_dma_lli));
                        struct axi_dma_hw_desc *cur_hw_desc = NULL;
                        u32 *tmp_hw_desc = NULL;
                        for (int j=0; j<16; j++) {
                                cur_hw_desc = &writeback_desc->hw_desc[j];
                                tmp_hw_desc = (u32 *)(cur_hw_desc->lli);
                                int k=0;
                                dev_dbg(chan2dev(chan), "**LLI:%d DATA**\n", j);
                                dev_dbg(chan2dev(chan), "LLI:%d transfer size from writeback reg:0x%x, requested size:0x%x \n",j,
						(cur_hw_desc->lli->status_lo)&WRITEBACK_MASK, cur_hw_desc->lli->block_ts_lo);
                                while(k < 16) {
                                        dev_dbg(chan2dev(chan), "addr:0x%x val:0x%x \n", tmp_hw_desc, *tmp_hw_desc);
                                        k++;
                                        tmp_hw_desc++;
                                }
                                dev_dbg(chan2dev(chan) ,"***************\n");

                        }
                }
	}
	axi_chan_irq_clear(chan, status);

	if (status & DWAXIDMAC_IRQ_ALL_ERR)
		axi_chan_handle_err(chan, status);
	else if (status & (DWAXIDMAC_IRQ_DMA_TRF))
		axi_chan_dma_xfer_complete(chan);
	else if (status & (DWAXIDMAC_IRQ_BLOCK_TRF))
		axi_chan_block_xfer_complete(chan);
}

static irqreturn_t dw_axi_dma_interrupt(int irq, void *dev_id)
{
	struct axi_dma_chip *chip = dev_id;
	struct dw_axi_dma *dw = chip->dw;
	struct axi_dma_chan *chan;

	u32 i;

	/* Disable DMAC interrupts. We'll enable them after processing channels */
	axi_dma_irq_disable(chip);

	/* Poll, clear and process every channel interrupt status */
	for (i = 0; i < dw->hdata->nr_channels; i++) {
		chan = &dw->chan[i];
		dw_axi_dma_process_interrupt(chan);
	}

	/* Re-enable interrupts */
	axi_dma_irq_enable(chip);

	return IRQ_HANDLED;
}

static irqreturn_t dw_axi_dma_ch_interrupt(int irq, void *dev_id)
{
	struct axi_dma_chan *chan = dev_id;

	dev_dbg(chan->chip->dev, "handling channel %u interrupt\n", chan->id);

	/* Disable per channel interrupt */
	axi_chan_irq_sig_set(chan, DWAXIDMAC_IRQ_NONE);

	dw_axi_dma_process_interrupt(chan);

	/* Re-enable interrupts */
	axi_chan_irq_sig_set(chan, DWAXIDMAC_IRQ_ALL & 
								(~DWAXIDMAC_IRQ_SRC_TRAN) &
								(~DWAXIDMAC_IRQ_DST_TRAN));

	return IRQ_HANDLED;
}

static int dma_chan_terminate_all(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	u32 offset = chan->id < 16 ? DMAC_CHEN_L : DMAC_CHEN_H;
	u32 chan_active = BIT(chan->id & 0xf) << DMAC_CHAN_EN_SHIFT;
	unsigned long flags;
	u32 val;
	int ret;

	LIST_HEAD(head);

	axi_chan_disable(chan);

	ret = readl_poll_timeout_atomic(chan->chip->regs + offset, val,
					!(val & chan_active), 1000, 10000);
	if (ret == -ETIMEDOUT)
		dev_warn(dchan2dev(dchan),
			 "%s failed to stop\n", axi_chan_name(chan));

	if (chan->direction != DMA_MEM_TO_MEM)
		dw_axi_dma_set_hw_channel(chan, false);
	if (chan->direction == DMA_MEM_TO_DEV)
		dw_axi_dma_set_byte_halfword(chan, false);

	spin_lock_irqsave(&chan->vc.lock, flags);

	vchan_get_all_descriptors(&chan->vc, &head);

	chan->cyclic = false;
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	vchan_dma_desc_free_list(&chan->vc, &head);

	dev_vdbg(dchan2dev(dchan), "terminated: %s\n", axi_chan_name(chan));

	return 0;
}

static int dma_chan_pause(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;
	unsigned int timeout = 20; /* timeout iterations */
	u32 offset = chan->id < 16 ? DMAC_CHSUSPREG_L : DMAC_CHSUSPREG_H;
	u32 val;

	spin_lock_irqsave(&chan->vc.lock, flags);

	if (chan->chip->dw->hdata->reg_map_8_channels) {
		val = axi_dma_ioread32(chan->chip, DMAC_CHEN);
		val |= BIT(chan->id) << DMAC_CHAN_SUSP_SHIFT |
			BIT(chan->id) << DMAC_CHAN_SUSP_WE_SHIFT;
		axi_dma_iowrite32(chan->chip, DMAC_CHEN, val);
	} else {
		val = axi_dma_ioread32(chan->chip, offset);
		val |= BIT(chan->id & 0xf) << DMAC_CHAN_SUSP2_SHIFT |
			BIT(chan->id & 0xf) << DMAC_CHAN_SUSP2_WE_SHIFT;
		axi_dma_iowrite32(chan->chip, offset, val);
	}

	do  {
		if (axi_chan_irq_read(chan) & DWAXIDMAC_IRQ_SUSPENDED)
			break;

		udelay(2);
	} while (--timeout);

	axi_chan_irq_clear(chan, DWAXIDMAC_IRQ_SUSPENDED);

	chan->is_paused = true;

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return timeout ? 0 : -EAGAIN;
}

/* Called in chan locked context */
static inline void axi_chan_resume(struct axi_dma_chan *chan)
{
	u32 val;
	u32 offset = chan->id < 16 ? DMAC_CHSUSPREG_L : DMAC_CHSUSPREG_H;

	if (chan->chip->dw->hdata->reg_map_8_channels) {
		val = axi_dma_ioread32(chan->chip, DMAC_CHEN);
		val &= ~(BIT(chan->id) << DMAC_CHAN_SUSP_SHIFT);
		val |=  (BIT(chan->id) << DMAC_CHAN_SUSP_WE_SHIFT);
		axi_dma_iowrite32(chan->chip, DMAC_CHEN, val);
	} else {
		val = axi_dma_ioread32(chan->chip, offset);
		val &= ~(BIT(chan->id & 0xf) << DMAC_CHAN_SUSP2_SHIFT);
		val |=  (BIT(chan->id & 0xf) << DMAC_CHAN_SUSP2_WE_SHIFT);
		axi_dma_iowrite32(chan->chip, offset, val);
	}

	chan->is_paused = false;
}

static int dma_chan_resume(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	if (chan->is_paused)
		axi_chan_resume(chan);

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return 0;
}

static int axi_dma_suspend(struct axi_dma_chip *chip)
{
	axi_dma_irq_disable(chip);
	axi_dma_disable(chip);

	clk_disable_unprepare(chip->core_clk);
	clk_disable_unprepare(chip->cfgr_clk);

	return 0;
}

static int axi_dma_resume(struct axi_dma_chip *chip)
{
	int ret;

	ret = clk_prepare_enable(chip->cfgr_clk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(chip->core_clk);
	if (ret < 0)
		return ret;

	axi_dma_enable(chip);
	axi_dma_irq_enable(chip);

	return 0;
}

static int __maybe_unused axi_dma_runtime_suspend(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_suspend(chip);
}

static int __maybe_unused axi_dma_runtime_resume(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_resume(chip);
}

static struct dma_chan *dw_axi_dma_of_xlate(struct of_phandle_args *dma_spec,
					    struct of_dma *ofdma)
{
	struct dw_axi_dma *dw = ofdma->of_dma_data;
	struct axi_dma_chan *chan;
	struct dma_chan *dchan;
	unsigned int index;

	if (dma_spec->args_count != 1)
		return NULL;

	index = dma_spec->args[0];

	if (dw->hdata->hardcoded_handshake) {
		dchan = dma_get_slave_channel(&dw->chan[index].vc.chan);
		if (!dchan)
			return NULL;

		dw->chan[index].hw_handshake_num = index;
	} else {
		dchan = dma_get_any_slave_channel(&dw->dma);
		if (!dchan)
			return NULL;

		chan = dchan_to_axi_dma_chan(dchan);
		chan->hw_handshake_num = index;
	}

	dev_dbg(chan->chip->dev, "handshake num : %#x\n", index);
	return dchan;
}

static int parse_device_properties(struct axi_dma_chip *chip)
{
	struct device *dev = chip->dev;
	u32 tmp, carr[DMAC_MAX_CHANNELS];
	u64 dev_addr_mask;
	int ret;

	ret = device_property_read_u32(dev, "dma-channels", &tmp);
	if (ret)
		return ret;
	if (tmp == 0 || tmp > DMAC_MAX_CHANNELS)
		return -EINVAL;

	chip->dw->hdata->nr_channels = tmp;
	if (tmp <= DMA_REG_MAP_CH_REF)
		chip->dw->hdata->reg_map_8_channels = true;

	ret = device_property_read_u32(dev, "snps,dma-masters", &tmp);
	if (ret)
		return ret;
	if (tmp == 0 || tmp > DMAC_MAX_MASTERS)
		return -EINVAL;

	chip->dw->hdata->nr_masters = tmp;

	ret = device_property_read_u32(dev, "snps,data-width", &tmp);
	if (ret)
		return ret;
	if (tmp > DWAXIDMAC_TRANS_WIDTH_MAX)
		return -EINVAL;

	chip->dw->hdata->m_data_width = tmp;

	ret = device_property_read_u32_array(dev, "snps,block-size", carr,
					     chip->dw->hdata->nr_channels);
	if (ret)
		return ret;
	for (tmp = 0; tmp < chip->dw->hdata->nr_channels; tmp++) {
		if (carr[tmp] == 0 || carr[tmp] > DMAC_MAX_BLK_SIZE)
			return -EINVAL;

		chip->dw->hdata->block_size[tmp] = carr[tmp];
	}

	ret = device_property_read_u32_array(dev, "snps,priority", carr,
					     chip->dw->hdata->nr_channels);
	if (ret)
		return ret;
	/* Priority value must be programmed within [0:nr_channels-1] range */
	for (tmp = 0; tmp < chip->dw->hdata->nr_channels; tmp++) {
		if (carr[tmp] >= chip->dw->hdata->nr_channels)
			return -EINVAL;

		chip->dw->hdata->priority[tmp] = carr[tmp];
	}

	/* axi-max-burst-len is optional property */
	ret = device_property_read_u32(dev, "snps,axi-max-burst-len", &tmp);
	if (!ret) {
		if (tmp > DWAXIDMAC_ARWLEN_MAX + 1)
			return -EINVAL;
		if (tmp < DWAXIDMAC_ARWLEN_MIN + 1)
			return -EINVAL;

		chip->dw->hdata->restrict_axi_burst_len = true;
		chip->dw->hdata->axi_rw_burst_len = tmp;
	}

	/* transfer-mode is optional property */
	ret = device_property_read_u32(dev, "snps,transfer-mode", &tmp);
	chip->dw->hdata->xfer_mode = 0;
	if (!ret) {
		if (tmp > DWAXIDMAC_MBLK_TYPE_LL)
			return -EINVAL;
		chip->dw->hdata->xfer_mode = tmp;
	} else
		chip->dw->hdata->xfer_mode = DWAXIDMAC_MBLK_TYPE_LL;

	/* dev-addr-mask is optional property */
	ret = device_property_read_u64(dev, "simaai,device-addr-mask", &dev_addr_mask);
	if (!ret)
		chip->dw->hdata->dev_addr_mask = dev_addr_mask;
	else
		chip->dw->hdata->dev_addr_mask = ULONG_MAX;

	chip->dw->hdata->hardcoded_handshake =
		device_property_read_bool(dev, "simaai,hardcoded-handshake");

	chip->dw->hdata->perch_irq = device_property_read_bool(dev, "snps,perch_irq");
	/* lms-axi_master is optional property */
	ret = device_property_read_u32(dev, "snps,lms-axi-master", &tmp);
	chip->dw->hdata->lms_axi_master = 0;
	if(!ret) {
		chip->dw->hdata->lms_axi_master = tmp;
	}
	/* read write back feature is optional property */
	chip->dw->hdata->rdwr_back_feature = device_property_read_bool(dev, "snps,rdwr-back-feature");
	return 0;
}

static int dw_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct axi_dma_chip *chip;
	struct resource *mem;
	struct dw_axi_dma *dw;
	struct dw_axi_dma_hcfg *hdata;
	u32 i;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	dw = devm_kzalloc(&pdev->dev, sizeof(*dw), GFP_KERNEL);
	if (!dw)
		return -ENOMEM;

	hdata = devm_kzalloc(&pdev->dev, sizeof(*hdata), GFP_KERNEL);
	if (!hdata)
		return -ENOMEM;

	chip->dw = dw;
	chip->dev = &pdev->dev;
	chip->dw->hdata = hdata;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->regs = devm_ioremap_resource(chip->dev, mem);
	if (IS_ERR(chip->regs))
		return PTR_ERR(chip->regs);

	if (of_device_is_compatible(node, "intel,kmb-axi-dma")) {
		chip->apb_regs = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(chip->apb_regs))
			return PTR_ERR(chip->apb_regs);
	}

	chip->core_clk = devm_clk_get(chip->dev, "core-clk");
	if (IS_ERR(chip->core_clk))
		return PTR_ERR(chip->core_clk);

	chip->cfgr_clk = devm_clk_get(chip->dev, "cfgr-clk");
	if (IS_ERR(chip->cfgr_clk))
		return PTR_ERR(chip->cfgr_clk);

	ret = parse_device_properties(chip);
	if (ret)
		return ret;

	dw->chan = devm_kcalloc(chip->dev, hdata->nr_channels,
				sizeof(*dw->chan), GFP_KERNEL);
	if (!dw->chan)
		return -ENOMEM;

	if (hdata->perch_irq) {
		for (i = 0; i < hdata->nr_channels; i++) {
			dw->chan[i].irq = platform_get_irq(pdev, i);
			if (dw->chan[i].irq < 0)
				return dw->chan[i].irq;

			dev_info(chip->dev, "channel: %u interrupt number: %u\n", i, dw->chan[i].irq); 

			ret = devm_request_irq(chip->dev, dw->chan[i].irq, dw_axi_dma_ch_interrupt,
					IRQF_SHARED, KBUILD_MODNAME, &dw->chan[i]);
			if (ret)
				return ret;
		}
	} else {
		chip->irq = platform_get_irq(pdev, 0);
		if (chip->irq < 0)
			return chip->irq;

		ret = devm_request_irq(chip->dev, chip->irq, dw_axi_dma_interrupt,
			       IRQF_SHARED, KBUILD_MODNAME, chip);
		if (ret)
			return ret;
	}

	INIT_LIST_HEAD(&dw->dma.channels);
	for (i = 0; i < hdata->nr_channels; i++) {
		struct axi_dma_chan *chan = &dw->chan[i];

		chan->chip = chip;
		chan->id = i;
		chan->chan_regs = chip->regs + COMMON_REG_LEN + i * CHAN_REG_LEN;
		atomic_set(&chan->descs_allocated, 0);

		chan->vc.desc_free = vchan_desc_put;
		vchan_init(&chan->vc, &dw->dma);
		INIT_LIST_HEAD(&chan->desc_list);
	}

	/* Set capabilities */
	dma_cap_set(DMA_MEMCPY, dw->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, dw->dma.cap_mask);
	dma_cap_set(DMA_CYCLIC, dw->dma.cap_mask);
	dma_cap_set(DMA_INTERLEAVE, dw->dma.cap_mask);

	/* DMA capabilities */
	dw->dma.chancnt = hdata->nr_channels;
	dw->dma.max_burst = hdata->axi_rw_burst_len;
	dw->dma.src_addr_widths = AXI_DMA_BUSWIDTHS;
	dw->dma.dst_addr_widths = AXI_DMA_BUSWIDTHS;
	dw->dma.directions = BIT(DMA_MEM_TO_MEM);
	dw->dma.directions |= BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	dw->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

	dw->dma.dev = chip->dev;
	dw->dma.device_tx_status = dma_chan_tx_status;
	dw->dma.device_issue_pending = dma_chan_issue_pending;
	dw->dma.device_terminate_all = dma_chan_terminate_all;
	dw->dma.device_pause = dma_chan_pause;
	dw->dma.device_resume = dma_chan_resume;

	dw->dma.device_alloc_chan_resources = dma_chan_alloc_chan_resources;
	dw->dma.device_free_chan_resources = dma_chan_free_chan_resources;

	dw->dma.device_prep_dma_memcpy = dma_chan_prep_dma_memcpy;
	dw->dma.device_synchronize = dw_axi_dma_synchronize;
	dw->dma.device_config = dw_axi_dma_chan_slave_config;
	dw->dma.device_prep_slave_sg = dw_axi_dma_chan_prep_slave_sg;
	dw->dma.device_prep_dma_cyclic = dw_axi_dma_chan_prep_cyclic;
	dw->dma.device_prep_interleaved_dma = dw_axi_dma_chan_prep_interleaved;

	/*
	 * Synopsis DesignWare AxiDMA datasheet mentioned Maximum
	 * supported blocks is 1024. Device register width is 4 bytes.
	 * Therefore, set constraint to 1024 * 4.
	 */
	dw->dma.dev->dma_parms = &dw->dma_parms;
	dma_set_max_seg_size(&pdev->dev, MAX_BLOCK_SIZE);
	platform_set_drvdata(pdev, chip);

	pm_runtime_enable(chip->dev);

	/*
	 * We can't just call pm_runtime_get here instead of
	 * pm_runtime_get_noresume + axi_dma_resume because we need
	 * driver to work also without Runtime PM.
	 */
	pm_runtime_get_noresume(chip->dev);
	ret = axi_dma_resume(chip);
	if (ret < 0)
		goto err_pm_disable;

	axi_dma_hw_init(chip);

	pm_runtime_put(chip->dev);

	ret = dmaenginem_async_device_register(&dw->dma);
	if (ret)
		goto err_pm_disable;

	/* Register with OF helpers for DMA lookups */
	ret = of_dma_controller_register(pdev->dev.of_node,
					 dw_axi_dma_of_xlate, dw);
	if (ret < 0)
		dev_warn(&pdev->dev,
			 "Failed to register OF DMA controller, fallback to MEM_TO_MEM mode\n");

	dev_info(chip->dev, "DesignWare AXI DMA Controller, %d channels\n",
		 dw->hdata->nr_channels);

	return 0;

err_pm_disable:
	pm_runtime_disable(chip->dev);

	return ret;
}

static int dw_remove(struct platform_device *pdev)
{
	struct axi_dma_chip *chip = platform_get_drvdata(pdev);
	struct dw_axi_dma *dw = chip->dw;
	struct axi_dma_chan *chan, *_chan;
	u32 i;

	/* Enable clk before accessing to registers */
	clk_prepare_enable(chip->cfgr_clk);
	clk_prepare_enable(chip->core_clk);
	axi_dma_irq_disable(chip);
	for (i = 0; i < dw->hdata->nr_channels; i++) {
		axi_chan_disable(&chip->dw->chan[i]);
		axi_chan_irq_disable(&chip->dw->chan[i], DWAXIDMAC_IRQ_ALL);
	}
	axi_dma_disable(chip);

	pm_runtime_disable(chip->dev);
	axi_dma_suspend(chip);

	devm_free_irq(chip->dev, chip->irq, chip);

	of_dma_controller_free(chip->dev->of_node);

	list_for_each_entry_safe(chan, _chan, &dw->dma.channels,
			vc.chan.device_node) {
		list_del(&chan->vc.chan.device_node);
		tasklet_kill(&chan->vc.task);
	}

	return 0;
}

static const struct dev_pm_ops dw_axi_dma_pm_ops = {
	SET_RUNTIME_PM_OPS(axi_dma_runtime_suspend, axi_dma_runtime_resume, NULL)
};

static const struct of_device_id dw_dma_of_id_table[] = {
	{ .compatible = "snps,axi-dma-1.01a" },
	{ .compatible = "intel,kmb-axi-dma" },
	{}
};
MODULE_DEVICE_TABLE(of, dw_dma_of_id_table);

static struct platform_driver dw_driver = {
	.probe		= dw_probe,
	.remove		= dw_remove,
	.driver = {
		.name	= KBUILD_MODNAME,
		.of_match_table = dw_dma_of_id_table,
		.pm = &dw_axi_dma_pm_ops,
	},
};
module_platform_driver(dw_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Synopsys DesignWare AXI DMA Controller platform driver");
MODULE_AUTHOR("Eugeniy Paltsev <Eugeniy.Paltsev@synopsys.com>");
