// SPDX-License-Identifier: GPL-2.0-only
/*
 * SIMAAI Remote Processor driver for EVXX processor family
 *
 * Copyright SiMa.ai (C) 2025 All rights reserved
 *
 * Author: Nikunj Kela <nikunj.kela@sima.ai>
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include "remoteproc_internal.h"

#include <linux/simaai-stu.h>

#define SIMAAI_RPROC_MAX_VRING		2
#define SIMAAI_TX_TOUT			500
#define CVU_FIRMWARE_SIZE		0x04000000
#define CSM_BASE_ADD			0x10000000
#define VCCM0_BASE_ADD			0x20000000
#define VCCM1_BASE_ADD			0x20040000
#define VCCM2_BASE_ADD			0x20080000
#define VCCM3_BASE_ADD			0x200C0000

#define ARC_INITIATE_RUN		BIT(25)
#define CVU_STU_INITIATOR_CLK_EN	BIT(0)

#define CVU_CTRL_CORE0_OFFSET		0x20
#define CVU_CTRL_CORE1_OFFSET		0x24
#define CVU_CTRL_CORE2_OFFSET		0x28
#define CVU_CTRL_CORE3_OFFSET		0x2C
#define CVU_CTRL_SYS_VCCM0_OFFSET	0x40
#define CVU_CTRL_SYS_VCCM1_OFFSET	0x44
#define CVU_CTRL_SYS_VCCM2_OFFSET	0x48
#define CVU_CTRL_SYS_VCCM3_OFFSET	0x4C
#define CVU_CTRL_SYS_CSM_OFFSET		0x50
#define CVU_CTRL_STU_OFFSET		0x14
#define CVU_BOOT_VECTOR_OFFSET		10

#define DAVINCI_CVU_CLK_EN		BIT(0)
#define DAVINCI_CVU_EV74_CLK_EN		BIT(1)
#define DAVINCI_CVU_PCLK_EN		BIT(2)
#define DAVINCI_CVU_DBG_PCLK_EN		BIT(3)
#define DAVINCI_CVU_EV74_RST_N		BIT(4)
#define DAVINCI_CVU_PRESETDBGN		BIT(5)
#define DAVINCI_CVU_PRESET_N		BIT(6)
#define DAVINCI_CVU_DBG_PRESETN		BIT(7)
#define DAVINCI_CVU_RST			BIT(8)

#define DAVINCI_CLK_EN_MASK	(DAVINCI_CVU_CLK_EN | DAVINCI_CVU_EV74_CLK_EN | DAVINCI_CVU_PCLK_EN | \
							DAVINCI_CVU_DBG_PCLK_EN)
#define DAVINCI_RESET_MASK	(DAVINCI_CVU_EV74_RST_N | DAVINCI_CVU_PRESETDBGN | DAVINCI_CVU_PRESET_N \
							| DAVINCI_CVU_DBG_PRESETN | DAVINCI_CVU_RST)

#define MODALIX_CVU_CLK_EN			BIT(0)
#define MODALIX_CVU_EV74_CLK_EN			BIT(1)
#define MODALIX_CVU_DBG_PCLK_EN			BIT(2)
#define MODALIX_CVU_EV74_RST_N			BIT(3)
#define MODALIX_CVU_PRESETDBGN			BIT(4)
#define MODALIX_CVU_APB_RESET_N			BIT(5)
#define MODALIX_CVU_DBG_PRESETN			BIT(6)
#define MODALIX_CVU_RESET_N			BIT(7)
#define MODALIX_CVU_DBG_CACHE_RST_DISABLE	BIT(8)
#define MODALIX_CVU_RST				BIT(9)

#define MODALIX_CLK_EN_MASK (MODALIX_CVU_CLK_EN | MODALIX_CVU_EV74_CLK_EN | MODALIX_CVU_DBG_PCLK_EN)
#define MODALIX_RESET_MASK	(MODALIX_CVU_EV74_RST_N | MODALIX_CVU_PRESETDBGN | \
					MODALIX_CVU_DBG_PRESETN | MODALIX_CVU_RESET_N | MODALIX_CVU_RST)

/**
 * @prc_base_addr : memory mapped base address for PRC register
 * @cvu_glue_base_addr : Memory mapped base address CVU glue registers
 * @cvu_clk_reset_offset : offset from PRC base register for clock register
 * @clock_enable_mask : Mask for enabling clocks in CVU clock register
 * @reset_mask : Reset mask in CVU clock register
 * @reset_bit_pos : Bit position of reset in CVU clock register
 */
struct simaai_rproc_config {
	phys_addr_t		prc_base_addr;
	phys_addr_t		cvu_glue_base_addr;
	uint32_t		cvu_clk_reset_offset;
	uint32_t		clock_enable_mask;
	uint32_t		reset_mask;
	u8			reset_bit_pos;
};

/**
 * struct simaai_rproc - sima remote processor state
 * @rproc: rproc handle
 * @pdev: pointer to platform device
 * @mem: sima memory information
 */
struct simaai_rproc {
	struct device *dev;
	struct mbox_client mc;
	struct mbox_chan *chan;
	struct work_struct rx_wq;
	struct simaai_rproc_config *config;
	struct simaai_stu *stu;
	struct rproc * rproc;
	u32	evmem_base;
	size_t	evmem_size;
};

static void simaai_rproc_kick(struct rproc *rproc, int vqid)
{
	struct simaai_rproc *srproc = rproc->priv;
	int ret;

	if (WARN_ON(vqid >= SIMAAI_RPROC_MAX_VRING))
		return;

	/* we just need to send the vq-id */
	ret = mbox_send_message(srproc->chan, (void *)&vqid);
	if (ret < 0)
		dev_err(srproc->dev, "failed to send message via mbox: %d\n", ret);
}

/*
 * EV startup sequence
 * If we reach this point this means the ev firwmare is loaded onto
 * DRAM and now ready to run the boot sequence.
 * 1. Enable PRC clocks for EV (troot should not have any firewalls here)
 * 2. Setup interrupt Vectors, this depends on the arc.met file for ev
 * 3. Setup VMEM address for the all the cores.
 * 4. Release EV out of Reset
 * 5. Setup CSM address
 * 6. Initiate STU clock_en signal
 * 7. Intiiate per core flag
 */
static int simaai_rproc_start(struct rproc *rproc)
{
	struct simaai_rproc *srproc = rproc->priv;
	uint32_t boot_vector = (uint32_t)(srproc->evmem_base + srproc->evmem_size
							- CVU_FIRMWARE_SIZE);
	uint32_t val = 0;
	void __iomem *prc_cvu_addr = ioremap(srproc->config->prc_base_addr +
					srproc->config->cvu_clk_reset_offset, 0x4);
	void __iomem *cvu_glue_base_addr = ioremap(srproc->config->cvu_glue_base_addr, 0xA0);

	// STEP 1
	val = readl(prc_cvu_addr);
	writel(val | srproc->config->clock_enable_mask, prc_cvu_addr);
	val = readl(prc_cvu_addr);
	writel(val | srproc->config->reset_mask, prc_cvu_addr);

	// STEP 2
	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE0_OFFSET);
	val &= ~0x3fffff;
	writel(val | (boot_vector >> CVU_BOOT_VECTOR_OFFSET), cvu_glue_base_addr + CVU_CTRL_CORE0_OFFSET);

	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE1_OFFSET);
	val &= ~0x3fffff;
	writel(val | (boot_vector >> CVU_BOOT_VECTOR_OFFSET), cvu_glue_base_addr + CVU_CTRL_CORE1_OFFSET);

	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE2_OFFSET);
	val &= ~0x3fffff;
	writel(val | (boot_vector >> CVU_BOOT_VECTOR_OFFSET), cvu_glue_base_addr + CVU_CTRL_CORE2_OFFSET);

	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE3_OFFSET);
	val &= ~0x3fffff;
	writel(val | (boot_vector >> CVU_BOOT_VECTOR_OFFSET), cvu_glue_base_addr + CVU_CTRL_CORE3_OFFSET);

	// STEP 3
	writel(VCCM0_BASE_ADD, cvu_glue_base_addr + CVU_CTRL_SYS_VCCM0_OFFSET);
	writel(VCCM1_BASE_ADD, cvu_glue_base_addr + CVU_CTRL_SYS_VCCM1_OFFSET);
	writel(VCCM2_BASE_ADD, cvu_glue_base_addr + CVU_CTRL_SYS_VCCM2_OFFSET);
	writel(VCCM3_BASE_ADD, cvu_glue_base_addr + CVU_CTRL_SYS_VCCM3_OFFSET);

	// STEP 4
	val = readl(prc_cvu_addr);
	writel((val & ~BIT(srproc->config->reset_bit_pos)), prc_cvu_addr);

	// STEP 5
	writel(CSM_BASE_ADD, cvu_glue_base_addr + CVU_CTRL_SYS_CSM_OFFSET);

	// STEP 6
	val = readl(cvu_glue_base_addr + CVU_CTRL_STU_OFFSET);
	writel(val | CVU_STU_INITIATOR_CLK_EN, cvu_glue_base_addr + CVU_CTRL_STU_OFFSET);

	// STEP 7
	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE0_OFFSET);
	writel(val | ARC_INITIATE_RUN, cvu_glue_base_addr + CVU_CTRL_CORE0_OFFSET);

	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE1_OFFSET);
	writel(val | ARC_INITIATE_RUN, cvu_glue_base_addr + CVU_CTRL_CORE1_OFFSET);

	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE2_OFFSET);
	writel(val | ARC_INITIATE_RUN, cvu_glue_base_addr + CVU_CTRL_CORE2_OFFSET);

	val = readl(cvu_glue_base_addr + CVU_CTRL_CORE3_OFFSET);
	writel(val | ARC_INITIATE_RUN, cvu_glue_base_addr + CVU_CTRL_CORE3_OFFSET);

	iounmap(prc_cvu_addr);
	iounmap(cvu_glue_base_addr);

	return 0;
}

static int simaai_rproc_stop(struct rproc *rproc)
{
	struct simaai_rproc *srproc = rproc->priv;
	void __iomem *prc_cvu_addr = ioremap(srproc->config->prc_base_addr +
									srproc->config->cvu_clk_reset_offset, 0x4);
	uint32_t val = readl(prc_cvu_addr);
	writel(val & (~(srproc->config->clock_enable_mask)), prc_cvu_addr);

	val = readl(prc_cvu_addr);
	writel(val & (~(srproc->config->reset_mask)), prc_cvu_addr);

	iounmap(prc_cvu_addr);
	dev_info(rproc->dev.parent, "Stopped EV core");

	return 0;
}

static int simaai_rproc_elf_load_rsc_table(struct rproc *rproc,
					 const struct firmware *fw)
{
	if (rproc_elf_load_rsc_table(rproc, fw)) {
		dev_warn(rproc->dev.parent,
				"no resource table found for this firmware\n");
	}

	return 0;
}

static int simaai_rproc_mem_alloc(struct rproc *rproc, struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	va = ioremap_wc(mem->dma, mem->len);
	if (!va) {
		dev_err(dev, "Unable to map memory region: %pa+%zx\n",
			 &mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int simaai_rproc_mem_release(struct rproc *rproc,
				struct rproc_mem_entry *mem)
{
	iounmap(mem->va);

	return 0;
}

static const char *get_carveout_name(const char *nodename)
{
#define NR_VDEV_BUF 3
	/* we only care about carveout names that will be used by virtio */
	const char *names[NR_VDEV_BUF] = {"vdev0buffer", "vdev0vring0", "vdev0vring1"};

	for (int i = 0; i < NR_VDEV_BUF; i++) {
		if (strstr(nodename, names[i]))
			return names[i];
	}

	return nodename;
}

static int simaai_rproc_add_carveout(struct simaai_rproc *srproc)
{
	struct device *dev = srproc->dev;
	struct device_node *np = dev->of_node;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	struct of_phandle_iterator it;
	int index = 0;

	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			of_node_put(it.node);
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		/* we need base and size for evmem for startup */
		if (strstr(it.node->name, "evmem")) {
			srproc->evmem_base = rmem->base;
			srproc->evmem_size = rmem->size;
		}

		/*  No need to map vdev buffer */
		if (strstr(it.node->name, "vdev0buffer")) {
			mem = rproc_of_resm_mem_entry_init(dev, index,
							   rmem->size,
							   rmem->base,
							   get_carveout_name(it.node->name));
		} else {
			dma_addr_t dma_addr = rmem->base;
			if (srproc->stu) {
				if (simaai_stu_get_dev_address(srproc->stu, rmem->base, &dma_addr)) {
					dev_err(dev, "Error getting device address\n");
					return -EINVAL;
				}
			}

			mem = rproc_mem_entry_init(dev, NULL,
						   dma_addr,
						   rmem->size, rmem->base,
						   simaai_rproc_mem_alloc,
						   simaai_rproc_mem_release,
						   get_carveout_name(it.node->name));
		}

		if (!mem) {
			of_node_put(it.node);
			return -ENOMEM;
		}

		rproc_add_carveout(srproc->rproc, mem);
		index++;
	}

	if (!srproc->evmem_size || !srproc->evmem_base) {
		dev_err(dev, "evmem memory region not found\n");
		return -EINVAL;
	}

	return 0;
}

static int simaai_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct simaai_rproc *srproc = rproc->priv;
	int ret;

	ret = simaai_rproc_add_carveout(srproc);
	if(ret)
		return ret;

	return simaai_rproc_elf_load_rsc_table(rproc, fw);
}

static int event_notified_idr_cb(int id, void *ptr, void *data)
{
	struct rproc *rproc = (struct rproc *)data;

	(void)rproc_vq_interrupt(rproc, id);

	return 0;
}

static void handle_event_notified(struct work_struct *work)
{
	struct rproc *rproc;
	struct simaai_rproc *srproc;

	srproc = container_of(work, struct simaai_rproc, rx_wq);

	rproc = srproc->rproc;

	idr_for_each(&rproc->notifyids, event_notified_idr_cb, rproc);
}

static void simaai_ev_rx_cb(struct mbox_client *cl, void *msg)
{
	struct simaai_rproc *srproc = container_of(cl, struct simaai_rproc, mc);

	schedule_work(&srproc->rx_wq);
}

static int simaai_setup_mbox(struct simaai_rproc *srproc)
{
	srproc->mc.dev = srproc->dev;
	srproc->mc.rx_callback = simaai_ev_rx_cb;
	srproc->mc.tx_done = NULL;
	srproc->mc.tx_block = true;
	srproc->mc.tx_tout = SIMAAI_TX_TOUT;
	srproc->mc.knows_txdone = false;

	INIT_WORK(&srproc->rx_wq, handle_event_notified);

	srproc->chan = mbox_request_channel(&srproc->mc, 0);
	if (IS_ERR(srproc->chan)) {
		dev_err(srproc->dev, "failed to request mbox channel.\n");
		srproc->chan = NULL;
		return -EINVAL;
	}

	return 0;
}

static const struct rproc_ops simaai_rproc_ops = {
	.start		= simaai_rproc_start,
	.stop		= simaai_rproc_stop,
	.parse_fw	= simaai_parse_fw,
	.load		= rproc_elf_load_segments,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr	= rproc_elf_get_boot_addr,
	.kick		= simaai_rproc_kick,
};

static const struct simaai_rproc_config davinci_rproc_config = {
	.prc_base_addr = 0x30100000,
	.cvu_glue_base_addr = 0x05841000,
	.cvu_clk_reset_offset = 0x108,
	.clock_enable_mask = DAVINCI_CLK_EN_MASK,
	.reset_mask = DAVINCI_RESET_MASK,
	.reset_bit_pos = 0x8,
};

static const struct simaai_rproc_config modalix_rproc_config = {
	.prc_base_addr = 0x0FF00000,
	.cvu_glue_base_addr = 0x040D0000,
	.cvu_clk_reset_offset = 0x50C,
	.clock_enable_mask = MODALIX_CLK_EN_MASK,
	.reset_mask = MODALIX_RESET_MASK,
	.reset_bit_pos = 0x9,
};

static const struct of_device_id simaai_rproc_of_match[] = {
	{ .compatible = "simaai,davinci-evxx-rproc", .data = &davinci_rproc_config},
	{ .compatible = "simaai,modalix-evxx-rproc", .data = &modalix_rproc_config},
	{},
};

MODULE_DEVICE_TABLE(of, simaai_rproc_of_match);

static int simaai_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simaai_rproc *srproc;
	struct device_node *np = dev->of_node;
	const char *fw_name;
	const char *cpu_name;
	struct rproc *rproc;
	int ret;
	const struct of_device_id *match;

	match = of_match_device(simaai_rproc_of_match, dev);
	if (!match || !match->data) {
		dev_err(dev, "No device match found\n");
		return -ENODEV;
	}

	ret = of_property_read_string(dev->of_node, "simaai,cvu-firmware", &fw_name);
	if (ret) {
		dev_err(dev, "Firmware filename is not provided in device tree\n");
		return -ENODEV;
	}

	ret = of_property_read_string(dev->of_node, "cpu-name", &cpu_name);
	if (ret) {
		dev_warn(dev, "No cpu-name specified in device tree.\n");
		return -EINVAL;
	}

	ret = of_property_count_elems_of_size(np, "memory-region",
						sizeof(phandle));
	if (ret <= 0) {
		dev_err(dev, "device does not reserved memory regions, ret = %d\n",
			ret);
		return -EINVAL;
	}

	rproc = rproc_alloc(dev, cpu_name, &simaai_rproc_ops, fw_name, sizeof(*srproc));
	if (!rproc)
		return -ENOMEM;

	srproc = rproc->priv;
	srproc->rproc = rproc;
	rproc->has_iommu = false;
	srproc->config = (struct simaai_rproc_config *)match->data;
	srproc->dev = dev;

	srproc->stu = simaai_stu_get_by_phandle(dev->of_node, "simaai,stu");
	if (IS_ERR(srproc->stu)) {
		if (PTR_ERR(srproc->stu) != -EPROBE_DEFER)
			srproc->stu = NULL;
		else {
			ret = PTR_ERR(srproc->stu);
			goto err_put_rproc;
		}
	} else
		dev_info(dev, "Success getting STU handle\n");

	platform_set_drvdata(pdev, rproc);

	ret = simaai_setup_mbox(srproc);
	if (ret)
		goto err_put_rproc;

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "ERROR : failed to add rproc handle\n");
		goto err_put_rproc;
	}

	dev_dbg(dev, "Successfully added rproc handle\n");

	return 0;

err_put_rproc:
	rproc_free(rproc);

	return ret;
}

static int simaai_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

/* TODO: SRIRAM Fix them with power management and ev platform driver
 * for power management
 */
#ifdef CONFIG_PM
static int simaai_rpm_suspend(struct device *dev)
{
	return -EBUSY;
}

static int simaai_rpm_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops simaai_rproc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(simaai_rpm_suspend, simaai_rpm_resume)
};

static struct platform_driver simaai_rproc_driver = {
	.probe = simaai_rproc_probe,
	.remove = simaai_rproc_remove,
	.driver = {
		.name = "simaai-rproc-evxx",
		.of_match_table = simaai_rproc_of_match,
		.pm = &simaai_rproc_pm_ops,
	},
};

module_platform_driver(simaai_rproc_driver);

MODULE_AUTHOR("Sriram Raghunathan <sriram.r@sima.ai>");
MODULE_AUTHOR("Manish Mathur <manish.mathur@sima.ai>");
MODULE_AUTHOR("Nileshkumar Raghuvanshi <nilesh.r@sima.ai>");
MODULE_DESCRIPTION("SIMA.ai Remote processor management driver for EV processor family");
MODULE_LICENSE("Dual MIT/GPL");
