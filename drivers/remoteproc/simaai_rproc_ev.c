// SPDX-License-Identifier: GPL-2.0-only
/*
 * SIMAAI Remote Processor driver for EVXX processor family
 *
 */

#include <linux/delay.h>
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
#include "remoteproc_internal.h"

#define NOC_REG_READ(x) (readl(x))
#define NOC_REG_WRITE(reg, val) (writel(val, reg))

#define SIMA_MAX_MEM_REGIONS 1

#define SIMA_ARCONNECT_ADDR 0x820000
#define SIMA_ARCONNECT_SZ 0x10000

// TODO: Move this to vdk platform specific code
#define SIMA_ARCONNECT_CMD_ADDR 0x100
#define SIMA_ARCONNECT_WRITE_ADDR 0x104

#define SIMA_EVXX_VECTOR_RST_BASE 0x220000
#define SIMA_EVXX_VECTOR_INTR_BASE 0x10091

#define SIMA_EVXX_CLEAR_BITS 0xF
#define SIMA_EVXX_CMD_DEBUG_RST 0x31
#define SIMA_EVXX_CMD_RUN 0x33

#define PRC_BASE      0x30100000
#define CVU_CTRL_ADDR 0x05841000

// Offsets
#define CVU_CTRL_REG_OFFSET 0x20
#define CVU_SYS_REG_OFFSET 0x40
#define CVU_STU_REG_OFFSET (0x14)

#define CVU_FIRMWARE_SIZE	0x04000000
#define PRC_REG__CKG_RST_REG__CVU_CK_RST_ADDR 0x00000108

const char *ev_mem_names[SIMA_MAX_MEM_REGIONS] = {
		"ev_mem"
};

/**
 * struct sima_mem - sima internal memory structure
 * @cpu_addr: MPU virtual address of the memory region
 * @bus_addr: Bus address used to access the memory region
 * @dev_addr: Device address from SIMA remote cores view
 * @size: Size of the memory region
 */
struct sima_mem {
	char name[20];
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

/**
 * struct sima_rproc - sima remote processor state
 * @rproc: rproc handle
 * @pdev: pointer to platform device
 * @mem: sima memory information
 */
struct sima_rproc {
	struct sima_mem mem[SIMA_MAX_MEM_REGIONS];
	int num_mems;
	struct sima_mem *rmem;
	int num_rmems;

	int is_rtc_only;
	u32 nb_rmems;
};

struct cvu_ck_rst_s {
	uint32_t cvu_clk_en : 1;
	uint32_t cvu_ev74_clk_en : 1;
	uint32_t cvu_pclk_en : 1;
	uint32_t cvu_dbg_pclk_en : 1;
	uint32_t cvu_ev74_rst_n : 1;
	uint32_t cvu_presetdbgn : 1;
	uint32_t cvu_presetn : 1;
	uint32_t cvu_dbg_presetn : 1;
	uint32_t cvu_rst : 1;
	uint32_t reserved0 : 23;
} __packed;

union cvu_ck_rst_u {
	struct cvu_ck_rst_s reg;
	uint32_t u32;
} __packed;

/* Enable this when you see spurious SEerrors, this would
   make sure the writes to cache lines are streamlined
   without any compiler optimization errors. Helps debugging */

#if 0
static void sima_barrier(void)
{
	printk("Barrier");
	__asm__("dmb sy;"
		"dsb sy;"
		"isb;"
		);
}
#endif

// TODO: Fix this, stub until mailbox & virtio are brought into remoteproc
static void sima_rproc_kick(struct rproc *rproc, int vqid)
{
	return;
}

/* [NOTE]:Enable this to use remoteproc with SynopsysVDK, do not commit this
 * when upstreaming the driver  */
#if defined(EV_VDK)
static int sima_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;

	dev_dbg(dev, "[%s][%d], Inside sima_rproc_start ", __FILE__, __LINE__);
	// sima_vdk_cpu_boot(arc_mmio);
	/* Map 1kb of arcconnect region */
	void __iomem *arc_mmio = devm_ioremap(dev,
					      SIMA_ARCONNECT_ADDR,
					      SIMA_ARCONNECT_SZ);

	writel(SIMA_EVXX_VECTOR_RST_BASE, arc_mmio + SIMA_ARCONNECT_WRITE_ADDR);
	writel(SIMA_EVXX_VECTOR_INTR_BASE, arc_mmio + SIMA_ARCONNECT_CMD_ADDR);

	writel(SIMA_EVXX_CLEAR_BITS , arc_mmio + SIMA_ARCONNECT_WRITE_ADDR);
	writel(SIMA_EVXX_CMD_DEBUG_RST, arc_mmio + SIMA_ARCONNECT_CMD_ADDR);

	usleep_range(10000, 10001);

	writel(SIMA_EVXX_CLEAR_BITS, arc_mmio + SIMA_ARCONNECT_WRITE_ADDR);
	writel(SIMA_EVXX_CMD_RUN, arc_mmio + SIMA_ARCONNECT_CMD_ADDR);

	iounmap(arc_mmio);
	return 0;
}
#else
/*
 * EV startup sequence
 * If we reach this point this means the ev firwmare is loaded onto
 * DRAM and now ready to run the boot sequence.
 * 1. Enable PRC clocks for EV (troot should not have any firewalls
 * here)
 * 2. Setup interrupt Vectors, this depends on the arc.met file for ev
 * 3. Setup VMEM address for the all the cores.
 * 4. Release EV out of Reset
 * 5. Setup CSM address
 * 6. Initiate STU clock_en signal
 * 7. Setup arc_initate to run the the firmware on all the cores
 */

static int sima_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct sima_rproc *srproc = rproc->priv;
	uint32_t boot_vector = (uint32_t)(srproc->mem[0].dev_addr + srproc->mem[0].size
					  - CVU_FIRMWARE_SIZE);

	uint32_t clk_offset = PRC_REG__CKG_RST_REG__CVU_CK_RST_ADDR;


	// Map the entire prc-update and release
	void __iomem *addr = ioremap(PRC_BASE, 0x1000);
	// Map the CVU-glue logic, do not use ioremap_wc here, this
	// leads to cache-line problems.
	void __iomem *cvu_ctrl_addr = ioremap(CVU_CTRL_ADDR, 0xa0);

	uint32_t *cvu_ctrl0 =
		(uint32_t *)(cvu_ctrl_addr + CVU_CTRL_REG_OFFSET);
	uint32_t *cvu_ctrl1 =
		(uint32_t *)(cvu_ctrl_addr + CVU_CTRL_REG_OFFSET + 4);
	uint32_t *cvu_ctrl2 =
		(uint32_t *)(cvu_ctrl_addr + CVU_CTRL_REG_OFFSET + 8);
	uint32_t *cvu_ctrl3 =
		(uint32_t *)(cvu_ctrl_addr + CVU_CTRL_REG_OFFSET + 12);

	uint32_t *cvu_ctrlsys0 =
		(uint32_t *)(cvu_ctrl_addr + CVU_SYS_REG_OFFSET);
	uint32_t *cvu_ctrlsys1 =
		(uint32_t *)(cvu_ctrl_addr + CVU_SYS_REG_OFFSET + 4);
	uint32_t *cvu_ctrlsys2 =
		(uint32_t *)(cvu_ctrl_addr + CVU_SYS_REG_OFFSET + 8);
	uint32_t *cvu_ctrlsys3 =
		(uint32_t *)(cvu_ctrl_addr + CVU_SYS_REG_OFFSET + 12);
	uint32_t *cvu_ctrlsys4 =
		(uint32_t *)(cvu_ctrl_addr + CVU_SYS_REG_OFFSET + 0x10);

	uint32_t *cvu_ctrl_stu = (uint32_t *)(cvu_ctrl_addr + CVU_STU_REG_OFFSET);

	union cvu_ck_rst_u cvu;
	cvu.u32 = NOC_REG_READ(addr + clk_offset);
	cvu.reg.cvu_clk_en = 1;
	cvu.reg.cvu_ev74_clk_en = 1;
	cvu.reg.cvu_pclk_en = 1;
	cvu.reg.cvu_dbg_pclk_en = 1;
	NOC_REG_WRITE(addr + clk_offset, cvu.u32);

	/* udelay(100); */

	cvu.u32 = NOC_REG_READ(addr + clk_offset);
	cvu.reg.cvu_ev74_rst_n = 1;
	cvu.reg.cvu_presetdbgn = 1;
	cvu.reg.cvu_presetn = 1;
	cvu.reg.cvu_dbg_presetn = 1;
	cvu.reg.cvu_rst = 1;
	NOC_REG_WRITE(addr + clk_offset, cvu.u32);

	/* udelay(100); */

	*cvu_ctrl0 &= ~0x3fffff;
	*cvu_ctrl0 = *cvu_ctrl0 | (boot_vector >> 10);
	*cvu_ctrlsys0 = 0x20000000;

	*cvu_ctrl1 &= ~0x3fffff;
	*cvu_ctrl1 = *cvu_ctrl1 | (boot_vector >> 10);
	*cvu_ctrlsys1 = 0x20040000; //here same inside or different outside - tbd

	*cvu_ctrl2 &= ~0x3fffff;
	*cvu_ctrl2 = *cvu_ctrl2 | (boot_vector >> 10);
	*cvu_ctrlsys2 = 0x20080000;

	*cvu_ctrl3 &= ~0x3fffff;
	*cvu_ctrl3 = *cvu_ctrl3 | (boot_vector >> 10);
	*cvu_ctrlsys3 = 0x200C0000;

	cvu.u32 = NOC_REG_READ(addr + clk_offset);
	cvu.reg.cvu_rst = 0;
	NOC_REG_WRITE(addr + clk_offset, cvu.u32);

	*cvu_ctrlsys4 = 0x10000000;

	*cvu_ctrl_stu = *cvu_ctrl_stu | (1 << 0);

	*cvu_ctrl0 = *cvu_ctrl0 | (1<<25);
	*cvu_ctrl1 = *cvu_ctrl1 | (1<<25);
	*cvu_ctrl2 = *cvu_ctrl2 | (1<<25);
	*cvu_ctrl3 = *cvu_ctrl3 | (1<<25);

	iounmap(addr);
	iounmap(cvu_ctrl_addr);
	dev_dbg(dev, "EVXX is up, loaded firmware and out of reset");
	return 0;
}
#endif

static int sima_rproc_stop(struct rproc *rproc)
{

	void __iomem *addr = ioremap(PRC_BASE, 0x1000);
	uint32_t clk_offset = PRC_REG__CKG_RST_REG__CVU_CK_RST_ADDR;
	union cvu_ck_rst_u cvu;
	cvu.u32 = NOC_REG_READ(addr + clk_offset);
	cvu.reg.cvu_clk_en = 0;
	cvu.reg.cvu_ev74_clk_en = 0;
	cvu.reg.cvu_pclk_en = 0;
	cvu.reg.cvu_dbg_pclk_en = 0;
	NOC_REG_WRITE(addr + clk_offset, cvu.u32);

	/* udelay(100); */

	cvu.u32 = NOC_REG_READ(addr + clk_offset);
	cvu.reg.cvu_ev74_rst_n = 0;
	cvu.reg.cvu_presetdbgn = 0;
	cvu.reg.cvu_presetn = 0;
	cvu.reg.cvu_dbg_presetn = 0;
	cvu.reg.cvu_rst = 0;
	NOC_REG_WRITE(addr + clk_offset, cvu.u32);

	iounmap(addr);
	dev_info(rproc->dev.parent, "Stopped EV core");
	return 0;
}


static int sima_rproc_elf_load_rsc_table(struct rproc *rproc,
					 const struct firmware *fw)
{
	if (rproc_elf_load_rsc_table(rproc, fw)) {
		dev_warn(rproc->dev.parent,
				"no resource table found for this firmware\n");
	}

	return 0;
}

static int simaai_rproc_mem_alloc(struct rproc *rproc,
                                  struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	va = ioremap_wc(mem->dma, mem->len);
	if (!va) {
		dev_warn(dev, "Unable to map memory region: %pa+%zx\n",
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

static int sima_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct sima_rproc *srproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	struct rproc_mem_entry *mem;
	int i = 0;

	for (i = 0; i < SIMA_MAX_MEM_REGIONS; i++) {
		mem = rproc_mem_entry_init(dev, NULL,
							   (dma_addr_t)srproc->mem[i].bus_addr,
							   srproc->mem[i].size,
							   srproc->mem[i].dev_addr,
							   simaai_rproc_mem_alloc,
							   simaai_rproc_mem_release,
							   srproc->mem[i].name);

		if (!mem) {
			dev_err(dev, "ERROR : unable to initialize memory-region %s\n",
							srproc->mem[i].name);
			return -ENOMEM;
		}
		dev_info(dev,"srproc: region_name:[%s], phys_addr:[0x%lx], size:[0x%x],"
				"bus_addr:[0x%llx], cpu_addr:[0x%llx]", srproc->mem[i].name,
				srproc->mem[i].dev_addr, srproc->mem[i].size,
				srproc->mem[i].bus_addr, srproc->mem[i].cpu_addr);

		rproc_add_carveout(rproc, mem);
	}

	return sima_rproc_elf_load_rsc_table(rproc, fw);
}

static const struct rproc_ops sima_rproc_ops = {
	.start		= sima_rproc_start,
	.stop		= sima_rproc_stop,
	.parse_fw	= sima_parse_fw,
	.load		= rproc_elf_load_segments,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr	= rproc_elf_get_boot_addr,
	.kick		= sima_rproc_kick,
};

static const struct of_device_id sima_rproc_of_match[] = {
	{ .compatible = "simaai,evxx-rproc" },
	{},
};

MODULE_DEVICE_TABLE(of, sima_rproc_of_match);

static int sima_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sima_rproc *sproc;
	struct device_node *np = dev->of_node;
	const char *fw_name;
	struct rproc *rproc;
	int ret, i;
	struct resource *res;

	ret = of_property_read_string(dev->of_node, "sima,pm-firmware",
				      &fw_name);
	if (ret) {
		dev_err(dev, "No firmware filename given\n");
		return -ENODEV;
	}

	rproc = rproc_alloc(dev, np->name, &sima_rproc_ops, fw_name,
							sizeof(*sproc));
	if (!rproc) {
		ret = -ENOMEM;
		goto err;
	}
	sproc = rproc->priv;
	sproc->is_rtc_only = true;
	rproc->has_iommu = false;
	platform_set_drvdata(pdev, rproc);

	for (i = 0; i < SIMA_MAX_MEM_REGIONS; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, ev_mem_names[i]);
		if(!res) {
			dev_err(dev, "ERROR : getting mem region %s\n", ev_mem_names[i]);
			goto err_put_rproc;
		}

		sproc->mem[i].bus_addr = res->start;
		sproc->mem[i].dev_addr = res->start;
		sproc->mem[i].size = (res->end - res->start) + 1;
		strcpy(sproc->mem[i].name, ev_mem_names[i]);
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "ERROR : failed to add rproc handle\n");
		goto err_put_rproc;
	}

	dev_dbg(dev, "Successfully added rproc handle\n");

err_put_rproc:
	rproc_free(rproc);
err:
	return ret;
}

static int sima_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

/* TODO: SRIRAM Fix them with power management and ev platform driver
 * for poewr management */
#ifdef CONFIG_PM
static int sima_rpm_suspend(struct device *dev)
{
	return -EBUSY;
}

static int sima_rpm_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops sima_rproc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sima_rpm_suspend, sima_rpm_resume)
};

static struct platform_driver sima_rproc_driver = {
	.probe = sima_rproc_probe,
	.remove = sima_rproc_remove,
	.driver = {
		.name = "simaai-rproc-evxx",
		.of_match_table = sima_rproc_of_match,
		.pm = &sima_rproc_pm_ops,
	},
};

module_platform_driver(sima_rproc_driver);

MODULE_AUTHOR("Sriram Raghunathan <sriram.r@sima.ai>");
MODULE_AUTHOR("Manish Mathur <manish.mathur@sima.ai>");
MODULE_DESCRIPTION("SIMA.ai Remote processor management driver for EV processor family");
MODULE_LICENSE("Dual MIT/GPL");
