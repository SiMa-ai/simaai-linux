/*
 * SIMAAI Remote Processor driver for M4 processor family
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

#define NOC_REG_READ(x)                         (readl(x))
#define NOC_REG_WRITE(reg, val)         (writel(val, reg))

// work with only the M4-SRAM
#define SIMA_MAX_MEM_REGIONS 1

#define DAVINCI_MLA_CLK_EN           BIT(0)
#define DAVINCI_MLA_AHB_CLK_EN       BIT(1)
#define DAVINCI_MLA_APB_CLK_EN       BIT(2)
#define DAVINCI_MLA_AXI_CLK_EN       BIT(3)
#define DAVINCI_MLA_DBG_CLK_EN       BIT(4)
#define DAVINCI_MLA_AHB_RESET_N      BIT(5)
#define DAVINCI_MLA_APB_RESET_N      BIT(6)
#define DAVINCI_MLA_AXI_RESET_N      BIT(7)
#define DAVINCI_MLA_POR_RST          BIT(8)
#define DAVINCI_MLA_UCTRL_POR_RST    BIT(9)
#define DAVINCI_MLA_UCTRL_SYS_RST    BIT(10)
#define DAVINCI_MLA_DBG_RST_N        BIT(11)

#define DAVINCI_MLA_CLK_EN_MASK (DAVINCI_MLA_CLK_EN | DAVINCI_MLA_AHB_CLK_EN | \
							DAVINCI_MLA_APB_CLK_EN | DAVINCI_MLA_AXI_CLK_EN | \
							DAVINCI_MLA_DBG_CLK_EN)
#define DAVINCI_MLA_RESET_MASK (DAVINCI_MLA_AHB_RESET_N | DAVINCI_MLA_APB_RESET_N | \
							DAVINCI_MLA_AXI_RESET_N | DAVINCI_MLA_POR_RST | \
							DAVINCI_MLA_DBG_RST_N)

#define DAVINCI_M4_RESET_MASK (DAVINCI_MLA_UCTRL_POR_RST | DAVINCI_MLA_UCTRL_SYS_RST)

#define MODALIX_MLA_CLK_EN            BIT(0)
#define MODALIX_MLA_DBG_CLK_EN        BIT(1)
#define MODALIX_MLA_XXX_RESET_N       BIT(2)
#define MODALIX_MLA_UCTRL_POR_RST     BIT(3)
#define MODALIX_MLA_UCTRL_SYS_RST     BIT(4)
#define MODALIX_MLA_DBG_RST_N         BIT(5)

#define MODALIX_MLA_CLK_EN_MASK (MODALIX_MLA_CLK_EN | MODALIX_MLA_DBG_CLK_EN)
#define MODALIX_MLA_RESET_MASK (MODALIX_MLA_XXX_RESET_N | MODALIX_MLA_DBG_RST_N)
#define MODALIX_M4_RESET_MASK (MODALIX_MLA_UCTRL_SYS_RST | MODALIX_MLA_UCTRL_POR_RST)

const char *m4_mem_names[SIMA_MAX_MEM_REGIONS] = {
		"m4_sram"
};

struct simaai_rproc_config {
	phys_addr_t		prc_base_addr;
	phys_addr_t		m4_sram_base_addr;
	uint32_t		m4_sram_size;
	uint32_t		prc_mla_ck_rst_offset;
	uint32_t		mla_clk_en_mask;
	uint32_t		mla_reset_mask;
	uint32_t		m4_reset_mask;
	uint32_t		reset_bit_pos;
};

static const struct simaai_rproc_config davinci_rproc_config = {
	.prc_base_addr         = 0x30100000,
	.m4_sram_base_addr     = 0x01d00000,
	.m4_sram_size          = 0x00080000,
	.prc_mla_ck_rst_offset = 0x00000150,
	.mla_clk_en_mask       = DAVINCI_MLA_CLK_EN_MASK,
	.mla_reset_mask        = DAVINCI_MLA_RESET_MASK,
	.m4_reset_mask         = DAVINCI_M4_RESET_MASK,
	.reset_bit_pos         = 8u
};

static const struct simaai_rproc_config modalix_rproc_config = {
	.prc_base_addr         = 0x0ff00000,
	.m4_sram_base_addr     = 0x05100000,
	.m4_sram_size          = 0x00080000,
	.prc_mla_ck_rst_offset = 0x00000594,
	.mla_clk_en_mask       = MODALIX_MLA_CLK_EN_MASK,
	.mla_reset_mask        = MODALIX_MLA_RESET_MASK,
	.m4_reset_mask         = MODALIX_M4_RESET_MASK,
	.reset_bit_pos         = 31u // in mx there are no active high reset bits
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
	void __iomem * cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

/**
 * struct sima_rproc - sima remote processor state
 * @mem: sima memory information
 */
struct sima_rproc {
	struct sima_mem mem[SIMA_MAX_MEM_REGIONS];
	int is_rtc_only;
	struct simaai_rproc_config *config;
};

// TODO: Fix this, stub until mailbox & virtio are brought into dts
static void sima_rproc_kick(struct rproc *rproc, int vqid)
{
	return;
}

static int sima_rproc_start(struct rproc *rproc)
{
	uint32_t val = 0;
	void __iomem *addr;
	struct device *dev = rproc->dev.parent;
	const struct simaai_rproc_config *data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "No device match data found\n");
		return -ENODEV;
	}
	addr = ioremap(data->prc_base_addr + data->prc_mla_ck_rst_offset, 0x1000);

	val = readl(addr);
	writel(val & (~(data->m4_reset_mask)), addr);
	dev_dbg(dev, "Started M4.\n");

	iounmap(addr);
	return 0;
}

static int sima_rproc_stop(struct rproc *rproc)
{
	uint32_t val = 0;
	void __iomem *addr;
	struct device *dev = rproc->dev.parent;
	const struct simaai_rproc_config *data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "No device match data found\n");
		return -ENODEV;
	}
	addr = ioremap(data->prc_base_addr + data->prc_mla_ck_rst_offset, 0x1000);

	val = readl(addr);
	val &= ~data->mla_reset_mask;
	val |= data->m4_reset_mask;
	val |= BIT(data->reset_bit_pos);
	writel(val, addr);

	dev_dbg(dev, "Stopped M4 and MLA.\n");

	iounmap(addr);
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

static int sima_rproc_elf_load(struct rproc *rproc, const struct firmware *fw)
{
	uint32_t val = 0;
	const struct simaai_rproc_config *data;
	void __iomem *addr = NULL;
	void __iomem *sram = NULL;
	struct device *dev = rproc->dev.parent;

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "No device match data found\n");
		return -ENODEV;
	}
	addr = ioremap(data->prc_base_addr, 0x1000);

	val = readl(addr + data->prc_mla_ck_rst_offset);
	val |= data->mla_clk_en_mask;
	writel(val, addr + data->prc_mla_ck_rst_offset);
	dev_dbg(dev, "MLA clock enabled., 0x%08x\n", val);

	udelay(1);

	val |= (data->mla_reset_mask);
	val &= ~(BIT(data->reset_bit_pos));
	dev_dbg(dev, "Starting MLA., 0x%08x\n", val);
	writel(val, addr + data->prc_mla_ck_rst_offset);

	iounmap(addr);

	udelay(1);

	// workaround for SWMLA-4552
	dev_dbg(dev, "M4 SRAM zero init...\n");
	sram = ioremap(data->m4_sram_base_addr, data->m4_sram_size);
	dev_dbg(dev, "M4 SRAM %lx\n", sram);
	memset_io(sram, 0, data->m4_sram_size);
	if (!sram) {
		dev_err(dev, "Sram addr null\n");
		return -ENODEV;
	}

	iounmap(sram);
	return rproc_elf_load_segments(rproc, fw);
}

static int simaai_rproc_mem_alloc(struct rproc *rproc,
				struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;
	
	va = ioremap_wc(mem->dma, mem->len);
	if (!va) {
		dev_warn(dev, "ERROR : unable to map memory region: %pa+%zx\n",
			 &mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	dev_dbg(dev, "da:0x%llx, len:%lu va:0x%p\n", mem->dma, mem->len, va);
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
	int i;

	for (i = 0; i < SIMA_MAX_MEM_REGIONS; i++) {
		mem = rproc_mem_entry_init(dev, NULL,
					   (dma_addr_t)srproc->mem[i].bus_addr, srproc->mem[i].size,
						srproc->mem[i].dev_addr,
						simaai_rproc_mem_alloc,
						simaai_rproc_mem_release,
						srproc->mem[i].name);
		if (!mem) {
			dev_err(dev, "ERROR : unable to initialize memory-region %s\n",
						srproc->mem[i].name);
			return -ENOMEM;
		}

		dev_dbg(dev, "region_name:[%s],phys_addr:[0x%x],size:[0x%lx],"
						"bus_addr:[0x%llx],cpu_addr:[0x%llx]",
						srproc->mem[i].name, srproc->mem[i].dev_addr,
						srproc->mem[i].size, srproc->mem[i].bus_addr,
						(uint64_t)srproc->mem[i].cpu_addr);

    	rproc_add_carveout(rproc, mem);

	}

	return sima_rproc_elf_load_rsc_table(rproc, fw);
}

static const struct rproc_ops sima_rproc_ops = {
	.start					= sima_rproc_start,
	.stop					= sima_rproc_stop,
	.load               	= sima_rproc_elf_load,
	.parse_fw	        	= sima_parse_fw,
	.find_loaded_rsc_table 	= rproc_elf_find_loaded_rsc_table,
	.get_boot_addr			= rproc_elf_get_boot_addr,
	// [SRIRAM] TODO: Improve the kick to handle mbox init, i.e I/O with m4
	.kick               	= sima_rproc_kick,
	.sanity_check			= rproc_elf_sanity_check,
};

static const struct of_device_id sima_rproc_of_match[] = {
	{ .compatible = "simaai,davinci-m4-rproc", .data = &davinci_rproc_config},
	{ .compatible = "simaai,modalix-m4-rproc", .data = &modalix_rproc_config},
	{},
};
MODULE_DEVICE_TABLE(of, sima_rproc_of_match);

static int sima_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sima_rproc *sproc;
	const char *fw_name;
	const char *cpu_name;
	struct rproc *rproc;
	struct resource *res;
	int ret, i;

	ret = of_property_read_string(dev->of_node, "simaai,pm-firmware", &fw_name);
	if (ret) {
		dev_err(dev, "ERROR : Firmware name is missing\n");
		return -ENODEV;
	}
	ret = of_property_read_string(dev->of_node, "cpu-name", &cpu_name);
	if (ret) {
			dev_err(dev, "ERROR : cpu name is missing\n");
			return -ENODEV;
	}

	rproc = rproc_alloc(dev, cpu_name, &sima_rproc_ops, fw_name,
							sizeof(*sproc));
	if (!rproc) {
		dev_err(dev, "ERROR : allocating rproc handle\n");
		ret = -ENOMEM;
		goto err;
	}

	sproc = rproc->priv;
	rproc->has_iommu = false;
	sproc->is_rtc_only = true;
	platform_set_drvdata(pdev, rproc);

	for (i = 0; i < SIMA_MAX_MEM_REGIONS; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, m4_mem_names[i]);
		if (!res) {
			dev_err(dev, "ERROR : getting mem region %s\n", m4_mem_names[i]);
			goto err_put_rproc;
		}

		sproc->mem[i].bus_addr = res->start;
		sproc->mem[i].dev_addr = 0;
		sproc->mem[i].size = (res->end - res->start) + 1;
		strcpy(sproc->mem[i].name, m4_mem_names[i]);
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "ERROR : Failed to add rproc handle\n");
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
		.name = "simaai-rproc-m4",
		.of_match_table = sima_rproc_of_match,
		.pm = &sima_rproc_pm_ops,
	},
};

module_platform_driver(sima_rproc_driver);

MODULE_AUTHOR("Sriram Raghunathan <sriram.r@sima.ai>");
MODULE_AUTHOR("Nilesh Raghuvanshi <nilesh.r@sima.ai>");
MODULE_DESCRIPTION("SIMA.ai remote processor control driver for M4");
MODULE_LICENSE("Dual MIT/GPL");
