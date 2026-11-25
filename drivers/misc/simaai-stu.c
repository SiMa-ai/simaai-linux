// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2021 Sima ai
 *
 * Author: Nilesh Raghuvanshi <nilesh.r@sima.ai>
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define STU_MAX_WINDOWS				8
#define STU_VERSION_OFFSET			0x0
#define STU_GFILL_OFFSET			0x20
#define STU_WBASE_OFFSET			0x400
#define STU_WINDOW_BITS				3
#define STU_ORIGINAL_ADD_START_BIT	(32 - STU_WINDOW_BITS)

struct simaai_stu {
	struct device		*dev;
	void __iomem		*regs;
	u32			base[STU_MAX_WINDOWS];
	DECLARE_BITMAP(programmed, STU_MAX_WINDOWS);
	u32			stu_gfill_value;
};

static const struct of_device_id simaai_stu_id_table[] = {
	{ .compatible = "simaai,stu" },
	{}, /* sentinel */
};
MODULE_DEVICE_TABLE(of, simaai_stu_id_table);

static struct platform_driver simaai_stu_driver;

/**
 * simaai_stu_set_window() - map a bus (device) address to a physical one
 * @stu:  STU handle
 * @bus:  bus/device address; the window is (bus >> 29). Low 29 bits are
 *        ignored - a window maps the whole 512MB block.
 * @phys: target physical address (window value = phys >> 29)
 *
 * Address-based to match simaai_stu_get_{bus,dev}_address(); callable at
 * runtime by other drivers to (re)program a window.
 */
static int simaai_stu_set_window(struct simaai_stu *stu, dma_addr_t bus, phys_addr_t phys)
{
	u32 id = bus >> STU_ORIGINAL_ADD_START_BIT;

	if (!stu || id >= STU_MAX_WINDOWS)
		return -EINVAL;

	stu->base[id] = phys >> STU_ORIGINAL_ADD_START_BIT;
	set_bit(id, stu->programmed);
	writel(stu->base[id], stu->regs + STU_WBASE_OFFSET + id * 4);
	dev_dbg(stu->dev, "bus %pad -> phys %pap (window %u)\n", &bus, &phys, id);

	return 0;
}

struct simaai_stu *simaai_stu_get_by_phandle(struct device_node *np,
			const char *phandle_name)
{
	struct platform_device *pdev;
	struct device_node *stu_np;
	struct simaai_stu *stu;

	stu_np = of_parse_phandle(np, phandle_name, 0);
	if (!stu_np)
		return ERR_PTR(-EINVAL);

	if (!of_match_node(simaai_stu_driver.driver.of_match_table, stu_np)) {
		of_node_put(stu_np);
		return ERR_PTR(-EINVAL);
	}

	pdev = of_find_device_by_node(stu_np);
	of_node_put(stu_np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	stu = platform_get_drvdata(pdev);
	if (!stu) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return stu;
}
EXPORT_SYMBOL(simaai_stu_get_by_phandle);

int simaai_stu_get_bus_address(struct simaai_stu *stu, dma_addr_t phys_addr,
				dma_addr_t *bus_addr)
{
	u32 val, id;
	int ret = -EINVAL;

	if (!stu)
		return -EINVAL;

	val = phys_addr >> STU_ORIGINAL_ADD_START_BIT;

	for_each_set_bit(id, stu->programmed, STU_MAX_WINDOWS) {
		if (stu->base[id] == val) {
			*bus_addr = (phys_addr & (BIT(STU_ORIGINAL_ADD_START_BIT) - 1)) |
				    ((dma_addr_t)id << STU_ORIGINAL_ADD_START_BIT);
			ret = 0;
			break;
		}
	}

	if (ret)
		dev_dbg(stu->dev, "No match found for phys addr : %pad\n", &phys_addr);
	return ret;
}
EXPORT_SYMBOL(simaai_stu_get_bus_address);

int simaai_stu_get_dev_address(struct simaai_stu *stu, dma_addr_t bus_addr,
				dma_addr_t *dev_addr)
{
	u32 window_id;
	int ret = 0;

	if (!stu)
		return -EINVAL;

	window_id = bus_addr >> STU_ORIGINAL_ADD_START_BIT;
	if (window_id >= STU_MAX_WINDOWS)
		return -EINVAL;

	if (test_bit(window_id, stu->programmed))
		*dev_addr = (bus_addr & (BIT(STU_ORIGINAL_ADD_START_BIT) - 1)) |
			    ((u64)stu->base[window_id] << STU_ORIGINAL_ADD_START_BIT);
	else
		ret = -EINVAL;

	if (ret)
		dev_err(stu->dev, "No mapping for bus addr : %pad\n", &bus_addr);
	return ret;
}
EXPORT_SYMBOL(simaai_stu_get_dev_address);

void simaai_stu_put(struct simaai_stu *stu)
{
	put_device(stu->dev);
}

/* true if some programmed window already maps this 512MB block */
static bool stu_block_mapped(struct simaai_stu *stu, u32 blk)
{
	u32 id;

	for_each_set_bit(id, stu->programmed, STU_MAX_WINDOWS)
		if (stu->base[id] == blk)
			return true;
	return false;
}

static int simaai_stu_region_map(struct simaai_stu *stu)
{
	struct device *dev = stu->dev;
	u32 range[2], win, win_end;
	int i, nregions;

	nregions = of_count_phandle_with_args(dev->of_node, "memory-region", NULL);
	if (nregions <= 0)
		return dev_err_probe(dev, -EINVAL, "missing memory-region\n");
	if (of_property_read_u32_array(dev->of_node, "simaai,stu-bus-range", range, 2))
		return dev_err_probe(dev, -EINVAL, "missing simaai,stu-bus-range\n");

	win     = range[0] >> STU_ORIGINAL_ADD_START_BIT;
	win_end = range[1] >> STU_ORIGINAL_ADD_START_BIT;
	if (win > win_end || win_end >= STU_MAX_WINDOWS)
		return dev_err_probe(dev, -EINVAL, "bad simaai,stu-bus-range\n");

	for (i = 0; i < nregions; i++) {
		struct device_node *mem_np;
		struct reserved_mem *rmem;
		u32 b, nblk;

		mem_np = of_parse_phandle(dev->of_node, "memory-region", i);
		if (!mem_np)
			return dev_err_probe(dev, -EINVAL, "no memory-region[%d]\n", i);
		rmem = of_reserved_mem_lookup(mem_np);
		of_node_put(mem_np);
		if (!rmem)
			return dev_err_probe(dev, -EINVAL, "region %d not reserved\n", i);

		nblk = (u32)((rmem->base + rmem->size - 1) >> STU_ORIGINAL_ADD_START_BIT)
		     - (u32)(rmem->base >> STU_ORIGINAL_ADD_START_BIT) + 1;

		for (b = 0; b < nblk; b++) {
			phys_addr_t phys = rmem->base + (phys_addr_t)b * SZ_512M;
			u32 blk = phys >> STU_ORIGINAL_ADD_START_BIT;

			if (stu_block_mapped(stu, blk))		/* overlap: already reachable */
				continue;
			while (win <= win_end && test_bit(win, stu->programmed))
				win++;				/* step over reserved windows */
			if (win > win_end)
				return dev_err_probe(dev, -EINVAL,
						     "regions exceed simaai,stu-bus-range\n");
			simaai_stu_set_window(stu,
					      (dma_addr_t)win << STU_ORIGINAL_ADD_START_BIT, phys);
			win++;
		}
	}

	return 0;
}


static int simaai_stu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simaai_stu *stu;
	int rc;

	stu = devm_kzalloc(dev, sizeof(*stu), GFP_KERNEL);
	if (!stu)
		return -ENOMEM;
	stu->dev = dev;

	stu->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(stu->regs))
		return dev_err_probe(dev, PTR_ERR(stu->regs),
				     "Failed to map registers\n");

	dev_dbg(dev, "Build version : %#x\n",
		readl(stu->regs + STU_VERSION_OFFSET));

	rc = of_property_read_u32(dev->of_node, "simaai,stu-gfill-value",
				  &stu->stu_gfill_value);
	if (rc)
		return dev_err_probe(dev, rc, "Failed getting STU gfill value\n");
	writel(stu->stu_gfill_value, stu->regs + STU_GFILL_OFFSET);

	/* For both ISP and EV74 window 0 is used to reach their respective
	 * MMIO registers.Since it doesn't come under any memory-region,
	 * so need to hardcode it here.
	 */
	simaai_stu_set_window(stu, 0, 0);

	rc = simaai_stu_region_map(stu);
	if (rc)
		return rc;

	platform_set_drvdata(pdev, stu);
	dev_info(dev, "programmed %u window(s)\n",
		 bitmap_weight(stu->programmed, STU_MAX_WINDOWS));

	return 0;
}

static struct platform_driver simaai_stu_driver = {
	.driver = {
		.name = "stu-manager",
		.of_match_table = of_match_ptr(simaai_stu_id_table),
	},
	.probe = simaai_stu_probe,
};
module_platform_driver(simaai_stu_driver);

MODULE_AUTHOR("Nileshkumar Raghuvanshi <nilesh.r@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai STU IP driver");
MODULE_LICENSE("Dual MIT/GPL");

