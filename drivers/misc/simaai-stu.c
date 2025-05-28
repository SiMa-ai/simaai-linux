// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2021 Sima ai
 *
 * Author: Nilesh Raghuvanshi <nilesh.r@sima.ai>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/platform_device.h>


#define STU_MAX_WINDOWS				8
#define STU_VERSION_OFFSET			0x0
#define STU_GFILL_OFFSET			0x20
#define STU_WBASE_OFFSET			0x400
#define STU_WINDOW_BITS				3
#define STU_ORIGINAL_ADD_START_BIT	(32 - STU_WINDOW_BITS)

struct __stu_wbase {
	u32 window_id;
	u32 value;
};

struct simaai_stu {
	struct device		*dev;
	struct __stu_wbase	*stu_wbase;
	void __iomem		*regs;
	u32					stu_gfill_value;
	int					num_of_windows;
};

static const struct of_device_id simaai_stu_id_table[] = {
	{ .compatible = "simaai,stu" },
	{} /* sentinel */
};

MODULE_DEVICE_TABLE(of, simaai_stu_id_table);

static struct platform_driver simaai_stu_driver;

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

	unsigned int iter = 0;
	unsigned int val = 0;

	if (!stu)
		return -EINVAL;

	val = phys_addr >> STU_ORIGINAL_ADD_START_BIT;

	for (iter = 0; iter < stu->num_of_windows; iter++) {
		if (val == stu->stu_wbase[iter].value) {
			*bus_addr = (phys_addr & (BIT(STU_ORIGINAL_ADD_START_BIT) - 1)) |
						stu->stu_wbase[iter].window_id << STU_ORIGINAL_ADD_START_BIT;
			return 0;
		}
	}

	dev_err(stu->dev, "No match found for phys addr : %pad\n", &phys_addr);
	return -EINVAL;
}
EXPORT_SYMBOL(simaai_stu_get_bus_address);

int simaai_stu_get_dev_address(struct simaai_stu *stu, dma_addr_t bus_addr, dma_addr_t *dev_addr)
{

	unsigned int iter = 0;
	unsigned int window_id = 0;

	if (!stu)
		return -EINVAL;

	window_id = bus_addr >> STU_ORIGINAL_ADD_START_BIT;

	for (iter = 0; iter < stu->num_of_windows; iter++) {
		if (window_id == stu->stu_wbase[iter].window_id) {
			*dev_addr = (bus_addr & (BIT(STU_ORIGINAL_ADD_START_BIT) - 1)) |
						(((u64)stu->stu_wbase[iter].value) << STU_ORIGINAL_ADD_START_BIT);
			return 0;
		}
	}

	dev_err(stu->dev, "No match found for phys addr : %pad\n", &bus_addr);
	return -EINVAL;
}
EXPORT_SYMBOL(simaai_stu_get_dev_address);

void simaai_stu_put(struct simaai_stu *stu)
{
	put_device(stu->dev);
}

static int simaai_stu_probe(struct platform_device *pdev)
{
	struct simaai_stu *stu;
	struct device *dev = &pdev->dev;
	int rc = 0;
	int iter = 0;
	u32 index = 0;
	u32 val = 0;
	int offset = 0;

	stu = devm_kzalloc(&pdev->dev, sizeof(*stu), GFP_KERNEL);
	if (!stu)
		return -ENOMEM;

	stu->dev = &pdev->dev;

	stu->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(stu->regs)) {
		dev_err(dev, "Failed to map base address register\n");
		return PTR_ERR(stu->regs);
	}

	dev_dbg(dev, "Build version : %#x\n", readl(stu->regs));

	rc = of_property_read_u32(dev->of_node, "simaai,stu-gfill-value", &stu->stu_gfill_value);
	if (rc) {
		dev_err(dev, "Failed getting STU gfill value\n");
		return -EINVAL;
	}

	dev_dbg(dev, "Gfill value : %#x\n", stu->stu_gfill_value);
	writel(stu->stu_gfill_value, stu->regs + STU_GFILL_OFFSET);

	stu->num_of_windows = of_property_count_elems_of_size(dev->of_node, "simaai,stu-table", sizeof(u32)*2);
	if (stu->num_of_windows < 0) {
		dev_err(dev, "Failed to read STU table\n");
		return -EINVAL;
	}

	if (stu->num_of_windows > STU_MAX_WINDOWS) {
		dev_warn(dev, "Number of enteries are %d, more than supported, truncating it to %d\n",
				stu->num_of_windows, STU_MAX_WINDOWS);
		stu->num_of_windows = STU_MAX_WINDOWS;
	}

	stu->stu_wbase = devm_kzalloc(&pdev->dev, sizeof(struct __stu_wbase) * (stu->num_of_windows), GFP_KERNEL);
	if (!stu->stu_wbase)
		return -ENOMEM;

	for (iter = 0; iter < stu->num_of_windows; iter++) {

		offset = iter * 2;
		if (of_property_read_u32_index(dev->of_node, "simaai,stu-table", offset, &index)) {
			dev_err(dev, "Failed reading STU index %d\n", iter);
			return -EINVAL;
		}

		if (index >= STU_MAX_WINDOWS) {
			dev_err(dev, "Invalid stu table index %u at entry %u\n", index, iter);
			return -EINVAL;
		}

		if (of_property_read_u32_index(dev->of_node, "simaai,stu-table", offset + 1,
					&val)) {
			dev_err(dev, "Failed reading value for STU index %d\n", iter);
			return -EINVAL;
		}

		stu->stu_wbase[iter].window_id = index;
		stu->stu_wbase[iter].value = val;
	}

	for (iter = 0; iter < stu->num_of_windows; iter++) {
		writel(stu->stu_wbase[iter].value,
			(stu->regs + STU_WBASE_OFFSET + (stu->stu_wbase[iter].window_id)*4));
	}

	platform_set_drvdata(pdev, stu);

	dev_info(dev, "Successfully programmed %d entries\n", stu->num_of_windows);

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
