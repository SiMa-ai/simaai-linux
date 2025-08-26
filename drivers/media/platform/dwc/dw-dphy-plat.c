// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Synopsys, Inc.
 *
 * Synopsys MIPI D-PHY controller driver.
 * Platform dependent functions
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#include "dw-dphy-rx.h"

static struct phy *dw_dphy_xlate(struct device *dev,
				 struct of_phandle_args *args)
{
	struct dw_dphy_rx *dphy = dev_get_drvdata(dev);

	return dphy->phy;
}

static ssize_t dphy_reset_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);
	char buffer[15];

	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 0);
	usleep_range(100, 200);
	dw_dphy_write(dphy, R_CSI2_DPHY_RSTZ, 1);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t dphy_freq_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret;
	unsigned long freq;

	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);

	ret = kstrtoul(buf, 10, &freq);
	if (ret < 0)
		return ret;

	if (freq > 2500) {
		dev_info(dev, "Freq must be under 2500 Mhz\n");
		return count;
	}
	if (freq < 80) {
		dev_info(dev, "Freq must be over 80 Mhz\n");
		return count;
	}

	dev_info(dev, "Data Rate %lu Mbps\n", freq);
	dphy->dphy_freq = freq * 1000;

	return count;
}

static ssize_t dphy_freq_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);
	char buffer[15];

	snprintf(buffer, 15, "Freq %d\n", dphy->dphy_freq / 1000);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t dphy_addr_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);
	unsigned long val;
	int ret;
	u8 addr, payload;

	ret = kstrtoul(buf, 32, &val);
	if (ret < 0)
		return ret;

	payload = (u16)val;
	addr = (u16)(val >> 16);

	dev_info(dev, "addr 0x%lX\n", val);
	dev_info(dev, "payload: 0x%X\n", addr);
	dev_info(dev, "Addr [0x%x] -> 0x%x\n", (unsigned int)addr,
		 dw_dphy_te_read(dphy, addr));

	return count;
}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
static ssize_t idelay_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);
	char buffer[15];

	snprintf(buffer, 15, "idelay %d\n", dw_dphy_if_get_idelay(dphy));

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t idelay_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);
	int ret;
	unsigned long val;
	u8 lane, delay;

	ret = kstrtoul(buf, 16, &val);
	if (ret < 0)
		return ret;

	lane = (u8)val;
	delay = (u8)(val >> 8);

	dev_dbg(dev, "Lanes %u\n", lane);
	dev_dbg(dev, "Delay %u\n", delay);

	dw_dphy_if_set_idelay_lane(dphy, delay, lane);

	return count;
}
#endif

static ssize_t len_config_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int ret;
	unsigned long length;

	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);

	ret = kstrtoul(buf, 10, &length);
	if (ret < 0)
		return ret;

	if (length == BIT8)
		pr_info("Configured for 8-bit interface\n");
	else if (length == BIT12)
		pr_info("Configured for 12-bit interface\n");
	else
		return count;

	dphy->dphy_te_len = length;

	return count;
}

static ssize_t len_config_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);
	char buffer[20];

	snprintf(buffer, 20, "Length %d\n", dphy->dphy_te_len);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t dw_dphy_g118_settle_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned long lp_time;

	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);

	ret = kstrtoul(buf, 10, &lp_time);
	if (ret < 0)
		return ret;

	if ((lp_time > 1) && (lp_time < 10000))
		dphy->lp_time = lp_time;
	else {
		pr_info("Invalid Value configuring for 1000 ns\n");
		dphy->lp_time = 1000;
	}

	dphy->lp_time = lp_time;

	return count;
}

static ssize_t dw_dphy_g118_settle_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);
	char buffer[10];

	snprintf(buffer, 10, "Settle %d ns\n", dphy->lp_time);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static DEVICE_ATTR_RO(dphy_reset);
static DEVICE_ATTR_RW(dphy_freq);
static DEVICE_ATTR_WO(dphy_addr);
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
static DEVICE_ATTR_RW(idelay);
#endif
static DEVICE_ATTR_RW(len_config);
static DEVICE_ATTR_RW(dw_dphy_g118_settle);

static struct phy_ops dw_dphy_ops = {
	.init = dw_dphy_init,
	.reset = dw_dphy_reset,
	.configure = dw_dphy_config,
	.power_on = dw_dphy_power_on,
	.power_off = dw_dphy_power_off,
	.owner = THIS_MODULE,
};

static int dw_dphy_rx_probe(struct platform_device *pdev)
{
	struct dw_dphy_rx *dphy;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct phy_provider *phy_provider;
	struct phy *phy;

	dphy = devm_kzalloc(dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;

	dphy->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dphy->base_address = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(dphy->base_address)) {
		dev_err(dev, "error requesting base address\n");
		return PTR_ERR(dphy->base_address);
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dphy->glue_base_address = devm_ioremap_resource(dev, res);
	if (IS_ERR(dphy->glue_base_address)) {
		dev_err(dev, "error requesting glue reg base address for dphy\n");
		return PTR_ERR(dphy->glue_base_address);
	}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dphy->dphy1_if_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(dphy->dphy1_if_addr)) {
		dev_err(dev, "error requesting dphy 1 if regbank\n");
		return PTR_ERR(dphy->dphy1_if_addr);
	}

	dphy->max_lanes =
		dw_dphy_if_read_msk(dphy, DPHYID, DPHY_ID_LANE_SUPPORT, 4);

	dphy->dphy_gen = dw_dphy_if_read_msk(dphy, DPHYID, DPHY_ID_GEN, 4);
	dev_info(dev, "DPHY GEN %s with maximum %s lanes\n",
		 dphy->dphy_gen == GEN3 ? "3" : "2",
		 dphy->max_lanes == CTRL_8_LANES ? "8" : "4");

	if (dphy->max_lanes == CTRL_8_LANES) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		dphy->dphy2_if_addr =
			devm_ioremap(dev, res->start, resource_size(res));

		if (IS_ERR(dphy->dphy2_if_addr)) {
			dev_err(dev, "error requesting dphy 2 if regbank\n");
			return PTR_ERR(dphy->dphy2_if_addr);
		}

		dphy->config_gpio = of_get_gpio(dev->of_node, 0);
		if (!gpio_is_valid(dphy->config_gpio)) {
			dev_err(dev, "failed to parse config gpio\n");
			return dphy->config_gpio;
		}
	}
#endif
	if (of_property_read_u32(dev->of_node, "snps,dphy-frequency",
				 &dphy->dphy_freq)) {
		dev_err(dev, "failed to find dphy frequency\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "snps,dphy-te-len",
				 &dphy->dphy_te_len)) {
		dev_err(dev, "failed to find dphy te length\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "simaai,mipi-generator",
				 &dphy->mipi_generator)) {
		dev_err(dev, "failed to find mipi generator field\n");
		return -EINVAL;
	} else {
		dev_info(dev, "MIPI generator is %s connected",(dphy->mipi_generator)?"":"not");
	}

	dev_set_drvdata(dev, dphy);
	spin_lock_init(&dphy->slock);

	phy = devm_phy_create(dev, NULL, &dw_dphy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(phy);
	}

	dphy->phy = phy;
	phy_set_drvdata(phy, dphy);

	phy_provider = devm_of_phy_provider_register(dev, dw_dphy_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "error getting phy provider\n");
		return PTR_ERR(phy_provider);
	}

	dphy->lp_time = 1000; /* 1000 ns */
	dphy->lanes_config = dw_dphy_setup_config(dphy);
	dev_dbg(dev, "rx-dphy created\n");

	device_create_file(&pdev->dev, &dev_attr_dphy_reset);
	device_create_file(&pdev->dev, &dev_attr_dphy_freq);
	device_create_file(&pdev->dev, &dev_attr_dphy_addr);
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	device_create_file(&pdev->dev, &dev_attr_idelay);
#endif
	device_create_file(&pdev->dev, &dev_attr_len_config);
	device_create_file(&pdev->dev, &dev_attr_dw_dphy_g118_settle);
	dev_info(dev, "rx-dphy done!!! \n");

	return 0;
}

static const struct of_device_id dw_dphy_rx_of_match[] = {
	{ .compatible = "snps,dphy-rx" },
	{},
};
MODULE_DEVICE_TABLE(of, dw_dphy_rx_of_match);

static struct platform_driver dw_dphy_rx_driver =
	{ .probe = dw_dphy_rx_probe,
	  .driver = {
		  .of_match_table = dw_dphy_rx_of_match,
		  .name = "snps-dphy-rx",
		  .owner = THIS_MODULE,
	  } };
module_platform_driver(dw_dphy_rx_driver);

MODULE_DESCRIPTION("Synopsys DesignWare MIPI DPHY Rx driver");
MODULE_AUTHOR("Luis Oliveira <lolivei@synopsys.com>");
MODULE_LICENSE("GPL v2");
