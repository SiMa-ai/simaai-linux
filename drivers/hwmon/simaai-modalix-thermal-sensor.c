// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2025 Sima.ai
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include "simaai-modalix-pvt-addr-map.h"

/* --- Constants --- */
#define SDIF_BUSY_BIT BIT(0)
#define SDIF_LOCK_BIT BIT(1)
#define SDIF_POLL_DELAY_MS 1
#define SDIF_DONE_DELAY_US 10
#define MAX_TEMP_CHANNELS 14
#define MAX_RETRIES 10

#define SDIF_MANUAL_EXTRA_BIT BIT(21)
#define SDIF_PROG_BIT BIT(31)
#define SDIF_WRN_BIT BIT(27)
#define SDIF_ADDR_MASK GENMASK(26, 24)
#define SDIF_WDATA_MASK GENMASK(23, 0)

#define IP_CFG0_ID_MASK GENMASK(15, 8)
#define IP_POLLING_ID_MASK GENMASK(23, 16)
#define TEMP_FLAGS (HWMON_T_INPUT | HWMON_T_LABEL)

#define CLK_SYNTH_EN_BIT BIT(24)
#define CLK_SYNTH_HI_MASK GENMASK(15, 8)
#define CLK_SYNTH_LO_MASK GENMASK(7, 0)

#define SDIF_WAIT_TIMEOUT_MS 100
#define SDIF_CH_DATA_STRIDE 0x04
#define CLK_SYNTH_DIVIDER 0x04

/* IP control register addresses */
#define IP_CTRL 0x0
#define IP_CFG0 0x1
#define IP_CFGA 0x2
#define IP_DATA 0x3
#define IP_POLLING 0x4
#define IP_TMR 0x5
#define IP_CFG1 0x6

#define IP_PD BIT(0)
#define IP_RUN_ONCE BIT(2)
#define IP_VM_MODE BIT(10)
#define IP_AUTO BIT(8)
#define DTSN_MODE BIT(0)
#define IP_CONFIG3_N6 0xA3
#define IP_CLK_CYCLES BIT(9)
#define VM_DTS_CH_REQ BIT(21)

#define IP_CFG0_CONFIG DTSN_MODE
#define IP_CFG1_CONFIG IP_CONFIG3_N6
#define IP_TIMER_CONFIG IP_CLK_CYCLES
#define IP_POLLING_CONFIG VM_DTS_CH_REQ
#define IP_CTRL_CONFIG2 (IP_RUN_ONCE | IP_VM_MODE | IP_AUTO)
#define IP_CTRL_CONFIG1 IP_PD

#define DTSN_Y_COEFF 6989
#define DTSN_K_OFFSET 2830
#define DTSN_SCALE_MDEG 100
#define DTSN_INPUT_SHIFT 4096

struct thermal_priv {
	void __iomem *pvt_base; /* Base address for PVT block */
	struct device *dev; /* Back-reference to device */
	struct device *hwmon_dev; /* HWMON registered device */
	struct mutex lock; /* Per-device lock */
};

static const char *const rtsn_name[MAX_TEMP_CHANNELS] = {
	"CH-0",	       "MLA RTSN-1",  "MLA RTSN-2",  "MLA RTSN-3", "APU RTSN-4",
	"CVU RTSN-5",  "TOP RTSN-6",  "MLA RTSN-7",  "MLA RTSN-8", "MLA RTSN-9",
	"MLA RTSN-10", "APU RTSN-11", "CVU RTSN-12", "TOP RTSN-13"
};

static inline int sdif_wait_ready(void __iomem *reg, int timeout_ms)
{
	u32 status;

	while (timeout_ms-- > 0) {
		status = ioread32(reg);
		if (!(status & (SDIF_BUSY_BIT | SDIF_LOCK_BIT)))
			return 0; // Ready
		msleep(SDIF_POLL_DELAY_MS);
	}
	// Final read to report error
	status = ioread32(reg);
	pr_err("SDIF timeout: status = 0x%08x (busy/lock bits still set)\n",
	       status);
	return -ETIMEDOUT;
}

static int convert_to_millicelsius(u32 raw)
{
	s32 temp;
	temp = ((raw * DTSN_Y_COEFF) / DTSN_INPUT_SHIFT) - DTSN_K_OFFSET;
	return temp * DTSN_SCALE_MDEG;
}

static inline u32 build_sdif_write(u8 reg_addr, u32 data_24bit, bool is_write)
{
	u32 val = SDIF_PROG_BIT | FIELD_PREP(SDIF_ADDR_MASK, reg_addr) |
		  (data_24bit & SDIF_WDATA_MASK);

	if (is_write)
		val |= SDIF_WRN_BIT;

	return val;
}

static int start_measurement_sequence(struct thermal_priv *priv, int id)
{
	void __iomem *pvt_base = priv->pvt_base;
	u32 raw, data;
	int i, ret = 0, retries = MAX_RETRIES;

	mutex_lock(&priv->lock);

#define SEQ_WRITE(reg, val)									\
	do {											\
		u32 wdata = (val);								\
		data = build_sdif_write((reg), wdata, true);					\
		dev_dbg(priv->dev, "[%s] write: 0x%08x\n", #reg, data);				\
		iowrite32(data,	pvt_base +							\
				PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_SDIF_ADDR);		\
		ret = sdif_wait_ready(pvt_base +						\
				PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_SDIF_STATUS_ADDR,	\
				SDIF_WAIT_TIMEOUT_MS);						\
		if (ret)									\
			goto unlock_and_return;							\
	} while (0)

	SEQ_WRITE(IP_CFG0, FIELD_PREP(IP_CFG0_ID_MASK, id));
	SEQ_WRITE(IP_CFG1, IP_CFG1_CONFIG);
	SEQ_WRITE(IP_TMR, IP_TIMER_CONFIG);
	SEQ_WRITE(IP_POLLING, IP_POLLING_CONFIG | BIT(id) |
				      FIELD_PREP(IP_POLLING_ID_MASK, id));
	SEQ_WRITE(IP_CTRL, IP_CTRL_CONFIG2);

#undef SEQ_WRITE

	/* Wait for SDIF_DONE to become non-zero */
	do {
		data = ioread32(
			pvt_base +
			PVT_REG__PVT_DTS0_IP_REGS__VM_DTS_SDIF_DONE_ADDR);
		if (data)
			break;
		udelay(SDIF_DONE_DELAY_US);
	} while (--retries);

	if (!retries) {
		dev_err(priv->dev,
			"Timeout: SDIF_DONE did not become non-zero\n");
		ret = -ETIMEDOUT;
		goto unlock_and_return;
	}

	/* Read SDIF result only for channel 0 */
	if (!id) {
		for (i = 0; i < MAX_TEMP_CHANNELS; i++) {
			raw = ioread32(
				pvt_base +
				PVT_REG__PVT_DTS0_IP_REGS__VM_DTS_CH_SDIF_DATA_ARRAY_ADDR +
				i * SDIF_CH_DATA_STRIDE);
			dev_dbg(priv->dev, "Temp CH-%d (0x%04X) = 0x%05X\n", i,
				PVT_REG__PVT_DTS0_IP_REGS__VM_DTS_CH_SDIF_DATA_ARRAY_ADDR +
					i * SDIF_CH_DATA_STRIDE,
				raw);
		}
	}

unlock_and_return:
	mutex_unlock(&priv->lock);
	return ret;
}

static u32 read_temperature_raw(void __iomem *pvt_base, int id)
{
	u32 raw = 0;
	raw = ioread32(
		pvt_base +
		PVT_REG__PVT_DTS0_IP_REGS__VM_DTS_CH_SDIF_DATA_ARRAY_ADDR +
		id * SDIF_CH_DATA_STRIDE);
	return raw;
}

static int thermal_hwmon_read_string(struct device *dev,
				     enum hwmon_sensor_types type, u32 attr,
				     int channel, const char **str)
{
	if (type == hwmon_temp && attr == hwmon_temp_label &&
	    channel < MAX_TEMP_CHANNELS) {
		*str = rtsn_name[channel];
		return 0;
	}
	return -EOPNOTSUPP;
}

static int thermal_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct thermal_priv *priv = dev_get_drvdata(dev);
	u32 raw, ret = 0;

	dev_dbg(dev, "priv->pvt_base %p\n", priv->pvt_base);

	if (!priv) {
		dev_err(dev, "priv is NULL\n");
		return -ENODEV;
	}

	if (!priv->pvt_base || IS_ERR(priv->pvt_base)) {
		dev_err(dev, "pvt_base not mapped (channel %d)\n", channel);
		return -ENODEV;
	}

	if (channel < 0 || channel >= MAX_TEMP_CHANNELS) {
		dev_err(dev, "invalid channel %d (max %d)\n", channel,
			MAX_TEMP_CHANNELS - 1);
		return -EINVAL;
	}

	if (type != hwmon_temp || attr != hwmon_temp_input)
		return -EOPNOTSUPP;

	ret = start_measurement_sequence(priv, channel);
	if (ret) {
		dev_err(dev, "measurement failed on channel %d (err=%d)\n",
			channel, ret);
		return ret;
	}
	raw = read_temperature_raw(priv->pvt_base, channel);
	if (raw == 0) {
		dev_err(dev, "Temperature read failed: raw=0 on channel %d\n",
			channel);
		return -ENODATA;
	}
	*val = convert_to_millicelsius(raw);

	dev_dbg(dev, "ch%d: raw=0x%x -> %d.%d C\n", channel, raw, *val / 10,
		abs(*val % 10));

	return ret;
}

static umode_t thermal_hwmon_is_visible(const void *data,
					enum hwmon_sensor_types type, u32 attr,
					int channel)
{
	if (type != hwmon_temp || channel >= MAX_TEMP_CHANNELS)
		return 0;

	if (attr == hwmon_temp_input || attr == hwmon_temp_label)
		return 0444;

	return 0;
}

static const struct hwmon_channel_info *thermal_info[] = {
	HWMON_CHANNEL_INFO(temp, TEMP_FLAGS, TEMP_FLAGS, TEMP_FLAGS, TEMP_FLAGS,
			   TEMP_FLAGS, TEMP_FLAGS, TEMP_FLAGS, TEMP_FLAGS,
			   TEMP_FLAGS, TEMP_FLAGS, TEMP_FLAGS, TEMP_FLAGS,
			   TEMP_FLAGS, TEMP_FLAGS),
	NULL,
};

static const struct hwmon_ops thermal_hwmon_ops = {
	.is_visible = thermal_hwmon_is_visible,
	.read = thermal_hwmon_read,
	.read_string = thermal_hwmon_read_string,
};

static const struct hwmon_chip_info thermal_chip_info = {
	.ops = &thermal_hwmon_ops,
	.info = thermal_info,
};

static int thermal_sensor_probe(struct platform_device *pdev)
{
	struct thermal_priv *priv;
	u32 data;
	int ret;

	dev_dbg(&pdev->dev, "thermal_sensor probe started\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	mutex_init(&priv->lock);

	priv->pvt_base = devm_platform_ioremap_resource_byname(pdev, "pvt");
	if (IS_ERR(priv->pvt_base))
		return PTR_ERR(priv->pvt_base);

	dev_dbg(&pdev->dev, "pvt base = %p\n", priv->pvt_base);

	/* Verify hardware identity */
	data = ioread32(priv->pvt_base +
			PVT_REG__PVT_GLOBAL_REGS__PVT_COMP_ID_NUM_ADDR);
	dev_dbg(&pdev->dev, "pvt_comp_id_num = 0x%08X\n", data);

	data = ioread32(priv->pvt_base +
			PVT_REG__PVT_GLOBAL_REGS__PVT_ID_NUM_ADDR);
	dev_dbg(&pdev->dev, "pvt_id_num = 0x%08X\n", data);

	data = ioread32(priv->pvt_base +
			PVT_REG__PVT_GLOBAL_REGS__PVT_IP_CONFIG_ADDR);
	dev_dbg(&pdev->dev, "pvt_ip_config = 0x%08X\n", data);

	/* Configure DTSN clock */
	data = ioread32(priv->pvt_base +
			PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_CLK_SYNTH_ADDR);
	data &= ~(CLK_SYNTH_HI_MASK | CLK_SYNTH_LO_MASK | CLK_SYNTH_EN_BIT);
	data |= FIELD_PREP(CLK_SYNTH_HI_MASK, CLK_SYNTH_DIVIDER);
	data |= FIELD_PREP(CLK_SYNTH_LO_MASK, CLK_SYNTH_DIVIDER);
	data |= CLK_SYNTH_EN_BIT;
	iowrite32(
		data,
		priv->pvt_base +
			PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_CLK_SYNTH_ADDR);

	data = ioread32(priv->pvt_base +
			PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_CLK_SYNTH_ADDR);
	dev_dbg(&pdev->dev, "DTSN clock config = 0x%08X\n", data);

	/* Wait for SDIF ready before first write */
	ret = sdif_wait_ready(
		priv->pvt_base +
			PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_SDIF_STATUS_ADDR,
		SDIF_WAIT_TIMEOUT_MS);
	if (ret)
		return ret;

	/* Program initial SDIF value */
	data = build_sdif_write(IP_CTRL, IP_CTRL_CONFIG1, true);
	iowrite32(data,
		  priv->pvt_base +
			  PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_SDIF_ADDR);

	/* Wait again after write */
	ret = sdif_wait_ready(
		priv->pvt_base +
			PVT_REG__PVT_DTS_CMN_REGS__TS_DTS_VM_PD_SDIF_STATUS_ADDR,
		SDIF_WAIT_TIMEOUT_MS);
	if (ret)
		return ret;

	/* IRQ debug */
	data = ioread32(priv->pvt_base + PVT_REG__PVT_GLOBAL_REGS__IRQ_EN_ADDR);
	dev_dbg(&pdev->dev, "irq_en = 0x%08X\n", data);

	/* Register hwmon interface */
	priv->hwmon_dev = devm_hwmon_device_register_with_info(
		priv->dev, "simaai_modalix_thermal_sensor", priv,
		&thermal_chip_info, NULL);
	if (IS_ERR(priv->hwmon_dev))
		return PTR_ERR(priv->hwmon_dev);

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "thermal sensor probed successfully");
	return 0;
}

static int thermal_sensor_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "thermal sensor driver removed\n");
	return 0;
}

static const struct of_device_id thermal_of_match[] = {
	{ .compatible = "simaai,modalix-thermal-sensor" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, thermal_of_match);

static struct platform_driver thermal_driver = {
	.driver = {
		.name = "modalix_thermal_sensor",
		.of_match_table = thermal_of_match,
	},
	.probe = thermal_sensor_probe,
	.remove = thermal_sensor_remove,
};

module_platform_driver(thermal_driver);

MODULE_AUTHOR("Bhimeswararao Matsa <bhimeswararao.matsa@sima.ai>");
MODULE_DESCRIPTION("HWMON driver for Thermal Sensor");
MODULE_LICENSE("Dual MIT/GPL");
