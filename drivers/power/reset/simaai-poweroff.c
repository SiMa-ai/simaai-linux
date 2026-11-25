// SPDX-License-Identifier: GPL-2.0-only
/*
 * simaai-poweroff.c - Modalix power-off.
 *
 * Power-off is attempted in order, using whatever the board provides:
 *   1. shutdown GPIO      - SoM boards drive SIO5[0]
 *   2. PMIC rail soft-off - PMBus sequence to the board PMIC
 *   3. HPI request        - non-SoM boards ask the secure processor
 *
 * Copyright (c) 2026 SiMa ai
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

#include <linux/simaai-hpi.h>

#define SIMA_HPI_MSG_POWEROFF		0x10000016

#define GPIO_WAIT_MS			1000
#define PMIC_WAIT_MS			1000
#define HPI_WAIT_MS			10000

#define PMBUS_PAGE			0x00
#define PMBUS_OPERATION			0x01
#define PMBUS_ON_OFF_CONFIG		0x02
#define PMBUS_ON_OFF_USE_OPERATION	0x18
#define PMBUS_OPERATION_SOFT_OFF	0x40

struct simaai_poweroff {
	struct gpio_desc	*shutdown_gpio;
	struct i2c_adapter	*pmic_adap;
	u16			pmic_addr;
	u8			pmic_page;
};

static void simaai_poweroff_pmic(struct simaai_poweroff *pd, struct device *dev)
{
	union i2c_smbus_data d;
	int ret;

	dev_dbg(dev, "PMIC rail soft-off\n");

	d.byte = pd->pmic_page;
	ret = i2c_smbus_xfer(pd->pmic_adap, pd->pmic_addr, 0, I2C_SMBUS_WRITE,
			     PMBUS_PAGE, I2C_SMBUS_BYTE_DATA, &d);
	if (!ret) {
		d.byte = PMBUS_ON_OFF_USE_OPERATION;
		ret = i2c_smbus_xfer(pd->pmic_adap, pd->pmic_addr, 0, I2C_SMBUS_WRITE,
				     PMBUS_ON_OFF_CONFIG, I2C_SMBUS_BYTE_DATA, &d);
	}
	if (!ret) {
		d.byte = PMBUS_OPERATION_SOFT_OFF;
		ret = i2c_smbus_xfer(pd->pmic_adap, pd->pmic_addr, 0, I2C_SMBUS_WRITE,
				     PMBUS_OPERATION, I2C_SMBUS_BYTE_DATA, &d);
	}
	if (ret)
		dev_warn(dev, "PMIC soft-off write failed: %d\n", ret);
}

/* PREPARE phase: sleepable - GPIO and i2c are legal here. */
static int simaai_poweroff_board(struct sys_off_data *data)
{
	struct simaai_poweroff *pd = data->cb_data;

	if (pd->shutdown_gpio) {
		dev_info(data->dev, "asserting shutdown GPIO\n");
		gpiod_set_value_cansleep(pd->shutdown_gpio, 1);
		mdelay(GPIO_WAIT_MS);
	}

	if (pd->pmic_adap) {
		simaai_poweroff_pmic(pd, data->dev);
		mdelay(PMIC_WAIT_MS);
	}

	return NOTIFY_DONE;
}

/* POWER_OFF phase: atomic - mailbox only. Satisfies kernel_can_power_off(). */
static int simaai_poweroff_soc_blks(struct sys_off_data *data)
{
	dev_dbg(data->dev, "requesting power off over HPI\n");
	simaai_hpi_send(SIMA_HPI_MSG_POWEROFF, 0);
	mdelay(HPI_WAIT_MS);
	dev_emerg(data->dev, "power off failed\n");
	return NOTIFY_DONE;
}

static void simaai_poweroff_put_adap(void *adap)
{
	i2c_put_adapter(adap);
}

static int simaai_poweroff_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simaai_poweroff *pd;
	struct device_node *np;
	u32 addr, page = 0;
	int ret;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	/* HPI: dependency-free last resort, satisfies kernel_can_power_off(). */
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
					    SYS_OFF_PRIO_DEFAULT,
					    simaai_poweroff_soc_blks, pd);
	if (ret)
		return ret;

	/* Optional shutdown GPIO */
	pd->shutdown_gpio = devm_gpiod_get_optional(dev, "poweroff", GPIOD_OUT_LOW);
	if (IS_ERR(pd->shutdown_gpio))
		return dev_err_probe(dev, PTR_ERR(pd->shutdown_gpio),
				     "failed to get poweroff GPIO\n");

	/* Optional PMIC soft-off */
	np = of_parse_phandle(dev->of_node, "simaai,poweroff-i2c-bus", 0);
	if (np) {
		pd->pmic_adap = of_find_i2c_adapter_by_node(np);
		of_node_put(np);
		if (!pd->pmic_adap)
			return -EPROBE_DEFER;
		ret = devm_add_action_or_reset(dev, simaai_poweroff_put_adap,
					       pd->pmic_adap);
		if (ret)
			return ret;

		if (of_property_read_u32(dev->of_node, "simaai,poweroff-i2c-addr", &addr))
			return dev_err_probe(dev, -EINVAL, "poweroff-i2c-addr missing\n");
		pd->pmic_addr = addr;
		of_property_read_u32(dev->of_node, "simaai,poweroff-i2c-page", &page);
		pd->pmic_page = page;
	}

	if (pd->shutdown_gpio || pd->pmic_adap)
		return devm_register_sys_off_handler(dev,
					SYS_OFF_MODE_POWER_OFF_PREPARE,
					SYS_OFF_PRIO_DEFAULT,
					simaai_poweroff_board, pd);
	return 0;
}

static const struct of_device_id simaai_poweroff_of_match[] = {
	{ .compatible = "simaai,modalix-poweroff" },
	{ }
};
MODULE_DEVICE_TABLE(of, simaai_poweroff_of_match);

static struct platform_driver simaai_poweroff_driver = {
	.probe = simaai_poweroff_probe,
	.driver = {
		.name = "simaai-poweroff",
		.of_match_table = simaai_poweroff_of_match,
	},
};
module_platform_driver(simaai_poweroff_driver);

MODULE_DESCRIPTION("SiMa.ai Modalix power-off driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bharanidharan Ramalingam <bharanidharan.ramalingam@sima.ai>");

