// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pinctrl driver for SiMa.ai SoCs NVS pinctrl driver
 * Copyright (c) 2023 Sima ai
 * Author: Yurii Konovalenko <yurii.konovalenko@sima.ai>
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>

#include "pinctrl-utils.h"

#define SIMMAAI_NVS_ID_ADDR		0x00000000
#define SIMMAAI_NVS_RT_ADDR		0x00000040
#define SIMMAAI_NVS_IE_ADDR		0x00000100
#define SIMMAAI_NVS_HYS_ADDR		0x00000104
#define SIMMAAI_NVS_PUP_ADDR		0x00000108
#define SIMMAAI_NVS_PDN_ADDR		0x0000010c
#define SIMMAAI_NVS_OE_ADDR		0x00000110
#define SIMMAAI_NVS_SLEW_ADDR		0x00000114
#define SIMMAAI_NVS_STR_ADDR		0x00000120

/* Custom pinconf parameters */
#define SIMMAAI_NVS_PIN_CONFIG_RT	(PIN_CONFIG_END + 1)
#define SIMMAAI_NVS_PIN_CONFIG_HYS	(PIN_CONFIG_END + 2)

#define pcsimaai_spi0oe			0xf7ff
#define pcsimaai_spi0ie			0x08ff
#define pcsimaai_spi1oe			0xf7ff
#define pcsimaai_spi1ie			0x08ff
#define pcsimaai_emmcoe			0x07ff
#define pcsimaai_emmcie			0x00ff
#define pcsimaai_sdoe			0x017f
#define pcsimaai_sdie			0x008f

#define SIMAAI_NVS_PINGROUP(block) { \
	.name = #block"_group", \
	.pins = block##_pins, \
	.num_pins = ARRAY_SIZE(block##_pins), \
}

struct pcsimaai_pin_group {
	const char		*name;
	const u32		*pins;
	const u32		num_pins;
};

struct pcsimaai_pin_plat {
	const struct pcsimaai_pin_group *pin_groups;
	u32			num_pin_groups;
	const struct pinctrl_desc desc;
};

struct pcsimaai_pinctrl {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
	struct pcsimaai_pin_plat *plat;
	void __iomem		*base;
	struct mutex		lock;
};

static struct pinctrl_pin_desc pcsimaai_spi_pins[] = {
	PINCTRL_PIN(0, "SPI0_D0"),
	PINCTRL_PIN(1, "SPI0_D1"),
	PINCTRL_PIN(2, "SPI0_D2"),
	PINCTRL_PIN(3, "SPI0_D3"),
	PINCTRL_PIN(4, "SPI0_D4"),
	PINCTRL_PIN(5, "SPI0_D5"),
	PINCTRL_PIN(6, "SPI0_D6"),
	PINCTRL_PIN(7, "SPI0_D7"),
	PINCTRL_PIN(8, "SPI0_DMS"),
	PINCTRL_PIN(9, "SPI0_SCLK_N"),
	PINCTRL_PIN(10, "SPI0_SCLK_P"),
	PINCTRL_PIN(11, "SPI0_SS_IN"),
	PINCTRL_PIN(12, "SPI0_SS0"),
	PINCTRL_PIN(13, "SPI0_SS1"),
	PINCTRL_PIN(14, "SPI0_SS2"),
	PINCTRL_PIN(15, "SPI0_SS3"),
	PINCTRL_PIN(16, "SPI1_D0"),
	PINCTRL_PIN(17, "SPI1_D1"),
	PINCTRL_PIN(18, "SPI1_D2"),
	PINCTRL_PIN(19, "SPI1_D3"),
	PINCTRL_PIN(20, "SPI1_D4"),
	PINCTRL_PIN(21, "SPI1_D5"),
	PINCTRL_PIN(22, "SPI1_D6"),
	PINCTRL_PIN(23, "SPI1_D7"),
	PINCTRL_PIN(24, "SPI1_DMS"),
	PINCTRL_PIN(25, "SPI1_SCLK_N"),
	PINCTRL_PIN(26, "SPI1_SCLK_P"),
	PINCTRL_PIN(27, "SPI1_SS_IN"),
	PINCTRL_PIN(28, "SPI1_SS0"),
	PINCTRL_PIN(29, "SPI1_SS1"),
	PINCTRL_PIN(30, "SPI1_SS2"),
	PINCTRL_PIN(31, "SPI1_SS3"),
};

static struct pinctrl_pin_desc pcsimaai_sdioemmc_pins[] = {
	PINCTRL_PIN(0, "EMMC_D0"),
	PINCTRL_PIN(1, "EMMC_D1"),
	PINCTRL_PIN(2, "EMMC_D2"),
	PINCTRL_PIN(3, "EMMC_D3"),
	PINCTRL_PIN(4, "EMMC_D4"),
	PINCTRL_PIN(5, "EMMC_D5"),
	PINCTRL_PIN(6, "EMMC_D6"),
	PINCTRL_PIN(7, "EMMC_D7"),
	PINCTRL_PIN(8, "EMMC_CMD"),
	PINCTRL_PIN(9, "EMMC_CLK"),
	PINCTRL_PIN(10, "EMMC_RST"),
	PINCTRL_PIN(16, "SDIO_D0"),
	PINCTRL_PIN(17, "SDIO_D1"),
	PINCTRL_PIN(18, "SDIO_D2"),
	PINCTRL_PIN(19, "SDIO_D3"),
	PINCTRL_PIN(20, "SDIO_CMD"),
	PINCTRL_PIN(21, "SDIO_CLK"),
	PINCTRL_PIN(22, "SDIO_WP"),
	PINCTRL_PIN(23, "SDIO_DET"),
	PINCTRL_PIN(24, "SDIO_VSEL"),
};

static const u32 spi0_pins[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, };
static const u32 spi1_pins[] = { 16, 17, 18, 19, 20, 21, 22, 23, 24, 24, 25, 26, 27, 28, 29, 30, 31, };
static const u32 emmc_pins[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, };
static const u32 sdio_pins[] = { 16, 17, 18, 19, 20, 21, 22, 23, 24, };

static const struct pcsimaai_pin_group pcsimaai_spipin_groups[] = {
	SIMAAI_NVS_PINGROUP(spi0),
	SIMAAI_NVS_PINGROUP(spi1),
};

static const struct pcsimaai_pin_group pcsimaai_sdioemmcpin_groups[] = {
	SIMAAI_NVS_PINGROUP(emmc),
	SIMAAI_NVS_PINGROUP(sdio),
};

static const struct pinconf_generic_params pcsimaai_custom_bindings[] = {
	{"simaai,retention", SIMMAAI_NVS_PIN_CONFIG_RT, 0},
	{"simaai,hysteresys", SIMMAAI_NVS_PIN_CONFIG_HYS, 0},
};

#ifdef CONFIG_DEBUG_FS
static const struct pin_config_item pcsimaai_conf_items[] = {
	PCONFDUMP(SIMMAAI_NVS_PIN_CONFIG_RT, "retention", NULL, true),
	PCONFDUMP(SIMMAAI_NVS_PIN_CONFIG_HYS, "hysteresys", NULL, true),
};
#endif

static int pcsimaai_get_group_pins(struct pinctrl_dev *pctldev,
				  unsigned int group,
				  const unsigned int **pins,
				  unsigned int *num_pins)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if (group >= pctl->plat->num_pin_groups) {
		dev_err(pctl->dev, "Pinconf is not supported for pin group %d\n", group);
		return -EOPNOTSUPP;
	}

	*pins = (u32 *)pctl->plat->pin_groups[group].pins;
	*num_pins = pctl->plat->pin_groups[group].num_pins;

	return 0;
}

static int pcsimaai_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->plat->num_pin_groups;
}

static const char *pcsimaai_get_group_name(struct pinctrl_dev *pctldev,
					  unsigned int group)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->plat->pin_groups[group].name;
}

///TODO: Implement actual mA to field conversion

static u32 pcsimaai_pconf_mA_to_field(u32 mA)
{
	return mA & 0xf;
}

static u32 pcsimaai_pconf_field_to_mA(u32 field)
{
	return field & 0xf;
}

static int find_pin_id(struct pcsimaai_pinctrl *pctl, u32 pin)
{
	int i;

	for(i = 0; i < pctl->plat->desc.npins; i++) {
		if(pctl->plat->desc.pins[i].number == pin)
			return i;
	}

	return -1;
}
static int pcsimaai_pconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
				 unsigned long *config)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	u32 param = pinconf_to_config_param(*config);
	u32 arg, val;
	int pinid = find_pin_id(pctl, pin);

	/* Check for valid pin */
	if (pinid < 0) {
		dev_err(pctl->dev, "Pinconf is not supported for pin %d\n", pin);
		return -EOPNOTSUPP;
	}

	dev_dbg(pctl->dev, "Getting configuration for pin %s\n",
		pctl->plat->desc.pins[pinid].name);

	switch (param) {
	case SIMMAAI_NVS_PIN_CONFIG_RT:
		arg = !!(readl(pctl->base + SIMMAAI_NVS_RT_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = !!(readl(pctl->base + SIMMAAI_NVS_PDN_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		arg = !!(readl(pctl->base + SIMMAAI_NVS_PUP_ADDR) & (1 << pin));
		break;
	case SIMMAAI_NVS_PIN_CONFIG_HYS:
		arg = !!(readl(pctl->base + SIMMAAI_NVS_HYS_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		val = (readl(pctl->base + SIMMAAI_NVS_STR_ADDR + ((pin / 8) * 4)) >> ((pin & 7) * 4)) & 0xf;
		arg = pcsimaai_pconf_field_to_mA(val);
		break;
	case PIN_CONFIG_SLEW_RATE:
		arg = !!(readl(pctl->base + SIMMAAI_NVS_SLEW_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = !!(readl(pctl->base + SIMMAAI_NVS_OE_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		arg = !!(readl(pctl->base + SIMMAAI_NVS_IE_ADDR) & (1 << pin));
		break;
	default:
		dev_dbg(pctl->dev, "Property %u not supported\n", param);
		return -EOPNOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int pcsimaai_pconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				 unsigned long *configs, unsigned int num_configs)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	u32 param;
	u32 arg;
	u32 regval;
	u32 val, mask;
	int offset;
	int i;
	int pinid = find_pin_id(pctl, pin);

	/* Check for valid pin */
	if (pinid < 0) {
		dev_err(pctl->dev, "Pinconf is not supported for pin %d\n", pin);
		return -EOPNOTSUPP;
	}

	dev_dbg(pctl->dev, "Setting configuration for pin %s\n",
		pctl->plat->desc.pins[pinid].name);

	mutex_lock(&pctl->lock);

	for (i = 0; i < num_configs; i++) {

		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		val = arg >= 1 ? 1 << pin : 0;
		mask = 1 << pin;

		switch (param) {
		case SIMMAAI_NVS_PIN_CONFIG_RT:
			offset = SIMMAAI_NVS_RT_ADDR;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			offset = SIMMAAI_NVS_PDN_ADDR;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			offset = SIMMAAI_NVS_PUP_ADDR;
			break;
		case SIMMAAI_NVS_PIN_CONFIG_HYS:
			offset = SIMMAAI_NVS_HYS_ADDR;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			val = pcsimaai_pconf_mA_to_field(arg);
			mask = 0xf << ((pin & 7) * 4);
			offset = SIMMAAI_NVS_STR_ADDR + ((pin / 8) * 4);
			break;
		case PIN_CONFIG_SLEW_RATE:
			offset = SIMMAAI_NVS_SLEW_ADDR;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			offset = SIMMAAI_NVS_OE_ADDR;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			offset = SIMMAAI_NVS_IE_ADDR;
			break;
		default:
			dev_err(pctl->dev, "Property %u not supported\n", param);
			offset = -1;
		}

		if (offset < 0)
			continue;

		regval = readl(pctl->base + offset);
		regval &= ~mask;
		regval |= val;
		writel(regval, pctl->base + offset);
	}

	mutex_unlock(&pctl->lock);

	return 0;
}

static int pcsimaai_pconf_group_set(struct pinctrl_dev *pctldev,
				   unsigned int group, unsigned long *configs,
				   unsigned int num_configs)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int num_pins, current_pin;
	int ret;

	if (group >= pctl->plat->num_pin_groups) {
		dev_err(pctl->dev, "Pinconf is not supported for pin group %d\n", group);
		return -EOPNOTSUPP;
	}

	dev_dbg(pctl->dev, "Setting group %s configuration\n",
		pcsimaai_get_group_name(pctldev, group));

	num_pins = pctl->plat->pin_groups[group].num_pins;

	for (current_pin = 0; current_pin < num_pins; current_pin++) {
		ret = pcsimaai_pconf_set(pctldev,
				pctl->plat->pin_groups[group].pins[current_pin],
				configs, num_configs);

		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct pinctrl_ops pcsimaai_pctrl_ops = {
	.get_group_pins		= pcsimaai_get_group_pins,
	.get_groups_count	= pcsimaai_get_groups_count,
	.get_group_name		= pcsimaai_get_group_name,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinctrl_utils_free_map,
};

static const struct pinconf_ops pcsimaai_pconf_ops = {
	.is_generic		= true,
	.pin_config_get		= pcsimaai_pconf_get,
	.pin_config_set		= pcsimaai_pconf_set,
	.pin_config_group_set	= pcsimaai_pconf_group_set,
};

struct pcsimaai_pin_plat spi_plat = {
	.pin_groups		= pcsimaai_spipin_groups,
	.num_pin_groups		= ARRAY_SIZE(pcsimaai_spipin_groups),
	.desc			= {
		.name		= "simaai-nvs-spi-pinctrl",
		.owner		= THIS_MODULE,
		.pctlops	= &pcsimaai_pctrl_ops,
		.confops	= &pcsimaai_pconf_ops,
		.pins		= pcsimaai_spi_pins,
		.npins		= ARRAY_SIZE(pcsimaai_spi_pins),
		.num_custom_params = ARRAY_SIZE(pcsimaai_custom_bindings),
		.custom_params	= pcsimaai_custom_bindings,
#ifdef CONFIG_DEBUG_FS
		.custom_conf_items = pcsimaai_conf_items,
#endif
	},
};

struct pcsimaai_pin_plat sdioemmc_plat = {
	.pin_groups		= pcsimaai_sdioemmcpin_groups,
	.num_pin_groups		= ARRAY_SIZE(pcsimaai_sdioemmcpin_groups),
	.desc			= {
		.name		= "simaai-nvs-sdioemmc-pinctrl",
		.owner		= THIS_MODULE,
		.pctlops	= &pcsimaai_pctrl_ops,
		.confops	= &pcsimaai_pconf_ops,
		.pins		= pcsimaai_sdioemmc_pins,
		.npins		= ARRAY_SIZE(pcsimaai_sdioemmc_pins),
		.num_custom_params = ARRAY_SIZE(pcsimaai_custom_bindings),
		.custom_params	= pcsimaai_custom_bindings,
#ifdef CONFIG_DEBUG_FS
		.custom_conf_items = pcsimaai_conf_items,
#endif
	},
};

static int pcnvssimaai_probe(struct platform_device *pdev)
{
	struct pcsimaai_pinctrl *pctl;
	int err;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->dev = &pdev->dev;

	pctl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	pctl->plat = of_device_get_match_data(&pdev->dev);
	if (!pctl->plat)
		return -ENOENT;

	err = devm_pinctrl_register_and_init(&pdev->dev, &pctl->plat->desc, pctl, &pctl->pctl);
	if (err)
		return err;

	err = pinctrl_enable(pctl->pctl);
	if (err)
		return err;

	mutex_init(&pctl->lock);

	platform_set_drvdata(pdev, pctl);

	/* Clear pull up register */
	writel(0, pctl->base + SIMMAAI_NVS_PUP_ADDR);

	dev_info(&pdev->dev, "Initialised SiMa.ai NVS pinctrl driver\n");

	return 0;
}

static int pcnvssimaai_remove(struct platform_device *pdev)
{
	struct pcsimaai_pinctrl *pctl = platform_get_drvdata(pdev);

	pinctrl_unregister(pctl->pctl);

	return 0;
}

static const struct of_device_id pcnvssimaai_of_match[] = {
	{
		.compatible = "simaai,pinctrl-nvs-spi",
		.data = &spi_plat,
	},
	{
		.compatible = "simaai,pinctrl-nvs-sdemmc",
		.data = &sdioemmc_plat,
	},
};

MODULE_DEVICE_TABLE(of, pcnvssimaai_of_match);

static struct platform_driver pcnvssimaai_driver = {
	.probe		= pcnvssimaai_probe,
	.remove		= pcnvssimaai_remove,
	.driver = {
		.name		= "simaai-nvs-pinctrl",
		.of_match_table	= pcnvssimaai_of_match,
	},
};

module_platform_driver(pcnvssimaai_driver);

MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai NVS Pin Control Driver");
MODULE_LICENSE("GPL");

