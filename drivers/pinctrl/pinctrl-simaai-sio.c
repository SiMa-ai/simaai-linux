// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pinctrl driver for SiMa.ai SoCs SIO pinctrl driver
 * Copyright (c) 2023 Sima ai
 * Author: Yurii Konovalenko <yurii.konovalenko@sima.ai>
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>

#include "pinctrl-utils.h"

#define SIMMAAI_SIO_ID_ADDR		0x00000000
#define SIMMAAI_SIO_RT_ADDR		0x00000040
#define SIMMAAI_SIO_PDN_ADDR		0x00000104
#define SIMMAAI_SIO_PUP_ADDR		0x00000108
#define SIMMAAI_SIO_HYS_ADDR		0x0000010c
#define SIMMAAI_SIO_STR_ADDR		0x00000110
#define SIMMAAI_SIO_SLEW_ADDR		0x00000114
#define SIMMAAI_SIO_OE_ADDR		0x00000118
#define SIMMAAI_SIO_IE_ADDR		0x0000011c
#define SIMMAAI_SIO_ODEM_ADDR		0x00000120
#define SIMMAAI_SIO_ODPOL_ADDR		0x00000124
#define SIMMAAI_SIO_MX1_ADDR		0x00000204
#define SIMMAAI_SIO_MX2_ADDR		0x00000208

/* Custom pinconf parameters */
#define SIMMAAI_SIO_PIN_CONFIG_RT	(PIN_CONFIG_END + 1)
#define SIMMAAI_SIO_PIN_CONFIG_HYS	(PIN_CONFIG_END + 2)
#define SIMMAAI_SIO_PIN_CONFIG_ODPOL	(PIN_CONFIG_END + 3)

#define pcsimaai_spioe			0x7
#define pcsimaai_spiie			0xf
#define pcsimaai_i2coe			0x3
#define pcsimaai_i2cie			0x3
#define pcsimaai_uartoe			0x2
#define pcsimaai_uartie			0x3
#define pcsimaai_gpiooe			0x1
#define pcsimaai_gpioie			0x1

#define SIMAAI_SIO_PIN(n)		PINCTRL_PIN(n, "PIN"#n)

#define SIMAAI_SIO_PINGROUP(block, n, m1, m2) { \
	.name = #block#n"_group", \
	.pins = block##n##_pins, \
	.num_pins = ARRAY_SIZE(block##n##_pins), \
	.oe_val = (pcsimaai_##block##oe) << ((ARRAY_SIZE(block##n##_pins)) * n), \
	.ie_val = (pcsimaai_##block##ie) << ((ARRAY_SIZE(block##n##_pins)) * n), \
	.mux1_offset = m1, \
	.mux2_offset = m2, \
}

#define SIMAAI_SIO_PINFUNCTION(block, m1, m2, g) { \
	.name = #block, \
	.groups = block##_groups, \
	.num_groups = ARRAY_SIZE(block##_groups), \
	.mux1_val = m1, \
	.mux2_val = m2, \
	.gpio_val = g, \
}

struct pcsimaai_pin_group {
	const char		*name;
	const u32		*pins;
	const u32		num_pins;
	const u32		oe_val;
	const u32		ie_val;
	const u32		mux1_offset;
	const u32		mux2_offset;
};

struct pcsimaai_pin_func {
	const char		*name;
	const char * const	*groups;
	const u32		num_groups;
	const u32		mux1_val;
	const u32		mux2_val;
	const u32		gpio_val;
};

struct pcsimaai_pinctrl {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
	void __iomem		*ctl_base;
	void __iomem		*gpio_base;
	struct pinctrl_pin_desc	*pins;
	u32			num_pins;
	const struct pcsimaai_pin_group *pin_groups;
	u32			num_pin_groups;
	const struct pcsimaai_pin_func *functions;
	u32			num_functions;
	struct mutex		lock;
};

static struct pinctrl_pin_desc pcsimaai_pins[] = {
	SIMAAI_SIO_PIN(0),
	SIMAAI_SIO_PIN(1),
	SIMAAI_SIO_PIN(2),
	SIMAAI_SIO_PIN(3),
	SIMAAI_SIO_PIN(4),
	SIMAAI_SIO_PIN(5),
	SIMAAI_SIO_PIN(6),
	SIMAAI_SIO_PIN(7),
};

static const u32 spi0_pins[] = { 0, 1, 2, 3 };
static const u32 spi1_pins[] = { 4, 5, 6, 7 };
static const u32 i2c0_pins[] = { 0, 1 };
static const u32 i2c1_pins[] = { 2, 3 };
static const u32 i2c2_pins[] = { 4, 5 };
static const u32 i2c3_pins[] = { 6, 7 };
static const u32 uart0_pins[] = { 0, 1 };
static const u32 uart1_pins[] = { 2, 3 };
static const u32 uart2_pins[] = { 4, 5 };
static const u32 uart3_pins[] = { 6, 7 };
static const u32 gpio0_pins[] = { 0 };
static const u32 gpio1_pins[] = { 1 };
static const u32 gpio2_pins[] = { 2 };
static const u32 gpio3_pins[] = { 3 };
static const u32 gpio4_pins[] = { 4 };
static const u32 gpio5_pins[] = { 5 };
static const u32 gpio6_pins[] = { 6 };
static const u32 gpio7_pins[] = { 7 };

static const char * const spi_groups[]	  = { "spi0_group", "spi1_group", };
static const char * const i2c_groups[]	  = { "i2c0_group", "i2c1_group", "i2c2_group",
					      "i2c3_group", };
static const char * const uart_groups[]	  = { "uart0_group", "uart1_group", "uart2_group",
					      "uart3_group", };
static const char * const gpio_groups[]	  = { "gpio0_group", "gpio1_group", "gpio2_group",
					      "gpio3_group", "gpio4_group", "gpio5_group",
					      "gpio6_group", "gpio7_group", };

static const struct pcsimaai_pin_group pcsimaai_pin_groups[] = {
	SIMAAI_SIO_PINGROUP(spi, 0, 0, 0),
	SIMAAI_SIO_PINGROUP(spi, 1, 8, 0),
	SIMAAI_SIO_PINGROUP(i2c, 0, 0, 0),
	SIMAAI_SIO_PINGROUP(i2c, 1, 0, 8),
	SIMAAI_SIO_PINGROUP(i2c, 2, 8, 16),
	SIMAAI_SIO_PINGROUP(i2c, 3, 8, 24),
	SIMAAI_SIO_PINGROUP(uart, 0, 0, 0),
	SIMAAI_SIO_PINGROUP(uart, 1, 0, 8),
	SIMAAI_SIO_PINGROUP(uart, 2, 8, 16),
	SIMAAI_SIO_PINGROUP(uart, 3, 8, 24),
	SIMAAI_SIO_PINGROUP(gpio, 0, 0, 0),
	SIMAAI_SIO_PINGROUP(gpio, 1, 0, 0),
	SIMAAI_SIO_PINGROUP(gpio, 2, 0, 8),
	SIMAAI_SIO_PINGROUP(gpio, 3, 0, 8),
	SIMAAI_SIO_PINGROUP(gpio, 4, 8, 16),
	SIMAAI_SIO_PINGROUP(gpio, 5, 8, 16),
	SIMAAI_SIO_PINGROUP(gpio, 6, 8, 24),
	SIMAAI_SIO_PINGROUP(gpio, 7, 8, 24),
};

static const struct pcsimaai_pin_func pcsimaai_pin_functions[] = {
	SIMAAI_SIO_PINFUNCTION(spi, 0, 0, 1),
	SIMAAI_SIO_PINFUNCTION(i2c, 1, 0, 1),
	SIMAAI_SIO_PINFUNCTION(uart, 1, 1, 1),
	SIMAAI_SIO_PINFUNCTION(gpio, 1, 2, 0),
};

static const struct pinconf_generic_params pcsimaai_custom_bindings[] = {
	{"simaai,retention", SIMMAAI_SIO_PIN_CONFIG_RT, 0},
	{"simaai,hysteresys", SIMMAAI_SIO_PIN_CONFIG_HYS, 0},
	{"simaai,drive-open-drain-polarity", SIMMAAI_SIO_PIN_CONFIG_ODPOL, 0},
};

#ifdef CONFIG_DEBUG_FS
static const struct pin_config_item pcsimaai_conf_items[] = {
	PCONFDUMP(SIMMAAI_SIO_PIN_CONFIG_RT, "retention", NULL, true),
	PCONFDUMP(SIMMAAI_SIO_PIN_CONFIG_HYS, "hysteresys", NULL, true),
	PCONFDUMP(SIMMAAI_SIO_PIN_CONFIG_ODPOL, "odpol", NULL, true),
};
#endif

static int pcsimaai_get_group_pins(struct pinctrl_dev *pctldev,
				  unsigned int group,
				  const unsigned int **pins,
				  unsigned int *num_pins)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if (group >= pctl->num_pin_groups) {
		dev_err(pctl->dev, "Pinconf is not supported for pin group %d\n", group);
		return -EOPNOTSUPP;
	}

	*pins = (u32 *)pctl->pin_groups[group].pins;
	*num_pins = pctl->pin_groups[group].num_pins;

	return 0;
}

static int pcsimaai_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->num_pin_groups;
}

static const char *pcsimaai_get_group_name(struct pinctrl_dev *pctldev,
					  unsigned int group)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->pin_groups[group].name;
}

static int pcsimaai_pmx_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->num_functions;
}

static const char *pcsimaai_pmx_get_fname(struct pinctrl_dev *pctldev,
					 u32 func)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->functions[func].name;
}

static int pcsimaai_pmx_get_fgroups(struct pinctrl_dev *pctldev,
				   u32 func,
				   const char * const **groups,
				   u32 * const num_groups)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if (func >= pctl->num_functions) {
		dev_err(pctl->dev, "Pinconf is not supported for pin function %d\n", func);
		return -EOPNOTSUPP;
	}

	*groups = pctl->functions[func].groups;
	*num_groups = pctl->functions[func].num_groups;

	return 0;
}

static int pcsimaai_pmx_set(struct pinctrl_dev *pctldev, unsigned int func,
				unsigned int group)
{
	u32 ie, oe, mux;
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct pcsimaai_pin_group *pin_group;
	int i;

	if (group >= pctl->num_pin_groups) {
		dev_err(pctl->dev, "Pinconf is not supported for pin group %d\n", group);
		return -EOPNOTSUPP;
	}

	if (func >= pctl->num_functions) {
		dev_err(pctl->dev, "Pinconf is not supported for pin function %d\n", func);
		return -EOPNOTSUPP;
	}

	pin_group = &pctl->pin_groups[group];

	mutex_lock(&pctl->lock);

	/* Configure Input/Output enable */
	oe = readl(pctl->ctl_base + SIMMAAI_SIO_OE_ADDR);
	ie = readl(pctl->ctl_base + SIMMAAI_SIO_IE_ADDR);
	for (i = 0; i < pin_group->num_pins; i++) {
		oe &= ~(1 << pin_group->pins[i]);
		ie &= ~(1 << pin_group->pins[i]);
	}
	oe |= pin_group->oe_val;
	ie |= pin_group->ie_val;
	writel(oe, pctl->ctl_base + SIMMAAI_SIO_OE_ADDR);
	writel(ie, pctl->ctl_base + SIMMAAI_SIO_IE_ADDR);

	/* Write MUX1 and MUX2 */
	mux = readl(pctl->ctl_base + SIMMAAI_SIO_MX1_ADDR);
	mux &= ~(1 << pin_group->mux1_offset);
	mux |= pctl->functions[func].mux1_val << pin_group->mux1_offset;
	writel(mux, pctl->ctl_base + SIMMAAI_SIO_MX1_ADDR);
	mux = readl(pctl->ctl_base + SIMMAAI_SIO_MX2_ADDR);
	mux &= ~(3 << pin_group->mux2_offset);
	mux |= pctl->functions[func].mux2_val << pin_group->mux2_offset;
	writel(mux, pctl->ctl_base + SIMMAAI_SIO_MX2_ADDR);

	/* Configure GPIO register */
	mux = readl(pctl->gpio_base);
	for (i = 0; i < pin_group->num_pins; i++) {
		mux &= ~(1 << pin_group->pins[i]);
		mux |= (pctl->functions[func].gpio_val & 1) << pin_group->pins[i];
	}
	writel(mux, pctl->gpio_base);

	mutex_unlock(&pctl->lock);

	return 0;
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

static int pcsimaai_pconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
				 unsigned long *config)
{
	struct pcsimaai_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	u32 param = pinconf_to_config_param(*config);
	u32 arg, val;

	/* Check for valid pin */
	if (pin >= pctl->num_pins) {
		dev_err(pctl->dev, "Pinconf is not supported for pin %d\n", pin);
		return -EOPNOTSUPP;
	}

	dev_dbg(pctl->dev, "Setting configuration for pin %s\n",
		pctl->pins[pin].name);

	switch (param) {
	case SIMMAAI_SIO_PIN_CONFIG_RT:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_RT_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_PDN_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_PUP_ADDR) & (1 << pin));
		break;
	case SIMMAAI_SIO_PIN_CONFIG_HYS:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_HYS_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		val = (readl(pctl->ctl_base + SIMMAAI_SIO_HYS_ADDR) >> (pin * 4)) & 0xf;
		arg = pcsimaai_pconf_field_to_mA(val);
		break;
	case PIN_CONFIG_SLEW_RATE:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_SLEW_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_OE_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_IE_ADDR) & (1 << pin));
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_ODEM_ADDR) & (1 << pin));
		break;
	case SIMMAAI_SIO_PIN_CONFIG_ODPOL:
		arg = !!(readl(pctl->ctl_base + SIMMAAI_SIO_ODPOL_ADDR) & (1 << pin));
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

	/* Check for valid pin */
	if (pin >= pctl->num_pins) {
		dev_err(pctl->dev, "Pinconf is not supported for pin %d\n", pin);
		return -EOPNOTSUPP;
	}

	dev_dbg(pctl->dev, "Setting configuration for pin %s\n",
		pctl->pins[pin].name);

	mutex_lock(&pctl->lock);

	for (i = 0; i < num_configs; i++) {

		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		val = arg >= 1 ? 1 << pin : 0;
		mask = 1 << pin;

		switch (param) {
		case SIMMAAI_SIO_PIN_CONFIG_RT:
			offset = SIMMAAI_SIO_RT_ADDR;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			offset = SIMMAAI_SIO_PDN_ADDR;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			offset = SIMMAAI_SIO_PUP_ADDR;
			break;
		case SIMMAAI_SIO_PIN_CONFIG_HYS:
			offset = SIMMAAI_SIO_HYS_ADDR;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			val = pcsimaai_pconf_mA_to_field(arg);
			mask = 0xf << (pin * 4);
			offset = SIMMAAI_SIO_STR_ADDR;
			break;
		case PIN_CONFIG_SLEW_RATE:
			offset = SIMMAAI_SIO_SLEW_ADDR;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			offset = SIMMAAI_SIO_OE_ADDR;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			offset = SIMMAAI_SIO_IE_ADDR;
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			offset = SIMMAAI_SIO_ODEM_ADDR;
			break;
		case SIMMAAI_SIO_PIN_CONFIG_ODPOL:
			offset = SIMMAAI_SIO_ODPOL_ADDR;
			break;
		default:
			dev_err(pctl->dev, "Property %u not supported\n", param);
			offset = -1;
		}

		if (offset < 0)
			continue;

		regval = readl(pctl->ctl_base + offset);
		regval &= ~mask;
		regval |= val;
		writel(regval, pctl->ctl_base + offset);
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

	if (group >= pctl->num_pin_groups) {
		dev_err(pctl->dev, "Pinconf is not supported for pin group %d\n", group);
		return -EOPNOTSUPP;
	}

	dev_dbg(pctl->dev, "Setting group %s configuration\n",
		pcsimaai_get_group_name(pctldev, group));

	num_pins = pctl->pin_groups[group].num_pins;

	for (current_pin = 0; current_pin < num_pins; current_pin++) {
		ret = pcsimaai_pconf_set(pctldev,
				pctl->pin_groups[group].pins[current_pin],
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

static const struct pinmux_ops pcsimaai_pmx_ops = {
	.get_functions_count	= pcsimaai_pmx_get_functions_count,
	.get_function_name	= pcsimaai_pmx_get_fname,
	.get_function_groups	= pcsimaai_pmx_get_fgroups,
	.set_mux		= pcsimaai_pmx_set,
};

static const struct pinconf_ops pcsimaai_pconf_ops = {
	.is_generic		= true,
	.pin_config_get		= pcsimaai_pconf_get,
	.pin_config_set		= pcsimaai_pconf_set,
	.pin_config_group_set	= pcsimaai_pconf_group_set,
};

static struct pinctrl_desc pcsimaai_desc = {
	.name			= "simaai-sio-pinctrl",
	.owner			= THIS_MODULE,
	.pins			= pcsimaai_pins,
	.npins			= ARRAY_SIZE(pcsimaai_pins),
	.pctlops		= &pcsimaai_pctrl_ops,
	.pmxops			= &pcsimaai_pmx_ops,
	.confops		= &pcsimaai_pconf_ops,
	.num_custom_params	= ARRAY_SIZE(pcsimaai_custom_bindings),
	.custom_params		= pcsimaai_custom_bindings,
#ifdef CONFIG_DEBUG_FS
	.custom_conf_items	= pcsimaai_conf_items,
#endif
};

static int pcsiosimaai_probe(struct platform_device *pdev)
{
	struct pcsimaai_pinctrl *pctl;
	struct resource *res;
	int err;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->dev = &pdev->dev;

	pctl->ctl_base = devm_platform_ioremap_resource_byname(pdev, "ctl");
	if (IS_ERR(pctl->ctl_base))
		return PTR_ERR(pctl->ctl_base);

	/*
	* Can't use devm_platform_ioremap_resource_byname() for gpio region
	* because the same region is used by GPIO driver, so let's use
	* platform_get_resource_byname() + devm_ioremap() as a workaround
	*/
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gpio");
	if (!res)
		return -EINVAL;
	pctl->gpio_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(pctl->gpio_base))
		return PTR_ERR(pctl->gpio_base);

	pctl->pins		= pcsimaai_pins;
	pctl->num_pins		= ARRAY_SIZE(pcsimaai_pins);
	pctl->functions		= pcsimaai_pin_functions;
	pctl->num_functions	= ARRAY_SIZE(pcsimaai_pin_functions);
	pctl->pin_groups	= pcsimaai_pin_groups;
	pctl->num_pin_groups	= ARRAY_SIZE(pcsimaai_pin_groups);

	err = devm_pinctrl_register_and_init(&pdev->dev, &pcsimaai_desc, pctl, &pctl->pctl);
	if (err)
		return err;

	err = pinctrl_enable(pctl->pctl);
	if (err)
		return err;

	mutex_init(&pctl->lock);

	platform_set_drvdata(pdev, pctl);

	dev_info(&pdev->dev, "Initialised SiMa.ai SIO pinctrl driver\n");

	return 0;
}

static int pcsiosimaai_remove(struct platform_device *pdev)
{
	struct pcsimaai_pinctrl *pctl = platform_get_drvdata(pdev);

	pinctrl_unregister(pctl->pctl);

	return 0;
}

static const struct of_device_id pcsiosimaai_of_match[] = {
	{ .compatible = "simaai,pinctrl-sio" },
};

MODULE_DEVICE_TABLE(of, pcsiosimaai_of_match);

static struct platform_driver pcsiosimaai_driver = {
	.probe		= pcsiosimaai_probe,
	.remove		= pcsiosimaai_remove,
	.driver = {
		.name		= "simaai-sio-pinctrl",
		.of_match_table	= pcsiosimaai_of_match,
	},
};

module_platform_driver(pcsiosimaai_driver);

MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai SIO Pin Control Driver");
MODULE_LICENSE("GPL");

