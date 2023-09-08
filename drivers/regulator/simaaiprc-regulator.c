// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * simaaiprc-regulator.c
 *
 * Copyright (c) 2023 Sima ai
 *
 * based on gpio-regulator.c
 *
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/gpio-regulator.h>
#include <linux/slab.h>
#include <linux/of.h>

struct simaai_prc_regulator_data {
	struct regulator_desc desc;
	void __iomem *base;
	struct gpio_regulator_state *states;
	int nr_states;
	int state;
	unsigned int mask;
};

static int simaai_prc_regulator_get_value(struct regulator_dev *dev)
{
	struct simaai_prc_regulator_data *data = rdev_get_drvdata(dev);
	int ptr;

	for (ptr = 0; ptr < data->nr_states; ptr++)
		if (data->states[ptr].gpios == data->state)
			return data->states[ptr].value;

	return -EINVAL;
}

static int simaai_prc_regulator_set_voltage(struct regulator_dev *dev,
					int min_uV, int max_uV,
					unsigned *selector)
{
	struct simaai_prc_regulator_data *data = rdev_get_drvdata(dev);
	int ptr, target = 0, state, best_val = INT_MAX;

	for (ptr = 0; ptr < data->nr_states; ptr++)
		if (data->states[ptr].value < best_val &&
		    data->states[ptr].value >= min_uV &&
		    data->states[ptr].value <= max_uV) {
			target = data->states[ptr].gpios;
			best_val = data->states[ptr].value;
			if (selector)
				*selector = ptr;
		}

	if (best_val == INT_MAX)
		return -EINVAL;

	state = readl_relaxed(data->base);
	state &= ~data->mask;
	state |= target & data->mask;
	writel_relaxed(state, data->base);
	data->state = target;

	return 0;
}

static int simaai_prc_regulator_list_voltage(struct regulator_dev *dev,
				      unsigned selector)
{
	struct simaai_prc_regulator_data *data = rdev_get_drvdata(dev);

	if (selector >= data->nr_states)
		return -EINVAL;

	return data->states[selector].value;
}

static const struct regulator_ops simaai_prc_regulator_voltage_ops = {
	.get_voltage = simaai_prc_regulator_get_value,
	.set_voltage = simaai_prc_regulator_set_voltage,
	.list_voltage = simaai_prc_regulator_list_voltage,
};

static struct gpio_regulator_config *
of_get_simaai_prc_regulator_config(struct device *dev, struct device_node *np,
			     const struct regulator_desc *desc)
{
	struct gpio_regulator_config *config;
	int proplen, i;

	config = devm_kzalloc(dev,
			sizeof(struct gpio_regulator_config),
			GFP_KERNEL);
	if (!config)
		return ERR_PTR(-ENOMEM);

	config->init_data = of_get_regulator_init_data(dev, np, desc);
	if (!config->init_data)
		return ERR_PTR(-EINVAL);

	config->supply_name = config->init_data->constraints.name;

	of_property_read_u32(np, "startup-delay-us", &config->startup_delay);

	/* Fetch states. */
	proplen = of_property_count_u32_elems(np, "states");
	if (proplen < 0) {
		dev_err(dev, "No 'states' property found\n");
		return ERR_PTR(-EINVAL);
	}

	config->states = devm_kcalloc(dev,
				proplen / 2,
				sizeof(struct gpio_regulator_state),
				GFP_KERNEL);
	if (!config->states)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < proplen / 2; i++) {
		of_property_read_u32_index(np, "states", i * 2,
					   &config->states[i].value);
		of_property_read_u32_index(np, "states", i * 2 + 1,
					   &config->states[i].gpios);
	}
	config->nr_states = i;

	config->type = REGULATOR_VOLTAGE;

	return config;
}

static int simaai_prc_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_regulator_config *config;
	struct device_node *np = dev->of_node;
	struct simaai_prc_regulator_data *drvdata;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	int boot_val, ret;

	drvdata = devm_kzalloc(dev, sizeof(struct simaai_prc_regulator_data),
			       GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata->base = devm_of_iomap(&pdev->dev, np, 0, NULL);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	of_property_read_u32(np, "regulator-mask", &drvdata->mask);
	of_property_read_u32(np, "regulator-boot-val", &boot_val);

	config = of_get_simaai_prc_regulator_config(dev, np,
							&drvdata->desc);
	if (IS_ERR(config))
		return PTR_ERR(config);

	drvdata->desc.name = devm_kstrdup(dev, config->supply_name, GFP_KERNEL);
	if (drvdata->desc.name == NULL) {
		dev_err(dev, "Failed to allocate supply name\n");
		return -ENOMEM;
	}

	drvdata->states = devm_kmemdup(dev,
				       config->states,
				       config->nr_states *
				       sizeof(struct gpio_regulator_state),
				       GFP_KERNEL);
	if (drvdata->states == NULL) {
		dev_err(dev, "Failed to allocate state data\n");
		return -ENOMEM;
	}

	drvdata->nr_states = config->nr_states;
	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.enable_time = config->startup_delay;

	/* handle regulator type*/
	switch (config->type) {
	case REGULATOR_VOLTAGE:
		drvdata->desc.type = REGULATOR_VOLTAGE;
		drvdata->desc.ops = &simaai_prc_regulator_voltage_ops;
		drvdata->desc.n_voltages = config->nr_states;
		break;
	default:
		dev_err(dev, "No regulator type set\n");
		return -EINVAL;
	}

	/* build initial state from the PRC */
	boot_val &= drvdata->mask;
	drvdata->state = boot_val;
	boot_val |= readl_relaxed(drvdata->base) & ~drvdata->mask;
	writel_relaxed(boot_val, drvdata->base);

	cfg.dev = dev;
	cfg.init_data = config->init_data;
	cfg.driver_data = drvdata;
	cfg.of_node = np;

	rdev = devm_regulator_register(dev, &drvdata->desc, &cfg);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(dev, "Failed to register regulator: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, drvdata);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id regulator_simaai_prc_of_match[] = {
	{ .compatible = "simaai-prc-regulator", },
	{},
};
MODULE_DEVICE_TABLE(of, regulator_gsimaai_prc_of_match);
#endif

static struct platform_driver simaai_prc_regulator_driver = {
	.probe		= simaai_prc_regulator_probe,
	.driver		= {
		.name		= "simaai-prc-regulator",
		.of_match_table = of_match_ptr(regulator_simaai_prc_of_match),
	},
};

static int __init simaai_prc_regulator_init(void)
{
	return platform_driver_register(&simaai_prc_regulator_driver);
}
subsys_initcall(simaai_prc_regulator_init);

static void __exit simaai_prc_regulator_exit(void)
{
	platform_driver_unregister(&simaai_prc_regulator_driver);
}
module_exit(simaai_prc_regulator_exit);

MODULE_AUTHOR("Yurii Konovalenko <yurii.konovalenko@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai PRC voltage regulator");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:simaaiprc-regulator");
