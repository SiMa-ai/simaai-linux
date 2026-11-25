// SPDX-License-Identifier: GPL-2.0-only
/*
 * li-iic2io.c - Leopard Imaging "iic2io" I/O expander
 *
 * Copyright (c) 2025 Leopard Imaging Inc.
 * Copyright (c) 2026 SiMa ai
 *
 * Ported from the Leopard LI-ONX-CB-GMSL2-8CAM BSP.  The part is an 8-bit
 * I2C I/O expander that sits on the second deserializer's I2C bus on the
 * LI GMSL carrier.  It needs a single one-shot init at probe: three pins
 * are switched to outputs, two are driven high immediately and the third
 * is released high after a settling delay.  Nothing touches it again, so
 * the driver exposes no runtime interface.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define IIC2IO_REG_OUTPUT		0x01
#define IIC2IO_REG_CONFIG		0x03

/* P1, P2 and P6 as outputs; every other pin is left as an input. */
#define IIC2IO_CONFIG_INIT		0xb9
/* P1 and P2 high, P6 low. */
#define IIC2IO_OUTPUT_ASSERT		0x06
/* P1, P2 and P6 high. */
#define IIC2IO_OUTPUT_RELEASE		0x46

/* Settling time before the expander will accept its first transfer. */
#define IIC2IO_POWER_ON_DELAY_MS	100
/* How long P6 is held low before it is released. */
#define IIC2IO_RELEASE_DELAY_MS		400

struct iic2io {
	struct i2c_client *client;
	struct regmap *regmap;
};

static const struct regmap_config iic2io_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	/*
	 * Three unconditional writes at probe and no reads ever, so there
	 * is nothing for a cache to accelerate.
	 */
	.cache_type = REGCACHE_NONE,
};

static int iic2io_write(struct iic2io *priv, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(&priv->client->dev,
			"failed to write 0x%02x to reg 0x%02x: %d\n",
			val, reg, ret);

	return ret;
}

static int iic2io_probe(struct i2c_client *client)
{
	struct iic2io *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->regmap = devm_regmap_init_i2c(client, &iic2io_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	i2c_set_clientdata(client, priv);

	msleep(IIC2IO_POWER_ON_DELAY_MS);

	ret = iic2io_write(priv, IIC2IO_REG_CONFIG, IIC2IO_CONFIG_INIT);
	if (ret)
		return ret;

	ret = iic2io_write(priv, IIC2IO_REG_OUTPUT, IIC2IO_OUTPUT_ASSERT);
	if (ret)
		return ret;

	msleep(IIC2IO_RELEASE_DELAY_MS);

	return iic2io_write(priv, IIC2IO_REG_OUTPUT, IIC2IO_OUTPUT_RELEASE);
}

static const struct i2c_device_id iic2io_id[] = {
	{ "iic2io" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, iic2io_id);

static const struct of_device_id iic2io_of_match[] = {
	{ .compatible = "leopard,iic2io" },
	{ }
};
MODULE_DEVICE_TABLE(of, iic2io_of_match);

static struct i2c_driver iic2io_i2c_driver = {
	.driver = {
		.name = "li-iic2io",
		.of_match_table = iic2io_of_match,
	},
	.probe = iic2io_probe,
	.id_table = iic2io_id,
};
module_i2c_driver(iic2io_i2c_driver);

MODULE_DESCRIPTION("Leopard Imaging iic2io I/O expander");
MODULE_AUTHOR("Leopard Imaging Inc.");
MODULE_LICENSE("GPL");
