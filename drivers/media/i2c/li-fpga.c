/*
 * li_fpga.c - li_fpga IO Expander driver
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* #define DEBUG */

#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

struct fpga {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
 	u16	channel;
};

struct fpga *gFPGA[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
static int i2c_wr8(struct i2c_client *client, u8 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	u8 data[2];

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = addr;
	data[1] = val; 

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		printk("%s: wr8 register failed 0x%x = %x \n", __func__, addr, val);
		return 0;
	}

	return 0;
}

int li_fpga_write_reg(struct device *dev,
				u8 addr, u8 val, int video_num)
{
	struct fpga *priv = gFPGA[video_num];
	struct i2c_client *i2c_client = priv->i2c_client;
	int err; 
	err = i2c_wr8(priv->i2c_client, addr, val); 
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}
EXPORT_SYMBOL_GPL(li_fpga_write_reg);

static int fpga_stats_show(struct seq_file *s, void *data)
{
	return 0;
}

static int fpga_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, fpga_stats_show, inode->i_private);
}
 
static ssize_t fpga_debugfs_write(struct file *s,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	char buf[255];
	int buf_size;

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (buf[0] == 'd') {
		return count;
	}

	if (buf[0] == 'n') {
		return count;
	}


	return count;
}


static const struct file_operations fpga_debugfs_fops = {
	.open = fpga_debugfs_open,
	.read = seq_read,
	.write = fpga_debugfs_write,
	.llseek = seq_lseek,
	.release = single_release, 
};
 
static const struct regmap_config fpga_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

static int fpga_probe(struct i2c_client *client)
{
	struct fpga *priv;
	int err = 0;
	static int i;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	gFPGA[i] = priv;
	i++;
	priv->i2c_client = client;

	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&fpga_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	dev_set_drvdata(&client->dev, priv);

	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}

static void fpga_remove(struct i2c_client *client)
{

	if (client != NULL) {
		i2c_unregister_device(client);
		client = NULL;
	}
}

static const struct i2c_device_id fpga_id[] = {
	{ "fpga", 0 },
	{ },
}; 

static const struct of_device_id fpga_of_match[] = {
	{ .compatible = "nvidia,fpga", },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fpga_id);

static struct i2c_driver fpga_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "li-fpga", 
	},
	.probe = fpga_probe,
	.remove = fpga_remove,
	.id_table = fpga_id,
};

static int __init fpga_init(void)
{
	return i2c_add_driver(&fpga_i2c_driver);
}

static void __exit fpga_exit(void)
{
	i2c_del_driver(&fpga_i2c_driver);
}

module_init(fpga_init);
module_exit(fpga_exit);

MODULE_DESCRIPTION("IO Expander driver fpga");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
