/*
 * Helper for the LCD-KIT-B01 LCD Panel
 *
 * Copyright (C) 2014 Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <mach/rza1.h>

/************************************************************************/

/* All interrupt requests from LCD-KIT-B01 are handled by the 
 * touch-screen driver (drivers/input/touchscreen/ili210x.c).  */

/* LCD-KIT-B01 CONTROL CPU COMMANDS */
#define REG_GET_FW_VERSION	0x00	/* 1+1(R) */
#define REG_CTRL_BACKLIGHT	0x03	/* 1+1(R/W) */
#define REG_CTRL_BUZZER		0x04	/* 1+1(R/W) */
#define REG_RESET_TPC		0x05	/* 1+0 */
#define	VERSION_MAJOR(ver)	((ver) / 0x10)
#define	VERSION_MINOR(ver)	((ver) % 0x10)

struct lcd_kit_b01_priv {
	struct i2c_client *client;
	u8 fw_version;
	u8 backlight;
	u8 buzzer;
};

/************************************************************************/

static int lcd_kit_b01_i2c_transfer(struct i2c_client *client,
	u8 *txbuf, size_t txlen, u8 *rxbuf, size_t rxlen)
{
	struct i2c_msg msg[2];
	int msglen;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = txbuf;
	msg[0].len = txlen;
	msglen = 1;

	if (rxbuf && rxlen) {
		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = rxbuf;
		msg[1].len = rxlen;
		msglen = ARRAY_SIZE(msg);
	}

	if (i2c_transfer(client->adapter, msg, msglen) != msglen) {
		dev_err(&client->dev, "i2c transfer failed\n");
		return -EIO;
	}
	return 0;
}

static int lcd_kit_b01_i2c_send(struct i2c_client *client, u8 cmd, u8 data)
{
	u8 txbuf[2];

	txbuf[0] = cmd;
	txbuf[1] = data;
	return lcd_kit_b01_i2c_transfer(client, txbuf, 2, NULL, 0);
}

static int lcd_kit_b01_i2c_recv(struct i2c_client *client, u8 cmd, u8 *data)
{
	return lcd_kit_b01_i2c_transfer(client, &cmd, 1, data, 1);
}

/************************************************************************/

static void lcd_kit_b01_reset_panel(struct device *dev, unsigned int i)
{
	dev_info(dev, "Reset LCD panel on i2c.%d\n", dev->id);
	switch (dev->id) {
	case 0:
		rza1_pfc_pin_assign(P11_14, PMODE, PORT_OUT_HIGH);
		udelay(500);
		gpio_set_value(P11_14, 0);
		udelay(500);
		gpio_set_value(P11_14, 1);
		break;
	case 3:
		rza1_pfc_pin_assign(P4_9, PMODE, PORT_OUT_HIGH);
		udelay(500);
		gpio_set_value(P4_9, 0);
		udelay(500);
		gpio_set_value(P4_9, 1);
		break;
	default:
		break;
	}
}

static ssize_t lcd_kit_b01_store_reset_panel(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int i;
	int rc;

	rc = kstrtouint(buf, 10, &i);
	if (rc)
		return rc;
	if (i) {
		lcd_kit_b01_reset_panel(dev, i);
	}
	return count;
}
static DEVICE_ATTR(reset_panel, (S_IWUSR | S_IWGRP), NULL, lcd_kit_b01_store_reset_panel);

/************************************************************************/

static ssize_t lcd_kit_b01_show_fw_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lcd_kit_b01_priv *priv = i2c_get_clientdata(client);
	int rc;

	rc = sprintf(buf, "%d.%d\n", VERSION_MAJOR(priv->fw_version),
		VERSION_MINOR(priv->fw_version));
	return rc;
}
static DEVICE_ATTR(fw_version, S_IRUGO, lcd_kit_b01_show_fw_version, NULL);

static ssize_t lcd_kit_b01_show_backlight(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lcd_kit_b01_priv *priv = i2c_get_clientdata(client);
	int rc;

	rc = sprintf(buf, "%d\n", priv->backlight);
	return rc;
}

static ssize_t lcd_kit_b01_store_backlight(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lcd_kit_b01_priv *priv = i2c_get_clientdata(client);
	unsigned int i;
	int rc;

	rc = kstrtouint(buf, 10, &i);
	if (rc)
		return rc;
	if (i > 0xff)
		i = 0xff;
	priv->backlight = i;
	lcd_kit_b01_i2c_send(client, REG_CTRL_BACKLIGHT, i);
	return count;
}
static DEVICE_ATTR(backlight, (S_IRUGO | S_IWUGO),
	lcd_kit_b01_show_backlight, lcd_kit_b01_store_backlight);

static ssize_t lcd_kit_b01_show_buzzer(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lcd_kit_b01_priv *priv = i2c_get_clientdata(client);
	int rc;

	rc = sprintf(buf, "%d\n", priv->buzzer);
	return rc;
}

static ssize_t lcd_kit_b01_store_buzzer(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lcd_kit_b01_priv *priv = i2c_get_clientdata(client);
	unsigned int i;
	int rc;

	rc = kstrtouint(buf, 10, &i);
	if (rc)
		return rc;
	if (i > 0xff)
		i = 0xff;
	priv->buzzer = i;
	lcd_kit_b01_i2c_send(client, REG_CTRL_BUZZER, i);
	return count;
}
static DEVICE_ATTR(buzzer, (S_IRUGO | S_IWUGO),
	lcd_kit_b01_show_buzzer, lcd_kit_b01_store_buzzer);

static ssize_t lcd_kit_b01_store_reset_tsc(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int i;
	int rc;
	u8 cmd;

	rc = kstrtouint(buf, 10, &i);
	if (rc)
		return rc;
	if (i) {
		cmd = REG_RESET_TPC;
		lcd_kit_b01_i2c_transfer(client, &cmd, 1, NULL, 0);
		/* The system controller will send the reset signal
		 * to the touch screen controller during 20ms. */
	}
	return count;
}
static DEVICE_ATTR(reset_tsc, (S_IWUSR | S_IWGRP), NULL, lcd_kit_b01_store_reset_tsc);

static struct attribute *lcd_kit_b01_attributes[] = {
	&dev_attr_reset_panel.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_backlight.attr,
	&dev_attr_buzzer.attr,
	&dev_attr_reset_tsc.attr,
	NULL,
};

static const struct attribute_group lcd_kit_b01_attr_group = {
	.attrs = lcd_kit_b01_attributes,
};

/************************************************************************/

static int lcd_kit_b01_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lcd_kit_b01_priv *priv;
	int error = -ENOMEM;
	u8 data = 0;	/* -Wuninitialized */

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto err;

	error = sysfs_create_group(&dev->kobj, &lcd_kit_b01_attr_group);
	if (error)
		goto err_kfree;

/* RESET */
	lcd_kit_b01_reset_panel(&client->dev, 1);
	msleep(100);

	priv->client = client;
	i2c_set_clientdata(client, priv);

	error = lcd_kit_b01_i2c_recv(client, REG_GET_FW_VERSION, &data);
	if (error)
		goto err_kfree;
	priv->fw_version = data;

	data = 0xff;
	error = lcd_kit_b01_i2c_send(client, REG_CTRL_BACKLIGHT, data);
	if (error)
		goto err_kfree;
	priv->backlight = data;

	data = 0;
	error = lcd_kit_b01_i2c_send(client, REG_CTRL_BUZZER, data);
	if (error)
		goto err_kfree;
	priv->buzzer = data;

	device_init_wakeup(&client->dev, 1);

	dev_info(dev, "Helper initialized, F/W version %d.%d\n",
		VERSION_MAJOR(priv->fw_version),
		VERSION_MINOR(priv->fw_version));

	return 0;

err_kfree:
	kfree(priv);
err:
	return error;
}

static int lcd_kit_b01_i2c_remove(struct i2c_client *client)
{
	struct lcd_kit_b01 *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &lcd_kit_b01_attr_group);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lcd_kit_b01_i2c_suspend(struct device *dev)
{
	/* Handled by the system and touch-screen controllers */
	return 0;
}

static int lcd_kit_b01_i2c_resume(struct device *dev)
{
	/* Handled by the system and touch-screen controllers */
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(
	lcd_kit_b01_i2c_pm,
	lcd_kit_b01_i2c_suspend,
	lcd_kit_b01_i2c_resume);

static const struct i2c_device_id lcd_kit_b01_i2c_id[] = {
	{ "lcd_kit_b01", 0 },
	{ "lcd_kit_b01", 3 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lcd_kit_b01_i2c_id);

static struct i2c_driver lcd_kit_b01_i2c = {
	.driver = {
		.name = "lcd_kit_b01_i2c",
		.owner = THIS_MODULE,
		.pm = &lcd_kit_b01_i2c_pm,
	},
	.id_table = lcd_kit_b01_i2c_id,
	.probe = lcd_kit_b01_i2c_probe,
	.remove = lcd_kit_b01_i2c_remove,
};

module_i2c_driver(lcd_kit_b01_i2c);

MODULE_DESCRIPTION("Helper for the LCD-KIT-B01");
MODULE_AUTHOR("Renesas Solution Corp.");
MODULE_LICENSE("GPL");
