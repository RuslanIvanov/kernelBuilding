/*
 * An I2C driver for the Intersil ISL 12029EEPROM
 *
 * Author: Russl
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.

// д.б символьный драйвер по запис чению в вамять
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define DRV_VERSION "0.1"

/* ISL register offsets */
#define ISL12029_EEPROM_REG_BEGIN 0x0
#define ISL12029_EEPROM_REG_END	  0x14

static struct i2c_driver isl12029_eeprom_driver;

struct isl12029_eeprom 
{
	struct rtc_device *rtc;
	bool write_enabled;	/* true if write enable is set */
};

static int isl12029_eeprom_read_regs(struct i2c_client *client, uint8_t reg,uint8_t *data, size_t n)
{
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= data
		},		/* setup read ptr */
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= n,
			.buf	= data
		}
	};

	int ret;

	data[0] = reg;
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) 
	{
		dev_err(&client->dev, "%s: read error, ret=%d\n",__func__, ret);
		return -EIO;
	}

	return 0;
}


static int isl12029_eeprom_write_reg(struct i2c_client *client,uint8_t reg, uint8_t val)
{
	uint8_t data[2] = { reg, val };
	int err;

	err = i2c_master_send(client, data, sizeof(data));
	if (err != sizeof(data)) 
	{
		dev_err(&client->dev,"%s: err=%d addr=%02x, data=%02x\n",__func__, err, data[0], data[1]);return -EIO;
	}

	return 0;
}


static int isl12029_eeprom_get_alarm(struct i2c_client *client, struct rtc_wkalrm *al)
{
	return 0;
}

static int isl12029_eeprom_set_alarm(struct i2c_client *client, struct rtc_wkalrm *al)
{
	struct isl12029_eeprom *isl12029_eeprom = i2c_get_clientdata(client);

	return 0;
}

static int isl12029_eeprom_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	printk(KERN_INFO "isl12029_eeprom_rtc_read_alarm: run");

	return isl12029_eeprom_get_alarm(to_i2c_client(dev), al);
}

static int isl12029_eeprom_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *al)
{
	printk(KERN_INFO "isl12029_eeprom_rtc_set_alarm: run");

	return isl12029_eeprom_set_alarm(to_i2c_client(dev), al);
}

int isl12029_eeprom_rtc_irq_set_freq(struct device *, int freq)
{
	return 0;
}

static const struct rtc_class_ops isl12029_eeprom_rtc_ops = 
{
	.read_alarm	= isl12029_eeprom_rtc_read_alarm,
	.set_alarm	= isl12029_eeprom_rtc_set_alarm,
	.irq_set_freq   = isl12029_eeprom_rtc_irq_set_freq,
};

static int isl12029_eeprom_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct isl12029_eeprom *isl12029_eeprom;

	int ret = 0;

	printk(KERN_INFO "isl12029_eeprom_probe: run");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "Error isl12029_eeprom_probe: i2c_check_functionality");
		return -ENODEV;
	}

	isl12029_eeprom = kzalloc(sizeof(struct isl12029_eeprom), GFP_KERNEL);
	if (!isl12029_eeprom)
	{
		printk(KERN_ERR "Error isl12029_eeprom_probe: kzalloc");
		return -ENOMEM;
	}
	dev_dbg(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	i2c_set_clientdata(client, isl12029_eeprom);

	isl12029_eeprom->rtc = rtc_device_register(isl12029_eeprom_driver.driver.name,
					    &client->dev,
					    &isl12029_eeprom_rtc_ops,
					    THIS_MODULE);

	if (IS_ERR(isl12029_eeprom->rtc)) {
		ret = PTR_ERR(isl12029_eeprom->rtc);
		printk(KERN_ERR "Error isl12029_eeprom_probe: isl12029_eeprom->rtc");

		goto exit_kfree;
	}

	printk(KERN_INFO "isl12029_eeprom_probe: ok");

	return 0;

exit_kfree:
	kfree(isl12029_eeprom);

	return ret;
}

static int isl12029_eeprom_remove(struct i2c_client *client)
{
	struct isl12029_eeprom *isl12029_eeprom = i2c_get_clientdata(client);

	rtc_device_unregister(isl12029_eeprom->rtc);
	kfree(isl12029_eeprom);

	return 0;
}

static const struct i2c_device_id isl12029_eeprom_id[] = {
	{ "isl12029er", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl12029_eeprom_id);

static struct i2c_driver isl12029_eeprom_driver = {
	.driver		= {
		.name	= "rtc-isl12029er",
	},
	.probe		= isl12029_eeprom_probe,
	.remove		= isl12029_eeprom_remove,
	.id_table	= isl12029_eeprom_id,
};

static int __init isl12029_eeprom_init(void)
{
	printk(KERN_INFO "isl12029_eeprom_init: run");

	return i2c_add_driver(&isl12029_eeprom_driver);
}

static void __exit isl12029_eeprom_exit(void)
{
	i2c_del_driver(&isl12029_eeprom_driver);
}

module_init(isl12029_eeprom_init);
module_exit(isl12029_eeprom_exit);

MODULE_AUTHOR("russl");
MODULE_DESCRIPTION("ISL 12029_EEPROM C driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
