/*
 * An I2C driver for the Intersil ISL 12029
 *
 * Author: Russl
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define DRV_VERSION "0.1"

/* ISL register offsets */
#define ISL12029_REG_SC		0x30
#define ISL12029_REG_MN		0x31
#define ISL12029_REG_HR		0x32
#define ISL12029_REG_DT		0x33
#define ISL12029_REG_MO		0x34
#define ISL12029_REG_YR		0x35
#define ISL12029_REG_DW		0x36
#define ISL12029_REG_Y2K        0x37

#define ISL12029_REG_SR		0x3f

/* ISL register bits */
#define ISL12029_HR_MIL		(1 << 7)	/* military or 24 hour time */
#define ISL12029_SR_RWEL	0x6
#define ISL12029_SR_WEL         0x2
#define ISL12029_SR_BAT		(1 << 7)

/* ISL EEPROM register offsets */
#define ISL12029_REG_SCA1        0xF
#define ISL12029_REG_MNA1        0xE
#define ISL12029_REG_HRA1        0xD
#define ISL12029_REG_DTA1        0xC
#define ISL12029_REG_MOA1        0xB
#define ISL12029_REG_YRA1        0xA
#define ISL12029_REG_DWA1        0x9
#define ISL12029_REG_Y2K1        0x8

#define ISL12029_REG_SCA0        0x7
#define ISL12029_REG_MNA0        0x6
#define ISL12029_REG_HRA0        0x5
#define ISL12029_REG_DTA0        0x4
#define ISL12029_REG_MOA0        0x3
#define ISL12029_REG_YRA0        0x2
#define ISL12029_REG_DWA0        0x1
#define ISL12029_REG_Y2K0        0x0

#define ISL12029_REG_PWR 0x14
#define ISL12029_REG_DTR 0x13
#define ISL12029_REG_ATR 0x12
#define ISL12029_REG_INT 0x11
#define ISL12029_REG_BL  0x10

static struct i2c_driver isl12029_driver;

struct isl12029 
{
	struct rtc_device *rtc;
	bool write_enabled;	/* true if write enable is set */
};

static int isl12029_write_reg(struct i2c_client *client, uint16_t reg, uint8_t val)
{
	uint8_t data[3];
	int err;

	data[0] = reg>>8;
	data[1] = reg&0xff;
	data[2] = val;

	err = i2c_master_send(client, data, sizeof(data));
	if (err != sizeof(data)) 
	{
		dev_err(&client->dev,"%s: err=%d addr=%02x, data=%02x\n",__func__, err, data[0], data[1]);return -EIO;
	}

	return 0;
}

static int isl12029_write_master_nreg(struct i2c_client *client, uint16_t reg, uint8_t* val, uint16_t n)
{
	uint8_t data[20];
	int err; uint16_t i;

	data[0] = reg>>8;
	data[1] = reg&0xff;

	i=0; err=0;

	for(;i<n;i++)
	    data[2+i] = val[i];

	err = i2c_master_send(client, data, n+2);
	if (err != (n+2)) 
	{
		dev_err(&client->dev,"%s: err=%d addr=%02x, data=%02x\n",__func__, err, data[0], data[1]);return -EIO;
	}

	return 0;
}

static int isl12029_read_regs(struct i2c_client *client, uint16_t reg,uint8_t *data, size_t n)
{
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
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

	data[0] = reg>>8;
	data[1] = reg&0xff;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

//	dev_info(&client->dev, "%s: _read addr %x reg %x, n=%d\n",__func__, client->addr,reg,n);

	if (ret != ARRAY_SIZE(msgs)) 
	{
		dev_err(&client->dev, "%s: read error, ret=%d\n",__func__, ret);
		return -EIO;
	}

	return 0;
}

static int isl12029_write_transfer_nreg(struct i2c_client *client, uint16_t reg, uint8_t* val, uint16_t n)
{

	uint8_t data[20];
	uint16_t i;
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2+n,
			.buf	= data
		}
	};

	data[0] = reg>>8;
	data[1] = reg&0xff;
	i=0; ret=0;

	for(;i<n;i++)
	    data[2+i] = val[i];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

//	dev_info(&client->dev, "%s: _write_ addr %x reg %x\n",__func__, client->addr,reg);

	if (ret != ARRAY_SIZE(msgs)) 
	{
		dev_err(&client->dev, "%s: write error, ret=%d\n",__func__, ret);
		return -EIO;
	}

	return 0;
}


static int isl12029_write_transfer_reg(struct i2c_client *client, uint16_t reg, uint8_t val)
{
	uint8_t data[3];
	int ret;

	struct i2c_msg msgs[] = 
	{
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 3,
			.buf	= data
		}
	};

	ret = 0; data[0]=0; data[1]=0; data[2]=0;

	data[0] = reg>>8;
	data[1] = reg&0xff;
	data[2] = val;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

//	dev_info(&client->dev, "%s: write addr %x reg %x\n",__func__, client->addr,reg);

	if (ret != ARRAY_SIZE(msgs)) 
	{
		dev_err(&client->dev, "%s: write error, ret=%d\n",__func__, ret);
		return -EIO;
	}

	return 0;
}


/*
 * In the routines that deal directly with the isl12029 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int isl12029_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	uint8_t bSr;
	uint8_t buf[8];
	int ret;
	ret=0; bSr=0;

	ret = isl12029_read_regs(client, ISL12029_REG_SC, buf, sizeof(buf));
	if (ret)
	{
		dev_err(&client->dev, "isl12029_get_datetime: isl12029_read_regs : ISL12029_REG_SC\n");
		return ret;
	}

	ret= isl12029_read_regs(client, ISL12029_REG_SR, &bSr, 1);
	if(!ret) 
	{
		//dev_warn(&client->dev, "devices is [REG_SR=%x] %s\n",bSr,bSr & ISL12029_SR_BAT ? "operating from Vbat" : "operating from Vdd");
	}

	dev_dbg(&client->dev,
		"%s: raw data is sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, mon=%02x, year=%02x, wday=%02x, "
		"y2k=%02x"
		,
		__func__,
		buf[ISL12029_REG_SC -0x30],
		buf[ISL12029_REG_MN -0x30],
		buf[ISL12029_REG_HR -0x30],
		buf[ISL12029_REG_DT -0x30],
		buf[ISL12029_REG_MO -0x30],
		buf[ISL12029_REG_YR -0x30],
		buf[ISL12029_REG_DW -0x30],
		buf[ISL12029_REG_Y2K-0x30]);//,
		//buf[ISL12029_REG_SR -0x37]);

	tm->tm_sec = bcd2bin(buf[ISL12029_REG_SC-0x30] & 0x7F);
	tm->tm_min = bcd2bin(buf[ISL12029_REG_MN-0x30] & 0x7F);
	tm->tm_hour = bcd2bin(buf[ISL12029_REG_HR-0x30] & 0x3F);
	tm->tm_mday = bcd2bin(buf[ISL12029_REG_DT-0x30] & 0x3F);
	tm->tm_wday = buf[ISL12029_REG_DW-0x30] & 0x07;
	tm->tm_mon = bcd2bin(buf[ISL12029_REG_MO-0x30] & 0x1F) - 1;
	tm->tm_year = bcd2bin(buf[ISL12029_REG_YR-0x30]) + 100;

	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* The clock can give out invalid datetime, but we cannot return
	 * -EINVAL otherwise hwclock will refuse to set the time on bootup. */
	if (rtc_valid_tm(tm) < 0)
		dev_err(&client->dev, "retrieved date and time is invalid.\n");

	return 0;
}

static int isl12029_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct isl12029 *isl12029 = i2c_get_clientdata(client);
	size_t i;
	int ret;
	uint8_t buf[8];
	uint8_t tmp; 
	ret=0;tmp=0; i=0;

	ret = isl12029_read_regs(client, ISL12029_REG_PWR, buf, 1);
        if (ret)
        {
                dev_err(&client->dev, "ISL12029_REG_PWR\n");
                return ret;
        }

        dev_dbg(&client->dev, "PWR %x\n",buf[0]); 
	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);


	//if (!isl12029->write_enabled) 
	{
		ret = isl12029_read_regs(client, ISL12029_REG_SR, buf, 1);
		if (ret)
		{
			dev_err(&client->dev, "isl12029_read_regs: ISL12029_REG_SR\n");
			return ret;
		}

		dev_dbg(&client->dev, "ISL12029_REG_SR is %x\n",buf[0]);

		/* Check if WEL (write rtc enable) is set factory default is
		 * 0 (not set) */
		if ((buf[0] & ISL12029_SR_WEL) != ISL12029_SR_WEL) 
		{

			dev_dbg(&client->dev, "set bit WEL\n");

			/* Set the write enable bit. */
                        ret = isl12029_write_reg(client, ISL12029_REG_SR, ISL12029_SR_WEL);
                        if (ret)
                        {
                                 dev_err(&client->dev, "write enable WEL\n");
                                 return ret;
                        }
		}	

		/* Check if RWEL (write rtc enable) is set factory default is
		 * 0 (not set) */
		if ((buf[0] & ISL12029_SR_RWEL) != ISL12029_SR_RWEL) 
		{
			dev_dbg(&client->dev, "set bit RWEL\n");
			/* Set the write enable bit. */
			ret = isl12029_write_reg(client,ISL12029_REG_SR, ISL12029_SR_RWEL);
			if (ret)
			{
				isl12029_write_reg(client,ISL12029_REG_SR,0);
				dev_err(&client->dev, "write enable RWEL\n");
				return ret;
			}
		}

		ret = isl12029_read_regs(client, ISL12029_REG_SR, buf, 1);
		if (ret)
		{
			dev_err(&client->dev, "isl12029_read_regs_: ISL12029_REG_SR\n");
			return ret;
		}
		dev_dbg(&client->dev, "ISL12029_REG_SR is %x\n",buf[0]);

		//dev_info(&client->dev, "write enabled\n");
		//isl12029->write_enabled = 1;
	}

	/* hours, minutes and seconds */
	buf[ISL12029_REG_SC-0x30] = bin2bcd(tm->tm_sec);
	buf[ISL12029_REG_MN-0x30] = bin2bcd(tm->tm_min);
	buf[ISL12029_REG_HR-0x30] = bin2bcd(tm->tm_hour) | ISL12029_HR_MIL;

	buf[ISL12029_REG_DT-0x30] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[ISL12029_REG_MO-0x30] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	buf[ISL12029_REG_YR-0x30] = bin2bcd(tm->tm_year % 100);

	buf[ISL12029_REG_DW-0x30] = tm->tm_wday & 0x07;

	buf[ISL12029_REG_Y2K-0x30] = 0x20;//2000 year

	dev_dbg(&client->dev,"write time...\n");

	//ret = isl12029_write_transfer_nreg(client, ISL12029_REG_SC, &buf[(ISL12029_REG_SC-0x30)],8);
	ret = isl12029_write_master_nreg(client, ISL12029_REG_SC, &buf[(ISL12029_REG_SC-0x30)],8);
	if (ret)
	{
		dev_err(&client->dev,"isl12029_write_nreg\n");
		isl12029_write_reg(client,ISL12029_REG_SR,0);
		return -EIO;
	}

	/////////////
	//mdelay(20);
	/////////////

	/* write register's data */
	/*for (i = 0; i < ARRAY_SIZE(buf); i++) 
	{
		ret = isl12029_write_reg(client, ISL12029_REG_SC + i, buf[(ISL12029_REG_SC-0x30) + i],1);
		if (ret)
		{
			isl12029_write_reg(client,ISL12029_REG_SR,0);
			return -EIO;
		}
	}*/

	ret = isl12029_read_regs(client, ISL12029_REG_SR, buf, 1);
	if (ret)
	{
		dev_err(&client->dev, "isl12029_read_regs_: ISL12029_REG_SR\n");
		return ret;
	}
	dev_dbg(&client->dev, "ISL12029_REG_SR is %x\n",buf[0]);

	dev_info(&client->dev, "set time is OK\n");

	return 0;
}

static int isl12029_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	//printk(KERN_INFO "isl12029_rtc_read_time: run");

	return isl12029_get_datetime(to_i2c_client(dev), tm);
}

static int isl12029_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	//printk(KERN_INFO "isl12029_rtc_set_time: run");

	return isl12029_set_datetime(to_i2c_client(dev), tm);
}

int isl12029_rtc_irq_set_freq(struct device *dev, int freq)
{//для wd 0 - 1.75s, 1 - 750ms, 2 - 250ms, 3 = disable (default)

	int ret;
	unsigned char byte;
	unsigned char freq_shift;
	struct i2c_client *client;
	ret=0;byte=0;freq_shift=0;

	client = to_i2c_client(dev);

	dev_info(&client->dev, "isl12029_rtc_irq_set_freq: run\n");

	ret = isl12029_write_reg(client,ISL12029_REG_SR, ISL12029_SR_WEL);
        if (ret)
        {
	         isl12029_write_reg(client,ISL12029_REG_SR,0);
                 dev_err(&client->dev, "write enable RWEL\n");
                 return ret;
        }

	ret = isl12029_write_reg(client,ISL12029_REG_SR, ISL12029_SR_RWEL);
        if (ret)
        {
	         isl12029_write_reg(client,ISL12029_REG_SR,0);
                 dev_err(&client->dev, "write enable RWEL\n");
                 return ret;
        }

	ret = isl12029_read_regs(client, ISL12029_REG_BL, &byte, 1);
        if (ret)
        {
		dev_err(&client->dev, "isl12029_rtc_irq_set_freq: ISL12029_REG_BL\n");
                return ret;
        }

	freq_shift = ((freq<<3)&0x18); //WD0,WD1
        dev_dbg(&client->dev, "ISL12029_REG_BL is %x, freq bits %x\n",byte,freq_shift);

        ret = isl12029_write_reg(client, ISL12029_REG_BL, byte | freq_shift);
        if (ret)
        {
	        dev_err(&client->dev, "write WD freq\n");
                return ret;
        }

	dev_info(&client->dev, "isl12029_rtc_irq_set_freq: OK!\n");

        return 0;
}

static const struct rtc_class_ops isl12029_rtc_ops = {
	.read_time	= isl12029_rtc_read_time,
	.set_time	= isl12029_rtc_set_time,
	.irq_set_freq   = isl12029_rtc_irq_set_freq,
};

static int isl12029_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct isl12029 *isl12029;

	int ret = 0;

	printk(KERN_INFO "isl12029_probe: run");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "Error isl12029_probe: i2c_check_functionality");
		return -ENODEV;
	}

	isl12029 = kzalloc(sizeof(struct isl12029), GFP_KERNEL);
	if (!isl12029)
	{
		printk(KERN_ERR "Error isl12029_probe: kzalloc");
		return -ENOMEM;
	}

	dev_dbg(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	i2c_set_clientdata(client, isl12029);

	isl12029->rtc = rtc_device_register(isl12029_driver.driver.name,
								    &client->dev,
								    &isl12029_rtc_ops,
								    THIS_MODULE);
	dev_dbg(&client->dev, "CCR %s\n",id->name);


	if (IS_ERR(isl12029->rtc)) 
	{
		ret = PTR_ERR(isl12029->rtc);
		printk(KERN_ERR "Error isl12029_probe: isl12029->rtc");

		goto exit_kfree;
	}

	printk(KERN_INFO "isl12029_probe: ok");

	return 0;

exit_kfree:
	kfree(isl12029);

	return ret;
}

static int isl12029_remove(struct i2c_client *client)
{
	struct isl12029 *isl12029 = i2c_get_clientdata(client);

	rtc_device_unregister(isl12029->rtc);
	kfree(isl12029);

	return 0;
}

static const struct i2c_device_id isl12029_id[] = {
	{ "isl12029", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl12029_id);

static struct i2c_driver isl12029_driver = {
	.driver	= 
	{
		.name	= "rtc-isl12029",
	},
	.probe		= isl12029_probe,
	.remove		= isl12029_remove,
	.id_table	= isl12029_id,
};

static int __init isl12029_init(void)
{
	printk(KERN_INFO "isl12029_init: run");

	return i2c_add_driver(&isl12029_driver);
}

static void __exit isl12029_exit(void)
{
	i2c_del_driver(&isl12029_driver);
}

module_init(isl12029_init);
module_exit(isl12029_exit);

MODULE_AUTHOR("russl");
MODULE_DESCRIPTION("ISL 12029 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
