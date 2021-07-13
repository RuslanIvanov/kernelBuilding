/*
 *  linux/drivers/char/ttyprintk.c
 *
 *  Copyright (C) 2010  Samo Pogacnik
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the smems of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

/*
 * This pseudo device allows user to make printk messages. It is possible
 * to store "console" messages inline with kernel messages for better analyses
 * of the boot process, for example.
 */

#include <linux/device.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>


#define TST_MAX_ROOM 3584 /* we could assume 4K for instance */
#define PORT_NR 3

struct ttytest_port {
	struct device *dev[PORT_NR];
	struct tty_port *port[PORT_NR];
	struct mutex port_write_mutex;
};
static struct ttytest_port tst_port;


/*
 * TTY operations open function.
 */
static int tst_open(struct tty_struct *tty, struct file *filp)
{
	int i = 0;
	int port_id = tty->index;
	dev_t dev_num = 0;
	struct tty_driver *pdrv = tty->driver;

	printk("tst open\n");
	for (i = 0; i < PORT_NR; i++) {
		printk("drv_prot%d = [0x%px]", i, pdrv->ports[i]);
		printk("my_prot%d = [0x%px]", i, tst_port.port[i]);
	}

	dev_num = tty_devnum(tty);
	printk("tty port major:%d, minor:%d\n", MAJOR(dev_num), MINOR(dev_num));
		
	tty->driver_data = tst_port.port[port_id];
	return tty_port_open(tst_port.port[port_id], tty, filp);
}

/*
 * TTY operations close function.
 */
static void tst_close(struct tty_struct *tty, struct file *filp)
{
	struct tty_port *pport = tty->driver_data;
	printk("tst close\n");
	tty_port_close(pport, tty, filp);
}

/*
 * TTY operations write function.
 */
static int tst_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	//struct tst_port *pport = tty->driver_data;
	printk("tst_write:port_id[%d], drv:[0x%px]\n", tty->index, tty->driver);
	printk("%d: %s\n", count, buf);
	printk("%s(0): %px\n", __func__, __builtin_return_address(0));  
    printk("%s(1): %px\n", __func__, __builtin_return_address(1)); 
	return count;
}

/*
 * TTY operations ioctl function.
 */
static int tst_ioctl(struct tty_struct *tty,
			unsigned int cmd, unsigned long arg)
{
	struct tst_port *tstp = tty->driver_data;

	if (!tstp)
		return -EINVAL;

	switch (cmd) {
	/* Stop TIOCCONS */
	case TIOCCONS:
		return -EOPNOTSUPP;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static int tst_write_room(struct tty_struct *tty)
{
	printk("write room\n");
	return TST_MAX_ROOM;
}

static const struct tty_operations tst_ops = {
	.open = tst_open,
	.close = tst_close,
	.write = tst_write,
	.ioctl = tst_ioctl,
	.write_room = tst_write_room,
};

static const struct tty_port_operations null_ops = { };

static struct tty_driver *tst_driver;

static ssize_t show_my_device(struct device *dev,
                  struct device_attribute *attr, char *buf)
{
	printk("show\n");
    return 0;
}

static ssize_t store_my_device(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
	int i = 0;
	int port_id = 0;
	struct tty_port *pport = NULL;
	char data[10] = {0};

	printk("store\n");
	for (i = 0; i < 10; i++)
		data[i] = i + '0';

	data[8] = '\r';
	data[9] = '\n';

	if (count > 0) {
		switch (buf[0]) {
		case '0':
			port_id = 0;
			break;
		case '1':
			port_id = 1;
			break;
		case '2':
			port_id = 2;
			break;
		default:
			port_id = 0;
		}
	}
	printk("prot_id:%d\n", port_id);
	pport = tst_port.port[port_id];

	for (i = 0; i < 10; ++i) {
		tty_flip_buffer_push(pport);
		tty_insert_flip_char(pport, data[i], TTY_NORMAL);
	}
	tty_flip_buffer_push(pport);
	
    return count;
}
static DEVICE_ATTR(my_device_test, S_IWUSR|S_IRUSR, show_my_device, store_my_device);
static struct attribute *tst_dev_attrs[] = {
	&dev_attr_my_device_test.attr,
	NULL
};
static const struct attribute_group tst_attr_grp = {
       .attrs = tst_dev_attrs,
};


static int __init tst_init(void)
{
	int ret = -ENOMEM;
	int i = 0;
	struct tty_port *pport = NULL;
	struct device *pdev = NULL;
	printk("tty test driver init1\n");
	mutex_init(&tst_port.port_write_mutex);

	/*alloc tty driver*/
	tst_driver = alloc_tty_driver(PORT_NR);
	if (IS_ERR(tst_driver))
		return PTR_ERR(tst_driver);

	/*init tty driver*/
	tst_driver->driver_name = "tst";
	tst_driver->name = "tst";
	tst_driver->major = 0;
	tst_driver->minor_start = 0;
	tst_driver->subtype = SERIAL_TYPE_NORMAL;
	tst_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;;
	tst_driver->init_termios = tty_std_termios;
	tst_driver->init_termios.c_oflag = OPOST | OCRNL | ONOCR | ONLRET;
	tty_set_operations(tst_driver, &tst_ops);


	/*register tty driver*/
	ret = tty_register_driver(tst_driver);
	if (ret < 0) {
		printk(KERN_ERR "Couldn't register tst driver\n");
		goto error;
	}

	/*init tty port and device*/
	for (i = 0; i < PORT_NR; i++) {
		pport = kzalloc(sizeof(struct tty_port), GFP_KERNEL);
		tty_port_init(pport);
		pport->ops = &null_ops;
		pdev = tty_port_register_device(pport, tst_driver ,i, NULL);
		if (IS_ERR(pdev)) {
			ret = PTR_ERR(pdev);
			printk("could not register tty (ret=%i)\n", ret);
		}
		tst_port.port[i] = pport;
		tst_port.dev[i] = pdev;
	}

	/*init device attr*/
	for (i = 0; i < PORT_NR; i++) {
		pdev = tst_port.dev[i];
		ret = sysfs_create_group(&pdev->kobj, &tst_attr_grp);
	}
	
	printk("tty driver major:%d\n", tst_driver->major);

	return 0;

error:
	put_tty_driver(tst_driver);
	for (i = 0; i < PORT_NR; i++) {
		tty_port_destroy(tst_port.port[i]);
	}
	return ret;
}

static void __exit tst_exit(void)
{
	int i = 0;
	tty_unregister_driver(tst_driver);
	put_tty_driver(tst_driver);
	for (i = 0; i < PORT_NR; i++) {
		tty_port_destroy(tst_port.port[i]);
	}

}

device_initcall(tst_init);
module_exit(tst_exit);

MODULE_LICENSE("GPL");


