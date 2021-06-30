/*
 * Tiny Serial driver
 *
 * Copyright (C) 2002-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, version 2 of the License.
 *
 * This driver shows how to create a minimal serial driver.  It does not rely on
 * any backing hardware, but creates a timer that emulates data being received
 * from some kind of hardware.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/version.h>

#include <linux/timer.h>

/*
setserial -a /dev/ttyS0
*/


#define DRIVER_AUTHOR "Greg Kroah-Hartman <greg@kroah.com>"
#define DRIVER_DESC "Tiny serial driver"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

#define DELAY_TIME		HZ * 2	/* 2 seconds per character */
#define TINY_DATA_CHARACTER	't'

#define TINY_SERIAL_MAJOR	707	/* experimental range */
#define TINY_SERIAL_MINORS	1	/* only have one minor */
#define UART_NR			1	/* only use one port */

#define TINY_SERIAL_NAME "ttyRM"

#define MY_NAME	TINY_SERIAL_NAME

struct port_data 
{
	struct timer_list timer;
	struct uart_port* port;
};

struct port_data tiny_port_data = {
	.port = NULL,
};

static int tiny_startup(struct uart_port *port);
//static inline void timer_setup(struct timer_list *timer, void (*callback)(struct timer_list *),unsigned int flags)

static void tiny_stop_tx(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_stop_tx\n");
}

static void tiny_stop_rx(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_stop_rx\n");
}

static void tiny_enable_ms(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_enable_ms\n");
}

static void tiny_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	printk(KERN_INFO "BRP tiny_tx_chars\n");

	int count;

	if (port->x_char) 
	{
		pr_debug("wrote %2x", port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		tiny_stop_tx(port);
		return;
	}

	count = port->fifosize >> 1;
	do {
		pr_debug("wrote %2x", xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		tiny_stop_tx(port);
}

static void tiny_start_tx(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_start_tx\n");
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static void tiny_timer(unsigned long arg)
{
#else
static void tiny_timer(struct timer_list* arg)
{
#endif

	struct uart_port *port;
	struct tty_port *tport;

	port = tiny_port_data.port;

	if (!port)
		return;
	if (!port->state)
		return;

	tport = &port->state->port;

	/* add one character to the tty port */
	/* this doesn't actually push the data through unless tty->low_latency is set */
	tty_insert_flip_char(tport, TINY_DATA_CHARACTER, 0);

	tty_flip_buffer_push(tport);

	/* resubmit the timer again */
	tiny_port_data.timer.expires = jiffies + DELAY_TIME;
	add_timer(&tiny_port_data.timer);

	/* see if we have any data to transmit */
	tiny_tx_chars(port);

	printk(KERN_INFO "BRP tiny_timer\n");
}

static unsigned int tiny_tx_empty(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_tx_empty\n");
	return 0;
}

static unsigned int tiny_get_mctrl(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_get_mctrl\n");
	return 0;
}

static void tiny_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	printk(KERN_INFO "BRP tiny_set_mctrl\n");
}

static void tiny_break_ctl(struct uart_port *port, int break_state)
{
	printk(KERN_INFO "BRP tiny_break_ctl\n");
}

static void tiny_set_termios(struct uart_port *port,
			     struct ktermios *new, struct ktermios *old)
{
	int baud, quot, cflag = new->c_cflag;

	printk(KERN_INFO "BRP tiny_set_termios\n");

	/* get the byte size */
	switch (cflag & CSIZE) 
	{
	case CS5:
		printk(KERN_DEBUG " - data bits = 5\n");
		break;
	case CS6:
		printk(KERN_DEBUG " - data bits = 6\n");
		break;
	case CS7:
		printk(KERN_DEBUG " - data bits = 7\n");
		break;
	default: // CS8
		printk(KERN_DEBUG " - data bits = 8\n");
		break;
	}

	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD)
			pr_debug(" - parity = odd\n");
		else
			pr_debug(" - parity = even\n");
	else
		pr_debug(" - parity = none\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		pr_debug(" - stop bits = 2\n");
	else
		pr_debug(" - stop bits = 1\n");

	/* figure out the flow control settings */
	if (cflag & CRTSCTS)
		pr_debug(" - RTS/CTS is enabled\n");
	else
		pr_debug(" - RTS/CTS is disabled\n");

	/* Set baud rate */
        baud = uart_get_baud_rate(port, new, old, 0, port->uartclk/16);
        quot = uart_get_divisor(port, baud);
	
	//UART_PUT_DIV_LO(port, (quot & 0xff));
	//UART_PUT_DIV_HI(port, ((quot & 0xf00) >> 8));
}

static int tiny_startup(struct uart_port *port)
{
	/* this is the first time this port is opened */
	/* do any hardware initialization needed here */

	/* create our timer and submit it */
	tiny_port_data.port = port;

	//timer_setup(&tiny_port_data.timer, tiny_timer, 0);

	#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    init_timer(&tiny_port_data.timer);
    tiny_port_data.timer.data = (unsigned long) port;
    tiny_port_data.timer.function = tiny_timer;
    /* ... */
    add_timer(&tiny_port_data.timer);
	#else
    timer_setup(&tiny_port_data.timer, tiny_timer, 0);
    /* the third argument may include TIMER_* flags */
    /* ... */
	#endif

	tiny_port_data.timer.expires = jiffies + DELAY_TIME;
	add_timer(&tiny_port_data.timer);

	printk(KERN_INFO "BRP tiny_startup\n");

	return 0;
}

static void tiny_shutdown(struct uart_port *port)
{
	/* The port is being closed by the last user. */
	/* Do any hardware specific stuff here */

	/* shut down our timer */

	printk(KERN_INFO "BRP iny_shutdown\n");
	del_timer(&tiny_port_data.timer);
}

static const char *tiny_type(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_type\n");
	return "ttyRM";
}

static void tiny_release_port(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_release_port\n");
}

static int tiny_request_port(struct uart_port *port)
{
	printk(KERN_INFO "BRP tiny_request_port\n");
	return 0;
}

static void tiny_config_port(struct uart_port *port, int flags)
{
	printk(KERN_INFO "BRP tiny_config_port\n");
}

static int tiny_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	printk(KERN_INFO "BRP tiny_verify_port\n");
	return 0;
}

static struct uart_ops tiny_ops = {
	.tx_empty	= tiny_tx_empty,
	.set_mctrl	= tiny_set_mctrl,
	.get_mctrl	= tiny_get_mctrl,
	.stop_tx	= tiny_stop_tx,
	.start_tx	= tiny_start_tx,
	.stop_rx	= tiny_stop_rx,
	.enable_ms	= tiny_enable_ms,
	.break_ctl	= tiny_break_ctl,
	.startup	= tiny_startup,
	.shutdown	= tiny_shutdown,
	.set_termios	= tiny_set_termios,
	.type		= tiny_type,
	.release_port	= tiny_release_port,
	.request_port	= tiny_request_port,
	.config_port	= tiny_config_port,
	.verify_port	= tiny_verify_port,
};

static struct uart_port tiny_port = {
	.ops		= &tiny_ops,
};

static struct uart_driver tiny_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= TINY_SERIAL_NAME,
	.dev_name	= TINY_SERIAL_NAME,
	.major		= TINY_SERIAL_MAJOR,
	.minor		= TINY_SERIAL_MINORS,
	.nr		= UART_NR,
};

static int __init tiny_init(void)
{
	int result;

	printk(KERN_INFO "BRP serial driver loaded\n");

	result = uart_register_driver(&tiny_reg);
	if (result)
	{
		printk(KERN_INFO "BRP serial driver loaded is error1 %d\n",result);
		return result;
	}

	result = uart_add_one_port(&tiny_reg, &tiny_port);

	if (result)
	{
		uart_unregister_driver(&tiny_reg);

		printk(KERN_INFO "BRP serial driver loaded is error2 %d\n",result);
	}

	printk(KERN_INFO "BRP serial driver loaded - OK %d\n",result);
	return result;
}


module_init(tiny_init);




/*
кто первым dызывается, device_register() или driver_register() ?

Как указано в Documentation/driver-model/binding.txt , не имеет значения, в каком конкретном порядке вы вызываете device_register() и driver_register() .

    device_register() добавляет устройство в список устройств и перебирает список драйверов , чтобы найти совпадение
    driver_register() добавляет драйвер в список драйверов и перебирает список устройств , чтобы найти совпадение

Как только совпадение найдено, сопоставленное устройство и драйвер связываются, и соответствующая функция зонда вызывается в коде драйвера.

Если вам все еще интересно , какой из них вызывается первым (потому что это не имеет значения) - обычно это device_register() , потому что устройства обычно регистрируются на initcalls от core_initcall до arch_initcall , а драйверы обычно регистрируются на device_initcall, которые выполняются позже. 
*/
