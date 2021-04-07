
#ifndef _LINUX_SERIAL_SC16IX7XX_H
#define _LINUX_SERIAL_SC16IX7XX_H

#include <linux/serial_core.h>
#include <linux/platform_device.h>

struct plat_serial_sc16ix7xx
{
	char   		name_i2c[5];    /* name i2c bus */
	unsigned long   num_i2c;        /* number i2c bus */
        unsigned long   iobase;         /* io base address */
        void __iomem    *membase;       /* ioremap cookie or NULL */
        resource_size_t mapbase;        /* resource base */
        unsigned int    irq;            /* interrupt number */
        unsigned long   irqflags;       /* request_irq flags */
        unsigned int    uartclk;        /* UART clock rate */
        void            *private_data;
        unsigned char   regshift;       /* register shift */
        unsigned char   iotype;         /* UPIO_* */
        unsigned char   hub6;
        upf_t           flags;          /* UPF_* flags */
        unsigned int    type;           /* If UPF_FIXED_TYPE */
        unsigned int    (*serial_in)	(struct uart_port *, int);
        void            (*serial_out)	(struct uart_port *, int, int);
        void            (*set_termios)	(struct uart_port *, 
                                       		struct ktermios *new,
                                       		struct ktermios *old);
        int             (*handle_irq)(struct uart_port *);
        void            (*pm)(struct uart_port *, unsigned int state, unsigned old);
};

struct uart_port;

#endif
