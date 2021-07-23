/*
 * Tiny TTY driver
 *
 * Copyright (C) 2002-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, version 2 of the License.
 *
 * This driver shows how to create a minimal tty driver.  It does not rely on
 * any backing hardware, but creates a timer that emulates data being received
 * from some kind of hardware.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/sched.h>
//#include <linux/sched/signal.h>
#include <linux/signal.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
//#include <asm/termios.h>
#include <linux/termios.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#define DRIVER_VERSION "v2.2"
#define DRIVER_AUTHOR "Russl 2021"
#define DRIVER_DESC "Tiny TTY driver on THREAD"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

#define DELAY_TIME		HZ * 2 	/* 2 seconds per character */
#define TINY_DATA_CHARACTER	't'

#define TINY_TTY_MAJOR		0	/* experimental range */
#define TINY_TTY_MINORS		1	/* only have 4 devices */

#define TINY_MAX_BUF 20000

#define USE_SIMULATOR

#if defined(USE_SIMULATOR)
static struct task_struct *thread_id;
static wait_queue_head_t wq_thread;
static DECLARE_COMPLETION(on_exit);
#endif /* USE_SIMULATOR */

char buf[TINY_MAX_BUF];

struct tiny_serial 
{
	struct tty_port	port;		/* pointer to the tty for this device */
	struct mutex port_write_mutex;
	struct tty_struct   *tty;       /* pointer to the tty for this device */

	/* for tiocmget and tiocmset functions */
	int			msr;		/* MSR shadow */
	int			mcr;		/* MCR shadow */

	/* for ioctl fun */
	struct serial_struct serial;
	wait_queue_head_t	wait;
	struct async_icount	icount;

	unsigned char bufferIn[TINY_MAX_BUF];
	unsigned char bufferOut[TINY_MAX_BUF];

	int indexIn;
	int indexOut;
	unsigned int session;

	int  open_count;         /* number of times this port has been opened */
    struct semaphore  sem;        /* locks this structure */
};

static struct tiny_serial *tiny_table[TINY_TTY_MINORS];	/* initially all NULL */

#if defined(USE_SIMULATOR) 
static int tiny_thread(void *thread_data)
{
    unsigned int timeoutMs;
    struct tiny_serial *tiny = (struct tiny_serial*)thread_data;
    struct tty_struct *tty;
    struct tty_port *port;

    
    int i;
	int endRead;
	int size_buf;

	i=0;
	endRead=0; 
	tty = NULL; 
	port =NULL;
	size_buf=0;
	memset(buf,0,TINY_MAX_BUF);

	strcpy(buf,"");

    allow_signal(SIGTERM);    

    pr_info("%s %s\n", __func__,__TIME__);

	//pr_info("%s, size_buf %d\n",__func__, size_buf);
	//pr_info("%s, tty->port %p\n",__func__,tty->port);
	
    tty = tiny->tty;
    port = &tiny->port;

	if(tiny && tty && port)
	{

		while(kthread_should_stop() == 0)
		{
		    timeoutMs = 1000;

			pr_info("%s, thread wait...\n",__func__);

		    timeoutMs = wait_event_interruptible_timeout( wq_thread, (timeoutMs==0), msecs_to_jiffies(timeoutMs) );//msecs_to_jiffies(timeoutMs) );

		    if(timeoutMs == -ERESTARTSYS)
		    {
		        pr_err("%s - signal break\n", __func__);
		        up(&tiny->sem);
		        //break;
				goto BREAK;
		    }

		    pr_info("%s %s\n", __func__,__TIME__);

		   // down(&tiny->sem);
			if(down_interruptible(&tiny->sem))
			{
				 pr_err("%s WAITING...(sleeping has been interrupted by a signal)\n", __func__);
		 		// return(-EINTR);  //sleeping has been interrupted by a signal 
			}else
			{
		    	if(tiny)
		   	 	{
					memcpy(buf,tiny->bufferIn,tiny->indexIn);
					size_buf = tiny->indexIn;

					printk(KERN_ERR "%s: size_buf %d:\n", __func__,size_buf);
		        	for (i = 0; i < size_buf; ++i)
		        	{
		          	  	if (!tty_buffer_request_room((struct tty_struct*)tty, 1))
		           	 	{
					    	tty_flip_buffer_push((struct tty_struct*)tty);
							printk(KERN_ERR "#i=%d...exit ",i);
							endRead=1;
							break;
						}
						
						printk(KERN_ERR "%x",buf[i]);
		           	 	tty_insert_flip_char((struct tty_struct*)tty, buf[i], TTY_NORMAL);
						//Функция, которая вставляет символы в переключаемый буфер tty устройства для чтения пользователем.

		        	}

					if(endRead==0)
					{
		        			tty_flip_buffer_push((struct tty_struct*)tty);//Функция, которая заталкивает данные для пользователя в текущий переключаемый буфер.
							printk(KERN_ERR "#send on user = %d bytes\n",tiny->indexIn);
						 	tiny->indexIn = 0;
					}
		    	}
		    	up(&tiny->sem);

				mdelay(2);
			}
		}
		//*/

	} else {  pr_err("%s: Error NULL pointers...tiny %p tty %p port %p\n", __func__,tiny,tty,port);  }

BREAK:

	pr_err("%s: thread end1\n", __func__);
    complete_and_exit(&on_exit, 0);
	pr_err("%s: thread end2\n", __func__);
}
#endif /* USE_SIMULATOR */


/*
 * this is the first time this port is opened
 * do any hardware initialization needed here
 */
static int tiny_activate(struct tty_port *tport, struct tty_struct *tty)
{
	struct tiny_serial *tiny;
	int size_buf;
	char our_thread[80];
	
	printk(KERN_INFO "\ntiny_activate\n");
	
	tiny = container_of(tport, struct tiny_serial, port);

	if(tiny)
	{

		#ifdef USE_SIMULATOR
	
		/////////////////////////////////////////////////////////////////////////////////////////////////////	
		down(&tiny->sem);
		//if(down_interruptible(&tiny->sem))
		//{
		// 	pr_err("%s WAITING...(sleeping has been interrupted by a signal)\n", __func__);
		// 	return(-EINTR);  // sleeping has been interrupted by a signal /
		//}

	 	// this is the first time this port is opened 
        // do any hardware initialization needed here 

		// save our structure within the tty structure 
    	//tty->driver_data = tiny;
    	tiny->tty = tty;

    	++tiny->open_count;

		 printk(KERN_INFO "tiny_activate: open_count %d\n",tiny->open_count);
    	if (tiny->open_count == 1) 
		{
	
      		if(thread_id == NULL)
        	{
				printk(KERN_INFO "%s: startr thread\n\n",__func__);	
				strcpy(&our_thread[0],"tiny_thread");
				
				thread_id = kthread_create(tiny_thread, (void*)tiny,our_thread);	
										
			} else pr_err("%s WAITING...thread id already yet\n", __func__); 

			if(thread_id)
			{
				printk(KERN_INFO "%s: wake_up thread\n",__func__);
				mdelay(3);
	        	wake_up_process(thread_id); 
			} else 
			{ 
				pr_err("%s WAITING...no wake_up_process for reading? thId==NULL\n", __func__); 
				up(&tiny->sem);
				return(-EINTR);
			}
		} else { printk(KERN_INFO "tiny_activate: thread already started %p\n",thread_id);}

		up(&tiny->sem);
		/////////////////////////////////////////////////////////////////////////////////////////////////////

		
	} else { pr_err("%s WAITING...error start thread, tiny %p\n", __func__,tiny); printk(KERN_INFO "tiny_activate: WAITING...error start thread, tiny %p\n",tiny); }//*/

	return 0;
}

/*
 * The port is being closed by the last user.
 * Do any hardware specific stuff here *
 */
static void tiny_shutdown(struct tty_port *tport)
{
	struct tiny_serial *tiny;

	printk(KERN_INFO "\ntiny_shutdown\n");

	tiny = container_of(tport, struct tiny_serial, port);
}

static int tiny_open(struct tty_struct *tty, struct file *file)
{
	struct tiny_serial *tiny;
	int index;
	struct tty_port *port;
	int status;
	//int size_buf;
	//char our_thread[80];
	
	printk(KERN_INFO "\ntiny[%d] is open\n",tty->index);

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	/* get the serial object associated with this tty pointer */
	index = tty->index;
	tiny = tiny_table[index];

	port = &tiny->port;

	tiny->indexIn=0;
	tiny->indexOut=0;
	tiny->session++;

	status = tty_port_open(port, tty, file);//??

	//if(!status) 
	//{
		/* save our structure within the tty structure */
		printk(KERN_INFO "\ntiny open save our structure within the tty structure\n");
		tty->driver_data = tiny;
	//}else { pr_err("%s Error tty_port_open\n", __func__); return(-EINTR); }
	
	printk("open session %d, status %d\n",tiny->session,status);

	/*if(tiny)
	{

		#ifdef USE_SIMULATOR
	
		/////////////////////////////////////////////////////////////////////////////////////////////////////	
		down(&tiny->sem);
		//if(down_interruptible(&tiny->sem))
		//{
		// 	pr_err("%s WAITING...(sleeping has been interrupted by a signal)\n", __func__);
		// 	return(-EINTR);  // sleeping has been interrupted by a signal /
		//}

	 	// this is the first time this port is opened 
        // do any hardware initialization needed here 

		// save our structure within the tty structure 
    	//tty->driver_data = tiny;
    	tiny->tty = tty;

    	++tiny->open_count;

		 printk(KERN_INFO "tiny_activate: open_count %d\n",tiny->open_count);
    	if (tiny->open_count == 1) 
		{
			
      		if(thread_id == NULL)
        	{
				printk(KERN_INFO "%s: startr thread\n\n",__func__);	
				strcpy(&our_thread[0],"tiny_thread");
				
				
			    thread_id = kthread_create(tiny_thread, (void*)tiny,our_thread);	
				
						
			} else pr_err("%s WAITING...thread id already yet\n", __func__); 

			if(thread_id)
			{
				printk(KERN_INFO "%s: wake_up thread\n",__func__);
				mdelay(3);
	        	wake_up_process(thread_id); 
			} else 
			{ 
				pr_err("%s WAITING...no wake_up_process for reading? thId==NULL\n", __func__); 
				up(&tiny->sem);
				return(-EINTR);
			}
		} else { printk(KERN_INFO "tiny_activate: thread already started %p\n",thread_id);}

		up(&tiny->sem);
		/////////////////////////////////////////////////////////////////////////////////////////////////////

		#endif 
	} else { pr_err("%s WAITING...error start thread, tiny %p\n", __func__,tiny); printk(KERN_INFO "tiny_activate: WAITING...error start thread, tiny %p\n",tiny); }//*/

	return 0;
//	return status;
}

static void tiny_close(struct tty_struct *tty, struct file *file)
{

	struct tiny_serial *tiny = tty->driver_data;
	struct tty_port *port;

	printk("tiny_close session %d\n",tiny->session);

	if (!tiny->open_count) 
	{
        /* port was never opened */
        goto exit;
    }
	printk("tiny_close: wait thread ended...\n");

	/////////////////////////////////////////////////////////////////////
	down(&tiny->sem);

	printk("tiny_close count %d (--)\n",tiny->open_count);
	--tiny->open_count;
    if (tiny->open_count <= 0) 
	{
        /* The port is being closed by the last user. */
        /* Do any hardware specific stuff here */   

		#if defined(USE_SIMULATOR)
    	 /* shut down our timer and free the memory */
	    if(thread_id)
       {

			printk("tiny_close: stop thread\n");
	
            kill_pid(task_pid(thread_id), SIGTERM, 1);
            wait_for_completion(&on_exit);
            thread_id = NULL;
			
			printk("tiny_close kill thread for session %d\n",tiny->session);

        }
		#endif /* USE_SIMULATOR */   
	
	}
	up(&tiny->sem);	
	

exit:

	port = &tiny->port;

	//tiny->indexIn=0;
	tiny->indexOut=0;
	tiny->session=0;

	if (tiny)
	{	tty_port_close(port, tty, file);
		printk("tiny_close port\n");
	}
	

	printk("tiny_close: END session %d\n",tiny->session);
	//////////////////////////////////////////////////////////////////////
}	

static int tiny_write(struct tty_struct *tty, const unsigned char *buffer, int count)
{//по 255 байт
	struct tiny_serial *tiny = tty->driver_data;
	int i;
	int ii;
	int retval;
	struct tty_port *port;
	unsigned long flags;

	if (!tiny) { return -ENODEV; }

	mutex_lock(&tiny->port_write_mutex);

	port = &tiny->port;
	spin_lock_irqsave(&port->lock, flags);

	if (!port->count) 
	{
		spin_unlock_irqrestore(&port->lock, flags);
		/* port was not opened */
		retval = -EINVAL;
		goto exit;
	}

	spin_unlock_irqrestore(&port->lock, flags);

	/* fake sending the data out a hardware port by
	 * writing it to the kernel debug log.
	 */
	
	printk(KERN_DEBUG "%s - ", __FUNCTION__);

	printk("session %d: count bytes apply %d: \n",tiny->session,count);

	for (i = 0; i < count; ++i)
	{
//			printk("%x", buffer[i]);

			if(((tiny->indexIn)+i) < TINY_MAX_BUF )
			{
				tiny->bufferIn[i+(tiny->indexIn)] = buffer[i];
			}
			
	}
	printk("\n");

	if(tiny->indexIn<TINY_MAX_BUF)
	{
		printk("bytes[%d]\n",i);

		for (ii = tiny->indexIn; (ii < TINY_MAX_BUF) && ( ii <  (i+tiny->indexIn) ) ; ++ii) { printk("%x.", tiny->bufferIn[ii]); }

		tiny->indexIn+=i;

		printk("\n");

		printk("all bytes in buf %d: \n",tiny->indexIn);
	} else {	printk("buf is full\n"); }
	
	retval = count;
		
exit:
	mutex_unlock(&tiny->port_write_mutex);

	return retval;
}

static int tiny_write_room(struct tty_struct *tty) 
{
	struct tiny_serial *tiny = tty->driver_data;
	int room = -EINVAL;
	//int i;
	struct tty_port *port;
	unsigned long flags;

	if (!tiny)
		return -ENODEV;

	mutex_lock(&tiny->port_write_mutex);
	
	port = &tiny->port;
	spin_lock_irqsave(&port->lock, flags);
	if (!port->count) {
		spin_unlock_irqrestore(&port->lock, flags);
		/* port was not opened */
		goto exit;
	}
	spin_unlock_irqrestore(&port->lock, flags);

	//printk("\n\nsession %d\n",tiny->session); 
	//for (i = 0; i <  tiny->indexIn; ++i)
	//{
	//	printk("%x", tiny->bufferIn[i]);
	//}
	//printk("\n"); 
	/* calculate how much room is left in the device */
	//room = 255;

	room = (TINY_MAX_BUF - tiny->indexIn);
	room = (room>=0)?room:0;

	printk("write_room: tail buf %d bytes, writed %d bytes", room,tiny->indexIn);

exit:
	mutex_unlock(&tiny->port_write_mutex);
	return room;
}

#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

static void tiny_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
	unsigned int cflag;

	printk(KERN_INFO "TINY: tiny_set_termios\n");

	cflag = tty->termios->c_cflag;

	/* check that they really want us to change something */
	if (old_termios) 
	{
		if ((cflag == old_termios->c_cflag) &&
		    (RELEVANT_IFLAG(tty->termios->c_iflag) ==
		     RELEVANT_IFLAG(old_termios->c_iflag))) {
			printk(KERN_DEBUG "termios - nothing to change...\n");
			return;
		}
	}

	/* get the byte size */
	switch (cflag & CSIZE) {
		case CS5:
			printk(KERN_DEBUG " - data bits = 5\n");
			break;
		case CS6:
			printk(KERN_DEBUG " - data bits = 6\n");
			break;
		case CS7:
			printk(KERN_DEBUG " - data bits = 7\n");
			break;
		default:
		case CS8:
			printk(KERN_DEBUG " - data bits = 8\n");
			break;
	}
	
	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD)
			printk(KERN_DEBUG " - parity = odd\n");
		else
			printk(KERN_DEBUG " - parity = even\n");
	else
		printk(KERN_DEBUG " - parity = none\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		printk(KERN_DEBUG " - stop bits = 2\n");
	else
		printk(KERN_DEBUG " - stop bits = 1\n");

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS)
		printk(KERN_DEBUG " - RTS/CTS is enabled\n");
	else
		printk(KERN_DEBUG " - RTS/CTS is disabled\n");
	
	/* determine software flow control */
	/* if we are implementing XON/XOFF, set the start and 
	 * stop character in the device */
	if (I_IXOFF(tty) || I_IXON(tty)) 
	{
		unsigned char stop_char  = STOP_CHAR(tty);
		unsigned char start_char = START_CHAR(tty);

		/* if we are implementing INBOUND XON/XOFF */
		if (I_IXOFF(tty))
			printk(KERN_DEBUG " - INBOUND XON/XOFF is enabled, "
				"XON = %2x, XOFF = %2x", start_char, stop_char);
		else
			printk(KERN_DEBUG" - INBOUND XON/XOFF is disabled");

		/* if we are implementing OUTBOUND XON/XOFF */
		if (I_IXON(tty))
			printk(KERN_DEBUG" - OUTBOUND XON/XOFF is enabled, "
				"XON = %2x, XOFF = %2x", start_char, stop_char);
		else
			printk(KERN_DEBUG" - OUTBOUND XON/XOFF is disabled");
	}

	/* get the baud rate wanted */
	printk(KERN_DEBUG " - baud rate = %d", tty_get_baud_rate(tty));
}

/* Our fake UART values */
#define MCR_DTR		0x01
#define MCR_RTS		0x02
#define MCR_LOOP	0x04
#define MSR_CTS		0x08
#define MSR_CD		0x10
#define MSR_RI		0x20
#define MSR_DSR		0x40

static int tiny_tiocmget(struct tty_struct *tty)
{
	struct tiny_serial *tiny = tty->driver_data;

	unsigned int result = 0;
	unsigned int msr = tiny->msr;
	unsigned int mcr = tiny->mcr;

	result = ((mcr & MCR_DTR)  ? TIOCM_DTR  : 0) |	/* DTR is set */
             ((mcr & MCR_RTS)  ? TIOCM_RTS  : 0) |	/* RTS is set */
             ((mcr & MCR_LOOP) ? TIOCM_LOOP : 0) |	/* LOOP is set */
             ((msr & MSR_CTS)  ? TIOCM_CTS  : 0) |	/* CTS is set */
             ((msr & MSR_CD)   ? TIOCM_CAR  : 0) |	/* Carrier detect is set*/
             ((msr & MSR_RI)   ? TIOCM_RI   : 0) |	/* Ring Indicator is set */
             ((msr & MSR_DSR)  ? TIOCM_DSR  : 0);	/* DSR is set */

	return result;
}

static int tiny_tiocmset(struct tty_struct *tty,
                         unsigned int set, unsigned int clear)
{
	struct tiny_serial *tiny = tty->driver_data;
	unsigned int mcr = tiny->mcr;

	if (set & TIOCM_RTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= MCR_RTS;

	if (clear & TIOCM_RTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~MCR_RTS;

	/* set the new MCR value in the device */
	tiny->mcr = mcr;
	return 0;
}

static int  tiny_tty_proc_show(struct seq_file *m, void *v)
{
	struct tiny_serial *tiny;
	int i;

	seq_printf(m, "<tinyserinfo:1.0 driver:%s>\n", DRIVER_VERSION);

	for (i = 0; i < TINY_TTY_MINORS; ++i) 
	{
		tiny = tiny_table[i];

		if (tiny == NULL)
		{
			printk("\n#device %d  is out, session %d",i,tiny->session); 
			continue;
		}

		seq_printf(m, "%d\n", i);

		printk("\nsession %d,",tiny->session); 
		printk(" N %d\n",tiny->indexIn);
		for (i = 0; i <  tiny->indexIn; ++i)
		{
			printk("%x", tiny->bufferIn[i]);
		}
		printk("\n"); 
	
	}
	return 0;
}


#define tiny_ioctl tiny_ioctl_tiocgserial
static int tiny_ioctl(struct tty_struct *tty,
                      unsigned int cmd, unsigned long arg)
{
	struct tiny_serial *tiny = tty->driver_data;

	if (cmd == TIOCGSERIAL) {
		struct serial_struct tmp;

		if (!arg)
			return -EFAULT;

		memset(&tmp, 0, sizeof(tmp));

		tmp.type		= tiny->serial.type;
		tmp.line		= tiny->serial.line;
		tmp.port		= tiny->serial.port;
		tmp.irq			= tiny->serial.irq;
		tmp.flags		= ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
		tmp.xmit_fifo_size	= tiny->serial.xmit_fifo_size;
		tmp.baud_base		= tiny->serial.baud_base;
		tmp.close_delay		= 5*HZ;
		tmp.closing_wait	= 30*HZ;
		tmp.custom_divisor	= tiny->serial.custom_divisor;
		tmp.hub6		= tiny->serial.hub6;
		tmp.io_type		= tiny->serial.io_type;

		if (copy_to_user((void __user *)arg, &tmp, sizeof(struct serial_struct)))
			return -EFAULT;
		return 0;
	}
	return -ENOIOCTLCMD;
}
#undef tiny_ioctl

#define tiny_ioctl tiny_ioctl_tiocmiwait
static int tiny_ioctl(struct tty_struct *tty,
                      unsigned int cmd, unsigned long arg)
{
	struct tiny_serial *tiny = tty->driver_data;

	if (cmd == TIOCMIWAIT) {
		DECLARE_WAITQUEUE(wait, current);
		struct async_icount cnow;
		struct async_icount cprev;

		cprev = tiny->icount;
		while (1) {
			add_wait_queue(&tiny->wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			remove_wait_queue(&tiny->wait, &wait);

			/* see if a signal woke us up */
			if (signal_pending(current))
				return -ERESTARTSYS;

			cnow = tiny->icount;
			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
			    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO; /* no change => error */
			if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
			    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
			    ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
			    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
				return 0;
			}
			cprev = cnow;
		}

	}
	return -ENOIOCTLCMD;
}
#undef tiny_ioctl

#define tiny_ioctl tiny_ioctl_tiocgicount
static int tiny_ioctl(struct tty_struct *tty,
                      unsigned int cmd, unsigned long arg)
{
	struct tiny_serial *tiny = tty->driver_data;

	if (cmd == TIOCGICOUNT) {
		struct async_icount cnow = tiny->icount;
		struct serial_icounter_struct icount;

		icount.cts	= cnow.cts;
		icount.dsr	= cnow.dsr;
		icount.rng	= cnow.rng;
		icount.dcd	= cnow.dcd;
		icount.rx	= cnow.rx;
		icount.tx	= cnow.tx;
		icount.frame	= cnow.frame;
		icount.overrun	= cnow.overrun;
		icount.parity	= cnow.parity;
		icount.brk	= cnow.brk;
		icount.buf_overrun = cnow.buf_overrun;

		if (copy_to_user((void __user *)arg, &icount, sizeof(icount)))
			return -EFAULT;
		return 0;
	}
	return -ENOIOCTLCMD;
}
#undef tiny_ioctl

/* the real tiny_ioctl function.  The above is done to get the small functions in the book */
static int tiny_ioctl(struct tty_struct *tty,
                      unsigned int cmd, unsigned long arg)
{
	switch (cmd) 
	{
		case TIOCGSERIAL:
			return tiny_ioctl_tiocgserial(tty, cmd, arg);
		case TIOCMIWAIT:
			return tiny_ioctl_tiocmiwait(tty, cmd, arg);
		case TIOCGICOUNT:
			return tiny_ioctl_tiocgicount(tty, cmd, arg);
	}

	return -ENOIOCTLCMD;
}

static struct tty_operations serial_ops = {
	.open = tiny_open,
	.close = tiny_close,
	.write = tiny_write,
	.write_room = tiny_write_room,
	.set_termios = tiny_set_termios,
	.tiocmget = tiny_tiocmget,
	.tiocmset = tiny_tiocmset,
	.ioctl = tiny_ioctl,
	//.proc_show       = tiny_tty_proc_show,
};

static const struct tty_port_operations tiny_port_ops = {
	.activate		= tiny_activate,
	.shutdown		= tiny_shutdown,
};

static struct tty_driver *tiny_tty_driver;

static int __init tiny_init(void)
{
	int retval;
	int i;
	int error;
	
	struct tiny_serial *tiny;
	
	pr_info("%s %s\n", __func__,__TIME__);

	i=0; 
	retval = 0;
	error =0;
	
#if defined(USE_SIMULATOR)
    init_waitqueue_head(&wq_thread);
    thread_id = NULL;
	pr_info("%s: USE_SIMULATOR\n",__func__);	
#endif /* USE_SIMULATOR */  

	/* allocate the tty driver */
	tiny_tty_driver = alloc_tty_driver(TINY_TTY_MINORS);
	if (!tiny_tty_driver)
		return -ENOMEM;

	/* initialize the tty driver */
	tiny_tty_driver->owner = THIS_MODULE;
	tiny_tty_driver->driver_name = "tty_BRM";//Имя драйвера, используемое в /proc/tty и sysfs
	tiny_tty_driver->name = "ttyBR";// Имя узла драйвера.
//	tiny_tty_driver->devfs_name = "tts/tttBR%d";
	tiny_tty_driver->major = TINY_TTY_MAJOR,
	tiny_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	tiny_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	tiny_tty_driver->flags = TTY_DRIVER_REAL_RAW /*| TTY_DRIVER_DYNAMIC_DEV*/, //TTY_DRIVER_NO_DEVFS
	tiny_tty_driver->init_termios = tty_std_termios;
	tiny_tty_driver->init_termios.c_lflag &= ~ECHO;
	tiny_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(tiny_tty_driver, &serial_ops);

	// register the tty driver 
	//При вызове tty_register_driver ядро создаёт в sysfs все различные tty файлы на весь
	//диапазон младших номеров tty устройств, которые может иметь этот tty драйвер

	retval = tty_register_driver(tiny_tty_driver);

	if (retval) 
	{
		printk(KERN_ERR "failed to register tiny tty driver, retval=%d", retval);
		goto err_tty_register_driver;
	}

	printk(KERN_ERR "ttyBR: register success. major=%d", tiny_tty_driver->major);

	for (i = 0; i < TINY_TTY_MINORS; ++i) 
	{
		/* let's create it */
		tiny = kmalloc(sizeof(struct tiny_serial/*tiny*/), GFP_KERNEL);
		if (!tiny) 
		{
			retval = -ENOMEM;
			printk(KERN_ERR "failed to alloc tiny_serial");
			goto err_kmalloc_tiny;
		}else {printk(KERN_ERR "size tiny_serial %d (%d)",sizeof(struct tiny_serial),sizeof(*tiny)); }

		mutex_init(&tiny->port_write_mutex);

		tiny->indexIn=0;
		tiny->indexOut=0;
		tiny->session=0;

		sema_init(&tiny->sem, 1);
    	tiny->open_count = 0;

		tiny_table[i] = tiny;
		tty_port_init(&tiny->port);
		tiny->port.ops = &tiny_port_ops;
	}


	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");

	return retval;

err_kmalloc_tiny:

	for (i = 0; i < TINY_TTY_MINORS; ++i) 
	{

		tiny = tiny_table[i];
		if (tiny) 
		{
			//tty_port_destroy(&tiny->port);
			kfree(tiny);
		}
	}

err_tty_register_driver:
	put_tty_driver(tiny_tty_driver);

	return retval;
}

static void __exit tiny_exit(void)
{
	struct tiny_serial *tiny;
	int i;

	printk(KERN_INFO "tiny_exit");

	for (i = 0; i < TINY_TTY_MINORS; ++i)
		tty_unregister_device(tiny_tty_driver, i);
	tty_unregister_driver(tiny_tty_driver);
	put_tty_driver(tiny_tty_driver);

	/* shut down all of the timers and free the memory */
	for (i = 0; i < TINY_TTY_MINORS; ++i) {
		tiny = tiny_table[i];
		/* close the port */
		/* FIXME: how to close the port ???
		 * using tty_hangup ??? */
		if(tiny->port.count)
			tiny_shutdown(&tiny->port);

		//tty_port_destroy(&tiny->port);

		kfree(tiny);
		tiny_table[i] = NULL;
	}
}

module_init(tiny_init);
module_exit(tiny_exit);

/*
Нет функции read?

Только с этими функциями драйвер tiny_tty может быть зарегистрирован, узел устройства открыт, данные записаны в устройство, узел устройства закрыт и отменена регистрация драйвера и он выгружен из ядра. Но ядро tty и структура tty_driver не предоставляет функцию чтения; в других словах, не существует функции обратного вызова для получения данных из драйвера ядром tty.

 

Вместо обычной функции чтения, за отправку в ядро tty любых данных, полученных от оборудования, несёт ответственность tty драйвер, когда он их получает. Ядро tty буферизует эти данные, пока они не запрошены пользователем. Поскольку ядро tty предоставляет логику буферизации, каждому tty драйверу нет необходимости реализовывать свою собственную логику буферизации. Ядро tty уведомляет tty драйвер, когда пользователь хочет, чтобы драйвер остановил и начал передачу данных, но если внутренние буферы tty полны, такого уведомления не происходит.

 

Ядро tty буферизует данные, полученные tty драйверами в структуре, названной struct tty_flip_buffer. Переключаемый буфер является структурой, которая содержит два основных массива данных. Данные, полученные от tty устройства хранятся в первом массиве. Когда этот массив полон, любой пользователь, ожидающий данные, уведомляется, что данные доступны для чтения. Хотя пользователь читает данные из этого массива, любые новые входящие данные хранятся во втором массиве. Когда этот массив заполнен, данные вновь сбрасываются пользователю, а драйвер начинает заполнять первый массив. По сути, принятые данные "переключаются" с одного буфера в другой, в надежде не переполнить их обоих. Чтобы попытаться предотвратить потерю данных, tty драйвер может контролировать, насколько велик входной массив, и если он заполнится, приказать tty драйверу очистить переключаемый буфер в этот момент времени, а не ожидать следующего шанса.

 

Детали структуры struct tty_flip_buffer не имеют значения для tty драйвера за с одним исключением, переменной count. Эта переменная содержит сколько байт в настоящее время остаётся в буфере, который используется для приёма данных. Если это значение равно значению TTY_FLIPBUF_SIZE, буфер необходимо сбросить пользователю вызовом tty_flip_buffer_push. Это показано следующим фрагментом кода:

 

for (i = 0; i < data_size; ++i) {

    if (tty->flip.count >= TTY_FLIPBUF_SIZE)

        tty_flip_buffer_push(tty);

    tty_insert_flip_char(tty, data[i], TTY_NORMAL);

}

tty_flip_buffer_push(tty);

 

Символы, полученные от tty драйвера для отправки пользователю, добавляются в переключаемый буфер вызовом tty_insert_flip_char. Первым параметром этой функции является struct tty_struct, где должны быть сохранены данные, вторым параметром является символ для сохранения и третьим параметром являются любые флаги, которые должны быть установлены для этого символа. Значение флагов должно быть установлено в TTY_NORMAL, если получен обычный символ. Если это особый тип символа, указывающий на ошибку передачу данных, оно должно быть установлено на TTY_BREAK, TTY_FRAME, TTY_PARITY или TTY_OVERRUN, в зависимости от ошибки.

 

Для того, чтобы "протолкнуть" данные пользователю, выполняется вызов tty_flip_buffer_push. Этот функция должна быть также вызвана, если переключаемый буфера близок к переполнению, как показано в этом примере. Поэтому, когда данные добавляются в переключаемый буфер, или когда переключаемый буфер заполнен, tty драйвер должен вызвать tty_flip_buffer_push. Если tty драйвер может принимать данные на очень высоких скоростях, должен быть установлен флаг tty->low_latency, что приводит при вызове к немедленному вызову tty_flip_buffer_push. В противном случае, вызов tty_flip_buffer_push планирует себя для выталкивания данных из буфера когда-то позднее в ближайшем будущем.
Предыдущая  Содержание  Следующая
*/
