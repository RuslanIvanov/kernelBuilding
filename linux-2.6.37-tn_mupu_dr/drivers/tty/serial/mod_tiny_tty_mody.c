 /*
 Почему я получаю ошибку при использовании echo в файле устройства?

    echo test > /dev/ttyBR0
    bash: echo: write error: Invalid argument

Драйвер работает на Raspberry Pi kernel 4.9.56-v7.

UPDATE: первая проблема (частично) решена с помощью решения в tty_flip_buffer_push() отправляет данные обратно к себе . Есть ли способ сделать это непосредственно в драйвере устройства, чтобы пользователь не требовал никакого взаимодействия? */

/*


Сообщение уже немного устарело, но я наткнулся на те же проблемы и решил поделиться решением:

    вы можете отключить эхо, установив соответствующие флаги в структуре termios:

    tiny_tty_driver->init_termios.c_lflag &= ~ECHO;

    это связано с тем , что tiny_write всегда возвращает -EINVAL, установите retval = count; перед возвратом, чтобы исправить это.


*/

/*
 * Tiny TTY driver
 *
 * Base on tiny tty driver from Greg Kroah-Hartman
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
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>

#define USE_SIMULATOR

#define DELAY_TIME      HZ * 2  /* 2 seconds per character */

#define TINY_TTY_MAJOR      252 /* experimental range */
#define TINY_TTY_MINORS     1   /* only have 4 devices */

#if defined(USE_SIMULATOR)
static struct task_struct *thread_id;
static wait_queue_head_t wq_thread;
static DECLARE_COMPLETION(on_exit);
#endif /* USE_SIMULATOR */

struct tiny_serial {
    struct tty_struct   *tty;       /* pointer to the tty for this device */
    int         open_count;         /* number of times this port has been opened */
    struct semaphore    sem;        /* locks this structure */
};

static struct tiny_serial *tiny_serial; /* initially all NULL */

#if defined(USE_SIMULATOR)
static int tiny_thread(void *thread_data)
{
    unsigned int timeoutMs;
    struct tiny_serial *tiny = (struct tiny_serial*)thread_data;
    struct tty_struct *tty;
    struct tty_port *port;
    char buf[] = "hello world\n";
    int i = 0;

    allow_signal(SIGTERM);    

    pr_info("%s\n", __func__);

    tty = tiny->tty;
    port = tty->port;

    while(kthread_should_stop() == 0)
    {
        timeoutMs = 1000;
        timeoutMs = wait_event_interruptible_timeout(wq_thread, (timeoutMs==0), msecs_to_jiffies(timeoutMs));

        if(timeoutMs == -ERESTARTSYS)
        {
            pr_info("%s - signal break\n", __func__);
            up(&tiny->sem);
            break;
        }

        pr_info("%s %s\n", __func__,__TIME__);

        down(&tiny->sem);

        if(tiny)
        {
            for (i = 0; i < strlen(buf); ++i)
            {
                if (!tty_buffer_request_room(tty->port, 1))
                    tty_flip_buffer_push(tty->port);
                tty_insert_flip_char(tty->port, buf[i], TTY_NORMAL);

            }
            tty_flip_buffer_push(tty->port);
        }
        up(&tiny->sem);
    }

    complete_and_exit(&on_exit, 0);
}
#endif /* USE_SIMULATOR */

static int tiny_open(struct tty_struct *tty, struct file *file)
{
    pr_info("%s\n", __func__);

    /* initialize the pointer in case something fails */
    tty->driver_data = NULL;

    /* get the serial object associated with this tty pointer */
    if(tiny_serial == NULL) {
        /* first time accessing this device, let's create it */
        tiny_serial = kmalloc(sizeof(*tiny_serial), GFP_KERNEL);
        if (!tiny_serial)
            return -ENOMEM;

        sema_init(&tiny_serial->sem, 1);
        tiny_serial->open_count = 0;
    }

    down(&tiny_serial->sem);

    /* save our structure within the tty structure */
    tty->driver_data = tiny_serial;
    tiny_serial->tty = tty;

    ++tiny_serial->open_count;
    if (tiny_serial->open_count == 1) {
        /* this is the first time this port is opened */
        /* do any hardware initialization needed here */
#if defined(USE_SIMULATOR)      
        if(thread_id == NULL)
            thread_id = kthread_create(tiny_thread, (void*)tiny_serial, "tiny_thread");
        wake_up_process(thread_id); 
#endif /* USE_SIMULATOR */        
    }

    up(&tiny_serial->sem);
    return 0;
}

static void do_close(struct tiny_serial *tiny)
{
    pr_info("%s\n", __func__);

    down(&tiny->sem);

    if (!tiny->open_count) {
        /* port was never opened */
        goto exit;
    }

    --tiny->open_count;
    if (tiny->open_count <= 0) {
        /* The port is being closed by the last user. */
        /* Do any hardware specific stuff here */   

#if defined(USE_SIMULATOR)
        /* shut down our timer and free the memory */
        if(thread_id)
        {
            kill_pid(task_pid(thread_id), SIGTERM, 1);
            wait_for_completion(&on_exit);
            thread_id = NULL;
        }
#endif /* USE_SIMULATOR */                  

    }
exit:
    up(&tiny->sem);
}

static void tiny_close(struct tty_struct *tty, struct file *file)
{
    struct tiny_serial *tiny = tty->driver_data;

    pr_info("%s\n", __func__);

    if (tiny)
        do_close(tiny);
}   

static int tiny_write(struct tty_struct *tty, 
              const unsigned char *buffer, int count)
{
    struct tiny_serial *tiny = tty->driver_data;
    int i;
    int retval = -EINVAL;

    if (!tiny)
        return -ENODEV;

    down(&tiny->sem);

    if (!tiny->open_count)
        /* port was not opened */
        goto exit;

    /* fake sending the data out a hardware port by
     * writing it to the kernel debug log.
     */
    printk(KERN_DEBUG "%s - ", __FUNCTION__);
    for (i = 0; i < count; ++i)
    {
        printk("%02x ", buffer[i]);        
    }
    printk("\n");


	retval = count;

exit:
    up(&tiny->sem);

    return retval;
}

static int tiny_write_room(struct tty_struct *tty) 
{
    struct tiny_serial *tiny = tty->driver_data;
    int room = -EINVAL;

    pr_info("%s\n", __func__);

    if (!tiny)
        return -ENODEV;

    down(&tiny->sem);

    if (!tiny->open_count) {
        /* port was not opened */
        goto exit;
    }

    /* calculate how much room is left in the device */
    room = 255;

exit:
    up(&tiny->sem);
    return room;
}

static void tiny_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
    pr_info("%s\n", __func__);    
}

static int tiny_install(struct tty_driver *driver, struct tty_struct *tty)
{
    int retval = -ENOMEM;

    pr_info("%s\n", __func__);

    tty->port = kmalloc(sizeof *tty->port, GFP_KERNEL);
    if (!tty->port)
        goto err;

    tty_init_termios(tty);
    driver->ttys[0] = tty;

    tty_port_init(tty->port);
   // tty_buffer_set_limit(tty->port, 8192);
    tty_driver_kref_get(driver);
    tty->count++;   

    return 0;

err:
    pr_info("%s - err\n", __func__);
    kfree(tty->port);
    return retval;
}

static struct tty_operations serial_ops = {
    .open = tiny_open,
    .close = tiny_close,
    .write = tiny_write,
    .write_room = tiny_write_room,
    .set_termios = tiny_set_termios,
    .install        = tiny_install,
};

static struct tty_driver *tiny_tty_driver;

static int __init tiny_init(void)
{
    int retval;

    pr_info("%s %s\n", __func__,__TIME__);

#if defined(USE_SIMULATOR)
    init_waitqueue_head(&wq_thread);
    thread_id = NULL;
#endif /* USE_SIMULATOR */  

    /* allocate the tty driver */
    tiny_tty_driver = alloc_tty_driver(TINY_TTY_MINORS);
    if (!tiny_tty_driver)
        return -ENOMEM;

    /* initialize the tty driver */
    tiny_tty_driver->owner = THIS_MODULE;
    tiny_tty_driver->driver_name = "tty_BRM";
    tiny_tty_driver->name = "ttyBR";
    tiny_tty_driver->major = TINY_TTY_MAJOR,
    tiny_tty_driver->type = TTY_DRIVER_TYPE_SYSTEM,
    tiny_tty_driver->subtype = SYSTEM_TYPE_CONSOLE,
    tiny_tty_driver->flags = TTY_DRIVER_REAL_RAW /*| TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_UNNUMBERED_NODE*/,
    tiny_tty_driver->init_termios = tty_std_termios;
	tiny_tty_driver->init_termios.c_lflag &= ~ECHO;
    tiny_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
    tty_set_operations(tiny_tty_driver, &serial_ops);

	

    /* register the tty driver */
    retval = tty_register_driver(tiny_tty_driver);
    if (retval) {
        printk(KERN_ERR "failed to register tiny tty driver");
        put_tty_driver(tiny_tty_driver);
        return retval;
    }

	printk(KERN_ERR "ttyBR: register success. major=%d", tiny_tty_driver->major);

    //tty_register_device(tiny_tty_driver, 0, NULL);

    //tiny_install(tiny_tty_driver, tiny_table[0]->tty);
    return retval;
}

static void __exit tiny_exit(void)
{
    pr_info("%s\n", __func__);

#if defined(USE_SIMULATOR)
    if(thread_id)
    {
        /* shut down our timer and free the memory */
        kill_pid(task_pid(thread_id), SIGTERM, 1);
        wait_for_completion(&on_exit);
    }
#endif /* USE_SIMULATOR */  

    tty_unregister_device(tiny_tty_driver, 0);
    tty_unregister_driver(tiny_tty_driver);

    if (tiny_serial) {
        /* close the port */
        while (tiny_serial->open_count)
            do_close(tiny_serial);

        if(tiny_serial->tty->port)
        {
            kfree(tiny_serial->tty->port);
            tiny_serial->tty->port = NULL;
        }
        kfree(tiny_serial);
        tiny_serial = NULL;
    }
}

module_init(tiny_init);
module_exit(tiny_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Tiny TTY driver");
MODULE_LICENSE("GPL");
