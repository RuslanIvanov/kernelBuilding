#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/crc32.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/workqueue.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/syscalls.h>
#include <linux/sys.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/poll.h>

#include <asm/io.h>

#include "ioctl_chst.h"

#define KSDT_DRV_VERSION 	"2017-03-28"
#define KSDT_DEVNAME 	 	"tksd"
#define KSDT_CHDEVNAME   	"chtksd"

#define  COUNT_DEVICES 1

#define DELAY 350 // HZ = 100 (где, HZ тиков в секунду) => 3.5c

static int period = DELAY;
module_param(period,int,0);

MODULE_LICENSE("GPL");
MODULE_VERSION(KSDT_DRV_VERSION);
MODULE_ALIAS("chdriver:ksdt");

static int SUCCESS;
static int FAIL = -1;
static int Major = 702;
static int Minor = 0;
static int Device_Status = 0;

static dev_t dev_ksdt;

// для внутренного представления символьных устройств:
//static struct cdev c_dev; 
struct cksdt_dev
{
        struct mutex mutex;
        struct cdev cdev;

	//struct timer_list ksd_timer;
	//wait_queue_head_t wqksdt ;//init_waitqueue_head(&(scull_p_devices[i].inq));
	//int flagksdt;
};

struct cksdt_dev c_dev;
struct cksdt_dev *ksdt_cdev; //&c_dev

static DECLARE_WAIT_QUEUE_HEAD(wqksdt);

struct timer_list ksd_timer;
static int flagksdt;

static void wakeUpUserMode(unsigned long par)
{
	flagksdt = 1;
	wake_up_interruptible(&wqksdt);// пробудить поток чтения
}

static void ksd_timer_function(unsigned long data)
{//t за 350мс (def)
	if(data == 1982)
	{
		//printk(KERN_INFO " _t_ ");

		mod_timer(&ksd_timer,jiffies + period);   
		wakeUpUserMode(period);
	}
}

static void startTimer(int period)
{
	init_timer(&ksd_timer);

        ksd_timer.expires = jiffies + period;
        ksd_timer.data = 1982;
        ksd_timer.function = ksd_timer_function;
        add_timer(&ksd_timer);
	printk(KERN_INFO "start timer %d mc",period);
}

static void stopTimer(void)
{
	del_timer_sync(&ksd_timer);
	printk(KERN_INFO "stop timer %d mc",period);
}

static long ksdt_ioctl(struct file *pfile,unsigned int cmd,unsigned long arg)
{
	int err;
	int retval;

	err = 0; retval = 0; 
	/* проверить тип и номер битовых полей и не декодировать
	* неверные команды: вернуть ENOTTY (неверный ioctl) перед access_ok( )
	*/

	if (_IOC_TYPE(cmd) != KSDT_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > KSDT_IOC_MAXNR) return -ENOTTY;

	/*
	 * направление является битовой маской и VERIFY_WRITE отлавливает передачи R/W
	 * `направление' является ориентированным на пользователя, в то время как
	 * access_ok является ориентированным на ядро, так что концепции "чтение" и
	 * "запись" являются обратными
	 */

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));


	if (err) return -EFAULT;

	switch(cmd) 
	{
        	case KSDT_IOCRESET:
		{
			flagksdt = 1;
			wake_up_interruptible(&wqksdt);

			retval = 0;
		}
		break;
		case KSDT_IOCG_PERIOD:
		{
			//для записи простой переменной из пространства ядра в пространство пользователя
			retval = __put_user(period, (int __user *)arg);
                	if(retval!=0) { printk(KERN_ERR "KSDT: KSDT_IOCG_PERIOD error get period witch IOCTL"); }

		}
		break;
		case KSDT_IOCS_PERIOD:
		{
			//для считывания простой переменной из пространства пользователя. 
			retval = __get_user(period, (int __user *)arg);
			if(retval!=0) { printk(KERN_ERR "KSDT: KSDT_IOCS_PERIOD error set period witch IOCTL"); }
			else { 
					if(period!=0) 
					{
						printk(KERN_INFO "KSDT: KSDT_IOCS_PERIOD set period %d",period);
						stopTimer();
						udelay(5);
						startTimer(period);
					} else { printk(KERN_INFO "KSDT: KSDT_IOCS_PERIOD don't set period %d",period); }
			     }
		}
		break;
		default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}

	return retval;
}

static int ksdt_open(struct inode *inode, struct file *pfile)
{

	if(Device_Status>=COUNT_DEVICES) return -EBUSY;

	Device_Status++;

    	try_module_get(THIS_MODULE);//увеличить счётчик ссылок для модуля (возвращается признак успешности операции);

	startTimer(period);

    	printk(KERN_INFO "KSDT: ksdt_open, count %d\n",Device_Status);

    	return SUCCESS;	
}

static int ksdt_release(struct inode *inode, struct file *pfile)
{//close

	if(Device_Status>0)
	{
		Device_Status--;

		module_put(THIS_MODULE);//уменьшить счётчик ссылок для модуля;

		printk(KERN_INFO "KSDT: ksdt_release, count %d\n",Device_Status+1);
	}

	stopTimer();	

	return SUCCESS;
}


static ssize_t ksdt_read (struct file *pfile, char __user *buffer, size_t length, loff_t *offset)
{//ssize_t - целое,

	if(wait_event_interruptible(wqksdt, flagksdt != 0)>0)
    	{
        	printk(KERN_ERR "KSDT: read error ERESTARTSYS");
        	return -ERESTARTSYS;
    	}
	flagksdt = 0;

	return 0;
}

static unsigned int ksdt_poll(struct file *pfile, poll_table *wait)
{
        unsigned int mask;
        mask = 0;

//-----------------------------------------------------------------
//	printk(KERN_ERR "KSDT: pos don't work!!!");
//	return POLLERR; //заглушка
//-----------------------------------------------------------------

	//cksdt_dev* dev = pfile->private_data; 

        poll_wait(pfile, &wqksdt, wait); // не верно работает, не ждет (?)

	if(flagksdt)//??
        {	mask |= POLLIN;//| POLLRDNORM; //чтение 
		flagksdt=0;
	}

        //mutex_unlock(&dev->mutex);

        return mask;
}

static struct file_operations fops = {
	.owner	= THIS_MODULE,
	.open = ksdt_open,
	.release = ksdt_release,
	.read = ksdt_read,
	.poll = ksdt_poll,
	.unlocked_ioctl = ksdt_ioctl,
};


static int ksdt_init(void)
{
	int err; 
	err=0;

	printk(KERN_INFO "KSDT: START TIMER %d\n",period);

	//выделение номеров устройств динамически-----------------------------------
	/*err = alloc_chrdev_region(&dev_ksdt, 0, COUNT_DEVICES, KSDT_CHDEVNAME);
    	if (err < 0)
    	{
        	printk(KERN_ALERT "KSDT: alloc_chrdev_region() error %d\n", err);
        	return FAIL;
	}

	Major = MAJOR(dev_ksdt);
	Minor = MINOR(dev_ksdt);
	printk(KERN_INFO  "KSDT: Major,Minor numbers [%d,%d]",Major,Minor);*/

	//новый способ регисtрации------------------------------------------------------
        dev_ksdt = MKDEV(Major,Minor);
        err = register_chrdev_region(dev_ksdt,COUNT_DEVICES,KSDT_CHDEVNAME);// получение номера(ов) символьного устройства

        if(err!=0) 
        {
                printk(KERN_ERR" Error register_chdev_region for %s code %d\n",KSDT_CHDEVNAME,err);
                unregister_chrdev_region(dev_ksdt, COUNT_DEVICES);
                return err;
        }
	//--------------------------------------------------------------------------------


	ksdt_cdev = &c_dev;

	// mutex_init(&(ksdt_cdev->mutex));


    	//инициализация структуры ksdt_cdev
    	cdev_init(&ksdt_cdev->cdev, &fops);
    	//регистрация структуры
    	if (cdev_add(&ksdt_cdev->cdev, dev_ksdt, COUNT_DEVICES) == -1)
    	{
        	unregister_chrdev_region(dev_ksdt, COUNT_DEVICES);
        	return FAIL;
    	}//*/
	
	return SUCCESS;
}

static void ksdt_cleanup(void)
{
    	unregister_chrdev_region(dev_ksdt,COUNT_DEVICES);
	cdev_del(&ksdt_cdev->cdev);

	printk(KERN_INFO "KSDT: STOP TIMER %d\n",period);
}

module_init(ksdt_init);
module_exit(ksdt_cleanup);


