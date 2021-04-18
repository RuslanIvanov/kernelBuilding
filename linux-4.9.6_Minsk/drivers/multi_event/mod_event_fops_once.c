#include <linux/module.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>

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

#include "ioctl_event.h"

#define KSD_EVENT_DRV_VERSION   "2021-03-15"
#define KSD_EVENT_DEVNAME 	"ksd_event"
static char* KSD_EVENT_CHNAME = "cheventksd0";

#define COUNT_DEVICES 2

#define DELAY 350 // 350 mc

static  int period = DELAY;
module_param(period,int,0);


MODULE_LICENSE("GPL");
MODULE_VERSION(KSD_EVENT_DRV_VERSION);
MODULE_ALIAS("chdriver:ksdevent");

static int SUCCESS;
static int FAIL = -1;
static int Major = 704;
static int Minor = 0;
static int Device_Status = 0;

static dev_t dev_ksdev;

//для внутренного представления символьных устройств:
struct cksdev_dev
{
        struct mutex mutex;
        struct cdev cdev;
};

struct cksdev_dev ksdev_cdev;
struct cksdev_dev *pksdev_cdev; //&ksdev_cdev

static DECLARE_WAIT_QUEUE_HEAD(wqksdt);

static int flagksdt;

static unsigned char param;

static void wakeUpUserMode(unsigned long par)
{
	flagksdt = 1;
	wake_up_interruptible(&wqksdt);// пробудить поток чтения
}

static long ksdev_ioctl(struct file *pfile,unsigned int cmd,unsigned long arg)
{
	int err;
	int retval;

	err = 0; retval = 0; 
	/* проверить тип и номер битовых полей и не декодировать
	* неверные команды: вернуть ENOTTY (неверный ioctl) перед access_ok( )
	*/


	if (_IOC_TYPE(cmd) != KSD_EVENT_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > KSD_EVENT_IOC_MAXNR) return -ENOTTY;

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
        	case KSD_EVENT_IOCRESET:
		{
			flagksdt = 1;
			wake_up_interruptible(&wqksdt);// приводит к отработка по событию (ф. read() - разблокируется)

			retval = 0;
		}
		break;
		case KSD_EVENT_IOCCLEAR:
			flagksdt = 0;

            retval = 0;
		break;
		case KSD_EVENT_IOCG_PERIOD:
		{
			//для записи простой переменной из пространства ядра в пространство пользователя
			retval = __put_user(period, (int __user *)arg);
            if(retval!=0) { printk(KERN_ERR "KSDTHR: KSDT_IOCG_PERIOD error get period witch IOCTL"); }

		}
		break;
		case KSD_EVENT_IOCS_PERIOD:
		{
			//для считывания простой переменной из пространства пользователя. 
			retval = __get_user(period, (int __user *)arg);
			if(retval!=0) { printk(KERN_ERR "KSDTHR: KSDT_IOCS_PERIOD error set period witch IOCTL"); }
			else { 
					if(period!=0) 
					{
						printk(KERN_INFO "KSDTHR: KSDT_IOCS_PERIOD set period %d",period);
						
					} else 
					{ 						
						printk(KERN_INFO "KSDTHR: KSDT_IOCS_PERIOD don't set period %d, appply default %d",period,DELAY); 
						period=DELAY;
					}
			     }
		}
		break;
		default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}

	return retval;
}

static int ksdev_open(struct inode *inode, struct file *pfile)
{

	if(Device_Status>=COUNT_DEVICES) return -EBUSY;

	Device_Status++;

    	try_module_get(THIS_MODULE);//увеличить счётчик ссылок для модуля (возвращается признак успешности операции);

	printk(KERN_INFO "KSD_EVENT: ksdev_open, count %d\n",Device_Status);

	if(Device_Status==1)
	{
		printk(KERN_INFO "KSD_EVENT: ksdev_open, device %d \n",Device_Status);
	}

    	return SUCCESS;
}

static int ksdev_release(struct inode *inode, struct file *pfile)
{//close

	if(Device_Status>0)
	{
		Device_Status--;

		module_put(THIS_MODULE);//уменьшить счётчик ссылок для модуля;

		printk(KERN_INFO "KSD_EVENT: ksdev_release, count %d\n",Device_Status+1);
	}

	if(Device_Status<=0)
	{
		printk(KERN_INFO "KSD_EVENT: ksdev_release, device %d",Device_Status+1);
	}

	return SUCCESS;
}


static ssize_t ksdev_write (struct file *filp, const char __user *buff, size_t count,loff_t *offp)
{//ssize_t - целое,
	int retval;
	retval=0;

	wakeUpUserMode(0);

	if (copy_from_user(&param,buff,1))
                retval = -EFAULT;
        else  retval = count;

	return retval;
}

/*
wait_event_interruptible_timeout - sleep until a condition gets true or a timeout elapses 

DESCRIPTION
The process is put to sleep (TASK_INTERRUPTIBLE) until the condition evaluates to true or a signal is received.
The condition is checked each time the waitqueue wq is woken up.
wake_up has to be called after changing any variable that could change the result of the wait condition.

RETURN
0 if the condition evaluated to false after the timeout elapsed, 1 if the condition 
evaluated to true after the timeout elapsed, the remaining jiffies (at least 1) 
if the condition evaluated to true before the timeout elapsed, or -ERESTARTSYS if it was interrupted by a signal. 
*/

static ssize_t ksdev_read (struct file *pfile, char __user *buffer, size_t length, loff_t *offset)
{//ssize_t - целое,
	int retval;
	long ret;
	unsigned long timeout;
	struct timespec tVal;

	retval=0;
	ret=0;

	tVal.tv_sec=0;
	tVal.tv_nsec =(unsigned long)period * 1000000;

	timeout = timespec_to_jiffies(&tVal);

 	//timeout = 350;//500;//?
	//timeout = jiffies + HZ*2; //через 2 сек от текущего момента
	//timeout = jiffies + HZ/2;//0,5c

	ret = wait_event_interruptible_timeout(wqksdt,flagksdt != 0,timeout);
	if(ret==0) // время истекло
    {
        	printk(KERN_INFO "\nKSD_EVENT: read error ERESTARTSYS, TIMEOUT %d",period);        	
			length = 0;

			//return -ERESTARTSYS;
    }

	flagksdt = 0;

	if (copy_to_user(buffer,&param, 1))
		retval = -EFAULT;
	else
		retval = length;

	return retval;
}

// old:
/*
static ssize_t ksdev_read (struct file *pfile, char __user *buffer, size_t length, loff_t *offset)
{//ssize_t - целое,
	int retval;
	retval=0;

 	if(wait_event_interruptible(wqksdt,flagksdt != 0)>0)
    {
        	printk(KERN_ERR "KSD_EVENT: read error ERESTARTSYS");
        	return -ERESTARTSYS;
    }

	flagksdt = 0;

	if (copy_to_user(buffer,&param, 1))
		retval = -EFAULT;
	else
		retval = length;

	return retval;
}//*/

static unsigned int ksdev_poll(struct file *pfile, poll_table *wait)
{
	unsigned int mask;
	mask = 0;

//-----------------------------------------------------------------
//	printk(KERN_ERR "KSD_EVENT: pos don't work!!!");
//	return POLLERR; //заглушка
//-----------------------------------------------------------------

	//cksdev_dev* dev = pfile->private_data; 

    poll_wait(pfile, &wqksdt, wait); // не верно работает, не ждет (?)

	if(flagksdt)//??
    {
		mask |= POLLIN;//| POLLRDNORM; //чтение 
		flagksdt=0;
	}

	//mutex_unlock(&dev->mutex);

	return mask;
}

static struct file_operations fops = {
	.owner	= THIS_MODULE,
	.open = ksdev_open,
	.release = ksdev_release,
	.read = ksdev_read,
	.write = ksdev_write,
	.poll = ksdev_poll,
	.unlocked_ioctl = ksdev_ioctl,
};


static int ksdev_init(void)
{
	int err;
	err=0;

	//выделение номеров устройств динамически-----------------------------------
	/*err = alloc_chrdev_region(&dev_ksdev, 0, COUNT_DEVICES, KSD_EVENT_CHNAME);
    	if (err < 0)
    	{
        	printk(KERN_ALERT "KSD_EVENT: alloc_chrdev_region() error %d\n", err);
        	return FAIL;
	}

	Major = MAJOR(dev_ksdev);
	Minor = MINOR(dev_ksdev);
	printk(KERN_INFO  "KSD_EVENT: Major,Minor numbers [%d,%d]",Major,Minor);*/

	//новый способ регисtрации------------------------------------------------------
        dev_ksdev = MKDEV(Major,Minor);
        err = register_chrdev_region(dev_ksdev,COUNT_DEVICES,KSD_EVENT_CHNAME);// получение номера(ов) символьного устройства

        if(err!=0) 
        {
                printk(KERN_ERR"KSD_EVENT: Error register_chdev_region for %s code %d\n",KSD_EVENT_CHNAME,err);
                unregister_chrdev_region(dev_ksdev, COUNT_DEVICES);
                return err;
        }
	//--------------------------------------------------------------------------------


	pksdev_cdev = &ksdev_cdev;
	// mutex_init(&(pksdev_cdev->mutex));

    	//инициализация структуры ksdev_cdev
    	cdev_init(&pksdev_cdev->cdev, &fops);
    	//регистрация структуры
    	if (cdev_add(&pksdev_cdev->cdev, dev_ksdev, COUNT_DEVICES) == -1)
    	{
        	unregister_chrdev_region(dev_ksdev, COUNT_DEVICES);
		printk(KERN_ERR"KSD_EVENT: Error cdev_add for %s code %d\n",KSD_EVENT_CHNAME,err);

        	return FAIL;
    	}//*/

	printk(KERN_INFO "KSD_EVENT: LOAD EVENT MODULE: Major %d, Minor %d",Major,Minor);

	return SUCCESS;
}

static void ksdev_cleanup(void)
{
    unregister_chrdev_region(dev_ksdev,COUNT_DEVICES);
	cdev_del(&pksdev_cdev->cdev);

	printk(KERN_INFO "KSD_EVENT: UNLOAD EVENT MODULE \n");
}

module_init(ksdev_init);
module_exit(ksdev_cleanup);


