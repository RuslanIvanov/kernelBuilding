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

#include "mod_pvv.h"
#include "ioctl_pvv5.h"

#define PVV1_DRV_VERSION	"2019-10-17"
#define PVV1_DEVNAME "pvv5"
#define PVV1_CHDEVNAME "chpvv5"
#define GPIO_PVV1_DEVICE_DESC   "pvv5_device"

#define  COUNT_DEVICES 2 //pvv5:  two dev

MODULE_LICENSE("GPL");
MODULE_VERSION(PVV1_DRV_VERSION);
MODULE_ALIAS("chdriver:pvv5");

static int SUCCESS;
static int FAIL = -1;
static int Major = 702;
static int Minor = 0;
static int Device_Status = 0;

static dev_t dev_pvv;

// для внутренного представления символьных устройств:
//static struct cdev c_dev; 
struct cpvv_dev
{
        int current_irq;
	wait_queue_head_t wq;
	struct mutex mutex;
        struct cdev cdev;
};

struct cpvv_dev c_dev;
struct cpvv_dev *pvv_cdev;

struct pvv_data *pvvdev;

static DECLARE_WAIT_QUEUE_HEAD(wqrm);
//static DECLARE_WAIT_QUEUE_HEAD(wqdpu);

static int flagrm = 0;
//static int flagdpu = 0;

unsigned char errorPvv;

#define IRQ_PVV1   1
#define GPIO_PVV1  0
struct mutex mutex;
spinlock_t r_spinlock = SPIN_LOCK_UNLOCKED;
int irqN;

void wakeUpUserMode(unsigned long par)
{
	flagrm = 1;
	wake_up_interruptible(&wqrm);// пробудить поток чтения

	//flagdpu = 1;
	//wake_up_interruptible(&wqdpu);// пробудить поток чтения
}

static irqreturn_t pvv_irqhandler(int irq, void *dev_id)
{
	int id;
	static int count;
	id = *((int*)dev_id);

	irqN  = irq;
	wakeUpUserMode(IRQ_PVV1);

	if(count==0)
	{
		printk(KERN_INFO "MPVV1: IRQ_PVV1 irq %d\n",irq);
		count =1;
	}

	return IRQ_HANDLED;
}

static long pvv_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{
	int err;
	int retval;
	int typeIrq;
	int rez;

	err = 0; retval = 0; typeIrq = 0;
	rez = 0;
	/* проверить тип и номер битовых полей и не декодировать
	* неверные команды: вернуть ENOTTY (неверный ioctl) перед access_ok( )
	*/

	if (_IOC_TYPE(cmd) != PVV1_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > PVV1_IOC_MAXNR) return -ENOTTY;

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
        	case PVV1_IOCRESET_RM:
		{
			flagrm = 1;
			wake_up_interruptible(&wqrm);

			retval = 0;
		}
		break;
		case PVV1_IOCRESET_DPU:
		{			
			return -EFAULT;
		}
		break;
		case PVV1_IOCG_IRQ_RM:
		{
			typeIrq=0;

			if(wait_event_interruptible(wqrm, flagrm != 0)>0)
    			{
        			printk(KERN_ERR "MPVV1: PVV1_IOCG_IRQ_RM error ERESTARTSYS");
        			return -ERESTARTSYS;
    			}

    			flagrm = 0;

   			typeIrq = irqN;

			mutex_lock_interruptible(&mutex);
			rez=copy_to_user((int __user *)arg, (char*)&typeIrq, sizeof(typeIrq));
			mutex_unlock(&mutex);

                	if(rez)
                	{
                        	printk(KERN_ERR "MPVV1: PVV1_IOCG_IRQ_RM error copy_to_user witch IOCTL");
                        	return -EFAULT;
                	}

                	retval = 0;

		}
		break;
		case PVV1_IOCG_IRQ_DPU:
		{			
			return -EFAULT;
		}
		break;
		
		case PVV1_IOCG_IRQ_WD: 
			return -ENOTTY;
		break;
		default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}

	return retval;
}

static int pvv_open(struct inode *inode, struct file *pFile)
{

	if(Device_Status>=COUNT_DEVICES) return -EBUSY;

	Device_Status++;

    	try_module_get(THIS_MODULE);//увеличить счётчик ссылок для модуля (возвращается признак успешности операции);

    	printk(KERN_INFO "MPVV1: pvv_open, count %d\n",Device_Status);

    	return SUCCESS;	
}

static int pvv_release(struct inode *inode, struct file *pFile)
{//close

	if(Device_Status>0)
	{
		Device_Status--;

		module_put(THIS_MODULE);//уменьшить счётчик ссылок для модуля;

		printk(KERN_INFO "MPVV1: pvv_release, count %d\n",Device_Status+1);
	}

	return SUCCESS;
}


static ssize_t pvv_read (struct file *pFile, char __user *buffer, size_t length, loff_t *offset)
{//ssize_t - целое,

	int typeIrq;
	typeIrq=0;

	if(wait_event_interruptible(wqrm, flagrm != 0)>0)
    	{
        	printk(KERN_ERR "MPVV1: Error ERESTARTSYS");
        	return -ERESTARTSYS;
    	}

    	flagrm = 0;

	mutex_lock_interruptible(&mutex);
   	typeIrq = irqN;
	mutex_unlock(&mutex);

	return typeIrq;
}

static unsigned int pvv_poll(struct file *pfile, poll_table *wait)
{// для режима инициализации
        unsigned int mask;
        mask = 0;

        poll_wait(pfile, &wqrm, wait);

	mutex_lock_interruptible(&mutex);

	//if (pfile->f_pos != dev->posW)
        mask |= POLLIN | POLLRDNORM; //чтение

        mutex_unlock(&mutex);

        return mask;
}

static struct file_operations fops = {
	.owner	= THIS_MODULE,
	.open = pvv_open,
	.release = pvv_release,
	.read = pvv_read,
	.poll = pvv_poll,
	.unlocked_ioctl = pvv_ioctl,
};


static int pvv_init(void)
{

	int rez; int irq_flags;
	int err; int retp; int irq_pvv1; 
	int res; 
	printk(KERN_INFO "MPVV1: START");

	irq_flags = (IORESOURCE_IRQ | IRQF_TRIGGER_FALLING) & IRQF_TRIGGER_MASK;

	if(Major == 0 && Minor == 0)//==-1
	{
		//выделение номеров устройств динамически
		err = alloc_chrdev_region(&dev_pvv, 0, COUNT_DEVICES, PVV1_CHDEVNAME);
	    	if (err < 0)
    		{
	        	printk(KERN_ALERT "MPVV1: alloc_chrdev_region() error %d\n", err);
        		return FAIL;
		}

		Major = MAJOR(dev_pvv);
		Minor = MINOR(dev_pvv);
	}
	else
        {
                //новый способ регисtрации------------------------------------------------------
                dev_pvv = MKDEV(Major,Minor);
                err = register_chrdev_region(dev_pvv,COUNT_DEVICES,PVV1_CHDEVNAME);// получение номера(ов) символьного устройства

                if(err!=0) 
                {
                        printk(KERN_ERR" Error register_chdev_region for %s code %d\n",PVV1_CHDEVNAME,err);
                        unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
                        return err;
                }

        }


	printk(KERN_INFO  "MPVV1: Major,Minor numbers [%d,%d]",Major,Minor);

	pvv_cdev = &c_dev;

	pvv_cdev->current_irq=0;
    	//инициализация структуры pvv_cdev
    	cdev_init(&pvv_cdev->cdev, &fops);
    	//регистрация структуры
    	if (cdev_add(&pvv_cdev->cdev, dev_pvv, COUNT_DEVICES) == -1)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	return FAIL;
    	}//

//--------------------------------------------------------------------------------------------------------------
	printk(KERN_INFO "------------------------1-----------------------");

	rez = gpio_is_valid(GPIO_PVV1);
	if(rez == -EINVAL) printk(KERN_ERR "not valid pin 27 (PVV1)");
	if(!rez)  printk(KERN_ERR "not valid pin 27 (PVV1)");

	printk(KERN_INFO "------------------------2-----------------------");

    	pvvdev = kzalloc(sizeof(struct pvv_data),GFP_KERNEL);// GFP_KERNEL, означает, что выделение производится от имени процесса, запущенного в пространстве ядра
    	if (!pvvdev)
    	{
        	printk(KERN_ERR "MPVV1: could not allocate device for pvv1");
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
        	return -1;
	}

    	printk(KERN_INFO "------------------------3-----------------------");	

	//in board-tam3517.c

        //omap_mux_init_gpio(GPIO_PVV, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE4);
        //omap_mux_init_gpio(GPIO_CHILD,  OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE4);

	printk(KERN_INFO "------------------------3'-----------------------");

	/*
	 *Note that we assume that pin multiplexing is done in the board-*.c file,
	 * or in the bootloader.
 	*/


	retp = gpio_request(GPIO_PVV1, "irq_pvv1");
    	if (retp < 0)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
	        printk(KERN_INFO "error gpio_request(%d) error = %d",GPIO_PVV1,retp);
        	return -1;
    	}

	retp = gpio_direction_input(GPIO_PVV1);
   	if (retp < 0)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
	        printk(KERN_INFO "error gpio_direction_input(%d) error = %d",GPIO_PVV1,retp);
        	return -1;
    	}

	irq_pvv1 = -1;
	if(retp == 0) {gpio_export(GPIO_PVV1,0); irq_pvv1 = OMAP_GPIO_IRQ(GPIO_PVV1);}

	//irq = gpio_to_irq(GPIO_PVV1);
	if (irq_pvv1 < 0) 
	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
		printk(KERN_INFO ": Unable to get irq: error %d\n", irq_pvv1);
		return -1;
	}

	pvvdev->irq = irq_pvv1;
	printk(KERN_INFO "irq_pvv1: %d",irq_pvv1);
	pvvdev->idrev = pvvdev;
  	pvvdev->ioaddr = 0;

    	strcpy(pvvdev->name,"pvv1");

	enable_irq(pvvdev->irq);
	res = request_irq(pvvdev->irq, pvv_irqhandler,irq_flags|IRQF_SHARED,pvvdev->name, GPIO_PVV1_DEVICE_DESC);
	
	printk(KERN_INFO "------------------------4-----------------------");
	
	if (res!=0)
    	{
        	printk(KERN_ERR "MPVV1: Unable to claim requested irq: %d",  pvvdev->irq);

	        unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
	        cdev_del(&pvv_cdev->cdev);

	        return FAIL;
    	}//*/

	spin_lock_init(&r_spinlock);
	mutex_init(&mutex);

	printk(KERN_INFO "------------------------5-----------------------");
	return SUCCESS;
}

static void pvv_cleanup(void)
{
	gpio_free(GPIO_PVV1);
	free_irq(pvvdev->irq,GPIO_PVV1_DEVICE_DESC);
	kfree(pvvdev);
    	unregister_chrdev_region(dev_pvv,COUNT_DEVICES);
	cdev_del(&pvv_cdev->cdev);

	printk(KERN_INFO "MPVV1: STOP");
}

module_init(pvv_init);
module_exit(pvv_cleanup);


