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
#include "ioctl_pvv.h"

#define PVV_DRV_VERSION	"2016-01-21"
#define PVV_DEVNAME "pvv"
#define PVV_CHDEVNAME "chpvv"
#define GPIO_PVV_DEVICE_DESC    "pvv_device"

#define  COUNT_DEVICES 4 //pvv1: dpu, rm & pvv2: dpu, rm // верно

MODULE_LICENSE("GPL");
MODULE_VERSION(PVV_DRV_VERSION);
MODULE_ALIAS("chdriver:pvv");

static int SUCCESS;
static int FAIL = -1;
static int Major;
static int Minor;
static int Device_Status = 0;

static dev_t dev_pvv;

// для внутренного представления символьных устройств:
//static struct cdev c_dev; 
struct cpvv_dev
{//for private_date => irq и wq ?
        int current_irq;
	wait_queue_head_t wq;
	struct mutex mutex;
        struct cdev cdev;
};

struct cpvv_dev c_dev;
struct cpvv_dev *pvv_cdev;

struct pvv_data *pvvbdevParent;
struct pvv_data *pvvbdevChild;

//parent
static DECLARE_WAIT_QUEUE_HEAD(wq);
static int flag = 0;

//child
static DECLARE_WAIT_QUEUE_HEAD(wqc);
static int flagc = 0;

unsigned char errorPvv;

//spinlock_t r_spinlock;// = SPIN_LOCK_UNLOCKED;
//spinlock_t w_spinlock;// = SPIN_LOCK_UNLOCKED;

#define IRQ_PARENT 1
#define IRQ_CHILD  2

#define GPIO_PARENT  0
#define GPIO_CHILD   136

int irqN;
int irqNc;

void wakeUpUserMode(unsigned long par)
{

	//irqN = par;

    	if(par == IRQ_PARENT)
	{
	    	flag = 1;
		wake_up_interruptible(&wq);// пробудить поток чтения
	}

	if(par == IRQ_CHILD)
	{
	    	flagc = 1;
		wake_up_interruptible(&wqc);// пробудить поток чтения
	}
}

static irqreturn_t pvv_irqhandler_parent(int irq, void *dev_id)
{
	int id;
	id = *((int*)dev_id);

	irqN  = irq;
	wakeUpUserMode(IRQ_PARENT);

	//printk(KERN_INFO "MPVV: IRQ_PARENT");

	return IRQ_HANDLED;
}

static irqreturn_t pvv_irqhandler_child(int irq, void *dev_id)
{
       	int id;
	id = *((int*)dev_id);
	irqNc = irq;
	wakeUpUserMode(IRQ_CHILD);

	//printk(KERN_INFO "MPVV: IRQ_CHILD");

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

	if (_IOC_TYPE(cmd) != PVV_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > PVV_IOC_MAXNR) return -ENOTTY;

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
        	case PVV_IOCRESET:
		{
			flag = 1;
			wake_up_interruptible(&wq);
			flagc = 1;
			wake_up_interruptible(&wqc);

			retval = 0;
		}
		break;
		case PVV_IOCRESET_PARENT:
		{
			flag = 1;
			wake_up_interruptible(&wq);	

			retval = 0;
		}
		break;
		case PVV_IOCRESET_CHILD:
		{
			flagc = 1;
			wake_up_interruptible(&wqc);

			retval = 0;
		}
		break;
		case PVV_IOCG_IRQ_PARENT:// если использовать несколько очередей wq_parent & wq_child
		{
			typeIrq=0;

			if(wait_event_interruptible(wq, flag != 0)>0)
    			{
        			printk(KERN_ERR "MPVV PVV_IOCG_IRQ_PARENT: error ERESTARTSYS");
        			return -ERESTARTSYS;
    			}

    			flag = 0;

   			typeIrq = irqN;
			
			rez=copy_to_user((int __user *)arg, (char*)&typeIrq, sizeof(typeIrq));

                	if(rez)
                	{
                        	printk(KERN_ERR "MPVV PVV_IOCG_IRQ_PARENT: error copy_to_user witch IOCTL");
                        	return -EFAULT;
                	}

                	retval = 0;


		}	
		break;
		case PVV_IOCG_IRQ_CHILD:
		{
			typeIrq=0;

			if(wait_event_interruptible(wqc, flagc != 0)>0)
    			{
        			printk(KERN_ERR "MPVV PVV_IOCG_IRQ_CHILD: error ERESTARTSYS");
        			return -ERESTARTSYS;
    			}

    			flagc = 0;

   			typeIrq = irqNc;// тут должен быть номер какой выставлила система

			rez=copy_to_user((int __user *)arg, (char*)&typeIrq, sizeof(typeIrq));

                	if(rez)
                	{
                        	printk(KERN_ERR "MPVV PVV_IOCG_IRQ_CHILD: error copy_to_user witch IOCTL");
                        	return -EFAULT;
                	}

                	retval = 0;
		}
		break;
		case PVV_IOCG_IRQ_CHILD_WD: 
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

    	printk(KERN_INFO "MPVV: pvv_open, count %d\n",Device_Status);

    	return SUCCESS;	
}

static int pvv_release(struct inode *inode, struct file *pFile)
{//close

	if(Device_Status>0)
	{
		Device_Status--;

		module_put(THIS_MODULE);//уменьшить счётчик ссылок для модуля;

		printk(KERN_INFO "MPVV: pvv_release, count %d\n",Device_Status+1);
	}

	return SUCCESS;
}


static ssize_t pvv_read (struct file *pFile, char __user *buffer, size_t length, loff_t *offset)
{//ssize_t - целое,
 //only parent
	int typeIrq;
	typeIrq=0;

	if(wait_event_interruptible(wq, flag != 0)>0)
    	{
        	printk(KERN_ERR "MPVV: Error ERESTARTSYS");
        	return -ERESTARTSYS;
    	}

    	flag = 0;

   	typeIrq = irqN;

	//printk(KERN_INFO "MPVV: read %d",typeIrq);

   	//irqN=0;

	return typeIrq;
}

static unsigned int pvv_poll(struct file *pfile, poll_table *wait)
{// для режима инициализации, пока только родитель 
        unsigned int mask;
//      struct pvv_dev *dev; //для нескольких устр PVV1 и PVV2

//      dev = pfile->private_data;  //для нескольких устр PVV1 и PVV2
        mask = 0;

        poll_wait(pfile, &wq, wait);

 //      mutex_lock_interruptible(&dev->mutex);

 //       if (pfile->f_pos != dev->posW)
        mask |= POLLIN | POLLRDNORM; //чтение 

      //  mutex_unlock(&dev->mutex);
 
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
	//spin_lock_init(&r_spinlock);
	//spin_lock_init(&w_spinlock);
	int rez; int irq_flags;
	int err; int retp; int irqp; 
	int res; int retc; int irqc;
	printk(KERN_INFO "MPVV: START");

	irq_flags = (IORESOURCE_IRQ | IRQF_TRIGGER_FALLING) & IRQF_TRIGGER_MASK;
	//выделение номеров устройств динамически
	err = alloc_chrdev_region(&dev_pvv, 0, COUNT_DEVICES, PVV_CHDEVNAME);
    	if (err < 0)
    	{
        	printk(KERN_ALERT "MPVV: alloc_chrdev_region() error %d\n", err);
        	return FAIL;
	}

	Major = MAJOR(dev_pvv);
	Minor = MINOR(dev_pvv);
	printk(KERN_INFO  "MPVV: Major,Minor numbers [%d,%d]",Major,Minor);

	pvv_cdev = &c_dev;

	pvv_cdev->current_irq=0;
    	//инициализация структуры pvv_cdev
    	cdev_init(&pvv_cdev->cdev, &fops);
    	//регистрация структуры
    	if (cdev_add(&pvv_cdev->cdev, dev_pvv, COUNT_DEVICES) == -1)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	return FAIL;
    	}//*/

//--------------------------------------------------------------------------------------------------------------
	printk(KERN_INFO "------------------------1-----------------------");
	rez = gpio_is_valid(GPIO_CHILD);
	if(rez == -EINVAL) printk(KERN_ERR "not valid pin 76 (CHILD)");
	if(!rez)  printk(KERN_ERR "not valid pin 76 (CHILD)");

	rez = gpio_is_valid(GPIO_PARENT);
	if(rez == -EINVAL) printk(KERN_ERR "not valid pin 27 (PARENT)");
	if(!rez)  printk(KERN_ERR "not valid pin 27 (PARENT)");

	printk(KERN_INFO "------------------------2-----------------------");

    	pvvbdevParent = kzalloc(sizeof(struct pvv_data),GFP_KERNEL);// GFP_KERNEL, означает, что выделение производится от имени процесса, запущенного в пространстве ядра
    	if (!pvvbdevParent)
    	{
        	printk(KERN_ERR "MPVV: could not allocate device for parent");
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
        	return -1;
	}

    	pvvbdevChild = kzalloc(sizeof(struct pvv_data),GFP_KERNEL);
    	if (!pvvbdevChild)
    	{
        	printk(KERN_ERR "MPVV: could not allocate device for child");
	        unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
	        return -1;
    	}
   
	printk(KERN_INFO "------------------------3-----------------------");	

	//in board-tam3517.c

        //omap_mux_init_gpio(GPIO_PARENT, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE4);
        //omap_mux_init_gpio(GPIO_CHILD,  OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE4);

	printk(KERN_INFO "------------------------3'-----------------------");

	retp = gpio_request(GPIO_PARENT, "pvvirqp");
    	if (retp < 0)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
	        printk(KERN_INFO "error gpio_request(%d) error = %d",GPIO_PARENT,retp);
        	return -1;
    	}

	retp = gpio_direction_input(GPIO_PARENT);
   	if (retp < 0)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
	        printk(KERN_INFO "error gpio_direction_input(%d) error = %d",GPIO_PARENT,retp);
        	return -1;
    	}

	irqp = -1;
	if(retp == 0) {gpio_export(GPIO_PARENT,0); irqp = OMAP_GPIO_IRQ(GPIO_PARENT);}

	//irq = gpio_to_irq(GPIO_PARENT);
	if (irqp < 0) 
	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
		printk(KERN_INFO ": Unable to get irq: error %d\n", irqp);
		return -1;
	}

	pvvbdevParent->irq = irqp;
	printk(KERN_INFO "irqp: %d",irqp);
	pvvbdevParent->idrev = pvvbdevParent;
  	pvvbdevParent->ioaddr = 0;

    	strcpy(pvvbdevParent->name,"pvv-parent");

	enable_irq(pvvbdevParent->irq);
	res = request_irq(pvvbdevParent->irq, pvv_irqhandler_parent,irq_flags|IRQF_SHARED,pvvbdevParent->name, GPIO_PVV_DEVICE_DESC);
	
	printk(KERN_INFO "------------------------4-----------------------");

	retc = gpio_request(GPIO_CHILD, "pvvirqc");
	if (retc < 0)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
        	printk(KERN_INFO "error gpio_request(%d) error = %d",GPIO_CHILD,retc);
        	return -1;
    	}

	retc = gpio_direction_input(GPIO_CHILD);
	if (retc < 0)
    	{
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
        	printk(KERN_INFO "error gpio_direction_input(%d) error = %d",GPIO_CHILD,retc);
        	return -1;
    	}

	irqc = -1;
	if(retc == 0) {gpio_export(GPIO_CHILD,0); irqc = OMAP_GPIO_IRQ(GPIO_CHILD);}

	//irq = gpio_to_irq(GPIO_PARENT);
	if (irqc < 0) 
	{
		printk(KERN_INFO ": Unable to get irqc: error %d\n", irqc);
        	unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
        	cdev_del(&pvv_cdev->cdev);
		return -1;
	}

    	pvvbdevChild->irq = irqc;
	printk(KERN_INFO "irqc: %d",irqc);
    	pvvbdevChild->idrev = pvvbdevChild;
	pvvbdevChild->ioaddr = 0;
	strcpy(pvvbdevChild->name,"pvv-child");

	enable_irq(pvvbdevChild->irq);
	res = request_irq(pvvbdevChild->irq, pvv_irqhandler_child,irq_flags|IRQF_SHARED,pvvbdevChild->name, GPIO_PVV_DEVICE_DESC);

	printk(KERN_INFO "------------------------5-----------------------");


	if (res!=0)
    	{
        	printk(KERN_ERR "MPVV: Unable to claim requested irq: %d %d",  pvvbdevParent->irq, pvvbdevChild->irq);

	        unregister_chrdev_region(dev_pvv, COUNT_DEVICES);
	        cdev_del(&pvv_cdev->cdev);

	        return FAIL;
    	}//*/

	printk(KERN_INFO "------------------------6-----------------------");
	return SUCCESS;
}

static void pvv_cleanup(void)
{
	gpio_free(GPIO_PARENT);
	gpio_free(GPIO_CHILD);

	free_irq(pvvbdevParent->irq,GPIO_PVV_DEVICE_DESC);
	free_irq(pvvbdevChild->irq, GPIO_PVV_DEVICE_DESC);

	kfree(pvvbdevParent);
	kfree(pvvbdevChild);
   
    	unregister_chrdev_region(dev_pvv,COUNT_DEVICES);
	cdev_del(&pvv_cdev->cdev);

	printk(KERN_INFO "MPVV: STOP");
}

module_init(pvv_init);
module_exit(pvv_cleanup);


