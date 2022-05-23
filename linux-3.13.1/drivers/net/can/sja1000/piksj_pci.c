#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/can/dev.h>
#include <linux/io.h>
//#include <linux/init.h>

#include "sja1000.h"

#define DRV_NAME  "piksj_pci"

//MODULE_LICENSE("GPL");

MODULE_AUTHOR("Anton Tikhonov <anto-tikhonov@yandex.ru>");
MODULE_DESCRIPTION("Socket-CAN driver for NIITM Pik SJ1000 PCI cards");
MODULE_SUPPORTED_DEVICE("NIITM Pik SJ1000 PCI CAN card");
MODULE_LICENSE("GPL v2");

#define MAX_NO_OF_CHANNELS        2 /* max no of channels on a single card */

struct pik_pci
{
	int channel;
	struct pci_dev *pci_dev;
	struct net_device *slave_dev[MAX_NO_OF_CHANNELS-1];
	void __iomem *conf_addr;
	u16          base_port;
	u16		major;
	int no_channels;
	u8 xilinx_ver;
};

#define PIK_PCI_CAN_CLOCK      (24000000/2)

/*
 * The board configuration is probably following:
 * RX1 is connected to ground.
 * TX1 is not connected.
 * CLKO is not connected.
 * Setting the OCR register to 0xDA is a good idea.
 * This means  normal output mode , push-pull and the correct polarity.
 */
#define PIK_PCI_OCR            0x05;//(OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL)

/*
 * In the CDR register, you should set CBP to 1.
 * You will probably also want to set the clock divider value to 0
 * (meaning divide-by-2), the Pelican bit, and the clock-off bit
 * (you will have no need for CLKOUT anyway).
 */
#define PIK_PCI_CDR            0x23;//(CDR_CBP | CDR_CLKOUT_MASK)

/*
 * These register values are valid for revision 14 of the Xilinx logic.
 */
#define XILINX_VERINT             7   /* Lower nibble simulate interrupts,
					 high nibble version number. */

#define XILINX_PRESUMED_VERSION   14

/*
 * Important S5920 registers
 */
#define PIK_INTCSR              0x01
#define S5920_PTCR                0x60
#define INTCSR_ADDON_INTENABLE_M  0x70

#define PIK_PCI_PORT_BYTES       0x100//0x20

#define PCI_CONFIG_PORT_SIZE      0x80      /* size of the config io-memory */
#define PCI_PORT_SIZE             0x80      /* size of a channel io-memory */
#define PCI_PORT_XILINX_SIZE      0x08      /* size of a xilinx io-memory */
#define DEFAULT_MAJOR			0x0003

#define PIK_PCI_VENDOR_ID1     0x504e    /* the PCI device and vendor IDs */
#define PIK_PCI_DEVICE_ID1     0x1853


//#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
static struct pci_device_id pik_pci_tbl[] = {
//#else
//static DEFINE_PCI_DEVICE_TABLE(pik_pci_tbl) = {
//#endif
	{PIK_PCI_VENDOR_ID1, PIK_PCI_DEVICE_ID1, PCI_ANY_ID, PCI_ANY_ID,},
	{ 0,}
};

MODULE_DEVICE_TABLE(pci, pik_pci_tbl);

static u8 pik_pci_read_reg(const struct sja1000_priv *priv, int port)
{
	return ioread8(priv->reg_base + port);
}

static void pik_pci_write_reg(const struct sja1000_priv *priv,
				 int port, u8 val)
{
	iowrite8(val, priv->reg_base + port);
}

static void pik_pci_disable_irq(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct pik_pci *board = priv->priv;
	u16 intcsr;

	/* Disable interrupts from card */
	intcsr = inw(board->base_port + PIK_INTCSR);
	intcsr &= ~(1<<board->channel);
	outw(intcsr, board->base_port + PIK_INTCSR);
}

static void pik_pci_enable_irq(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct pik_pci *board = priv->priv;
	u16 tmp_en_io;

	/* Enable interrupts from card */
	tmp_en_io = inw(board->base_port + PIK_INTCSR);
	tmp_en_io |= (1<<board->channel);
	outw(tmp_en_io, board->base_port + PIK_INTCSR);
}
static void pik_pci_post_irq(const struct sja1000_priv *priv)
{
	struct pik_pci *board = (struct pik_pci *)priv->priv;
	u16 err_major;

	/* read int flag of error major */
	err_major = inb(board->base_port + board->channel + 0x04);
}

static int number_of_sja1000_chip(void __iomem *base_addr)
{
	u8 status;
	int i;

	for (i = 0; i < MAX_NO_OF_CHANNELS; i++) {
		/* reset chip */
		writeb(MOD_RM, base_addr + 0x0C00 +
			 (i * PIK_PCI_PORT_BYTES) + SJA1000_MOD);
		status = readb(base_addr + 0x0C00 +
				 (i * PIK_PCI_PORT_BYTES) + SJA1000_MOD);
		printk(KERN_INFO "%s status=0x%x\n",DRV_NAME,status);

		/* check reset bit */
		if (!(status & MOD_RM))
			break;
	}

	return i;
}

static void pik_pci_del_chan(struct net_device *dev,int init_step)
{
	struct sja1000_priv *priv;
	struct pik_pci *board;
	int i;

	if (!dev)
		return;
	priv = netdev_priv(dev);
	if(!priv)
		return;
	board = priv->priv;
	if (!board)
		return;

	dev_info(&board->pci_dev->dev, "Removing device %s\n",
		 dev->name);

	/* Disable PCI interrupts */
	pik_pci_disable_irq(dev);

	dev_info(&board->pci_dev->dev, "disable_irq_dev %s\n",dev->name);
	for (i = 0; i < board->no_channels - 1; i++) {
		if (board->slave_dev[i]) {
			dev_info(&board->pci_dev->dev, "Removing device %s\n",
				 board->slave_dev[i]->name);
			unregister_sja1000dev(board->slave_dev[i]);
			free_sja1000dev(board->slave_dev[i]);
		}
	}
	dev_info(&board->pci_dev->dev, "unregistrsja \n");
	if(init_step)
		unregister_sja1000dev(dev);

//	pci_iounmap(board->pci_dev, priv->reg_base);
//	pci_iounmap(board->pci_dev, board->conf_addr);
//	pci_iounmap(board->pci_dev, board->res_addr);

	dev_info(&board->pci_dev->dev, "free sja \n");
	free_sja1000dev(dev);
}

static int pik_pci_add_chan(struct pci_dev *pdev, int channel,
			       struct net_device **master_dev,
			       u16 base_port,
			       void __iomem *base_addr)
{
	struct net_device *dev;
	struct sja1000_priv *priv;
	struct pik_pci *board;
	int err, init_step;

	dev = alloc_sja1000dev(sizeof(struct pik_pci));
	if (dev == NULL)
		return -ENOMEM;

	priv = netdev_priv(dev);
	board = priv->priv;

	board->pci_dev = pdev;
	board->channel = channel;
	board->base_port = base_port;
	board->conf_addr = base_addr;
	board->major = DEFAULT_MAJOR;

	if (channel == 0) {
		init_step = 2;
		/* Enable interrupts from card */
		pik_pci_enable_irq(dev);
	} else {
		struct sja1000_priv *master_priv = netdev_priv(*master_dev);
		struct pik_pci *master_board = master_priv->priv;
		master_board->slave_dev[channel - 1] = dev;
		master_board->no_channels = channel + 1;
		board->xilinx_ver = master_board->xilinx_ver;
		pik_pci_enable_irq(dev);
	}

	priv->reg_base = base_addr + 0x0C00 + channel * PIK_PCI_PORT_BYTES;

	priv->read_reg = pik_pci_read_reg;
	priv->write_reg = pik_pci_write_reg;
	priv->post_irq = pik_pci_post_irq;

	priv->can.clock.freq = PIK_PCI_CAN_CLOCK;

	priv->ocr = PIK_PCI_OCR;
	priv->cdr = PIK_PCI_CDR;
	
	outw(board->major,board->base_port + 0x2+channel);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
	priv->irq_flags = SA_SHIRQ;
#else
	priv->irq_flags = IRQF_SHARED;
#endif
	dev->irq = pdev->irq;

	init_step = 4;

	dev_info(&pdev->dev, "reg_base=%p conf_addr=%p irq=%d\n",
		 priv->reg_base, board->conf_addr, dev->irq);

	SET_NETDEV_DEV(dev, &pdev->dev);

	/* Register SJA1000 device */
	err = register_sja1000dev(dev);
	if (err) {
		dev_err(&pdev->dev, "Registering device failed (err=%d)\n",
			err);
		goto failure;
	}

	if (channel == 0)
		*master_dev = dev;

	return 0;

failure:
	pik_pci_del_chan(dev,channel);
	return err;
}


static int pik_pci_init_one(struct pci_dev *pdev,
					 const struct pci_device_id *ent)
{
	int err;
	struct net_device *master_dev = NULL;
	struct sja1000_priv *priv;
	struct pik_pci *board;
	int no_channels;
	void __iomem *base_addr = NULL;
	void __iomem *conf_addr = NULL;
	u16 base_port;

	int i;

	dev_info(&pdev->dev, "initializing device %04x:%04x\n",
		 pdev->vendor, pdev->device);

	err = pci_enable_device(pdev);
	if (err)
		goto failure;

	err = pci_request_regions(pdev, DRV_NAME);
	if (err)
		goto failure_release_pci;

	base_port = pci_resource_start(pdev, 0);
	printk(KERN_INFO "%s base_port = 0x%x\n", DRV_NAME,base_port);
	if (base_port == 0) {
		err = -ENODEV;
		goto failure_release_regions;
	}
	/* XILINX board wide address */
	base_addr = pci_iomap(pdev, 1,525312);
	//printk(KERN_INFO "%s base_addr=0x%x\n",DRV_NAME,(u32)base_addr);
	printk(KERN_INFO "%s base_addr=0x%p\n",DRV_NAME,base_addr);
	if (base_addr == 0) {
		err = -ENOMEM;
		goto failure_iounmap;
	}


	printk(KERN_INFO "%s base_addr+c(0x%x)=0x%x\n",DRV_NAME,base_port + 0x0000000C,(u16)inw(base_port + 0x0c));
	outw(0xffff,base_port + 0x0C);
	printk(KERN_INFO "%s base_addr+c(0x%x)=0x%x\n",DRV_NAME,base_port + 0x0000000c,(u16)inw(base_port + 0x0c));

	conf_addr = base_addr;
	no_channels = number_of_sja1000_chip(base_addr);
	if (no_channels == 0) {
		err = -ENOMEM;
		goto failure_iounmap;
	}

	for (i = 0; i < no_channels; i++) {
		err = pik_pci_add_chan(pdev, i, &master_dev,
					  base_port,base_addr);
		if (err)
			goto failure_cleanup;
	}

	priv = netdev_priv(master_dev);
	board = priv->priv;

	dev_info(&pdev->dev, "xilinx version=%d number of channels=%d\n",
		 board->xilinx_ver, board->no_channels);


	pci_set_drvdata(pdev, master_dev);
	return 0;

failure_cleanup:
	if(master_dev)
	pik_pci_del_chan(master_dev,1);

failure_iounmap:
	if (base_addr != NULL)
		pci_iounmap(pdev, base_addr);

failure_release_regions:
	pci_release_regions(pdev);

failure_release_pci:
	pci_disable_device(pdev);

failure:
	return err;

}

static void pik_pci_remove_one(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);

	pik_pci_del_chan(dev,1);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_driver pik_pci_driver =
{
	.name = DRV_NAME,
	.id_table = pik_pci_tbl,
	.probe = pik_pci_init_one,
	.remove = pik_pci_remove_one,
};

module_pci_driver(pik_pci_driver);
