
#ifndef __MOD_PVV_H__
#define __MOD_PVV_H__

struct pvv_data
{
    unsigned int idrev;
    unsigned int irq;
    void __iomem *ioaddr;
    char   name[20];

//    struct platform_device  *pdev;
    /* spinlock to ensure register accesses are serialised */
    spinlock_t dev_lock;

};

#define PVV_USE_32BIT 2
#define PVV_USE_16BIT 1
#define PVV_USE_8BIT  0

unsigned char flagsPvv = PVV_USE_8BIT;

static inline u32 __pvv_reg_read(void __iomem *base, u32 reg)
{

    if (flagsPvv == PVV_USE_32BIT)
        return readl(base + reg);

    if (flagsPvv == PVV_USE_16BIT)
         return (readw(base  + reg) & 0xFFFF);

     if (flagsPvv == PVV_USE_8BIT)
         return (readb(base  + reg)&0xFF);

    return 0;
}

static inline void __pvv_reg_write(void __iomem *base, u32 reg,u32 val)
{

    if (flagsPvv == PVV_USE_32BIT)
    {
        writel(val, base + reg);
        return;
    }

    if (flagsPvv == PVV_USE_16BIT)
    {
        writew(val & 0xFFFF, base + reg);
        return;
    }

    if (flagsPvv == PVV_USE_8BIT)
    {
        writeb(val & 0xFF, base + reg);
        return;
    }

}

static inline u32 pvv_reg_read(struct pvv_data *pdata, u32 reg)
{
    u32 data;
    unsigned long flags;

    spin_lock_irqsave(&pdata->dev_lock, flags);
    data = __pvv_reg_read(pdata->ioaddr, reg);
    spin_unlock_irqrestore(&pdata->dev_lock, flags);

    return data;
}

static inline void pvv_reg_write(struct pvv_data *pdata, u32 reg, u32 val)
{
    unsigned long flags;

    spin_lock_irqsave(&pdata->dev_lock, flags);
    __pvv_reg_write(pdata->ioaddr, reg, val);
    spin_unlock_irqrestore(&pdata->dev_lock, flags);
}

#endif
