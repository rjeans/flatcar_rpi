
// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/acpi.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include "mailbox-bcm2835-acpi.h"

#define MAIL1_STA   0x18
#define MAIL1_WRT   0x20
#define MAIL0_RD    0x00
#define MAIL0_STA   0x18

#define ARM_MS_FULL  0x80000000
#define ARM_MS_EMPTY 0x40000000

struct bcm2835_mbox {
    void __iomem *regs;
    struct mbox_controller controller;
    struct device *dev;
    struct mbox_chan chan;
    struct completion tx_complete;
    spinlock_t lock;
};





static irqreturn_t bcm2835_mbox_irq(int irq, void *dev_id)
{
    struct bcm2835_mbox *mbox = dev_id;
    u32 value;

    pr_info(">>> IRQ handler entered: dev_id=%px\n", dev_id);

    if (!mbox || !mbox->regs) {
        pr_err("IRQ: NULL mbox or regs\n");
        return IRQ_NONE;
    }

    value = readl(mbox->regs + MAIL0_RD);
    dev_info(mbox->dev, "IRQ received: MAIL0_RD = 0x%08x\n", value);

    complete(&mbox->tx_complete);
    dev_info(mbox->dev, "Completion signaled for mailbox transaction\n");
    return IRQ_HANDLED;
}

static int bcm2835_send_data(struct mbox_chan *chan, void *data)
{
    struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);
    if (!mbox || !mbox->regs) {
        pr_err("send_data: NULL mbox or regs\n");
        return -ENODEV;
    }
    if (!data) {
        pr_err("send_data: NULL data\n");
        return -EINVAL;
    }

    pr_info(">>> bcm2835_send_data called, chan=%p data=%p\n", chan, data);


    reinit_completion(&mbox->tx_complete);
 
  

    spin_lock(&mbox->lock);
    while (readl(mbox->regs + MAIL1_STA) & ARM_MS_FULL)
        cpu_relax();


    dma_addr_t dma = (dma_addr_t)data;
    pr_info("Mailbox message DMA address: 0x%llx (before write)\n", dma | 8);



    u32 msg_addr = dma | 8;  // Channel 8 = property channel
    writel(msg_addr, mbox->regs + MAIL1_WRT);
    spin_unlock(&mbox->lock);

    dev_info(mbox->dev, "Sent message to mailbox: 0x%08x\n", msg_addr);

    return 0;
}

static bool bcm2835_last_tx_done(struct mbox_chan *chan)
{
    struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);

    /* Assuming IRQ signals completion */
    return completion_done(&mbox->tx_complete);
}

static int bcm2835_startup(struct mbox_chan *chan)
{
    struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);
   /* You could init per-channel state here if needed */
    dev_info(mbox->dev, "Mailbox channel startup\n");
    return 0;
}

static void bcm2835_shutdown(struct mbox_chan *chan)
{
    struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);

    dev_info(mbox->dev, "Mailbox channel shutdown\n");
    /* Clean up if needed */
}

static const struct mbox_chan_ops bcm2835_chan_ops = {
    .send_data     = bcm2835_send_data,
    .startup       = bcm2835_startup,
    .shutdown      = bcm2835_shutdown,
    .last_tx_done  = bcm2835_last_tx_done,
};








static int bcm2835_mbox_probe(struct platform_device *pdev)
{
    struct bcm2835_mbox *mbox;
    struct resource *res;
    int irq, ret;

    mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
    if (!mbox)
        return -ENOMEM;

    platform_set_drvdata(pdev, mbox);
    mbox->dev = &pdev->dev;

    


    spin_lock_init(&mbox->lock);
    init_completion(&mbox->tx_complete);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    mbox->regs = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(mbox->regs))
        return PTR_ERR(mbox->regs);

    irq = platform_get_irq(pdev, 0);
    if (irq < 0)
        return irq;

    ret = devm_request_irq(&pdev->dev, irq, bcm2835_mbox_irq, 0, dev_name(&pdev->dev), mbox);
    if (ret)
        return ret;

    mbox->controller.dev = &pdev->dev;
    mbox->controller.chans = &mbox->chan;
    mbox->controller.num_chans = 1;
    mbox->controller.ops = &bcm2835_chan_ops;
    mbox->chan.name = "property";  // name used in ACPI _DSD


    mbox->chan.mbox = &mbox->controller;  

    if (!mbox->chan.mbox)
	dev_err(&pdev->dev, "mbox->chan.mbox is NULL before registration!\n");
      else
	dev_info(&pdev->dev, "mbox->chan.mbox is OK\n");


    ret = devm_mbox_controller_register(&pdev->dev, &mbox->controller);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register mailbox controller: %d\n", ret);
        return ret;
    }
 
    pr_info("Mailbox controller registered: chans=%d\n", mbox->controller.num_chans);
    if (!mbox->chan.mbox)
	   pr_err("ERROR: mbox->chan.mbox is NULL!\n");

    

    dev_info(&pdev->dev, "BCM2835 ACPI mailbox controller initialized successfully\n");
    return 0;
}

static const struct acpi_device_id bcm2835_mbox_acpi_ids[] = {
    { "BCM2849", 0 },
    { }
};
MODULE_DEVICE_TABLE(acpi, bcm2835_mbox_acpi_ids);

static struct platform_driver bcm2835_mbox_driver = {
    .driver = {
        .name = "bcm2835-mbox-acpi",
        .acpi_match_table = bcm2835_mbox_acpi_ids,
    },
    .probe = bcm2835_mbox_probe,
};

module_platform_driver(bcm2835_mbox_driver);

MODULE_AUTHOR("Richard Jeans");
MODULE_DESCRIPTION("BCM2835 ACPI mailbox controller");
MODULE_LICENSE("GPL");
