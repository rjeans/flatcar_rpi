
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



/* Mailboxes */
#define ARM_0_MAIL0	0x00
#define ARM_0_MAIL1	0x20

/*
 * Mailbox registers. We basically only support mailbox 0 & 1. We
 * deliver to the VC in mailbox 1, it delivers to us in mailbox 0. See
 * BCM2835-ARM-Peripherals.pdf section 1.3 for an explanation about
 * the placement of memory barriers.
 */
#define MAIL0_RD	(ARM_0_MAIL0 + 0x00)
#define MAIL0_POL	(ARM_0_MAIL0 + 0x10)
#define MAIL0_STA	(ARM_0_MAIL0 + 0x18)
#define MAIL0_CNF	(ARM_0_MAIL0 + 0x1C)
#define MAIL1_WRT	(ARM_0_MAIL1 + 0x00)
#define MAIL1_STA	(ARM_0_MAIL1 + 0x18)


#define PROPERTY_CHANNEL_IRQ (1 << 8)

#define ARM_MS_FULL  0x80000000
#define ARM_MS_EMPTY 0x40000000

/* Configuration register: Enable interrupts. */
#define ARM_MC_IHAVEDATAIRQEN	BIT(0)


struct bcm2835_mbox {
    void __iomem *regs;
    struct mbox_controller controller;
    struct device *dev;
    struct mbox_chan chan;
    struct completion tx_complete;
    int irq;
    spinlock_t lock;
};

struct mbox_chan *global_rpi_mbox_chan;
EXPORT_SYMBOL_GPL(global_rpi_mbox_chan);




static irqreturn_t bcm2835_mbox_irq(int irq, void *dev_id)
{
 
    pr_info(">>> IRQ handler entered: dev_id=%px\n", dev_id);
 	struct bcm2835_mbox *mbox = dev_id;
	struct device *dev = mbox->controller.dev;
	struct mbox_chan *link = &mbox->controller.chans[0];

     dev_info(mbox->controller.dev, "MBOX IRQ: IRQ: link->cl=%px\n", link->cl);


	while (!(readl(mbox->regs + MAIL0_STA) & ARM_MS_EMPTY)) {
		u32 msg = readl(mbox->regs + MAIL0_RD);
        pr_info(">>> IRQ: Received message 0x%08X on mailbox\n", msg);
        dev_info(dev, "Reply 0x%08X\n", msg);
 	    mbox_chan_received_data(link, &msg);       
	
    }
    dev_info(mbox->dev, "Completion signaled for mailbox transaction\n");
    // Clear any sticky IRQ status — write 0 first
    writel(0, mbox->regs + MAIL0_CNF);
    writel(ARM_MC_IHAVEDATAIRQEN, mbox->regs + MAIL0_CNF);
    dev_info(mbox->dev, "MAIL0_CNF (IRQ enable register) = 0x%08X\n", readl(mbox->regs + MAIL0_CNF));
  

    return IRQ_HANDLED;
}

static int bcm2835_send_data(struct mbox_chan *chan, void *data)
{
	struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);
	u32 msg = *(u32 *)data;

 


    dev_info(mbox->controller.dev, "SEND DATA: IRQ: chan->cl=%px\n", chan->cl);

    u32 irq_enable = readl(mbox->regs + MAIL0_CNF);
    dev_info(mbox->dev, "MAIL0_CNF (IRQ enable register) = 0x%08X\n", irq_enable);


	spin_lock(&mbox->lock);
	writel(msg, mbox->regs + MAIL1_WRT);
	dev_info(mbox->controller.dev, "Request 0x%08X\n", msg);
	spin_unlock(&mbox->lock);
	

    dev_info(mbox->dev, "Mailbox message sent successfully\n");

    return 0;
}

static bool bcm2835_last_tx_done(struct mbox_chan *chan)
{
    struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);
	bool ret;
    dev_info(mbox->dev, "Checking if last transmission is done\n");

	spin_lock(&mbox->lock);
	ret = !(readl(mbox->regs + MAIL1_STA) & ARM_MS_FULL);
	spin_unlock(&mbox->lock);
    dev_info(mbox->dev, "Last transmission done: %s\n", ret ? "Yes" : "No");
    if (ret) {
        dev_info(mbox->dev, "Last transmission completed successfully\n");
    } else {
        dev_info(mbox->dev, "Last transmission not completed yet\n");
    }
	return ret;
}

static int bcm2835_startup(struct mbox_chan *chan)
{
    struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);
   /* You could init per-channel state here if needed */
	/* Enable the interrupt on data reception */
	writel(ARM_MC_IHAVEDATAIRQEN, mbox->regs + MAIL0_CNF);

    dev_info(mbox->dev, "Mailbox channel startup\n");
    return 0;
}

static void bcm2835_shutdown(struct mbox_chan *chan)
{
    struct bcm2835_mbox *mbox;

    if (!chan || !chan->mbox)
        return;

    mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);

    if (!mbox || !mbox->regs) {
        pr_warn("bcm2835_shutdown called with invalid context\n");
        return;
    }

    writel(0, mbox->regs + MAIL0_CNF);
    synchronize_irq(mbox->irq);   
    dev_info(mbox->dev, "Mailbox channel shutdown\n");
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
    int ret;
    

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

    mbox->irq = platform_get_irq(pdev, 0);
    if (mbox->irq < 0)
        return mbox->irq;


    dev_info(&pdev->dev, "Requesting IRQ %d for mailbox\n", mbox->irq);
    dev_info(&pdev->dev, "Mailbox registers mapped at %p\n", mbox->regs);
    mbox->controller.dev = &pdev->dev;
    mbox->controller.chans = &mbox->chan;
    mbox->controller.num_chans = 1;
    mbox->controller.ops = &bcm2835_chan_ops;
    mbox->controller.txdone_irq = false;
    mbox->controller.txdone_poll = true;
    mbox->controller.txpoll_period = 1;

//    ret = devm_request_irq(&pdev->dev, mbox->irq, bcm2835_mbox_irq, IRQF_NO_SUSPEND, dev_name(&pdev->dev), mbox);
//    if (ret)
//        return ret;

    

   
   

    global_rpi_mbox_chan = &mbox->chan;
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
