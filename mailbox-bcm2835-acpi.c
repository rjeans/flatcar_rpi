
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

#define BCM2835_MAX_CHANNELS     16

struct bcm2835_mbox {
    void __iomem *regs;
    struct mbox_controller controller;
    struct device *dev;
    struct mbox_chan chans[BCM2835_MAX_CHANNELS];
    struct completion tx_completions[BCM2835_MAX_CHANNELS];
    int irq;
    spinlock_t lock;
};


struct bcm2835_mbox *bcm2835_mbox_global;



struct mbox_chan *bcm2835_mbox_request_channel(struct mbox_client *cl)
{
    struct mbox_chan *chan;
    int i, ret;

    if (!cl || !bcm2835_mbox_global)
        return ERR_PTR(-ENODEV);

    for (i = 0; i < bcm2835_mbox_global->controller.num_chans; i++) {
        chan = &bcm2835_mbox_global->chans[i];

        if (!chan->cl) {
            ret = mbox_bind_client(chan, cl);
            if (ret) {
                pr_err("Failed to bind mailbox client: %d\n", ret);
                return ERR_PTR(ret);
            }

            init_completion(&bcm2835_mbox_global->tx_completions[i]);
            chan->mbox = &bcm2835_mbox_global->controller;
            return chan;
        }
    }

    return ERR_PTR(-EBUSY);  // No free channel
}
EXPORT_SYMBOL_GPL(bcm2835_mbox_request_channel);

int bcm2835_mbox_free_channel(struct mbox_chan *chan)
{
	if (!chan)
		return -EINVAL;

	if (!chan->cl)
		return -ENODEV;  // Already unbound or never bound

	chan->cl = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(bcm2835_mbox_free_channel);

static int bcm2835_send_data(struct mbox_chan *chan, void *data)
{
	struct bcm2835_mbox *mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);
	u32 msg = *(u32 *)data;

 


    dev_info(mbox->controller.dev, "SEND DATA: chan->cl=%px\n", chan->cl);

 

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

           /* Enable the interrupt on data reception */
    writel(ARM_MC_IHAVEDATAIRQEN, mbox->regs + MAIL0_CNF);


    dev_info(mbox->dev, "Mailbox channel startup\n");
    return 0;
}

static void bcm2835_shutdown(struct mbox_chan *chan)
{
    struct bcm2835_mbox *mbox;

 
    mbox = container_of(chan->mbox, struct bcm2835_mbox, controller);

 
 
    dev_info(mbox->dev, "Mailbox channel shutdown\n");
}



static irqreturn_t bcm2835_mbox_irq(int irq, void *dev_id)
{
    struct bcm2835_mbox *mbox = dev_id;
    struct device *dev = mbox->controller.dev;
    irqreturn_t handled = IRQ_NONE;

    dev_info(dev, "Mailbox IRQ triggered: irq=%d\n", irq);

    while (!(readl(mbox->regs + MAIL0_STA) & ARM_MS_EMPTY)) {
        u32 msg = readl(mbox->regs + MAIL0_RD);
        u32 chan_index = msg & 0xf;
        u32 payload = msg & ~0xf;

        if (chan_index >= BCM2835_MAX_CHANNELS) {
            dev_warn(dev, "Invalid channel index %u in IRQ msg 0x%08X\n", chan_index, msg);
            continue;
        }

        struct mbox_chan *chan = &mbox->chans[chan_index];
        if (!chan->cl || !chan->cl->rx_callback) {
            dev_warn(dev, "Unbound mailbox channel %u (msg=0x%08X), skipping\n", chan_index, msg);
            continue;
        }

        dev_dbg(dev, "Dispatching msg 0x%08X to channel %u\n", payload, chan_index);
        mbox_chan_received_data(chan, &msg);
        handled = IRQ_HANDLED;
    }

    return handled;
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
    bcm2835_mbox_global = mbox;

    


    mbox->irq  = platform_get_irq(pdev, 0);
    if (mbox->irq  < 0)
        return dev_err_probe(&pdev->dev, mbox->irq, "Failed to get IRQ\n");

    ret = devm_request_irq(&pdev->dev, mbox->irq , bcm2835_mbox_irq,
                        0, dev_name(&pdev->dev), mbox);
    if (ret) {
        return dev_err_probe(&pdev->dev, ret, "Failed to request IRQ\n");
    }

    



    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    mbox->regs = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(mbox->regs))
        return PTR_ERR(mbox->regs);




    dev_info(&pdev->dev, "Mailbox registers mapped at %p\n", mbox->regs);
    mbox->controller.dev = &pdev->dev;
    mbox->controller.chans = mbox->chans;
    mbox->controller.num_chans = BCM2835_MAX_CHANNELS;
    mbox->controller.ops = &bcm2835_chan_ops;
    mbox->controller.txdone_poll = true;
    mbox->controller.txpoll_period = 5;


    

   







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
