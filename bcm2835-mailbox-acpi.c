// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/property.h>
#include <linux/mailbox_client.h>


#define MAIL0_RD    0x00
#define MAIL0_STA   0x18
#define MAIL0_CNF   0x1C
#define MAIL1_WRT   0x00
#define MAIL1_STA   0x18

#define ARM_MS_FULL     BIT(31)
#define ARM_MS_EMPTY    BIT(30)
#define ARM_MC_IHAVEDATAIRQEN BIT(0)

struct mbox_chan *rpi_mbox_chan0;
EXPORT_SYMBOL_GPL(rpi_mbox_chan0);


struct bcm2835_mbox {
	void __iomem *regs;
	spinlock_t lock;
	struct mbox_controller controller;
	struct mbox_client client;

};



static struct bcm2835_mbox *bcm2835_link_mbox(struct mbox_chan *link)
{
	return container_of(link->mbox, struct bcm2835_mbox, controller);
}

static irqreturn_t bcm2835_mbox_irq(int irq, void *dev_id)
{
	struct bcm2835_mbox *mbox = dev_id;
	struct device *dev = mbox->controller.dev;
	struct mbox_chan *link = &mbox->controller.chans[0];

	dev_info(dev, "IRQ %d triggered for mailbox\n", irq);

	while (!(readl(mbox->regs + MAIL0_STA) & ARM_MS_EMPTY)) {
		u32 raw = readl(mbox->regs + MAIL0_RD);

		dev_info(dev, "Mailbox raw reply received: 0x%08X\n", raw);

		/* Retrieve original message from framework state */
		

	
		/* Notify the mailbox core that the TX is done */
		dev_info(dev, "Calling mbox_chan_txdone(chan = %px)\n", link);
		mbox_chan_txdone(link, 0);  // Must be called before tx_done()

		/* Notify the client */
		if (link->cl && link->cl->tx_done) {
			dev_info(dev, "Calling client tx_done callback (msg = %px)\n", raw);
			link->cl->tx_done(link->cl, raw, 0);
			dev_info(dev, "Client tx_done callback completed\n");
		} else {
			dev_warn(dev, "No tx_done callback set on channel client\n");
		}
	}

	dev_info(dev, "IRQ %d handled successfully\n", irq);
	return IRQ_HANDLED;
}


static int bcm2835_send_data(struct mbox_chan *link, void *data)
{
	if (!link || !link->mbox || !data) {
		pr_err("bcm2835-mbox: invalid arguments in send_data: link=%p, link->mbox=%p, data=%p\n",
		       link, link ? link->mbox : NULL, data);
		return -EINVAL;
	}

	struct bcm2835_mbox *mbox = container_of(link->mbox, struct bcm2835_mbox, controller);
	u32 msg = *(u32 *)data;

	dev_info(mbox->controller.dev, "SEND_DATA called with 0x%08X\n", msg);

	spin_lock(&mbox->lock);

	while (readl(mbox->regs + MAIL1_STA) & ARM_MS_FULL)
		cpu_relax();

	writel(msg, mbox->regs + MAIL1_WRT);
	dev_dbg(mbox->controller.dev, "Data 0x%08X written to MAIL1_WRT\n", msg);

	spin_unlock(&mbox->lock);

	dev_info(mbox->controller.dev, "chan=%p, &tx_complete=%p\n",
         link, &link->tx_complete);

	// Notify mailbox subsystem if required

	return 0;
}


static int bcm2835_startup(struct mbox_chan *link)
{
	struct bcm2835_mbox *mbox = bcm2835_link_mbox(link);

	dev_dbg(mbox->controller.dev, "Starting up mailbox channel\n");
	writel(ARM_MC_IHAVEDATAIRQEN, mbox->regs + MAIL0_CNF);
	dev_dbg(mbox->controller.dev, "Interrupts enabled for mailbox\n");

	return 0;
}

static void bcm2835_shutdown(struct mbox_chan *link)
{
	struct bcm2835_mbox *mbox = bcm2835_link_mbox(link);

	dev_dbg(mbox->controller.dev, "Shutting down mailbox channel\n");
	writel(0, mbox->regs + MAIL0_CNF);
	dev_dbg(mbox->controller.dev, "Interrupts disabled for mailbox\n");
}



static const struct mbox_chan_ops bcm2835_mbox_chan_ops = {
	.send_data     = bcm2835_send_data,
	.startup       = bcm2835_startup,
	.shutdown      = bcm2835_shutdown,

};

static int bcm2835_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_mbox *mbox;
	struct resource *res;
	int irq, ret;

	dev_info(dev, "Probing BCM2835 mailbox driver\n");

	/* Allocate and initialize private data */
	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return dev_err_probe(dev, -ENOMEM, "Failed to allocate mailbox struct\n");

	spin_lock_init(&mbox->lock);

	/* Map MMIO region from ACPI _CRS */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox->regs))
		return dev_err_probe(dev, PTR_ERR(mbox->regs), "Failed to map MMIO resource\n");

	/* Set up IRQ from ACPI _CRS */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "Failed to retrieve IRQ\n");

	ret = devm_request_irq(dev, irq, bcm2835_mbox_irq, IRQF_NO_SUSPEND,
	                       dev_name(dev), mbox);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ %d\n", irq);

	/* Prepare mailbox channel */
	mbox->controller.num_chans = 1;
	mbox->controller.dev = dev;
	mbox->controller.ops = &bcm2835_mbox_chan_ops;
	mbox->controller.txdone_irq = false;


mbox->controller.txdone_poll = false;




	mbox->controller.chans = devm_kcalloc(dev, 1, sizeof(struct mbox_chan), GFP_KERNEL);
	if (!mbox->controller.chans)
		return dev_err_probe(dev, -ENOMEM, "Failed to allocate mailbox channel array\n");
	/* Initialize mailbox client */
	
	mbox->controller.chans[0].cl = &mbox->client; 

	
	mbox->controller.chans[0].mbox = &mbox->controller;
    mbox->client.dev = dev;
    mbox->client.tx_block = true;
    mbox->client.knows_txdone = true;


	rpi_mbox_chan0 = &mbox->controller.chans[0];



init_completion(&mbox->controller.chans[0].tx_complete);




	/* Register the mailbox controller */
	ret = devm_mbox_controller_register(dev, &mbox->controller);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register mailbox controller\n");

	/* Global references for ACPI power driver */

	
	rpi_mbox_chan0 = &mbox->controller.chans[0];

	dev_info(dev, "rpi_mbox_chan0 = %px\n", rpi_mbox_chan0);
	dev_info(dev, "rpi_mbox_chan0->cl = %px\n", rpi_mbox_chan0->cl);







    




	platform_set_drvdata(pdev, mbox);

	dev_info(dev, "BCM2835 ACPI mailbox controller initialized successfully\n");
	return 0;
}

static int bcm2835_mbox_remove(struct platform_device *pdev)
{
	

	dev_info(&pdev->dev, "Removing BCM2835 mailbox driver\n");


    rpi_mbox_chan0 = NULL;
	return 0;
}


static const struct acpi_device_id bcm2835_mbox_acpi_ids[] = {
	{ "BCM2849", 0 }, // Matches your ACPI table
	{ },
};
MODULE_DEVICE_TABLE(acpi, bcm2835_mbox_acpi_ids);

static struct platform_driver bcm2835_mbox_driver = {
	.probe  = bcm2835_mbox_probe,
	.remove = bcm2835_mbox_remove,
	.driver = {
		.name = "bcm2835-mbox-acpi",
		.acpi_match_table = bcm2835_mbox_acpi_ids,
	},
};
module_platform_driver(bcm2835_mbox_driver);

MODULE_AUTHOR("Converted by ChatGPT for ACPI use");
MODULE_DESCRIPTION("BCM2835 Mailbox driver (ACPI-only)");
MODULE_LICENSE("GPL v2");
