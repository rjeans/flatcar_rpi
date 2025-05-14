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

#define MAIL0_RD    0x00
#define MAIL0_STA   0x18
#define MAIL0_CNF   0x1C
#define MAIL1_WRT   0x00
#define MAIL1_STA   0x18

#define ARM_MS_FULL     BIT(31)
#define ARM_MS_EMPTY    BIT(30)
#define ARM_MC_IHAVEDATAIRQEN BIT(0)

struct mbox_controller *rpi_mbox_global = NULL;
EXPORT_SYMBOL(rpi_mbox_global);

struct bcm2835_mbox {
	void __iomem *regs;
	spinlock_t lock;
	struct mbox_controller controller;
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

	dev_dbg(dev, "IRQ %d triggered\n", irq);

	while (!(readl(mbox->regs + MAIL0_STA) & ARM_MS_EMPTY)) {
		u32 msg = readl(mbox->regs + MAIL0_RD);
		dev_dbg(dev, "Reply 0x%08X received\n", msg);
		mbox_chan_received_data(link, &msg);
	}

	dev_dbg(dev, "IRQ %d handled\n", irq);
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
	
		dev_info(mbox->controller.dev, "About to call mbox_chan_txdone: link=%p, mbox=%p\n",
		         link, link->mbox);
		mbox_chan_txdone(link, 0);
		dev_info(mbox->controller.dev, "Successfully called mbox_chan_txdone\n");
	

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

static bool bcm2835_last_tx_done(struct mbox_chan *link)
{
	struct bcm2835_mbox *mbox = bcm2835_link_mbox(link);
	bool ret;

	dev_dbg(mbox->controller.dev, "Checking if last transmission is done\n");

	spin_lock(&mbox->lock);
	ret = !(readl(mbox->regs + MAIL1_STA) & ARM_MS_FULL);
	spin_unlock(&mbox->lock);

	dev_dbg(mbox->controller.dev, "Last transmission done: %s\n", ret ? "yes" : "no");
	return ret;
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

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox) {
		dev_err(dev, "Failed to allocate memory for mailbox structure\n");
		return -ENOMEM;
	}

	spin_lock_init(&mbox->lock);

	dev_dbg(dev, "Requesting memory region from ACPI-defined _CRS\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox->regs)) {
		dev_err(dev, "Failed to map memory resource\n");
		return PTR_ERR(mbox->regs);
	}

	dev_dbg(dev, "Getting IRQ from ACPI-defined _CRS\n");
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}

	dev_dbg(dev, "Requesting IRQ %d\n", irq);
	ret = devm_request_irq(dev, irq, bcm2835_mbox_irq, IRQF_NO_SUSPEND,
	                       dev_name(dev), mbox);
	if (ret) {
		dev_err(dev, "Failed to request IRQ %d: %d\n", irq, ret);
		return ret;
	}

	dev_dbg(dev, "Initializing mailbox controller\n");
	mbox->controller.ops = &bcm2835_mbox_chan_ops;
	mbox->controller.dev = dev;
	mbox->controller.num_chans = 1;

ret = devm_mbox_controller_register(dev, &mbox->controller);
	if (ret) {
		dev_err(dev, "Failed to register mailbox controller: %d\n", ret);
		return ret;
	}

	ret = devm_mbox_controller_register(dev, &mbox->controller);
	if (ret) {
		dev_err(dev, "Failed to register mailbox controller: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, mbox);

	dev_info(dev, "platform_set_drvdata confirmed: %p\n", dev_get_drvdata(&pdev->dev));

    rpi_mbox_global = &mbox->controller; // Set the global pointer to the mailbox controller
	dev_info(dev, "Global mailbox controller set: %p\n", rpi_mbox_global);

	dev_info(dev, "BCM2835 ACPI mailbox controller initialized successfully\n");
	return 0;
}

static const struct acpi_device_id bcm2835_mbox_acpi_ids[] = {
	{ "BCM2849", 0 }, // Matches your ACPI table
	{ },
};
MODULE_DEVICE_TABLE(acpi, bcm2835_mbox_acpi_ids);

static struct platform_driver bcm2835_mbox_driver = {
	.probe  = bcm2835_mbox_probe,
	.driver = {
		.name = "bcm2835-mbox-acpi",
		.acpi_match_table = bcm2835_mbox_acpi_ids,
	},
};
module_platform_driver(bcm2835_mbox_driver);

MODULE_AUTHOR("Converted by ChatGPT for ACPI use");
MODULE_DESCRIPTION("BCM2835 Mailbox driver (ACPI-only)");
MODULE_LICENSE("GPL v2");
