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

	while (!(readl(mbox->regs + MAIL0_STA) & ARM_MS_EMPTY)) {
		u32 msg = readl(mbox->regs + MAIL0_RD);
		dev_dbg(dev, "Reply 0x%08X\n", msg);
		mbox_chan_received_data(link, &msg);
	}
	return IRQ_HANDLED;
}

static int bcm2835_send_data(struct mbox_chan *link, void *data)
{
	struct bcm2835_mbox *mbox = bcm2835_link_mbox(link);
	u32 msg = *(u32 *)data;

	spin_lock(&mbox->lock);
	writel(msg, mbox->regs + MAIL1_WRT);
	dev_dbg(mbox->controller.dev, "Request 0x%08X\n", msg);
	spin_unlock(&mbox->lock);
	return 0;
}

static int bcm2835_startup(struct mbox_chan *link)
{
	struct bcm2835_mbox *mbox = bcm2835_link_mbox(link);
	writel(ARM_MC_IHAVEDATAIRQEN, mbox->regs + MAIL0_CNF);
	return 0;
}

static void bcm2835_shutdown(struct mbox_chan *link)
{
	struct bcm2835_mbox *mbox = bcm2835_link_mbox(link);
	writel(0, mbox->regs + MAIL0_CNF);
}

static bool bcm2835_last_tx_done(struct mbox_chan *link)
{
	struct bcm2835_mbox *mbox = bcm2835_link_mbox(link);
	bool ret;

	spin_lock(&mbox->lock);
	ret = !(readl(mbox->regs + MAIL1_STA) & ARM_MS_FULL);
	spin_unlock(&mbox->lock);
	return ret;
}

static const struct mbox_chan_ops bcm2835_mbox_chan_ops = {
	.send_data     = bcm2835_send_data,
	.startup       = bcm2835_startup,
	.shutdown      = bcm2835_shutdown,
	.last_tx_done  = bcm2835_last_tx_done
};

static int bcm2835_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_mbox *mbox;
	struct resource *res;
	int irq, ret;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	spin_lock_init(&mbox->lock);

	// Request memory region from ACPI-defined _CRS
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox->regs))
		return PTR_ERR(mbox->regs);

	// Get IRQ from _CRS
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, bcm2835_mbox_irq, IRQF_NO_SUSPEND,
	                       dev_name(dev), mbox);
	if (ret) {
		dev_err(dev, "Failed to request IRQ %d: %d\n", irq, ret);
		return ret;
	}

	mbox->controller.txdone_poll = true;
	mbox->controller.txpoll_period = 5;
	mbox->controller.ops = &bcm2835_mbox_chan_ops;
	mbox->controller.dev = dev;
	mbox->controller.num_chans = 1;

	mbox->controller.chans = devm_kzalloc(dev,
		sizeof(*mbox->controller.chans), GFP_KERNEL);
	if (!mbox->controller.chans)
		return -ENOMEM;

	ret = devm_mbox_controller_register(dev, &mbox->controller);
	if (ret)
		return ret;

	dev_info(dev, "BCM2835 ACPI mailbox controller initialized\n");
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
