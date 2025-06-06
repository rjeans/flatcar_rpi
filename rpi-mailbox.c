// SPDX-License-Identifier: GPL-2.0
/*
 * rpi-mailbox.c - ACPI adaptation of the BCM2835 mailbox driver
 *
 * Copyright (C) 2010,2015 Broadcom
 * Copyright (C) 2013-2014 Lubomir Rintel
 * Copyright (C) 2013 Craig McGeachie
 * Copyright (C) 2023 Richard Jeans <rich@jeansy.org>
 *
 * This driver is based on the original code from drivers/mailbox/bcm2835-mailbox.c.
 * Parts of the driver are derived from:
 *  - arch/arm/mach-bcm2708/vcio.c by Gray Girling, obtained from branch
 *    "rpi-3.6.y" of git://github.com/raspberrypi/linux.git
 *  - drivers/mailbox/bcm2835-ipc.c by Lubomir Rintel at
 *    https://github.com/hackerspace/rpi-linux/blob/lr-raspberry-pi/drivers/
 *    mailbox/bcm2835-ipc.c
 *  - Documentation available at:
 *    https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 */

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
#include "rpi-mailbox.h"



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

struct rpi_mbox {
    void __iomem *regs;
    struct mbox_controller controller;
    struct device *dev;
    struct mbox_chan chans[BCM2835_MAX_CHANNELS];
    struct completion tx_completions[BCM2835_MAX_CHANNELS];
    int irq;
    spinlock_t lock;
};


struct rpi_mbox *rpi_mbox_global;

#define RPI_MBOX_CHAN_FIRMWARE 8

struct mbox_chan *rpi_mbox_request_firmware_channel(struct mbox_client *cl)
{
	struct rpi_mbox *mbox = rpi_mbox_global;
	struct mbox_chan *chan;
	int ret;

	if (!cl || !mbox) {
		pr_err("rpi_mbox_request_firmware_channel: Invalid client or uninitialized mailbox\n");
		return ERR_PTR(-ENODEV);
	}

	if (RPI_MBOX_CHAN_FIRMWARE >= mbox->controller.num_chans) {
		pr_err("rpi_mbox_request_firmware_channel: Firmware channel index out of range\n");
		return ERR_PTR(-EINVAL);
	}

	chan = &mbox->chans[RPI_MBOX_CHAN_FIRMWARE];

	if (chan->cl) {
		pr_err("rpi_mbox_request_firmware_channel: Firmware channel already bound\n");
		return ERR_PTR(-EBUSY);
	}

	ret = mbox_bind_client(chan, cl);
	if (ret) {
		pr_err("rpi_mbox_request_firmware_channel: Failed to bind client: %d\n", ret);
		return ERR_PTR(ret);
	}

	init_completion(&mbox->tx_completions[RPI_MBOX_CHAN_FIRMWARE]);
	chan->mbox = &mbox->controller;

	return chan;
}
EXPORT_SYMBOL_GPL(rpi_mbox_request_firmware_channel);



struct mbox_chan *rpi_mbox_request_channel(struct mbox_client *cl)
{
	struct mbox_chan *chan;
	int i, ret;

	if (!cl || !rpi_mbox_global) {
		pr_err("rpi_mbox_request_channel: Invalid client or uninitialized mailbox\n");
		return ERR_PTR(-ENODEV);
	}

	for (i = 0; i < rpi_mbox_global->controller.num_chans; i++) {
		if (i == RPI_MBOX_CHAN_FIRMWARE)
			continue;

		chan = &rpi_mbox_global->chans[i];
		if (!chan->cl) {
			ret = mbox_bind_client(chan, cl);
			if (ret) {
				pr_err("rpi_mbox_request_channel: Failed to bind client: %d\n", ret);
				return ERR_PTR(ret);
			}

			init_completion(&rpi_mbox_global->tx_completions[i]);
			chan->mbox = &rpi_mbox_global->controller;
			return chan;
		}
	}

	pr_err("rpi_mbox_request_channel: No free channel available\n");
	return ERR_PTR(-EBUSY);
}
EXPORT_SYMBOL_GPL(rpi_mbox_request_channel);

int rpi_mbox_free_channel(struct mbox_chan *chan)
{
	if (!chan) {
		pr_err("rpi_mbox_free_channel: Invalid channel\n");
		return -EINVAL;
	}

	if (!chan->cl) {
		pr_err("rpi_mbox_free_channel: Channel not bound or already unbound\n");
		return -ENODEV;
	}

	chan->cl = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(rpi_mbox_free_channel);

static int rpi_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct rpi_mbox *mbox = container_of(chan->mbox, struct rpi_mbox, controller);
	u32 msg = *(u32 *)data;

	if (!chan || !data) {
		pr_err("rpi_mbox_send_data: Invalid channel or data\n");
		return -EINVAL;
	}

	spin_lock(&mbox->lock);
	writel(msg, mbox->regs + MAIL1_WRT);
	spin_unlock(&mbox->lock);

	return 0;
}

static bool rpi_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct rpi_mbox *mbox = container_of(chan->mbox, struct rpi_mbox, controller);
	bool ret;

	spin_lock(&mbox->lock);
	ret = !(readl(mbox->regs + MAIL1_STA) & ARM_MS_FULL);
	spin_unlock(&mbox->lock);

	return ret;
}

static int rpi_mbox_startup(struct mbox_chan *chan)
{
	struct rpi_mbox *mbox = container_of(chan->mbox, struct rpi_mbox, controller);

	/* Enable the interrupt on data reception */
	writel(ARM_MC_IHAVEDATAIRQEN, mbox->regs + MAIL0_CNF);

	return 0;
}

static void rpi_mbox_shutdown(struct mbox_chan *chan)
{
	struct rpi_mbox *mbox;

	mbox = container_of(chan->mbox, struct rpi_mbox, controller);
}

static irqreturn_t rpi_mbox_irq(int irq, void *dev_id)
{
	struct rpi_mbox *mbox = dev_id;
	struct device *dev = mbox->controller.dev;
	irqreturn_t handled = IRQ_NONE;

	if (!mbox) {
		pr_err("rpi_mbox_irq: Invalid mailbox context\n");
		return IRQ_NONE;
	}

	// Process all pending messages
	while (!(readl(mbox->regs + MAIL0_STA) & ARM_MS_EMPTY)) {
		u32 msg = readl(mbox->regs + MAIL0_RD);
		u32 chan_index = msg & 0xf;

		// Validate channel index
		if (chan_index >= BCM2835_MAX_CHANNELS) {
			dev_warn(dev, "rpi_mbox_irq: Invalid channel index %u in IRQ msg 0x%08X\n", chan_index, msg);
			continue;
		}

		// Get the channel and ensure it is bound
		struct mbox_chan *chan = &mbox->chans[chan_index];
		if (!chan->cl || !chan->cl->rx_callback) {
			dev_warn(dev, "rpi_mbox_irq: Unbound mailbox channel %u (msg=0x%08X), skipping\n", chan_index, msg);
			continue;
		}

		// Dispatch the message to the client
		mbox_chan_received_data(chan, &msg);
		handled = IRQ_HANDLED;
	}

	return handled;
}

static const struct mbox_chan_ops rpi_mbox_chan_ops = {
	.send_data     = rpi_mbox_send_data,
	.startup       = rpi_mbox_startup,
	.shutdown      = rpi_mbox_shutdown,
	.last_tx_done  = rpi_mbox_last_tx_done,
};

static int rpi_mbox_probe(struct platform_device *pdev)
{
	struct rpi_mbox *mbox;
	struct resource *res;
	int ret;

	// Check if CONFIG_MAILBOX is enabled
#ifndef CONFIG_MAILBOX
	dev_err(&pdev->dev, "CONFIG_MBOX is not enabled. Cannot initialize rpi-mailbox.\n");
	return -ENODEV;
#endif

	// Log the start of the probe function
	dev_info(&pdev->dev, "Probing rpi-mailbox device\n");

	// Allocate memory for the mailbox structure
	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox) {
		dev_err(&pdev->dev, "Failed to allocate memory for mailbox structure\n");
		return -ENOMEM;
	}

	// Store the mailbox structure in the platform device's driver data
	platform_set_drvdata(pdev, mbox);
	mbox->dev = &pdev->dev;
	rpi_mbox_global = mbox;

	// Get the IRQ resource for the mailbox
	mbox->irq = platform_get_irq(pdev, 0);
	if (mbox->irq < 0) {
		ret = dev_err_probe(&pdev->dev, mbox->irq, "Failed to get IRQ\n");
		goto err_free_mbox;
	}

	// Request the IRQ and associate it with the mailbox IRQ handler
	ret = devm_request_irq(&pdev->dev, mbox->irq, rpi_mbox_irq,
			       0, dev_name(&pdev->dev), mbox);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "Failed to request IRQ\n");
		goto err_free_mbox;
	}

	// Get the memory resource for the mailbox registers
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mbox->regs)) {
		ret = PTR_ERR(mbox->regs);
		dev_err(&pdev->dev, "Failed to map mailbox registers: %d\n", ret);
		goto err_free_mbox;
	}

	// Initialize the mailbox controller
	mbox->controller.dev = &pdev->dev;
	mbox->controller.chans = mbox->chans;
	mbox->controller.num_chans = BCM2835_MAX_CHANNELS;
	mbox->controller.ops = &rpi_mbox_chan_ops;
	mbox->controller.txdone_poll = true;
	mbox->controller.txpoll_period = 5;

	// Register the mailbox controller
	ret = devm_mbox_controller_register(&pdev->dev, &mbox->controller);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register mailbox controller: %d\n", ret);
		goto err_free_mbox;
	}

	// Log successful initialization
	dev_info(&pdev->dev, "rpi-mailbox device initialized successfully\n");
	return 0;

err_free_mbox:
	// Cleanup in case of any errors
	devm_kfree(&pdev->dev, mbox);
	return ret;
}

static int rpi_mbox_remove(struct platform_device *pdev)
{
	struct rpi_mbox *mbox = platform_get_drvdata(pdev);

	// Log the start of the remove function
	dev_info(&pdev->dev, "Removing rpi-mailbox device\n");

	if (mbox) {
		// Perform any necessary cleanup here
		devm_kfree(&pdev->dev, mbox);
	}

	dev_info(&pdev->dev, "rpi-mailbox device removed successfully\n");
	return 0;
}

static const struct acpi_device_id rpi_mbox_acpi_ids[] = {
	{ "BCM2849", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rpi_mbox_acpi_ids);

static struct platform_driver rpi_mbox_driver = {
	.driver = {
		.name = "rpi-mbox",
		.acpi_match_table = rpi_mbox_acpi_ids,
	},
	.probe = rpi_mbox_probe,
	.remove = rpi_mbox_remove,
};

module_platform_driver(rpi_mbox_driver);

MODULE_AUTHOR("Richard Jeans <rich@jeansy.org>");
MODULE_DESCRIPTION("ACPI adaptation of the BCM2835 mailbox controller");
MODULE_LICENSE("GPL v2");
