// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/export.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include "mailbox-bcm2835-acpi.h"




#define CLOCK_ON   0x03
#define CLOCK_OFF  0x02
#define FIXED_RATE 19200000 // 19.2 MHz example

struct bcm2835_clk {
	struct clk_hw hw;
	struct mbox_chan *chan;
	struct mbox_client mbox_client;
	struct completion tx_done;
	const char *name;
	bool enabled;
	unsigned long rate;
};

#define to_bcm2835_clk(_hw) container_of(_hw, struct bcm2835_clk, hw)

static int bcm2835_clk_send(struct bcm2835_clk *clk, bool enable)
{
	u32 *msg = kzalloc(sizeof(u32), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	*msg = enable ? CLOCK_ON : CLOCK_OFF;

	reinit_completion(&clk->tx_done);
	int ret = mbox_send_message(clk->chan, msg);
	if (ret < 0)
		return ret;

	ret = wait_for_completion_timeout(&clk->tx_done, msecs_to_jiffies(100));
	return (ret == 0) ? -ETIMEDOUT : 0;
}

static void bcm2835_clk_tx_done(struct mbox_client *cl, void *msg, int r)
{
	struct device *dev = cl->dev;
	struct bcm2835_clk *clk = dev_get_drvdata(dev);
	complete(&clk->tx_done);
	kfree(msg);
}


static int bcm2835_clk_set_rate(struct clk_hw *hw, unsigned long rate,
                                unsigned long parent_rate)
{
	struct bcm2835_clk *clk = to_bcm2835_clk(hw);
	clk->rate = rate;
	return 0;
}

static long bcm2835_clk_round_rate(struct clk_hw *hw, unsigned long rate,
                                   unsigned long *parent_rate)
{
	return rate;
}
static int bcm2835_clk_enable(struct clk_hw *hw)
{
	struct bcm2835_clk *clk = to_bcm2835_clk(hw);
	if (clk->enabled)
		return 0;
	int ret = bcm2835_clk_send(clk, true);
	if (ret == 0)
		clk->enabled = true;
	return ret;
}

static void bcm2835_clk_disable(struct clk_hw *hw)
{
	struct bcm2835_clk *clk = to_bcm2835_clk(hw);
	if (!clk->enabled)
		return;
	if (bcm2835_clk_send(clk, false) == 0)
		clk->enabled = false;
}

static unsigned long bcm2835_clk_recalc_rate(struct clk_hw *hw,
                                             unsigned long parent_rate)
{
	struct bcm2835_clk *clk = to_bcm2835_clk(hw);
	return clk->rate;
}

static const struct clk_ops bcm2835_clk_ops = {
	.enable = bcm2835_clk_enable,
	.set_rate = bcm2835_clk_set_rate,
	.round_rate = bcm2835_clk_round_rate,
	.disable = bcm2835_clk_disable,
	.recalc_rate = bcm2835_clk_recalc_rate,
};

static int bcm2835_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_clk *clk;
	struct clk_init_data init = {};
	int ret;

	dev_info(dev, "Probing bcm2835-clk-acpi driver\n");

	clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
	if (!clk) {
		dev_info(dev, "Returning from probe with error: -ENOMEM\n");
		return -ENOMEM;
	}

	if (device_property_read_string(dev, "rpi,devicename", &clk->name)) {
		dev_err(dev, "Missing required property 'rpi,devicename'\n");
		dev_info(dev, "Returning from probe with error: -EINVAL\n");
		return -EINVAL;
	}
	dev_info(dev, "Clock domain name: %s\n", clk->name);

	clk->mbox_client.dev = dev;
	clk->mbox_client.tx_done = bcm2835_clk_tx_done;
	clk->mbox_client.tx_block = false;
	clk->mbox_client.knows_txdone = true;

	/* Optional: set rate from property, or use fixed default */
	device_property_read_u32(dev, "clock-frequency", (u32 *)&clk->rate);
	if (!clk->rate) {
		clk->rate = FIXED_RATE;
		dev_info(dev, "No clock-frequency property, using default: %lu\n", clk->rate);
	} else {
		dev_info(dev, "Clock frequency set from property: %lu\n", clk->rate);
	}

	clk->chan = global_rpi_mbox_chan;
	if (IS_ERR(clk->chan)) {
		dev_err(dev, "Failed to acquire mailbox channel: %ld\n", PTR_ERR(clk->chan));
		return PTR_ERR(clk->chan);
	}
	dev_info(dev, "Mailbox channel acquired\n");

	init_completion(&clk->tx_done);

	if (!clk->chan->cl) {
	clk->chan->cl = &clk->mbox_client;
	   dev_info(dev, "Mailbox client assigned (without bind)\n");
    } else {
	   dev_warn(dev, "Mailbox channel already in use — assuming shared access\n");
    }

	init.name = clk->name;
	init.ops = &bcm2835_clk_ops;
	init.flags = 0; /* No CLK_IS_BASIC; use appropriate CLK_ flags if needed */

	clk->hw.init = &init;

	ret = devm_clk_hw_register(dev, &clk->hw);
	if (ret) {
		dev_err(dev, "Failed to register clk_hw: %d\n", ret);
		return ret;
	}
	dev_info(dev, "clk_hw registered\n");

	struct clk *c = clk_hw_get_clk(&clk->hw, NULL);
	if (IS_ERR(c)) {
		dev_err(dev, "Failed to get clk from clk_hw: %ld\n", PTR_ERR(c));
		return PTR_ERR(c);
	}
	dev_info(dev, "Got clk from clk_hw: %p\n", c);


	ret = clk_register_clkdev(c, clk->name, dev_name(dev));
	if (ret) {
		dev_err(dev, "Failed to register clk provider: %d\n", ret);
	}

	platform_set_drvdata(pdev, clk);
	dev_info(dev, "bcm2835-clk-acpi probe completed successfully\n");
	return ret;
}

static const struct acpi_device_id bcm2835_clk_acpi_ids[] = {
	{ "BCM2852", 0 }, // adjust this ID to match your SSDT
	{ }
};
MODULE_DEVICE_TABLE(acpi, bcm2835_clk_acpi_ids);

static struct platform_driver bcm2835_clk_driver = {
	.probe = bcm2835_clk_probe,
	.driver = {
		.name = "bcm2835-clk-acpi",
		.acpi_match_table = bcm2835_clk_acpi_ids,
	},
};
module_platform_driver(bcm2835_clk_driver);

MODULE_AUTHOR("ChatGPT + Richard");
MODULE_DESCRIPTION("Raspberry Pi firmware clock via ACPI mailbox");
MODULE_LICENSE("GPL");