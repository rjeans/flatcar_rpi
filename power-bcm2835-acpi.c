
// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mailbox_controller.h>  

#include <linux/property.h>
#include <linux/slab.h>

extern struct mbox_chan *rpi_mbox_chan0;


#define POWER_DOMAIN_ON     0x03
#define POWER_DOMAIN_OFF    0x02

struct rpi_power_domain {
	struct mbox_chan *chan;
	struct mbox_client mbox_client;
	const char *name;
	struct completion tx_done;
};

static int rpi_power_send(struct rpi_power_domain *rpd, bool enable)
{
	struct mbox_chan *chan = rpd->chan;
	struct device *dev = rpd->mbox_client.dev;
	int ret;
	u32 *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	if (!chan ) {
		dev_err(dev, "Cannot send message: NULL chan or client\n");
		return -ENODEV;
	}

	*msg = (enable ? POWER_DOMAIN_ON : POWER_DOMAIN_OFF);

	dev_info(dev, "Sending firmware power %s for domain '%s'\n",
	         enable ? "ON" : "OFF", rpd->name);

	reinit_completion(&rpd->tx_done);
	ret = mbox_send_message(chan, msg);
	if (ret < 0) {
		dev_err(dev, "Failed to send message: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&rpd->tx_done, msecs_to_jiffies(100));
	if (ret == 0) {
		dev_err(dev, "Timeout waiting for mailbox tx completion\n");
		return -ETIMEDOUT;
	}

	dev_info(dev, "Firmware mailbox transaction complete\n");
	return 0;
}

static int rpi_power_runtime_resume(struct device *dev)
{
	struct rpi_power_domain *rpd = dev_get_drvdata(dev);
	return rpi_power_send(rpd, true);
}

static int rpi_power_runtime_suspend(struct device *dev)
{
	struct rpi_power_domain *rpd = dev_get_drvdata(dev);
	return rpi_power_send(rpd, false);
}

static void rpi_power_tx_done(struct mbox_client *cl, void *msg_ptr, int r)
{
	struct rpi_power_domain *rpd = dev_get_drvdata(cl->dev);
	u32 *msg = msg_ptr;

	complete(&rpd->tx_done);
	kfree(msg);
}

static int rpi_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpi_power_domain *rpd;

	dev_info(dev, "Probing raspberrypi-power ACPI runtime-PM driver\n");

	rpd = devm_kzalloc(dev, sizeof(*rpd), GFP_KERNEL);
	if (!rpd)
		return -ENOMEM;

	if (device_property_read_string(dev, "rpi,devicename", &rpd->name)) {
		dev_err(dev, "Missing property 'rpi,devicename'\n");
		return -EINVAL;
	}
	dev_info(dev, "Power domain name: %s\n", rpd->name);

	rpd->mbox_client.dev = dev;
	rpd->mbox_client.tx_block = false;
	rpd->mbox_client.knows_txdone = true;
	rpd->mbox_client.tx_done = rpi_power_tx_done;

	rpd->chan = rpi_mbox_chan0;
    rpd->chan->cl = &rpd->mbox_client;

	dev_info(dev, "Mailbox channel client set: cl = %px\n", rpd->chan->cl);
    dev_info(dev, "Mailbox channel address: %px\n", rpd->chan);


	if (IS_ERR(rpd->chan)) {
		dev_err(dev, "Failed to acquire mailbox channel\n");
		return PTR_ERR(rpd->chan);
	}

	

	init_completion(&rpd->tx_done);

	platform_set_drvdata(pdev, rpd);

	dev->pm_domain = NULL;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_resume(dev);

	dev_info(dev, "Power domain '%s' runtime PM ready\n", rpd->name);
	return 0;
}

static int rpi_power_remove(struct platform_device *pdev)
{
	struct rpi_power_domain *rpd = platform_get_drvdata(pdev);
	pm_runtime_disable(&pdev->dev);
	if (rpd->chan)
		mbox_free_channel(rpd->chan);
	return 0;
}

static const struct dev_pm_ops rpi_power_pm_ops = {
	.runtime_resume = rpi_power_runtime_resume,
	.runtime_suspend = rpi_power_runtime_suspend,
};

static const struct acpi_device_id rpi_power_acpi_ids[] = {
	{ "BCM2851", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rpi_power_acpi_ids);

static struct platform_driver rpi_power_driver = {
	.probe = rpi_power_probe,
	.remove = rpi_power_remove,
	.driver = {
		.name = "raspberrypi-power-acpi",
		.acpi_match_table = rpi_power_acpi_ids,
		.pm = &rpi_power_pm_ops,
	},
};
module_platform_driver(rpi_power_driver);

MODULE_AUTHOR("ChatGPT + Richard");
MODULE_DESCRIPTION("Raspberry Pi firmware power domain using runtime PM and ACPI");
MODULE_LICENSE("GPL");