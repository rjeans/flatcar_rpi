// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/pm_domain.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/mailbox_client.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/fwnode.h>
#include <linux/mailbox_controller.h>

// External reference to mailbox channel
extern struct mbox_chan *rpi_mbox_chan0;

#define POWER_DOMAIN_ON     0x03  // ON (bit 0) + WAIT (bit 1)
#define POWER_DOMAIN_OFF    0x02  // OFF (bit 0 clear) + WAIT (bit 1)

// Structure representing a Raspberry Pi power domain
struct rpi_power_domain {
	struct generic_pm_domain genpd;
	struct mbox_chan *chan;
	struct mbox_client mbox_client;
	const char *name;
	struct completion tx_done;
};




// Function to send power domain messages
static int rpi_power_send(struct rpi_power_domain *rpd, bool enable)
{
	struct mbox_chan *chan = rpd->chan;
	struct device *dev = rpd->mbox_client.dev;
	
	
	int ret;
	u32 * msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	if (!chan || !chan->cl) {
		dev_err(dev, "Cannot send message: NULL chan or client\n");
		return -ENODEV;
	}

	


    *msg = (enable ? POWER_DOMAIN_ON : POWER_DOMAIN_OFF);
	


	dev_info(dev, "Sending firmware power %s for domain '%s'\n",
	         enable ? "ON" : "OFF", rpd->name);

	

	dev_info(dev, "Message: 0x%px| chan = %px | chan->cl = %px\n",
	        msg, chan, chan->cl);
	

	reinit_completion(&rpd->tx_done);  // 🔧 Reset before send
 
	ret = mbox_send_message(chan, msg);
	if (ret < 0) {
		dev_err(dev, "Failed to send message: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Waiting for tx completion...\n");

	ret = wait_for_completion_timeout(&rpd->tx_done, msecs_to_jiffies(100));
	if (ret == 0) {
		dev_err(dev, "Timeout waiting for mailbox tx completion\n");
		return -ETIMEDOUT;
	}

	dev_info(dev, "Mailbox tx completed successfully for domain '%s'\n", rpd->name);
	return 0;
}

// Power on callback for the power domain
static int rpi_power_on(struct generic_pm_domain *genpd)
{
	struct rpi_power_domain *rpd = container_of(genpd, struct rpi_power_domain, genpd);
	return rpi_power_send(rpd, true);
}

// Power off callback for the power domain
static int rpi_power_off(struct generic_pm_domain *genpd)
{
	struct rpi_power_domain *rpd = container_of(genpd, struct rpi_power_domain, genpd);
	return rpi_power_send(rpd, false);
}

// Release callback for the power domain device
static void rpi_power_release(struct device *dev)
{
	dev_info(dev, "Released power domain device\n");
}

static void rpi_power_tx_done(struct mbox_client *cl, void *msg_ptr, int r)
{
	u32 *msg = msg_ptr;
	struct device *dev = cl->dev;
	struct rpi_power_domain *rpd = dev_get_drvdata(dev);

	if (!rpd) {
		dev_err(dev, "No rpd set\n");
		kfree(msg);
		return;
	}


	dev_info(cl->dev, "tx_done callback called");
	dev_info(cl->dev, "TX_DONE: msg pointer = %px, val = 0x%u\n", msg, *msg);

	complete(&rpd->tx_done);
	kfree(msg);

	dev_info(cl->dev, "tx_done callback completed");
}

// Probe function for the power domain driver
static int rpi_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpi_power_domain *rpd;
	int ret;

	dev_info(dev, "Probing raspberrypi-power ACPI driver\n");

	// Allocate memory for the power domain structure
	rpd = devm_kzalloc(dev, sizeof(*rpd), GFP_KERNEL);
	if (!rpd)
		return -ENOMEM;

	// Read the power domain name
	if (device_property_read_string(dev, "rpi,devicename", &rpd->name)) {
		dev_err(dev, "Missing required property 'rpi,devicename'\n");
		return -EINVAL;
	}
	dev_info(dev, "Firmware domain name: %s\n", rpd->name);


	// Initialize the mailbox client
	rpd->mbox_client.dev = dev;
	rpd->mbox_client.tx_block = false;
	rpd->mbox_client.knows_txdone = true;
	rpd->mbox_client.tx_done = rpi_power_tx_done;


	// Acquire the mailbox channel
	rpd->chan = rpi_mbox_chan0;
	if (IS_ERR(rpd->chan)) {
		dev_err(dev, "Failed to acquire mailbox channel: %ld\n", PTR_ERR(rpd->chan));
		return PTR_ERR(rpd->chan);
	}

	// Initialize the completion structure and bind the client
	
	init_completion(&rpd->tx_done);
if (!rpd->chan->cl) {
	rpd->chan->cl = &rpd->mbox_client;
	dev_info(dev, "Mailbox client assigned (without bind)\n");
} else {
	dev_warn(dev, "Mailbox channel already in use — assuming shared access\n");
}
	dev_info(dev, "Mailbox channel acquired\n");

	// Setup the generic power domain
	rpd->genpd.name = rpd->name;
	rpd->genpd.dev.release = rpi_power_release;
	rpd->genpd.power_on = rpi_power_on;
	rpd->genpd.power_off = rpi_power_off;
	rpd->genpd.flags = GENPD_FLAG_PM_CLK ;

	// Initialize the power domain
	ret = pm_genpd_init(&rpd->genpd, NULL, false);
	if (ret) {
		dev_err(dev, "Failed to initialize generic power domain: %d\n", ret);
		mbox_free_channel(rpd->chan);
		return ret;
	}

	// Add the device to the power domain
	ret = pm_genpd_add_device(&rpd->genpd, dev);
	if (ret) {
		dev_err(dev, "Failed to add device to power domain: %d\n", ret);
		pm_genpd_remove(&rpd->genpd);
		mbox_free_channel(rpd->chan);
		return ret;
	}

	platform_set_drvdata(pdev, rpd);
	dev_info(dev, "Power domain '%s' initialized\n", rpd->name);



	return 0;
}

// Remove function for the power domain driver
static int rpi_power_remove(struct platform_device *pdev)
{
	struct rpi_power_domain *rpd = platform_get_drvdata(pdev);

	pm_genpd_remove_device(&pdev->dev);
	pm_genpd_remove(&rpd->genpd);

	if (rpd->chan)
		mbox_free_channel(rpd->chan);

	return 0;
}

// ACPI device ID table
static const struct acpi_device_id rpi_power_acpi_ids[] = {
	{ "BCM2851", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rpi_power_acpi_ids);

// Platform driver definition
static struct platform_driver rpi_power_driver = {
	.probe = rpi_power_probe,
	.remove = rpi_power_remove,
	.driver = {
		.name = "raspberrypi-power-acpi",
		.acpi_match_table = rpi_power_acpi_ids,
	},
};
module_platform_driver(rpi_power_driver);

MODULE_AUTHOR("ChatGPT + Richard");
MODULE_DESCRIPTION("Raspberry Pi firmware power domain via ACPI");
MODULE_LICENSE("GPL");
