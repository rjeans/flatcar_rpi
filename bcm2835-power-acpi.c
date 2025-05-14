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


#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/mailbox_controller.h>
#include <linux/platform_device.h>
#include <linux/property.h>



extern struct mbox_chan *rpi_mbox_chan0;

#define POWER_DOMAIN_ON     0x03  // ON (bit 0) + WAIT (bit 1)
#define POWER_DOMAIN_OFF    0x02  // OFF (bit 0 clear) + WAIT (bit 1)

struct rpi_power_domain {
	struct generic_pm_domain genpd;
	struct mbox_chan *chan;
	struct mbox_client mbox_client;
	const char *name;
};

static int rpi_power_send(struct rpi_power_domain *rpd, bool enable)
{
	struct device *dev = rpd->mbox_client.dev;
	u32 msg;
	int ret_val,ret;

	dev_info(dev,
    "Sending message: chan=%px, chan->cl=%px\n",
    rpd->chan, rpd->chan ? rpd->chan->cl : NULL);


    if (!rpd->chan || !rpd->chan->cl) {
	dev_err(dev, "Cannot send message: NULL chan or client\n");
	return -ENODEV;
}

	msg = (enable ? POWER_DOMAIN_ON : POWER_DOMAIN_OFF);
	dev_info(dev, "Sending firmware power %s for domain '%s': 0x%08X\n",
	         enable ? "ON" : "OFF", rpd->name, msg);

			 dev_info(dev, "Power driver: sending via chan = %px, tx_complete = %px\n",
         rpd->chan, &rpd->chan->tx_complete);

    reinit_completion(&rpd->chan->tx_complete);  // ← THIS IS ESSENTIAL

	ret= mbox_send_message(rpd->chan, &msg);
if (ret < 0) {
	dev_err(dev, "Failed to send message: %d\n", ret);
	return ret;
}

if (!wait_for_completion_timeout(&rpd->chan->tx_complete, msecs_to_jiffies(100))) {
	dev_err(dev, "Timeout waiting for mailbox tx completion\n");
	return -ETIMEDOUT;
}

return 0;


}

static int rpi_power_on(struct generic_pm_domain *genpd)
{
	struct rpi_power_domain *rpd = container_of(genpd, struct rpi_power_domain, genpd);
	return rpi_power_send(rpd, true);
}

static int rpi_power_off(struct generic_pm_domain *genpd)
{
	struct rpi_power_domain *rpd = container_of(genpd, struct rpi_power_domain, genpd);
	return rpi_power_send(rpd, false);
}

static void rpi_power_release(struct device *dev)
{
	dev_info(dev, "Released power domain device\n");
}
static int rpi_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpi_power_domain *rpd;
	u32 active;
	int ret;

	dev_info(dev, "Probing raspberrypi-power ACPI driver\n");

	rpd = devm_kzalloc(dev, sizeof(*rpd), GFP_KERNEL);
	if (!rpd)
		return -ENOMEM;

	// Required: power domain name (e.g. "pwm", "vec")
	if (device_property_read_string(dev, "rpi,devicename", &rpd->name)) {
		dev_err(dev, "Missing required property 'rpi,devicename'\n");
		return -EINVAL;
	}
	dev_info(dev, "Firmware domain name: %s\n", rpd->name);

	// Optional: whether to power on immediately
	if (device_property_read_u32(dev, "rpi,active", &active))
		active = 0;

	rpd->mbox_client.dev = dev;
    rpd->mbox_client.tx_block = true;
    rpd->mbox_client.knows_txdone = true;  
	//rpd->mbox_client.fwnode = dev_fwnode(dev);
	
	// Acquire mailbox channel via ACPI _DSD "mbox-names" = "property"
	rpd->chan = rpi_mbox_chan0;
if (IS_ERR(rpd->chan)) {
	dev_err(dev, "Failed to acquire mailbox channel: %ld\n", PTR_ERR(rpd->chan));
	return PTR_ERR(rpd->chan);
}

init_completion(&rpd->chan->tx_complete);    // ← binds the TX completion handler
rpd->chan->cl = &rpd->mbox_client;           // ← binds your client to the channel

	dev_info(dev, "Mailbox channel acquired\n");

	// Setup generic power domain

	rpd->genpd.name = rpd->name;
	rpd->genpd.dev.release = rpi_power_release;
	rpd->genpd.power_on = rpi_power_on;
	rpd->genpd.power_off = rpi_power_off;
	rpd->genpd.flags = GENPD_FLAG_PM_CLK | GENPD_FLAG_ALWAYS_ON;

	ret = pm_genpd_init(&rpd->genpd, NULL, false);
	if (ret) {
		dev_err(dev, "Failed to initialize generic power domain: %d\n", ret);
		mbox_free_channel(rpd->chan);
		return ret;
	}

	ret = pm_genpd_add_device(&rpd->genpd, dev);
	if (ret) {
		dev_err(dev, "Failed to add device to power domain: %d\n", ret);
		pm_genpd_remove(&rpd->genpd);
		mbox_free_channel(rpd->chan);
		return ret;
	}

	platform_set_drvdata(pdev, rpd);
	dev_info(dev, "Power domain '%s' initialized\n", rpd->name);

	// Power on immediately if requested
	if (active) {
		dev_info(dev, "Powering on domain '%s' (rpi,active = 1)\n", rpd->name);
		ret = rpi_power_send(rpd, true);
		if (ret)
			dev_err(dev, "Failed to power on domain '%s': %d\n", rpd->name, ret);
	}

	return 0;
}

static int rpi_power_remove(struct platform_device *pdev)
{
	struct rpi_power_domain *rpd = platform_get_drvdata(pdev);

	pm_genpd_remove_device(&pdev->dev);
	pm_genpd_remove(&rpd->genpd);

	if (rpd->chan)
		mbox_free_channel(rpd->chan);

	return 0;
}


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
	},
};
module_platform_driver(rpi_power_driver);

MODULE_AUTHOR("ChatGPT + Richard");
MODULE_DESCRIPTION("Raspberry Pi firmware power domain via ACPI");
MODULE_LICENSE("GPL");
