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

struct mbox_chan *rpi_acpi_find_mbox_channel(struct device *dev)
{
	struct fwnode_handle *mbox_fwnode;
	struct mbox_chan *chan;

	// Read mbox reference from ACPI _DSD
	mbox_fwnode = fwnode_find_reference(dev_fwnode(dev), "mbox", 0);
	if (IS_ERR(mbox_fwnode)) {
		dev_err(dev, "Failed to find ACPI mailbox fwnode\n");
		return ERR_CAST(mbox_fwnode);
	}

	// Get mbox controller device from the fwnode
	struct device *mbox_dev = bus_find_device_by_fwnode(&platform_bus_type, mbox_fwnode);
	fwnode_handle_put(mbox_fwnode);
	if (!mbox_dev) {
		dev_err(dev, "Failed to find mailbox device\n");
		return ERR_PTR(-ENODEV);
	}

	// Get mbox controller instance
	struct mbox_controller *mbox_ctrl = dev_get_drvdata(mbox_dev);
	if (!mbox_ctrl) {
		dev_err(dev, "Mailbox controller not ready\n");
		put_device(mbox_dev);
		return ERR_PTR(-ENODEV);
	}

	// Access the first channel using mbox_client
	chan = mbox_request_channel(&mbox_ctrl->client, 0);
	if (IS_ERR(chan)) {
		dev_err(dev, "Failed to request mailbox channel\n");
		put_device(mbox_dev);
		return chan;
	}

	put_device(mbox_dev);
	return chan;
}

#define POWER_DOMAIN_ON     0x03  // ON (bit 0) + WAIT (bit 1)
#define POWER_DOMAIN_OFF    0x02  // OFF (bit 0 clear) + WAIT (bit 1)

struct rpi_power_domain {
	struct generic_pm_domain genpd;
	struct mbox_client mbox_client;
	struct mbox_chan *chan;
	const char *name;
};

static int rpi_power_send(struct rpi_power_domain *rpd, bool enable)
{
	struct device *dev = rpd->mbox_client.dev;
	u32 msg;

	msg = (enable ? POWER_DOMAIN_ON : POWER_DOMAIN_OFF);
	dev_info(dev, "Sending firmware power %s for domain '%s': 0x%08X\n",
	         enable ? "ON" : "OFF", rpd->name, msg);

	return mbox_send_message(rpd->chan, &msg);
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

static int rpi_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpi_power_domain *rpd;
	struct genpd_onecell_data *genpd_data;
	u32 active;
	int ret;

	dev_info(dev, "Probing raspberrypi-power ACPI driver\n");

	rpd = devm_kzalloc(dev, sizeof(*rpd), GFP_KERNEL);
	if (!rpd)
		return -ENOMEM;

	// Required: name of the firmware power domain ("pwm", "vec", etc.)
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
	rpd->mbox_client.knows_txdone = false;

    rpd->chan = rpi_acpi_find_mbox_channel(dev);	if (IS_ERR(rpd->chan)) {
		ret = PTR_ERR(rpd->chan);
		dev_err(dev, "Failed to acquire mailbox channel: %d\n", ret);
		return ret;
	}
	dev_info(dev, "Mailbox channel acquired\n");

	// Configure GENPD
	rpd->genpd.name = rpd->name;
	rpd->genpd.power_on = rpi_power_on;
	rpd->genpd.power_off = rpi_power_off;
	rpd->genpd.flags = GENPD_FLAG_PM_CLK | GENPD_FLAG_ALWAYS_ON;

	// Register as a single power domain (onecell)
	genpd_data = devm_kzalloc(dev, sizeof(*genpd_data), GFP_KERNEL);
	if (!genpd_data)
		return -ENOMEM;

	genpd_data->domains = devm_kzalloc(dev, sizeof(struct generic_pm_domain *), GFP_KERNEL);
	if (!genpd_data->domains)
		return -ENOMEM;

	genpd_data->num_domains = 1;
	genpd_data->domains[0] = &rpd->genpd;

	ret = of_genpd_add_provider_onecell(dev->of_node, genpd_data);
	if (ret)
		dev_warn(dev, "GENPD registration failed: %d (continuing)\n", ret);

	// Power on immediately if requested
	if (active) {
		dev_info(dev, "Powering on domain '%s' (rpi,active = 1)\n", rpd->name);
		ret = rpi_power_send(rpd, true);
		if (ret)
			dev_err(dev, "Failed to power on domain '%s': %d\n", rpd->name, ret);
	}

	return 0;
}

static const struct acpi_device_id rpi_power_acpi_ids[] = {
	{ "BCM2851", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rpi_power_acpi_ids);

static struct platform_driver rpi_power_driver = {
	.probe = rpi_power_probe,
	.driver = {
		.name = "raspberrypi-power-acpi",
		.acpi_match_table = rpi_power_acpi_ids,
	},
};
module_platform_driver(rpi_power_driver);

MODULE_AUTHOR("ChatGPT + Richard");
MODULE_DESCRIPTION("Raspberry Pi firmware power domain via ACPI");
MODULE_LICENSE("GPL");
