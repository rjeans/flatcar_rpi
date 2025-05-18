
// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <soc/bcm2835/raspberrypi-firmware.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/pm_domain.h>


#include "mailbox-bcm2835-acpi.h"


#define RPI_POWER_ON    0x00000001
#define RPI_WAIT        0x00000002




#define RPI_FIRMWARE_POWER_DOMAIN_PWM 0x000000008


#define POWER_DOMAIN_ON     0x03
#define POWER_DOMAIN_OFF    0x02

struct rpi_power_domain {
	struct mbox_chan *chan;
	struct mbox_client mbox_client;
	const char *name;
	struct completion tx_done;
	bool completed;
	u32 domain_id;
	struct rpi_firmware_power_msg *msg;
};

static int rpi_power_send(struct rpi_power_domain *rpd, bool on)
{
	struct device *dev = rpd->mbox_client.dev;
	int ret;

	// Allocate DMA-coherent buffer
	rpd->msg = kzalloc(sizeof(*rpd->msg), GFP_KERNEL);

	if (!rpd->msg)
		return -ENOMEM;

	
    dev_info(dev,"Sending power message to firmware: %s\n", on ? "ON" : "OFF");
	dev_info(dev, "power_send: DMA buffer: msg=%px \n",
                 rpd->msg);

	memset(rpd->msg, 0, sizeof(*rpd->msg));

	// Construct mailbox firmware message
	rpd->msg->size = sizeof(*rpd->msg);
	rpd->msg->code = 0;
	rpd->msg->body.tag = RPI_FIRMWARE_SET_POWER_STATE;
	rpd->msg->body.buf_size = 8;
	rpd->msg->body.val_len = 8;
	rpd->msg->body.domain = rpd->domain_id;
	rpd->msg->body.state = on ? RPI_POWER_ON | RPI_WAIT : 0;
	rpd->msg->end_tag = 0;

	// Reset state
	dev_info(dev, "Resetting tx_done completion\n");
	reinit_completion(&rpd->tx_done);
	rpd->completed = false;

	// Send mailbox message
	dev_info(dev, "Sending firmware power domain message: %s\n", on ? "ON" : "OFF");
	dev_info(dev, "msg->size = 0x%08x\n", rpd->msg->size);
	ret = mbox_send_message(rpd->chan, rpd->msg);
	dev_info(dev, "mbox_send_message() returned %d\n", ret);
	if (ret < 0) {
		kfree(rpd->msg);
		rpd->msg = NULL;
		return ret;
	}


	dev_info(dev, "Firmware power domain response received and rpi_power_send complete\n");
	return 0;
}

static int rpi_power_runtime_resume(struct device *dev)
{
	struct rpi_power_domain *rpd = dev_get_drvdata(dev);
	dev_info(dev, "Resuming power domain '%s'\n", rpd->name);

	return rpi_power_send(rpd, true);
}

static int rpi_power_runtime_suspend(struct device *dev)
{
	struct rpi_power_domain *rpd = dev_get_drvdata(dev);
	dev_info(dev, "Suspending power domain '%s'\n", rpd->name);

	return rpi_power_send(rpd, false);
}







static int rpi_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpi_power_domain *rpd;
	int ret;
	

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
	rpd->mbox_client.tx_block = true;        // Wait for completion inside mbox_send_message()
	rpd->mbox_client.knows_txdone = false;   // Let the controller (IRQ) notify tx completion
	rpd->mbox_client.tx_tout = 500;          // Timeout if IRQ doesn't arrive
	rpd->mbox_client.rx_callback = NULL;  // Optional if you process response
	rpd->mbox_client.tx_done = NULL;         // Optional if you don’t need extra tx-done logic

	pr_info("Requesting mailbox channel...\n");

	if (!global_rpi_mbox_chan) {
		dev_err(dev, "Mailbox channel not available\n");
		return -ENODEV;
	}

	rpd->chan = global_rpi_mbox_chan;
    if (IS_ERR(rpd->chan)) {
		dev_err(dev, "Failed to acquire mailbox channel\n");
		return PTR_ERR(rpd->chan);
	}

  


	ret = mbox_bind_client(rpd->chan, &rpd->mbox_client);
	if (ret) {
		dev_err(dev, "Failed to bind mailbox client: %d\n", ret);
		return ret;
	}


    dev_info(dev, "Mailbox channel address: %px\n", rpd->chan);


	
	

	init_completion(&rpd->tx_done);

	platform_set_drvdata(pdev, rpd);


	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	
	pm_genpd_init(&rpd->genpd, NULL, false);  // ← generic_pm_domain structure in your rpd struct
	

    rpd->genpd.name = rpd->name;

    dev_set_genpd_dev(&pdev->dev, &rpd->genpd);  // <-- key ACPI-safe registration
	

	dev_info(dev, "Power domain '%s' runtime PM ready\n", rpd->name);
	return 0;
}

static int rpi_power_remove(struct platform_device *pdev)
{
	struct rpi_power_domain *rpd = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	dev_info(dev, "Removing power domain '%s'\n", rpd->name);
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