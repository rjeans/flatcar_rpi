
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
	dma_addr_t dma_handle;
};

static int rpi_power_send(struct rpi_power_domain *rpd, bool on)
{
	struct device *dev = rpd->mbox_client.dev;
	struct rpi_firmware_power_msg *msg;
	int ret;

	// Allocate DMA-coherent buffer
	msg = dma_alloc_coherent(dev, sizeof(*msg), &rpd->dma_handle, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	rpd->msg = msg;
    dev_info(dev,"Sending power message to firmware: %s\n", on ? "ON" : "OFF");
	dev_info(dev, "DMA alloc: msg=%px dma_handle=0x%pad\n", msg, &rpd->dma_handle);

	memset(msg, 0, sizeof(*msg));

	// Construct mailbox firmware message
	msg->size = sizeof(*msg);
	msg->code = 0;
	msg->body.tag = RPI_FIRMWARE_SET_POWER_STATE;
	msg->body.buf_size = 8;
	msg->body.val_len = 8;
	msg->body.domain = rpd->domain_id;
	msg->body.state = on ? RPI_POWER_ON | RPI_WAIT : 0;
	msg->end_tag = 0;

	// Reset state
	reinit_completion(&rpd->tx_done);
	rpd->completed = false;

	// Send mailbox message
	ret = mbox_send_message(rpd->chan, rpd->msg);
	if (ret < 0) {
		dma_free_coherent(dev, sizeof(*msg), msg, rpd->dma_handle);
		rpd->msg = NULL;
		return ret;
	}

	// Wait for firmware response (polling will call complete)
	if (!wait_for_completion_timeout(&rpd->tx_done, msecs_to_jiffies(100))) {
		dev_err(dev, "Timeout waiting for firmware power domain response\n");
		// Do NOT free memory here — tx_done() handles it (or doesn't)
		return -ETIMEDOUT;
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

static void rpi_power_tx_done(struct mbox_client *cl, void *msg, int r)
{
    struct rpi_power_domain *rpd = dev_get_drvdata(cl->dev);
    pr_info("Received firmware power message response: %d completed: %u\n", r, rpd->completed);
	dev_info(cl->dev, "DMA free:  msg=%px dma_handle=0x%pad\n", rpd->msg, &rpd->dma_handle);

    if (rpd->completed)
        return;
    rpd->completed = true;

    dev_info(cl->dev, "Firmware power message completed successfully in tx_done");

    complete(&rpd->tx_done);  

    if (rpd->msg) {
 //       dma_free_coherent(cl->dev, sizeof(*rpd->msg), rpd->msg, rpd->dma_handle);
        rpd->msg = NULL;
    }
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
    rpd->domain_id = RPI_FIRMWARE_POWER_DOMAIN_PWM;
	rpd->mbox_client.dev = dev;
	rpd->mbox_client.tx_block = true;
	rpd->mbox_client.knows_txdone = true;


	rpd->mbox_client.tx_done = rpi_power_tx_done;
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