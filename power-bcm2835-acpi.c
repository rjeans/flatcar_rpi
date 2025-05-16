
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



struct rpi_firmware_power_msg {
	u32 size;           // Total size of the buffer in bytes
	u32 code;           // Request code (0 = process request)

	struct {
		u32 tag;        // Tag ID (0x00028001 = set power state)
		u32 buf_size;   // Size of the value buffer (8)
		u32 val_len;    // Length of the actual value data (8)
		u32 domain;     // Power domain ID (e.g., 0x00000000 = SD card)
		u32 state;      // Bit 0: 1 = ON, Bit 1: 1 = WAIT
	} __packed body;

	u32 end_tag;        // 0
} __packed;



#define RPI_FIRMWARE_POWER_DOMAIN_PWM 0x000000008


#define POWER_DOMAIN_ON     0x03
#define POWER_DOMAIN_OFF    0x02

struct rpi_power_domain {
	struct mbox_chan *chan;
	struct mbox_client mbox_client;
	const char *name;
	struct completion tx_done;
	u32 fw_domain_id;
};

static int rpi_power_send(struct rpi_power_domain *rpd, bool enable)
{
	struct mbox_chan *chan = rpd->chan;
	struct device *dev = rpd->mbox_client.dev;
	int ret;

	struct rpi_firmware_power_msg msg;

	if (!chan) {
		dev_err(dev, "Cannot send message: mailbox channel is NULL\n");
		return -ENODEV;
	}

	memset(&msg, 0, sizeof(msg)); // Ensure it's clean

	msg.size = sizeof(msg);
	msg.code = 0;  // process request

	msg.body.tag = RPI_FIRMWARE_SET_POWER_STATE;
	msg.body.buf_size = 8;
	msg.body.val_len = 8;
	msg.body.domain = rpd->fw_domain_id;
	msg.body.state = enable ? 3 : 0; // bit 0 = ON, bit 1 = WAIT

	msg.end_tag = 0;

	dev_info(dev, "Sending firmware power %s for domain '%s' (domain_id=0x%08x)\n",
	         enable ? "ON" : "OFF", rpd->name, rpd->fw_domain_id);

	dev_info(dev, "msg ptr: %p\n",  &msg);
	dev_info(dev, "msg->size = 0x%08x", msg.size);
	dev_info(dev, "msg->code = 0x%08x", msg.code);
	dev_info(dev, "msg->body.tag = 0x%08x", msg.body.tag);
	dev_info(dev, "msg->body.buf_size = 0x%08x", msg.body.buf_size);
	dev_info(dev, "msg->body.val_len = 0x%08x", msg.body.val_len);
	dev_info(dev, "msg->body.domain = 0x%08x", msg.body.domain);
	dev_info(dev, "msg->body.state = 0x%08x", msg.body.state);
	dev_info(dev, "msg->end_tag = 0x%08x", msg.end_tag);

	reinit_completion(&rpd->tx_done);

	ret = mbox_send_message(chan, &msg);
	if (ret < 0) {
		dev_err(dev, "Failed to send mailbox message: %d\n", ret);
		return ret;
	}



	dev_info(dev, "Firmware mailbox power message completed successfully\n");
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
    rpd->fw_domain_id = RPI_FIRMWARE_POWER_DOMAIN_PWM;
	rpd->mbox_client.dev = dev;
	rpd->mbox_client.tx_block = true;
	rpd->mbox_client.knows_txdone = true;
	rpd->mbox_client.tx_done = rpi_power_tx_done;
	
	rpd->chan = bcm2835_get_mbox_chan(&rpd->mbox_client);
	if (IS_ERR(rpd->chan)) {
		dev_err(dev, "Failed to get mailbox channel\n");
		return PTR_ERR(rpd->chan);
	}
  
	ret=bcm2835_register_client(&rpd->mbox_client);
	if (ret) {
		dev_err(dev, "Failed to register mailbox client: %d\n", ret);
		return ret;
	}
 

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