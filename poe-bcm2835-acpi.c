// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include <linux/mailbox_client.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include "mailbox-bcm2835-acpi.h"

#define PWM_PERIOD_NS 80000 // 12.5kHz
#define PWM_MAX_DUTY 255

#define RPI_PWM_CUR_DUTY_REG         0x00000001
#define RPI_FIRMWARE_SET_POE_HAT_VAL 0x00038041
#define RPI_FIRMWARE_PROPERTY_END    0x00000000
#define RPI_FIRMWARE_STATUS_REQUEST  0x00000000
#define RPI_MBOX_CHAN_FIRMWARE       8

struct acpi_pwm_driver_data {
	struct pwm_chip chip;
	struct mbox_client mbox;
	struct mbox_chan *chan;
	struct device *dev;
	u8 last_duty;
};

static inline struct acpi_pwm_driver_data *to_acpi_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct acpi_pwm_driver_data, chip);
}

static int send_pwm_duty(struct device *dev, struct mbox_chan *chan, u8 duty)
{
	dma_addr_t dma_handle;
	u32 *buf;
	u32 msg;
	int ret;

	buf = dma_alloc_coherent(dev, 64, &dma_handle, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = cpu_to_le32(7 * sizeof(u32));          // buffer size in bytes
	buf[1] = cpu_to_le32(RPI_FIRMWARE_STATUS_REQUEST);
	buf[2] = cpu_to_le32(RPI_FIRMWARE_SET_POE_HAT_VAL);  // tag
	buf[3] = cpu_to_le32(8);                        // value buffer size
	buf[4] = cpu_to_le32(8);                        // request/response size
	buf[5] = cpu_to_le32(RPI_PWM_CUR_DUTY_REG);     // register
	buf[6] = cpu_to_le32(duty);                     // value
	buf[7] = cpu_to_le32(RPI_FIRMWARE_PROPERTY_END);

	wmb();

	msg = (u32)dma_handle | RPI_MBOX_CHAN_FIRMWARE;
	ret = mbox_send_message(chan, &msg);
	if (ret < 0) {
		dev_warn(dev, "mbox_send_message failed: %d (duty=%u)\n", ret, duty);
	} else {
		dev_info(dev, "PWM duty sent: %u\n", duty);
	}

	dma_free_coherent(dev, 64, buf, dma_handle);
	return ret;
}

static int acpi_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);
	u8 duty;
	int ret;

	dev_info(data->dev, "apply: period=%llu, duty_cycle=%llu, enabled=%d, polarity=%d\n",
		state->period, state->duty_cycle, state->enabled, state->polarity);

	if (state->period < PWM_PERIOD_NS || state->polarity != PWM_POLARITY_NORMAL) {
		dev_info(data->dev, "Invalid period or polarity\n");
		return -EINVAL;
	}

	if (!state->enabled || state->duty_cycle == 0) {
		duty = 0;
	} else if (state->duty_cycle >= PWM_PERIOD_NS) {
		duty = PWM_MAX_DUTY;
	} else {
		duty = DIV_ROUND_DOWN_ULL(state->duty_cycle * PWM_MAX_DUTY, PWM_PERIOD_NS);
	}

	if (duty == data->last_duty) {
		dev_info(data->dev, "Duty unchanged (%u), skipping\n", duty);
		return 0;
	}

	ret = send_pwm_duty(data->dev, data->chan, duty);
	if (ret) {
		dev_warn(data->dev, "Failed to send PWM duty: %d\n", ret);
		return ret;
	}

	dev_info(data->dev, "PWM duty updated: %u\n", duty);
	data->last_duty = duty;
	return 0;
}

static int acpi_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			      struct pwm_state *state)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

	state->period = PWM_PERIOD_NS;
	state->duty_cycle = DIV_ROUND_UP(data->last_duty * PWM_PERIOD_NS, PWM_MAX_DUTY);
	state->enabled = !!data->last_duty;
	state->polarity = PWM_POLARITY_NORMAL;

	dev_info(data->dev, "get_state: period=%llu, duty_cycle=%llu, enabled=%d, polarity=%d\n",
		state->period, state->duty_cycle, state->enabled, state->polarity);

	return 0;
}

static const struct pwm_ops acpi_pwm_ops = {
	.apply = acpi_pwm_apply,
	.get_state = acpi_pwm_get_state,
	.owner = THIS_MODULE,
};

static int acpi_pwm_enable_firmware(struct device *dev, struct mbox_chan *chan)
{
	dma_addr_t dma_handle;
	u32 *dma_buf;
	int ret;

	dma_buf = dma_alloc_coherent(dev, 32, &dma_handle, GFP_KERNEL);
	if (!dma_buf)
		return -ENOMEM;

	dma_buf[0] = 7 * sizeof(u32); // Total size
	dma_buf[1] = RPI_FIRMWARE_STATUS_REQUEST;
	dma_buf[2] = RPI_FIRMWARE_SET_POE_HAT_VAL;
	dma_buf[3] = 8;
	dma_buf[4] = 8;
	dma_buf[5] = 0x00000000;   // Tag = enable
	dma_buf[6] = 1;            // Value = enable
	dma_buf[7] = RPI_FIRMWARE_PROPERTY_END;

	wmb(); // Ensure DMA memory is visible

	ret = mbox_send_message(chan, &dma_handle);
	dev_info(dev, "Enable PoE logic message sent, ret = %d\n", ret);

	dma_free_coherent(dev, 32, dma_buf, dma_handle);
	return ret;
}


static int acpi_pwm_probe(struct platform_device *pdev)
{
	struct acpi_pwm_driver_data *data;
	struct mbox_client *cl;
    int ret;

	dev_info(&pdev->dev, "acpi_pwm_probe: probing device\n");

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;
	cl = &data->mbox;
	cl->dev = &pdev->dev;
	cl->tx_block = true;
	cl->knows_txdone = false;

	data->chan = bcm2835_mbox_request_channel(cl);
	if (IS_ERR(data->chan))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->chan), "mbox request failed\n");

    dev_info(&pdev->dev, "Mailbox channel startup\n");

    ret = acpi_pwm_enable_firmware(&pdev->dev, data->chan);
    if (ret < 0) {
        dev_warn(&pdev->dev, "PoE firmware enable failed: %d\n", ret);
    }

	data->chip.dev = &pdev->dev;
	data->chip.ops = &acpi_pwm_ops;
	data->chip.npwm = 1;

	platform_set_drvdata(pdev, data);

	return devm_pwmchip_add(&pdev->dev, &data->chip);
}

static const struct acpi_device_id acpi_pwm_ids[] = {
	{ "BCM2853", 0 },
	{}
};
MODULE_DEVICE_TABLE(acpi, acpi_pwm_ids);

static struct platform_driver acpi_pwm_driver = {
	.driver = {
		.name = "acpi-mailbox-pwm",
		.acpi_match_table = acpi_pwm_ids,
	},
	.probe = acpi_pwm_probe,
};

module_platform_driver(acpi_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("You");
MODULE_DESCRIPTION("ACPI PWM driver using mailbox to control firmware duty");
