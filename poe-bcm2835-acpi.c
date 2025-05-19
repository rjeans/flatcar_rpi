// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include <linux/mailbox_client.h>
#include <linux/delay.h>
#include "mailbox-bcm2835-acpi.h"

#define PWM_PERIOD_NS 80000 // 12.5kHz
#define PWM_MAX_DUTY 255
#define RPI_PWM_CUR_DUTY_REG 0x0


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

struct rpi_pwm_message {
	__le32 reg;
	__le32 val;
	__le32 ret;
};

static int send_pwm_duty(struct mbox_chan *chan, u8 duty)
{
	struct rpi_pwm_message msg = {
		.reg = cpu_to_le32(RPI_PWM_CUR_DUTY_REG),
		.val = cpu_to_le32(duty),
	};

    struct device *dev = chan->cl ? chan->cl->dev : NULL;


	int ret = mbox_send_message(chan, &msg);
	if (ret < 0) {
		if (dev)
			dev_info(dev, "mbox_send_message failed: %d (duty=%u)\n", ret, duty);
		return ret;
	}
    
    dev_info(dev, "Sent PWM duty %d, response ret=0x%x\n", duty, le32_to_cpu(msg.ret));
	// optional delay to allow hardware to settle
	usleep_range(1000, 2000);

	if (chan->cl && chan->cl->dev)
		dev_info(chan->cl->dev, "PWM duty sent: %u\n", duty);

	return 0;
}

static int acpi_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);
	u8 duty;
	int ret;

	dev_info(data->dev, "acpi_pwm_apply: period=%llu, duty_cycle=%llu, enabled=%d, polarity=%d\n",
		state->period, state->duty_cycle, state->enabled, state->polarity);

	if (state->period < PWM_PERIOD_NS ||
	    state->polarity != PWM_POLARITY_NORMAL) {
		dev_info(data->dev, "Invalid period or polarity\n");
		return -EINVAL;
	}

	if (!state->enabled || state->duty_cycle == 0) {
		duty = 0;
	} else if (state->duty_cycle >= PWM_PERIOD_NS) {
		duty = PWM_MAX_DUTY;
	} else {
		duty = DIV_ROUND_CLOSEST(state->duty_cycle * PWM_MAX_DUTY, PWM_PERIOD_NS);
	}

	if (duty == data->last_duty) {
		dev_info(data->dev, "Duty unchanged (%u), skipping\n", duty);
		return 0;
	}

	ret = send_pwm_duty(data->chan, duty);
	if (ret) {
		dev_info(data->dev, "Failed to send PWM duty: %d\n", ret);
		return dev_err_probe(data->dev, ret, "Failed to send PWM duty\n");
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

	dev_info(data->dev, "acpi_pwm_get_state: period=%llu, duty_cycle=%llu, enabled=%d, polarity=%d\n",
		state->period, state->duty_cycle, state->enabled, state->polarity);

    return 0;
}

static const struct pwm_ops acpi_pwm_ops = {
	.apply = acpi_pwm_apply,
	.get_state = acpi_pwm_get_state,
	.owner = THIS_MODULE,
};

static int acpi_pwm_probe(struct platform_device *pdev)
{
	struct acpi_pwm_driver_data *data;
	struct mbox_client *cl;

	dev_info(&pdev->dev, "acpi_pwm_probe: probing device\n");

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_info(&pdev->dev, "Failed to allocate driver data\n");
		return -ENOMEM;
	}

    data->dev = &pdev->dev;

	cl = &data->mbox;
	cl->dev = &pdev->dev;
	cl->tx_block = true;
	cl->knows_txdone = false;

	data->chan = bcm2835_mbox_request_channel(cl);
	if (IS_ERR(data->chan)) {
		dev_info(&pdev->dev, "mbox request failed: %ld\n", PTR_ERR(data->chan));
		return dev_err_probe(&pdev->dev, PTR_ERR(data->chan), "mbox request failed\n");
	}

	data->chip.dev = &pdev->dev;
	data->chip.ops = &acpi_pwm_ops;
	data->chip.npwm = 1;

	platform_set_drvdata(pdev, data);

	dev_info(&pdev->dev, "Registering PWM chip\n");
	return devm_pwmchip_add(&pdev->dev, &data->chip);
}

static const struct acpi_device_id acpi_pwm_ids[] = {
	{ "BCM2853", 0 }, // match _HID in ACPI SSDT
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

static int __init acpi_pwm_init(void)
{
	pr_info("acpi-mailbox-pwm: module init\n");
	return platform_driver_register(&acpi_pwm_driver);
}

static void __exit acpi_pwm_exit(void)
{
	pr_info("acpi-mailbox-pwm: module exit\n");
	platform_driver_unregister(&acpi_pwm_driver);
}

module_init(acpi_pwm_init);
module_exit(acpi_pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("You");
MODULE_DESCRIPTION("ACPI PWM driver using mailbox to control firmware duty");
