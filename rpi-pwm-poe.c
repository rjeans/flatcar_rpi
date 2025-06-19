// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include <linux/mailbox_client.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include "rpi-mailbox.h"

static DEFINE_MUTEX(transaction_lock);




#define RPI_MBOX_CHAN_FIRMWARE       8

#define RPI_PWM_MAX_DUTY		255
#define RPI_PWM_PERIOD_NS		80000 /* 12.5 kHz */

#define MBOX_MSG(chan, data28)		(((data28) & ~0xf) | ((chan) & 0xf))

struct acpi_pwm_driver_data {
	struct pwm_chip chip;
	struct mbox_client mbox;
	struct mbox_chan *chan;
	struct device *dev;
    struct completion c;
	unsigned int scaled_duty_cycle;
    struct pwm_state state;
};

static inline struct acpi_pwm_driver_data *to_acpi_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct acpi_pwm_driver_data, chip);
}

static void response_callback(struct mbox_client *cl, void *msg)
{
	struct acpi_pwm_driver_data *data = container_of(cl, struct acpi_pwm_driver_data, mbox);
	complete(&data->c);
}


#define RPI_FIRMWARE_GET_POE_HAT_VAL    0x00030049
#define RPI_FIRMWARE_SET_POE_HAT_VAL    0x00038049
#define RPI_FIRMWARE_STATUS_REQUEST 0x00000000

#define RPI_PWM_CUR_DUTY_REG         0x0
#define RPI_PWM_CUR_ENABLE_REG         0x0

#
// Sub-registers used inside the payload


static int build_poe_firmware_msg(u32 *buf,
                                  bool is_get,
                                  u32 property_tag,
                                  u32 reg,
                                  u32 value)
{
	if (!buf)
		return -EINVAL;

	buf[0] = cpu_to_le32(9 * sizeof(u32));      // total size: 32 bytes
	buf[1] = cpu_to_le32(RPI_FIRMWARE_STATUS_REQUEST);           // request
	buf[2] = cpu_to_le32(property_tag);   // compound PoE property tag
	buf[3] = cpu_to_le32(3*sizeof(u32));                    // tag payload size
	buf[4] = cpu_to_le32(0);       // 0 for GET, 0 for SET
	buf[5] = cpu_to_le32(reg);               // register to read or write
	buf[6] = cpu_to_le32(value);                // value (unused for GET)
	buf[7] = cpu_to_le32(0);                    
	buf[8] = cpu_to_le32(0);                    

	return 0;
}

static int send_mbox_message(struct completion *c, struct device *dev, struct mbox_chan *chan,
                             u32 property_tag, u32 reg, u32 value, bool is_get, u32 *value_out)
{
    dma_addr_t dma_handle;
    u32 *dma_buf;
    int ret;

 
    dma_buf = dma_alloc_coherent(chan->mbox->dev, PAGE_ALIGN(9 * sizeof(u32)), &dma_handle, GFP_ATOMIC);
    if (!dma_buf) {
        dev_err(dev, "send_mbox_message: Failed to allocate DMA buffer\n");
        return -ENOMEM;
    }

    ret = build_poe_firmware_msg(dma_buf, is_get, property_tag, reg, value);

    

    dev_dbg(dev, "Sending tag 0x%08x reg 0x%08x val %u\n", dma_buf[2], dma_buf[5], dma_buf[6]);


    mutex_lock(&transaction_lock);

    reinit_completion(c);

    u32 msg = MBOX_MSG(RPI_MBOX_CHAN_FIRMWARE, dma_handle);


    ret = mbox_send_message(chan, &msg);
    if (ret < 0) {
        dev_err(dev, "send_mbox_message: Failed to send message: %pe\n", ERR_PTR(ret));
        goto out_free;
    }

    if (!wait_for_completion_timeout(c, HZ)) {
        dev_err(dev, "Timeout waiting for response\n");
        ret = -ETIMEDOUT;
        goto out_free;
    }

    if (!(dma_buf[4] & 0x80000000)) {
        dev_err(dev, "Firmware did not acknowledge property tag 0x%08x\n", property_tag);
        ret = -EIO;
        goto out_free;
    }

    if (is_get && value_out)
        *value_out = le32_to_cpu(dma_buf[6]);

    ret = 0;

out_free:
	mutex_unlock(&transaction_lock);
	dma_free_coherent(chan->mbox->dev, PAGE_ALIGN(7 * sizeof(u32)), dma_buf, dma_handle);

	return ret;
}

static int send_pwm_duty(struct completion *c, struct device *dev, struct mbox_chan *chan, u8 duty)
{
    return send_mbox_message(c, dev, chan, RPI_FIRMWARE_SET_POE_HAT_VAL, RPI_PWM_CUR_DUTY_REG, duty, false, NULL);
}


static int get_pwm_duty(struct completion *c, struct device *dev, struct mbox_chan *chan, u32 *value_out)
{
    return send_mbox_message(c, dev, chan, RPI_FIRMWARE_GET_POE_HAT_VAL, RPI_PWM_CUR_DUTY_REG,0, true, value_out);
}



static int rpi_pwm_poe_apply(struct pwm_chip *chip, struct pwm_device *pwm,
                             const struct pwm_state *state)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);
	int ret;
	unsigned new_scaled_duty_cycle;

	// Validate the PWM state
	if (state->period != RPI_PWM_PERIOD_NS || state->polarity != PWM_POLARITY_NORMAL) {
		return -EINVAL;
	}

	data->state = *state;

	// Calculate the new scaled duty cycle
	if (!state->enabled) {
		new_scaled_duty_cycle = 0;
	} else if (state->duty_cycle < RPI_PWM_PERIOD_NS) {
		new_scaled_duty_cycle = DIV_ROUND_DOWN_ULL(state->duty_cycle * RPI_PWM_MAX_DUTY, RPI_PWM_PERIOD_NS);
	} else {
		new_scaled_duty_cycle = RPI_PWM_MAX_DUTY;
	}

	// Skip updating if the duty cycle hasn't changed
	if (new_scaled_duty_cycle == data->scaled_duty_cycle) {
		return 0;
	}

	// Send the new duty cycle to the firmware
	ret = send_pwm_duty(&data->c, data->dev, data->chan, new_scaled_duty_cycle);
	if (ret) {
		return ret;
	}

	data->scaled_duty_cycle = new_scaled_duty_cycle;
	return 0;
}

static int rpi_pwm_poe_get_state(struct pwm_chip *chip,
                                 struct pwm_device *pwm,
                                 struct pwm_state *state)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

	// Populate the PWM state with the current values
	state->period = RPI_PWM_PERIOD_NS;
	state->polarity = PWM_POLARITY_NORMAL;
	state->duty_cycle = data->state.duty_cycle;
	state->enabled = data->state.enabled;

	return 0;
}

static int rpi_pwm_poe_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	// No special handling required for requesting a PWM device
	return 0;
}

static void rpi_pwm_poe_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

	// Reset the PWM state to disabled
	data->state.period = RPI_PWM_PERIOD_NS;
	data->state.duty_cycle = 0;
	data->state.enabled = false;
	data->state.polarity = PWM_POLARITY_NORMAL;

	rpi_pwm_poe_apply(chip, pwm, &data->state);
}

static int rpi_pwm_poe_capture(struct pwm_chip *chip, struct pwm_device *pwm,
                               struct pwm_capture *capture, unsigned long timeout)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

	// Return the cached period and duty cycle
	capture->period = data->state.period;
	capture->duty_cycle = data->state.duty_cycle;

	return 0;
}

static const struct pwm_ops rpi_pwm_poe_ops = {
	.apply = rpi_pwm_poe_apply,
	.get_state = rpi_pwm_poe_get_state,
	.request = rpi_pwm_poe_request,
	.free = rpi_pwm_poe_free,
	.capture = rpi_pwm_poe_capture,
	.owner = THIS_MODULE,
};

static int rpi_pwm_poe_probe(struct platform_device *pdev)
{
	struct acpi_pwm_driver_data *data;
	struct mbox_client *cl;
	int ret;

	// Check if CONFIG_PWM is enabled
#ifndef CONFIG_PWM
	dev_err(&pdev->dev, "CONFIG_PWM is not enabled. Cannot initialize rpi-pwm-poe.\n");
	return -ENODEV;
#endif

	// Log the start of the probe function
	dev_info(&pdev->dev, "Probing rpi-pwm-poe device\n");

	// Allocate memory for the driver data
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate memory for driver data\n");
		return -ENOMEM;
	}

	data->dev = &pdev->dev;
	cl = &data->mbox;
	cl->dev = &pdev->dev;
	cl->tx_block = true;
	cl->rx_callback = response_callback;

	init_completion(&data->c);

	// Request the firmware mailbox channel
	data->chan = rpi_mbox_request_firmware_channel(cl);
	if (IS_ERR(data->chan)) {
		ret = PTR_ERR(data->chan);
		dev_err(&pdev->dev, "Failed to request firmware mailbox channel: %d\n", ret);
		return ret;
	}

	// Get the current duty cycle from the firmware
	ret = get_pwm_duty(&data->c, &pdev->dev, data->chan, &data->scaled_duty_cycle);
	if (ret < 0) {
		dev_warn(&pdev->dev, "Failed to get current duty cycle: %d\n", ret);
	}

	// Initialize the PWM state
	data->state.period = RPI_PWM_PERIOD_NS;
	data->state.duty_cycle = 0;
	data->state.enabled = false;
	data->state.polarity = PWM_POLARITY_NORMAL;

	// Initialize the PWM chip
	data->chip.dev = &pdev->dev;
	data->chip.ops = &rpi_pwm_poe_ops;
	data->chip.npwm = 1;

	platform_set_drvdata(pdev, data);

	// Register the PWM chip
	ret = devm_pwmchip_add(&pdev->dev, &data->chip);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register PWM chip: %d\n", ret);
		rpi_mbox_free_channel(data->chan);
		return ret;
	}

	dev_info(&pdev->dev, "rpi-pwm-poe device initialized successfully\n");
	return 0;
}

static int rpi_pwm_poe_remove(struct platform_device *pdev)
{
	struct acpi_pwm_driver_data *data = platform_get_drvdata(pdev);
	int ret;

	// Log the start of the remove function
	dev_info(&pdev->dev, "Removing rpi-pwm-poe device\n");

	// Reset the duty cycle to 0
	ret = send_pwm_duty(&data->c, data->dev, data->chan, 0);
	if (ret) {
		dev_warn(data->dev, "Failed to send PWM duty: %d\n", ret);
		return ret;
	}

	// Free the mailbox channel
	if (data->chan) {
		rpi_mbox_free_channel(data->chan);
	}

	return 0;
}

static const struct acpi_device_id rpi_pwm_poe_ids[] = {
	{ "POEF0001", 0 },
	{}
};
MODULE_DEVICE_TABLE(acpi, rpi_pwm_poe_ids);

static struct platform_driver rpi_pwm_poe_driver = {
	.driver = {
		.name = "rpi-pwm-poe",
		.acpi_match_table = rpi_pwm_poe_ids,
	},
	.probe = rpi_pwm_poe_probe,
	.remove = rpi_pwm_poe_remove,
};

module_platform_driver(rpi_pwm_poe_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("You");
MODULE_DESCRIPTION("ACPI PWM driver using mailbox to control firmware duty");
