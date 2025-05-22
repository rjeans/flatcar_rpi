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

static DEFINE_MUTEX(transaction_lock);


#define PWM_PERIOD_NS 80000 // 12.5kHz
#define PWM_MAX_DUTY 255

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
    dev_info(data->dev, "RX callback invoked\n");
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
                             u32 property_tag, u32 reg,u32 value, bool is_get, u32 *value_out)
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

    

    dev_info(dev, "DMA buffer BEFORE: [%08x %08x %08x %08x %08x %08x %08x %08x %08x]\n",
        dma_buf[0], dma_buf[1], dma_buf[2], dma_buf[3],
        dma_buf[4], dma_buf[5], dma_buf[6], dma_buf[7],dma_buf[8]);

    wmb(); // Ensure DMA memory is visible to the firmware

    dev_info(dev, "------------------------  send_mbox_message: IN: Sending tag 0x%08x reg 0x%08x val %u\n",
         dma_buf[2], dma_buf[5], dma_buf[6]);


    mutex_lock(&transaction_lock);

    reinit_completion(c);

    u32 msg = MBOX_MSG(RPI_MBOX_CHAN_FIRMWARE, dma_handle);


    ret = mbox_send_message(chan, &msg);
    if (ret < 0) {
        dev_err(dev, "send_mbox_message: Failed to send message: %pe\n", ERR_PTR(ret));
        goto out_free;
    }

    if (!wait_for_completion_timeout(c, HZ)) {
        dev_err(dev, "send_mbox_message: Timeout waiting for response\n");
        ret = -ETIMEDOUT;
        goto out_free;
    }

    if (!(dma_buf[4] & 0x80000000)) { // Check response success bit
        dev_err(dev, "send_mbox_message: Firmware did not acknowledge property tag 0x%08x\n", property_tag);
        ret = -EIO;
        goto out_free;
    }

    if (is_get && value_out)
        *value_out = le32_to_cpu(dma_buf[6]);

    ret = 0;

out_free:

    dev_info(dev, "DMA buffer AFTER: [%08x %08x %08x %08x %08x %08x %08x %08x %08x]\n",
        dma_buf[0], dma_buf[1], dma_buf[2], dma_buf[3],
        dma_buf[4], dma_buf[5], dma_buf[6], dma_buf[7],dma_buf[8]);


    mutex_unlock(&transaction_lock);

   dev_info(dev, "------------------------  send_mbox_message: OUT: Sending tag 0x%08x reg 0x%08x val %u\n",
         dma_buf[2], dma_buf[5], dma_buf[6]);

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



static int acpi_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
               const struct pwm_state *state)
{
    struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);
    int ret;
    unsigned new_scaled_duty_cycle,updated_scaled_duty_cycle;

    dev_info(data->dev, "acpi_pwm_apply: called for pwm=%u\n", pwm->hwpwm);
    dev_info(data->dev, "acpi_pwm_apply: input state: period=%llu, duty_cycle=%llu, enabled=%d, polarity=%d, scaled duty cycle=%d\n",
        state->period, state->duty_cycle, state->enabled, state->polarity,data->scaled_duty_cycle);

    if (state->period < PWM_PERIOD_NS) {
        dev_info(data->dev, "acpi_pwm_apply: Invalid period (%llu < %u)\n", state->period, PWM_PERIOD_NS);
        return -EINVAL;
    }
    if (state->polarity != PWM_POLARITY_NORMAL) {
        dev_info(data->dev, "acpi_pwm_apply: Unsupported polarity (%d)\n", state->polarity);
        return -EINVAL;
    }

    if (!state->enabled) {
        new_scaled_duty_cycle = 0;
        dev_info(data->dev, "acpi_pwm_apply: PWM disabled, setting duty_cycle=0\n");
    } else if (state->duty_cycle < RPI_PWM_PERIOD_NS) {
        new_scaled_duty_cycle = DIV_ROUND_DOWN_ULL(state->duty_cycle * RPI_PWM_MAX_DUTY,
                        RPI_PWM_PERIOD_NS);
        dev_info(data->dev, "acpi_pwm_apply: Calculated duty_cycle=%u from duty_cycle=%llu, period=%llu\n",
            new_scaled_duty_cycle, state->duty_cycle, state->period);
    } else {
        data->scaled_duty_cycle = RPI_PWM_MAX_DUTY;
        dev_info(data->dev, "acpi_pwm_apply: duty_cycle >= period, setting duty_cycle=RPI_PWM_MAX_DUTY (%u)\n",
            RPI_PWM_MAX_DUTY);
    }

    dev_info(data->dev, "acpi_pwm_apply: Current stored duty_cycle=%u, new duty_cycle=%u\n",
        data->scaled_duty_cycle, new_scaled_duty_cycle);

    if (new_scaled_duty_cycle == data->scaled_duty_cycle) {
        dev_info(data->dev, "acpi_pwm_apply: No change in duty_cycle, skipping update\n");
        return 0;
    }

    
    dev_info(data->dev, "acpi_pwm_apply: Sending new scaled_duty_cycle=%u to firmware\n", new_scaled_duty_cycle);
    ret = send_pwm_duty(&data->c, data->dev, data->chan, new_scaled_duty_cycle);
    if (ret) {
        dev_warn(data->dev, "acpi_pwm_apply: Failed to send PWM duty: %d\n", ret);
        return ret;
    }

    ret = get_pwm_duty(&data->c,data->dev, data->chan
                                      , &updated_scaled_duty_cycle);
    if (ret < 0) {  
        dev_warn(data->dev, "Failed to get current duty cycle: %d\n", ret);
        return ret;
    }
    dev_info(data->dev, "Stored duty cycle: %u\n", updated_scaled_duty_cycle);

    dev_info(data->dev, "acpi_pwm_apply: PWM duty updated successfully to %u\n", new_scaled_duty_cycle);
    data->scaled_duty_cycle = new_scaled_duty_cycle;


    


    return 0;
}

static int acpi_pwm_get_state(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     struct pwm_state *state)
{
	struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

    dev_info(data->dev, "get_state BEFORE: period=%llu, duty_cycle=%llu, enabled=%d, polarity=%d (scaled=%d)\n",
		state->period, state->duty_cycle, state->enabled, state->polarity,data->scaled_duty_cycle);


	state->period = RPI_PWM_PERIOD_NS;
	state->duty_cycle = DIV_ROUND_UP(data->scaled_duty_cycle * RPI_PWM_PERIOD_NS,
					 RPI_PWM_MAX_DUTY);
	state->enabled = !!(data->scaled_duty_cycle);
	state->polarity = PWM_POLARITY_NORMAL;



	dev_info(data->dev, "get_state AFTER: period=%llu, duty_cycle=%llu, enabled=%d, polarity=%d (scaled=%d)\n",
		state->period, state->duty_cycle, state->enabled, state->polarity,data->scaled_duty_cycle);

	return 0;
}

static int acpi_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
    struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

    dev_info(data->dev, "acpi_pwm_request: requested PWM device %u\n", pwm->hwpwm);
    return 0;
}

static void acpi_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
    struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

    dev_info(data->dev, "acpi_pwm_free: freeing PWM device %u\n", pwm->hwpwm);
}
static int acpi_pwm_capture(struct pwm_chip *chip, struct pwm_device *pwm,
                 struct pwm_capture *capture,long unsigned int timeout)
{
    struct acpi_pwm_driver_data *data = to_acpi_pwm(chip);

    dev_info(data->dev, "acpi_pwm_capture: capturing PWM device %u\n", pwm->hwpwm);
    return acpi_pwm_get_state(chip, pwm, state);
}

static const struct pwm_ops acpi_pwm_ops = {
	.apply = acpi_pwm_apply,
	.get_state = acpi_pwm_get_state,
    .request = acpi_pwm_request,
    .free = acpi_pwm_free,
    .capture = acpi_pwm_capture,
	.owner = THIS_MODULE,
};

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
	cl->rx_callback = response_callback;

    init_completion(&data->c);

	data->chan = bcm2835_mbox_request_firmware_channel(cl);
	if (IS_ERR(data->chan))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->chan), "mbox request failed\n");

    dev_info(&pdev->dev, "Mailbox channel startup\n");

 

    ret = get_pwm_duty(&data->c,&pdev->dev, data->chan, &data->scaled_duty_cycle);
    if (ret < 0) {  
        dev_warn(&pdev->dev, "Failed to get current duty cycle: %d\n", ret);
    }
    dev_info(&pdev->dev, "Current scaled duty cycle: %u\n", data->scaled_duty_cycle);

	data->chip.dev = &pdev->dev;
	data->chip.ops = &acpi_pwm_ops;
	data->chip.npwm = 1;

	platform_set_drvdata(pdev, data);

	return devm_pwmchip_add(&pdev->dev, &data->chip);
}

static int acpi_pwm_remove(struct platform_device *pdev)
{
    struct acpi_pwm_driver_data *data = platform_get_drvdata(pdev);

    int ret = send_pwm_duty(&data->c, data->dev, data->chan, 0);
    if (ret) {
        dev_warn(data->dev, "acpi_pwm_apply: Failed to send PWM duty: %d\n", ret);
        return ret;
    }

    if (data->chan) {
        bcm2835_mbox_free_channel(data->chan);
    }

    
	dev_info(&pdev->dev, "acpi_pwm_remove: mailbox channel freed\n");
    return 0;
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
    .remove = acpi_pwm_remove,
};

module_platform_driver(acpi_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("You");
MODULE_DESCRIPTION("ACPI PWM driver using mailbox to control firmware duty");
