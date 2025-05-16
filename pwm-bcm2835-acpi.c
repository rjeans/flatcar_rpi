// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2014 Richard Jeans <rich@jeansy.org>
 */


#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h> 
#include <linux/pinctrl/machine.h>
#include <linux/delay.h>
#include <soc/bcm2835/raspberrypi-firmware.h>
#include <linux/platform_device.h>


#define RPI_FIRMWARE_DOMAIN_PWM 2

#define PWM_CONTROL		0x000
#define PWM_CONTROL_SHIFT(x)	((x) * 8)
#define PWM_CONTROL_MASK	0xff
#define PWM_MODE		0x80		/* set timer in PWM mode */
#define PWM_ENABLE		(1 << 0)
#define PWM_POLARITY		(1 << 4)

#define PERIOD(x)		(((x) * 0x10) + 0x10)
#define DUTY(x)			(((x) * 0x10) + 0x14)

#define BCM2711_CLK_BASE 0xFE101000     // For Pi 4 

#define CM_PWMCTL  0xA0
#define CM_PWMDIV  0xA4

#define CM_PASSWD      0x5A000000
#define CM_ENABLE      (1 << 4)
#define CM_SRC_OSC     1           // Use 19.2 MHz oscillator

#define CM_BASE_PHYS     0xFE101000  // Clock manager base on Raspberry Pi 4
#define CM_PWMCTL        0xA0        // PWM control register offset
#define CM_PWMDIV        0xA4        // PWM divider register offset
#define CM_SRC_PLLD      0x6         // 500 MHz PLLD

#define FALLBACK_PWM_CLK_HZ (500000000 / 32) // 15.625 MHz

#define PERIOD_MIN		0x2

struct bcm2835_pwm {
	struct pwm_chip chip;
	struct device *dev;
	void __iomem *base;
	void __iomem *cm_base;
	unsigned long clk_rate;
	

};

static inline struct bcm2835_pwm *to_bcm2835_pwm(struct pwm_chip *chip)
{
        return container_of(chip, struct bcm2835_pwm, chip);
}



static const struct pinctrl_map bcm2835_pwm_map[] = {
    {
        .dev_name = "BCM2844:00",           // ACPI _HID of your PWM device
        .name = "default",
        .type = PIN_MAP_TYPE_MUX_GROUP,
        .ctrl_dev_name = "BCM2845:00",      // ACPI _HID of your pinctrl (GPIO) device
        .data.mux = {
            .group = "gpio18",
            .function = "alt5",            
        },
    },
};




static int bcm2835_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
	u32 value;

	value = readl(pc->base + PWM_CONTROL);
	value &= ~(PWM_CONTROL_MASK << PWM_CONTROL_SHIFT(pwm->hwpwm));
	value |= (PWM_MODE << PWM_CONTROL_SHIFT(pwm->hwpwm));
	writel(value, pc->base + PWM_CONTROL);

  dev_info(pc->dev, "Register dump:\n");
  dev_info(pc->dev, "  CONTROL: 0x%08x\n", readl(pc->base + PWM_CONTROL));
  dev_info(pc->dev, "  PERIOD0: %u\n", readl(pc->base + PERIOD(0)));
  dev_info(pc->dev, "  DUTY0:   %u\n", readl(pc->base + DUTY(0)));
  dev_info(pc->dev, "  PERIOD1: %u\n", readl(pc->base + PERIOD(1)));
  dev_info(pc->dev, "  DUTY1:   %u\n", readl(pc->base + DUTY(1)));


	return 0;
}

static void bcm2835_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
	u32 value;

	value = readl(pc->base + PWM_CONTROL);
	value &= ~(PWM_CONTROL_MASK << PWM_CONTROL_SHIFT(pwm->hwpwm));
	writel(value, pc->base + PWM_CONTROL);
}

static int bcm2835_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
                             const struct pwm_state *state)
{
    struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
    unsigned long rate = pc->clk_rate;
    u64 period_cycles, duty_cycles;
    u32 ctrl;
    int retries;

    if (!rate)
        return -EINVAL;

    pm_runtime_get_sync(pc->dev);
    dev_info(pc->dev, "Configuring PWM: rate=%lu Hz", rate);

    period_cycles = DIV_ROUND_CLOSEST_ULL((u64)state->period * rate, NSEC_PER_SEC);
    duty_cycles   = DIV_ROUND_CLOSEST_ULL((u64)state->duty_cycle * rate, NSEC_PER_SEC);

    if (period_cycles < PERIOD_MIN) {
        dev_warn(pc->dev, "Period too small (%llu cycles), skipping configuration\n", period_cycles);
        goto out;
    }

    writel(CM_PASSWD | 0, pc->cm_base + CM_PWMCTL);
    udelay(10);
    writel(CM_PASSWD | (32 << 12), pc->cm_base + CM_PWMDIV);
    writel(CM_PASSWD | CM_SRC_PLLD | CM_ENABLE, pc->cm_base + CM_PWMCTL);
    udelay(10);

    for (retries = 5; retries; retries--) {
        writel(period_cycles, pc->base + PERIOD(pwm->hwpwm));
        udelay(100);
        if (readl(pc->base + PERIOD(pwm->hwpwm)) == period_cycles)
            break;
        dev_warn(pc->dev, "PERIOD write verify failed, retrying (%d left)\n", retries - 1);
    }

    if (!retries)
        dev_err(pc->dev, "PERIOD register stuck: wrote %llu, read %u\n",
                period_cycles, readl(pc->base + PERIOD(pwm->hwpwm)));

    writel(duty_cycles, pc->base + DUTY(pwm->hwpwm));
    udelay(100);
    if (readl(pc->base + DUTY(pwm->hwpwm)) != duty_cycles)
        dev_warn(pc->dev, "DUTY write mismatch: wrote %llu, read %u\n",
                 duty_cycles, readl(pc->base + DUTY(pwm->hwpwm)));

    ctrl = readl(pc->base + PWM_CONTROL);
    ctrl &= ~(PWM_ENABLE | PWM_POLARITY) << PWM_CONTROL_SHIFT(pwm->hwpwm);

    if (state->enabled)
        ctrl |= PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm);
    if (state->polarity == PWM_POLARITY_INVERSED)
        ctrl |= PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm);

    writel(ctrl, pc->base + PWM_CONTROL);
    udelay(100);

    dev_info(pc->dev, "APPLY: hwpwm=%d period=%llu duty=%llu polarity=%s",
             pwm->hwpwm, period_cycles, duty_cycles,
             state->polarity == PWM_POLARITY_INVERSED ? "inversed" : "normal");

out:
    pm_runtime_put(pc->dev);
    return 0;
}

static int bcm2835_pwm_get_state(struct pwm_chip *chip,
                                 struct pwm_device *pwm,
                                 struct pwm_state *state)
{
    struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
    u32 ctrl, period, duty;
    int ret;

    ret = pm_runtime_get_sync(pc->dev);
    if (ret < 0) {
        dev_warn(pc->dev, "Failed to resume device for get_state: %d\n", ret);
        pm_runtime_put_noidle(pc->dev);
        return ret;
    }

    ctrl = readl(pc->base + PWM_CONTROL);
    period = readl(pc->base + PERIOD(pwm->hwpwm));
    duty = readl(pc->base + DUTY(pwm->hwpwm));

    state->enabled = !!(ctrl & (PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm)));
    state->polarity = (ctrl & (PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm)))
                      ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

    if (period) {
        state->period = (u64)period * NSEC_PER_SEC / pc->clk_rate;
    } else {
        state->period = 1000000; // default to 1ms
        dev_warn(pc->dev, "PERIOD is zero; using fallback value: 1ms\n");
    }

    if (duty) {
        state->duty_cycle = (u64)duty * NSEC_PER_SEC / pc->clk_rate;
    } else {
        state->duty_cycle = 0;
    }

    dev_info(pc->dev, "PWM GET_STATE: hwpwm=%u ctrl=0x%08x period=%u duty=%u -> enabled=%d polarity=%s",
             pwm->hwpwm, ctrl, period, duty,
             state->enabled,
             state->polarity == PWM_POLARITY_INVERSED ? "inversed" : "normal");

    pm_runtime_put(pc->dev);
    return 0;
}

static int bcm2835_pwm_suspend(struct device *dev)
{
	pm_runtime_put_sync_suspend(dev);
	return 0;
}

static int bcm2835_pwm_resume(struct device *dev)
{
	pm_runtime_get_sync(dev);
	return 0;
}



static const struct pwm_ops bcm2835_pwm_ops = {
	.request = bcm2835_pwm_request,
	.free = bcm2835_pwm_free,
	.apply = bcm2835_pwm_apply,
	.get_state = bcm2835_pwm_get_state,
	.owner = THIS_MODULE,
};

static int bcm2835_pwm_probe(struct platform_device *pdev)
{
	struct bcm2835_pwm *pc;
	struct pinctrl *pinctrl;
	int ret;
    u32 val;
	struct pinctrl_state *state;

	dev_info(&pdev->dev, "Probing BCM2835 PWM driver\n");

	pc = devm_kzalloc(&pdev->dev, sizeof(*pc), GFP_KERNEL);
	if (!pc) {
		dev_err(&pdev->dev, "Failed to allocate memory for PWM driver\n");
		return -ENOMEM;
	}
	dev_info(&pdev->dev, "Memory allocated for PWM driver\n");

	pc->dev = &pdev->dev;

	ret = dev_pm_domain_attach(&pdev->dev, true);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to attach power domain: %d\n", ret);
	} else {
		dev_info(&pdev->dev, "Power domain attached successfully\n");
	}

	pm_runtime_enable(&pdev->dev);
    pm_runtime_get_sync(&pdev->dev);  // force power-on for setup


	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base)) {
		dev_err(&pdev->dev, "Failed to map I/O memory\n");
		return PTR_ERR(pc->base);
	}
	dev_info(&pdev->dev, "I/O memory mapped successfully\n");

		// Add the power-on call here, before accessing hardware



	pc->chip.dev = &pdev->dev;
	pc->chip.ops = &bcm2835_pwm_ops;
	pc->chip.npwm = 2;
	pc->chip.base = -1;
	pc->chip.of_xlate = NULL;
	pc->chip.of_pwm_n_cells = 0;
	pc->chip.dev->of_node = NULL;

	


	ret = pinctrl_register_mappings(bcm2835_pwm_map, ARRAY_SIZE(bcm2835_pwm_map));
	if (ret)
		dev_warn(&pdev->dev, "Failed to register pinctrl mappings: %d\n", ret);

	dev_info(&pdev->dev, "Before pinctrl_get_select_default\n");


	pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pinctrl)) {
	  dev_warn(&pdev->dev, "Failed to get pinctrl handle\n");
    } else {
	  state = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(state)) {
		dev_warn(&pdev->dev, "Failed to lookup pinctrl default state\n");
	} else {
		ret = pinctrl_select_state(pinctrl, state);
		if (ret)
			dev_warn(&pdev->dev, "Failed to select pinctrl default state: %d\n", ret);
		else
			dev_info(&pdev->dev, "Applied default pinctrl state explicitly\n");
	}
}

	platform_set_drvdata(pdev, pc);
	dev_info(&pdev->dev, "PWM chip initialized\n");




	// Dump chip configuration before registering
	dev_info(&pdev->dev, "About to add PWM chip...");
	dev_info(&pdev->dev, "chip.dev = %p", pc->chip.dev);
	dev_info(&pdev->dev, "chip.npwm = %d", pc->chip.npwm);
	dev_info(&pdev->dev, "chip.base = %d", pc->chip.base);
	dev_info(&pdev->dev, "chip.ops = %p", pc->chip.ops);
	dev_info(&pdev->dev, "chip.of_xlate = %p", pc->chip.of_xlate);
	dev_info(&pdev->dev, "chip.of_pwm_n_cells = %d", pc->chip.of_pwm_n_cells);


     // Map the Clock Manager registers
    pc->cm_base = ioremap(CM_BASE_PHYS, 0x100);
    if (!pc->cm_base) {
        dev_err(&pdev->dev, "Failed to ioremap Clock Manager\n");
        return -ENOMEM;
    }

    // Disable PWM clock before configuring
    writel(CM_PASSWD | 0x0, pc->cm_base + CM_PWMCTL);
    udelay(10);

    // Set divider (500 MHz / 32 = 15.625 MHz)
    writel(CM_PASSWD | (32 << 12), pc->cm_base + CM_PWMDIV);

    // Enable PWM clock with PLLD as source
    val = CM_PASSWD | CM_SRC_PLLD | CM_ENABLE;
    writel(val, pc->cm_base + CM_PWMCTL);
	pc->clk_rate = FALLBACK_PWM_CLK_HZ;

	// Optional: add a delay to ensure the clock is stable
	udelay(10);

val = readl(pc->cm_base + CM_PWMCTL);
dev_info(pc->dev, "CM_PWMCTL readback: 0x%08x", val);

	ret = pwmchip_add(&pc->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add PWM chip, error: %d\n", ret);
		

		// Print out pwm_ops function pointers
		if (pc->chip.ops) {
			dev_err(&pdev->dev, "pwm_ops: apply=%p, request=%p, free=%p\n",
				pc->chip.ops->apply, pc->chip.ops->request, pc->chip.ops->free);
		} else {
			dev_err(&pdev->dev, "pwm_ops is NULL\n");
		}

		return ret;
	}


	


dev_info(&pdev->dev, "PWM clock manually enabled via MMIO\n");
dev_info(&pdev->dev, "PWM clock rate used: %lu Hz\n", pc->clk_rate);


	dev_info(&pdev->dev, "PWM chip added successfully\n");

	return 0;


}



static int bcm2835_pwm_remove(struct platform_device *pdev)
{
	struct bcm2835_pwm *pc = platform_get_drvdata(pdev);

pwmchip_remove(&pc->chip);
pinctrl_unregister_mappings(bcm2835_pwm_map);


if (pc->cm_base) {
    iounmap(pc->cm_base);
	dev_info(&pdev->dev, "Unmapped base\n");
} else {
	dev_warn(&pdev->dev, "Base was NULL\n");
}

pm_runtime_disable(&pc->chip.dev->dev);



dev_info(&pdev->dev, "Removed BCM2835 PWM driver\n");
return 0;

}

static DEFINE_SIMPLE_DEV_PM_OPS(bcm2835_pwm_pm_ops, bcm2835_pwm_suspend,
				bcm2835_pwm_resume);

static const struct acpi_device_id bcm2835_pwm_acpi_match[] = {
	{ "BCM2844", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, bcm2835_pwm_acpi_match);

static struct platform_driver bcm2835_pwm_driver = {
	.driver = {
		.name = "bcm2835-pwm",
		.acpi_match_table = ACPI_PTR(bcm2835_pwm_acpi_match),
		.pm = pm_ptr(&bcm2835_pwm_pm_ops),
	},
	.probe = bcm2835_pwm_probe,
	.remove = bcm2835_pwm_remove,
};
module_platform_driver(bcm2835_pwm_driver);


MODULE_AUTHOR("Richard Jeans <rich@jeansy.org>");
MODULE_DESCRIPTION("Broadcom BCM2835 PWM driver with acpi support");
MODULE_LICENSE("GPL v2");
