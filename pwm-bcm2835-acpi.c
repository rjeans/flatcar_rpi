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
    int retries, ret;

    if (!rate)
        return -EINVAL;

    ret = pm_runtime_get_sync(pc->dev);
    if (ret < 0) {
        dev_warn(pc->dev, "Failed to power up device: %d", ret);
        pm_runtime_put_noidle(pc->dev);
        return ret;
    }

    dev_info(pc->dev, "Configuring PWM at %lu Hz", rate);

    if (!(readl(pc->cm_base + CM_PWMCTL) & CM_ENABLE)) {
        writel(CM_PASSWD | 0x0, pc->cm_base + CM_PWMCTL);
        udelay(10);
        writel(CM_PASSWD | (32 << 12), pc->cm_base + CM_PWMDIV);
        writel(CM_PASSWD | CM_SRC_PLLD | CM_ENABLE, pc->cm_base + CM_PWMCTL);
        udelay(10);
    }

    period_cycles = DIV_ROUND_CLOSEST_ULL((u64)state->period * rate, NSEC_PER_SEC);
    duty_cycles = DIV_ROUND_CLOSEST_ULL((u64)state->duty_cycle * rate, NSEC_PER_SEC);

    if (period_cycles < PERIOD_MIN) {
        dev_warn(pc->dev, "Period too small (%llu cycles), skipping configuration", period_cycles);
        ret = -EINVAL;
        goto out;
    }

    for (retries = 5; retries; retries--) {
        writel(period_cycles, pc->base + PERIOD(pwm->hwpwm));
        udelay(100);
        if (readl(pc->base + PERIOD(pwm->hwpwm)) == period_cycles)
            break;
        dev_warn(pc->dev, "PERIOD write verify failed, retrying (%d left)", retries - 1);
    }

    if (!retries) {
        dev_err(pc->dev, "PERIOD register stuck: wrote %llu, read %u",
                period_cycles, readl(pc->base + PERIOD(pwm->hwpwm)));
        ret = -EIO;
        goto out;
    }

    writel(duty_cycles, pc->base + DUTY(pwm->hwpwm));
    udelay(100);
    if (readl(pc->base + DUTY(pwm->hwpwm)) != duty_cycles)
        dev_warn(pc->dev, "DUTY write mismatch: wrote %llu, read %u",
                 duty_cycles, readl(pc->base + DUTY(pwm->hwpwm)));

    ctrl = readl(pc->base + PWM_CONTROL);
    ctrl &= ~(PWM_ENABLE | PWM_POLARITY | PWM_MODE) << PWM_CONTROL_SHIFT(pwm->hwpwm);
    ctrl |= PWM_MODE << PWM_CONTROL_SHIFT(pwm->hwpwm);

    if (state->enabled)
        ctrl |= PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm);
    if (state->polarity == PWM_POLARITY_INVERSED)
        ctrl |= PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm);

    writel(ctrl, pc->base + PWM_CONTROL);
    udelay(100);

    dev_info(pc->dev, "Applied PWM config: hwpwm=%d period=%llu duty=%llu enabled=%d polarity=%s",
             pwm->hwpwm, period_cycles, duty_cycles,
             state->enabled,
             state->polarity == PWM_POLARITY_INVERSED ? "inversed" : "normal");

out:
    pm_runtime_put(pc->dev);
    return ret;
}

static int bcm2835_pwm_get_state(struct pwm_chip *chip,
                                 struct pwm_device *pwm,
                                 struct pwm_state *state)
{
    struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
    u32 ctrl, period, duty;

    pm_runtime_get_sync(pc->dev);

    ctrl = readl(pc->base + PWM_CONTROL);
    period = readl(pc->base + PERIOD(pwm->hwpwm));
    duty = readl(pc->base + DUTY(pwm->hwpwm));

    state->enabled = !!(ctrl & (PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm)));
    state->polarity = (ctrl & (PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm)))
                      ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

    if (!(ctrl & (PWM_MODE << PWM_CONTROL_SHIFT(pwm->hwpwm))))
        dev_warn(pc->dev, "PWM channel %u not in PWM mode!", pwm->hwpwm);

    state->period = period ? (u64)period * NSEC_PER_SEC / pc->clk_rate : 1000000;
    state->duty_cycle = duty ? (u64)duty * NSEC_PER_SEC / pc->clk_rate : 0;

    dev_info(pc->dev, "PWM GET_STATE: hwpwm=%u ctrl=0x%08x period=%u cycles duty=%u cycles",
             pwm->hwpwm, ctrl, period, duty);
    dev_info(pc->dev, "Converted: period=%llu ns, duty=%llu ns",
             state->period, state->duty_cycle);

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
	struct pinctrl_state *state;
	int ret;

	dev_info(&pdev->dev, "Probing BCM2835 PWM driver");

	pc = devm_kzalloc(&pdev->dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return dev_err_probe(&pdev->dev, -ENOMEM, "Failed to allocate memory");

	pc->dev = &pdev->dev;

	// Attach power domain
	ret = dev_pm_domain_attach(&pdev->dev, true);
	if (ret)
		dev_warn(&pdev->dev, "Failed to attach power domain: %d", ret);
	else
		dev_info(&pdev->dev, "Power domain attached successfully");

	// Enable runtime PM
	pm_runtime_enable(&pdev->dev);

	// Map PWM base register
	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base))
		return dev_err_probe(&pdev->dev, PTR_ERR(pc->base), "Failed to map PWM I/O");

	// Map Clock Manager registers (for CM_PWMCTL, CM_PWMDIV)
	pc->cm_base = ioremap(CM_BASE_PHYS, 0x100);
	if (!pc->cm_base)
		return dev_err_probe(&pdev->dev, -ENOMEM, "Failed to ioremap Clock Manager");

	// Use fallback clock rate until real one is queried
	pc->clk_rate = FALLBACK_PWM_CLK_HZ;

	// Register optional pinctrl mapping
	ret = pinctrl_register_mappings(bcm2835_pwm_map, ARRAY_SIZE(bcm2835_pwm_map));
	if (ret)
		dev_warn(&pdev->dev, "Failed to register pinctrl mappings: %d", ret);

	// Apply default pinctrl state
	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (!IS_ERR(pinctrl)) {
		state = pinctrl_lookup_state(pinctrl, "default");
		if (!IS_ERR(state)) {
			ret = pinctrl_select_state(pinctrl, state);
			if (!ret)
				dev_info(&pdev->dev, "Applied default pinctrl state explicitly");
			else
				dev_warn(&pdev->dev, "Failed to select default pinctrl state: %d", ret);
		}
	}

	// Register PWM chip
	pc->chip.dev = &pdev->dev;
	pc->chip.ops = &bcm2835_pwm_ops;
	pc->chip.npwm = 2;
	pc->chip.base = -1;
	pc->chip.of_xlate = NULL;
	pc->chip.of_pwm_n_cells = 0;

	ret = pwmchip_add(&pc->chip);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "Failed to add PWM chip");

	platform_set_drvdata(pdev, pc);
	dev_info(&pdev->dev, "BCM2835 PWM driver probed successfully");

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

pm_runtime_disable(pc->chip.dev);



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
