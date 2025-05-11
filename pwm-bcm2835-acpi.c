// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2014 Richard Jeans <rich@jeansy.org>
 */


#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h> 
#include <linux/pinctrl/machine.h>
#include <linux/delay.h>


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
            .group = "gpio13",
            .function = "alt0",            
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
	unsigned long long period_cycles;
	u64 max_period;

	u32 duty_val,ctrl_val;

	dev_info(pc->dev, "Clock rate: %lu\n", rate);
	if (!rate) {
		dev_err(pc->dev, "failed to get clock rate\n");
		return -EINVAL;


   }



	/*
	 * period_cycles must be a 32 bit value, so period * rate / NSEC_PER_SEC
	 * must be <= U32_MAX. As U32_MAX * NSEC_PER_SEC < U64_MAX the
	 * multiplication period * rate doesn't overflow.
	 * To calculate the maximal possible period that guarantees the
	 * above inequality:
	 *
	 *     round(period * rate / NSEC_PER_SEC) <= U32_MAX
	 * <=> period * rate / NSEC_PER_SEC < U32_MAX + 0.5
	 * <=> period * rate < (U32_MAX + 0.5) * NSEC_PER_SEC
	 * <=> period < ((U32_MAX + NSEC_PER_SEC/2) / rate
	 * <=> period <= ceil((U32_MAX * NSEC_PER_SEC + NSEC_PER_SEC/2) / rate) - 1
	 */
	max_period = DIV_ROUND_UP_ULL((u64)U32_MAX * NSEC_PER_SEC + NSEC_PER_SEC / 2, rate) - 1;

	if (state->period > max_period)
		return -EINVAL;

	/* set period */
	period_cycles = DIV_ROUND_CLOSEST_ULL(state->period * rate, NSEC_PER_SEC);

	/* don't accept a period that is too small */
	if (period_cycles < PERIOD_MIN) {
	dev_warn(pc->dev, "period_cycles < PERIOD_MIN (%llu), skipping write\n", period_cycles);
	return 0; // Accept without programming hardware
   }

writel(period_cycles, pc->base + PERIOD(pwm->hwpwm));

udelay(10);  // give time to latch

u32 readback = readl(pc->base + PERIOD(pwm->hwpwm));
if (readback != period_cycles) {
	dev_warn(pc->dev, "PERIOD write verification failed: wrote %llu, read back %u",
	         period_cycles, readback);
}

	/* set duty cycle */
    duty_val = DIV_ROUND_CLOSEST_ULL(state->duty_cycle * rate, NSEC_PER_SEC);
    writel(duty_val, pc->base + DUTY(pwm->hwpwm));

	/* set polarity */
	ctrl_val = readl(pc->base + PWM_CONTROL);

	if (state->polarity == PWM_POLARITY_NORMAL)
		ctrl_val &= ~(PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm));
	else
		ctrl_val |= PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm);

	/* enable/disable */
	if (state->enabled)
		ctrl_val |= PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm);
	else
		ctrl_val &= ~(PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm));

    dev_info(pc->dev, "APPLY: pwm->hwpwm = %u", pwm->hwpwm);
	dev_info(pc->dev, "APPLY: state->enabled = %d", state->enabled);

	dev_info(pc->dev, "PERIOD register (0x%x): %llu ns -> %llu cycles\n",
		 PERIOD(pwm->hwpwm), state->period, period_cycles);

    dev_info(pc->dev, "DUTY register (0x%x): %llu ns -> %u cycles\n",
         DUTY(pwm->hwpwm), state->duty_cycle, duty_val);

    dev_info(pc->dev, "PWM_CONTROL before write: 0x%08x\n", ctrl_val);

	writel(ctrl_val, pc->base + PWM_CONTROL);

	dev_info(pc->dev, "PWM_CONTROL after write:  0x%08x\n",
         readl(pc->base + PWM_CONTROL));


	return 0;
}
static int bcm2835_pwm_get_state(struct pwm_chip *chip,
                                 struct pwm_device *pwm,
                                 struct pwm_state *state)
{
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
	u32 ctrl = readl(pc->base + PWM_CONTROL);
	u32 period = readl(pc->base + PERIOD(pwm->hwpwm));
	u32 duty = readl(pc->base + DUTY(pwm->hwpwm));

	dev_info(pc->dev, "GET_STATE: CONTROL = 0x%08x", ctrl);
	dev_info(pc->dev, "GET_STATE: hwpwm = %u", pwm->hwpwm);
	dev_info(pc->dev, "GET_STATE: PERIOD reg = %u", period);
	dev_info(pc->dev, "GET_STATE: DUTY reg   = %u", duty);

	state->enabled = !!(ctrl & (PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm)));
	state->polarity = (ctrl & (PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm))) ?
	                  PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

	if (period != 0) {
		state->period = (u64)period * NSEC_PER_SEC / pc->clk_rate;
		dev_info(pc->dev, "GET_STATE: Converted period = %llu ns", state->period);
	} else {
		dev_warn(pc->dev, "GET_STATE: PERIOD is zero — fallback period used (1 ms)");
		state->period = 1000000;
	}

	if (duty != 0) {
		state->duty_cycle = (u64)duty * NSEC_PER_SEC / pc->clk_rate;
		dev_info(pc->dev, "GET_STATE: Converted duty   = %llu ns", state->duty_cycle);
	} else {
		state->duty_cycle = 0;
		dev_info(pc->dev, "GET_STATE: DUTY is zero → duty_cycle = 0");
	}

	dev_info(pc->dev, "GET_STATE: polarity = %s, enabled = %d",
	         state->polarity == PWM_POLARITY_INVERSED ? "inversed" : "normal",
	         state->enabled);

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

	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base)) {
		dev_err(&pdev->dev, "Failed to map I/O memory\n");
		return PTR_ERR(pc->base);
	}
	dev_info(&pdev->dev, "I/O memory mapped successfully\n");


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

static int bcm2835_pwm_suspend(struct device *dev)
{
	

	

	return 0;
}

static int bcm2835_pwm_resume(struct device *dev)
{
	

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
