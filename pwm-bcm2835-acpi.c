// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2014 Richard Jeans <rich@jeansy.org>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/acpi.h>
#include <linux/clk-provider.h>


#define PWM_CONTROL		0x000
#define PWM_CONTROL_SHIFT(x)	((x) * 8)
#define PWM_CONTROL_MASK	0xff
#define PWM_MODE		0x80		/* set timer in PWM mode */
#define PWM_ENABLE		(1 << 0)
#define PWM_POLARITY		(1 << 4)

#define PERIOD(x)		(((x) * 0x10) + 0x10)
#define DUTY(x)			(((x) * 0x10) + 0x14)

#define PERIOD_MIN		0x2

struct bcm2835_pwm {
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	struct pwm_chip chip;
	struct clk_hw *clk_hw;
};

static inline struct bcm2835_pwm *to_bcm2835_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct bcm2835_pwm, chip);
}

static int bcm2835_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
	u32 value;

	value = readl(pc->base + PWM_CONTROL);
	value &= ~(PWM_CONTROL_MASK << PWM_CONTROL_SHIFT(pwm->hwpwm));
	value |= (PWM_MODE << PWM_CONTROL_SHIFT(pwm->hwpwm));
	writel(value, pc->base + PWM_CONTROL);

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
	unsigned long long period_cycles;
	u64 max_period;

	u32 val;

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
	max_period = DIV_ROUND_UP_ULL((u64)U32_MAX * NSEC_PER_SEC + NSEC_PER_SEC / 2, pc->rate) - 1;

	if (state->period > max_period)
		return -EINVAL;

	/* set period */
	period_cycles = DIV_ROUND_CLOSEST_ULL(state->period * pc->rate, NSEC_PER_SEC);

	/* don't accept a period that is too small */
	if (period_cycles < PERIOD_MIN)
		return -EINVAL;

	writel(period_cycles, pc->base + PERIOD(pwm->hwpwm));

	/* set duty cycle */
	val = DIV_ROUND_CLOSEST_ULL(state->duty_cycle * pc->rate, NSEC_PER_SEC);
	writel(val, pc->base + DUTY(pwm->hwpwm));

	/* set polarity */
	val = readl(pc->base + PWM_CONTROL);

	if (state->polarity == PWM_POLARITY_NORMAL)
		val &= ~(PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm));
	else
		val |= PWM_POLARITY << PWM_CONTROL_SHIFT(pwm->hwpwm);

	/* enable/disable */
	if (state->enabled)
		val |= PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm);
	else
		val &= ~(PWM_ENABLE << PWM_CONTROL_SHIFT(pwm->hwpwm));

	writel(val, pc->base + PWM_CONTROL);

	return 0;
}



static struct clk *register_fallback_clk(struct device *dev, struct bcm2835_pwm *pc)
{
	struct clk_fixed_rate *fixed;
	struct clk_init_data *init;
	struct clk *clk;

	fixed = devm_kzalloc(dev, sizeof(*fixed), GFP_KERNEL);
	if (!fixed)
		return ERR_PTR(-ENOMEM);

	init = devm_kzalloc(dev, sizeof(*init), GFP_KERNEL);
	if (!init) {
		devm_kfree(dev, fixed); // Free allocated memory for `fixed`
		return ERR_PTR(-ENOMEM);
	}

	init->name = "bcm2835-pwm-fallback-clk";
	init->ops = &clk_fixed_rate_ops;

	fixed->fixed_rate = 1000000;
	fixed->hw.init = init;

	clk = clk_register(dev, &fixed->hw);
	if (IS_ERR(clk)) {
		dev_warn(dev, "Fallback clock registration failed, error: %ld\n", PTR_ERR(clk));
		devm_kfree(dev, init);  // Free allocated memory for `init`
		devm_kfree(dev, fixed); // Free allocated memory for `fixed`
		
	} else {
		dev_info(dev, "Fallback clock registered successfully\n");
		pc->clk_hw = &fixed->hw;
	}

	return clk;
}

static const struct pwm_ops bcm2835_pwm_ops = {
	.request = bcm2835_pwm_request,
	.free = bcm2835_pwm_free,
	.apply = bcm2835_pwm_apply,
};

static int bcm2835_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_pwm *pc;
	int ret;

	dev_info(dev, "Probing BCM2835 PWM driver\n");

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc) {
		dev_err(dev, "Failed to allocate memory for PWM driver\n");
		return -ENOMEM;
	}

	dev_info(dev, "Allocating memory for PWM driver\n");

	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base)) {
		dev_err(dev, "Failed to map I/O memory\n");
		return PTR_ERR(pc->base);
	}

	dev_info(dev, "I/O memory mapped successfully\n");

	pc->clk = devm_clk_get(dev, NULL);
    if (IS_ERR(pc->clk)) {
        dev_warn(dev, "No clock found via ACPI, falling back\n");
        pc->clk = register_fallback_clk(dev, pc);
		if (IS_ERR(pc->clk)) {
			dev_err(dev, "Clock not registered\n");
			return -ENODEV;
		}
    }
	
	

	pc->rate = clk_get_rate(pc->clk);
	if (pc->rate == 0) {
		dev_err(dev, "Failed to get clock rate\n");
		ret = -EINVAL;
		goto err_unregister_clk;
	}
	dev_info(dev, "Clock rate: %lu\n", pc->rate);

	pc->chip.dev = dev;
	pc->chip.ops = &bcm2835_pwm_ops;
	pc->chip.npwm = 1;
	pc->chip.base = -1; // Use the default base value

	platform_set_drvdata(pdev, pc);

	dev_info(dev, "Registering PWM chip\n");

	ret = devm_pwmchip_add(dev, &pc->chip);
	if (ret < 0) {
		dev_err(dev, "Failed to add PWM chip, error: %d\n", ret);
		dev_err(dev, "Debug info: base=%p, clk=%p, rate=%lu\n",
			pc->base, pc->clk, pc->rate);
		goto err_unregister_clk;
	}

	dev_info(dev, "BCM2835 PWM driver probed successfully\n");
	return 0;

err_unregister_clk:
	if (pc->clk_hw)
		clk_hw_unregister(pc->clk_hw);
	return ret;
}

static int bcm2835_pwm_suspend(struct device *dev)
{
	struct bcm2835_pwm *pc = dev_get_drvdata(dev);

	clk_disable_unprepare(pc->clk);

	return 0;
}

static int bcm2835_pwm_resume(struct device *dev)
{
	struct bcm2835_pwm *pc = dev_get_drvdata(dev);

	return clk_prepare_enable(pc->clk);
}

static int bcm2835_pwm_remove(struct platform_device *pdev)
{
	struct bcm2835_pwm *pc = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "Removing BCM2835 PWM driver\n");

	if (pc->clk_hw)
		clk_hw_unregister(pc->clk_hw);

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
