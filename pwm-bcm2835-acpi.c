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
	struct pwm_chip chip;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;

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
	unsigned long rate = clk_get_rate(pc->clk);
	unsigned long long period_cycles;
	u64 max_period;

	u32 val;

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
	if (period_cycles < PERIOD_MIN)
		return -EINVAL;

	writel(period_cycles, pc->base + PERIOD(pwm->hwpwm));

	/* set duty cycle */
	val = DIV_ROUND_CLOSEST_ULL(state->duty_cycle * rate, NSEC_PER_SEC);
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






static const struct pwm_ops bcm2835_pwm_ops = {
	.request = bcm2835_pwm_request,
	.free = bcm2835_pwm_free,
	.apply = bcm2835_pwm_apply,
	.owner = THIS_MODULE,
};

static int bcm2835_pwm_probe(struct platform_device *pdev)
{
	struct bcm2835_pwm *pc;
	int ret;

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


	dev_info(&pdev->dev, "Attempting to get clock\n");
	pc->clk = devm_clk_get(&pdev->dev, "apb_pclk");


	if (IS_ERR(pc->clk)) {
		dev_err_probe(&pdev->dev, PTR_ERR(pc->clk), "No usable clock found\n");
		return PTR_ERR(pc->clk);
	}
	dev_info(&pdev->dev, "Clock obtained successfully\n");

	ret = clk_prepare_enable(pc->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock, error: %d\n", ret);
		return ret;
	}
	dev_info(&pdev->dev, "Clock enabled successfully\n");

	pc->chip.dev = &pdev->dev;
	pc->chip.ops = &bcm2835_pwm_ops;
	pc->chip.npwm = 2;
	pc->chip.base = -1;
	pc->chip.of_xlate = NULL;
	pc->chip.of_pwm_n_cells = 0;
	pc->chip.dev->of_node = NULL;


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

	ret = pwmchip_add(&pc->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add PWM chip, error: %d\n", ret);
		dev_err(&pdev->dev, "Debug info: base=%p, clk=%p\n", pc->base, pc->clk);

		// Print out pwm_ops function pointers
		if (pc->chip.ops) {
			dev_err(&pdev->dev, "pwm_ops: apply=%p, request=%p, free=%p\n",
				pc->chip.ops->apply, pc->chip.ops->request, pc->chip.ops->free);
		} else {
			dev_err(&pdev->dev, "pwm_ops is NULL\n");
		}

		goto add_fail;
	}
	dev_info(&pdev->dev, "PWM chip added successfully\n");

	return 0;

add_fail:
	clk_disable_unprepare(pc->clk);
	dev_err(&pdev->dev, "PWM probe failed, cleaning up\n");
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

pwmchip_remove(&pc->chip);
clk_disable_unprepare(pc->clk);

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
