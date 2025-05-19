// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/acpi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/delay.h>

#define PWM_CTL     0x0
#define PWM_RNG1    0x10
#define PWM_DAT1    0x14

struct pwm_test {
	void __iomem *base;
	struct clk *clk;
	struct device *dev;
	struct kobject *kobj;
	u32 period;
	u32 duty;
};

static struct pwm_test *g_test;

static void pwm_update(struct pwm_test *test)
{
	writel(0, test->base + PWM_CTL);
	udelay(5);

	writel(test->period, test->base + PWM_RNG1);
	writel(test->duty, test->base + PWM_DAT1);
	udelay(5);

	writel((1 << 7) | (1 << 0), test->base + PWM_CTL); // MSEN1 | PWEN1
	udelay(5);
}

static ssize_t period_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", g_test->period);
}

static ssize_t period_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (kstrtou32(buf, 10, &val) == 0) {
		g_test->period = val;
		pwm_update(g_test);
	}
	return count;
}

static ssize_t duty_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", g_test->duty);
}

static ssize_t duty_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	if (kstrtou32(buf, 10, &val) == 0) {
		g_test->duty = val;
		pwm_update(g_test);
	}
	return count;
}

static struct kobj_attribute period_attr = __ATTR(period, 0664, period_show, period_store);
static struct kobj_attribute duty_attr   = __ATTR(duty_cycle, 0664, duty_show, duty_store);

static struct attribute *pwm_attrs[] = {
	&period_attr.attr,
	&duty_attr.attr,
	NULL,
};

static struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};

static int pwm_acpi_test_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct pwm_test *test;

	dev_info(&pdev->dev, "pwm_acpi_test: probing\n");

	test = devm_kzalloc(&pdev->dev, sizeof(*test), GFP_KERNEL);
	if (!test)
		return -ENOMEM;

	test->dev = &pdev->dev;
	g_test = test;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	test->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(test->base))
		return PTR_ERR(test->base);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	test->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (!IS_ERR_OR_NULL(test->clk))
		clk_prepare_enable(test->clk);

	test->period = 15625;
	test->duty = 6250;
	pwm_update(test);

	test->kobj = kobject_create_and_add("pwm-acpi-test", kernel_kobj);
	if (!test->kobj)
		return -ENOMEM;

	return sysfs_create_group(test->kobj, &pwm_attr_group);
}

static int pwm_acpi_test_remove(struct platform_device *pdev)
{
	struct pwm_test *test = g_test;

	writel(0, test->base + PWM_CTL);

	if (test->kobj)
		kobject_put(test->kobj);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (!IS_ERR_OR_NULL(test->clk))
		clk_disable_unprepare(test->clk);

	dev_info(&pdev->dev, "pwm_acpi_test: removed\n");
	return 0;
}

static const struct acpi_device_id pwm_acpi_ids[] = {
	{ "BCM2844", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, pwm_acpi_ids);

static struct platform_driver pwm_acpi_test_driver = {
	.probe  = pwm_acpi_test_probe,
	.remove = pwm_acpi_test_remove,
	.driver = {
		.name = "pwm-acpi-test",
		.acpi_match_table = ACPI_PTR(pwm_acpi_ids),
	},
};
module_platform_driver(pwm_acpi_test_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard Jeans + GPT-4");
MODULE_DESCRIPTION("Test harness for ACPI PWM validation on BCM2711 (with sysfs control)");
