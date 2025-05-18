// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define PWM_BASE_PHYS  0xfe20c000
#define PWM_REG_SIZE   0x28

#define PWM_CTL        0x00
#define PWM_STA        0x04
#define PWM_RNG1       0x10
#define PWM_DAT1       0x14

#define PWM_CTL_PWEN1  (1 << 0)

#define PERIOD_VALUE   15625
#define DUTY_VALUE     (PERIOD_VALUE / 4)

static void __iomem *pwm_base;
static struct device *pwr_dev;
static struct kobject *pwm_kobj;

static ssize_t trigger_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	u32 val_ctl = 0, val_rng = 0, val_dat = 0;

	if (!pwr_dev)
		return sprintf(buf, "Power device not bound\n");

	ret = pm_runtime_get_sync(pwr_dev);
	if (ret < 0)
		return sprintf(buf, "Failed to power ON domain: %d\n", ret);

	if (!pwm_base)
		pwm_base = ioremap(PWM_BASE_PHYS, PWM_REG_SIZE);

	if (!pwm_base) {
		pm_runtime_put(pwr_dev);
		return sprintf(buf, "Failed to ioremap PWM base\n");
	}

	// Configure PWM channel 1
	writel(0, pwm_base + PWM_CTL);                 // Disable before config
	writel(PERIOD_VALUE, pwm_base + PWM_RNG1);     // Period
	writel(DUTY_VALUE, pwm_base + PWM_DAT1);       // Duty cycle
	writel(PWM_CTL_PWEN1, pwm_base + PWM_CTL);     // Enable

	// Read back values
	val_ctl = readl(pwm_base + PWM_CTL);
	val_rng = readl(pwm_base + PWM_RNG1);
	val_dat = readl(pwm_base + PWM_DAT1);

	pm_runtime_put(pwr_dev);

	return sprintf(buf,
		"PWM RNG1 (period) = %u\n"
		"PWM DAT1 (duty)   = %u\n"
		"PWM CTL           = 0x%08x\n",
		val_rng, val_dat, val_ctl);
}

static ssize_t negative_test_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u32 val = 0;

	if (!pwm_base)
		pwm_base = ioremap(PWM_BASE_PHYS, PWM_REG_SIZE);

	if (!pwm_base)
		return sprintf(buf, "Failed to ioremap PWM base\n");

	writel(0xdeadbeef, pwm_base + PWM_RNG1);
	val = readl(pwm_base + PWM_RNG1);

	return sprintf(buf, "PWM RNG1 (without power) = 0x%08x\n", val);
}

static struct kobj_attribute trigger_attr = __ATTR_RO(trigger);
static struct kobj_attribute negative_test_attr = __ATTR_RO(negative_test);

static struct attribute *attrs[] = {
	&trigger_attr.attr,
	&negative_test_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int match_bcm2851(struct device *dev, void *data)
{
	if (strcmp(dev_name(dev), "BCM2851:00") == 0) {
		*((struct device **)data) = get_device(dev);
		return 1;
	}
	return 0;
}

static int __init pwm_test_init(void)
{
	int ret;

	ret = bus_for_each_dev(&platform_bus_type, NULL, &pwr_dev, match_bcm2851);
	if (!pwr_dev) {
		pr_err("pwm_power_test: Could not find platform device 'BCM2851:00'\n");
		return -ENODEV;
	}

	pr_info("pwm_power_test: Found power device: %s\n", dev_name(pwr_dev));

	pwm_kobj = kobject_create_and_add("pwm_power_test", kernel_kobj);
	if (!pwm_kobj) {
		put_device(pwr_dev);
		return -ENOMEM;
	}

	ret = sysfs_create_group(pwm_kobj, &attr_group);
	if (ret) {
		kobject_put(pwm_kobj);
		put_device(pwr_dev);
		return ret;
	}

	pr_info("pwm_power_test: Module loaded\n");
	return 0;
}

static void __exit pwm_test_exit(void)
{
	if (pwm_base)
		iounmap(pwm_base);
	if (pwr_dev)
		put_device(pwr_dev);
	if (pwm_kobj)
		kobject_put(pwm_kobj);
	pr_info("pwm_power_test: Module unloaded\n");
}

module_init(pwm_test_init);
module_exit(pwm_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ACPI for Pi");
MODULE_DESCRIPTION("PWM power test module with period and duty write support");
