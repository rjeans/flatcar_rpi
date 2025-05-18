// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define PWM_BASE_PHYS  0xfe20c000
#define PWM_REG_SIZE   0x28
#define PWM_RNG1       0x10

static void __iomem *pwm_base;
static struct device *pwr_dev;
static struct kobject *pwm_kobj;

static ssize_t trigger_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	u32 val = 0;

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

	writel(0x1234, pwm_base + PWM_RNG1);
	val = readl(pwm_base + PWM_RNG1);

	pm_runtime_put(pwr_dev);

	return sprintf(buf, "PWM RNG1 (with power ON) = 0x%08x\n", val);
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

// Match function to find platform device BCM2851:00
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

	// Find the real platform device created by ACPI
	ret = bus_for_each_dev(&platform_bus_type, NULL, &pwr_dev, match_bcm2851);
	if (!pwr_dev) {
		pr_err("pwm_power_test: Could not find platform device 'BCM2851:00'\n");
		return -ENODEV;
	}

	pr_info("pwm_power_test: Found power device: %s\n", dev_name(pwr_dev));

	// Create sysfs
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
MODULE_DESCRIPTION("Test module for Raspberry Pi power domain with PWM MMIO validation");
