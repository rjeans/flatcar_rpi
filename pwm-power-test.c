// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define PWM_BASE_PHYS  0xfe20c000  // MMIO base (bus address)
#define PWM_REG_SIZE   0x28
#define PWM_RNG1       0x10

static void __iomem *pwm_base;
static struct device *pwr_dev;
static struct kobject *pwm_kobj;

// Sysfs read to trigger test
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

	// Write and read test value
	writel(0x1234, pwm_base + PWM_RNG1);
	val = readl(pwm_base + PWM_RNG1);

	pm_runtime_put(pwr_dev);

	return sprintf(buf, "PWM RNG1 = 0x%08x\n", val);
}

static struct kobj_attribute trigger_attr = __ATTR_RO(trigger);

static int __init pwm_test_init(void)
{
	struct acpi_device *dev;

	pwm_kobj = kobject_create_and_add("pwm_power_test", kernel_kobj);
	if (!pwm_kobj)
		return -ENOMEM;

	if (sysfs_create_file(pwm_kobj, &trigger_attr.attr)) {
		kobject_put(pwm_kobj);
		return -ENOMEM;
	}

	// Match by ACPI HID (e.g. "BCM2851")
	for_each_acpi_dev_match(dev, "BCM2851", NULL, -1) {
		pwr_dev = get_device(dev); // Take ref
		break;
	}

	if (!pwr_dev)
		pr_warn("pwm_power_test: ACPI power device 'BCM2851' not found\n");
	else
		pr_info("pwm_power_test: Linked to device %s\n", dev_name(pwr_dev));

	return 0;
}

static void __exit pwm_test_exit(void)
{
	if (pwm_base)
		iounmap(pwm_base);

	if (pwr_dev)
		put_device(pwr_dev);

	sysfs_remove_file(pwm_kobj, &trigger_attr.attr);
	kobject_put(pwm_kobj);

	pr_info("pwm_power_test: Unloaded\n");
}

module_init(pwm_test_init);
module_exit(pwm_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("acpi-for-pi");
MODULE_DESCRIPTION("Power domain test module with PWM MMIO access via ACPI runtime PM");
