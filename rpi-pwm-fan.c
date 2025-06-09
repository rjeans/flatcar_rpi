// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pwm-fan.c - Hwmon driver for fans connected to PWM lines.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Author: Kamil Debski <k.debski@samsung.com>
 */

#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/property.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include "rpi-pwm-fan.h"



#define MAX_PWM 255








static int pwm_fan_power_on(struct pwm_fan_ctx *ctx)
{
	if (ctx->enabled)
		return 0;

	ctx->pwm_state.enabled = true;
	int ret = pwm_apply_might_sleep(ctx->pwm, &ctx->pwm_state);
	if (!ret)
		ctx->enabled = true;

	return ret;
}


static int pwm_fan_power_off(struct pwm_fan_ctx *ctx)
{
	if (!ctx->enabled)
		return 0;

	ctx->pwm_state.enabled = false;
	ctx->pwm_state.duty_cycle = 0;
	int ret = pwm_apply_might_sleep(ctx->pwm, &ctx->pwm_state);
	if (!ret)
		ctx->enabled = false;

	return ret;
}

static int  __set_pwm(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	struct pwm_state *state = &ctx->pwm_state;
	unsigned long period;
	int ret = 0;

	if (pwm > 0) {


		period = state->period;
		state->duty_cycle = DIV_ROUND_UP(pwm * (period - 1), MAX_PWM);
		ret = pwm_apply_might_sleep(ctx->pwm, state);
		if (ret)
			return ret;
		ret = pwm_fan_power_on(ctx);
	} else {
		ret = pwm_fan_power_off(ctx);
	}
	if (!ret)
		ctx->pwm_value = pwm;

	return ret;
}

static int set_pwm(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	int ret;

	mutex_lock(&ctx->lock);
	ret = __set_pwm(ctx, pwm);
	mutex_unlock(&ctx->lock);

	return ret;
}

static void pwm_fan_update_state(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	int i;

	for (i = 0; i < ctx->pwm_fan_max_state; ++i)
		if (pwm < ctx->pwm_fan_cooling_levels[i + 1])
			break;

	ctx->pwm_fan_state = i;
}


static int pwm_fan_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	int ret;

	switch (attr) {
	case hwmon_pwm_input:
		if (val < 0 || val > MAX_PWM)
			return -EINVAL;
		ret = set_pwm(ctx, val);
		if (ret)
			return ret;
		pwm_fan_update_state(ctx, val);
		break;
	case hwmon_pwm_enable:
		if (val != 1)
			return -EOPNOTSUPP;
		return 0;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int pwm_fan_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			*val = ctx->pwm_value;
			return 0;
		case hwmon_pwm_enable:
			*val = 1;
			return 0;
		}
		return -EOPNOTSUPP;

	default:
		return -ENOTSUPP;
	}
}

static umode_t pwm_fan_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_pwm:
		return 0644;


	default:
		return 0;
	}
}

static const struct hwmon_ops pwm_fan_hwmon_ops = {
	.is_visible = pwm_fan_is_visible,
	.read = pwm_fan_read,
	.write = pwm_fan_write,
};

/* thermal cooling device callbacks */
static int pwm_fan_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = NULL;
	struct acpi_device *adev = cdev->devdata;
	if (adev->driver_data)
		ctx = adev->driver_data;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_max_state;

	return 0;
}

static int pwm_fan_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = NULL;
	struct acpi_device *adev = cdev->devdata;
	if (adev->driver_data)
		ctx = adev->driver_data;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_state;

	return 0;
}

static int
pwm_fan_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct pwm_fan_ctx *ctx = NULL;
	int ret;
	struct acpi_device *adev = cdev->devdata;
	if (adev->driver_data)
		ctx = adev->driver_data;

	if (!ctx || (state > ctx->pwm_fan_max_state))
		return -EINVAL;

	if (state == ctx->pwm_fan_state)
		return 0;

	ret = set_pwm(ctx, ctx->pwm_fan_cooling_levels[state]);
	if (ret) {
		dev_err(&cdev->device, "Cannot set pwm!\n");
		return ret;
	}

	ctx->pwm_fan_state = state;

	return ret;
}

static const struct thermal_cooling_device_ops pwm_fan_cooling_ops = {
	.get_max_state = pwm_fan_get_max_state,
	.get_cur_state = pwm_fan_get_cur_state,
	.set_cur_state = pwm_fan_set_cur_state,
};


static int pwm_fan_get_cooling_data(struct device *dev,
				       struct pwm_fan_ctx *ctx)
{
	
	int num, i, ret;

	if (!fwnode_property_present(dev_fwnode(dev), "cooling-levels")) {
		dev_info(dev, "No cooling levels property found\n");
		return 0;
	}

	ret = fwnode_property_count_u32(dev_fwnode(dev), "cooling-levels");
	if (ret <= 0) {
		dev_err(dev, "Wrong data!\n");
		return ret ? : -EINVAL;
	}

	num = ret;
	ctx->pwm_fan_cooling_levels = devm_kcalloc(dev, num, sizeof(u32),
						   GFP_KERNEL);
	if (!ctx->pwm_fan_cooling_levels)
		return -ENOMEM;

	ret = fwnode_property_read_u32_array(dev_fwnode(dev), "cooling-levels",
					 ctx->pwm_fan_cooling_levels, num);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
		return ret;
	}

	for (i = 0; i < num; i++) {
		if (ctx->pwm_fan_cooling_levels[i] > MAX_PWM) {
			dev_err(dev, "PWM fan state[%d]:%d > %d\n", i,
				ctx->pwm_fan_cooling_levels[i], MAX_PWM);
			return -EINVAL;
		}
	}


	ctx->pwm_fan_max_state = num - 1;

	return 0;
}

static void pwm_fan_cleanup(void *__ctx)
{
	struct pwm_fan_ctx *ctx = __ctx;



	pwm_fan_power_off(ctx);
}

static int pwm_fan_probe(struct platform_device *pdev)
{
	struct pwm_fan_ctx *ctx;
	struct device *dev = &pdev->dev;
	const struct hwmon_channel_info *ctx_channels[] = {
		HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
		NULL
	};
	struct thermal_cooling_device *cdev;
	struct device *hwmon;
	int ret;
	struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);

	if (!adev) {
		dev_err(dev, "No ACPI companion found\n");
		return -ENODEV;
	}



	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);
	ctx->dev = dev;

	ctx->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(ctx->pwm))
		return dev_err_probe(dev, PTR_ERR(ctx->pwm), "Could not get PWM\n");

	platform_set_drvdata(pdev, ctx);
    adev->driver_data = ctx;


	pwm_init_state(ctx->pwm, &ctx->pwm_state);
	ctx->pwm_state.usage_power = true;

	if (ctx->pwm_state.period > ULONG_MAX / MAX_PWM + 1) {
		dev_err(dev, "Configured period too big\n");
		return -EINVAL;
	}


	ret = set_pwm(ctx, MAX_PWM);
	if (ret) {
		dev_err(dev, "Failed to configure PWM: %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, pwm_fan_cleanup, ctx);
	if (ret)
		return ret;


	ctx->info.ops = &pwm_fan_hwmon_ops;
	ctx->info.info = ctx_channels;

	hwmon = devm_hwmon_device_register_with_info(dev, "pwmfan", ctx, &ctx->info, NULL);
	if (IS_ERR(hwmon)) {
		dev_err(dev, "Failed to register hwmon device\n");
		return PTR_ERR(hwmon);
	}

	ret = pwm_fan_get_cooling_data(dev, ctx);  // Still useful for thermal binding
	if (ret) {
		dev_err(dev, "Failed to get cooling data: %d\n", ret);
		return ret;
	}

	ctx->pwm_fan_state = ctx->pwm_fan_max_state;

	if (IS_ENABLED(CONFIG_THERMAL)) {
         cdev = thermal_cooling_device_register( "pwm-fan", adev,
					    &pwm_fan_cooling_ops);

		if (IS_ERR(cdev)) {
			ret = PTR_ERR(cdev);
			dev_err(dev, "Failed to register pwm-fan as cooling device: %d\n", ret);
			return ret;
		}
		ctx->cdev = cdev;
		dev_info(dev, "Registered as cooling device\n");


	}

	ret = sysfs_create_link(&dev->kobj, &ctx->cdev->device.kobj, "thermal_cooling");
if (ret) {
	dev_err(dev, "Failed to create sysfs link 'thermal_cooling'\n");
	
	return ret;
	
}

ret = sysfs_create_link(&ctx->cdev->device.kobj, &dev->kobj, "device");
if (ret) {
	dev_err(dev, "Failed to create sysfs link 'device'\n");
	return ret;
}




	return 0;


}


static void pwm_fan_shutdown(struct platform_device *pdev)
{
	struct pwm_fan_ctx *ctx = platform_get_drvdata(pdev);

	pwm_fan_cleanup(ctx);
}

static int pwm_fan_suspend(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return pwm_fan_power_off(ctx);
}

static int pwm_fan_resume(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return set_pwm(ctx, ctx->pwm_value);
}

static DEFINE_SIMPLE_DEV_PM_OPS(pwm_fan_pm, pwm_fan_suspend, pwm_fan_resume);

static const struct acpi_device_id acpi_pwm_fan_match[] = {
	{ "PWMF0001", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, acpi_pwm_fan_match);

static int pwm_fan_remove(struct platform_device *pdev)
{
	struct pwm_fan_ctx *ctx = platform_get_drvdata(pdev);
	int i;

	if (!ctx)
		return -EINVAL;

	if (ctx->cdev) {
		/* Unbind from stored thermal zone if available */
		if (ctx->tz) {
			dev_info(ctx->dev,
			         "Unbinding cooling device from thermal zone: %s\n",
			         dev_name(&ctx->tz->device));

			for (i = 0; i < ctx->tz->num_trips; i++) {
				int ret = thermal_zone_unbind_cooling_device(ctx->tz, i, ctx->cdev);
				if (ret)
					dev_warn(ctx->dev,
					         "Failed to unbind from trip %d: %d\n", i, ret);
				else
					dev_info(ctx->dev,
					         "Unbound cooling device from trip %d\n", i);
			}
		} else {
			dev_warn(ctx->dev,
			         "No thermal zone recorded, skipping unbind\n");
		}

		sysfs_remove_link(&ctx->cdev->device.kobj, "device");
		sysfs_remove_link(&ctx->dev->kobj, "thermal_cooling");
		thermal_cooling_device_unregister(ctx->cdev);
	}

	pwm_fan_cleanup(ctx);

	return 0;
}

static struct platform_driver pwm_fan_driver = {
	.probe		= pwm_fan_probe,
	.remove		= pwm_fan_remove,
	.shutdown	= pwm_fan_shutdown,
	.driver	= {
		.name		= "pwm-fan",
		.pm		= pm_sleep_ptr(&pwm_fan_pm),
		.acpi_match_table	= acpi_pwm_fan_match,
	},
};

module_platform_driver(pwm_fan_driver);

MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_ALIAS("platform:pwm-fan");
MODULE_DESCRIPTION("PWM FAN driver");
MODULE_LICENSE("GPL");
