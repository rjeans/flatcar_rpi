// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef RPI_PWM_FAN_H
#define RPI_PWM_FAN_H

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pwm.h>
#include <linux/thermal.h>
#include <linux/hwmon.h>

struct pwm_fan_ctx {
	struct device *dev;

	struct mutex lock;
	struct pwm_device *pwm;
	struct pwm_state pwm_state;
	bool enabled;

	unsigned int pwm_value;
	unsigned int pwm_fan_state;
	unsigned int pwm_fan_max_state;
	unsigned int *pwm_fan_cooling_levels;
	struct thermal_cooling_device *cdev;

    struct thermal_zone_device * tz;

	struct hwmon_chip_info info;
};

#endif // RPI_PWM_FAN_H
