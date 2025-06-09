// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/err.h>
#include <acpi/acpi_bus.h>
#include "rpi-pwm-fan.h"

#define DRIVER_NAME "rpi_acpi_thermal"
#define RPI_HID     "RPIT0001"
#define MAX_TRIPS   8

static const guid_t dsd_guid = GUID_INIT(0xdaffd814, 0x6eba, 0x4d8c,
                                         0x8a, 0x91, 0xbc, 0x9b, 0xbf, 0x4a, 0xa3, 0x01);

struct rpi_acpi_thermal {
	struct thermal_zone_device *tzd;
	struct acpi_device *adev;
	struct acpi_device *cdev_adev;
	int trip_count;
	s32 trip_temps[MAX_TRIPS];
	s32 trip_hyst[MAX_TRIPS];
	s32 min_states[MAX_TRIPS];
	s32 max_states[MAX_TRIPS];
	struct thermal_trip trips[MAX_TRIPS];
};

static inline int check_array_length(struct device *dev, const char *prop, int expected)
{
	int count = device_property_count_u32(dev, prop);
	if (count < 0) {
		dev_err(dev, "Missing property: %s\n", prop);
		return count;
	}
	if (count != expected) {
		dev_err(dev, "Length mismatch for %s: expected %d, got %d\n", prop, expected, count);
		return -EINVAL;
	}
	return 0;
}

static int rpi_acpi_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct rpi_acpi_thermal *data = tz->devdata;
	acpi_handle handle = data->adev->handle;
	unsigned long long val;
	acpi_status status;

	status = acpi_evaluate_integer(handle, "_TMP", NULL, &val);
	if (ACPI_FAILURE(status))
		return -EIO;

	*temp = ((int)val - 2732) * 100;

	return 0;
}

static int rpi_acpi_bind(struct thermal_zone_device *tz,
                         struct thermal_cooling_device *cdev)
{
	struct rpi_acpi_thermal *data = tz->devdata;
	struct pwm_fan_ctx *ctx;
	int i;

	if (!data) {
		dev_err(&tz->device, "No thermal zone context available\n");
		return -EINVAL;
	}

	/* Fallback: match by cooling device type */
	if (!strstr(cdev->type, "pwm-fan")) {
		return 0;
	}

	ctx = cdev->devdata;
	if (!ctx) {
		dev_err(&tz->device, "Cooling device context not found\n");
		return -EINVAL;
	}

	ctx->tz = tz;


	dev_info(&tz->device, "Binding cooling device: %s\n", cdev->type);

	for (i = 0; i < data->trip_count; i++) {
		int ret = thermal_zone_bind_cooling_device(tz, i, cdev,
				data->min_states[i], data->max_states[i], THERMAL_WEIGHT_DEFAULT);
		if (ret)
			dev_err(&tz->device, "Failed to bind trip %d: %d\n", i, ret);
		else
			dev_info(&tz->device, "Bound trip %d to cooling device\n", i);
	}

	return 0;
}

static int rpi_acpi_unbind(struct thermal_zone_device *tz,
                           struct thermal_cooling_device *cdev)
{
	struct rpi_acpi_thermal *data = tz->devdata;
	struct pwm_fan_ctx *ctx;
	int i;

	if (!data) {
		dev_err(&tz->device, "No thermal zone context available for unbind\n");
		return -EINVAL;
	}

	if (!strstr(cdev->type, "pwm-fan")) {
		dev_info(&tz->device, "Ignoring unmatched cooling device on unbind: %s\n", cdev->type);
		return 0;
	}

	ctx = cdev->devdata;
	if (!ctx) {
		dev_err(&tz->device, "Cooling device context not found on unbind\n");
		return -EINVAL;
	}

	dev_info(&tz->device, "Unbinding cooling device: %s\n", cdev->type);

	for (i = 0; i < data->trip_count; i++) {
		int ret = thermal_zone_unbind_cooling_device(tz, i, cdev);
		if (ret)
			dev_err(&tz->device, "Failed to unbind trip %d: %d\n", i, ret);
		else
			dev_info(&tz->device, "Unbound trip %d from cooling device\n", i);
	}

	ctx->tz = NULL;


	return 0;
}

static int rpi_acpi_get_trend(struct thermal_zone_device *tz,
                              const struct thermal_trip *trip,
                              enum thermal_trend *trend)
{
	int temp=tz->temperature;
	

	if (temp < trip->temperature - trip->hysteresis)
		*trend = THERMAL_TREND_DROPPING;
	else if (temp >= trip->temperature)
		*trend = THERMAL_TREND_RAISING;
	else
		*trend = THERMAL_TREND_STABLE;


	return 0;
}

static struct thermal_zone_device_ops rpi_acpi_thermal_ops = {
	.get_temp = rpi_acpi_get_temp,
	.bind = rpi_acpi_bind,
	.unbind = rpi_acpi_unbind,
	.get_trend = rpi_acpi_get_trend,
};

static acpi_handle find_cooling_device_handle(struct device *dev, acpi_handle parent)
{
	acpi_status status;
	struct acpi_buffer buf = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *dsd;
	acpi_handle result = NULL;

	status = acpi_evaluate_object(parent, "_DSD", NULL, &buf);
	if (ACPI_FAILURE(status))
		return NULL;

	dsd = buf.pointer;
	if (!dsd || dsd->type != ACPI_TYPE_PACKAGE || dsd->package.count < 2)
		goto out;

	union acpi_object *uuid = &dsd->package.elements[0];
	union acpi_object *props = &dsd->package.elements[1];

	if (uuid->type != ACPI_TYPE_BUFFER || props->type != ACPI_TYPE_PACKAGE)
		goto out;

	for (int i = 0; i < props->package.count; i++) {
		union acpi_object *entry = &props->package.elements[i];

		if (entry->type != ACPI_TYPE_PACKAGE || entry->package.count != 2)
			continue;

		union acpi_object *key = &entry->package.elements[0];
		union acpi_object *val = &entry->package.elements[1];

		if (key->type != ACPI_TYPE_STRING)
			continue;

		if (!strcmp(key->string.pointer, "cooling-device")) {
			if (val->type == ACPI_TYPE_LOCAL_REFERENCE) {
				result = val->reference.handle;
			} else if (val->type == ACPI_TYPE_PACKAGE &&
			           val->package.count > 0 &&
			           val->package.elements[0].type == ACPI_TYPE_LOCAL_REFERENCE) {
				result = val->package.elements[0].reference.handle;
			}
			break;
		}
	}

out:
	kfree(buf.pointer);
	return result;
}

static int rpi_acpi_probe(struct platform_device *pdev)
{
	struct rpi_acpi_thermal *data;
	struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);
	acpi_handle fan_handle;
	int ret, i;

	if (!adev)
		return -ENODEV;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->adev = adev;
	platform_set_drvdata(pdev, data);

	ret = device_property_count_u32(&pdev->dev, "active-trip-temps");
	if (ret < 0)
		return ret;
	if (ret > MAX_TRIPS)
		return -EINVAL;
	data->trip_count = ret;

	if (device_property_read_u32_array(&pdev->dev, "active-trip-temps", data->trip_temps, data->trip_count) ||
	    device_property_read_u32_array(&pdev->dev, "cooling-min-states", data->min_states, data->trip_count) ||
	    device_property_read_u32_array(&pdev->dev, "cooling-max-states", data->max_states, data->trip_count)) {
		dev_err(&pdev->dev, "Failed to read cooling properties\n");
		return -EINVAL;
	}

	ret = device_property_read_u32_array(&pdev->dev, "active-trip-hysteresis", data->trip_hyst, data->trip_count);
	if (ret < 0)
		memset(data->trip_hyst, 0, sizeof(s32) * data->trip_count);

	for (i = 0; i < data->trip_count; i++) {
		data->trips[i].type = THERMAL_TRIP_ACTIVE;
		data->trips[i].temperature = data->trip_temps[i];
		data->trips[i].hysteresis = data->trip_hyst[i];
	}

	fan_handle = find_cooling_device_handle(&pdev->dev, data->adev->handle);
	if (!fan_handle)
		return -ENODEV;

	data->cdev_adev = acpi_fetch_acpi_dev(fan_handle);
	if (!data->cdev_adev) {
		dev_err(&pdev->dev, "Cooling device companion fetch failed\n");
		return -ENODEV;
	}

	data->tzd = thermal_zone_device_register_with_trips(DRIVER_NAME,
		data->trips, data->trip_count, 0, data,
		&rpi_acpi_thermal_ops, NULL, 0, 1000);

	if (IS_ERR(data->tzd)) {
		dev_err(&pdev->dev, "Failed to register thermal zone\n");
		return PTR_ERR(data->tzd);
	}



	for (int i = 0; i < data->trip_count; i++) {
	struct thermal_trip t;
	__thermal_zone_get_trip(data->tzd, i, &t);
	dev_info(&pdev->dev, "Trip %d: temp=%d hysteresis=%d\n",
	         i, t.temperature, t.hysteresis);
}
	dev_info(&pdev->dev, "Registered thermal zone %s with %d trips\n",
	         DRIVER_NAME, data->trip_count);


	ret= thermal_zone_device_enable(data->tzd);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable thermal zone: %d\n", ret);
		thermal_zone_device_unregister(data->tzd);
		return ret;
	}

	return 0;
}

static int rpi_acpi_remove(struct platform_device *pdev)
{
	struct rpi_acpi_thermal *data = platform_get_drvdata(pdev);
	if (data && data->tzd)
		thermal_zone_device_unregister(data->tzd);
	return 0;
}

static const struct acpi_device_id rpi_acpi_ids[] = {
	{ RPI_HID, 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rpi_acpi_ids);

static struct platform_driver rpi_acpi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.acpi_match_table = rpi_acpi_ids,
	},
	.probe = rpi_acpi_probe,
	.remove = rpi_acpi_remove,
};

module_platform_driver(rpi_acpi_driver);

MODULE_AUTHOR("Richard Jeans <rich@jeansy.org>");
MODULE_DESCRIPTION("ACPI Thermal Zone driver for RPIT0001 using _DSD properties and linked ACPI cooling device");
MODULE_LICENSE("GPL");


