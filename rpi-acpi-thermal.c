// SPDX-License-Identifier: GPL-2.0
/*
 * rpi-acpi-thermal.c - ACPI thermal zone driver for Raspberry Pi ACPI platforms
 *
 * Copyright (C) 2023 Richard Jeans <rich@jeansy.org>
 */

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/err.h>

#define DRIVER_NAME "rpi_acpi_thermal"
#define RPI_HID     "RPIT0001"
#define MAX_TRIPS   8

struct rpi_acpi_thermal {
	struct thermal_zone_device *tzd;
	struct acpi_device *adev;
	int trip_count;
	s32 trip_temps[MAX_TRIPS];
	s32 trip_hyst[MAX_TRIPS];
	s32 min_states[MAX_TRIPS];
	s32 max_states[MAX_TRIPS];
	struct thermal_trip trips[MAX_TRIPS];
	struct thermal_cooling_device *cdev;
};

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

static struct thermal_zone_device_ops rpi_acpi_thermal_ops = {
	.get_temp = rpi_acpi_get_temp,
};

static int rpi_acpi_probe(struct platform_device *pdev)
{
	struct rpi_acpi_thermal *data;
	struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);
	struct acpi_device *cdev_adev;
	acpi_handle fan_handle;
	int i;

	if (!adev)
		return -ENODEV;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->adev = adev;
	platform_set_drvdata(pdev, data);

	if (device_property_read_u32(&pdev->dev, "trip-count", &data->trip_count) < 0) {
		dev_err(&pdev->dev, "Missing property: trip-count\n");
		return -EINVAL;
	}

	if (data->trip_count > MAX_TRIPS) {
		dev_err(&pdev->dev, "trip-count exceeds MAX_TRIPS (%d)\n", MAX_TRIPS);
		return -EINVAL;
	}

	if (device_property_read_u32_array(&pdev->dev, "active-trip-temps", data->trip_temps, data->trip_count) < 0) {
		dev_err(&pdev->dev, "Missing property: active-trip-temps\n");
		return -EINVAL;
	}

	if (device_property_read_u32_array(&pdev->dev, "active-trip-hysteresis", data->trip_hyst, data->trip_count) < 0)
		memset(data->trip_hyst, 0, sizeof(s32) * data->trip_count);

	if (device_property_read_u32_array(&pdev->dev, "cooling-min-states", data->min_states, data->trip_count) < 0) {
		dev_err(&pdev->dev, "Missing property: cooling-min-states\n");
		return -EINVAL;
	}

	if (device_property_read_u32_array(&pdev->dev, "cooling-max-states", data->max_states, data->trip_count) < 0) {
		dev_err(&pdev->dev, "Missing property: cooling-max-states\n");
		return -EINVAL;
	}

	for (i = 0; i < data->trip_count; i++) {
		data->trips[i].type = THERMAL_TRIP_ACTIVE;
		data->trips[i].temperature = data->trip_temps[i];
		data->trips[i].hysteresis = data->trip_hyst[i];
	}

	data->tzd = thermal_zone_device_register_with_trips(DRIVER_NAME,
				data->trips, data->trip_count, 0, data,
				&rpi_acpi_thermal_ops, NULL, 0, 1000);
	if (IS_ERR(data->tzd))
		return PTR_ERR(data->tzd);

	/* Locate cooling device via _DSD-defined ACPI path */
	if (acpi_get_handle(data->adev->handle, "_DSD.CoolingDevice", &fan_handle)) {
		dev_err(&pdev->dev, "CoolingDevice handle from _DSD not found\n");
		goto unregister_tzd;
	}

	#if defined(CONFIG_ACPI)
	cdev_adev = acpi_bus_get_acpi_device(fan_handle);
#else
	cdev_adev = NULL;
#endif
	if (!cdev_adev) {
		dev_err(&pdev->dev, "Failed to resolve ACPI cooling device from fan_handle
");
		goto unregister_tzd;
	}

	data->cdev = cdev_adev->dev.driver_data;
	if (!data->cdev) {
		dev_err(&pdev->dev, "Cooling device driver data is NULL\n");
		goto unregister_tzd;
	}

	for (i = 0; i < data->trip_count; i++) {
		thermal_zone_bind_cooling_device(data->tzd, i, data->cdev,
					   data->max_states[i], data->min_states[i], 0);
	}

	dev_info(&pdev->dev, "Registered ACPI thermal zone with %d active trip(s)\n", data->trip_count);
	return 0;

unregister_tzd:
	thermal_zone_device_unregister(data->tzd);
	return -ENODEV;
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
