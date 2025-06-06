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

#define DRIVER_NAME "rpi_acpi_thermal"
#define RPI_HID     "RPIT0001"
#define MAX_TRIPS   8  // _CRT, _HOT, _PSV, _AC0.._AC4

struct rpi_acpi_thermal {
	struct thermal_zone_device *tzd;
	struct acpi_device *adev;
	int trip_temps[MAX_TRIPS]; // millidegree Celsius
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

static int rpi_acpi_parse_trip(struct rpi_acpi_thermal *data, const char *method, int *out_temp)
{
	acpi_status status;
	unsigned long long val;

	status = acpi_evaluate_integer(data->adev->handle, (acpi_string)method, NULL, &val);
	dev_info(&data->adev->dev, "ACPI name %s returned value: %llu\n", method, val);
	if (ACPI_FAILURE(status)){
		dev_err(&data->adev->dev, "Failed to evaluate %s: %d\n", method, status);
		return -ENODEV;
	}

	*out_temp = ((int)val - 2732) * 100;
	return 0;
}

static int rpi_acpi_probe(struct platform_device *pdev)
{
	struct rpi_acpi_thermal *data;
	struct thermal_trip trips[MAX_TRIPS];
	struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);
	int trip_count = 0;
	int temp;

	if (!adev) {
		dev_err(&pdev->dev, "No ACPI companion device found\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->adev = adev;
	platform_set_drvdata(pdev, data);

	if (!rpi_acpi_parse_trip(data, "_CRT", &temp)) {
		data->trip_temps[trip_count] = temp;
		trips[trip_count++] = (struct thermal_trip){
			.type = THERMAL_TRIP_CRITICAL,
			.temperature = temp,
			.hysteresis = 0,
		};
	}

	if (!rpi_acpi_parse_trip(data, "_HOT", &temp)) {
		data->trip_temps[trip_count] = temp;
		trips[trip_count++] = (struct thermal_trip){
			.type = THERMAL_TRIP_HOT,
			.temperature = temp,
			.hysteresis = 0,
		};
	}

	if (!rpi_acpi_parse_trip(data, "_PSV", &temp)) {
		data->trip_temps[trip_count] = temp;
		trips[trip_count++] = (struct thermal_trip){
			.type = THERMAL_TRIP_PASSIVE,
			.temperature = temp,
			.hysteresis = 0,
		};
	}

	for (int i = 0; i < 5; i++) {
		char method[5];
		snprintf(method, sizeof(method), "_AC%d", i);
		if (!rpi_acpi_parse_trip(data, method, &temp)) {
			data->trip_temps[trip_count] = temp;
			trips[trip_count++] = (struct thermal_trip){
				.type = THERMAL_TRIP_ACTIVE,
				.temperature = temp,
				.hysteresis = 0,
			};
		}
	}

	data->tzd = thermal_zone_device_register_with_trips(
		"rpi_acpi_thermal",
		trips,
		trip_count,
		0,
		data,
		&rpi_acpi_thermal_ops,
		NULL,
		0,
		1000
	);

	if (IS_ERR(data->tzd))
		return PTR_ERR(data->tzd);

	dev_info(&pdev->dev, "RPI ACPI thermal zone registered with %d trip points\n", trip_count);
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
MODULE_DESCRIPTION("ACPI Thermal Zone driver for RPIT0001");
MODULE_LICENSE("GPL");
