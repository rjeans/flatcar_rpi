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
	struct thermal_trip trips[MAX_TRIPS];
	int trip_count;
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

static int rpi_acpi_add_trip(struct rpi_acpi_thermal *data, const char *method, enum thermal_trip_type type)
{
	acpi_status status;
	unsigned long long val;
	int temp;

	status = acpi_evaluate_integer(data->adev->handle, (acpi_string)method, NULL, &val);
	if (ACPI_FAILURE(status)) {
		dev_dbg(&data->adev->dev, "Trip %s not defined\n", method);
		return -ENODEV;
	}

	temp = ((int)val - 2732) * 100;
	data->trip_temps[data->trip_count] = temp;
	data->trips[data->trip_count++] = (struct thermal_trip){
		.type = type,
		.temperature = temp,
		.hysteresis = 0,
	};

	dev_info(&data->adev->dev, "Trip %s = %d m°C\n", method, temp);
	return 0;
}

static int rpi_acpi_probe(struct platform_device *pdev)
{
	struct rpi_acpi_thermal *data;
	struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);

	if (!adev) {
		dev_err(&pdev->dev, "No ACPI companion device found\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->adev = adev;
	data->trip_count = 0;
	platform_set_drvdata(pdev, data);

	rpi_acpi_add_trip(data, "_CRT", THERMAL_TRIP_CRITICAL);
	rpi_acpi_add_trip(data, "_HOT", THERMAL_TRIP_HOT);
	rpi_acpi_add_trip(data, "_PSV", THERMAL_TRIP_PASSIVE);

	for (int i = 0; i < 5; i++) {
		char method[5];
		snprintf(method, sizeof(method), "_AC%d", i);
		rpi_acpi_add_trip(data, method, THERMAL_TRIP_ACTIVE);
	}

	data->tzd = thermal_zone_device_register_with_trips(
		"rpi_acpi_thermal",
		data->trips,
		data->trip_count,
		0,
		data,
		&rpi_acpi_thermal_ops,
		NULL,
		0,
		1000
	);

	if (IS_ERR(data->tzd))
		return PTR_ERR(data->tzd);

	dev_info(&pdev->dev, "Registered thermal zone with %d trip(s)\n", data->trip_count);
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
