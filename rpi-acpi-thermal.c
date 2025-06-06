// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/slab.h>

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

	// Convert tenths of Kelvin → millidegree Celsius
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

	status = acpi_evaluate_integer(data->adev->handle, (char *)method, NULL, &val);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	*out_temp = ((int)val - 2732) * 100;
	return 0;
}

static int rpi_acpi_probe(struct acpi_device *adev)
{
	struct rpi_acpi_thermal *data;
	struct thermal_trip trips[MAX_TRIPS];
	int trip_count = 0;
	int temp;

	data = devm_kzalloc(&adev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->adev = adev;
	adev->driver_data = data;

	// Parse _CRT
	if (!rpi_acpi_parse_trip(data, "_CRT", &temp)) {
		data->trip_temps[trip_count] = temp;
		trips[trip_count++] = (struct thermal_trip){
			.type = THERMAL_TRIP_CRITICAL,
			.temperature = temp,
			.hysteresis = 0,
		};
	}

	// Parse _HOT
	if (!rpi_acpi_parse_trip(data, "_HOT", &temp)) {
		data->trip_temps[trip_count] = temp;
		trips[trip_count++] = (struct thermal_trip){
			.type = THERMAL_TRIP_HOT,
			.temperature = temp,
			.hysteresis = 0,
		};
	}

	// Parse _PSV
	if (!rpi_acpi_parse_trip(data, "_PSV", &temp)) {
		data->trip_temps[trip_count] = temp;
		trips[trip_count++] = (struct thermal_trip){
			.type = THERMAL_TRIP_PASSIVE,
			.temperature = temp,
			.hysteresis = 0,
		};
	}

	// Parse _AC0.._AC4
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
		0,                      // read-only mask
		data,
		&rpi_acpi_thermal_ops,
		NULL,
		0,
		1000                   // polling interval in ms
	);

	if (IS_ERR(data->tzd))
		return PTR_ERR(data->tzd);

	dev_info(&adev->dev, "RPI ACPI thermal zone registered with %d trip points\n", trip_count);
	return 0;
}

static int rpi_acpi_remove(struct acpi_device *adev)
{
	struct rpi_acpi_thermal *data = adev->driver_data;

	if (data && data->tzd)
		thermal_zone_device_unregister(data->tzd);

	return 0;
}

static const struct acpi_device_id rpi_acpi_ids[] = {
	{ RPI_HID, 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rpi_acpi_ids);

// ✅ Flatcar-style ACPI driver callbacks
static struct acpi_driver_ops rpi_acpi_ops = {
	.add = rpi_acpi_probe,
	.remove = rpi_acpi_remove,
};

static struct acpi_driver rpi_acpi_driver = {
	.name = DRIVER_NAME,
	.class = "thermal",
	.ids = rpi_acpi_ids,
	.ops = &rpi_acpi_ops,
};

module_acpi_driver(rpi_acpi_driver);

MODULE_AUTHOR("Richard Jeans");
MODULE_DESCRIPTION("ACPI Thermal Zone driver for RPIT0001");
MODULE_LICENSE("GPL");
