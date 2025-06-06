// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/slab.h>

#define DRIVER_NAME "rpec_thermal"
#define RPEC_HID    "RPIT0001"
#define MAX_TRIPS   8  // CRT, HOT, PSV, AC0..AC4

struct rpec_thermal {
	struct thermal_zone_device *tzd;
	struct acpi_device *adev;
	int trip_temps[MAX_TRIPS]; // millidegree Celsius
};

static int rpec_get_temp(void *data, int *temp)
{
	struct rpec_thermal *rpec = data;
	acpi_handle handle = rpec->adev->handle;
	unsigned long long val;
	acpi_status status;

	status = acpi_evaluate_integer(handle, "_TMP", NULL, &val);
	if (ACPI_FAILURE(status))
		return -EIO;

	// Convert tenths of Kelvin to millidegree Celsius
	*temp = ((int)val - 2732) * 100;
	return 0;
}

static struct thermal_zone_device_ops rpec_thermal_ops = {
	.get_temp = rpec_get_temp,
};

static int rpec_parse_trip(struct rpec_thermal *rpec, const char *method, int index)
{
	acpi_status status;
	unsigned long long val;

	status = acpi_evaluate_integer(rpec->adev->handle, method, NULL, &val);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	rpec->trip_temps[index] = ((int)val - 2732) * 100;
	return 0;
}

static int rpec_probe(struct acpi_device *adev)
{
	struct rpec_thermal *rpec;
	struct thermal_trip trips[MAX_TRIPS];
	int num_trips = 0, i;

	rpec = devm_kzalloc(&adev->dev, sizeof(*rpec), GFP_KERNEL);
	if (!rpec)
		return -ENOMEM;

	rpec->adev = adev;
	acpi_driver_data(adev) = rpec;

	// Define trip points
	if (!rpec_parse_trip(rpec, "_CRT", num_trips))
		trips[num_trips++] = (struct thermal_trip){
			.type = THERMAL_TRIP_CRITICAL,
			.temperature = rpec->trip_temps[num_trips],
			.hysteresis = 0,
		};
	if (!rpec_parse_trip(rpec, "_HOT", num_trips))
		trips[num_trips++] = (struct thermal_trip){
			.type = THERMAL_TRIP_HOT,
			.temperature = rpec->trip_temps[num_trips],
			.hysteresis = 0,
		};
	if (!rpec_parse_trip(rpec, "_PSV", num_trips))
		trips[num_trips++] = (struct thermal_trip){
			.type = THERMAL_TRIP_PASSIVE,
			.temperature = rpec->trip_temps[num_trips],
			.hysteresis = 0,
		};

	// Add active trips for _AC0.._AC4
	for (i = 0; i < 5; i++) {
		char name[5];
		snprintf(name, sizeof(name), "_AC%d", i);
		if (!rpec_parse_trip(rpec, name, num_trips)) {
			trips[num_trips++] = (struct thermal_trip){
				.type = THERMAL_TRIP_ACTIVE,
				.temperature = rpec->trip_temps[num_trips],
				.hysteresis = 0,
			};
		}
	}

	rpec->tzd = thermal_zone_device_register_with_trips("rpec_thermal", trips,
							    num_trips, rpec,
							    &rpec_thermal_ops,
							    NULL, 0, 1000);
	if (IS_ERR(rpec->tzd))
		return PTR_ERR(rpec->tzd);

	dev_info(&adev->dev, "RPEC thermal zone registered\n");
	return 0;
}

static int rpec_remove(struct acpi_device *adev)
{
	struct rpec_thermal *rpec = acpi_driver_data(adev);

	thermal_zone_device_unregister(rpec->tzd);
	return 0;
}

static const struct acpi_device_id rpec_ids[] = {
	{ RPEC_HID, 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rpec_ids);

static struct acpi_driver rpec_driver = {
	.name = DRIVER_NAME,
	.class = "thermal",
	.ids = rpec_ids,
	.ops = &(struct acpi_driver_ops){
		.add = rpec_probe,
		.remove = rpec_remove,
	},
};

module_acpi_driver(rpec_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("ACPI Thermal Zone driver for RPEC");
MODULE_LICENSE("GPL");
