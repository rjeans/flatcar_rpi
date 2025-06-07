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
#include <acpi/acpi_bus.h>  // Ensure this is included at the top

#define DRIVER_NAME "rpi_acpi_thermal"
#define RPI_HID     "RPIT0001"
#define MAX_TRIPS   8

static const guid_t dsd_guid = GUID_INIT(0xdaffd814, 0x6eba, 0x4d8c,
                                         0x8a, 0x91, 0xbc, 0x9b, 0xbf, 0x4a, 0xa3, 0x01);

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

static acpi_handle find_cooling_device_handle(struct device *dev, acpi_handle parent)
{
	acpi_status status;
	struct acpi_buffer buf = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *dsd;
	acpi_handle result = NULL;

	status = acpi_evaluate_object(parent, "_DSD", NULL, &buf);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "_DSD evaluation failed: %s\n", acpi_format_exception(status));
		return NULL;
	}

	dsd = buf.pointer;
	if (!dsd || dsd->type != ACPI_TYPE_PACKAGE) {
		dev_err(dev, "_DSD is not a package (type=%d)\n", dsd ? dsd->type : -1);
		kfree(buf.pointer);
		return NULL;
	}

	if (dsd->package.count < 2) {
		dev_err(dev, "_DSD package too short (%d elements)\n", dsd->package.count);
		kfree(buf.pointer);
		return NULL;
	}

	union acpi_object *uuid = &dsd->package.elements[0];
	union acpi_object *props = &dsd->package.elements[1];

	if (uuid->type != ACPI_TYPE_BUFFER || props->type != ACPI_TYPE_PACKAGE) {
		dev_err(dev, "_DSD UUID or properties malformed (uuid type=%d, props type=%d)\n",
		        uuid->type, props->type);
		kfree(buf.pointer);
		return NULL;
	}

	for (int i = 0; i < props->package.count; i++) {
		union acpi_object *entry = &props->package.elements[i];

		if (entry->type != ACPI_TYPE_PACKAGE || entry->package.count != 2)
			continue;

		union acpi_object *key = &entry->package.elements[0];
		union acpi_object *val = &entry->package.elements[1];

		if (key->type != ACPI_TYPE_STRING)
			continue;

		dev_info(dev, "_DSD property: %s (type %d)\n", key->string.pointer, val->type);

		if (!strcmp(key->string.pointer, "cooling-device")) {
			if (val->type == ACPI_TYPE_LOCAL_REFERENCE) {
				result = val->reference.handle;
				dev_info(dev, "Found cooling-device as direct reference\n");
			} else if (val->type == ACPI_TYPE_PACKAGE &&
			           val->package.count > 0 &&
			           val->package.elements[0].type == ACPI_TYPE_LOCAL_REFERENCE) {
				result = val->package.elements[0].reference.handle;
				dev_info(dev, "Found cooling-device inside package (count=%d)\n",
				         val->package.count);
			} else {
				dev_err(dev, "cooling-device property is not a reference or valid package (type=%d)\n", val->type);
			}
			break;
		}
	}

	if (result) {
		struct acpi_buffer path_buf = { ACPI_ALLOCATE_BUFFER, NULL };
		status = acpi_get_name(result, ACPI_FULL_PATHNAME, &path_buf);
		if (ACPI_SUCCESS(status)) {
			dev_info(dev, "Cooling-device ACPI path: %s\n", (char *)path_buf.pointer);
			kfree(path_buf.pointer);
		} else {
			dev_info(dev, "Cooling-device: ACPI path resolution failed (%s)\n",
			         acpi_format_exception(status));
		}
	} else {
		dev_err(dev, "cooling-device not found in _DSD properties\n");
	}

	kfree(buf.pointer);
	return result;
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
	int ret, i;

	if (!adev)
		return -ENODEV;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->adev = adev;
	platform_set_drvdata(pdev, data);

	/* Infer trip count */
	ret = device_property_count_u32(&pdev->dev, "active-trip-temps");
	if (ret < 0) {
		dev_err(&pdev->dev, "Missing property: active-trip-temps\n");
		return ret;
	}
	if (ret > MAX_TRIPS) {
		dev_err(&pdev->dev, "Trip count (%d) exceeds MAX_TRIPS (%d)\n", ret, MAX_TRIPS);
		return -EINVAL;
	}
	data->trip_count = ret;

	/* Read trip temperatures */
	ret = device_property_read_u32_array(&pdev->dev, "active-trip-temps", data->trip_temps, data->trip_count);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read active-trip-temps array\n");
		return ret;
	}

	/* Optional hysteresis */
	ret = device_property_read_u32_array(&pdev->dev, "active-trip-hysteresis", data->trip_hyst, data->trip_count);
	if (ret < 0) {
		dev_warn(&pdev->dev, "active-trip-hysteresis missing or invalid, defaulting to zero\n");
		memset(data->trip_hyst, 0, sizeof(s32) * data->trip_count);
	}

	/* Validate and read cooling state arrays */
	ret = check_array_length(&pdev->dev, "cooling-min-states", data->trip_count);
	if (ret)
		return ret;
	ret = check_array_length(&pdev->dev, "cooling-max-states", data->trip_count);
	if (ret)
		return ret;

	if (device_property_read_u32_array(&pdev->dev, "cooling-min-states", data->min_states, data->trip_count) < 0 ||
	    device_property_read_u32_array(&pdev->dev, "cooling-max-states", data->max_states, data->trip_count) < 0) {
		dev_err(&pdev->dev, "Failed to read cooling state arrays\n");
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
	if (IS_ERR(data->tzd)) {
		dev_err(&pdev->dev, "Failed to register thermal zone\n");
		return PTR_ERR(data->tzd);
	}

	/* Find cooling device reference */
	fan_handle = find_cooling_device_handle(&pdev->dev, data->adev->handle);
	if (!fan_handle)
		goto unregister_tzd;

	dev_info(&pdev->dev, "Found cooling device handle: %p\n", fan_handle);
	cdev_adev = acpi_fetch_acpi_dev(fan_handle);
	dev_info(&pdev->dev, "Cooling device ACPI companion: %p\n", cdev_adev);
	if (!cdev_adev || !cdev_adev->dev.driver_data) {
		dev_err(&pdev->dev, "Cooling device ACPI fetch or driver_data failed\n");
		goto unregister_tzd;
	}

	data->cdev = cdev_adev->dev.driver_data;

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
