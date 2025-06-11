# Raspberry Pi ACPI Thermal and Fan Control Drivers

This project implements ACPI-based kernel drivers for Raspberry Pi platforms, targeting systems such as Flatcar Linux with UEFI and ACPI support. It includes a suite of components to control the PoE HAT fan through ACPI bindings, replacing legacy Device Tree-based drivers.

## Components

### 1. rpi-mailbox.c / rpi-mailbox.h

This driver adapts the BCM2835 mailbox hardware interface to ACPI. It provides a general-purpose mailbox controller with multiple channels and implements the `mailbox_controller` and `mailbox_chan_ops` interfaces.

- It exports:
  - `rpi_mbox_request_channel()`
  - `rpi_mbox_request_firmware_channel()` (specifically for channel 8)
  - `rpi_mbox_free_channel()`
- This forms the communication layer to send/receive firmware messages (used by `rpi-pwm-poe`).

**ACPI ID**: `BCM2849`

### 2. rpi-pwm-poe.c

This is the ACPI PWM driver for the Raspberry Pi PoE HAT fan. It uses the `rpi-mailbox` interface to communicate with the firmware and set/get fan PWM values via firmware property messages.

- Implements a `pwm_chip` via `struct pwm_ops`
- Uses mailbox channel 8 to send messages with tags:
  - `0x00038049` to set PoE HAT register values
  - `0x00030049` to read PoE HAT register values
- Period is fixed at 80,000 ns (12.5 kHz)
- Supports `pwm_apply`, `get_state`, `capture`, etc.

**ACPI ID**: `BCM2853`

### 3. rpi-pwm-fan.c / rpi-pwm-fan.h

This is a PWM-based fan driver exposing both HWMON and thermal cooling interfaces.

- Provides sysfs control over fan speed (via `hwmon` API)
- Registers as a thermal cooling device via `thermal_cooling_device_register`
- Reads cooling levels from ACPI `_DSD` (`cooling-levels` property)
- Creates mutual links between thermal and cooling devices in sysfs
- Compatible with the ACPI thermal zone binding logic from `rpi-acpi-thermal.c`

**ACPI ID**: `PWMF0001`

### 4. rpi-acpi-thermal.c

This driver implements a thermal zone device based on ACPI methods like `_TMP`, `_CRT`, etc., and associates it with a cooling device defined in the `_DSD` via a `cooling-device` property.

- Parses properties:
  - `active-trip-temps`
  - `active-trip-hysteresis`
  - `cooling-min-states`
  - `cooling-max-states`
- Uses `.bind` callback in `thermal_zone_device_ops` to find and bind the cooling device (i.e., `pwm-fan`)
- Dynamically links the fan to trip points

**ACPI ID**: `RPIT0001`

## ACPI Device Relationships

```
Thermal Zone Device (RPIT0001)
|- _TMP: temperature
|- _DSD: references Cooling Device (PWMF0001)
|- active trip points
   |- Bound to cooling device: pwm-fan

Cooling Device (PWMF0001)
|- ACPI companion stores struct pwm_fan_ctx
   |- Links back to Thermal Zone for coordinated control

PWM Device (BCM2853)
|- Provides PWM backend for PoE Fan (via mailbox firmware interface)
```

## How It Works

1. Boot: ACPI tables expose devices with the above IDs and linkages.
2. Mailbox driver (`rpi-mailbox`) loads and provides a communication interface for `rpi-pwm-poe`.
3. PWM driver (`rpi-pwm-poe`) registers a `pwm_chip`, backed by firmware mailbox control.
4. Fan driver (`rpi-pwm-fan`) binds to that PWM, registers as a cooling device, and exposes sysfs and hwmon interfaces.
5. Thermal driver (`rpi-acpi-thermal`) reads ACPI thermal properties and dynamically binds to the fan device via `_DSD`.

## Building

Ensure these configs are enabled in your Flatcar or kernel build:

```
CONFIG_MAILBOX=y
CONFIG_PWM=y
CONFIG_THERMAL=y
CONFIG_HWMON=y
CONFIG_RPI_MAILBOX_ACPI=m
CONFIG_RPI_PWM_FAN_ACPI=m
CONFIG_RPI_PWM_POE_ACPI=m
CONFIG_RPI_ACPI_THERMAL=m
```

## Credits

- Derived from Broadcom and Samsung drivers, adapted for ACPI by Richard Jeans <rich@jeansy.org>.
