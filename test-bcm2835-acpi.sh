#!/bin/bash
set -e

echo "=== GPIO/ACPI Driver Functional Test ==="

echo "[1] Kernel messages:"
dmesg | grep -i bcm || echo "No bcm2835-related dmesg output found"

echo "[2] Checking for GPIO chip:"
gpiochip=$(gpioinfo | grep -m1 'gpiochip' | awk '{print $1}')
if [ -z "$gpiochip" ]; then
    echo "No GPIO chip found! Is the driver loaded?"
    exit 1
else
    echo "Found GPIO chip: $gpiochip"
fi

echo "[3] GPIO line info:"
gpioinfo $gpiochip | head -n 10

echo "[4] GPIO read/write test on line 17 (CAUTION - ensure this is safe):"
gpioset $gpiochip 17=1
sleep 0.2
gpioget $gpiochip 17
gpioset $gpiochip 17=0

echo "[5] IRQ mapping check:"
grep -i bcm /proc/interrupts || echo "No BCM GPIO IRQs registered"

echo "[6] Pinmux configuration (if available):"
debugfs_path="/sys/kernel/debug/pinctrl"
if [ -d "$debugfs_path" ]; then
    driver_path=$(find "$debugfs_path" -type d -name "*bcm2835*" | head -n1)
    if [ -n "$driver_path" ]; then
        echo "Found pinctrl debug directory: $driver_path"
        cat "$driver_path/pins"
    else
        echo "No bcm2835 pinctrl driver found in debugfs."
    fi
else
    echo "debugfs not mounted or unavailable. Try mounting with: mount -t debugfs none /sys/kernel/debug"
fi

echo "=== Test Complete ==="
