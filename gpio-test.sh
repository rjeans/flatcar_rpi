#!/bin/bash
# gpio-test.sh — GPIO output test script (Flatcar/libgpiod 1.x friendly)

CHIP="/dev/gpiochip0"
PIN=${1:-17}  # default to GPIO 17

echo "== Testing GPIO $PIN on $CHIP =="

# Function: one-shot gpioset with better delay
gpioset_one_shot() {
    local val=$1
    echo "Setting GPIO $PIN to $val..."
    gpioset -c "$CHIP" "$PIN=$val" &
    local pid=$!
    sleep 0.5
    kill $pid 2>/dev/null
    wait $pid 2>/dev/null
}

# Function: parse gpioget output
read_gpio() {
    local out
    out=$(gpioget -c "$CHIP" "$PIN")
    if echo "$out" | grep -q "active"; then
        echo "GPIO $PIN reads as: 1 (active)"
    else
        echo "GPIO $PIN reads as: 0 (inactive)"
    fi
}

# Start tests
read_gpio
gpioset_one_shot 1
read_gpio
gpioset_one_shot 0
read_gpio
echo "== Done =="

