# Top-level Makefile for out-of-tree Raspberry Pi ACPI drivers

KDIR ?= /usr/src/linux/$(shell uname -r)
INSTALL_MOD_PATH ?=/usr
PWD := $(shell pwd)

# Kernel module object list
obj-m += rpi-mailbox.o
obj-m += rpi-pwm-poe.o
obj-m += rpi-pwm-fan.o

# Default target: build all modules
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Install modules to the current kernel's module path
install: all
	$(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) modules_install
	depmod -a

# Clean build artifacts
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
