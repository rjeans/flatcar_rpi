# Kernel module objects
obj-$(CONFIG_RPI_PWM_FAN_ACPI) += rpi-pwm-fan.o
obj-$(CONFIG_RPI_MAILBOX_ACPI) += rpi-mailbox.o
obj-$(CONFIG_RPI_PWM_POE_ACPI) += rpi-pwm-poe.o
obj-$(CONFIG_RPI_ACPI_THERMAL) += rpi-acpi-thermal.o



# Support for out-of-tree compilation
ifneq ($(KERNELRELEASE),)
# In-tree build
# ...existing code...
else
# Out-of-tree build
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# Default target
default: modules_install

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) CONFIG_RPI_PWM_FAN_ACPI=m CONFIG_RPI_MAILBOX_ACPI=m CONFIG_RPI_PWM_POE_ACPI=m  CONFIG_RPI_ACPI_THERMAL=m modules

modules_install: modules
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: default modules modules_install clean
endif

