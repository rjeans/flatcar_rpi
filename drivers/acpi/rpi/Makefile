# Kernel module objects
obj-$(CONFIG_RPI_PWM_FAN_ACPI) += rpi-pwm-fan.o
obj-$(CONFIG_RPI_MAILBOX_ACPI) += rpi-mailbox.o
obj-$(CONFIG_RPI_PWM_POE_ACPI) += rpi-pwm-poe.o
obj-$(CONFIG_ACPI_THERMAL) += thermal.o

# Support for out-of-tree compilation
ifneq ($(KERNELRELEASE),)
# In-tree build
# ...existing code...
else
# Out-of-tree build
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) CONFIG_RPI_PWM_FAN_ACPI=m CONFIG_RPI_MAILBOX_ACPI=m CONFIG_RPI_PWM_POE_ACPI=m CONFIG_ACPI_THERMAL=m CONFIG_THERMAL=m modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: modules modules_install clean
endif

