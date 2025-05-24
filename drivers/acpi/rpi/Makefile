# Module names
obj-$(CONFIG_RPI_MAILBOX_ACPI) += rpi-mailbox.o
obj-$(CONFIG_RPI_PWM_POE_ACPI) += rpi-pwm-poe.o
obj-y += rpi-pwm-fan.o

# Only define these rules when building out-of-tree
ifneq ($(KBUILD_EXTMOD),)

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

endif

