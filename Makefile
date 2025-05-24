
SUBDIR := drivers/acpi/rpi
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)/$(SUBDIR) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)/$(SUBDIR) clean

