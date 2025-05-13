obj-m += pinctrl-bcm2835-acpi.o pwm-bcm2835-acpi.o bcm2835-mailbox-acpi.o bcm2835-power-acpi.o bcm2835-pm-acpi.o bcm2835-thermal-acpi.o bcm2835-vchiq-acpi.o bcm2835-vchiq-mem-acpi.o

KBUILD_MODPOST_WARN=1

all: modules modules_install

modules:
	make -C /usr/src/linux M=$(PWD) modules

clean:
	make -C /usr/src/linux M=$(PWD) clean

modules_install:
	make -C /usr/src/linux M=$(PWD) modules_install

