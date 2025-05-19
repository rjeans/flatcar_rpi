obj-m += pinctrl-bcm2835-acpi.o pwm-bcm2835-acpi.o mailbox-bcm2835-acpi.o power-bcm2835-acpi.o clk-bcm2835-acpi.o pwm-power-test.o pwm-acpi-test.o poe-bcm2835-acpi.o

KBUILD_MODPOST_WARN=1

all: modules modules_install

modules:
	make -C /usr/src/linux M=$(PWD) modules

clean:
	make -C /usr/src/linux M=$(PWD) clean

modules_install:
	make -C /usr/src/linux M=$(PWD) modules_install

