obj-m += pinctrl-bcm2835-acpi.o i2c-bcm2835-acpi.o i2c-dev.o 

KBUILD_MODPOST_WARN=1

all: modules modules_install

modules:
	make -C /usr/src/linux M=$(PWD) modules

clean:
	make -C /usr/src/linux M=$(PWD) clean

modules_install:
	make -C /usr/src/linux M=$(PWD) modules_install

