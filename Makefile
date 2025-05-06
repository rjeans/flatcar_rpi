obj-m += pinctrl-bcm2835-acpi.o i2c-bcm2835-acpi.o i2c-dev.o 

all:
	make -C /usr/lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /usr/lib/modules/$(shell uname -r)/build M=$(PWD) clean

