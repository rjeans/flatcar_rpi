obj-m += rpi-mailbox.o rpi-pwm-fan.o rpi-pwm-fan.o

KBUILD_MODPOST_WARN=1

all: modules modules_install

modules:
	make -C /usr/src/linux M=$(PWD) modules

clean:
	make -C /usr/src/linux M=$(PWD) clean

modules_install:
	make -C /usr/src/linux M=$(PWD) modules_install

