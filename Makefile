obj-m = upd78f0730.o

all: upd78f0730.ko

upd78f0730.ko:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean
install: upd78f0730.ko
	modprobe -r upd78f0730
	install -m 644 -t /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial upd78f0730.ko
	install -m 644 -t /etc/udev/rules.d 77-mm-up78f0730-blacklist.rules
	depmod --quick

