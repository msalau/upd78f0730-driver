obj-m = upd78f0730.o

all: upd78f0730.ko

upd78f0730.ko:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
install: upd78f0730.ko
	modprobe -r upd78f0730
	install -m 644 -t /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial upd78f0730.ko
	depmod --quick
