obj-m = upd78f0730.o

all: upd78f0730.ko

upd78f0730.ko: upd78f0730.c
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

check:
	make C=2 CF="-D__CHECK_ENDIAN__" -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean

install: upd78f0730.ko
	install -m 644 -t /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial upd78f0730.ko
	install -m 644 -t /etc/udev/rules.d 77-mm-up78f0730-blacklist.rules
	depmod --quick

uninstall:
	rm -f /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/upd78f0730.ko
	rm -f /etc/udev/rules.d/77-mm-up78f0730-blacklist.rules
	depmod --quick

load:
	modprobe upd78f0730

load_debug:
	modprobe upd78f0730 dyndbg==pmf

unload:
	rmmod upd78f0730

enable_debug:
	echo -n 'module upd78f0730 +fp' > $(shell awk '$$1 == "debugfs" {print $$2}' /proc/mounts)/dynamic_debug/control
