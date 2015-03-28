obj-m += mrf89xa.o
#KERNEL_SRC_ROOT = /lib/modules/$(shell uname -r)/build
KERNEL_SRC_ROOT = /Ephemeral/tmp/raspbian/kernel-compilation/linux

all:
	make -C $(KERNEL_SRC_ROOT) M=$(PWD) modules

modules_install:
	make -C $(KERNEL_SRC_ROOT) M=$(PWD) modules_install

clean:
	make -C $(KERNEL_SRC_ROOT) M=$(PWD) clean

