obj-m += mrf89xa.o
#KERNEL_SRC = /lib/modules/$(shell uname -r)/build
KERNEL_SRC = /Ephemeral/r-pi/linux

all:
	make -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	make -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean

