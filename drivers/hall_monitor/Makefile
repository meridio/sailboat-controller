obj-m += hall.o

CROSS = arm-linux-gnueabi-

KDIR = /home/b4me/MDB1/Gumstix-Overo-Kernel
PWD := $(shell pwd)

all:
	make ARCH=arm -C $(KDIR) M=$(PWD) CROSS_COMPILE=$(CROSS) modules
	scp hall.ko root@10.42.0.32:/home/root
clean:
	make -C $(KDIR) M=$(PWD) CROSS_COMPILE=$(CROSS) clean