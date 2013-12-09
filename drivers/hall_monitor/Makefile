obj-m += hall.o

CROSS = arm-linux-gnueabi-

KDIR = /home/b4me/MDB/Gumstix-Overo-Kernel/kernel
PWD := $(shell pwd)

all:
	make ARCH=arm -C $(KDIR) M=$(PWD) CROSS_COMPILE=$(CROSS) modules
	#scp hall.ko root@192.168.7.2:/home/root
clean:
	make -C $(KDIR) M=$(PWD) CROSS_COMPILE=$(CROSS) clean