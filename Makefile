
all:
	#--- COMPILING [Controller] FOR x86 ---#
	gcc -Wall controller.c -o ./bin/controller_x86 -lm
	#--- COMPILING [Controller] FOR ARM ---#
	arm-linux-gnueabi-gcc -Wall controller.c -o ./bin/controller_arm -lm
	scp ./bin/controller_arm root@10.42.0.32:/home/root

