
all:
	#--- COMPILING [Controller] FOR x86 ---#
	gcc -Wall controller.c -o ./bin/controller_x86
	#--- COMPILING [Controller] FOR ARM ---#
	arm-linux-gnueabi-gcc -Wall controller.c -o ./bin/controller_arm
