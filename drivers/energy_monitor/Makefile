
all:
	#--- COMPILING [ENERGY] FOR x86 ---#
	gcc -Wall energy.c -o ./bin/energy_x86
	#--- COMPILING [ENERGY] FOR ARM ---#
	arm-linux-gnueabi-gcc -Wall energy.c -o ./bin/energy_arm
	scp ./bin/energy_arm root@10.42.0.32:/home/root

