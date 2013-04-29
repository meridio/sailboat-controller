
all:
	#--- COMPILING [ACTUATORS] FOR x86 ---#
	gcc -Wall actuators.c -o ./bin/actuators_x86
	#--- COMPILING [ACTUATORS] FOR ARM ---#
	arm-linux-gnueabi-gcc -Wall actuators.c -o ./bin/actuators_arm
	scp ./bin/actuators_arm root@10.42.0.32:/home/root

