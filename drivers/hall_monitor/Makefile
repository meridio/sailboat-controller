
all:
	#--- COMPILING [HALL] FOR x86 ---#
	gcc -Wall hall.c -o ./bin/hall_x86
	#--- COMPILING [HALL] FOR ARM ---#
	arm-linux-gnueabi-gcc -Wall hall.c -o ./bin/hall_arm
	scp ./bin/hall_arm root@10.42.0.32:/home/root

