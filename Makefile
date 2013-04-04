# local compilation only

all:
	gcc -Wall controller.c ./common/common.c ./sensors/weather_station.c ./sensors/canbus.c -o autopilot
