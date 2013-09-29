#!/bin/bash

# One time per second read the last line of the MCU sailboat-log/ log file
# redirect the log line to the xbee serial port device

stty -F /dev/ttyUSB1 38400 cread clocal -cstopb -parenb

REF=sailboat-log/current_logfile

while :
do
	#if [ -f "$REF" ]
	#then 
		FP=$(cat $REF)
		STR=$(tail -1 sailboat-log/$FP)
		echo $STR > /dev/ttyUSB1
	#fi
	sleep 1
done
