#!/bin/bash

# One time per second read the last line of the MCU sailboat-log/ log file
# redirect the log line to the xbee serial port device

while :
do
	FP=$(cat sailboat-log/current_logfile)
	STR=$(tail -1 sailboat-log/$FP)
	echo $STR > /tmp/xbee_stream
	sleep 1
done
