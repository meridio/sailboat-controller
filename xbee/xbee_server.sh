#!/bin/bash

# One time per second read the last line of the MCU sailboat-log/ log file
# redirect the log line to the xbee serial port device


# LOG FILENAME
REF=sailboat-log/current_logfile


# CHECK NUMBER OF PARAMETERS
if [ $# -ne 1 ]
then
	echo "ERROR, parameter required. Usage: ./xbee_server.sh /dev/ttyUSB0"
	exit 1
fi

# INITIALIZE SERIAL PORT
stty -F $1 9600 cread clocal -cstopb -parenb

echo "XBEE Server is running.."
while :
do
	if [ -f "$REF" ]; then
 
		FP=$(cat $REF)
		STR=$(tail -1 sailboat-log/$FP)
		
		# READ NAVSYSTEM
		NAV=$(echo $STR | cut -f2 -d,)

		# ADDITIONAL AREA INFO
		AINF="0,0,0"
		
		# IF MAPPING AREA
		if [ $NAV -eq 5 ] || [ $NAV -eq 2 ]; then		
			if [ -f sailboat-log/area/area_info ]; then
				# READ THE AREA LOG LINE, SKIPPING TIMESTAMP
				AINF=$(tail -1 sailboat-log/area/area_info | cut -c12-)
			fi
		fi
		
		# SEND STRING
		echo "$STR,$AINF" > $1
	fi
	sleep 1
done
