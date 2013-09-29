#!/bin/bash

# One time per second read the second last line of the incoming log file
# Split the string and overwrite the /tmp/sailboat and /tmp/u200 system files

echo "-> INITIALIZING SERIAL PORT"
stty -F /dev/ttyUSB0 38400 cread clocal -cstopb -parenb

echo "-> INITIALIZING FOLDER STRUCTURE"
mkdir -p /tmp/sailboat
mkdir -p /tmp/u200

echo "-> PREPARING THE STREAM BUFFER"
cat /dev/ttyUSB0 > /tmp/xbee_stream &

sleep 2

while :
do

	STR=$(tail -n 2 /tmp/xbee_stream | head -1)
	
	# [2] Navigation_System
	NAV=$(echo $STR | cut -f2 -d,)
	echo $NAV > /tmp/sailboat/Navigation_System

	# [3] Manual_Control
	echo $STR | cut -f3 -d, > /tmp/sailboat/Manual_Control

	# [4] Guidance_Heading
	echo $STR | cut -f4 -d, > /tmp/sailboat/Guidance_Heading

	# [5] Rudder_Desired_Angle
	echo $STR | cut -f5 -d, > /tmp/sailboat/Rudder_Desired_Angle

	# [6] Manual_Control_Rudder
	echo $STR | cut -f6 -d, > /tmp/sailboat/Manual_Control_Rudder

	# [7] Rudder_Feedback
	echo $STR | cut -f7 -d, > /tmp/sailboat/Rudder_Feedback

	# [8] Rate

	# [9] Heading
	echo $STR | cut -f9 -d, > /tmp/u200/Heading

	# [10] Pitch
	# [11] Roll


	if ["$NAV" != "5"]
	then
		# [12] Latitude
		echo $STR | cut -f12 -d, > /tmp/u200/Latitude
		# [13] Longitude
		echo $STR | cut -f13 -d, > /tmp/u200/Longitude
	else
		# [12] Latitude
		echo $STR | cut -f12 -d, > /tmp/sailboat/Simulated_Lat
		# [13] Longitude
		echo $STR | cut -f13 -d, > /tmp/sailboat/Simulated_Lon
	if
	

	# [14] COG
	# [15] SOG
	echo $STR | cut -f15 -d, > /tmp/u200/SOG

	# [16] Wind_Speed
	echo $STR | cut -f16 -d, > /tmp/u200/Wind_Speed

	# [17] Wind_Angle
	echo $STR | cut -f17 -d, > /tmp/u200/Wind_Angle

	# [18] Point_Start_Lat
	echo $STR | cut -f18 -d, > /tmp/sailboat/Point_Start_Lat

	# [19] Point_Start_Lon
	echo $STR | cut -f19 -d, > /tmp/sailboat/Point_Start_Lon

	# [20] Point_End_Lat
	echo $STR | cut -f20 -d, > /tmp/sailboat/Point_End_Lat

	# [21] Point_End_Lon
	echo $STR | cut -f21 -d, > /tmp/sailboat/Point_End_Lon

	echo "DATA: $STR"

	sleep 1

done
