#!/bin/bash

# One time per second read the second last line of the incoming log file
# Split the string and overwrite the /tmp/sailboat and /tmp/u200 system files


# CHECK NUMBER OF PARAMETERS
if [ $# -ne 1 ]
then
	echo "ERROR, parameter required. Usage: ./xbee_server.sh /dev/ttyUSB0"
	exit 1
fi

echo "-> INITIALIZING SERIAL PORT"
stty -F $1 9600 cread clocal -cstopb -parenb

echo "-> INITIALIZING FOLDER STRUCTURE"
mkdir -p /tmp/sailboat
mkdir -p /tmp/u200

echo "-> PREPARING THE STREAM BUFFER"
cat $1 > /tmp/xbee_stream &

sleep 2

while :
do

	if [ -f /tmp/xbee_stream ]
	then

		STR=$(tail -n 2 /tmp/xbee_stream | head -1)
		echo "Receiving: $STR"
	
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
	
		if [ $NAV -ne 5 ]
		then
			# [9] Heading
			echo $STR | cut -f9 -d, > /tmp/u200/Heading
		else
			# [9] Heading
			echo $STR | cut -f9 -d, > /tmp/sailboat/Simulated_Heading
		fi

		# [10] Pitch
		# [11] Roll

	
		if [ $NAV -ne 5 ]
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
		fi
	

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


		# [22] Area VertexNum
		VNUM=$(echo $STR | cut -f22 -d,)	
		echo $VNUM > /tmp/sailboat/Area_VertexNum

		# [23] Area Interval
		echo $STR | cut -f23 -d, > /tmp/sailboat/Area_Interval

		# [24-EOF] Area Vertex Coordinates
		if [ $VNUM -ne 0 ]; then

			for (( i=0; i<$VNUM; i++ ))
			do
				POS=$[24+(i*2)]
				LON=$(echo $STR | cut -f$POS -d, )
				LAT=$(echo $STR | cut -f$[POS+1] -d, )
				echo "$LON,$LAT" > /tmp/sailboat/Area_v$i
			done

		fi

	fi

	sleep 1
done
