#!/bin/bash

# One time per second read the second last line of the incoming log file
# Split the string and overwrite the /tmp/sailboat and /tmp/u200 system files

while :
do

	STR=$(tail -n 2 /tmp/xbee_stream | head -1)
	
	# Navigation_System
	echo $STR | cut -f2 -d, > /tmp/sailboat/Navigation_System

	# Manual_Control - 3
	echo $STR | cut -f3 -d, > /tmp/sailboat/Manual_Control

	# Guidance_Heading - 4
	echo $STR | cut -f4 -d, > /tmp/sailboat/Guidance_Heading

	# Rudder_Desired_Angle - 5
	echo $STR | cut -f5 -d, > /tmp/sailboat/Rudder_Desired_Angle

	# Manual_Control_Rudder - 6
	echo $STR | cut -f6 -d, > /tmp/sailboat/Manual_Control_Rudder

	# Rudder_Feedback - 7
	echo $STR | cut -f7 -d, > /tmp/sailboat/Rudder_Feedback

	# Rate - 8

	# Heading - 9
	echo $STR | cut -f9 -d, > /tmp/u200/Heading

	# Pitch - 10
	# Roll - 11

	# Latitude - 12
	echo $STR | cut -f12 -d, > /tmp/u200/Latitude

	# Longitude - 13
	echo $STR | cut -f13 -d, > /tmp/u200/Longitude

	# COG - 14
	# SOG - 15
	echo $STR | cut -f15 -d, > /tmp/u200/SOG

	# Wind_Speed - 16
	echo $STR | cut -f16 -d, > /tmp/u200/Wind_Speed

	# Wind_Angle - 17
	echo $STR | cut -f17 -d, > /tmp/u200/Wind_Angle

	# Point_Start_Lat - 18
	echo $STR | cut -f18 -d, > /tmp/sailboat/Point_Start_Lat

	# Point_Start_Lon - 19
	echo $STR | cut -f19 -d, > /tmp/sailboat/Point_Start_Lon

	# Point_End_Lat - 20
	echo $STR | cut -f20 -d, > /tmp/sailboat/Point_End_Lat

	# Point_End_Lon - 21
	echo $STR | cut -f21 -d, > /tmp/sailboat/Point_End_Lon

	sleep 1

done
