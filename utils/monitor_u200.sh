#!/bin/bash

while true;
do

	clear

	echo " "
	echo "[Rate of Turn]                         Manual Control: "$(</tmp/sailboat/Manual_Control)
	echo "  -  Rate: "$(</tmp/u200/Rate)
	echo " "
	echo "[Vessel Heading]"
	echo "  -  Heading:   "$(</tmp/u200/Heading)
	echo "  -  Deviation: "$(</tmp/u200/Deviation)
	echo "  -  Variation: "$(</tmp/u200/Variation)
	echo " "
	echo "[Attitude]"
	echo "  -  Yaw:   "$(</tmp/u200/Yaw)
	echo "  -  Pitch: "$(</tmp/u200/Pitch)
	echo "  -  Roll:  "$(</tmp/u200/Roll)
	echo " "
	echo "[Position]"
	echo "  -  Latitude:  "$(</tmp/u200/Latitude)
	echo "  -  Longitude: "$(</tmp/u200/Longitude)
	echo " "
	echo "[COG and SOG]"
	echo "  -  COG:	"$(</tmp/u200/COG)
	echo "  -  SOG:	"$(</tmp/u200/SOG)
	echo " "
	echo "[Wind Data]"
	echo "  -  Wind_Speed: "$(</tmp/u200/Wind_Speed)
	echo "  -  Wind_Angle: "$(</tmp/u200/Wind_Angle)
	echo " "
		
	sleep 1

done

			
