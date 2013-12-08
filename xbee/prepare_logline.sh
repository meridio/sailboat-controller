#!/bin/bash

# LOG FILENAME
REF=sailboat-log/current_logfile


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
	
	# PREPARE LOGLINE
	echo "#$STR,$AINF" > /tmp/logline

fi


