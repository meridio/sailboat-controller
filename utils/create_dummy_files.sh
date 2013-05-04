#!/bin/bash

mkdir -p /tmp/u200
mkdir -p /tmp/sailboat


echo 0 > /tmp/sailboat/Navigation_System
echo 0 > /tmp/sailboat/Navigation_System_Rudder
echo 0 > /tmp/sailboat/Navigation_System_Sail

echo 0 > /tmp/sailboat/Manual_Control
echo 0 > /tmp/sailboat/Manual_Control_Rudder
echo 0 > /tmp/sailboat/Manual_Control_Sail

echo 0 > /tmp/sailboat/Point_Start_Lat
echo 0 > /tmp/sailboat/Point_Start_Lon
echo 0 > /tmp/sailboat/Point_End_Lat
echo 0 > /tmp/sailboat/Point_End_Lon

echo 20 > /tmp/sailboat/Rudder_Feedback



echo "-0.0291999" > /tmp/u200/Rate

echo "95.6254858" > /tmp/u200/Heading
echo "-53.1723232" > /tmp/u200/Deviation
echo "4.1784478"  > /tmp/u200/Variation

echo "1.2745389" > /tmp/u200/Yaw
echo "6.8213234" > /tmp/u200/Pitch
echo "0.4212345" > /tmp/u200/Roll

echo "54.911886" > /tmp/u200/Latitude
echo "9.782295" > /tmp/u200/Longitude

echo "95.6454545" > /tmp/u200/COG
echo "22.8454545" > /tmp/u200/SOG

echo "1.8454545" > /tmp/u200/Wind_Speed
echo "-48.9454545" > /tmp/u200/Wind_Angle


