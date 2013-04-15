#!/bin/bash

mkdir /tmp/u200
mkdir /tmp/sailboat

echo "0" > /tmp/sailboat/Manual_Control

echo "15.1" > /tmp/u200/Rate

echo "95.6" > /tmp/u200/Heading
echo "53.1" > /tmp/u200/Deviation
echo "4.1"  > /tmp/u200/Variation

echo "1.2" > /tmp/u200/Yaw
echo "6.8" > /tmp/u200/Pitch
echo "0.4" > /tmp/u200/Roll

echo "45.26869" > /tmp/u200/Latitude
echo "9.93677" > /tmp/u200/Longitude

echo "95.6" > /tmp/u200/COG
echo "22.8" > /tmp/u200/SOG

echo "1.8" > /tmp/u200/Wind_Speed
echo "48.9" > /tmp/u200/Wind_Angle


