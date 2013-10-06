#!/bin/bash


echo "-> Stopping XBEE SERVER.."
ps aux | grep -ie mcu_launcher_xbee_server | awk '{print $2}' | xargs kill -9
ps aux | grep -ie xbee_server | awk '{print $2}' | xargs kill -9

echo "-> Stopping CONTROLLER.."
ps aux | grep -ie mcu_launcher_controller | awk '{print $2}' | xargs kill -9
ps aux | grep -ie controller_arm | awk '{print $2}' | xargs kill -9

echo "-> Stopping ACTUATORS process.."
ps aux | grep -ie mcu_launcher_actuators | awk '{print $2}' | xargs kill -9
ps aux | grep -ie actuators_arm | awk '{print $2}' | xargs kill -9

echo "-> Stopping U200 process.."
ps aux | grep -ie mcu_launcher_u200 | awk '{print $2}' | xargs kill -9
ps aux | grep -ie u200_arm | awk '{print $2}' | xargs kill -9


echo -e "\n.. all processes are stopped.\n"
