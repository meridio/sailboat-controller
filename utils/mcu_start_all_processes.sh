#!/bin/bash

mkdir -p sailboat-log/system

echo "-> Starting U200 process.."
nohup sh mcu_launcher_u200.sh > sailboat-log/system/u200.log &

echo "-> Starting ACTUATORS process.."
nohup sh mcu_launcher_actuators.sh > sailboat-log/system/actuators.log &

echo "-> Starting CONTROLLER.."
nohup sh mcu_launcher_controller.sh > sailboat-log/system/controller.log &

echo "-> Starting XBEE SERVER.."
nohup sh mcu_launcher_xbee_server.sh > sailboat-log/system/xbee_server.log &

sleep 1

echo -e "\n.. all processes are up and running.\n"
