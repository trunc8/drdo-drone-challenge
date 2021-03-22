#!/bin/bash

cd ~/ardupilot/ArduCopter/
#Uncomment this line if gazebo runs into a pid error or for other gazebo issues
killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
