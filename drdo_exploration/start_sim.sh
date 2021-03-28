#!/bin/bash

gnome-terminal -- roslaunch drdo_exploration launch_world.launch &&
gnome-terminal -- roslaunch drdo_exploration launch_nodes.launch &&
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console