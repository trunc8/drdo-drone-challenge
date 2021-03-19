#!/bin/bash

#uncomment this line if you are on VM and gazebo doesn't work
#export SVGA_VGPU10=0
source ~/.bashrc
export PATH=$PATH:$HOME/ardupilot/Tools/autotest:/usr/lib/ccache
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$HOME/ardupilot_gazebo/worlds
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/ardupilot_gazebo/models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/drdo-drone-challenge/interiit21/models
roslaunch gazebo_ros iris_world.launch &
