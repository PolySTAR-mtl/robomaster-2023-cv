#!/bin/bash

# Setup env

export WS="/home/polystar/robomaster-2023-cv/ros_ws"

source /home/polystar/.bashrc
source $WS/devel/setup.bash

# Record

cd /home/polystar/bags
rosbag record -a --duration=6m
