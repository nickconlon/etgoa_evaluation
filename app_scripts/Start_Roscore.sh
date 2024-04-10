#!/bin/bash
pushd /home/cohrint-skynet/Desktop/MDRS/
source ./ros_term.sh 155 155
source ~/catkin_ws/devel/setup.bash
roscore
