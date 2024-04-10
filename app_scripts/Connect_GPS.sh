#!/bin/bash

source ./ros_term.sh 155 155

pushd /home/cohrint-skynet/catkin_ws

source devel/setup.bash

roslaunch reach_rs_driver reach_ros.launch

popd
