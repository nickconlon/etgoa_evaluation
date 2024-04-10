#!/bin/bash

pushd /home/cohrint-skynet/Desktop/MDRS/

source ./ros_term.sh 155 155

pushd /home/cohrint-skynet/catkin_ws/src/etgoa_evaluation/

python3 start_all.py -m mdrs -c ET-GOA

popd
