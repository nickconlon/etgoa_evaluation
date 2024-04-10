#!/usr/bin/env bash

pushd '/home/cohrint-skynet/catkin_ws/src/etgoa_evaluation'
python3 start_survey.py -t mdrs_after  -s ./scenarios/settings_mdrs.yaml -c ET-GOA
popd

