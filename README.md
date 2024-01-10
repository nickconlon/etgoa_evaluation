# ET-GOA Evaluation

This project is for a human subject study evaluation of the Event-Triggered Generalized Outcome
Assessment (ET-GOA) algorithm.

ET-GOA leverages the Generalized Outcome Assessment (GOA) and Model Quality Assessment (MQA) metrics
from Factorized Machine Self-Confidence (FaMSeC) to enable an autonomous robot to understand when
and how its competency changes during task execution. This project extends the following:

[Event-triggered robot self-assessment to aid in autonomy adjustment](https://www.frontiersin.org/articles/10.3389/frobt.2023.1294533/full)

[Dynamic Competency Self-Assessment for Autonomous Agents](https://arxiv.org/abs/2303.01646)

### Installation
Python: 3.8.10

ROS: noetic

Webots: R2023a

### Usage
Start some ROS environment (I use the base Jackal): 
```commandline
$ roslaunch jackal_gazebo empty_world.launch
```

Start the waypoint follower:
```commandline
$ python3 ./motion_planning/waypoint_follower.py
```

Adjust any settings and chose the experimental condition: Telemetry only (TELEM), per-mission
Generalized Outcome Assessment (GOA), or pre-mission GOA plus the ET-GOA algorithm (ET-GOA).
```commandline
$ vi settings.yaml
```

Start the user interface:
```commandline
$ vi settings.yaml
$ python3 ./ros_interface_impl.py
```

The interface should be started. Telemetry should be flowing. Select a POI with the dropdown, plan, 
and accept the plan to unlock the drive button. Drive button signals the robot to drive autonomously 
along the planned waypoints to the POI.