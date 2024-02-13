# Robot Competency Assessment User Study

This project is for an eventual human subject study evaluation of a competency aware robot system.

We use the Generalized Outcome Assessment (GOA) and Model Quality Assessment (MQA) factors from
Factorized Machine Self-Confidence (FaMSeC) and the Event-Triggered Generalized Outcome
Assessment (ET-GOA) algorithm to enable an autonomous ground robot to quantify and communicate
predicted <i>a priori</i> competency and update those predictions <i>in situ</i> during execution. 
This project extends our previous work here:

[Event-triggered robot self-assessment to aid in autonomy adjustment](https://www.frontiersin.org/articles/10.3389/frobt.2023.1294533/full)

[Dynamic Competency Self-Assessment for Autonomous Agents](https://arxiv.org/abs/2303.01646)

[Generalizing Competency Self-Assessment for Autonomous Vehicles Using Deep Reinforcement Learning](https://arc.aiaa.org/doi/10.2514/6.2022-2496)

### Installation
Ubuntu 20.04

Python: 3.8.10

ROS: noetic

Webots: R2023a

### Usage
Build with ROS:
```commandline
catkin build 
```

Start some ROS environment. For example, I use the base Jackal Gazebo for sim: 
```commandline
roslaunch jackal_gazebo empty_world.launch
roslaunch jackal_gazebo empty_world.launch config:=front_bumblebee2
```

Adjust any settings for this episode:
```commandline
vim scenarios/settings.yaml
```

Start the waypoint follower:
```commandline
start_waypoint_follower.sh
```

Start the gazebo bridge:
```commandline
start_bridge.sh
```

Start the user interface:
```commandline
start_ui.sh
```

The interface should be started. Telemetry should be flowing. Select a POI with the dropdown, plan, 
and accept the plan to unlock the autonomous mode buttons. Alternatively use the teleoperation buttons
to drive manually.

### Building
Rebuild the GUI with the pyqt5 Designer then use this command:
```commandline
pyuic5 -x ui.ui -o ui.py
```

Figuring out where webcams are attached when there are several cameras available:
```commandline
ll /dev/v4l/by-id/
```
