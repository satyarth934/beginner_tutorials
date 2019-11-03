<h1 align="center"> ROS Tutorial: Simple implementation of Publisher and Subscriber Node
</h1>
ENPM808x - ROS Programming Assignment

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

This is a simple implementation of ROS publisher/subscriber node. 
The package consists of two nodes, a talker (the publisher) and a listener (the subscriber).
Talker node: Publishes a message of string type on the topic name "chatter".
Listener node: Subscribes and processes the message published by the Publisher. The processing here is kept simple for demonstration purposes and just prints the subscribed message on the terminal.

## Dependencies

This package has been tested in a system with following dependencies.
- Ubuntu 16.04 LTS
- ROS-Kinetic distro

## Build Instructions

1) Install ROS-Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)
2) Download and checkout to the relevant branch:
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/satyarth934/beginner_tutorials.git
cd beginner_tutorials
git checkout Week10_HW
```
3) Build and source the package:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run Instructions

1) Launch the launch file with a `looprate_arg` parameter specifying the loop rate of the publisher node.
```
roslaunch beginner_tutorials beginner_tutorials.launch looprate_arg:=15
```

2) In a new terminal, run the call the service using rosservice command-line tool.
```
rosservice call /change_string "Go Terps :D"
rosservice call /change_string "Go Terps"
rosservice call /change_string "Go Dogs"
rosservice call /change_string "Go Lions"
```
