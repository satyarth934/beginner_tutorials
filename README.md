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
2) Download and build this package as follows:
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/satyarth934/beginner_tutorials.git
cd ~/catkin_ws
catkin_make
```

## Run Instructions

1) Start `roscore`.
```
source /opt/ros/kinetic/setup.bash
roscore
```

2) In a new terminal, run the `talker` node from the `beginner_tutorials` package to start publishing the data.
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
3) In another new terminal, run the `listener` node from the `beginner_tutorials` package to subscribe and process the publisher data.
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

