<h1 align="center"> ROS Tutorial: Simple implementation of Publisher and Subscriber Node
</h1>
ENPM808x - ROS Programming Assignment

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

Agenda of this simple implementation is the demonstration of:
1. ROS Publisher/Subscriber
2. ROS Services, Logging, and Launch files
3. ROS TF, unit testing, and bag files

The package extends on the master branch nodes, where "talker" is the publisher and "listener" is the subscriber.

Talker node: Publishes a message of string type on the topic name "chatter".
Listener node: Subscribes and processes the message published by the Publisher.

In addition to the above nodes, the talker node consists of a ROS transform broadcaster, that broadcasts a tf frame called /talk with parent /world.

The test module runs a Level 2 integration test on the talker node.

The orginial launch file contains an additional node to record all rostopics. 

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
git checkout Week11_HW
```

3) Build and source the package:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run Instructions
### Services and Logging 

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

### TF transformations
1) Following commands view how different transforms are connected to each other. The following commands are run in a new terminal with the talker node running in the background terminal.
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosrun tf view_frames
evince frames.pdf
```
2) To visualize the same without making local files, `rqt_tf_tree` can be used.
```
rosrun rqt_tf_tree rqt_tf_tree
```
3) To view the tf broadcast data in the terminal, use the `tf_echo` command.
```
rosrun tf tf_echo world talk
```

### Level 2 Integration Testing
Run the following commands to check the integration test results on the `talker` rosnode.
1) To run the tests immediately after compilation:
```
cd ~/catkin_ws/
catkin_make run_tests_beginner_tutorials
```

2) To manually run the tests using the test launch file:
```
rostest beginner_tutorials talker_test.launch
```

### ROS Bags
The recorded rosbag can be found in the results directory.
1) To use rosbag instead of the `talker` node, first run the listener:
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
2) Once the listener node is up and running, open a new terminal and play the rosbag:
```
rosbag play ~/catkin_ws/src/beginner_tutorials/results/Data.bag
```

After these steps, the `listener` node should behave similar to how it behaves with the `talker` node running in the background.
