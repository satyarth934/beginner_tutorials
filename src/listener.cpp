/**
 * BSD 3-Clause License
 * @copyright (c) 2019, Satyarth Praveen
 * All rights reserved.
 * 
 * @file    listener.cpp
 * @author  Satyarth Praveen 
 * @version 1.0
 * @brief   Subscriber Node
 * @section DESCRIPTION
 * A C++ implementation to demonstrate simple receipt of messages over the ROS system.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief The subscribed messages are processed in this callback function.
 * @param msg The message subscribed from the publisher.
 * @return None.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * @bried Implementation of the subscriber module to demonstrate simple receipt of 
 * messages over the ROS system.
 * @param argc Count of parameters passes in the commandline argument.
 * @param argv An array of all commandline arguments.
 * @return 0 Return 0 for successful execution.
 */
int main(int argc, char **argv) {
  /// Initializing a ros node.
  ros::init(argc, argv, "listener");

  /// Creating an instance of the NodeHandle.
  ros::NodeHandle n;

  /// The topic "chatter" is subscribed using the subscribe() function.
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /// ros::spin() will enter a loop, pumping callbacks; until the node is shutdown.
  ros::spin();

  return 0;
}
