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

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @brief The subscribed messages are processed in this callback function.
 * @param msg The message subscribed from the publisher.
 * @return None.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == "Go Terps :D") {
    ROS_INFO_STREAM("I heard: [" << msg->data.c_str() << "] Yaaaayyyyyy");
  } else {
    if (msg->data == "Go Terps") {
      ROS_WARN_STREAM("I heard: [" << msg->data.c_str() << "] Show some excitement please !!");
    } else if (msg->data == "Go PennState") {
      ROS_FATAL_STREAM("I heard: [" << msg->data.c_str() << "] TRAITORRR !!!");
    } else {
      ROS_ERROR_STREAM("I heard: [" << msg->data.c_str() << "] CHANGE THE MESSAGE IMMEDIATELY!!!!");
    }
  }
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

  /// ros::spin() enters a loop, pumping callbacks, until the node is shutdown.
  ros::spin();

  return 0;
}
