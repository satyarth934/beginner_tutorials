/**
 * BSD 3-Clause License
 * @copyright (c) 2019, Satyarth Praveen
 * All rights reserved.
 * 
 * @file    talker.cpp
 * @author  Satyarth Praveen 
 * @version 1.0
 * @brief   Publisher Node
 * @section DESCRIPTION
 * A C++ implementation to demonstrate simple sending of messages over the ROS system.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

/**
 * @brief An implementation to demonstrate simple sending of messages over the ROS system.
 * @param argc Count of parameters passes in the commandline argument.
 * @param argv An array of all commandline arguments.
 * @return 0 Return 0 for successful execution.
 */
int main(int argc, char **argv) {
  /// Initializing a ros node.
  ros::init(argc, argv, "talker");

  /// Creating an instance of the NodeHandle.
  ros::NodeHandle n;

  /// Creating a publisher with the topic name "chatter" and buffer size 1000.
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  /// Defining the loop_rate to be 10.
  ros::Rate loop_rate(10);

  /// To count the number of messages sent.
  int count = 0;
  while (ros::ok()) {
    /// This is the message object that is published on the "chatter" topic.
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Go Terps!!! " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /// The publish() function publishes the messages over the "chatter" topic.
    chatter_pub.publish(msg);

    ros::spinOnce();

    /// Sleeps at regular intervals to get desired effective frequency (10hz).
    loop_rate.sleep();
    ++count;
  }

  return 0;
}

