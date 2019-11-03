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
 * A C++ implementation to demonstrate the usage of services.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <beginner_tutorials/ChangeString.h>

std::string str_msg("Go Terps :D");

bool changeString(beginner_tutorials::ChangeString::Request  &req,
                  beginner_tutorials::ChangeString::Response &res) {
  str_msg = req.in_msg;
}

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

  /// Creating a service server with the service name "change_string"
  ros::ServiceServer service = n.advertiseService("change_string", changeString);

  /// Initializing the msg to be published.
  std_msgs::String msg;

  ROS_DEBUG_STREAM("Declaration of publisher and service successful.");

  /// Defining the loop_rate to be 10.
  int looprate = 10;
  ros::Rate loop_rate(looprate);
  while (ros::ok()) {
    ROS_DEBUG_STREAM("Publishing Message: " << str_msg);
    /// setting the value of the message to be published.
    msg.data = str_msg;

    /// The publish() function publishes the messages over the "chatter" topic.
    chatter_pub.publish(msg);
    ros::spinOnce();

    /// Sleeps at regular intervals to get desired effective frequency (10hz).
    loop_rate.sleep();
  }

  return 0;
} 
