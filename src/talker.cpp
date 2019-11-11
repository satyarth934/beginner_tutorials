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
 * A C++ implementation to demonstrate the usage of services, logging, using command-arguments, TF, unit-testing, and bag files.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <beginner_tutorials/ChangeString.h>

/**
 * @brief Global variable that stores the message to be published by the talker node.
 */
std::string str_msg("Go Terps :D");

/**
 * @brief The function that defines the functionality of the service.
 * It updates the message that is published by the talked node.
 * @param req The input give to the service.
 * In this case, the message that is required to be published.
 * @param res Not used in this case. 
 * But the processed output is stored in the res reference variable.
 * @return None.
 */
bool changeString(beginner_tutorials::ChangeString::Request  &req,
                  beginner_tutorials::ChangeString::Response &res) {
  res.out_msg = req.in_msg;
  str_msg = req.in_msg;
  return true;
}

void broadcast() {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));
  tf::Quaternion q;
  q.setRPY(0.5, 0.5, 0.5);
  transform.setRotation(q);
  br.sendTransform(
    tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
}

/**
 * @brief An implementation to demonstrate simple sending of messages 
 * over the ROS system.
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
  ros::ServiceServer service = n.advertiseService("change_string",
      changeString);

  /// Initializing the msg to be published.
  std_msgs::String msg;

  /// Ensuring successful execution till here.
  ROS_DEBUG_STREAM("Declaration of publisher and service successful.");

  /// Defining the loop_rate using the command-line argument.
  int looprate = atoi(argv[1]);
  if (looprate <= 0) {
    ROS_FATAL_STREAM("INVALID LOOPRATE (Looprate should be > 0)");
    looprate = 10;
  } else if (looprate > 100) {
    ROS_ERROR_STREAM("Looprate too high (Looprate should be <= 100 )");
    looprate = 10;
  } else if (looprate == 10) {
    ROS_WARN_STREAM("Looprate value not provided. Using the default value 10.");
  } else {
    ROS_INFO_STREAM("Looprate : " << looprate);
  }

  ros::Rate loop_rate(looprate);
  while (ros::ok()) {
    ROS_DEBUG_STREAM("Publishing Message: " << str_msg);
    /// setting the value of the message to be published.
    msg.data = str_msg;

    /// The publish() function publishes the messages over the "chatter" topic.
    chatter_pub.publish(msg);

    /// broadcaster
    broadcast();

    ros::spinOnce();

    /// Sleeps at regular intervals to get desired effective frequency (10hz).
    loop_rate.sleep();
  }

  return 0;
}
