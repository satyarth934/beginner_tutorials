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
 * A C++ implementation to demonstrate the usage of ros unit-testing.
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <beginner_tutorials/ChangeString.h>
#include <gtest/gtest.h>

/**
 * @brief Test to check if the service was initialized correctly.
 * @param TESTSuite.
 * @param serviceTesting.
 * @return None.
 */
TEST(TESTSuite, serviceTesting) {
  ros::NodeHandle nh;
  ros::ServiceClient listener = nh.serviceClient<beginner_tutorials::ChangeString>("change_string");
  
  bool exists(listener.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}

/**
 * @brief main function to initiate the TESTs.
 * @param argc count of the input characters.
 * @param argv list of inputs.
 * @return boolean value depending if the execution was successful.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


