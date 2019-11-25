/*
 * main.cpp
 *
 *  Created on: Nov 25, 2019
 *      Author: charan
 */




#include "ros/ros.h"
#include "gtest/gtest.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "talkerTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
