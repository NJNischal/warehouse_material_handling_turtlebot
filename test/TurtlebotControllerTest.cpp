/**
 *  MIT License
 *
 *  Copyright (c) 2019 Charan Karthikeyan P V, Nagireddi Jagadesh Nischal
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */
/**
 * @file TurtlebotControllerTest.cpp
 * @author Charan Karthikeyan P V (Navigator), Nagireddi Jagadesh Nischal (Driver)
 * @copyright MIT License.
 * @date 28/11/2019
 * @brief test cpp file for TurtlebotController class
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <memory>
#include "../include/warehouse_material_handling_turtlebot/TurtlebotController.h"
/**
 * @brief To test for methods in TurtlebotController class
 * @return None
 */
TEST(TurtlebotControllerTest, TestMethods) {
  // Object for the TurtlebotController class.
  TurtlebotController rob;

  // Twist message creation
  geometry_msgs::Twist message;

  // Assign linear velocity in x as 2
  message.linear.x = 2;

  geometry_msgs::Twist::ConstPtr velocity(new geometry_msgs::Twist(message));

  // Send velocity to callBack function
  rob.velocityMsgCallback(velocity);

  geometry_msgs::Twist vel = rob.getVel();

  // Test velocity set by the the callback function
  EXPECT_EQ(2, vel.linear.x);
}

