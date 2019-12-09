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
 * @file PlanningAlgorithmTest.cpp
 * @author Charan Karthikeyan P V (Navigator), Nagireddi Jagadesh Nischal (Driver)
 * @copyright MIT License.
 * @date 27/11/2019
 * @brief Test file for PlanningAlgorithm class methods
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include "../include/warehouse_material_handling_turtlebot/PlanningAlgorithm.h"

/**
 * @brief To test for get goal setting and getting functions
 * @param PlanningAlgorithmTest gtest framework
 * @param  testingGetterandSetterForGoal name of test
 * @return None
 */
TEST(PlanningAlgorithmTest, testingGetterandSetterForGoal) {
  PlanningAlgorithm plan;
  plan.setGoalPt(1, 2);
  auto goal = plan.getGoal();
  EXPECT_EQ(1, goal.goal.target_pose.pose.position.x);
  EXPECT_EQ(2, goal.goal.target_pose.pose.position.y);
}

/**
 * @brief To test nodehandle initialization
 * @param PlanningAlgorithmTest gtest framework
 * @param InitialErrorCheck name of test
 * @return None
 */
TEST(PlanningAlgorithmTest, InitialErrorCheck) {
  ros::NodeHandle nodeH;
  EXPECT_NO_FATAL_FAILURE(PlanningAlgorithm plan);
}

/**
 * @brief To test GoalStatusArray message
 * @param PlanningAlgorithmTest  gtest framework
 * @param callbackTest  name of test
 * @return None
 */
TEST(PlanningAlgorithmTest, callbackTest) {
	PlanningAlgorithm plan;
  actionlib_msgs::GoalStatusArray msg;

//  EXPECT_EQ(3, plan.getStatus());
}


