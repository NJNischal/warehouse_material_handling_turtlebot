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
 * @file PlanningAlgorithm.cpp
 * @author Charan Karthikeyan P V (Driver), Nagireddi Jagadesh Nischal (Navigator)
 * @copyright MIT License.
 * @date 2/12/2019
 * @brief The Initialization file for the planner of turtlebot .
 */

#include "../include/warehouse_material_handling_turtlebot/PlanningAlgorithm.h"

PlanningAlgorithm::PlanningAlgorithm() {
}

PlanningAlgorithm::~PlanningAlgorithm() {
}

move_base_msgs::MoveBaseActionGoal PlanningAlgorithm::getGoal() {
  return goalMsg;
}

void PlanningAlgorithm::setGoalPt(double x, double y) {
  goalMsg.goal.target_pose.header.frame_id = "map";
  goalMsg.goal.target_pose.header.stamp = ros::Time::now();
  goalMsg.goal.target_pose.pose.position.x = x;
  goalMsg.goal.target_pose.pose.position.y = y;
  goalMsg.goal.target_pose.pose.position.z = 0;
  geometry_msgs::Quaternion quatVals = tf::createQuaternionMsgFromYaw(0);
  goalMsg.goal.target_pose.pose.orientation = quatVals;
}

void PlanningAlgorithm::sendGoalPt() {
  publishGoalPoint.publish(goalMsg);
}

uint8_t PlanningAlgorithm::getStatusPt() {
  return goalReachStatus;
}

void PlanningAlgorithm::subscriberStatus() {
  readSystemStatus = nodeH.subscribe("/move_base/status",200,&PlanningAlgorithm::reachedTargetStatus, this);
}

void PlanningAlgorithm::reachedTargetStatus(
    const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if(!msg->status_list.empty()) {
    actionlib_msgs::GoalStatus goalStatus = msg->status_list[0];
    goalReachStatus = goalStatus.status;
    std::cout << goalReachStatus << std::endl;
  }
}
