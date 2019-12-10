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
 * @file PlanningAlgorithm.h
 * @author Charan Karthikeyan P V (Driver), Nagireddi Jagadesh Nischal (Navigator)
 * @copyright MIT License.
 * @date 2/12/2019
 * @brief The Initialization file for the planner of turtlebot .
 */

#ifndef INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNINGALGORITHM_H_
#define INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNINGALGORITHM_H_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "tf/transform_broadcaster.h"
#include "move_base/move_base.h"
#include "nav_msgs/GetMap.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "nav_core/base_global_planner.h"
#include "nav_core/base_local_planner.h"

class PlanningAlgorithm {

public:

	/*
	* @brief Constructor for the plannerAlgo class.
	* @param None.
	* @return None.
	*/
	PlanningAlgorithm();
	/*
	  * @brief Destructor for the plannerAlgo class.
	  * @param None.
	  * return None.
	  */
	~PlanningAlgorithm();

	  /**
	   * @brief Set the goal (Private attribute) of type move_base_msgs::MoveBaseActionGoal
	   * @param double x The X coordinate of position of the goal sent.
	   * @param double y The Y coordinate of position of the goal sent.
	   * @return None
	   */
	  void setGoalPt(double x, double y);

	  /*
	   * @brief Function to get the Goal point from the system.
	   * @param None.
	   * @return None.
	   */
	  move_base_msgs::MoveBaseActionGoal getGoal();


	  /*
	   * @brief Function to send the goal point.
	   * @param none
	   * @return None
	   */
	  void sendGoalPt();

	  /*
	   * @brief Function to get the status of the planner
	   * @param None.
	   * @return The status value of planner.
	   */
	  uint8_t getStatusPt();

	  /*
	   * @brief Function to get the subscriber status
	   * @param None.
	   * @return None.
	   */
	  void subscriberStatus();

	  /*
	   * @brief Function to publish the cmd_vel from the planner
	   * @param pointer message of the cmd_vel.
	   * @return None.
	   */
	  void reachedTargetStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
	 private:

	  // MoveBaseAction object for the goal point.
	  move_base_msgs::MoveBaseActionGoal goalMsg;

	  // MoveBaseAction object for the startpoint.
	  move_base_msgs::MoveBaseActionGoal startPoint;

	  // nodeHandler for ros.
	  ros::NodeHandle nodeH;

	  /*
	   * @brief Function to publish the goal point.
	   * @param The node to be published to.
	   * @return None.
	   */
	  ros::Publisher publishGoalPoint = nodeH.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",100);

	  // Object to the subscriber.
	  ros::Subscriber readSystemStatus;

	  // Status for the goal reached value.
	  int goalReachStatus = 0;
	};




#endif /* INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNINGALGORITHM_H_ */
