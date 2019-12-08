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
 * @file TurtlebotController.h
 * @author Charan Karthikeyan P V (Navigator), Nagireddi Jagadesh Nischal (Driver)
 * @copyright MIT License.
 * @date 27/11/2019
 * @brief The Initialization file to send control messaged from the move_base
 * to the turtlebot.
 */

#ifndef INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_TURTLEBOTCONTROLLER_H_
#define INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_TURTLEBOTCONTROLLER_H_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseAction.h"


class TurtlebotController {
public:
	 /*
	  * @brief The constructor for the TurtlebotController class.
	  * @param None.
	  * @return None.
	  */
	TurtlebotController();

	 /*
	  * @brief The Destructor for the TurtlebotController Class.
	  * @param None.
	  * @return None.
	  */
	~TurtlebotController();

	  /**
	   * @brief subscribing to "/navigation_velocity_smoother/raw_cmd_vel" topic
	   * to which the move_base node publishes the velocity msg.
	   * @param None.
	   * @return None.
	   */
	  void readVel();

	  /**
	   * @brief publishes to "/cmd_vel topic" which is subscribed by the turtlebot
	   * and receives velocity messages.
	   * @param None.
	   * @return None.
	   */
	  void writeVel();

	  /**
	   * @brief is a callback function for the subscriber, this function sets the command
	   * message from the move_base message.
	   * @param None.
	   * @return None.
	   */
	  void velocityMsgCallback(const geometry_msgs::Twist::ConstPtr& msg);

	 private:
	  ros::NodeHandle nodeH;  // Node handle

	  // Publisher to the the topic: cmd_vel
	  ros::Publisher publishVel = nodeH.advertise<geometry_msgs::Twist>(
	      "/cmd_vel", 1000);

	  geometry_msgs::Twist cmdVelocity;

	  ros::Subscriber subscribeVel;  // Subscribing to the topic: "/navigation_velocity_smoother/raw_cmd_vel"
};

#endif /* INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_TURTLEBOTCONTROLLER_H_ */
