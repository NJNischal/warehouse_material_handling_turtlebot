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
 * @file ObjectManipulation.h
 * @author Charan Karthikeyan P V (Navigator), Nagireddi Jagadesh Nischal (Driver)
 * @copyright MIT License.
 * @date 27/11/2019
 * @brief header file for class to manipulate objects when Turtlebot reaches pick up or drop off
 * locations in the map
 */

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


#ifndef SRC_OBJECTMANIPULATION_H_
#define SRC_OBJECTMANIPULATION_H_

class ObjectManipulation {
public:

	/*
	 * @brief The constructor for the ObjectManipulation class.
	 * @param None.
	 * @return None.
	 */
	ObjectManipulation();

	/*
	 * @brief The destructor for the ObjectManipulation class.
	 * @param None.
	 * @return None.
	 */
	~ObjectManipulation();

	/**
	   * @brief Converts target point to string type
	   * It is of type move_base_msgs::MoveBaseActionGoal
	   * @param double targetPoint
	   * @return std::string This is the goal point as a string
	   */
	  std::string setTargetPoint(double targetPoint);

	  /**
	   * @brief Creates a wooden block in the gazebo world at location x,y coordinates.
	   * @param double x X coordinate of position on the map to show the object
	   * @param double y Y coordinate of position on the map to show the object
	   * @return None
	   */
	  std::string showObject(double x, double y);

	  /**
	   * @brief Will make the object disappear
	   * signifying that the robot picked it
	   * @param None
	   * @return None
	   */
	  std::string disappearObject();



};

#endif /* SRC_OBJECTMANIPULATION_H_ */
