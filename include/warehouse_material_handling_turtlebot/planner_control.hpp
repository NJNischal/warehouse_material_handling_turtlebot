/*
 * planner_control.hpp
 *
 *  Created on: Nov 24, 2019
 *      Author: charan
 */

#ifndef INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNER_CONTROL_HPP_
#define INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNER_CONTROL_HPP_


#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class planner_control {
 public:
  struct point {
     double x_coor;
     double y_coor;
   };
  vector<point> points;
  geometry_msgs::Twist twist;
  void traversedPath();
  void publishControlVals();


};


#endif /* INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNER_CONTROL_HPP_ */
