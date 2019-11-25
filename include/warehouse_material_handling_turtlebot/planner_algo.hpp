/*
 * planner_algo.hpp
 *
 *  Created on: Nov 24, 2019
 *      Author: charan
 */

#ifndef INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNER_ALGO_HPP_
#define INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNER_ALGO_HPP_
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class planner_algo{
 public:
  struct point {
    double x_coor;
    double y_coor;
  };
  struct pathVals {
    geometry_msgs::Twist twist;
    double coor;
  };
  vector<point> points;
  vector<pathVals> backTrack;

  planner_algo();
  virtual ~planner_algo();
  void generatePathVals();
  void publishPathVals();
};



#endif /* INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_PLANNER_ALGO_HPP_ */
