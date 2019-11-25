/*
 * turtlebotController.hpp
 *
 *  Created on: Nov 24, 2019
 *      Author: charan
 */

#ifndef INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_TURTLEBOTCONTROLLER_HPP_
#define INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_TURTLEBOTCONTROLLER_HPP_


#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class turtlebotController {
 public:
  struct point {
    double x_coor;
    double y_coor;
  };
  vector<point> objectPts;
  struct currentPos {
    double x_coor;
    double y_coor;
  };
  int objTransNum;
  vector<currentPos> targetPos;
  vector<int>execSteps;
  void moveBot();
  void updateCurPos();
  void planner();
  void addStates();

};


#endif /* INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_TURTLEBOTCONTROLLER_HPP_ */
