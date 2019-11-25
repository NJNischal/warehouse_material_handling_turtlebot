/*
 * obstcaleChange.hpp
 *
 *  Created on: Nov 24, 2019
 *      Author: charan
 */

#ifndef INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_OBSTACLECHANGE_HPP_
#define INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_OBSTACLECHANGE_HPP_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class obstacleChange{
 public:
  bool obstacleStatus;
  struct obstaclePoint {
    double x_coor;
    double y_coor;
  };
  int obstacleNumber;
  void targetPtSet(int targetNumber);
  void pickUpObj(int obstacleNumber);
  void dropObj(int targetNumber);
};



#endif /* INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_OBSTACLECHANGE_HPP_ */
