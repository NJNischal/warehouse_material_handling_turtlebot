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
 * @file main.cpp
 * @author Charan Karthikeyan P V (Driver), Nagireddi Jagadesh Nischal (Navigator)
 * @copyright MIT License.
 * @date 4/12/2019
 * @brief The main file for the package
 */
#include <iostream>

#include "../include/warehouse_material_handling_turtlebot/TurtlebotController.h"
#include "../include/warehouse_material_handling_turtlebot/ObjectManipulation.h"
#include "../include/warehouse_material_handling_turtlebot/PlanningAlgorithm.h"
#include "../include/warehouse_material_handling_turtlebot/WarehouseLocomotion.h"


int main(int argc, char* argv[]) {
  //Initialize the ros
  ros::init(argc, argv, "warehouse_material_handling_turtlebot");
  //Objects for the classes
  PlanningAlgorithm planner;
  TurtlebotController controller;
  ObjectManipulation obm;
  WarehouseLocomotion wareLoco;
  //Get the pick up and drop off points
  auto pickUp = wareLoco.getPickPoint1();
  auto dropOff = wareLoco.getDropPoint1();
  // Initialize the flag status
  bool flag = true;
  //Declare and populate the struct.
  std::vector<Location> order;
  order.push_back(pickUp);
  order.push_back(dropOff);
  std::cout << order.size() << std::endl;
  //Publish and subscribe for the velocity.
  controller.readVel();
  planner.subscriberStatus();
  for (auto iter = order.begin(); iter != order.end(); iter++) {
    auto goal = *iter;
    //Set the goal point
    planner.setGoalPt(goal.x, goal.y);
    std::cout << planner.getGoal() << std::endl;
    while (flag) {
      std::cout << "Initiating" << std::endl;
      ros::Rate loop_rate(1);
      planner.sendGoalPt();
      controller.writeVel();
      auto status = planner.getStatusPt();
      if (status == 3) {
        flag = false;
        obm.showObject(goal.x, goal.y);
        std::cout << "Exiting Loop" << std::endl;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    flag = true;
  }
  return 0;
}
