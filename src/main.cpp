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
  // Initialize the ros
  ros::init(argc, argv, "warehouse_material_handling_turtlebot");
  // Objects for the classes
  PlanningAlgorithm planner;
  TurtlebotController controller;
  ObjectManipulation obm;
  WarehouseLocomotion wareLoco;
  int loop = 1;
  Location pickUp;
  Location dropOff;
  // Get the pick up and drop off points
  while (loop == 1) {
    int pickPt;
    int dropPt;
    ROS_INFO("ENTER THE PICK UP POINT FROM 1,2,3 :");
    std::cin >> pickPt;
    ROS_INFO("ENTER THE DROP OFF PT FROM 1,2,3 :");
    std::cin >> dropPt;
    switch (pickPt) {
      case 1:
        ROS_INFO("The choice is Point 1");
        pickUp = wareLoco.getPickPoint1();
        break;
      case 2:
        ROS_INFO("The choice is Point 2");
        pickUp = wareLoco.getPickPoint2();
        break;
      case 3:
        ROS_INFO("The choice is Point 3");
        pickUp = wareLoco.getPickPoint3();
        break;
      default:
        ROS_INFO("Invalid Argument.Re-enter choice");
        std::cin >> pickPt;
    }
    switch (dropPt) {
      case 1:
        ROS_INFO("The choice is Point 1");
        dropOff = wareLoco.getDropPoint1();
        break;
      case 2:
        ROS_INFO("The choice is Point 2");
        dropOff = wareLoco.getDropPoint2();
        break;
      case 3:
        ROS_INFO("The choice is Point 3");
        dropOff = wareLoco.getDropPoint3();
        break;
      default:
        ROS_INFO("Invalid Argument.Re-enter choice");
        std::cin >> dropPt;
        }
    // Initialize the flag status
    bool flag = true;
    // Declare and populate the struct.
    std::vector<Location> order;
    order.push_back(pickUp);
    order.push_back(dropOff);
    obm.showObject(pickUp.x, pickUp.y);
    // Publish and subscribe for the velocity.
    controller.readVel();
    planner.subscriberStatus();
    for (auto iter = order.begin(); iter != order.end(); iter++) {
      auto goal = *iter;
      // Set the goal point
      planner.setGoalPt(goal.x, goal.y);
      obm.disappearObject();
      obm.showObject(goal.x, goal.y);
      while (flag) {
        ros::Rate loop_rate(1);
        planner.sendGoalPt();
        controller.writeVel();
        auto status = planner.getStatusPt();
        if (status == 3) {
          flag = false;
          ROS_INFO("EXITTING LOOP");
        }
        ros::spinOnce();
        loop_rate.sleep();
      }
      flag = true;
    }
    ROS_INFO("Do you want to continue (yes-1; No-Anyother Number):");
    std::cin >> loop;
  }
  return 0;
}
