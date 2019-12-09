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
 * @file ObjectManipulation.cpp
 * @author Charan Karthikeyan P V (Navigator), Nagireddi Jagadesh Nischal (Driver)
 * @copyright MIT License.
 * @date 2/12/2019
 * @brief Cpp file for class to manipulate objects when Turtlebot reaches pick up or drop off
 * locations in the map
 */

#include "../include/warehouse_material_handling_turtlebot/ObjectManipulation.h"
#include "ros/ros.h"
/*
 * @brief The constructor for the ObjectManipulation class.
 * @param None.
 * @return None.
 */
ObjectManipulation::ObjectManipulation() {
}

/*
 * @brief The destructor for the ObjectManipulation class.
 * @param None.
 * @return None.
 */
ObjectManipulation::~ObjectManipulation() {
}

std::string ObjectManipulation::setTargetPoint(double targetPoint) {
  std::ostringstream conv;
  // Convert the input to string
  conv << targetPoint;
  return conv.str();
}

std::string ObjectManipulation::showObject(double x, double y) {
  // Converting the double values to string
  std::string g_x = setTargetPoint(x);
  std::string g_y = setTargetPoint(y);
  // rosrun file for spawning the object in gazebo.
  std::string start = "rosrun gazebo_ros spawn_model "
      "-file src/warehouse_material_handling_turtlebot/"
      "data/gazebo_models/beer/model.sdf -sdf ";
  std::string x_vals = "-x ";
  std::string y_vals = " -y ";
  std::string end = " -z 0 -model beer";
  // Joining all the strings to form the command.
  std::string cmd = start+x_vals+g_x+y_vals+g_y+end;
  const char* command  = cmd.c_str();
  ROS_INFO("Set the objects in the map ");
  // Sending the command to the system.
  system(command);
  return cmd;
}

std::string ObjectManipulation::disappearObject() {
  // Rosservice to remove the model from the gazebo environment.
  std::string destroy = "rosservice call /gazebo/delete_model";
  std::string cont = " beer";
  // Joining the string to form the command.
  std::string cmd = destroy+cont;
  const char* command = cmd.c_str();
  ROS_INFO("Removed the Object from the map");
  // Send the command to the system to execute.
  system(command);
  return cmd;
}

