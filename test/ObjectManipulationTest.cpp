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
 * @file ObjectManipulationTest.cpp
 * @author Charan Karthikeyan P V (Driver), Nagireddi Jagadesh Nischal (Navigator)
 * @copyright MIT License.
 * @date 27/11/2019
 * @brief Cpp file for test class to manipulate objects when Turtlebot reaches pick up or drop off
 * locations in the map
 */

#include "gtest/gtest.h"
#include "ros/ros.h
#include "../include/warehouse_material_handling_turtlebot/ObjectManipulation.h"
/*
 * @brief Test function to test the string conversion.
 */
TEST(ObjectManipulationTest, stringTest) {
  ObjectManipulation obs;
  EXPECT_EQ("10", obs.setTargetPoint(10));
}
/*
 * @brief Test function to test the object spawn function.
 */
TEST(ObjectManipulationTest, showObjectTest) {
  ObjectManipulation obs;
  std::string command = "rosrun gazebo_ros spawn_model -"
      "file src/warehouse_material_handling_turtlebot/"
      "data/gazebo_models/beer/model.sdf -sdf -x 0 -y 0 -z 0 -model beer";
  EXPECT_EQ(command, obs.showObject(0, 0));
}
/*
 * @brief Test function for removing the object.
 */
TEST(ObjectManipulationTest, disappearObjectTest) {
  ObjectManipulation obs;
  std::string command = "rosservice call /gazebo/delete_model beer";
  EXPECT_EQ(command, obs.disappearObject());
}
/*
 * @brief Test function to test the object spawn function.
 */
TEST(ObjectManipulationTest, showObjectNETest) {
  ObjectManipulation obs;
  EXPECT_NE("10", obs.showObject(0, 0));
}
/*
 * @brief Test function for removing the object.
 */
TEST(ObjectManipulationTest, disappearObjectNETest) {
  ObjectManipulation obs;
  EXPECT_NE("0", obs.disappearObject());
}
