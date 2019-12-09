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
 * @file WarehouseLocomotion.cpp
 * @author Charan Karthikeyan P V (Driver), Nagireddi Jagadesh Nischal (Navigator)
 * @copyright MIT License.
 * @date 28/11/2019
 * @brief test cpp file for WarehouseLocomotion class
 */

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "../include/warehouse_material_handling_turtlebot/WarehouseLocomotion.h"

/*
 * @brief Test function to check methods of warehouselocomotion class.
 */
TEST(WarehouselocomotionTest, testingClassMethods) {
  // object for WarehouseLocomotion class
    WarehouseLocomotion n;

  // checking Location for drop point 1
  Location pos1 = n.getDropPoint1();
  EXPECT_EQ(3.0, pos1.x);
  EXPECT_EQ(2.0, pos1.y);


  // checking Location for drop point 2
  Location pos2 = n.getDropPoint2();
  EXPECT_EQ(3.0, pos2.x);
  EXPECT_EQ(4.0, pos2.y);

  // checking Location for drop point 3
  Location pos3 = n.getDropPoint3();
  EXPECT_EQ(-2.0, pos3.x);
  EXPECT_EQ(4.0, pos3.y);

  // checking Location for pick point 1
  Location pos4 = n.getPickPoint1();
  EXPECT_EQ(-8.0, pos4.x);
  EXPECT_EQ(-5.0, pos4.y);

  // checking Location for pick point 2
  Location pos5 = n.getPickPoint2();
  EXPECT_EQ(-7.0, pos5.x);
  EXPECT_EQ(-7.0, pos5.y);

  // checking Location for pick point 3
  Location pos6 = n.getPickPoint3();
  EXPECT_EQ(-5.0, pos6.x);
  EXPECT_EQ(-7.0, pos6.y);
}

