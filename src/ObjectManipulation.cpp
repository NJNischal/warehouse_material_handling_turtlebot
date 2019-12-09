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
 * @date 27/11/2019
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
	std::string a="a";
	return a;
}

std::string ObjectManipulation::showObject(double x, double y) {
  (void) x;
  (void) y;
  std::string temp = "Void";
  return temp;
}

std::string ObjectManipulation::disappearObject() {
  std::string temp = "Void";
  return temp;
}

