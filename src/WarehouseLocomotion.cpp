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
 * @author Charan Karthikeyan P V (Navigator), Nagireddi Jagadesh Nischal (Driver)
 * @copyright MIT License
 * @date 3/12/2019
 * @brief Class to send locations to the Turtlebot to navigate inside the warehouse
 */

#include "../include/warehouse_material_handling_turtlebot/WarehouseLocomotion.h"


	/*
	 * @brief The constructor for the WarehouseLocomotion class.
	 * @param None.
	 * @return None.
	 */
WarehouseLocomotion::WarehouseLocomotion() {
	dropLocation1.x = 3.0;
	dropLocation1.y = 2.0;
	dropLocation2.x = 3.0;
	dropLocation2.y = 4.0;
	dropLocation3.x = -2.0;
	dropLocation3.y = 4.0;

	pickLocation1.x = -8.0;
	pickLocation1.y = -5.0;
	pickLocation2.x = -7.0;
	pickLocation2.y = -7.0;
	pickLocation3.x = -5.0;
	pickLocation3.y = -7.0;

}

/*
 * @brief The destructor for the WarehouseLocomotion class.
 * @param None.
 * @return None.
 */
WarehouseLocomotion::~WarehouseLocomotion() {
}

Location WarehouseLocomotion::getDropPoint1() {
  return dropLocation1;
}

Location WarehouseLocomotion::getDropPoint2() {
  return dropLocation2;
}

Location WarehouseLocomotion::getDropPoint3() {
  return dropLocation3;
}


Location WarehouseLocomotion::getPickPoint1() {
  return pickLocation1;
}

Location WarehouseLocomotion::getPickPoint2() {
  return pickLocation2;
}

Location WarehouseLocomotion::getPickPoint3() {
  return pickLocation3;
}
