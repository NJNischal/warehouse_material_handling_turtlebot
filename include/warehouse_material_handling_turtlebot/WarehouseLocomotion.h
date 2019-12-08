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
 * @file WarehouseLocomotion.h
 * @author Charan Karthikeyan P V (Navigator), Nagireddi Jagadesh Nischal (Driver)
 * @copyright MIT License.
 * @date 27/11/2019
 * @brief header file for class to send locations to the Turtlebot to navigate inside the warehouse
 */


#ifndef INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_WAREHOUSELOCOMOTION_H_
#define INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_WAREHOUSELOCOMOTION_H_

#include <vector>

/**
 * @brief struct to store locations of predefined points with x,y coordinates of the warehouse
 */
struct Location {
  double x;
  double y;
};


class WarehouseLocomotion {
public:
	/*
	 * @brief The constructor for the WarehouseLocomotion class.
	 * @param None.
	 * @return None.
	 */
	WarehouseLocomotion();

	/*
	 * @brief The destructor for the WarehouseLocomotion class.
	 * @param None.
	 * @return None.
	 */
	~WarehouseLocomotion();


	/**
	   * @brief getter for dropPoint1 (Private attribute)
	   * @param None.
	   * @return struct Position Returns struct with drop point 1
	   * location.
	   */
	Location getDropPoint1();

	/**
	   * @brief getter for dropPoint2 (Private attribute)
	   * @param None.
	   * @return struct Position Returns struct with drop point 2
	   * location.
	   */
	Location getDropPoint2();

	/**
	   * @brief getter for dropPoint3 (Private attribute)
	   * @param None.
	   * @return struct Position Returns struct with drop point 3
	   * location.
	   */
	Location getDropPoint3();

	/**
	   * @brief getter for pickPoint1 (Private attribute)
	   * @param None.
	   * @return struct Position Returns struct with pick point 1
	   * location.
	   */
	Location getPickPoint1();

	/**
	   * @brief getter for pickPoint2 (Private attribute)
	   * @param None.
	   * @return struct Position Returns struct with pick point 2
	   * location.
	   */
	Location getPickPoint2();

	/**
	   * @brief getter for pickPoint3 (Private attribute)
	   * @param None.
	   * @return struct Position Returns struct with pick point 3
	   * location.
	   */
	Location getPickPoint3();

	 private:

	Location dropLocation1;  // Struct for drop location 1
    Location dropLocation2;  // Struct for drop location 2
	Location dropLocation3;  // Struct for drop location 3

	Location pickLocation1;  // Struct for pick location 1
	Location pickLocation2;  // Struct for pick location 2
	Location pickLocation3;  // Struct for pick location 3

};

#endif /* INCLUDE_WAREHOUSE_MATERIAL_HANDLING_TURTLEBOT_WAREHOUSELOCOMOTION_H_ */
