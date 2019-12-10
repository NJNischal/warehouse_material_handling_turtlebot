[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/Charan-Karthikeyan/warehouse_material_handling_turtlebot.svg?branch=master)](https://travis-ci.org/Charan-Karthikeyan/warehouse_material_handling_turtlebot)
[![Coverage Status](https://coveralls.io/repos/github/Charan-Karthikeyan/warehouse_material_handling_turtlebot/badge.svg?branch=master)](https://coveralls.io/github/Charan-Karthikeyan/warehouse_material_handling_turtlebot?branch=master)

# warehouse_material_handling_turtlebot

## Overview

(ACME Project codename: RapidGoBot) This package is a ROS implementation of a material handling robot for a warehouse scenario. The Package uses ROS navigation stack along with Turtlebot to autonomously navigate in a warehouse while demonstrating optimal path planning and obstacle avoidance behaviour.
This package also demonstrates the pick and place of an object with the turtlebot at certain designated locations in the custom designed world map.
 

## Personnel
Nagireddi Jagadesh Nischal: Robotics graduate student and a Mechanical engineer. Interested in Machine Learning and Planning for Autonomous Robots.

Charan Karthikeyan Parthasarathy: Robotics graduate student with an interest in Machine learning and autonomous vehicles.


## License
The Repository is Licensed under the MIT License.
```
MIT License

Copyright (c) 2019 Charan Karthikeyan Parthasarathy Vasanthi, Nagireddi Jagadesh Nischal.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## AIP Process
AIP is a process to keep track of the development process of the project and the various stages of it. This is to ensure proper documentation and coding procedure [AIP Sheet].(https://docs.google.com/spreadsheets/d/1iTVag578S2opVGFuAbI7zZNWsmBAWEnu3JR6vZmqdl8/edit#gid=0) 

## Sprint File 
The sprint is a one time boxed iteration of a continuous development cycle. The planned amount of work has to be completed within the timeframe and made ready for review. It also records the problems faced in the particular development cycle and also explains what was done to overcome the shortcomings of that particular model. The link to the file is given in the link [Sprint].(https://docs.google.com/document/d/1NIg2KEz3llf0xRq-ydAG3ZItjLUa9o6z1Mq-7OKM3RY/edit?usp=sharing)

## Requirements and Dependencies:

The project uses the following packages:
1. ROS distro: Kinetic
2. Ubuntu 16.04
3. Packages Dependencies:
All these packages are already included in the full installation of ROS kinetic. 
 * Turtlebot ROS packages
 * gmapping slam packages
 * roscpp
 * rospy
 * std_msgs
 * geometry_msgs
 * tf
 * rostest
 * rosbag
 * sensor_msgs
 * move_base_msgs

## Instruction to install ROS kinetic distro and get the dependencies up and running:
```
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init 
rosdep update
source /opt/ros/kinetic/setup.bash
``` 


## Build instructions for the package:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/Charan-Karthikeyan/warehouse_material_handling_turtlebot.git
cd ..
catkin_make
```


## Running Test for the package:
```
cd ~/catkin_ws
catkin_make run_tests warehouse_material_handling_turtlebot
```


## Instructions for running the Demo:
```
cd ~/catkin_ws
roslaunch warehouse_material_handling_turtlebot demo.launch
rosrun warehouse_material_handling_turtlebot warehouse_material_handling_turtlebot
```
The user will be prompted to enter the Pickup and Dropoff location numbers here and the robot will proceed to navigate to the desired points.


## Recording ROSBAG

Record the rostopics using the following command with the launch file:
```
roslaunch  warehouse_material_handling_turtlebot demo.launch record:=true
```
recorded bag file will be stored in the results folder and records all except camera topics, for 30 seconds.



## Running ROSBAG
We have added rosbag files for 15 and 30 seconds already in the package

Navigate to the results folder
```
cd ~/catkin_ws/src/ warehouse_material_handling_turtlebot/results
```
play the bag file
```
rosbag play turtlebotRecord.bag
```
## Project presentation file:
```
https://docs.google.com/presentation/d/1EqtoMBCBFGFMVVp5oWs3WTs19BYG9kjA4Choq2yttuA/edit?ts=5ded4c3c#slide=id.p
```



# Known issues/bugs:

 * Had issues with gmapping and localisation of robot

 * Slippage of robot during gmapping due to error in localisation in open world files.

 * Errors in spawning objects at pickup locations  and re-spawning them at drop locations in gazebo environment.

 * Ros bag too big due to multiple scan topics by camera and lasers.

 * Travis CI ROS dependencies build failure due to unknown reasons.

# Doxygen File generation
```
sudo apt-get install doxygen
doxygen -g
Open Doxygen file and source file in "INPUT" prameter and add the include and app folder
Add "*.hpp *.cpp" in the "FILE_PATTERNS" parameter in the doxygen file
Run "doxygen ./Doxyfile" in te terminal
Open html folder
open index.html
 ```



# Plugins (style guide, eclipse cpp checks integration):
- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Style

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml



