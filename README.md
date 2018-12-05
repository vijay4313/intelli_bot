# ENPM 808X - Software Development for Robotics- IntelliBot
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/vijay4313/intelli_bot.svg?branch=master)](https://travis-ci.org/vijay4313/intelli_bot)
[![Coverage Status](https://coveralls.io/repos/github/vijay4313/intelli_bot/badge.svg?branch=master)](https://coveralls.io/github/vijay4313/intelli_bot?branch=master)

## Overview
 Acme Robotics Inc. aims to explore new realms in defense robotics by developing a
quadrotor drone capable for exploring an unknown user-defined land patch and
collect intelligence (number of hostages & terrorists) all the while developing a map
of the explored area. The scope of this project is to develop the end-to-end software
package for the above-mentioned drone operation.

## Authors
 - Venkatraman Narayanan
 - Amrish Baskaran

## Dependencies
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
- [Catkin](http://wiki.ros.org/catkin)
- [Gtest](http://wiki.ros.org/gtest)
- [Rostest](http://wiki.ros.org/rostest)
- Travis CI [Documentation](https://docs.travis-ci.com/user/for-beginners/)
- Coveralls [Documentation](https://docs.coveralls.io/about-coveralls)
- [OpenCV](https://opencv.org/license.html) 
- [Rviz](http://wiki.ros.org/rviz)
- [Gazebo 7.0.0](http://gazebosim.org/)
- [tum_Simulator](http://wiki.ros.org/tum_simulator)
- [ardrone_autonomy](https://ardrone-autonomy.readthedocs.io/en/latest/)
- [LSD SLAM](https://vision.in.tum.de/research/vslam/lsdslam)
- [industrial_ci](https://github.com/ros-industrial/industrial_ci)

# Development Process
This module will be developed using the Solo Iterative Process(SIP), Test Driven Development and agile development in a 3 week sprint method.
The spreadsheet for the Product log, iteration backlog, work log and sprint details can be found in this link-[Agile Development Spreadsheet](https://docs.google.com/spreadsheets/d/1cRZ1Yc6He_yjTwrT3RzN5OtVLop9_kAH2wNIdsGixpU/edit#gid=383324177)

Notes from the sprint review sessions can be found in the link-[Sprint review Doc](https://docs.google.com/document/d/1bJjVpGoex2Z11x2BASVN002mspXWGj8SX9v-RlrVU2g/edit)

### Dependencies Installation
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
- [Catkin](http://wiki.ros.org/catkin)
- [OpenCV](https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html)
- [tum_Simulator and ardrone_autonomy](http://wiki.ros.org/tum_simulator)
- [LSD SLAM](https://github.com/kevin-george/lsd_slam/wiki/LSD-SLAM-with-ROS-and-Ubuntu-16.04)

To Do

### Preliminary Results
##### To implement prototyped human detection:

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/vijay4313/intelli_bot.git
// With other dependencies (tum_simulator, ardrone_autonomy) built into catkin_ws....
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```

In a new terminal
```
source devel/setup.bash
rosrun intelli_bot intelli_bot
```
It opens a window with detections, like:
![human_detection_example](https://github.com/vijay4313/intelli_bot/blob/master/results/human_detection.png)

##### To implement prototyped LSD SLAM:
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/vijay4313/lsd_slam.git
// With other dependencies (tum_simulator, ardrone_autonomy, intelli_bot) built into catkin_ws....
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```
In a new terminal:
```
source devel/setup.bash
rosrun lsd_slam_core live_slam /image:=/ardrone/front/image_raw _calib:=/{catkin_ws}/src/intelli_bot/include/OpenCV_example_calib.cfg
```
Open a new terminal and start the SLAM viewer
```
source devel/setup.bash
rosrun lsd_slam_viewer viewer
```
It opens a window with point cloud map, like:
![slam_map](https://github.com/vijay4313/intelli_bot/blob/master/results/map.png)
### Presentation

### Setup

### Demo Instructions

### Running Instructions

### Instructions for Running Tests

### Known issues and Bugs

### Documentation- API and Other Packages

