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
Currently pursuing Masters in Robotics at the University of Maryland. Interests include Motion Planning, Machine Learning, drones, SLAM algorithms and deep learning.

 - Amrish Baskaran
Robotics enthusiast interested in automation and control. Has 2 years experience in hobby CNC manufacturing and control software programming.
Has an Undergrad degree in Mechanical Engineering from VIT University, India currently pursuing his Master's degree in Robotics at University of Maryland College Park.


## License
License file can be found [here](https://github.com/vijay4313/intelli_bot/blob/master/LICENSE)
```
MIT License

Copyright (c) 2018 Venkatraman Narayanan, Amrish Baskaran

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

## Development Process
This module will be developed using the Solo Iterative Process(SIP), Test Driven Development and agile development in a 3 week sprint method.
The spreadsheet for the Product log, iteration backlog, work log and sprint details can be found in this link-[Agile Development Spreadsheet](https://docs.google.com/spreadsheets/d/1cRZ1Yc6He_yjTwrT3RzN5OtVLop9_kAH2wNIdsGixpU/edit#gid=383324177)

Notes from the sprint review sessions can be found in the link-[Sprint review Doc](https://docs.google.com/document/d/1bJjVpGoex2Z11x2BASVN002mspXWGj8SX9v-RlrVU2g/edit)


## Demo of Installation and running
 Demo video for the installation and execution of the provided package can be seen below-
[![presentation_video](https://img.youtube.com/vi/tS7hv_bSMsw/0.jpg)](https://youtu.be/tS7hv_bSMsw)

## Presentation Slides
 The corresponding slides for the demo can be found [here](https://docs.google.com/presentation/d/13MoS4AskVX5kfeVUKJib4nGYUCThcdXsdTdWZPlvL6I/edit).

## Dependencies
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
- Ubuntu 16.04
- [Catkin](http://wiki.ros.org/catkin)
- [Gtest](http://wiki.ros.org/gtest)
- [Rostest](http://wiki.ros.org/rostest)
- Travis CI [Documentation](https://docs.travis-ci.com/user/for-beginners/)
- Coveralls [Documentation](https://docs.coveralls.io/about-coveralls)
- [tf](http://wiki.ros.org/tf)
- [Rosbag](http://wiki.ros.org/rosbag)
- [OpenCV](https://opencv.org/license.html) 
- [Rviz](http://wiki.ros.org/rviz)
- [Gazebo 7.0.0](http://gazebosim.org/)
- [tum_Simulator](http://wiki.ros.org/tum_simulator)
- [ardrone_autonomy](https://ardrone-autonomy.readthedocs.io/en/latest/)
- [LSD SLAM](https://vision.in.tum.de/research/vslam/lsdslam)
- [industrial_ci](https://github.com/ros-industrial/industrial_ci)


### Dependencies Installation
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
- [Catkin](http://wiki.ros.org/catkin)
- [OpenCV](https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html)
- [tum_Simulator and ardrone_autonomy](http://wiki.ros.org/tum_simulator)
- LSD SLAM
```
sudo apt install libsuitesparse-dev libqglviewer-dev-qt4 ros-kinetic-libg2o  ros-kinetic-opencv3
sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so /usr/lib/x86_64-linux-gnu/libQGLViewer.so
```
Go to Catkin workspace and clone the lsd slam repository
```
roscd
git clone https://github.com/vijay4313/lsd_slam.git
cd ..
catkin_make
```
- [timed_roslaunch](http://wiki.ros.org/timed_roslaunch)

# intelli_bot package Installation
## Build Instructions
If Catkin worspace is not available or needs to be created-
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make install
source devel/setup.bash
``` 
 After creating the catkin workspace move to the src/ folder.
```
cd src/
git clone --recursive https://github.com/vijay4313/intelli_bot.git
cd ..
catkin_make
source devel/setup.bash
```

## Running tests-
Go to Catkin Workspace
```
cd ~/catkin_ws/
catkin_make run_tests intelli_bot_test
```

## Demo Instructions
The demo can be run using the commandline instructions bellow
```
cd <path_to_catkin_workspace>
source devel/setup.bash
roslaunch intelli_bot intelli_bot_demo.launch
```

## Options for running the demo launch file
- Recording rosbag with launch file. It is turned off by default
```
roslaunch intelli_bot intelli_bot_demo.launch record:= true
```
The intelli_bot_bag.bag file is saved in the results directory

- To change the world environment 
```
roslaunch intelli_bot intelli_bot_demo.launch world_name:= <Location of World.world file>
```

## Useful Hotkeys
- ```r```: Reset, will clear all displayed data.

- ```w```: Print the number of points / currently displayed points / keyframes / constraints to the console.

- ```p```: Write currently displayed points as point cloud to file lsd_slam_viewer/pc.ply, which can be opened e.g. in meshlab. Use in combination with sparsityFactor to reduce the number of points.

## Overview of Environment and process
 The simulation is achieved using Gazebo simulator.The demo consists of an environment with buildings and people(not friendly) scaterred around. The drone will take off from a known position and traverse through the area following a rectangular path. During this it will detect the Human objects, draw a bounding box around them, publish the image for viewing and record their position that will be displayed as blocks in rviz. This is accomplished using the onboard monocular camera of resolution 640 x 480. It also uses a Monocular SLAM package called LSD SLAM to map the area and display a point cloud in a viewer to the user.

Example of Human detection using the camera :
![human_detection_example](https://github.com/vijay4313/intelli_bot/blob/master/results/human_detection.png)

Example of the point cloud generated by LSD SLAM using the monocular slam :
![slam_map](https://github.com/vijay4313/intelli_bot/blob/master/results/map.png)

## Rosbag

### Record the rostopics using the following command with the launch file:
```
roslaunch intelli_bot demo.launch record:=1
```

recorded bag file will be stored in the results folder and records all except camera topics

To record for a specific time use the following command, as an example records 20 seconds.
```
roslaunch intelli_bot demo.launch record:=1 seconds:=20
```

### Running the rosbag files
Navigate to the results folder
and run the following command-
```
rosbag play intelli_bot_bag.bag
```

## Doxygen Documentation
The doxygen documentation can be generated manually using the following commands
- Installation
```
sudo apt-get install doxygen
sudo apt-get install doxygen-gui
```
- Open Doxywizard and follow the instructions to generate the required files
```
doxywizard
```

### Known issues and Bugs
- Occasional tracking loss due to sudden translation or rotational motion
- Usage of 3rd party non-ROS packages in project directories leading to reduction in coveralls percentage. Inclusion in project directory was necessary to avoid travis build errors.

