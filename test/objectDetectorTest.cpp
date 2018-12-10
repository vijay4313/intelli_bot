/**
 * MIT License
 *
 * Copyright (c) 2018 Venkatraman Narayanan, Amrish Baskaran
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 *
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *  @file    objectDetectorTest.cpp
 *  @author  Amrish Baskaran(amrish1222)
 *  @copyright	MIT
 *  @date    12/8/2018
 *  @brief	Routine to test ObjectDetector Class
 */

#include "../include/ObjectDetector.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

/**
 * @brief Testing function for checking the execution
 *        of the ObjectDetector class
 */
TEST(TestNodeObjectDetect, executionTest){
  // waiting for the rosbag to initiate
	ros::WallDuration(2.0).sleep();
  // creating the ObjectDetector object
  ObjectDetector detector;
  // waiting for the subscriber and publisher initialization
  ros::WallDuration(2.0).sleep();
  ros::spinOnce();
  // retrieve the info on the located pedestrian
  double x = detector.pedestrians_msg.pedestrians[0].center.x;
  double y = detector.pedestrians_msg.pedestrians[0].center.y;
  int width = detector.pedestrians_msg.pedestrians[0].width;
  int height = detector.pedestrians_msg.pedestrians[0].height;
  // Testing
  EXPECT_NEAR(185, x, 20.0);
  EXPECT_NEAR(-72, y, 20.0);
  EXPECT_NEAR(130, width, 20.0);
  EXPECT_NEAR(265, height, 20.0);
 }

