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
 *  @brief	objectDetector class test
 *  @section DESCRIPTION
 *	A simple testing routine handler
 *  for the ROS package
 *
 */

#include "../include/ObjectDetector.h"
#include <ros/ros.h>
#include <gtest/gtest.h>


TEST(TestNodeObjectDetect, executionTest){
	ObjectDetector detector;
	double x = detector.pedestrians_msg.pedestrians[1].center.x;
	double y = detector.pedestrians_msg.pedestrians[1].center.y;
	double z = detector.pedestrians_msg.pedestrians[1].center.z;
	int width = detector.pedestrians_msg.pedestrians[1].width;
	int height = detector.pedestrians_msg.pedestrians[1].height;
	EXPECT_NEAR(185, x, 10.0);
	EXPECT_NEAR(-105,y, 10.0);
	EXPECT_NEAR(0, z, 10.0);
	EXPECT_NEAR(138, width, 10.0);
	EXPECT_NEAR(275, height, 10.0);

 }


