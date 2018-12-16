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
 */

/**
 *
 *  @file    ImageProcessor_test.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  Routine to test ImageProcessor Class
 */

#include <gtest/gtest.h>
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "ros/package.h"
#include "ros/ros.h"
#include "../include/ImageProcessor.h"

/**
 * @brief testing whether human detected at correct location
 * from image
 */
TEST(ImageProcessorTest, pedMsgTest) {
  ImageProcessor imgProc;
  // Read the file
  std::string fileName = ros::package::getPath("intelli_bot") +
  "/results/ImageProcTestImg.jpg";
  cv::Mat image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);
  auto pedMsg = imgProc.bBoxDetects(image);
  EXPECT_NEAR(550, pedMsg.pedestrians[0].center.x, 20);
  EXPECT_NEAR(-88, pedMsg.pedestrians[0].center.y, 20);
  EXPECT_NEAR(252, pedMsg.pedestrians[0].width, 20);
  EXPECT_NEAR(504, pedMsg.pedestrians[0].height, 20);
}

/**
 * @brief Checking if bounding box is place correctly on
 * a static image of a human
 */
TEST(ImageProcessorTest, getImgTest) {
  std::string fileName = ros::package::getPath("intelli_bot") +
  "/results/ImageProcTestImg.jpg";
  ImageProcessor imgProc;
  // Read the file
  cv::Mat image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);

  auto pedMsg = imgProc.bBoxDetects(image);
  auto detImg = imgProc.getDetImg();
  EXPECT_EQ(1366, detImg.cols);
  EXPECT_EQ(768, detImg.rows);
}
