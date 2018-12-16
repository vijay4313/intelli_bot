/*
 * ImageProcessor_test.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: venkatraman
 */

#include "ros/ros.h"
#include <string>
#include "../include/ImageProcessor.h"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>
#include "ros/package.h"

TEST(ImageProcessorTest, pedMsgTest) {
  ImageProcessor imgProc;
  // Read the file
  std::string fileName = ros::package::getPath("intelli_bot") + "/results/ImageProcTestImg.jpg";
  cv::Mat image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);
  auto pedMsg = imgProc.bBoxDetects(image);
  EXPECT_NEAR(550, pedMsg.pedestrians[0].center.x, 20);
  EXPECT_NEAR(-88, pedMsg.pedestrians[0].center.y, 20);
  EXPECT_NEAR(252, pedMsg.pedestrians[0].width, 20);
  EXPECT_NEAR(504, pedMsg.pedestrians[0].height, 20);

}

TEST(ImageProcessorTest, getImgTest) {
  std::string fileName = ros::package::getPath("intelli_bot") + "/results/ImageProcTestImg.jpg";
  ImageProcessor imgProc;
  // Read the file
  cv::Mat image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);

  auto pedMsg = imgProc.bBoxDetects(image);
  auto detImg = imgProc.getDetImg();
  EXPECT_EQ(1366, detImg.cols);
  EXPECT_EQ(768, detImg.rows);
}
