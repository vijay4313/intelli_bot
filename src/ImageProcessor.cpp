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
 *  @file    ImageProcessor.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  ImageProcessor routine file
 */

#include "../include/ImageProcessor.h"
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"

/**
 * @brief Constructor
 * @param none
 * @return none
 */
ImageProcessor::ImageProcessor() {
  hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}

/**
 * @brief Destructor
 * @param none
 * @return none
 */
ImageProcessor::~ImageProcessor() {
}

/**
 * @brief Detects human and puts a bounding box
 * around it
 * @param Image
 * @return Pedestrian message containing the location
 * of the human
 */
intelli_bot::Pedestrians ImageProcessor::bBoxDetects(cv::Mat &imBGR) {
  imCpy = imBGR;
  // Processing of image- grayscale and hysterisis
  cv::cvtColor(imCpy, imGray, CV_BGR2GRAY);
  cv::equalizeHist(imGray, imGray);

  // HOG pedestrian detector
  hog_.detectMultiScale(imGray, detPedestrian, 0.0, cv::Size(4, 4),
                        cv::Size(0, 0), 1.05, 4);

  // Publish message of location and confident of detected pedestrians.
  // Draw detections from HOG to the screen.
  intelli_bot::Pedestrians pedMsg;
  for (unsigned i = 0; i < detPedestrian.size(); i++) {
    // Draw on screen.
    cv::rectangle(imCpy, detPedestrian[i], cv::Scalar(255));

    // Add to published message.
    intelli_bot::bbox pedestrian;
    pedestrian.center.x = detPedestrian[i].x - detPedestrian[i].width / 2;
    pedestrian.center.y = detPedestrian[i].y - detPedestrian[i].height / 2;
    pedestrian.width = detPedestrian[i].width;
    pedestrian.height = detPedestrian[i].height;
    pedMsg.pedestrians.push_back(pedestrian);
  }
  // return msg
  return(pedMsg);
}

/**
 * @brief Get the detected image
 * @param none
 * @return detected image
 */
cv::Mat ImageProcessor::getDetImg() {
  return (imCpy);
}
