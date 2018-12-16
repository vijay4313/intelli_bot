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
 *  @file    ImageProcessor.h
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  ImageProcessor header file
 */
#ifndef IMAGEPROCESSOR_H_
#define IMAGEPROCESSOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>

class ImageProcessor {
 private:
  /**
   * @brief Pedestrian message
   */
  intelli_bot::Pedestrians pedMsg;

  /**
   * @brief hog detector
   */
  cv::HOGDescriptor hog_;

  /**
   * @brief grey scale image
   */
  cv::Mat imGray;

  /**
   * @brief copy of image
   */
  cv::Mat imCpy;

  /**
   * @brief detected pedestrian
   */
  std::vector<cv::Rect> detPedestrian;
 public:
  /**
   * @brief Constructor
   */
  ImageProcessor();

  /**
   * @brief Destructor
   */
  virtual ~ImageProcessor();

  /**
   * @brief gets the bounding box
   */
  intelli_bot::Pedestrians bBoxDetects(cv::Mat &imBGR);

  /**
   * @brief gets the detected image
   */
  cv::Mat getDetImg();

};

#endif /* IMAGEPROCESSOR_H_ */
