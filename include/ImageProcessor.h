/*
 * ImageProcessor.h
 *
 *  Created on: Dec 14, 2018
 *      Author: venkatraman
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
  intelli_bot::Pedestrians pedMsg;
  cv::HOGDescriptor hog_;
  cv::Mat imGray;
  cv::Mat imCpy;
  std::vector<cv::Rect> detPedestrian;
 public:
  ImageProcessor();
  virtual ~ImageProcessor();
  intelli_bot::Pedestrians bBoxDetects(cv::Mat &imBGR);
  cv::Mat getDetImg();

};

#endif /* IMAGEPROCESSOR_H_ */
