/*
 * ImageProcessor.cpp
 *
 *  Created on: Dec 14, 2018
 *      Author: venkatraman
 */

#include "../include/ImageProcessor.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>

ImageProcessor::ImageProcessor() {
  hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

}

ImageProcessor::~ImageProcessor() {
}

intelli_bot::Pedestrians ImageProcessor::bBoxDetects(cv::Mat &imBGR) {
  imCpy = imBGR;
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
  return(pedMsg);
}

cv::Mat ImageProcessor::getDetImg() {
  return (imCpy);
}
