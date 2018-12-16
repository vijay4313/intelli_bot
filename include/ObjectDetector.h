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
 *  @file    ObjectDetector.h
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  Control header file
 */

#ifndef INTELLI_BOT_INCLUDE_OBJECTDETECTOR_H_
#define INTELLI_BOT_INCLUDE_OBJECTDETECTOR_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>
#include "../include/ImageProcessor.h"
#include "../include/Sophus/sophus/sim3.hpp"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "intelli_bot/keyframeMsg.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "geometry_msgs/Point.h"


class ObjectDetector {
 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber imSub_;
  image_transport::Publisher imPub_;
  ros::Subscriber lsdSub;
  ros::Publisher pedPub_;
  ros::Publisher pedMarkerPub;
  ImageProcessor imgPr;
  sensor_msgs::ImagePtr outMsg_;
  cv_bridge::CvImagePtr cvPtr_;
  cv::Mat imBGR;
  intelli_bot::Pedestrians pedMsg;
  Sophus::Sim3f camToWorld;
  float fx,fy,cx,cy;
  float fxi,fyi,cxi,cyi;
  tf::TransformListener tfListener_;
  tf::StampedTransform nav2BasTF;
  int id;
  int height, width;


 public:
  /**
   * @brief Constructor
   * @param none
   * @return none
   */
  ObjectDetector();

  /**
   * @brief Destructor
   * @param none
   * @return none
   */
  virtual ~ObjectDetector();

  /**
   * @brief routine to train
   * the ObjectDetector object
   * @param  none
   * @return none
   */
  void personDetector(const sensor_msgs::ImageConstPtr& msg);

  intelli_bot::Pedestrians getPedMsg();

  void get3dMarker();

  void camPoseCB(const intelli_bot::keyframeMsgConstPtr msg);

  geometry_msgs::Point transformCam2World(geometry_msgs::Point &gPt);

};

#endif /* INTELLI_BOT_INCLUDE_OBJECTDETECTOR_H_ */
