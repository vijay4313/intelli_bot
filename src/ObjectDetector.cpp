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
 *  @file    ObjectDetector.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  ObjectDetector routine file
 */

#include "../include/ObjectDetector.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>
#include "../include/ImageProcessor.h"
#include "sophus/sim3.hpp"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "intelli_bot/keyframeMsg.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "geometry_msgs/Point.h"

// static const std::string OPENCV_WINDOW = "Image window";
/**
 * @brief Constructor
 * @param none
 * @return none
 */
ObjectDetector::ObjectDetector()
    : it_(nh_) {

  // subscribe to required topic image_raw
  imSub_ = it_.subscribe("/ardrone/front/image_raw", 1,
                         &ObjectDetector::personDetector, this);
  // advertise to topic
  imPub_ = it_.advertise("/camera_person_tracker/output_video", 1);

  // subscribe to camera pose topic from LSD SLAM
  lsdSub = nh_.subscribe("/lsd_slam/liveframes", 20, &ObjectDetector::camPoseCB,
                         this);

  // Publish detected pedestrians.
  pedPub_ = nh_.advertise < intelli_bot::Pedestrians
      > ("/person_detection/pedestrians", 1000);

  pedMarkerPub = nh_.advertise < visualization_msgs::Marker
      > ("/humanMarker", 10);

  //cv::namedWindow(OPENCV_WINDOW);

}

/**
 * @brief Destructor
 * @param none
 * @return none
 */
ObjectDetector::~ObjectDetector() {
  // cv::destroyWindow(OPENCV_WINDOW);
}

/**
 * @brief routine to train
 * the ObjectDetector object
 * @param  none
 * @return none
 */
void ObjectDetector::personDetector(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cvPtr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat imBGR = cvPtr_->image;
  pedMsg = imgPr.bBoxDetects(imBGR);
  cv::Mat bBoxImg = imgPr.getDetImg();
  outMsg_ = cv_bridge::CvImage(cvPtr_->header, "bgr8", bBoxImg).toImageMsg();
  get3dMarker();
  pedPub_.publish(pedMsg);
  imPub_.publish(outMsg_);

}

intelli_bot::Pedestrians ObjectDetector::getPedMsg() {
  return (pedMsg);
}

void ObjectDetector::camPoseCB(const lsd_slam_viewer::keyframeMsgConstPtr msg) {
  memcpy(camToWorld.data(), msg->camToWorld.data(), 7 * sizeof(double));
  fx = msg->fx;
  fy = msg->fy;
  cx = msg->cx;
  cy = msg->cy;
  id = msg->id;
  height = msg->height;
  width = msg->width;
}

void ObjectDetector::get3dMarker() {
  Sophus::Matrix4f m = camToWorld.matrix();
  auto scale = camToWorld.scale();
  fxi = 1 / fx;
  fyi = 1 / fy;
  cxi = -cx / fx;
  cyi = -cy / fy;

  visualization_msgs::Marker points;
  points.header.frame_id = "/nav";
  points.header.stamp = ros::Time::now();
  points.ns = "intelli_bot";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = id;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 1;
  points.scale.y = 1;
  points.scale.z = 0.1;

  // Points are green
  points.color.g = 1.0;
  points.color.a = 1.0;

  for (unsigned i = 0; i < pedMsg.pedestrians.size(); i++) {
    int x = pedMsg.pedestrians[i].center.x;
    int y = pedMsg.pedestrians[i].center.y;
    Sophus::Vector3f pt = camToWorld
        * Sophus::Vector3f((x * fxi + cxi), (y * fyi + cyi), 1);
    geometry_msgs::Point gPt;
    gPt.x = pt[0];
    gPt.y = pt[1];
    gPt.z = pt[2];
    auto transPt = transformCam2World(gPt);

    points.points.push_back(transPt);
  }
  pedMarkerPub.publish(points);
}

geometry_msgs::Point ObjectDetector::transformCam2World(
    geometry_msgs::Point &gPt) {

  try {
    tfListener_.waitForTransform("/nav", "/ardrone_base_frontcam", ros::Time::now(),
                                 ros::Duration(10.0));
    tfListener_.lookupTransform("/nav", "/ardrone_base_frontcam", ros::Time(0),
                                nav2BasTF);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Matrix3x3 m(nav2BasTF.getRotation());

  float r11 = m.getColumn(0).getX();
  float r21 = m.getColumn(0).getY();
  float r31 = m.getColumn(0).getZ();

  float r12 = m.getColumn(1).getX();
  float r22 = m.getColumn(1).getY();
  float r32 = m.getColumn(1).getZ();

  float r13 = m.getColumn(2).getX();
  float r23 = m.getColumn(2).getY();
  float r33 = m.getColumn(2).getZ();

  float Tx = nav2BasTF.getOrigin().x();
  float Ty = nav2BasTF.getOrigin().y();
  float Tz = nav2BasTF.getOrigin().z();

  float wX = r11 * gPt.x + r12 * gPt.y + r13 * gPt.z + Tx;
  float wY = r21 * gPt.x + r22 * gPt.y + r23 * gPt.z + Ty;
  float wZ = r31 * gPt.z + r32 * gPt.y + r33 * gPt.z + Tz;

  geometry_msgs::Point transPt;

  transPt.x = wX;
  transPt.y = wY;
  transPt.z = wZ;

  return (transPt);

}

