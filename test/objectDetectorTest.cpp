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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <intelli_bot/Pedestrians.h>
#include <intelli_bot/bbox.h>
#include "../include/ImageProcessor.h"
#include "sophus/sim3.hpp"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "geometry_msgs/Point.h"

class point3D {
public:
  void point3dCB(const visualization_msgs::Marker points) {
    markrPtsX = points.points[0].x;
    markrPtsY = points.points[0].y;
    markrPtsZ = points.points[0].z;
  }
  double markrPtsX;
  double markrPtsY;
  double markrPtsZ;
};

TEST(TestNodeObjectDetect, detectionImageTest) {
  ros::WallDuration(2.0).sleep();
  ObjectDetector detector;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/nav", "/ardrone_base_frontcam"));

  ros::WallDuration(2.0).sleep();
  ros::spinOnce();
  double x = detector.getPedMsg().pedestrians[0].center.x;
  double y = detector.getPedMsg().pedestrians[0].center.y;
  int width = detector.getPedMsg().pedestrians[0].width;
  int height = detector.getPedMsg().pedestrians[0].height;
  EXPECT_NEAR(185, x, 20.0);
  EXPECT_NEAR(-72,y, 20.0);
  EXPECT_NEAR(130, width, 20.0);
  EXPECT_NEAR(265, height, 20.0);
}

TEST(TestNodeObjectDetect, detection3DPointTest) {
  ros::NodeHandle nh;
  point3D ptCBHandle;
  ros::WallDuration(2.0).sleep();
  ObjectDetector detector;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/nav", "/ardrone_base_frontcam"));

  ros::WallDuration(2.0).sleep();
  ros::Subscriber pointSub = nh.subscribe("/humanMarker", 20, &point3D::point3dCB,
      &ptCBHandle);
  ros::spinOnce();
  EXPECT_NEAR(0, ptCBHandle.markrPtsX, 5.0);
  EXPECT_NEAR(-68,ptCBHandle.markrPtsY, 5.0);
  EXPECT_NEAR(0, ptCBHandle.markrPtsZ, 5.0);
}
