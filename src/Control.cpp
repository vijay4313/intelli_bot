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
 *  @file    Control.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  Control Source file
 */

#include <ros/ros.h>
#include "../include/Control.h"
#include "../include/PathPlanning.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "../include/PID.h"
#include <cmath>
#include <math.h>

/**
 * @brief Constructor for Control Class
 * Initializing required variables, publishers and subscribers
 * @param  none
 * @return none
 */
Control::Control() {
  // Getting the Path
  PathPlanning path;
  path.generatePath();
  // Getting the Path
  travPath = path.getPath();
  ROS_INFO_STREAM("path: "<<travPath[0]);

  // Publisher to cmd_vel topic
  conVel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Subscriber to /ground_truth/state topic
  currPoseSub = nh.subscribe("/ground_truth/state", 1000,
                             &Control::currPoseCallback, this);

  // Publisher for Take off
  takeOffPub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);

  // Publisher for landing
  landPub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1, true);

  // Publish empty msg to takeoff topic
  takeOffPub.publish(emptyMsg);

  // Flag to check for landing
  landFlag = false;

  // PID object for X, Z velocity and Z Angular velocity
  pidX.setKpKiKd(0.3, 0.0, 0.1);
  pidZ.setKpKiKd(0.2, 0.0, 0.1);
  pidYaw.setKpKiKd(0.5, 0.0, 0.2);

  // time for PID
  dt = 0.0;

  // setting the previous time variable
  seconds_prev = (double) ros::Time::now().toSec();
  // Computing a step to publish velocity
  computeStep();
}

/**
 * @brief Destructor for Control class
 * @param  none
 * @return none
 */
Control::~Control() {
}

/**
 * @brief gets the target location and pose
 * @param  none
 * @return Index of nearest point
 */
int Control::getTargetPos() {
  int cnt = 0, min_ptr = 0;
  // High minimum distance before iterations
  double dist_min = 100000000;

  // checking through all points for shortest distance
  for (auto &i : travPath) {
    double dist = std::pow((currPose.position.x - i.position.x), 2)
        + std::pow((currPose.position.y - i.position.y), 2)
        + std::pow((currPose.position.z - i.position.z), 2);

    // if point is closer then update output variables
    if (dist < dist_min) {
      dist_min = dist;
      min_ptr = cnt;
    }
    cnt++;
  }

  // return the shortest distance point index
  return (min_ptr);
}

/**
 * @brief Publishes the required velocity after
 * computing using PID
 * @param  none
 * @return none
 */
void Control::computeStep() {

  double distToTarget;
  //  getting the target point with index in path
  auto currTargetPtr = getTargetPos();
  ROS_INFO_STREAM("Target index: " << currTargetPtr);
  auto currTarget = travPath[currTargetPtr];

  // variables for setpoint and current point for PID
  double ep1 = 0;
  double ep2 = 0;
  double ep3 = 0;
  double setPtX = 0;
  double currentPidPtX = 0;
  double setPtZ = 0;
  double currentPidPtZ = 0;
  double setPtYaw = 0.15;
  double CurrentPtPidYaw = 0;

  // Difference in x,y coordinate position
  ep1 = currTarget.position.x - currPose.position.x;
  ep2 = currTarget.position.y - currPose.position.y;
  // Avoiding problems with divide by zero
  if (std::fabs(ep1) < 0.1) {
    ep1 = 0.001;
  }

  // Calculate Height Difference
  ep3 = currTarget.position.z - currPose.position.z;

  // Set height difference
  setPtZ = ep3;

  // Display target and current position to terminal
  ROS_INFO_STREAM("Target: " << currTarget);
  ROS_INFO_STREAM("Current Point: " << currPose);
  ROS_INFO_STREAM("ep3: " << ep3);

  // calculating distance to target
  distToTarget = std::sqrt(
      std::pow(ep1, 2) + std::pow(ep2, 2) + std::pow(ep3, 2));
  setPtX = std::sqrt(std::pow(ep1, 2) + std::pow(ep2, 2));

  // Removing point from target after getting near target
  if (distToTarget < 0.5) {
    if (travPath.empty()) {
      if (fabs(currPose.position.x) < 0.5 && fabs(currPose.position.y) < 0.5) {
        landPub.publish(emptyMsg);
        ros::shutdown();
      } else {
        geometry_msgs::Pose p;
        p.position.x = 0;
        p.position.y = 0;
        p.position.z = currTarget.position.z;
        p.orientation.x = 0;
        p.orientation.y = 0;
        p.orientation.z = 0;
        p.orientation.w = 1;
        travPath.push_back(p);
      }
    } else {
      travPath.erase(travPath.begin() + currTargetPtr);
      if (travPath.empty()) {
        if (fabs(currPose.position.x) < 0.5
            && fabs(currPose.position.y) < 0.5) {
          landPub.publish(emptyMsg);
          ros::shutdown();
        } else {
          geometry_msgs::Pose p;
          p.position.x = 0;
          p.position.y = 0;
          p.position.z = currTarget.position.z;
          p.orientation.x = 0;
          p.orientation.y = 0;
          p.orientation.z = 0;
          p.orientation.w = 1;
          travPath.push_back(p);
        }
      }
    }
  }
  // getting pose in RPY from quaternion
  geometry_msgs::Point currRPY = quat2RPY(currPose);
  double M_theta;
  // Angle from X axis using ArcTan2
  M_theta = atan2(ep2, ep1);

  // Calculating  absolute difference
  double yaw = currRPY.z;
  // Trignometry to calculate the difference in angle
  double Mag = std::sqrt(std::pow(ep1, 2) + std::pow(ep2, 2));
  double AbsDiff = acos((ep1 * cos(yaw) + ep2 * sin(yaw)) / Mag);
  double YawDiffVal = 0.15;

  // Angle between orientation of drone and target vector
  ROS_INFO_STREAM("AbsDiff: "<<AbsDiff);

  // Calculating the error for yaw using trignometry
  if (AbsDiff < 1.50) {
    // Getting turning direction using
    // cross product numerator
    double direction = (ep1 * sin(yaw) - ep2 * cos(yaw));
    int sign = 1;
    if (direction < 0) {
      sign = -1;
    }
    YawDiffVal = -1 * sign * AbsDiff;
  } else {
    YawDiffVal = 2;
  }

  // using calculated difference as set point for PID
  setPtYaw = YawDiffVal;
  // Displaying error in rotation
  ROS_INFO_STREAM("Rotation Error: "<<setPtYaw);
  // time from previous iteration
  auto seconds = (double) ros::Time::now().toSec();
  // time after previous iteration
  dt = seconds - seconds_prev;
  seconds_prev = seconds;

  // Publishing velocity after computing PID
  if (dt != 0) {
    // Setting dt for each PID object
    pidX.setDt(dt);
    pidZ.setDt(dt);
    pidYaw.setDt(dt);

    // Only sending X velocity if oriented correctly
    ROS_INFO_STREAM(fabs(setPtYaw));
    if (fabs(setPtYaw) < 0.3) {
      cmdVel.linear.x = pidX.compute(setPtX, currentPidPtX);
    } else {
      cmdVel.linear.x = 0.0;
    }
    // avoiding motion in Y axis of drone
    cmdVel.linear.y = 0.0;
    cmdVel.linear.z = pidZ.compute(setPtZ, currentPidPtZ);
    cmdVel.angular.x = 0.0;
    cmdVel.angular.y = 0.0;
    // Yaw rotation
    cmdVel.angular.z = pidYaw.compute(setPtYaw, CurrentPtPidYaw);
    // Publishing the command velocity
    conVel.publish(cmdVel);
  }
  ROS_INFO_STREAM(
      "CmdVel(x,z,yaw): " << cmdVel.linear.x <<"," << cmdVel.linear.z <<"," << cmdVel.angular.z);
  ROS_INFO_STREAM("Time " << dt);
  ros::WallDuration(0.01).sleep();
}

/**
 * @brief Converts quaternion to RPY
 * @param  Pose data in Quaternion form
 * @return Pose data in RPY
 */
geometry_msgs::Point Control::quat2RPY(geometry_msgs::Pose &quat) {
  geometry_msgs::Point rpy;
  // creating tf quaternion
  tf::Quaternion q(quat.orientation.x, quat.orientation.y, quat.orientation.z,
                   quat.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  // getting to RPY
  m.getRPY(roll, pitch, yaw);
  //Setting to required variable
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  // returning the converted pose data
  return (rpy);
}

/**
 * @brief Callback function for Odometry subscriber
 * @param  Odometry data
 * @return none
 */
void Control::currPoseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  currPose.position.x = msg->pose.pose.position.x;
  currPose.position.y = msg->pose.pose.position.y;
  currPose.position.z = msg->pose.pose.position.z;
  currPose.orientation.x = msg->pose.pose.orientation.x;
  currPose.orientation.y = msg->pose.pose.orientation.y;
  currPose.orientation.z = msg->pose.pose.orientation.z;
  currPose.orientation.w = msg->pose.pose.orientation.w;
  // Call compute
  computeStep();
}
