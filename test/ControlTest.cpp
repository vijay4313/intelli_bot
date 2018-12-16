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
 *  @file    ControlTest.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  Routine to test Control Class
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <gtest/gtest.h>
#include <cmath>
#include "../include/Control.h"
#include "../include/PathPlanning.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "../include/PID.h"

/**
 * @brief Control test class to check
 * the working of control ros class
 */
class ControlTest {
 public:
  bool anyMsgInVelTopic = false;
  bool anyMsgInTakeOffTopic = false;
  void CmdVelCallback(const geometry_msgs::Twist &msg) {
    anyMsgInVelTopic = true;
  }
  void TakeOffCallback(const std_msgs::Empty &msg) {
    anyMsgInTakeOffTopic = true;
}
};

/**
 * @brief Testing function for checking publishing
 * of velocity and takeOff msg
 */
TEST(TestNodeControl, publishTest) {
  // waiting for the rosbag to initiate

  // NodeHandle initialization
  ros::NodeHandle nh;
  ControlTest ctest;
  ros::Subscriber TakeOffSub = nh.subscribe("/ardrone/takeoff", 1000,
                                            &ControlTest::TakeOffCallback,
                                            &ctest);
  ros::Subscriber cmdSub = nh.subscribe("cmd_vel", 1000,
                                        &ControlTest::CmdVelCallback, &ctest);
  // waiting for subscribers and publishers to get initialised
  ros::WallDuration(1).sleep();
  // creating the Control object for testing
  Control _control;
  // waiting for the subscriber and publisher initialization
  ros::WallDuration(1).sleep();
  ros::spinOnce();
  // Testing
  EXPECT_TRUE(ctest.anyMsgInVelTopic);
  EXPECT_TRUE(ctest.anyMsgInTakeOffTopic);
}
