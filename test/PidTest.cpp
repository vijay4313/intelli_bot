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
#include <gtest/gtest.h>
#include <iostream>
#include "../include/PID.h"

/**
 * @brief Test routine for Setting and
 * getting gains
 */
TEST(TestPID, setGetGainTest) {
  PID pid;
  // set gains
  pid.setKpKiKd(1.0, 2.0, 3.0);
  // get gains
  double kp = pid.getKp();
  double ki = pid.getKi();
  double kd = pid.getKd();
  // testing for setting and getting gains
  EXPECT_EQ(1.0, kp);
  EXPECT_EQ(2.0, ki);
  EXPECT_EQ(3.0, kd);
}

/**
 * @brief Test routine for Setting and
 * getting time difference
 */
TEST(TestPID, setDt) {
  PID pid;
  // set dt
  pid.setDt(0.5);
  // get dt
  double dt = pid.getDt();
  // testing for setting and getting dt
  EXPECT_EQ(0.5, dt);
}

/**
 * @brief Test routine for PID
 * compute method
 */
TEST(TestPID, computeTest) {
  PID pid;
  // setting params
  pid.setKpKiKd(1.0, 2.0, 3.0);
  pid.setDt(0.5);
  double setPoint = 1;
  double currentVal = 0.5;
  // running control output
  double controlOutput = pid.compute(setPoint, currentVal);
  // testing for compute method
  EXPECT_EQ(4.5, controlOutput);
}
