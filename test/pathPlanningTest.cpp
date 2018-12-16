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
 *  @file    pathPlanningTest.cpp
 *  @author  Amrish Baskaran(amrish1222)
 *  @copyright  MIT
 *  @date    12/10/2018
 *  @brief  PathPlanning class test
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Pose.h>
#include "../include/PathPlanning.h"

/**
 * @brief Test for Path generation using the
 *        PathPlanning class
 */
TEST(pathPlanningTest, pathGenerationTest) {
  // Creaating instance of PathPlanning class
  PathPlanning pp;
  double length = 100;
  double breadth = 200.0;
  // setting the cover area with
  // the length and breadth
  pp.setCovArea(length, breadth);
  // generating the path
  pp.generatePath();
  // retrieving the path for testing
  auto generatedPath = pp.getPath();
  // Tests
  EXPECT_EQ(generatedPath[0].position.z, 2.5);
  EXPECT_EQ(generatedPath[1].position.x, -length);
  EXPECT_EQ(generatedPath[3].position.y, breadth);
  EXPECT_NEAR(generatedPath[0].orientation.x, 0, 0.01);
  EXPECT_NEAR(generatedPath[1].orientation.y, 0, 0.01);
  EXPECT_NEAR(generatedPath[2].orientation.z, 0.707, 0.01);
  EXPECT_NEAR(generatedPath[3].orientation.w, 1, 0.01);
}
