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
 *  @file    PathPlanning.h
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  PathPlanning source file
 */

#include "../include/PathPlanning.h"
#include <vector>
/**
 * @brief constructor for AStarAlgorithm class
 * @param  none
 * @return none
 */
PathPlanning::PathPlanning() {
  heightPt = 0;
  covArea = 0;
  generatedPath = 0;
}

/**
 * @brief destructor for AStarAlgorithm class
 * @param  none
 * @return none
 */
PathPlanning::~PathPlanning() {

}

/**
 * @brief sets the height
 * @param  none
 * @return none
 */

void PathPlanning::setHt() {

}

/**
 * @brief generates the required path
 * @param  none
 * @return none
 */

void PathPlanning::generatePath() {

}

/**
 * @brief outputs the path
 * @param  none
 * @return Path in vector<point3d>
 */

std::vector<PathPlanning::point3d> PathPlanning::getPath() {
  std::vector<PathPlanning::point3d> vec = 0;
  return vec;
}
