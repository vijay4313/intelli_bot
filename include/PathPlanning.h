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
 *  @Author Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  PathPlanning header file
 */

#ifndef INTELLI_BOT_INCLUDE_PATHPLANNING_H_
#define INTELLI_BOT_INCLUDE_PATHPLANNING_H_

#include<vector>

class PathPlanning {
 public:

  struct point3d {
    double x;
    double y;
    double z;
  };

  /**
   * @brief Constructor
   * @param none
   * @return none
   */
  PathPlanning();

  /**
   * @brief Destructor
   * @param none
   * @return none
   */
  virtual ~PathPlanning();

  /**
   * @brief Updates the heightPt variable
   * @param none
   * @return none
   */
  void setHt();

  /**
   * @brief generates the path
   * @param none
   * @return none
   */
  void generatePath();

  /**
   * @brief Outputs the path that was generated
   * @param none
   * @return none
   */
  std::vector<point3d> getPath();

 private:
  /**
   * @brief Stores the height of travel
   */
  double heightPt;

  /**
   * @brief Stores cover area coordinates
   */
  std::vector<double> covArea;

  /**
   * @brief Stores the generated path
   */
  std::vector<point3d> generatedPath;
};

#endif /* INTELLI_BOT_INCLUDE_PATHPLANNING_H_ */
