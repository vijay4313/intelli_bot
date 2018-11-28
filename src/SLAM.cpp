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
 *  @file    SLAM.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  SLAM routine file
 */

#include "../include/SLAM.h"

  /**
   * @brief Constructor
   * @param none
   * @return none
   */
  SLAM::SLAM() {}

  /**
   * @brief Destructor
   * @param none
   * @return none
   */
  SLAM::~SLAM() {}

  /**
   * @brief subscribe to image topic
   * @param  none
   * @return none
   */
  void SLAM::subscribeImg() {}

  /**
   * @brief routine to train
   * the ObjectDetector object
   * @param  none
   * @return none
   */
  SLAM::mapObject SLAM::publishMap(){
      SLAM::mapobject map;
      return(map);
  }

  /**
   * @brief routine to predict human
   * required Topic
   * publishing to the required topic
   * @param  none
   * @return none
   */
  void SLAM::computeSLAM() {}