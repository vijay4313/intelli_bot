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
 *  @file    PID.h
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  PID header file
 */
#ifndef INTELLI_BOT_SRC_PID_H_
#define INTELLI_BOT_SRC_PID_H_

#include <iostream>

class PID {
 private:
  double dt = 0.01;     ///< time step
  double kp;          ///< proportional gain
  double ki;          ///< integral gain
  double kd;          ///< differential gain
  double prevError;     ///< error at previous time step
  double integralError;   ///< Accumulation of error over time
  double ctrlOp;      ///< Control Output from the PID controller
 public:
  /**
   * @brief Constructor
   */
  PID();

  /// PID Destructor
  ~PID();

  /**
   * @brief calculate new velocity using setpoint
   *        and current velocity
   * @param setPoint - target velocity
   * @param currentVel - current velocity
   * @return new velocity
   */
  double compute(const double& setPoint, const double& currentVel);

  /**
   * @brief set value of kp
   * @param Kp - proportional gain
   */
  void setKpKiKd(const double &kp, const double &ki, const double &kd);

  /**
   * @brief set the dt value
   * @param dt - duration of each time step
   */
  void setDt(const double &dt);

  /**
   * @brief get value of ki
   * @return Ki value of PID
   */
  double getKi();

  /**
   * @brief get value of kp
   * @return kp value of PID
   */
  double getKp();

  /**
   * @brief get value of kd
   * @return kd value of PID
   */
  double getKd();

  /**
   * @brief get the value of dt
   * @return dt value of PID
   */
  double getDt();

};
#endif /* INTELLI_BOT_SRC_PID_H_ */
