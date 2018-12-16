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
 *  @file    PID.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  PID routine
 */
#include "../include/PID.h"

/**
 * Initialize all values to 0.0
 * if default constructor is called.
 */
PID::PID() {
  this->kp = 0;         // proportional gain
  this->ki = 0;         // integral gain
  this->kd = 0;         // differential gain
  this->prevError = 0;      // error at previous time step
  this->integralError = 0;    // Accumulation of error over time
  this->ctrlOp = 0;     // Control Output from the PID controller
  this->dt = 0.01;         // time
}

/// Default destructor
PID::~PID() {
}

/**
 * @brief Compute method
 * @param  required setpoint
 * @param  current value
 * @return control output
 */
double PID::compute(const double& setPoint, const double& currentVel) {
  double error = setPoint - currentVel;
  this->integralError += error;
  this->ctrlOp = kp * error + ki * integralError
      + (kd / dt) * (error - prevError);
  this->prevError = error;
  return ctrlOp;
}

/**
 * @brief setting gains for PID
 * @param  proportional gain
 * @param  integral gain
 * @param  derivative gain
 * @return none
 */
void PID::setKpKiKd(const double &kp, const double &ki, const double &kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

/**
 * @brief setting dt
 * @param  difference in time
 * @return none
 */
void PID::setDt(const double &dt) {
  this->dt = dt;
}

/**
 * @brief getting value of gain
 * @param  none
 * @return value of ki
 */
double PID::getKi() {
  return this->ki;
}

/**
 * @brief getting value of gain
 * @param  none
 * @return value of kp
 */
double PID::getKp() {
  return this->kp;
}

/**
 * @brief getting value of gain
 * @param  none
 * @return value of kd
 */
double PID::getKd() {
  return this->kd;
}

/**
 * @brief getting value of dt
 * @param  none
 * @return value of dt
 */
double PID::getDt() {
  return this->dt;
}
