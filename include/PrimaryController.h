// Copyright (c) 2019-2020 Jens Klimke <jens.klimke@rwth-aachen.de>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Created by Jens Klimke on 2019-03-22.
//
// This implementation is based on the vehicle model in the SimCore project:
// https://github.com/JensKlimke/SimCore Adapted for this project by Jens Klimke
// on 2020-03-10
//

#ifndef SIMDRIVER_VM_PRIMARY_CONTROLLER_H
#define SIMDRIVER_VM_PRIMARY_CONTROLLER_H

class PrimaryController {

 protected:
  double *value_ = nullptr;   // the actual value
  double *target_ = nullptr;  // the target value
  double *offset_ = nullptr;  // an offset to be controlled directly
  double *y_ = nullptr;       // the output value

  double in_ = 0.0;  // the integral error
  double u_ = 0.0;   // the error

  double k_P_ = 0.0;
  double k_I_ = 0.0;
  double k_D_ = 0.0;
  double o_P_ = 1.0;

  double range_[2] = {-1.0, 1.0};
  double max_change_ = 1.0;

  bool reset_ = false;

 public:
  /**
   * Reset the controller states (except the output value)
   */
  void reset();

  /**
   * Perform a controller step with the given time step size
   * @param time_step_size Time step size
   */
  bool step(double time_step_size);

  /**
   * Set the controller variables
   * @param value The actual value
   * @param target The desired value
   * @param output The output value
   * @param offset An offset added to the output
   */
  void setVariables(double *value, double *target, double *output,
                    double *offset = nullptr);

  /**
   * Sets the parameters
   * @param k_p Proportional parameter
   * @param k_i Integral parameter
   * @param k_d Derivative parameter
   * @param o_p Offset controller proportional parameter
   */
  void setParameters(double k_p, double k_i, double k_d, double o_p = 1.0);

  /**
   * Sets the range of the output value
   * @param lower Lower limit
   * @param upper Upper limit
   * @param max_change The maximum change of the output value
   */
  void setRange(double lower, double upper, double max_change);
};

#endif  // SIMDRIVER_VM_PRIMARY_CONTROLLER_H
