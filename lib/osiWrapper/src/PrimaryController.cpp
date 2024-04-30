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
// Created by Jens Klimke on 2019-04-15.
//

#include "PrimaryController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

bool PrimaryController::step(double time_step_size) {

  auto &dt = time_step_size;

  // check if value is set
  if (target_ == nullptr) return false;

  // calculate error (P)
  auto u = *target_ - *value_;

  if (std::isinf(u)) throw std::runtime_error("Input value is not finite.");

  // calculate integral (I)
  in_ += (u_ + u) * dt;

  // calculate derivative (D)
  auto der = reset_ ? 0.0 : (u - u_) / dt;
  u_ = u;

  // unset reset flag
  reset_ = false;

  // calculate controller change
  auto dy = k_P_ * u_ + k_I_ * in_ + k_D_ * der;

  // apply desired controller state
  if (offset_ != nullptr && !std::isinf(*offset_)) {

    // change controller value
    dy = (*offset_ - *y_) * o_P_;

    // reset controller
    reset();
  }

  // limit change of controller
  dy = std::min(max_change_, std::max(-max_change_, dy));

  // integrate
  *y_ = std::max(range_[0], std::min(range_[1], *y_ + dy * dt));

  return true;
}

void PrimaryController::reset() {
  in_ = 0.0;
  reset_ = true;
}

void PrimaryController::setVariables(double *value, double *target,
                                     double *output, double *offset) {
  this->value_ = value;
  this->target_ = target;
  this->offset_ = offset;
  this->y_ = output;
}

void PrimaryController::setParameters(double k_p, double k_i, double k_d,
                                      double o_p) {
  this->k_P_ = k_p;
  this->k_I_ = k_i;
  this->k_D_ = k_d;
  this->o_P_ = o_p;
}

void PrimaryController::setRange(double lower, double upper, double maxChange) {
  this->range_[0] = lower;
  this->range_[1] = upper;
  this->max_change_ = maxChange;
}