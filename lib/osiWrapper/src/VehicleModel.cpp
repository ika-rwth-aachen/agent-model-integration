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
// Created by Jens Klimke on 2019-03-20.
//

#include "VehicleModel.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#ifndef G_ACC
#define G_ACC 9.81
#endif

#ifndef RHO_AIR
#define RHO_AIR 1.2041
#endif

void VehicleModel::reset() {

  // set initial states
  state_.s = 0.0;
  state_.v = 0.0;
  state_.psi = 0.0;
  state_.position = {0.0, 0.0};

  // set calculated states
  state_.ds = 0.0;
  state_.a = 0.0;
  state_.d_psi = 0.0;
  state_.delta = 0.0;
  state_.kappa = 0.0;
  state_.ay = 0.0;
  state_.force = 0.0;

  // set inputs
  input_.slope = 0.0;
  input_.pedal = 0.0;
  input_.steer = 0.0;
}

bool VehicleModel::step(double time_step_size) {

  // short cuts
  auto &dt = time_step_size;
  auto p = &parameters_;
  auto st = &state_;

  // calculate wheel steer angle and curvature
  st->delta = p->steer_transmission * input_.steer;
  st->kappa = st->delta / p->wheel_base;

  // calculate distance and velocity
  st->ds = std::max(0.0, st->v * dt + 0.5 * st->a * dt * dt);
  st->v = std::max(0.0, st->v + st->a * dt);

  // calculate position
  st->s += st->ds;
  st->position.x += cos(st->psi) * st->ds;
  st->position.y += sin(st->psi) * st->ds;

  // calculate yaw rate and yaw angle
  st->d_psi = st->v * st->kappa;
  st->psi += st->d_psi * dt;

  // squared velocity
  auto v2 = st->v * st->v;

  // coefficients
  auto air_coeff = 0.5 * RHO_AIR * p->cwA;
  auto roll_coeff = p->roll_coefficient[0] + p->roll_coefficient[1] * st->v +
                   p->roll_coefficient[2] * v2;

  // limit power and gas pedal
  auto throttle = std::max(input_.pedal, 0.0) * (1.0 - p->idle) + p->idle;

  // calculate accelerations
  auto a_ground = cos(input_.slope) * G_ACC;
  auto a_air = air_coeff * v2 / p->mass;
  auto a_roll = roll_coeff * a_ground;
  auto a_slope = sin(input_.slope) * G_ACC;
  auto a_brake = a_ground * std::min(input_.pedal, 0.0);

  // calculate smooth force curve
  double f0 = p->force_max;
  double f1 = p->power_max * 0.1;  // / 10 m/s (low speed boundary)
  double x = st->v * 0.1;        // / 10 m/s (low speed boundary)

  // calculate drive force
  if (x < 1.0)
    st->force = (f0 + pow(x, 2) * (4.0 * f1 - 3.0 * f0) +
                 pow(x, 3) * (2.0 * f0 - 3.0 * f1));  // low speed
  else
    st->force = p->power_max / st->v;  // high speed

  // calculate acceleration
  st->a = -a_roll - a_air - a_slope + a_brake + throttle * st->force / p->mass;
  st->ay = st->kappa * st->v * st->v;

  // unset acceleration, when standing
  if (st->v == 0.0 && st->a < 0.0) st->a = 0.0;

  return true;
}

VehicleModel::Input *VehicleModel::getInput() {

  return &this->input_;
}

VehicleModel::State *VehicleModel::getState() {

  return &this->state_;
}

VehicleModel::Parameters *VehicleModel::getParameters() {

  return &this->parameters_;
}

const VehicleModel::Input *VehicleModel::getInput() const {

  return &this->input_;
}

const VehicleModel::State *VehicleModel::getState() const {

  return &this->state_;
}

const VehicleModel::Parameters *VehicleModel::getParameters() const {

  return &this->parameters_;
}