/*
 * Author: Daniel Becker
 *
 * (C) 2019 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "IkaAgent.h"

void IkaAgent::init(osi3::BaseMoving &host) {

  // host values
  osi3::Vector3d v = host.velocity();
  double length = host.dimension().length();
  double width = host.dimension().width();
  double x = host.position().x();
  double y = host.position().y();
  double yaw = host.orientation().yaw();

  // check external variables
  if (v_init_ == 0) v_init_ = sqrt(pow(v.x(), 2) + pow(v.y(), 2));
  if (v_desired_ == 0) v_desired_ = 50.0 / 3.6;

  // get global pointers
  driver_state_ = this->getState();
  vehicle_state_ = vehicle_.getState();

  // set driver parameters
  agent_model::Parameters *driver_param = this->getParameters();

  double wheel_base = 3.22;

  // vehicle components
  driver_param->vehicle.pos.x = 0.0;
  driver_param->vehicle.pos.y = 0.0;
  driver_param->vehicle.size.length = length;
  driver_param->vehicle.size.width = width;

  // velocity components
  driver_param->velocity.a = 2.0;
  driver_param->velocity.b = -2.0;
  driver_param->velocity.thwMax = 10.0;
  driver_param->velocity.delta = 4.0;
  driver_param->velocity.deltaPred = 3.0;
  driver_param->velocity.ayMax = 1.5;
  driver_param->velocity.vComfort = v_desired_;

  // stop components
  driver_param->stop.T = 2.0;
  driver_param->stop.TMax = 7.0;
  driver_param->stop.tSign = 0.5;
  driver_param->stop.vStopped = 0.2;
  driver_param->stop.pedalDuringStanding = -0.3;

  // following components
  driver_param->follow.dsStopped = 2.0;
  driver_param->follow.thwMax = 10.0;
  driver_param->follow.timeHeadway = 1.8;

  // steering components
  driver_param->steering.thw[0] = 0.5;
  driver_param->steering.thw[1] = 3.0;
  driver_param->steering.dsMin[0] = 1.0;
  driver_param->steering.dsMin[1] = 10.0;
  driver_param->steering.P[0] = 0.075 * wheel_base;
  driver_param->steering.P[1] = 0.03 * wheel_base;
  driver_param->steering.D[0] = 0.25;
  driver_param->steering.D[1] = 0.25;

  AgentModel::init();

  // set vehicle parameters
  VehicleModel::Parameters *vehicle_param = vehicle_.getParameters();
  VehicleModel::Input *vehicle_input = vehicle_.getInput();

  vehicle_param->steer_transmission = 0.474;
  vehicle_param->steer_transmission = 0.5;
  vehicle_param->wheel_base = wheel_base;
  vehicle_param->mass = 1.5e3;
  vehicle_param->power_max = 1.0e4;
  vehicle_param->force_max = 6.0e3;
  vehicle_param->idle = 0.05;
  vehicle_param->roll_coefficient[0] = 4.0 * 9.91e-3;
  vehicle_param->roll_coefficient[1] = 4.0 * 1.95e-5;
  vehicle_param->roll_coefficient[2] = 4.0 * 1.76e-9;
  vehicle_param->driver_position.x = 0.0;
  vehicle_param->driver_position.y = 0.0;
  vehicle_param->size.x = length;
  vehicle_param->size.y = width;

  // set initial vehicle state
  vehicle_.reset();
  vehicle_state_->position.x = x;
  vehicle_state_->position.y = y;
  vehicle_state_->psi = yaw;
  vehicle_state_->v = v_init_;

  // set controller parameters (lateral motion control)
  steering_controller_.setParameters(10.0 * wheel_base, 0.1 * wheel_base, 0.0,
                                     1.0);
  steering_controller_.setRange(-1.0, 1.0, INFINITY);

  // set controller parameters (longitudinal motion control)
  pedal_controller_.setParameters(1.75, .5, 0.01, 1.0);
  pedal_controller_.setRange(-1.0, 1.0, INFINITY);

  // set variables
  pedal_controller_.setVariables(
    &vehicle_state_->a, &driver_state_->subconscious.a, &vehicle_input->pedal,
    &driver_state_->subconscious.pedal);
  steering_controller_.setVariables(&vehicle_state_->kappa,
                                    &driver_state_->subconscious.kappa,
                                    &vehicle_input->steer);

  pedal_controller_.reset();
  steering_controller_.reset();
}

int IkaAgent::step(double time, double step_size, osi3::SensorView &sensor_view,
                   osi3::TrafficCommand &traffic_command,
                   osi3::TrafficUpdate &traffic_update,
                   setlevel4to5::DynamicsRequest &dynamic_request) {
  int id = sensor_view.host_vehicle_id().value();

  std::cout << "---------- time: " << time << " ---------- id: " << id << " ----------" << std::endl;

  // in the first step, the desired curvature should be zero
  bool firstStep = false;
  // initialize agent
  if (!initialized_) {
    osi3::BaseMoving host = sensor_view.host_vehicle_data().location();
    
    IkaAgent::init(host);
    logger.init(id);
    firstStep = true;
    initialized_ = true;
  }

  // converter converts from osi to agent_model::input
  converter_.convert(sensor_view, traffic_command, _input, _param);

  // ika agent model step
  this->AgentModel::step(time);

  if (firstStep) driver_state_->subconscious.kappa = 0;
  // controller translates acc. and curv. to pedal and steering
  pedal_controller_.step(step_size);
  steering_controller_.step(step_size);

  // vehicle model step
  vehicle_.step(step_size);

  // update DynamicsRequest and TrafficUpdate
  dynamic_request.set_longitudinal_acceleration_target(
    driver_state_->subconscious.a);
  dynamic_request.set_curvature_target(driver_state_->subconscious.kappa);
  this->buildTrafficUpdate(traffic_update);

  if (debug_) {
    logger.saveDebugInformation(time, _input, driver_state_, vehicle_state_);
  }
  return 0;
}

int IkaAgent::buildTrafficUpdate(osi3::TrafficUpdate &traffic_update) {
  osi3::MovingObject *object = traffic_update.mutable_update(0);

  object->mutable_base()->mutable_position()->set_x(vehicle_state_->position.x);
  object->mutable_base()->mutable_position()->set_y(vehicle_state_->position.y);
  object->mutable_base()->mutable_velocity()->set_x(vehicle_state_->v);
  object->mutable_base()->mutable_velocity()->set_y(0);
  object->mutable_base()->mutable_acceleration()->set_x(vehicle_state_->a);
  object->mutable_base()->mutable_acceleration()->set_y(0);
  object->mutable_base()->mutable_orientation()->set_yaw(vehicle_state_->psi);
  object->mutable_base()->mutable_orientation_rate()->set_yaw(
    vehicle_state_->d_psi);

  return 0;
}