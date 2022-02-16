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
  driver_param->steering.thw[0] = 1.0;
  driver_param->steering.thw[1] = 3.0;
  driver_param->steering.dsMin[0] = 1.0;
  driver_param->steering.dsMin[1] = 3.0;
  driver_param->steering.P[0] = 0.03 * wheel_base;
  driver_param->steering.P[1] = 0.015 * wheel_base;
  driver_param->steering.D[0] = 0.1;
  driver_param->steering.D[1] = 0.1;

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

  // check if debug folder exist
  if (debug_) {
    struct stat buffer;
    if (stat ("debug", &buffer) != 0) {
      std::cout << "Please create the 'debug' folder" << std::endl;
      exit(0);
    }
  }
}

int IkaAgent::step(double time, double step_size, osi3::SensorView &sensor_view,
                   osi3::TrafficCommand &traffic_command,
                   osi3::TrafficUpdate &traffic_update,
                   setlevel4to5::DynamicsRequest &dynamic_request) {
  std::cout << "----------- time: " << time << " --------------" << std::endl;

  // initialize agent
  if (!initialized_) {
    osi3::BaseMoving host = sensor_view.host_vehicle_data().location();
    ego_id_ = sensor_view.host_vehicle_id().value();
    
    IkaAgent::init(host);
    initialized_ = true;
  }

  // converter converts from osi to agent_model::input
  converter_.convert(sensor_view, traffic_command, _input, _param);

  // ika agent model step
  this->AgentModel::step(time);

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

  saveDebugInformation(time);
  
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

void IkaAgent::saveDebugInformation(double time){

  // settings
  double dt_log = 0.1;
  double dt_save = 1.0;

  // convert time and dt_log to milliseconds (int) to allow modulo operator 
  // add 0.5 for proper rounding
  if (int(1000*time+0.5) % int(1000*dt_log+0.5) == 0) {

    json json_conscious_follow;
    json_conscious_follow["distance"] = driver_state_->conscious.follow.distance;
    json_conscious_follow["standing"] = driver_state_->conscious.follow.standing;
    json_conscious_follow["velocity"] = driver_state_->conscious.follow.velocity;

    json json_conscious_stop;
    json_conscious_stop["ds"] = driver_state_->conscious.stop.ds;
    json_conscious_stop["dsMax"] = driver_state_->conscious.stop.dsMax;
    json_conscious_stop["standing"] = driver_state_->conscious.stop.standing;

    
    json json_conscious_lateral;
    for (int i=0; i < 3; i++)
    {
      json_conscious_lateral[i]["factor"] = driver_state_->conscious.lateral.paths[i].factor;
      json_conscious_lateral[i]["offset"] = driver_state_->conscious.lateral.paths[i].offset;
      json_conscious_lateral[i]["ref_point_x"] = driver_state_->conscious.lateral.paths[i].refPoints->x;
      json_conscious_lateral[i]["ref_point_y"] = driver_state_->conscious.lateral.paths[i].refPoints->y;
    }

    json json_conscious_velocity;
    json_conscious_velocity["local"] = driver_state_->conscious.velocity.local;
    json_conscious_velocity["prediction"] = driver_state_->conscious.velocity.prediction;

    json json_conscious;
    json_conscious["follow"] = json_conscious_follow; 
    json_conscious["stop"] = json_conscious_stop;
    json_conscious["lateral"] = json_conscious_lateral;
    json_conscious["velocity"] = json_conscious_velocity; 
    
    json json_subconscious;
    json_subconscious["a"] = driver_state_->subconscious.a;
    json_subconscious["dPsi"] = driver_state_->subconscious.dPsi;
    json_subconscious["kappa"] = driver_state_->subconscious.kappa;
    json_subconscious["pedal"] = driver_state_->subconscious.pedal;
    json_subconscious["steering"] = driver_state_->subconscious.steering;

    json json_driver_state;
    json_driver_state["conscious"] = json_conscious;
    json_driver_state["subconscious"] = json_subconscious;
    
    json json_vehicle_state;
    json_vehicle_state["a"] = vehicle_state_->a;
    json_vehicle_state["ay"] = vehicle_state_->ay;
    json_vehicle_state["d_psi"] = vehicle_state_->d_psi;
    json_vehicle_state["d_psi"] = vehicle_state_->d_psi;
    json_vehicle_state["delta"] = vehicle_state_->delta;
    json_vehicle_state["ds"] = vehicle_state_->ds;
    json_vehicle_state["force"] = vehicle_state_->force;
    json_vehicle_state["kappa"] = vehicle_state_->kappa;
    json_vehicle_state["position_x"] = vehicle_state_->position.x;
    json_vehicle_state["position_y"] = vehicle_state_->position.y;
    json_vehicle_state["psi"] = vehicle_state_->psi;
    json_vehicle_state["s"] = vehicle_state_->s;
    json_vehicle_state["v"] = vehicle_state_->v;

    json json_horizon;
    json_horizon["x"] = _input.horizon.x;
    json_horizon["y"] = _input.horizon.y;
    json_horizon["ds"] = _input.horizon.ds;
    json_horizon["kappa"] = _input.horizon.kappa;
    json_horizon["psi"] = _input.horizon.psi;

    // concatenate debug information
    json_logger[json_counter]["ego_id"] = ego_id_;
    json_logger[json_counter]["time"] = time;
    json_logger[json_counter]["vehicle_state"] = json_vehicle_state;
    json_logger[json_counter]["driver_state"] = json_driver_state;
    json_logger[json_counter]["horizon"] = json_horizon;

    json_counter++;
  }

  // save debug file
  if (debug_ && std::fmod(time, dt_save) == 0) {
    std::ofstream output("debug/vehicle_" + std::to_string(ego_id_) + ".json", std::ofstream::out);
    output << json_logger.dump(4);
    output.close();
  }
}