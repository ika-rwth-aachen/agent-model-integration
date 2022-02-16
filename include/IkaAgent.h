/*
 * Author: Daniel Becker
 *
 * (C) 2019 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <nlohmann/json.hpp>

#include "AgentModel.h"
#include "Interface.h"
#include "OsiConverter.h"
#include "PrimaryController.h"
#include "VehicleModel.h"

using json = nlohmann::json;

class IkaAgent : public AgentModel {
 public:
  explicit IkaAgent(){
    v_init_ = 0;
    v_desired_ = 0;
    debug_ = false;
  };

  ~IkaAgent(){};

  int step(double time, double step_size, osi3::SensorView &sensor_view,
           osi3::TrafficCommand &traffic_command,
           osi3::TrafficUpdate &traffic_update,
           setlevel4to5::DynamicsRequest &dynamic_request);

  // external variables
  double v_init_;
  double v_desired_;
  bool debug_;

 private:
  bool initialized_ = false;

  // debug information logger
  int ego_id_;
  json json_logger;
  int json_counter = 0;

  // vehicle
  VehicleModel vehicle_;

  // converter
  OsiConverter converter_;

  // pointers
  agent_model::State *driver_state_;
  VehicleModel::State *vehicle_state_;

  // controllers
  PrimaryController steering_controller_;
  PrimaryController pedal_controller_;

  void init(osi3::BaseMoving &host);
  int buildTrafficUpdate(osi3::TrafficUpdate &traffic_update);
  void saveDebugInformation(double time);
};
