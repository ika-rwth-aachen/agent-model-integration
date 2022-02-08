/*
 * Author: Christian Geller
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

#include "AgentModel.h"
#include "Interface.h"
#include "osi_sensorview.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "sl45_dynamicsrequest.pb.h"

using Point2D = agent_model::Position;

struct JunctionPath {
  std::vector<int> lane_ids;
  std::vector<Point2D> pts;
  int signal_id;
};

class OsiConverter {
 public:
  OsiConverter(){};
  ~OsiConverter(){};

  void convert(osi3::SensorView &sensor_view,
               osi3::TrafficCommand &traffic_command,
               agent_model::Input &input,
               agent_model::Parameters &param);

 private:
  bool debug_ = false;  // CGE sould be defined with compile flags

  // global path vectors
  std::vector<Point2D> path_centerline_;
  std::vector<double> path_kappa_;
  std::vector<double> path_s_;
  std::vector<double> path_psi_;

  // global lanes
  std::vector<int> lanes_;
  std::vector<JunctionPath> priority_lanes_;
  std::vector<JunctionPath> yielding_lanes_;

  // last action id's
  int traj_action_id_ = -1;
  int path_action_id_ = -1;
  int speed_action_id_ = -1;

  // last position values
  agent_model::Position last_position_;
  double last_s_ = 0;

  // current ego values
  int ego_id_;
  osi3::BaseMoving ego_base_;
  double ego_psi_;
  Point2D ego_centerline_point_;
  osi3::Lane *ego_lane_ptr_;
  std::unordered_map<int, int> ego_lane_mapping_;

  // helper functions
  void processTrafficCommand(osi3::SensorView &sensor_view,
                             osi3::TrafficCommand &traffic_command,
                             agent_model::Input &input,
                             agent_model::Parameters &param);
  void classifyManeuver(osi3::SensorView &sensor_view,
                        agent_model::Input &input);
  void generatePath(osi3::SensorView &sensor_view, agent_model::Input &input);

  // filling functions
  void fillVehicle(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillSignals(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillTargets(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillHorizon(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillLanes(osi3::SensorView &sensor_view, agent_model::Input &input);
};
