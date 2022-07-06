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
  std::vector<uint64_t> lane_ids;
  std::vector<Point2D> pts;
  uint64_t signal_id;
  int type; // -1 unknown, 0 changing, 1 priority, 2 give way
};

class OsiConverter {
 public:
  OsiConverter(){};
  ~OsiConverter(){};

  void convert(osi3::SensorView &sensor_view,
               osi3::TrafficCommand &traffic_command, agent_model::Input &input,
               agent_model::Parameters &param);

 private:

  // flags
  bool initialized_ = false;
  bool calculate_lanes_ = false;
  bool ignore_all_targets_ = false; // use global flag for now - TODO: store ids
  
  // global path vectors
  std::vector<Point2D> path_centerline_;
  std::vector<double> path_kappa_;
  std::vector<double> path_s_;
  std::vector<double> path_psi_;

  // global lanes
  std::vector<uint64_t> lanes_;
  std::vector<uint64_t> intersection_lanes_;
  std::vector<JunctionPath> junction_paths_;
  std::unordered_map<uint64_t, int> lane_mapping_;

  // last action id's
  uint64_t traj_action_id_ = -1;
  uint64_t path_action_id_ = -1;
  uint64_t glob_pos_action_id_ = -1;
  uint64_t speed_action_id_ = -1;
  uint64_t custom_action_id_ = -1;

  // current ego values
  uint64_t ego_id_;
  uint64_t ego_lane_id_;
  osi3::MovingObject ego_;
  osi3::BaseMoving ego_base_;
  osi3::Lane *ego_lane_ptr_;
  Point2D ego_position_;
  Point2D ego_centerline_point_;

  // where to end the simulation
  Point2D dest_point_;
  
  // helper functions
  
  void preprocess(osi3::SensorView &sensor_view,
                             osi3::TrafficCommand &traffic_command,
                             agent_model::Input &input,
                             agent_model::Parameters &param);

  void processTrafficCommand(osi3::TrafficCommand &traffic_command, 
                            agent_model::Parameters &param);
  void newLanes(osi3::SensorView &sensor_view);

  void classifyManeuver(osi3::SensorView &sensor_view,
                        agent_model::Input &input);
  void generatePath(osi3::SensorView &sensor_view, agent_model::Input &input);

  void extractEgoInformation(osi3::SensorView &sensor_view,
                                         agent_model::Input &input);

  // filling functions
  void fillVehicle(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillSignals(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillTargets(osi3::SensorView &sensor_view, agent_model::Input &input, agent_model::Parameters &param);
  void fillHorizon(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillLanes(osi3::SensorView &sensor_view, agent_model::Input &input);
};
