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
#include <list>

#include "AgentModel.h"
#include "Interface.h"
#include "Logger.h"

#include "osi_sensorview.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "sl45_dynamicsrequest.pb.h"

#include <nlohmann/json.hpp>

using Point2D = agent_model::Position;

struct JunctionPath {
  std::vector<uint64_t> lanes;
  std::vector<Point2D> pts;
  uint64_t signal_id;
  int type; // -1 unknown, 0 changing, 1 priority, 2 give way
};

struct LaneGroup {
  int id;                                 // id of lane group
  int change_amount;                      // lane change amount to left/right
  std::vector<uint64_t> lanes;            // lane ids of lane group
  std::vector<uint64_t> lanes_changeable; // lane ids where change is possible
};

class OsiConverter {
 public:
  OsiConverter(){};
  ~OsiConverter(){};

  void convert(osi3::SensorView &sensor_view,
               osi3::TrafficCommand &traffic_command, 
               agent_model::Input &input,
               agent_model::Parameters &param,
               agent_model::Memory &memory);

 private:

  // flags
  bool initialized_ = false;
  bool calculate_lanes_ = false;
  bool ignore_all_targets_ = false;
  bool ignore_right_of_way_ = false;
  bool ignore_traffic_signals_ = false;
  
  // path variables
  std::vector<Point2D> path_centerline_;
  std::vector<double> path_s_;
  std::vector<double> path_kappa_;
  std::vector<double> path_psi_;

  std::vector<double> path_width_;
  std::vector<double> path_toff_left_;
  std::vector<double> path_toff_right_;
  
  std::vector<std::tuple<double, double> > changeable_;

  // lane variables
  std::vector<LaneGroup> lane_groups_;
  std::vector<uint64_t> lanes_;
  std::vector<uint64_t> lanes_changeable_;
  std::vector<uint64_t> intersection_lanes_;
  std::vector<JunctionPath> junction_paths_;

  // last action id's
  uint64_t lc_action_id_ = -1;
  uint64_t traj_action_id_ = -1;
  uint64_t path_action_id_ = -1;
  uint64_t glob_pos_action_id_ = -1;
  uint64_t speed_action_id_ = -1;
  uint64_t custom_action_id_ = -1;

  // ego variables
  uint64_t ego_id_;
  uint64_t ego_lane_id_;
  uint64_t ego_lane_group_id_;
  double ego_s_;
  bool ego_approaching_junction_;

  Point2D ego_position_;
  Point2D ego_centerline_point_;

  osi3::MovingObject ego_;
  osi3::BaseMoving ego_base_;
  osi3::Lane *ego_lane_ptr_;
  
  // where to end the simulation
  Point2D dest_point_;
  
  // helper functions
  void preprocess(osi3::SensorView &sensor_view,
                             osi3::TrafficCommand &traffic_command,
                             agent_model::Input &input,
                             agent_model::Parameters &param,
                             agent_model::Memory &memory);

  void processTrafficCommand(osi3::TrafficCommand &traffic_command, 
                            agent_model::Parameters &param);
  void newLanes(osi3::SensorView &sensor_view);

  void classifyManeuver(osi3::SensorView &sensor_view,
                        agent_model::Input &input);

  void generatePath(osi3::SensorView &sensor_view);

  void generateJunctionPaths(osi3::SensorView &sensor_view);

  void extractEgoInformation(osi3::SensorView &sensor_view,
                                         agent_model::Input &input);

  // filling functions
  void fillVehicle(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillSignals(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillTargets(osi3::SensorView &sensor_view, agent_model::Input &input, agent_model::Parameters &param);
  void fillHorizon(osi3::SensorView &sensor_view, agent_model::Input &input);
  void fillLanes(osi3::SensorView &sensor_view, agent_model::Input &input);
};
