/*
 * Author: Christian Geller
 *
 * (C) 2022 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "OsiConverter.h"
#include "OSI_helper.h"
#include <nlohmann/json.hpp>

void OsiConverter::convert(osi3::SensorView &sensor_view,
                           osi3::TrafficCommand &traffic_command,
                           agent_model::Input &input,
                           agent_model::Parameters &param,
                           agent_model::Memory &memory) {
  extractEgoInformation(sensor_view, input);

  preprocess(sensor_view, traffic_command, input, param, memory);

  fillVehicle(sensor_view, input);
  fillSignals(sensor_view, input);
  fillTargets(sensor_view, input, param);
  fillHorizon(sensor_view, input);
  fillLanes(sensor_view, input);
}

void OsiConverter::extractEgoInformation(osi3::SensorView &sensor_view,
                                         agent_model::Input &input) {

  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // find ego object
  ego_id_ = sensor_view.host_vehicle_id().value();
  for (int i = 0; i < ground_truth->moving_object_size(); i++) {
    if (ground_truth->moving_object(i).id().value() == ego_id_) {
      ego_ = ground_truth->moving_object(i);
      ego_base_ = ego_.base();
      break;
    }
  }

  // find ego_lane_id_

  // if only one lane is assigned
  if (ego_.assigned_lane_id_size() == 1) {
    ego_lane_id_ = ego_.assigned_lane_id(0).value();
  }
  // multiple assigned lanes
  else if (ego_.assigned_lane_id_size() > 1) {
    //after lanes_ was created:
    // e.g. on intersection: iterate over lanes_ in reverse order, (take last)
    if (lanes_.size() > 0) {
      for (int j = lanes_.size() - 1; j >= 0; j--) {
        for (int i = 0; i < ego_.assigned_lane_id_size(); i++) {
          if (find(lanes_.begin(), lanes_.end(),
                  ego_.assigned_lane_id(i).value()) != lanes_.end()) {
            ego_lane_id_ = ego_.assigned_lane_id(i).value();
            break;
          }
        }
      }
    } else {
      // take last entry
      ego_lane_id_ = ego_.assigned_lane_id(ego_.assigned_lane_id_size()-1).value();
    }
  }
  // no lane assignment possible, take closest lane
  else {
    Point2D ego_position =
      Point2D(ego_base_.position().x(), ego_base_.position().y());
    ego_lane_id_ = closestLane(ground_truth, ego_position);
  }

  // find lane pointer
  ego_lane_ptr_ = findLane(ego_lane_id_, ground_truth);

  if (!initialized_)
  {
    ego_position_.x = ego_base_.position().x();
    ego_position_.y = ego_base_.position().y();
    calculate_lanes_ = true;
    initialized_ = true;
  }

}

void OsiConverter::preprocess(osi3::SensorView &sensor_view,
                              osi3::TrafficCommand &traffic_command,
                              agent_model::Input &input,
                              agent_model::Parameters &param,
                              agent_model::Memory &memory) 
{
  bool compute_lane = false;

  // analyize traffic commands
  processTrafficCommand(traffic_command, param);

  // if lane changed 
  if (memory.laneChange.switchLane != 0)
  {
    //current_lane_group_ += memory.laneChange.switchLane;
    calculate_lanes_ = true; // compute new lane groups
  }

  // skip if lanes should not be calculated
  if (!calculate_lanes_) return;

  // calculate new lanes_ to reach destination
  newLanes(sensor_view);

  // generate paths
  generatePath(sensor_view);
  generateJunctionPaths(sensor_view);

  // determine type of maneuver on intersection
  classifyManeuver(sensor_view, input);
}

void OsiConverter::processTrafficCommand(osi3::TrafficCommand &traffic_command,
                                         agent_model::Parameters &param) {
  // iterate over all traffic commands
  for (int i = 0; i < traffic_command.action_size(); i++) {

    // take global position for lane calculation
    if (traffic_command.action(i).has_acquire_global_position_action()
        && traffic_command.action(i)
          .acquire_global_position_action()
          .action_header()
          .action_id()
          .value() != glob_pos_action_id_) {
      osi3::TrafficAction_AcquireGlobalPositionAction position 
        = traffic_command.action(i).acquire_global_position_action();      
      glob_pos_action_id_ = position.action_header().action_id().value();
      dest_point_ = Point2D(position.position().x(), position.position().y());
      calculate_lanes_ = true;      
    }

    // take end position of trajectory for lane calculation
    if (traffic_command.action(i).has_follow_trajectory_action()
        && traffic_command.action(i)
          .follow_trajectory_action()
          .action_header()
          .action_id()
          .value() != traj_action_id_) {
      osi3::TrafficAction_FollowTrajectoryAction traj 
        = traffic_command.action(i).follow_trajectory_action();
      traj_action_id_ = traj.action_header().action_id().value();
      dest_point_ = Point2D(
        traj.trajectory_point(traj.trajectory_point_size() - 1).position().x(),
        traj.trajectory_point(traj.trajectory_point_size() - 1).position().y());
      calculate_lanes_ = true;
    }

    // take end position of path for lane calculation
    if (traffic_command.action(i).has_follow_path_action() 
        && traffic_command.action(i)
          .follow_path_action()
          .action_header()
          .action_id()
          .value() != path_action_id_) {
      osi3::TrafficAction_FollowPathAction path
        = traffic_command.action(i).follow_path_action();
      path_action_id_ = path.action_header().action_id().value();
      dest_point_ =
        Point2D(path.path_point(path.path_point_size() - 1).position().x(),
                path.path_point(path.path_point_size() - 1).position().y());
      calculate_lanes_ = true;
    }

    // speed action
    if (traffic_command.action(i).has_speed_action() 
        && traffic_command.action(i)
          .speed_action()
          .action_header()
          .action_id()
          .value() != speed_action_id_) {
      osi3::TrafficAction_SpeedAction speed 
        = traffic_command.action(i).speed_action();
      speed_action_id_ = speed.action_header().action_id().value();
      param.velocity.vComfort = speed.absolute_target_speed();
    }

    // custom command
    if (traffic_command.action(i).has_custom_action()
      && traffic_command.action(i).custom_action()
         .action_header().action_id()
         .value() != custom_action_id_) {
      osi3::TrafficAction_CustomAction custom
        = traffic_command.action(i).custom_action();
      custom_action_id_ = custom.action_header().action_id().value();
      std::string command = custom.command();
      nlohmann::json custom_command;
      custom_command = nlohmann::json::parse(command);

      if (custom_command.contains("Ignore_AllTrafficParticipants")){
        ignore_all_targets_ = custom_command["Ignore_AllTrafficParticipants"];
          if (ignore_all_targets_)
            std::cout << "All targets will be ignored.\n";
      }
      if (custom_command.contains("Ignore_TrafficParticipants")){
        std::cout << "Not supported yet from sim. env. side\n";
        std::cout << "Todo: store targets that will be ignored.\n";
      }
      if (custom_command.contains("trafficRules")){
        if (custom_command["trafficRules"].contains("IgnoreRightOfWay"))          
        {
          bool ignore_right_of_way 
              = custom_command["trafficRules"]["IgnoreRightOfWay"];
          if (ignore_right_of_way)
            std::cout << "Soon: Right of way will be ignored.\n";
        }
      }
    }
  }  
}

void OsiConverter::newLanes(osi3::SensorView &sensor_view) {

  lanes_.clear();
  lanes_changeable_.clear();

  path_centerline_.clear();
  path_kappa_.clear();
  path_s_.clear();
  path_psi_.clear();

  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // find starting_lane_idx
  int starting_lane_idx = findLaneIdx(ground_truth, ego_lane_id_);

  // check if no traffic_command was sent
  if (lc_action_id_ == -1
      && traj_action_id_ == -1
      && path_action_id_ == -1
      && glob_pos_action_id_ == -1) {
    // set dest_point_ to end of lane
    int pos = 0;
    if (ego_lane_ptr_->classification().centerline_is_driving_direction()) {
      pos = ego_lane_ptr_->classification().centerline().size() - 1;
    }

    dest_point_ =
      Point2D(ego_lane_ptr_->classification().centerline(pos).x(),
              ego_lane_ptr_->classification().centerline(pos).y());
  }

  // calculate future lane groups
  lane_groups_.clear();
  futureLanes(ground_truth, starting_lane_idx, dest_point_, lane_groups_);

  LaneGroup lane_group = findLaneGroup(lane_groups_, current_lane_group_);
  lanes_ = lane_group.lane_ids;
  lanes_to_change_ = lane_group.lane_ids_to_change;
    
  // fill lane_mapping_
  mapLanes(ground_truth, lane_mapping_, ego_lane_ptr_, lanes_);

  // print lanes
  std::cout << "Destination at: " << dest_point_.x <<","<< dest_point_.y <<"\n";
  std::cout << "With lanes to pass: ";
  for (auto &lane : lanes_) std::cout << lane << " ";
  std::cout << std::endl;

  // unset flat
  calculate_lanes_ = false;
}

/**
 * @brief Generates paths and junction paths
 *
 *
 * @param sensor_view osi sensor view from first time step
 * @param input agent_models input
 */
void OsiConverter::generatePath(osi3::SensorView &sensor_view) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  int gap_idx = 0;  // idx in path_centerline_ for free boundary lane points

  // clear path variable when new path is generated
  path_centerline_.clear();
  path_kappa_.clear();
  path_psi_.clear();
  path_s_.clear();
  // get all relevant path points from lanes_
  for (auto &l : lanes_) {
    osi3::Lane *tmp_lane = findLane(l, ground_truth);
    bool is_free_boundary_lane = false;

    // determine lane type if lane normal
    if (tmp_lane->classification().free_lane_boundary_id_size() > 0 &&
        tmp_lane->classification().type() ==
          osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
      is_free_boundary_lane = true;
    }

    if (!is_free_boundary_lane)
      getXY(tmp_lane, path_centerline_);
    else
      gap_idx = path_centerline_.size();
  }

  // fill gap with interpolation based on two points at each end
  if (gap_idx > 0) {
    std::vector<Point2D> gap_points(&path_centerline_[gap_idx - 2],
                                    &path_centerline_[gap_idx + 2]);
    calcXYGap(gap_points, path_centerline_, gap_idx);
  }

  // remove duplicates (if ds is very small)
  removeDuplicates(path_centerline_);
  
  // calculate s, psi, kappa from centerline
  xy2Curv(path_centerline_, path_s_, path_psi_, path_kappa_);
}

void OsiConverter::generateJunctionPaths(osi3::SensorView &sensor_view) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // generate junction paths for traffic lights
  std::vector<int> start_lane_ids;

  for (auto &light : *ground_truth->mutable_traffic_light()) {

    // create path from assigned lane of light backwards
    for (auto &assigned_lane : light.classification().assigned_lane_id()) {

      int lane_id = assigned_lane.value();

      // check if lane_id already contained in start_lane_ids
      if (std::find(start_lane_ids.begin(), start_lane_ids.end(), lane_id) !=
          start_lane_ids.end())
        continue;
      start_lane_ids.push_back(lane_id);

      // calculate path backwards
      JunctionPath junction_path = calcJunctionPath(ground_truth, lane_id);

      // add signal_id
      junction_path.signal_id = light.id().value();

      // mark as dynamic type
      junction_path.type = 0; 
      
      // push junction path to global junction_paths_
      junction_paths_.push_back(junction_path);
    }
  }

  // generate junction paths for traffic signs 
  // assumption: sign starts always on same lane as the parent traffic light
  for (auto &sign : *ground_truth->mutable_traffic_sign()) {

    // only RIGHT_OF_WAY, GIVE_WAY and STOP are supported
    if (sign.main_sign().classification().type() !=
      osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN && 
        sign.main_sign().classification().type() !=
          osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY &&
        sign.main_sign().classification().type() !=
          osi3::TrafficSign_MainSign_Classification_Type_TYPE_STOP)
      continue;

    // create path from assigned lane of sign backwards
    for (auto &assigned_lane :
         sign.main_sign().classification().assigned_lane_id()) {

      int lane_id = assigned_lane.value();

      // check if lane_id already contained in start_lane_ids
      if (std::find(start_lane_ids.begin(), start_lane_ids.end(), lane_id) !=
          start_lane_ids.end())
        continue;
      start_lane_ids.push_back(lane_id);

      // calculate path backwards
      JunctionPath junction_path = calcJunctionPath(ground_truth, lane_id);

      // add signal_id
      junction_path.signal_id = sign.id().value();

      // mark as priority path
      if (sign.main_sign().classification().type() ==
        osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN)
        junction_path.type = 1;

      // mark as give way path
      if (sign.main_sign().classification().type() ==
        osi3::TrafficSign_MainSign_Classification_Type_TYPE_STOP ||
          sign.main_sign().classification().type() ==
        osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY )
        junction_path.type = 2;
      
      // push junction path to global junction_paths_
      junction_paths_.push_back(junction_path);
    }
  }

  // if no signals available - check for roads with type intersection 
  // assumption: only one intersection is supported
  if (junction_paths_.size() == 0)
  {
    for (int i = 0; i < ground_truth->lane_size(); i++)
    {
      int lane_id = ground_truth->lane(i).id().value();
      auto* lane = findLane(lane_id, ground_truth);

      // check for intersection lane
      if (lane->classification().type() !=
            osi3::Lane_Classification_Type_TYPE_INTERSECTION) continue;

      // iterate over all lane pairings to find starting lanes
      for (auto &l_pairs : lane->classification().lane_pairing()) 
      {
        // always take antecessor if free lane boundary intersection
        if (lane->classification().free_lane_boundary_id_size() > 0)
        {
          if (!l_pairs.has_antecessor_lane_id()) break;
          lane_id = l_pairs.antecessor_lane_id().value();
        }
        // if openPASS intersection "hack"
        else {
          if (lane->classification().centerline_is_driving_direction()) {
            if (!l_pairs.has_antecessor_lane_id()) break;
            lane_id = l_pairs.antecessor_lane_id().value();
          } else {
            if (!l_pairs.has_successor_lane_id()) break;
            lane_id = l_pairs.successor_lane_id().value();
          }
        }
         
        // get next_lane if exist
        auto* next_lane = findLane(lane_id, ground_truth);
        if (next_lane == nullptr) continue;

        // check if next_lane is of type driving
        if (next_lane->classification().type() !=
          osi3::Lane_Classification_Type_TYPE_DRIVING) continue;
        
        // check if lane_id already contained in start_lane_ids
        if (std::find(start_lane_ids.begin(), start_lane_ids.end(), lane_id) != start_lane_ids.end())
          continue;
        start_lane_ids.push_back(lane_id);

        // calculate path backwards
        JunctionPath junction_path = calcJunctionPath(ground_truth, lane_id);

        // add signal_id
        junction_path.signal_id = -1;

        // mark as dynamic type
        junction_path.type = -1; 
        
        // push junction path to global junction_paths_
        junction_paths_.push_back(junction_path);
      }
    }
  }

  // flip global lanes and add points of centerlines
  for (auto &yl : junction_paths_) {
    std::reverse(yl.lanes.begin(), yl.lanes.end());
    for (auto &l : yl.lanes) getXY(findLane(l, ground_truth), yl.pts);
  }

  // set intersection lanes
  for (int i = 0; i < ground_truth->lane_size(); i++) {
    osi3::Lane lane = ground_truth->lane(i);
    if (ground_truth->lane(i).classification().type() ==
        osi3::Lane_Classification_Type_TYPE_INTERSECTION)
      intersection_lanes_.push_back(ground_truth->lane(i).id().value());
  }
}

/**
 * @brief classifies the host vehicle's path as either
 * TURN_RIGHT | TURN_LEFT | STRAIGHT (default)
 * based on curvature of the first lane with type INTERSECTION on path
 * @param sensor_view osi sensor view from first time step
 * @param input agent_models input
 */
void OsiConverter::classifyManeuver(osi3::SensorView &sensor_view,
                                    agent_model::Input &input) {

  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // Assumption: at least one lane of lanes_ is of type INTERSECION

  // get all path positions on intersection
  std::vector<Point2D> positions;
  for (auto it : lanes_) {
    osi3::Lane *lane = findLane(it, ground_truth);
    if (lane->classification().type() ==
        osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
      if (lane->classification().free_lane_boundary_id_size() > 0) {
        // TODO: not yet implemented for standard OSI
      } else {
        getXY(lane, positions);
      }
    }
  }

  // remove duplicates (if ds is very small)
  removeDuplicates(positions);

  if (positions.empty() || positions.size() < 3) {
    input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;
  } else {
    // calculate curvature k
    std::vector<double> s, k, p;
    xy2Curv(positions, s, p, k);

    double avg = std::accumulate(k.cbegin(), k.cend(), 0.0) / k.size();
    double eps = 0.01;

    // decision based on average curvature
    if (avg > eps)
      input.vehicle.maneuver = agent_model::Maneuver::TURN_LEFT;
    else if (avg < -eps)
      input.vehicle.maneuver = agent_model::Maneuver::TURN_RIGHT;
    else
      input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;
  }
}

void OsiConverter::fillVehicle(osi3::SensorView &sensor_view,
                               agent_model::Input &input) {

  // calculate traveled distance
  double dx = ego_base_.position().x() - ego_position_.x;
  double dy = ego_base_.position().y() - ego_position_.y;
  input.vehicle.s += sqrt(dx * dx + dy * dy);

  // set velocity and acceleration
  input.vehicle.v = getNorm(ego_base_.velocity());
  input.vehicle.a = getNorm(ego_base_.acceleration());

  // update positions
  ego_position_.x = ego_base_.position().x();
  ego_position_.y = ego_base_.position().y();

  // projection of ego coordinates on centerline
  closestCenterlinePoint(ego_position_, path_centerline_, ego_centerline_point_);

  // calculate s, psi, k of ego lane
  std::vector<double> s, psi, k;
  xy2Curv(path_centerline_, s, psi, k);

  // angle between ego yaw and deviation from centerline point
  input.vehicle.psi = ego_base_.orientation().yaw() - 
              interpolateXY2Value(psi, path_centerline_, ego_centerline_point_);
  input.vehicle.dPsi = ego_base_.orientation_rate().yaw();

  // compute distance between ego and centerline
  input.vehicle.d = computeDistanceInRefAngleSystem(ego_position_, 
                        ego_centerline_point_, ego_base_.orientation().yaw());

  // set dummy values
  input.vehicle.pedal = 0;
  input.vehicle.steering = 0;
  input.vehicle.dsIntersection = INFINITY;

  // if ego is approaching junction set dsIntersection
  for (auto &junction_path : junction_paths_) 
  {
    if (find(junction_path.lanes.begin(), junction_path.lanes.end(),
      ego_lane_id_) != junction_path.lanes.end()) 
    {
      double ds_intersection = abs(xy2s(ego_position_, junction_path.pts.back(), junction_path.pts, ego_base_.orientation().yaw()));

      input.vehicle.dsIntersection = ds_intersection;

      break;
    }
  }
}

void OsiConverter::fillSignals(osi3::SensorView &sensor_view,
                               agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  double ds_gap = 5;

  int signal = 0;
  
  std::vector<int> signal_lanes;  
  std::vector<int> traffic_light_ids;            
  std::vector<Point2D> traffic_light_positions;

  // iterate over all traffic lights
  for (int i = 0; i < ground_truth->traffic_light_size(); i++) {

    // break when signals array is full
    if (signal >= agent_model::NOS-1) break;

    osi3::TrafficLight light = ground_truth->traffic_light(i);
    osi3::TrafficLight_Classification cls = light.classification();
    Point2D signal_point(light.base().position().x(), light.base().position().y());

    // continue if off, flashing, counting, unknown or other
    if (cls.mode() != osi3::TrafficLight_Classification_Mode_MODE_CONSTANT)
      continue;

    // check if traffic light is assigned along the route
    int assigned_lane_id;
    bool assigned = isSigAssigned(cls, signal_lanes, lanes_, assigned_lane_id);

    // continue if not assigned to route
    if (!assigned) continue;

    // projection of signal position to centerline
    Point2D centerline_point;
    closestCenterlinePoint(signal_point, path_centerline_, centerline_point);
    traffic_light_positions.push_back(signal_point);

    // save all original signal ids
    traffic_light_ids.push_back(i);

    // add signal with id
    input.signals[signal].id = signal + 1;

    // ds along centerline to reach signal 
    double ds = calcDsSignal(*ground_truth, path_centerline_, signal_point, 
                              ego_centerline_point_, assigned_lane_id,
                              ego_base_.orientation().yaw(), ds_gap);
    input.signals[signal].ds = ds;
    
    // set defaults
    input.signals[signal].type = agent_model::SignalType::SIGNAL_TLS;
    input.signals[signal].subsignal = false;
    input.signals[signal].sign_is_in_use = true;

    // set color and type/icon
    if (cls.color() == osi3::TrafficLight_Classification_Color_COLOR_RED) {
      input.signals[signal].color = agent_model::COLOR_RED;

      if (cls.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
        input.signals[signal].icon = agent_model::ICON_NONE;
      else if (cls.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
        input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
      else if (cls.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
        input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;
    } else if (cls.color() ==
               osi3::TrafficLight_Classification_Color_COLOR_YELLOW) {
      input.signals[signal].color = agent_model::COLOR_YELLOW;

      if (cls.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
        input.signals[signal].icon = agent_model::ICON_NONE;
      else if (cls.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
        input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
      else if (cls.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
        input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;
    } else if (cls.color() ==
               osi3::TrafficLight_Classification_Color_COLOR_GREEN) {
      input.signals[signal].color = agent_model::COLOR_GREEN;

      if (cls.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
        input.signals[signal].icon = agent_model::ICON_NONE;
      else if (cls.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
        input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
      else if (cls.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
        input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;
    }
    signal++;
  }

  // iterate over all traffic signs
  for (int i = 0; i < ground_truth->traffic_sign_size(); i++) {

    // only consider NOS signals
    if (signal >= agent_model::NOS-1) break;

    osi3::TrafficSign sign = ground_truth->traffic_sign(i);
    osi3::TrafficSign_MainSign_Classification cls =
      sign.main_sign().classification();

    // check if traffic light is assigned along the route
    int assigned_lane_id;
    bool assigned = isSigAssigned(cls, signal_lanes, lanes_, assigned_lane_id);

    // skip a non-assigned signal    
    if (!assigned) continue;

    // projection of signal position to centerline
    Point2D centerline_point;
    Point2D signal_point(sign.main_sign().base().position().x(),
                   sign.main_sign().base().position().y());
    closestCenterlinePoint(signal_point, path_centerline_, centerline_point);

    // add signal with id
    input.signals[signal].id = signal + 1; 

    // ds along centerline to reach signal 
    double ds = calcDsSignal(*ground_truth, path_centerline_, signal_point, 
                              ego_centerline_point_, assigned_lane_id,
                              ego_base_.orientation().yaw(), ds_gap);
    input.signals[signal].ds = ds;

    // set defaults
    input.signals[signal].type = agent_model::SignalType::SIGNAL_TLS;
    input.signals[signal].subsignal = false;
    input.signals[signal].sign_is_in_use = true;

    // check if signal position is already known (due to prior traffic light)
    auto known_position =
      find(traffic_light_positions.begin(), traffic_light_positions.end(), signal_point);

    // iterate over known_position's
    int count = 0;
    bool all_out_of_service = true;
    while ((known_position != traffic_light_positions.end())) {
      
      // calculate and set paired_signal_id
      int paired_signal_id = traffic_light_ids[known_position - traffic_light_positions.begin()];

      // calculate if all signals out of service
      if (!ground_truth->traffic_light(paired_signal_id).classification().        is_out_of_service()) all_out_of_service = false;

      // get next known position
      known_position =
        find(known_position, traffic_light_positions.end(), signal_point);

      count++;

      // break if three known positions are detected
      if (count == 3)
      {
        if (!all_out_of_service) input.signals[signal].sign_is_in_use = false;
        break;
      }
    }

    // set type/value
    if (cls.type() ==
        osi3::TrafficSign_MainSign_Classification_Type_TYPE_STOP) {
      input.signals[signal].type = agent_model::SIGNAL_STOP;
      input.signals[signal].value = 0;
    } else if (
      cls.type() ==
      osi3::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN) {
      input.signals[signal].type = agent_model::SIGNAL_SPEED_LIMIT;
      input.signals[signal].value = cls.value().value();
    } else if (
      cls.type() ==
      osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN) {
      input.signals[signal].type = agent_model::SIGNAL_PRIORITY;
    } else if (
      cls.type() ==
        osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY ||
      cls.type() ==
        osi3::
          TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_BEFORE_LEFT_NEXT_INTERSECTION) {
      input.signals[signal].type = agent_model::SIGNAL_YIELD;
    } else {
      input.signals[signal].type = agent_model::SIGNAL_NOT_SET;
      input.signals[signal].value = 0;
    }
    signal++;
  }

  // fill all but last remaining signals with default values
  for (int i = signal; i < agent_model::NOS-1; i++) {
    input.signals[i].id = 127;
    input.signals[i].ds = INFINITY;
    input.signals[i].type = agent_model::SIGNAL_NOT_SET;
    input.signals[i].value = 0;
  }
}

void OsiConverter::fillTargets(osi3::SensorView &sensor_view,
                               agent_model::Input &input,
                               agent_model::Parameters &param) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // iterate over all targets
  int target = 0;
  if (!ignore_all_targets_) {
    for (auto &tar : *ground_truth->mutable_moving_object()) {

      // skip ego vehicle
      if (tar.id().value() == ego_id_) continue;

      osi3::BaseMoving target_base = tar.base();

      // set general properties
      input.targets[target].id = target + 1;
      input.targets[target].priority = agent_model::TARGET_PRIORITY_NOT_SET;
      input.targets[target].position = agent_model::TARGET_NOT_RELEVANT;
      input.targets[target].dsIntersection = 0;    
      input.targets[target].ds = INFINITY;
      input.targets[target].d = 0;
      input.targets[target].lane = 127;

      input.targets[target].xy.x = target_base.position().x() - ego_position_.x;
      input.targets[target].xy.y = target_base.position().y() - ego_position_.y;

      input.targets[target].v = getNorm(target_base.velocity());
      input.targets[target].a = getNorm(target_base.acceleration());
      input.targets[target].psi = target_base.orientation().yaw() - ego_base_.orientation().yaw();

      input.targets[target].size.length = target_base.dimension().length();
      input.targets[target].size.width = target_base.dimension().width();

      // check if target is assigned along the route
      bool assigned = false;
      int assigned_lane_idx = -1;
      for (int j = 0; j < tar.assigned_lane_id_size(); j++) {
        
        // check if assigned to intersection
        if (std::find(intersection_lanes_.begin(), intersection_lanes_.end(),
                    tar.assigned_lane_id(j).value()) != intersection_lanes_.end()){
          input.targets[target].position = agent_model::TARGET_ON_INTERSECTION;
        }

        // check if target on route
        auto target_lane = find(lanes_.begin(), lanes_.end(), tar.assigned_lane_id(j).value());
        if (target_lane != lanes_.end()) {
          assigned_lane_idx = lane_mapping_[tar.assigned_lane_id(j).value()];
          assigned = true;
          break;
        }
      }

      // only fill ds, d, lane and fields when target is assigned to host's path
      if (assigned) {

        // projection of target position to centerline
        Point2D centerline_point;
        Point2D target_point(target_base.position().x(), target_base.position().y());
        closestCenterlinePoint(target_point, path_centerline_, centerline_point);

        // ds along centerline to reach target 
        double ds_target = xy2s(ego_centerline_point_, target_point,  
                          path_centerline_, ego_base_.orientation().yaw());

        double ds_correction = 0.5 * (ego_base_.dimension().length() + target_base.dimension().length());

        // apply length correction
        if (ds_target > 0)
          ds_target -= ds_correction;
        else
          ds_target += ds_correction;

        input.targets[target].ds = ds_target;

        // calculate distance to centerline point
        Point2D target_position(target_base.position().x(), target_base.position().y());
        input.targets[target].d = computeDistanceInRefAngleSystem(target_position, centerline_point, target_base.orientation().yaw());

        // set assigned lane id
        input.targets[target].lane = assigned_lane_idx;
        input.targets[target].position = agent_model::TARGET_ON_PATH;
      } 
      
      // compute if target (not on path and intersection) approaches intersection
      if (!assigned && input.targets[target].position != agent_model::TARGET_ON_INTERSECTION) {
        
        // iterate over assigned lanes
        for (auto &assigned_lane : tar.assigned_lane_id()) {

          std::vector<Point2D> path_points;
          bool approaching_junction = false;

          // target is on a yield lane to the heading to the junction
          for (auto &junction_path : junction_paths_) {
            if (find(junction_path.lane_ids.begin(), junction_path.lane_ids.end(),
                      assigned_lane.value()) != junction_path.lane_ids.end()) {

              if (junction_path.type == 1)
                input.targets[target].priority = agent_model::TARGET_ON_PRIORITY_LANE;
              else if (junction_path.type == 2)
                input.targets[target].priority = agent_model::TARGET_ON_GIVE_WAY_LANE;
              else
                input.targets[target].priority = agent_model::TARGET_PRIORITY_NOT_SET;

              path_points = junction_path.pts;
              approaching_junction = true;
              break;
            }
          }
          
          // calculate distance to intersection if approaching intersection
          if (approaching_junction)
          {
            double ds_intersection = abs(xy2s(Point2D(target_base.position().x(), target_base.position().y()), path_points.back(), path_points, target_base.orientation().yaw()));

            input.targets[target].dsIntersection = ds_intersection;  

            // compute target position if close enough (always if closer than 10m)
            double ds_thres = std::max(10.0, input.targets[target].v * param.stop.TMax);
            if (ds_intersection <= ds_thres)
            {
              double tol = M_PI/8;
              double psi = input.targets[target].psi;

              if (abs(wrapAngle(M_PI/2 - psi)) < tol)
              {
                input.targets[target].position = agent_model::TARGET_ON_RIGHT;
              }
              if (abs(wrapAngle(M_PI - psi)) < tol)
              {
                input.targets[target].position = agent_model::TARGET_ON_OPPOSITE;
              }
              if (abs(wrapAngle(-M_PI/2 - psi)) < tol)
              {
                input.targets[target].position = agent_model::TARGET_ON_LEFT;
              }
            }
            break;
          }
        }
      }
      target++;
    }
  }

  // fill remaining targets with default values
  for (int i = target; i < agent_model::NOT; i++) {
    input.targets[i].id = 0;
    input.targets[i].ds = INFINITY;
    input.targets[i].xy.x = 0;
    input.targets[i].xy.y = 0;
    input.targets[i].v = 0;
    input.targets[i].a = 0;
    input.targets[i].d = 0;
    input.targets[i].psi = 0;
    input.targets[i].lane = 127;
    input.targets[i].size.width = 0;
    input.targets[i].size.length = 0;
    input.targets[i].dsIntersection = 0;
    input.targets[i].position = agent_model::TARGET_NOT_RELEVANT;
    input.targets[i].priority = agent_model::TARGET_PRIORITY_NOT_SET;
  }
}

void OsiConverter::fillHorizon(osi3::SensorView &sensor_view,
                               agent_model::Input &input) {
  
  double horizon_thw = 15;
  double s_max = std::max(15.0, horizon_thw * input.vehicle.v);

  // distance (along centerline) to each horizon point from current location
  std::vector<double> ds(agent_model::NOH, 0);
  for (int i = 0; i < agent_model::NOH; i++) {    
    ds[i] = pow((i + 1) / double(agent_model::NOH), 2)  * s_max;
  }

  // caculate current ego s
  Point2D ego_position_cl;
  int hor_idx = closestCenterlinePoint(ego_position_, path_centerline_, ego_position_cl);
  
  // crop idx to boundaries
  if (hor_idx == 0) hor_idx = 1;
  if (hor_idx == path_centerline_.size()) hor_idx = path_centerline_.size();

  double ego_s = path_s_[hor_idx - 1] + sqrt(
              pow(ego_position_cl.x - path_centerline_[hor_idx - 1].x,2) + 
              pow(ego_position_cl.y - path_centerline_[hor_idx - 1].y,2));
  double ego_psi = ego_base_.orientation().yaw();

  // iterate over all horizon points
  for (int i = 0; i < agent_model::NOH; i++) {

    // get correct idx on path array
    int idx = -1;
    for (auto &ss : path_s_) {
      if (ss > ego_s + ds[i]) break;
      idx++;
    }

    // calculate properties at current horizon knot
    Point2D horizon_knot;

    // interpolate if end of path not reached
    if (ego_s + ds[i] < path_s_.back()) {
      
      double ds_path = path_s_[idx + 1] - path_s_[idx];
      double frac = (ego_s + ds[i] - path_s_[idx]) / ds_path;

      double dx_path = path_centerline_[idx + 1].x - path_centerline_[idx].x;
      double dy_path = path_centerline_[idx + 1].y - path_centerline_[idx].y;
      double dpsi_path = path_psi_[idx + 1] - path_psi_[idx];
      double dkappa_path = path_kappa_[idx + 1] - path_kappa_[idx];

      horizon_knot.x = path_centerline_[idx].x + frac * dx_path;
      horizon_knot.y = path_centerline_[idx].y + frac * dy_path;

      input.horizon.psi[i] = path_psi_[idx] + frac * dpsi_path - ego_psi;
      input.horizon.kappa[i] = path_kappa_[idx] + frac * dkappa_path;

      input.horizon.ds[i] = ds[i];
    }
    // take last values if end of path reached (no extrapolation)
    else {
      horizon_knot.x = path_centerline_.back().x;
      horizon_knot.y = path_centerline_.back().y;

      input.horizon.psi[i] = path_psi_.back();
      input.horizon.kappa[i] = path_kappa_.back();

      input.horizon.ds[i] = path_s_.back() - ego_s;
    }

    // set x and y relative to ego position
    input.horizon.x[i] = std::cos(ego_psi) * (horizon_knot.x - ego_position_.x)
                       + std::sin(ego_psi) * (horizon_knot.y - ego_position_.y);
    input.horizon.y[i] =-std::sin(ego_psi) * (horizon_knot.x - ego_position_.x) 
                       + std::cos(ego_psi) * (horizon_knot.y - ego_position_.y);

    // set lane widths
    input.horizon.egoLaneWidth[i] = 3.75;
    input.horizon.leftLaneWidth[i] = 3.75;
    input.horizon.rightLaneWidth[i] = 3.75;
  }
  
  // add destination point to horizon
  input.horizon.destinationPoint = xy2s(ego_centerline_point_, dest_point_, 
                          path_centerline_, ego_base_.orientation().yaw());
}

void OsiConverter::fillLanes(osi3::SensorView &sensor_view,
                             agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();
  
  int lane = 0;

  // get ego lane information
  int ego_lane_idx = findLaneIdx(ground_truth, ego_lane_id_);
  std::vector<Point2D> ego_lane_points;
  getXY(ego_lane_ptr_, ego_lane_points);

  // check if lane change is possible from current ego lane
  bool lane_change_possible = (find(lanes_changeable_.begin(), lanes_changeable_.end(), ego_lane_id_) != lanes_changeable_.end());

  std::vector<int> processed_lanes;

  // loop over all lane groups
  for (auto lane_group : lane_groups_)
  {
    int lane_id = lane_group.id - current_lane_group_;

    // lane group properties
    int start_lane;
    int end_lane;
    int end_of_route;
    double width;
    bool lane_change;

    // skip if not ego or neighbouring lane group
    if (abs(lane_id) > 1) {
      continue;
    }

    // neighbouring lane group
    if (abs(lane_id) == 1)
    {
      // continue if lane change not possible from ego lane
      if (!lane_change_possible) continue;

      // get adjacent lanes and try to find start_lane on lane_group
      std::vector<int> adj_idx;
      if (lane_id == 1){
        adj_idx = getAdjacentLanes(ground_truth, ego_lane_idx,'L');
        width = input.horizon.leftLaneWidth[0];
      }
      if (lane_id == -1) {
        adj_idx = getAdjacentLanes(ground_truth, ego_lane_idx,'R');
        width = input.horizon.rightLaneWidth[0];
      }

      bool found = false;
      for (auto a_idx: adj_idx)
      {
        int a_id = ground_truth->lane(a_idx).id().value();
        if (find(lane_group.lanes.begin(), lane_group.lanes.end(), a_id) != lane_group.lanes.end()) 
        {
          start_lane = a_id;
          found = true;
          break;
        }
      }

      if (!found) continue;
    }

    // ego lane group
    if (lane_id == 0){
      width = input.horizon.egoLaneWidth[0];
      start_lane = ego_lane_id_;
    }

    // compute start and end points
    end_lane = lane_group.lanes.back();

    end_of_route = (lane_group.lane_changes == 0)? end_lane : lane_group.lanes_changeable.back();

    lane_change = (lane_change_possible && lane_group.lane_changes != 0) ? true : false;
  

    // fill general information
    input.lanes[lane].id = lane_id;
    input.lanes[lane].width = width;
    input.lanes[lane].access = agent_model::Accessibility::ACC_ACCESSIBLE;
    input.lanes[lane].dir = agent_model::DrivingDirection::DD_FORWARDS;
    input.lanes[lane].lane_change = lane_change;

    // properties
    double ds_route = 0;
    double ds_closed = 0;
    bool on_route = false;
    Point2D current_end_point = ego_lane_points.back();

    // iterate backwards over lanes and compute ds
    for (int i = lane_group.lanes.size()-1; i >= 0; i--){

      int cur_lane_id = lane_group.lanes[i];

      // add current lane to processed_lanes
      processed_lanes.push_back(cur_lane_id);

      // check if now on route on_route
      if (!on_route && cur_lane_id == end_of_route) {
        on_route = true;
      }

      // break if ego_lane reached
      if (cur_lane_id == start_lane) break;

      // get ds of current lane
      std::vector<Point2D> tmp_lane_points;
      getXY(findLane(cur_lane_id, ground_truth), tmp_lane_points);

      
      double start_psi = atan2(tmp_lane_points[1].y - tmp_lane_points[0].y, tmp_lane_points[1].x - tmp_lane_points[0].x);
      double ds = xy2s(tmp_lane_points.front(), tmp_lane_points.back(), tmp_lane_points, start_psi);

      // increase distance_to_end
      ds_closed += ds;

      if (on_route) ds_route += ds;

      // update current_end_point
      current_end_point = tmp_lane_points.front();
    }

    // get remaining distance (always on ego lane)
    double dist = xy2s(ego_centerline_point_, current_end_point, ego_lane_points, ego_base_.orientation().yaw());

    // set road end in input struct
    input.lanes[lane].closed = dist + ds_closed;
    input.lanes[lane].route = dist + ds_route;
    lane++;
  }

  //iterate over all remaining lanes
  for (int i=0; i < ground_truth->lane_size() && lane < agent_model::NOL; i++) {
    osi3::Lane tmp_lane = ground_truth->lane(i);

    // skip if tmp_lane on host path, already processed before
    if (find(processed_lanes.begin(), processed_lanes.end(), tmp_lane.id().value()) != processed_lanes.end()) {
      continue;
    }

    // set id from lane_mapping_
    input.lanes[lane].id = lane_mapping_[tmp_lane.id().value()];

    // calculate type
    if (tmp_lane.classification().type() ==
        osi3::Lane_Classification_Type_TYPE_DRIVING) {
      input.lanes[lane].access = agent_model::Accessibility::ACC_ACCESSIBLE;
    } else if (tmp_lane.classification().type() ==
               osi3::Lane_Classification_Type_TYPE_NONDRIVING) {
      input.lanes[lane].access = agent_model::Accessibility::ACC_NOT_ACCESSIBLE;
    } else
    {
      input.lanes[lane].access = agent_model::ACC_NOT_SET;
    }

    // calculate driving direction
    if (tmp_lane.classification().centerline_is_driving_direction() ==
        ego_lane_ptr_->classification().centerline_is_driving_direction()){
      input.lanes[lane].dir = agent_model::DrivingDirection::DD_FORWARDS;
    }
    else {
      input.lanes[lane].dir = agent_model::DrivingDirection::DD_BACKWARDS;
    }

    // save average of width
    input.lanes[lane].width = input.horizon.egoLaneWidth[0];

    input.lanes[lane].closed = -1;
    input.lanes[lane].route = -1;

    lane++;
  }

  // fill remaining lanes with default values
  for (int i = lane; i < agent_model::NOL; i++) {
    input.lanes[i].id = 127;
    input.lanes[i].width = 0;
    input.lanes[i].route = 0;
    input.lanes[i].closed = 0;
    input.lanes[i].access = agent_model::ACC_NOT_SET;
    input.lanes[i].dir = agent_model::DD_NOT_SET;
  }
}

