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

#include <fstream>
#include <iostream>

#include "OSI_helper.h"

void OsiConverter::convert(osi3::SensorView &sensor_view,
                           osi3::TrafficCommand &traffic_command,
                           agent_model::Input &input,
                           agent_model::Parameters &param) {
  extractEgoInformation(sensor_view, input);

  preprocess(sensor_view, traffic_command, input, param);

  fillVehicle(sensor_view, input);
  fillSignals(sensor_view, input);
  fillTargets(sensor_view, input);
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
  // multiple assigned lanes, only possible after lanes_ was created
  // e.g. on intersection: iterate over lanes_ in reverse order, (take last)
  else if (ego_.assigned_lane_id_size() > 1 && lanes_.size() > 0) {
    for (int j = lanes_.size() - 1; j >= 0; j--) {
      for (int i = 0; i < ego_.assigned_lane_id_size(); i++) {
        if (find(lanes_.begin(), lanes_.end(),
                 ego_.assigned_lane_id(i).value()) != lanes_.end()) {
          ego_lane_id_ = ego_.assigned_lane_id(i).value();
          break;
        }
      }
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

  if (!initialized)
  {
    ego_position_.x = ego_base_.position().x();
    ego_position_.y = ego_base_.position().y();
    initialized = true;
  }

}

void OsiConverter::preprocess(osi3::SensorView &sensor_view,
                              osi3::TrafficCommand &traffic_command,
                              agent_model::Input &input,
                              agent_model::Parameters &param) {

  // skip if lanes_ already exist and no new traffic command exist
  if (lanes_.size() > 0 && traffic_command.action_size() == 0) return;

  // analyize traffic commands
  trafficCommandToLanes(sensor_view, traffic_command, param);

  // generate paths
  generatePath(sensor_view, input);

  // determine type of maneuver on intersection
  classifyManeuver(sensor_view, input);
}

void OsiConverter::trafficCommandToLanes(osi3::SensorView &sensor_view,
                                         osi3::TrafficCommand &traffic_command,
                                         agent_model::Parameters &param) {

  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // find starting_lane_idx
  int starting_lane_idx = findLaneId(ground_truth, ego_lane_id_);

  // iterate over all traffic commands
  for (int i = 0; i < traffic_command.action_size(); i++) {

    // take global position for lane calculation
    if (traffic_command.action(i).has_acquire_global_position_action()) {
      osi3::TrafficAction_AcquireGlobalPositionAction position =
        traffic_command.action(i).acquire_global_position_action();

      Point2D dest_point =
        Point2D(position.position().x(), position.position().y());

      lanes_.clear();
      futureLanes(ground_truth, starting_lane_idx, dest_point, lanes_);
    }

    // take end position of trajectory for lane calculation
    if (traffic_command.action(i).has_follow_trajectory_action() &&
        (traffic_command.action(i)
           .follow_trajectory_action()
           .action_header()
           .action_id()
           .value() != traj_action_id_)) {
      osi3::TrafficAction_FollowTrajectoryAction traj =
        traffic_command.action(i).follow_trajectory_action();
      traj_action_id_ = traj.action_header().action_id().value();

      Point2D dest_point = Point2D(
        traj.trajectory_point(traj.trajectory_point_size() - 1).position().x(),
        traj.trajectory_point(traj.trajectory_point_size() - 1).position().y());

      lanes_.clear();
      futureLanes(ground_truth, starting_lane_idx, dest_point, lanes_);
    }

    // take end position of path for lane calculation
    if (traffic_command.action(i).has_follow_path_action() &&
        (traffic_command.action(i)
           .follow_path_action()
           .action_header()
           .action_id()
           .value() != path_action_id_)) {
      osi3::TrafficAction_FollowPathAction path =
        traffic_command.action(i).follow_path_action();
      path_action_id_ = path.action_header().action_id().value();

      Point2D dest_point =
        Point2D(path.path_point(path.path_point_size() - 1).position().x(),
                path.path_point(path.path_point_size() - 1).position().y());

      lanes_.clear();
      futureLanes(ground_truth, starting_lane_idx, dest_point, lanes_);
    }

    // speed action - no lane calculation
    if (traffic_command.action(i).has_speed_action() &&
        (traffic_command.action(i)
           .speed_action()
           .action_header()
           .action_id()
           .value() != speed_action_id_)) {
      osi3::TrafficAction_SpeedAction speed =
        traffic_command.action(i).speed_action();

      speed_action_id_ = speed.action_header().action_id().value();
      param.velocity.vComfort = speed.absolute_target_speed();
    }
  }

  // check if lanes_ still empty (e.g. when no traffic_command is available)
  if (lanes_.size() == 0) {
    // set dest_point to end of lane
    int pos = 0;
    if (ego_lane_ptr_->classification().centerline_is_driving_direction()) {
      pos = ego_lane_ptr_->classification().centerline().size() - 1;
    }

    Point2D dest_point =
      Point2D(ego_lane_ptr_->classification().centerline(pos).x(),
              ego_lane_ptr_->classification().centerline(pos).y());

    lanes_.clear();
    futureLanes(ground_truth, starting_lane_idx, dest_point, lanes_);
  }

  // fill lane_mapping_
  mapLanes(ground_truth, lane_mapping_, ego_lane_ptr_, lanes_);

  // print lanes
  std::cout << "This are the lanes to pass: ";
  for (auto &lane : lanes_) std::cout << lane << " ";
  std::cout << std::endl;
}

/**
 * @brief Generates paths and junction paths
 *
 *
 * @param sensor_view osi sensor view from first time step
 * @param input agent_models input
 */
void OsiConverter::generatePath(osi3::SensorView &sensor_view,
                                agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  int gap_idx = 0;  // idx in path_centerline_ for free boundary lane points

  // get all relevant path points from lanes_
  for (auto &l : lanes_) {
    osi3::Lane *tmp_lane = findLane(l, ground_truth);
    bool is_free_boundary_lane = false;

    // determine lane typeif lane normal
    if (tmp_lane->classification().free_lane_boundary_id_size() > 0 &&
        tmp_lane->classification().type() ==
          osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
      is_free_boundary_lane = true;
    }


    if (!is_free_boundary_lane)
      getXY(tmp_lane, path_centerline_);
    else
      gap_idx = path_centerline_.size() - 1;
  }

  // fill gap with interpolation based on two points at each end
  if (gap_idx > 0) {
    std::vector<Point2D> gap_points(&path_centerline_[gap_idx - 1],
                                    &path_centerline_[gap_idx + 3]);
    calcXYGap(gap_points, path_centerline_, gap_idx);
  }

  // remove duplicates (if ds is very small)
  removeDuplicates(path_centerline_);

  // calculate s, psi, kappa from centerline
  xy2Curv(path_centerline_, path_s_, path_psi_, path_kappa_);

  // generate junction paths for traffic signs
  std::vector<int> start_lane_ids;
  for (auto &sign : *ground_truth->mutable_traffic_sign()) {

    // only RIGHT_OF_WAY and GIVE_WAY is supported
    if (sign.main_sign().classification().type() !=
          osi3::
            TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN &&
        sign.main_sign().classification().type() !=
          osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY)
      continue;

    // create path from assigned lane of sign backwards
    for (auto &assigned_lane :
         sign.main_sign().classification().assigned_lane_id()) {

      // every lane can be starting point for only ONE sign
      int lane_id = assigned_lane.value();
      if (std::find(start_lane_ids.begin(), start_lane_ids.end(), lane_id) !=
          start_lane_ids.end())
        continue;
      start_lane_ids.push_back(lane_id);

      // create junction_path
      JunctionPath junction_path;
      junction_path.signal_id = sign.id().value();
      junction_path.lane_ids.push_back(lane_id);

      // create path until no next lane is found
      bool found_next_lane = true;
      while (found_next_lane) {
        auto *lane = findLane(lane_id, ground_truth);

        // iterate over all lane pairings
        for (auto &l_pairs : lane->classification().lane_pairing()) {
          found_next_lane = false;
          int next_id, from_id;

          // set next_id of lane in front of signal as well as from_id
          if (lane->classification().centerline_is_driving_direction()) {
            next_id = l_pairs.antecessor_lane_id().value();
            from_id = l_pairs.successor_lane_id().value();
          } else {
            next_id = l_pairs.successor_lane_id().value();
            from_id = l_pairs.antecessor_lane_id().value();
          }

          // break when another intersection is reached
          if (findLane(next_id, ground_truth)->classification().type()) break;

          // from_id is only equal to current lane_id in openPASS OSI
          if (open_pass && from_id != lane_id) continue;

          // add new lane to junction path
          found_next_lane = true;
          junction_path.lane_ids.push_back(lane_id);
          lane_id = next_id;
          break;
        }
      }

      // push junction path to global lanes
      if (sign.main_sign().classification().type() ==
          osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY)
        yielding_lanes_.push_back(junction_path);
      else
        priority_lanes_.push_back(junction_path);
    }
  }

  // flip global lanes and add points of centerlines
  for (auto &yl : yielding_lanes_) {
    std::reverse(yl.lane_ids.begin(), yl.lane_ids.end());
    for (auto &l : yl.lane_ids) getXY(findLane(l, ground_truth), yl.pts);
  }
  for (auto &pl : priority_lanes_) {
    std::reverse(pl.lane_ids.begin(), pl.lane_ids.end());
    for (auto &l : pl.lane_ids) getXY(findLane(l, ground_truth), pl.pts);
  }

  // set interesection lanes
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
 * @param sensor_view
 * @param input
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

  // projection of ego coordiantes on centerline
  closestCenterlinePoint(ego_position_, path_centerline_, ego_centerline_point_);

  // calculate s, psi, k of ego lane
  std::vector<double> s, psi, k;
  xy2Curv(path_centerline_, s, psi, k);

  // angle between ego yaw and deviation from centerline point
  input.vehicle.psi = ego_base_.orientation().yaw() - 
              interpolateXY2Value(psi, path_centerline_, ego_centerline_point_);
  input.vehicle.dPsi = ego_base_.orientation_rate().yaw();

  // compute distance between ego and centerline
  input.vehicle.d = computeDistanceInRefAngleSystem(ego_position_, ego_centerline_point_, ego_base_.orientation().yaw());

  // set dummy values
  input.vehicle.pedal = 0;
  input.vehicle.steering = 0;
}

void OsiConverter::fillSignals(osi3::SensorView &sensor_view,
                               agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  int signal = 0;
  std::vector<int> signal_lanes; // TODO CGE: is this correct? we could also have multiple signals on a single lane?

  std::vector<int> traffic_light_ids;            
  std::vector<Point2D> traffic_light_positions;

  // iterate over all traffic lights
  for (int i = 0; i < ground_truth->traffic_light_size(); i++) {

    // only consider NOTL signals
    if (signal >= agent_model::NOTL) break;

    osi3::TrafficLight light = ground_truth->traffic_light(i);
    osi3::TrafficLight_Classification cls = light.classification();

    // check if signal is assigned along the route
    bool assigned = false;
    for (int j = 0; j < cls.assigned_lane_id_size(); j++) {

      // check that a lane has only one signal
      if (find(signal_lanes.begin(), signal_lanes.end(),
               cls.assigned_lane_id(j).value()) != signal_lanes.end())
        continue;
      else
        signal_lanes.push_back(cls.assigned_lane_id(j).value());

      // check if lane on route
      auto signal_lane = find(lanes_.begin(), lanes_.end(), cls.assigned_lane_id(j).value());
      if (signal_lane != lanes_.end()) {
        assigned = true;
        break;
      }
    }
    // skip a non-assigned signal    
    if (!assigned) continue;

    // projection of signal position to centerline
    Point2D centerline_point;
    Point2D signal_point(light.base().position().x(), light.base().position().y());
    closestCenterlinePoint(signal_point, path_centerline_, centerline_point);
    traffic_light_positions.push_back(signal_point);

    // save all orignal signal ids
    traffic_light_ids.push_back(light.id().value());

    // add signal with id
    input.signals[signal].id = signal + 1;

    // ds along centerline to reach signal 
    input.signals[signal].ds = xy2SSng(ego_centerline_point_, centerline_point, path_centerline_, ego_base_.orientation().yaw());

    // set Color and type/icon
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

    // only consider NOTL signals
    if (signal >= agent_model::NOS) break;

    osi3::TrafficSign sign = ground_truth->traffic_sign(i);
    osi3::TrafficSign_MainSign_Classification cls =
      sign.main_sign().classification();

    // check that the sign is assigned to a lane along the route.
    bool assigned = false;
    for (int j = 0; j < cls.assigned_lane_id_size(); j++) {
      if (find(signal_lanes.begin(), signal_lanes.end(),
               cls.assigned_lane_id(j).value()) != signal_lanes.end())
        continue;
      else
        signal_lanes.push_back(cls.assigned_lane_id(j).value());

      // check if lane on route
      auto signal_lane = find(lanes_.begin(), lanes_.end(), cls.assigned_lane_id(j).value());
      if (signal_lane != lanes_.end()) {
        assigned = true;
        break;
      }
    }
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
    input.signals[signal].ds = xy2SSng(ego_centerline_point_, centerline_point, path_centerline_, ego_base_.orientation().yaw());

    // set defaults
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
      input.signals[signal].pairedSignalID[count] = paired_signal_id;

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

  // fill remaining signals with default values
  for (int i = signal; i < agent_model::NOS; i++) {
    input.signals[i].id = 127;
    input.signals[i].ds = INFINITY;
    input.signals[i].type = agent_model::SIGNAL_NOT_SET;
    input.signals[i].value = 0;
  }
}

void OsiConverter::fillTargets(osi3::SensorView &sensor_view,
                               agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // iterate over all targets
  int target = 0;
  for (auto &tar : *ground_truth->mutable_moving_object()) {

    // skip ego vehicle
    if (tar.id().value() == ego_id_) continue;

    osi3::BaseMoving target_base = tar.base();

    // set general properties
    input.targets[target].id = target + 1;
    input.targets[target].priority = agent_model::TARGET_PRIORITY_NOT_SET;
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
        input.targets[target].priority = agent_model::TARGET_ON_INTERSECTION;
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
      double ds_target = xy2SSng(ego_centerline_point_, target_point,  
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
    } 
    
    // compute if target (not on path and interesection) approaches intersection
    if (!assigned && input.targets[target].priority != agent_model::TARGET_ON_INTERSECTION) {
      
      // iterate over assigned lanes
      for (auto &assigned_lane : tar.assigned_lane_id()) {

        std::vector<Point2D> path_points;
        bool approaching_junction = false;

        // target is on a yield lane to the heading to the junction
        for (auto &yield_lane : yielding_lanes_) {
          if (find(yield_lane.lane_ids.begin(), yield_lane.lane_ids.end(),
                    assigned_lane.value()) != yield_lane.lane_ids.end()) {
            input.targets[target].priority = agent_model::TARGET_ON_GIVE_WAY_LANE;
            path_points = yield_lane.pts;
            approaching_junction = true;
            break;
          }
        }
        
        // target is on a priority lane to the heading to the junction
        for (auto &prioL : priority_lanes_) {
          if (find(prioL.lane_ids.begin(), prioL.lane_ids.end(),
                    assigned_lane.value()) != prioL.lane_ids.end()) {
            input.targets[target].priority =
              agent_model::TARGET_ON_PRIORITY_LANE;
            path_points = prioL.pts;
            approaching_junction = true;
            break;
          }
        }

        // calculate distance to intersection if approaching intersection
        if (approaching_junction)
        {
          input.targets[target].dsIntersection =
            xy2s(Point2D(target_base.position().x(), target_base.position().y()), path_points.back(), path_points);  
          break;
        }
      }
    }
    target++;
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
  Point2D dummy;
  int hor_idx = closestCenterlinePoint(ego_position_, path_centerline_, dummy);
  double ego_s = path_s_[hor_idx - 1] + sqrt(
              pow(ego_position_.x - path_centerline_[hor_idx - 1].x,2) + 
              pow(ego_position_.y - path_centerline_[hor_idx - 1].y,2));
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
    // take last values if end of path reached
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
    input.horizon.leftLaneWidth[i] = 0;
    input.horizon.rightLaneWidth[i] = 0;
  }
}

void OsiConverter::fillLanes(osi3::SensorView &sensor_view,
                             agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  int lane = 0;

  // host path (lanes_) constitutes only one lane in input
  input.lanes[lane].id = lane_mapping_[lanes_[lane]];
  input.lanes[lane].access = agent_model::Accessibility::ACC_ACCESSIBLE; 
  input.lanes[lane].width = input.horizon.egoLaneWidth[0];
  input.lanes[lane].dir = agent_model::DrivingDirection::DD_FORWARDS;

  // get points of ego lane
  std::vector<Point2D> ego_lane_points;
  getXY(ego_lane_ptr_, ego_lane_points);

  // initialize parameters
  auto it = lanes_.end();
  double distance_to_end = 0;
  Point2D current_end_point = ego_lane_points.back();

  // iterate backwards over path and increase distance_to_end over all lanes
  while (it != lanes_.begin()) {
    it--;
    
    // break if ego_lane_ptr_ reached
    if (*it ==  ego_lane_ptr_->id().value()) break;

    std::vector<Point2D> tmp_lane_points;
    getXY(findLane(*it, ground_truth), tmp_lane_points);

    // increase distance_to_end
    distance_to_end += xy2s(tmp_lane_points.front(), tmp_lane_points.back(), tmp_lane_points);

    // update current_end_point
    current_end_point = tmp_lane_points.front();
  }

  // get remaining distance on ego lane
  double dist = xy2s(ego_centerline_point_, current_end_point, ego_lane_points);

  // set road end in input struct
  input.lanes[lane].closed = dist + distance_to_end;
  input.lanes[lane].route = input.lanes[lane].closed;
  lane++;

  //iterate over all remaining lanes
  for (int i = 0; i < ground_truth->lane_size() && i < agent_model::NOL; i++) {
    osi3::Lane tmp_lane = ground_truth->lane(i);

    // skip if tmp_lane on host path, already considered before
    if (find(lanes_.begin(), lanes_.end(), tmp_lane.id().value()) != lanes_.end()) {
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

    std::vector<Point2D> lane_points;
    getXY(&tmp_lane, lane_points);

    input.lanes[lane].closed = 0;
    input.lanes[lane].route = 0;

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
