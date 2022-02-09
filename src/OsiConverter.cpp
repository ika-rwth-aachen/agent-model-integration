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
}

void OsiConverter::preprocess(osi3::SensorView &sensor_view,
                              osi3::TrafficCommand &traffic_command,
                              agent_model::Input &input,
                              agent_model::Parameters &param) {

  // skip if lanes_ already exist and no new traffic command exist
  if (lanes_.size() > 0 && traffic_command.action_size() == 0) return;

  // analyize traffic commands
  trafficCommandToLanes(sensor_view, traffic_command, param);

  // determine type of maneuver on intersection
  classifyManeuver(sensor_view, input);

  // generate horizon
  generatePath(sensor_view, input);
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
        // TODO: not yet implemented (standard OSI)
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

  input.vehicle.v = getNorm(ego_base_.velocity());
  input.vehicle.a = getNorm(ego_base_.acceleration());

  // compute ego road coordinates and set member variables to new values
  double dx = ego_base_.position().x() - last_position_.x;
  double dy = ego_base_.position().y() - last_position_.y;

  double ego_psi = ego_base_.orientation().yaw();

  // calculate traveled distance
  last_position_.x = ego_base_.position().x();
  last_position_.y = ego_base_.position().y();
  last_s_ += sqrt(dx * dx + dy * dy);
  input.vehicle.s = last_s_;

  // save angle of ego x/s-axis because targets.psi and horizon.psi are
  // relative
  input.vehicle.dPsi = ego_base_.orientation_rate().yaw();

  // projection of ego coordiantes on centerline
  int curCLidx = closestCenterlinePoint(last_position_, path_centerline_,
                                        ego_centerline_point_);

  // calculate s, psi, k of ego lane
  std::vector<double> s, psi, k;
  xy2Curv(path_centerline_, s, psi, k);

  // seems to have errors...
  input.vehicle.psi =
    ego_psi - interpolateXY2Value(psi, path_centerline_, ego_centerline_point_);

  double dirCL = atan2(ego_centerline_point_.y - last_position_.y,
                       ego_centerline_point_.x - last_position_.x);
  double orientation = ego_psi - dirCL;
  if (orientation < -M_PI) orientation = orientation + 2 * M_PI;
  if (orientation > M_PI) orientation = orientation - 2 * M_PI;
  int d_sig = (orientation > 0) - (orientation < 0);
  double distCL = sqrt(pow(last_position_.x - ego_centerline_point_.x, 2) +
                       pow(last_position_.y - ego_centerline_point_.y, 2));
  input.vehicle.d = d_sig * distCL;

  // dummy values
  input.vehicle.pedal = 0;     // TODO
  input.vehicle.steering = 0;  // TODO
}


void OsiConverter::fillSignals(osi3::SensorView &sensor_view,
                               agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  int signal = 0;
  std::vector<int> knownAsLane;
  std::vector<int> allTLS_IDs;            // save all IDs of TrafficLight's
  std::vector<Point2D> knownSigPosition;  // save all known/processed signal
                                          // positions in this vector

  auto it_knownSigPosition = knownSigPosition.begin();

  int DEBUG_TLS_Size = ground_truth->traffic_light_size();
  int DEBUG_SIGN_Size = ground_truth->traffic_sign_size();
  int DEBUG_ID_Value;

  for (int i = 0; i < ground_truth->traffic_light_size(); i++) {
    if (signal >= agent_model::NOTL) break;

    osi3::TrafficLight light = ground_truth->traffic_light(i);
    osi3::TrafficLight_Classification clas = light.classification();

    auto it = lanes_.end();
    // check: sign is assigned to lane along the route?
    bool assigned = false;
    for (int k = 0; k < clas.assigned_lane_id_size(); k++) {
      if (find(knownAsLane.begin(), knownAsLane.end(),
               clas.assigned_lane_id(k).value()) != knownAsLane.end())
        continue;
      else
        knownAsLane.push_back(clas.assigned_lane_id(k).value());

      it = find(lanes_.begin(), lanes_.end(), clas.assigned_lane_id(k).value());
      if (it != lanes_.end()) {
        assigned = true;
        break;
      }
    }

    if (!assigned) continue;
    // projection of trafficlight_signal position on centerline: cPoint
    Point2D cPoint;
    Point2D sPoint(light.base().position().x(), light.base().position().y());
    Point2D target = sPoint;

    closestCenterlinePoint(sPoint, path_centerline_, cPoint);

    // sLight holds s value along centerlines to reach the signal
    double sLight = xy2SSng(ego_centerline_point_, cPoint, path_centerline_,
                            ego_base_.orientation().yaw());

    // signal on future lane / deleted comments
    input.signals[signal].id =
      signal + 1;  // could also take light.id().value() as id here, OSI
                   // ids are often larger numbers
    input.signals[signal].ds = sLight;

    // save postition to check later on: signal is standalone signal or sign
    // combined with TLS?
    knownSigPosition.push_back(sPoint);

    // int k = light.id().value();
    // save all TLS IDs to assign IDs of combined signs later on
    allTLS_IDs.push_back(light.id().value());

    DEBUG_ID_Value = light.id().value();

    // TODO: Write function to set Icons
    // Calculate Color and Type/Icon
    if (clas.color() == osi3::TrafficLight_Classification_Color_COLOR_RED) {
      // if(clas.is_out_of_service() ==
      // osi3::TrafficLight_Classification::is_out_of_service)
      // osi3::TrafficLight_Classification::is_out_of_service;

      input.signals[signal].color = agent_model::COLOR_RED;

      if (clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
        input.signals[signal].icon = agent_model::ICON_NONE;
      else if (clas.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
        input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
      else if (clas.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
        input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;
    } else if (clas.color() ==
               osi3::TrafficLight_Classification_Color_COLOR_YELLOW) {
      input.signals[signal].color = agent_model::COLOR_YELLOW;

      if (clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
        input.signals[signal].icon = agent_model::ICON_NONE;
      else if (clas.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
        input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
      else if (clas.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
        input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;
    } else if (clas.color() ==
               osi3::TrafficLight_Classification_Color_COLOR_GREEN) {
      input.signals[signal].color = agent_model::COLOR_GREEN;

      if (clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
        input.signals[signal].icon = agent_model::ICON_NONE;
      else if (clas.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
        input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
      else if (clas.icon() ==
               osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
        input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;
    }
    signal++;
  }

  int amountLights = signal;

  auto it_allTLS_IDs = allTLS_IDs.begin();

  for (int i = 0; i < ground_truth->traffic_sign_size(); i++) {
    if (signal >= agent_model::NOS) break;

    osi3::TrafficSign sign = ground_truth->traffic_sign(i);
    osi3::TrafficSign_MainSign_Classification clas =
      sign.main_sign().classification();

    auto it = lanes_.end();
    // check that the sign is assigned to a lane along the route.
    bool assigned = false;
    for (int j = 0; j < clas.assigned_lane_id_size(); j++) {
      if (find(knownAsLane.begin(), knownAsLane.end(),
               clas.assigned_lane_id(j).value()) != knownAsLane.end())
        continue;
      else
        knownAsLane.push_back(clas.assigned_lane_id(j).value());

      it = find(lanes_.begin(), lanes_.end(), clas.assigned_lane_id(j).value());
      if (it != lanes_.end()) {
        assigned = true;
        // std::cout << "\nsignal on lane: " <<
        // clas.assigned_lane_id(j).value()
        // << std::endl;
        break;
      }
    }

    if (!assigned) continue;

    // sSig will hold s value along centerlines to reach the signal
    // projection of signal position on centerline: cPoint
    Point2D cPoint;
    Point2D sPoint(sign.main_sign().base().position().x(),
                   sign.main_sign().base().position().y());
    // closestCenterlinePoint(sPoint, elPoints, cPoint);
    // knownSigPosition.push_back(sPoint);

    Point2D target = sPoint;
    closestCenterlinePoint(sPoint, path_centerline_, cPoint);
    double sSig = xy2SSng(ego_centerline_point_, cPoint, path_centerline_,
                          ego_base_.orientation().yaw());
    // std::cout << "Spoint: (" << sPoint.x << "," << sPoint.y << ")" <<
    // std::endl; signal on future lane
    /*if (*it != ego_lane_ptr_->id().value()) {
            // signal is not assigned to the current lane -> add distance
    along the signal's assigned lane
            // meaning assigned_lane beginning to sPoint
            std::vector<Point2D> pos;
            getXY(findLane(*it, ground_truth), pos);
            sSig += xy2s(pos.front(), sPoint, pos);
            pos.clear();
    }
    while (it != futureLanes.begin() && *(it - 1) !=
    ego_lane_ptr_->id().value()) { it--; std::vector<Point2D> pos;
            getXY(findLane(*it, ground_truth), pos);
            sSig += xy2s(pos.front(), pos.back(), pos);
            target = pos.front();
    }

    //std::cout << "\nhost : (" << last_position_.x << "," <<
    last_position_.y <<
    ")\nsignal: (" << sPoint.x << "," << sPoint.y << ") " << "\ntarget (" <<
    target.x << "," << target.y << ") " << std::endl;

    sSig += xy2s(ego_centerline_point_, target, elPoints);*/

    input.signals[signal].id =
      signal + 1;  // could also take sign.id().value() as id here, OSI
                   // ids are often larger numbers
    input.signals[signal].ds = sSig;

    // check if signal position is already known (due to prior signs)
    it_knownSigPosition =
      find(knownSigPosition.begin(), knownSigPosition.end(), sPoint);

    if (it_knownSigPosition != knownSigPosition.end()) {
      input.signals[signal].subsignal = true;
    }

    int itDistance[3];
    int k = 0;

    while ((it_knownSigPosition != knownSigPosition.end())) {
      itDistance[k] = it_knownSigPosition - knownSigPosition.begin();
      it_knownSigPosition =
        find(it_knownSigPosition, knownSigPosition.end(), sPoint);

      k++;
      if (k >= 3) break;
    }

    if (!allTLS_IDs.empty() /*||amountLights != 0*/) {
      for (int k = 0; k < 3; k++) {
        // it_allTLS_IDs is at same index as it_knownSigPosition was
        std::advance(it_allTLS_IDs, itDistance[k]);
        // it_allTLS_IDs + itDistance[k];
        input.signals[signal].pairedSignalID[k] = *it_allTLS_IDs;
      }

      // check if whole Trafficlight is out of service -> Sign must be in
      // use
      if (ground_truth->mutable_traffic_light()
            ->Get(input.signals[signal].pairedSignalID[0])
            .classification()
            .is_out_of_service() &&
          ground_truth->mutable_traffic_light()
            ->Get(input.signals[signal].pairedSignalID[1])
            .classification()
            .is_out_of_service() &&
          ground_truth->mutable_traffic_light()
            ->Get(input.signals[signal].pairedSignalID[2])
            .classification()
            .is_out_of_service())
        input.signals[signal].sign_is_in_use = true;
      else
        input.signals[signal].sign_is_in_use = false;
    } else {
      input.signals[signal].subsignal = false;
      input.signals[signal].sign_is_in_use = true;
    }

    // if (find(knownSigPosition.begin(),knownSigPosition.end(),sPoint) !=
    // knownSigPosition.end()) 	input.signals[signal].subsignal=true;

    // calculate type
    if (clas.type() ==
        osi3::TrafficSign_MainSign_Classification_Type_TYPE_STOP) {
      input.signals[signal].type = agent_model::SIGNAL_STOP;
      // no value for stop
      input.signals[signal].value = 0;
    } else if (
      clas.type() ==
      osi3::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN) {
      input.signals[signal].type = agent_model::SIGNAL_SPEED_LIMIT;
      input.signals[signal].value = clas.value().value();
    } else if (
      clas.type() ==
      osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN) {
      // for (int j = 0; j < clas.assigned_lane_id_size(); j++)
      //	priority_lanes_.push_back(clas.assigned_lane_id(j).value());
      input.signals[signal].type = agent_model::SIGNAL_PRIORITY;
    } else if (
      clas.type() ==
        osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY ||
      clas.type() ==
        osi3::
          TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_BEFORE_LEFT_NEXT_INTERSECTION) {

      // for (int j = 0; j < clas.assigned_lane_id_size(); j++)
      //	yielding_lanes_.push_back(clas.assigned_lane_id(j).value());
      input.signals[signal].type = agent_model::SIGNAL_YIELD;
    } else {
      input.signals[signal].type = agent_model::SIGNAL_NOT_SET;
      // no value for unset type
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

  std::vector<int> intersectionLanes;

  for (int i = 0; i < ground_truth->lane_size(); i++) {
    osi3::Lane lane = ground_truth->lane(i);
    // Workaround until intersection type is filled TODO
    if (lane.classification().type() ==
        osi3::Lane_Classification_Type_TYPE_INTERSECTION)
      intersectionLanes.push_back(lane.id().value());
  }

  double ego_psi = ego_base_.orientation().yaw();

  int ti = 0;
  for (auto &mo : *ground_truth->mutable_moving_object()) {
    if (mo.id().value() == ego_id_) continue;

    osi3::BaseMoving moBase = mo.base();
    input.targets[ti].id = ti + 1;
    input.targets[ti].priority = agent_model::TARGET_PRIORITY_NOT_SET;
    input.targets[ti].dsIntersection = 0;

    // determine whether target is assigned to a lane along the host's path
    // (following possible)
    bool onPath = false;
    auto it = lanes_.end();
    int commonLaneIdx = 0;
    for (int j = 0; j < mo.assigned_lane_id_size(); j++) {

      if (std::find(intersectionLanes.begin(), intersectionLanes.end(),
                    mo.assigned_lane_id(j).value()) != intersectionLanes.end())
        input.targets[ti].priority = agent_model::TARGET_ON_INTERSECTION;

      it = find(lanes_.begin(), lanes_.end(), mo.assigned_lane_id(j).value());
      if (it != lanes_.end()) {
        commonLaneIdx = j;
        onPath = true;
        break;
      }
    }

    // only fill ds, lane and d fields when target is along the host's path
    if (onPath) {
      Point2D cPoint;
      Point2D bPoint(moBase.position().x(), moBase.position().y());

      closestCenterlinePoint(bPoint, path_centerline_, cPoint);
      // osi3::Lane *tarLane =
      // findLane(mo.assigned_lane_id(commonLane).value(), ground_truth);
      // Point2D current = bPoint;

      double sTar =
        xy2SSng(ego_centerline_point_, bPoint, path_centerline_, ego_psi);
      /*double sTar = 0;
      if (*it != ego_lane_ptr_->id().value()) {
              // target is not assigned to the current lane -> add
      distance along the target's assigned lane
              // meaning assigned_lane's beginning to bPoint
              std::vector<Point2D> pos;
              getXY(findLane(*it, ground_truth), pos);
              sTar += xy2s(pos.front(), bPoint, pos);
              pos.clear();
      }
      while (it != futureLanes.begin() && *(it - 1) !=
      ego_lane_ptr_->id().value()) { it--; std::vector<Point2D> pos;
      getXY(findLane(*it, ground_truth), pos); sTar += xy2s(pos.front(),
      pos.back(), pos); current = pos.front();
      }
      sTar += xy2s(ego_centerline_point_, current, elPoints);*/

      double sTarNet;
      if (sTar > 0)
        sTarNet = sTar - 0.5 * moBase.dimension().length() -
                  0.5 * ego_base_.dimension().length();
      else
        sTarNet = sTar + 0.5 * moBase.dimension().length() +
                  0.5 * ego_base_.dimension().length();

      input.targets[ti].ds = sTarNet;  // TODO: check what happens when
                                       // target is behind host vehicle
      input.targets[ti].d = sqrt(pow(moBase.position().x() - cPoint.x, 2) +
                                 pow(moBase.position().y() - cPoint.y, 2));
      input.targets[ti].lane =
        lane_mapping_[mo.assigned_lane_id(commonLaneIdx).value()];
    } else {
      input.targets[ti].ds = INFINITY;
      input.targets[ti].d = 0;
      input.targets[ti].lane = 127;

      if (input.targets[ti].priority != agent_model::TARGET_ON_INTERSECTION) {
        // for (int j = 0; j < mo.assigned_lane_id_size(); j++) {
        for (auto &assigned_lane : mo.assigned_lane_id()) {
          JunctionPath tmp;
          bool apprJunc = false;
          for (auto &yieldL : yielding_lanes_) {
            // target is on its way on a yield lane to the junction
            if (find(yieldL.lane_ids.begin(), yieldL.lane_ids.end(),
                     assigned_lane.value()) != yieldL.lane_ids.end()) {
              input.targets[ti].priority = agent_model::TARGET_ON_GIVE_WAY_LANE;
              tmp.lane_ids = yieldL.lane_ids;
              tmp.pts = yieldL.pts;
              apprJunc = true;
            }
          }
          if (!apprJunc) {
            for (auto &prioL : priority_lanes_) {
              // target is on its way on a priority lane to the
              // junction
              if (find(prioL.lane_ids.begin(), prioL.lane_ids.end(),
                       assigned_lane.value()) != prioL.lane_ids.end()) {
                input.targets[ti].priority =
                  agent_model::TARGET_ON_PRIORITY_LANE;
                tmp.lane_ids = prioL.lane_ids;
                tmp.pts = prioL.pts;
                apprJunc = true;
              }
            }
          }
          if (apprJunc)
            input.targets[ti].dsIntersection =
              xy2s(Point2D(moBase.position().x(), moBase.position().y()),
                   tmp.pts.back(), tmp.pts);

          /*if (find(intersectionLanes.begin(),
          intersectionLanes.end(), mo.assigned_lane_id(j).value())
                  != intersectionLanes.end()) {
                  //target is on a lane ending in an intersection
                  std::vector<Point2D> cl;
                  getXY(findLane(mo.assigned_lane_id(j).value(),
          ground_truth), cl); input.targets[ti].dsIntersection =
          xy2s(Point2D(base.position().x(), base.position().y()),
          cl.back(), cl); if (find(priority_lanes_.begin(),
          priority_lanes_.end(), mo.assigned_lane_id(j).value())
                          != priority_lanes_.end())
                          input.targets[ti].priority =
          agent_model::TARGET_ON_PRIORITY_LANE; else if
          (find(yielding_lanes_.begin(), yielding_lanes_.end(),
          mo.assigned_lane_id(j).value())
                          != yielding_lanes_.end())
                          input.targets[ti].priority =
          agent_model::TARGET_ON_GIVE_WAY_LANE;

                  break;
          }
          else if (findLane(mo.assigned_lane_id(j).value(),
          ground_truth)->classification().type() ==
          osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
                  input.targets[ti].priority =
          agent_model::TARGET_ON_INTERSECTION;
          }*/
        }
      }
    }

    input.targets[ti].xy.x = moBase.position().x() - last_position_.x;
    input.targets[ti].xy.y = moBase.position().y() - last_position_.y;

    input.targets[ti].v = sqrt(moBase.velocity().x() * moBase.velocity().x() +
                               moBase.velocity().y() * moBase.velocity().y());
    input.targets[ti].a =
      sqrt(moBase.acceleration().x() * moBase.acceleration().x() +
           moBase.acceleration().y() * moBase.acceleration().y());
    input.targets[ti].psi = moBase.orientation().yaw() - ego_psi;

    input.targets[ti].size.length = moBase.dimension().length();
    input.targets[ti].size.width = moBase.dimension().width();

    ti++;
  }
  for (int i = ti; i < agent_model::NOT; i++) {
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
  /*
  for (int i = 0; i < agent_model::NOT; i++)
  {
          if (i < ground_truth->moving_object_size())
          {
                  osi3::MovingObject egoObj = ground_truth->moving_object(i);

                  if (egoObj.id().value() == ego_id_)
                  {
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
                          continue;
                  }

                  input.targets[i].priority =
  agent_model::TARGET_PRIORITY_NOT_SET; osi3::BaseMoving base = egoObj.base();

                  input.targets[i].id = i+1;

                  auto it = futureLanes.end();
                  bool assigned = false;

                  double sTar = 0;

                  // determine whether target is assigned to a lane along the
  host's path (following possible) for (int j = 0; j <
  egoObj.assigned_lane_id_size(); j++) {

                          it = find(futureLanes.begin(), futureLanes.end(),
  egoObj.assigned_lane_id(j).value());

                          if (it != futureLanes.end()) {
                                  assigned = true;
                                  break;
                          }
                  }

                  for (int j = 0; j < egoObj.assigned_lane_id_size(); j++) {
                          if (find(lanesEnteringIntersection.begin(),
  lanesEnteringIntersection.end(), egoObj.assigned_lane_id(j).value())
                                  != lanesEnteringIntersection.end()) {
                                  //target is on a lane ending in an
  intersection

                                  std::vector<Point2D> cl;
                                  getXY(findLane(egoObj.assigned_lane_id(j).value(),
  ground_truth), cl); input.targets[i].dsIntersection =
  xy2s(Point2D(base.position().x(), base.position().y()), cl.back(), cl); if
  (find(priority_lanes_.begin(), priority_lanes_.end(),
  egoObj.assigned_lane_id(j).value())
                                          != priority_lanes_.end())
                                          input.targets[i].priority =
  agent_model::TARGET_ON_PRIORITY_LANE; else if (find(yielding_lanes_.begin(),
  yielding_lanes_.end(), egoObj.assigned_lane_id(j).value())
                                          != yielding_lanes_.end())
                                          input.targets[i].priority =
  agent_model::TARGET_ON_GIVE_WAY_LANE;

                                  break;
                          }
                          else if
  (findLane(egoObj.assigned_lane_id(j).value(),
  ground_truth)->classification().type() ==
  osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
  input.targets[i].priority = agent_model::TARGET_ON_INTERSECTION;
                          }
                  }

                  // only fill ds and d fields when target is along the host's
  path if (assigned) { Point2D cPoint; Point2D bPoint(base.position().x(),
  base.position().y());

                          closestCenterlinePoint(bPoint, elPoints, cPoint);

                          osi3::Lane* tarLane =
  findLane(egoObj.assigned_lane_id(0).value(), ground_truth);

                          Point2D current = bPoint;

                          if (*it != ego_lane_ptr_->id().value()) {
                                  // target is not assigned to the current
  lane
  -> add distance along the target's assigned lane
                                  // meaning assigned_lane's beginning to
  bPoint std::vector<Point2D> pos; getXY(findLane(*it, ground_truth), pos);
                                  sTar += xy2s(pos.front(), bPoint, pos);
                                  pos.clear();
                          }
                          while (it != futureLanes.begin() && *(it - 1) !=
  ego_lane_ptr_->id().value()) { it--; std::vector<Point2D> pos;
                                  getXY(findLane(*it, ground_truth), pos);
                                  sTar += xy2s(pos.front(), pos.back(), pos);
                                  current = pos.front();
                          }

                          sTar += xy2s(ego_centerline_point_, current,
  elPoints);

                          input.targets[i].ds = sTar; //TODO: check what
  happens when target is behind host vehicle input.targets[i].d =
  sqrt(pow(base.position().x() - cPoint.x, 2) + pow(base.position().y() -
  cPoint.y, 2));
                  }
                  else {
                          input.targets[i].ds = 0;
                          input.targets[i].d = 0;
                  }

                  input.targets[i].xy.x = base.position().x() -
  last_position_.x; input.targets[i].xy.y = base.position().y() -
  last_position_.y;

                  input.targets[i].v = sqrt(base.velocity().x() *
  base.velocity().x() + base.velocity().y() * base.velocity().y());
                  input.targets[i].a = sqrt(base.acceleration().x() *
  base.acceleration().x() + base.acceleration().y() *
  base.acceleration().y()); input.targets[i].psi = base.orientation().yaw() -
  ego_psi;

                  input.targets[i].size.length = base.dimension().length();
                  input.targets[i].size.width = base.dimension().width();


                  input.targets[i].lane =
  lane_mapping_[egoObj.assigned_lane_id(0).value()];

          }
          else
          {
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
          }
  }*/
}

void OsiConverter::fillHorizon(osi3::SensorView &sensor_view,
                               agent_model::Input &input) {
  double sMax = std::max(15.0, 15.0 * input.vehicle.v);
  double ego_psi = ego_base_.orientation().yaw();

  // double sMax = 100;

  // distance (along centerline) to each horizon point from current location
  std::vector<double> ds(agent_model::NOH, 0);

  double delta = sqrt(sMax) / double(agent_model::NOH);  // linear spaces
  for (int i = 0; i < agent_model::NOH; i++) {  // create squared spaces
    ds[i] = (i + 1) * (i + 1) * delta * delta;
  }

  double interpolation = 0;

  // NEW	//
  Point2D dummy;
  // std::cout << "look here: " << last_position_.x << ", " <<
  // last_position_.y << " cl lenght: " << path_centerline_.size() << "\n";
  int hor_idx = closestCenterlinePoint(last_position_, path_centerline_, dummy);
  double sTravel = path_s_[hor_idx - 1] +
                   sqrt((last_position_.x - path_centerline_[hor_idx - 1].x) *
                          (last_position_.x - path_centerline_[hor_idx - 1].x) +
                        (last_position_.y - path_centerline_[hor_idx - 1].y) *
                          (last_position_.y - path_centerline_[hor_idx - 1].y));
  // std::cout << "sTravel: " << sTravel << " after idx: " << hor_idx << "\n";

  for (int i = 0; i < agent_model::NOH; i++) {
    double dsCur = (i + 1) * (i + 1) * delta * delta;
    // get correct space for interpolation
    int j = -1;
    for (auto &ss : path_s_) {
      if (ss > sTravel + dsCur) break;
      j++;
    }

    Point2D hKnot;
    if (sTravel + dsCur < path_s_.back()) {
      interpolation =
        (sTravel + dsCur - path_s_[j]) / (path_s_[j + 1] - path_s_[j]);
      hKnot.x =
        path_centerline_[j].x +
        interpolation * (path_centerline_[j + 1].x - path_centerline_[j].x);
      hKnot.y =
        path_centerline_[j].y +
        interpolation * (path_centerline_[j + 1].y - path_centerline_[j].y);

      input.horizon.ds[i] = dsCur;
      input.horizon.psi[i] = path_psi_[j] +
                             interpolation * (path_psi_[j + 1] - path_psi_[j]) -
                             ego_psi;
      input.horizon.kappa[i] =
        path_kappa_[j] + interpolation * (path_kappa_[j + 1] - path_kappa_[j]);
    } else {
      hKnot.x = path_centerline_.back().x;
      hKnot.y = path_centerline_.back().y;
      input.horizon.ds[i] = path_s_.back() - sTravel;
      input.horizon.psi[i] = 0;
      input.horizon.kappa[i] = 0;
    }

    input.horizon.x[i] = std::cos(ego_psi) * (hKnot.x - last_position_.x) +
                         std::sin(ego_psi) * (hKnot.y - last_position_.y);
    input.horizon.y[i] =
      -1.0 * std::sin(ego_psi) * (hKnot.x - last_position_.x) +
      std::cos(ego_psi) * (hKnot.y - last_position_.y);
    input.horizon.egoLaneWidth[i] = 3.75;
    input.horizon.leftLaneWidth[i] = 0;
    input.horizon.rightLaneWidth[i] = 0;
  }
  std::cout << std::endl;

  // NEW(end)	//
  /*
  std::vector<Point2D> horizon;
  std::vector<Point2D> currentCl = elPoints;
  std::vector<double> kappa = k;

  int currentLane = ego_lane_ptr_->id().value();

  int idx = 127;
  bool spline = false;
  int finalLane = *(futureLanes.end() - 1);
  int nextLane = (currentLane != finalLane) ? *(find(futureLanes.begin(),
  futureLanes.end(), currentLane) + 1) : 127; double sStart = 0; double sPast
  = 0;

  Point2D p1;
  Point2D p2;

  idx = closestCenterlinePoint(ego_centerline_point_, currentCl, p2);

  //setup for horizon calculation: in most cases the current position will be
  somewhere within the scope of the centerline of the current lane.
  //If not, use return value of 'closestCenterlinePoint' to determine the
  relative position if (idx == 0) {
          //Host vehicle before scope of the centerline, create linear
  connection s.clear(); psi.clear(); kappa.clear(); if (ds.back() != 0) {
                  currentCl.insert(currentCl.begin(), ego_centerline_point_);
                  xy2Curv(currentCl, s, psi, kappa);
          }
  }
  else if (idx == currentCl.size()) {
          //Host vehicle after scope of the centerline, move on to next lane
          s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
          spline = true;

          if (currentLane != finalLane) {
                  nextLane = *(find(futureLanes.begin(), futureLanes.end(),
  currentLane) + 1); std::vector<Point2D> tempLane; getXY(findLane(nextLane,
  ground_truth), tempLane);

                  //curved path? TODO criterium
                  //spline3(ego_centerline_point_, tempLane.front(),
  Point2D(std::cos(ego_psi), std::sin(ego_psi)),
  Point2D((tempLane[1].x-tempLane[0].x),tempLane[1].y-tempLane[0].y),
  currentCl);

                  //straight path
                  spline1(ego_centerline_point_, tempLane.front(), currentCl);

                  xy2Curv(currentCl, s, psi, kappa);
          }
  }
  else {
          //somewhere within scope of currentCl, set sStart to the distance
  along current lane that should be disregarded sStart = s[idx - 1] +
  sqrt((last_position_.x - elPoints[idx - 1].x) * (last_position_.x -
  elPoints[idx - 1].x) + (last_position_.y - elPoints[idx - 1].y) *
  (last_position_.y - elPoints[idx - 1].y));
  }

  [&] {
          for (int i = 0; i < agent_model::NOH; i++) {
                  //determine correct lane. If s is already of the correct
  lane for this horizon point, the loop is not executed while ((s.back() +
  sPast - sStart) < ds[i]) //s.back: s at the end of current lane. sPast: s
  along the lanes_ on the path already considered, sStart: s at starting point
                  {
                          if (currentLane == finalLane) return;

                          sPast += s.back();
                          s.clear(); psi.clear(); kappa.clear();
  currentCl.clear();

                          currentLane = nextLane;
                          nextLane = *(find(futureLanes.begin(),
  futureLanes.end(), currentLane) + 1); getXY(findLane(currentLane,
  ground_truth), currentCl); xy2Curv(currentCl, s, psi, kappa);

                  }

                  //s, currentCl should now contain correct lane

                  //setup for width calculation
                  osi3::Lane* lane = findLane(currentLane, ground_truth);
                  if (lane == nullptr)std::cout << "ERROR lane not found!" <<
  std::endl;

                  //actual horizon
                  for (int k = 1; k < s.size(); k++) {

                          if ((s[k - 1] + sPast - sStart < ds[i]) && (s[k] +
  sPast - sStart >= ds[i])) {

                                  if (s[k - 1] == s[k]) s[k] += 0.3;

                                  interpolation = (ds[i] - (s[k - 1] + sPast -
  sStart)) / (s[k] - s[k - 1]);

                                  Point2D hKnot(currentCl[k - 1].x +
  interpolation * (currentCl[k].x - currentCl[k - 1].x), currentCl[k - 1].y +
  interpolation * (currentCl[k].y - currentCl[k - 1].y));

                                  horizon.push_back(hKnot);

                                  input.horizon.x[i] = std::cos(ego_psi) *
  (horizon.back().x - last_position_.x) + std::sin(ego_psi) *
  (horizon.back().y - last_position_.y); input.horizon.y[i] = -1.0 *
  std::sin(ego_psi) * (horizon.back().x - last_position_.x) +
  std::cos(ego_psi) * (horizon.back().y - last_position_.y);
  input.horizon.ds[i] = ds[i];

                                  input.horizon.psi[i] = psi[k - 1] +
  interpolation * (psi[k] - psi[k - 1]) - ego_psi;


                                  input.horizon.kappa[i] = kappa[k - 1] +
  interpolation * (kappa[k] - kappa[k - 1]);

                                  //width calculation

                                  input.horizon.egoLaneWidth[i] =
  calcWidth(hKnot, lane, ground_truth);

                                  if (input.horizon.egoLaneWidth[i] == 0) {
                                          //lane has no boundaries. Take last
  width value if available or default value if (i > 0)
                                                  input.horizon.egoLaneWidth[i]
  = input.horizon.egoLaneWidth[i - 1]; else input.horizon.egoLaneWidth[i] =
  defaultWidth;
                                  }
                                  //left and right lanes_
                                  osi3::Lane* lLane =
  lane->classification().left_adjacent_lane_id_size() > 0 ?
                                          findLane(lane->classification().left_adjacent_lane_id(0).value(),
  ground_truth) : nullptr; osi3::Lane* rLane =
  lane->classification().right_adjacent_lane_id_size() > 0 ?
                                          findLane(lane->classification().right_adjacent_lane_id(0).value(),
  ground_truth) : nullptr;

                                  input.horizon.rightLaneWidth[i] = rLane !=
  nullptr ? calcWidth(hKnot, rLane, ground_truth) : defaultWidth; if
  (input.horizon.rightLaneWidth[i] == 0) { if (i > 0)
                                                  input.horizon.rightLaneWidth[i]
  = input.horizon.rightLaneWidth[i - 1]; else input.horizon.rightLaneWidth[i]
  = defaultWidth;
                                  }

                                  input.horizon.leftLaneWidth[i] = lLane !=
  nullptr ? calcWidth(hKnot, lLane, ground_truth) : defaultWidth; if
  (input.horizon.leftLaneWidth[i] == 0) { if (i > 0)
                                                  input.horizon.leftLaneWidth[i]
  = input.horizon.leftLaneWidth[i - 1]; else input.horizon.leftLaneWidth[i] =
  defaultWidth;
                                  }
                          }
                  }
          }
  }();

*/
  // set helpers because currentCl is overwritten when lane changes
  // p1.x = currentCl.back().x; p1.y = currentCl.back().y;
  // p2.x = currentCl[currentCl.size() - 2].x; p2.y =
  // currentCl[currentCl.size()
  // - 2].y;

  //[&] {
  //	for (int i = 0; i < agent_model::NOH; i++) {

  //		//determine correct lane
  //		while ((s.back() + sPast - sStart) < ds[i])
  //		{
  //			sPast += s.back();

  //			if (!spline) {	//currently on a regular lane. Switching
  // to spline and setting nextLane

  //				if (currentLane == finalLane)return;

  //				nextLane = *(find(futureLanes.begin(),
  // futureLanes.end(), currentLane) + 1);
  // s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
  //				std::vector<Point2D> tempLane;
  //				getXY(findLane(nextLane, ground_truth),
  // tempLane);

  //				//curved connection? TODO criterium
  //				/*spline3(p1, tempLane.front(),
  // Point2D(p1.x-p2.x, p1.y
  //- p2.y), 					Point2D((tempLane[1].x -
  // tempLane[0].x), tempLane[1].y - tempLane[0].y), currentCl);*/

  //					//straight connection?
  //				spline1(p1, tempLane.front(), currentCl);
  //				xy2Curv(currentCl, s, psi, kappa);
  //				spline = true;
  //			}
  //			else {			//currently on a spline.
  // Switching to lane determined by nextLane

  //				s.clear(); psi.clear(); kappa.clear();
  // currentCl.clear(); 				currentLane = nextLane;
  // nextLane = 127; 				getXY(findLane(currentLane,
  // ground_truth), currentCl); xy2Curv(currentCl, s, psi, kappa);

  //				p1.x = currentCl.back().x; p1.y =
  // currentCl.back().y; 				p2.x =
  // currentCl[currentCl.size()
  // - 2].x; p2.y = currentCl[currentCl.size() - 2].y;

  //				spline = false;
  //			}
  //		}

  //		//s, currentCl should now contain correct lane
  //		bool stop = false;

  //		for (int k = 1; k < s.size() && !stop; k++) {

  //			if ((s[k - 1] + sPast - sStart < ds[i]) && (s[k] + sPast
  //- sStart >= ds[i])) {
  //
  //				if (s[k - 1] == s[k]) s[k] += 0.3;

  //				interpolation = (ds[i] - (s[k - 1] + sPast -
  // sStart))
  /// (s[k] - s[k - 1]);

  //				Point2D hKnot(currentCl[k - 1].x + interpolation
  //* (currentCl[k].x - currentCl[k - 1].x),
  // currentCl[k - 1].y + interpolation * (currentCl[k].y - currentCl[k -
  // 1].y));

  //				horizon.push_back(hKnot);

  //				input.horizon.x[i] =  std::cos(ego_psi) *
  //(horizon.back().x
  //- last_position_.x)+ std::sin(ego_psi) * (horizon.back().y -
  // last_position_.y);
  ////pre.x * c + pre.y * s; ego_psi input.horizon.y[i] =
  ///-1.0*std::sin(ego_psi) *
  //(horizon.back().x - last_position_.x) + std::cos(ego_psi) *
  //(horizon.back().y - last_position_.y); //pre.x * (-1 * s) + pre.y * c;
  // input.horizon.ds[i] = ds[i];

  //				input.horizon.psi[i] = psi[k - 1] +
  // interpolation
  //* (psi[k]
  //- psi[k - 1]) - ego_psi; //heading of road at horizon point relative to
  // heading of host vehicle.
  //					//Alternatively: angle between heading
  // of host compared to heading needed to reach horizon point meant here?
  //
  //				//if(abs(kappa[k - 1] + interpolation *
  //(kappa[k]
  //- kappa[k
  //- 1])) < 2) 					input.horizon.kappa[i] =
  // kappa[k
  //- 1]
  //+ interpolation * (kappa[k] - kappa[k - 1]);
  //				//else
  //					//input.horizon.kappa[i] = 0;

  //			}
  //		}
  //	}

  //}();
  /*

  if (horizon.size() == 0) {

          horizon.push_back(Point2D(last_position_.x, last_position_.y));
          input.horizon.x[0] = horizon.back().x - last_position_.x;
          input.horizon.y[0] = horizon.back().y - last_position_.y;
          input.horizon.ds[0] = ds.back();
          input.horizon.psi[0] = 0;
          input.horizon.kappa[0] = 0;
  }
  for (int i = horizon.size(); i < agent_model::NOH; i++) {
          input.horizon.x[i] =  std::cos(ego_psi) * (horizon.back().x -
  last_position_.x)+ std::sin(ego_psi) * (horizon.back().y -
  last_position_.y);
  //pre.x * c + pre.y * s; ego_psi input.horizon.y[i] = -1.0*std::sin(ego_psi)
  * (horizon.back().x - last_position_.x) + std::cos(ego_psi) *
  (horizon.back().y - last_position_.y); //pre.x * (-1 * s) + pre.y * c;
  input.horizon.ds[i] = ds.back(); input.horizon.psi[i] = input.horizon.psi[i
  - 1]; input.horizon.kappa[i] = 0;

          Point2D knot(horizon.back().x, horizon.back().y);
          horizon.push_back(knot);
          input.horizon.egoLaneWidth[i] = defaultWidth;
  }
  */
}

void OsiConverter::fillLanes(osi3::SensorView &sensor_view,
                             agent_model::Input &input) {
  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  int laneCounter = 0;
  double defaultWidth = 3.5;

  // lanes_ along the host's path constitute one lane
  osi3::Lane lane = *ego_lane_ptr_;
  input.lanes[laneCounter].id = lane_mapping_[lanes_[laneCounter]];

  input.lanes[laneCounter].access =
    agent_model::Accessibility::ACC_ACCESSIBLE;  // lanes_ along route are
                                                 // accessible by definition
  input.lanes[laneCounter].width = input.horizon.egoLaneWidth[0];

  input.lanes[laneCounter].dir = agent_model::DrivingDirection::DD_FORWARDS;

  std::vector<Point2D> lPoints;
  getXY(&lane, lPoints);

  auto it = lanes_.end();
  double dist = 0;
  Point2D current = lPoints.back();

  // iterate backwards over futureLanes and add distance along all lanes_ in
  // front of host
  while (it != lanes_.begin() && *(it - 1) != ego_lane_ptr_->id().value()) {
    it--;
    std::vector<Point2D> pos;
    getXY(findLane(*it, ground_truth), pos);
    dist += xy2s(pos.front(), pos.back(), pos);
    current = pos.front();
  }

  input.lanes[laneCounter].closed =
    dist + xy2s(ego_centerline_point_, current, lPoints);
  input.lanes[laneCounter].route = input.lanes[laneCounter].closed;

  laneCounter++;

  for (int i = 0; i < ground_truth->lane_size() && i < agent_model::NOL; i++) {
    osi3::Lane lane = ground_truth->lane(i);
    // skip if lane is in futureLanes, that means it was already considered
    // before
    if (find(lanes_.begin(), lanes_.end(), lane.id().value()) != lanes_.end()) {
      continue;
    }

    input.lanes[i].id = lane_mapping_[lane.id().value()];

    // calculate type
    if (lane.classification().type() ==
        osi3::Lane_Classification_Type_TYPE_DRIVING) {
      input.lanes[i].access = agent_model::Accessibility::ACC_ACCESSIBLE;
    } else if (lane.classification().type() ==
               osi3::Lane_Classification_Type_TYPE_NONDRIVING) {
      input.lanes[i].access = agent_model::Accessibility::ACC_NOT_ACCESSIBLE;
    }

    // calculate driving direction
    if (lane.classification().centerline_is_driving_direction() ==
        ego_lane_ptr_->classification().centerline_is_driving_direction())
      input.lanes[i].dir = agent_model::DrivingDirection::DD_FORWARDS;
    else
      input.lanes[i].dir = agent_model::DrivingDirection::DD_BACKWARDS;

    // save average of width
    input.lanes[i].width = defaultWidth;

    std::vector<Point2D> lPoints;

    getXY(&lane, lPoints);

    input.lanes[i].closed = xy2s(lPoints.front(), lPoints.back(), lPoints);
    input.lanes[i].route = input.lanes[i].closed;  // TODO

    laneCounter++;
  }
  while (laneCounter < agent_model::NOL) {
    input.lanes[laneCounter].id = 127;
    input.lanes[laneCounter].width = 0;
    input.lanes[laneCounter].route = 0;
    input.lanes[laneCounter].closed = 0;
    input.lanes[laneCounter].access = agent_model::ACC_NOT_SET;
    input.lanes[laneCounter].dir = agent_model::DD_NOT_SET;
    laneCounter++;
  }
}
