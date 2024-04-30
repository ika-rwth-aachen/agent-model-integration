#pragma once

#ifndef __UTILS_CHECKER_H__
#define __UTILS_CHECKER_H__

#include <iostream>

#endif // __UTILS_CHECKER_H__ 


void checkObjects(osi3::SensorView &sensor_view){

  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // ego_id_ retrievable from sensor view check
  uint64_t ego_id_ = sensor_view.host_vehicle_id().value();

  if (typeid(ego_id_) != typeid(uint64_t)){
    SPDLOG_ERROR("ego_id_ from sensor_view.host_vehicle_id() is not of type uint64_t");
    exit(EXIT_FAILURE);
  }
  if (ego_id_ < 0){
    SPDLOG_ERROR("ego_id_ from sensor_view.host_vehicle_id() is < 0");
    exit(EXIT_FAILURE);
  }

  // moving objects have valid and existing ids check
  for (auto &moving_object : *ground_truth->mutable_moving_object()){
    for (auto &assigned_lane : moving_object.moving_object_classification().assigned_lane_id()){
      uint64_t lane_id = assigned_lane.value();
      findLane(lane_id, ground_truth);
    }
  }
}

void checkLanes(osi3::SensorView &sensor_view){

  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();

  // retrieve ego_id_ from sensor view check
  uint64_t ego_id_ = sensor_view.host_vehicle_id().value();

  // iterate over all lanes
  for (int i = 0; i < ground_truth->lane_size(); i++){
    uint64_t lane_id = ground_truth->lane(i).id().value();
    auto* lane = findLane(lane_id, ground_truth);

    // lane pairings have valid and existing ids check
    for (auto &l_pairs : lane->classification().lane_pairing()){

      // antecessor and successor lane check
      if (l_pairs.has_antecessor_lane_id() && l_pairs.antecessor_lane_id().value() != uint64_t INFINITY){
        findLane(l_pairs.antecessor_lane_id().value(), ground_truth);
      }
      if(l_pairs.has_successor_lane_id() && l_pairs.successor_lane_id().value() != uint64_t INFINITY){
        findLane(l_pairs.successor_lane_id().value(), ground_truth);
      }
    }
    
    // left adjacent lane check
    for (int j = 0; j < lane->classification().left_adjacent_lane_id_size(); j++) 
    {
      uint64_t adj_id = lane->classification().left_adjacent_lane_id(j).value();
      findLane(adj_id, ground_truth);
    }

    // right adjacent lane check
    for (int j = 0; j < lane->classification().right_adjacent_lane_id_size(); j++) 
    {
      uint64_t adj_id = lane->classification().right_adjacent_lane_id(j).value();
      findLane(adj_id, ground_truth);
    }
        
    // left adjacent lane boundary check
    for (int j = 0; j < lane->classification().left_lane_boundary_id_size(); j++) 
    {
      uint64_t adj_boundary_id = lane->classification().left_lane_boundary_id(j).value();
      findLaneBoundary(adj_boundary_id, ground_truth);
    }

    // right adjacent lane boundary check
    for (int j = 0; j < lane->classification().right_lane_boundary_id_size(); j++) 
    {
      uint64_t adj_boundary_id = lane->classification().right_lane_boundary_id(j).value();
      findLaneBoundary(adj_boundary_id, ground_truth);
    }

    std::vector<Point2D> centerline_points;
    // valid centerlines in lanes for all lanes except intersections check
    if (lane->classification().type() != osi3::Lane_Classification_Type_TYPE_INTERSECTION){
      if (lane->classification().centerline_is_driving_direction()){
        for (int i = 0; i < lane->classification().centerline_size(); i++) {
          Point2D centerline_point (lane->classification().centerline(i).x(),
                  lane->classification().centerline(i).y());
          centerline_points.push_back(centerline_point);
        }
      }
      else{
        for (int i = lane->classification().centerline_size() - 1; i >= 0; i--) {
          Point2D centerline_point(lane->classification().centerline(i).x(),
                    lane->classification().centerline(i).y());
          centerline_points.push_back(centerline_point);
        }
      }
      if(centerline_points.size() < 2){
        SPDLOG_ERROR("centerline size of lane id({}) is < 2", i);
        exit(EXIT_FAILURE);
      }
    }
  }
}


void checkTrafficSignals(osi3::SensorView &sensor_view){

  osi3::GroundTruth *ground_truth = sensor_view.mutable_global_ground_truth();
  
  // traffic lights have valid and existing ids check
  for (auto &light : *ground_truth->mutable_traffic_light()){
    for (auto &assigned_lane : light.classification().assigned_lane_id()) {
      uint64_t lane_id = assigned_lane.value();
      findLane(lane_id, ground_truth);
    }
  }

  // traffic signs have valid and existing ids check
  for (auto &sign : *ground_truth->mutable_traffic_sign()){
    for (auto &assigned_lane : sign.main_sign().classification().assigned_lane_id()) {
      uint64_t lane_id = assigned_lane.value();
      findLane(lane_id, ground_truth);
    }
  }
}
