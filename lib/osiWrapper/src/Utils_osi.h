#pragma once

#ifndef __UTILS_OSI_H__
#define __UTILS_OSI_H__

#endif // __UTILS_OSI_H__ 


/**
 * @brief get vector of xy centerline-coordinates of a lane
 *
 * @param lane pointer
 * @param pos result vector
 * @return success
 */
int getXY(osi3::Lane* l, std::vector<Point2D>& pos) {

  if (l->classification().centerline_is_driving_direction()) {
    for (int i = 0; i < l->classification().centerline_size(); i++) {
      Point2D point(l->classification().centerline(i).x(),
                    l->classification().centerline(i).y());
      pos.push_back(point);
    }
    return 0;
  } else {
    for (int i = l->classification().centerline_size() - 1; i >= 0; i--) {
      Point2D point(l->classification().centerline(i).x(),
                    l->classification().centerline(i).y());
      pos.push_back(point);
    }
    return 0;
  }
}


/**
 * @brief get vector of xy centerline-coordinates of a lane boundary
 *
 * @param l pointer to lane boundary
 * @param pos result vector
 * @return success
 */
int getXY(osi3::LaneBoundary* l, std::vector<Point2D>& pos) {

  for (int i = 0; i < l->boundary_line_size(); i++) {
    Point2D point(l->boundary_line(i).position().x(),
                  l->boundary_line(i).position().y());
    pos.push_back(point);
  }
  return 0;
}


/**
 * @brief find a laneID in ground_truth.lane and return pointer
 *
 * @param ID
 * @param ground_truth
 */
osi3::Lane* findLane(uint64_t id, osi3::GroundTruth* ground_truth) {

  for (int i = 0; i < ground_truth->lane_size(); i++) {
    if (ground_truth->lane(i).id().value() == id){
      return (ground_truth->mutable_lane(i));
    }
  }
  SPDLOG_ERROR("lane with id ({}) could not be found in ground truth", id);
  exit(EXIT_FAILURE);
  return nullptr;
}


/**
 * @brief find a laneID in ground_truth.lane and return its index
 *
 * @param ground_truth
 * @param ID
 */
int findLaneIdx(osi3::GroundTruth* ground_truth, uint64_t id) {

  for (int i = 0; i < ground_truth->lane_size(); i++) {
    if (ground_truth->lane(i).id().value() == id) return i;
  }
  SPDLOG_ERROR("lane idx of id ({}) could not be found in ground truth", id);
  exit(EXIT_FAILURE);
  return -1;
}


/**
 * @brief find a laneboundaryID in ground_truth.lane_boundary and return pointer
 *
 * @param ID
 * @param ground_truth
 * @return osi3::LaneBoundary* pointer to desired lane boundary
 */
osi3::LaneBoundary* findLaneBoundary(uint64_t id, osi3::GroundTruth* ground_truth) {

  for (int i = 0; i < ground_truth->lane_boundary_size(); i++) {
    if (ground_truth->lane_boundary(i).id().value() == id)
      return (ground_truth->mutable_lane_boundary(i));
  }
  SPDLOG_ERROR("lane boundary with id ({}) could not be found in ground truth", id);
  exit(EXIT_FAILURE);
  return nullptr;
}


/**
 * @brief find a lane group by lane group id
 * 
 * @param lane_groups vector of all lane groups
 * @param id 
 * @return LaneGroup desired lane group
 */
LaneGroup findLaneGroup(std::vector<LaneGroup> lane_groups, int id) {

  LaneGroup tmp;

  for (int i = 0; i < lane_groups.size(); i++) {
    if (lane_groups[i].id == id) return lane_groups[i];
  }
  SPDLOG_ERROR("lane group with id ({}) could not be found", id);
  return tmp;
}


/**
 * @brief Get the adjacent lanes object
 * 
 * @param ground_truth 
 * @param lane_idx 
 * @param mode mode determines if searching for:
 *    straight adjacent lanes (S)
 *    left adjacent lanes (L)
 *    right adjacent lanes (R)
 * @return std::vector<int> 
 */
std::vector<int> findAdjacentLanes(osi3::GroundTruth* ground_truth, int lane_idx, std::string mode) {

  std::vector<int> lanes; 

  osi3::Lane lane = ground_truth->lane(lane_idx);
  
  // get driving direction
  bool is_driving_direction = lane.classification().centerline_is_driving_direction() || (lane.classification().centerline_size() == 0 && lane.classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION);
  
  for(int k = 0; k < mode.length(); k++)
  {  

    if (mode.at(k) == 'S')
    {
      // iterate over all lane pairings
      for (int j = 0; j < lane.classification().lane_pairing_size(); j++) 
      {
        // get adjacent_id
        uint64_t adjacent_id;
        if (is_driving_direction) 
        {
          adjacent_id = lane.classification().lane_pairing(j).successor_lane_id().value();
        } else 
        {
          adjacent_id = lane.classification().lane_pairing(j).antecessor_lane_id().value();
        }

        if (adjacent_id == -1) continue;

        // skip if wrong type
        if (findLane(adjacent_id, ground_truth)->classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING && findLane(adjacent_id, ground_truth)->classification().type() != osi3::Lane_Classification_Type_TYPE_INTERSECTION) continue;

        if (typeid(adjacent_id) != typeid(uint64_t)){
          SPDLOG_ERROR("adjacent_id of straight adjacent lane pairing {} is not of type unit64_t", j);
          exit(EXIT_FAILURE);
        }
        if (adjacent_id < 0){
          SPDLOG_ERROR("adjacent_id of straight adjacent lane pairing {} is < 0", j);
          exit(EXIT_FAILURE);
        }

        lanes.push_back(findLaneIdx(ground_truth, adjacent_id));
      }
    }

    if (mode.at(k) == 'L' || mode.at(k) == 'R')
    {
        // skip if wrong type
      if (lane.classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

      // check for left neighbouring lanes
      if ((mode.at(k) == 'L' && is_driving_direction) || (mode.at(k) == 'R' && !is_driving_direction))
      {
        for (int i = 0; i < lane.classification().left_adjacent_lane_id_size() > 0; i++)
        {
          uint64_t adjacent_id = lane.classification().left_adjacent_lane_id(i).value();

          // skip if wrong type
          if (findLane(adjacent_id, ground_truth)->classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

          // skip if opposite driving direction (assume same reference line)
          if (findLane(adjacent_id, ground_truth)->classification().centerline_is_driving_direction() != is_driving_direction) continue;
          
          if (typeid(adjacent_id) != typeid(uint64_t)){
            SPDLOG_ERROR("adjacent_id of left adjacent lane pairing {} is not of type unit64_t", i);
            exit(EXIT_FAILURE);
          }
          if (adjacent_id < 0){
            SPDLOG_ERROR("adjacent_id of left adjacent lane pairing {} is < 0", i);
            exit(EXIT_FAILURE);
          }

          lanes.push_back(findLaneIdx(ground_truth, adjacent_id));
        }
      }

      // check for right neighbouring lanes
      if ((mode.at(k) == 'R' && is_driving_direction) || (mode.at(k) == 'L' && !is_driving_direction))
      {
        for (int i = 0; i < lane.classification().right_adjacent_lane_id_size() > 0; i++)
        {
          uint64_t adjacent_id = lane.classification().right_adjacent_lane_id(i).value();

          // skip if wrong type
          if (findLane(adjacent_id, ground_truth)->classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

          // skip if opposite driving direction (assume same reference line)
          if (findLane(adjacent_id, ground_truth)->classification().centerline_is_driving_direction() != is_driving_direction) continue;
          
          if (typeid(adjacent_id) != typeid(uint64_t)){
            SPDLOG_ERROR("adjacent_id of right adjacent lane pairing {} is not of type unit64_t", i);
          }
          if (adjacent_id < 0){
            SPDLOG_ERROR("adjacent_id of right adjacent lane pairing {} is < 0", i);
          }

          lanes.push_back(findLaneIdx(ground_truth, adjacent_id));
        }
      }
    }
  }
  
  return lanes;
}


/**
 * @brief Check if a signal is assigned to a lane along the path.
 *
 * @param cls[in] either the classification of a traffic sign or a traffic light
 * @param signal_lanes[in] either the classification of a traffic sign or a traffic light
 * @param lanes either[in] the classification of a traffic sign or a traffic light
 * @param assigned_lane_id [out] the lane id which the signal is assinged to
 * @return Boolean value that states if the signal is assinged to lane on the path
 */
template<class T>
bool isSigAssigned(T &cls, std::vector<uint64_t> &signal_lanes, std::vector<uint64_t> &lanes, uint64_t &assigned_lane_id) {

  bool assigned = false;

    // iterate over all assigned lanes
    for (int j = 0; j < cls.assigned_lane_id_size(); j++) {

      // add lane to signal_lanes
      if (find(signal_lanes.begin(), signal_lanes.end(),
               cls.assigned_lane_id(j).value()) == signal_lanes.end()) {
        signal_lanes.push_back(cls.assigned_lane_id(j).value());
      }

      // check if lane on route
      auto signal_lane = find(lanes.begin(), lanes.end(), cls.assigned_lane_id(j).value());
      if (signal_lane != lanes.end()) {
        assigned = true;
        assigned_lane_id = *signal_lane;
        break;
      }
    }

    return assigned;
}