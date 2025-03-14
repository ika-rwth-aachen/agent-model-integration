#pragma once

#ifndef __UTILS_ROUTE_H__
#define __UTILS_ROUTE_H__

#include <iostream>

#endif // __UTILS_ROUTE_H__ 



/**
 * @brief determines lane closest to a point according to cartesian distance
 *
 * caution: some lanes may share points causing the result to be unreliable.
 * Only use when necessary.
 *
 * @param ground_truth
 * @param point
 * @return id of lane
 */
uint64_t closestLane(osi3::GroundTruth* ground_truth, const Point2D point) {

  std::vector<Point2D> centerline;
  double distance = INFINITY;
  double distance_valid_idx = INFINITY;

  // lanes with valid idx have only priority if closer `max_distance_valid_idx`
  double max_distance_valid_idx = 5;        

  uint64_t dest_id = -1;
  uint64_t dest_id_valid_idx = -1;

  for (int i = 0; i < ground_truth->lane_size(); i++) {
    centerline.clear();
    osi3::Lane cur_lane;
    cur_lane.CopyFrom(ground_truth->lane(i));

    // skip intersections without centerlines
    if (cur_lane.classification().centerline_size() == 0 && cur_lane.classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION)
      continue;
      
    getXY(&cur_lane, centerline);

    Point2D closest;
    int idx = closestCenterlinePoint(point, centerline, closest, false);

    // compute distance 
    double d = (closest.x - point.x) * (closest.x - point.x) +
               (closest.y - point.y) * (closest.y - point.y);

    if (idx > 0 && idx < centerline.size() && d < distance_valid_idx && d < max_distance_valid_idx) {
        distance_valid_idx = d;
        dest_id_valid_idx = cur_lane.id().value();
    }

    if (d < distance) {
      distance = d;
      dest_id = cur_lane.id().value();
    }
  }

  if (dest_id_valid_idx != -1) {
    return dest_id_valid_idx;
  }
  else {
    return dest_id;
  }
}


/**
 * @brief determines junction path starting from lane with id lane_id
 *
 *
 * @param ground_truth  OSI ground truth 
 * @param lane_id       index of starting lane
 * @param junction_path result
 */
JunctionPath calcJunctionPath(osi3::GroundTruth* ground_truth, uint64_t lane_id)
{

  // create junction_path
  JunctionPath junction_path;
  junction_path.lanes.push_back(lane_id);

  // create path until no next lane is found
  bool found_next_lane = true;
  while (found_next_lane) {
    auto *lane = findLane(lane_id, ground_truth);
    found_next_lane = false;

    // iterate over all lane pairings
    for (auto &l_pairs : lane->classification().lane_pairing()) 
    {
      uint64_t next_id, from_id;

      // set next_id of lane in front of signal as well as from_id
      if (lane->classification().centerline_is_driving_direction()) {
        // only continue when antecessor exists
        if (!l_pairs.has_antecessor_lane_id()) break;
        next_id = l_pairs.antecessor_lane_id().value();
      } else {
        // only continue when successor exists
        if (!l_pairs.has_successor_lane_id()) break;
        next_id = l_pairs.successor_lane_id().value();
      }

      // break if road end is reached
      if (next_id == -1) break;

      // break if another intersection is reached
      if (findLane(next_id, ground_truth)->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION) break;

      // add new lane to junction path
      found_next_lane = true;
      junction_path.lanes.push_back(next_id);
      lane_id = next_id;
      break;
    }
  }

  return junction_path; 
}


/**
 * @brief recursively calculate route
 * 
 * @param cur_idx 
 * @param dest_idx 
 * @param ground_truth 
 * @return std::vector<int> 
 */
std::vector<int> isReachable(int cur_idx, int dest_idx, osi3::GroundTruth* ground_truth, std::vector<int> distance_straight) {

  std::vector<int> route;

  // return lane_idx if reached destination
  if (cur_idx == dest_idx)
  {
    route.push_back(cur_idx);
    return route;
  }

  // get straight adjacent lanes
  std::vector<int> adjacent_lanes = findAdjacentLanes(ground_truth, cur_idx,"S");

  // recursivly try to reach the destination from adjacent lanes
  for (int j = 0; j < adjacent_lanes.size(); j++) 
  {
    // stop searching in this direction if distance to destination is higher than before
    if (distance_straight[cur_idx] < distance_straight[adjacent_lanes[j]]){
      continue;
    }
    route = isReachable(adjacent_lanes[j], dest_idx, ground_truth, distance_straight);
    if (route.size() > 0)  
    {
      route.push_back(cur_idx);
      return route;
    }
  }

  // if no adjacent lane with potential to reach the detination
  return route;
}


/**
 * @brief recursively calculates lane groups which are relevant to reach the detination lane from the starting lane
 * 
 * As a side product, also the amount of required lane changes, and the lanes where a lane change is possible are computed.
 * 
 * @param base_idx start lane
 * @param dest_idx end lane
 * @param ground_truth 
 * @param mode can be a combindation of 'L' and 'R', determine allowed changes
 * @return std::vector<LaneGroup> 
 */
std::vector<LaneGroup> calculateLaneGroups(int base_idx, int dest_idx, osi3::GroundTruth* ground_truth, std::vector<int> distance, std::vector<int> distance_straight) {

  std::vector<LaneGroup> groups;
  LaneGroup group; 

  // skip if distance can not be reached
  if (distance[base_idx] > 1000000) return groups;

  // check if destination can be reached from current lane WITHOUT lane change
  std::vector<int> route = isReachable(base_idx, dest_idx, ground_truth, distance_straight);

  // if destination can be reached from current lane, add single lane group
  if (route.size() > 0) 
  {
    for (int i = route.size() - 1; i >= 0; i--) 
    {
      group.lanes.push_back(ground_truth->lane(route[i]).id().value());
    }
    group.id = 0;
    group.change_amount = 0;

    groups.push_back(group);
    return groups;
  }

  int cur_idx = base_idx;
  uint64_t cur_id = ground_truth->lane(cur_idx).id().value();
  double cur_d = distance[cur_idx];
    
  bool reachable = false;     // if destination reachable
  bool end_of_lane = false;   // if end of current lane reached
  int dir = 0;                // direction of lane change

  // iterate over successor lanes and check adjacent lanes for reachability
  while(!end_of_lane)
  {
    group.lanes.push_back(cur_id);

    // iterate over neighbouring adjacent lanes
    int min_idx = -1;
    int type = 0;
    std::vector<int> left_adj = findAdjacentLanes(ground_truth, cur_idx, "L");
    for (int i = 0; i < left_adj.size(); i++)
    {
      double d = distance[left_adj[i]];
      if (d < cur_d)
      {
        min_idx = left_adj[i];
        type = 1;
      }
    }     

    std::vector<int> right_adj = findAdjacentLanes(ground_truth, cur_idx, "R");
    for (int i = 0; i < right_adj.size(); i++)
    {
      double d = distance[right_adj[i]];
      if (d < cur_d)
      {
        min_idx = right_adj[i];
        type = -1;
      }
    }    

    if (min_idx != -1) {
      group.lanes_changeable.push_back(cur_id);

      // if first possible change calculate new lanegroup
      if (!reachable) {
      
        reachable = true;
        groups = calculateLaneGroups(min_idx, dest_idx, ground_truth, distance, distance_straight);
        dir = type;
      }
    }  

    // iterate over straight adjacent lanes
    min_idx = -1;
    std::vector<int> next_lanes = findAdjacentLanes(ground_truth, cur_idx, "S");
    for (int i = 0; i < next_lanes.size(); i++)
    {
      double d = distance[next_lanes[i]];
      if (d < cur_d)
      {
        min_idx = next_lanes[i];
      }
    }     

    if (min_idx != -1) {
      cur_idx = min_idx;
      cur_id = ground_truth->lane(cur_idx).id().value();
      cur_d = distance[cur_idx];
    }  
    else
    {
      end_of_lane = true;
    }
  }
  
  // add properties to group if reachable
  if (reachable)
  {
    group.id = groups.back().id - dir;
    group.change_amount = groups.back().change_amount + dir;

    groups.push_back(group);
  }

  return groups;
}


/**
 * @brief creates an adjacency matrix with weights for the dijkstra algorithm
 * 
 * @param ground_truth 
 * @param weight_center optional weight that is put on continuing to drive on the center
 * @param weight_adjacent optional weight that is put on changing to a adjacent lane
 * @return int 
 */
std::vector<std::vector<int>> createAdjacencyMatrix(osi3::GroundTruth* ground_truth, int weight_center = 1, int weight_adjacent = 1){

  int num_of_vertices = ground_truth->lane_size();
  std::vector<std::vector<int>> adj_matrix(num_of_vertices, std::vector<int>(num_of_vertices, 0));

  // create edge connections and put their weight into the adjacency matrix
  for (int i = 0; i < ground_truth->lane_size(); i++) 
  {
    osi3::Lane::Classification cls = ground_truth->lane(i).classification();

    // only consider driving and intersection lanes
    if (cls.type() != osi3::Lane_Classification_Type_TYPE_DRIVING && cls.type() != osi3::Lane_Classification_Type_TYPE_INTERSECTION) continue;

    for (int j = 0; j < cls.lane_pairing_size(); j++) {

      uint64_t id = ground_truth->lane(i).id().value();
      uint64_t ant_id = cls.lane_pairing(j).antecessor_lane_id().value();
      uint64_t suc_id = cls.lane_pairing(j).successor_lane_id().value();

      // in driving direction:
      if (cls.centerline_is_driving_direction() || (cls.type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION && cls.centerline_size() == 0)) {
        if (suc_id == -1) continue;
        
        auto* next_lane = findLane(suc_id, ground_truth);
        if (next_lane == nullptr) continue;

        osi3::Lane::Classification::Type type = next_lane->classification().type();

        if (type != osi3::Lane_Classification_Type_TYPE_DRIVING && type != osi3::Lane_Classification_Type_TYPE_INTERSECTION) continue;

        int lane_idx = findLaneIdx(ground_truth, suc_id);

        adj_matrix[lane_idx][i] = weight_center;

      // opposite to driving direction:
      } else {
        if (ant_id == -1) continue;
        
        auto* next_lane = findLane(ant_id, ground_truth);
        if (next_lane == nullptr) continue;

        osi3::Lane::Classification::Type type = next_lane->classification().type();

        if (type != osi3::Lane_Classification_Type_TYPE_DRIVING && type != osi3::Lane_Classification_Type_TYPE_INTERSECTION) continue;

        int lane_idx = findLaneIdx(ground_truth, ant_id);

        adj_matrix[lane_idx][i] = weight_center;
      }
    }

    // only consider driving lanes for adjacent lanes
    if (cls.type() != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

    // left adjacent lanes
    for (int j = 0; j < cls.left_adjacent_lane_id_size(); j++) 
    {
      uint64_t adj_id = cls.left_adjacent_lane_id(j).value();

      auto* next_lane = findLane(adj_id, ground_truth);
      if (next_lane == nullptr) continue;

      osi3::Lane::Classification::Type type = next_lane->classification().type();

      if (type != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

      int lane_idx = findLaneIdx(ground_truth, adj_id);

      adj_matrix[lane_idx][i] = weight_adjacent;
    }

    // right adjacent lanes
    for (int j = 0; j < cls.right_adjacent_lane_id_size(); j++) 
    {
      uint64_t adj_id = cls.right_adjacent_lane_id(j).value();

      auto* next_lane = findLane(adj_id, ground_truth);
      if (next_lane == nullptr) continue;

      osi3::Lane::Classification::Type type = next_lane->classification().type();

      if (type != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

      int lane_idx = findLaneIdx(ground_truth, adj_id);

      adj_matrix[lane_idx][i] = weight_adjacent;
    }
  }

  return adj_matrix;
}


/**
 * @brief calculates the minimum distance for a set of univisited vertices
 * 
 * @param dist distance vector with distances from start vertex to all vertices in the graph
 * @param num_of_vertices number of all vertices in the graph
 * @param t_set set of vertices which have (true) / have not been visited (false)
 * @return int 
 */
int minimumDist(std::vector<int> &dist, int num_of_vertices, bool t_set[]){
  int min = INT_MAX;
  int index;

  for (int i = 0; i < num_of_vertices; i++){
    if(t_set[i] == false && dist[i] <= min){
      min = dist[i];
      index = i;
    }
  }
  return index;
}


/**
 * @brief computes the shortest path from a start point to a destination using the dijkstra algorithm (no need to save the parent nodes for path computation)
 *
 * @param adj_matrix adjacency matrix of the graph
 * @param start index of starting lane (index in ground_truth->lane field)
 * @param destination point to be reached
 */
void computeDijkstra (std::vector<std::vector<int>> adj_matrix, std::vector<int> &dist, int start, bool debug = false){
  int num_of_vertices = adj_matrix.size();  // get row size of adj_matrix (= col size)
  bool t_set[num_of_vertices];  // boolean array to mark visted/unvisted for each vertex
	
	for(int i = 0; i < num_of_vertices; i++)
	{
		dist[i] = INT_MAX;  // set the vertices to infinity distance
		t_set[i] = false; // mark all vertices as unvisited
	}
	
	dist[start] = 0;  // set distance of start vertex to 0
	
	for(int i = 0; i < num_of_vertices; i++)                           
	{
		int m = minimumDist(dist, num_of_vertices, t_set); // vertex not yet included
		t_set[m] = true; // m with minimum distance included in t_set

		for(int i = 0; i < num_of_vertices; i++)                  
		{
			// Updating the minimum distance for the particular vertex
			if(!t_set[i] && adj_matrix[m][i] && dist[m]!=INT_MAX && dist[m]+adj_matrix[m][i] < dist[i])
				dist[i] = dist[m] + adj_matrix[m][i];
		}
	}
  if (debug){
    SPDLOG_INFO("Dijkstra Algorithm: Reachability of vertices:");
    for (int i = 0; i < num_of_vertices; i++){
      SPDLOG_INFO("Distance ({}) = {}", i, dist[i]);
    }
  }
}


/**
 * @brief determines lanes along trajectory from the lane with start_idx to the
 * (x,y) point destination.
 *
 *
 * @param ground_truth
 * @param start index of starting lane (index in ground_truth->lane field)
 * @param destination point to be reached
 * @param future_lanes result
 */
void futureLanes(osi3::GroundTruth* ground_truth, const int& start_idx,
                  const Point2D& destination, std::vector<LaneGroup>& lane_groups) {

  // destination index
  int dest_idx;

  uint64_t dest_id = closestLane(ground_truth, destination);
  dest_idx = findLaneIdx(ground_truth, dest_id);

  // create an adjacency matrix out of the ground truth
  std::vector<std::vector<int>> adj_matrix;
  std::vector<std::vector<int>> adj_matrix_straight;
  adj_matrix = createAdjacencyMatrix(ground_truth);
  adj_matrix_straight = createAdjacencyMatrix(ground_truth, 1, 0); //INT_MAX

  // calculate shortest path with Dijkstra out of the adjacency matrix
  std::vector<int> dist(ground_truth->lane_size());
  std::vector<int> dist_straight(ground_truth->lane_size());

  // dest_idx is the start point for which we compute dijkstra
  computeDijkstra(adj_matrix, dist, dest_idx);
  computeDijkstra(adj_matrix_straight, dist_straight, dest_idx);

  // calculate lane groups
  lane_groups = calculateLaneGroups(start_idx, dest_idx, ground_truth, dist, dist_straight);

  // shift ids so that ego lane group has id 0
  for (int i = 0; i < lane_groups.size(); i++)
  {
    lane_groups[i].id -= lane_groups.back().id;
  }

  // add starting lane within single lane group, if no route could be found
  if (lane_groups.size() == 0)
  {
    SPDLOG_INFO("no route could be found for id({}) and dest_id({}), add starting lane within single lane group", start_idx, dest_id);
    LaneGroup group;
    group.id = 0;
    group.change_amount = 0;
    group.lanes.push_back(ground_truth->lane(start_idx).id().value());

    lane_groups.push_back(group);
  }
}


/**
 * @brief Calculate ds of signal 
 * 
 * @param ground_truth 
 * @param center_line 
 * @param signal_point 
 * @param ego_cl_point 
 * @param lane_id 
 * @param angle 
 * @param ds_gap 
 * @return double 
 */
double calcDsSignal(osi3::GroundTruth &ground_truth, std::vector<Point2D> &center_line, Point2D signal_point, Point2D ego_cl_point, uint64_t lane_id,
                    double angle, double ds_gap) {

  // distance in meters that the road marking's origin may be away
  // from the signal to make sure it is not across the map
  double dist_tol = 25; 
  bool found_road_marking = false;
  double ds = 0;
  Point2D centerline_point;
  osi3::RoadMarking rm;
  
  for (auto &cur_rm : ground_truth.road_marking()) {
    if (!found_road_marking)
    {
      for (auto &as_lid : cur_rm.classification().assigned_lane_id())
        if (as_lid.value() == lane_id) {
          if (cur_rm.classification().type() 
              == osi3::RoadMarking::Classification::TYPE_SYMBOLIC_TRAFFIC_SIGN
              && cur_rm.classification().traffic_main_sign_type()
              == osi3::TrafficSign::MainSign::Classification::TYPE_STOP)
          {
            // check if road marking origin is in reasonable distance to signal
            double dist_sig = sqrt(
                      pow( cur_rm.base().position().x() - signal_point.x , 2)
                    + pow( cur_rm.base().position().y() - signal_point.y , 2));
            if (dist_sig < dist_tol)
            {
              rm = cur_rm;
              found_road_marking = true;
            }
          }
        }
    }
  }

  if (found_road_marking) {
    // set centerline point w.r.t. road marking position.
    // Assumption: Road marking is perpendicular to lanes(s).
    Point2D rm_position(rm.base().position().x(), rm.base().position().y());
    closestCenterlinePoint(rm_position, center_line, centerline_point);
  } else {
    // set centerline point w.r.t. signal position
    closestCenterlinePoint(signal_point, center_line, centerline_point);
    ds = -ds_gap; // when no road marking was found apply ds_gap
  }

  ds += xy2s(ego_cl_point, centerline_point, center_line, angle);
  return ds;
}


/**
 * @brief calculate toffset to lane
 * 
 * @param land_id 
 * @param ground_truth 
 * @return double toffset
 */
double calcOffsetToLane(uint64_t land_id, Point2D position, osi3::GroundTruth* ground_truth) {

  osi3::Lane* adj_lane = findLane(land_id, ground_truth);

  // skip if not driving
  if (adj_lane->classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) return INFINITY;

  // get points on adjacent_lane
  std::vector<Point2D> adjacent_points; 
  getXY(adj_lane, adjacent_points);

  // calculate closest point on adjacent centerline
  Point2D adjacent_point;
  closestCenterlinePoint(position, adjacent_points, adjacent_point);

  // calculate euclidean distance
  double offset = euclideanDistance(adjacent_point, position);

  return offset;
}


/**
 * @brief calculate toffset to lane boundary
 * 
 * @param land_id 
 * @param ground_truth 
 * @return double toffset
 */
double calcOffsetToLaneBoundary(uint64_t boundary_id, Point2D position, osi3::GroundTruth* ground_truth) {

  // get points on boundary
  std::vector<Point2D> boundary_points; 
  getXY(findLaneBoundary(boundary_id, ground_truth), boundary_points);

  // calculate closest point on boundary centerline
  Point2D boundary_point;
  closestCenterlinePoint(position, boundary_points, boundary_point);

  // calculate euclidean distance
  double offset = euclideanDistance(boundary_point, position);

  return offset;
}


// --- DEPRECATED FUNCTIONS ----------------------------------------------------

/**
 * @brief map OSI lane IDs to corresponding agent_model lane IDs
 *
 * using unordered_map: keys are OSI lane IDs, values are agent_model lane IDs
 * @param ground_truth
 * @param mapping map
 * @param ego_lane_ptr
 * @param future_lanes vector of all lanes along the host's path
 */
void mapLanes(osi3::GroundTruth* ground_truth,
              std::unordered_map<uint64_t, int>& mapping, osi3::Lane* ego_lane_ptr,
              std::vector<uint64_t> future_lanes) {

  // assign over all lanes along the path
  for (int i = 0; i < future_lanes.size(); i++)
  {
    osi3::Lane* current = findLane(future_lanes[i], ground_truth);
    
    // assign ego lane
    mapping[future_lanes[i]] = 0;

    // assign left adjacent lane: 2
    if (current->classification().left_adjacent_lane_id_size() > 0)
    {
      uint64_t lane_id = current->classification().left_adjacent_lane_id(0).value();
      if (current->classification().centerline_is_driving_direction()) {
        mapping[lane_id] = 2;
      }
      else {  
        mapping[lane_id] = -2;
      }
    }
    // assign right adjacent lane: -2
    if (current->classification().right_adjacent_lane_id_size() > 0)
    {
      uint64_t lane_id =current->classification().right_adjacent_lane_id(0).value();
      if (current->classification().centerline_is_driving_direction()) {
        mapping[lane_id] = -2;
      }
      else {  
        mapping[lane_id] = 2;
      }
    }
  }

  // not all lanes are adjacent to ego_lane, assign arbitrary ID to the rest 
  int lane_count = 100;
  for (int i = 0; i < ground_truth->lane_size(); i++) {

    if (mapping.find(ground_truth->lane(i).id().value()) == mapping.end()) {
      // lane has not been mapped already
      mapping[ground_truth->lane(i).id().value()] = ++lane_count;
    }
  }
}


/**
 * @brief creates adjacency list corresponding to lane_pairings
 *
 * @param ground_truth
 * @param adjacency
 */
void createGraph(osi3::GroundTruth* ground_truth, std::vector<int> adj[]) {

  for (int i = 0; i < ground_truth->lane_size(); i++) 
  {
    osi3::Lane lane = ground_truth->lane(i);

    for (int j = 0; j < lane.classification().lane_pairing_size(); j++) 
    {
      uint64_t ant_id = lane.classification().lane_pairing(j).antecessor_lane_id().value();
      uint64_t suc_id = lane.classification().lane_pairing(j).successor_lane_id().value();

      if (lane.classification().centerline_is_driving_direction()) {
        if (suc_id == -1) continue;
        adj[i].push_back(findLaneIdx(ground_truth, suc_id));
      } else {
        if (ant_id == -1) continue;
        adj[i].push_back(findLaneIdx(ground_truth, ant_id));
      }
    }
  }
}


/**
 *  @brief a modified version of BFS that determines a path from src to dest and
 *  stores predecessor of each vertex in array pred
 *
 * @param adj adjacency list
 * @param src starting vertex
 * @param dest destination vertex
 * @param num_vertices number of vertices
 * @param pred predecessor array
 *
 */
bool BFS(std::vector<int> adj[], int src, int dest, int num_vertices,
         int pred[]) {

  std::list<int> queue;

  std::vector<bool> visited(num_vertices);

  for (int i = 0; i < num_vertices; i++) {
    visited[i] = false;

    pred[i] = -1;
  }

  visited[src] = true;

  queue.push_back(src);

  // standard BFS algorithm
  while (!queue.empty()) {

    int u = queue.front();
    queue.pop_front();
    for (int i = 0; i < adj[u].size(); i++) {
      if (visited[adj[u][i]] == false) {
        visited[adj[u][i]] = true;

        pred[adj[u][i]] = u;
        queue.push_back(adj[u][i]);

        // We stop BFS when we find
        // destination.
        if (adj[u][i] == dest) return true;
      }
    }
  }

  return false;
}


/**
 * @brief calculates width of lane at a point if the lane has adjacent lanes
 *
 *
 * @param point width at this point
 * @param lane lane corresponding to point
 * @param ground_truth
 * @return width
 */
double calcLaneWidth(const Point2D point, osi3::Lane* lane,
                 osi3::GroundTruth* ground_truth) {

  std::vector<uint64_t> lb_ids, rb_ids;
  std::vector<Point2D> lb_points, rb_points;

  // get boundary ids  
  for (int i = 0; i < lane->classification().left_lane_boundary_id_size();
       i++) {
    lb_ids.push_back(lane->classification().left_lane_boundary_id(i).value());
  }
  for (int i = 0; i < lane->classification().right_lane_boundary_id_size();
       i++) {
    rb_ids.push_back(lane->classification().right_lane_boundary_id(i).value());
  }

  if (lb_ids.size() == 0 || rb_ids.size() == 0) return 0;

  // get boundary points
  for (int i = 0; i < ground_truth->lane_boundary_size(); i++) {
    if (find(lb_ids.begin(), lb_ids.end(),
             ground_truth->lane_boundary(i).id().value()) != lb_ids.end()) {
      for (int k = 0; k < ground_truth->lane_boundary(i).boundary_line_size();
           k++) {
        Point2D tpoint(
          ground_truth->lane_boundary(i).boundary_line(k).position().x(),
          ground_truth->lane_boundary(i).boundary_line(k).position().y());
        lb_points.push_back(tpoint);
      }
    } else if (find(rb_ids.begin(), rb_ids.end(),
                    ground_truth->lane_boundary(i).id().value()) !=
               rb_ids.end()) {
      for (int k = 0; k < ground_truth->lane_boundary(i).boundary_line_size();
           k++) {

        Point2D tpoint(
          ground_truth->lane_boundary(i).boundary_line(k).position().x(),
          ground_truth->lane_boundary(i).boundary_line(k).position().y());
        rb_points.push_back(tpoint);
      }
    }
  }

  // get desired point on boundaries
  Point2D r_point, l_point;
  closestCenterlinePoint(point, lb_points, l_point);
  closestCenterlinePoint(point, rb_points, r_point);

  // calculate width
  return sqrt(pow(l_point.x - r_point.x, 2) + pow(l_point.y - r_point.y, 2));
}