#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "osi_sensorview.pb.h"

/**
 * @brief computes euclidean distance between two points
 * 
 * @param p 
 * @param q 
 * @return double euclidean distance
 */
double euclideanDistance(Point2D p, Point2D q){
  return sqrt(pow(p.x-q.x,2) + pow(p.y-q.y,2));
}

/**
 * @brief computes gradients with finite differences
 *
 * @param x 
 * @param y
 * @param order of differentiation
 * @return gradient at every point
 */
std::vector<double> gradient(std::vector<double> x, std::vector<double> y,
                             int order) {
  std::vector<double> res;

  assert(x.size() == y.size());
  int n = x.size();
  res.assign(n, 0);

  if (order == 1) {
    // calculate interior gradient values with central differences
    for (int i = 1; i < n - 1; i++)
      res[i] = (y[i + 1] - y[i - 1]) / (x[i + 1] - x[i - 1]);

    // calculate gradient at edges with one sided differences
    res[0] = (y[1] - y[0]) / (x[1] - x[0]);
    res[n - 1] = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
  }

  if (order == 2) {
    // calculate interior gradient values with central differences
    for (int i = 1; i < n - 1; i++) {
      double h1 = (x[i] - x[i - 1]), h2 = (x[i + 1] - x[i]);
      res[i] =
        (2.0 / (h1 + h2)) * ((y[i + 1] - y[i]) / h2 - (y[i] - y[i - 1]) / h1);
    }

    // calculate gradient at edges with one sided differences
    // res[0] = (y[2] - 2 * y[1] + y[0]) / pow(x[1] - x[0], 2);
    double h1 = (x[1] - x[0]), h2 = (x[2] - x[1]);
    res[0] = (2.0 / (h1 + h2)) * ((y[2] - y[1]) / h2 - (y[1] - y[0]) / h1);
    h1 = (x[n - 1] - x[n - 2]), h2 = (x[n - 2] - x[n - 3]);
    res[n - 1] = (2.0 / (h1 + h2)) *
                 ((y[n - 1] - y[n - 2]) / h1 - (y[n - 2] - y[n - 3]) / h2);
  }

  return res;
}

/**
 * @brief calculates s (distance), psi (global angle w.r.t. x-axis), kappa
 * (curvature) from xy coordinates
 *
 *
 * @param pos position points in global xy-coordinates
 * @param s distance result
 * @param psi angle result
 * @param k curvature result
 * @return int success
 */
int xy2Curv(std::vector<Point2D> pos, std::vector<double>& s,
            std::vector<double>& psi, std::vector<double>& k) {
  // calculate s from xy coordinates
  s.push_back(0);
  std::vector<double> x(1, pos[0].x), y(1, pos[0].y);

  for (int i = 1; i < pos.size(); i++) {
    double dx = pos[i].x - pos[i - 1].x;
    double dy = pos[i].y - pos[i - 1].y;
    x.push_back(pos[i].x);
    y.push_back(pos[i].y);

    s.push_back(s.back() + sqrt(pow(dx, 2) + pow(dy, 2)));
    double p = atan2(dy, dx);
    // psi.push_back(p >= 0 ? p : p + 2 * M_PI);
    psi.push_back(p);
  }

  // calculate psi and kappa
  std::vector<double> dxds = gradient(s, x, 1);
  std::vector<double> dyds = gradient(s, y, 1);
  std::vector<double> dxxdss = gradient(s, x, 2);
  std::vector<double> dyydss = gradient(s, y, 2);


  for (int i = 0; i < pos.size(); i++) {
    if (pos.size() > 2) {
      double top = dxds[i] * dyydss[i] - dxxdss[i] * dyds[i];
      double bot = pow(dxds[i] * dxds[i] + dyds[i] * dyds[i], 1.5);
      k.push_back(top / bot);
    } else {
      k.push_back(0.0);
    }
  }

  return 0;
}

/**
 * @brief find closest point on centerline of current lane to a point(x,y)
 *
 * @param point
 * @param cl centerline (can also be boundary line)
 * @param closest point
 * @param extrap if closest point can be also an extrapolation beyond boundaries
 * @return index of the next centerline point following the projected point. 
 *    0 when before scope of cl
 *    cl.size() when after scope of cl
 */
int closestCenterlinePoint(const Point2D point, const std::vector<Point2D>& cl,
                           Point2D& closest, bool extrap) {

  assert(cl.size() > 1);

  // initialize variables
  double min_dist = INFINITY;
  int min_i;
  Point2D tmp;

  // iterate over all segments of centerline
  for (int i = 1; i < cl.size(); i++) {

    // get coordinates
    double x1 = cl[i - 1].x;
    double x2 = cl[i].x;
    double y1 = cl[i - 1].y;
    double y2 = cl[i].y;

    double dx = x2 - x1;
    double dy = y2 - y1;

    // skip if identical centerline points
    if (x1 == x2 && y1 == x2) continue;

    // compute orthogonal projection of (x,y) onto the parameterized line
    // connecting (x1,y1) and (x2,y2)
    double l2 = dx * dx + dy * dy;
    double dot = (point.x - x1) * dx + (point.y - y1) * dy;
    double t = dot / l2;

    // if in segment
    if (t >= 0 && t <= 1) {
      
      // compute point on centerline
      tmp.x = x1 + t * dx;
      tmp.y = y1 + t * dy;
      
      double dist = pow(tmp.x - point.x, 2) + pow(tmp.y - point.y, 2);

      if (dist < min_dist) {
        min_dist = dist;
        closest = tmp;
        min_i = i;
      }
    }
    // if outside of segment
    else {
      double dist_end = pow(point.x - x2, 2) + pow(point.y - y2, 2);

      if (dist_end < min_dist) {
        min_dist = dist_end;
        closest = cl[i];
        min_i = i;
      }
    }
  }

  // check if distance to start point is smaller
  double dist_start = pow(point.x - cl.front().x, 2) + pow(point.y - cl.front().y, 2);
  double dist_end = pow(point.x - cl.back().x, 2) + pow(point.y - cl.back().y, 2);

  if (dist_start < min_dist || dist_end < min_dist) {
    
    Point2D start, end;

    // if distance to start is minimal
    if (dist_start < dist_end){
      min_i = 0;

      start = cl[0];
      end = cl[1];

      closest = start;
    }

    // if distance to end is minimal
    if (dist_end < dist_start){
      min_i = cl.size();

      start = cl[cl.size() - 2];
      end = cl[cl.size() - 1];

      closest = end;
    }

    // calculate closest if extrapolation desired
    if (extrap) {
      double dx = end.x - start.x;
      double dy = end.y - start.y;


      double l2 = dx * dx + dy * dy;
      double dot = (point.x - start.x) * dx + (point.y - start.y) * dy;
      double t = dot / l2;

      closest.x = start.x + t * dx;
      closest.y = start.y + t * dy;
    }
  }

  return min_i;
}

/**
 * @brief Calculate approximate centerline connection for intersection area
 *
 * This function creates centerline points between two road ends. To do so,
 * two points at each road end are required. With that information, the
 * parameterized cubic polynomials x(t) and y(t) can be solved unique.
 * The ODE system for x(t) is set up as follows:
 * x(0) = x1, x(\delta x) = x2 with \delta x = abs(x2-x1)
 * x'(0) = cos(ang1), x(\delta x) = cos(ang2) where ang1 and ang2 are the angles
 * at the end of each road. With those four equations, the ODE A*c=b can be
 * solved for c
 *
 * @param p1_p2 a vector of four points that describe both road ends
 * @param pos the centerline with a gap that should be filled by this function
 * @param idx index within pos where the gap is located
 * @return length of inserted gap vector
 */

int calcXYGap(std::vector<Point2D> p1_p2, std::vector<Point2D>& pos, int idx) {
  // gap start/end plus second last point, respectively
  double x1 = p1_p2[1].x;
  double y1 = p1_p2[1].y;
  double x1_h = p1_p2[0].x;
  double y1_h = p1_p2[0].y;
  double x2 = p1_p2[2].x;
  double y2 = p1_p2[2].y;
  double x2_h = p1_p2[3].x;
  double y2_h = p1_p2[3].y;

  // help/intermediate variables
  double dx = abs(x2 - x1);
  double dy = abs(y2 - y1);
  double ang1 = atan2(y1 - y1_h, x1 - x1_h);
  double ang2 = atan2(y2_h - y2, x2_h - x2);

  // manual coefficient calculation after paper+pen
  // x(0) = x1, x(dx) = x2, x'(0) = cos(ang1), x'(dx) = cos(ang2)
  // y similar but with sin()
  // Remark: Those calculation are the solution to an ODE of the form A \ b = c
  double ca_x = (cos(ang1)*dx + 2*x1 - 2*x2 + dx*cos(ang2)) / pow(dx, 3);
  double cb_x = (-2*cos(ang1)*dx - 3*x1 + 3*x2 - dx*cos(ang2)) / pow(dx, 2);
  double cc_x = cos(ang1);
  double cd_x = x1;
  double ca_y = (sin(ang1)*dy + 2*y1 - 2*y2 + dy*sin(ang2)) / pow(dy, 3);
  double cb_y = (-2*sin(ang1)*dy - 3*y1 + 3*y2 - dy*sin(ang2)) / pow(dy, 2);
  double cc_y = sin(ang1);
  double cd_y = y1;

  std::vector<Point2D> gapXY;
  int N = 30;  // number of interpolated points
  double ddx = dx / (N - 1);
  double ddy = dy / (N - 1);
  double tx = 0;
  double ty = 0;
  for (int i = 0; i < N; i++) {
    Point2D tmp;
    // calculate parameter polynomial function
    tmp.x = ca_x * pow(tx, 3) + cb_x * pow(tx, 2) + cc_x * tx + cd_x;
    tmp.y = ca_y * pow(ty, 3) + cb_y * pow(ty, 2) + cc_y * ty + cd_y;

    tx += ddx;
    ty += ddy;
    // add point
    gapXY.push_back(tmp);
  }

  pos.insert(pos.begin() + idx, gapXY.begin(), gapXY.end());

  // return size of gap
  return gapXY.size();
}

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

  std::vector<int> lb_ids, rb_ids;
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
  closestCenterlinePoint(point, lb_points, l_point, true);
  closestCenterlinePoint(point, rb_points, r_point, true);

  // calculate width
  return sqrt(pow(l_point.x - r_point.x, 2) + pow(l_point.y - r_point.y, 2));
}

/**
 * @brief ds along xy starting at (startX,startY) and ending at (endX,endY) with
 * correct sign
 *
 * @param start point
 * @param end point
 * @param cl centerline
 * @return resulting distance
 */
double xy2s(const Point2D start, const Point2D end,
               const std::vector<Point2D>& cl, double start_psi) {
  double s = 0;

  // calculate distance
  int dir = 0;
  double start_end_psi = atan2(end.y - start.y, end.x - start.x);
    
  if (abs(start_end_psi - start_psi) > M_PI/2) {
    dir = -1;
  }
  else {
    dir = 1;
  }

  Point2D start_centerline, end_centerline;
  int start_idx = closestCenterlinePoint(start, cl, start_centerline, true);
  int end_idx = closestCenterlinePoint(end, cl, end_centerline, true);

  // if start and end in same interval 
  if (start_idx == end_idx) {
    return dir * sqrt(pow(start_centerline.x - end_centerline.x, 2) 
              + pow(start_centerline.y - end_centerline.y, 2));
  }
  else if (start_idx < end_idx) {
    for (int i = start_idx; i < end_idx-1; i++) {
      double dx = cl[i+1].x - cl[i].x;
      double dy = cl[i+1].y - cl[i].y;

      s += sqrt(dx * dx + dy * dy);
    }
    s += sqrt(pow(start_centerline.x - cl[start_idx].x, 2) 
            + pow(start_centerline.y - cl[start_idx].y, 2));
    s += sqrt(pow(end_centerline.x - cl[end_idx-1].x, 2) 
            + pow(end_centerline.y - cl[end_idx-1].y, 2));
    return s;
  }
  else if (start_idx > end_idx) {
    for (int i = end_idx; i < start_idx-1; i++) {
      double dx = cl[i+1].x - cl[i].x;
      double dy = cl[i+1].y - cl[i].y;

      s -= sqrt(dx * dx + dy * dy);
    }
    s -= sqrt(pow(start_centerline.x - cl[start_idx-1].x, 2) +
              pow(start_centerline.y - cl[start_idx-1].y, 2));
    s -= sqrt(pow(end_centerline.x - cl[end_idx].x, 2) 
            + pow(end_centerline.y - cl[end_idx].y, 2));
    return s;
  }
  else {
    return 0;
  }
}

/**
 * @brief find a laneID in ground_truth.lane and return pointer
 *
 * @param ID
 * @param ground_truth
 */
osi3::Lane* findLane(int id, osi3::GroundTruth* ground_truth) {
  for (int i = 0; i < ground_truth->lane_size(); i++) {
    if (ground_truth->lane(i).id().value() == id)
      return (ground_truth->mutable_lane(i));
  }
  return nullptr;
}

/**
 * @brief find a laneID in ground_truth.lane and return its index
 *
 * @param ground_truth
 * @param ID
 */
int findLaneIdx(osi3::GroundTruth* ground_truth, int id) {
  for (int i = 0; i < ground_truth->lane_size(); i++) {
    if (ground_truth->lane(i).id().value() == id) return i;
  }
  return -1;
}

/**
 * @brief find a laneboundaryID in ground_truth.lane_boundary and return pointer
 *
 * @param ID
 * @param ground_truth
 * @return osi3::LaneBounddary* pointer to desired lane boundary
 */
osi3::LaneBoundary* findLaneBoundary(int id, osi3::GroundTruth* ground_truth) {
  for (int i = 0; i < ground_truth->lane_boundary_size(); i++) {
    if (ground_truth->lane_boundary(i).id().value() == id)
      return (ground_truth->mutable_lane_boundary(i));
  }
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
  return tmp;
}

/**
 * @brief find a point in a point vector based on distanc (small tolerance eps) 
 * 
 * @param points 
 * @param point 
 * @return int  index of point in vector
 */
int findPointInPoints(std::vector<Point2D> points, Point2D point) {
  double eps = 0.001;

  for (int i = 0; i < points.size(); i++)
  {
    if (euclideanDistance(point, points[i]) < eps) return i;
  }
  return -1;
}


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
              std::unordered_map<int, int>& mapping, osi3::Lane* ego_lane_ptr,
              std::vector<int> future_lanes) {

  // assign over all lanes along the path
  for (int i = 0; i < future_lanes.size(); i++)
  {
    osi3::Lane* current = findLane(future_lanes[i], ground_truth);
    
    // assign ego lane
    mapping[future_lanes[i]] = 0;

    // assign left adjacent lane: 2
    if (current->classification().left_adjacent_lane_id_size() > 0)
    {
      int lane_id = current->classification().left_adjacent_lane_id(0).value();
      mapping[lane_id] = 2;
    }
    // assign right adjacent lane: -2
    if (current->classification().right_adjacent_lane_id_size() > 0)
    {
      int lane_id =current->classification().right_adjacent_lane_id(0).value();
      mapping[lane_id] = -2;
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
      
      int ant_id = lane.classification().lane_pairing(j).antecessor_lane_id().value();
      int suc_id = lane.classification().lane_pairing(j).successor_lane_id().value();

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

  bool visited[num_vertices];

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

int interpolateXY2Value(std::vector<double> y, std::vector<Point2D> xy,
                        Point2D pos) {
  Point2D dummy;
  int i = closestCenterlinePoint(pos, xy, dummy, false);

  // crop idx to boundaries
  if (i == 0) i = 1;
  if (i == xy.size()) i = xy.size()-1;

  double x1 = xy[i - 1].x;
  double x2 = xy[i].x;
  double y1 = xy[i - 1].y;
  double y2 = xy[i].y;

  // compute orthogonal projection of (x,y) onto the parameterized line
  // connecting (x1,y1) and (x2,y2)
  double l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  double dot = (pos.x - x1) * (x2 - x1) + (pos.y - y1) * (y2 - y1);
  double t = dot / l2;

  return y[i - 1] + t * (y[i] - y[i - 1]);
}

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
int closestLane(osi3::GroundTruth* ground_truth, const Point2D point) {
  std::vector<Point2D> centerline;
  double distance = INFINITY;
  double distance_valid_idx = INFINITY;

  // lanes with valid idx have only priority if closer `max_distance_valid_idx`
  double max_distance_valid_idx = 5;        

  int dest_id = -1;
  int dest_id_valid_idx = -1;

  for (int i = 0; i < ground_truth->lane_size(); i++) {
    centerline.clear();
    osi3::Lane cur_lane;
    cur_lane.CopyFrom(ground_truth->lane(i));
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
JunctionPath calcJunctionPath(osi3::GroundTruth* ground_truth, int lane_id)
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
      int next_id, from_id;

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
std::vector<int> getAdjacentLanes(osi3::GroundTruth* ground_truth, int lane_idx, char mode)
{
  std::vector<int> lanes; 

  osi3::Lane lane = ground_truth->lane(lane_idx);

  // get driving direction
  bool is_driving_direction = lane.classification().centerline_is_driving_direction();

  if (mode == 'S')
  {
    // iterate over all lane pairings
    for (int j = 0; j < lane.classification().lane_pairing_size(); j++) 
    {
      // get adjacent_id
      int adjacent_id;
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

      lanes.push_back(findLaneIdx(ground_truth, adjacent_id));
    }
  }

  if (mode == 'L' || mode == 'R')
  {
      // skip if wrong type
    if (lane.classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) return lanes;

    // check for left neighbouring lanes
    if ((mode == 'L' && is_driving_direction) || (mode == 'R' && !is_driving_direction))
    {
      for (int i = 0; i < lane.classification().left_adjacent_lane_id_size() > 0; i++)
      {
        int adjacent_id = lane.classification().left_adjacent_lane_id(i).value();

        // skip if wrong type
        if (findLane(adjacent_id, ground_truth)->classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

        // skip if opposite driving direction (assumption: same reference line)
        if (findLane(adjacent_id, ground_truth)->classification().centerline_is_driving_direction() != is_driving_direction) continue;
        
        lanes.push_back(findLaneIdx(ground_truth, adjacent_id));
      }
    }

    // check for right neighbouring lanes
    if ((mode == 'R' && is_driving_direction) || (mode == 'L' && !is_driving_direction))
    {
      for (int i = 0; i < lane.classification().right_adjacent_lane_id_size() > 0; i++)
      {
        int adjacent_id = lane.classification().right_adjacent_lane_id(i).value();

        // skip if wrong type
        if (findLane(adjacent_id, ground_truth)->classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) continue;

        // skip if opposite driving direction (assumption: same reference line)
        if (findLane(adjacent_id, ground_truth)->classification().centerline_is_driving_direction() != is_driving_direction) continue;
        
        lanes.push_back(findLaneIdx(ground_truth, adjacent_id));
      }
    }
  }
  
  return lanes;
}

/**
 * @brief recursively calculate route
 * 
 * @param cur_idx 
 * @param dest_idx 
 * @param ground_truth 
 * @return std::vector<int> 
 */
std::vector<int> calculateRoute(int cur_idx, int dest_idx, osi3::GroundTruth* ground_truth)
{
  std::vector<int> route;

  // return lane_idx if reached destination
  if (cur_idx == dest_idx)
  {
    route.push_back(cur_idx);
    return route;
  }

  // get straight adjacent lanes
  std::vector<int> adjacent_lanes = getAdjacentLanes(ground_truth, cur_idx,'S');

  // recursivly try to reach the destination from adjacent lanes
  for (int j = 0; j < adjacent_lanes.size(); j++) 
  {
    route = calculateRoute(adjacent_lanes[j], dest_idx, ground_truth);
    
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
std::vector<LaneGroup> calculateLaneGroups(int base_idx, int dest_idx, osi3::GroundTruth* ground_truth, std::string mode)
{
  std::vector<LaneGroup> groups;
  LaneGroup group; 

  // check if destination can be reached from current lane without lane change
  std::vector<int> route = calculateRoute(base_idx, dest_idx, ground_truth);

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

  // go to left/right neighbouring lane (according to mode) to find destination
  for(int k = 0; k < mode.length(); k++)
  {
    char m = mode.at(k);

    int cur_idx = base_idx;
    int cur_id = ground_truth->lane(cur_idx).id().value();
      
    bool reachable = false;     // if destination reachable
    bool end_of_lane = false;   // if end of current lane reached

    // iterate over successor lanes and check adjacent lanes for reachability
    while(!end_of_lane)
    {
      group.lanes.push_back(cur_id);

      // iterate over adjacent lanes
      std::vector<int> adj_lanes = getAdjacentLanes(ground_truth, cur_idx, m);
      for (int i = 0; i < adj_lanes.size(); i++)
      {
        std::vector<LaneGroup> tmp_groups;
        tmp_groups = calculateLaneGroups(adj_lanes[i], dest_idx, ground_truth, std::string(1,m)); 

        // if lane change can be performed at current lane
        if (tmp_groups.size() > 0)
        {
          // mark first reachable lane
          if (!reachable)
          {
            reachable = true;
            groups = tmp_groups;
          }

          // add current lane to changable lanes
          group.lanes_changeable.push_back(cur_id);
          break;
        }
      }
      
      // get straight adjacent lanes to move forward on current lane
      std::vector<int> next_lanes = getAdjacentLanes(ground_truth, cur_idx,'S');
      if (next_lanes.size() > 0) 
      {
        cur_idx = next_lanes[0]; // TODO only one lane supported
        cur_id = ground_truth->lane(cur_idx).id().value();
      }
      else
      {
        end_of_lane = true;
      }
    }
    
    // add properties to group if reachable
    if (reachable)
    {
      int dir = 0;
      if (m == 'L') dir = 1;
      if (m == 'R') dir = -1;
      
      group.id = groups.back().id - dir;
      group.change_amount = groups.back().change_amount + dir;

      groups.push_back(group);

      // if destination reachable
      break;
    }
  } 

  return groups;
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
                  const Point2D& destination, std::vector<LaneGroup>& lane_groups) 
{
  // destination index
  int dest_idx;

  int dest_id = closestLane(ground_truth, destination);
  dest_idx = findLaneIdx(ground_truth, dest_id);

  // calculate lane groups
  lane_groups = calculateLaneGroups(start_idx, dest_idx, ground_truth, "LR");

  // shift ids so that ego lane group has id 0
  for (int i = 0; i < lane_groups.size(); i++)
  {
    lane_groups[i].id -= lane_groups.back().id;
  }

  // add starting lane within single lane group, if no route could be found
  if (lane_groups.size() == 0)
  {
    LaneGroup group;
    group.id = 0;
    group.change_amount = 0;
    group.lanes.push_back(ground_truth->lane(start_idx).id().value());

    lane_groups.push_back(group);
  }
}

/**
 * @brief coordinate rotation with angle phi
 *
 * @param pre-rotation point
 * @param post-rotation point
 * @param angle
 */
void transform(Point2D& pre, Point2D& post, double phi) {
  double c = std::cos(phi);
  double s = std::sin(phi);
  post.x = pre.x * c + pre.y * s;
  post.y = pre.x * (-1 * s) + pre.y * c;
}

/**
 * @brief create 3rd order spline connecting start and end
 * d_start, d_end contain direction vectors in Start and End point. Spline
 * points are generated in vector cl
 *
 * @param start point
 * @param end point
 * @param d_start heading at start
 * @param d_end heading at end
 * @param cl resulting spline
 *
 */
void spline3(Point2D start, Point2D end, Point2D d_start, Point2D d_end,
             std::vector<Point2D>& cl) {

  // transform into local coordinate system (rotation so that new x-axis passes
  // through start and end)
  Point2D s, e, ds, de, se, der;
  se.x = end.x - start.x;
  se.y = end.y - start.y;

  // angle between coordinate systems. Ensured that start --> end is positive
  // x-direction in new coordiante system
  double phi = (se.y > 0)
                 ? std::acos(se.x / sqrt(se.x * se.x + se.y * se.y))
                 : -1 * std::acos(se.x / sqrt(se.x * se.x + se.y * se.y));


  transform(start, s, phi);
  transform(end, e, phi);
  transform(d_start, ds, phi);
  transform(d_end, de, phi);

  // maximum derivative is set to 50 ~ 89 degrees
  der.x = abs(ds.x) > abs(ds.y / 50.0) ? ds.y / ds.x
                                       : std::copysignf(1, ds.x * ds.y) * 50;
  der.y = abs(de.x) > abs(de.y / 50.0) ? de.y / de.x
                                       : std::copysignf(1, de.x * de.y) * 50;

  // creating spline of form f(x)= a + bx + cx^2 + dx^3, der.x contains
  // f'(start.x), der.y contains f'(end.x)

  double tmp = 1.0 / (pow(s.x - e.x, 3));

  double a = -tmp * (der.y * e.x * pow(s.x, 3) + der.x * e.x * e.x * s.x * s.x -
                     der.y * e.x * e.x * s.x * s.x - der.x * pow(e.x, 3) * s.x -
                     pow(s.x, 3) * e.y + 3 * e.x * s.x * s.x * e.y -
                     3 * e.x * e.x * s.x * s.y + pow(e.x, 3) * s.y);

  double b = -tmp * (-der.y * pow(s.x, 3) - 2 * der.x * e.x * s.x * s.x -
                     der.y * e.x * s.x * s.x + der.x * e.x * e.x * s.x +
                     2 * der.y * e.x * e.x * s.x + der.x * pow(e.x, 3) +
                     6 * e.x * s.x * s.y - 6 * e.x * s.x * e.y);

  double c =
    -tmp * (der.x * s.x * s.x + 2 * der.y * s.x * s.x + der.x * e.x * s.x -
            der.y * e.x * s.x - 2 * der.x * e.x * e.x - der.y * e.x * e.x -
            3 * s.x * s.y + 3 * s.x * e.y - 3 * e.x * s.y + 3 * e.x * e.y);

  double d = -tmp * (-der.x * s.x - der.y * s.x + der.x * e.x + der.y * e.x +
                     2 * s.y - 2 * e.y);

  const float max_error = 0.035;

  // generate "centerline" points.
  double x = s.x;
  double y = 0;

  while (x < e.x) {
    double kappa = abs(2 * c + 6 * d * x);
    double delta_x =
      (kappa < 0.001)
        ? 1.0
        : 2 * sqrt(2 * max_error * 1 / kappa - max_error * max_error);
    if (2 * max_error * 1 / kappa < max_error * max_error) delta_x = 1 / kappa;
    x += delta_x;
    y = a + b * x + c * x * x + d * x * x * x;
    if (x < e.x) cl.push_back(Point2D(x, y));
  }

  // transform back

  Point2D temp;
  for (int i = 0; i < cl.size(); i++) {
    transform(cl[i], temp, -1.0 * phi);
    cl[i] = temp;
  }
}

/**
 * @brief create linear spline connecting start and end
 *
 * @param start point
 * @param end point
 * @param cl resulting spline
 *
 */
void spline1(Point2D start, Point2D end, std::vector<Point2D>& cl) {

  Point2D se(end.x - start.x, end.y - start.y);
  double norm = sqrt(se.x * se.x + se.y * se.y);

  se.x = se.x / norm;
  se.y = se.y / norm;

  if (norm < 1) {
    cl.push_back(Point2D(start.x + 0.5 * se.x, start.y + 0.5 * se.y));
    return;
  }

  for (int i = 1; i < norm; i++) {
    cl.push_back(Point2D(start.x + i * se.x, start.y + i * se.y));
  }
}

/**
 * @brief get norm of osi3::Vector3d
 *
 * @param v vector
 * @return euclidean norm
 */
double getNorm(osi3::Vector3d v) {
  return sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2));
}

/**
 * @brief get wrapped angle to [-pi,pi]
 *
 * @param psi input angle
 * @return wrapped angle
 */
double wrapAngle(double psi) {
  return std::atan2(std::sin(psi), std::cos(psi));
}

void removeDuplicates(std::vector<Point2D> &v) {
  if (v.size() == 0) return;

  for (int i = 0; i < v.size() - 1; i++) {
    double ds = pow(v[i + 1].x - v[i].x, 2) + pow(v[i + 1].y - v[i].y, 2);

    if (ds < 0.01) {
        v.erase(v.begin() + i);
        i--;
    }
  }
}

double computeDistanceInRefAngleSystem(Point2D ego, Point2D centerline, double ref_angle) {

  // calculate orientation from position to centerline projection
  double orientation = ref_angle - atan2(centerline.y - ego.y,
                                       centerline.x - ego.x);

  // adjust orientation
  if (orientation < -M_PI) orientation = orientation + 2 * M_PI;
  if (orientation > M_PI) orientation = orientation - 2 * M_PI;
  int d_sig = (orientation > 0) - (orientation < 0);

  // calculate distance to centerline point
  double d = sqrt(pow(ego.x - centerline.x, 2) +
                       pow(ego.y - centerline.y, 2));
  
  return d_sig * d;
}

double calcDsSignal(osi3::GroundTruth &ground_truth, std::vector<Point2D> &center_line, 
                    Point2D signal_point, Point2D ego_cl_point, int lane_id,
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
    closestCenterlinePoint(rm_position, center_line, centerline_point, true);
  } else {
    // set centerline point w.r.t. signal position
    closestCenterlinePoint(signal_point, center_line, centerline_point, true);
    ds = -ds_gap; // when no road marking was found apply ds_gap
  }

  ds += xy2s(ego_cl_point, centerline_point, center_line, angle);
  return ds;
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
bool isSigAssigned(T &cls, std::vector<int> &signal_lanes, std::vector<int> &lanes, int &assigned_lane_id)
{
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

/**
 * @brief calculate toffset to lane
 * 
 * @param land_id 
 * @param ground_truth 
 * @return double toffset
 */
double calcOffsetToLane(int land_id, Point2D position, osi3::GroundTruth* ground_truth) {

  osi3::Lane* adj_lane = findLane(land_id, ground_truth);

  // skip if not driving
  if (adj_lane->classification().type() != osi3::Lane_Classification_Type_TYPE_DRIVING) return INFINITY;

  // get points on adjacent_lane
  std::vector<Point2D> adjacent_points; 
  getXY(adj_lane, adjacent_points);

  // calculate closest point on adjacent centerline
  Point2D adjacent_point;
  closestCenterlinePoint(position, adjacent_points, adjacent_point, true);

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
double calcOffsetToLaneBoundary(int boundary_id, Point2D position, osi3::GroundTruth* ground_truth) {

  // get points on boundary
  std::vector<Point2D> boundary_points; 
  getXY(findLaneBoundary(boundary_id, ground_truth), boundary_points);

  // calculate closest point on boundary centerline
  Point2D boundary_point;
  closestCenterlinePoint(position, boundary_points, boundary_point, true);

  // calculate euclidean distance
  double offset = euclideanDistance(boundary_point, position);

  return offset;
}