#pragma once
#include <cmath>
#include <numeric>
#include <vector>
#include <unordered_map>
#include <list>
#include "osi_sensorview.pb.h"
#include "Interface.h"
#include <iostream>

using Point2D = agent_model::Position;

/**
 * @brief computes gradients with finite differences
 *
 * @param x
 * @param y
 * @param order
 * @return std::vector<double>
 */
std::vector<double> gradient(std::vector<double> x, std::vector<double> y, int order)
{
	std::vector<double> res;

	assert(x.size() == y.size());
	int n = x.size();
	res.assign(n, 0);

	if (order == 1)
	{
		// calculate interior gradient values with central differences
		for (int i = 1; i < n - 1; i++)
			res[i] = (y[i + 1] - y[i - 1]) / (x[i + 1] - x[i - 1]);

		// calculate gradient at edges with one sided differences
		res[0] = (y[1] - y[0]) / (x[1] - x[0]);
		res[n - 1] = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
	}

	if (order == 2)
	{
		// calculate interior gradient values with central differences
		for (int i = 1; i < n - 1; i++)
			res[i] = (y[i + 1] - 2 * y[i] + y[i - 1]) / pow(x[i + 1] - x[i], 2);

		// calculate gradient at edges with one sided differences
		res[0] = (y[2] - 2 * y[1] + y[0]) / pow(x[1] - x[0], 2);
		res[n - 1] = (y[n - 1] - 2 * y[n - 2] + y[n - 3]) / pow(x[n - 1] - x[n - 2], 2);
	}

	return res;
}


/**
 * @brief s, psi, kappa from xy coordinates
 *	
 *
 * @param pos position
 * @param s
 * @param psi
 * @param k
 * @return int
 */
int xy2curv(std::vector<Point2D> pos, std::vector<double>& s, std::vector<double>& psi, std::vector<double>& k)
{
	// calculate s from xy coordinates
	s.push_back(0);
	std::vector<double> x(1, 0), y(1, 0);
	for (int i = 1; i < pos.size(); i++)
	{
		double dx = pos[i].x - pos[i - 1].x;
		double dy = pos[i].y - pos[i - 1].y;
		x.push_back(pos[i].x);
		y.push_back(pos[i].y);

		s.push_back(s.back() + sqrt(pow(dx, 2) + pow(dy, 2)));
	}

	// calculate psi and kappa
	std::vector<double> dxds = gradient(s, x, 1);
	std::vector<double> dyds = gradient(s, y, 1);
	std::vector<double> dxxdss = gradient(s, x, 2);
	std::vector<double> dyydss = gradient(s, y, 2);

	for (int i = 0; i < x.size(); i++)
	{
		double top = dxds[i] * dyydss[i] - dxxdss[i] * dyds[i];
		double bot = pow(dxds[i] * dxds[i] + dyds[i] * dyds[i], 1.5);

		k.push_back(top / bot);
		psi.push_back(atan2(dyds[i], dxds[i]));
	}

	return 0;
}

/**
 * @brief find clostest point on centerline of current lane to a point(x,y)
 *
 * @param point
 * @param cl centerline (can also be boundary line)
 * @param closest
 */
int closestCenterlinePoint(Point2D point, std::vector<Point2D>& cl, Point2D& closest) {

	for (int i = 1; i < cl.size(); i++) {	//IMPROVE finding correct points (x1,y1)(x2,y2) without iterating over entire centerline
		double x1 = cl[i - 1].x;
		double x2 = cl[i].x;
		double y1 = cl[i - 1].y;
		double y2 = cl[i].y;

		//compute orthogonal projection of (x,y) onto the parameterized line connecting (x1,y1) and (x2,y2)
		double l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
		double dot = (point.x - x1) * (x2 - x1) + (point.y - y1) * (y2 - y1);
		double t = dot / l2;

		if (t >= 0 && t <= 1) { //correct points (x1,y1)(x2,y2) were found (projection is in segment)
			closest.x = x1 + t * (x2 - x1);
			closest.y = y1 + t * (y2 - y1);
			return i;
		}
	}

	//x,y beyond scope of cl
	if ((pow(point.x - cl.back().x, 2) + pow(point.y - cl.back().y, 2))
		< (pow(point.x - cl.front().x, 2) + pow(point.y - cl.front().y, 2))) {

		if (cl.size() < 2) {
			closest = cl.back();
			return cl.size();
		}
		else {
			double l2 = (cl.back().x - cl[cl.size() - 2].x) * (cl.back().x - cl[cl.size() - 2].x) + (cl.back().y - cl[cl.size() - 2].y) * (cl.back().y - cl[cl.size() - 2].y);
			double dot = (point.x - cl.back().x) * (cl.back().x - cl[cl.size() - 2].x) + (point.y - cl.back().y) * (cl.back().y - cl[cl.size() - 2].y);
			double t = dot / l2;
			closest.x = cl.back().x + t * (cl.back().x - cl[cl.size() - 2].x);
			closest.y = cl.back().y + t * (cl.back().y - cl[cl.size() - 2].y);
			return cl.size();
		}

	}
	else {
		if (cl.size() < 2) {
			closest = cl.front();
			return 0;
		}
		else {
			double l2 = (cl[0].x - cl[1].x) * (cl[0].x - cl[1].x) + (cl[0].y - cl[1].y) * (cl[0].y - cl[1].y);
			double dot = (point.x - cl[0].x) * (cl[0].x - cl[1].x) + (point.y - cl[0].y) * (cl[0].y - cl[1].y);
			double t = dot / l2;
			closest.x = cl[0].x + t * (cl[0].x - cl[1].x);
			closest.y = cl[0].y + t * (cl[0].y - cl[1].y);
			return 0;
		}
		
	}

	return 0;
}

/**
 * @brief get x y  centerline-coordinates of lane
 *
 * @param lane
 * @param pos
 * @return int
 */
int getXY(osi3::Lane* l, std::vector<Point2D>& pos)
{
	/*if (l->classification().centerline_is_driving_direction()) {
		for (int i = 0; i < l->classification().centerline_size(); i++)
		{
			Point2D point(l->classification().centerline(i).x(), l->classification().centerline(i).y());
			pos.push_back(point);
		}
		return 0;
	}
	else {
		for (int i = l->classification().centerline_size()-1; i >= 0; i--)
		{
			Point2D point(l->classification().centerline(i).x(), l->classification().centerline(i).y());
			pos.push_back(point);
		}
		return 0;
	}	*/

	//temporary workaround for incorrect lane directions
	if (l->classification().centerline_is_driving_direction() && l->id().value() != 42 ) {
		for (int i = 0; i < l->classification().centerline_size(); i++)
		{
			Point2D point(l->classification().centerline(i).x(), l->classification().centerline(i).y());
			pos.push_back(point);
		}
		return 0;
	}
	else {
		for (int i = l->classification().centerline_size() - 1; i >= 0; i--)
		{
			Point2D point(l->classification().centerline(i).x(), l->classification().centerline(i).y());
			pos.push_back(point);
		}
		return 0;
	}

}

/**
 * @brief calculates width of lane
 *
 * width of lane computed at all centerline points and stored in vector 'width'
 *
 * @param cl centerline points
 * @param lane
 * @param width
 * @param groundTruth
 */
int calcWidth(std::vector<Point2D>& cl, osi3::Lane* lane, std::vector<double>& width, osi3::GroundTruth* groundTruth) {

	// find correct boundaries
	osi3::LaneBoundary lB;
	osi3::LaneBoundary rB;

	int lBId = lane->classification().left_lane_boundary_id(0).value();
	int rBId = lane->classification().right_lane_boundary_id(0).value();

	for (int i = 0; i < groundTruth->lane_boundary_size(); i++)
	{
		if (groundTruth->lane_boundary(i).id().value() == lBId)
		{
			lB = groundTruth->lane_boundary(i);
		}
		if (groundTruth->lane_boundary(i).id().value() == rBId)
		{
			rB = groundTruth->lane_boundary(i);
		}
	}

	std::vector<Point2D> lBPoints, rBPoints;

	for (int i = 0; i < lB.boundary_line_size(); i++) {

		Point2D point(lB.boundary_line(i).position().x(), lB.boundary_line(i).position().y());
		lBPoints.push_back(point);
	}
	for (int i = 0; i < rB.boundary_line_size(); i++) {

		Point2D point(rB.boundary_line(i).position().x(), rB.boundary_line(i).position().y());
		rBPoints.push_back(point);
	}

	// for each horizon knot, calculate the distance between the
	//corresponding  x,y point and the lane boundary
	for (int i = 0; i < cl.size(); i++) {

		Point2D rPoint, lPoint;
		closestCenterlinePoint(cl[i], lBPoints, lPoint);
		closestCenterlinePoint(cl[i], rBPoints, rPoint);

		double w = sqrt(pow(lPoint.x - rPoint.x, 2) + pow(lPoint.y - rPoint.y, 2));
		width.push_back(w);
	}
	return 0;
}


/**
 * @brief ds along curved centerline starting at (startX,startY) and ending at (endX,endY)
 *
 *
 * @param start
 * @param end
 * @param cl centerline
 */
double xy2s(Point2D start, Point2D end, std::vector<Point2D>& cl) {

	double s = 0;
	double ds = 0;
	int startIdx = -1, endIdx = -1;
	bool foundS = false, foundE = false;
	double sum_s = 0, sum_e = 0;


	for (int i = 1; i < cl.size(); i++) {	//IMPROVE finding correct points (x1,y1)(x2,y2) without iterating over entire centerline
		double x1 = cl[i - 1].x;
		double x2 = cl[i].x;
		double y1 = cl[i - 1].y;
		double y2 = cl[i].y;

		//compute orthogonal projection of (x,y) onto the parameterized line connecting (x1,y1) and (x2,y2)
		double l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
		double dot = (start.x - x1) * (x2 - x1) + (start.y - y1) * (y2 - y1);
		double t_s = dot / l2;
		sum_s += t_s;

		l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
		dot = (end.x - x1) * (x2 - x1) + (end.y - y1) * (y2 - y1);
		double t_e = dot / l2;
		sum_e += t_e;

		if (t_s >= 0 && t_s <= 1) { //correct points (x1,y1)(x2,y2) were found (projection is in segment)
			ds = sqrt(pow(start.x - cl[i].x, 2) + pow(start.y - cl[i].y, 2));
			s += ds;
			startIdx = i;
			foundS = true;
		}

		if (t_e >= 0 && t_e <= 1) { //correct points (x1,y1)(x2,y2) were found (projection is in segment)
			ds = sqrt(pow(end.x - cl[i - 1].x, 2) + pow(end.y - cl[i - 1].y, 2));
			endIdx = i - 1;
			s += ds;
			foundE = true;
		}
		
	}

	//start, end beyond scope of cl. 
	//Since cl is always in driving direction; start/ end can be before the scope of cl, in cl (then they were found before) or after cl ends.
	if (!foundS && !foundE) {
		if (sum_s < 0 && sum_e > 0) {
			//start before cl, end after
			s += sqrt(pow(start.x - cl.front().x, 2) + pow(start.y - cl.front().y, 2));
			startIdx = -1;
			s += sqrt(pow(end.x - cl.back().x, 2) + pow(end.y - cl.back().y, 2));
			endIdx = cl.size() - 1;

		}
		else {
			//start after, end before should not happen
			//both after or both before:
			return sqrt((start.x - end.x) * (start.x - end.x) + (start.y - end.y) * (start.y - end.y));
		}
	}
	else if (!foundS && foundE) { //start before cl, end inside
		s += sqrt(pow(start.x - cl.front().x, 2) + pow(start.y - cl.front().y, 2));
		startIdx = -1;
	}
	else if (foundS && !foundE) { //start inside cl, end after
		s += sqrt(pow(end.x - cl.back().x, 2) + pow(end.y - cl.back().y, 2));
		endIdx = cl.size() - 1;
	}
		

	for (int i = startIdx + 1; i <= endIdx; i++)
	{
		double dx = cl[i].x - cl[i - 1].x;
		double dy = cl[i].y - cl[i - 1].y;

		s += sqrt(pow(dx, 2) + pow(dy, 2));
	}

	return s;
}



/**
 * @brief find a laneID in groundTruth.lane and return pointer
 *
 * @param ID
 * @param groundTruth
 */
osi3::Lane* findLane(int id, osi3::GroundTruth* groundTruth) {
	for (int i = 0; i < groundTruth->lane_size(); i++) {
		if (groundTruth->lane(i).id().value() == id) return (groundTruth->mutable_lane(i));
	}
	return nullptr;
}

/**
 * @brief find a laneID in groundTruth.lane and return its index 
 *
 * @param groundTruth
 * @param ID
 */
int findLaneId(osi3::GroundTruth* groundTruth, int id) {
	for (int i = 0; i < groundTruth->lane_size(); i++) {
		if (groundTruth->lane(i).id().value() == id) return i;
	}
	return -1;
}

/**
 * @brief map OSI lane IDs to corresponding agent_model lane IDs
 *
 * using unordered_map: keys are OSI lane IDs, values are agent_model lane IDs
 * @param groundTruth
 * @param mapping map
 * @param egoLanePtr
 */
void mapLanes(osi3::GroundTruth* groundTruth, std::unordered_map<int,int>& mapping, osi3::Lane* egoLanePtr) {

	osi3::Lane* current = egoLanePtr;
	osi3::Lane* temp = nullptr;
	int rightLaneCount = 0;
	int leftLaneCount = 0;

	do {//first iteration assigns egolane the id 0
		mapping[current->id().value()] = rightLaneCount--;
		
		if (current->classification().right_adjacent_lane_id_size() > 0) {
			current = findLane(current->classification().right_adjacent_lane_id(0).value(), groundTruth);
		}
		else {
			current = nullptr;
		}
	} while (current != nullptr);


	// reset current and do left lanes
	if(egoLanePtr->classification().left_adjacent_lane_id_size() > 0)
		current = findLane(egoLanePtr->classification().left_adjacent_lane_id(0).value(), groundTruth);

	while (current != nullptr) {
		mapping[current->id().value()] = ++leftLaneCount;

		if (current->classification().left_adjacent_lane_id_size() > 0) {
			current = findLane(current->classification().left_adjacent_lane_id(0).value(), groundTruth);
		}
		else {
			current = nullptr;
		}
	}


	//not all lanes are adjascent to EgoLane, assign arbitrary ID to the rest (using leftLaneCount so IDs are positive and not used multiple times)
	for (int i = 0; i < groundTruth->lane_size(); i++) {

		if (mapping.find(groundTruth->lane(i).id().value()) == mapping.end()) {
			//lane has not been mapped already
			mapping[groundTruth->lane(i).id().value()] = ++leftLaneCount;
		}
	}

}






/**
 * @brief creates adjacency list corresponding to lane_pairings
 *
 * @param groundTruth
 * @param adjacency 
 */
void createGraph(osi3::GroundTruth* groundTruth, std::vector<int> adj[]) {
	
	for (int i = 0; i < groundTruth->lane_size(); i++) {
		
		for (int j = 0; j < groundTruth->lane(i).classification().lane_pairing_size(); j++) {

			if (groundTruth->lane(i).classification().lane_pairing(j).antecessor_lane_id().value()== groundTruth->lane(i).id().value())
				adj[i].push_back(findLaneId(groundTruth,groundTruth->lane(i).classification().lane_pairing(j).successor_lane_id().value()));

			else adj[findLaneId(groundTruth, groundTruth->lane(i).classification().lane_pairing(j).antecessor_lane_id().value())].push_back(i);
		}
	}
}

/**
*  @brief a modified version of BFS that determines a path from src to dest and
*  stores predecessor of each vertex in array pred
*
*/
bool BFS(std::vector<int> adj[], int src, int dest, int num_vertices,
	int pred[])
{
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
				if (adj[u][i] == dest)
					return true;
			}
		}
	}

	return false;
}


/**
 * @brief determines lanes along Trajectory passed by TrafficCommand
 *
 * @param SensorView
 * @param TrafficCommand
 * @param futureLanes
 */
void futureLanes(osi3::SensorView& sensorView, osi3::TrafficCommand& commandData, std::vector<int>& futureLanes) {

	osi3::GroundTruth* groundTruth = sensorView.mutable_global_ground_truth();
	osi3::MovingObject host;
	int start;
	int dest;

	//find Host
	for (int i = 0; i < groundTruth->moving_object_size(); i++) {
		if (groundTruth->moving_object(i).id().value() == sensorView.host_vehicle_id().value())
			host = groundTruth->moving_object(i);
	}

	//needs an initial assigned lane that is unambiguous
	if (host.assigned_lane_id_size() < 2) {
		start = findLaneId(groundTruth, host.assigned_lane_id(0).value());
	}
	else {
		//determine assigned lane TODO
	}

	//TrajectoryAction contains desired Trajectory 
	osi3::FollowTrajectoryAction action;
	action.CopyFrom(commandData.action(0).follow_trajectory_action ());	
	Point2D destination(action.trajectory_point(action.trajectory_point_size() - 1).position().x(),
		action.trajectory_point(action.trajectory_point_size() - 1).position().y());

	//determine last lane
	std::vector<Point2D> centerline;
	double distance = INFINITY;
	int destId = 127;

	for (int i = 0; i < groundTruth->lane_size(); i++) {
		centerline.clear();
		getXY((groundTruth->mutable_lane(i)), centerline);

		Point2D closest;
		closestCenterlinePoint(destination, centerline, closest);
		double d = (closest.x - destination.x) * (closest.x - destination.x) + (closest.y - destination.y) * (closest.y - destination.y);

		if (distance > d) {
			distance = d;
			destId = groundTruth->lane(i).id().value();
			
		}
	}

	dest = findLaneId(groundTruth, destId);

	//Graph setup
	//create adjacency list for graph representing lane connections
	std::vector<int> adj[groundTruth->lane_size()];
	
	createGraph(groundTruth, adj);

	//remove possible dublicates in adj[i]
	for (int i = 0; i < groundTruth->lane_size(); i++) {
		std::sort(adj[i].begin(), adj[i].end());
		adj[i].erase(unique(adj[i].begin(), adj[i].end()), adj[i].end());
	}

	int pred[groundTruth->lane_size()];
	

	if (BFS(adj, start, dest, groundTruth->lane_size(), pred) == false) {
		
		futureLanes.push_back(groundTruth->lane(start).id().value());
		return;
	}

	// vector path stores the shortest path 
	std::vector<int> path;
	int crawl = dest;
	path.push_back(crawl);
	while (pred[crawl] != -1) {
		path.push_back(pred[crawl]);
		crawl = pred[crawl];
	}

	for (int i = path.size() - 1; i >= 0; i--)
		futureLanes.push_back(groundTruth->lane(path[i]).id().value());

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
 * dStart, dEnd contain direction vectors in Start and End point. Spline points are generated in vector cl
 *
 */
void spline3(Point2D start, Point2D end, Point2D dStart, Point2D dEnd, std::vector<Point2D>& cl) {

	// transform into local coordinate system (rotation so that new x-axis passes through start and end)
	Point2D s, e, ds, de, se, der;
	se.x = end.x - start.x;
	se.y = end.y - start.y;

	//angle between coordinate systems. Ensured that start --> end is positive x-direction in new coordiante system
	double phi = (se.y > 0) ? std::acos(se.x / sqrt(se.x * se.x + se.y * se.y)) : -1 * std::acos(se.x / sqrt(se.x * se.x + se.y * se.y));
	

	transform(start, s, phi);
	transform(end, e, phi);
	transform(dStart, ds, phi);
	transform(dEnd, de, phi);

	//maximum derivative is set to 50 ~ 89 degrees
	der.x = abs(ds.x) > abs(ds.y / 50.0) ? ds.y / ds.x : std::copysignf(1, ds.x * ds.y) * 50;
	der.y = abs(de.x) > abs(de.y / 50.0) ? de.y / de.x : std::copysignf(1, de.x * de.y) * 50;
	
	// creating spline of form f(x)= a + bx + cx^2 + dx^3, der.x contains f'(start.x), der.y contains f'(end.x)

	double tmp = 1.0 / (pow(s.x - e.x, 3));

	double a = -tmp * (der.y * e.x * pow(s.x, 3) + der.x * e.x * e.x * s.x * s.x - der.y * e.x * e.x * s.x * s.x -
		der.x * pow(e.x, 3) * s.x - pow(s.x, 3) * e.y + 3 * e.x * s.x * s.x * e.y - 3 * e.x * e.x * s.x * s.y + pow(e.x, 3) * s.y);

	double b = -tmp * (-der.y * pow(s.x, 3) - 2 * der.x * e.x * s.x * s.x - der.y * e.x * s.x * s.x + der.x * e.x * e.x * s.x +
		2 * der.y * e.x * e.x * s.x + der.x * pow(e.x, 3) + 6 * e.x * s.x * s.y - 6 * e.x * s.x * e.y);

	double c = -tmp * (der.x * s.x * s.x + 2 * der.y * s.x * s.x + der.x * e.x * s.x - der.y * e.x * s.x - 2 * der.x * e.x * e.x -
		der.y * e.x * e.x - 3 * s.x * s.y + 3 * s.x * e.y - 3 * e.x * s.y + 3 * e.x * e.y);

	double d = -tmp * (-der.x * s.x - der.y * s.x + der.x * e.x + der.y * e.x + 2 * s.y - 2 * e.y);

	const float maxError = 0.035;

	// generate "centerline" points.
	double x = s.x;
	double y = 0;

	while (x < e.x) {
		double kappa = abs(2 * c + 6 * d * x);
		double delta_x = (kappa < 0.001) ? 1.0 : 2 * sqrt(2 * maxError * 1 / kappa - maxError * maxError);
		if (2 * maxError * 1 / kappa < maxError * maxError) delta_x = 1 / kappa;
		x += delta_x;
		y = a + b * x + c * x * x + d * x * x * x;
		if (x < e.x)cl.push_back(Point2D(x, y));
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
 * @brief create horizon knots according to distances in ds vector
 *
 * @param centerline
 * @param s
 * @param psi
 * @param kappa
 * @param ds to horizon points
 * @param horizon
 * @param environment
 * @param nextS
 * @param index in ds vector
 *
 */
int horizonKnots(std::vector<Point2D> cl, std::vector<double>& s, std::vector<double>& psi, std::vector<double>& k, std::vector<double>& ds, std::vector<Point2D>& horizon_global,
	agent_model::Horizon& horizon, double& nextS, int& j) {

	bool stop = false;
	std::vector<double> interpolation(agent_model::NOH, 0);
	
		for (int i = 1; i < s.size() && !stop; i++) {

			while ((s[i - 1] <= nextS) && (s[i] >= nextS)) {
				
				if (s[i - 1] == s[i]) s[i] += 0.3;

				interpolation[j] = (nextS - s[i - 1]) / (s[i] - s[i - 1]);

				Point2D hKnot(cl[i - 1].x + interpolation[j] * (cl[i].x - cl[i - 1].x),
					cl[i - 1].y + interpolation[j] * (cl[i].y - cl[i - 1].y));

				horizon_global.push_back(hKnot);


				horizon.x[j] = horizon_global.back().x;
				horizon.y[j] = horizon_global.back().y;
				horizon.ds[j] = ds[j];

				horizon.psi[j] = psi[i - 1] + interpolation[j] * (psi[i] - psi[i - 1]);
				horizon.kappa[j] = k[i - 1] + interpolation[j] * (k[i] - k[i - 1]);

				j++;
				if (j >= agent_model::NOH) {
					stop = true;
					return 0;
				}
				nextS += ds[j] - ds[j - 1];
			}
		}
	
	return 0;

}