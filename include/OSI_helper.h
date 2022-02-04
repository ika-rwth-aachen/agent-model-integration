#pragma once
#include <cmath>
#include <numeric>
#include <vector>
#include <unordered_map>
#include <list>
#include "osi_sensorview.pb.h"
#include "Interface.h"
#include <iostream>
#include <algorithm>

/**
 * @brief computes gradients with finite differences
 *
 * @param x
 * @param y
 * @param order of differentiation
 * @return gradient at every point
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
		for (int i = 1; i < n - 1; i++) {
			double h1 = (x[i] - x[i - 1]), h2 = (x[i + 1] - x[i]);
			res[i] = (2.0 / (h1 + h2)) * ((y[i + 1] - y[i]) / h2 - (y[i] - y[i - 1]) / h1);
		}

		// calculate gradient at edges with one sided differences
		//res[0] = (y[2] - 2 * y[1] + y[0]) / pow(x[1] - x[0], 2);
		double h1 = (x[1] - x[0]), h2 = (x[2] - x[1]);
		res[0] = (2.0 / (h1 + h2)) * ((y[2] - y[1]) / h2 - (y[1] - y[0]) / h1);
		h1 = (x[n - 1] - x[n - 2]), h2 = (x[n - 2] - x[n - 3]);
		res[n - 1] = (2.0 / (h1 + h2)) * ((y[n - 1] - y[n - 2]) / h1 - (y[n - 2] - y[n - 3]) / h2);
	}

	return res;
}


/**
 * @brief calculates s (distance), psi (global angle w.r.t. x-axis), kappa (curvature) from xy coordinates
 *
 *
 * @param pos position points in global xy-coordinates
 * @param s distance result
 * @param psi angle result
 * @param k curvature result
 * @return int success
 */
int xy2curv(std::vector<Point2D> pos, std::vector<double>& s, std::vector<double>& psi, std::vector<double>& k)
{
	// calculate s from xy coordinates
	s.push_back(0);
	std::vector<double> x(1, pos[0].x), y(1, pos[0].y);

	for (int i = 1; i < pos.size(); i++)
	{
		double dx = pos[i].x - pos[i - 1].x;
		double dy = pos[i].y - pos[i - 1].y;
		x.push_back(pos[i].x);
		y.push_back(pos[i].y);

		s.push_back(s.back() + sqrt(pow(dx, 2) + pow(dy, 2)));
		double p = atan2(dy, dx);
		//psi.push_back(p >= 0 ? p : p + 2 * M_PI);
		psi.push_back(p);
	}


	// calculate psi and kappa
	std::vector<double> dxds = gradient(s, x, 1);
	std::vector<double> dyds = gradient(s, y, 1);
	std::vector<double> dxxdss = gradient(s, x, 2);
	std::vector<double> dyydss = gradient(s, y, 2);


	for (int i = 0; i < pos.size(); i++)
	{
		if (pos.size() > 2) {
			double top = dxds[i] * dyydss[i] - dxxdss[i] * dyds[i];
			double bot = pow(dxds[i] * dxds[i] + dyds[i] * dyds[i], 1.5);
			k.push_back(top / bot);
		}
		else {
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
 * @return index of the next centerline point following the projected point. 0 when before scope of cl, cl.size() when after
 */
int closestCenterlinePoint(const Point2D point, const std::vector<Point2D>& cl, Point2D& closest) {

	double min_dist=INFINITY;
	int min_i=0;
	Point2D tmp;

	for (int i = 1; i < cl.size(); i++) {	
		double x1 = cl[i - 1].x;
		double x2 = cl[i].x;
		double y1 = cl[i - 1].y;
		double y2 = cl[i].y;

		// compute orthogonal projection of (x,y) onto the parameterized line connecting (x1,y1) and (x2,y2)
		double l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
		double dot = (point.x - x1) * (x2 - x1) + (point.y - y1) * (y2 - y1);
		double t = dot / l2;			
		if (t >= 0 && t <= 1) { //correct points (x1,y1)(x2,y2) were found (projection is in segment)
			tmp.x = x1 + t * (x2 - x1);
			tmp.y = y1 + t * (y2 - y1);	
			double dist = pow(tmp.x - point.x ,2) + pow(tmp.y - point.y ,2);
			if(dist < min_dist) {
				min_dist = dist;
				closest = tmp;
				min_i = i;
			}
		} else {
			double distI = pow(x2 - point.x ,2) + pow(y2 - point.y ,2);
			if(distI < min_dist) {
				closest = cl[i];
				min_dist = distI;
				min_i = i;
			}
		}
	}
	double distEnd = pow(point.x - cl.back().x, 2) + pow(point.y - cl.back().y, 2);
	double distStart = pow(point.x - cl.front().x, 2) + pow(point.y - cl.front().y, 2);	
	if (min_i > 0 && (min_dist < distEnd || min_dist < distStart)) return min_i;

	// x,y beyond scope of cl
	if (distEnd < distStart) {

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
 * @param p1p2 a vector of four points that describe both road ends
 * @param pos the centerline with a gap that should be filled by this function
 * @param idx index within pos where the gap is located
 * 
*/

int calcXYgap(std::vector<Point2D> p1p2, std::vector<Point2D>& pos, int idx)
{
	// gap start/end plus second last point, respectively
	double x1 = p1p2[1].x;
	double y1 = p1p2[1].y;
	double x1_h = p1p2[0].x;
	double y1_h = p1p2[0].y;
	double x2 = p1p2[2].x;
	double y2 = p1p2[2].y;
	double x2_h = p1p2[3].x;
	double y2_h = p1p2[3].y;
	
	// help/intermediate variables
	double dx = abs(x2-x1);
	double dy = abs(y2-y1);
	double ang1 = atan2(y1 - y1_h, x1 - x1_h);
	double ang2 = atan2(y2_h - y2, x2_h - x2);

	// manual coefficient calculation after paper+pen
	// x(0) = x1, x(dx) = x2, x'(0) = cos(ang1), x'(dx) = cos(ang2)
	// y similar but with sin()
	// Remark: Those calculation are the solution to an ODE of the form A \ b = c
	double ca_x = (-2/pow(dx,3))*(dx-dx*cos(ang1)) + (1/pow(dx,2))*(cos(ang2)-cos(ang1));
	double cb_x = (3/pow(dx,2))*(dx-dx*cos(ang1)) + (-1/dx)*(cos(ang2)-cos(ang1));
	double cc_x = cos(ang1);
	double cd_x = x1;
	double ca_y = (-2/pow(dy,3))*(dy-dy*sin(ang1)) + (1/pow(dy,2))*(sin(ang2)-sin(ang1));
	double cb_y = (3/pow(dy,2))*(dy-dy*sin(ang1)) + (-1/dy)*(sin(ang2)-sin(ang1));
	double cc_y = sin(ang1);
	double cd_y = y1;

	std::vector<Point2D> gapXY;
	int N = 30; //number of interpolated points
	double ddx = dx/(N-1);
	double ddy = dy/(N-1);
	double tx = 0;
	double ty = 0;
	for (int i=0; i<N; i++)
	{
		Point2D tmp;
		// calculate parameter polynomial function 
		tmp.x = ca_x*pow(tx, 3) + cb_x*pow(tx, 2) + cc_x*tx + cd_x;
		tmp.y = ca_y*pow(ty, 3) + cb_y*pow(ty, 2) + cc_y*ty + cd_y;

		tx += ddx;
		ty += ddy;
		// add point
		gapXY.push_back(tmp);
	}

	pos.insert(pos.begin() + idx, gapXY.begin(), gapXY.end());

	return 0;
}

/**
 * @brief get vector of x y  centerline-coordinates of a lane
 *
 * @param lane pointer
 * @param pos result vector
 * @return success
 */
int getXY(osi3::Lane* l, std::vector<Point2D>& pos)
{
	if (l->classification().centerline_is_driving_direction()) {
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
 * @brief calculates width of lane at a point if the lane has adjascent lanes
 *
 *
 * @param point width at this point
 * @param lane lane corresponding to point
 * @param groundTruth
 * @return width
 */
double calcWidth(const Point2D point, osi3::Lane* lane, osi3::GroundTruth* groundTruth) {

	std::vector<int> lBIds, rBIds;
	std::vector<Point2D> lBPoints, rBPoints;

	//std::cout << "left:"<< lane->classification().left_lane_boundary_id_size()<<" right:"<< lane->classification().right_lane_boundary_id_size()<<"\n";
	for (int i = 0; i < lane->classification().left_lane_boundary_id_size(); i++) {
		lBIds.push_back(lane->classification().left_lane_boundary_id(i).value());
	}
	for (int i = 0; i < lane->classification().right_lane_boundary_id_size(); i++) {
		rBIds.push_back(lane->classification().right_lane_boundary_id(i).value());
	}

	if (lBIds.size() == 0 || rBIds.size() == 0) return 0;
	

	for (int i = 0; i < groundTruth->lane_boundary_size(); i++)
	{
		if (find(lBIds.begin(), lBIds.end(), groundTruth->lane_boundary(i).id().value()) != lBIds.end())
		{
			for (int k = 0; k < groundTruth->lane_boundary(i).boundary_line_size(); k++) {
				Point2D tpoint(
					groundTruth->lane_boundary(i).boundary_line(k).position().x(),
					groundTruth->lane_boundary(i).boundary_line(k).position().y());
				lBPoints.push_back(tpoint);
			}
		}
		else if (find(rBIds.begin(), rBIds.end(), groundTruth->lane_boundary(i).id().value()) != rBIds.end())
		{
			for (int k = 0; k < groundTruth->lane_boundary(i).boundary_line_size(); k++) {

				Point2D tpoint(
					groundTruth->lane_boundary(i).boundary_line(k).position().x(),
					groundTruth->lane_boundary(i).boundary_line(k).position().y());
				rBPoints.push_back(tpoint);
			}
		}
	}

	Point2D rPoint, lPoint;
	closestCenterlinePoint(point, lBPoints, lPoint);
	closestCenterlinePoint(point, rBPoints, rPoint);

	return sqrt(pow(lPoint.x - rPoint.x, 2) + pow(lPoint.y - rPoint.y, 2));
}

/**
 * @brief ds along xy starting at (startX,startY) and ending at (endX,endY) with correct sign
 *
 * @param start point
 * @param end point
 * @param cl centerline
 * @return resulting distance
 */
double xy2s_sgn(const Point2D start, const Point2D end, const std::vector<Point2D>& cl, double startPsi) {
	double s = 0;
	Point2D startCl, endCl;
	int startIdx = closestCenterlinePoint(start, cl, startCl);
	int endIdx = closestCenterlinePoint(end, cl, endCl);
	if (cl.size() == 2) 
	{
		double startEndPsi = atan2(end.y-start.y, end.x-start.x);
		//std::cout << "target dir: " << startEndPsi-startPsi << "\n";
		double dist =  sqrt( pow(end.x-start.x, 2) + pow(end.y-start.y, 2) );
		if (abs(startEndPsi-startPsi) > 1.5)
			return -dist;
		else
			return dist;
	}
	else
	{
		int startIdx = closestCenterlinePoint(start, cl, startCl);
		int endIdx = closestCenterlinePoint(end, cl, endCl);

		if(startIdx < endIdx)
		{
			for (int i = startIdx + 1; i <= endIdx; i++)
			{
				double dx = cl[i].x - cl[i - 1].x;
				double dy = cl[i].y - cl[i - 1].y;

				s += sqrt(dx * dx + dy * dy);
			}
			s += sqrt( pow(start.x-cl[startIdx].x, 2) + pow(start.y-cl[startIdx].y, 2) );
			s -= sqrt( pow(end.x-cl[endIdx].x, 2) + pow(end.y-cl[endIdx].y, 2) );
			return s;
		}
		else if(startIdx == endIdx)
		{
			// unsure about sign...
			return sqrt( pow(start.x-end.x, 2) + pow(start.y-end.y, 2) );
		}
		else
		{
			for (int i = endIdx + 1; i <= startIdx; i++)
			{
				double dx = cl[i].x - cl[i - 1].x;
				double dy = cl[i].y - cl[i - 1].y;

				s -= sqrt(dx * dx + dy * dy);
			}
			s += sqrt( pow(start.x-cl[startIdx].x, 2) + pow(start.y-cl[startIdx].y, 2) );
			s -= sqrt( pow(end.x-cl[endIdx].x, 2) + pow(end.y-cl[endIdx].y, 2) );
			return s;
		}
	}
	
}

/**
 * @brief ds along centerline starting at (startX,startY) and ending at (endX,endY)
 *
 *
 * @param start point
 * @param end point
 * @param cl centerline
 * @return resulting distance
 */
double xy2s(const Point2D start, const Point2D end, const std::vector<Point2D>& cl) {
	double s = 0;
	Point2D startCl, endCl;
	int startIdx = closestCenterlinePoint(start, cl, startCl);
	int endIdx = closestCenterlinePoint(end, cl, endCl);

	if (startIdx == endIdx)
		return sqrt((endCl.x - startCl.x) * (endCl.x - startCl.x) + (endCl.y - startCl.y) * (endCl.y - startCl.y));

	if (startIdx == 0) {
		// starting point before scope of centerline
		s += sqrt((cl.front().x - startCl.x) * (cl.front().x - startCl.x) + (cl.front().y - startCl.y) * (cl.front().y - startCl.y));
	}
	else if (startIdx < cl.size()) {
		if (endIdx == 0 || startIdx > endIdx) //this means end before cl, but start within/after cl. 
			return xy2s(end, start, cl);
		// start/end points are most likely between two centerline points. Add distance to closest centerline point for start.
		s += sqrt((cl[startIdx].x - startCl.x) * (cl[startIdx].x - startCl.x) + (cl[startIdx].y - startCl.y) * (cl[startIdx].y - startCl.y));
	}
	if (endIdx >= cl.size()) {
		//ending point after scope of centerline
		s += sqrt((endCl.x - cl.back().x) * (endCl.x - cl.back().x) + (endCl.y - cl.back().y) * (endCl.y - cl.back().y));
	}
	else if (endIdx != 0) {
		// start/end points are most likely between two centerline points. Add distance to closest centerline point for end.
		if (startIdx >= cl.size() || startIdx > endIdx) //this means start after cl, but end within/before cl. 
			return xy2s(end, start, cl);
		s += sqrt((endCl.x - cl[endIdx - 1].x) * (endCl.x - cl[endIdx - 1].x) + (endCl.y - cl[endIdx - 1].y) * (endCl.y - cl[endIdx - 1].y));
	}

	for (int i = startIdx + 1; i < endIdx; i++)
	{
		double dx = cl[i].x - cl[i - 1].x;
		double dy = cl[i].y - cl[i - 1].y;

		s += sqrt(dx * dx + dy * dy);
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
 * @param futureLanes vector of all lanes along the host's path
 */
void mapLanes(osi3::GroundTruth* groundTruth, std::unordered_map<int, int>& mapping, osi3::Lane* egoLanePtr, std::vector<int> futureLanes) {

	osi3::Lane* current = nullptr;
	int rightLaneCount = 0;
	int leftLaneCount = 0;

	//assigns all lanes along the path the id 0
	for (int i = 0; i < futureLanes.size(); i++)
		mapping[futureLanes[i]] = rightLaneCount;

	if(egoLanePtr->classification().right_adjacent_lane_id_size() > 0)
		current = findLane(egoLanePtr->classification().right_adjacent_lane_id(0).value(), groundTruth);

	while (current != nullptr) {
		
		mapping[current->id().value()] = --rightLaneCount;
		
		if (current->classification().right_adjacent_lane_id_size() > 0) {
			current = findLane(current->classification().right_adjacent_lane_id(0).value(), groundTruth);
		}
		else {
			current = nullptr;
		}
	}


	// reset current and do left lanes
	if (egoLanePtr->classification().left_adjacent_lane_id_size() > 0)
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
			if (groundTruth->lane(i).classification().centerline_is_driving_direction()) {
				//if (groundTruth->lane(i).classification().lane_pairing(j).successor_lane_id().value() == groundTruth->lane(i).id().value())
				//	adj[i].push_back(findLaneId(groundTruth, groundTruth->lane(i).classification().lane_pairing(j).antecessor_lane_id().value()));
				if (groundTruth->lane(i).classification().lane_pairing(j).antecessor_lane_id().value() == groundTruth->lane(i).id().value())
					adj[i].push_back(findLaneId(groundTruth, groundTruth->lane(i).classification().lane_pairing(j).successor_lane_id().value()));

			}
			else {
				//if (groundTruth->lane(i).classification().lane_pairing(j).antecessor_lane_id().value() == groundTruth->lane(i).id().value())
				//	adj[i].push_back(findLaneId(groundTruth, groundTruth->lane(i).classification().lane_pairing(j).successor_lane_id().value()));
				if (groundTruth->lane(i).classification().lane_pairing(j).successor_lane_id().value() == groundTruth->lane(i).id().value())
					adj[i].push_back(findLaneId(groundTruth, groundTruth->lane(i).classification().lane_pairing(j).antecessor_lane_id().value()));

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

int interpolateXY2value(std::vector<double> y, std::vector<Point2D> xy, Point2D pos)
{
	Point2D dummy;
	int i = closestCenterlinePoint(pos, xy, dummy);
	double x1 = xy[i - 1].x;
	double x2 = xy[i].x;
	double y1 = xy[i - 1].y;
	double y2 = xy[i].y;

	//compute orthogonal projection of (x,y) onto the parameterized line connecting (x1,y1) and (x2,y2)
	double l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	double dot = (pos.x - x1) * (x2 - x1) + (pos.y - y1) * (y2 - y1);
	double t = dot / l2;
	
	return y[i - 1] + t * (y[i] - y[i - 1]);
/*

	for (i = 1; i < elPoints.size(), !set; i++) {
		double x1 = elPoints[i - 1].x;
		double x2 = elPoints[i].x;
		double y1 = elPoints[i - 1].y;
		double y2 = elPoints[i].y;

		//compute orthogonal projection of (x,y) onto the parameterized line connecting (x1,y1) and (x2,y2)
		double l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
		double dot = (egoClPoint.x - x1) * (x2 - x1) + (egoClPoint.y - y1) * (y2 - y1);
		double t = dot / l2;

		if (t >= 0 && t <= 1) { //correct points (x1,y1)(x2,y2) were found (egoClPoint is in segment)
			// global psi of ego minus psi of lane at current location (interpolated)
			input.vehicle.psi = egoPsi - (psi[i - 1] + t * (psi[i] - psi[i - 1]));
			//std::cout << "vehicle psi " << input.vehicle.psi * 180 / 3.14159 << "=" << egoBase.orientation().yaw() * 180 / 3.14159 << "-"<< (psi[i - 1] + t * (psi[i] - psi[i - 1])) * 180 / 3.14159 << std::endl;
			set = true;
		}
	}*/
}

/**
 * @brief determines lane closest to a point according to cartesian distance
 * 
 * caution: some lanes may share points causing the result to be unreliable. Only use when necessary.
 *
 * @param groundTruth
 * @param point
 * @return id of lane
 */
int closestLane(osi3::GroundTruth* groundTruth, const Point2D& point) {
	std::vector<Point2D> centerline;
	double distance = INFINITY;
	int destId = 127;

	for (int i = 0; i < groundTruth->lane_size(); i++) {
		centerline.clear();
		osi3::Lane cur_lane;
		cur_lane.CopyFrom(groundTruth->lane(i));
		getXY(&cur_lane, centerline);

		Point2D closest;
		int idx =closestCenterlinePoint(point, centerline, closest);
		double d = (closest.x - point.x) * (closest.x - point.x) + (closest.y - point.y) * (closest.y - point.y);
		//std::cout <<cur_lane.id().value() << ":"<< d << " idx:" << idx<< "\n";
		if (distance > d) {
			distance = d;
			destId = cur_lane.id().value();
			
		}
	}
	
	return destId;
}

/**
 * @brief determines lanes along Trajectory from the lane with startIdx to the (x,y) point destination.
 * 
 * 
 * @param groundTruth
 * @param start index of starting lane (index in groundTruth->lane field)
 * @param destination point to be reached
 * @param futureLanes result
 */
void futureLanes(osi3::GroundTruth* groundTruth, const int& startIdx, const Point2D& destination, std::vector<int>& futureLanes) {

	osi3::MovingObject host;
	// destination INDEX
	int destIdx;

	std::cout << "destination :" << destination.x << "," << destination.y << "\n";
	int destId = closestLane(groundTruth, destination);
	destIdx = findLaneId(groundTruth, destId);
	std::cout << " on lane " << destId << std::endl;
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

	if (BFS(adj, startIdx, destIdx, groundTruth->lane_size(), pred) == false) {
		//add starting lane into futureLanes if it is not already contained
		if (futureLanes.empty()||(!futureLanes.empty() && futureLanes.back() != groundTruth->lane(startIdx).id().value())) 
			futureLanes.push_back(groundTruth->lane(startIdx).id().value());
		return;
	}

	// vector path stores the shortest path 
	std::vector<int> path;
	int crawl = destIdx;
	path.push_back(crawl);
	while (pred[crawl] != -1) {
		path.push_back(pred[crawl]);
		crawl = pred[crawl];
	}

	for (int i = path.size() - 1; i >= 0; i--) {
		if (!futureLanes.empty() && futureLanes.back() == groundTruth->lane(path[i]).id().value())
			continue;
		futureLanes.push_back(groundTruth->lane(path[i]).id().value());
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
 * dStart, dEnd contain direction vectors in Start and End point. Spline points are generated in vector cl
 * 
 * @param start point
 * @param end point
 * @param dStart heading at start
 * @param dEnd heading at end
 * @param cl resulting spline
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

