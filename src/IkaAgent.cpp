 /*
 * Author: Daniel Becker
 *
 * (C) 2019 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */ 
#include "IkaAgent.h"
#include <fstream>
#include <iostream>
#include "OSI_helper.h"

constexpr double EPSILON = 0.000000000000001;

using Point2D = agent_model::Position;

void IkaAgent::init()
{
	lastS = 0;
	lastPosition.x = 0;
	lastPosition.y = 0;
	horizonTHW = 20;

	drParam = getParameters();

	drParam->velocity.a = 2.0;
	drParam->velocity.b = -2.0;
	drParam->velocity.thwMax = 10.0;
	drParam->velocity.delta = 4.0;
	drParam->velocity.deltaPred = 3.0;
	drParam->velocity.vComfort = 50.0 / 3.6;
	drParam->velocity.ayMax = 1.3;

	// stop control
	drParam->stop.T = 2.0;
	drParam->stop.TMax = 10.0;
	drParam->stop.tSign = 0.5;
	drParam->stop.pedalDuringStanding = -0.3;

	// following
	drParam->follow.dsStopped = 2.0;
	drParam->follow.thwMax = 10.0;
	drParam->follow.timeHeadway = 1.8;

	// steering
	drParam->steering.thw[0] = 1.0;
	drParam->steering.thw[1] = 2.0;
	drParam->steering.dsMin[0] = 5.0;
	drParam->steering.dsMin[1] = 10.0;
	drParam->steering.D[0] = 0.0;
	drParam->steering.D[1] = 0.0;

	AgentModel::init();
	
}


int IkaAgent::step(double time, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out)
{
	//----------------
	std::ofstream output("debug.txt", std::ofstream::out | std::ofstream::app);
	
	//Initialize agent in first step
	if (!initialized) {

		IkaAgent::init();

		//set initial Position
		for (int i = 0; i < sensorViewData.global_ground_truth().moving_object_size(); i++) {
			if (sensorViewData.global_ground_truth().moving_object(i).id().value() == sensorViewData.host_vehicle_id().value()) {
				osi3::BaseMoving base = sensorViewData.global_ground_truth().moving_object(i).base();
				lastPosition.x = base.position().x();
				lastPosition.y = base.position().y();
				break;
			}
		}

		//determine lanes along Trajectory passed by commandData

		futureLanes(sensorViewData, commandData, lanes);

		initialized = true;
	}

	std::cout << "------------ time: " << out.timestamp().seconds() + (out.timestamp().nanos() * 0.000000001) << "---------------" << std::endl;
	
	adapterOsiToInput(sensorViewData, _input, lanes , out.timestamp().seconds()+(out.timestamp().nanos()*0.000000001));
	
	this->AgentModel::step(time);

	//output << "   state:\ns= " << _input.vehicle.s << " d= " << _input.vehicle.d << " v= " << _input.vehicle.v << " psi= " << _input.vehicle.psi<< " Position: ("<<lastPosition.x <<" , "<<lastPosition.y<<")" << std::endl << std::endl;
	//----------------

    for (int i = 0; i < commandData.action_size(); i++)
    {
        if(commandData.action(i).has_follow_trajectory_action())
        {
            traj.CopyFrom(commandData.action(i).follow_trajectory_action());
            trajSet = true;
        }
    }

	
	return getTrajPoint(time, out);
}

int IkaAgent::terminate()
{
    return 0;
}

int IkaAgent::getTrajPoint(double time, osi3::TrafficUpdate &out)
{
    osi3::MovingObject *update = out.mutable_update();
	// desired index which assures sufficient "room" for interpolation 
	int iDes = 0; 
    if(trajSet) 
    {
		if (traj.trajectory_point_size() == 2)
		{
			interpolateStateLinear(time);
		}
		else
		{
			//interpolate at current time step
			int i;
			double tCheck;
			osi3::StatePoint *sCheck;
			for (i = 0; i < traj.trajectory_point_size(); i++)
			{
				sCheck = traj.mutable_trajectory_point(i);
				tCheck = sCheck->timestamp().seconds()
					+ sCheck->timestamp().nanos() / 1000000000.0;
				if (time < tCheck)
					break;
			}
			if (i == 0) // trajectory is in future: set pose to first traj. pose
			{
				// set index to zero and time step to first traj. time step
				iDes = 0;
				interpolateState(iDes, tCheck);
				// reset vel. and acc. values
				pose.vx = 0;
				pose.vy = 0;
				pose.ax = 0;
				pose.ay = 0;
				pose.dyaw = 0;
			}
			else
			{
				if (time <= tCheck) // only change pose if time does not exceed 
				// trajectory time. Otherwise last pose (which is a member variable)
				// is set for the rest of the time
				{

					iDes = i - 1; // standard case
					if (i == traj.trajectory_point_size() - 1)
						iDes = i - 2; // in the last interval, use last 3 points
					interpolateState(iDes, time);
				}
				else
				{
					// only use old values for pos. and heading after the traj.
					pose.vx = 0;
					pose.vy = 0;
					pose.ax = 0;
					pose.ay = 0;
					pose.dyaw = 0;
				}
			}
		}
    }
		
	update->mutable_base()->mutable_position()->set_x(pose.x);
	update->mutable_base()->mutable_position()->set_y(pose.y);
	update->mutable_base()->mutable_velocity()->set_x(pose.vx);
	update->mutable_base()->mutable_velocity()->set_y(pose.vy);
	update->mutable_base()->mutable_acceleration()->set_x(pose.ax);
	update->mutable_base()->mutable_acceleration()->set_y(pose.ay);
	update->mutable_base()->mutable_orientation()->set_yaw(pose.yaw);
	update->mutable_base()->mutable_orientation_rate()->set_yaw(pose.dyaw);

    return 0;
}

// Quadratic interpolation w.r.t. time of 3 state points.
int IkaAgent::interpolateState(int iStart, double t)
{
	// get states around which quad. interpolation will be done
	// "m" - minus, "i" - center point of interpolation, "p" - plus
	osi3::StatePoint *sm;
	sm = traj.mutable_trajectory_point(iStart);
	osi3::StatePoint *si;
	si = traj.mutable_trajectory_point(iStart+1);
	osi3::StatePoint *sp;
	sp = traj.mutable_trajectory_point(iStart+2);
	// get coordinates
	double xm = sm->position().x();
	double xi = si->position().x();
	double xp = sp->position().x();
	double ym = sm->position().y();
	double yi = si->position().y();
	double yp = sp->position().y();
	double tm = sm->timestamp().seconds()
		+ sm->timestamp().nanos() / 1000000000.0;
	double ti = si->timestamp().seconds()
		+ si->timestamp().nanos() / 1000000000.0;
	double tp = sp->timestamp().seconds()
		+ sp->timestamp().nanos() / 1000000000.0;
	
	double alphaX = xm / ((tm - ti) * (tm - tp));
	double alphaY = ym / ((tm - ti) * (tm - tp));
	double betaX = xi / ((ti - tm) * (ti - tp));
	double betaY = yi / ((ti - tm) * (ti - tp));
	double gammaX = xp / ((tp - tm) * (tp - ti));
	double gammaY = yp / ((tp - tm) * (tp - ti));
	
	pose.x = alphaX * (t - ti) * (t - tp)
		+ betaX * (t - tm) * (t - tp)
		+ gammaX * (t - tm) * (t - ti);
	pose.y = alphaY * (t - ti) * (t - tp)
		+ betaY * (t - tm) * (t - tp)
		+ gammaY * (t - tm) * (t - ti);
	pose.vx = alphaX * (2 * t - tp - ti)
		+ betaX * (2 * t - tp - tm)
		+ gammaX * (2 * t - ti - tm);
	pose.vy = alphaY * (2 * t - tp - ti)
		+ betaY * (2 * t - tp - tm)
		+ gammaY * (2 * t - ti - tm);
	pose.ax = 2 * (alphaX + betaX + gammaX);
	pose.ay = 2 * (alphaY + betaY + gammaY);
	pose.yaw = std::atan2(pose.vy, pose.vx);
	pose.dyaw = (pose.ay * pose.vx - pose.vy * pose.ax) 
			  / (pose.vx * pose.vx + pose.vy * pose.vy);


	return 0;
}

// only use this function if trajectory consists of only 2 points
int IkaAgent::interpolateStateLinear(double t)
{
	osi3::StatePoint *s0;
	s0 = traj.mutable_trajectory_point(0);
	osi3::StatePoint *s1;
	s1 = traj.mutable_trajectory_point(1);

	double x0 = s0->position().x();
	double y0 = s0->position().y();
	double x1 = s1->position().x();
	double y1 = s1->position().y();
	double t0 = s0->timestamp().seconds()
		+ s0->timestamp().nanos() / 1000000000.0;
	double t1 = s1->timestamp().seconds()
		+ s1->timestamp().nanos() / 1000000000.0;
	
	// yaw angle is constant.
	pose.yaw = std::atan2(y1 - y0, x1 - x0);
	pose.dyaw = 0;
	// init
	pose.vx = 0;
	pose.vy = 0;
	pose.ax = 0; // only two points. a has to be const.
	pose.ay = 0; // problem arises when traj. begins...

	if (t < t0)
	{
		pose.x = x0;
		pose.y = y0;
		return 0;
	}

	if (t > t1)
	{ 
		pose.x = x1;
		pose.y = y1;
		return 0;
	}

	double dt = t1 - t0;
	double alpha = (t - t0) / dt;

	pose.x  = (1 - alpha) * x0 + alpha * x1;
	pose.y  = (1 - alpha) * y0 + alpha * y1;
	pose.vx = (x1 - x0) / dt;
	pose.vy = (y1 - y0) / dt;

	return 0;
}


/**
 * @brief fills the fields of agent_model interface based on data from OSI SensorView
 *
 * @param sensorView
 * @param input
 * @param time
 */
int IkaAgent::adapterOsiToInput(osi3::SensorView& sensorView, agent_model::Input& input, std::vector<int>& futureLanes, double time) {
	if (sensorView.host_vehicle_id().value() != 77)return 0;
	/*static int count = 0;
	if (count > 0)return 0;
	count++;*/
	osi3::GroundTruth* groundTruth= sensorView.mutable_global_ground_truth();

	std::ofstream output("debug.txt", std::ofstream::out | std::ofstream::app);
	
	// --- ego ---
	// save properties needed often
	int egoId = sensorView.host_vehicle_id().value();
	osi3::Lane* egoLanePtr = nullptr;
	osi3::MovingObject obj;

	// find ego object

	for (int i = 0; i < groundTruth->moving_object_size(); i++) {

		if (groundTruth->moving_object(i).id().value() == egoId) {
			obj = groundTruth->moving_object(i);
			break;
		}
	}
	
	//determine assigned lane
	if (obj.assigned_lane_id_size() < 2) {
		egoLanePtr = findLane(obj.assigned_lane_id(0).value(), groundTruth);
	}
	else {
		//multiple assigned_lanes (intersection)
		//iterate over future lanes in reverse --> the first assigned_lane encoutered is correct
		for (int j = futureLanes.size()-1; j >=0; j--) {
			for (int i = 0; i < obj.assigned_lane_id_size(); i++) {

				if (find(futureLanes.begin(), futureLanes.end(), obj.assigned_lane_id(i).value()) != futureLanes.end())
					egoLanePtr = findLane(obj.assigned_lane_id(i).value(), groundTruth);
			}
		}
	}
	
	//save ego lane centerline coordinates for later 
	std::vector<Point2D> elPoints;
	getXY(egoLanePtr, elPoints);

	

	osi3::BaseMoving base = obj.base();

	input.vehicle.v = sqrt(base.velocity().x() * base.velocity().x() + base.velocity().y() * base.velocity().y());
	input.vehicle.a = sqrt(base.acceleration().x() * base.acceleration().x() + base.acceleration().y() * base.acceleration().y());


	//compute ego road coordinates and set member variables to new values
	double dx = base.position().x() - lastPosition.x;
	double dy = base.position().y() - lastPosition.y;

	lastPosition.x = base.position().x();
	lastPosition.y = base.position().y();
	lastS += sqrt(dx * dx + dy * dy);

	input.vehicle.s = lastS;
	

	//save angle of ego x-Axis/s-Axix because targets.psi and horizon.psi are relative to it
	double egoPsi = base.orientation().yaw();
	

	input.vehicle.dPsi = base.orientation_rate().yaw();
	
	//projection of ego coordiantes on centerline

	Point2D egoClPoint;
	closestCenterlinePoint(lastPosition, elPoints, egoClPoint);
	input.vehicle.d = sqrt(pow(lastPosition.x - egoClPoint.x, 2) + pow(lastPosition.y - egoClPoint.y, 2));

	
	// calculate s, psi, k of ego lane
	std::vector<double> s, psi, k;
	xy2curv(elPoints, s, psi, k);

	
	int i = 1;
	bool set = false;
	//compute psi of lane at current location (vehicle.psi must be relative to lane)
	for (i = 1; i < elPoints.size(),!set; i++) {	
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
			input.vehicle.psi = base.orientation().yaw() - (psi[i - 1] + t * (psi[i] - psi[i - 1]));
			set = true;
		}
	}
	if(!set && (pow(egoClPoint.x - elPoints.back().x, 2) + pow(egoClPoint.y - elPoints.back().y, 2))
		< (pow(egoClPoint.x - elPoints.front().x, 2) + pow(egoClPoint.y - elPoints.front().y, 2))) {
		input.vehicle.psi = base.orientation().yaw() - psi.back();
	}
	else if (!set) {
		input.vehicle.psi = base.orientation().yaw() - psi.front();
	}
	
	input.vehicle.pedal = 0;         // TODO
	input.vehicle.steering = 0;      // TODO  

	
	// --- signals --- 

	
	for (int i = 0; i < agent_model::NOS; i++)
	{
		if (i < groundTruth->traffic_sign_size())
		{
			osi3::TrafficSign sign = groundTruth->traffic_sign(i);
			osi3::TrafficSign_MainSign_Classification clas = sign.main_sign().classification();

			// check that sign is assigned to current ego lane. lane changes in the future are not considered currently
			bool assigned = false;
			for (int j = 0; j < clas.assigned_lane_id_size(); j++) 
				if (clas.assigned_lane_id(j).value() == egoLanePtr->id().value())
					assigned = true;
	
			if (!assigned) continue;

			input.signals[i].id = sign.id().value();
			
			
			//projection of signal position on centerline
			Point2D cPoint;
			Point2D sPoint(sign.main_sign().base().position().x(), sign.main_sign().base().position().y());

			closestCenterlinePoint(sPoint, elPoints, cPoint);
			
			double sSig = 0;

			sSig = xy2s(egoClPoint, cPoint, elPoints);

			input.signals[i].ds = sSig;


			// calculate type
			if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_STOP)
				input.signals[i].type = agent_model::SIGNAL_STOP;
			else if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN)
				input.signals[i].type = agent_model::SIGNAL_SPEED_LIMIT;
			else
				input.signals[i].type = agent_model::SIGNAL_NOT_SET;

			// TODO? calculate value
			input.signals[i].value = clas.value().value();
			
		}
		else
		{
			input.signals[i].id = 127;						 //TODO what value for unset
			input.signals[i].ds = INFINITY;
			input.signals[i].type = agent_model::SIGNAL_NOT_SET;
			input.signals[i].value = 0;
		}
		
	}
	
	
	
	// --- traffic ---
	std::unordered_map<int, int> laneMapping;
	
	mapLanes(groundTruth, laneMapping, egoLanePtr);
	
	
	for (int i = 0; i < agent_model::NOT; i++)
	{

		if (i < groundTruth->moving_object_size())
		{
			osi3::MovingObject obj = groundTruth->moving_object(i);

			if (obj.id().value() == egoId)
			{
				input.targets[i].id = 0;
				input.targets[i].ds = 0;
				input.targets[i].xy.x = 0;
				input.targets[i].xy.y = 0;
				input.targets[i].v = 0;
				input.targets[i].a = 0;
				input.targets[i].d = 0;
				input.targets[i].psi = 0;
				input.targets[i].lane = 0;
				input.targets[i].size.width = 0;
				input.targets[i].size.length = 0;
				continue;
			}

			osi3::BaseMoving base = obj.base();

			input.targets[i].id = obj.id().value();

			
			Point2D cPoint;
			Point2D bPoint; bPoint.x = base.position().x(); bPoint.y = base.position().y();

			closestCenterlinePoint(bPoint, elPoints, cPoint);

			osi3::Lane* tarLane = findLane(obj.assigned_lane_id(0).value(), groundTruth);

			double sTar = 0;

			sTar = xy2s(egoClPoint, cPoint, elPoints);

			input.targets[i].ds = sTar;
			input.targets[i].d = sqrt(pow(base.position().x() - cPoint.x, 2) + pow(base.position().y() - cPoint.y, 2));

			
			input.targets[i].xy.x = base.position().x() - lastPosition.x;
			input.targets[i].xy.y = base.position().y() - lastPosition.y;

			input.targets[i].v = sqrt(base.velocity().x()*base.velocity().x()+base.velocity().y()*base.velocity().y());
			input.targets[i].a = sqrt(base.acceleration().x() * base.acceleration().x() + base.acceleration().y() * base.acceleration().y());
			input.targets[i].psi = base.orientation().yaw() - egoPsi; 

			input.targets[i].size.length = base.dimension().length();
			input.targets[i].size.width = base.dimension().width();


			input.targets[i].lane = laneMapping[obj.assigned_lane_id(0).value()];
			
		}
		else
		{
			input.targets[i].id = 0;
			input.targets[i].ds = 0;
			input.targets[i].xy.x = 0;
			input.targets[i].xy.y = 0;
			input.targets[i].v = 0;
			input.targets[i].a = 0;
			input.targets[i].d = 0;
			input.targets[i].psi = 0;
			input.targets[i].lane = 0;
			input.targets[i].size.width = 0;
			input.targets[i].size.length = 0;
		}
	}
	
	// --- horizon ---

	double sMax = horizonTHW * input.vehicle.v;
	//double sMax = 100;

	//distance (along centerline) to each horizon point from current location
	std::vector<double> ds(agent_model::NOH,0);

	double delta = sqrt(sMax) / double(agent_model::NOH);	//linear spaces
	for (int i = 0; i < agent_model::NOH; i++) {			//create squared spaces
		ds[i]= (i+1) * (i+1) * delta * delta;
	}
	std::cout << std::endl;
	

	std::vector<Point2D> horizon;
	std::vector<Point2D> currentCl = elPoints;
	std::vector<double> kappa = k;

	double interpolation=0;

	int currentLane = egoLanePtr->id().value();
	int nextLane = 127;
	int idx = 127;
	bool spline = false;
	int finalLane = *(futureLanes.end() - 1);
	double sStart = 0;
	double sPast = 0;
	
	Point2D p1;
	Point2D p2;

	idx = closestCenterlinePoint(egoClPoint, currentCl, p2);
	
	//setup for horizon calculation
	if (idx == 0) {
		//Host vehicle before scope of the centerline.
		s.clear(); psi.clear(); kappa.clear();
		if (ds.back() != 0) {
			currentCl.insert(currentCl.begin(), egoClPoint);
			xy2curv(currentCl, s, psi, kappa);
		}

	}
	else if (idx == currentCl.size()) {
		//Host vehicle after scope of the centerline.
		s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
		spline = true;
		
		if(currentLane != finalLane) {
			nextLane = *(find(futureLanes.begin(), futureLanes.end(), currentLane) + 1);
			std::vector<Point2D> tempLane;
			getXY(findLane(nextLane, groundTruth), tempLane);

			//curved path? TODO criterium
			//spline3(egoClPoint, tempLane.front(), Point2D(std::cos(egoPsi), std::sin(egoPsi)), Point2D((tempLane[1].x-tempLane[0].x),tempLane[1].y-tempLane[0].y), currentCl);

			//straight path
			spline1(egoClPoint, tempLane.front(), currentCl);

			xy2curv(currentCl, s, psi, kappa);
		}

	}
	else {
		//somewhere within scope of currentCl
		sStart = s[idx - 1] + sqrt((egoClPoint.x - elPoints[idx].x) * (egoClPoint.x - elPoints[idx].x) + (egoClPoint.y - elPoints[idx].y) * (egoClPoint.y - elPoints[idx].y));
	}
	
	//set helpers because currentCl is overwritten when lane changes
	p1.x = currentCl.back().x; p1.y = currentCl.back().y;
	p2.x = currentCl[currentCl.size() - 2].x; p2.y = currentCl[currentCl.size() - 2].y;
	
	
	[&] {
		for (int i = 0; i < agent_model::NOH; i++) {

			//determine correct lane
			while ((s.back() + sPast - sStart) < ds[i])
			{
				sPast += s.back();

				if (!spline) {	//currently on a regular lane. Switching to spline and setting nextLane

					if (currentLane == finalLane)return;

					nextLane = *(find(futureLanes.begin(), futureLanes.end(), currentLane) + 1);
					s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
					std::vector<Point2D> tempLane;
					getXY(findLane(nextLane, groundTruth), tempLane);

					//curved connection? TODO criterium
					/*spline3(p1, tempLane.front(), Point2D(p1.x-p2.x, p1.y - p2.y),
						Point2D((tempLane[1].x - tempLane[0].x), tempLane[1].y - tempLane[0].y), currentCl);*/

					//straight connection?
					spline1(p1, tempLane.front(), currentCl);
					xy2curv(currentCl, s, psi, kappa);
					spline = true;
				}
				else {			//currently on a spline. Switching to lane determined by nextLane

					s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
					currentLane = nextLane;
					nextLane = 127;
					getXY(findLane(currentLane, groundTruth), currentCl);
					xy2curv(currentCl, s, psi, kappa);

					p1.x = currentCl.back().x; p1.y = currentCl.back().y;
					p2.x = currentCl[currentCl.size() - 2].x; p2.y = currentCl[currentCl.size() - 2].y;

					spline = false;
				}
			}
			
			//s, currentCl should now contain correct lane
			bool stop = false;

			for (int k = 1; k < s.size() && !stop; k++) {

				if ((s[k - 1] + sPast - sStart < ds[i]) && (s[k] + sPast - sStart >= ds[i])) {
					
					if (s[k - 1] == s[k]) s[k] += 0.3;

					interpolation = (ds[i] - (s[k - 1] + sPast - sStart)) / (s[k] - s[k - 1]);

					Point2D hKnot(currentCl[k - 1].x + interpolation * (currentCl[k].x - currentCl[k - 1].x),
						currentCl[k - 1].y + interpolation * (currentCl[k].y - currentCl[k - 1].y));

					horizon.push_back(hKnot);

					input.horizon.x[i] = horizon.back().x - lastPosition.x;
					input.horizon.y[i] = horizon.back().y - lastPosition.y;
					input.horizon.ds[i] = ds[i];

					input.horizon.psi[i] = psi[k - 1] + interpolation * (psi[k] - psi[k - 1]) - egoPsi; //not correct for relative psi
					input.horizon.kappa[i] = kappa[k - 1] + interpolation * (kappa[k] - kappa[k - 1]);

				}
			}
		}
		
	}();
	
	if (horizon.size() == 0) {
		
		horizon.push_back(Point2D(lastPosition.x, lastPosition.y));
		input.horizon.x[0] = horizon.back().x - lastPosition.x;
		input.horizon.y[0] = horizon.back().y - lastPosition.y;
		input.horizon.ds[0] = ds.back();
		input.horizon.psi[0] = 0; 
		input.horizon.kappa[0] = 0;//TODO
	}
	for (int i = horizon.size(); i < agent_model::NOH; i++) {
		input.horizon.x[i] = horizon.back().x  -lastPosition.x;
		input.horizon.y[i] = horizon.back().y - lastPosition.y;
		input.horizon.ds[i] = ds.back();
		input.horizon.psi[i] = input.horizon.psi[i-1]; 
		input.horizon.kappa[i] = k.back();

		Point2D knot(horizon.back().x, horizon.back().y);
		horizon.push_back(knot);
	}

	
	for (int i = 0; i < agent_model::NOH; i++) {
		std::cout << "(" << input.horizon.x[i] << "," << input.horizon.y[i] << ")\n";
	}
	

	int l, e, r;

	l = egoLanePtr->classification().left_adjacent_lane_id_size() > 0 ?
		egoLanePtr->classification().left_adjacent_lane_id(0).value() : 127;
	e = egoLanePtr->id().value();
	r = egoLanePtr->classification().right_adjacent_lane_id_size() > 0 ?
		egoLanePtr->classification().right_adjacent_lane_id(0).value() : 127;

	/*
	// --- lanes ---
	std::vector<double> width;
	calcWidth(horizon, egoLanePtr->id().value(), futureLanes, width, groundTruth);

	
	for (int i = 0; i < agent_model::NOL; i++)
	{
		if (i < groundTruth->lane_size())
		{

			osi3::Lane lane = groundTruth->lane(i);

			input.lanes[i].id = laneMapping[lane.id().value()];

			// calculate type
			if (lane.classification().type() == osi3::Lane_Classification_Type_TYPE_DRIVING)
			{
				input.lanes[i].access = agent_model::Accessibility::ACC_ACCESSIBLE;
			}
			else if (lane.classification().type() == osi3::Lane_Classification_Type_TYPE_NONDRIVING)
			{
				input.lanes[i].access = agent_model::Accessibility::ACC_NOT_ACCESSIBLE;
			}


			// calculate driving direction
			if (lane.classification().centerline_is_driving_direction())
				input.lanes[i].dir = agent_model::DrivingDirection::DD_FORWARDS;
			else
				input.lanes[i].dir = agent_model::DrivingDirection::DD_BACKWARDS;


			// calculate lane width at horizon points
			
			

			// store width of horizon TODO: correct value for neighbor lanes
			if (lane.id().value() == l)
				for (int i = 0; i < agent_model::NOH; i++)
					input.horizon.leftLaneWidth[i] = width[i];


			if (lane.id().value() == e)
				for (int i = 0; i < agent_model::NOH; i++)
					input.horizon.egoLaneWidth[i] = width[i];

			if (lane.id().value() == r)
				for (int i = 0; i < agent_model::NOH; i++)
					input.horizon.rightLaneWidth[i] = width[i];


			// save average of width
			input.lanes[i].width = accumulate(width.begin(), width.end(), 0.0) / width.size();

			std::vector<Point2D> lPoints;

			getXY(&lane, lPoints);

			input.lanes[i].closed = xy2s(egoClPoint, lPoints.back(), lPoints);
			input.lanes[i].route = input.lanes[i].closed;   // TODO
			// -> where do we get this value from? 
		}
		else
		{
			input.lanes[i].id = 127;
			input.lanes[i].width = 0;
			input.lanes[i].route = 0;
			input.lanes[i].closed = 0;
			input.lanes[i].access = agent_model::ACC_NOT_SET;
			input.lanes[i].dir = agent_model::DD_NOT_SET;
		}
	}

	//if egolane has no right/left adjacent lane
	if (l == 127)
		for (int i = 0; i < agent_model::NOH; i++)
			input.horizon.leftLaneWidth[i] = 0;

	if (r == 127)
		for (int i = 0; i < agent_model::NOH; i++)
			input.horizon.rightLaneWidth[i] = 0;
	
	for (int i = 0; i < width.size(); i++) {
		std::cout << "w: " << width[i] << std::endl;
		input.horizon.egoLaneWidth[i] = width[i];
	}
	*/
	return 0;
}