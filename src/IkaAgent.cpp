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

//using Point2D = agent_model::Position;


void IkaAgent::init()
{
	lastS = 0;
	horizonTHW = 15;
	double l = 3.22; // wheel base

	drParam = getParameters();
	drState = getState();

	drParam->velocity.a = 2.0;
	drParam->velocity.b = -2.0;
	drParam->velocity.thwMax = 10.0;
	drParam->velocity.delta = 4.0;
	drParam->velocity.deltaPred = 3.0;
	drParam->velocity.vComfort = 50.0 / 3.6;
	drParam->velocity.ayMax = 1.5;

	// stop control
	drParam->stop.T = 2.0;
	drParam->stop.TMax = 7.0;
	drParam->stop.tSign = 0.5;
	drParam->stop.pedalDuringStanding = -0.3;

	// following
	drParam->follow.dsStopped = 2.0;
	drParam->follow.thwMax = 10.0;
	drParam->follow.timeHeadway = 1.8;
	
	// steering
	drParam->steering.thw[0] = 1.0;
	drParam->steering.thw[1] = 3.0;
	drParam->steering.dsMin[0] = 1.0;
	drParam->steering.dsMin[1] = 3.0;
   	drParam->steering.P[0] = 0.03 * l;
    drParam->steering.P[1] = 0.015 * l;
    drParam->steering.D[0] = 0.1;
    drParam->steering.D[1] = 0.1;

	AgentModel::init();
   	vehInput = _vehicle.getInput();
    vehState = _vehicle.getState();
    vehParam = _vehicle.getParameters();
	
	
    // vehicle parameters
    vehParam->steerTransmission  = 0.474;
    vehParam->steerTransmission  = 0.5;
    vehParam->wheelBase          = l;
    vehParam->cwA                = 0.6;
    vehParam->mass               = 1.5e3;
    vehParam->powerMax           = 1.0e4;
    vehParam->forceMax           = 6.0e3;
    vehParam->idle               = 0.05;
    vehParam->rollCoefficient[0] = 4.0 * 9.91e-3;
    vehParam->rollCoefficient[1] = 4.0 * 1.95e-5;
    vehParam->rollCoefficient[2] = 4.0 * 1.76e-9;
    vehParam->size.x = 5.0;
    vehParam->size.y = 2.0;
	vehParam->driverPosition.x = 0.0;// 0.50;
    vehParam->driverPosition.y = 0.0;//0.5;
	
	// set controller parameters (lateral motion control)
    steeringContr.setParameters(10.0 * l, 0.1 * l, 0.0, 1.0);
    steeringContr.setRange(-1.0, 1.0, INFINITY);

    // set controller parameters (longitudinal motion control)
    pedalContr.setParameters(1.75, .5, 0.01, 1.0);
    pedalContr.setRange(-1.0, 1.0, INFINITY);

    // set states
    _vehicle.reset();
	vehState->position.x =  lastPosition.x;
    vehState->position.y = lastPosition.y;
	vehState->v =  v;//tbd
	vehState->psi = psi; //tbd
	
    // set variables
    pedalContr.setVariables(&vehState->a, &drState->subconscious.a, &vehInput->pedal, &drState->subconscious.pedal);
    steeringContr.setVariables(&vehState->kappa, &drState->subconscious.kappa, &vehInput->steer);
	
	pedalContr.reset();
    steeringContr.reset();

	_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;

	// clean output file
	std::ofstream output("debug" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::trunc);
	output.close();
	std::ofstream hor("horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::trunc);
	hor.close();
	std::ofstream hor_test("test_horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::trunc);
	hor.close();
}


int IkaAgent::step(double time, double stepSize, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out, setlevel4to5::DynamicsRequest & dynOut)
{
	if (false) return 0;
	//----------------
	
	//Initialize agent in first step
	if (!initialized) {
		hostVehicleId = sensorViewData.host_vehicle_id().value();
		//set initial Position
		for (int i = 0; i < sensorViewData.global_ground_truth().moving_object_size(); i++) {
			if (sensorViewData.global_ground_truth().moving_object(i).id().value() == sensorViewData.host_vehicle_id().value()) {
				osi3::BaseMoving base = sensorViewData.global_ground_truth().moving_object(i).base();
				lastPosition.x = base.position().x();
				lastPosition.y = base.position().y();
				psi = base.orientation().yaw();
				v = 7;// sqrt(base.velocity().x() * base.velocity().x() + base.velocity().y() * base.velocity().y());
				break;
			}
		}

		IkaAgent::init();
		initialized = true;
	}	
	std::ofstream output("debug" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::app);
	std::cout << "------------ time: " << out.timestamp().seconds() + (out.timestamp().nanos() * 0.000000001) << " ---------------" << std::endl;
	//check for new traffic command
	if (commandData.action_size() > 0) {	//&&commandData.traffic_participant_id().value() == sensorViewData.host_vehicle_id().value(), traffic_participant not set currently
		parseTrafficCommand(sensorViewData, commandData);
		//determine type of maneuver (if crossing intersection)
		classifyManeuver(sensorViewData);
		for (auto& lane:lanes) std::cout << lane << " ";
		std::cout << std::endl;
		generateHorizon(sensorViewData, _input, lanes);
	}
	

	//Translate sensorViewData to agent_model::input
	adapterOsiToInput(sensorViewData, _input, lanes , out.timestamp().seconds()+(out.timestamp().nanos()*0.000000001));
	this->AgentModel::step(time);
	//std::cout << "Vehicle before Step: x=\t" << vehState->position.x << ", y=\t" << vehState->position.y << ", psi=\t" << vehState->psi << std::endl;
	pedalContr.step(stepSize);
	steeringContr.step(stepSize);	
	// Perform Vehicle Model step
    vehInput->slope = 0.0; 
	_vehicle.step(stepSize); // STEP SIZE not TIME!
	//std::cout << "a=" << drState->subconscious.a <<" kappa="<< drState->subconscious.kappa << " dpsiDes=" << drState->subconscious.dPsi <<std::endl;
    //std::cout << "pedal = " << vehInput->pedal << "\tveh_a = " << vehState->a << "\tstep_size = " << stepSize << std::endl;
	if(true)
		output << out.timestamp().seconds() + (out.timestamp().nanos() * 0.000000001) << "," 
			<< vehState->position.x << "," 
			<< vehState->position.y << ","
			<< vehState->a << "," 
			<< vehState->v << ","  
			<< vehState->dPsi << ","  
			<< drState->subconscious.a << ","  
			<< drState->conscious.velocity.local << ","  
			<< drState->conscious.velocity.prediction << ","  
			<< _input.vehicle.d << ","  
			<< vehInput->pedal << std::endl;
	
	//std::cout << "Driver out: pedal=\t" << vehInput->pedal << ", steer?\t" << vehInput->steer << std::endl;
	//std::cout << "Vehicle before Step: x=\t" << vehState->position.x << ", y=\t" << vehState->position.y << ", psi=\t" << vehState->psi << std::endl;
	dynOut.set_longitudinal_acceleration_target(drState->subconscious.a);
	dynOut.set_curvature_target(drState->subconscious.kappa);
	return applyDriverOutput(time, out);
}


/**
 * @brief Handle Traffic Commands of type follow_trajectory_action, follow_path_action, follow_speed_action
 *	
 * checks whether the received trafficCommand has an Id != the last processed Id, then fills the IkaAgent::lanes field with the lanes along the new trajectory or updates the desired speed.
 * @param sensorViewData
 */
int IkaAgent::parseTrafficCommand(osi3::SensorView& sensorViewData, osi3::TrafficCommand& commandData) {
	osi3::GroundTruth* groundTruth = sensorViewData.mutable_global_ground_truth();
	osi3::MovingObject host;
	int startingLaneIdx;
	Point2D destPoint;
	
	for (int i = 0; i < commandData.action_size(); i++) {
		if (commandData.action(i).has_follow_trajectory_action() && (commandData.action(i).follow_trajectory_action().action_header().action_id().value() != trajActionId)) {

			osi3::FollowTrajectoryAction traj = commandData.action(i).follow_trajectory_action();
			trajActionId = traj.action_header().action_id().value();
			//find Host
			for (int i = 0; i < groundTruth->moving_object_size(); i++) {
				if (groundTruth->moving_object(i).id().value() == sensorViewData.host_vehicle_id().value())
					host = groundTruth->moving_object(i);
			}

			//needs an initial assigned lane that is unambiguous
			if (host.assigned_lane_id_size() < 2) {
				startingLaneIdx = findLaneId(groundTruth, host.assigned_lane_id(0).value());
			}
			else {
				int id = closestLane(groundTruth,Point2D(traj.trajectory_point(0).position().x(), traj.trajectory_point(0).position().y()));
				startingLaneIdx = findLaneId(groundTruth,id);
			}
			
			destPoint = Point2D(traj.trajectory_point(traj.trajectory_point_size() - 1).position().x(), traj.trajectory_point(traj.trajectory_point_size() - 1).position().y());
			lanes.clear();
			futureLanes(groundTruth, startingLaneIdx, destPoint, lanes);
			

		}
		if (commandData.action(i).has_follow_path_action() && (commandData.action(i).follow_path_action().action_header().action_id().value() != pathActionId)) {

			osi3::FollowPathAction path = commandData.action(i).follow_path_action();
			pathActionId = path.action_header().action_id().value();

			//find Host
			for (int i = 0; i < groundTruth->moving_object_size(); i++) {
				if (groundTruth->moving_object(i).id().value() == sensorViewData.host_vehicle_id().value())
					host = groundTruth->moving_object(i);
			}

			//needs an initial assigned lane that is unambiguous
			if (host.assigned_lane_id_size() < 2) {
				startingLaneIdx = findLaneId(groundTruth, host.assigned_lane_id(0).value());
			}
			else {
				int id = closestLane(groundTruth, Point2D(path.path_point(0).position().x(), path.path_point(0).position().y()));
				startingLaneIdx = findLaneId(groundTruth, id);
			}

			destPoint = Point2D(path.path_point(path.path_point_size() - 1).position().x(), path.path_point(path.path_point_size() - 1).position().y());
			lanes.clear();
			futureLanes(groundTruth, startingLaneIdx, destPoint, lanes);
		}
		if (commandData.action(i).has_speed_action() && (commandData.action(i).speed_action().action_header().action_id().value() != speedActionId)) {

			osi3::SpeedAction speed = commandData.action(i).speed_action();
			speedActionId = speed.action_header().action_id().value();

			//vehState->v = speed.absolute_target_speed();
			drParam->velocity.vComfort = speed.absolute_target_speed();
		}
	}
	return 0;
}

/**
 * @brief classifies the host vehicle's path as either TURn_RIGHT, TURN_LEFT or STRAIGHT (default)
 * 
 * based on the curvature of the first lane of type INTERSECTION encountered along the host's path
 * @param sensorViewData
 */
void IkaAgent::classifyManeuver(osi3::SensorView& sensorViewData) {

	osi3::GroundTruth* groundTruth = sensorViewData.mutable_global_ground_truth();

	// Maneuver is only applicable when the host traverses an intersection --> one futureLane must be of type INTERSECION
	// Assuming there is only one intersection along the path, considering the first encountered
	std::vector<Point2D> pos;
	std::vector<double> s, k, p;
	for (auto it : lanes) {
		osi3::Lane* lane = findLane(it, groundTruth);
		if (lane->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION)
			getXY(lane, pos);
	}	
	/*std::cout << "curve: ";
	for (auto& p:pos) std::cout << p.x << "," << p.y << " ";
	std::cout << "\n";*/
	for (int i = 0; i < pos.size()-1; i++) 
	{
		double dsNext = (pos[i+1].x - pos[i].x) * (pos[i+1].x - pos[i].x)
					  + (pos[i+1].y - pos[i].y) * (pos[i+1].y - pos[i].y);

		if (dsNext < 0.1) {
			pos.erase(pos.begin()+i+1);
			i++;
		}
	}
	if (pos.size() <= 2) 
		_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;
	else
	{
		xy2curv(pos, s, p, k);
		double avg = std::accumulate(k.cbegin(), k.cend(), 0.0) / k.size();
		std::cout << "avg: " << avg << "\n";
		double eps = 0.0000001;
		if (avg > eps)
			_input.vehicle.maneuver = agent_model::Maneuver::TURN_LEFT;
		else if(avg < (-1*eps))
			_input.vehicle.maneuver = agent_model::Maneuver::TURN_RIGHT;
		else
			_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;
	}
	

		/*if (lane->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION && lane->classification().centerline_size() > 2) {
		//if(lane->id().value() == 200029 || lane->id().value() == 200030 || lane->id().value() == 200002 || lane->id().value() == 200009 || lane->id().value() == 200010 ||
		//	lane->id().value() == 200000 || lane->id().value() == 200025 || lane->id().value() == 200001 || lane->id().value() == 200008 || lane->id().value() == 200028 ||
		//	lane->id().value() == 200026 || lane->id().value() == 200027){

			// take three points of the centerline in the intersection and calculate (potential) turn direction from curvature
			if (lane->classification().centerline_size() <= 2) {
				_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;
				break;
			}

			double cl_size = lane->classification().centerline_size();
			pos.push_back(Point2D(lane->classification().centerline(0).x(), lane->classification().centerline(0).y()));
			pos.push_back(Point2D(lane->classification().centerline(cl_size / 2).x(), lane->classification().centerline(cl_size / 2).y()));
			pos.push_back(Point2D(lane->classification().centerline(cl_size - 1).x(), lane->classification().centerline(cl_size - 1).y()));

			xy2curv(pos, s, p, k);

			double avg = std::accumulate(k.cbegin(), k.cend(), 0.0) / k.size();
			double eps = 0.001;
			if (avg > eps)
				_input.vehicle.maneuver = agent_model::Maneuver::TURN_RIGHT;
			else if(avg < (-1*eps))
				_input.vehicle.maneuver = agent_model::Maneuver::TURN_LEFT;
			else
				_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;

			
		}
	}*/
	std::cout << "\nMANEUVER (0=straight, 1=left, 2=right): " << _input.vehicle.maneuver << std::endl;
}

int IkaAgent::terminate()
{
    return 0;
}

int IkaAgent::applyDriverOutput(double time, osi3::TrafficUpdate &out)
{
	osi3::MovingObject *update = out.mutable_update();

	update->mutable_base()->mutable_position()->set_x(vehState->position.x);
	update->mutable_base()->mutable_position()->set_y(vehState->position.y);
	update->mutable_base()->mutable_velocity()->set_x(vehState->v);
	update->mutable_base()->mutable_velocity()->set_y(0);
	update->mutable_base()->mutable_acceleration()->set_x(vehState->a);
	update->mutable_base()->mutable_acceleration()->set_y(0);
	update->mutable_base()->mutable_orientation()->set_yaw(vehState->psi);
	update->mutable_base()->mutable_orientation_rate()->set_yaw(vehState->dPsi);

	return 0;
}

int IkaAgent::generateHorizon(osi3::SensorView& sensorView, agent_model::Input& input, std::vector<int>& futureLanes) {
	
	// first get all relevant points from futureLanes

	osi3::GroundTruth* groundTruth = sensorView.mutable_global_ground_truth();
	

	for (auto& l:futureLanes) 
		getXY(findLane(l, groundTruth), pathCenterLine);

	// remove duplicates
	for (int i = 0; i < pathCenterLine.size()-1; i++) 
	{
		double dsNext = (pathCenterLine[i+1].x - pathCenterLine[i].x) * (pathCenterLine[i+1].x - pathCenterLine[i].x)
					  + (pathCenterLine[i+1].y - pathCenterLine[i].y) * (pathCenterLine[i+1].y - pathCenterLine[i].y);

		if (dsNext < 0.1) {
			pathCenterLine.erase(pathCenterLine.begin()+i+1);
			i++;
		}
	}

	// calculate s, kappa, and psi
	xy2curv(pathCenterLine, pathS, pathPsi, pathKappa);

	// debug info
	std::ofstream test_hor("test_horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::app);
	for(int i = 0; i < pathCenterLine.size(); i++) {
		test_hor << pathCenterLine[i].x << "," 
			<< pathCenterLine[i].y << "," 
			<< pathS[i] << "," 
			<< pathPsi[i] << "," 
			<< pathKappa[i] << "\n";
	}

	test_hor.close();

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
	if (false)return 0;
	
	osi3::GroundTruth* groundTruth = sensorView.mutable_global_ground_truth();
	
	std::ofstream horizon_out("horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::app);
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
		for (int j = futureLanes.size() - 1; j >= 0; j--) {
			for (int i = 0; i < obj.assigned_lane_id_size(); i++) {

				if (find(futureLanes.begin(), futureLanes.end(), obj.assigned_lane_id(i).value()) != futureLanes.end())
					egoLanePtr = findLane(obj.assigned_lane_id(i).value(), groundTruth);
			}
		}
	}

	//save ego lane centerline coordinates for later 
	std::vector<Point2D> elPoints;
	getXY(egoLanePtr, elPoints);
	//std::cout  << "assigned_lane: " << egoLanePtr->id().value() << "\n";


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
	//closestCenterlinePoint(lastPosition, elPoints, egoClPoint);
	int test_idx = closestCenterlinePoint(lastPosition, pathCenterLine, egoClPoint);
	
	// calculate s, psi, k of ego lane
	std::vector<double> s, psi, k;
	xy2curv(elPoints, s, psi, k);
	/*std::cout << "psi:\n";
	for (auto it : psi) {
		std::cout << it*180/3.1416 << std::endl;
	}*/
	

	int i = 1;
	bool set = false;
	//compute psi of lane at current location (vehicle.psi must be relative to lane)
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
			input.vehicle.psi = base.orientation().yaw() - (psi[i - 1] + t * (psi[i] - psi[i - 1]));
			//std::cout << "vehicle psi " << input.vehicle.psi * 180 / 3.14159 << "=" << base.orientation().yaw() * 180 / 3.14159 << "-"<< (psi[i - 1] + t * (psi[i] - psi[i - 1])) * 180 / 3.14159 << std::endl;
			set = true;
		}
	}
	if (!set && (pow(egoClPoint.x - elPoints.back().x, 2) + pow(egoClPoint.y - elPoints.back().y, 2))
		< (pow(egoClPoint.x - elPoints.front().x, 2) + pow(egoClPoint.y - elPoints.front().y, 2))) {
		input.vehicle.psi = base.orientation().yaw() - psi.back();
		//std::cout << "vehicle psi " << input.vehicle.psi * 180 / 3.14159 << "=" << base.orientation().yaw()*180 / 3.14159 << "-"<< psi.back() * 180 / 3.14159 << std::endl;
	}
	else if (!set) {
		input.vehicle.psi = base.orientation().yaw() - psi.front();
		//std::cout << "vehicle psi " << input.vehicle.psi * 180 / 3.14159 << "=" << base.orientation().yaw()*180/3.14159 << "-" << psi.back() * 180 / 3.14159 << std::endl;
	}


	double dirCL = atan2(egoClPoint.y-lastPosition.y, egoClPoint.x-lastPosition.x);
	//sign?!
	double orientation = egoPsi-dirCL;
	if(orientation < -M_PI) orientation = orientation + 2* M_PI;
	if(orientation > M_PI) orientation = orientation - 2* M_PI;
	int d_sig = (orientation>0) - (orientation<0);
	double distCL = sqrt(pow(lastPosition.x - egoClPoint.x, 2) + pow(lastPosition.y - egoClPoint.y, 2));	
	input.vehicle.d = d_sig*distCL;
	
	input.vehicle.pedal = 0;         // TODO
	input.vehicle.steering = 0;      // TODO  

	
	// --- signals --- 
	std::vector<int> priorityLanes;
	std::vector<int> yieldingLanes;	
	

	int signal = 0;

	for (int i = 0; i < groundTruth->traffic_sign_size(); i++)
	{
		if (signal >= agent_model::NOS)break;

		osi3::TrafficSign sign = groundTruth->traffic_sign(i);
		osi3::TrafficSign_MainSign_Classification clas = sign.main_sign().classification();

		auto it = futureLanes.end();
		// check that the sign is assigned to a lane along the route. 
		bool assigned = false;
		// sSig will hold s value along centerlines to reach the signal
		double sSig = 0;
		for (int j = 0; j < clas.assigned_lane_id_size(); j++) {
			it = find(futureLanes.begin(), futureLanes.end(), clas.assigned_lane_id(j).value());
			if (it != futureLanes.end()) {
				assigned = true;
				//std::cout << "\nsignal on lane: " << clas.assigned_lane_id(j).value() << std::endl;
				break;
			}
		}

		if (!assigned) continue;

		

		//projection of signal position on centerline: cPoint
		Point2D cPoint;
		Point2D sPoint(sign.main_sign().base().position().x(), sign.main_sign().base().position().y());
		//closestCenterlinePoint(sPoint, elPoints, cPoint);
		Point2D target = sPoint;
		//std::cout << "Spoint: (" << sPoint.x << "," << sPoint.y << ")" << std::endl;
		//signal on future lane
		if (*it != egoLanePtr->id().value()) {
			// signal is not assigned to the current lane -> add distance along the signal's assigned lane
			// meaning assigned_lane beginning to sPoint
			std::vector<Point2D> pos;
			getXY(findLane(*it, groundTruth), pos);
			sSig += xy2s(pos.front(), sPoint, pos);
			pos.clear();
		}
		while (it != futureLanes.begin() && *(it - 1) != egoLanePtr->id().value()) {
			it--;
			std::vector<Point2D> pos;
			getXY(findLane(*it, groundTruth), pos);
			sSig += xy2s(pos.front(), pos.back(), pos);
			target = pos.front();
		}


		//std::cout << "\nhost : (" << lastPosition.x << "," << lastPosition.y << ")\nsignal: (" << sPoint.x << "," << sPoint.y << ") " << "\ntarget (" << target.x << "," << target.y << ") " << std::endl;

		sSig += xy2s(egoClPoint, target, elPoints);

		input.signals[signal].id = signal; //could also take sign.id().value() as id here, OSI ids are often larger numbers
		input.signals[signal].ds = sSig;
		
		// calculate type 
		if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_STOP) {
			input.signals[signal].type = agent_model::SIGNAL_STOP;
			//no value for stop
			input.signals[signal].value = 0;
		}
		else if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN) {
			input.signals[signal].type = agent_model::SIGNAL_SPEED_LIMIT;
			input.signals[signal].value = clas.value().value();
		}
		else if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN) {
			for (int j = 0; j < clas.assigned_lane_id_size(); j++)
				priorityLanes.push_back(clas.assigned_lane_id(j).value());
			input.signals[signal].type = agent_model::SIGNAL_PRIORITY;
		}
		else if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY ||
			clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_BEFORE_LEFT_NEXT_INTERSECTION) {

			for (int j = 0; j < clas.assigned_lane_id_size(); j++)
				yieldingLanes.push_back(clas.assigned_lane_id(j).value());
			input.signals[signal].type = agent_model::SIGNAL_YIELD;
		}
		else {
			input.signals[signal].type = agent_model::SIGNAL_NOT_SET;
			//no value for unset type
			input.signals[signal].value = 0;
		}

		signal++;
	}
	//fill remaining signals with default values
	for (i = signal; i < agent_model::NOS; i++) {
		input.signals[i].id = 127;
		input.signals[i].ds = INFINITY;
		input.signals[i].type = agent_model::SIGNAL_NOT_SET;
		input.signals[i].value = 0;
	}

	//for (i = 0; i < agent_model::NOS; i++) {
		//std::cout << "Signal " << input.signals[i].id << " ds=" << input.signals[i].ds << " type: " << input.signals[i].type << " value: " << input.signals[signal].value << std::endl;
	//}


	// --- traffic ---
	std::unordered_map<int, int> laneMapping;

	mapLanes(groundTruth, laneMapping, egoLanePtr, futureLanes);

	std::vector<int> lanesEnteringIntersection;

	for (int i = 0; i < groundTruth->lane_size(); i++) {
		osi3::Lane lane = groundTruth->lane(i);
		// Workaround until intersection type is filled TODO
		if (lane.classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION)
		//if(groundTruth->lane(i).id().value() == 200024 || groundTruth->lane(i).id().value() == 200017 || groundTruth->lane(i).id().value() == 200035 || groundTruth->lane(i).id().value() == 200005  )
			lanesEnteringIntersection.push_back(lane.id().value());

		/*for (int j = 0; j < lane.classification().lane_pairing_size(); j++) {
			if (findLane(lane.classification().lane_pairing(j).successor_lane_id().value(), groundTruth)->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
				lanesEnteringIntersection.push_back(lane.id().value());
			}
		}*/
	}
	

	for (int i = 0; i < agent_model::NOT; i++)
	{

		if (i < groundTruth->moving_object_size())
		{
			osi3::MovingObject obj = groundTruth->moving_object(i);

			if (obj.id().value() == egoId)
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

			input.targets[i].priority = agent_model::TARGET_PRIORITY_NOT_SET;
			osi3::BaseMoving base = obj.base();

			input.targets[i].id = i+1;

			auto it = futureLanes.end(); 
			bool assigned = false;

			double sTar = 0;

			// determine whether target is assigned to a lane along the host's path (following possible)
			for (int j = 0; j < obj.assigned_lane_id_size(); j++) {

				it = find(futureLanes.begin(), futureLanes.end(), obj.assigned_lane_id(j).value());

				if (it != futureLanes.end()) {
					assigned = true;
					break;
				}
			}

			for (int j = 0; j < obj.assigned_lane_id_size(); j++) {
				if (find(lanesEnteringIntersection.begin(), lanesEnteringIntersection.end(), obj.assigned_lane_id(j).value())
					!= lanesEnteringIntersection.end()) {
					//target is on a lane ending in an intersection

					std::vector<Point2D> cl;
					getXY(findLane(obj.assigned_lane_id(j).value(), groundTruth), cl);
					input.targets[i].dsIntersection = xy2s(Point2D(base.position().x(), base.position().y()), cl.back(), cl);
					if (find(priorityLanes.begin(), priorityLanes.end(), obj.assigned_lane_id(j).value())
						!= priorityLanes.end())
						input.targets[i].priority = agent_model::TARGET_ON_PRIORITY_LANE;
					else if (find(yieldingLanes.begin(), yieldingLanes.end(), obj.assigned_lane_id(j).value())
						!= yieldingLanes.end())
						input.targets[i].priority = agent_model::TARGET_ON_GIVE_WAY_LANE;

					break;
				}
				else if (findLane(obj.assigned_lane_id(j).value(), groundTruth)->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
					input.targets[i].priority = agent_model::TARGET_ON_INTERSECTION;
				}
			}

			// only fill ds and d fields when target is along the host's path
			if (assigned) {
				Point2D cPoint;
				Point2D bPoint(base.position().x(), base.position().y());

				closestCenterlinePoint(bPoint, elPoints, cPoint);

				osi3::Lane* tarLane = findLane(obj.assigned_lane_id(0).value(), groundTruth);

				Point2D current = bPoint;

				if (*it != egoLanePtr->id().value()) {
					// target is not assigned to the current lane -> add distance along the target's assigned lane
					// meaning assigned_lane's beginning to bPoint
					std::vector<Point2D> pos;
					getXY(findLane(*it, groundTruth), pos);
					sTar += xy2s(pos.front(), bPoint, pos);
					pos.clear();
				}
				while (it != futureLanes.begin() && *(it - 1) != egoLanePtr->id().value()) {
					it--;
					std::vector<Point2D> pos;
					getXY(findLane(*it, groundTruth), pos);
					sTar += xy2s(pos.front(), pos.back(), pos);
					current = pos.front();
				}

				sTar += xy2s(egoClPoint, current, elPoints);

				input.targets[i].ds = sTar; //TODO: check what happens when target is behind host vehicle
				input.targets[i].d = sqrt(pow(base.position().x() - cPoint.x, 2) + pow(base.position().y() - cPoint.y, 2));
			}
			else {
				input.targets[i].ds = 0; 
				input.targets[i].d = 0;
			}

			input.targets[i].xy.x = base.position().x() - lastPosition.x;
			input.targets[i].xy.y = base.position().y() - lastPosition.y;

			input.targets[i].v = sqrt(base.velocity().x() * base.velocity().x() + base.velocity().y() * base.velocity().y());
			input.targets[i].a = sqrt(base.acceleration().x() * base.acceleration().x() + base.acceleration().y() * base.acceleration().y());
			input.targets[i].psi = base.orientation().yaw() - egoPsi;

			input.targets[i].size.length = base.dimension().length();
			input.targets[i].size.width = base.dimension().width();


			input.targets[i].lane = laneMapping[obj.assigned_lane_id(0).value()];

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
	}

	// --- horizon ---

	double sMax = std::max(15.0, horizonTHW * input.vehicle.v);
	//double sMax = 100;

	//distance (along centerline) to each horizon point from current location
	std::vector<double> ds(agent_model::NOH, 0);

	double delta = sqrt(sMax) / double(agent_model::NOH);	//linear spaces
	for (int i = 0; i < agent_model::NOH; i++) {			//create squared spaces
		ds[i] = (i + 1) * (i + 1) * delta * delta;
	}

	double interpolation = 0;
	double defaultWidth = 3.5;
	//		//
	// NEW	//
	//		//
	Point2D dummy;
	//std::cout << "look here: " << lastPosition.x << ", " << lastPosition.y << " cl lenght: " << pathCenterLine.size() << "\n"; 
	int hor_idx = closestCenterlinePoint(lastPosition, pathCenterLine, dummy);
	double sTravel = pathS[hor_idx - 1] + sqrt((lastPosition.x - pathCenterLine[hor_idx - 1].x) * (lastPosition.x - pathCenterLine[hor_idx - 1].x) 
										 + (lastPosition.y - pathCenterLine[hor_idx - 1].y) * (lastPosition.y - pathCenterLine[hor_idx - 1].y));
	//std::cout << "sTravel: " << sTravel << " after idx: " << hor_idx << "\n";
	
	for (int i = 0; i < agent_model::NOH; i++) {
		double dsCur = (i + 1) * (i + 1) * delta * delta;
		int j=-1;
		for (auto& ss:pathS) {			
			if (ss > sTravel+dsCur) break;
			j++;
		}
		//std::cout << j << " ";

		//int i_idx = closestCenterlinePoint(lastKnot, pathCenterLine, dummy);
		
		interpolation = (sTravel + dsCur - pathS[j]) / (pathS[j+1] - pathS[j]);
		Point2D hKnot(pathCenterLine[j].x + interpolation * (pathCenterLine[j+1].x - pathCenterLine[j].x),
					  pathCenterLine[j].y + interpolation * (pathCenterLine[j+1].y - pathCenterLine[j].y));
		
		input.horizon.ds[i] = dsCur;
		input.horizon.x[i] = std::cos(egoPsi) * (hKnot.x - lastPosition.x) + std::sin(egoPsi) * (hKnot.y - lastPosition.y); 
		input.horizon.y[i] = -1.0 * std::sin(egoPsi) * (hKnot.x - lastPosition.x) + std::cos(egoPsi) * (hKnot.y - lastPosition.y);
		input.horizon.psi[i] = pathPsi[j] + interpolation * (pathPsi[j+1] - pathPsi[j]) - egoPsi;
		input.horizon.kappa[i] = pathKappa[j] + interpolation * (pathKappa[j+1] - pathKappa[j]);
		input.horizon.egoLaneWidth[i] = 3.75;
		input.horizon.leftLaneWidth[i] = 3.75;
		input.horizon.rightLaneWidth[i] = 3.75;
		
	}
	std::cout << std::endl;

	//			//
	// NEW(end)	//
	//			//
/*
	std::vector<Point2D> horizon;
	std::vector<Point2D> currentCl = elPoints;
	std::vector<double> kappa = k;

	int currentLane = egoLanePtr->id().value();

	int idx = 127;
	bool spline = false;
	int finalLane = *(futureLanes.end() - 1);
	int nextLane = (currentLane != finalLane) ? *(find(futureLanes.begin(), futureLanes.end(), currentLane) + 1) : 127;
	double sStart = 0;
	double sPast = 0;

	Point2D p1;
	Point2D p2;

	idx = closestCenterlinePoint(egoClPoint, currentCl, p2);

	//setup for horizon calculation: in most cases the current position will be somewhere within the scope of the centerline of the current lane.
	//If not, use return value of 'closestCenterlinePoint' to determine the relative position
	if (idx == 0) {
		//Host vehicle before scope of the centerline, create linear connection
		s.clear(); psi.clear(); kappa.clear();
		if (ds.back() != 0) {
			currentCl.insert(currentCl.begin(), egoClPoint);
			xy2curv(currentCl, s, psi, kappa);
		}

	}
	else if (idx == currentCl.size()) {
		//Host vehicle after scope of the centerline, move on to next lane
		s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
		spline = true;

		if (currentLane != finalLane) {
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
		//somewhere within scope of currentCl, set sStart to the distance along current lane that should be disregarded 
		sStart = s[idx - 1] + sqrt((lastPosition.x - elPoints[idx - 1].x) * (lastPosition.x - elPoints[idx - 1].x) + (lastPosition.y - elPoints[idx - 1].y) * (lastPosition.y - elPoints[idx - 1].y));
	}
	
	

	[&] {
		for (int i = 0; i < agent_model::NOH; i++) {
			//determine correct lane. If s is already of the correct lane for this horizon point, the loop is not executed
			while ((s.back() + sPast - sStart) < ds[i]) //s.back: s at the end of current lane. sPast: s along the lanes on the path already considered, sStart: s at starting point
			{
				if (currentLane == finalLane) return;
				
				sPast += s.back();
				s.clear(); psi.clear(); kappa.clear(); currentCl.clear();

				currentLane = nextLane;
				nextLane = *(find(futureLanes.begin(), futureLanes.end(), currentLane) + 1);
				getXY(findLane(currentLane, groundTruth), currentCl);
				xy2curv(currentCl, s, psi, kappa);

			}

			//s, currentCl should now contain correct lane

			//setup for width calculation
			osi3::Lane* lane = findLane(currentLane, groundTruth);
			if (lane == nullptr)std::cout << "ERROR lane not found!" << std::endl;
			
			//actual horizon
			for (int k = 1; k < s.size(); k++) {
				
				if ((s[k - 1] + sPast - sStart < ds[i]) && (s[k] + sPast - sStart >= ds[i])) {

					if (s[k - 1] == s[k]) s[k] += 0.3;

					interpolation = (ds[i] - (s[k - 1] + sPast - sStart)) / (s[k] - s[k - 1]);

					Point2D hKnot(currentCl[k - 1].x + interpolation * (currentCl[k].x - currentCl[k - 1].x),
						currentCl[k - 1].y + interpolation * (currentCl[k].y - currentCl[k - 1].y));

					horizon.push_back(hKnot);

					input.horizon.x[i] = std::cos(egoPsi) * (horizon.back().x - lastPosition.x) + std::sin(egoPsi) * (horizon.back().y - lastPosition.y); 
					input.horizon.y[i] = -1.0 * std::sin(egoPsi) * (horizon.back().x - lastPosition.x) + std::cos(egoPsi) * (horizon.back().y - lastPosition.y);
					input.horizon.ds[i] = ds[i];

					input.horizon.psi[i] = psi[k - 1] + interpolation * (psi[k] - psi[k - 1]) - egoPsi;


					input.horizon.kappa[i] = kappa[k - 1] + interpolation * (kappa[k] - kappa[k - 1]);
					
					//width calculation
					
					input.horizon.egoLaneWidth[i] = calcWidth(hKnot, lane, groundTruth);
					
					if (input.horizon.egoLaneWidth[i] == 0) {
						//lane has no boundaries. Take last width value if available or default value
						if (i > 0)
							input.horizon.egoLaneWidth[i] = input.horizon.egoLaneWidth[i - 1];
						else
							input.horizon.egoLaneWidth[i] = defaultWidth;
					}
					//left and right lanes
					osi3::Lane* lLane = lane->classification().left_adjacent_lane_id_size() > 0 ?
						findLane(lane->classification().left_adjacent_lane_id(0).value(), groundTruth) : nullptr;
					osi3::Lane* rLane = lane->classification().right_adjacent_lane_id_size() > 0 ?
						findLane(lane->classification().right_adjacent_lane_id(0).value(), groundTruth) : nullptr;

					input.horizon.rightLaneWidth[i] = rLane != nullptr ? calcWidth(hKnot, rLane, groundTruth) : defaultWidth;
					if (input.horizon.rightLaneWidth[i] == 0) {
						if (i > 0)
							input.horizon.rightLaneWidth[i] = input.horizon.rightLaneWidth[i - 1];
						else
							input.horizon.rightLaneWidth[i] = defaultWidth;
					}

					input.horizon.leftLaneWidth[i] = lLane != nullptr ? calcWidth(hKnot, lLane, groundTruth) : defaultWidth;
					if (input.horizon.leftLaneWidth[i] == 0) {
						if (i > 0)
							input.horizon.leftLaneWidth[i] = input.horizon.leftLaneWidth[i - 1];
						else
							input.horizon.leftLaneWidth[i] = defaultWidth;
					}

				}
			}
		}
	}();
	
*/
	//set helpers because currentCl is overwritten when lane changes
	//p1.x = currentCl.back().x; p1.y = currentCl.back().y;
	//p2.x = currentCl[currentCl.size() - 2].x; p2.y = currentCl[currentCl.size() - 2].y;


	//[&] {
	//	for (int i = 0; i < agent_model::NOH; i++) {

	//		//determine correct lane
	//		while ((s.back() + sPast - sStart) < ds[i])
	//		{
	//			sPast += s.back();

	//			if (!spline) {	//currently on a regular lane. Switching to spline and setting nextLane

	//				if (currentLane == finalLane)return;

	//				nextLane = *(find(futureLanes.begin(), futureLanes.end(), currentLane) + 1);
	//				s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
	//				std::vector<Point2D> tempLane;
	//				getXY(findLane(nextLane, groundTruth), tempLane);

	//				//curved connection? TODO criterium
	//				/*spline3(p1, tempLane.front(), Point2D(p1.x-p2.x, p1.y - p2.y),
	//					Point2D((tempLane[1].x - tempLane[0].x), tempLane[1].y - tempLane[0].y), currentCl);*/

	//					//straight connection?
	//				spline1(p1, tempLane.front(), currentCl);
	//				xy2curv(currentCl, s, psi, kappa);
	//				spline = true;
	//			}
	//			else {			//currently on a spline. Switching to lane determined by nextLane

	//				s.clear(); psi.clear(); kappa.clear(); currentCl.clear();
	//				currentLane = nextLane;
	//				nextLane = 127;
	//				getXY(findLane(currentLane, groundTruth), currentCl);
	//				xy2curv(currentCl, s, psi, kappa);

	//				p1.x = currentCl.back().x; p1.y = currentCl.back().y;
	//				p2.x = currentCl[currentCl.size() - 2].x; p2.y = currentCl[currentCl.size() - 2].y;

	//				spline = false;
	//			}
	//		}

	//		//s, currentCl should now contain correct lane
	//		bool stop = false;

	//		for (int k = 1; k < s.size() && !stop; k++) {

	//			if ((s[k - 1] + sPast - sStart < ds[i]) && (s[k] + sPast - sStart >= ds[i])) {
	//				
	//				if (s[k - 1] == s[k]) s[k] += 0.3;

	//				interpolation = (ds[i] - (s[k - 1] + sPast - sStart)) / (s[k] - s[k - 1]);

	//				Point2D hKnot(currentCl[k - 1].x + interpolation * (currentCl[k].x - currentCl[k - 1].x),
	//					currentCl[k - 1].y + interpolation * (currentCl[k].y - currentCl[k - 1].y));

	//				horizon.push_back(hKnot);


	//				input.horizon.x[i] =  std::cos(egoPsi) * (horizon.back().x - lastPosition.x)+ std::sin(egoPsi) * (horizon.back().y - lastPosition.y); //pre.x * c + pre.y * s; egoPsi
	//				input.horizon.y[i] = -1.0*std::sin(egoPsi) * (horizon.back().x - lastPosition.x) + std::cos(egoPsi) * (horizon.back().y - lastPosition.y); //pre.x * (-1 * s) + pre.y * c;
	//				input.horizon.ds[i] = ds[i];

	//				input.horizon.psi[i] = psi[k - 1] + interpolation * (psi[k] - psi[k - 1]) - egoPsi; //heading of road at horizon point relative to heading of host vehicle. 
	//					//Alternatively: angle between heading of host compared to heading needed to reach horizon point meant here?
	//				
	//				//if(abs(kappa[k - 1] + interpolation * (kappa[k] - kappa[k - 1])) < 2)
	//					input.horizon.kappa[i] = kappa[k - 1] + interpolation * (kappa[k] - kappa[k - 1]);
	//				//else
	//					//input.horizon.kappa[i] = 0;

	//			}
	//		}
	//	}

	//}();
/*	

	if (horizon.size() == 0) {

		horizon.push_back(Point2D(lastPosition.x, lastPosition.y));
		input.horizon.x[0] = horizon.back().x - lastPosition.x;
		input.horizon.y[0] = horizon.back().y - lastPosition.y;
		input.horizon.ds[0] = ds.back();
		input.horizon.psi[0] = 0;
		input.horizon.kappa[0] = 0;
	}
	for (int i = horizon.size(); i < agent_model::NOH; i++) {
		input.horizon.x[i] =  std::cos(egoPsi) * (horizon.back().x - lastPosition.x)+ std::sin(egoPsi) * (horizon.back().y - lastPosition.y); //pre.x * c + pre.y * s; egoPsi
		input.horizon.y[i] = -1.0*std::sin(egoPsi) * (horizon.back().x - lastPosition.x) + std::cos(egoPsi) * (horizon.back().y - lastPosition.y); //pre.x * (-1 * s) + pre.y * c;
		input.horizon.ds[i] = ds.back();
		input.horizon.psi[i] = input.horizon.psi[i - 1];
		input.horizon.kappa[i] = 0;

		Point2D knot(horizon.back().x, horizon.back().y);
		horizon.push_back(knot);
		input.horizon.egoLaneWidth[i] = defaultWidth;
	}
	*/
	for (int i = 0; i < agent_model::NOH; i++) {
		//if (time >= 36 && time <= 37)std::cout << input.horizon.kappa[i] << std::endl;
			//std::cout << "(" << horizon[i].x << "," << horizon[i].y << ") \n";
		horizon_out << input.horizon.x[i] << "," << input.horizon.y[i] << "," << input.horizon.ds[i] << "," << input.horizon.kappa[i] << "\n";
		//horizon_out << input.horizon.ds[i] << "," << input.horizon.kappa[i] << "\n";
	}


	horizon_out.close();
	

	// --- lanes ---	

	int laneCounter = 0;

	// lanes along the host's path constitute one lane

	osi3::Lane lane = *egoLanePtr;
	input.lanes[laneCounter].id = laneMapping[futureLanes[laneCounter]];

	input.lanes[laneCounter].access = agent_model::Accessibility::ACC_ACCESSIBLE; //lanes along route are accessible by definition
	input.lanes[laneCounter].width = input.horizon.egoLaneWidth[0];

	input.lanes[laneCounter].dir = agent_model::DrivingDirection::DD_FORWARDS;

	std::vector<Point2D> lPoints;
	getXY(&lane, lPoints);

	auto it = futureLanes.end();
	double dist = 0;
	Point2D current = lPoints.back();
	// iterate backwards over futureLanes and add distance along all lanes in front of host
	while (it != futureLanes.begin() && *(it - 1) != egoLanePtr->id().value()) {
		it--;
		std::vector<Point2D> pos;
		getXY(findLane(*it, groundTruth), pos);
		dist += xy2s(pos.front(), pos.back(), pos);
		current = pos.front();
	}

	input.lanes[laneCounter].closed = dist + xy2s(egoClPoint, current, lPoints);
	input.lanes[laneCounter].route = input.lanes[laneCounter].closed;

	laneCounter++;

	for (int i = 0; i < groundTruth->lane_size() && laneCounter < agent_model::NOL; i++) {
		osi3::Lane lane = groundTruth->lane(i);
		//skip if lane is in futureLanes, that means it was already considered before
		if (find(futureLanes.begin(), futureLanes.end(), lane.id().value()) != futureLanes.end()) {
			continue;
		}

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
		if (lane.classification().centerline_is_driving_direction() == egoLanePtr->classification().centerline_is_driving_direction())
			input.lanes[i].dir = agent_model::DrivingDirection::DD_FORWARDS;
		else
			input.lanes[i].dir = agent_model::DrivingDirection::DD_BACKWARDS;


		// save average of width
		input.lanes[i].width = defaultWidth;

		std::vector<Point2D> lPoints;

		getXY(&lane, lPoints);

		input.lanes[i].closed = xy2s(lPoints.front(), lPoints.back(), lPoints);
		input.lanes[i].route = input.lanes[i].closed;   // TODO

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


	return 0;
}