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

//using Point2D = agent_model::Position;

void IkaAgent::init(osi3::BaseMoving &host)
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
	drParam->velocity.vComfort = 50.0 / 3.6; //s.u.
	drParam->velocity.ayMax = 1.5;

	// stop control
	drParam->stop.T = 2.0;
	drParam->stop.TMax = 7.0;
	drParam->stop.tSign = 0.5;
	drParam->stop.vStopped = 0.2;
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
	vehParam->steerTransmission = 0.474;
	vehParam->steerTransmission = 0.5;
	vehParam->wheelBase = l;
	vehParam->cwA = 0.6;
	vehParam->mass = 1.5e3;
	vehParam->powerMax = 1.0e4;
	vehParam->forceMax = 6.0e3;
	vehParam->idle = 0.05;
	vehParam->rollCoefficient[0] = 4.0 * 9.91e-3;
	vehParam->rollCoefficient[1] = 4.0 * 1.95e-5;
	vehParam->rollCoefficient[2] = 4.0 * 1.76e-9;
	vehParam->size.x = host.dimension().length();
	vehParam->size.y = host.dimension().width();
	vehParam->driverPosition.x = 0.0; // 0.50;
	vehParam->driverPosition.y = 0.0; //0.5;

	// set controller parameters (lateral motion control)
	steeringContr.setParameters(10.0 * l, 0.1 * l, 0.0, 1.0);
	steeringContr.setRange(-1.0, 1.0, INFINITY);

	// set controller parameters (longitudinal motion control)
	pedalContr.setParameters(1.75, .5, 0.01, 1.0);
	pedalContr.setRange(-1.0, 1.0, INFINITY);

	// set states
	_vehicle.reset();
	vehState->position.x = host.position().x();
	vehState->position.y = host.position().y();
	vehState->v = sqrt(host.velocity().x() * host.velocity().x() + host.velocity().y() * host.velocity().y()); //tbd
	//vehState->v = 40.0 / 3.6;
	//drParam->velocity.vComfort = 12;
	vehState->psi = host.orientation().yaw(); //tbd

	// set variables
	pedalContr.setVariables(&vehState->a, &drState->subconscious.a, &vehInput->pedal, &drState->subconscious.pedal);
	steeringContr.setVariables(&vehState->kappa, &drState->subconscious.kappa, &vehInput->steer);

	pedalContr.reset();
	steeringContr.reset();

	_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;

	// clean output file
	if(debug_files)
	{
		std::ofstream output("debug" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::trunc);
		output.close();
		std::ofstream hor("horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::trunc);
		hor.close();
		std::ofstream hor_test("test_horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::trunc);
		hor_test.close();
	}
}

void IkaAgent::setVehicleSpeed(double v0)
{
	_vehicle.getState()->v = v0;
}
int IkaAgent::step(double time, double stepSize, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out, setlevel4to5::DynamicsRequest &dynOut)
{
	//Initialize agent in first step
	if (!initialized)
	{
		osi3::BaseMoving hostData = sensorViewData.host_vehicle_data().location();
		hostVehicleId = sensorViewData.host_vehicle_id().value();
		IkaAgent::init(hostData);
		initialized = true;
	}

	std::cout << "------------ time: " << out.timestamp().seconds() + (out.timestamp().nanos() * 0.000000001) << " ---------------" << std::endl;
	//check for new traffic command
	if (commandData.action_size() > 0)
	{ //&&commandData.traffic_participant_id().value() == sensorViewData.host_vehicle_id().value(), traffic_participant not set currently
		parseTrafficCommand(sensorViewData, commandData);
		//determine type of maneuver (if crossing intersection)
		classifyManeuver(sensorViewData);
		std::cout << "lanes to pass: ";
		for (auto &lane : lanes)
			std::cout << lane << " ";
		std::cout << std::endl;
		generateHorizon(sensorViewData, lanes);
	}

	//Translate sensorViewData to agent_model::input
	adapterOsiToInput(sensorViewData, _input, lanes, out.timestamp().seconds() + (out.timestamp().nanos() * 0.000000001));

	//perform ika agent model step
	this->AgentModel::step(time);

	//perform controller step that translates acc. and curv. into pedal and steering
	pedalContr.step(stepSize);
	steeringContr.step(stepSize);

	// Perform Vehicle Model step
	vehInput->slope = 0.0;	 //TODO
	_vehicle.step(stepSize); // STEP SIZE not TIME!
	if (debug_files) // replace with debug flag or similar
	{
		std::ofstream output("debug" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::app);
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
		output.close();
	}

	// update outputs: TrafficUpdate and DynamicsRequest
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
int IkaAgent::parseTrafficCommand(osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData)
{
	osi3::GroundTruth *groundTruth = sensorViewData.mutable_global_ground_truth();
	osi3::MovingObject host;
	int startingLaneIdx;
	Point2D destPoint;
	//find Host
	for (int i = 0; i < groundTruth->moving_object_size(); i++)
	{
		if (groundTruth->moving_object(i).id().value() == hostVehicleId)
			host = groundTruth->moving_object(i);
	}

	//needs an initial assigned lane that is unambiguous
	if (host.assigned_lane_id_size() < 2)
	{
		startingLaneIdx = findLaneId(groundTruth, host.assigned_lane_id(0).value());
	}
	else
	{
		int id = closestLane(groundTruth, Point2D(sensorViewData.host_vehicle_data().location().position().x(),
												  sensorViewData.host_vehicle_data().location().position().y()));
		startingLaneIdx = findLaneId(groundTruth, id);
	}

	for (int i = 0; i < commandData.action_size(); i++)
	{
		if (commandData.action(i).has_acquire_global_position_action())
		{
			osi3::TrafficAction_AcquireGlobalPositionAction globPos = commandData.action(i).acquire_global_position_action();
			destPoint = Point2D(globPos.position().x(), globPos.position().y());
			lanes.clear();
			futureLanes(groundTruth, startingLaneIdx, destPoint, lanes);
		}
		if (commandData.action(i).has_follow_trajectory_action() && (commandData.action(i).follow_trajectory_action().action_header().action_id().value() != trajActionId))
		{
			osi3::TrafficAction_FollowTrajectoryAction traj = commandData.action(i).follow_trajectory_action();
			trajActionId = traj.action_header().action_id().value();
			destPoint = Point2D(traj.trajectory_point(traj.trajectory_point_size() - 1).position().x(), traj.trajectory_point(traj.trajectory_point_size() - 1).position().y());
			lanes.clear();
			futureLanes(groundTruth, startingLaneIdx, destPoint, lanes);
		}
		if (commandData.action(i).has_follow_path_action() && (commandData.action(i).follow_path_action().action_header().action_id().value() != pathActionId))
		{
			osi3::TrafficAction_FollowPathAction path = commandData.action(i).follow_path_action();
			pathActionId = path.action_header().action_id().value();
			destPoint = Point2D(path.path_point(path.path_point_size() - 1).position().x(), path.path_point(path.path_point_size() - 1).position().y());
			lanes.clear();
			futureLanes(groundTruth, startingLaneIdx, destPoint, lanes);
		}
		if (commandData.action(i).has_speed_action() && (commandData.action(i).speed_action().action_header().action_id().value() != speedActionId))
		{
			osi3::TrafficAction_SpeedAction speed = commandData.action(i).speed_action();
			speedActionId = speed.action_header().action_id().value();
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
void IkaAgent::classifyManeuver(osi3::SensorView &sensorViewData)
{

	osi3::GroundTruth *groundTruth = sensorViewData.mutable_global_ground_truth();

	// Maneuver is only applicable when the host traverses an intersection --> one futureLane must be of type INTERSECION
	// Assuming there is only one intersection along the path, considering the first encountered
	std::vector<Point2D> pos;
	std::vector<double> s, k, p;
	bool isOsiStandard = false;

	for (auto it : lanes)
	{
		osi3::Lane *lane = findLane(it, groundTruth);
		if (lane->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION)
			if (lane->classification().free_lane_boundary_id_size() > 0)
			{
				isOsiStandard = true;
				std::cout << "intersection is modeled as free lane boundary (as stated in the standard).";
			}
			else
				getXY(lane, pos);
	}

	// TODO: calculate maneuver when intersection is an area

	/*std::cout << "curve: ";
	for (auto& p:pos) std::cout << p.x << "," << p.y << " ";
	std::cout << "\n";*/
	if (pos.empty())
		_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;
	else
	{
		for (int i = 0; i < pos.size() - 1; i++)
		{
			double dsNext = (pos[i + 1].x - pos[i].x) * (pos[i + 1].x - pos[i].x) + (pos[i + 1].y - pos[i].y) * (pos[i + 1].y - pos[i].y);

			if (dsNext < 0.1)
			{
				pos.erase(pos.begin() + i);
				i--;
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
			else if (avg < (-1 * eps))
				_input.vehicle.maneuver = agent_model::Maneuver::TURN_RIGHT;
			else
				_input.vehicle.maneuver = agent_model::Maneuver::STRAIGHT;
		}
	}
	std::cout << "\nMANEUVER (0=straight, 1=left, 2=right): " << _input.vehicle.maneuver << std::endl;
}

int IkaAgent::terminate()
{
	return 0;
}

int IkaAgent::applyDriverOutput(double time, osi3::TrafficUpdate &out)
{
	osi3::MovingObject *update = out.mutable_update(0);

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

/**
 * @brief Generates horizon and junction paths in init phase
 * 
 *
 * @param sensorView osi sensor view from first time step
 * @param futureLanes the lanes the driver will pass on its path
 */
int IkaAgent::generateHorizon(osi3::SensorView &sensorView, std::vector<int> &futureLanes)
{
	// first get all relevant points from futureLanes
	osi3::GroundTruth *groundTruth = sensorView.mutable_global_ground_truth();

	int gapIdx=0;
	for (auto &l : futureLanes)
	{
		if (findLane(l, groundTruth)->classification().free_lane_boundary_id_size() > 0 
		 && findLane(l, groundTruth)->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION)
		{ // mark gap and compute points later, when gap end is known as well
			gapIdx = pathCenterLine.size()-1;
		} 
		else
			getXY(findLane(l, groundTruth), pathCenterLine);
	}

	if (gapIdx > 0)
	{
		std::vector<Point2D> gapPts(&pathCenterLine[gapIdx-1], &pathCenterLine[gapIdx+3]);
		calcXYgap(gapPts, pathCenterLine, gapIdx);
	}

	// remove duplicates
	for (int i = 0; i < pathCenterLine.size() - 1; i++)
	{
		double dsNext = (pathCenterLine[i + 1].x - pathCenterLine[i].x) * (pathCenterLine[i + 1].x - pathCenterLine[i].x) + (pathCenterLine[i + 1].y - pathCenterLine[i].y) * (pathCenterLine[i + 1].y - pathCenterLine[i].y);

		if (dsNext < 0.01)
		{
			pathCenterLine.erase(pathCenterLine.begin() + i);
			i--;
		}
	}

	// calculate s, kappa, and psi
	xy2curv(pathCenterLine, pathS, pathPsi, pathKappa);

	if (debug_files) // replace with debug flag or similar
	{
		std::ofstream test_hor("test_horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::app);
		for (int i = 0; i < pathCenterLine.size(); i++)
		{
			test_hor << pathCenterLine[i].x << ","
					 << pathCenterLine[i].y << ","
					 << pathS[i] << ","
					 << pathPsi[i] << ","
					 << pathKappa[i] << "\n";
		}
		test_hor.close();
	}

	

	// generate junction paths for traffic rules
	std::vector<int> knownStartingLid;
	for (auto &sig : *groundTruth->mutable_traffic_sign())
	{
		if (sig.main_sign().classification().type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN 
		|| sig.main_sign().classification().type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY)
		{
			// create path from assigned lane of sign backwards
			for (auto &asLane : sig.main_sign().classification().assigned_lane_id())
			{
				junctionPath tmp;
				osi3::Lane *sigLane = findLane(asLane.value(), groundTruth);
				tmp.signalId = sig.id().value();

				int lid = asLane.value();

				if (std::find(knownStartingLid.begin(), knownStartingLid.end(), lid) != knownStartingLid.end())
					continue;
				else
					knownStartingLid.push_back(lid);
				bool found_next_lane = true;
				tmp.laneIds.push_back(lid);
				while (found_next_lane)
				{
					auto *lane = findLane(lid, groundTruth);
					for (auto &lpairs : lane->classification().lane_pairing())
					{
						found_next_lane = false;
						int lidSuc = lpairs.successor_lane_id().value();
						int lidAnt = lpairs.antecessor_lane_id().value();
						if (lane->classification().centerline_is_driving_direction())
						{
							if (lidSuc == lid)
							{
								found_next_lane = true;
								tmp.laneIds.push_back(lidAnt);
								lid = lidAnt;
								break;
							}
						}
						else
						{
							if (lidAnt == lid)
							{
								found_next_lane = true;
								tmp.laneIds.push_back(lidSuc);
								lid = lidSuc;
								break;
							}
						}
					}
				}
				//why not both conditions?
				if (sig.main_sign().classification().type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY)
					yieldingLanes.push_back(tmp);
				else
					priorityLanes.push_back(tmp);
			}
		}
	}

	for (auto &yl : yieldingLanes)
	{
		std::reverse(yl.laneIds.begin(), yl.laneIds.end());
		for (auto &l : yl.laneIds)
			getXY(findLane(l, groundTruth), yl.pts);
	}
	for (auto &pl : priorityLanes)
	{
		std::reverse(pl.laneIds.begin(), pl.laneIds.end());
		for (auto &l : pl.laneIds)
			getXY(findLane(l, groundTruth), pl.pts);
	}

	return 0;
}

/**
 * @brief fills the fields of agent_model interface based on data from OSI SensorView
 *
 * @param sensorView
 * @param input
 * @param time
 */
int IkaAgent::adapterOsiToInput(osi3::SensorView &sensorView, agent_model::Input &input, std::vector<int> &futureLanes, double time)
{
	osi3::GroundTruth *groundTruth = sensorView.mutable_global_ground_truth();

	
	// --- ego ---
	// save properties needed often
	int egoId = sensorView.host_vehicle_id().value();
	osi3::Lane *egoLanePtr = nullptr;
	osi3::MovingObject egoObj;
	osi3::BaseMoving egoBase;

	// find ego object
	for (int i = 0; i < groundTruth->moving_object_size(); i++)
	{
		if (groundTruth->moving_object(i).id().value() == egoId)
		{
			egoObj = groundTruth->moving_object(i);
			egoBase = egoObj.base();
			break;
		}
	}

	//determine assigned lane
	if (egoObj.assigned_lane_id_size() < 2)
		egoLanePtr = findLane(egoObj.assigned_lane_id(0).value(), groundTruth);
	else
	{
		//multiple assigned_lanes (intersection)
		//iterate over future lanes in reverse --> the first assigned_lane encoutered is correct
		for (int j = futureLanes.size() - 1; j >= 0; j--)
		{
			for (int i = 0; i < egoObj.assigned_lane_id_size(); i++)
				if (find(futureLanes.begin(), futureLanes.end(), egoObj.assigned_lane_id(i).value()) != futureLanes.end())
					egoLanePtr = findLane(egoObj.assigned_lane_id(i).value(), groundTruth);
		}
	}

	//save ego lane centerline coordinates for later
	//std::vector<Point2D> elPoints;
	//getXY(egoLanePtr, elPoints);
	//std::cout  << "assigned_lane: " << egoLanePtr->id().value() << "\n";

	input.vehicle.v = sqrt(egoBase.velocity().x() * egoBase.velocity().x() + egoBase.velocity().y() * egoBase.velocity().y());
	input.vehicle.a = sqrt(egoBase.acceleration().x() * egoBase.acceleration().x() + egoBase.acceleration().y() * egoBase.acceleration().y());

	//compute ego road coordinates and set member variables to new values
	double dx = egoBase.position().x() - lastPosition.x;
	double dy = egoBase.position().y() - lastPosition.y;

	// calculate traveled distance
	lastPosition.x = egoBase.position().x();
	lastPosition.y = egoBase.position().y();
	lastS += sqrt(dx * dx + dy * dy);
	input.vehicle.s = lastS;

	//save angle of ego x-Axis/s-Axix because targets.psi and horizon.psi are relative to it
	double egoPsi = egoBase.orientation().yaw();
	input.vehicle.dPsi = egoBase.orientation_rate().yaw();

	//projection of ego coordiantes on centerline
	Point2D egoClPoint;
	//closestCenterlinePoint(lastPosition, elPoints, egoClPoint);
	int curCLidx = closestCenterlinePoint(lastPosition, pathCenterLine, egoClPoint);

	// calculate s, psi, k of ego lane
	std::vector<double> s, psi, k;
	xy2curv(pathCenterLine, s, psi, k);
	// seems to have errors...
	input.vehicle.psi = egoPsi - interpolateXY2value(psi, pathCenterLine, egoClPoint);

	double dirCL = atan2(egoClPoint.y - lastPosition.y, egoClPoint.x - lastPosition.x);
	double orientation = egoPsi - dirCL;
	if (orientation < -M_PI)
		orientation = orientation + 2 * M_PI;
	if (orientation > M_PI)
		orientation = orientation - 2 * M_PI;
	int d_sig = (orientation > 0) - (orientation < 0);
	double distCL = sqrt(pow(lastPosition.x - egoClPoint.x, 2) + pow(lastPosition.y - egoClPoint.y, 2));
	input.vehicle.d = d_sig * distCL;

	//dummy values
	input.vehicle.pedal = 0;	// TODO
	input.vehicle.steering = 0; // TODO

	// --- signals ---
	int signal = 0;
	std::vector<int> knownAsLane;

	std::vector<Point2D> knownSigPosition; 			//save all known/processed signal positions in this vector
	std::vector<int> allTLS_IDs;					//save all IDs of TrafficLight signals
	//::vector<osi3::Identifier> allTLS_IDs;

	auto it_knownSigPosition = knownSigPosition.begin();
	//auto it_allTLS_IDs = allTLS_IDs.begin();

	//std::vector<Point2D>::iterator it_knownSigPosition = knownSigPosition.begin();
	//std::vector<osi3::Identifier>::iterator it_allTLS_IDs = allTLS_IDs.begin();
	

	int DEBUG_TLS_Size = groundTruth->traffic_light_size();
	int DEBUG_SIGN_Size = groundTruth->traffic_sign_size();
	int DEBUG_ID_Value;

	for(int i = 0; i < groundTruth->traffic_light_size(); i++)
	{
		if (signal>= agent_model::NOTL)
			break;
		//if (signal>= agent_model::NOS + agent_model::NOTL)
		//	break;

		osi3::TrafficLight light = groundTruth->traffic_light(i);
		osi3::TrafficLight_Classification clas = light.classification();

		auto it = futureLanes.end();
		// check: sign is assigned to lane along the route?
		bool assigned = false;
		for (int k = 0; k < clas.assigned_lane_id_size(); k++)
		{
			if(find(knownAsLane.begin(),knownAsLane.end(),clas.assigned_lane_id(k).value()) !=knownAsLane.end())
				continue;
			else
				knownAsLane.push_back(clas.assigned_lane_id(k).value());

			it = find(futureLanes.begin(), futureLanes.end(),clas.assigned_lane_id(k).value());
			if(it != futureLanes.end())
				{assigned = true; 	break;}

		}

		if (!assigned)
			continue;
		//projection of trafficlight_signal position on centerline: cPoint
		Point2D cPoint;
		Point2D sPoint(light.base().position().x(), light.base().position().y());
		Point2D target = sPoint;

		closestCenterlinePoint(sPoint, pathCenterLine, cPoint);

		// sLight holds s value along centerlines to reach the signal
		double sLight = xy2s_sgn(egoClPoint, cPoint, pathCenterLine, egoPsi);
		//std::cout << "Spoint: (" << sPoint.x << "," << sPoint.y << ")" << std::endl;

		//signal on future lane / deleted comments

		input.signals[signal].id = signal + 1; //could also take light.id().value() as id here, OSI ids are often larger numbers
		input.signals[signal].ds = sLight;


		//save postition to check later on: signal is standalone signal or sign combined with TLS?
		knownSigPosition.push_back(sPoint);
		///xxx
		
		//int k = light.id().value();
		//save all TLS IDs to assign IDs of combined signs later on
		allTLS_IDs.push_back(light.id().value());

		DEBUG_ID_Value = light.id().value();
		


		//TODO: Write function to set Icons
		// Calculate Color and Type/Icon
		if (clas.color() == osi3::TrafficLight_Classification_Color_COLOR_RED)
		{
			//if(clas.is_out_of_service() == osi3::TrafficLight_Classification::is_out_of_service)
			//osi3::TrafficLight_Classification::is_out_of_service;

			input.signals[signal].color = agent_model::COLOR_RED;

			if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
				input.signals[signal].icon =  agent_model::ICON_NONE;
			else if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
				input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
			else if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
				input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;	
		}
		else if (clas.color() == osi3::TrafficLight_Classification_Color_COLOR_YELLOW)
		{
			input.signals[signal].color = agent_model::COLOR_YELLOW;

			if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
				input.signals[signal].icon = agent_model::ICON_NONE;
			else if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
				input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
			else if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
				input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;	
		}
		else if (clas.color() == osi3::TrafficLight_Classification_Color_COLOR_GREEN)
		{
			input.signals[signal].color = agent_model::COLOR_GREEN;
			
			if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_NONE)
				input.signals[signal].icon = agent_model::ICON_NONE;
			else if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_ARROW_LEFT)
				input.signals[signal].icon = agent_model::ICON_ARROW_LEFT;
			else if(clas.icon() == osi3::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT)
				input.signals[signal].icon = agent_model::ICON_ARROW_RIGHT;	
		}

	signal++;
	}

	int amountLights = signal;

	auto it_allTLS_IDs = allTLS_IDs.begin();
   
	for (int i = 0; i < groundTruth->traffic_sign_size(); i++)
	{
		if (signal >= agent_model::NOS)
			break;

		osi3::TrafficSign sign = groundTruth->traffic_sign(i);
		osi3::TrafficSign_MainSign_Classification clas = sign.main_sign().classification();

		auto it = futureLanes.end();
		// check that the sign is assigned to a lane along the route.
		bool assigned = false;
		for (int j = 0; j < clas.assigned_lane_id_size(); j++)
		{
			if (find(knownAsLane.begin(), knownAsLane.end(), clas.assigned_lane_id(j).value()) != knownAsLane.end())
				continue;
			else
				knownAsLane.push_back(clas.assigned_lane_id(j).value());

			it = find(futureLanes.begin(), futureLanes.end(), clas.assigned_lane_id(j).value());
			if (it != futureLanes.end())
			{
				assigned = true;
				//std::cout << "\nsignal on lane: " << clas.assigned_lane_id(j).value() << std::endl;
				break;
			}
		}

		if (!assigned)
			continue;

		// sSig will hold s value along centerlines to reach the signal
		//projection of signal position on centerline: cPoint
		Point2D cPoint;
		Point2D sPoint(sign.main_sign().base().position().x(), sign.main_sign().base().position().y());
		//closestCenterlinePoint(sPoint, elPoints, cPoint);
		//knownSigPosition.push_back(sPoint);

		Point2D target = sPoint;
		closestCenterlinePoint(sPoint, pathCenterLine, cPoint);
		double sSig = xy2s_sgn(egoClPoint, cPoint, pathCenterLine, egoPsi);
		//std::cout << "Spoint: (" << sPoint.x << "," << sPoint.y << ")" << std::endl;
		//signal on future lane
		/*if (*it != egoLanePtr->id().value()) {
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

		sSig += xy2s(egoClPoint, target, elPoints);*/

		input.signals[signal].id = signal + 1; //could also take sign.id().value() as id here, OSI ids are often larger numbers
		input.signals[signal].ds = sSig;
		
	
		//check if signal position is already known (due to prior signs)
		it_knownSigPosition = find(knownSigPosition.begin(),knownSigPosition.end(),sPoint);

		if (it_knownSigPosition != knownSigPosition.end())
		{
			input.signals[signal].subsignal=true;
		}

		int itDistance[3];
		int k = 0;

		while((it_knownSigPosition != knownSigPosition.end()))
		{
			itDistance[k] = it_knownSigPosition - knownSigPosition.begin();
			it_knownSigPosition = find(it_knownSigPosition,knownSigPosition.end(),sPoint);	

			k++;
			if(k>=3)
				break;
		}

		if (!allTLS_IDs.empty()/*||amountLights != 0*/)
		{
			for (int k = 0; k < 3; k++)
			{
				//it_allTLS_IDs is at same index as it_knownSigPosition was
				std::advance(it_allTLS_IDs, itDistance[k]);
				//it_allTLS_IDs + itDistance[k];
				input.signals[signal].pairedSignalID[k] = *it_allTLS_IDs;
			}

			//check if whole Trafficlight is out of service -> Sign must be in use
			if (groundTruth->mutable_traffic_light()->Get(input.signals[signal].pairedSignalID[0]).classification().is_out_of_service() 
			 && groundTruth->mutable_traffic_light()->Get(input.signals[signal].pairedSignalID[1]).classification().is_out_of_service() 
			 && groundTruth->mutable_traffic_light()->Get(input.signals[signal].pairedSignalID[2]).classification().is_out_of_service())
				input.signals[signal].sign_is_in_use = true;
			else
				input.signals[signal].sign_is_in_use = false;
		}
		else
		{
			input.signals[signal].subsignal = false;
			input.signals[signal].sign_is_in_use = true;
		}

		//if (find(knownSigPosition.begin(),knownSigPosition.end(),sPoint) != knownSigPosition.end())
		//	input.signals[signal].subsignal=true;



		// calculate type
		if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_STOP)
		{
			input.signals[signal].type = agent_model::SIGNAL_STOP;
			//no value for stop
			input.signals[signal].value = 0;
		}
		else if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN)
		{
			input.signals[signal].type = agent_model::SIGNAL_SPEED_LIMIT;
			input.signals[signal].value = clas.value().value();
		}
		else if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_OF_WAY_BEGIN)
		{
			//for (int j = 0; j < clas.assigned_lane_id_size(); j++)
			//	priorityLanes.push_back(clas.assigned_lane_id(j).value());
			input.signals[signal].type = agent_model::SIGNAL_PRIORITY;
		}
		else if (clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY ||
				 clas.type() == osi3::TrafficSign_MainSign_Classification_Type_TYPE_RIGHT_BEFORE_LEFT_NEXT_INTERSECTION)
		{

			//for (int j = 0; j < clas.assigned_lane_id_size(); j++)
			//	yieldingLanes.push_back(clas.assigned_lane_id(j).value());
			input.signals[signal].type = agent_model::SIGNAL_YIELD;
		}
		else
		{
			input.signals[signal].type = agent_model::SIGNAL_NOT_SET;
			//no value for unset type
			input.signals[signal].value = 0;
		}

		signal++;
	}


	//fill remaining signals with default values
	for (int i = signal; i < agent_model::NOS; i++)
	{
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

	std::vector<int> intersectionLanes;

	for (int i = 0; i < groundTruth->lane_size(); i++)
	{
		osi3::Lane lane = groundTruth->lane(i);
		// Workaround until intersection type is filled TODO
		if (lane.classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION)
			//if(groundTruth->lane(i).id().value() == 200024 || groundTruth->lane(i).id().value() == 200017 || groundTruth->lane(i).id().value() == 200035 || groundTruth->lane(i).id().value() == 200005  )
			intersectionLanes.push_back(lane.id().value());

		/*for (int j = 0; j < lane.classification().lane_pairing_size(); j++) {
			if (findLane(lane.classification().lane_pairing(j).successor_lane_id().value(), groundTruth)->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
				intersectionLanes.push_back(lane.id().value());
			}
		}*/
	}

	int ti = 0;
	for (auto &mo : *groundTruth->mutable_moving_object())
	//for(int k=0; k<groundTruth->moving_object_size();k++)
	{
		//auto mo = groundTruth->moving_object(k);
		if (mo.id().value() == egoId)
			continue;

		osi3::BaseMoving moBase = mo.base();
		input.targets[ti].id = ti + 1;
		input.targets[ti].priority = agent_model::TARGET_PRIORITY_NOT_SET;
		input.targets[ti].dsIntersection = 0;

		// determine whether target is assigned to a lane along the host's path (following possible)
		bool onPath = false;
		auto it = futureLanes.end();
		int commonLaneIdx = 0;
		for (int j = 0; j < mo.assigned_lane_id_size(); j++)
		{

			if (std::find(intersectionLanes.begin(), intersectionLanes.end(), mo.assigned_lane_id(j).value()) != intersectionLanes.end())
				input.targets[ti].priority = agent_model::TARGET_ON_INTERSECTION;

			it = find(futureLanes.begin(), futureLanes.end(), mo.assigned_lane_id(j).value());
			if (it != futureLanes.end())
			{
				commonLaneIdx = j;
				onPath = true;
				break;
			}
		}

		// only fill ds, lane and d fields when target is along the host's path
		if (onPath)
		{
			Point2D cPoint;
			Point2D bPoint(moBase.position().x(), moBase.position().y());

			closestCenterlinePoint(bPoint, pathCenterLine, cPoint);
			//osi3::Lane *tarLane = findLane(mo.assigned_lane_id(commonLane).value(), groundTruth);
			//Point2D current = bPoint;

			double sTar = xy2s_sgn(egoClPoint, bPoint, pathCenterLine, egoPsi);
			/*double sTar = 0;
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
			sTar += xy2s(egoClPoint, current, elPoints);*/

			double sTarNet;
			if (sTar > 0)
				sTarNet = sTar - 0.5 * moBase.dimension().length() - 0.5 * egoBase.dimension().length();
			else
				sTarNet = sTar + 0.5 * moBase.dimension().length() + 0.5 * egoBase.dimension().length();

			input.targets[ti].ds = sTarNet; //TODO: check what happens when target is behind host vehicle
			input.targets[ti].d = sqrt(pow(moBase.position().x() - cPoint.x, 2) + pow(moBase.position().y() - cPoint.y, 2));
			input.targets[ti].lane = laneMapping[mo.assigned_lane_id(commonLaneIdx).value()];
		}
		else
		{
			input.targets[ti].ds = INFINITY;
			input.targets[ti].d = 0;
			input.targets[ti].lane = 127;

			if (input.targets[ti].priority != agent_model::TARGET_ON_INTERSECTION)
			{
				//for (int j = 0; j < mo.assigned_lane_id_size(); j++) {
				for (auto &asLane : mo.assigned_lane_id())
				{
					junctionPath tmp;
					bool apprJunc = false;
					for (auto &yieldL : yieldingLanes)
					{
						// target is on its way on a yield lane to the junction
						if (find(yieldL.laneIds.begin(), yieldL.laneIds.end(), asLane.value()) != yieldL.laneIds.end())
						{
							input.targets[ti].priority = agent_model::TARGET_ON_GIVE_WAY_LANE;
							tmp.laneIds = yieldL.laneIds;
							tmp.pts = yieldL.pts;
							apprJunc = true;
						}
					}
					if (!apprJunc)
					{
						for (auto &prioL : priorityLanes)
						{
							// target is on its way on a priority lane to the junction
							if (find(prioL.laneIds.begin(), prioL.laneIds.end(), asLane.value()) != prioL.laneIds.end())
							{
								input.targets[ti].priority = agent_model::TARGET_ON_PRIORITY_LANE;
								tmp.laneIds = prioL.laneIds;
								tmp.pts = prioL.pts;
								apprJunc = true;
							}
						}
					}
					if (apprJunc)
						input.targets[ti].dsIntersection = xy2s(Point2D(moBase.position().x(), moBase.position().y()), tmp.pts.back(), tmp.pts);

					/*if (find(intersectionLanes.begin(), intersectionLanes.end(), mo.assigned_lane_id(j).value())
						!= intersectionLanes.end()) {
						//target is on a lane ending in an intersection
						std::vector<Point2D> cl;
						getXY(findLane(mo.assigned_lane_id(j).value(), groundTruth), cl);
						input.targets[ti].dsIntersection = xy2s(Point2D(base.position().x(), base.position().y()), cl.back(), cl);
						if (find(priorityLanes.begin(), priorityLanes.end(), mo.assigned_lane_id(j).value())
							!= priorityLanes.end())
							input.targets[ti].priority = agent_model::TARGET_ON_PRIORITY_LANE;
						else if (find(yieldingLanes.begin(), yieldingLanes.end(), mo.assigned_lane_id(j).value())
							!= yieldingLanes.end())
							input.targets[ti].priority = agent_model::TARGET_ON_GIVE_WAY_LANE;

						break;
					}
					else if (findLane(mo.assigned_lane_id(j).value(), groundTruth)->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
						input.targets[ti].priority = agent_model::TARGET_ON_INTERSECTION;
					}*/
				}
			}
		}

		input.targets[ti].xy.x = moBase.position().x() - lastPosition.x;
		input.targets[ti].xy.y = moBase.position().y() - lastPosition.y;

		input.targets[ti].v = sqrt(moBase.velocity().x() * moBase.velocity().x() + moBase.velocity().y() * moBase.velocity().y());
		input.targets[ti].a = sqrt(moBase.acceleration().x() * moBase.acceleration().x() + moBase.acceleration().y() * moBase.acceleration().y());
		input.targets[ti].psi = moBase.orientation().yaw() - egoPsi;

		input.targets[ti].size.length = moBase.dimension().length();
		input.targets[ti].size.width = moBase.dimension().width();

		ti++;
	}
	for (int i = ti; i < agent_model::NOT; i++)
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
		input.targets[i].priority = agent_model::TARGET_PRIORITY_NOT_SET;
	}
	/*
	for (int i = 0; i < agent_model::NOT; i++)
	{

		if (i < groundTruth->moving_object_size())
		{
			osi3::MovingObject egoObj = groundTruth->moving_object(i);

			if (egoObj.id().value() == egoId)
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
			osi3::BaseMoving base = egoObj.base();

			input.targets[i].id = i+1;

			auto it = futureLanes.end(); 
			bool assigned = false;

			double sTar = 0;

			// determine whether target is assigned to a lane along the host's path (following possible)
			for (int j = 0; j < egoObj.assigned_lane_id_size(); j++) {

				it = find(futureLanes.begin(), futureLanes.end(), egoObj.assigned_lane_id(j).value());

				if (it != futureLanes.end()) {
					assigned = true;
					break;
				}
			}

			for (int j = 0; j < egoObj.assigned_lane_id_size(); j++) {
				if (find(lanesEnteringIntersection.begin(), lanesEnteringIntersection.end(), egoObj.assigned_lane_id(j).value())
					!= lanesEnteringIntersection.end()) {
					//target is on a lane ending in an intersection

					std::vector<Point2D> cl;
					getXY(findLane(egoObj.assigned_lane_id(j).value(), groundTruth), cl);
					input.targets[i].dsIntersection = xy2s(Point2D(base.position().x(), base.position().y()), cl.back(), cl);
					if (find(priorityLanes.begin(), priorityLanes.end(), egoObj.assigned_lane_id(j).value())
						!= priorityLanes.end())
						input.targets[i].priority = agent_model::TARGET_ON_PRIORITY_LANE;
					else if (find(yieldingLanes.begin(), yieldingLanes.end(), egoObj.assigned_lane_id(j).value())
						!= yieldingLanes.end())
						input.targets[i].priority = agent_model::TARGET_ON_GIVE_WAY_LANE;

					break;
				}
				else if (findLane(egoObj.assigned_lane_id(j).value(), groundTruth)->classification().type() == osi3::Lane_Classification_Type_TYPE_INTERSECTION) {
					input.targets[i].priority = agent_model::TARGET_ON_INTERSECTION;
				}
			}

			// only fill ds and d fields when target is along the host's path
			if (assigned) {
				Point2D cPoint;
				Point2D bPoint(base.position().x(), base.position().y());

				closestCenterlinePoint(bPoint, elPoints, cPoint);

				osi3::Lane* tarLane = findLane(egoObj.assigned_lane_id(0).value(), groundTruth);

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


			input.targets[i].lane = laneMapping[egoObj.assigned_lane_id(0).value()];

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

	// --- horizon ---

	double sMax = std::max(15.0, horizonTHW * input.vehicle.v);
	//double sMax = 100;

	//distance (along centerline) to each horizon point from current location
	std::vector<double> ds(agent_model::NOH, 0);

	double delta = sqrt(sMax) / double(agent_model::NOH); //linear spaces
	for (int i = 0; i < agent_model::NOH; i++)
	{ //create squared spaces
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
	double sTravel = pathS[hor_idx - 1] + sqrt((lastPosition.x - pathCenterLine[hor_idx - 1].x) * (lastPosition.x - pathCenterLine[hor_idx - 1].x) + (lastPosition.y - pathCenterLine[hor_idx - 1].y) * (lastPosition.y - pathCenterLine[hor_idx - 1].y));
	//std::cout << "sTravel: " << sTravel << " after idx: " << hor_idx << "\n";

	for (int i = 0; i < agent_model::NOH; i++)
	{
		double dsCur = (i + 1) * (i + 1) * delta * delta;
		// get correct space for interpolation
		int j = -1;
		for (auto &ss : pathS)
		{
			if (ss > sTravel + dsCur)
				break;
			j++;
		}

		Point2D hKnot;
		if (sTravel + dsCur < pathS.back())
		{
			interpolation = (sTravel + dsCur - pathS[j]) / (pathS[j + 1] - pathS[j]);
			hKnot.x = pathCenterLine[j].x + interpolation * (pathCenterLine[j + 1].x - pathCenterLine[j].x);
			hKnot.y = pathCenterLine[j].y + interpolation * (pathCenterLine[j + 1].y - pathCenterLine[j].y);

			input.horizon.ds[i] = dsCur;
			input.horizon.psi[i] = pathPsi[j] + interpolation * (pathPsi[j + 1] - pathPsi[j]) - egoPsi;
			input.horizon.kappa[i] = pathKappa[j] + interpolation * (pathKappa[j + 1] - pathKappa[j]);
		}
		else
		{
			hKnot.x = pathCenterLine.back().x;
			hKnot.y = pathCenterLine.back().y;
			input.horizon.ds[i] = pathS.back() - sTravel;
			input.horizon.psi[i] = 0;
			input.horizon.kappa[i] = 0;
		}

		input.horizon.x[i] = std::cos(egoPsi) * (hKnot.x - lastPosition.x) + std::sin(egoPsi) * (hKnot.y - lastPosition.y);
		input.horizon.y[i] = -1.0 * std::sin(egoPsi) * (hKnot.x - lastPosition.x) + std::cos(egoPsi) * (hKnot.y - lastPosition.y);
		// TODO!!
		input.horizon.egoLaneWidth[i] = 3.75;
		input.horizon.leftLaneWidth[i] = 0;
		input.horizon.rightLaneWidth[i] = 0;
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

	if (debug_files) // replace with debug flag or similar
	{
		std::ofstream horizon_out("horizon" + std::to_string(hostVehicleId) + ".txt", std::ofstream::out | std::ofstream::app);
		for (int i = 0; i < agent_model::NOH; i++)
		{
			horizon_out << input.horizon.x[i] << "," << input.horizon.y[i] << "," << input.horizon.ds[i] << "," << input.horizon.kappa[i] << "\n";
		}
		horizon_out.close();
	}

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
	while (it != futureLanes.begin() && *(it - 1) != egoLanePtr->id().value())
	{
		it--;
		std::vector<Point2D> pos;
		getXY(findLane(*it, groundTruth), pos);
		dist += xy2s(pos.front(), pos.back(), pos);
		current = pos.front();
	}

	input.lanes[laneCounter].closed = dist + xy2s(egoClPoint, current, lPoints);
	input.lanes[laneCounter].route = input.lanes[laneCounter].closed;

	laneCounter++;

	for (int i = 0; i < groundTruth->lane_size() && i < agent_model::NOL; i++)
	{
		osi3::Lane lane = groundTruth->lane(i);
		//skip if lane is in futureLanes, that means it was already considered before
		if (find(futureLanes.begin(), futureLanes.end(), lane.id().value()) != futureLanes.end())
		{
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
		input.lanes[i].route = input.lanes[i].closed; // TODO

		laneCounter++;
	}
	while (laneCounter < agent_model::NOL)
	{
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
