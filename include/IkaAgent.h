/*
 * Author: Daniel Becker
 *
 * (C) 2019 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */ 
#pragma once
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "osi_sensorview.pb.h"
#include "sl45_dynamicsrequest.pb.h"
#include "Interface.h"
#include "AgentModel.h"
#include "VehicleModel.h"
#include "PrimaryController.h"
#include <vector>
#include <cmath>
using Point2D = agent_model::Position;


class IkaAgent : public AgentModel {
public:
    explicit IkaAgent() {};
    ~IkaAgent() {};// = default;

    
    int step(double time, double stepSize, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out, setlevel4to5::DynamicsRequest & dynOut);
    int terminate();
   

private:
    void init();
    bool initialized = false;

    // Ids identifying the last processed TrafficCommands
    int trajActionId = -1;
    int pathActionId = -1;
    int speedActionId = -1;
    int hostVehicleId;

    double lastS;
    double horizonTHW;
    double psi;
    double v;
    agent_model::Position lastPosition;
    std::vector<Point2D> pathCenterLine;
    std::vector<double> pathKappa;
	std::vector<double> pathS;
	std::vector<double> pathPsi;

    // lanes along the path of the agent
    std::vector<int> lanes;
    

    agent_model::Parameters* drParam;
    agent_model::State* drState;
    VehicleModel _vehicle;
    PrimaryController steeringContr;
    PrimaryController pedalContr;
    VehicleModel::Input      *vehInput;
    VehicleModel::State      *vehState;
    VehicleModel::Parameters *vehParam;

    int adapterOsiToInput(osi3::SensorView& sensorView, agent_model::Input& input, std::vector<int>& futureLanes, double time);
    int parseTrafficCommand(osi3::SensorView& sensorViewData, osi3::TrafficCommand& commandData);
    void classifyManeuver(osi3::SensorView& sensorViewData);
    int generateHorizon(osi3::SensorView& sensorView, agent_model::Input& input, std::vector<int>& futureLanes);

	int applyDriverOutput(double time, osi3::TrafficUpdate &out);
};
