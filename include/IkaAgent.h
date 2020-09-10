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
#include "Interface.h"
#include "AgentModel.h"
#include "VehicleModel.h"
#include "PrimaryController.h"
#include <vector>
#include <cmath>



struct Pose {
    double x;
    double y;
	double yaw;
	double dyaw;
	double vx;
	double vy;
	double ax;
	double ay;

};

class IkaAgent : public AgentModel {
public:
    explicit IkaAgent() {}
    ~IkaAgent() override = default;

    virtual void init();
    int step(double time, double stepSize, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out);
    int terminate();


private:
    bool trajSet;
    bool initialized = false;
    osi3::FollowTrajectoryAction traj;
    Pose pose;

    double lastS;
    double horizonTHW;
    agent_model::Position lastPosition;
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

    int getTrajPoint(double time, osi3::TrafficUpdate &out);
	int applyDriverOutput(double time, osi3::TrafficUpdate &out);
	int interpolateState(int iStart, double t);
	int interpolateStateLinear(double t);
};
