/*
 * Author: Christian Geller
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
#include <vector>
#include <cmath>
using Point2D = agent_model::Position;

struct junctionPath
{
    std::vector<int> laneIds;
    std::vector<Point2D> pts;
    int signalId;
};

class OsiConverter
{
public:
    OsiConverter(){};
    ~OsiConverter(){};

    void init(AgentModel::Parameters *driver_param);

    void convert(osi3::SensorView &sensor_view, 
                osi3::TrafficCommand &traffic_command,
                agent_model::Input &input,
                double time);
    
    void convert2(osi3::SensorView &sensor_view,
                agent_model::Input &input, double time);//CGE

private:
    bool debug = false; //CGE sould be defined with compile flags

    AgentModel::Parameters *driver_param;

    // last action id's
    int trajActionId = -1;
    int pathActionId = -1;
    int speedActionId = -1;

    // last values
    agent_model::Position lastPosition;
    double lastS = 0;

    // path vectors
    std::vector<Point2D> pathCenterLine;
    std::vector<double> pathKappa;
    std::vector<double> pathS;
    std::vector<double> pathPsi;

    // lanes
    std::vector<int> lanes;
    std::vector<junctionPath> priorityLanes;
    std::vector<junctionPath> yieldingLanes;


    void processTrafficCommand(osi3::SensorView &sensor_view,  
                               osi3::TrafficCommand &traffic_command,
                               agent_model::Input &input);
    void classifyManeuver(osi3::SensorView &sensor_view,
                          agent_model::Input &input);
    void generatePath(osi3::SensorView &sensor_view,
                        agent_model::Input &input);
};
