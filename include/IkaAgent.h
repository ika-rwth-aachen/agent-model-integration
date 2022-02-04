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
#include "Interface.h"
#include "AgentModel.h"
#include "OsiConverter.h"
#include "VehicleModel.h"
#include "PrimaryController.h"

#include <vector>
#include <cmath>

class IkaAgent : public AgentModel
{
public:
    explicit IkaAgent(){};
    ~IkaAgent(){};

    
    int step(double time, 
				   double step_size, 
				   osi3::SensorView &sensor_view, 
				   osi3::TrafficCommand &traffic_command, 
				   osi3::TrafficUpdate &traffic_update, 
				   setlevel4to5::DynamicsRequest &dynamic_request);

    VehicleModel vehicle;    

private:
    bool initialized = false;

    // converter
    OsiConverter converter;

    // pointers 
    agent_model::State *driver_state; 
    VehicleModel::State *vehicle_state;

    // controllers
    PrimaryController steeringContr;
    PrimaryController pedalContr;

    void init(osi3::BaseMoving &host);
    int buildTrafficUpdate(osi3::TrafficUpdate &traffic_update);
};
