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

void IkaAgent::init(osi3::BaseMoving &host)
{
	// get global pointers
    driver_state = this->getState();
    vehicle_state = vehicle.getState();


	// set driver parameters
    agent_model::Parameters *driver_param = this->getParameters();	

	double wheel_base = 3.22;

	// velocity components
	driver_param->velocity.a = 2.0;
	driver_param->velocity.b = -2.0;
	driver_param->velocity.thwMax = 10.0;
	driver_param->velocity.delta = 4.0;
	driver_param->velocity.deltaPred = 3.0;
	driver_param->velocity.vComfort = 50.0 / 3.6;
	driver_param->velocity.ayMax = 1.5;

	// stop components
	driver_param->stop.T = 2.0;
	driver_param->stop.TMax = 7.0;
	driver_param->stop.tSign = 0.5;
	driver_param->stop.vStopped = 0.2;
	driver_param->stop.pedalDuringStanding = -0.3;

	// following components
	driver_param->follow.dsStopped = 2.0;
	driver_param->follow.thwMax = 10.0;
	driver_param->follow.timeHeadway = 1.8;

	// steering components
	driver_param->steering.thw[0] = 1.0;
	driver_param->steering.thw[1] = 3.0;
	driver_param->steering.dsMin[0] = 1.0;
	driver_param->steering.dsMin[1] = 3.0;
	driver_param->steering.P[0] = 0.03 * wheel_base;
	driver_param->steering.P[1] = 0.015 * wheel_base;
	driver_param->steering.D[0] = 0.1;
	driver_param->steering.D[1] = 0.1;

	AgentModel::init();
	

	// set vehicle parameters
	VehicleModel::Parameters *vehicle_param = vehicle.getParameters();
	VehicleModel::Input *vehicle_input = vehicle.getInput();

	vehicle_param->steerTransmission = 0.474;
	vehicle_param->steerTransmission = 0.5;
	vehicle_param->wheelBase = wheel_base;
	vehicle_param->mass = 1.5e3;
	vehicle_param->powerMax = 1.0e4;
	vehicle_param->forceMax = 6.0e3;
	vehicle_param->idle = 0.05;
	vehicle_param->rollCoefficient[0] = 4.0 * 9.91e-3;
	vehicle_param->rollCoefficient[1] = 4.0 * 1.95e-5;
	vehicle_param->rollCoefficient[2] = 4.0 * 1.76e-9;
	vehicle_param->size.x = host.dimension().length();
	vehicle_param->size.y = host.dimension().width();
	vehicle_param->driverPosition.x = 0.0;
	vehicle_param->driverPosition.y = 0.0;

	// set initial vehicle state
	vehicle.reset();
	vehicle_state->position.x = host.position().x();
	vehicle_state->position.y = host.position().y();
	osi3::Vector3d v = host.velocity();
	vehicle_state->v = sqrt(pow(v.x(),2) + pow(v.y(),2) + pow(v.z(),2));
	vehicle_state->psi = host.orientation().yaw();


	// set controller parameters (lateral motion control)
	steeringContr.setParameters(10.0 * wheel_base, 0.1 * wheel_base, 0.0, 1.0);
	steeringContr.setRange(-1.0, 1.0, INFINITY);

	// set controller parameters (longitudinal motion control)
	pedalContr.setParameters(1.75, .5, 0.01, 1.0);
	pedalContr.setRange(-1.0, 1.0, INFINITY);


	// set variables
	pedalContr.setVariables(&vehicle_state->a, &driver_state->subconscious.a, &vehicle_input->pedal, &driver_state->subconscious.pedal);
	steeringContr.setVariables(&vehicle_state->kappa, &driver_state->subconscious.kappa, &vehicle_input->steer);

	pedalContr.reset();
	steeringContr.reset();
}

int IkaAgent::step(double time, 
				   double step_size, 
				   osi3::SensorView &sensor_view, 
				   osi3::TrafficCommand &traffic_command, 
				   osi3::TrafficUpdate &traffic_update, 
				   setlevel4to5::DynamicsRequest &dynamic_request)
{
	std::cout << "----------- time: " << time << " --------------" << std::endl;
	
	// initialize agent
	if (!initialized)
	{	
		osi3::BaseMoving host = sensor_view.host_vehicle_data().location();
		IkaAgent::init(host);
		converter.init(this->getParameters());
		initialized = true;
	}

	// converter converts from osi to agent_model::input
	converter.convert(sensor_view, traffic_command, _input);

	// ika agent model step
	this->AgentModel::step(time);

	// controller translates acc. and curv. to pedal and steering
	pedalContr.step(step_size);
	steeringContr.step(step_size);

	// vehicle model step
	vehicle.getInput()->slope = 0.0; // CGE do we need this?
	vehicle.step(step_size);

	// update DynamicsRequest TrafficUpdate
	dynamic_request.set_longitudinal_acceleration_target(driver_state->subconscious.a);
	dynamic_request.set_curvature_target(driver_state->subconscious.kappa);
	this->buildTrafficUpdate(traffic_update);

	return 0;
}

int IkaAgent::buildTrafficUpdate(osi3::TrafficUpdate &traffic_update)
{
	osi3::MovingObject *object = traffic_update.mutable_update(0);

	object->mutable_base()->mutable_position()->set_x(vehicle_state->position.x);
	object->mutable_base()->mutable_position()->set_y(vehicle_state->position.y);
	object->mutable_base()->mutable_velocity()->set_x(vehicle_state->v);
	object->mutable_base()->mutable_velocity()->set_y(0);
	object->mutable_base()->mutable_acceleration()->set_x(vehicle_state->a);
	object->mutable_base()->mutable_acceleration()->set_y(0);
	object->mutable_base()->mutable_orientation()->set_yaw(vehicle_state->psi);
	object->mutable_base()->mutable_orientation_rate()->set_yaw(vehicle_state->dPsi);

	return 0;
}