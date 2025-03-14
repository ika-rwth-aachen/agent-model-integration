/*
 * Author: Christian Geller
 *
 * (C) 2022 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "Logger.h"

void Logger::init(uint64_t ego_id, bool debug=false) {
  
  debug_ = debug;

  ego_id_ = ego_id;
  path_debug_ = DEBUG_OUTDIR;
  path_log_ = LOG_OUTDIR;

  // create new directories if not exist
  struct stat buffer;
  if (debug_ && stat (path_debug_.c_str(), &buffer) != 0) {
    fs::create_directories(path_debug_); 
  }
  if (stat (path_log_.c_str(), &buffer) != 0) {
    fs::create_directories(path_log_); 
  }

  // get time string
  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  char output[20];
  strftime(output, 20, "%Y-%m-%d_%H-%M-%S", timeinfo);
  debug_string = std::to_string(ego_id) + "_" + std::string(output);


  // setup logging sinks
  std::vector<spdlog::sink_ptr> sinks;
  
  // create sink to write to console
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
  console_sink->set_level(spdlog::level::trace);
  console_sink->set_pattern("[%^%l%$] %v");
  sinks.push_back(console_sink);

  // create sink to write to file
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>(path_log_ + "/log_" + debug_string + ".txt", true);
  file_sink->set_pattern("[%^%l%$] %v [%s #%# Function: %!] [Time: %H:%M:%S::%e]");
  sinks.push_back(file_sink);

  // set configuration as default logger
  std::shared_ptr<spdlog::logger> spd_logger = std::make_shared<spdlog::logger>("logger", begin(sinks), end(sinks));
  spd_logger->flush_on(spdlog::level::trace);

  spdlog::set_default_logger(spd_logger);

  SPDLOG_INFO("Logger is enabled for id {} and files are stored in: {}", ego_id_, path_log_);

  if (debug_){
    SPDLOG_INFO("Logger is enabled for id {} and files are stored in: {}", ego_id_, path_debug_);
  }
}

void Logger::saveOSI(osi3::SensorView &sensor_view,
                   osi3::TrafficCommand &traffic_command) {
  
  if (!debug_) return;
  
  std::string sensor_view_string;
  sensor_view.SerializeToString(&sensor_view_string);

  std::ofstream file_sensor_view;
  file_sensor_view.open(path_debug_ + "/sensor_view_" + debug_string + ".osi", std::ofstream::app|std::ofstream::binary);
  file_sensor_view.imbue(std::locale::classic());

  uint32_t val_sv = (uint32_t) sensor_view.ByteSizeLong();
  file_sensor_view.write(reinterpret_cast<const char*>(&val_sv), sizeof val_sv);

  file_sensor_view << sensor_view_string;
  file_sensor_view.close();


  std::string traffic_command_string;
  traffic_command.SerializeToString(&traffic_command_string);

  std::ofstream file_traffic_command;
  file_traffic_command.open(path_debug_ + "/traffic_command_" + debug_string + ".osi", std::ofstream::app|std::ofstream::binary);
  file_traffic_command.imbue(std::locale::classic());

  uint32_t val_tc = (uint32_t) traffic_command.ByteSizeLong();
  file_traffic_command.write(reinterpret_cast<const char*>(&val_tc), sizeof val_tc);

  file_traffic_command << traffic_command_string;
  file_traffic_command.close();


  std::string debug_sensorview_string = sensor_view.DebugString();
  std::ofstream debug_sensorview (path_debug_ + "/sensor_view_" + debug_string + ".txt",  std::ofstream::app);
  debug_sensorview << debug_sensorview_string;
  debug_sensorview.close();


  std::string debug_trafficcommand_string = traffic_command.DebugString();
  std::ofstream debug_trafficcommand (path_debug_ + "/traffic_command_" + debug_string + ".txt",  std::ofstream::app);
  debug_trafficcommand << debug_trafficcommand_string;
  debug_trafficcommand.close();
}


void Logger::saveDebugInformation(double time, agent_model::Input input, agent_model::State *driver_state, VehicleModel::State *vehicle_state) {
  
  if (!debug_) return;

  // convert time and dt_log to milliseconds (uint64_t) to allow modulo operator
  // add 0.5 for proper rounding
  if (uint64_t(1000*time + 0.5) % uint64_t(1000*dt_log_ + 0.5) == 0) {

    json json_conscious_follow;
    json_conscious_follow["distance_0"] = driver_state->conscious.follow.targets[0].distance;
    json_conscious_follow["velocity_0"] = driver_state->conscious.follow.targets[0].velocity; 
    json_conscious_follow["distance_1"] = driver_state->conscious.follow.targets[1].distance;
    json_conscious_follow["velocity_1"] = driver_state->conscious.follow.targets[1].velocity;
    json_conscious_follow["standing"] = driver_state->conscious.follow.standing;

    json json_conscious_stop;
    json_conscious_stop["ds"] = driver_state->conscious.stop.ds;
    json_conscious_stop["dsMax"] = driver_state->conscious.stop.dsMax;
    json_conscious_stop["standing"] = driver_state->conscious.stop.standing;

    
    json json_conscious_lateral;
    for (int i = 0; i < 3; i++)
    {
      json_conscious_lateral[i]["factor"] = driver_state->conscious.lateral.paths[i].factor;
      json_conscious_lateral[i]["offset"] = driver_state->conscious.lateral.paths[i].offset;
      json_conscious_lateral[i]["ref_point_x"] = driver_state->conscious.lateral.paths[i].refPoints->x;
      json_conscious_lateral[i]["ref_point_y"] = driver_state->conscious.lateral.paths[i].refPoints->y;
    }

    json json_conscious_velocity;
    json_conscious_velocity["local"] = driver_state->conscious.velocity.local;
    json_conscious_velocity["prediction"] = driver_state->conscious.velocity.prediction;

    json json_conscious;
    json_conscious["follow"] = json_conscious_follow; 
    json_conscious["stop"] = json_conscious_stop;
    json_conscious["lateral"] = json_conscious_lateral;
    json_conscious["velocity"] = json_conscious_velocity; 
    
    json json_subconscious;
    json_subconscious["a"] = driver_state->subconscious.a;
    json_subconscious["dPsi"] = driver_state->subconscious.dPsi;
    json_subconscious["kappa"] = driver_state->subconscious.kappa;
    json_subconscious["pedal"] = driver_state->subconscious.pedal;
    json_subconscious["steering"] = driver_state->subconscious.steering;

    json json_driver_state;
    json_driver_state["conscious"] = json_conscious;
    json_driver_state["subconscious"] = json_subconscious;
    
    json json_vehicle_state;
    json_vehicle_state["a"] = vehicle_state->a;
    json_vehicle_state["ay"] = vehicle_state->ay;
    json_vehicle_state["d_psi"] = vehicle_state->d_psi;
    json_vehicle_state["d_psi"] = vehicle_state->d_psi;
    json_vehicle_state["delta"] = vehicle_state->delta;
    json_vehicle_state["ds"] = vehicle_state->ds;
    json_vehicle_state["force"] = vehicle_state->force;
    json_vehicle_state["kappa"] = vehicle_state->kappa;
    json_vehicle_state["position_x"] = vehicle_state->position.x;
    json_vehicle_state["position_y"] = vehicle_state->position.y;
    json_vehicle_state["psi"] = vehicle_state->psi;
    json_vehicle_state["s"] = vehicle_state->s;
    json_vehicle_state["v"] = vehicle_state->v;

    json json_horizon;
    json_horizon["x"] = input.horizon.x;
    json_horizon["y"] = input.horizon.y;
    json_horizon["ds"] = input.horizon.ds;
    json_horizon["kappa"] = input.horizon.kappa;
    json_horizon["psi"] = input.horizon.psi;

    json json_ego_input;
    json_ego_input["v"] = input.vehicle.v;
    json_ego_input["a"] = input.vehicle.a;
    json_ego_input["psi"] = input.vehicle.psi;
    json_ego_input["dPsi"] = input.vehicle.dPsi;
    json_ego_input["s"] = input.vehicle.s;
    json_ego_input["d"] = input.vehicle.d;

    // concatenate debug information
    json_logger_[json_counter_]["ego_id"] = ego_id_;
    json_logger_[json_counter_]["time"] = time;
    json_logger_[json_counter_]["vehicle_state"] = json_vehicle_state;
    json_logger_[json_counter_]["driver_state"] = json_driver_state;
    json_logger_[json_counter_]["ego_input"] = json_ego_input;
    json_logger_[json_counter_]["horizon"] = json_horizon;

    json_counter_++;
  }

  // save debug file
  if (uint64_t(1000*time + 0.5) % uint64_t(1000*dt_save_ + 0.5) == 0) {
    std::ofstream output(path_debug_ + "/vehicle_" + std::to_string(ego_id_) + "_" + debug_string + ".json", std::ofstream::out);
    output << json_logger_.dump(4);
    output.close();
  }
}