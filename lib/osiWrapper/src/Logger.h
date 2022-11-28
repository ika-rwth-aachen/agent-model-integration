#pragma once

#include <fstream>
#include <sys/stat.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <unistd.h>
#include <string>
#include <iostream>

#include "osi_sensorview.pb.h"
#include "osi_trafficcommand.pb.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "AgentModel.h"
#include "Interface.h"
#include "VehicleModel.h"

#if __cplusplus < 201703L // If the version of C++ is less than 17
#include <experimental/filesystem>
    // It was still in the experimental:: namespace
    namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
    namespace fs = std::filesystem;
#endif

using json = nlohmann::json;
using std::filesystem::current_path;

class Logger {
  public:
    Logger(){};
    ~Logger(){
      spdlog::drop_all();
      spdlog::shutdown();
    };

    void init(uint64_t ego_id, bool debug);
    void saveDebugInformation(double time, agent_model::Input input, agent_model::State *driver_state, VehicleModel::State *vehicle_state);

    void saveOSI(osi3::SensorView &sensor_view,
                   osi3::TrafficCommand &traffic_command);

  private:

    bool debug_ = false;

    uint64_t ego_id_;
    std::string time_string;

    json json_logger_;
    int json_counter_ = 0;
    
    std::string path_debug_;
    std::string path_log_;
    double dt_log_ = 0.1;
    double dt_save_ = 1.0;
};