#pragma once

#include <fstream>
#include <sys/stat.h>
#include <nlohmann/json.hpp>

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

class Logger {
  public:
    Logger(){};
    ~Logger(){};

    void init(uint64_t ego_id);
    void saveDebugInformation(double time, agent_model::Input input, agent_model::State *driver_state, VehicleModel::State *vehicle_state);

  private:
    uint64_t ego_id_;

    json json_logger_;
    int json_counter_ = 0;
    
    std::string path_;
    double dt_log_ = 0.1;
    double dt_save_ = 1.0;
};