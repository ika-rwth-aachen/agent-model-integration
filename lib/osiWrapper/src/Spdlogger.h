#pragma once

#include <fstream>
#include <sys/stat.h>

#include "../lib/spdlog/include/spdlog/spdlog.h"
#include "../lib/spdlog/include/spdlog/sinks/basic_file_sink.h"
#include "../lib/spdlog/include/spdlog/sinks/stdout_color_sinks.h"

#include <chrono>
#include <ctime>

#if __cplusplus < 201703L // If the version of C++ is less than 17
#include <experimental/filesystem>
    // It was still in the experimental:: namespace
    namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
    namespace fs = std::filesystem;
#endif

class Spdlogger {
  public:
    Spdlogger(){};
    ~Spdlogger(){};

    void init();

  private:
    std::string path_;
    std::shared_ptr<spdlog::logger> spdlogger;
};