
#include "Spdlogger.h"

void Spdlogger::init() {

  path_ = LOG_OUTDIR;
  
  // get time string
  auto time = std::chrono::system_clock::now();
  auto time_c = std::chrono::system_clock::to_time_t(time);
  auto time_tm = *std::localtime(&time_c);
  auto time_string = std::asctime(&time_tm);

  // create new directory if not exist
  struct stat buffer;
  if (stat (path_.c_str(), &buffer) != 0) {
    fs::create_directories(path_); 
  }

  // create sink to write to console
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::trace);
  console_sink->set_pattern("[%^%l%$] %v [Thread: %t] [Time: %H:%M:%S::%e]");

  // create sink to write to file
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path_ + "/logfile - " + time_string + ".txt", true);
  file_sink->set_level(spdlog::level::trace);

  spdlog::sinks_init_list sink_list = { file_sink, console_sink };

  // combine sinks
  spdlog::logger spdlogger("multi_sink", sink_list.begin(), sink_list.end());
  spdlogger.set_level(spdlog::level::debug);
  spdlogger.set_pattern("[%^%l%$] %v [Thread: %t] [Time: %H:%M:%S::%e]");

  // set configuration as default logger
  spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({console_sink, file_sink})));

}