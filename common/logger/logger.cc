

#include "logger.h"

#include <cstdlib>
#include <string>
#include <unordered_map>
#include <utility>

#include "glog/logging.h"

#include "log_file_object.h"
#include "logger_util.h"


static std::unordered_map<std::string, LogFileObject*> moduleLoggerMap;

Logger::Logger(google::base::Logger* wrapped) : wrapped_(wrapped) {}

Logger::~Logger() {
  for (auto itr = moduleLoggerMap.begin(); itr != moduleLoggerMap.end();
       ++itr) {
    delete itr->second;
  }
  moduleLoggerMap.clear();
}

void Logger::Write(bool force_flush, time_t timestamp, const char* message,
                   int message_len) {
  std::string log_message = std::string(message, message_len);
  std::string module_name;
  // set the same bracket as the bracket in log.h
  FindModuleName(&log_message, &module_name);

  LogFileObject* fileobject = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (moduleLoggerMap.find(module_name) != moduleLoggerMap.end()) {
      fileobject = moduleLoggerMap[module_name];
    } else {
      std::string file_name = module_name + ".log.INFO.";
      if (!FLAGS_log_dir.empty()) {
        file_name = FLAGS_log_dir + "/" + file_name;
      }
      fileobject = new LogFileObject(google::INFO, file_name.c_str());
      fileobject->SetSymlinkBasename(module_name.c_str());
      moduleLoggerMap[module_name] = fileobject;
    }
  }
  if (fileobject) {
    fileobject->Write(force_flush, timestamp, log_message.c_str(),
                      static_cast<int>(log_message.size()));
  }
}

void Logger::Flush() { wrapped_->Flush(); }

uint32_t Logger::LogSize() { return wrapped_->LogSize(); }
