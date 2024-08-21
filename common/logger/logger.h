

#pragma once

#include <mutex>

#include "glog/logging.h"


class Logger : public google::base::Logger {
 public:
  explicit Logger(google::base::Logger* wrapped);
  ~Logger();
  void Write(bool force_flush, time_t timestamp, const char* message,
             int message_len) override;
  void Flush() override;
  uint32_t LogSize() override;

 private:
  google::base::Logger* const wrapped_;
  std::mutex mutex_;
};


