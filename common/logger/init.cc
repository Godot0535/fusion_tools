//
// Created by lee on 24-5-22.
//

#include "init.h"

#include <glog/logging.h>

#include "async_logger.h"
#include "binary.h"

AsyncLogger* async_logger = nullptr;

void InitLogger(const char* binary_name) {
  const char* slash = strrchr(binary_name, '/');
  if (slash) {
    SetName(slash + 1);
  } else {
    SetName(binary_name);
  }

  // Init glog
  google::InitGoogleLogging(binary_name);
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");

  // Init async logger
  async_logger = new AsyncLogger(google::base::GetLogger(FLAGS_minloglevel));
  google::base::SetLogger(FLAGS_minloglevel, async_logger);
  async_logger->Start();
}

void StopLogger() {
  delete async_logger;
  google::ShutdownGoogleLogging();
}
