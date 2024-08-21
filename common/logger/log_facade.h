//
// Created by lee on 24-4-8.
//

#pragma once

/**
 * 日志门面，主要是在日志前加上前缀，区分不同模块，以及以后方便替换其他日志实现
 */

#include <cstdarg>
#include <string>

#include <glog/logging.h>
#include <glog/raw_logging.h>

#include "binary.h"
#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

#ifndef MODULE_NAME
#define MODULE_NAME GetName().c_str()
#endif

#define DEBUG_MODULE(module) \
  VLOG(4) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG] "
#define LOG_DEBUG DEBUG_MODULE(MODULE_NAME)
#define LOG_INFO LOG_MODULE(MODULE_NAME, INFO)
#define LOG_WARN LOG_MODULE(MODULE_NAME, WARN)
#define LOG_ERROR LOG_MODULE(MODULE_NAME, ERROR)
#define LOG_FATAL LOG_MODULE(MODULE_NAME, FATAL)

#ifndef LOG_MODULE_STREAM
#define LOG_MODULE_STREAM(log_severity) LOG_MODULE_STREAM_##log_severity
#endif

#ifndef LOG_MODULE
#define LOG_MODULE(module, log_severity) \
  LOG_MODULE_STREAM(log_severity)(module)
#endif

#define LOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define LOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define LOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define LOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define LOG_INFO_IF(cond) LOG_MODULE_IF(INFO, cond, MODULE_NAME)
#define LOG_WARN_IF(cond) LOG_MODULE_IF(WARN, cond, MODULE_NAME)
#define LOG_ERROR_IF(cond) LOG_MODULE_IF(ERROR, cond, MODULE_NAME)
#define LOG_FATAL_IF(cond) LOG_MODULE_IF(FATAL, cond, MODULE_NAME)
#define LOG_MODULE_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
          : google::LogMessageVoidify() & LOG_MODULE(module, severity)

#define MODULE_CHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define LOG_INFO_EVERY(freq) \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define LOG_WARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define LOG_ERROR_EVERY(freq) \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    LOG_WARN << #ptr << " is nullptr."; \
    return;                          \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    LOG_WARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)           \
  if (condition) {                     \
    LOG_WARN << #condition << " is met."; \
    return;                            \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)  \
  if (condition) {                     \
    LOG_WARN << #condition << " is met."; \
    return val;                        \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
  if (ptr == nullptr) {               \
    return (val);                     \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
  if (condition) {                     \
    return (val);                      \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
  if (condition) {            \
    return;                   \
  }
#endif
