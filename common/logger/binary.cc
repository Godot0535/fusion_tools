//
// Created by lee on 24-5-21.
//

#include "binary.h"


#include <mutex>
#include <string>

namespace {
std::mutex m;
std::string binary_name; // NOLINT
}  // namespace


std::string GetName() {
  std::lock_guard<std::mutex> lock(m);
  return binary_name;
}
void SetName(const std::string& name) {
  std::lock_guard<std::mutex> lock(m);
  binary_name = name;
}