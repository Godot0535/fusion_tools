#pragma once
#include <string>

namespace infrared_track {
class InfraredTrackConfig {
 public:
  bool ReadConfig(std::string path);

 public:
  int directory_mode{0};
  std::string infrared_detect_path;
  std::string infrared_track_path;
};
}  // namespace infrared_track