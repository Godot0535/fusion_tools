#pragma once
#include <string>

namespace obj_track {
class ObjTrackConfig {
 public:
  bool ReadConfig(std::string path);

 public:
  int directory_mode{0};
  std::string obj_detect_path;
  std::string obj_track_path;
};
}  // namespace obj_track