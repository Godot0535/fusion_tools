#pragma once
#include <string>

namespace obj_ranging {
class ObjRangingConfig {
 public:
  bool ReadConfig(std::string path);

 public:
  int obj_track_mode{0};
  int lane_detect_mode{0};
  std::string obj_track_path;
  std::string lane_detect_path;
  std::string lane_track_path;
  std::string obj_ranging_path;

  int display_switch{0};
  int image_mode{1};
  int interval{0};
  std::string image_path;
  std::string mobileye_path;
};
}  // namespace obj_ranging