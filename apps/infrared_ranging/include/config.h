#pragma once
#include <string>

namespace infrared_ranging {
class InfraredRangingConfig {
 public:
  bool ReadConfig(std::string path);

 public:
  int infrared_track_mode{0};

  std::string infrared_track_path;
  std::string infrared_ranging_path;

  int display_switch{0};
  int image_mode{1};
  int interval{0};
  std::string image_path;

  int video_save{1};
  std::string video_save_path;
};
}  // namespace obj_ranging