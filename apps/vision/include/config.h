#pragma once
#include <string>

namespace vision {
class VisionConfig {
 public:
  bool ReadConfig(std::string path);

 public:
  bool display_switch{false};
  bool undistort{false};
  int obj_detect{0};
  int obj_track{0};
  int lane_detect{0};
  int lane_track{0};
  int obj_ranging{0};
  int radar{0};
  int fusion{0};
  int mobileye{0};
  int interval{0};

  int directory_mode{1};
  std::string image_path;
  std::string obj_detect_path;
  std::string obj_track_path;
  std::string lane_detect_path;
  std::string lane_track_path;
  std::string obj_ranging_path;
  std::string radar_path;
  std::string fusion_path;
  std::string mobileye_path;

  bool image_save{false};
  bool video_save{false};
  float video_fps{20.0};
  std::string image_save_path;
  std::string video_save_path;
};
}  // namespace vision