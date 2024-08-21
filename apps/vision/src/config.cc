#include "config.h"
#include "3rdparty/toml11/toml.hpp"

namespace vision {
bool VisionConfig::ReadConfig(std::string config_path) {
  const auto config_all = toml::parse(config_path);

  const auto &display = toml::find(config_all, "display");
  display_switch = toml::find<int>(display, "switch");
  undistort = toml::find<int>(display, "undistort");
  obj_detect = toml::find<int>(display, "obj_detect");
  obj_track = toml::find<int>(display, "obj_track");
  lane_detect = toml::find<int>(display, "lane_detect");
  lane_track = toml::find<int>(display, "lane_track");
  obj_ranging = toml::find<int>(display, "obj_ranging");
  radar = toml::find<int>(display, "radar");
  fusion = toml::find<int>(display, "fusion");
  mobileye = toml::find<int>(display, "mobileye");
  interval = toml::find<int>(display, "interval");

  const auto &path = toml::find(config_all, "path");
  directory_mode = toml::find<int>(path, "directory_mode");
  image_path = toml::find<std::string>(path, "image");
  obj_detect_path = toml::find<std::string>(path, "obj_detect");
  obj_track_path = toml::find<std::string>(path, "obj_track");
  lane_detect_path = toml::find<std::string>(path, "lane_detect");
  lane_track_path = toml::find<std::string>(path, "lane_track");
  obj_ranging_path = toml::find<std::string>(path, "obj_ranging");
  radar_path = toml::find<std::string>(path, "radar");
  fusion_path = toml::find<std::string>(path, "fusion");
  mobileye_path = toml::find<std::string>(path, "mobileye");

  const auto &save = toml::find(config_all, "save");
  image_save = toml::find<int>(save, "image_save");
  video_save = toml::find<int>(save, "video_save");
  video_fps = toml::find<float>(save, "video_fps");
  image_save_path = toml::find<std::string>(save, "image_save_path");
  video_save_path = toml::find<std::string>(save, "video_save_path");

  return true;
}
}  // namespace vision