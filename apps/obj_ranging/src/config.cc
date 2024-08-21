#include "apps/obj_ranging/include/config.h"

#include "3rdparty/toml11/toml.hpp"

namespace obj_ranging {
bool ObjRangingConfig::ReadConfig(std::string config_path) {
  const auto config_all = toml::parse(config_path);
  const auto &mode = toml::find(config_all, "mode");
  obj_track_mode = toml::find<int>(mode, "obj_track");
  lane_detect_mode = toml::find<int>(mode, "lane_detect");

  const auto &path = toml::find(config_all, "path");
  obj_track_path = toml::find<std::string>(path, "obj_track");
  lane_detect_path = toml::find<std::string>(path, "lane_detect");
  lane_track_path = toml::find<std::string>(path, "lane_track");
  obj_ranging_path = toml::find<std::string>(path, "obj_ranging");

  const auto &show = toml::find(config_all, "show");
  display_switch = toml::find<int>(show, "display_switch");
  image_mode = toml::find<int>(show, "image_mode");
  interval = toml::find<int>(show, "interval");
  image_path = toml::find<std::string>(show, "image_path");
  mobileye_path = toml::find<std::string>(show, "mobileye_path");
  return true;
}
}  // namespace obj_ranging