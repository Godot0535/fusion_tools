#include "apps/infrared_ranging/include/config.h"
#include <iostream>
#include "3rdparty/toml11/toml.hpp"

namespace infrared_ranging {
bool InfraredRangingConfig::ReadConfig(std::string config_path) {
  const auto config_all = toml::parse(config_path);
  const auto &mode = toml::find(config_all, "mode");
  infrared_track_mode = toml::find<int>(mode, "infrared_track");

  const auto &path = toml::find(config_all, "path");
  infrared_track_path = toml::find<std::string>(path, "infrared_track");
  infrared_ranging_path = toml::find<std::string>(path, "infrared_ranging");

  const auto &show = toml::find(config_all, "show");
  display_switch = toml::find<int>(show, "display_switch");
  image_mode = toml::find<int>(show, "image_mode");
  interval = toml::find<int>(show, "interval");
  image_path = toml::find<std::string>(show, "image_path");

  video_save_path = "/home/liuyiming/workspace/data/demo_1_1_new.avi";
  return true;
};
}  // namespace infrared_ranging