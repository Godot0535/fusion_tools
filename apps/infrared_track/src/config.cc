#include "apps/infrared_track/include/config.h"

#include "3rdparty/toml11/toml.hpp"

namespace infrared_track {
bool InfraredTrackConfig::ReadConfig(std::string config_path) {
  const auto config_all = toml::parse(config_path);
  const auto &path = toml::find(config_all, "path");
  directory_mode = toml::find<int>(path, "directory_mode");
  infrared_detect_path = toml::find<std::string>(path, "infrared_detect");
  infrared_track_path = toml::find<std::string>(path, "infrared_track");
  return true;
}
}  // namespace infrared_track