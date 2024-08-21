#include "apps/obj_track/include/config.h"

#include "3rdparty/toml11/toml.hpp"

namespace obj_track {
bool ObjTrackConfig::ReadConfig(std::string config_path) {
  const auto config_all = toml::parse(config_path);
  const auto &path = toml::find(config_all, "path");
  directory_mode = toml::find<int>(path, "directory_mode");
  obj_detect_path = toml::find<std::string>(path, "obj_detect");
  obj_track_path = toml::find<std::string>(path, "obj_track");
  return true;
}
}  // namespace obj_track