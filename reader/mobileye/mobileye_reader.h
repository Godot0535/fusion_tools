#pragma once
#include <string>
#include <vector>
#include "common/mobileye_types.h"

namespace reader {
class MobileyeReader {
 public:
  bool ReadData(std::string path);
  const MOBILEYE_OBSTACLES& GetObjectsData(int frame_id) const;
  const MOBILEYE_LANE_SHAPES& GetLanesData(int frame_id) const;
  bool IsValid(int frame_id) {
    if (frame_id < 0 || frame_id > objs_data.size() - 1) return false;
    return true;
  }

 private:
  std::vector<MOBILEYE_OBSTACLES> objs_data;
  std::vector<MOBILEYE_LANE_SHAPES> lanes_data;
};
}  // namespace reader