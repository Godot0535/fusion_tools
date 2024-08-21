#pragma once
#include <string>
#include <vector>

#include "common/ICC_common/include/percept_lane_output_define.h"

namespace reader {
class LaneDetectReader {
 public:
  bool ReadData(std::string path);
  const DETECT_LANES& GetData(int frame_id) const;
  bool IsValid(int frame_id) {
    if (frame_id < 0 || frame_id > data.size() - 1) return false;
    return true;
  }

 private:
  std::vector<DETECT_LANES> data;
};
}  // namespace reader