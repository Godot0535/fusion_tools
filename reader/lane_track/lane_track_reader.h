#pragma once
#include <string>
#include <vector>

#include "common/icc_common/include/percept_lane_output_define.h"

namespace reader {
class LaneTrackReader {
 public:
  bool ReadData(std::string path);
  const TRACK_LANES& GetData(int frame_id) const;
  bool IsValid(int frame_id) {
    if (frame_id < 0 || frame_id > data.size() - 1) return false;
    return true;
  }

 private:
  std::vector<TRACK_LANES> data;
};
}  // namespace reader