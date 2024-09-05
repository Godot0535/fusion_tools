#pragma once
#include <string>
#include <vector>

#include "../../common/ICC_common/include/self_dev_vision_input_define.h"

namespace reader {
class ObjTrackReader {
 public:
  bool ReadData(std::string path);
  const TRACK_OBJS& GetData(int frame_id) const;
  TRACK_OBJS GetData(int frame_id);
  bool IsValid(int frame_id) {
    if (frame_id < 0 || frame_id > data.size() - 1) return false;
    return true;
  }

 private:
  std::vector<TRACK_OBJS> data;
};
}  // namespace reader