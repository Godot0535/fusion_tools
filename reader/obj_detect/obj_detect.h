#pragma once
#include <string>
#include <vector>

#include "../../common/ICC_common/include/self_dev_vision_input_define.h"

namespace reader {
class ObjDetectReader {
 public:
  bool ReadData(std::string path);
  const DETECT_OBJS& GetData(int frame_id) const;
  bool IsValid(int frame_id) {
    if (frame_id < 0 || frame_id > data.size() - 1) return false;
    return true;
  }

 private:
  std::vector<DETECT_OBJS> data;
};
}  // namespace reader
