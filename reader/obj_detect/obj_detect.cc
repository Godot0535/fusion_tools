#include "obj_detect.h"

#include <fstream>
#include <sstream>

#include "common/utils.h"

namespace reader {
bool ObjDetectReader::ReadData(std::string path) {
  data.clear();
  std::vector<std::string> files;
  GetFileNames(path, files);
  for (auto& file : files) {
    std::ifstream data_in(file, std::ios::binary);
    if (!data_in.is_open()) {
      return false;
    }

    DETECT_OBJS objs;
    objs.objs_num = 0;

    std::string data_line;
    getline(data_in, data_line);  // 第一行为标题，直接略过
    while (getline(data_in, data_line)) {
      std::istringstream data_stream(data_line);  // 每一行的数据流
      std::string unuse;
      int category, left, right, top, bottom = 0;
      int sub_left, sub_right, sub_top, sub_bottom = 0;
      data_stream >> unuse >> category >> left >> top >> right >> bottom >>
          objs.detect_objs[objs.objs_num].confidence >> sub_left >> sub_top >>
          sub_right >> sub_bottom >> objs.detect_objs[objs.objs_num].sub_box_confidence;
      objs.detect_objs[objs.objs_num].obj_category =
          static_cast<OBJ_CATEGORY>(category);
      objs.detect_objs[objs.objs_num].box.left_top_u =
          static_cast<uint16_t>(left);
      objs.detect_objs[objs.objs_num].box.left_top_v =
          static_cast<uint16_t>(top);
      objs.detect_objs[objs.objs_num].box.right_bottom_u =
          static_cast<uint16_t>(right);
      objs.detect_objs[objs.objs_num].box.right_bottom_v =
          static_cast<uint16_t>(bottom);
      objs.detect_objs[objs.objs_num].sub_box.left_top_u =
          static_cast<uint16_t>(sub_left);
      objs.detect_objs[objs.objs_num].sub_box.left_top_v =
          static_cast<uint16_t>(sub_top);
      objs.detect_objs[objs.objs_num].sub_box.right_bottom_u =
          static_cast<uint16_t>(sub_right);
      objs.detect_objs[objs.objs_num].sub_box.right_bottom_v =
          static_cast<uint16_t>(sub_bottom);
      objs.objs_num++;
    }
    data.push_back(objs);
  }
  return true;
}

const DETECT_OBJS& ObjDetectReader::GetData(int frame_id) const {
  return data.at(frame_id);
}
}  // namespace reader
