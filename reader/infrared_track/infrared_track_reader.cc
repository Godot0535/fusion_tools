#include <fstream>
#include <sstream>

#include "../../common/utils.h"
#include "infrared_track_reader.h"

namespace reader {
bool InfraredTrackReader::ReadData(std::string path) {
  data.clear();
  std::vector<std::string> files;
  GetFileNames(path, files);
  for (auto& file : files) {
    // std::cout << "----------------------" << std::endl;
    std::ifstream data_in(file, std::ios::binary);
    if (!data_in.is_open()) {
      return false;
    }

    TRACK_OBJS objs;
    objs.objs_num = 0;

    std::string data_line;
    while (getline(data_in, data_line)) {
      std::istringstream data_stream(data_line);  // 每一行的数据流
      int category = 0;
      float left, right, top, bottom = 0;
      int det_left, det_right, det_top, det_bottom = 0;
      int sub_left, sub_right, sub_top, sub_bottom = 0;
      // std::cout << data_line << std::endl;
      data_stream >> objs.track_objs[objs.objs_num].track_id >> left >> top >>
          right >> bottom >> objs.track_objs[objs.objs_num].confidence >>
          category >> det_left >> det_top >> det_right >> det_bottom >>
          objs.track_objs[objs.objs_num].obj_info.confidence >> sub_left >>
          sub_top >> sub_right >> sub_bottom >>
          objs.track_objs[objs.objs_num].obj_info.sub_box_confidence;
      objs.track_objs[objs.objs_num].obj_info.obj_category =
          static_cast<OBJ_CATEGORY>(category);
      objs.track_objs[objs.objs_num].box.left_top_u =
          static_cast<uint16_t>(left);
      objs.track_objs[objs.objs_num].box.left_top_v =
          static_cast<uint16_t>(top);
      objs.track_objs[objs.objs_num].box.right_bottom_u =
          static_cast<uint16_t>(right);
      objs.track_objs[objs.objs_num].box.right_bottom_v =
          static_cast<uint16_t>(bottom);
      objs.track_objs[objs.objs_num].obj_info.box.left_top_u =
          static_cast<uint16_t>(det_left);
      objs.track_objs[objs.objs_num].obj_info.box.left_top_v =
          static_cast<uint16_t>(det_top);
      objs.track_objs[objs.objs_num].obj_info.box.right_bottom_u =
          static_cast<uint16_t>(det_right);
      objs.track_objs[objs.objs_num].obj_info.box.right_bottom_v =
          static_cast<uint16_t>(det_bottom);
      objs.track_objs[objs.objs_num].obj_info.sub_box.left_top_u =
          static_cast<uint16_t>(sub_left);
      objs.track_objs[objs.objs_num].obj_info.sub_box.left_top_v =
          static_cast<uint16_t>(sub_top);
      objs.track_objs[objs.objs_num].obj_info.sub_box.right_bottom_u =
          static_cast<uint16_t>(sub_right);
      objs.track_objs[objs.objs_num].obj_info.sub_box.right_bottom_v =
          static_cast<uint16_t>(sub_bottom);

      // std::cout << objs.track_objs[objs.objs_num].track_id << " "
      //           << objs.track_objs[objs.objs_num].box.left_top_u << " "
      //           << objs.track_objs[objs.objs_num].box.left_top_v << " "
      //           << objs.track_objs[objs.objs_num].box.right_bottom_u << " "
      //           << objs.track_objs[objs.objs_num].box.right_bottom_v
      //           << std::endl;
      objs.objs_num++;
    }
    data.push_back(objs);
  }
  return true;
}
}
}  // namespace reader