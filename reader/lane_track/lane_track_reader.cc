#include "reader/lane_track/lane_track_reader.h"

#include <json/json.h>

#include <fstream>
#include <sstream>

#include "3rdparty/magic_enum/magic_enum.hpp"
#include "3rdparty/magic_enum/magic_enum_iostream.hpp"
#include "common/utils.h"

using magic_enum::iostream_operators::operator>>;

namespace reader {
bool LaneTrackReader::ReadData(std::string path) {
  data.clear();
  std::vector<std::string> files;
  GetFileNames(path, files);
  for (auto& file : files) {
    std::ifstream data_in(file, std::ios::binary);
    if (!data_in.is_open()) {
      return false;
    }
    TRACK_LANES lanes;
    std::string data_line;
    if (getline(data_in, data_line)) {
      std::istringstream data_stream(data_line);
      std::string unuse;
      data_stream >> unuse >> lanes.pitch >> lanes.width >> lanes.offset >>
          lanes.pitch_pl >> lanes.lanes_num;
    }
    for (size_t i = 0; i < lanes.lanes_num; i++) {
      std::string unuse;
      getline(data_in, data_line);
      std::istringstream data_pxl_stream(data_line);
      data_pxl_stream >> unuse >> lanes.lanes_pxl[i].lane_id >>
          lanes.lanes_pxl[i].param_num >> lanes.lanes_pxl[i].start_y0 >>
          lanes.lanes_pxl[i].end_y0 >> lanes.lanes_pxl[i].c00 >>
          lanes.lanes_pxl[i].c01 >> lanes.lanes_pxl[i].c02 >>
          lanes.lanes_pxl[i].c03 >> lanes.lanes_pxl[i].start_y1 >>
          lanes.lanes_pxl[i].end_y1 >> lanes.lanes_pxl[i].c10 >>
          lanes.lanes_pxl[i].c11 >> lanes.lanes_pxl[i].c12 >>
          lanes.lanes_pxl[i].c13 >> lanes.lanes_pxl[i].category >>
          lanes.lanes_pxl[i].color >> lanes.lanes_pxl[i].quality >>
          lanes.lanes_pxl[i].side >> lanes.lanes_pxl[i].confidence >>
          lanes.lanes_pxl[i].measuring_state >> lanes.lanes_pxl[i].crossing;
      getline(data_in, data_line);
      std::istringstream data_bev_stream(data_line);
      data_bev_stream >> unuse >> lanes.lanes_wrd[i].lane_id >>
          lanes.lanes_wrd[i].param_num >> lanes.lanes_wrd[i].start_y0 >>
          lanes.lanes_wrd[i].end_y0 >> lanes.lanes_wrd[i].c00 >>
          lanes.lanes_wrd[i].c01 >> lanes.lanes_wrd[i].c02 >>
          lanes.lanes_wrd[i].c03 >> lanes.lanes_wrd[i].start_y1 >>
          lanes.lanes_wrd[i].end_y1 >> lanes.lanes_wrd[i].c10 >>
          lanes.lanes_wrd[i].c11 >> lanes.lanes_wrd[i].c12 >>
          lanes.lanes_wrd[i].c13 >> lanes.lanes_wrd[i].category >>
          lanes.lanes_wrd[i].color >> lanes.lanes_wrd[i].quality >>
          lanes.lanes_wrd[i].side >> lanes.lanes_wrd[i].confidence >>
          lanes.lanes_wrd[i].measuring_state >> lanes.lanes_wrd[i].crossing;
    }
    data.push_back(lanes);
  }
  // for (auto& data_frame : data) {
  //   std::cout << data_frame.pitch << std::endl;
  // }
  return true;
}

const TRACK_LANES& LaneTrackReader::GetData(int frame_id) const {
  return data.at(frame_id);
}

}  // namespace reader
