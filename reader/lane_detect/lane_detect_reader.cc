#include "reader/lane_detect/lane_detect_reader.h"

#include <json/json.h>

#include <fstream>
#include <sstream>

#include "common/utils.h"

namespace reader {
bool LaneDetectReader::ReadData(std::string path) {
  data.clear();
  std::vector<std::string> files;
  GetFileNames(path, files);
  for (auto& file : files) {
    std::ifstream data_in(file, std::ios::binary);
    if (!data_in.is_open()) {
      return false;
    }

    DETECT_LANES lanes;
    lanes.lanes_num = 0;

    Json::Reader reader;
    Json::Value root;
    if (reader.parse(data_in, root)) {
      for (int i = 0; i < root["lane_lines"].size(); i++) {
        lanes.lanes_pxl[lanes.lanes_num].lane_id =
            root["lane_lines"][i]["lane_id"].asInt();
        lanes.lanes_pxl[lanes.lanes_num].param_num =
            root["lane_lines"][i]["param_num"].asInt();
        lanes.lanes_pxl[lanes.lanes_num].c03 =
            root["lane_lines"][i]["params_1"][0].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].c02 =
            root["lane_lines"][i]["params_1"][1].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].c01 =
            root["lane_lines"][i]["params_1"][2].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].c00 =
            root["lane_lines"][i]["params_1"][3].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].c13 =
            root["lane_lines"][i]["params_2"][0].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].c12 =
            root["lane_lines"][i]["params_2"][1].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].c11 =
            root["lane_lines"][i]["params_2"][2].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].c10 =
            root["lane_lines"][i]["params_2"][3].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].start_y0 =
            root["lane_lines"][i]["y_value_1"][0].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].end_y0 =
            root["lane_lines"][i]["y_value_1"][1].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].start_y1 =
            root["lane_lines"][i]["y_value_2"][0].asDouble();
        lanes.lanes_pxl[lanes.lanes_num].end_y1 =
            root["lane_lines"][i]["y_value_2"][1].asDouble();

        // std::cout << "---------------------------" << std::endl;
        // std::cout << lanes.lanes_pxl[lanes.lanes_num].lane_id << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].param_num << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c00 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c01 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c02 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c03 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c10 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c11 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c12 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].c13 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].start_y0 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].end_y0 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].start_y1 << " "
        //           << lanes.lanes_pxl[lanes.lanes_num].end_y1 << std::endl;

        lanes.lanes_num++;
      }
    }
    data.push_back(lanes);
  }
  return true;
}

const DETECT_LANES& LaneDetectReader::GetData(int frame_id) const {
  return data.at(frame_id);
}

}  // namespace reader