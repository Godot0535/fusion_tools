#include "reader/mobileye/mobileye_reader.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "3rdparty/magic_enum/magic_enum.hpp"
#include "3rdparty/magic_enum/magic_enum_iostream.hpp"
#include "common/utils.h"

using magic_enum::iostream_operators::operator>>;

namespace reader {

bool MobileyeReader::ReadData(std::string path) {
  objs_data.clear();
  lanes_data.clear();
  std::vector<std::string> files;
  std::vector<std::string> folders;
  GetAllFolders(path, folders);
  for (auto& folder : folders) {
    files.push_back(GetMobileye(folder));
  }

  for (auto& file : files) {
    std::ifstream data_in(file, std::ios::binary);
    if (!data_in.is_open()) {
      return false;
    }

    MOBILEYE_OBSTACLES objs;
    MOBILEYE_LANE_SHAPES lanes;
    objs.clear();
    lanes.clear();

    std::string data_line;
    while (getline(data_in, data_line)) {
      std::istringstream data_stream(data_line);
      std::string flag;
      data_stream >> flag;
      if (flag == "#") {
        MOBILEYE_OBSTACLE obj;
        data_stream >> obj.obj_id;
        if (obj.obj_id == 0) continue;
        int type;
        data_stream >> type >> obj.f_lateral_distance >> obj.f_long_distance >>
            obj.f_obj_length >> obj.f_obj_width >> obj.f_obj_height >>
            obj.f_relative_long_velocity >> obj.f_relative_lat_velocity;
        obj.e_obs_type = static_cast<OBSTACLE_TYPE>(type);
        objs.push_back(obj);
      } else if (flag == "$") {
        MOBILEYE_LANE_SHAPE mobileye_lane;
        data_stream >> mobileye_lane.iAdsTimeStamp >> mobileye_lane.lane_type >>
            mobileye_lane.f_confidence >> mobileye_lane.f_start >>
            mobileye_lane.f_end >> mobileye_lane.role >> mobileye_lane.f_c0 >>
            mobileye_lane.f_c1 >> mobileye_lane.f_c2 >> mobileye_lane.f_c3;
        lanes.push_back(mobileye_lane);
      }
    }
    data_in.close();
    objs_data.push_back(objs);
    lanes_data.push_back(lanes);
  }
  return true;
}

const MOBILEYE_OBSTACLES& MobileyeReader::GetObjectsData(int frame_id) const {
  return objs_data.at(frame_id);
}

const MOBILEYE_LANE_SHAPES& MobileyeReader::GetLanesData(int frame_id) const {
  return lanes_data.at(frame_id);
}
}  // namespace reader
