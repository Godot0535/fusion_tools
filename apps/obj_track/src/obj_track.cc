#include "apps/obj_track/include/obj_track.h"

#include <fstream>
#include <unordered_map>

#include "common/utils.h"

namespace obj_track {
ObjTrack::ObjTrack() {
  config_.ReadConfig("../config/config.toml");
  obj_detect_reader_.ReadData(config_.obj_detect_path);
}

bool ObjTrack::Run() {
  // std::cout << "----------------------" << std::endl;

  const DETECT_OBJS &objs = obj_detect_reader_.GetData(frame_id_);
  std::unordered_map<size_t, DETECT_OBJ_INFO> map;

  std::vector<Obj> detect_objs;

  // std::cout << "finish define" << std::endl;
  // std::cout << "objs_num: " << objs.objs_num << std::endl;
  for (size_t i = 0; i < objs.objs_num; i++) {
    // std::cout << i << std::endl;
    map[i] = objs.detect_objs[i];
    Obj detect_obj;
    detect_obj.index = i;
    detect_obj.label = static_cast<int>(objs.detect_objs[i].obj_category);
    detect_obj.prob = static_cast<float>(objs.detect_objs[i].confidence);
    detect_obj.rect = cv::Rect_<float>(objs.detect_objs[i].box.left_top_u,
                                       objs.detect_objs[i].box.left_top_v,
                                       objs.detect_objs[i].box.right_bottom_u -
                                           objs.detect_objs[i].box.left_top_u,
                                       objs.detect_objs[i].box.right_bottom_v -
                                           objs.detect_objs[i].box.left_top_v);
    detect_objs.push_back(detect_obj);
    // std::cout << detect_obj.index << " " << detect_obj.label << " "
    //           << detect_obj.prob << " " << detect_obj.rect << std::endl;
    // std::cout << objs.detect_objs[i].sub_box.left_top_u << " "
    //           << objs.detect_objs[i].sub_box.left_top_v << " "
    //           << objs.detect_objs[i].sub_box.right_bottom_u << " "
    //           << objs.detect_objs[i].sub_box.right_bottom_v << std::endl;
  }
  std::cout << "map size: " << map.size() << std::endl;

  // std::cout << "update finish" << std::endl;
  std::vector<STrack> &&tracks = tracker_.Update(detect_objs);
  // std::cout << tracks.size() << std::endl;

  std::string file_path =
      config_.obj_track_path + "/" + IndexString(frame_id_) + ".txt";
  std::ofstream file(file_path, std::ios::out | std::ios::trunc);
  for (auto &track : tracks) {
    auto det = map.find(track.det_index_);
    std::cout << "det id: " << track.det_index_ << " track id: " << track.track_id_ << std::endl;
    file << track.track_id_ << " " << track.tlwh[0] << " " << track.tlwh[1]
         << " " << (track.tlwh[0] + track.tlwh[2]) << " "
         << (track.tlwh[1] + track.tlwh[3]) << " " << track.conf_ << " "
         << (int)track.label_ << " ";
    if (det != map.end()) {
      file << det->second.box.left_top_u << " " << det->second.box.left_top_v
           << " " << det->second.box.right_bottom_u << " "
           << det->second.box.right_bottom_v << " " << det->second.confidence
           << " " << det->second.sub_box.left_top_u << " "
           << det->second.sub_box.left_top_v << " "
           << det->second.sub_box.right_bottom_u << " "
           << det->second.sub_box.right_bottom_v << " "
           << det->second.sub_box_confidence << std::endl;
    } else {
      file << track._tlwh[0] << " " << track._tlwh[1] << " "
           << track._tlwh[0] + track._tlwh[2] << " "
           << track._tlwh[1] + track._tlwh[3] << " " << track.score_ << " " << 0
           << " " << 0 << " " << 0 << " " << 0 << " " << 0.0 << std::endl;
    }

    // std::cout << det->second.sub_box.left_top_u << " "
    //           << det->second.sub_box.left_top_v << " "
    //           << det->second.sub_box.right_bottom_u << " "
    //           << det->second.sub_box.right_bottom_v << std::endl;
  }
  file.close();
  // std::cout << frame_id_ << std::endl;
  if (obj_detect_reader_.IsValid(++frame_id_)) return true;

  return false;
}

}  // namespace obj_track