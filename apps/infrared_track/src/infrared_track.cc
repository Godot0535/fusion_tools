#include "apps/infrared_track/include/infrared_track.h"

#include <fstream>

#include "common/utils.h"

namespace infrared_track {
InfraredTrack::InfraredTrack() {
  config_.ReadConfig("../config/config.toml");
  infrared_detect_reader_.ReadData(config_.infrared_detect_path);
}

bool InfraredTrack::Run() {
  const DETECT_OBJS &objs = infrared_detect_reader_.GetData(frame_id_);
  std::vector<Obj> detect_objs;

  for (size_t i = 0; i < objs.objs_num; i++) {
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
  }

  std::vector<STrack> &&tracks = tracker_.Update(detect_objs);
  std::string file_path =
      config_.infrared_track_path + "/" + IndexString(frame_id_) + ".txt";
  std::ofstream file(file_path, std::ios::out | std::ios::trunc);
  for (auto &track : tracks) {
    std::cout << "det id: " << track.det_index_
              << " track id: " << track.track_id_ << std::endl;
    file << track.track_id_ << " " << track.tlwh[0] << " " << track.tlwh[1]
         << " " << (track.tlwh[0] + track.tlwh[2]) << " "
         << (track.tlwh[1] + track.tlwh[3]) << " " << track.conf_ << " "
         << (int)track.label_ << " " << track._tlwh[0] << " " << track._tlwh[1]
         << " " << track._tlwh[0] + track._tlwh[2] << " "
         << track._tlwh[1] + track._tlwh[3] << " " << track.score_ << " " << 0
         << " " << 0 << " " << 0 << " " << 0 << " " << 0.0 << std::endl;
  }
  file.close();
  if (infrared_detect_reader_.IsValid(++frame_id_)) return true;
  return false;
}
}  // namespace infrared_track