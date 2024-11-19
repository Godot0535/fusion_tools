#pragma once

#include "apps/infrared_ranging/include/config.h"
#include "apps/obj_ranging/obj_ranging_core/include/monocular_ranging.h"
#include "reader/obj_track/obj_track.h"
#include "common/infrared_show/infrared_show.h"

namespace infrared_ranging {
class InfraredRanging {
 public:
  InfraredRanging();
  bool Run();

 private:
  int frame_id_{0};
  mr::MonocularRanging ranging_;
  InfraredRangingConfig config_;
  reader::ObjTrackReader obj_track_reader_;
  InfraredShow infrared_show_;
  ImageInfo image_info_;
  std::vector<std::string> image_files_;
  cv::VideoWriter video_writer;
};
}  // namespace infrared_ranging