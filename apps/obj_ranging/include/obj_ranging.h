#pragma once

#include "apps/obj_ranging/include/config.h"
#include "apps/obj_ranging/obj_ranging_core/include/monocular_ranging.h"
#include "reader/lane_detect/lane_detect_reader.h"
#include "reader/lane_track/lane_track_reader.h"
#include "reader/obj_track/obj_track.h"
#include "reader/mobileye/mobileye_reader.h"
#include "common/show/show.h"

namespace obj_ranging {
class ObjRanging {
 public:
  ObjRanging();
  bool Run();

 private:
  int frame_id_{0};
  mr::MonocularRanging ranging_;
  ObjRangingConfig config_;
  reader::ObjTrackReader obj_track_reader_;
  reader::LaneDetectReader lane_detect_reader_;
  reader::LaneTrackReader lane_track_reader_;
  reader::MobileyeReader mobileye_reader_;
  Show show_;
  ImageInfo image_info_;
  std::vector<std::string> image_files_;
};
}  // namespace obj_ranging