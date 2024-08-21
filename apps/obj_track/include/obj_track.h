#pragma once

#include "apps/obj_track/obj_track_core/include/BYTETracker.h"
#include "config.h"
#include "reader/obj_detect/obj_detect.h"

namespace obj_track {
class ObjTrack {
 public:
  ObjTrack();
  bool Run();
 private:
  int frame_id_{0};
  BYTETracker tracker_{20, 30};
  ObjTrackConfig config_;
  reader::ObjDetectReader obj_detect_reader_;
};
}  // namespace obj_track