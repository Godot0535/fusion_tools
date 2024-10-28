#pragma once

#include "apps/obj_track/obj_track_core/include/BYTETracker.h"
#include "config.h"
#include "reader/infrared_detect/infrared_detect_reader.h"

namespace infrared_track {
class InfraredTrack {
 public:
  InfraredTrack();
  bool Run();

 private:
  int frame_id_{0};
  BYTETracker tracker_{30, 30};
  InfraredTrackConfig config_;
  reader::InfraredDetectReader infrared_detect_reader_;
};
}  // namespace infrared_track