#pragma once
#include <opencv2/opencv.hpp>

#include "apps/vision/include/config.h"
#include "common/icc_common/include/percept_obstacle_output_define.h"
#include "common/show/show.h"
#include "reader/lane_detect/lane_detect_reader.h"
#include "reader/lane_track/lane_track_reader.h"
#include "reader/mobileye/mobileye_reader.h"
#include "reader/obj_detect/obj_detect.h"
#include "reader/obj_ranging/obj_ranging_reader.h"
#include "reader/obj_track/obj_track.h"

namespace vision {
class Vision {
 public:
  Vision();
  ~Vision() {
    if (config_.video_save) video_writer.release();
  }
  bool Run();
  bool SaveVideo();

 private:
  void ProcessImage();

 private:
  ImageInfo image_info_;
  VisionConfig config_;
  cv::Mat image_;
  Show show_;
  std::vector<std::string> image_files_;
  cv::VideoWriter video_writer;

  reader::ObjDetectReader obj_detect_reader_;
  reader::ObjTrackReader obj_track_reader_;
  reader::LaneDetectReader lane_detect_reader_;
  reader::LaneTrackReader lane_track_reader_;
  reader::ObjRangingReader obj_ranging_reader_;
  reader::MobileyeReader mobileye_reader_;
};
}  // namespace vision