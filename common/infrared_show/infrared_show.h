#pragma once
#include <opencv2/opencv.hpp>

#include "common/ICC_common/include/self_dev_vision_input_define.h"
#include "common/infrared_show/convas.h"

typedef struct {
  int frame_id;
  std::string scene_name;
} ImageInfo;

class InfraredShow {
 public:
  void Reset(cv::Mat& image);
  const cv::Mat& Display();
  void Undistort();

  void AddInfo(ImageInfo& info);
  void DrawObjDetect(const DETECT_OBJS& detect_objs, bool frontail = true);
  void DrawTrackObjs(const TRACK_OBJS& track_objs, bool frontail = false);

 private:
  cv::Mat image_;
  cv::Mat display_;
  Canvas canvas_{cv::Size(240, 512), cv::Rect(-15, 50, 30, 55)};
};