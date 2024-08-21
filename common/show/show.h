#pragma once
#include <opencv2/opencv.hpp>

#include "common/ICC_common/include/percept_lane_output_define.h"
#include "common/ICC_common/include/percept_obstacle_output_define.h"
#include "common/mobileye_types.h"
#include "common/show/convas.h"

typedef struct {
  int frame_id;
  std::string scene_name;
} ImageInfo;

class Show {
 public:
  void Reset(cv::Mat& image);
  const cv::Mat& Display();
  void Undistort();

  void AddInfo(ImageInfo& info);
  void DrawObjDetect(const DETECT_OBJS& detect_objs, bool frontail = true);
  void DrawLaneDetect(const DETECT_LANES& detect_lanes);
  void DrawLaneTrack(const TRACK_LANES& track_lanes);
  void DrawTrackObjs(const TRACK_OBJS& track_objs, bool frontail = false);
  void DrawMobileyeData(const MOBILEYE_OBSTACLES&, const MOBILEYE_LANE_SHAPES&);

 private:
  cv::Mat image_;
  cv::Mat display_;
  Canvas canvas_{cv::Size(480, 1080), cv::Rect(-30, 155, 60, 160)};
};