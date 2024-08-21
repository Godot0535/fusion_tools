#pragma once
#include <Eigen/Core>
#include <memory>
#include <unordered_map>

#include "extend_kalman.h"
#include "motion_measurer.h"
#include "slope_estimation.h"

namespace mr {

// @brief 单目测距相关的参数
struct MonocularRangingConfig {
  CameraParams camera_parameters;  // 相机参数
  time_t delete_time_diff;         // 轨迹保存时长

  MonocularRangingConfig() {
    camera_parameters = CameraParams();
    delete_time_diff = 3;
  }
};

class MonocularRanging {
 public:
  MonocularRanging();
  ~MonocularRanging() = default;
  MonocularRanging(MonocularRangingConfig &config);
  bool SetParameter(MonocularRangingConfig &config);

  bool UpdateTracks(std::vector<TrackedObject> &objs, float pitch, int quality);
  bool UpdateSlope(DoubleLine &, DoubleLine &, Line &, Line &);

  const std::unordered_map<int, MotionMeasurerPtr> &GetMap() const {
    return id_map_;
  }

 private:
  bool PreprocessTracks();

 private:
  std::unordered_map<int, MotionMeasurerPtr> id_map_;
  time_t delete_time_diff_;
};

}  // namespace mr