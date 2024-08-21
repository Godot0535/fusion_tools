#include "../include/monocular_ranging.h"

#include <iostream>

namespace mr {
CameraParams camera_params = CameraParams();
MonocularRanging::MonocularRanging() {}

MonocularRanging::MonocularRanging(MonocularRangingConfig &config) {
  camera_params = config.camera_parameters;
  delete_time_diff_ = config.delete_time_diff;
}

bool MonocularRanging::SetParameter(MonocularRangingConfig &config) {
  camera_params = config.camera_parameters;
  delete_time_diff_ = config.delete_time_diff;
  return true;
}

bool MonocularRanging::UpdateTracks(std::vector<TrackedObject> &objs,
                                    float pitch, int quality) {
  PreprocessTracks();
  LOG_INFO << "pitch: " << pitch;
  LOG_INFO << "pitch quality: " << quality;
  for (TrackedObject &obj : objs) {
    if (id_map_.find(obj.track_id) == id_map_.end()) {
      // 初次出现的ID
      MotionMeasurerPtr new_track = std::make_shared<MotionMeasurer>();
      if (new_track->Initialize(obj, pitch, quality))
        id_map_.emplace(obj.track_id, new_track);
    } else {
      // 重复出现的ID
      MotionMeasurerPtr track = id_map_[obj.track_id];
      track->Update(obj, pitch, quality);
    }
  }
  return true;
}

bool MonocularRanging::UpdateSlope(DoubleLine &left_d, DoubleLine &right_d,
                                   Line &left_t, Line &right_t) {
  slope.ResetState();
  slope.UpdateDetectLane(left_d, right_d);
  slope.UpdateTrackLane(left_t, right_t);
  return true;
}

// 每次更新前对所有的轨迹进行预处理
bool MonocularRanging::PreprocessTracks() {
  time_t time_now = time(nullptr);
  for (auto iter = id_map_.begin(); iter != id_map_.end();) {
    if (time_now - iter->second->GetTime() > delete_time_diff_) {
      // 删除已经超出失踪时间限制的轨迹,测试环境下不执行
#ifdef __aarch64__
      LOG_INFO << "Delete track: " << iter->first;
      iter->second = nullptr;
      iter = id_map_.erase(iter);
#else
      iter++;
#endif
    } else {
      // 未超出时间限制进行轨迹预测
      iter->second->Predict();
      iter++;
    }
  }
  return true;
}
}  // namespace mr