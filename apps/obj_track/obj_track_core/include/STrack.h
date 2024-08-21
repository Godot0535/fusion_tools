#pragma once

#include "id_allocator.h"
#include "kalmanFilter.h"
#include "opencv2/opencv.hpp"

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack {
 public:
  STrack(std::vector<float> tlwh_, float score, int label);
  ~STrack();

  std::vector<float> static tlbr_to_tlwh(std::vector<float> &tlbr);
  void static MultiPredict(std::vector<STrack *> &stracks,
                           byte_kalman::KalmanFilter &kalman_filter);
  void static_tlwh();
  void static_tlbr();
  std::vector<float> tlwh_to_xyah(std::vector<float> tlwh_tmp);
  std::vector<float> to_xyah();
  void MarkLost();
  void MarkRemoved();
  int NextId();
  int EndFrame();
  void UpdateWithDet(float ps, float loss);
  void UpdateWithoutDet(float ps);
  void Activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id);
  void ReActivate(STrack &new_track, int frame_id, bool new_id = false);
  void Update(STrack &new_track, int frame_id);

 public:
  bool is_activated_;
  int track_id_;
  int state_;
  int frame_lost_;
  int det_index_;

  std::vector<float> _tlwh;
  std::vector<float> tlwh;
  std::vector<float> tlbr;
  int frame_id_;
  int tracklet_len_;
  int start_frame_;
  int label_;
  float conf_;  // 追踪置信度

  KAL_MEAN mean_;
  KAL_COVA covariance_;
  float score_;  // 检测得分

 private:
  byte_kalman::KalmanFilter kalman_filter_;
  static IdAllocator allocator_;
};