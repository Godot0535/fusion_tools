#pragma once

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "../include/extend_kalman.h"
#include "common/logger/log_facade.h"

namespace mr {
struct CameraParams {
  Eigen::Matrix3d intrinsic_distort;
  Eigen::Matrix3d intrinsic;
  Eigen::Matrix4d extrinsic;
  Eigen::VectorXd distortion;
  Eigen::Matrix2d rotation;
  cv::Mat intrinsic_distort_cv;
  cv::Mat intrinsic_cv;
  cv::Mat distortion_cv;
  float camera_height;
  float camera_yaw;
  float camera_roll;
  float image_width;
  float image_height;
  float frame_interval;

  CameraParams() {
    intrinsic_distort = Eigen::Matrix3d::Identity();
    intrinsic_distort.block<2, 3>(0, 0) << 1143.398167, 0.0, 956.0743379, 0.0,
        1169.03, 532.9676489;
    extrinsic = Eigen::Matrix4d::Identity();
    distortion.resize(5);
    distortion << -0.356918618, 0.167589043, -0.000186537, -0.00026963,
        -0.045005666;
    rotation = Eigen::Matrix2d::Identity();
    camera_height = 2.25;
    camera_yaw = 0;
    // camera_roll = 0.0;
    camera_roll = -0.02116767;
    image_width = 1920.0;
    image_height = 1080.0;
    frame_interval = 0.05;
    Init();
  }

  void Init() {
    cv::eigen2cv(intrinsic_distort, intrinsic_distort_cv);
    cv::eigen2cv(distortion, distortion_cv);
    intrinsic_cv =
        cv::getOptimalNewCameraMatrix(intrinsic_distort_cv, distortion_cv,
                                      cv::Size(image_width, image_height), 0.0,
                                      cv::Size(image_width, image_height));
    cv::cv2eigen(intrinsic_cv, intrinsic);
    rotation << std::cos(camera_roll), std::sin(camera_roll),
        -std::sin(camera_roll), std::cos(camera_roll);
  }
};

extern CameraParams camera_params;

struct TrackedObject {
  int track_id;
  int type;
  uint16_t left;
  uint16_t top;
  uint16_t right;
  uint16_t bottom;
  bool sub_flag;
  uint16_t sub_left;
  uint16_t sub_top;
  uint16_t sub_right;
  uint16_t sub_bottom;
};

class MotionMeasurer {
 public:
  MotionMeasurer() = default;
  ~MotionMeasurer() = default;

  bool Initialize(TrackedObject &obj, float pitch, int quality);
  void Predict() { filter_.Predict(); }
  bool Update(TrackedObject &obj, float pitch, int quality);
  time_t GetTime() { return last_tracked_time_; }
  bool IsActivate() { return activate_; }

  const float &GetVerticalDistance() const { return vertical_distance_; }
  const float &GetHorizontalDistance() const { return horizontal_distance_; }
  const float &GetVerticalVelocity() const { return vertical_velocity_; }
  const float &GetHorizontalVelocity() const { return horizontal_velocity_; }
  const float &GetWidth() const { return width_; }
  const float &GetHeight() const { return height_; }
  const float &GetVerticalDistanceProb() const {
    return vertical_distance_prob_;
  }
  const float &GetVerticalVelocityProb() const {
    return vertical_velocity_prob_;
  }
  const float &GetHorizontalDistanceProb() const {
    return horizontal_distance_prob_;
  }
  const float &GetHorizontalVelocityProb() const {
    return horizontal_velocity_prob_;
  }

 private:
  bool InitFilter();
  bool UpdateObservation(TrackedObject &obj, float pitch, int quality);
  bool GetFilterResults();
  bool ConvertCoordinateSystem();

  void LeftRight(TrackedObject &obj);
  bool DistortionCorrect(TrackedObject &obj);
  void VerticalDist(TrackedObject &obj, float pitch, int quality);
  void HorizontalDist(TrackedObject &obj);
  void WidthObserved();
  void UpdateHeight();
  // void AngleEstimated();

  bool PriorWidth(TrackedObject &obj);
  bool LaneWidth(double bottom, int quality, float vertical_dist);
  bool PitchBottom(double bottom, float pitch, int quality);

 private:
  // 状态
  time_t last_tracked_time_;
  bool activate_{false};
  bool left_flag_{false};
  float width_prior_{0.0};
  float width_prior_ratio_{0.0};
  int width_update_{0};

  ExtendKalmanFilter filter_{5};

  // 畸变矫正和横滚角矫正后的点
  Eigen::Vector2d lb, rb, lt, rt, sub_lt, sub_rt;

  // 纵向不同方法的观测结果和方差
  float vertical_dist_pitch_{0.0};  // 通过pitch计算出的纵向距离
  float variance_dist_pitch_{0.0};  // 通过pitch计算出的纵向距离方差
  float vertical_dist_prior_{0.0};  // 通过先验宽度计算出的纵向距离
  float variance_dist_prior_{0.0};  // 通过先验宽度计算出的纵向距离方差
  float vertical_dist_lane_{0.0};  // 通过车道线宽度计算出的纵向距离
  float variance_dist_lane_{0.0};  // 通过车道线宽度计算出的纵向距离方差

  // 横向不同方法的观测结果和方差
  float horizontal_dist_2D_{0.0};   // 通过2D计算出的横向距离
  float variance_dist_2D_{0.0};     // 通过2D计算出的横向距离方差
  float horizontal_dist_25D_{0.0};  // 通过2.5D计算出的横向距离
  float variance_dist_25D_{0.0};    // 通过2.5D计算出的横向距离方差

  // 观测量
  float vertical_dist_{0.0};
  float horizontal_dist_{0.0};
  float width_observed_{0.0};

  // 观测误差
  float variance_vertical_{0.0};
  float variance_horizontal_{0.0};
  float variance_width_{0.0};

  // 当前帧结果
  // 滤波结束后更新一次,坐标系转换后更新一次
  float vertical_distance_{0.0};
  float vertical_velocity_{0.0};
  float horizontal_distance_{0.0};
  float horizontal_velocity_{0.0};
  float vertical_distance_prob_{0.0};
  float vertical_velocity_prob_{0.0};
  float horizontal_distance_prob_{0.0};
  float horizontal_velocity_prob_{0.0};
  float width_{0.0};
  float height_{0.0};
  // float angle_{0.0};
};
typedef std::shared_ptr<MotionMeasurer> MotionMeasurerPtr;

}  // namespace mr