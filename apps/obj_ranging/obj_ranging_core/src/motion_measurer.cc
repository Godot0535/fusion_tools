#include "../include/motion_measurer.h"

#include <algorithm>

#include "../include/kalman.h"
#include "../include/slope_estimation.h"

namespace mr {
/**
 * @brief 根据类型分配先验宽度
 */
float GetPriorWidth(int type) {
  // CAR = 0, TRUCK = 1, MOTORCYCLE = 2, BICYCLE = 3,TRICYCLE = 4,
  // BUS = 5, PEDESTRAIN = 6
  // 以下数据均为搜索得到的范围中值
  switch (type) {
    case 0:  // 1.4-1.8
      return 1.8;
    case 1:  // 1.9-2.5 卡车的宽度随载重变化而变化,范围较大
      return 2.0;
    case 5:  // 2.4-2.6 公交车宽度范围比较集中
      return 2.4;
    case 2:
    case 3:
      return 0.5;
    case 4:
      return 1.2;
    case 6:
      return 0.4;
    default:
      return 1.0;
  }
  return 1.0;
}

/**
 * @brief 2D滤波器的雅各比矩阵
 */
Eigen::MatrixXd HJacobi2D(const Eigen::VectorXd &x) {
  Eigen::MatrixXd jacobi(2, 5);
  jacobi << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  return jacobi;
}

/**
 * @brief 2D滤波器的观测矩阵
 */
Eigen::VectorXd HX2D(const Eigen::VectorXd &x) {
  Eigen::VectorXd hx(2);
  hx << x(0), x(1);
  return hx;
}

/**
 * @brief 25D滤波器的雅各比矩阵
 */
Eigen::MatrixXd HJacobi25D(const Eigen::VectorXd &x) {
  Eigen::MatrixXd jacobi(3, 5);
  jacobi << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0;
  return jacobi;
}

/**
 * @brief 25D滤波器的观测矩阵
 */
Eigen::VectorXd HX25D(const Eigen::VectorXd &x) {
  Eigen::VectorXd hx(3);
  hx << x(0), x(1), x(2);
  return hx;
}

/**
 * @brief 初始化目标
 */
bool MotionMeasurer::Initialize(TrackedObject &obj, float pitch, int quality) {
  LOG_INFO << "Initialize: " << obj.track_id;
  last_tracked_time_ = time(nullptr);
  width_prior_ = GetPriorWidth(obj.type);
  width_observed_ = width_prior_;
  filter_.x_(2) = width_prior_;
  LOG_INFO << "width_prior_: " << width_prior_;
  if (!UpdateObservation(obj, pitch, quality)) {
    activate_ = false;
    return false;
  }
  InitFilter();
  // GetFilterResults();
  // UpdateHeight();
  // ConvertCoordinateSystem();
  return true;
}

/**
 * @brief 更新目标
 */
bool MotionMeasurer::Update(TrackedObject &obj, float pitch, int quality) {
  LOG_INFO << "Update: " << obj.track_id;
  last_tracked_time_ = time(nullptr);
  if (!UpdateObservation(obj, pitch, quality)) {
    activate_ = false;
    return false;
  }
  activate_ = true;

  if (obj.sub_flag) {
    Eigen::VectorXd z(3);
    z << vertical_dist_, horizontal_dist_, width_observed_;
    filter_.R_.resize(3, 3);
    filter_.R_.setIdentity();
    filter_.R_.diagonal() << variance_vertical_, variance_horizontal_,
        variance_width_;
    filter_.PredictUpdate(z, HJacobi25D, HX25D);
  } else {
    Eigen::VectorXd z(2);
    z << vertical_dist_, horizontal_dist_;
    filter_.R_.resize(2, 2);
    filter_.R_.setIdentity();
    filter_.R_.diagonal() << variance_vertical_, variance_horizontal_;
    filter_.PredictUpdate(z, HJacobi2D, HX2D);
  }

  GetFilterResults();
  UpdateHeight();
  ConvertCoordinateSystem();
  LOG_INFO << "result: " << vertical_distance_ << " " << horizontal_distance_
           << " " << vertical_velocity_ << " " << horizontal_velocity_ << " "
           << width_ << " " << height_;
  return true;
}

bool MotionMeasurer::UpdateObservation(TrackedObject &obj, float pitch,
                                       int quality) {
  LeftRight(obj);
  if (!DistortionCorrect(obj)) return false;
  VerticalDist(obj, pitch, quality);
  HorizontalDist(obj);
  if (obj.sub_flag) WidthObserved();
  return true;
}

/**
 * @brief 判断目标相对于自车是左斜/右斜
 *        主要用于判断2D情况下使用左/右边界计算横向距离
 *        目前直接通过相对光心位置判断,避免出现抖动
 */
void MotionMeasurer::LeftRight(TrackedObject &obj) {
  // if (obj.sub_flag) {
  //   float horizontal_center_pixel =
  //       static_cast<float>((obj.right + obj.left)) / 2.0;
  //   float sub_center_pixel =
  //       static_cast<float>((obj.sub_left + obj.sub_right)) / 2.0;
  //   left_flag_ = horizontal_center_pixel < sub_center_pixel;
  // } else {
  float horizontal_center_pixel =
      static_cast<float>((obj.right + obj.left)) / 2.0;
  float lateral_deviation =
      horizontal_center_pixel - camera_params.intrinsic_distort(0, 2);
  left_flag_ = lateral_deviation < 0;
  // 更新先验宽度在计算中的占比
  // 仅car/bus/truck使用先验宽度计算横向
  if (obj.type != 0 && obj.type != 1 && obj.type != 5)
    width_prior_ratio_ = 0.0;
  else
    width_prior_ratio_ =
        left_flag_
            ? (-lateral_deviation / camera_params.intrinsic_distort(0, 2))
            : (lateral_deviation / (camera_params.image_width -
                                    camera_params.intrinsic_distort(0, 2)));
  LOG_INFO << "left_flag_: " << left_flag_
           << " width_prior_ratio_: " << width_prior_ratio_;
  // }
}

/**
 * @brief 畸变矫正
 */
bool MotionMeasurer::DistortionCorrect(TrackedObject &obj) {
  std::vector<cv::Point2d> original, undistort;
  original.push_back(cv::Point2d(obj.left, obj.bottom));
  original.push_back(cv::Point2d(obj.right, obj.bottom));
  original.push_back(cv::Point2d(obj.left, obj.top));
  original.push_back(cv::Point2d(obj.right, obj.top));
  if (obj.sub_flag) {
    original.push_back(cv::Point2d(obj.sub_left, obj.sub_top));
    original.push_back(cv::Point2d(obj.sub_right, obj.sub_top));
  }
  cv::undistortPoints(original, undistort, camera_params.intrinsic_distort_cv,
                      camera_params.distortion_cv, cv::noArray(),
                      camera_params.intrinsic_cv);
  for (auto &point : undistort) {
    if (point.x < 0.0 || point.x > camera_params.image_width || point.y < 0.0 ||
        point.y > camera_params.image_height) {
      return false;
    }
  }

  // 以光心为原点矫正
  lb = camera_params.rotation *
       Eigen::Vector2d(undistort[0].x - camera_params.intrinsic(0, 2),
                       undistort[0].y - camera_params.intrinsic(1, 2));
  rb = camera_params.rotation *
       Eigen::Vector2d(undistort[1].x - camera_params.intrinsic(0, 2),
                       undistort[1].y - camera_params.intrinsic(1, 2));
  lt = camera_params.rotation *
       Eigen::Vector2d(undistort[2].x - camera_params.intrinsic(0, 2),
                       undistort[2].y - camera_params.intrinsic(1, 2));
  rt = camera_params.rotation *
       Eigen::Vector2d(undistort[3].x - camera_params.intrinsic(0, 2),
                       undistort[3].y - camera_params.intrinsic(1, 2));
  if (obj.sub_flag) {
    sub_lt = camera_params.rotation *
             Eigen::Vector2d(undistort[4].x - camera_params.intrinsic(0, 2),
                             undistort[4].y - camera_params.intrinsic(1, 2));
    sub_rt = camera_params.rotation *
             Eigen::Vector2d(undistort[5].x - camera_params.intrinsic(0, 2),
                             undistort[5].y - camera_params.intrinsic(1, 2));
  }
  lb[0] += camera_params.intrinsic(0, 2);
  rb[0] += camera_params.intrinsic(0, 2);
  lt[0] += camera_params.intrinsic(0, 2);
  rt[0] += camera_params.intrinsic(0, 2);
  lb[1] += camera_params.intrinsic(1, 2);
  rb[1] += camera_params.intrinsic(1, 2);
  lt[1] += camera_params.intrinsic(1, 2);
  rt[1] += camera_params.intrinsic(1, 2);
  if (obj.sub_flag) {
    sub_lt[0] += camera_params.intrinsic(0, 2);
    sub_rt[0] += camera_params.intrinsic(0, 2);
    sub_lt[1] += camera_params.intrinsic(1, 2);
    sub_rt[1] += camera_params.intrinsic(1, 2);
  }

  LOG_INFO << "undistort: (" << lt[0] << "," << lt[1] << ") (" << rt[0] << ","
           << rt[1] << ") (" << lb[0] << "," << lb[1] << ") (" << rb[0] << ","
           << rb[1] << ")";
  // LOG_INFO << "p_l: "
  //          << std::abs((lb[1] + rb[1]) / 2.0 - camera_params.intrinsic(1, 2))
  //          << " p_d: "
  //          << std::abs((sub_lt[0] + sub_rt[0]) / 2.0 -
  //                      camera_params.intrinsic(0, 2));
  return true;
}

/**
 * @brief 计算当前帧纵向距离观测值和方差
 */
void MotionMeasurer::VerticalDist(TrackedObject &obj, float pitch,
                                  int quality) {
  KalmanFilter filter;
  bool flag_usable = false;

  // Pitch-Bottom
  double bottom = (lb[1] + rb[1]) / 2.0;
  if (PitchBottom(bottom, pitch, quality)) {
    flag_usable = true;
    filter.Update(vertical_dist_pitch_, variance_dist_pitch_);
  }

  // 先验宽度
  if (PriorWidth(obj)) {
    flag_usable = true;
    filter.Update(vertical_dist_prior_, variance_dist_prior_);
  }

  // 车道线宽度
  // 需要前两种计算方式成立一种
  if (flag_usable && LaneWidth(bottom, quality, filter.GetState()))
    filter.Update(vertical_dist_lane_, variance_dist_lane_);
  if (flag_usable) {
    vertical_dist_ = filter.GetState();
    variance_vertical_ = filter.GetCovariance();
  } else {
    // 所有测距方式都失效
    LOG_WARN << "Unable to measure through normal means";
    vertical_dist_ =
        camera_params.intrinsic(1, 1) / (rt[0] - lt[0]) * width_prior_;
    variance_vertical_ = std::pow(vertical_dist_prior_ / 5.0, 2);
  }
  // 限制纵向距离方差
  variance_vertical_ = std::max(variance_vertical_, (float)1.0);
  LOG_INFO << "vertical_dist_: " << vertical_dist_;
  LOG_INFO << "variance_vertical_: " << variance_vertical_;
}

bool MotionMeasurer::PitchBottom(double bottom, float pitch, int quality) {
  float alpha = std::atan2(bottom - camera_params.intrinsic(1, 2),
                           camera_params.intrinsic(0, 0));
  vertical_dist_pitch_ =
      camera_params.camera_height / std::tan(pitch / 180 * M_PI + alpha);
  // variance_dist_pitch_ = std::pow(vertical_dist_pitch_ / ((quality + 1.0)), 2);
  variance_dist_pitch_ = vertical_dist_pitch_ * (2.0 - 0.4 * quality);
  variance_dist_prior_ = std::max((double)variance_dist_prior_, 1.0);
  if (vertical_dist_pitch_ > 150 || vertical_dist_pitch_ < 0.5) {
    return false;
  }
  LOG_INFO << "pitch-bottom: " << vertical_dist_pitch_ << ", "
           << variance_dist_pitch_;
  return true;
}

bool MotionMeasurer::LaneWidth(double bottom, int quality,
                               float vertical_dist) {
  // pitch质量为1时有可能出现左右车道线检测缺失的情况
  if (quality < 2 || !slope.GetState()) return false;
  vertical_dist_lane_ = slope.GetDistance(bottom, vertical_dist);
  variance_dist_lane_ = vertical_dist_pitch_ * (2.0 - 0.4 * quality);
  variance_dist_prior_ = std::max((double)variance_dist_prior_, 1.0);
  if (vertical_dist_lane_ != -NAN && vertical_dist_lane_ > 0.0) {
    LOG_INFO << "lane-width: " << vertical_dist_lane_ << ", "
             << variance_dist_lane_;
    return true;
  } else {
    return false;
  }
}

// 累计10个周期在正前方,且上周期也处于正前方的车辆,可通过修正后的先验宽度,计算距离
bool MotionMeasurer::PriorWidth(TrackedObject &obj) {
  // 横向速度过大返回false
  if (std::abs(horizontal_velocity_) > 0.5) return false;
  // 没有2.5D框返回false
  if (!obj.sub_flag) return false;
  // 宽度更新次数小于5返回false
  if (width_update_ < 6) return false;

  vertical_dist_prior_ =
      camera_params.intrinsic(1, 1) / (sub_rt[0] - sub_lt[0]) * width_;
  variance_dist_prior_ = 0.7 * vertical_dist_prior_ + 9.0;
  variance_dist_prior_ = std::max((double)variance_dist_prior_, 1.0);
  LOG_INFO << "prior-width: " << vertical_dist_prior_ << ", "
           << variance_dist_prior_;
  return true;
}

/**
 * @brief 计算当前帧纵向距离观测值和方差
 */
void MotionMeasurer::HorizontalDist(TrackedObject &obj) {
  KalmanFilter filter;

  if (left_flag_) {
    double horizontal_pixel_dist = lt[0] - camera_params.intrinsic(0, 2);
    horizontal_dist_2D_ =
        vertical_dist_ / camera_params.intrinsic(0, 0) * horizontal_pixel_dist +
        filter_.x_(2) / 2.0;
    // width_prior_ / 2.0;
    variance_dist_2D_ = std::max((double)std::abs(horizontal_dist_2D_), 1.0);
  } else {
    double horizontal_pixel_dist = rt[0] - camera_params.intrinsic(0, 2);
    horizontal_dist_2D_ =
        vertical_dist_ / camera_params.intrinsic(0, 0) * horizontal_pixel_dist -
        filter_.x_(2) / 2.0;
    // width_prior_ / 2.0;
    variance_dist_2D_ = std::max((double)std::abs(horizontal_dist_2D_), 1.0);
  }
  filter.Update(horizontal_dist_2D_, variance_dist_2D_);
  LOG_INFO << "horizontal-2D: " << horizontal_dist_2D_ << ", "
           << variance_dist_2D_;

  if (obj.sub_flag) {
    double horizontal_pixel_dist =
        (sub_lt[0] + sub_rt[0]) / 2.0 - camera_params.intrinsic(0, 2);
    horizontal_dist_25D_ =
        vertical_dist_ / camera_params.intrinsic(0, 0) * horizontal_pixel_dist;
    variance_dist_25D_ =
        std::max((double)std::abs(horizontal_dist_25D_) - 1.0, 1.0);
    filter.Update(horizontal_dist_25D_, variance_dist_25D_);
    LOG_INFO << "horizontal-2.5D: " << horizontal_dist_25D_ << ", "
             << variance_dist_25D_;
  }
  horizontal_dist_ = filter.GetState();
  variance_horizontal_ = filter.GetCovariance();
  LOG_INFO << "horizontal_dist_: " << horizontal_dist_;
  LOG_INFO << "variance_horizontal_: " << variance_horizontal_;
}

void MotionMeasurer::WidthObserved() {
  double width_pixel = sub_rt[0] - sub_lt[0];
  width_observed_ =
      vertical_dist_ / camera_params.intrinsic(0, 0) * width_pixel;
  variance_width_ = 1.0;
  width_update_++;
}

bool MotionMeasurer::InitFilter() {
  filter_.F_.resize(5, 5);
  filter_.F_.setIdentity();
  filter_.F_(0, 3) = camera_params.frame_interval;
  filter_.F_(1, 4) = camera_params.frame_interval;

  filter_.P_.setIdentity();
  filter_.P_(3, 3) = 1000;

  filter_.Q_.resize(5, 5);
  filter_.Q_.setIdentity();
  filter_.Q_.diagonal() << 1, 1, 1, 1, 1;

  filter_.x_ << vertical_dist_, horizontal_dist_, width_observed_, 0, 0;

  return true;
}

bool MotionMeasurer::GetFilterResults() {
  vertical_distance_ = filter_.x_(0);
  vertical_velocity_ = filter_.x_(3);
  horizontal_distance_ = filter_.x_(1);
  horizontal_velocity_ = filter_.x_(4);
  vertical_distance_prob_ = filter_.P_(0, 0);
  vertical_velocity_prob_ = filter_.P_(3, 3);
  horizontal_distance_prob_ = filter_.P_(1, 1);
  horizontal_velocity_prob_ = filter_.P_(4, 4);
  width_ = filter_.x_(2);
  return true;
}

/**
 * @brief 坐标系转换
 * 右下前 转 前左上
 */
bool MotionMeasurer::ConvertCoordinateSystem() {
  // 距离转换
  Eigen::Vector4d loc_camera;
  loc_camera << horizontal_distance_, 0, vertical_distance_, 1;
  auto loc_world = camera_params.extrinsic * loc_camera;
  horizontal_distance_ = loc_world(1);
  vertical_distance_ = loc_world(0);
  if (vertical_dist_ < 1.0) activate_ = false;

  // 速度转换
  Eigen::Vector4d vec_camera;
  vec_camera << horizontal_velocity_, 0, vertical_velocity_, 1;
  auto vec_world = camera_params.extrinsic * vec_camera;
  horizontal_velocity_ = vec_world(1);
  vertical_velocity_ = vec_world(0);

  return true;
}

void MotionMeasurer::UpdateHeight() {
  float height_pixel_ = left_flag_ ? (rb[1] - rt[1]) : (lb[1] - lt[1]);
  height_ = height_pixel_ * vertical_distance_ / camera_params.intrinsic(1, 1);
}

// void MotionMeasurer::AngleEstimated() {
//   double cos = std::max(std::min((double)(width_ / width_prior_), 1.0), 0.0);
//   double angle = std::acos(cos);
// }

}  // namespace mr