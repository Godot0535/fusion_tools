#include "../include/slope_estimation.h"

#include <algorithm>

#include "../include/motion_measurer.h"

namespace mr {
SlopeEstimation slope = SlopeEstimation();
bool SlopeEstimation::UpdateDetectLane(DoubleLine &left, DoubleLine &right) {
  detect_left_ = left;
  LOG_INFO << "left lane line segments: " << detect_left_.param_num;
  if (detect_left_.param_num == 1) {
    UndistortLane(detect_left_.first);
    LOG_INFO << detect_left_.first.ToString();
  } else if (detect_left_.param_num == 2) {
    UndistortLane(detect_left_.first);
    UndistortLane(detect_left_.second);
    LOG_INFO << detect_left_.first.ToString();
    LOG_INFO << detect_left_.second.ToString();
  } else {
    state_ = false;
    LOG_ERROR << "Wrong number of left lane line segments";
    return false;
  }

  detect_right_ = right;
  LOG_INFO << "right lane line segments: " << detect_right_.param_num;
  if (detect_right_.param_num == 1) {
    UndistortLane(detect_right_.first);
    LOG_INFO << detect_right_.first.ToString();
  } else if (detect_right_.param_num == 2) {
    UndistortLane(detect_right_.first);
    UndistortLane(detect_right_.second);
    LOG_INFO << detect_right_.first.ToString();
    LOG_INFO << detect_right_.second.ToString();
  } else {
    state_ = false;
    LOG_ERROR << "Wrong number of right lane line segments";
    return false;
  }
  return true;
}

// 检测主车道线不全会导致state_为false
// 跟踪主车道线不全会导致quality不大于1,所以不必对state_赋值
void SlopeEstimation::UpdateTrackLane(Line &left, Line &right) {
  track_left_ = left;
  track_right_ = right;
  LOG_INFO << "track_left_: " << left.ToString();
  LOG_INFO << "track_right_: " << right.ToString();
}

float SlopeEstimation::GetDistance(double pixel, float dist_pitch) {
  // LOG_INFO << "pixel: " << pixel << " dist_pitch: " << dist_pitch;
  double x_left, x_right;
  // detect_left_.CalX(pixel, x_left);
  // detect_right_.CalX(pixel, x_right);
  // LOG_INFO << "x_left: " << x_left << " x_right: " << x_right;
  if (detect_left_.CalX(pixel, x_left) && detect_right_.CalX(pixel, x_right)) {
    double width_pixel = x_right - x_left;
    LOG_INFO << "Width on pixel: " << width_pixel;
    if (track_left_.CalX(dist_pitch, x_left) &&
        track_right_.CalX(dist_pitch, x_right)) {
      double width_bev = std::abs(x_right - x_left);
      LOG_INFO << "Width on bev: " << width_bev;
      double dist =
          camera_params.intrinsic(1, 1) / width_pixel * width_bev;
      LOG_INFO << "recursion: " << recursion_ << " value: " << dist;
      // 递归调用至多三次
      if (abs(dist - dist_pitch) < 0.3 || recursion_ > 1) {
        recursion_ = 0;
        return dist;
      } else {
        recursion_++;
        return GetDistance(pixel, dist);
      }
    }
  }
  LOG_WARN << "Unable to calculate through lane markings";
  recursion_ = 0;
  return -1.0;
}

Eigen::VectorXd linspace(double start, double end, const int num_points) {
  Eigen::VectorXd ret(num_points);

  for (int i = 0; i < num_points; ++i) {
    ret(i) = start + i * (end - start) / (num_points - 1.0);
  }

  return ret;
}

double polyval(const Eigen::VectorXd &coeffs, const double x) {
  int degree = coeffs.size() - 1;
  double ret = 0.0;

  for (int i = degree; i >= 0; --i) {
    ret += std::pow(x, i) * coeffs(degree - i);
  }

  return ret;
}

Eigen::VectorXd polyfit(const Eigen::VectorXd &x, const Eigen::VectorXd &y,
                        const int degree) {
  int num_points = x.size();

  Eigen::MatrixXd A(num_points, degree + 1);
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < degree + 1; ++j) {
      A(i, j) = std::pow(x(i), degree - j);
    }
  }

  Eigen::VectorXd coefficients = A.householderQr().solve(y);

  return coefficients;
}

void SlopeEstimation::UndistortLane(Line &line) {
  int n = 80;
  Eigen::MatrixXd xy0(n, 2), xy1(n, 2);
  std::vector<cv::Point2d> pts0_, pts1_;
  auto Y = linspace(line.start_y, line.end_y, n);
  for (int i = 0; i < n; ++i) {
    double x =
        polyval(Eigen::Vector4d(line.c3, line.c2, line.c1, line.c0), Y(i));
    pts0_.emplace_back(x, Y(i));
  }

  cv::undistortPoints(pts0_, pts1_, camera_params.intrinsic_distort_cv,
                      camera_params.distortion_cv, cv::noArray(),
                      camera_params.intrinsic_cv);

  for (int i = 0; i < n; ++i) {
    xy1.row(i) << pts1_[i].x, pts1_[i].y;
  }

  Eigen::Vector4d param = polyfit(xy1.col(1), xy1.col(0), 3);
  Eigen::Vector2d yy(xy1(0, 1), xy1(n - 1, 1));
  line.c3 = param(0);
  line.c2 = param(1);
  line.c1 = param(2);
  line.c0 = param(3);
  line.start_y = yy(0);
  line.end_y = yy(1);
}

}  // namespace mr