#pragma once
#include <iostream>
#include <string>

namespace mr {
struct Line {
  double start_y{-1};  // 车道起始y
  double end_y{-1};    // 车道终点y
  double c0{-1};       // 常系数
  double c1{-1};       // y^1系数
  double c2{-1};       // y^2系数
  double c3{-1};       // y^3系数

  bool CalX(double y, double &x) {
    if (y > end_y || y < start_y) return false;
    x = c0 + c1 * y + c2 * y * y + c3 * y * y * y;
    return true;
  }

  std::string ToString() {
    return std::string(
        "start_y: " + std::to_string(start_y) +
        " end_y: " + std::to_string(end_y) + " c3: " + std::to_string(c3) +
        " c2: " + std::to_string(c2) + " c1: " + std::to_string(c1) +
        " c0: " + std::to_string(c0));
  }
};

// 如果有两段检测线,一定让下面的线放在first
struct DoubleLine {
  int param_num{0};
  Line first;
  Line second;

  bool CalX(double y, double &x) {
    if (param_num == 1) {
      return first.CalX(y, x);
    } else {
      return first.CalX(y, x) || second.CalX(y, x) || CalMiddle(y, x);
    }
  }

  // 连接两条线的端点,计算中间线上的值
  bool CalMiddle(double y, double &x) {
    if (y > second.end_y && y < first.start_y) {
      double x_first, x_second;
      second.CalX(second.end_y, x_second);
      first.CalX(first.start_y, x_first);
      // 这里不必对0做除数的情况限制,if条件可以保证
      double gradient = (x_first - x_second) / (first.start_y - second.end_y);
      x = gradient * (y - second.end_y) + x_second;
      return true;
    }
    return false;
  }
};

class SlopeEstimation {
 public:
  bool UpdateDetectLane(DoubleLine &left, DoubleLine &right);
  void UpdateTrackLane(Line &left, Line &right);
  float GetDistance(double pixel, float dist_pitch);
  void ResetState() { state_ = true; }
  void InvalidState() { state_ = false; }
  bool GetState() { return state_; }

 private:
  void UndistortLane(Line &);

 private:
  bool state_{true};  // 可用状态
  DoubleLine detect_left_;
  DoubleLine detect_right_;
  Line track_left_;
  Line track_right_;
  int recursion_{0};
};

extern SlopeEstimation slope;
}  // namespace mr