#ifndef KALMAN_FILER_H
#define KALMAN_FILER_H

#include <iostream>
#include <vector>

class KalmanFilter {
 public:
  // KalmanFilter(double x0, double P0) : x(x0), P(P0) {}

  void Update(double z, double R) {
    if (init_) {
      double y = z - x;  // Measurement residual
      double S = P + R;  // Residual covariance
      double K = P / S;  // Kalman gain

      x = x + K * y;    // Updated state estimatecd
      P = (1 - K) * P;  // Updated covariance estimate
    } else {
      x = z;
      P = R;
      init_ = true;
    }
  }

  double GetState() const { return x; }

  double GetCovariance() const { return P; }

 private:
  bool init_{false};
  double x{0.0};  // State estimate
  double P{0.0};  // Covariance estimate
};

#endif