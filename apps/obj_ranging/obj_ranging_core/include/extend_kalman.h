#ifndef EXTEND_KALMAN_FILER_H
#define EXTEND_KALMAN_FILER_H

#include <Eigen/Dense>

class ExtendKalmanFilter {
 public:
  ExtendKalmanFilter(int dim_x) : dim_x_(dim_x) {
    x_ = Eigen::VectorXd::Zero(dim_x);
    P_ = Eigen::MatrixXd::Identity(dim_x, dim_x);
  }

  void Predict() {
    // 预测步骤
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
  }

  void PredictUpdate(
      const Eigen::VectorXd &z,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> &HJacobi,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &Hx) {
    // 更新步骤
    Eigen::VectorXd predicted_measurement = Hx(x_);
    Eigen::MatrixXd H = HJacobi(x_);
    Eigen::VectorXd innovation = z - predicted_measurement;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * innovation;
    P_ = (Eigen::MatrixXd::Identity(dim_x_, dim_x_) - K * H) * P_;
  }

  void SetInitialConditions(const Eigen::VectorXd &x0,
                            const Eigen::MatrixXd &F0,
                            const Eigen::MatrixXd &Q0,
                            const Eigen::MatrixXd &R0) {
    x_ = x0;
    F_ = F0;
    Q_ = Q0;
    R_ = R0;
  }

  int dim_x_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

#endif  // !EXTEND_KALMAN_FILER_H