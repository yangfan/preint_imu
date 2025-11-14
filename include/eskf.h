#pragma once
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "gnss.h"
#include "imu.h"
#include "initializer.h"
#include "odom.h"
#include "utils/math.hpp"

class ESKF {
public:
  struct Noise {
    double var_bg_ = 1e-6; // contineous: rad / (s * sqrt(s))
    double var_ba_ = 1e-4; // continuous: m / (s^2 * sqr(s))
    double var_odom = 0.25;
    double var_gnss_pos = 0.01;
    double var_gnss_height = 0.01;
    double var_gnss_ang = 1.0 * Math::kdeg2rad * Math::kdeg2rad;
    Eigen::Vector3d dia_var_g_ = Eigen::Vector3d::Zero(); // discrete rad / s
    Eigen::Vector3d dia_var_a_ = Eigen::Vector3d::Zero(); // discrete m / (s*s)
  };

  void set_origin(const Sophus::SE3d &Two) { origin_ = Two; }
  Sophus::SE3d origin() const { return origin_; }
  bool noise_initialized() const { return set_init_noise_; }
  bool pose_initialized() const { return set_init_pose_; }
  IMUState state() const { return nominal_state_; }

  void initialize_noise(const Initializer &initializer);
  void initialize_pose(const Sophus::SE3d &Tob, const double gnss_timestamp);
  void initialize_pose(const GNSS &gnss_data);

  bool predict_imu(const IMU &imu_data);
  bool correct_gnss(const Sophus::SE3d &Tob, const double gnss_timestamp);
  bool correct_gnss(const GNSS &gnss_data);

  bool correct_odom(const Odom &odom_data);
  bool correct_state();
  bool reset_error();

  bool update_time(const double time);

private:
  IMUState nominal_state_;
  IMUState error_state_;
  Eigen::Matrix<double, 18, 18> cov_ = Eigen::Matrix<double, 18, 18>::Zero();

  Noise noise_;
  Sophus::SE3d origin_; // T_wo
  double timestamp_ = 0.0;

  bool set_init_pose_ = false;
  bool set_init_noise_ = false;
};