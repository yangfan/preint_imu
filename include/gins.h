#pragma once

#include <Eigen/Core>
#include <memory>

#include "gnss.h"
#include "imu.h"
#include "initializer.h"
#include "odom.h"
#include "utils/math.hpp"

class GINS {
public:
  struct Noise {
    double var_odom = 0.0025;
    double var_gnss_pos = 0.01;
    double var_gnss_height = 0.01;
    double var_gnss_ang = 1.0 * Math::kdeg2rad * Math::kdeg2rad;
    Eigen::Vector3d dia_var_g = Eigen::Vector3d::Zero(); // discrete rad / s
    Eigen::Vector3d dia_var_a = Eigen::Vector3d::Zero(); // discrete m / (s *s)
  };
  struct Keyframe {
    IMUState state;
    GNSS gnss;
  };

  void initialize_noise(const Initializer &initializer);
  bool noise_initialized() const { return set_init_noise_; }

  void initialize_pose(const GNSS &gnss_data);
  bool pose_initialized() const { return set_init_pose_; }

  bool add_imu(const IMU &imu_data);
  bool add_gnss(const GNSS &gnss_data);
  bool add_odom(const Odom &odom_data);

  bool optimize();

  IMUState state() const { return preintegrator_->predict(prev_kf_.state); }
  Sophus::SE3d origin() const { return origin_; }

  void set_odom_correction() { odom_correction_ = true; }

private:
  Keyframe curr_kf_;
  Keyframe prev_kf_;

  IMU last_imu_;
  Odom last_odom_;
  bool last_odom_used = true;

  std::unique_ptr<IMUPreintegrator> preintegrator_;

  bool set_init_pose_ = false;
  Sophus::SE3d origin_; // T_wo
  bool set_init_noise_ = false;
  Noise noise_;
  Eigen::Matrix<double, 15, 15> prior_info_ =
      Eigen::Matrix<double, 15, 15>::Identity() * 1e2;
  Eigen::Matrix3d bias_ainfo_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d bias_ginfo_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d odom_info_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d gnss_pos_info_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 6> gnss_info_ =
      Eigen::Matrix<double, 6, 6>::Identity();

  bool odom_correction_ = false;
  void odom_correct(const Odom &odom_data);
};