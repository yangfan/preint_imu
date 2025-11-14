#pragma once
#include "imu.h"
#include "odom.h"
#include <Eigen/Core>
#include <deque>

class Initializer {
public:
  bool addImu(const IMU &imu_data);
  bool addOdom(const Odom &odom_data);
  bool initialize();
  bool success() const { return initialized_; }

  struct Config {
    size_t deque_cap = 2000;
    size_t deque_min = 10;
    double static_time = 10.0;
    double static_vel = 0.01;
    double kgravity = 9.81;
  };

  Eigen::Vector3d bias_a() const { return init_bias_a_; }
  Eigen::Vector3d var_a() const { return init_var_a_; }

  Eigen::Vector3d bias_g() const { return init_bias_g_; }
  Eigen::Vector3d var_g() const { return init_var_g_; }

  Eigen::Vector3d gravity() const { return init_grav_; }
  double timestamp() const { return timestamp_; }

private:
  Eigen::Vector3d init_bias_g_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_bias_a_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_var_g_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_var_a_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_grav_ = Eigen::Vector3d::Zero();

  std::deque<IMU> imu_deque_;

  bool initialized_ = false;
  bool static_status_ = false;
  double timestamp_ = 0;

  Config config_;
};