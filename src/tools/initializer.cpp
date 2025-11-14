#include "initializer.h"
#include "utils/math.hpp"
#include <glog/logging.h>

bool Initializer::addImu(const IMU &imu_data) {
  if (initialized_) {
    return true;
  }
  if (!static_status_) {
    return false;
  }
  while (imu_deque_.size() >= config_.deque_cap) {
    imu_deque_.pop_front();
  }
  imu_deque_.push_back(imu_data);

  if (imu_deque_.size() > config_.deque_min &&
      imu_data.timestamp - imu_deque_.front().timestamp >=
          config_.static_time) {
    initialized_ = initialize();
  }
  return initialized_;
}

bool Initializer::addOdom(const Odom &odom_data) {
  if (initialized_) {
    return true;
  }
  static_status_ = odom_data.velocity() < config_.static_vel;
  // LOG(INFO) << "vel: " << odom_data.velocity();
  return true;
}

bool Initializer::initialize() {
  LOG(INFO) << "deque size: " << imu_deque_.size();
  Math::MeanVariance(imu_deque_, init_bias_g_, init_var_g_,
                     [](const auto &imu) { return imu.gyr; });
  Eigen::Vector3d raw_bias_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d raw_var_a = Eigen::Vector3d::Zero();
  Math::MeanVariance(imu_deque_, raw_bias_a, raw_var_a,
                     [](const auto &imu) { return imu.acc; });

  LOG(INFO) << "raw bias a: " << raw_bias_a.transpose()
            << ", norm: " << raw_bias_a.norm();
  init_grav_ = -(raw_bias_a.normalized()) * config_.kgravity;
  Math::MeanVariance(
      imu_deque_, init_bias_a_, init_var_a_,
      [this](const auto &imu) { return (imu.acc + init_grav_); });
  timestamp_ = imu_deque_.back().timestamp;

  LOG(INFO) << "Initial mean acc bias: " << init_bias_a_.transpose();
  LOG(INFO) << "Initial mean acc varianc: " << init_var_a_.transpose();

  LOG(INFO) << "Initial mean gyr bias: " << init_bias_g_.transpose();
  LOG(INFO) << "Initial mean gyr varianc: " << init_var_g_.transpose();

  LOG(INFO) << "Initial gravity: " << init_grav_.transpose();
  LOG(INFO) << std::fixed << "Initial timestamp: " << timestamp_;

  return true;
}
