#include "eskf.h"
#include <glog/logging.h>

void ESKF::initialize_noise(const Initializer &initializer) {
  noise_.dia_var_g_ = initializer.var_g();
  noise_.dia_var_a_ = initializer.var_a();
  nominal_state_.gravity = initializer.gravity();
  nominal_state_.vel = Eigen::Vector3d::Zero(); // static status
  cov_ = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;
  set_init_noise_ = true;
  update_time(initializer.timestamp());
  LOG(INFO) << "Noise initialized.";
}
void ESKF::initialize_pose(const Sophus::SE3d &Tob,
                           const double gnss_timestamp) {
  nominal_state_.pos = Tob.translation();
  nominal_state_.rot = Tob.so3();
  set_init_pose_ = true;
  update_time(gnss_timestamp);
  LOG(INFO) << "Pose initialized.";
}

void ESKF::initialize_pose(const GNSS &gnss_data) {
  origin_ = gnss_data.body_pose(Sophus::SE3d());
  nominal_state_.pos.setZero();
  nominal_state_.rot = origin_.so3();
  origin_.so3() = Sophus::SO3d();
  set_init_pose_ = true;
  update_time(gnss_data.timestamp());
  LOG(INFO) << "Pose initialized.";
}

bool ESKF::update_time(const double time) {
  if (timestamp_ >= time) {
    LOG(INFO) << "time " << time << " is older than current: " << timestamp_;
    return false;
  }
  timestamp_ = time;
  nominal_state_.timestamp = time;
  error_state_.timestamp = time;
  return true;
}

bool ESKF::predict_imu(const IMU &imu_data) {
  const double dt = imu_data.timestamp - nominal_state_.timestamp;
  if (timestamp_ == 0 || dt <= 0) {
    LOG(INFO) << "Invalid time interval. Skip current data.";
    update_time(imu_data.timestamp);
    return false;
  }
  IMUState predicted_state;
  predicted_state.pos =
      nominal_state_.pos + nominal_state_.vel * dt +
      0.5 * (nominal_state_.rot * (imu_data.acc - nominal_state_.bias_a)) * dt *
          dt +
      0.5 * nominal_state_.gravity * dt * dt;
  predicted_state.vel =
      nominal_state_.vel +
      nominal_state_.rot * (imu_data.acc - nominal_state_.bias_a) * dt +
      nominal_state_.gravity * dt;
  predicted_state.rot =
      nominal_state_.rot *
      Sophus::SO3d::exp((imu_data.gyr - nominal_state_.bias_g) * dt);
  // ba, bg, g unchanged

  Eigen::Matrix<double, 18, 18> F =
      Eigen::Matrix<double, 18, 18>::Identity(); // jacobian of motion model
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(3, 6) =
      -(nominal_state_.rot.matrix() *
        Sophus::SO3d::hat(imu_data.acc - nominal_state_.bias_a) * dt);
  F.block<3, 3>(3, 12) = -nominal_state_.rot.matrix() * dt;
  F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(6, 6) =
      Sophus::SO3d::exp(-(imu_data.gyr - nominal_state_.bias_g) * dt).matrix();
  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

  // error state unchanged
  // IMUState predict_err_state(F * error_state_.vec());

  Eigen::Matrix<double, 18, 18> Q =
      Eigen::Matrix<double, 18, 18>::Zero(); // covariance of motion noise
  Q.diagonal() << 0, 0, 0, noise_.dia_var_a_ * dt * dt,
      noise_.dia_var_g_ * dt * dt, noise_.var_bg_ * dt, noise_.var_bg_ * dt,
      noise_.var_bg_ * dt, noise_.var_ba_ * dt, noise_.var_ba_ * dt,
      noise_.var_ba_ * dt, 0, 0, 0;
  // Q.diagonal() << 0, 0, 0, noise_.dia_var_a_, noise_.dia_var_g_,
  // noise_.var_bg_,
  //     noise_.var_bg_, noise_.var_bg_, noise_.var_ba_, noise_.var_ba_,
  //     noise_.var_ba_, 0, 0, 0;

  cov_ = F * cov_.eval() * F.transpose() + Q;
  nominal_state_.rot = predicted_state.rot;
  nominal_state_.pos = predicted_state.pos;
  nominal_state_.vel = predicted_state.vel;
  update_time(imu_data.timestamp);

  return true;
}
bool ESKF::correct_gnss(const GNSS &gnss_data) {
  Sophus::SE3d Tob = gnss_data.body_pose(origin_);
  if (!gnss_data.valid_heading()) {
    Tob.so3() = state().rot;
  }
  return correct_gnss(Tob, gnss_data.timestamp());
}
bool ESKF::correct_gnss(const Sophus::SE3d &Tob, const double gnss_timestamp) {
  if (gnss_timestamp <= timestamp_) {
    LOG(WARNING) << "The GNSS data timestamp is older than the system current "
                    "time stamp. Skip data.";
    return false;
  }
  Eigen::Matrix<double, 6, 1> obs_err;
  obs_err.head<3>() = Tob.translation() - nominal_state_.pos;
  obs_err.tail<3>() = (nominal_state_.rot.inverse() * Tob.so3()).log();

  Eigen::Matrix<double, 6, 18> H =
      Eigen::Matrix<double, 6, 18>::Zero(); // jacobian of observation model
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> V =
      Eigen::Matrix<double, 6, 6>::Zero(); // covariance of observation noise
  V.diagonal() << noise_.var_gnss_pos, noise_.var_gnss_pos,
      noise_.var_gnss_height, noise_.var_gnss_ang, noise_.var_gnss_ang,
      noise_.var_gnss_ang;
  Eigen::Matrix<double, 18, 6> K =
      cov_ * H.transpose() *
      ((H * cov_ * H.transpose() + V).inverse()); // Kalman gain

  Eigen::Matrix<double, 18, 1> err_state = K * obs_err;
  error_state_ = IMUState(err_state);
  cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_.eval();
  correct_state();

  reset_error();
  update_time(gnss_timestamp);

  return true;
}
bool ESKF::correct_odom(const Odom &odom_data) {
  if (odom_data.timestamp() <= timestamp_) {
    LOG(WARNING)
        << "The odom data is older than the current timestamp. Skip data.";
    return false;
  }
  Eigen::Vector3d vel_b(odom_data.velocity(), 0, 0);
  Eigen::Matrix<double, 3, 1> obs_err =
      nominal_state_.rot * vel_b - nominal_state_.vel;
  Eigen::Matrix<double, 3, 18> H =
      Eigen::Matrix<double, 3, 18>::Zero(); // jacobian of observation model
  H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d V =
      Eigen::Matrix3d::Identity() * noise_.var_odom; // covariance of odom noise
  Eigen::Matrix<double, 18, 3> K =
      cov_ * H.transpose() * ((H * cov_ * H.transpose() + V).inverse());

  Eigen::Matrix<double, 18, 1> err_state = K * obs_err;
  // err_state.segment(3, 3) = obs_err; // use odom velocity directly
  error_state_ = IMUState(err_state);
  cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_.eval();
  correct_state();

  reset_error();
  update_time(odom_data.timestamp());
  return true;
}
bool ESKF::correct_state() {
  nominal_state_.pos += error_state_.pos;
  nominal_state_.vel += error_state_.vel;
  nominal_state_.rot *= error_state_.rot;
  nominal_state_.bias_g += error_state_.bias_g;
  nominal_state_.bias_a += error_state_.bias_a;
  nominal_state_.gravity += error_state_.gravity;

  return true;
}
bool ESKF::reset_error() {
  Eigen::Matrix<double, 18, 18> J =
      Eigen::Matrix<double, 18, 18>::Identity(); // jacobian of reset function
  J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() -
                        0.5 * Sophus::SO3d::hat(error_state_.rot.log());
  cov_ = J * cov_ * J.transpose();
  error_state_ = IMUState(Eigen::Matrix<double, 18, 1>::Zero());
  error_state_.rot = Sophus::SO3d();
  return true;
}