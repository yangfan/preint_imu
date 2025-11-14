#pragma once
#include <Eigen/Core>

class Odom {
public:
  Odom() = default;
  Odom(const double time, const double left_impulse, const double right_impulse)
      : timestamp_(time), left_impulse_(left_impulse),
        right_impulse_(right_impulse) {
    const double left_v =
        (2 * M_PI * radius_ * left_impulse) / (loop_impulse_ * time_interval_);
    const double right_v =
        (2 * M_PI * radius_ * right_impulse) / (loop_impulse_ * time_interval_);
    velocity_ = (left_v + right_v) / 2.0;
  }
  double velocity() const { return velocity_; };
  double timestamp() const { return timestamp_; }

private:
  static inline double radius_ = 0.155;
  static inline double loop_impulse_ = 1024.0;
  static inline double time_interval_ = 0.1;

  double timestamp_ = 0.0;
  double left_impulse_ = 0.0;
  double right_impulse_ = 0.0;
  double velocity_ = 0.0;
};