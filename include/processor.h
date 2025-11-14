#pragma once

#include "gnss.h"
#include "imu.h"
#include "odom.h"
#include <fstream>

class Processor {
public:
  bool process(std::ifstream &ifs);
  bool write(std::ofstream &ofs, const double timestamp);
  bool write(std::ofstream &ofs, const Eigen::Vector3d &vec);
  bool write(std::ofstream &ofs, const Eigen::Quaterniond &quat);

  using GNSSProcessor = std::function<void(GNSS &)>;
  using IMUProcessor = std::function<void(const IMU)>;
  using OdomProcessor = std::function<void(const Odom &)>;
  Processor &setGNSSProcessor(GNSSProcessor process_fun);
  Processor &setIMUProcessor(IMUProcessor process_fun);
  Processor &setOdomProcessor(OdomProcessor process_fun);

private:
  GNSSProcessor gnss_processor_;
  IMUProcessor imu_processor_;
  OdomProcessor odom_processor_;
};