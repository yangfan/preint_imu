#include "processor.h"

#include <glog/logging.h>
#include <iomanip>
#include <sstream>

bool Processor::process(std::ifstream &ifs) {
  if (!ifs.is_open()) {
    LOG(ERROR) << "Failed to load input file.";
    return false;
  }
  while (!ifs.eof()) {
    std::string line;
    std::getline(ifs, line);

    if (line[0] == '#') {
      continue;
    }
    std::stringstream ss(line);
    std::string type;
    ss >> type;
    if (type == "GNSS" && gnss_processor_) {
      double time{}, lat{}, lon{}, alt{}, head{};
      bool head_valid = false;
      ss >> time >> lat >> lon >> alt >> head >> head_valid;
      auto gps = GNSS(time, lat, lon, alt, head, head_valid);
      gnss_processor_(gps);
    } else if (type == "IMU" && imu_processor_) {
      double time{}, gx{}, gy{}, gz{}, ax{}, ay{}, az{};
      ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
      imu_processor_(
          IMU(time, Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az)));
    } else if (type == "ODOM" && odom_processor_) {
      double time{}, lw{}, rw{};
      ss >> time >> lw >> rw;

      odom_processor_(Odom(time, lw, rw));
    }
  }
  return true;
}

bool Processor::write(std::ofstream &ofs, const Eigen::Vector3d &vec) {
  if (!ofs) {
    LOG(ERROR) << "Failed to write output file.";
    return false;
  }
  ofs << std::setprecision(9) << vec[0] << " " << vec[1] << " " << vec[2]
      << " ";
  return true;
}

bool Processor::write(std::ofstream &ofs, const Eigen::Quaterniond &quat) {
  if (!ofs) {
    LOG(ERROR) << "Failed to write output file.";
    return false;
  }
  ofs << std::setprecision(9) << quat.w() << " " << quat.x() << " " << quat.y()
      << " " << quat.z() << " ";
  return true;
}

bool Processor::write(std::ofstream &ofs, const double timestamp) {
  if (!ofs) {
    LOG(ERROR) << "Failed to write output file.";
    return false;
  }
  ofs << std::setprecision(18) << timestamp << " ";
  return true;
}

Processor &Processor::setGNSSProcessor(GNSSProcessor process_fun) {
  gnss_processor_ = std::move(process_fun);
  return *this;
}

Processor &Processor::setIMUProcessor(IMUProcessor process_fun) {
  imu_processor_ = std::move(process_fun);
  return *this;
}

Processor &Processor::setOdomProcessor(OdomProcessor process_fun) {
  odom_processor_ = std::move(process_fun);
  return *this;
}