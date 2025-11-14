#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <string>

#include "gins.h"
#include "initializer.h"
#include "processor.h"

DEFINE_string(data_path, "./data/input/sensor_data.txt", "Data file path");
DEFINE_string(output_path, "./data/output/gins.txt", "Output file path");
DEFINE_double(antenna_x, -0.17, "Antenna pos x to IMU");
DEFINE_double(antenna_y, -0.20, "Antenna pos y to IMU");
DEFINE_double(antenna_angle, 12.06, "Antenna angle (deg) to IMU"); // deg
DEFINE_bool(only_gnss_valid_heading, false,
            "Use only gnss data with valid heading");

int main(int argc, char **argv) {
  FLAGS_log_dir = std::string("./logs");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::ifstream ifs(FLAGS_data_path);
  std::ofstream ofs(FLAGS_output_path);
  Processor processor;
  Initializer initializer;
  GINS gins;

  auto save_state = [&](const IMUState state) {
    processor.write(ofs, state.timestamp);
    processor.write(ofs, state.pos);
    processor.write(ofs, state.rot.unit_quaternion());
    processor.write(ofs, state.vel);
    ofs << std::endl;
  };

  auto imu_processor = [&](const IMU &imu_data) {
    if (!initializer.success()) {
      initializer.addImu(imu_data);
      return;
    }
    if (!gins.noise_initialized()) {
      gins.initialize_noise(initializer);
    }
    if (gins.noise_initialized() && gins.pose_initialized()) {
      gins.add_imu(imu_data);
      save_state(gins.state());
    }
  };

  auto odom_processor = [&](const Odom &odom_data) {
    if (!gins.noise_initialized()) {
      initializer.addOdom(odom_data);
    }
    if (gins.noise_initialized() && gins.pose_initialized()) {
      gins.add_odom(odom_data);
    }
  };

  GNSS::set_config(FLAGS_antenna_x, FLAGS_antenna_y, FLAGS_antenna_angle);
  auto gnss_processor = [&](GNSS &gnss_data) {
    // if (!gins.noise_initialized() || !gnss_data.convert_utm() ||
    //     !gnss_data.valid_heading()) {
    if (!gins.noise_initialized() || !gnss_data.convert_utm()) {
      return;
    }
    if (FLAGS_only_gnss_valid_heading && !gnss_data.valid_heading()) {
      return;
    }
    if (gins.pose_initialized() && gins.noise_initialized()) {
      gins.add_gnss(gnss_data);
      save_state(gins.state());
    }
    if (!gins.pose_initialized() && gnss_data.valid_heading()) {
      gins.initialize_pose(gnss_data);
    }
  };

  processor.setGNSSProcessor(gnss_processor);
  processor.setIMUProcessor(imu_processor);
  processor.setOdomProcessor(odom_processor);
  processor.process(ifs);

  return 0;
}