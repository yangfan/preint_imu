#include <fstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <string>

#include "eskf.h"
#include "initializer.h"
#include "odom.h"
#include "processor.h"

DEFINE_string(data_path, "./data/input/sensor_data.txt", "Data file path");
DEFINE_string(output_path, "./data/output/eskf.txt", "Output file path");
DEFINE_double(antenna_x, -0.17, "Antenna pos x to IMU");
DEFINE_double(antenna_y, -0.20, "Antenna pos y to IMU");
DEFINE_double(antenna_angle, 12.06, "Antenna angle (deg) to IMU"); // deg
DEFINE_bool(use_odom, true, "Use odom for the correction.");

int main(int argc, char **argv) {
  FLAGS_log_dir = std::string("./logs");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::ifstream ifs(FLAGS_data_path);
  std::ofstream ofs(FLAGS_output_path);
  Processor processor;
  ESKF eskf;
  Initializer initializer;

  auto save_state = [&]() {
    processor.write(ofs, eskf.state().timestamp);
    processor.write(ofs, eskf.state().pos);
    processor.write(ofs, eskf.state().rot.unit_quaternion());
    processor.write(ofs, eskf.state().vel);
    // processor.write(ofs, eskf.state().bias_g);
    // processor.write(ofs, eskf.state().bias_a);
    // processor.write(ofs, eskf.state().gravity);
    ofs << std::endl;
  };
  LOG(INFO) << "Input data path: " << FLAGS_data_path;
  LOG(INFO) << "Output data path: " << FLAGS_output_path;

  auto imu_processor = [&](const IMU &imu_data) {
    if (!initializer.success()) {
      initializer.addImu(imu_data);
      return;
    }
    if (!eskf.noise_initialized()) {
      eskf.initialize_noise(initializer);
    }
    if (eskf.noise_initialized() && eskf.pose_initialized()) {
      eskf.predict_imu(imu_data);
      save_state();
    }
  };

  auto odom_processor = [&](const Odom &odom_data) {
    initializer.addOdom(odom_data);
    if (FLAGS_use_odom && eskf.noise_initialized() && eskf.pose_initialized()) {
      eskf.correct_odom(odom_data);
      save_state();
    }
  };

  GNSS::set_config(FLAGS_antenna_x, FLAGS_antenna_y, FLAGS_antenna_angle);
  auto gnss_processor = [&](GNSS &gnss_data) {
    if (!eskf.noise_initialized() || !gnss_data.convert_utm() ||
        !gnss_data.valid_heading()) {
      return;
    }
    // Sophus::SE3d Tob = gnss_data.body_pose(FLAGS_antenna_x, FLAGS_antenna_y,
    //                                        FLAGS_antenna_angle,
    //                                        eskf.origin());
    if (eskf.noise_initialized() && eskf.pose_initialized()) {
      // if (!gnss_data.valid_heading()) {
      //   Tob.so3() = eskf.state().rot; // keep rotation state unchanged
      // }
      eskf.correct_gnss(gnss_data);
      save_state();
    }
    if (!eskf.pose_initialized() && gnss_data.valid_heading()) {
      // eskf.set_origin(Sophus::SE3d(Sophus::SO3d(), Tob.translation()));
      // Tob.translation() = Eigen::Vector3d::Zero();
      // eskf.initialize_pose(Tob, gnss_data.timestamp());
      eskf.initialize_pose(gnss_data);
    }
  };
  processor.setIMUProcessor(imu_processor);
  processor.setOdomProcessor(odom_processor);
  processor.setGNSSProcessor(gnss_processor);
  processor.process(ifs);

  return 0;
}