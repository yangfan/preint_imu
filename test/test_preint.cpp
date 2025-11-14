#include "eskf.h"
#include "imu.h"
#include "initializer.h"
#include "processor.h"
#include "types.h"

#include <Eigen/Core>
#include <fstream>
#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

TEST(TestPreint, Acc) {
  constexpr double dt = 0.01;
  const Eigen::Vector3d acc(1, 0, 0);
  const Eigen::Vector3d gravity(0, 0, -9.8);
  double timestamp = 0.0;

  IMUPreintegrator preintegrator;
  const IMUState init_state;
  IMUIntegrator integrator(init_state);

  for (int i = 0; i < 100; ++i) {
    timestamp += dt;
    Eigen::Vector3d accmeter = acc - gravity;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    const IMU imu_data(timestamp, gyro, accmeter);
    preintegrator.integrate(imu_data);
    IMUState prediction = preintegrator.predict(init_state);
    integrator.integrate(imu_data);

    Eigen::Quaterniond qaut_pre = prediction.rot.unit_quaternion();
    Eigen::Quaterniond qaut_int = integrator.state().rot.unit_quaternion();

    EXPECT_THAT(prediction.pos, testing::Pointwise(testing::FloatNear(1e-2),
                                                   integrator.state().pos));
    EXPECT_THAT(prediction.vel, testing::Pointwise(testing::FloatNear(1e-2),
                                                   integrator.state().vel));
    EXPECT_THAT(qaut_pre.coeffs(), testing::Pointwise(testing::FloatNear(1e-2),
                                                      qaut_int.coeffs()));
  }

  IMUState prediction = preintegrator.predict(init_state);
  LOG(INFO) << "vel: " << prediction.vel.transpose() << '\n';
  LOG(INFO) << "pos: " << prediction.pos.transpose() << '\n';
  LOG(INFO) << "rot: " << prediction.rot.unit_quaternion() << '\n';

  LOG(INFO) << "vel: " << integrator.state().vel.transpose() << '\n';
  LOG(INFO) << "pos: " << integrator.state().pos.transpose() << '\n';
  LOG(INFO) << "rot: " << integrator.state().rot.unit_quaternion() << '\n';
}

TEST(TestPreint, Rotate) {
  const double dt = 0.01;
  const Eigen::Vector3d accmeter(0, 0, 9.8);
  const Eigen::Vector3d gyrscope(0, 0, M_PI / 2);
  double timestamp = 0.0;

  IMUPreintegrator preintegrator;
  IMUState init_state;
  IMUIntegrator integrator(init_state);

  for (int i = 0; i < 100; ++i) {
    timestamp += dt;
    IMU imu_data(timestamp, gyrscope, accmeter);
    preintegrator.integrate(imu_data);
    integrator.integrate(imu_data);

    const IMUState prediction = preintegrator.predict(init_state);
    EXPECT_THAT(prediction.pos, testing::Pointwise(testing::FloatNear(1e-2),
                                                   integrator.state().pos));
    EXPECT_THAT(prediction.vel, testing::Pointwise(testing::FloatNear(1e-2),
                                                   integrator.state().vel));
    const Eigen::Vector4d quat_pre = prediction.rot.unit_quaternion().coeffs();
    const Eigen::Vector4d quat_int =
        integrator.state().rot.unit_quaternion().coeffs();
    EXPECT_THAT(quat_pre,
                testing::Pointwise(testing::FloatNear(1e-2), quat_int));
  }

  IMUState prediction = preintegrator.predict(init_state);
  LOG(INFO) << "vel: " << prediction.vel.transpose() << '\n';
  LOG(INFO) << "pos: " << prediction.pos.transpose() << '\n';
  LOG(INFO) << "rot: " << prediction.rot.unit_quaternion() << '\n';

  LOG(INFO) << "vel: " << integrator.state().vel.transpose() << '\n';
  LOG(INFO) << "pos: " << integrator.state().pos.transpose() << '\n';
  LOG(INFO) << "rot: " << integrator.state().rot.unit_quaternion() << '\n';
}

void optimize(IMUState &last_state, IMUState &this_state,
              const Sophus::SE3d &last_gnss_pose,
              const Sophus::SE3d &this_gnss_pose,
              IMUPreintegrator *preintegrator);
TEST(TestPreint, ESKF) {

  const std::string data_path = "./data/input/sensor_data.txt";
  const std::string output_path = "./data/output/gins.txt";
  const double antenna_x = -0.17;
  const double antenna_y = -0.20;
  const double antenna_angle = 12.06; // deg
  std::ifstream ifs(data_path);
  std::ofstream ofs(output_path);
  Processor processor;
  Initializer initializer;
  // GINS gins;
  ESKF eskf;
  std::unique_ptr<IMUPreintegrator> preintegrator = nullptr;

  IMUState last_state;
  bool set_last_state = false;
  GNSS last_gnss;
  bool set_last_gnss = false;
  size_t eskf_imu_cnt = 0;
  size_t preint_imu_cnt = 0;

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
      // eskf_imu_cnt++;
      if (preintegrator) {
        preintegrator->integrate(imu_data);
        // preint_imu_cnt++;
      }
      if (set_last_state) {
        IMUState preint_state = preintegrator->predict(last_state);
        Eigen::Vector3d diff = preint_state.pos - eskf.state().pos;
        // if (preint_state.timestamp - eskf.state().timestamp > 1e-5) {
        //   LOG(INFO) << "time_diff: "
        //             << preint_state.timestamp - eskf.state().timestamp;
        // }
        // if (diff.norm() > 1e-2) {
        //   LOG(INFO) << "pos_diff: " << diff.norm();
        // }
        EXPECT_NEAR((diff).norm(), 0, 0.01);
        diff = preint_state.vel - eskf.state().vel;
        // if (diff.norm() > 1e-2) {
        //   LOG(INFO) << "vel_diff: " << diff.norm();
        //   LOG(INFO) << "preint imu cnt: " << preint_imu_cnt;
        //   LOG(INFO) << "eskf imu cnt: " << eskf_imu_cnt;
        //   // LOG(INFO) << "inter vel: "
        //   //           << preintegrator->inter.state().vel.transpose();
        //   LOG(INFO) << "preint vel: " << preint_state.vel.transpose();
        //   LOG(INFO) << "eskf vel: " << eskf.state().vel.transpose();
        // }
        EXPECT_NEAR((diff).norm(), 0, 0.01);
        diff = (preint_state.rot.inverse() * eskf.state().rot).log();
        // if (diff.norm() > 0.1) {
        //   LOG(INFO) << "rot_diff: " << diff.norm();
        // }
        EXPECT_NEAR((diff).norm(), 0, 0.01);
      }
    }
  };

  auto odom_processor = [&](const Odom &odom_data) {
    if (!eskf.noise_initialized()) {
      initializer.addOdom(odom_data);
    }
  };

  GNSS::set_config(antenna_x, antenna_y, antenna_angle);
  auto gnss_processor = [&](GNSS &gnss_data) {
    if (!eskf.noise_initialized() || !gnss_data.convert_utm() ||
        !gnss_data.valid_heading()) {
      return;
    }
    if (eskf.pose_initialized() && eskf.noise_initialized()) {
      const Eigen::Vector3d pos_before = eskf.state().pos;
      eskf.correct_gnss(gnss_data);
      if (set_last_state && set_last_gnss) {
        IMUState eskf_state = eskf.state();
        const IMUState preint_state = preintegrator->predict(last_state);
        // LOG(INFO) << "eskf pos before: " << pos_before.transpose();
        // LOG(INFO) << "eskf pos after: " << eskf_state.pos.transpose();
        // LOG(INFO) << "preint pos: " << preint_state.pos.transpose();
        eskf_state.timestamp = gnss_data.timestamp();
        optimize(last_state, eskf_state, last_gnss.body_pose(eskf.origin()),
                 gnss_data.body_pose(eskf.origin()), preintegrator.get());
      }
      last_state = eskf.state();
      set_last_state = true;
      last_gnss = gnss_data;
      set_last_gnss = true;
      const Eigen::Vector3d var = Eigen::Vector3d::Ones() * 1e-4;
      preintegrator = std::make_unique<IMUPreintegrator>(
          last_state.bias_g, last_state.bias_a, var, var, last_state.timestamp);
      preintegrator->set_gravity(eskf.state().gravity);
      // preintegrator->inter = IMUIntegrator(last_state);
      // IMUState preint_state = preintegrator->predict(last_state);
      // Eigen::Vector3d diff = preint_state.pos - eskf.state().pos;
      // LOG(INFO) << "reset_pos_diff: " << diff.norm();
      // diff = preint_state.vel - eskf.state().vel;
      // LOG(INFO) << "reset_vel_diff: " << diff.norm();
      // LOG(INFO) << "reset_time_diff: "
      //           << preint_state.timestamp - eskf.state().timestamp;
      // LOG(INFO) << "reset_gravity_diff: "
      //           << preint_state.gravity - eskf.state().gravity;
      // preint_imu_cnt = 0;
      // eskf_imu_cnt = 0;
    }
    if (!eskf.pose_initialized() && gnss_data.valid_heading()) {
      eskf.initialize_pose(gnss_data);
    }
  };

  processor.setGNSSProcessor(gnss_processor);
  processor.setIMUProcessor(imu_processor);
  processor.setOdomProcessor(odom_processor);
  processor.process(ifs);
}

void optimize(IMUState &last_state, IMUState &this_state,
              const Sophus::SE3d &last_gnss_pose,
              const Sophus::SE3d &this_gnss_pose,
              IMUPreintegrator *preintegrator) {

  // LOG(INFO) << "start optimizaton.";
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  size_t vid = 0;
  VertexSO3 *rot_i = new VertexSO3();
  rot_i->setId(vid++);
  rot_i->setEstimate(last_state.rot);
  optimizer.addVertex(rot_i);

  VertexPos *pos_i = new VertexPos();
  pos_i->setId(vid++);
  pos_i->setEstimate(last_state.pos);
  optimizer.addVertex(pos_i);

  VertexVel *vel_i = new VertexVel();
  vel_i->setId(vid++);
  vel_i->setEstimate(last_state.vel);
  optimizer.addVertex(vel_i);

  VertexBiasG *bias_gi = new VertexBiasG();
  bias_gi->setId(vid++);
  bias_gi->setEstimate(last_state.bias_g);
  optimizer.addVertex(bias_gi);

  VertexBiasA *bias_ai = new VertexBiasA();
  bias_ai->setId(vid++);
  bias_ai->setEstimate(last_state.bias_a);
  optimizer.addVertex(bias_ai);

  VertexSO3 *rot_j = new VertexSO3();
  rot_j->setId(vid++);
  rot_j->setEstimate(this_state.rot);
  optimizer.addVertex(rot_j);

  VertexPos *pos_j = new VertexPos();
  pos_j->setId(vid++);
  pos_j->setEstimate(this_state.pos);
  optimizer.addVertex(pos_j);

  VertexVel *vel_j = new VertexVel();
  vel_j->setId(vid++);
  vel_j->setEstimate(this_state.vel);
  optimizer.addVertex(vel_j);

  VertexBiasG *bias_gj = new VertexBiasG();
  bias_gj->setId(vid++);
  bias_gj->setEstimate(this_state.bias_g);
  optimizer.addVertex(bias_gj);

  VertexBiasA *bias_aj = new VertexBiasA();
  bias_aj->setId(vid++);
  bias_aj->setEstimate(this_state.bias_a);
  optimizer.addVertex(bias_aj);

  size_t eid = 0;
  EdgePreint *edge_preint = new EdgePreint(
      preintegrator, this_state.timestamp - last_state.timestamp);
  edge_preint->setId(eid++);
  edge_preint->setVertex(0, rot_i);
  edge_preint->setVertex(1, rot_j);
  edge_preint->setVertex(2, vel_i);
  edge_preint->setVertex(3, vel_j);
  edge_preint->setVertex(4, pos_i);
  edge_preint->setVertex(5, pos_j);
  edge_preint->setVertex(6, bias_ai);
  edge_preint->setVertex(7, bias_gi);
  edge_preint->setInformation(preintegrator->info());
  auto *rk_huber = new g2o::RobustKernelHuber();
  rk_huber->setDelta(200.0);
  edge_preint->setRobustKernel(rk_huber);
  optimizer.addEdge(edge_preint);

  edge_preint->computeError();
  LOG(INFO) << "preint init error: " << edge_preint->chi2() << "/"
            << edge_preint->error().transpose();

  EdgeBiasA *edge_ba = new EdgeBiasA();
  edge_ba->setId(eid++);
  edge_ba->setVertex(0, bias_ai);
  edge_ba->setVertex(1, bias_aj);
  edge_ba->setInformation(Eigen::Matrix3d::Identity() * 1e6);
  optimizer.addEdge(edge_ba);

  EdgeBiasG *edge_bg = new EdgeBiasG();
  edge_bg->setId(eid++);
  edge_bg->setVertex(0, bias_gi);
  edge_bg->setVertex(1, bias_gj);
  edge_bg->setInformation(Eigen::Matrix3d::Identity() * 1e6);
  optimizer.addEdge(edge_bg);

  EdgeGNSS *edge_gnss_i = new EdgeGNSS();
  edge_gnss_i->setId(eid++);
  edge_gnss_i->setMeasurement(last_gnss_pose);
  edge_gnss_i->setVertex(0, rot_i);
  edge_gnss_i->setVertex(1, pos_i);
  edge_gnss_i->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 1e2);
  optimizer.addEdge(edge_gnss_i);

  EdgeGNSS *edge_gnss_j = new EdgeGNSS();
  edge_gnss_j->setId(eid++);
  edge_gnss_j->setMeasurement(this_gnss_pose);
  edge_gnss_j->setVertex(0, rot_j);
  edge_gnss_j->setVertex(1, pos_j);
  edge_gnss_j->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 1e2);
  optimizer.addEdge(edge_gnss_j);
  LOG(INFO) << "gnss pos: "
            << edge_gnss_j->measurement().translation().transpose();

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  LOG(INFO) << "Optimization report chi2/error:";
  LOG(INFO) << "preint: " << edge_preint->chi2() << "/"
            << edge_preint->error().transpose();
  LOG(INFO) << "gnss i: " << edge_gnss_i->chi2() << "/"
            << edge_gnss_i->error().transpose();
  LOG(INFO) << "gnss j: " << edge_gnss_j->chi2() << "/"
            << edge_gnss_j->error().transpose();
  LOG(INFO) << "bias a: " << edge_ba->chi2() << "/"
            << edge_ba->error().transpose();
  LOG(INFO) << "bias g: " << edge_bg->chi2() << "/"
            << edge_bg->error().transpose();
  LOG(INFO) << "optimized pos: " << pos_j->estimate().transpose();
}

int main(int argc, char **argv) {
  FLAGS_log_dir = std::string("./logs");
  FLAGS_stderrthreshold = google::ERROR;
  FLAGS_colorlogtostderr = true;
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}