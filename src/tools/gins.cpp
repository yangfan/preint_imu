#include "gins.h"
#include "optimizer/g2o/types.h"
#include <glog/logging.h>

void GINS::initialize_noise(const Initializer &initializer) {
  LOG(INFO) << "Initializing noise.";
  noise_.dia_var_g = initializer.var_g();
  noise_.dia_var_a = initializer.var_a();

  preintegrator_ = std::make_unique<IMUPreintegrator>(
      initializer.bias_g(), initializer.bias_a(), initializer.var_g(),
      initializer.var_a(), initializer.timestamp());
  const Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.8);
  preintegrator_->set_gravity(gravity);
  prev_kf_.state.gravity = gravity;
  // preintegrator_->set_gravity(initializer.gravity());
  // prev_kf_.state.gravity = initializer.gravity();
  prev_kf_.state.bias_g = initializer.bias_g();
  prev_kf_.state.bias_a = initializer.bias_a();

  prior_info_.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;

  bias_ainfo_.diagonal() << 1 / noise_.dia_var_a[0], 1 / noise_.dia_var_a[1],
      1 / noise_.dia_var_a[2];
  bias_ginfo_.diagonal() << 1 / noise_.dia_var_g[0], 1 / noise_.dia_var_g[1],
      1 / noise_.dia_var_g[2];

  odom_info_.diagonal() << 1 / noise_.var_odom, 1 / noise_.var_odom,
      1 / noise_.var_odom;

  gnss_info_.diagonal() << 1 / noise_.var_gnss_ang, 1 / noise_.var_gnss_ang,
      1 / noise_.var_gnss_ang, 1 / noise_.var_gnss_pos, 1 / noise_.var_gnss_pos,
      1 / noise_.var_gnss_height;

  gnss_pos_info_.diagonal() << 1 / noise_.var_gnss_pos, 1 / noise_.var_gnss_pos,
      1 / noise_.var_gnss_height;

  set_init_noise_ = true;
  LOG(INFO) << "noise initialized.";
}

void GINS::initialize_pose(const GNSS &gnss_data) {
  origin_ = gnss_data.body_pose(Sophus::SE3d());
  prev_kf_.gnss = gnss_data;
  prev_kf_.state.pos = Eigen::Vector3d::Zero();
  prev_kf_.state.rot = origin_.so3();
  origin_.so3() = Sophus::SO3d();
  prev_kf_.state.vel = Eigen::Vector3d::Zero();
  prev_kf_.state.timestamp = gnss_data.timestamp();
  preintegrator_->set_time(gnss_data.timestamp());
  set_init_pose_ = true;
  LOG(INFO) << "Pose initialized.";
}

bool GINS::add_imu(const IMU &imu_data) {
  preintegrator_->integrate(imu_data);
  last_imu_ = imu_data;
  return true;
}

bool GINS::add_gnss(const GNSS &gnss_data) {
  last_imu_.timestamp = gnss_data.timestamp();
  // preintegrate motion between last imu and gnss data.
  // todo: add threshold on time diff between last imu and gnss
  preintegrator_->integrate(last_imu_);

  curr_kf_.state = preintegrator_->predict(prev_kf_.state);
  curr_kf_.gnss = gnss_data;
  optimize();

  preintegrator_->reset(curr_kf_.state.bias_g, curr_kf_.state.bias_a,
                        noise_.dia_var_g, noise_.dia_var_a,
                        curr_kf_.gnss.timestamp());
  prev_kf_ = curr_kf_;
  return true;
}

bool GINS::add_odom(const Odom &odom_data) {
  last_odom_ = odom_data;
  last_odom_used = false;
  IMUState predicted_state = preintegrator_->predict(prev_kf_.state);
  return true;
}

bool GINS::optimize() {
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
  rot_i->setEstimate(prev_kf_.state.rot);
  optimizer.addVertex(rot_i);

  VertexPos *pos_i = new VertexPos();
  pos_i->setId(vid++);
  pos_i->setEstimate(prev_kf_.state.pos);
  optimizer.addVertex(pos_i);

  VertexVel *vel_i = new VertexVel();
  vel_i->setId(vid++);
  vel_i->setEstimate(prev_kf_.state.vel);
  optimizer.addVertex(vel_i);

  VertexBiasG *bias_gi = new VertexBiasG();
  bias_gi->setId(vid++);
  bias_gi->setEstimate(prev_kf_.state.bias_g);
  optimizer.addVertex(bias_gi);

  VertexBiasA *bias_ai = new VertexBiasA();
  bias_ai->setId(vid++);
  bias_ai->setEstimate(prev_kf_.state.bias_a);
  optimizer.addVertex(bias_ai);

  VertexSO3 *rot_j = new VertexSO3();
  rot_j->setId(vid++);
  rot_j->setEstimate(curr_kf_.state.rot);
  optimizer.addVertex(rot_j);

  VertexPos *pos_j = new VertexPos();
  pos_j->setId(vid++);
  pos_j->setEstimate(curr_kf_.state.pos);
  optimizer.addVertex(pos_j);

  VertexVel *vel_j = new VertexVel();
  vel_j->setId(vid++);
  vel_j->setEstimate(curr_kf_.state.vel);
  optimizer.addVertex(vel_j);

  VertexBiasG *bias_gj = new VertexBiasG();
  bias_gj->setId(vid++);
  bias_gj->setEstimate(curr_kf_.state.bias_g);
  optimizer.addVertex(bias_gj);

  VertexBiasA *bias_aj = new VertexBiasA();
  bias_aj->setId(vid++);
  bias_aj->setEstimate(curr_kf_.state.bias_a);
  optimizer.addVertex(bias_aj);

  size_t eid = 0;
  EdgePreint *edge_preint =
      new EdgePreint(preintegrator_.get(),
                     curr_kf_.gnss.timestamp() - prev_kf_.gnss.timestamp());
  edge_preint->setId(eid++);
  edge_preint->setVertex(0, rot_i);
  edge_preint->setVertex(1, rot_j);
  edge_preint->setVertex(2, vel_i);
  edge_preint->setVertex(3, vel_j);
  edge_preint->setVertex(4, pos_i);
  edge_preint->setVertex(5, pos_j);
  edge_preint->setVertex(6, bias_ai);
  edge_preint->setVertex(7, bias_gi);
  edge_preint->setInformation(preintegrator_->info());
  auto *rk_huber = new g2o::RobustKernelHuber();
  rk_huber->setDelta(200.0);
  edge_preint->setRobustKernel(rk_huber);
  optimizer.addEdge(edge_preint);

  EdgePrior *edge_prior = new EdgePrior(prev_kf_.state);
  edge_prior->setId(eid++);
  edge_prior->setVertex(0, rot_i);
  edge_prior->setVertex(1, pos_i);
  edge_prior->setVertex(2, vel_i);
  edge_prior->setVertex(3, bias_ai);
  edge_prior->setVertex(4, bias_gi);
  edge_prior->setInformation(prior_info_);
  optimizer.addEdge(edge_prior);

  EdgeBiasA *edge_ba = new EdgeBiasA();
  edge_ba->setId(eid++);
  edge_ba->setVertex(0, bias_ai);
  edge_ba->setVertex(1, bias_aj);
  edge_ba->setInformation(bias_ainfo_);
  optimizer.addEdge(edge_ba);

  EdgeBiasG *edge_bg = new EdgeBiasG();
  edge_bg->setId(eid++);
  edge_bg->setVertex(0, bias_gi);
  edge_bg->setVertex(1, bias_gj);
  edge_bg->setInformation(bias_ginfo_);
  optimizer.addEdge(edge_bg);

  // todo: add threshold on time diff between gnss and odom
  EdgeOdom *edge_odom = nullptr;
  if (!last_odom_used) {
    edge_odom = new EdgeOdom();
    edge_odom->setId(eid++);
    edge_odom->setMeasurement(curr_kf_.state.rot *
                              Eigen::Vector3d(last_odom_.velocity(), 0, 0));
    edge_odom->setVertex(0, vel_j);
    edge_odom->setInformation(odom_info_);
    optimizer.addEdge(edge_odom);
    last_odom_used = true;
    // vel_j->setEstimate(edge_odom->measurement());
    // LOG(INFO) << "odom j: " << edge_odom->measurement().transpose();
  }

  if (prev_kf_.gnss.valid_heading()) {
    EdgeGNSS *edge_gnss_i = new EdgeGNSS();
    edge_gnss_i->setId(eid++);
    edge_gnss_i->setMeasurement(prev_kf_.gnss.body_pose(origin_));
    edge_gnss_i->setVertex(0, rot_i);
    edge_gnss_i->setVertex(1, pos_i);
    edge_gnss_i->setInformation(gnss_info_);
    optimizer.addEdge(edge_gnss_i);
    // LOG(INFO) << "gnss i: "
    //           << edge_gnss_i->measurement().translation().transpose()
    //           << ", rot: "
    //           << edge_gnss_i->measurement().so3().unit_quaternion();
  } else {
    EdgeGNSSPos *edge_gnss_pos_i = new EdgeGNSSPos();
    edge_gnss_pos_i->setId(eid++);
    edge_gnss_pos_i->setMeasurement(
        prev_kf_.gnss.body_pose(origin_).translation());
    edge_gnss_pos_i->setVertex(0, pos_i);
    edge_gnss_pos_i->setInformation(gnss_pos_info_);
    optimizer.addEdge(edge_gnss_pos_i);
    // LOG(INFO) << "gnss pos i: " <<
    // edge_gnss_pos_i->measurement().transpose();
  }

  if (curr_kf_.gnss.valid_heading()) {
    EdgeGNSS *edge_gnss_j = new EdgeGNSS();
    edge_gnss_j->setId(eid++);
    edge_gnss_j->setMeasurement(curr_kf_.gnss.body_pose(origin_));
    edge_gnss_j->setVertex(0, rot_j);
    edge_gnss_j->setVertex(1, pos_j);
    edge_gnss_j->setInformation(gnss_info_);
    optimizer.addEdge(edge_gnss_j);
    // LOG(INFO) << "gnss j: "
    //           << edge_gnss_j->measurement().translation().transpose()
    //           << ", rot: "
    //           << edge_gnss_j->measurement().so3().unit_quaternion();
  } else {
    EdgeGNSSPos *edge_gnss_pos_j = new EdgeGNSSPos();
    edge_gnss_pos_j->setId(eid++);
    edge_gnss_pos_j->setMeasurement(
        prev_kf_.gnss.body_pose(origin_).translation());
    edge_gnss_pos_j->setVertex(0, pos_j);
    edge_gnss_pos_j->setInformation(gnss_pos_info_);
    optimizer.addEdge(edge_gnss_pos_j);
    // LOG(INFO) << "gnss pos j: " <<
    // edge_gnss_pos_j->measurement().transpose();
  }

  // LOG(INFO) << "Before optimization: ";
  // LOG(INFO) << "rot i: " << rot_i->estimate().unit_quaternion();
  // LOG(INFO) << "pos i: " << pos_i->estimate().transpose();
  // LOG(INFO) << "vel i: " << vel_i->estimate().transpose();
  // LOG(INFO) << "bias a i: " << bias_ai->estimate().transpose();
  // LOG(INFO) << "bias g i: " << bias_gi->estimate().transpose();

  // LOG(INFO) << "rot j: " << rot_j->estimate().unit_quaternion();
  // LOG(INFO) << "pos j: " << pos_j->estimate().transpose();
  // LOG(INFO) << "vel j: " << vel_j->estimate().transpose();
  // LOG(INFO) << "bias a j: " << bias_aj->estimate().transpose();
  // LOG(INFO) << "bias g j: " << bias_gj->estimate().transpose();

  optimizer.initializeOptimization();
  optimizer.optimize(20);

  prev_kf_.state.rot = rot_i->estimate();
  prev_kf_.state.vel = vel_i->estimate();
  prev_kf_.state.pos = pos_i->estimate();
  prev_kf_.state.bias_a = bias_ai->estimate();
  prev_kf_.state.bias_g = bias_gi->estimate();

  curr_kf_.state.rot = rot_j->estimate();
  curr_kf_.state.vel = vel_j->estimate();
  curr_kf_.state.pos = pos_j->estimate();
  curr_kf_.state.bias_a = bias_aj->estimate();
  curr_kf_.state.bias_g = bias_gj->estimate();

  // LOG(INFO) << "Optimization report:";
  // LOG(INFO) << "preintegration error: " << edge_preint->error().transpose();
  // LOG(INFO) << "prior error: " << edge_prior->error().transpose();
  // if (edge_odom) {
  //   LOG(INFO) << "odom error: " << edge_odom->error().transpose();
  // }
  // LOG(INFO) << "Bais acc error: " << edge_ba->error().transpose();
  // LOG(INFO) << "preintegration error: " << edge_preint->error().transpose();

  // LOG(INFO) << "After optimization: ";
  // LOG(INFO) << "rot i: " << rot_i->estimate().unit_quaternion();
  // LOG(INFO) << "pos i: " << pos_i->estimate().transpose();
  // LOG(INFO) << "vel i: " << vel_i->estimate().transpose();
  // LOG(INFO) << "bias a i: " << bias_ai->estimate().transpose();
  // LOG(INFO) << "bias g i: " << bias_gi->estimate().transpose();

  // LOG(INFO) << "rot j: " << rot_j->estimate().unit_quaternion();
  // LOG(INFO) << "pos j: " << pos_j->estimate().transpose();
  // LOG(INFO) << "vel j: " << vel_j->estimate().transpose();
  // LOG(INFO) << "bias a j: " << bias_aj->estimate().transpose();
  // LOG(INFO) << "bias g j: " << bias_gj->estimate().transpose();
  return true;
}
