#include "ekf/ekf.h"


namespace ekf
{


vehicle::State EKF::getVehicleState() const
{
  // Copy state but then substitute angular rate and linear acceleration
  // for the gyro bias and accel bias positions
  vehicle::State x;
  x.p = x_.t.p();
  x.v = x_.v;
  x.lin_accel = imu_.segment<3>(UAX);
  x.q = x_.t.q();
  x.omega = imu_.segment<3>(UGX);
  return x;
}


EKF::EKF()
{
  pts_k_.reserve(10000);
  pts_match_.reserve(10000);
  pts_match_k_.reserve(10000);
  dv_.reserve(10000);
  dv_k_.reserve(10000);
}


EKF::EKF(string filename)
{
  load(filename);
  pts_k_.reserve(10000);
  pts_match_.reserve(10000);
  pts_match_k_.reserve(10000);
  dv_.reserve(10000);
  dv_k_.reserve(10000);
}


EKF::~EKF()
{
  state_log_.close();
  cov_log_.close();
  ekf_global_pos_euler_log_.close();
  true_global_euler_log_.close();

}


void EKF::load(const string &filename)
{
  // Random seeding
  bool use_random_seed;
  common::get_yaml_node("use_random_seed", filename, use_random_seed);
  if (use_random_seed)
    srand(chrono::system_clock::now().time_since_epoch().count());

  // EKF initializations
  xVector x0;
  dxVector P0_diag, Qx_diag;
  Vector4d q_bc, q_bu;
  Matrix<double, NUM_INPUTS, 1> Qu_diag;
  Matrix<double, 5, 1> R_vo_diag;
  common::get_yaml_eigen("xhat0", filename, x0);
  x_ = State<double>(x0);
  common::get_yaml_eigen("P0_diag", filename, P0_diag);
  P_ = P0_diag.asDiagonal();
  common::get_yaml_eigen("Qx_diag", filename, Qx_diag);
  Qx_ = Qx_diag.asDiagonal();
  common::get_yaml_eigen("Qu_diag", filename, Qu_diag);
  Qu_ = Qu_diag.asDiagonal();
  common::get_yaml_eigen("R_vo_diag", filename, R_vo_diag);
  R_vo_ = R_vo_diag.asDiagonal();
  common::get_yaml_eigen("lambda", filename, lambda_);
  Lambda_ = ones_vec_ * lambda_.transpose() + lambda_ * ones_vec_.transpose() - lambda_ * lambda_.transpose();
  common::get_yaml_node("vo_meas_gate_upper", filename, vo_meas_gate_upper_);
  common::get_yaml_node("vo_meas_gate_lower", filename, vo_meas_gate_lower_);

  // Camera information
  common::get_yaml_eigen("camera_matrix", filename, K_);
  K_inv_ = K_.inverse();
  common::get_yaml_eigen("q_bc", filename, q_bc);
  common::get_yaml_eigen("q_bu", filename, q_bu);
  q_bc_ = common::Quaterniond(q_bc);
  q_bc_.normalize();
  q_bu_ = common::Quaterniond(q_bu);
  q_bu_.normalize();
  common::get_yaml_eigen("p_bc", filename, p_bc_);
  common::get_yaml_eigen("p_bu", filename, p_bu_);

  // Keyframe and update
  common::get_yaml_node("pixel_disparity_threshold", filename, pixel_disparity_threshold_);
  common::get_yaml_node("max_tracked_features", filename, max_tracked_features_);
  tracked_pts_.reserve(max_tracked_features_);
  new_tracked_pts_.reserve(max_tracked_features_);
  common::get_yaml_node("min_kf_feature_matches", filename, min_kf_feature_matches_);

  // Logging
  common::get_yaml_node("log_directory", filename, directory_);
  state_log_.open(directory_ + "/ekf_state.bin");
  cov_log_.open(directory_ + "/ekf_cov.bin");
  ekf_global_pos_euler_log_.open(directory_ + "/ekf_global_pos_euler.bin");
  true_global_euler_log_.open(directory_ + "/true_global_euler.bin");

  // Log b2u pose
  true_pose_b2u_log_.open(directory_ + "/true_pose_b2u.bin");
  true_pose_b2u_log_.write((char*)p_bu_.data(), 3 * sizeof(double));
  true_pose_b2u_log_.write((char*)q_bu_.data(), 4 * sizeof(double));
  true_pose_b2u_log_.close();

  // Initial IMU
  imu_.setZero();

  // Init global pose estimate
  global_node_position_ = x_.t.p();
  global_node_heading_ = x_.t.q().yaw();
  q_global_heading_ = common::Quaterniond(0,0,global_node_heading_);
}


void EKF::log(const double &t)
{
  Vector3d pg = global_node_position_ + q_global_heading_.inv().rot(x_.t.p());
  double er = x_.t.q().roll();
  double ep = x_.t.q().pitch();
  double hg = global_node_heading_ + x_.t.q().yaw();
  double tr = x_true_.q.roll();
  double tp = x_true_.q.pitch();
  double ty = x_true_.q.yaw();
  Matrix<double, NUM_STATES, 1> x = x_.toEigen();
  Matrix<double, NUM_DOF, 1> P_diag = P_.diagonal();

  state_log_.write((char*)&t, sizeof(double));
  state_log_.write((char*)x.data(), x.rows() * sizeof(double));

  cov_log_.write((char*)&t, sizeof(double));
  cov_log_.write((char*)P_diag.data(), P_diag.rows() * sizeof(double));

  ekf_global_pos_euler_log_.write((char*)&t, sizeof(double));
  ekf_global_pos_euler_log_.write((char*)pg.data(), 3 * sizeof(double));
  ekf_global_pos_euler_log_.write((char*)&er, sizeof(double));
  ekf_global_pos_euler_log_.write((char*)&ep, sizeof(double));
  ekf_global_pos_euler_log_.write((char*)&hg, sizeof(double));

  true_global_euler_log_.write((char*)&t, sizeof(double));
  true_global_euler_log_.write((char*)&tr, sizeof(double));
  true_global_euler_log_.write((char*)&tp, sizeof(double));
  true_global_euler_log_.write((char*)&ty, sizeof(double));

}


void EKF::run(const double &t, const sensors::Sensors &sensors, const vehicle::State &x_true)
{
//  cout << "t = " << t << endl;
  // Store truth
  x_true_ = x_true;

  // Log data
  log(t);

  // Apply updates then predict
  if (t > 0 && sensors.new_camera_meas_)
  {
//    cout << "** New Image **" << endl;
    // Track features by mimicking KLT Optical Flow then apply update
    if (trackFeatures(sensors.cam_))
      imageUpdate();
  }

  // Propagate the state and covariance to the next time step
  if (sensors.new_imu_meas_)
    propagate(t, sensors.gyro_, sensors.accel_);
}


void EKF::propagate(const double &t, const Vector3d &gyro, const Vector3d &acc)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Center the IMU measurement on the body
  Vector3d gyrob = q_bu_.inv().rot(gyro);
  Vector3d accb = q_bu_.inv().rot(acc - gyro.cross(gyro.cross(q_bu_.rot(p_bu_))));

  // Store unbiased IMU for control
  imu_.segment<3>(UAX) = accb - x_.ba;
  imu_.segment<3>(UGX) = gyrob - x_.bg;

  // Propagate the state
  dynamics<double>(x_, imu_, xdot_);
  x_ += xdot_ * dt;


  // Propagate the covariance - guarantee positive-definite P with discrete propagation
  getFG(x_.toEigen(), imu_, F_, G_);
  A_ = I_num_dof_ + F_*dt + F_*F_*dt*dt/2.0 + F_*F_*F_*dt*dt*dt/6.0; // Approximate state transition matrix
  B_ = (dt*(I_num_dof_ + F_*dt/2.0 + F_*F_*dt*dt/6.0 + F_*F_*F_*dt*dt*dt/24.0))*G_;
  P_ = A_ * P_ * A_.transpose() + B_ * Qu_ * B_.transpose() + Qx_;
}


void EKF::imageUpdate()
{
  // Remove camera rotation and compute mean image point disparity
  dv_.clear();
  dv_k_.clear();
//  double mean_disparity = 0;
//  common::Quaterniond q_c2kc = q_bc_.inv() * x_.t.q().inv() * x_.tk.q() * q_bc_;
  for (int n = 0; n < pts_match_.size(); ++n)
  {
//    // Remove rotation from current image points and compute the average disparity recursively
//    Vector3d pix_derotated = K_ * q_c2kc.inv().rot(K_inv_ * Vector3d{pts_match_[n](0), pts_match_[n](1), 1.0});
//    pix_derotated /= pix_derotated(2);
//    mean_disparity = (n * mean_disparity + (pix_derotated.topRows(2) - pts_match_k_[n]).norm()) / (n + 1);

    // Convert point matches to direction vectors
    Vector3d dv = K_inv_ * Vector3d{pts_match_[n](0), pts_match_[n](1), 1.0};
    Vector3d dv_k = K_inv_ * Vector3d{pts_match_k_[n](0), pts_match_k_[n](1), 1.0};
    dv_.push_back(dv.normalized());
    dv_k_.push_back(dv_k.normalized());
  }

  // Estimate camera pose relative to keyframe via nonlinear optimization and apply update
//  if (mean_disparity > pixel_disparity_threshold_)
  if (x_.t.p().norm() > 0.25)
  {
//    cout << "* State Update *" << endl;
    // Compute relative camera pose initial guess from state then optimize it
    common::Quaterniond zt, zq, ht, hq;
    rel_pose_model(x_, q_bc_, p_bc_, ht, hq);
    zt = ht;
    zq = hq;
    optimizePose(zq, zt, dv_k_, dv_, 30);

    // Measurement model and jacobian
    Matrix<double, 5, NUM_DOF> H;
    getH(x_.toEigen(), ht, hq, q_bc_, p_bc_, H);

//    Vector3d t_dir = q_bc_.rot(x_true_.q.rot(pk_true_ - x_true_.p) +
//                     (x_true_.q.R() * qk_true_.inv().R() - common::I_3x3) * p_bc_).normalized();
//    common::Quaterniond zt_ = common::Quaterniond(t_dir);
//    common::Quaterniond zq_ = q_bc_.inv() * x_true_.q.inv() * qk_true_ * q_bc_;
//    cout << "t_meas: " << zt.uvec().transpose() << endl;
//    cout << "t_true: " << zt_.uvec().transpose() << endl;
//    cout << "q_meas: " << zq.toEigen().transpose() << endl;
//    cout << "q_true: " << zq_.toEigen().transpose() << endl;

    // Error in translation direction and rotation
    Vector2d err_t = common::Quaterniond::log_uvec(zt,ht);
    Vector3d err_q = zq - hq;
//    std::cout << "err_t: " << err_t.transpose() << ", err_q: " << err_q.transpose() << std::endl;

    // Innovation error
    Matrix<double, 5, 1> err_i;
    err_i << err_t, err_q;

    // Measurement gating
    Matrix<double, 5, 5> S_inv = (R_vo_ + H * P_ * H.transpose()).inverse();
    double NEES = err_i.transpose() * S_inv * err_i;
//    std::cout << NEES << std::endl;
//    if (NEES < vo_meas_gate_lower_ || NEES > vo_meas_gate_upper_)
//      return;

    // Change in state and covariance
    Matrix<double, NUM_DOF, 5> K = P_ * H.transpose() * S_inv;
    dxVector delta_x = lambda_.cwiseProduct(K * err_i);
    dxMatrix delta_P = Lambda_.cwiseProduct((I_num_dof_ - K * H) * P_ * (I_num_dof_ - K * H).transpose() +
                       K * R_vo_ * K.transpose() - P_);

    // Apply update
    x_ += delta_x;
    P_ += delta_P;
  }
}



bool EKF::trackFeatures(const vector<Vector3d, aligned_allocator<Vector3d> > &pts)
{
  // Match current image points to image points tracked in the previous image
  new_tracked_pts_.clear();
  for (int i = 0; i < tracked_pts_.size(); ++i)
  {
    for (int j = 0; j < pts.size(); ++j)
    {
      if (tracked_pts_[i](2) == pts[j](2))
        new_tracked_pts_.push_back(pts[j]);
    }
  }
  tracked_pts_ = new_tracked_pts_;

  // Match current image points to keyframe image points
  pts_match_.clear();
  pts_match_k_.clear();
  for (int i = 0; i < pts_k_.size(); ++i)
  {
    for (int j = 0; j < tracked_pts_.size(); ++j)
    {
      if (pts_k_[i](2) == tracked_pts_[j](2))
      {
        pts_match_.push_back(tracked_pts_[j].topRows(2));
        pts_match_k_.push_back(pts_k_[i].topRows(2));
      }
    }
  }

  // Create new keyframe and add new feature points to tracked points when
  // number of keyframe point matches drops below threshold
  if (pts_match_.size() < min_kf_feature_matches_)
  {
    // Randomly select and add new points to the tracked points container
    static bool pt_used;
    int count = 0;
    while (tracked_pts_.size() < max_tracked_features_ && pts.size() > 0)
    {
      // Get random index and make sure point isn't already tracked
      pt_used = false;
      int idx = rand() % pts.size();
      for (auto const &tracked_pt : tracked_pts_)
      {
        if (tracked_pt(2) == pts[idx](2))
          pt_used = true;
      }

      if (!pt_used)
        tracked_pts_.push_back(pts[idx]);

      if (count > 2000) break;
      ++count;
    }

    // Save keyframe image points and reset the keyframe
    global_node_position_ += q_global_heading_.inv().rot(x_.t.p());
    global_node_heading_ += x_.t.q().yaw();
    q_global_heading_ = common::Quaterniond(0,0,global_node_heading_);
    keyframeReset(x_, P_);
    pts_k_ = tracked_pts_;
    pk_true_ = x_true_.p;
    qk_true_ = x_true_.q;
    return false;
  }
  else
    return true;
}


void EKF::keyframeReset(State<double> &x, dxMatrix& P)
{
//  cout << "\n\n** Keyframe Reset **\n\n" << endl;
  // State reset
  state_reset_model(x);

  // Covariance reset
  dxMatrix N;
  getN(x.toEigen(), N);
//  cout << "\nN=\n" << N << "\n\n";
  P = N * P * N.transpose();
}


void getN_analytical(State<double> &x, dxMatrix &N)
{
  double sp = sin(x.t.q().roll());
  double cp = cos(x.t.q().roll());
  double st = sin(x.t.q().pitch());
  double ct = cos(x.t.q().pitch());
  double tt = st / ct;
  N.setIdentity();
  N.topLeftCorner(3,3).setZero();
  N.bottomRightCorner(6,6).setZero();
  Matrix3d N_theta;
  N_theta << 1.0,  sp * tt,  cp * tt,
             0.0,  cp * cp, -cp * sp,
             0.0, -cp * sp,  sp * sp;
  N.block<3,3>(ekf::DQX,ekf::DQX) = N_theta;
  N.block<3,3>(ekf::DKQX,ekf::DQX) = N_theta;
}


// Relative pose optimizer using the Ceres Solver
void EKF::optimizePose(common::Quaterniond& q, common::Quaterniond& qt,
                       const vector<Vector3d,aligned_allocator<Vector3d>>& e1,
                       const vector<Vector3d,aligned_allocator<Vector3d>>& e2,
                       const unsigned &iters)
{
  // Ensure number of directions vectors from each camera match
  if (e1.size() != e2.size())
  {
    std::cout << "\nError in optimizePose. Direction vector arrays must be the same size.\n\n";
    return;
  }

  // Build optimization problem with Ceres-Solver
  ceres::Problem problem;

  // Does passing a dynamic rvalue result in a memory leak?
  problem.AddParameterBlock(q.data(), 4, new ceres::AutoDiffLocalParameterization<EKF::S3Plus,4,3>);
  problem.AddParameterBlock(qt.data(), 4, new ceres::AutoDiffLocalParameterization<EKF::S2Plus,4,2>);

  for (int i = 0; i < e1.size(); ++i)
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<EKF::SampsonError, 1, 4, 4>
    (new EKF::SampsonError(e1[i], e2[i])), NULL, q.data(), qt.data());

  // Solve for the optimal rotation and translation direciton
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = iters;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
//  std::cout << summary.FullReport() << "\n\n";
}



}
