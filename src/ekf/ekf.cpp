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
  q_bc_ = common::Quaternion<double>(q_bc);
  q_bc_.normalize();
  q_bu_ = common::Quaternion<double>(q_bu);
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

  // Initial IMU
  imu_.setZero();

  q_time_avg_ = 0;
  R_time_avg_ = 0;
  ceres_time_avg_ = 0;
  nn_ = 0;
}


void EKF::log(const double &t)
{
  Matrix<double, NUM_STATES, 1> x = x_.toEigen();
  Matrix<double, NUM_DOF, 1> P_diag = P_.diagonal();
  state_log_.write((char*)&t, sizeof(double));
  state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  cov_log_.write((char*)&t, sizeof(double));
  cov_log_.write((char*)P_diag.data(), P_diag.rows() * sizeof(double));
}


void EKF::run(const double &t, const sensors::Sensors &sensors)
{
  // Log data
  log(t);

//  // Apply updates then predict
//  if (t > 0 && sensors.new_camera_meas_)
//  {
//    // Track features by mimicking KLT Optical Flow then apply update
//    if (trackFeatures(sensors.cam_))
//      imageUpdate();
//  }

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
  getFG<double>(x_.toEigen(), imu_, F_, G_);
  A_ = I_num_dof_ + F_*dt + F_*F_*dt*dt/2.0 + F_*F_*F_*dt*dt*dt/6.0; // Approximate state transition matrix
  B_ = (dt*(I_num_dof_ + F_*dt/2.0 + F_*F_*dt*dt/6.0 + F_*F_*F_*dt*dt*dt/24.0))*G_;
  P_ = A_ * P_ * A_.transpose() + B_ * Qu_ * B_.transpose() + Qx_;
}


//void EKF::imageUpdate()
//{
//  // Remove camera rotation and compute mean image point disparity
//  dv_.clear();
//  dv_k_.clear();
//  double mean_disparity = 0;
//  common::Quaternion<double> q_c2ck = q_bc_.inv() * x_.q.inv() * qk_ * q_bc_;
//  for (int n = 0; n < pts_match_.size(); ++n)
//  {
//    // Image points without rotation
//    Vector3d pt1 = K_ * q_c2ck.rot(K_inv_ * Vector3d(pts_match_[n](0), pts_match_[n](1), 1.0));
//    Vector3d pt2 = pt1 / pt1(2);

//    // Recursive mean of disparity
//    mean_disparity = (n * mean_disparity + (pt2.topRows(2) - pts_match_k_[n]).norm()) / (n + 1);

//    // Convert point matches to direction vectors
//    Vector3d dv = K_inv_ * Vector3d(pts_match_[n](0), pts_match_[n](1), 1.0);
//    Vector3d dv_k = K_inv_ * Vector3d(pts_match_k_[n](0), pts_match_k_[n](1), 1.0);
//    dv_.push_back(dv / dv.norm());
//    dv_k_.push_back(dv_k / dv_k.norm());
//  }

//  // Estimate camera pose relative to keyframe via nonlinear optimization and apply update
//  if (mean_disparity > pixel_disparity_threshold_)
//  {
//    // Compute relative camera pose initial guess from state then optimize it
//    Vector3d pt = q_bc_.rot(x_.q.rot(pk_ + qk_.inv().rot(p_bc_) - x_.p) - p_bc_);
//    Vector3d t = pt / pt.norm();
//    common::Quaternion<double> zt(t);
//    common::Quaternion<double> zq = q_c2ck;
//    Matrix3d R, Rt, R2, Rt2;
//    R = zq.R();
//    Rt = zt.R();
//    R2 = R;
//    Rt2 = Rt;
//    auto t0 = chrono::system_clock::now();
//    optimizePose(zq, zt, dv_k_, dv_, 10);
//    auto t1 = chrono::system_clock::now();
//    optimizePose2(R, Rt, dv_k_, dv_, 10);
//    auto t2 = chrono::system_clock::now();
//    auto q_time = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
//    auto R_time = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
//    q_time_avg_ = (nn_ * q_time_avg_ + q_time) / (nn_ + 1);
//    R_time_avg_ = (nn_ * R_time_avg_ + R_time) / (nn_ + 1);
//    ++nn_;
////    cout << tracked_pts_.size() << ", " << q_time_avg_ << ", " << R_time_avg_ << ", "
////              << (zq.rot(common::e1) - R * common::e1).norm() << ", "
////              << (zt.rot(common::e1) - Rt * common::e1).norm() << ", "
////              << (zq.rot(common::e1) - R2 * common::e1).norm() << ", "
////              << (zt.rot(common::e1) - Rt2 * common::e1).norm() << endl;

//    // Measurement model and jacobian
//    common::Quaternion<double> ht, hq;
//    Matrix<double, 5, NUM_DOF> H;
//    imageH(ht, hq, H, x_, q_bc_, p_bc_, qk_, pk_);

//    // Error in translation direction and rotation
//    Vector2d err_t = common::Quaternion<double>::log_uvec(zt,ht);
//    Vector3d err_q = zq - hq;
////    cout << "err_t: " << err_t.transpose() << ", err_q: " << err_q.transpose() << endl;

//    // Innovation error
//    Matrix<double, 5, 1> err_i;
//    err_i << err_t, err_q;

//    // Measurement gating
//    Matrix<double, 5, 5> S_inv = (R_vo_ + H * P_ * H.transpose()).inverse();
//    double NEES = err_i.transpose() * S_inv * err_i;
////    cout << NEES << endl;
////    if (NEES < vo_meas_gate_lower_ || NEES > vo_meas_gate_upper_)
////      return;

//    // Change in state and covariance
//    Matrix<double, NUM_DOF, 5> K = P_ * H.transpose() * S_inv;
//    dxVector delta_x = lambda_.cwiseProduct(K * err_i);
//    dxMatrix delta_P = Lambda_.cwiseProduct((I_num_dof_ - K * H) * P_ * (I_num_dof_ - K * H).transpose() +
//                       K * R_vo_ * K.transpose() - P_);

//    // Apply update
//    x_ += delta_x;
//    P_ += delta_P;
//  }
//}


//bool EKF::trackFeatures(const vector<Vector3d, aligned_allocator<Vector3d> > &pts)
//{
//  // Match current image points to image points tracked in the previous image
//  new_tracked_pts_.clear();
//  for (int i = 0; i < tracked_pts_.size(); ++i)
//  {
//    for (int j = 0; j < pts.size(); ++j)
//    {
//      if (tracked_pts_[i](2) == pts[j](2))
//        new_tracked_pts_.push_back(pts[j]);
//    }
//  }
//  tracked_pts_ = new_tracked_pts_;

//  // Match current image points to keyframe image points
//  pts_match_.clear();
//  pts_match_k_.clear();
//  for (int i = 0; i < pts_k_.size(); ++i)
//  {
//    for (int j = 0; j < tracked_pts_.size(); ++j)
//    {
//      if (pts_k_[i](2) == tracked_pts_[j](2))
//      {
//        pts_match_.push_back(tracked_pts_[j].topRows(2));
//        pts_match_k_.push_back(pts_k_[i].topRows(2));
//      }
//    }
//  }

//  // Create new keyframe and add new feature points to tracked points when
//  // number of keyframe point matches drops below threshold
//  if (pts_match_.size() < min_kf_feature_matches_)
//  {
//    // Randomly select and add new points to the tracked points container
//    static bool pt_used;
//    int count = 0;
//    while (tracked_pts_.size() < max_tracked_features_ && pts.size() > 0)
//    {
//      // Get random index and make sure point isn't already tracked
//      pt_used = false;
//      int idx = rand() % pts.size();
//      for (auto const &tracked_pt : tracked_pts_)
//      {
//        if (tracked_pt(2) == pts[idx](2))
//          pt_used = true;
//      }

//      if (!pt_used)
//        tracked_pts_.push_back(pts[idx]);

//      if (count > 2000) break;
//      ++count;
//    }

//    // Establish new keyframe
//    pk_ = x_.p;
//    qk_ = x_.q;
//    pts_k_ = tracked_pts_;
//    return false;
//  }
//  else
//    return true;
//}


//void EKF::dynamics(const State<double> &x, const uVector &imu, dxVector &xdot)
//{
//  xdot.segment<3>(DPX) = x.t.q().inv().rot(x.v);
//  xdot.segment<3>(DQX) = imu.segment<3>(UGX) - x.bg;
//  xdot.segment<3>(DVX) = imu.segment<3>(UAX) - x.ba + common::gravity * x.t.q().rot(common::e3) - (imu.segment<3>(UGX) - x.bg).cross(x.v);
//  xdot.segment<3>(DGX).setZero();
//  xdot.segment<3>(DAX).setZero();
//}


//void EKF::dynamics(const State<double> &x, const uVector &imu, const uVector &noise, dxVector &xdot)
//{
//  xdot.segment<3>(DPX) = x.t.q().inv().rot(x.v);
//  xdot.segment<3>(DQX) = imu.segment<3>(UGX) - x.bg - noise.segment<3>(UGX);
//  xdot.segment<3>(DVX) = imu.segment<3>(UAX) - x.ba - noise.segment<3>(UAX) + common::gravity * x.t.q().rot(common::e3) -
//                         (imu.segment<3>(UGX) - x.bg - noise.segment<3>(UGX)).cross(x.v);
//  xdot.segment<3>(DGX).setZero();
//  xdot.segment<3>(DAX).setZero();
//}


//void EKF::getF(dxMatrix &F, const State &x, const Vector3d &gyro)
//{
//  F.setZero();
//  F.block<3,3>(DPX,DQX) = -x.q.R().transpose() * common::skew(x.v);
//  F.block<3,3>(DPX,DVX) = x.q.R().transpose();
//  F.block<3,3>(DQX,DGX) = -common::I_3x3;
//  F.block<3,3>(DVX,DQX) = common::gravity * common::skew(x.q.rot(common::e3));
//  F.block<3,3>(DVX,DVX) = -common::skew(Vector3d(gyro - x.bg));
//  F.block<3,3>(DVX,DGX) = -common::skew(x.v);
//  F.block<3,3>(DVX,DAX) = -common::I_3x3;
//}


//void EKF::getG(Matrix<double, NUM_DOF, NUM_INPUTS> &G, const State &x)
//{
//  G.setZero();
//  G.block<3,3>(DQX,UAX) = -common::I_3x3;
//  G.block<3,3>(DVX,UAX) = -common::skew(x.v);
//  G.block<3,3>(DVX,UGX) = -common::I_3x3;
//}


//void EKF::imageH(common::Quaternion<double> &ht, common::Quaternion<double> &hq, Matrix<double, 5, NUM_DOF> &H, const State &x,
//                 const common::Quaternion<double> &q_bc, const Vector3d &p_bc, const common::Quaternion<double> &q_ik,
//                 const Vector3d &p_ik)
//{
//  // Declarations
//  static Vector3d pt, t, t_x_k, at;
//  static Matrix3d Gamma_at, dat_dt, dt_dpt, dpt_dp, dpt_dq;
//  static Matrix<double, 2, 3> dexpat_dat;

//  pt = q_bc.rot(x.q.rot(p_ik + q_ik.inv().rot(p_bc) - x.p) - p_bc);
//  t = pt / pt.norm();
//  t_x_k = t.cross(common::e3);
//  double tT_k = t.transpose() * common::e3;
//  at = acos(tT_k) * t_x_k / t_x_k.norm();

//  ht = common::Quaternion<double>::exp(at);
//  hq = q_bc.inv() * x.q.inv() * q_ik * q_bc;

//  Gamma_at = common::Quaternion<double>::dexp(at);
//  double txk_mag = t_x_k.norm();
//  dexpat_dat = common::I_2x3 * ht.R().transpose() * Gamma_at;
//  dat_dt = acos(tT_k) / txk_mag * ((t_x_k * t_x_k.transpose()) /
//                           (txk_mag * txk_mag) - common::I_3x3) * common::skew(common::e3) -
//                           (t_x_k * common::e3.transpose()) / (txk_mag * sqrt(1.0 - tT_k * tT_k));
//  double ptmag = pt.norm();
//  dt_dpt = (1.0 / ptmag) * (common::I_3x3 - pt * pt.transpose() / (ptmag * ptmag));
//  dpt_dp = -q_bc.R() * x.q.R();
//  dpt_dq = q_bc.R() * common::skew(x.q.rot(p_ik + q_ik.inv().rot(p_bc) - x.p));

//  H.setZero();
//  H.block<2,3>(0,DPX) = dexpat_dat * dat_dt * dt_dpt * dpt_dp;
//  H.block<2,3>(0,DQX) = dexpat_dat * dat_dt * dt_dpt * dpt_dq;
//  H.block<3,3>(2,DQX) = -q_bc.R() * q_ik.R() * x.q.R().transpose();
//}


//// Relative camera pose optimizer
//void EKF::optimizePose(common::Quaternion<double>& q, common::Quaternion<double>& qt,
//                       const vector<Vector3d, aligned_allocator<Vector3d> >& e1,
//                       const vector<Vector3d, aligned_allocator<Vector3d> >& e2,
//                       const unsigned &iters)
//{
//  // Ensure number of directions vectors from each camera match
//  if (e1.size() != e2.size())
//  {
//    cout << "\nError in optimizePose. Direction vector arrays must be the same size.\n\n";
//    return;
//  }

//  // Constants
//  const int N = e1.size();
//  static const Matrix3d e1x = common::skew(common::e1);
//  static const Matrix3d e2x = common::skew(common::e2);
//  static const Matrix3d e3x = common::skew(common::e3);

//  // Declare things that change each loop
//  static Matrix3d R, Rtx, e1x_Rtx, e2x_Rtx, e3x_Rtx, e1tx, e2tx, R_e1tx_tx, R_e2tx_tx;
//  static Vector3d t, e1tx_t, e2tx_t;
//  static Matrix<double,5,1> delta;
//  static Matrix<double, 1000, 1> r;
//  static Matrix<double, 1000, 5> J;

//  // Main optimization loop
//  for (int i = 0; i < iters; ++i)
//  {
//    // Pre-compute a few things
//    R = q.R();
//    t = qt.uvec();
//    Rtx = R * common::skew(t);
//    e1x_Rtx = e1x * Rtx;
//    e2x_Rtx = e2x * Rtx;
//    e3x_Rtx = e3x * Rtx;
//    e1tx = common::skew(qt.rot(common::e1));
//    e2tx = common::skew(qt.rot(common::e2));
//    e1tx_t = e1tx * t;
//    e2tx_t = e2tx * t;
//    R_e1tx_tx = R * common::skew(e1tx_t);
//    R_e2tx_tx = R * common::skew(e2tx_t);

//    // Vector of residual errors
//    for (int j = 0; j < N; ++j)
//      se(r(j), e1[j], e2[j], Rtx);

//    // Stop if error is small enough
//    if (r.topRows(N).sum() < 1e-6) break;

//    // Jacobian of residual errors
//    for (int j = 0; j < N; ++j)
//    {
//      dse(J(j,0), e1[j], e2[j], Rtx, -e1x_Rtx);
//      dse(J(j,1), e1[j], e2[j], Rtx, -e2x_Rtx);
//      dse(J(j,2), e1[j], e2[j], Rtx, -e3x_Rtx);
//      dse(J(j,3), e1[j], e2[j], Rtx, -R_e1tx_tx);
//      dse(J(j,4), e1[j], e2[j], Rtx, -R_e2tx_tx);
//    }

//    // Innovation
//    delta = -(J.topRows(N).transpose() * J.topRows(N)).inverse() * J.topRows(N).transpose() * r.topRows(N);

//    // Update camera rotation and translation
//    q += Vector3d(delta.segment<3>(0));
//    qt += Vector3d(qt.proj() * delta.segment<2>(3));
//  }
//}


//// Relative camera pose optimizer using rotation matrices
//void EKF::optimizePose2(Matrix3d& R, Matrix3d& Rt,
//                        const vector<Vector3d, aligned_allocator<Vector3d> >& e1,
//                        const vector<Vector3d, aligned_allocator<Vector3d> >& e2,
//                        const unsigned &iters)
//{
//  // Ensure number of directions vectors from each camera match
//  if (e1.size() != e2.size())
//  {
//    cout << "\nError in optimizePose. Direction vector arrays must be the same size.\n\n";
//    return;
//  }

//  // Constants
//  const int N = e1.size();
//  static const Matrix3d e1x = common::skew(common::e1);
//  static const Matrix3d e2x = common::skew(common::e2);
//  static const Matrix3d e3x = common::skew(common::e3);

//  // Declare things that change each loop
//  static Matrix3d Rtx, e1x_Rtx, e2x_Rtx, e3x_Rtx, e1tx, e2tx, R_e1tx_tx, R_e2tx_tx;
//  static Vector3d t, e1tx_t, e2tx_t, Rte1, Rte2, delta2, delta3;
//  static Matrix<double,5,1> delta;
//  static Matrix<double, 1000, 1> r;
//  static Matrix<double, 1000, 5> J;

//  // Main optimization loop
//  for (int i = 0; i < iters; ++i)
//  {
//    // Pre-compute a few things
//    t = Rt * common::e3;
//    Rtx = R * common::skew(t);
//    e1x_Rtx = e1x * Rtx;
//    e2x_Rtx = e2x * Rtx;
//    e3x_Rtx = e3x * Rtx;
//    Rte1 = Rt * common::e1;
//    Rte2 = Rt * common::e2;
//    e1tx = common::skew(Rte1);
//    e2tx = common::skew(Rte2);
//    e1tx_t = e1tx * t;
//    e2tx_t = e2tx * t;
//    R_e1tx_tx = R * common::skew(e1tx_t);
//    R_e2tx_tx = R * common::skew(e2tx_t);

//    // Vector of residual errors
//    for (int j = 0; j < N; ++j)
//      se(r(j), e1[j], e2[j], Rtx);

//    // Stop if error is small enough
//    if (r.topRows(N).sum() < 1e-1) break;

//    // Jacobian of residual errors
//    for (int j = 0; j < N; ++j)
//    {
//      dse(J(j,0), e1[j], e2[j], Rtx, -e1x_Rtx);
//      dse(J(j,1), e1[j], e2[j], Rtx, -e2x_Rtx);
//      dse(J(j,2), e1[j], e2[j], Rtx, -e3x_Rtx);
//      dse(J(j,3), e1[j], e2[j], Rtx, -R_e1tx_tx);
//      dse(J(j,4), e1[j], e2[j], Rtx, -R_e2tx_tx);
//    }

//    // Innovation
//    delta = -(J.topRows(N).transpose() * J.topRows(N)).inverse() * J.topRows(N).transpose() * r.topRows(N);

//    // Update camera rotation and translation
//    R = common::expR(common::skew(Vector3d(-delta.segment<3>(0)))) * R;
//    Rt = common::expR(common::skew(Vector3d(-Rt * common::I_2x3.transpose() * delta.segment<2>(3)))) * Rt;
//  }
//}


//// Sampson's error
//void EKF::se(double& err, const Vector3d& e1, const Vector3d& e2, const Matrix3d& E)
//{
//  static Vector3d e1T_E, E_e2;
//  double e1T_E_e2 = e1.transpose() * E * e2;
//  if (e1T_E_e2 < 1e-6)
//    err = 0;
//  else
//  {
//    e1T_E = (e1.transpose() * E).transpose();
//    E_e2 = E * e2;
//    err = e1T_E_e2 / sqrt(e1T_E(0) * e1T_E(0) + e1T_E(1) * e1T_E(1) + E_e2(0) * E_e2(0) + E_e2(1) * E_e2(1));
//  }
//}


}
