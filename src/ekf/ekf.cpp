#include "ekf/ekf.h"


namespace ekf
{


State::State()
{
  p.setZero();
  q = common::Quaterniond();
  v.setZero();
  bg.setZero();
  ba.setZero();
  pk.setZero();
  qk = common::Quaterniond();
}


State::State(const xVector &x)
{
  p = x.segment<3>(PX);
  q = common::Quaterniond(Vector4d(x.segment<4>(QW)));
  v = x.segment<3>(VX);
  bg = x.segment<3>(GX);
  ba = x.segment<3>(AX);
  pk = x.segment<3>(KPX);
  qk = common::Quaterniond(Vector4d(x.segment<4>(KQW)));
}


State State::operator+(const dxVector &delta) const
{
  State x;
  x.p = p + delta.segment<3>(DPX);
  x.q = q + Vector3d(delta.segment<3>(DQX));
  x.v = v + delta.segment<3>(DVX);
  x.bg = bg + delta.segment<3>(DGX);
  x.ba = ba + delta.segment<3>(DAX);
  x.pk = pk + delta.segment<3>(DKPX);
  x.qk = qk + Vector3d(delta.segment<3>(DKQX));
  return x;
}


dxVector State::operator-(const State &x2) const
{
  dxVector dx;
  dx.segment<3>(DPX) = p - x2.p;
  dx.segment<3>(DQX) = q - x2.q;
  dx.segment<3>(DVX) = v - x2.v;
  dx.segment<3>(DGX) = bg - x2.bg;
  dx.segment<3>(DAX) = ba - x2.ba;
  dx.segment<3>(DKPX) = pk - x2.pk;
  dx.segment<3>(DKQX) = qk - x2.qk;
  return dx;
}


void State::operator+=(const dxVector &delta)
{
  *this = *this + delta;
}


Matrix<double, NUM_STATES, 1> State::toEigen() const
{
  Matrix<double, NUM_STATES, 1> x;
  x << p, q.toEigen(), v, bg, ba, pk, qk.toEigen();
  return x;
}


Matrix<double, NUM_DOF, 1> State::minimal() const
{
  Matrix<double, NUM_DOF, 1> x;
  x << p, common::Quaterniond::log(q), v, bg, ba, pk, common::Quaterniond::log(qk);
  return x;
}


vehicle::State EKF::getVehicleState() const
{
  // Copy state but then substitute angular rate and linear acceleration
  // for the gyro bias and accel bias positions
  vehicle::State x;
  x.p = x_.p;
  x.v = x_.v;
  x.lin_accel = imu_.segment<3>(UAX);
  x.q = x_.q;
  x.omega = imu_.segment<3>(UWX);
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


EKF::EKF(std::string filename)
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


void EKF::load(const std::string &filename)
{
  // Random seeding
  bool use_random_seed;
  common::get_yaml_node("use_random_seed", filename, use_random_seed);
  if (use_random_seed)
    srand(std::chrono::system_clock::now().time_since_epoch().count());

  // EKF initializations
  xVector x0;
  dxVector P0_diag, Qx_diag;
  Vector4d q_bc, q_bu;
  Matrix<double, NUM_INPUTS, 1> Qu_diag;
  Matrix<double, 5, 1> R_vo_diag;
  common::get_yaml_eigen("xhat0", filename, x0);
  x_ = State(x0);
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
  global_node_position_ = x_.p;
  global_node_heading_ = x_.q.yaw();
  q_global_heading_ = common::Quaterniond(0,0,global_node_heading_);
}


void EKF::run(const double &t, const sensors::Sensors &sensors, const vehicle::State &x_true)
{
  // Store truth
  x_true_ = x_true;

  // Log data
  log(t);

  // Apply updates then predict
  if (t > 0 && sensors.new_camera_meas_)
  {
    // Track features by mimicking KLT Optical Flow then apply update
    if (trackFeatures(sensors.cam_))
      imageUpdate();
  }

  // Propagate the state and covariance to the next time step
  if (sensors.new_imu_meas_)
    propagate(t, sensors.gyro_, sensors.accel_);
}


void EKF::log(const double &t)
{
  Vector3d pg = global_node_position_ + q_global_heading_.inv().rot(x_.p);
  double er = x_.q.roll();
  double ep = x_.q.pitch();
  double hg = global_node_heading_ + x_.q.yaw();
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


void EKF::propagate(const double &t, const Vector3d &gyro, const Vector3d &acc)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Center the IMU measurement on the body
  Vector3d gyrob = q_bu_.inv().rot(gyro);
  Vector3d accb = q_bu_.inv().rot(acc - gyro.cross(gyro.cross(p_bu_)));

  // Store unbiased IMU for control
  imu_.segment<3>(UAX) = accb - x_.ba;
  imu_.segment<3>(UWX) = gyrob - x_.bg;

  // Propagate the state
  f(x_, gyrob, accb, xdot_);
  x_ += xdot_ * dt;

  // Propagate the covariance - guarantee positive-definite P with discrete propagation
  getFG(x_, gyro, F_, G_);
  A_ = I_num_dof_ + F_*dt + F_*F_*dt*dt/2.0 + F_*F_*F_*dt*dt*dt/6.0; // Approximate state transition matrix
  B_ = (dt*(I_num_dof_ + F_*dt/2.0 + F_*F_*dt*dt/6.0 + F_*F_*F_*dt*dt*dt/24.0))*G_;
  P_ = A_ * P_ * A_.transpose() + B_ * Qu_ * B_.transpose() + Qx_;
}


void EKF::imageUpdate()
{
  // Remove camera rotation and compute mean image point disparity
  dv_.clear();
  dv_k_.clear();
  double mean_disparity = 0;
  for (int n = 0; n < pts_match_.size(); ++n)
  {
    // Remove rotation from current image points and compute the average disparity recursively
    Vector3d pix_derotated = K_ * x_.q.inv().rot(K_inv_ * Vector3d(pts_match_[n](0), pts_match_[n](1), 1.0));
    pix_derotated /= pix_derotated(2);
    mean_disparity = (n * mean_disparity + (pix_derotated.topRows(2) - pts_match_k_[n]).norm()) / (n + 1);

    // Convert point matches to direction vectors
    Vector3d dv = K_inv_ * Vector3d(pts_match_[n](0), pts_match_[n](1), 1.0);
    Vector3d dv_k = K_inv_ * Vector3d(pts_match_k_[n](0), pts_match_k_[n](1), 1.0);
    dv_.push_back(dv / dv.norm());
    dv_k_.push_back(dv_k / dv_k.norm());
  }

  // Estimate camera pose relative to keyframe via nonlinear optimization and apply update
  if (mean_disparity > pixel_disparity_threshold_)
  {
    // Measurement model and jacobian
    common::Quaterniond ht, hq;
    Matrix<double, 5, NUM_DOF> H;
    getH(x_, q_bc_, p_bc_, ht, hq, H);

    // Compute relative camera pose initial guess from state then optimize it
//    common::Quaterniond zt(x_.p);
//    common::Quaterniond zq = x_.q;
//    optimizePose(zq, zt, dv_k_, dv_, 10);
    Vector3d t_dir = q_bc_.rot(x_true_.q.rot(pk_true_ - x_true_.p)).normalized();
    common::Quaterniond zt = common::Quaterniond(t_dir);
    common::Quaterniond zq = q_bc_.inv() * x_true_.q.inv() * qk_true_ * q_bc_;

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


bool EKF::trackFeatures(const std::vector<Vector3d, aligned_allocator<Vector3d> > &pts)
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
    global_node_position_ += q_global_heading_.inv().rot(x_.p);
    global_node_heading_ += x_.q.yaw();
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


void EKF::f(const State &x, const Vector3d &gyro, const Vector3d &acc, dxVector &xdot)
{
  xdot.setZero();
  xdot.segment<3>(DPX) = x.q.inv().rot(x.v);
  xdot.segment<3>(DQX) = gyro - x.bg;
  xdot.segment<3>(DVX) = acc - x.ba + common::gravity * x.q.rot(common::e3) - (gyro - x.bg).cross(x.v);
}


void EKF::getFG(const State &x, const Vector3d &gyro, dxMatrix &F, gMatrix &G)
{
  F.setZero();
  F.block<3,3>(DPX,DQX) = -x.q.R().transpose() * common::skew(x.v);
  F.block<3,3>(DPX,DVX) = x.q.R().transpose();
  F.block<3,3>(DQX,DGX) = -common::I_3x3;
  F.block<3,3>(DVX,DQX) = common::gravity * common::skew(x.q.rot(common::e3));
  F.block<3,3>(DVX,DVX) = -common::skew(Vector3d(gyro - x.bg));
  F.block<3,3>(DVX,DGX) = -common::skew(x.v);
  F.block<3,3>(DVX,DAX) = -common::I_3x3;

  G.setZero();
  G.block<3,3>(DQX,UWX) = -common::I_3x3;
  G.block<3,3>(DVX,UWX) = -common::skew(x.v);
  G.block<3,3>(DVX,UAX) = -common::I_3x3;
}


void EKF::getH(const State &x, const common::Quaterniond &q_bc, const Vector3d &p_bc,
               common::Quaterniond &ht, common::Quaterniond &hq, Matrix<double, 5, NUM_DOF> &H)
{
  // Declarations
  static Vector3d pt, t, t_x_k, at;
  static Matrix3d Gamma_at, dat_dt, dt_dpt, dpt_dp, dpt_dq, dpt_dpk, dpt_dqk;
  static Matrix<double, 2, 3> dexpat_dat;

  // Axis-angle representation of translation direction
  pt = q_bc.rot(x.q.rot(x.pk - x.p) + (x.q.R() * x.qk.inv().R() - common::I_3x3) * p_bc);
  t = pt / pt.norm();
  t_x_k = t.cross(common::e3);
  double tT_k = common::saturate<double>(t.transpose() * common::e3, 1.0, -1.0);
  at = acos(tT_k) * t_x_k / t_x_k.norm();

  // Create measurement models
  ht = common::Quaterniond::exp(at);
  hq = q_bc.inv() * x.q.inv() * x.qk * q_bc;

  // Sub-derivative calculations
  Gamma_at = common::Quaterniond::dexp(at);
  double txk_mag = t_x_k.norm();
  dexpat_dat = common::I_2x3 * ht.R().transpose() * Gamma_at;
  dat_dt = acos(tT_k) / txk_mag * ((t_x_k * t_x_k.transpose()) /
                           (txk_mag * txk_mag) - common::I_3x3) * common::skew(common::e3) -
                           (t_x_k * common::e3.transpose()) / (txk_mag * sqrt(1.0 - tT_k * tT_k));
  double ptmag = pt.norm();
  dt_dpt = (1.0 / ptmag) * (common::I_3x3 - pt * pt.transpose() / (ptmag * ptmag));
  Matrix3d R_bc = q_bc.R();
  dpt_dp = -R_bc * x.q.R();
  dpt_dq = R_bc * common::skew(x.q.rot(x.pk - x.p + x.qk.inv().rot(p_bc)));
  dpt_dpk = R_bc * x.q.R();
  dpt_dqk = -R_bc * x.q.R() * x.qk.inv().R() * common::skew(p_bc);
  Matrix<double,2,3> dexpat_dpt = dexpat_dat * dat_dt * dt_dpt;

  // Create measurement Jacobian
  H.setZero();
  H.block<2,3>(0,DPX) = dexpat_dpt * dpt_dp;
  H.block<2,3>(0,DQX) = dexpat_dpt * dpt_dq;
  H.block<2,3>(0,DKPX) = dexpat_dpt * dpt_dpk;
  H.block<2,3>(0,DKQX) = dexpat_dpt * dpt_dqk;
  H.block<3,3>(2,DQX) = -R_bc * x.qk.R() * x.q.inv().R();
  H.block<3,3>(2,DKQX) = R_bc;
}


void EKF::keyframeReset(State &x, dxMatrix& P)
{
  // State reset
  x.p.setZero();
  x.q = common::Quaterniond(0,0,x.q.yaw()).inv() * x.q;
  x.pk.setZero();
  x.qk = x.q;

  // Covariance reset
  dxMatrix N;
  getN(x, N);
  P = N * P * N.transpose();
}


void EKF::getN(const State &x, dxMatrix &N)
{
  double sp = sin(x.q.roll());
  double cp = cos(x.q.roll());
  double st = sin(x.q.pitch());
  double ct = cos(x.q.pitch());
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


// Relative camera pose optimizer
void EKF::optimizePose(common::Quaterniond& q, common::Quaterniond& qt,
                       const std::vector<Vector3d, aligned_allocator<Vector3d> >& e1,
                       const std::vector<Vector3d, aligned_allocator<Vector3d> >& e2,
                       const unsigned &iters)
{
  // Ensure number of directions vectors from each camera match
  if (e1.size() != e2.size())
  {
    std::cout << "\nError in optimizePose. Direction vector arrays must be the same size.\n\n";
    return;
  }

  // Constants
  const int N = e1.size();
  static const Matrix3d e1x = common::skew(common::e1);
  static const Matrix3d e2x = common::skew(common::e2);
  static const Matrix3d e3x = common::skew(common::e3);

  // Declare things that change each loop
  static Matrix3d R, Rtx, e1x_Rtx, e2x_Rtx, e3x_Rtx, e1tx, e2tx, R_e1tx_tx, R_e2tx_tx;
  static Vector3d t, e1tx_t, e2tx_t;
  static Matrix<double,5,1> delta;
  static Matrix<double, 1000, 1> r;
  static Matrix<double, 1000, 5> J;

  // Main optimization loop
  for (int i = 0; i < iters; ++i)
  {
    // Pre-compute a few things
    R = q.R();
    t = qt.uvec();
    Rtx = R * common::skew(t);
    e1x_Rtx = e1x * Rtx;
    e2x_Rtx = e2x * Rtx;
    e3x_Rtx = e3x * Rtx;
    e1tx = common::skew(qt.rot(common::e1));
    e2tx = common::skew(qt.rot(common::e2));
    e1tx_t = e1tx * t;
    e2tx_t = e2tx * t;
    R_e1tx_tx = R * common::skew(e1tx_t);
    R_e2tx_tx = R * common::skew(e2tx_t);

    // Vector of residual errors
    for (int j = 0; j < N; ++j)
      se(r(j), e1[j], e2[j], Rtx);

    // Stop if error is small enough
    if (r.topRows(N).sum() < 1e-6) break;

    // Jacobian of residual errors
    for (int j = 0; j < N; ++j)
    {
      dse(J(j,0), e1[j], e2[j], Rtx, -e1x_Rtx);
      dse(J(j,1), e1[j], e2[j], Rtx, -e2x_Rtx);
      dse(J(j,2), e1[j], e2[j], Rtx, -e3x_Rtx);
      dse(J(j,3), e1[j], e2[j], Rtx, -R_e1tx_tx);
      dse(J(j,4), e1[j], e2[j], Rtx, -R_e2tx_tx);
    }

    // Innovation
    delta = -(J.topRows(N).transpose() * J.topRows(N)).inverse() * J.topRows(N).transpose() * r.topRows(N);

    // Update camera rotation and translation
    q += Vector3d(delta.segment<3>(0));
    qt = common::Quaterniond::exp(qt.proj() * delta.segment<2>(3)) * qt;
  }
}


// Sampson's error
void EKF::se(double& err, const Vector3d& e1, const Vector3d& e2, const Matrix3d& E)
{
  static Vector3d e1T_E, E_e2;
  double e1T_E_e2 = e1.transpose() * E * e2;
  if (e1T_E_e2 < 1e-6)
    err = 0;
  else
  {
    e1T_E = (e1.transpose() * E).transpose();
    E_e2 = E * e2;
    err = e1T_E_e2 / sqrt(e1T_E(0) * e1T_E(0) + e1T_E(1) * e1T_E(1) + E_e2(0) * E_e2(0) + E_e2(1) * E_e2(1));
  }
}


// Derivative of Sampson's error
void EKF::dse(double& derr, const Vector3d& e1, const Vector3d& e2, const Matrix3d& E, const Matrix3d& dE)
{
  static Vector3d e1T_E, E_e2, e1T_dE, dE_e2;
  double e1T_E_e2 = e1.transpose() * E * e2;
  if (e1T_E_e2 < 1e-6)
    derr = 0;
  else
  {
    e1T_E = (e1.transpose() * E).transpose();
    E_e2 = E * e2;
    e1T_dE = (e1.transpose() * dE).transpose();
    dE_e2 = dE * e2;
    double val1 = sqrt(e1T_E(0) * e1T_E(0) + e1T_E(1) * e1T_E(1) + E_e2(0) * E_e2(0) + E_e2(1) * E_e2(1));
    double val2 = e1T_E(0) * e1T_dE(0) + e1T_E(1) * e1T_dE(1) + E_e2(0) * dE_e2(0) + E_e2(1) * dE_e2(1);
    double e1T_dE_e2 = e1.transpose() * dE * e2;
    derr = (e1T_dE_e2 * val1 - e1T_E_e2 * val2) / (val1 * val1);
  }
}


}
