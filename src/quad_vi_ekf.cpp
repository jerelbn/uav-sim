#include "quad_vi_ekf.h"


namespace qviekf
{


EKF::EKF() : t_prev_(0), nfa_(0) {}


EKF::~EKF()
{
  true_state_log_.close();
  ekf_state_log_.close();
  cov_log_.close();
}


void EKF::load(const string &filename, const std::string& name)
{
  // Resize arrays to the correct sizes
  common::get_yaml_node("ekf_num_features", filename, nfm_);
  common::get_yaml_node("ekf_use_drag", filename, use_drag_);
  common::get_yaml_node("ekf_use_partial_update", filename, use_partial_update_);
  common::get_yaml_node("ekf_use_keyframe_reset", filename, use_keyframe_reset_);
  if (use_drag_)
    nbs_ = 17;
  else
    nbs_ = 16;
  nbd_ = nbs_ - 1;
  num_states_ = nbs_ + 3 * nfm_;
  num_dof_ = nbd_ + 3 * nfm_;

  xdot_ = VectorXd::Zero(num_dof_);
  xdot_prev_ = VectorXd::Zero(num_dof_);
  dxp_ = VectorXd::Zero(num_dof_);
  dxm_ = VectorXd::Zero(num_dof_);
  P_ = MatrixXd::Zero(num_dof_,num_dof_);
  F_ = MatrixXd::Zero(num_dof_,num_dof_);
  A_ = MatrixXd::Zero(num_dof_,num_dof_);
  Qx_ = MatrixXd::Zero(num_dof_,num_dof_);
  I_DOF_ = MatrixXd::Zero(num_dof_,num_dof_);
  G_ = MatrixXd::Zero(num_dof_,NI);
  B_ = MatrixXd::Zero(num_dof_,NI);
  R_cam_big_ = MatrixXd::Zero(2*nfm_,2*nfm_);
  z_cam_ = VectorXd::Zero(2*nfm_);
  h_cam_ = VectorXd::Zero(2*nfm_);
  H_cam_ = MatrixXd::Zero(2*nfm_,num_dof_);
  K_cam_ = MatrixXd::Zero(num_dof_,2*nfm_);
  H_gps_ = MatrixXd::Zero(6,num_dof_);
  K_gps_ = MatrixXd::Zero(num_dof_,6);
  H_mocap_ = MatrixXd::Zero(6,num_dof_);
  K_mocap_ = MatrixXd::Zero(num_dof_,6);
  feats_true_.reserve(nfm_);

  // Initializations
  Vector3d lambda_feat;
  VectorXd x0(17), P0_base(16), Qx_base(16), lambda_base(16);
  common::get_yaml_node("ekf_rho0", filename, rho0_);
  common::get_yaml_node("ekf_init_imu_bias", filename, init_imu_bias_);
  common::get_yaml_eigen("ekf_x0", filename, x0);
  common::get_yaml_eigen("ekf_P0", filename, P0_base);
  common::get_yaml_eigen("ekf_Qx", filename, Qx_base);
  common::get_yaml_eigen_diag("ekf_Qu", filename, Qu_);
  common::get_yaml_eigen_diag("ekf_P0_feat", filename, P0_feat_);
  common::get_yaml_eigen_diag("ekf_Qx_feat", filename, Qx_feat_);
  x_ = State<double>(x0, nbs_, nfm_);
  P_.topLeftCorner(nbd_,nbd_) = P0_base.topRows(nbd_).asDiagonal();
  Qx_.topLeftCorner(nbd_,nbd_) = Qx_base.topRows(nbd_).asDiagonal();
  for (int i = 0; i < nfm_; ++i)
    Qx_.block<3,3>(nbd_+3*i,nbd_+3*i) = Qx_feat_;
  for (int i = 0; i < nfm_; ++i)
    H_cam_.block<2,2>(2*i,nbd_+3*i).setIdentity();
  I_DOF_.setIdentity();

  if (use_partial_update_)
  {
    common::get_yaml_eigen("ekf_lambda", filename, lambda_base);
    common::get_yaml_eigen("ekf_lambda_feat", filename, lambda_feat);
    dx_ones_ = VectorXd::Ones(num_dof_);
    lambda_ = VectorXd::Zero(num_dof_);
    Lambda_ = MatrixXd::Zero(num_dof_,num_dof_);
    lambda_.topRows(nbd_) = lambda_base.topRows(nbd_);
    for (int i = 0; i < nfm_; ++i)
      lambda_.segment<3>(nbd_+3*i) = lambda_feat;
    Lambda_ = dx_ones_ * lambda_.transpose() + lambda_*dx_ones_.transpose() - lambda_*lambda_.transpose();
  }

  // Load sensor parameters
  Vector4d q_ub, q_um, q_uc;
  common::get_yaml_eigen_diag("ekf_R_gps", filename, R_gps_);
  common::get_yaml_eigen_diag("ekf_R_mocap", filename, R_mocap_);
  common::get_yaml_eigen_diag("ekf_R_cam", filename, R_cam_);
  common::get_yaml_eigen("p_ub", filename, p_ub_);
  common::get_yaml_eigen("q_ub", filename, q_ub);
  common::get_yaml_eigen("p_um", filename, p_um_);
  common::get_yaml_eigen("q_um", filename, q_um);
  common::get_yaml_eigen("p_uc", filename, p_uc_);
  common::get_yaml_eigen("q_uc", filename, q_uc);
  common::get_yaml_eigen("camera_matrix", filename, cam_matrix_);
  q_u2b_ = quat::Quatd(q_ub);
  q_u2m_ = quat::Quatd(q_um);
  q_u2c_ = quat::Quatd(q_uc);
  for (int i = 0; i < nfm_; ++i)
    R_cam_big_.block<2,2>(2*i,2*i) = R_cam_;
  fx_ = cam_matrix_(0,0);
  fy_ = cam_matrix_(1,1);
  u0_ = cam_matrix_(0,2);
  v0_ = cam_matrix_(1,2);
  image_center_ << u0_, v0_;

  // Logging
  std::stringstream ss_t, ss_e, ss_c;
  ss_t << "/tmp/" << name << "_ekf_truth.log";
  ss_e << "/tmp/" << name << "_ekf_est.log";
  ss_c << "/tmp/" << name << "_ekf_cov.log";
  true_state_log_.open(ss_t.str());
  ekf_state_log_.open(ss_e.str());
  cov_log_.open(ss_c.str());
}


void EKF::run(const double &t, const sensors::Sensors &sensors, const Vector3d& vw, const vehicle::Stated& x_true)
{
  // Initialize IMU close to the truth (simulate flat ground calibration)
  if (t == 0 && init_imu_bias_)
  {
    x_.ba = sensors.getAccelBias() + 0.1 * Vector3d::Random();
    x_.bg = sensors.getGyroBias() + 0.01 * Vector3d::Random();
  }

  // Propagate the state and covariance to the current time step
  if (sensors.new_imu_meas_)
    propagate(t, sensors.imu_);

  // Apply updates
  if (t > 0)
  {
    if (sensors.new_gps_meas_)
      gpsUpdate(sensors.gps_);
    if (sensors.new_mocap_meas_)
      mocapUpdate(sensors.mocap_);
    if (sensors.new_camera_meas_)
      cameraUpdate(sensors.cam_);
  }

  // Log data
  logTruth(t, sensors, x_true);
  logEst(t);
}


void EKF::propagate(const double &t, const uVector& imu)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Kinematics
  f(x_, imu, xdot_);

  if (t > 0)
  {
    // TODO: try trapezoidal integration of continuous covariance kinematics
    // Propagate the covariance - guarantee positive-definite P with discrete propagation
//    analyticalFG(x_, imu, F_, G_);
    numericalFG(x_, imu, F_, G_);
    A_ = I_DOF_ + F_ * dt + F_ * F_ * dt * dt / 2.0; // Approximate state transition matrix
    B_ = (I_DOF_ + F_ * dt / 2.0) * G_ * dt;
    P_.topLeftCorner(nbd_+3*nfa_,nbd_+3*nfa_) = A_.topLeftCorner(nbd_+3*nfa_,nbd_+3*nfa_) * P_.topLeftCorner(nbd_+3*nfa_,nbd_+3*nfa_) * A_.topLeftCorner(nbd_+3*nfa_,nbd_+3*nfa_).transpose() +
                                              B_.topRows(nbd_+3*nfa_) * Qu_ * B_.topRows(nbd_+3*nfa_).transpose() + Qx_.topLeftCorner(nbd_+3*nfa_,nbd_+3*nfa_);

    // Trapezoidal integration
    x_ += 0.5 * (xdot_ + xdot_prev_) * dt;
  }

  // Save current kinematics for next iteration
  xdot_prev_ = xdot_;
}


void EKF::gpsUpdate(const Vector6d& z)
{
  // Measurement model and matrix
  h_gps_.head<3>() = x_.p;
  h_gps_.tail<3>() = x_.q.rota(x_.v);

  H_gps_.block<3,3>(0,DP).setIdentity();
  H_gps_.block<3,3>(3,DV) = x_.q.inverse().R();
  H_gps_.block<3,3>(3,DQ) = -x_.q.inverse().R() * common::skew(x_.v);

  // Apply the update
  update(z-h_gps_, R_gps_, H_gps_, K_gps_);
}


void EKF::mocapUpdate(const Vector7d& z)
{
  // Pack measurement into Xform
  xform::Xformd Xz(z);

  // Measurement model and matrix
  h_mocap_.t_ = x_.p + x_.q.rota(p_um_);
  h_mocap_.q_ = x_.q * q_u2m_;

  H_mocap_.block<3,3>(0,DP).setIdentity();
  H_mocap_.block<3,3>(0,DQ) = -x_.q.inverse().R() * common::skew(p_um_);
  H_mocap_.block<3,3>(3,DQ) = q_u2m_.inverse().R();

  // Apply the update
  update(Xz-h_mocap_, R_mocap_, H_mocap_, K_mocap_);
}


void EKF::cameraUpdate(const sensors::FeatVec &tracked_feats)
{
  // Collect measurement of each feature in the state and remove feature states that have lost tracking
  getPixMatches(tracked_feats);

  // Apply the update
  if (nfa_ > 0)
  {
    // Build measurement vector and model
    z_cam_.setZero();
    h_cam_.setZero();
    for (int i = 0; i < nfa_; ++i)
    {
      z_cam_.segment<2>(2*i) = matched_feats_[i];
      h_cam_.segment<2>(2*i) = x_.feats[i].pix;
    }

    update(z_cam_-h_cam_, R_cam_big_, H_cam_, K_cam_);
  }

  // Fill state with new features if needed
  if (nfa_ < nfm_)
    addFeatToState(tracked_feats);
}


void EKF::getPixMatches(const sensors::FeatVec &tracked_feats)
{
  int i = 0;
  matched_feats_.clear();
  while (i < nfa_)
  {
    // Look for an ID match to the current state feature
    bool id_match_found = false;
    for (auto& tf : tracked_feats)
    {
      if (x_.feats[i].id == tf.id)
      {
        matched_feats_.push_back(tf.pix);
        id_match_found = true;
        feats_true_[i].pix = tf.pix;
        feats_true_[i].rho = tf.rho;
        break;
      }
    }

    // Remove feature from state when no longer tracked
    if (!id_match_found)
      removeFeatFromState(i);
    else
      ++i;
  }

  if (matched_feats_.size() != nfa_)
    cerr << "Matched camera measurements does not match number of active features!" << endl;
}


void EKF::removeFeatFromState(const int &idx)
{
  // Shift and reset state and covariance
  int j;
  for (j = idx; j < nfa_-1; ++j)
  {
    x_.feats[j] = x_.feats[j+1];
    P_.row(nbd_+3*j) = P_.row(nbd_+3*(j+1));
    P_.row(nbd_+3*j+1) = P_.row(nbd_+3*(j+1)+1);
    P_.row(nbd_+3*j+2) = P_.row(nbd_+3*(j+1)+2);
    P_.col(nbd_+3*j) = P_.col(nbd_+3*(j+1));
    P_.col(nbd_+3*j+1) = P_.col(nbd_+3*(j+1)+1);
    P_.col(nbd_+3*j+2) = P_.col(nbd_+3*(j+1)+2);
  }
  x_.feats[j] = sensors::Feat();
  P_.row(nbd_+3*j).setZero();
  P_.row(nbd_+3*j+1).setZero();
  P_.row(nbd_+3*j+2).setZero();
  P_.col(nbd_+3*j).setZero();
  P_.col(nbd_+3*j+1).setZero();
  P_.col(nbd_+3*j+2).setZero();

  // Decrement the active feature counter
  --nfa_;
}


void EKF::addFeatToState(const sensors::FeatVec &tracked_feats)
{
  // Sort features by proximity to image center
  auto sorted_feats = tracked_feats;
  sort(sorted_feats.begin(), sorted_feats.end(),
       [this](const sensors::Feat& f1, const sensors::Feat& f2)
       {return (f1.pix - image_center_).norm() < (f2.pix - image_center_).norm();});

  // Loop through features, adding the best ones to the state
  for (auto& f : sorted_feats)
  {
    // Check if current sorted feature ID is already in the state then add it to the state
    bool id_used = false;
    for (auto sf : x_.feats)
    {
      if (f.id == sf.id)
      {
        id_used = true;
        break;
      }
    }

    if (!id_used)
    {
      // Initialize feature state and corresponding covariance block
      x_.feats[nfa_] = sensors::Feat(f.pix,rho0_,f.id);
      P_.block<3,3>(nbd_+3*nfa_,nbd_+3*nfa_) = P0_feat_;

      // Store true inverse depth and pixel position
      feats_true_[nfa_].pix = f.pix;
      feats_true_[nfa_].rho = f.rho;

      // Increment number of active features
      ++nfa_;
    }

    // Don't try adding more features than allowed
    if (nfa_ == nfm_) break;
  }
}


void EKF::update(const VectorXd &err, const MatrixXd& R, const MatrixXd &H, MatrixXd &K)
{
  K = P_ * H.transpose() * (R + H * P_ * H.transpose()).inverse();
  if (use_partial_update_)
  {
    x_ += lambda_.cwiseProduct(K * err);
    P_ += Lambda_.cwiseProduct((I_DOF_ - K * H) * P_ * (I_DOF_ - K * H).transpose() + K * R * K.transpose() - P_);
  }
  else
  {
    x_ += K * err;
    P_ = (I_DOF_ - K * H) * P_ * (I_DOF_ - K * H).transpose() + K * R * K.transpose();
  }
}


void EKF::f(const Stated &x, const uVector &u, VectorXd &dx, const uVector &eta)
{
  Vector3d accel = u.segment<3>(UA) - x.ba - eta.segment<3>(UA);
  Vector3d omega = u.segment<3>(UG) - x.bg - eta.segment<3>(UG);

  Vector3d omega_c = q_u2c_.rotp(omega);
  Vector3d v_c = q_u2c_.rotp(x.v + omega.cross(p_uc_));

  dx.setZero();
  dx.segment<3>(DP) = x.q.rota(x.v);
  if (use_drag_)
    dx.segment<3>(DV) = common::e3 * common::e3.transpose() * accel + common::gravity * x.q.rotp(common::e3) -
                        x.mu * (common::I_3x3 - common::e3 * common::e3.transpose()) * x.v - omega.cross(x.v);
  else
    dx.segment<3>(DV) = accel + common::gravity * x.q.rotp(common::e3) - omega.cross(x.v);
  dx.segment<3>(DQ) = omega;
  for (int i = 0; i < nfm_; ++i)
  {
    Vector2d pix = x.feats[i].pix;
    double rho = x.feats[i].rho;
    dx.segment<2>(nbd_+3*i) = Omega(pix) * omega_c + rho * V(pix) * v_c;
    dx(nbd_+3*i+2) = rho * M(pix) * omega_c + rho * rho * common::e3.dot(v_c);
  }
}


void EKF::analyticalFG(const Stated &x, const uVector &u, MatrixXd &F, MatrixXd &G)
{
  F.setZero();
  F.block<3,3>(DP,DV) = x.q.inverse().R();
  F.block<3,3>(DP,DQ) = -x.q.inverse().R() * common::skew(x.v);
  F.block<3,3>(DV,DV) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DV,DQ) = common::gravity * common::skew(x.q.rotp(common::e3));
  F.block<3,3>(DV,DBA) = -common::I_3x3;
  F.block<3,3>(DV,DBG) = -common::skew(x.v);
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DQ,DBG) = -common::I_3x3;

  G.setZero();
  G.block<3,3>(DV,UA) = -common::I_3x3;
  G.block<3,3>(DV,UG) = -common::skew(x.v);
  G.block<3,3>(DQ,UG) = -common::I_3x3;
}


void EKF::numericalFG(const Stated &x, const uVector &u, MatrixXd &F, MatrixXd &G)
{
  static const double eps(1e-5);
  static Matrix6d I6 = Matrix6d::Identity();
  static Vector6d eta = Vector6d::Ones();

  for (int i = 0; i < F.cols(); ++i)
  {
    Stated xp = x + I_DOF_.col(i) * eps;
    Stated xm = x + I_DOF_.col(i) * -eps;

    f(xp, u, dxp_);
    f(xm, u, dxm_);

    F.col(i) = (dxp_ - dxm_) / (2.0 * eps);
  }
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);

  for (int i = 0; i < G.cols(); ++i)
  {
    uVector etap = eta + I6.col(i) * eps;
    uVector etam = eta + I6.col(i) * -eps;

    f(x, u, dxp_, etap);
    f(x, u, dxm_, etam);

    G.col(i) = (dxp_ - dxm_) / (2.0 * eps);
  }
}


void EKF::logTruth(const double &t, const sensors::Sensors &sensors, const vehicle::Stated& x_true)
{
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x_true.p.data(), 3 * sizeof(double));
  true_state_log_.write((char*)x_true.v.data(), 3 * sizeof(double));
  true_state_log_.write((char*)x_true.q.euler().data(), 3 * sizeof(double));
  true_state_log_.write((char*)sensors.getAccelBias().data(), 3 * sizeof(double));
  true_state_log_.write((char*)sensors.getGyroBias().data(), 3 * sizeof(double));
  if(use_drag_)
    true_state_log_.write((char*)&x_true.drag, sizeof(double));

  // Compute true landmark pixel measurement
  for (int i = 0; i < nfm_; ++i)
  {
    if (i+1 <= nfa_)
    {
      true_state_log_.write((char*)feats_true_[i].pix.data(), 2 * sizeof(double));
      true_state_log_.write((char*)&feats_true_[i].rho, sizeof(double));
    }
    else
    {
      static const Vector2d a(NAN,NAN);
      static const double b(NAN);
      true_state_log_.write((char*)a.data(), 2 * sizeof(double));
      true_state_log_.write((char*)&b, sizeof(double));
    }
  }
}


void EKF::logEst(const double &t)
{
  ekf_state_log_.write((char*)&t, sizeof(double));
  ekf_state_log_.write((char*)x_.p.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.v.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.q.euler().data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.ba.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.bg.data(), 3 * sizeof(double));
  if (use_drag_)
    ekf_state_log_.write((char*)&x_.mu, sizeof(double));
  for (int i = 0; i < nfm_; ++i)
  {
    if (i+1 <= nfa_)
    {
      ekf_state_log_.write((char*)x_.feats[i].pix.data(), 2 * sizeof(double));
      ekf_state_log_.write((char*)&x_.feats[i].rho, sizeof(double));
    }
    else
    {
      static const Vector2d a(NAN,NAN);
      static const double b(NAN);
      ekf_state_log_.write((char*)a.data(), 2 * sizeof(double));
      ekf_state_log_.write((char*)&b, sizeof(double));
    }
  }

  P_diag_ = P_.diagonal();
  cov_log_.write((char*)&t, sizeof(double));
  cov_log_.write((char*)P_diag_.data(), P_diag_.rows() * sizeof(double));
}


vehicle::Stated EKF::getState() const
{
  vehicle::Stated x;
  x.p = x_.p;
  x.v = x_.q.rotp(x_.v);
  x.q = x_.q;
  x.omega = xdot_.segment<3>(DQ);
  return x;
}


Matrix<double,2,3> EKF::Omega(const Vector2d &nu)
{
  double eps_x = nu(0) - u0_;
  double eps_y = nu(1) - v0_;

  Matrix<double,2,3> m;
  m(0,0) = eps_x * eps_y / fy_;
  m(0,1) = -(fx_ + eps_x * eps_x / fx_);
  m(0,2) = fx_ / fy_ * eps_y;
  m(1,0) = fy_ + eps_y * eps_y / fy_;
  m(1,1) = -eps_x * eps_y / fx_;
  m(1,2) = -fy_ / fx_ * eps_x;
  return m;
}


Matrix<double,2,3> EKF::V(const Vector2d &nu)
{
  double eps_x = nu(0) - u0_;
  double eps_y = nu(1) - v0_;

  Matrix<double,2,3> m;
  m.setZero();
  m(0,0) = -fx_;
  m(0,2) = eps_x;
  m(1,1) = -fy_;
  m(1,2) = eps_y;
  return m;
}


RowVector3d EKF::M(const Vector2d &nu)
{
  double eps_x = nu(0) - u0_;
  double eps_y = nu(1) - v0_;

  RowVector3d m;
  m(0) = eps_y / fy_;
  m(1) = -eps_x / fx_;
  m(2) = 0;
  return m;
}


} // namespace qviekf