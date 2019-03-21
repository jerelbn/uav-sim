#include "quad_vi_ekf.h"


namespace qviekf
{


EKF::EKF() : t_prev_(0) {}


EKF::~EKF()
{
  true_state_log_.close();
  ekf_state_log_.close();
  cov_log_.close();
}


void EKF::load(const string &filename, const std::string& name)
{
  // EKF initializations
  baseXVector x0;
  baseDxVector P0_diag, Qx_diag;
  Vector3d P0_feat_diag, Qx_feat_diag;
  Matrix<double, NUM_INPUTS, 1> Qu_diag;
  common::get_yaml_eigen("ekf_x0", filename, x0);
  common::get_yaml_eigen("ekf_P0", filename, P0_diag);
  common::get_yaml_eigen("ekf_Qx", filename, Qx_diag);
  common::get_yaml_eigen("ekf_Qu", filename, Qu_diag);
  common::get_yaml_eigen("ekf_P0_feat", filename, P0_feat_diag);
  common::get_yaml_eigen("ekf_Qx_feat", filename, Qx_feat_diag);
  x_ = State<double>(x0);
  P_.topLeftCorner<NUM_BASE_DOF,NUM_BASE_DOF>() = P0_diag.asDiagonal();
  P_.bottomRightCorner<3,3>() = P0_feat_diag.asDiagonal();
  Qx_.topLeftCorner<NUM_BASE_DOF,NUM_BASE_DOF>() = Qx_diag.asDiagonal();
  Qx_.bottomRightCorner<3,3>() = Qx_feat_diag.asDiagonal();
  Qu_ = Qu_diag.asDiagonal();
  I_NUM_DOF_.setIdentity();

  // Load sensor parameters
  Vector4d q_ub, q_um, q_uc;
  Vector6d R_gps_diag, R_mocap_diag;
  Vector2d R_pix_diag;
  common::get_yaml_eigen("ekf_R_gps", filename, R_gps_diag);
  common::get_yaml_eigen("ekf_R_mocap", filename, R_mocap_diag);
  common::get_yaml_eigen("ekf_R_pix", filename, R_pix_diag);
  common::get_yaml_eigen("p_ub", filename, p_ub_);
  common::get_yaml_eigen("q_ub", filename, q_ub);
  common::get_yaml_eigen("p_um", filename, p_um_);
  common::get_yaml_eigen("q_um", filename, q_um);
  common::get_yaml_eigen("p_uc", filename, p_uc_);
  common::get_yaml_eigen("q_um", filename, q_uc);
  common::get_yaml_eigen("camera_matrix", filename, cam_matrix_);
  q_u2b_ = quat::Quatd(q_ub);
  q_u2m_ = quat::Quatd(q_um);
  q_u2c_ = quat::Quatd(q_uc);
  R_gps_ = R_gps_diag.asDiagonal();
  R_mocap_ = R_mocap_diag.asDiagonal();
  R_pix_ = R_pix_diag.asDiagonal();
  fx_ = cam_matrix_(0,0);
  fy_ = cam_matrix_(1,1);
  u0_ = cam_matrix_(0,2);
  v0_ = cam_matrix_(1,2);

  // Logging
  std::stringstream ss_t, ss_e, ss_c;
  ss_t << "/tmp/" << name << "_ekf_truth.log";
  ss_e << "/tmp/" << name << "_ekf_est.log";
  ss_c << "/tmp/" << name << "_ekf_cov.log";
  true_state_log_.open(ss_t.str());
  ekf_state_log_.open(ss_e.str());
  cov_log_.open(ss_c.str());

  // Temporaries
  lm1_ << 0, 0, 0;

  // Compute true landmark pixel measurement
  Vector3d lm1c = q_u2c_.rotp(q_u2b_.rota(x_.q.rotp(lm1_ - x_.p)));
  double rho_true = 1.0 / lm1c(2);
  Vector2d pix_true = rho_true * cam_matrix_.topRows<2>() * lm1c;

  x_.pix = pix_true;
  x_.rho = rho_true;
}


void EKF::run(const double &t, const sensors::Sensors &sensors, const Vector3d& vw, const vehicle::Stated& x_true)
{
  // Propagate the state and covariance to the current time step
  if (sensors.new_imu_meas_)
    propagate(t, sensors.imu_);

  // Apply updates
  if (sensors.new_gps_meas_)
    updateGPS(sensors.gps_);
  if (sensors.new_mocap_meas_)
    updateMocap(sensors.mocap_);

  // Compute true landmark pixel measurement
  Vector3d lm1c = q_u2c_.rotp(q_u2b_.rota(x_true.q.rotp(lm1_ - x_true.p)));
  double rho_true = 1.0 / lm1c(2);
  Vector2d pix_true = rho_true * cam_matrix_.topRows<2>() * lm1c;
  updateCamera(pix_true);

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
    A_ = I_NUM_DOF_ + F_ * dt + F_ * F_ * dt * dt / 2.0; // Approximate state transition matrix
    B_ = (I_NUM_DOF_ + F_ * dt / 2.0 + F_ * F_ * dt * dt / 6.0) * G_ * dt;
    P_ = A_ * P_ * A_.transpose() + B_ * Qu_ * B_.transpose() + Qx_;

    // Trapezoidal integration
    x_ += 0.5 * (xdot_ + xdot_prev_) * dt;
  }

  // Save current kinematics for next iteration
  xdot_prev_ = xdot_;
}


void EKF::updateGPS(const Vector6d& z)
{
  // Measurement model and matrix
  Matrix<double,6,1> h;
  h.head<3>() = x_.p;
  h.tail<3>() = x_.q.rota(x_.v);

  Matrix<double,6,NUM_DOF> H = Matrix<double,6,NUM_DOF>::Zero();
  H.block<3,3>(0,DP).setIdentity();
  H.block<3,3>(3,DV) = x_.q.inverse().R();
  H.block<3,3>(3,DQ) = -x_.q.inverse().R() * common::skew(x_.v);

  // Kalman gain and update
  Matrix<double,NUM_DOF,6> K = P_ * H.transpose() * (R_gps_ + H * P_ * H.transpose()).inverse();
  x_ += K * (z - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_gps_ * K.transpose();
}


void EKF::updateMocap(const Vector7d& z)
{
  // Pack measurement into Xform
  xform::Xformd Xz(z);

  // Measurement model and matrix
  xform::Xformd h;
  h.t_ = x_.p + x_.q.rota(p_um_);
  h.q_ = x_.q * q_u2m_;

  Matrix<double,6,NUM_DOF> H = Matrix<double,6,NUM_DOF>::Zero();
  H.block<3,3>(0,DP).setIdentity();
  H.block<3,3>(0,DQ) = -x_.q.inverse().R() * common::skew(p_um_);
  H.block<3,3>(3,DQ) = q_u2m_.inverse().R();

  // Kalman gain and update
  Matrix<double,NUM_DOF,6> K = P_ * H.transpose() * (R_mocap_ + H * P_ * H.transpose()).inverse();
  x_ += K * (Xz - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_mocap_ * K.transpose();
}


void EKF::updateCamera(const Vector2d& z)
{
  // Measurement model and matrix
  Vector2d h = x_.pix;

  Matrix<double,2,NUM_DOF> H = Matrix<double,2,NUM_DOF>::Zero();
  H.block<2,2>(0,NUM_BASE_DOF).setIdentity();

  // Kalman gain and update
  Matrix<double,NUM_DOF,2> K = P_ * H.transpose() * (R_pix_ + H * P_ * H.transpose()).inverse();
  x_ += K * (z - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_pix_ * K.transpose();
}


void EKF::f(const Stated &x, const uVector &u, dxVector &dx)
{
  Vector3d omega_c = q_u2c_.rotp(u.segment<3>(UG) - x.bg);
  Vector3d v_c = q_u2c_.rotp(x.v + (u.segment<3>(UG) - x.bg).cross(p_uc_));

  dx.setZero();
  dx.segment<3>(DP) = x.q.rota(x.v);
  dx.segment<3>(DV) = u.segment<3>(UA) - x.ba + common::gravity * x.q.rotp(common::e3) - (u.segment<3>(UG) - x.bg).cross(x.v);
  dx.segment<3>(DQ) = u.segment<3>(UG) - x.bg;
  dx.segment<2>(NUM_BASE_DOF) = Omega(x.pix) * omega_c + x.rho * V(x.pix) * v_c;
  dx(NUM_BASE_DOF+2) = x.rho * M(x.pix) * omega_c + x.rho * x.rho * common::e3.dot(v_c);
}


void EKF::f2(const Stated &x, const uVector &u, const uVector& eta, dxVector &dx)
{
  Vector3d omega_c = q_u2c_.rotp(u.segment<3>(UG) - x.bg - eta.segment<3>(UG));
  Vector3d v_c = q_u2c_.rotp(x.v + (u.segment<3>(UG) - x.bg).cross(p_uc_));

  dx.setZero();
  dx.segment<3>(DP) = x.q.rota(x.v);
  dx.segment<3>(DV) = u.segment<3>(UA) - x.ba - eta.segment<3>(UA) + common::gravity * x.q.rotp(common::e3) - (u.segment<3>(UG) - x.bg).cross(x.v);
  dx.segment<3>(DQ) = u.segment<3>(UG) - x.bg;
  dx.segment<2>(NUM_BASE_DOF) = Omega(x.pix) * omega_c + x.rho * V(x.pix) * v_c;
  dx(NUM_BASE_DOF+2) = x.rho * M(x.pix) * omega_c + x.rho * x.rho * common::e3.dot(v_c);
}


void EKF::analyticalFG(const Stated &x, const uVector &u, dxMatrix &F, nuMatrix &G)
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


void EKF::numericalFG(const Stated &x, const uVector &u, dxMatrix &F, nuMatrix &G)
{
  static const double eps(1e-5);
  static Matrix6d I6 = Matrix6d::Identity();
  static Vector6d eta = Vector6d::Ones();

  for (int i = 0; i < F.cols(); ++i)
  {
    Stated xp = x + I_NUM_DOF_.col(i) * eps;
    Stated xm = x + I_NUM_DOF_.col(i) * -eps;

    dxVector dxp, dxm;
    f(xp, u, dxp);
    f(xm, u, dxm);

    F.col(i) = (dxp - dxm) / (2.0 * eps);
  }
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);

  for (int i = 0; i < G.cols(); ++i)
  {
    uVector etap = eta + I6.col(i) * eps;
    uVector etam = eta + I6.col(i) * -eps;

    dxVector dxp, dxm;
    f2(x, u, etap, dxp);
    f2(x, u, etam, dxm);

    G.col(i) = (dxp - dxm) / (2.0 * eps);
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

  // Compute true landmark pixel measurement
  Vector3d lm1c = q_u2c_.rotp(q_u2b_.rota(x_true.q.rotp(lm1_ - x_true.p)));
  double rho_true = 1.0 / lm1c(2);
  Vector2d pix_true = rho_true * cam_matrix_.topRows<2>() * lm1c;

  true_state_log_.write((char*)pix_true.data(), 2 * sizeof(double));
  true_state_log_.write((char*)&rho_true, sizeof(double));
}


void EKF::logEst(const double &t)
{
  ekf_state_log_.write((char*)&t, sizeof(double));
  ekf_state_log_.write((char*)x_.p.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.v.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.q.euler().data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.ba.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.bg.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.pix.data(), 2 * sizeof(double));
  ekf_state_log_.write((char*)&x_.rho, sizeof(double));

  dxVector P_diag = P_.diagonal();
  cov_log_.write((char*)&t, sizeof(double));
  cov_log_.write((char*)P_diag.data(), P_diag.rows() * sizeof(double));
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
