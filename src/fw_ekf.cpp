#include "fw_ekf.h"


namespace ekf
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
  xVector x0;
  dxVector P0_diag, Qx_diag;
  Matrix<double, 2*NUM_INPUTS, 1> Qu_diag;
  Matrix<double, 5, 1> R_vo_diag;
  common::get_yaml_eigen("ekf_x0", filename, x0);
  common::get_yaml_eigen("ekf_P0", filename, P0_diag);
  common::get_yaml_eigen("ekf_Qx", filename, Qx_diag);
  common::get_yaml_eigen("ekf_Qu", filename, Qu_diag);
  x_ = State<double>(x0);
  P_ = P0_diag.asDiagonal();
  Qx_ = Qx_diag.asDiagonal();
  Qu_ = Qu_diag.asDiagonal();
  I_NUM_DOF_.setIdentity();

  // Load sensor parameters
  Vector4d q_ub;
  Matrix<double,6,1> R_gps_diag;
  common::get_yaml_eigen("ekf_R_gps", filename, R_gps_diag);
  common::get_yaml_eigen("p_ub", filename, p_ub_);
  common::get_yaml_eigen("q_ub", filename, q_ub);
  q_u2b_ = quat::Quatd(q_ub);
  R_gps_ = R_gps_diag.asDiagonal();

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
  // Propagate the state and covariance to the current time step
  if (sensors.new_imu_meas_)
    propagate(t, sensors.imu_);

  // Apply updates
  if (sensors.new_gps_meas_)
    updateGPS(sensors.gps_);

  // Log data
  logTruth(t, sensors, vw, x_true);
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
    getFG(x_, imu, F_, G_);
    A_ = I_NUM_DOF_ + F_ * dt + F_ * F_ * dt * dt / 2.0; // Approximate state transition matrix
    B_ = (I_NUM_DOF_ + F_ * dt / 2.0 + F_ * F_ * dt * dt / 6.0) * G_ * dt;
    P_ = A_ * P_ * A_.transpose() + B_ * Qu_ * B_.transpose() + Qx_;

    // Trapezoidal integration
    x_ += 0.5 * (xdot_ + xdot_prev_) * dt;
  }

  // Save current kinematics for next iteration
  xdot_prev_ = xdot_;
}


void EKF::updateGPS(const Matrix<double,6,1> &z)
{
  // Measurement model and matrix
  Matrix<double,6,1> h;
  h.head<3>() = x_.p;
  h.tail<3>() = x_.v;

  Matrix<double,6,NUM_DOF> H = Matrix<double,6,NUM_DOF>::Zero();
  H.block<3,3>(0,DP).setIdentity();
  H.block<3,3>(3,DV).setIdentity();

  // Kalman gain and update
  Matrix<double,NUM_DOF,6> K = P_ * H.transpose() * (R_gps_ + H * P_ * H.transpose()).inverse();
  x_ += K * (z - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_gps_ * K.transpose();
}


void EKF::f(const Stated &x, const uVector &u, dxVector &dx)
{
  dx.setZero();
  dx.segment<3>(DP) = x.v;
  dx.segment<3>(DV) = x.q.rota(u.segment<3>(UA) - x.ba) + common::gravity * common::e3;
  dx.segment<3>(DQ) = u.segment<3>(UG) - x.bg;
}


void EKF::getFG(const Stated &x, const uVector &u, dxMatrix &F, nuMatrix &G)
{
  F.setZero();
  F.block<3,3>(DP,DV).setIdentity();
  F.block<3,3>(DV,DQ) = -x.q.inverse().R() * common::skew(u.segment<3>(UA) - x.ba);
  F.block<3,3>(DV,DBA) = -x.q.inverse().R();
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DQ,DBG) = -common::I_3x3;

  G.setZero();
  G.block<3,3>(DV,UA) = -x.q.inverse().R();
  G.block<3,3>(DQ,UG) = -common::I_3x3;
  G.block<3,3>(DBA,DBA).setIdentity();
  G.block<3,3>(DBG,DBG).setIdentity();
}


void EKF::logTruth(const double &t, const sensors::Sensors &sensors, const Vector3d& vw, const vehicle::Stated& x_true)
{
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x_true.p.data(), 3 * sizeof(double));
  true_state_log_.write((char*)x_true.q.rota(x_true.v).data(), 3 * sizeof(double));
  true_state_log_.write((char*)x_true.q.euler().data(), 3 * sizeof(double));
  true_state_log_.write((char*)sensors.getAccelBias().data(), 3 * sizeof(double));
  true_state_log_.write((char*)sensors.getGyroBias().data(), 3 * sizeof(double));
  true_state_log_.write((char*)vw.data(), 3 * sizeof(double));
  true_state_log_.write((char*)&sensors.getBaroBias(), sizeof(double));
}


void EKF::logEst(const double &t)
{
  ekf_state_log_.write((char*)&t, sizeof(double));
  ekf_state_log_.write((char*)x_.p.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.v.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.q.euler().data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.ba.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.bg.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.vw.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)&x_.bb, sizeof(double));

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



} // namespace ekf
