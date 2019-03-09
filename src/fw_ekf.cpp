#include "fw_ekf.h"


namespace ekf
{


EKF::EKF() : t_prev_(0) {}


EKF::~EKF()
{
  state_log_.close();
  cov_log_.close();
}


void EKF::load(const string &filename, const std::string& name)
{
  // EKF initializations
  xVector x0;
  dxVector P0_diag, Qx_diag;
  Matrix<double, NUM_INPUTS, 1> Qu_diag;
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
  common::get_yaml_eigen("p_ub", filename, p_ub_);
  common::get_yaml_eigen("q_ub", filename, q_ub);
  q_u2b_ = quat::Quatd(q_ub);

  // Logging
  std::stringstream ss_s, ss_c;
  ss_s << "/tmp/" << name << "_ekf_state.log";
  ss_c << "/tmp/" << name << "_ekf_cov.log";
  state_log_.open(ss_s.str());
  cov_log_.open(ss_c.str());
}


void EKF::run(const double &t, const sensors::Sensors &sensors)
{
  // Propagate the state and covariance to the current time step
  if (sensors.new_imu_meas_)
    propagate(t, sensors.imu_);

  // Apply updates

  // Log data
  log(t);
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


void EKF::f(const Stated &x, const uVector &u, dxVector &dx)
{
  Vector3d acc = u.segment<3>(UA) - x.ba;
  Vector3d omega = u.segment<3>(UG) - x.bg;

  dx.setZero();
  dx.segment<3>(P) = x.q.rota(x.v);
  dx.segment<3>(V) = acc + common::gravity * x.q.rotp(common::e3) - omega.cross(x.v);
  dx.segment<3>(Q) = omega;
}


void EKF::getFG(const Stated &x, const uVector &u, dxMatrix &F, nuMatrix &G)
{
  F.setZero();
  F.block<3,3>(DP,DV) = x.q.inverse().R();
  F.block<3,3>(DP,DQ) = -x.q.inverse().R() * common::skew(x.v);
  F.block<3,3>(DV,DV) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DV,DQ) = common::skew(common::gravity * x.q.rotp(common::e3));
  F.block<3,3>(DV,DBA) = -common::I_3x3;
  F.block<3,3>(DV,DBG) = -common::skew(x.v);
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DQ,DBG) = -common::I_3x3;

  G.setZero();
  G.block<3,3>(DV,UA) = -common::I_3x3;
  G.block<3,3>(DV,UG) = -common::skew(x.v);
  G.block<3,3>(DQ,UG) = -common::I_3x3;
}


void EKF::log(const double &t)
{
  xVector x = x_.toEigen();
  dxVector P_diag = P_.diagonal();
  state_log_.write((char*)&t, sizeof(double));
  state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  cov_log_.write((char*)&t, sizeof(double));
  cov_log_.write((char*)P_diag.data(), P_diag.rows() * sizeof(double));
}



} // namespace ekf
