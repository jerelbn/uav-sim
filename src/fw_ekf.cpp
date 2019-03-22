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
  common::get_yaml_eigen("ekf_x0", filename, x0);
  common::get_yaml_eigen_diag("ekf_P0", filename, P_);
  common::get_yaml_eigen_diag("ekf_Qx", filename, Qx_);
  common::get_yaml_eigen_diag("ekf_Qu", filename, Qu_);
  x_ = State<double>(x0);
  I_NUM_DOF_.setIdentity();

  // Load sensor parameters
  double origin_alt, origin_temp;
  common::get_yaml_node("origin_altitude", filename, origin_alt);
  common::get_yaml_node("origin_temperature", filename, origin_temp);
  rho_ = common::airDense(origin_alt, origin_temp);

  double pitot_az, pitot_el, wv_roll;
  Vector4d q_ub;
  common::get_yaml_eigen_diag("ekf_R_gps", filename, R_gps_);
  common::get_yaml_eigen("p_ub", filename, p_ub_);
  common::get_yaml_eigen("q_ub", filename, q_ub);
  common::get_yaml_node("pitot_azimuth", filename, pitot_az);
  common::get_yaml_node("pitot_elevation", filename, pitot_el);
  common::get_yaml_node("wvane_roll", filename, wv_roll);
  common::get_yaml_node("ekf_R_baro", filename, R_baro_);
  common::get_yaml_node("ekf_R_pitot", filename, R_pitot_);
  common::get_yaml_node("ekf_R_wvane", filename, R_wv_);
  q_u2b_ = quat::Quatd(q_ub);
  q_u2pt_ = quat::Quatd(0, pitot_el, pitot_az);
  q_u2wv_ = quat::Quatd(wv_roll, 0, 0);

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
  if (sensors.new_baro_meas_)
    updateBaro(sensors.baro_);
  if (sensors.new_pitot_meas_)
    updatePitot(sensors.pitot_);
  if (sensors.new_wvane_meas_)
    updateWVane(sensors.wvane_);

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


void EKF::updateBaro(const double &z)
{
  // Measurement model and matrix
  double h = rho_ * common::gravity * -x_.p(2) + x_.bb;

  Matrix<double,1,NUM_DOF> H = Matrix<double,1,NUM_DOF>::Zero();
  H(DP+2) = -rho_ * common::gravity;
  H(DBB) = 1.0;

  // Kalman gain and update
  Matrix<double,NUM_DOF,1> K = P_ * H.transpose() / (R_baro_ + H * P_ * H.transpose());
  x_ += K * (z - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_baro_ * K.transpose();
}


void EKF::updatePitot(const double &z)
{
  // Measurement model and matrix
  double Va = common::e1.dot(q_u2pt_.rotp(x_.q.rotp(x_.v - x_.vw)));
  double h = rho_ / 2.0 * Va * Va;

  Matrix<double,1,NUM_DOF> H = Matrix<double,1,NUM_DOF>::Zero();
  double eps = 1e-5;
  for (int i = 0; i < H.cols(); ++i)
  {
    Stated xp = x_ + I_NUM_DOF_.col(i) * eps;
    Stated xm = x_ + I_NUM_DOF_.col(i) * -eps;
    double Vap = common::e1.dot(q_u2pt_.rotp(xp.q.rotp(xp.v - xp.vw)));
    double Vam = common::e1.dot(q_u2pt_.rotp(xm.q.rotp(xm.v - xm.vw)));
    double hp = rho_ / 2.0 * Vap * Vap;
    double hm = rho_ / 2.0 * Vam * Vam;
    H(i) = (hp - hm) / (2.0 * eps);
  }

  // Kalman gain and update
  Matrix<double,NUM_DOF,1> K = P_ * H.transpose() / (R_pitot_ + H * P_ * H.transpose());
  x_ += K * (z - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_pitot_ * K.transpose();
}


void EKF::updateWVane(const double &z)
{
  // Measurement model and matrix
  double h = asin(common::e1.dot(q_u2wv_.rotp(x_.q.rotp(x_.v - x_.vw))) / (x_.v - x_.vw).norm());

  Matrix<double,1,NUM_DOF> H = Matrix<double,1,NUM_DOF>::Zero();
  double eps = 1e-5;
  for (int i = 0; i < H.cols(); ++i)
  {
    Stated xp = x_ + I_NUM_DOF_.col(i) * eps;
    Stated xm = x_ + I_NUM_DOF_.col(i) * -eps;
    double hp = asin(common::e1.dot(q_u2wv_.rotp(xp.q.rotp(xp.v - xp.vw))) / (xp.v - xp.vw).norm());
    double hm = asin(common::e1.dot(q_u2wv_.rotp(xm.q.rotp(xm.v - xm.vw))) / (xm.v - xm.vw).norm());
    H(i) = (hp - hm) / (2.0 * eps);
  }

  // Kalman gain and update
  Matrix<double,NUM_DOF,1> K = P_ * H.transpose() / (R_wv_ + H * P_ * H.transpose());
  x_ += K * (z - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_wv_ * K.transpose();
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
