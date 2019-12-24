#include "gmbl_ekf.h"


namespace gmbl_ekf
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
  double origin_lat, origin_lon, origin_alt, origin_temp;
  common::get_yaml_node("origin_latitude", filename, origin_lat);
  common::get_yaml_node("origin_longitude", filename, origin_lon);
  common::get_yaml_node("origin_altitude", filename, origin_alt);
  common::get_yaml_node("origin_temperature", filename, origin_temp);
  rho_ = common::airDense(origin_alt, origin_temp);
  X_ecef2ned_ = WGS84::x_ecef2ned(WGS84::lla2ecef(Vector3d(origin_lat,origin_lon,origin_alt)));

  Vector4d q_bu;
  common::get_yaml_eigen_diag("ekf_R_gps", filename, R_gps_);
  common::get_yaml_eigen("p_bu", filename, p_bu_);
  common::get_yaml_eigen("q_bu", filename, q_bu);
  common::get_yaml_node("ekf_R_mag", filename, R_mag_);
  q_bu_ = quat::Quatd(q_bu);

  // GPS covariance is defined in local NED frame, transform it to ECEF
  R_gps_ = X_ecef2ned_.q_.inverse().R() * R_gps_ * X_ecef2ned_.q_.R();

  // Stuff needed for mag update
  mnp_ecef_ = (WGS84::lla2ecef(Vector3d(common::MNP_lat*M_PI/180.0, common::MNP_lon*M_PI/180.0, 0.0))).normalized();
  Vector3d axis = (common::e3.cross(mnp_ecef_)).normalized();
  double angle = common::angDiffBetweenVecs(common::e3, mnp_ecef_);
  q_ecef_to_mnp_.from_axis_angle(axis, angle);
  last_gps_pos_.setZero();

  // Randomly initialize pos/vel/att
  bool random_init;
  double v0_err, q0_err;
  common::get_yaml_node("ekf_random_init", filename, random_init);
  common::get_yaml_node("ekf_v0_err", filename, v0_err);
  common::get_yaml_node("ekf_q0_err", filename, q0_err);
  if (random_init)
  {
    x_.v += v0_err * Vector3d::Random();
    Vector3d q0_err3 = q0_err * Vector3d::Random();
    q0_err3(2) *= 2.0;
    x_.q += q0_err3;
  }

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
    propagate(t, sensors.imu_.vec());

  // Apply updates
  if (sensors.new_gps_meas_ && t > 0)
    updateGPS(sensors.gps_.vec());
  if (sensors.new_mag_meas_ && t > 0)
    updateMag(sensors.mag_.field);

  // Log data
  logTruth(t, sensors, vw, x_true);
  logEst(t);
}


void EKF::propagate(const double &t, const uVector& imu)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  if (t > 0)
  {
    // Propagate the covariance - guarantee positive-definite P with discrete propagation
    getFG(x_, imu, F_, G_);
    A_ = I_NUM_DOF_ + F_ * dt + F_ * F_ * dt * dt / 2.0;
    B_ = (I_NUM_DOF_ + F_ * dt / 2.0 + F_ * F_ * dt * dt / 6.0) * G_ * dt;
    P_ = A_ * P_ * A_.transpose() + B_ * Qu_ * B_.transpose() + Qx_;

    // Euler integration
    f(x_, imu, xdot_);
    x_ += xdot_ * dt;
  }
}


void EKF::updateGPS(const Matrix<double,6,1> &z)
{
  // Store GPS position for magnetometer update
  last_gps_pos_ = z.topRows<3>();

  // Measurement model and matrix
  Vector3d h = X_ecef2ned_.q_.rota(x_.v);

  Matrix<double,3,NUM_DOF> H = Matrix<double,3,NUM_DOF>::Zero();
  H.block<3,3>(0,DV) = X_ecef2ned_.q_.inverse().R();

  // Kalman gain and update
  Matrix<double,NUM_DOF,3> K = P_ * H.transpose() * (R_gps_ + H * P_ * H.transpose()).inverse();
  x_ += K * (z.bottomRows<3>() - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_gps_ * K.transpose();
}


void EKF::updateMag(const Vector3d &z)
{
  // Make sure GPS runs first
  if (last_gps_pos_.sum() < 1e-6)
    return;

  // Calculate magnetic field vector at current location (currently uses simple dipole model)
  // NOTE: MNP coordinates are rotated by the shortest rotation between ECEF Z-axis and the magnetic north pole
  double theta = common::angDiffBetweenVecs(last_gps_pos_, mnp_ecef_);
  double Re_r = common::R_earth / last_gps_pos_.norm();
  double Re_r3 = Re_r * Re_r * Re_r;
  double Br = -2.0 * common::B0 * Re_r3 * cos(theta);
  double Btheta = -common::B0 * Re_r3 * sin(theta);
  Vector3d B_mnp(-Btheta, 0.0, -Br);
  Vector3d m0 = x_.q.rotp(X_ecef2ned_.q_.rotp(q_ecef_to_mnp_.rota(B_mnp)));

  // Pre-calcs
  double mx = common::e1.dot(x_.q.rota(m0));
  double my = common::e2.dot(x_.q.rota(m0));
  double zx = common::e1.dot(x_.q.rota(z));
  double zy = common::e2.dot(x_.q.rota(z));
  RowVector3d dmx_dq = -common::e1.transpose() * common::skew(x_.q.rota(m0));
  RowVector3d dmy_dq = -common::e2.transpose() * common::skew(x_.q.rota(m0));
  
  // Measurement model and matrix
  double h =  atan2(my, mx) + x_.bm;

  Matrix<double,1,NUM_DOF> H = Matrix<double,1,NUM_DOF>::Zero();
  H.segment<3>(DQ) = (mx*dmy_dq - my*dmx_dq)/(mx*mx + my*my);
  H(DBM) = 1.0;

  // Kalman gain and update
  double z_psi = atan2(zy, zx);
  Matrix<double,NUM_DOF,1> K = P_ * H.transpose() / (R_mag_ + H * P_ * H.transpose());
  x_ += K * (z_psi - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_mag_ * K.transpose();
}


void EKF::f(const Stated &x, const uVector &u, dxVector &dx)
{
  dx.setZero();
  dx.segment<3>(DV) = x.sa * x.q.rota(u.segment<3>(UA)) + common::gravity * common::e3;
  dx.segment<3>(DQ) = u.segment<3>(UG) - x.bg;
}


void EKF::getFG(const Stated &x, const uVector &u, dxMatrix &F, nuMatrix &G)
{
  F.setZero();
  F.block<3,3>(DV,DQ) = -x.sa * x.q.inverse().R() * common::skew(u.segment<3>(UA));
  F.block<3,1>(DV,DSA) = x.q.rota(u.segment<3>(UA));
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DQ,DBG) = -common::I_3x3;

  G.setZero();
  G.block<3,3>(DV,UA) = -x.sa * x.q.inverse().R();
  G.block<3,3>(DQ,UG) = -common::I_3x3;
}


void EKF::logTruth(const double &t, const sensors::Sensors &sensors, const Vector3d& vw, const vehicle::Stated& x_true)
{
  double accel_bias = sensors.getAccelBias().norm();
  double mag_bias = sensors.getMagBias().norm();
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x_true.q.rota(x_true.v).data(), 3 * sizeof(double));
  true_state_log_.write((char*)x_true.q.euler().data(), 3 * sizeof(double));
  true_state_log_.write((char*)&accel_bias, sizeof(double)); // This is wrong. Need to compare with magnitude of de-noised measurement.
  true_state_log_.write((char*)sensors.getGyroBias().data(), 3 * sizeof(double));
  true_state_log_.write((char*)&mag_bias, sizeof(double)); // This is wrong. Should be error in heading.
}


void EKF::logEst(const double &t)
{
  ekf_state_log_.write((char*)&t, sizeof(double));
  ekf_state_log_.write((char*)x_.v.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)x_.q.euler().data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)&x_.sa, sizeof(double));
  ekf_state_log_.write((char*)x_.bg.data(), 3 * sizeof(double));
  ekf_state_log_.write((char*)&x_.bm, sizeof(double));

  dxVector P_diag = P_.diagonal();
  cov_log_.write((char*)&t, sizeof(double));
  cov_log_.write((char*)P_diag.data(), P_diag.rows() * sizeof(double));
}


vehicle::Stated EKF::getState() const
{
  vehicle::Stated x;
  x.p = Vector3d::Constant(NAN);
  x.v = x_.q.rotp(x_.v);
  x.q = x_.q;
  x.omega = xdot_.segment<3>(DQ);
  return x;
}



} // namespace gmbl_ekf
