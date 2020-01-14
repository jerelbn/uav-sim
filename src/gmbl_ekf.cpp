#include "gmbl_ekf.h"


namespace gmbl_ekf
{


EKF::EKF() : t_prev_(0)
{
  imu_prev_.setZero();
}


EKF::EKF(const string &filename, const string &name) : t_prev_(0)
{
  imu_prev_.setZero();
  load(filename, name);
}


EKF::~EKF() {}


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

  Vector4d q_bu, q_gcb;
  common::get_yaml_eigen_diag("ekf_R_gps", filename, R_gps_);
  common::get_yaml_eigen_diag("ekf_R_cam", filename, R_cam_);
  common::get_yaml_eigen("p_bu", filename, p_bu_);
  common::get_yaml_eigen("p_bcb", filename, p_gcb_);
  common::get_yaml_eigen("q_bu", filename, q_bu);
  common::get_yaml_eigen("q_bcb", filename, q_gcb);
  common::get_yaml_node("ekf_R_mag", filename, R_mag_);
  common::get_yaml_node("ekf_mag_initialized", filename, mag_initialized_);
  q_bu_ = quat::Quatd(q_bu);
  q_gcb_ = quat::Quatd(q_gcb);

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

  std::string logname_truth, logname_est, logname_cov;
  common::get_yaml_node("logname_ekf_truth", filename, logname_truth);
  common::get_yaml_node("logname_ekf_est", filename, logname_est);
  common::get_yaml_node("logname_ekf_cov", filename, logname_cov);
  true_state_log_.open(logname_truth);
  ekf_state_log_.open(logname_est);
  cov_log_.open(logname_cov);
}


void EKF::run(const double &t, const sensors::Sensors &gimbal_sensors, const sensors::Sensors &aircraft_sensors,
              const quat::Quatd& q_bg, const vehicle::Stated& xg_true, const vehicle::Stated& xac_true)
{
  // Propagate the state and covariance to the current time step
  if (gimbal_sensors.new_imu_meas_)
  {
    propagate(t, imu_prev_);
    imu_prev_ = gimbal_sensors.imu_.vec();
  }

  // Initialize mag bias at truth for now. Should to mag calibration later.
  if (!mag_initialized_ && last_gps_pos_.norm() > 1e-6)
  {
    Vector3d m_I_true = magFieldNED(last_gps_pos_, X_ecef2ned_.q_, mnp_ecef_, q_ecef_to_mnp_);
    Vector3d m_I_biased = m_I_true + aircraft_sensors.getMagBias();
    Vector3d m_proj_true = (common::I_3x3 - common::e3 * common::e3.transpose()) * m_I_true;
    Vector3d m_proj_biased = (common::I_3x3 - common::e3 * common::e3.transpose()) * m_I_biased;
    Vector3d true_x_biased = m_proj_true.cross(m_proj_biased);
    x_.bm = common::sign(true_x_biased(2)) * acos(m_proj_true.dot(m_proj_biased) / (m_proj_true.norm()*m_proj_biased.norm()));
    mag_initialized_ = true;
  }

  // Apply updates
  if (aircraft_sensors.new_gps_meas_ && t > 0)
    updateGPS(aircraft_sensors.gps_.vec()); // Gimbal is close enough to aircraft. Don't worry about tranforming measurement to Gimbal frame right now.
  if (aircraft_sensors.new_mag_meas_ && t > 0)
    updateMag(aircraft_sensors.mag_.field, q_bg);
  if (gimbal_sensors.new_camera_meas_ && t > 0)
    updateCam(xac_true, xg_true);

  // Log data
  logTruth(t, gimbal_sensors, aircraft_sensors, xg_true, xac_true);
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
    getF(x_, imu, F_);
    getG(x_, imu, G_);
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
  Vector3d h;
  Matrix<double,3,NUM_DOF> H;
  h_gps(x_, X_ecef2ned_.q_, h);
  getH_gps(x_, X_ecef2ned_.q_.R(), H);

  // Kalman gain and update
  Matrix<double,NUM_DOF,3> K = P_ * H.transpose() * (R_gps_ + H * P_ * H.transpose()).inverse();
  x_ += K * (z.bottomRows<3>() - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_gps_ * K.transpose();
}


void EKF::updateMag(const Vector3d &z, const quat::Quatd &q_bg)
{
  // Make sure GPS runs first
  if (last_gps_pos_.norm() < 1e-6)
    return;
  
  // Measurement model and matrix
  double h;
  Matrix<double,1,NUM_DOF> H;
  h_mag(x_, last_gps_pos_, X_ecef2ned_.q_, mnp_ecef_, q_ecef_to_mnp_, h);
  getH_mag(x_, last_gps_pos_, X_ecef2ned_.q_, mnp_ecef_, q_ecef_to_mnp_, H);

  // Kalman gain and update
  quat::Quatd q_g1_to_g = quat::Quatd::from_euler(x_.q.roll(), x_.q.pitch(), 0);
  double zx = common::e1.dot(q_g1_to_g.rota(q_bg.rotp(z)));
  double zy = common::e2.dot(q_g1_to_g.rota(q_bg.rotp(z)));
  double z_psi = atan2(zy, zx);
  Matrix<double,NUM_DOF,1> K = P_ * H.transpose() / (R_mag_ + H * P_ * H.transpose());
  x_ += K * (z_psi - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_mag_ * K.transpose();
}


void EKF::updateCam(const vehicle::Stated& xac_true, const vehicle::Stated& xg_true)
{
  // Compute true gimbal attitude
  quat::Quatd q_Ig = xac_true.q * xg_true.q;

  // Establish new keyframe
  Vector3d dq = x_.q - qk_;
  if (dq.norm() > 0.5)
  {
    qk_ = x_.q;
    qkt_ = q_Ig;
    return;
  }

  // Compute measurement
  Vector3d noise = 0.35 * Vector3d::Random();
  Vector3d dqk = q_Ig - qkt_ + noise;
  quat::Quatd z = qk_ + dqk;

  // Compute measurement model
  quat::Quatd h;
  Matrix<double,3,NUM_DOF> H;
  h_cam(x_, h);
  getH_cam(x_, H);

  // Kalman gain and update
  Matrix<double,NUM_DOF,3> K = P_ * H.transpose() * (R_cam_ + H * P_ * H.transpose()).inverse();
  x_ += K * (z - h);
  P_ = (I_NUM_DOF_ - K * H) * P_ * (I_NUM_DOF_ - K * H).transpose() + K * R_cam_ * K.transpose();
}


void EKF::f(const Stated &x, const uVector &u, dxVector &dx)
{
  dx.setZero();
  dx.segment<3>(DV) = x.sa * x.q.rota(u.segment<3>(UA)) + common::gravity * common::e3;
  dx.segment<3>(DQ) = u.segment<3>(UG) - x.bg;
}


void EKF::h_gps(const Stated &x, const quat::Quatd &q_ecef2ned, Vector3d& h)
{
  h = q_ecef2ned.rota(x.v);
}


void EKF::h_mag(const Stated &x, const Vector3d &pos_ecef, const quat::Quatd &q_ecef2ned, const Vector3d &mnp_ecef, const quat::Quatd &q_ecef2mnp, double& h)
{
  // Calculate magnetic field vector at current location (currently uses simple dipole model)
  Vector3d m_I = magFieldNED(pos_ecef, q_ecef2ned, mnp_ecef, q_ecef2mnp);

  // Get heading relative to magnetic field
  quat::Quatd q_I_to_g1 = quat::Quatd::from_euler(0, 0, x.q.yaw());
  Vector3d m_g1 = q_I_to_g1.rotp(m_I);
  double psi_m = atan2(m_g1(1), m_g1(0));

  // Compute output
  h = psi_m + x.bm;
}


void EKF::h_cam(const Stated &x, quat::Quatd& h)
{
  h = x.q;
}


void EKF::getF(const Stated &x, const uVector &u, dxMatrix &F)
{
  F.setZero();
  F.block<3,3>(DV,DQ) = -x.sa * x.q.inverse().R() * common::skew(u.segment<3>(UA));
  F.block<3,1>(DV,DSA) = x.q.rota(u.segment<3>(UA));
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DQ,DBG) = -common::I_3x3;
}


void EKF::getG(const Stated &x, const uVector &u, nuMatrix &G)
{
  G.setZero();
  G.block<3,3>(DV,UA) = -x.sa * x.q.inverse().R();
  G.block<3,3>(DQ,UG) = -common::I_3x3;
}


// This is constant and should be a class variable.
void EKF::getH_gps(const Stated &x, const Matrix3d &R_ecef2ned, Matrix<double,3,NUM_DOF> &H)
{
  H.setZero();
  H.block<3,3>(0,DV) = R_ecef2ned.transpose();
}


void EKF::getH_mag(const Stated &x, const Vector3d &pos_ecef, const quat::Quatd &q_ecef2ned, const Vector3d &mnp_ecef, const quat::Quatd &q_ecef2mnp, Matrix<double,1,NUM_DOF> &H)
{
  // Calculate magnetic field vector at current location (currently uses simple dipole model)
  Vector3d m_I = magFieldNED(pos_ecef, q_ecef2ned, mnp_ecef, q_ecef2mnp);

  // Rotate magnetic field by aircraft heading
  quat::Quatd q_psi = quat::Quatd::from_euler(0, 0, x.q.yaw());
  Vector3d m_g1 = q_psi.rotp(m_I);

  // Numerical derivative for unknown piece of precalculations
  Matrix3d dm_dq;
  static double eps = 1e-5;
  for (int i = 0; i < dm_dq.cols(); ++i)
  {
    quat::Quatd qp = x.q + eps * common::I_3x3.col(i);
    quat::Quatd qm = x.q + -eps * common::I_3x3.col(i);

    quat::Quatd q_psip = quat::Quatd::from_euler(0,0,qp.yaw());
    quat::Quatd q_psim = quat::Quatd::from_euler(0,0,qm.yaw());

    Vector3d mp = q_psip.rotp(m_I);
    Vector3d mm = q_psim.rotp(m_I);

    // Derivative of model w.r.t. x
    dm_dq.col(i) = (mp - mm) / (2.0 * eps);
  }

  // Precalcs
  double mx = m_g1(0);
  double my = m_g1(1);
  RowVector3d dmx_dq = common::e1.transpose() * dm_dq;
  RowVector3d dmy_dq = common::e2.transpose() * dm_dq;
  
  // Populate matrix
  H.setZero();
  H.segment<3>(DQ) = (mx*dmy_dq - my*dmx_dq)/(mx*mx + my*my);
  H(DBM) = 1.0;
}


// This is constant and should be a class variable.
void EKF::getH_cam(const Stated &x, Matrix<double,3,NUM_DOF> &H)
{
  H.setZero();
  H.block<3,3>(0,DQ) = common::I_3x3;
}


Vector3d EKF::magFieldNED(const Vector3d &pos_ecef, const quat::Quatd &q_ecef2ned, const Vector3d &mnp_ecef, const quat::Quatd &q_ecef2mnp)
{
  // Calculate magnetic field vector at current location (currently uses simple dipole model)
  // NOTE: MNP coordinates are rotated by the shortest rotation between ECEF Z-axis and the magnetic north pole
  double theta = common::angDiffBetweenVecs(pos_ecef, mnp_ecef);
  double Re_r = common::R_earth / pos_ecef.norm();
  double Re_r3 = Re_r * Re_r * Re_r;
  double Br = -2.0 * common::B0 * Re_r3 * cos(theta);
  double Btheta = -common::B0 * Re_r3 * sin(theta);
  Vector3d B_mnp(-Btheta, 0.0, -Br);

  return q_ecef2ned.rotp(q_ecef2mnp.rota(B_mnp));
}


vehicle::Stated EKF::stateRelToBody(const vehicle::Stated& x_Ib) const
{
  vehicle::Stated x;
  x.p = Vector3d::Constant(NAN);
  x.v.setZero();
  x.lin_accel.setConstant(NAN);
  x.q = x_Ib.q.inverse() * x_.q;
  x.omega = xdot_.segment<3>(DQ) - x_Ib.omega;
  x.ang_accel.setConstant(NAN);
  return x;
}


void EKF::logTruth(const double &t, const sensors::Sensors &gimbal_sensors, const sensors::Sensors &aircraft_sensors,
                   const vehicle::Stated& xg_true, const vehicle::Stated& xac_true)
{
  // Compute true accelerometer scale error
  Vector3d accel_true = gimbal_sensors.getImuAccel() - gimbal_sensors.getAccelBias() - gimbal_sensors.getAccelNoise();
  Vector3d accel_biased = accel_true + gimbal_sensors.getAccelBias();
  double accel_scale = accel_true.norm() / accel_biased.norm();
  
  // Compute true heading bias
  Vector3d m_I_true = magFieldNED(last_gps_pos_, X_ecef2ned_.q_, mnp_ecef_, q_ecef_to_mnp_);
  Vector3d m_I_biased = m_I_true + aircraft_sensors.getMagBias();
  double psi_d_true = atan2(m_I_true(1), m_I_true(0));
  double psi_d_biased = atan2(m_I_biased(1), m_I_biased(0));
  double mag_bias = psi_d_biased - psi_d_true;

  // Create true gimbal states w.r.t. inertial frame
  quat::Quatd q_Ig = xac_true.q * xg_true.q;
  Vector3d v_gI_I = xac_true.q.rota(xac_true.v + xac_true.omega.cross(xg_true.p));

  true_state_log_.log(t);
  true_state_log_.logMatrix(v_gI_I, q_Ig.euler());
  true_state_log_.log(accel_scale);
  true_state_log_.logMatrix(gimbal_sensors.getGyroBias());
  true_state_log_.log(mag_bias);
}


void EKF::logEst(const double &t)
{
  ekf_state_log_.log(t);
  ekf_state_log_.logMatrix(x_.v, x_.q.euler());
  ekf_state_log_.log(x_.sa);
  ekf_state_log_.logMatrix(x_.bg);
  ekf_state_log_.log(x_.bm);

  dxVector P_diag = P_.diagonal();
  cov_log_.log(t);
  cov_log_.logMatrix(P_diag);
}



} // namespace gmbl_ekf
