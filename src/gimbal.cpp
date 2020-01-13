#include "gimbal.h"

namespace gimbal
{


Gimbal::Gimbal()  : t_prev_(0.0) {}


Gimbal::Gimbal(const std::string &filename, const std::default_random_engine& rng)
  : t_prev_(0.0)
{
  load(filename, rng);
}


Gimbal::~Gimbal() {}


void Gimbal::load(const std::string &filename, const std::default_random_engine& rng)
{
  // Instantiate Sensors, Controller, and Estimator classes
  common::get_yaml_node("name", filename, name_);
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("omega_f", filename, omega_f_);
  common::get_yaml_node("max_roll", filename, max_roll_);
  common::get_yaml_node("max_pitch", filename, max_pitch_);
  vw_.setZero();
  u_.setZero();

  // Load other modules (e.g. controller, estimator, sensors)
  ctrl_.load(filename, name_);
  sensors_.load(filename, rng, name_);
  ekf_.load(filename, name_);

  // Load all Gimbal parameters
  double roll, pitch, yaw;
  Vector3d rpy;
  x_.setZero();
  common::get_yaml_eigen_diag("inertia", filename, inertia_matrix_);
  common::get_yaml_eigen_diag("K_friction", filename, Kf_);
  common::get_yaml_eigen("p_bg", filename, x_.p);
  common::get_yaml_eigen("p_gcg", filename, p_gcg_);
  common::get_yaml_eigen("rpy", filename, rpy);
  common::get_yaml_eigen("omega", filename, x_.omega);
  inertia_inv_ = inertia_matrix_.inverse();
  x_.q = quat::Quatd(rpy(0), rpy(1), rpy(2));

  // Initialize loggers and log initial data
  std::stringstream ss_s, ss_e;
  ss_s << "/tmp/" << name_ << "_true_state.log";
  ss_e << "/tmp/" << name_ << "_euler_angles.log";
  state_log_.open(ss_s.str());
  euler_log_.open(ss_e.str());
  log(0);
}


void Gimbal::update(const double &t, const vehicle::Stated& aircraft_state, const sensors::Sensors &aircraft_sensors, environment::Environment& env)
{
  // Time step for controller later
  double dt = t - t_prev_;
  t_prev_ = t;
  
  // Integrate rotational states
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta
    vehicle::rk4<3>(std::bind(&Gimbal::f, this,
                    std::placeholders::_1,std::placeholders::_2,
                    std::placeholders::_3,std::placeholders::_4),
                    dt, x_, u_, vw_, dx_);
  }
  else
  {
    // Euler
    f(x_, u_, vw_, dx_);
    dx_ *= dt;
  }
  x_ += dx_;

  // Unpack aircraft states at new time for readability
  Vector3d p_bI_I = aircraft_state.p;
  Vector3d v_bI_b = aircraft_state.v;
  Vector3d vdot_bI_b = aircraft_state.lin_accel;
  quat::Quatd q_Ib = aircraft_state.q;
  Vector3d omega_bI_b = aircraft_state.omega;
  Vector3d omegadot_bI_b = aircraft_state.ang_accel;

  // Compute current translational states based on aircraft states
  Vector3d p_Ig = p_bI_I + q_Ib.rota(x_.p);
  Vector3d v_gI_g = x_.q.rotp(v_bI_b + omega_bI_b.cross(x_.p));
  Vector3d vdot_gI_g = x_.q.rotp(vdot_bI_b + omega_bI_b.cross(omega_bI_b.cross(x_.p)) + omegadot_bI_b.cross(x_.p));

  // Update controller
  vehicle::Stated xhat = ekf_.getState();
  Eigen::Vector3d dir_c = common::e1;
  if (env.getVehiclePositions().size() > 1) // Track second vehicle in multiple vehicle simulation
    dir_c = (env.getVehiclePositions()[1] - p_Ig).normalized();
  ctrl_.computeControl(t, xhat.q, dir_c, xhat.omega, x_.q, u_);

  // Update angular acceleration
  f(x_, u_, vw_, dx_);
  x_.ang_accel = dx_.segment<3>(vehicle::DW);

  // Saturate gimbal roll/pitch angles and zero out velocities/accelerations
  saturateRollPitch(omega_bI_b, omegadot_bI_b);

  // Create true gimbal state w.r.t. inertial frame
  vehicle::Stated x_Ig;
  x_Ig.p = p_Ig;
  x_Ig.v = v_gI_g;
  x_Ig.lin_accel = vdot_gI_g;
  x_Ig.q = q_Ib * x_.q;
  x_Ig.omega = x_.q.rotp(omega_bI_b) + x_.omega;
  x_Ig.ang_accel = x_.q.rotp(omegadot_bI_b) + x_.ang_accel;

  // Update the estimator, then collect new sensor measurements
  quat::Quatd q_bg = quat::Quatd::from_euler(sensors_.getRollAngle(), sensors_.getPitchAngle(), sensors_.getYawAngle());
  ekf_.run(t, sensors_, aircraft_sensors_, x_.p, q_bg, vw_, x_Ig);
  sensors_.updateMeasurements(t, x_Ig, env);
  sensors_.updateEncoders(t, x_);

  // FIX ACCELEROMETER MEASUREMENT
  // Because the gimbal is able to rotate independent of the aircraft, the traditionally computed 
  // accelerometer measurement is incorrect. Aircraft linear/angular velocities and angular acceleration
  // contribute to measured accelaration on the gimbal, NOT the gimbal counterparts.
  Vector3d p_bu = sensors_.getBodyToImuTranslation();
  quat::Quatd q_bu = sensors_.getBodyToImuRotation();
  Vector3d v = x_.q.rotp(v_bI_b);
  Vector3d omega = x_.q.rotp(omega_bI_b);
  Vector3d ang_accel = x_.q.rotp(omegadot_bI_b);
  Vector3d bad_part = q_bu.rotp(x_Ig.omega.cross(x_Ig.v) + x_Ig.omega.cross(x_Ig.omega.cross(p_bu)) + x_Ig.ang_accel.cross(p_bu));
  Vector3d good_part = q_bu.rotp(omega.cross(v) + omega.cross(omega.cross(p_bu)) + ang_accel.cross(p_bu));
  sensors_.setImuAccel(sensors_.getImuAccel() - bad_part + good_part);

  // Store aircraft sensors so that EKF propagates to current time using previous measurements
  aircraft_state_ = aircraft_state;
  aircraft_sensors_ = aircraft_sensors;

  // Log all of that juicy data
  log(t);
}


void Gimbal::f(const vehicle::Stated& x, const Vector3d& u,
               const Vector3d& vw, vehicle::dxVector& dx)
{
  Vector3d tau_f;
  if (x.omega.norm() < omega_f_)
    tau_f = Kf_ * x.omega;
  else
    tau_f.setZero();

  quat::Quatd q_Ig = aircraft_state_.q * x.q;
  Vector3d tau_g = p_gcg_.cross(mass_*common::gravity*q_Ig.rotp(common::e3));

  quat::Quatd q_g2_g = quat::Quatd(x.q.roll(), 0, 0);
  quat::Quatd q_g1_g = quat::Quatd(x.q.roll(), x.q.pitch(), 0);
  Vector3d tau_m = u(0) * common::e1 +
                   u(1) * q_g2_g.rotp(common::e2) +
                   u(2) * q_g1_g.rotp(common::e3);

  dx.setZero();
  dx.segment<3>(vehicle::DQ) = x.omega;
  dx.segment<3>(vehicle::DW) = inertia_inv_ * (tau_f + tau_g + tau_m - x.omega.cross(inertia_matrix_ * x.omega));
}


void Gimbal::saturateRollPitch(const Vector3d& omega_bI_b, const Vector3d& omegadot_bI_b)
{
  double roll = x_.q.roll();
  double pitch = x_.q.pitch();
  if (abs(roll) > max_roll_ && abs(pitch) > max_pitch_)
  {
    roll = max_roll_;
    pitch = max_pitch_;

    Matrix3d R = R_euler_to_body(roll, pitch);
    Vector3d euler_dot = R.transpose() * x_.omega;
    Vector3d euler_dotdot = R.transpose() * x_.ang_accel;

    euler_dot(0) = 0;
    euler_dot(1) = 0;
    euler_dotdot(0) = 0;
    euler_dotdot(1) = 0;

    x_.q = quat::Quatd::from_euler(roll, pitch, x_.q.yaw());
    x_.omega = R * euler_dot;
    x_.ang_accel = R * euler_dotdot;
  }
  if (abs(roll) > max_roll_)
  {
    roll = max_roll_;

    Matrix3d R = R_euler_to_body(roll, pitch);
    Vector3d euler_dot = R.transpose() * x_.omega;
    Vector3d euler_dotdot = R.transpose() * x_.ang_accel;

    euler_dot(0) = 0;
    euler_dotdot(0) = 0;

    x_.q = quat::Quatd::from_euler(roll, pitch, x_.q.yaw());
    x_.omega = R * euler_dot;
    x_.ang_accel = R * euler_dotdot;
  }
  if (abs(pitch) > max_pitch_)
  {
    pitch = max_pitch_;

    Matrix3d R = R_euler_to_body(roll, pitch);
    Vector3d euler_dot = R.transpose() * x_.omega;
    Vector3d euler_dotdot = R.transpose() * x_.ang_accel;

    euler_dot(1) = 0;
    euler_dotdot(1) = 0;

    x_.q = quat::Quatd::from_euler(roll, pitch, x_.q.yaw());
    x_.omega = R * euler_dot;
    x_.ang_accel = R * euler_dotdot;
  }
}


Matrix3d Gimbal::R_euler_to_body(const double& roll, const double& pitch)
{
  Matrix3d R = Matrix3d::Identity();
  R(0,2) = -sin(pitch);
  R(1,1) = cos(roll);
  R(1,2) = sin(roll) * cos(pitch);
  R(2,1) = -sin(roll);
  R(2,2) = cos(roll) * cos(pitch);
  return R;
}


void Gimbal::log(const double &t)
{
  // Write data to binary files and plot in another program
  state_log_.log(t);
  state_log_.logMatrix(x_.p, x_.v, x_.lin_accel, x_.q.elements(), x_.omega, x_.ang_accel);
  euler_log_.log(t);
  euler_log_.logMatrix(x_.q.euler());
}


} // namespace gimbal
