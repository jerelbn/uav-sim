#include "gimbal.h"

namespace gimbal
{


Gimbal::Gimbal()  : t_prev_(0.0) {}


Gimbal::Gimbal(const std::string &filename, const bool& use_random_seed)
  : t_prev_(0.0)
{
  load(filename, use_random_seed);
}


Gimbal::~Gimbal() {}


void Gimbal::load(const std::string &filename, const bool& use_random_seed)
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
  sensors_.load(filename, use_random_seed, name_);
  ekf_.load(filename, name_);

  // Load all Gimbal parameters
  double roll, pitch, yaw;
  Vector3d rpy;
  common::get_yaml_eigen_diag("inertia", filename, inertia_matrix_);
  common::get_yaml_eigen_diag("K_friction", filename, Kf_);
  common::get_yaml_eigen("p_bg", filename, p_bg_);
  common::get_yaml_eigen("p_gcg", filename, p_gcg_);
  common::get_yaml_eigen("rpy", filename, rpy);
  common::get_yaml_eigen("omega", filename, x_.omega);
  inertia_inv_ = inertia_matrix_.inverse();
  x_.q = quat::Quatd(rpy(0), rpy(1), rpy(2));
  x_.ang_accel.setZero();

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

  // Unpack aircraft states at previous time for readability
  Vector3d p_bI_I = aircraft_state_.p;
  Vector3d v_bI_b = aircraft_state_.v;
  Vector3d vdot_bI_b = aircraft_state_.lin_accel;
  quat::Quatd q_Ib = aircraft_state_.q;
  Vector3d omega_bI_b = aircraft_state_.omega;
  Vector3d omegadot_bI_b = aircraft_state_.ang_accel;
  
  // Integrate torques applied to gimbal for rotational states
  omega_aircraft_ = q_Ib.rota(omega_bI_b);
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
  p_bI_I = aircraft_state.p;
  v_bI_b = aircraft_state.v;
  vdot_bI_b = aircraft_state.lin_accel;
  q_Ib = aircraft_state.q;
  omega_bI_b = aircraft_state.omega;
  omegadot_bI_b = aircraft_state.ang_accel;

  // Compute current translational states based on aircraft states
  q_bg_ = q_Ib.inverse() * x_.q;
  x_.p = p_bI_I + q_Ib.rota(p_bg_);
  x_.v = q_bg_.rotp(v_bI_b + omega_bI_b.cross(p_bg_));
  x_.lin_accel = q_bg_.rotp(vdot_bI_b + omega_bI_b.cross(omega_bI_b.cross(p_bg_)) + omegadot_bI_b.cross(p_bg_));

  // Update controller
  Eigen::Vector3d dir_c;
  if (env.getVehiclePositions().size() > 0)
    dir_c = (env.getVehiclePositions()[1] - x_.p).normalized();
  else
    dir_c = common::e1;
  vehicle::Stated xhat = ekf_.getState();
  ctrl_.computeControl(t, xhat.q, dir_c, xhat.omega, q_bg_, u_);

  // Update angular acceleration
  f(x_, u_, vw_, dx_);
  x_.ang_accel = dx_.segment<3>(vehicle::DW);

  // Saturate gimbal roll/pitch angles
  q_bg_ = q_Ib.inverse() * x_.q;
  double roll = q_bg_.roll();
  double pitch = q_bg_.pitch();
  if (abs(roll) > max_roll_ || abs(pitch) > max_pitch_)
  {
    // Compute saturated attitude
    roll = roll > max_roll_ ? max_roll_ : roll;
    pitch = pitch > max_pitch_ ? max_pitch_ : pitch;
    q_bg_ = q_bg_.from_euler(roll, pitch, q_bg_.yaw());
    x_.q = q_Ib * q_bg_;

    // Remove roll/pitch velocities
    quat::Quatd q_g1_to_g;
    q_g1_to_g.from_euler(q_bg_.roll(), q_bg_.pitch(), 0);
    double omega_psi = q_g1_to_g.rota(x_.omega)(2);
    x_.omega = q_bg_.rotp(omega_bI_b) + q_g1_to_g.rotp(Vector3d(0,0,omega_psi));

    // Remove roll/pitch accelerations
    double domega_psi = q_g1_to_g.rota(x_.ang_accel)(2);
    x_.ang_accel = q_bg_.rotp(omegadot_bI_b) + q_g1_to_g.rotp(Vector3d(0,0,domega_psi));
  }

  // Update the estimator, then collect new sensor measurements
  ekf_.run(t, sensors_, aircraft_sensors_, p_bg_, q_bg_, vw_, x_);
  sensors_.updateMeasurements(t, x_, env);

  // FIX ACCELEROMETER MEASUREMENT
  // Because the gimbal is able to rotate independent of the aircraft, the traditionally computed 
  // accelerometer measurement is incorrect. Aircraft, NOT gimbal, linear/angular velocities and angular acceleration
  // contribute to measured accelaration on the gimbal.
  Vector3d p_bu = sensors_.getBodyToImuTranslation();
  quat::Quatd q_bu = sensors_.getBodyToImuRotation();
  Vector3d v = q_bg_.rotp(v_bI_b);
  Vector3d omega = q_bg_.rotp(omega_bI_b);
  Vector3d ang_accel = q_bg_.rotp(omegadot_bI_b);
  Vector3d bad_part = q_bu.rotp(x_.omega.cross(x_.v) + x_.omega.cross(x_.omega.cross(p_bu)) + x_.ang_accel.cross(p_bu));
  Vector3d good_part = q_bu.rotp(omega.cross(v) + omega.cross(omega.cross(p_bu)) + ang_accel.cross(p_bu));
  sensors_.setImuAccel(sensors_.getImuAccel() - bad_part + good_part);

  // Store aircraft state and sensors for correct propagation
  aircraft_state_ = aircraft_state;
  aircraft_sensors_ = aircraft_sensors;

  // Log all of that juicy data
  log(t);
}


void Gimbal::f(const vehicle::Stated& x, const Vector3d& u,
               const Vector3d& vw, vehicle::dxVector& dx)
{
  Vector3d omega_rel = x_.q.rotp(omega_aircraft_) - x_.omega;
  Vector3d tau_f;
  if (omega_rel.norm() < omega_f_)
    tau_f = Kf_ * omega_rel;
  else
    tau_f.setZero();

  Vector3d tau_g = p_gcg_.cross(mass_*common::gravity*x_.q.rotp(common::e3));

  quat::Quatd q_g2_g = quat::Quatd(q_bg_.roll(), 0, 0);
  quat::Quatd q_g1_g = quat::Quatd(q_bg_.roll(), q_bg_.pitch(), 0);
  Vector3d tau_m = u(0) * common::e1 +
                   u(1) * q_g2_g.rotp(common::e2) +
                   u(2) * q_g1_g.rotp(common::e3);

  dx.segment<3>(vehicle::DP) = x.v;
  dx.segment<3>(vehicle::DV) = x.lin_accel;
  dx.segment<3>(vehicle::DQ) = x.omega;
  dx.segment<3>(vehicle::DW) = inertia_inv_ * (tau_f + tau_g + tau_m - x_.omega.cross(inertia_matrix_ * x_.omega));
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
