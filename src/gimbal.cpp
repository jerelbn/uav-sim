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
  u_.setZero();

  // Load other modules (e.g. controller, estimator, sensors)
  sensors_.load(filename, use_random_seed, name_);

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


void Gimbal::update(const double &t, const vehicle::Stated& aircraft_state, environment::Environment& env)
{
  // Time step for controller later
  double dt = t - t_prev_;
  t_prev_ = t;

  // Unpack aircraft states for readability
  Vector3d p_bI_I = aircraft_state.p;
  Vector3d v_bI_b = aircraft_state.v;
  Vector3d vdot_bI_b = aircraft_state.lin_accel;
  quat::Quatd q_Ib = aircraft_state.q;
  Vector3d omega_bI_b = aircraft_state.omega;
  Vector3d omegadot_bI_b = aircraft_state.ang_accel;

  // Compute current translational states based on aircraft states
  x_.p = p_bI_I + q_Ib.rota(p_bg_);
  x_.v = q_Ib.rota(v_bI_b + omega_bI_b.cross(p_bg_));
  x_.lin_accel = q_Ib.rota(vdot_bI_b + omega_bI_b.cross(omega_bI_b.cross(p_bg_)) + omegadot_bI_b.cross(p_bg_));
  
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

  // Collect new sensor measurements
  sensors_.updateMeasurements(t, x_, env);

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

  quat::Quatd q_g2_g = quat::Quatd(x_.q.roll(), 0, 0);
  quat::Quatd q_g1_g = quat::Quatd(x_.q.roll(), x_.q.pitch(), 0);
  Vector3d tau_m = u(0) * common::e1 +
                   u(1) * q_g2_g.rotp(common::e2) +
                   u(2) * q_g1_g.rotp(common::e3);

  dx.segment<3>(vehicle::DP).setZero();
  dx.segment<3>(vehicle::DV).setZero();
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
