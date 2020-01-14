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
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("omega_f", filename, omega_f_);
  common::get_yaml_node("max_roll", filename, max_roll_);
  common::get_yaml_node("max_pitch", filename, max_pitch_);
  dummy_.setZero();

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
}


void Gimbal::propagate(const double &t, const Vector3d& u, const quat::Quatd& q_Ib)
{
  // Time step for controller later
  double dt = t - t_prev_;
  t_prev_ = t;
  
  // Integrate rotational states
  if (t > 0)
  {
    // 4th order Runge-Kutta
    vehicle::rk4<3>(std::bind(&Gimbal::f, this,
                    std::placeholders::_1,std::placeholders::_2,
                    std::placeholders::_3,std::placeholders::_4),
                    dt, x_, u, dummy_, dx_);
    x_ += dx_;
  }

  // Saturate gimbal roll/pitch angles and zero out velocities/accelerations
  saturateRollPitch();

  // Store aircraft attitude for C.G. calcluation in dynamics
  q_Ib_ = q_Ib;

  // Log all of that juicy data
  log(t);
}


void Gimbal::f(const vehicle::Stated& x, const Vector3d& u,
               const Vector3d& vw, vehicle::dxVector& dx)
{
  Vector3d tauf;
  if (x.omega.norm() < omega_f_)
    tauf = Kf_ * x.omega;
  else
    tauf.setZero();

  quat::Quatd q_Ig = q_Ib_ * x.q;
  Vector3d taug = p_gcg_.cross(mass_*common::gravity*q_Ig.rotp(common::e3));

  quat::Quatd q_g2_g = quat::Quatd(x.q.roll(), 0, 0);
  quat::Quatd q_g1_g = quat::Quatd(x.q.roll(), x.q.pitch(), 0);
  Vector3d taum = u(0) * common::e1 +
                   u(1) * q_g2_g.rotp(common::e2) +
                   u(2) * q_g1_g.rotp(common::e3);

  dx.setZero();
  dx.segment<3>(vehicle::DQ) = x.omega;
  dx.segment<3>(vehicle::DW) = inertia_inv_ * (tauf + taug + taum - x.omega.cross(inertia_matrix_ * x.omega));
}


void Gimbal::updateAccelerations(const Vector3d& u)
{
  f(x_, u, dummy_, dx_);
  x_.ang_accel = dx_.segment<3>(vehicle::DW);
}


void Gimbal::saturateRollPitch()
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
