#include "bicycle.h"

namespace bicycle
{


Bicycle::Bicycle()  : t_prev_(0.0), initialized_(false) {}


Bicycle::Bicycle(const std::string &filename, const environment::Environment& env)
  : t_prev_(0.0), initialized_(false)
{
  load(filename, env);
}


Bicycle::~Bicycle() {}


void Bicycle::load(const std::string &filename, const environment::Environment &env)
{
  // Load all parameters
  xVector x0;
  common::get_yaml_node("name", filename, name_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("inertia", filename, inertia_);
  common::get_yaml_node("length", filename, L_);
  common::get_yaml_node("max_steering_angle", filename, max_steering_angle_);
  common::get_yaml_node("flat_ground", filename, flat_ground_);
  common::get_yaml_eigen<xVector>("x0", filename, x0);
  x_ = State(x0);

  // Compute initial elevation
  updateElevation(env);

  // Initialize loggers
  std::string logname_true_state;
  common::get_yaml_node("logname_true_state", filename, logname_true_state);
  state_log_.open(logname_true_state);
}


void Bicycle::propagate(const double &t, const uVector& u, const environment::Environment& env)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  if (t > 0)
  {
    // 4th order Runge-Kutta integration
    rk4(std::bind(&Bicycle::f, this,
                  std::placeholders::_1,std::placeholders::_2, std::placeholders::_3),
                  dt, x_, u, dx_);
    x_ += dx_;

    // Wrap angles and enforce steering limits
    x_.psi = common::wrapAngle(x_.psi, M_PI);
    x_.theta = common::saturate(x_.theta, max_steering_angle_, -max_steering_angle_);
  }

  updateElevation(env);
  log(t);
}


void Bicycle::f(const State &x, const uVector& u, xVector& dx)
{
  dx(PX) = x.v * cos(x.psi);
  dx(PY) = x.v * sin(x.psi);
  dx(PZ) = 0;
  dx(PSI) = x.v * tan(x.theta) / L_;
  dx(VEL) = u(FORCE) / mass_;
  dx(THETA) = u(TORQUE) / inertia_;
}


void Bicycle::updateElevation(const environment::Environment& env)
{
  x_.p(PZ) = 0;
}


void Bicycle::log(const double &t)
{
  state_log_.log(t);
  state_log_.logMatrix(x_.toEigen());
}


}
