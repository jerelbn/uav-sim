#include "quadrotor.h"

namespace quadrotor
{


Quadrotor::Quadrotor()  : t_prev_(0.0) {}


Quadrotor::Quadrotor(const std::string &filename, const std::default_random_engine& rng)
{
  load(filename, rng);
}


Quadrotor::~Quadrotor() {}


void Quadrotor::load(const std::string &filename, const std::default_random_engine& rng)
{
  // Load all Quadrotor parameters
  common::get_yaml_node("name", filename, name_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);

  vehicle::xVector x0;
  common::get_yaml_eigen("x0", filename, x0);
  common::get_yaml_eigen_diag("inertia", filename, inertia_matrix_);
  common::get_yaml_eigen_diag("linear_drag", filename, linear_drag_matrix_);
  common::get_yaml_eigen_diag("angular_drag", filename, angular_drag_matrix_);
  x_ = vehicle::Stated(x0);
  x_.drag = linear_drag_matrix_(0,0);
  inertia_inv_ = inertia_matrix_.inverse();

  // Initialize loggers and log initial data
  std::stringstream ss_s;
  ss_s << "/tmp/" << name_ << "_true_state.log";
  state_log_.open(ss_s.str());
}


void Quadrotor::propagate(const double &t, const uVector& u, const Vector3d& vw)
{
  double dt = t - t_prev_;
  t_prev_ = t;

  if (t > 0)
  {
    // 4th order Runge-Kutta integration
    vehicle::rk4<COMMAND_SIZE>(std::bind(&Quadrotor::f, this,
                                std::placeholders::_1,std::placeholders::_2,
                                std::placeholders::_3,std::placeholders::_4),
                                dt, x_, u, vw, dx_);
    x_ += dx_;
  }

  log(t);
}


void Quadrotor::f(const vehicle::Stated& x, const uVector& u,
                  const Vector3d& vw, vehicle::dxVector& dx)
{
  v_rel_ = x.v - x.q.rotp(vw);
  dx.segment<3>(vehicle::DP) = x.q.rota(x.v);
  dx.segment<3>(vehicle::DV) = -common::e3 * u(THRUST) * max_thrust_ / mass_ - linear_drag_matrix_ * v_rel_ +
                                 common::gravity * x.q.rotp(common::e3) - x.omega.cross(x.v);
  dx.segment<3>(vehicle::DQ) = x.omega;
  dx.segment<3>(vehicle::DW) = inertia_inv_ * (u.segment<3>(TAUX) - x.omega.cross(inertia_matrix_ * x.omega) -
                                angular_drag_matrix_ * x.omega);
}


void Quadrotor::updateAccelerations(const uVector &u, const Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, vw, dx);
  x_.lin_accel = dx.segment<3>(vehicle::DV);
  x_.ang_accel = dx.segment<3>(vehicle::DW);
}


void Quadrotor::log(const double &t)
{
  // Write data to binary files and plot in another program
  state_log_.log(t);
  state_log_.logMatrix(x_.toEigen());
  state_log_.logMatrix(x_.q.euler());
}


} // namespace quadrotor
