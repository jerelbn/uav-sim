#include "quadrotor.h"

namespace quadrotor
{


Quadrotor::Quadrotor()  : t_prev_(0.0) {}


Quadrotor::Quadrotor(const std::string &filename, const environment::Environment& env, const bool& use_random_seed, const int& id)
  : t_prev_(0.0), id_(id)
{
  load(filename, env, use_random_seed);
}


Quadrotor::~Quadrotor()
{
  state_log_.close();
  euler_log_.close();
}


void Quadrotor::load(const std::string &filename, const environment::Environment& env, const bool& use_random_seed)
{
  // Instantiate Sensors, Controller, and Estimator classes
  common::get_yaml_node("name", filename, name_);
  controller_.load(filename, use_random_seed, name_);
  sensors_.load(filename, use_random_seed, name_);
  ekf_.load(filename, name_);

  // Load all Quadrotor parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);
  common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);

  vehicle::xVector x0;
  Vector3d inertia_diag, angular_drag_diag;
  common::get_yaml_eigen<vehicle::xVector>("x0", filename, x0);
  x_ = vehicle::Stated(x0);
  if (common::get_yaml_eigen<Vector3d>("inertia", filename, inertia_diag))
  {
    inertia_matrix_ = inertia_diag.asDiagonal();
    inertia_inv_ = inertia_matrix_.inverse();
  }
  if (common::get_yaml_eigen<Vector3d>("linear_drag", filename, linear_drag_))
    linear_drag_matrix_ = linear_drag_.asDiagonal();
  if (common::get_yaml_eigen<Vector3d>("angular_drag", filename, angular_drag_diag))
    angular_drag_matrix_ = angular_drag_diag.asDiagonal();

  // Initialize other classes
  controller_.computeControl(getState(), 0, u_, other_vehicle_positions_[0]);
  updateAccels(u_, env.get_vw());
  sensors_.updateMeasurements(0, x_, env.get_vw(), env.get_points());
  ekf_.run(0, sensors_, env.get_vw(), getState());

  // Initialize loggers and log initial data
  std::stringstream ss_s, ss_e;
  ss_s << "/tmp/" << name_ << "_true_state.log";
  ss_e << "/tmp/" << name_ << "_euler_angles.log";
  state_log_.open(ss_s.str());
  euler_log_.open(ss_e.str());
  log(0);
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


void Quadrotor::propagate(const double &t, const uVector& u, const Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Integration
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta
    vehicle::rk4<COMMAND_SIZE>(std::bind(&Quadrotor::f, this,
                               std::placeholders::_1,std::placeholders::_2,
                               std::placeholders::_3,std::placeholders::_4),
                               dt, x_, u, vw, dx_);
  }
  else
  {
    // Euler
    f(x_, u, vw, dx_);
    dx_ *= dt;
  }
  x_ += dx_;
}


void Quadrotor::run(const double &t, const environment::Environment& env)
{
  getOtherVehicles(env.getVehiclePositions());
  propagate(t, u_, env.get_vw()); // Propagate truth to current time step
  if (control_using_estimates_)
    controller_.computeControl(ekf_.getState(), t, u_, other_vehicle_positions_[0]);
  else
    controller_.computeControl(getState(), t, u_, other_vehicle_positions_[0]);
  updateAccels(u_, env.get_vw()); // Update true acceleration
  sensors_.updateMeasurements(t, x_, env.get_vw(), env.get_points());
  ekf_.run(t, sensors_, env.get_vw(), getState());
  log(t); // Log current data
}


void Quadrotor::updateAccels(const uVector &u, const Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, vw, dx);
  x_.lin_accel = dx.segment<3>(vehicle::DV);
  x_.ang_accel = dx.segment<3>(vehicle::DW);
}


void Quadrotor::log(const double &t)
{
  // Write data to binary files and plot in another program
  vehicle::xVector x = x_.toEigen();
  state_log_.write((char*)&t, sizeof(double));
  state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  euler_log_.write((char*)&t, sizeof(double));
  euler_log_.write((char*)x_.q.euler().data(), 3 * sizeof(double));
}


void Quadrotor::getOtherVehicles(const std::vector<Vector3d, aligned_allocator<Vector3d> > &all_vehicle_positions)
{
  other_vehicle_positions_.clear();
  for (int i = 0; i < all_vehicle_positions.size(); ++i)
    if (i != id_)
      other_vehicle_positions_.push_back(all_vehicle_positions[i]);
}


} // namespace quadrotor
