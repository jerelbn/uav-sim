#include "quadrotor.h"

namespace quadrotor
{


Quadrotor::Quadrotor()  : t_prev_(0.0) {}


Quadrotor::Quadrotor(const std::string &filename)  : t_prev_(0.0)
{
  load(filename);
}


Quadrotor::~Quadrotor()
{
  true_state_log_.close();
  command_log_.close();
}


void Quadrotor::load(const std::string &filename)
{
  // Instantiate Sensors, Controller, and Estimator classes
  controller_.load(filename);
  sensors_.load(filename);
  ekf_.load(filename);

  // Load all Quadrotor parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);
  common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);

  vehicle::xVector x0;
  Eigen::Vector3d inertia_diag, angular_drag_diag;
  common::get_yaml_eigen<vehicle::xVector>("x0", filename, x0);
  x_ = vehicle::State(x0);
  if (common::get_yaml_eigen<Eigen::Vector3d>("inertia", filename, inertia_diag))
  {
    inertia_matrix_ = inertia_diag.asDiagonal();
    inertia_inv_ = inertia_matrix_.inverse();
  }
  if (common::get_yaml_eigen<Eigen::Vector3d>("linear_drag", filename, linear_drag_))
    linear_drag_matrix_ = linear_drag_.asDiagonal();
  if (common::get_yaml_eigen<Eigen::Vector3d>("angular_drag", filename, angular_drag_diag))
    angular_drag_matrix_ = angular_drag_diag.asDiagonal();

  // Compute initial control and corresponding acceleration
  Eigen::Vector3d vw;
  controller_.computeControl(getTrueState(), 0, u_);
//  controller_.computeControl(ekf_.getVehicleState(), 0, u_);
  vw.setZero(); // get vw from environment
  updateAccel(u_,vw);

  // Initialize loggers
  common::get_yaml_node("log_directory", filename, directory_);
  true_state_log_.open(directory_ + "/true_state.bin");
  command_log_.open(directory_ + "/command.bin");
}


void Quadrotor::f(const vehicle::State& x, const commandVector& u, vehicle::dxVector& dx, const Eigen::Vector3d& vw)
{
  v_rel_ = x.v - x.q.rot(vw);
  dx.segment<3>(vehicle::DPX) = x.q.inv().rot(x.v);
  dx.segment<3>(vehicle::DQX) = x.omega;
  dx.segment<3>(vehicle::DVX) = -common::e3 * u(THRUST) * max_thrust_ / mass_ - linear_drag_.cwiseProduct(v_rel_).cwiseProduct(v_rel_) +
                                 common::gravity * x.q.rot(common::e3) - x.omega.cross(x.v);
  dx.segment<3>(vehicle::DWX) = inertia_inv_ * (u.segment<3>(TAUX) - x.omega.cross(inertia_matrix_ * x.omega) -
                                angular_drag_matrix_ * x.omega.cwiseProduct(x.omega));
}


void Quadrotor::propagate(const double &t, const commandVector& u, const Eigen::Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Differential Equations
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta integration
    f(x_, u, k1_, vw);

    x2_ = x_;
    x2_.p += k1_.segment<3>(vehicle::DPX) * dt / 2;
    x2_.q += k1_.segment<3>(vehicle::DQX) * dt / 2;
    x2_.v += k1_.segment<3>(vehicle::DVX) * dt / 2;
    x2_.omega += k1_.segment<3>(vehicle::DWX) * dt / 2;
    f(x2_, u, k2_, vw);

    x3_ = x_;
    x3_.p += k2_.segment<3>(vehicle::DPX) * dt / 2;
    x3_.q += k2_.segment<3>(vehicle::DQX) * dt / 2;
    x3_.v += k2_.segment<3>(vehicle::DVX) * dt / 2;
    x3_.omega += k2_.segment<3>(vehicle::DWX) * dt / 2;
    f(x3_, u, k3_, vw);

    x4_ = x_;
    x4_.p += k3_.segment<3>(vehicle::DPX) * dt;
    x4_.q += k3_.segment<3>(vehicle::DQX) * dt;
    x4_.v += k3_.segment<3>(vehicle::DVX) * dt;
    x4_.omega += k3_.segment<3>(vehicle::DWX) * dt;
    f(x4_, u, k4_, vw);

    dx_ = (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6.0;
  }
  else
  {
    // Euler integration
    f(x_, u, dx_, vw);
    dx_ *= dt;
  }

  // Copy output
  x_.p += dx_.segment<3>(vehicle::DPX);
  x_.q += dx_.segment<3>(vehicle::DQX);
  x_.v += dx_.segment<3>(vehicle::DVX);
  x_.omega += dx_.segment<3>(vehicle::DWX);
}


void Quadrotor::run(const double &t, const environment::Environment& env)
{
  sensors_.updateMeasurements(t, x_, env.get_points().matrix()); // Update sensor measurements
  log(t); // Log current data
  ekf_.run(t, sensors_);
  propagate(t, u_, env.get_vw()); // Propagate truth to next time step
  if (control_using_estimates_)
    controller_.computeControl(ekf_.getVehicleState(), t, u_); // Update control input with estimates
  else
    controller_.computeControl(getTrueState(), t, u_); // Update control input with truth
  updateAccel(u_, env.get_vw()); // Update true acceleration
}


void Quadrotor::updateAccel(const commandVector &u, const Eigen::Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, dx, vw);
  x_.accel = dx.segment<3>(vehicle::DVX);
}


void Quadrotor::log(const double &t)
{
  // Write data to binary files and plot in another program
  Eigen::Matrix<double, vehicle::NUM_STATES, 1> x = x_.toEigen();
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  controller::Controller::state_t commanded_state = controller_.getCommandedState();
  command_log_.write((char*)&commanded_state, sizeof(controller::Controller::state_t));
}


}
