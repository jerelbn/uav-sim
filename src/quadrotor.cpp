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
  controller_.computeControl(getTrueState(), 0, u_, Eigen::Vector3d(1,0,0));
//  controller_.computeControl(ekf_.getVehicleState(), 0, u_);
  vw.setZero(); // get vw from environment
  updateAccels(u_,vw);

  // Initialize loggers
  common::get_yaml_node("log_directory", filename, directory_);
  true_state_log_.open(directory_ + "/true_state.bin");
  command_log_.open(directory_ + "/command.bin");
}


void Quadrotor::f(const vehicle::State& x, const commandVector& u,
                  const Eigen::Vector3d& vw, vehicle::dxVector& dx)
{
  v_rel_ = x.v - x.q.rot(vw);
  dx.segment<3>(vehicle::DPX) = x.q.inv().rot(x.v);
  dx.segment<3>(vehicle::DVX) = -common::e3 * u(THRUST) * max_thrust_ / mass_ - linear_drag_.cwiseProduct(v_rel_).cwiseProduct(v_rel_) +
                                 common::gravity * x.q.rot(common::e3) - x.omega.cross(x.v);
  dx.segment<3>(vehicle::DQX) = x.omega;
  dx.segment<3>(vehicle::DWX) = inertia_inv_ * (u.segment<3>(TAUX) - x.omega.cross(inertia_matrix_ * x.omega) -
                                angular_drag_matrix_ * x.omega.cwiseProduct(x.omega));
}


void Quadrotor::rk4(std::function<void(const vehicle::State&, const commandVector&,
                                       const Eigen::Vector3d&, vehicle::dxVector&)> func,
         const double& dt, const vehicle::State& x, const commandVector& u,
         const Eigen::Vector3d& vw, vehicle::dxVector& dx)
{
  func(x, u, vw, k1_);
  func(x + k1_ * dt / 2, u, vw, k2_);
  func(x + k2_ * dt / 2, u, vw, k3_);
  func(x + k3_ * dt, u, vw, k4_);
  dx = (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6.0;
}


void Quadrotor::propagate(const double &t, const commandVector& u, const Eigen::Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Integration
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta
    rk4(std::bind(&quadrotor::Quadrotor::f, this,
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
  sensors_.updateMeasurements(t, x_, env.get_points().matrix()); // Update sensor measurements
  log(t); // Log current data
  ekf_.run(t, sensors_);
  propagate(t, u_, env.get_vw()); // Propagate truth to next time step
  if (control_using_estimates_)
    controller_.computeControl(ekf_.getVehicleState(), t, u_, other_vehicle_positions[0]); // Update control input with estimates
  else
    controller_.computeControl(getTrueState(), t, u_, other_vehicle_positions[0]); // Update control input with truth
  updateAccels(u_, env.get_vw()); // Update true acceleration
}


void Quadrotor::updateAccels(const commandVector &u, const Eigen::Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, vw, dx);
  x_.lin_accel = dx.segment<3>(vehicle::DVX);
  x_.ang_accel = dx.segment<3>(vehicle::DWX);
}


void Quadrotor::log(const double &t)
{
  // Write data to binary files and plot in another program
  Eigen::Matrix<double, vehicle::NUM_STATES, 1> x = x_.toEigen();
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  controller::Controller::state_t commanded_state = controller_.getCommandedState();
  command_log_.write((char*)&commanded_state, sizeof(controller::Controller::state_t));
  controller_.log(t);
}


void Quadrotor::getOtherVehicles(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &all_vehicle_positions)
{
  // Reserve memory for the other vehicle positions
  if (other_vehicle_positions.size() < all_vehicle_positions.size()-1)
    other_vehicle_positions.reserve(all_vehicle_positions.size()-1);

  // Store other vehicle positions
  int count = 0;
  for (int i = 0; i < all_vehicle_positions.size(); ++i)
  {
    Eigen::Vector3d error = all_vehicle_positions[i] - x_.p;
    if (error.norm() > 1e-6)
    {
      other_vehicle_positions[count] = all_vehicle_positions[i];
      ++count;
    }
  }
}


}
