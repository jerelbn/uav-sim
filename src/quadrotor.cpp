#include "quadrotor.h"

namespace quadrotor
{


Quadrotor::Quadrotor()  : t_prev_(0.0) {}


Quadrotor::Quadrotor(const std::string &filename, const int& id)  : t_prev_(0.0), id_(id)
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

  // Load all Quadrotor parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);
  common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);

  vehicle::xVector x0;
  Vector3d inertia_diag, angular_drag_diag;
  common::get_yaml_eigen<vehicle::xVector>("x0", filename, x0);
  x_ = vehicle::State(x0);
  if (common::get_yaml_eigen<Vector3d>("inertia", filename, inertia_diag))
  {
    inertia_matrix_ = inertia_diag.asDiagonal();
    inertia_inv_ = inertia_matrix_.inverse();
  }
  if (common::get_yaml_eigen<Vector3d>("linear_drag", filename, linear_drag_))
    linear_drag_matrix_ = linear_drag_.asDiagonal();
  if (common::get_yaml_eigen<Vector3d>("angular_drag", filename, angular_drag_diag))
    angular_drag_matrix_ = angular_drag_diag.asDiagonal();

  // Compute initial control and corresponding acceleration
  Vector3d vw;
  controller_.computeControl(getTrueState(), 0, u_, Vector3d(1,0,0));
  vw.setZero(); // get vw from environment
  updateAccels(u_,vw);

  // Initialize loggers
  true_state_log_.open("/tmp/true_state.bin");
  command_log_.open("/tmp/command.bin");
}


void Quadrotor::f(const vehicle::State& x, const uVector& u,
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
  sensors_.updateMeasurements(t, x_, env.get_points()); // Update sensor measurements
  log(t); // Log current data
  propagate(t, u_, env.get_vw()); // Propagate truth to next time step
  controller_.computeControl(getTrueState(), t, u_, other_vehicle_positions_[0]); // Update control input with truth
  updateAccels(u_, env.get_vw()); // Update true acceleration
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
  Matrix<double, vehicle::NUM_STATES, 1> x = x_.toEigen();
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  controller::Controller::state_t commanded_state = controller_.getCommandedState();
  command_log_.write((char*)&commanded_state, sizeof(controller::Controller::state_t));
  controller_.log(t);
}


void Quadrotor::getOtherVehicles(const std::vector<Vector3d, aligned_allocator<Vector3d> > &all_vehicle_positions)
{
  other_vehicle_positions_.clear();
  for (int i = 0; i < all_vehicle_positions.size(); ++i)
    if (i != id_)
      other_vehicle_positions_.push_back(all_vehicle_positions[i]);
}


}
