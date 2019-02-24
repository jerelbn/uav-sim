#include "bicycle.h"

namespace bicycle
{


Bicycle::Bicycle()  : t_prev_(0.0), initialized_(false) {}


Bicycle::Bicycle(const std::string &filename)  : t_prev_(0.0), initialized_(false)
{
  load(filename);
}


Bicycle::~Bicycle()
{
  true_state_log_.close();
  command_log_.close();
}


void Bicycle::load(const std::string &filename)
{
  // Load all parameters
  common::get_yaml_node("bicycle_accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("bicycle_mass", filename, mass_);
  common::get_yaml_node("bicycle_inertia", filename, inertia_);
  common::get_yaml_node("bicycle_length", filename, L_);
  common::get_yaml_node("bicycle_max_force", filename, max_force_);
  common::get_yaml_node("bicycle_max_torque", filename, max_torque_);
  common::get_yaml_node("bicycle_max_steering_angle", filename, max_steering_angle_);
  common::get_yaml_node("bicycle_k_u", filename, ku_);
  common::get_yaml_node("bicycle_k_theta", filename, ktheta_);
  common::get_yaml_node("bicycle_k_psi", filename, kpsi_);
  common::get_yaml_node("bicycle_velocity_command", filename, vel_cmd_);
  common::get_yaml_node("bicycle_flat_ground", filename, flat_ground_);
  common::get_yaml_eigen<xVector>("bicycle_x0", filename, x_);

  // Load waypoints
  std::vector<double> loaded_wps;
  common::get_yaml_node("bicycle_waypoint_threshold", filename, waypoint_threshold_);
  if (common::get_yaml_node("bicycle_waypoints", filename, loaded_wps))
  {
    int num_waypoints = std::floor(loaded_wps.size()/2.0);
    waypoints_ = Eigen::Map<Eigen::MatrixXd>(loaded_wps.data(), 2, num_waypoints);
    current_waypoint_id_ = 0;
  }

  // Initialize loggers
  true_state_log_.open("/tmp/bicycle_true_state.bin");
  command_log_.open("/tmp/bicycle_command.bin");

  // Compute initial control and elevation
  computeControl();
  updateElevation();
}


void Bicycle::f(const xVector& x, const uVector& u, const Vector3d& vw, xVector& dx)
{
  dx(PX) = x(VEL) * cos(x(PSI));
  dx(PY) = x(VEL) * sin(x(PSI));
  dx(PZ) = 0;
  dx(PSI) = x(VEL) * tan(x(THETA)) / L_;
  dx(VEL) = u(FORCE) / mass_;
  dx(THETA) = u(TORQUE) / inertia_;
}


void Bicycle::propagate(const double &t, const uVector& u, const Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Differential Equations
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta
    rk4(std::bind(&Bicycle::f, this,
                  std::placeholders::_1,std::placeholders::_2,
                  std::placeholders::_3,std::placeholders::_4),
                  dt, x_, u, vw, dx_);
  }
  else
  {
    // Euler integration
    f(x_, u, vw, dx_);
    dx_ *= dt;
  }

  // Add change to state
  x_ += dx_;

  // Wrap angles and enforce limits
  x_(PSI) = common::wrapAngle(x_(PSI), M_PI);
  x_(THETA) = common::saturate(x_(THETA), max_steering_angle_, -max_steering_angle_);
}


void Bicycle::run(const double &t, const environment::Environment& env)
{
  log(t); // Log current data
  propagate(t, u_, env.get_vw()); // Propagate truth to next time step
  computeControl(); // Update control input with truth
  updateElevation(); // Update vehicle z component
}


void Bicycle::log(const double &t)
{
  // Write data to binary files and plot in another program
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x_.data(), x_.rows() * sizeof(double));
  command_log_.write((char*)&t, sizeof(double));
  command_log_.write((char*)u_.data(), u_.rows() * sizeof(double));
  command_log_.write((char*)wp_.data(), wp_.rows() * sizeof(double));
}


void Bicycle::computeControl()
{
  // Cosntant velocity doesn't care about waypoints
  u_(FORCE) = common::saturate(-ku_ * mass_ * (x_(VEL) - vel_cmd_), max_force_, -max_force_);

  // Turn the vehicle toward the current waypoint
  updateWaypoint();
  double psi_d = atan2(wp_(PY) - x_(PY), wp_(PX) - x_(PX));
  double psi_err = common::wrapAngle(x_(PSI) - psi_d, M_PI);
  double theta_d = common::saturate(atan(-kpsi_ * L_ / x_(VEL) * psi_err), max_steering_angle_, -max_steering_angle_);
  u_(TORQUE) = common::saturate(-ktheta_ * inertia_ * (x_(THETA) - theta_d), max_torque_, -max_torque_);
}


void Bicycle::updateWaypoint()
{
  if (!initialized_)
  {
    initialized_ = true;
    wp_ = waypoints_.block<2,1>(0, 0);
  }

  // If waypoint error is small, increment waypoint id and update commanded waypoint
  Eigen::Vector2d error = x_.segment<2>(PX) - wp_;
  if (error.norm() < waypoint_threshold_)
  {
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();
    wp_ = waypoints_.block<2,1>(0, current_waypoint_id_);
  }
}


double Bicycle::groundFunction(const xVector &state)
{
  if (!flat_ground_)
  {
    double x = state(PX);
    double y = state(PY);
    return 1.0 * sin(0.25 * x) + 1.0 * sin(0.25 * y);
  }
  else
  {
    return x_(PZ);
  }
}


void Bicycle::updateElevation()
{
  x_(PZ) = groundFunction(x_);
}


}
