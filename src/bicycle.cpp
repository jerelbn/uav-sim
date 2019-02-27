#include "bicycle.h"

namespace bicycle
{


Bicycle::Bicycle()  : t_prev_(0.0), initialized_(false) {}


Bicycle::Bicycle(const std::string &filename, const environment::Environment& env, const bool& use_random_seed, const int& id)
  : t_prev_(0.0), initialized_(false), id_(id)
{
  load(filename, env, use_random_seed);
}


Bicycle::~Bicycle()
{
  true_state_log_.close();
  command_log_.close();
}


void Bicycle::load(const std::string &filename, const environment::Environment &env, const bool& use_random_seed)
{
  // Load all parameters
  xVector x0;
  common::get_yaml_node("name", filename, name_);
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("inertia", filename, inertia_);
  common::get_yaml_node("length", filename, L_);
  common::get_yaml_node("max_force", filename, max_force_);
  common::get_yaml_node("max_torque", filename, max_torque_);
  common::get_yaml_node("max_steering_angle", filename, max_steering_angle_);
  common::get_yaml_node("k_u", filename, ku_);
  common::get_yaml_node("k_theta", filename, ktheta_);
  common::get_yaml_node("k_psi", filename, kpsi_);
  common::get_yaml_node("velocity_command", filename, vel_cmd_);
  common::get_yaml_node("flat_ground", filename, flat_ground_);
  common::get_yaml_eigen<xVector>("x0", filename, x0);
  x_ = State(x0);

  // Load waypoints
  std::vector<double> loaded_wps;
  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);
  if (common::get_yaml_node("waypoints", filename, loaded_wps))
  {
    int num_waypoints = std::floor(loaded_wps.size()/2.0);
    waypoints_ = Eigen::Map<Eigen::MatrixXd>(loaded_wps.data(), 2, num_waypoints);
    current_waypoint_id_ = 0;
  }

  // Compute initial control and elevation
  computeControl();
  updateElevation(env);

  // Initialize loggers
  std::stringstream ss_ts, ss_c;
  ss_ts << "/tmp/" << name_ << "_true_state.log";
  ss_c << "/tmp/" << name_ << "_command.log";
  true_state_log_.open(ss_ts.str());
  command_log_.open(ss_c.str());
  log(0);
}


void Bicycle::f(const State &x, const uVector& u, const Vector3d& vw, xVector& dx)
{
  dx(PX) = x.v * cos(x.psi);
  dx(PY) = x.v * sin(x.psi);
  dx(PZ) = 0;
  dx(PSI) = x.v * tan(x.theta) / L_;
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
  x_.psi = common::wrapAngle(x_.psi, M_PI);
  x_.theta = common::saturate(x_.theta, max_steering_angle_, -max_steering_angle_);
}


void Bicycle::run(const double &t, const environment::Environment& env)
{
  propagate(t, u_, env.get_vw()); // Propagate truth to next time step
  computeControl(); // Update control input with truth
  updateElevation(env); // Update vehicle z component
  log(t); // Log current data
}


void Bicycle::log(const double &t)
{
  // Write data to binary files and plot in another program
  Matrix<double, NUM_STATES, 1> x = x_.toEigen();
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  command_log_.write((char*)&t, sizeof(double));
  command_log_.write((char*)u_.data(), u_.rows() * sizeof(double));
  command_log_.write((char*)wp_.data(), wp_.rows() * sizeof(double));
}


void Bicycle::computeControl()
{
  // Cosntant velocity doesn't care about waypoints
  u_(FORCE) = common::saturate(-ku_ * mass_ * (x_.v - vel_cmd_), max_force_, -max_force_);

  // Turn the vehicle toward the current waypoint
  updateWaypoint();
  double psi_d = atan2(wp_(PY) - x_.p(PY), wp_(PX) - x_.p(PX));
  double psi_err = common::wrapAngle(x_.psi - psi_d, M_PI);
  double theta_d = common::saturate(atan(-kpsi_ * L_ / x_.v * psi_err), max_steering_angle_, -max_steering_angle_);
  u_(TORQUE) = common::saturate(-ktheta_ * inertia_ * (x_.theta - theta_d), max_torque_, -max_torque_);
}


void Bicycle::updateWaypoint()
{
  if (!initialized_)
  {
    initialized_ = true;
    wp_ = waypoints_.block<2,1>(0, 0);
  }

  // If waypoint error is small, increment waypoint id and update commanded waypoint
  Eigen::Vector2d error = x_.p.segment<2>(PX) - wp_;
  if (error.norm() < waypoint_threshold_)
  {
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();
    wp_ = waypoints_.block<2,1>(0, current_waypoint_id_);
  }
}


void Bicycle::updateElevation(const environment::Environment& env)
{
  x_.p(PZ) = env.getElevation(x_.p(PX), x_.p(PY));
}


}
