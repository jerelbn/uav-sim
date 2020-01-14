#include "bicycle_ctrl_pid.h"

namespace bicycle_ctrl_pid
{


Controller::Controller()  : t_prev_(0.0), initialized_(false) {}


Controller::Controller(const std::string &filename, const std::string& name)
  : t_prev_(0.0), initialized_(false)
{
  load(filename, name);
}


Controller::~Controller() {}


void Controller::load(const std::string &filename, const std::string& name)
{
  // Load all parameters
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

  // Load waypoints
  std::vector<double> loaded_wps;
  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);
  if (common::get_yaml_node("waypoints", filename, loaded_wps))
  {
    int num_waypoints = std::floor(loaded_wps.size()/2.0);
    waypoints_ = Eigen::Map<Eigen::MatrixXd>(loaded_wps.data(), 2, num_waypoints);
    current_waypoint_id_ = 0;
  }

  // Initialize logger
  std::stringstream ss;
  ss << "/tmp/" << name << "_command.log";
  command_log_.open(ss.str());
}


void Controller::computeControl(const bicycle::State& x)
{
  // Cosntant velocity doesn't care about waypoints
  u_(bicycle::FORCE) = common::saturate(-ku_ * mass_ * (x.v - vel_cmd_), max_force_, -max_force_);

  // Turn the vehicle toward the current waypoint
  updateWaypoint(x);
  double psi_d = atan2(wp_(bicycle::PY) - x.p(bicycle::PY), wp_(bicycle::PX) - x.p(bicycle::PX));
  double psi_err = common::wrapAngle(x.psi - psi_d, M_PI);
  double theta_d = common::saturate(atan(-kpsi_ * L_ / x.v * psi_err), max_steering_angle_, -max_steering_angle_);
  u_(bicycle::TORQUE) = common::saturate(-ktheta_ * inertia_ * (x.theta - theta_d), max_torque_, -max_torque_);
}


void Controller::updateWaypoint(const bicycle::State& x)
{
  if (!initialized_)
  {
    initialized_ = true;
    wp_ = waypoints_.block<2,1>(0, 0);
  }

  // If waypoint error is small, increment waypoint id and update commanded waypoint
  Eigen::Vector2d error = x.p.segment<2>(bicycle::PX) - wp_;
  if (error.norm() < waypoint_threshold_)
  {
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();
    wp_ = waypoints_.block<2,1>(0, current_waypoint_id_);
  }
}


void Controller::log(const double &t)
{
  command_log_.log(t);
  command_log_.logMatrix(u_);
  command_log_.logMatrix(wp_);
}


} // namespace bicycle_ctrl_pid
