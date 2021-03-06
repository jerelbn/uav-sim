#include "fixed_wing_control.h"


namespace fixedwing
{


Controller::Controller() :
  prev_time_(0),
  initialized_(false)
{}


Controller::~Controller()
{
  command_state_log_.close();
  command_log_.close();
  euler_command_log_.close();
}


void Controller::load(const std::string& filename, const std::default_random_engine& rng, const std::string& name)
{
  // Initialize base class variables
  load_base(filename);

  // Get random number generator
  rng_ = rng;

  // General parameters
  common::get_yaml_node("controller_update_rate", filename, controller_update_rate_);
  common::get_yaml_node("path_type", filename, path_type_);

  // Waypoint parameters
  std::vector<double> loaded_wps;
  if (common::get_yaml_node("waypoints", filename, loaded_wps))
  {
    int num_waypoints = std::floor(loaded_wps.size()/3.0);
    waypoints_ = Map<MatrixXd>(loaded_wps.data(), 3, num_waypoints);
    current_waypoint_id_ = 0;
  }
  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);

  // Trajectory parameters
  double traj_north_period, traj_east_period, traj_alt_period;
  common::get_yaml_node("traj_delta_north", filename, traj_delta_north_);
  common::get_yaml_node("traj_delta_east", filename, traj_delta_east_);
  common::get_yaml_node("traj_delta_alt", filename, traj_delta_alt_);
  common::get_yaml_node("traj_nom_north", filename, traj_nom_north_);
  common::get_yaml_node("traj_nom_east", filename, traj_nom_east_);
  common::get_yaml_node("traj_nom_alt", filename, traj_nom_alt_);
  common::get_yaml_node("traj_north_period", filename, traj_north_period);
  common::get_yaml_node("traj_east_period", filename, traj_east_period);
  common::get_yaml_node("traj_alt_period", filename, traj_alt_period);
  traj_north_freq_ = 2.0 * M_PI / traj_north_period;
  traj_east_freq_ = 2.0 * M_PI / traj_east_period;
  traj_alt_freq_ = 2.0 * M_PI / traj_alt_period;

  // Initialize controllers
  lqr_.init(filename, rng_);

  // Initialize loggers
  std::stringstream ss_cs, ss_c, ss_ec;
  ss_cs << "/tmp/" << name << "_commanded_state.log";
  ss_c << "/tmp/" << name << "_command.log";
  ss_ec << "/tmp/" << name << "_euler_command.log";
  command_state_log_.open(ss_cs.str());
  command_log_.open(ss_c.str());
  euler_command_log_.open(ss_ec.str());
}


// need to pass in wind
void Controller::computeControl(const vehicle::Stated &x, const double t, uVector& u,
                                const Vector3d& p_target, const Vector3d& vw)
{
  double dt = t - prev_time_;
  if (t == 0)
    wp_prev_ = Vector3d(waypoints_(0,waypoints_.cols()-1),
                        waypoints_(1,waypoints_.cols()-1),
                        waypoints_(2,waypoints_.cols()-1));

  if (t == 0 || dt >= 1.0 / controller_update_rate_)
  {
    // Copy the current state and time
    xhat_ = x;
    prev_time_ = t;

    if (path_type_ == 0 || path_type_ == 1)
    {
      // Refresh the waypoint or trajectory
      if (path_type_ == 0)
        updateWaypointManager();
      else
        updateTrajectoryManager(t);

      // Compute control
      lqr_.computeControl(xhat_, vw, wp_prev_, wp_, xc_, u);
    }
    else
      throw std::runtime_error("Undefined path type in fixed wing controller.");
  }

  // Log all data
  log(t, u);
}


void Controller::updateWaypointManager()
{
  if (!initialized_)
  {
    initialized_ = true;
    Map<Vector3d> new_waypoint(waypoints_.block<3,1>(0, 0).data());
    wp_ = Vector3d(new_waypoint(PX),
                   new_waypoint(PY),
                   new_waypoint(PZ));
  }

  // Find the distance to the desired waypoint
  Vector3d radial_error = wp_ - xhat_.p;
  Vector3d line_dir = (wp_ - wp_prev_).normalized();
  Vector3d plane_origin = wp_ - waypoint_threshold_ * line_dir;
  Vector3d p_rel = xhat_.p - plane_origin;

  if (radial_error.norm() < waypoint_threshold_ || p_rel.dot(line_dir) >= 0)
  {
    // increment waypoint
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();

    // Save current waypoint and pdate the commanded state
    wp_prev_ = wp_;
    Map<Vector3d> new_waypoint(waypoints_.block<3,1>(0, current_waypoint_id_).data());
    wp_ = Vector3d(new_waypoint(PX),
                   new_waypoint(PY),
                   new_waypoint(PZ));
  }
}


void Controller::updateTrajectoryManager(const double& t)
{
  wp_ = Vector3d(traj_nom_north_ + traj_delta_north_ / 2.0 * cos(traj_north_freq_ * t),
                 traj_nom_east_ + traj_delta_east_ / 2.0 * sin(traj_east_freq_ * t),
                 -(traj_nom_alt_ + traj_delta_alt_ / 2.0 * sin(traj_alt_freq_ * t)));
}


void Controller::log(const double &t, const uVector& u)
{
  // Write data to binary files and plot in another program
  vehicle::xVector commanded_state = xc_.toEigen();
  command_state_log_.write((char*)&t, sizeof(double));
  command_state_log_.write((char*)commanded_state.data(), commanded_state.rows() * sizeof(double));
  command_log_.write((char*)&t, sizeof(double));
  command_log_.write((char*)u.data(), u.rows() * sizeof(double));
  euler_command_log_.write((char*)&t, sizeof(double));
  euler_command_log_.write((char*)xc_.q.euler().data(), 3 * sizeof(double));
}


} // namespace fixedwing
