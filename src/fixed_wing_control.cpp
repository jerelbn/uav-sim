#include "fixed_wing_control.h"


namespace fixedwing
{


Controller::Controller() :
  prev_time_(0),
  initialized_(false)
{}


Controller::~Controller()
{
  command_log_.close();
}


void Controller::load(const std::string& filename, const bool& use_random_seed, const std::string& name)
{
  // Initialize base class variables
  load_base(filename);

  // Initialize random number generator
  int seed;
  if (use_random_seed)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  else
    seed = 0;
  rng_ = std::default_random_engine(seed);
  srand(seed);

  common::get_yaml_node("throttle_eq", filename, throttle_eq_);
  common::get_yaml_node("mass", filename, mass_);
  // need to load more physical parameters for LQR jacobian calculations

  common::get_yaml_node("path_type", filename, path_type_);

  std::vector<double> loaded_wps;
  if (common::get_yaml_node("waypoints", filename, loaded_wps))
  {
    int num_waypoints = std::floor(loaded_wps.size()/4.0);
    waypoints_ = Map<MatrixXd>(loaded_wps.data(), 3, num_waypoints);
    current_waypoint_id_ = 0;
  }
  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);

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

  // Initialize loggers
  std::stringstream ss;
  ss << "/tmp/" << name << "_command.log";
  command_log_.open(ss.str());
}


// need to pass in wind
void Controller::computeControl(const vehicle::State<double> &x, const double t, quadrotor::uVector& u,
                                const Vector3d& p_target, const Vector3d& vw)
{
  // Copy the current state
  xhat_ = x;

  double dt = t - prev_time_;
  prev_time_ = t;

  if (dt < 1e-7)
  {
    u.setZero();
    return;
  }

  if (path_type_ == 0 || path_type_ == 1)
  {
    // Refresh the waypoint or trajectory
    if (path_type_ == 0)
      updateWaypointManager();
    else
      updateTrajectoryManager(t);
  }
  else
    throw std::runtime_error("Undefined path type in fixed wing controller.");

  // Calculate the Final Output Torques using PID
  u(AIL) = 0.0;
  u(ELE) = 0.0216435;
  u(THR) = 0.171139;
  u(RUD) = 0.0;

  // Log all data
  log(t);
}


void Controller::updateWaypointManager()
{
  if (!initialized_)
  {
    initialized_ = true;
    Map<Vector3d> new_waypoint(waypoints_.block<3,1>(0, 0).data());
    xc_.p = Vector3d(new_waypoint(PX),
                     new_waypoint(PY),
                     new_waypoint(PZ));
  }

  // Find the distance to the desired waypoint
  Vector3d current_waypoint = waypoints_.block<3,1>(0, current_waypoint_id_);
  Vector3d error = current_waypoint - xhat_.p;

  if (error.norm() < waypoint_threshold_)
  {
    // increment waypoint
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();

    // Update The commanded State
    Map<Vector3d> new_waypoint(waypoints_.block<3,1>(0, current_waypoint_id_).data());
    xc_.p = Vector3d(new_waypoint(PX),
                     new_waypoint(PY),
                     new_waypoint(PZ));
  }
}


void Controller::updateTrajectoryManager(const double& t)
{
  xc_.p = Vector3d(traj_nom_north_ + traj_delta_north_ / 2.0 * cos(traj_north_freq_ * t),
                   traj_nom_east_ + traj_delta_east_ / 2.0 * sin(traj_east_freq_ * t),
                   -(traj_nom_alt_ + traj_delta_alt_ / 2.0 * sin(traj_alt_freq_ * t)));
}


void Controller::log(const double &t)
{
  // Write data to binary files and plot in another program
  vehicle::xVector commanded_state = xc_.toEigen();
  command_log_.write((char*)&t, sizeof(double));
  command_log_.write((char*)commanded_state.data(), commanded_state.rows() * sizeof(double));
}


} // namespace fixedwing
