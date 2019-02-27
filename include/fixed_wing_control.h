#pragma once

#include "common_cpp/common.h"
#include "vehicle.h"
#include <fstream>

using namespace Eigen;


namespace fixedwing
{

class Controller
{

public:

  Controller();

  void load(const std::string &filename, const bool &use_random_seed, const std::string &name);
  void computeControl(const vehicle::State &x, const double t, quadrotor::uVector& u, const Vector3d &pt);
  void log(const double& t);
  inline vehicle::State getCommandedState() const { return xc_; }

private:

  // Waypoint Enumerations
  enum
  {
    PX,
    PY,
    PZ
  };

  // Parameters
  double throttle_eq_;
  double mass_;
  int path_type_;
  std::default_random_engine rng_;
  uVector u_prev_;

  // Waypoint Parameters
  MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;

  // Trajectory Parameters
  double traj_delta_north_;
  double traj_delta_east_;
  double traj_delta_alt_;
  double traj_delta_yaw_;
  double traj_nom_north_;
  double traj_nom_east_;
  double traj_nom_alt_;
  double traj_nom_yaw_;
  double traj_north_freq_;
  double traj_east_freq_;
  double traj_alt_freq_;
  double traj_yaw_freq_;

  // Memory for sharing information between functions
  bool initialized_;
  vehicle::State xhat_; // estimate
  vehicle::State xc_; // command
  double prev_time_;
  uint8_t control_mode_;

  // Functions
  void updateWaypointManager();
  void updateTrajectoryManager(const double& t);
};

} // namespace fixedwing
