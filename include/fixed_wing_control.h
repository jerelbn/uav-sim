#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "vehicle.h"
#include "fixed_wing_base.h"
#include "fw_lqr.h"

using namespace Eigen;


namespace fixedwing
{

class Controller : public FixedWingBase
{

public:

  Controller();
  ~Controller();

  void load(const std::string &filename, const bool &use_random_seed, const std::string &name);
  void computeControl(const vehicle::Stated &x, const double t, uVector& u,
                      const Vector3d &p_target, const Vector3d &vw);
  inline vehicle::Stated getCommandedState() const { return xc_; }

private:

  // Waypoint Enumerations
  enum
  {
    PX,
    PY,
    PZ
  };

  // Parameters
  int path_type_;
  int controller_update_rate_;
  std::default_random_engine rng_;

  // Logging
  std::ofstream command_state_log_;
  std::ofstream command_log_;
  std::ofstream euler_command_log_;

  // Waypoint Parameters
  MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;

  // Trajectory Parameters
  double traj_delta_north_;
  double traj_delta_east_;
  double traj_delta_alt_;
  double traj_nom_north_;
  double traj_nom_east_;
  double traj_nom_alt_;
  double traj_north_freq_;
  double traj_east_freq_;
  double traj_alt_freq_;

  // Controllers
  LQR lqr_;

  // Memory for sharing information between functions
  bool initialized_;
  vehicle::Stated xhat_; // estimate
  vehicle::Stated xc_; // command
  double prev_time_;
  uint8_t control_mode_;

  // Functions
  void updateWaypointManager();
  void updateTrajectoryManager(const double& t);
  void log(const double &t, const uVector &u);
};

} // namespace fixedwing
