#pragma once

#include "common_cpp/common.h"
#include "vehicle.h"
#include <fstream>

using namespace Eigen;


namespace quadrotor
{

class Controller
{

public:

  Controller();
  Controller(const std::string &filename, const std::default_random_engine& rng, const std::string &name);
  ~Controller();

  void load(const std::string &filename, const std::default_random_engine& rng, const std::string &name);
  void computeControl(const vehicle::Stated &x, const double t, const Vector3d &pt);
  inline vehicle::Stated getCommandedState() const { return xc_; }

  quadrotor::uVector u_;

private:

  // Waypoint Enumerations
  enum
  {
    PX,
    PY,
    PZ,
    PSI
  };

  typedef struct
  {
    double roll;
    double pitch;
    double yaw_rate;
    double throttle;
    double vel;
  } max_t;

  common::PID<double> roll_;
  common::PID<double> pitch_;
  common::PID<double> yaw_rate_;

  // Parameters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  int path_type_;
  std::default_random_engine rng_;

  // Waypoint Parameters
  MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;
  double waypoint_velocity_threshold_;

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

  // Circumnavigation parameters
  double circ_rd_;
  double circ_hd_;
  double circ_kr_;
  double circ_kp_;
  double circ_kh_;

  // Controller Gains
  Matrix3d K_p_; // position
  Matrix3d K_v_; // velocity
  Matrix3d K_d_; // disturbance acceleration


  // Memory for sharing information between functions
  bool initialized_;
  vehicle::Stated xhat_; // estimate
  vehicle::Stated xc_; // command
  max_t max_ = {};
  double prev_time_;
  uint8_t control_mode_;
  Vector3d dhat_; // disturbance acceleration

  // Target estimation parameters
  bool use_target_truth_, bearing_only_;
  Vector3d z_, vz_;
  double kz_, kvz_;
  Vector3d target_noise_;
  std::normal_distribution<double> target_noise_dist_;

  // Logging
  std::ofstream target_log_;
  std::ofstream command_state_log_;
  std::ofstream command_log_;
  std::ofstream euler_command_log_;

  // Functions
  void updateWaypointManager();
  void updateTrajectoryManager(const double &t);
  void log(const double& t);
};

} // namespace quadrotor
