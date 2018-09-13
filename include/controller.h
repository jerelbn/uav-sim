#pragma once

#include "common_cpp/common.h"
#include "vehicle_common.h"


namespace controller
{

class Controller
{

public:

  typedef struct
  {
    double t;
    double pn;
    double pe;
    double pd;

    double phi;
    double theta;
    double psi;

    double u;
    double v;
    double w;

    double p;
    double q;
    double r;

    double throttle;
  } state_t;

  Controller();
  Controller(const std::string filename);

  void load(const std::string filename);
  void computeControl(const vehicle::State &x, const double t, quadrotor::commandVector& u, const Eigen::Vector3d &pt);
  inline state_t getCommandedState() const { return xc_; }

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

  struct PID
  {
    PID();
    void init(float kp, float ki, float kd, float max, float min, float tau);
    float run(float dt, float x, float x_c, bool update_integrator);
    float run(float dt, float x, float x_c, bool update_integrator, float xdot);

    float kp_;
    float ki_;
    float kd_;

    float max_;

    float integrator_;
    float differentiator_;
    float prev_x_;
    float tau_;
  };
  PID roll_;
  PID pitch_;
  PID yaw_rate_;

  // Parameters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  int path_type_;

  // Waypoint Parameters
  Eigen::MatrixXd waypoints_;
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
  Eigen::Matrix3d K_p_; // position
  Eigen::Matrix3d K_v_; // velocity
  Eigen::Matrix3d K_d_; // disturbance acceleration


  // Memory for sharing information between functions
  bool initialized_;
  state_t xhat_ = {}; // estimate
  state_t xc_ = {}; // command
  max_t max_ = {};
  double prev_time_;
  uint8_t control_mode_;
  Eigen::Vector3d dhat_; // disturbance acceleration

  // Target estimation parameters
  bool use_target_truth_, bearing_only_;
  Eigen::Vector3d z_, vz_;
  double kz_, kvz_;
  Eigen::Vector3d target_noise_;
  std::normal_distribution<double> target_noise_dist_;

  // Functions
  void updateWaypointManager();
  void updateTrajectoryManager();
};

}
