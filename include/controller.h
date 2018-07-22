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
  void computeControl(const vehicle::xVector &x, const double t, quadrotor::commandVector& u);
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
  double waypoint_threshold_;
  double waypoint_velocity_threshold_;

  // Controller Gains
  Eigen::Matrix3d K_p_; // position
  Eigen::Matrix3d K_v_; // velocity
  Eigen::Matrix3d K_d_; // disturbance acceleration
  Eigen::MatrixXd waypoints_;
  int current_waypoint_id_;

  // Memory for sharing information between functions
  bool initialized_;
  state_t xhat_ = {}; // estimate
  state_t xc_ = {}; // command
  max_t max_ = {};
  double prev_time_;
  uint8_t control_mode_;
  Eigen::Vector3d dhat_; // disturbance acceleration

  // Functions
  void updateWaypointManager();
};

}
