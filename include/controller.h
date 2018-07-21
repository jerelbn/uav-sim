#pragma once

#include "common_cpp/common.h"


namespace quadrotor
{

typedef Eigen::Matrix<double, 16, 1> xVector;
typedef Eigen::Matrix<double, 12, 1> dxVector;
typedef Eigen::Matrix<double, 4, 1> commandVector;

// State Indexes
enum {
  PX = 0,
  PY = 1,
  PZ = 2,
  QW = 3,
  QX = 4,
  QY = 5,
  QZ = 6,
  VX = 7,
  VY = 8,
  VZ = 9,
  WX = 10,
  WY = 11,
  WZ = 12,
  AX = 13,
  AY = 14,
  AZ = 15,

  DVX = 6, // Derivative indices
  DVY = 7,
  DVZ = 8,
  DWX = 9,
  DWY = 10,
  DWZ = 11,
};

// Input indexes
enum {
  THRUST,
  TAUX,
  TAUY,
  TAUZ
};

}

namespace controller
{

class Controller
{

public:

  Controller();
  Controller(const std::string filename);

  void load(const std::string filename);
  void computeControl(const quadrotor::xVector &x, const double t, quadrotor::commandVector& u);

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
