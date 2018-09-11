#pragma once

#include <fstream>
#include "common_cpp/common.h"

namespace bicycle
{


// State indices
enum
{
  PX,
  PY,
  PZ,
  PSI,
  VEL,
  THETA,
  NUM_STATES
};

// Input indices
enum
{
  FORCE,
  TORQUE,
  NUM_INPUTS
};

// Convenient definitions
typedef Eigen::Matrix<double, NUM_STATES, 1> xVector;
typedef Eigen::Matrix<double, NUM_INPUTS, 1> uVector;


class Bicycle
{

public:

  Bicycle();
  Bicycle(const std::string &filename);
  ~Bicycle();

  void load(const std::string &filename);
  void run(const double &t);

private:

  void f(const xVector& x, const uVector& u, xVector& dx);
  void propagate(const double &dt);
  void computeControl();
  void updateWaypoint();
  double groundFunction(const xVector& state);
  void updateElevation();
  void log(const double &t);

  xVector x_, x2_, x3_, x4_;
  xVector dx_, k1_, k2_, k3_, k4_;
  uVector u_;

  bool accurate_integration_, initialized_;
  double mass_, inertia_, max_force_, max_torque_, max_steering_angle_, L_, t_prev_;
  double ku_, ktheta_, kpsi_;
  double vel_cmd_;

  std::string directory_;
  std::ofstream true_state_log_;
  std::ofstream command_log_;

  // Waypoint Parameters
  Eigen::MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;
  Eigen::Vector2d wp_;

};


}
