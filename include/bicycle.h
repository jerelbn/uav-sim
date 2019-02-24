#pragma once

#include <fstream>
#include "common_cpp/common.h"
#include "vehicle.h"
#include "environment.h"

using namespace Eigen;


namespace bicycle
{


class Bicycle
{

public:

  Bicycle();
  Bicycle(const std::string &filename, const int& id);
  ~Bicycle();

  void load(const std::string &filename);
  void run(const double &t, const environment::Environment& env);
  const xVector& getState() const { return x_; }

  int id_;

private:

  void f(const xVector& x, const uVector& u, const Vector3d& vw, xVector& dx);
  void propagate(const double &t, const uVector& u, const Vector3d& vw);
  void computeControl();
  void updateWaypoint();
  double groundFunction(const xVector& state);
  void updateElevation();
  void log(const double &t);

  xVector x_;
  xVector dx_;
  uVector u_;

  bool accurate_integration_, initialized_, flat_ground_;
  double mass_, inertia_, max_force_, max_torque_, max_steering_angle_, L_, t_prev_;
  double ku_, ktheta_, kpsi_;
  double vel_cmd_;

  std::ofstream true_state_log_;
  std::ofstream command_log_;

  // Waypoint Parameters
  MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;
  Vector2d wp_;

};


} // namespace bicycle
